#include "foc.h"
#include "globalControl.h"
/*****************************函数指针****************************************/
static void (*__User_Func_PID__)(Motor_t *motor) = NULL;   // PID函数指针
static void (*__User_Func_SVPWM__)(Motor_t *motor) = NULL; // SVPWM函数指针
/*****************************速度反馈滤波************************************/
static float g_omega_mech_filt;      /* 滤波后的机械角速度(rad/s) */

/**
 * @fn  void Clark(Motor_t *motor)
 * @brief   make the motor's three phase currents become the two phase currents
 * @param   Motor_t *motor
 * @return  null
 */
void Clark(Motor_t *motor)
{
    motor->FOC.I_alpha = motor->FOC.Ia;
    motor->FOC.I_beta = (motor->FOC.Ia + 2.0f * motor->FOC.Ib) / sqrtf(3.0f);
}
/**
 * @fn  void Park(Motor_t *motor)
 * @brief   make the motor's two phase currents become the spin two phase currents
 * @param   Motor_t *motor
 * @return  null
 */
void Park(Motor_t *motor)
{
    float sin_angle = sinf(motor->FOC.angle);
    float cos_angle = cosf(motor->FOC.angle);

    motor->FOC.Id = motor->FOC.I_alpha * cos_angle + motor->FOC.I_beta * sin_angle;
    motor->FOC.Iq = -(motor->FOC.I_alpha * sin_angle) + motor->FOC.I_beta * cos_angle;
}
/**
 * @brief  位置式 PID 通用计算
 * @param  pid: PID 实例；aim: 目标值；now: 当前值
 * @retval 限幅后的输出
 */
float PIDCalc(PID_t *pid, float aimValue, float nowValue)
{
    float error = aimValue - nowValue;

    /* 抗积分饱和（参照盛浩板 PID_Control）：
       仅当输出未饱和时才累加积分，防止积分过深导致退出饱和时响应滞后；
       积分项直接参与输出（单位与输出一致），不再被 i 二次缩小 + OutputMax 封顶，
       否则电流环输出能力被限制在 ~1V，远达不到 ±Vbus/√3 限幅 */
    if ((pid->Output < pid->OutputMax) && (pid->Output > pid->OutputMin))
    {
        pid->integral += pid->i * error;
    }

    float out = pid->p * error
              + pid->integral
              + pid->d * (error - pid->lastError);
    pid->lastError = error;

    pid->Output = _constrain(out, pid->OutputMax, pid->OutputMin);
    return pid->Output;
}
/**
 * @fn  void pid(Motor_t *motor)
 * @brief   incremental PID control
 * @param   Motor_t *motor
 * @return  null
 */
static void DefaultPID(Motor_t *motor)
{
    /* 电角速度(rad/s) ÷ 极对数 → 机械角速度（注意：用 omerga_hat 角速度，不是 theta_hat 角度） */
    float omega_mech = motor->PLL.omerga_hat / (float)MOTOR_POLE_PAIRS;
    /* 转速反馈一阶低通：滤掉 SMO 高频噪声，防止速度环输出抖动 */
    g_omega_mech_filt += SPEED_LPF_ALPHA * (omega_mech - g_omega_mech_filt);
    motor->PID_Speed.nowValue = g_omega_mech_filt;
    /* 速度环（外环）：目标转速 vs 滤波后实际转速 → Iq 目标 */
    float iq_ref = PIDCalc(&motor->PID_Speed, motor->PID_Speed.aimValue, g_omega_mech_filt);
    
    /* d 轴电流环：目标 Id = 0 */
    motor->FOC.Vd = PIDCalc(&motor->PID_Id, 0.0f, motor->FOC.Id);
    /* q 轴电流环：目标 Iq = 速度环输出 */
    motor->FOC.Vq = PIDCalc(&motor->PID_Iq, iq_ref, motor->FOC.Iq);
}
/**
 * @brief   速度反馈一阶低通滤波初始化（开环→闭环切换时调用，避免滤波从 0 爬升导致速度环误判）
 * @param   float omega_mech  当前机械角速度(rad/s)
 * @return  null
 */
void SpeedFeedbackFiltInit(float omega_mech)
{
    g_omega_mech_filt = omega_mech;
}
/**
 * @fn  void AntiPark(Motor_t *motor)
 * @brief   make the motor's two spin phase currents become the static phase currents
 * @param   Motor_t *motor
 * @return  null
 */
void AntiPark(Motor_t *motor)
{
    float sin_el = sinf(motor->FOC.angle);
    float cos_el = cosf(motor->FOC.angle);

    motor->FOC.V_alpha = motor->FOC.Vd * cos_el - motor->FOC.Vq * sin_el;
    motor->FOC.V_beta = motor->FOC.Vd * sin_el + motor->FOC.Vq * cos_el;
}
/**
 * @brief   this code include two SVPWM methods.
 *          defalut method is seven stage SVPWM,if you want to change the method,
 *          please change the SVPWM define in "globalControl.h".
 */
static void DefaultSVPWM(Motor_t *motor)
{
#if (defined(SVPWM_SECTOR_METHOD))

    float T1, T2;
    float Ta, Tb, Tc;

    float V_alpha = motor->FOC.V_alpha;
    float V_beta = motor->FOC.V_beta;
    float V_dc = motor->Current.voltage_bus;
    float Ts = motor->PWM.pwm_period;

    float U1 = V_beta;
    float U2 = SQRT3_DIV_TWO * V_alpha - 0.5f * V_beta;
    float U3 = -SQRT3_DIV_TWO * V_alpha - 0.5f * V_beta;
    // section judge
    uint8_t A = U1 > 0.0f ? 1 : 0;
    uint8_t B = U2 > 0.0f ? 1 : 0;
    uint8_t C = U3 > 0.0f ? 1 : 0;
    uint8_t section = (A << 2) | (B << 1) | C;
    // useful duty time
    float K = ONE_DIV_SQRT3 * Ts / V_dc;
    float X = V_beta * K;
    float Y = (SQRT3_DIV_TWO * V_alpha + 0.5f * V_beta) * K;
    float Z = (-SQRT3_DIV_TWO * V_alpha + 0.5f * V_beta) * K;
    // pwm useful time
    switch (section)
    {
    case 3:
        T1 = -Z;
        T2 = X;
        break;
    case 1:
        T1 = Z;
        T2 = Y;
        break;
    case 5:
        T1 = X;
        T2 = -Y;
        break;
    case 4:
        T1 = -X;
        T2 = Z;
        break;
    case 6:
        T1 = -Y;
        T2 = -Z;
        break;
    case 2:
        T1 = Y;
        T2 = -X;
        break;
    default:
        T1 = 0.0f;
        T2 = 0.0f;
        break;
    }
    // overmodulation limiting
    if ((T1 + T2) > Ts)
    {
        float scale = Ts / (T1 + T2);
        T1 *= scale;
        T2 *= scale;
    }
    // zero vector time
    float T0 = (Ts - T1 - T2) * 0.5f;
    // seven sector svpwm
    switch (section)
    {
    /* sction I (U4-U6): 000→100→110→111→110→100→000 */
    case 3:
        Ta = T0 + T1 + T2;
        Tb = T0 + T2;
        Tc = T0;
        break;
    /* sction II (U2-U6): 000→010→110→111→110→010→000 */
    case 1:
        Ta = T0 + T1;
        Tb = T0 + T1 + T2;
        Tc = T0;
        break;
    /* sction III (U2-U3): 000→010→011→111→011→010→000 */
    case 5:
        Ta = T0;
        Tb = T0 + T1 + T2;
        Tc = T0 + T2;
        break;
    /* sction IV (U1-U3): 000→001→011→111→011→001→000 */
    case 4:
        Ta = T0;
        Tb = T0 + T1;
        Tc = T0 + T1 + T2;
        break;
    /* sction V (U1-U5): 000→001→101→111→101→001→000 */
    case 6:
        Ta = T0 + T2;
        Tb = T0;
        Tc = T0 + T1 + T2;
        break;
    /* sction VI (U4-U5): 000→100→101→111→101→100→000 */
    case 2:
        Ta = T0 + T1 + T2;
        Tb = T0;
        Tc = T0 + T1;
        break;
    default:
        Ta = Tb = Tc = T0;
        break;
    }
    // duty output
    motor->PWM.Duty_a = Ta / Ts;
    motor->PWM.Duty_b = Tb / Ts;
    motor->PWM.Duty_c = Tc / Ts;
    
#elif (defined(SVPWM_ZERO_SQUENCE))
    // Anti Clark
    float Va = motor->FOC.V_alpha;
    float Vb = -0.5f * motor->FOC.V_alpha + SQRT3_DIV_TWO * motor->FOC.V_beta;
    float Vc = -0.5f * motor->FOC.V_alpha - SQRT3_DIV_TWO * motor->FOC.V_beta;
    // find max and min vlotage
    float Vmax = Va;
    if (Vb > Vmax)
        Vmax = Vb;
    if (Vc > Vmax)
        Vmax = Vc;

    float Vmin = Va;
    if (Vb < Vmin)
        Vmin = Vb;
    if (Vc < Vmin)
        Vmin = Vc;
    // Zero sequence injection
    float V_offset = (Vmax + Vmin) * 0.5f;

    Va -= V_offset;
    Vb -= V_offset;
    Vc -= V_offset;
    // Zero axis translation
    float V_peak = (Vmax - Vmin) * 0.5f;
    /* 过调制限制：零序注入后调制波峰值必须 ≤ Vbus/2（Duty=0.5+Va'/Vbus ∈ [0,1]）。
       原用 Vbus/√3(0.577Vbus) 偏大，限幅后 Duty 仍可达 1.077，被 _constrain 削顶畸变 */
    float vbus = motor->Current.voltage_bus;
    if (vbus <= 0.0f) /* 母线采样异常保护：输出零矢量（50%），避免除零后全压输出 */
    {
        motor->PWM.Duty_a = 0.5f;
        motor->PWM.Duty_b = 0.5f;
        motor->PWM.Duty_c = 0.5f;
        
        return;
    }
    float V_limit = vbus * 0.5f;
    // output limiting
    if (V_peak > V_limit)
    {
        float scale = V_limit / V_peak;

        Va *= scale;
        Vb *= scale;
        Vc *= scale;
    }

    motor->FOC.Va = Va;
    motor->FOC.Vb = Vb;
    motor->FOC.Vc = Vc;

    float duty_a = 0.5f + Va / vbus;
    float duty_b = 0.5f + Vb / vbus;
    float duty_c = 0.5f + Vc / vbus;
    /* 占空比限幅 [0.02, 0.92]（参照盛浩板 PwmLimit≈92%）：
       过调制饱和到 100%/0% 时该相下桥臂恒关断/恒开通，000 采样窗口消失，
       低边电流采样采不到 → 电流环/SMO/PLL 反馈全乱（现象：电流±9A 跳变、角度乱跳）。
       限制最大占空比保证每个 PWM 周期都存在 000 矢量采样窗口 */
    motor->PWM.Duty_a = _constrain(duty_a, 0.92f, 0.02f);
    motor->PWM.Duty_b = _constrain(duty_b, 0.92f, 0.02f);
    motor->PWM.Duty_c = _constrain(duty_c, 0.92f, 0.02f);
#endif
}
/**
 * @brief   实现用户自定义PID函数与默认函数的切换
 * @param   Motor_t *motor
 * @return  null
 */
void PID(Motor_t *motor)
{
    // PID函数指定
    if (__User_Func_PID__ != NULL)
    {
        __User_Func_PID__(motor);
    }
    else
    {
        __User_Func_PID__ = DefaultPID;
    }
}
/**
 * @brief   实现用户自定义函数与默认函数的切换
 * @param   Motor_t *motor
 * @return  null
 */
void SVPWM(Motor_t *motor)
{
    // SVPWM函数指定
    if (__User_Func_SVPWM__ != NULL)
    {
        __User_Func_SVPWM__(motor);
    }
    else
    {
        __User_Func_SVPWM__ = DefaultSVPWM;
    }
}
/**
 * @brief   实现用户自定义函数注册
 * @param   void (*UserFuncSVPWM)(Motor_t *motor)
 * @return  null
 */
void FuncRegister(void (*UserFuncPID)(Motor_t *motor),
                  void (*UserFuncSVPWM)(Motor_t *motor))
{
    __User_Func_PID__ = UserFuncPID;
    __User_Func_SVPWM__ = UserFuncSVPWM;
}