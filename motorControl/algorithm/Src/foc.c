#include "foc.h"
#include "globalControl.h"
/*****************************函数指针****************************************/
static void (*__User_Func_PID__)(Motor_t *motor) = NULL;   // PID函数指针
static void (*__User_Func_SVPWM__)(Motor_t *motor) = NULL; // SVPWM函数指针

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
static float PIDCalc(PID_t *pid, float aimValue, float nowValue)
{
    float error = aimValue - nowValue;

    pid->integral += error;
    pid->integral = _constrain(pid->integral, pid->OutputMax, pid->OutputMin);

    float out = pid->p * error
              + pid->i * pid->integral
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
   float omega_mech = motor->PLL.theta_hat / (float)MOTOR_POLE_PAIRS;
   float iq_ref = PIDCalc(&motor->PID_Speed, motor->PID_Speed.aimValue, omega_mech);

    motor->FOC.Vd = PIDCalc(&motor->PID_Id, 0.0f, motor->FOC.Id);
    motor->FOC.Vq = PIDCalc(&motor->PID_Iq, iq_ref, motor->FOC.Iq);
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
}
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
    float V_limit = motor->Current.voltage_bus * ONE_DIV_SQRT3;
    // output limiting
    if (V_peak > V_limit)
    {
        float scale = V_limit / V_peak;

        Va *= scale;
        Vb *= scale;
        Vc *= scale;
    }
    // output duty
    motor->PWM.Duty_a = 0.5f + Va / motor->Current.voltage_bus;
    motor->PWM.Duty_b = 0.5f + Vb / motor->Current.voltage_bus;
    motor->PWM.Duty_c = 0.5f + Vc / motor->Current.voltage_bus;
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