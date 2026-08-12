#include "globalControl.h"
#include "sample.h"
#include "foc.h"
#include "tim.h"
#include "adc.h"
#include "driver.h"

static uint16_t g_adc_buf[SAMPLE_BUFFER]; // 采样数组==>g_adc_buf[0]：母线电压；g_adc_buf[1]：波轮电位器；g_adc_buf[2]：温度
static uint16_t g_temp_sample_cnt;        // 温度计算节流计数器
static uint32_t g_align_start_tick;       // 系统tick获取
static float g_open_theta;                /* 开环角度（rad） */
static float g_open_omega;                /* 开环电角速度（rad/s） */
static uint16_t g_obs_speed_ok_cnt = 0;   /* 观测转速连续达标计数 */
static uint16_t g_obs_angle_ok_cnt = 0;   /* 观测角度连续收敛计数 */
static uint16_t g_bus_uv_cnt = 0;         /* 母线欠压持续计数 */
static uint16_t g_run_stall_cnt = 0;      /* 闭环失速连续计数 */
static uint32_t g_open_start_tick;        // 系统tick获取
static char Tx_buffer[TX_BUFFER_SIZE] = {0};
static uint32_t last_tick = 0;

/**
 * @brief   电机预定位时设置参数等信息，修改电机运行状态
 * @param   null
 * @return  null
 */
void MotorAlignStart(void)
{
    Motor_t *motor = GetMotorStruct();
    g_align_start_tick = HAL_GetTick();
    motor->State = MOTOR_ALIGN;
}
/**
 * @brief   电机开环强拖时设置参数等信息，修改电机运行状态
 * @param   null
 * @return  null
 */
static void MotorOpenLoopStart(void)
{
    Motor_t *motor = GetMotorStruct();
    g_open_theta = ALIGN_ANGLE; /* 从预定位角度继续，角度连续 */
    g_open_omega = OPENLOOP_OMEGA_START;
    g_open_start_tick = HAL_GetTick();
    g_obs_speed_ok_cnt = 0;   /* 重置收敛计数，避免残留计数导致立即切换 */
    g_obs_angle_ok_cnt = 0;
    motor->State = MOTOR_OPENLOOP;
}
/**
 * @brief   设置PWM占空比
 * @param   uint32_t PWMValue_A, uint32_t PWMValue_B, uint32_t PWMValue_C
 * @return  null
 */
static void SetPWMValue(uint32_t PWMValue_A, uint32_t PWMValue_B, uint32_t PWMValue_C)
{
    /* 占空比限幅到 [0, Period+1]，防止 CCR 溢出导致 100% 输出（恒吸） */
    uint32_t period = (uint32_t)htim1.Init.Period + 1U;
    if (PWMValue_A > period) PWMValue_A = period;
    if (PWMValue_B > period) PWMValue_B = period;
    if (PWMValue_C > period) PWMValue_C = period;
    
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, PWMValue_A);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, PWMValue_B);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, PWMValue_C);
}
/**
 * @brief   TIM1 更新中断回调：每个 PWM 周期软件触发一次 ADC2 注入转换（相电流采样）。
 *          参照盛浩板做法：注入组配为软件触发(JQDIS=1)，转换完成后 JADSTART 自清除，
 *          因此每次更新事件都要重新启动一次注入。
 * @param   htim: TIM handle
 * @return  null
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM1)
    {
        HAL_ADCEx_InjectedStart_IT(&hadc2);
    }
}
/**
 * @brief   规则组ADC通道采集中断回调函数，采集母线电压、波轮电位器以及温度数值
 * @param   null
 * @return  null
 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance == ADC2)
    {
        Motor_t *motor = GetMotorStruct();
        Temperature_t *temp = GetTemperatureStruct();

        /*母线电压 */
        motor->Current.adc_bus = g_adc_buf[0];
        motor->Current.voltage_bus = GetVoltageBus(motor);
        /*波轮电位器 */
        motor->Current.adc_postion = g_adc_buf[1];
        motor->Current.pot_ratio = (float)g_adc_buf[1] / ADC_FULL_SCALE;
        /*温度（节流计算，避免中断里频繁执行 logf） */
        temp->adc_value = g_adc_buf[2];
        if (++g_temp_sample_cnt >= TEMP_SAMPLE_NUM)
        {
            g_temp_sample_cnt = 0;
            /* 除零保护：adc_value 为 0（断路）或满量程（短路）时跳过 */
            if ((temp->adc_value > 0U) && (temp->adc_value < (uint16_t)ADC_FULL_SCALE))
            {
                GetTempture(temp);
            }
        }
    }
}
/**
 * @brief   adc注入中断回调函数，其中使用状态机实现电机校准、强拖、滑膜观测器等功能
 * @param   ADC_HandleTypeDef *hadc
 * @return  null
 */
void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance == ADC2)
    {
        // 获取结构体
        Motor_t *motor = GetMotorStruct();
        motor->SMO.K_slide = motor->Current.voltage_bus * ONE_DIV_SQRT3;
        // 获取到两相的adc原始值
        motor->Current.current_adc_a = HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_1);
        motor->Current.current_adc_c = HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_2);
        // 获取三相电流
        motor->FOC.Ia = GetPhaseCurrent(motor, CURRENT_FLAG_Ia);
        motor->FOC.Ic = GetPhaseCurrent(motor, CURRENT_FLAG_Ic);
        motor->FOC.Ib = -motor->FOC.Ia - motor->FOC.Ic;
        // 电流环输出限幅随母线电压动态更新（参照盛浩板：±Vbus/√3，防止过调制）
        float v_limit = motor->Current.voltage_bus * ONE_DIV_SQRT3;
        motor->PID_Id.OutputMax = v_limit;
        motor->PID_Id.OutputMin = -v_limit;
        motor->PID_Iq.OutputMax = v_limit;
        motor->PID_Iq.OutputMin = -v_limit;
        // 母线欠压保护：Vbus 持续低于阈值则停机，切断 SVPWM 正反馈（母线被拉低→占空比放大→电流更大 的崩溃链）
        if (motor->Current.voltage_bus < BUS_UNDERVOLTAGE_THRESHOLD)
        {
            if (++g_bus_uv_cnt >= BUS_UNDERVOLTAGE_CNT)
            {
                g_bus_uv_cnt = 0;
                motor->State = MOTOR_STOP;   /* 进 STOP 分支输出零占空比，等待电位器归零后由主循环重启 */
            }
        }
        else
        {
            g_bus_uv_cnt = 0;
        }
        // 校准==>低速强拖==>滑膜观测器
        switch (motor->State)
        {
        case MOTOR_CALIB:
            GetOffsetCurrent(motor);

            if (motor->Current.g_current_offset_state)
                MotorAlignStart();
            break;
        case MOTOR_ALIGN:
            /* 预定位：电流闭环。Id 给定励磁电流把转子吸到固定角度，电压由电流环输出，天然限流 */
            motor->FOC.angle = ALIGN_ANGLE; // 给D轴施加电压进行0位校准
            motor->FOC.Vd = PIDCalc(&motor->PID_Id, ALIGN_ID_REF, motor->FOC.Id);
            motor->FOC.Vq = PIDCalc(&motor->PID_Iq, 0.0f, motor->FOC.Iq);

            Clark(motor);
            Park(motor);
            AntiPark(motor);
            SVPWM(motor);
            SetPWMValue((uint32_t)(motor->PWM.Duty_a * (float)(htim1.Init.Period + 1U)),
                        (uint32_t)(motor->PWM.Duty_b * ((float)htim1.Init.Period + 1U)),
                        (uint32_t)(motor->PWM.Duty_c * ((float)htim1.Init.Period + 1U)));
            // 预定位到时间，进入开环强拖
            if (HAL_GetTick() - g_align_start_tick >= ALIGN_TIME_MS)
                MotorOpenLoopStart();
            break;
        case MOTOR_OPENLOOP:
        {
            float obs_theta_err;      /* 开环角度与观测角度误差(rad) */
            float omega_mech;         /* 观测机械角速度(rad/s) */
            if (g_open_omega < OPENLOOP_OMEGA_END) // 线性增加角速度值
                g_open_omega += OPENLOOP_ACCEL * motor->PWM.pwm_period;
            if (g_open_omega > OPENLOOP_OMEGA_END)
                g_open_omega = OPENLOOP_OMEGA_END;
            // 获取角度：角速度对视时间的积分值
            g_open_theta += g_open_omega * motor->PWM.pwm_period;
            // 角度归一化
            if (g_open_theta > PI)
                g_open_theta -= TWO_PI;
            else if (g_open_theta < -PI)
                g_open_theta += TWO_PI;
            // 滑膜观测器接入，提前进行收敛
            Clark(motor);
            ObserverSMO(motor);
            PLL(motor);
            // 电流闭环强拖：Id 给定励磁电流，旋转磁场牵引转子，电压由电流环输出
            motor->FOC.angle = g_open_theta;
            Park(motor);

            motor->FOC.Vd = PIDCalc(&motor->PID_Id, OPENLOOP_ID_REF, motor->FOC.Id);
            motor->FOC.Vq = PIDCalc(&motor->PID_Iq, 0.0f, motor->FOC.Iq);

            AntiPark(motor);
            SVPWM(motor);
            SetPWMValue((uint32_t)(motor->PWM.Duty_a * (float)(htim1.Init.Period + 1U)),
                        (uint32_t)(motor->PWM.Duty_b * ((float)htim1.Init.Period + 1U)),
                        (uint32_t)(motor->PWM.Duty_c * ((float)htim1.Init.Period + 1U)));
            // 开环/观测角度误差（归一化到 ±π），用于角度收敛判据
            obs_theta_err = motor->PLL.theta_hat - g_open_theta;
            if (obs_theta_err > PI)
                obs_theta_err -= TWO_PI;
            else if (obs_theta_err < -PI)
                obs_theta_err += TWO_PI;
            // 速度收敛计数：观测电角速度与开环给定对比，容差 OPENLOOP_OBS_SPEED_ERR
            if (fabsf(motor->PLL.omerga_hat - g_open_omega) < OPENLOOP_OBS_SPEED_ERR)
                g_obs_speed_ok_cnt++;
            else
                g_obs_speed_ok_cnt = 0;
            // 角度收敛计数：低速反电动势弱，速度对但角度可能未收敛，必须角度也达标
            if (fabsf(obs_theta_err) < OPENLOOP_OBS_THETA_ERR)
                g_obs_angle_ok_cnt++;
            else
                g_obs_angle_ok_cnt = 0;
            // 转速达标 + 稳定时间 + 速度/角度双收敛 → 切入闭环（仿盛浩板 StrongDragToObs）
            if ((g_open_omega >= OPENLOOP_OMEGA_END) && (HAL_GetTick() - g_open_start_tick >= OPENLOOP_STABLE_MS)
                && (g_obs_speed_ok_cnt >= OPENLOOP_OBS_OK_CNT)
                && (g_obs_angle_ok_cnt >= OPENLOOP_OBS_ANGLE_OK_CNT))
            {
                /* 速度环接力：反馈滤波、速度环输出/积分初始化为当前观测转速与强拖电流，
                   目标速度钳位到当前转速（误差=0），避免切换瞬间 Iq 参考阶跃顶死电源（24V→8V 失控根因） */
                omega_mech = motor->PLL.omerga_hat / (float)MOTOR_POLE_PAIRS;
                SpeedFeedbackFiltInit(omega_mech);      /* foc.c：初始化速度反馈一阶滤波 */
                motor->PID_Speed.nowValue = omega_mech;
                motor->PID_Speed.aimValue = omega_mech; /* 主循环会从该值继续向电位器目标爬升 */
                motor->PID_Speed.Output = CLOSE_LOOP_INIT_IQ;
                motor->PID_Speed.integral = CLOSE_LOOP_INIT_IQ;
                /* 角度已由判据保证与开环角一致，无需强制覆盖 PLL 状态，保持观测器内部自洽 */
                motor->State = MOTOR_RUN;
            }
        }
        break;
        case MOTOR_RUN:
            // 闭环运行状态：完整FOC
            Clark(motor);
            ObserverSMO(motor);
            PLL(motor);
            Park(motor);
            PID(motor);
            AntiPark(motor);
            SVPWM(motor);
            SetPWMValue((uint32_t)(motor->PWM.Duty_a * (float)(htim1.Init.Period + 1U)),
                        (uint32_t)(motor->PWM.Duty_b * ((float)htim1.Init.Period + 1U)),
                        (uint32_t)(motor->PWM.Duty_c * ((float)htim1.Init.Period + 1U)));

            if (fabsf(motor->PLL.omerga_hat) < RUN_MIN_OMEGA)
            {
                if (++g_run_stall_cnt >= RUN_STALL_CNT)
                {
                    g_run_stall_cnt = 0;
                    g_obs_speed_ok_cnt = 0;
                    MotorOpenLoopStart();   /* 回到强拖：g_open_omega 从低速重新爬升 */
                }
            }
            else
            {
                g_run_stall_cnt = 0;
            }
            break;
        case MOTOR_STOP:
                SetPWMValue(PWM_ARR_ZERO, PWM_ARR_ZERO, PWM_ARR_ZERO);
            break;
        default:
            SetPWMValue(PWM_ARR_ZERO, PWM_ARR_ZERO, PWM_ARR_ZERO);
            break;
        }
    }
}
/**************************************parameter Init********************************************/
/**
 * @brief   按键初始化
 * @param   Key_t *key
 * @return  null
 */
void KeyInit(Key_t *key)
{
    key->state = KEY_STATE_IDLE;
    key->debounce_cnt = 0U;
    key->pressed_event = 0U;
}
/**
 * @brief   温度参数初始化
 * @param   Temperature_t *temp
 * @return  null
 */
void TemperatureInit(Temperature_t *temp)
{
    temp->resistor_other = 10000.0f;
    temp->temperature_ref = 298.15f;
    temp->B_value = 3434.0f;
    temp->resistor_ref = 10000.0f;
    temp->curr_resistor = 0.0f;
    temp->ln_value = 0.0f;
    temp->adc_value = 0.0f;
    temp->curr_temp = 0.0f;
}
/**
 * @brief   电机参数初始化
 * @param   Motor_t *motor
 * @return  null
 */
void MotorParaInit(Motor_t *motor)
{
    // current parameter init
    motor->Current.adc_bus = 0.0f;
    motor->Current.current_adc_a = 0.0f;
    motor->Current.current_adc_c = 0.0f;
    motor->Current.current_offset_Ia = 0.0f;
    motor->Current.current_offset_Ic = 0.0f;
    motor->Current.current_phase_Ia = 0.0f;
    motor->Current.current_phase_Ib = 0.0f;
    motor->Current.current_phase_Ic = 0.0f;
    motor->Current.voltage_bus = 0.0f;
    motor->Current.adc_postion = 0U;
    motor->Current.pot_ratio = 0.0f;
    motor->Current.g_current_offset_state = false;
    // foc parameter init
    motor->FOC.Ia = 0.0f;
    motor->FOC.Ib = 0.0f;
    motor->FOC.Ic = 0.0f;
    motor->FOC.I_alpha = 0.0f;
    motor->FOC.I_beta = 0.0f;
    motor->FOC.Id = 0.0f;
    motor->FOC.Iq = 0.0f;
    motor->FOC.V_alpha = 0.0f;
    motor->FOC.V_beta = 0.0f;
    motor->FOC.Vd = 0.0f;
    motor->FOC.Vq = 0.0f;
    motor->FOC.angle = 0.0f;
    /* 速度环：带宽约 10~30 rad/s，输出限幅 = 允许的峰值电流(A) */
    motor->PID_Speed.p = 0.05f;
    motor->PID_Speed.i = 0.001f;
    motor->PID_Speed.d = 0.0f;
    motor->PID_Speed.OutputMax = 5.0f;
    motor->PID_Speed.OutputMin = -5.0f;
    motor->PID_Speed.aimValue = 0.0f;
    motor->PID_Speed.integral = 0.0f;
    /* d 轴电流环（参照盛浩板 Kp=0.2/Ki=0.002；输出限幅运行时按 ±Vbus/√3 动态更新） */
    motor->PID_Id.p = 0.2f;
    motor->PID_Id.i = 0.002f;
    motor->PID_Id.d = 0.0f;
    motor->PID_Id.OutputMax = 8.0f; /* 初始占位，运行时动态更新 */
    motor->PID_Id.OutputMin = -8.0f;
    motor->PID_Id.integral = 0.0f;
    /* q 轴电流环：同 d 轴 */
    motor->PID_Iq.p = 0.2f;
    motor->PID_Iq.i = 0.002f;
    motor->PID_Iq.d = 0.0f;
    motor->PID_Iq.OutputMax = 8.0f;
    motor->PID_Iq.OutputMin = -8.0f;
    motor->PID_Iq.integral = 0.0f;
    // pll parameter init（参照盛浩板 SPLL Kp=1200/Ki=100；原为 0 导致 theta_hat 恒 0、闭环失效）
    motor->PLL.p = 1200.0f;
    motor->PLL.i = 100.0f;
    motor->PLL.theta_error = 0.0f;
    motor->PLL.theta_hat = 0.0f;
    motor->PLL.omerga_hat = 0.0f;
    motor->PLL.integral = 0.0f;
    // smo parameter（参照盛浩板 SMO Gain=14，滤波系数 α=wc*Ts≈0.1；原为 0 导致反电动势观测恒 0）
    motor->SMO.e_alpha_hat = 0.0f;
    motor->SMO.e_beta_hat = 0.0f;
    motor->SMO.e_alpha_raw = 0.0f;
    motor->SMO.e_beta_raw = 0.0f;
    motor->SMO.I_alpha_hat = 0.0f;
    motor->SMO.I_beta_hat = 0.0f;
    motor->SMO.K_slide = 14.0f;
    motor->SMO.wc = 1000.0f;
    // pwm parameter init
    motor->PWM.Duty_a = 0.0f;
    motor->PWM.Duty_b = 0.0f;
    motor->PWM.Duty_c = 0.0f;
    motor->PWM.pwm_period = GetPwmPeriod();

    motor->State = MOTOR_STOP;

    HAL_ADC_Start_DMA(&hadc2, (uint32_t *)g_adc_buf, SAMPLE_BUFFER);
    HAL_ADCEx_InjectedStart_IT(&hadc2);
    /* 注入转换由 TIM1 更新中断（HAL_TIM_PeriodElapsedCallback）逐周期软件触发。
       关键：HAL_TIM_PWM_Start_IT 只使能 CC 中断、不使能更新中断，
       必须用 HAL_TIM_Base_Start_IT 使能更新中断（参照盛浩板），
       否则 HAL_TIM_PeriodElapsedCallback 永不执行、注入永不触发、状态机卡在 CALIB */
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIM_Base_Start_IT(&htim1);
}
/**
 * @brief   电机状态枚举转字符串（串口打印用）
 * @param   State_t state
 * @return  状态名
 */
const char *StateName(State_t state)
{
    switch (state)
    {
    case MOTOR_CALIB:    return "CALIB";
    case MOTOR_ALIGN:    return "ALIGN";
    case MOTOR_OPENLOOP: return "OPENLOOP";
    case MOTOR_RUN:      return "RUN";
    case MOTOR_STOP:     return "STOP";
    default:             return "ERROR";
    }
}
void TxMotorData(Motor_t *motor, Temperature_t *temp)
{
     // 数据发送（含调试量：Vd/Vq 电流环输出、Duty_a 占空比、adcA 注入采样原始值、ofA 偏置）
    int Tx_buff_len = snprintf(Tx_buffer, sizeof(Tx_buffer),
                               "|St|Aim|Now|Theta|Vbus|Va|Vb|Vc|Vd|Vq|Da|adcA|ofA|:%s,%.1f,%.1f,%.2f,%.1f,%.2f,%.2f,%.2f,%.1f,%.1f,%.3f,%u,%u\n",
                               StateName(motor->State),
                               motor->PID_Speed.aimValue,
                               motor->PID_Speed.nowValue,
                               motor->PLL.theta_hat,
                               motor->Current.voltage_bus,
                               motor->FOC.Va,
                               motor->FOC.Vb,
                               motor->FOC.Vc,
                               motor->FOC.Vd,
                               motor->FOC.Vq,
                               motor->PWM.Duty_a,
                               motor->Current.current_adc_a,
                               (unsigned int)motor->Current.current_offset_Ia);
    if ((Tx_buff_len > 0) && (Tx_buff_len <= TX_BUFFER_SIZE))
    {
      printf("%s", Tx_buffer);
    }
}
void MotorSpedControl(Motor_t *motor)
{
    if (HAL_GetTick() - last_tick > 2U)
    {
      last_tick = HAL_GetTick();

      // 电位器目标：0~MOTOR_SEPPD_COEFFICIENT 对应机械角速度(rad/s)。
      // 除以极对数，与 foc.c 速度环反馈(机械角速度 omerga_hat/POLE_PAIRS)单位一致
      float target = MOTOR_SEPPD_COEFFICIENT * motor->Current.pot_ratio / (float)MOTOR_POLE_PAIRS;
      float diff = target - motor->PID_Speed.aimValue;

      if (diff > 0.5f)
        diff = 0.5f;
      else if (diff < -0.5f)
        diff = -0.5f;
      motor->PID_Speed.aimValue += diff;
    }
}