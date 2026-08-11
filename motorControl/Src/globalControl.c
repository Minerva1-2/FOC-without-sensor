#include "globalControl.h"
#include "sample.h"
#include "foc.h"
#include "tim.h"
#include "adc.h"
#include "driver.h"

static uint16_t g_adc_buf[SAMPLE_BUFFER]; // 采样数组==>g_adc_buf[0]：母线电压；g_adc_buf[1]：波轮电位器；g_adc_buf[2]：温度
static uint16_t g_temp_sample_cnt;        // 温度计算节流计数器
static uint32_t g_align_start_tick;       // 系统tick获取
static float g_align_volt_ramp;           // 电压斜坡值
static float g_open_theta;                /* 开环角度（rad） */
static float g_open_omega;                /* 开环电角速度（rad/s） */
static float g_open_volt;                 /* 开环电压幅值（V） */
static uint32_t g_open_start_tick;        // 系统tick获取
/**
 * @brief   电机预定位时设置参数等信息，修改电机运行状态
 * @param   null
 * @return  null
 */
void MotorAlignStart(void)
{
    Motor_t *motor = GetMotorStruct();
    g_align_start_tick = HAL_GetTick();
    g_align_volt_ramp = 0.0f;
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
    g_open_volt = OPENLOOP_VOLTAGE_START;
    g_open_start_tick = HAL_GetTick();
    motor->State = MOTOR_OPENLOOP;
}
/**
 * @brief   设置PWM占空比
 * @param   uint32_t PWMValue_A, uint32_t PWMValue_B, uint32_t PWMValue_C
 * @return  null
 */
static void SetPWMValue(uint32_t PWMValue_A, uint32_t PWMValue_B, uint32_t PWMValue_C)
{
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, PWMValue_A);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, PWMValue_B);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, PWMValue_C);
}
/**
 * @brief   规则组ADC通道采集中断回调函数，采集母线电压、波轮典韦器以及温度数值
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
        // 获取到两相的adc原始值
        motor->Current.current_adc_a = HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_1);
        motor->Current.current_adc_c = HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_2);
        //  偏置电流校准
        if (!motor->Current.g_current_offset_state && motor->State == MOTOR_STOP)
        {
            GetOffsetCurrent(motor);

            return;
        }
        // 获取三相电流
        motor->FOC.Ia = GetPhaseCurrent(motor, CURRENT_FLAG_Ia);
        motor->FOC.Ic = GetPhaseCurrent(motor, CURRENT_FLAG_Ic);
        motor->FOC.Ib = -motor->FOC.Ia - motor->FOC.Ic;
        // 校准==>低速强拖==>滑膜观测器
        switch (motor->State)
        {
        case MOTOR_ALIGN:
            if (g_align_volt_ramp < ALIGN_VOLTAGE) // 施加预定电压，通过线性斜坡缓慢增加电压（离散积分）
                g_align_volt_ramp += ALIGN_VOLTAGE * (motor->PWM.pwm_period / 0.1f);
            if (g_align_volt_ramp > ALIGN_VOLTAGE)
                g_align_volt_ramp = ALIGN_VOLTAGE;

            motor->FOC.angle = ALIGN_ANGLE; // 给D轴施加电压进行0位校准
            motor->FOC.Vd = g_align_volt_ramp;
            motor->FOC.Vq = 0.0f;

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
            // 开环强拖电压获取,线性爬升:(OPENLOOP_VOLTAGE_END - OPENLOOP_VOLTAGE_START):爬升总量;(motor->PWM.pwm_period / 1.0f):每次爬升比例
            if (g_open_volt < OPENLOOP_VOLTAGE_END)
                g_open_volt += (OPENLOOP_VOLTAGE_END - OPENLOOP_VOLTAGE_START) * (motor->PWM.pwm_period / 1.0f);
            if (g_open_volt > OPENLOOP_VOLTAGE_END)
                g_open_volt = OPENLOOP_VOLTAGE_END;
            // 滑膜观测器接入，提前进行收敛
            ObserverSMO(motor);
            PLL(motor);
            // 给定电角度，D轴施加电压进行强拖
            motor->FOC.angle = g_open_theta;
            motor->FOC.Vd = g_open_volt;
            motor->FOC.Vq = 0.0f;
            // pwm设置
            AntiPark(motor);
            SVPWM(motor);
            SetPWMValue((uint32_t)(motor->PWM.Duty_a * (float)(htim1.Init.Period + 1U)),
                        (uint32_t)(motor->PWM.Duty_b * ((float)htim1.Init.Period + 1U)),
                        (uint32_t)(motor->PWM.Duty_c * ((float)htim1.Init.Period + 1U)));
            // 电角速度达到预定切换至并且稳定时间达到要求，进入闭环旋转
            if ((g_open_omega >= OPENLOOP_OMEGA_END) && (HAL_GetTick() - g_open_start_tick >= OPENLOOP_STABLE_MS))
                motor->State = MOTOR_RUN;
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
    /* d 轴电流环：带宽约 500~1000 rad/s，输出限幅 = 母线电压 ~80% */
    motor->PID_Id.p = 1.0f;
    motor->PID_Id.i = 0.01f;
    motor->PID_Id.d = 0.0f;
    motor->PID_Id.OutputMax = 8.0f; /* 母线电压估算值，实机按 Vbus 调 */
    motor->PID_Id.OutputMin = -8.0f;
    motor->PID_Id.integral = 0.0f;
    /* q 轴电流环：同 d 轴 */
    motor->PID_Iq.p = 1.0f;
    motor->PID_Iq.i = 0.01f;
    motor->PID_Iq.d = 0.0f;
    motor->PID_Iq.OutputMax = 8.0f;
    motor->PID_Iq.OutputMin = -8.0f;
    motor->PID_Iq.integral = 0.0f;
    // pll parameter init
    motor->PLL.p = 0.0f;
    motor->PLL.i = 0.0f;
    motor->PLL.theta_error = 0.0f;
    motor->PLL.theta_hat = 0.0f;
    motor->PLL.omerga_hat = 0.0f;
    motor->PLL.integral = 0.0f;
    // smo parameter
    motor->SMO.e_alpha_hat = 0.0f;
    motor->SMO.e_beta_hat = 0.0f;
    motor->SMO.e_alpha_raw = 0.0f;
    motor->SMO.e_beta_raw = 0.0f;
    motor->SMO.I_alpha_hat = 0.0f;
    motor->SMO.I_beta_hat = 0.0f;
    motor->SMO.K_slide = 0.0f;
    motor->SMO.wc = 0.0f;
    // pwm parameter init
    motor->PWM.Duty_a = 0.0f;
    motor->PWM.Duty_b = 0.0f;
    motor->PWM.Duty_c = 0.0f;
    motor->PWM.pwm_period = GetPwmPeriod();

    motor->State = MOTOR_STOP;

    HAL_ADC_Start_DMA(&hadc2, (uint32_t *)g_adc_buf, SAMPLE_BUFFER);
    HAL_ADCEx_InjectedStart(&hadc2);

    HAL_TIM_PWM_Start_IT(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start_IT(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start_IT(&htim1, TIM_CHANNEL_3);
}