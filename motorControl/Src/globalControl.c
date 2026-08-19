#include "globalControl.h"
#include "motorPara.h"
#include "sample.h"
#include "foc.h"
#include "tim.h"
#include "adc.h"
#include "driver.h"
#include "key.h"
#include "taccdec.h"
#include "statueSwitch.h"

static uint16_t g_adc_buf[SAMPLE_BUFFER]; // 采样数组==>g_adc_buf[0]：母线电压；g_adc_buf[1]：波轮电位器；g_adc_buf[2]：温度
static char Tx_buffer[TX_BUFFER_SIZE] = {0};

static uint16_t g_temp_sample_cnt;        // 温度计算节流计数器
static uint32_t g_align_start_tick;       // 系统tick获取
static uint16_t g_obs_speed_ok_cnt = 0;   /* 观测转速连续达标计数 */
static uint16_t g_obs_angle_ok_cnt = 0;   /* 观测角度连续收敛计数 */
static uint16_t g_bus_uv_cnt = 0;         /* 母线欠压持续计数 */
static uint16_t g_run_stall_cnt = 0;      /* 闭环失速连续计数 */
static uint32_t g_open_start_tick;        // 系统tick获取
static uint32_t last_tick = 0;            /* 梯形加减速实例：目标速度斜坡生成 */

static void MotorStateChange(Motor_t *motor);
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
        Temperature_t *temp = GetTempStruct();
        /*母线电压 */
        motor->Current.adc_bus = g_adc_buf[0];
        /*波轮电位器 */
        motor->Current.adc_postion = g_adc_buf[1];
        motor->Current.pot_ratio = (float)g_adc_buf[1] / ADC_FULL_SCALE;
        /*温度（节流计算，避免中断里频繁执行 logf） */
        temp->adc_value = g_adc_buf[2];
    }
}
/**
 * @brief   tim2进行按键状态扫描(已实现，未加入)、母线电压计算、波轮电位器速度设定、温度计算
 * @param   TIM_HandleTypeDef *htim
 * @return  null
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM2)
    {
        Motor_t *motor = GetMotorStruct();
        Temperature_t *temp = GetTempStruct();

        motor->Current.voltage_bus = g_API_Interface.Sample->GetVoltageBus(&motor->Current);
        motor->PID_Speed.aimValue = MOTOR_SEPPD_COEFFICIENT * motor->Current.pot_ratio;
        // 防止电位器归零时电压跳动导致状态频繁切换
        if (motor->PID_Speed.aimValue >= 10.0f)
        {
            motor->State = MOTOR_RUN;
        }
        else if (motor->PID_Speed.aimValue < 10.0f)
        {
            motor->State = MOTOR_STOP;
        }

        g_API_Interface.Sample->GetTempture(temp);
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

        MotorStateChange(motor);
    }
}
/**
 * @brief   电机运行状态控制
 * @attention   该函数使用状态机实现电机从零速启动到滑膜观测器接入的完整过程
 * @param   Motor_t *motor
 * @return  null
 */
static void MotorStateChange(Motor_t *motor)
{
    switch (motor->State)
    {
        case MOTOR_STOP:
            g_API_Interface.Driver->MotorDriverDisable();
            g_API_Interface.Driver->SetPWMValue(PWM_ARR_ZERO, PWM_ARR_ZERO, PWM_ARR_ZERO);
            break;
        case MOTOR_RUN:
            g_API_Interface.Driver->MotorDriverEnable();
            g_API_Interface.Switch->StrongDragSmoSpeedCurrentLoop(motor);
            g_API_Interface.Driver->SetPWMValue(motor->FOC.DutyCycleA, motor->FOC.DutyCycleB, motor->FOC.DutyCycleC);
            break;
        default:
            motor->State = MOTOR_STOP;
            break;
    }
}
/************************************************************************************************/
/**************************************parameter Init********************************************/
/************************************************************************************************/
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
    // 获取运行周期
    motor->pwm_period = g_API_Interface.Sample->GetPwmPeriod();
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
    motor->FOC.DutyCycleA = 0.0f;
    motor->FOC.DutyCycleB = 0.0f;
    motor->FOC.DutyCycleC = 0.0f;
    motor->FOC.Id_ref = 0.0f;
    motor->FOC.Iq_ref = 0.0f;
    motor->FOC.Ts = motor->pwm_period;
    /* 速度环：带宽约 10~30 rad/s，输出限幅 = 允许的峰值电流(A) */
    motor->PID_Speed.p = 0.05f;
    motor->PID_Speed.i = 0.001f;
    motor->PID_Speed.d = 0.0f;
    motor->PID_Speed.OutputMax = 5.0f;
    motor->PID_Speed.OutputMin = -5.0f;
    motor->PID_Speed.aimValue = 0.0f;
    motor->PID_Speed.nowValue = 0.0f;
    motor->PID_Speed.prevError = 0.0f;
    motor->PID_Speed.integral = 0.0f;
    motor->PID_Speed.SpeedCalculateCnt = 0;
    /* d 轴电流环（参照盛浩板 Kp=0.2/Ki=0.002；输出限幅运行时按 ±Vbus/√3 动态更新） */
    motor->PID_Id.p = 0.2f;
    motor->PID_Id.i = 0.002f;
    motor->PID_Id.d = 0.0f;
    motor->PID_Id.OutputMax = 8.0f; /* 初始占位，运行时动态更新 */
    motor->PID_Id.OutputMin = -8.0f;
    motor->PID_Id.integral = 0.0f;
    motor->PID_Id.aimValue = 0.0f;
    motor->PID_Id.nowValue = 0.0f;
    motor->PID_Id.SpeedCalculateCnt = 0;
    /* q 轴电流环：同 d 轴 */
    motor->PID_Iq.p = 0.2f;
    motor->PID_Iq.i = 0.002f;
    motor->PID_Iq.d = 0.0f;
    motor->PID_Iq.OutputMax = 8.0f;
    motor->PID_Iq.OutputMin = -8.0f;
    motor->PID_Iq.integral = 0.0f;
    motor->PID_Iq.aimValue = 0.0f;
    motor->PID_Iq.nowValue = 0.0f;
    motor->PID_Iq.SpeedCalculateCnt = 0;
    // pll parameter init
    motor->PLL.p = 1200.0f;
    motor->PLL.i = 100.0f;
    motor->PLL.theta_error = 0.0f;
    motor->PLL.theta_hat = 0.0f;
    motor->PLL.omerga_hat = 0.0f;
    motor->PLL.integral = 0.0f;
    motor->PLL.e_alpha_hat = 0.0f;
    motor->PLL.e_beta_hat = 0.0f;
    motor->PLL.theta_hat_lpf = 0.0f;
    motor->PLL.omerga_hat_lpf = 0.0f;
    motor->PLL.Ts = motor->pwm_period;
    // smo parameter
    motor->SMO.e_alpha_hat = 0.0f;
    motor->SMO.e_beta_hat = 0.0f;
    motor->SMO.e_alpha_raw = 0.0f;
    motor->SMO.e_beta_raw = 0.0f;
    motor->SMO.I_alpha_hat = 0.0f;
    motor->SMO.I_beta_hat = 0.0f;
    motor->SMO.K_slide = 14.0f;
    motor->SMO.wc = 1000.0f;
    motor->SMO.Ts = motor->pwm_period;
    // EAngle parameter
    motor->EAngle.accel = 0.0f;
    motor->EAngle.omega = 0.0f;
    motor->EAngle.omega_end = 0.0f;
    motor->EAngle.omega_start = 0.0f;
    motor->EAngle.theta_map = 0.0f;
    motor->EAngle.theta_pu = 0.0f;
    motor->EAngle.Ts = motor->pwm_period;
    // TAccDec parameter
    motor->TAccDec.AccSpeed = 0.0f;
    motor->TAccDec.SpeedChangeIncrement = 0.0f;
    motor->TAccDec.SpeedIincrementDelta = 0.0f;
    motor->TAccDec.SpeedIncrement = 0.0f;
    motor->TAccDec.SpeedIncrementLast = 0.0f;
    motor->TAccDec.SpeedOut = 0.0f;
    motor->TAccDec.SpeedTargetIncrement = 0.0f;
    motor->TAccDec.TargetSpeed = 0.0f;
    motor->TAccDec.Ts = motor->pwm_period;

    motor->ModeChange.CheckCnt = 0;
    motor->ModeChange.CloseMinSpeed = 0.0f;
    motor->ModeChange.CloseRunTime = 0;
    motor->ModeChange.CloseSpeed = 0.0f;
    motor->ModeChange.CloseToOpenSwitchSpeed = 0.0f;
    motor->ModeChange.CurrChangeRate = 0.0f;
    motor->ModeChange.EleCloseSpeedAbs = 0.0f;
    motor->ModeChange.EleOpenSpeedAbs = 0.0f;
    motor->ModeChange.ErrCnt = 0;
    motor->ModeChange.ErrFlag = 0;
    motor->ModeChange.ErrTimes = 0;
    motor->ModeChange.LastTargetSpeed = 0.0f;
    motor->ModeChange.ObsMag = 0.0f;
    motor->ModeChange.OpenCurr = 0.0f;
    motor->ModeChange.OpenCurrLast = 0.0f;
    motor->ModeChange.OpenCurrMax = 0.0f;
    motor->ModeChange.OpenSpeed = 0.0f;
    motor->ModeChange.OpenToCloseSwitchSpeed = 0.0f;
    motor->ModeChange.SpeedErr = 0.0f;
    motor->ModeChange.SpeedErrFlt = 0.0f;
    motor->ModeChange.ThetaErr = 0.0f;
    motor->ModeChange.ThetaObs = 0.0f;
    motor->ModeChange.ThetaRef = 0.0f;
    // 电机初始状态
    motor->State = MOTOR_STOP;

    HAL_ADC_Start_DMA(&hadc2, (uint32_t *)g_adc_buf, SAMPLE_BUFFER);
    HAL_ADCEx_InjectedStart_IT(&hadc2);

    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);

    HAL_TIM_Base_Start_IT(&htim1);
    HAL_TIM_Base_Start_IT(&htim2);
}
/**
 * @brief   电机状态枚举转字符串（串口打印用）
 * @param   State_t state
 * @return  状态名
 */
void FuncRegister(void)
{
    Driver_Register(&g_API_Interface);
    Sample_Register(&g_API_Interface);
    Observer_Register(&g_API_Interface);
    FOC_Register(&g_API_Interface);
    TAccDec_Register(&g_API_Interface);
    Switch_Register(&g_API_Interface);
}
const char *StateName(State_t state)
{
    switch (state)
    {
    case MOTOR_RUN:    return "RUN";
    case MOTOR_STOP:   return "STOP";
    default:           return "ERROR";
    }
}
/************************************************************************************************/
/**************************************parameter Init********************************************/
/************************************************************************************************/
void TxMotorData(Motor_t *motor, Temperature_t *temp)
{
    // 数据发送
    int Tx_buff_len = snprintf(Tx_buffer, sizeof(Tx_buffer),
                               "|St|Aim|Now|Theta|Vbus|Va|Vb|Vc|temp|:%s,%.1f,%.1f,%.2f,%.1f,%.2f,%.2f,%.2f, %.1f\n",
                               StateName(motor->State),
                               motor->PID_Speed.aimValue,
                               motor->PID_Speed.nowValue,
                               motor->PLL.theta_hat_lpf,
                               motor->Current.voltage_bus,
                               motor->FOC.Va,
                               motor->FOC.Vb,
                               motor->FOC.Vc,
                               temp->curr_temp);
    if ((Tx_buff_len > 0) && (Tx_buff_len <= TX_BUFFER_SIZE))
    {
        printf("%s", Tx_buffer);
    }
}