#include "globalControl.h"
#include "sample.h"
#include "foc.h"
#include "observer.h"
#include "tim.h"
#include "adc.h"
#include "driver.h"

static uint16_t g_adc_buf[SAMPLE_BUFFER];
static uint16_t g_temp_sample_cnt; // 温度计算节流计数器

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
void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance == ADC2)
    {
        Motor_t *motor = GetMotorStruct();

        motor->Current.current_adc_a = HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_1);
        motor->Current.current_adc_c = HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_2);

        if (!motor->Current.g_current_offset_state && motor->State == MOTOR_STOP)
        {
            GetOffsetCurrent(motor);

            return;
        }

        motor->FOC.Ia = GetPhaseCurrent(motor, CURRENT_FLAG_Ia);
        motor->FOC.Ic = GetPhaseCurrent(motor, CURRENT_FLAG_Ic);
        motor->FOC.Ib = -motor->FOC.Ia - motor->FOC.Ic;

        Clark(motor);
        ObserverSMO(motor);
        PLL(motor);
        Park(motor);
        Pid(motor);
        AntiPark(motor);
        SVPWM_ZeroSequence(motor);

        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, motor->PWM.Duty_a);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, motor->PWM.Duty_b);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, motor->PWM.Duty_c);
    }
}
/**
 * @fn  void KeyInit(Key_t *key)
 * @brief   init key
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
 * @fn  初始化温度参数
 * @brief   参数初始化
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
 * @fn  初始化电机参数
 * @brief   参数初始化
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
    // pid parameter init
    motor->PID.p = 0.0f;
    motor->PID.i = 0.0f;
    motor->PID.d = 0.0f;
    motor->PID.lastError = 0.0f;
    motor->PID.prevError = 0.0f;
    motor->PID.aimValue = 0.0f;
    motor->PID.nowValue = 0.0f;
    motor->PID.Output = 0.0f;
    motor->PID.OutputMax = 0.0f;
    motor->PID.OutputMin = 0.0f;
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