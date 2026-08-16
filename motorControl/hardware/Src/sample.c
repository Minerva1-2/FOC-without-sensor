/***************************************************电流采集********************************************************/
/*    @author   yankaifeng                                                                                         */                                                                                                            
/*    @dete     2026.08.02                                                                                         */   
/*    @brief：get the motor phase current                                                                          */ 
/*    @attention:just for individual learning                                                                      */ 
/*    @details：the file realized two functions,GetOffsetCurrent function gather the bias                          */
/*              voltagewhen the motor is static,GetPhaseCurrent function gain the phase current                    */
/*              when the motor is exercising                                                                       */
/*******************************************************************************************************************/
#include "sample.h"
#include "stm32g4xx_ll_rcc.h"

static float g_offset_Ia, g_offset_Ic;
static uint16_t g_current_sample_num;

/**
 * @fn      void GetOffsetCurrent(MotorFOC_t *motor)
 * @param   Motor_t *motor
 * @return  null
 * @brief   get the offset voltage when the motor is static
 */
static void GetOffsetCurrent(Current_t *Current)
{
    if (Current->g_current_offset_state)
        return;

	g_offset_Ia += Current->current_adc_a;
    g_offset_Ic += Current->current_adc_c;

    g_current_sample_num++;

    if (g_current_sample_num >= CURRENT_SAMPLE_NUM)
    {
        Current->current_offset_Ia = (float)g_offset_Ia / (float)CURRENT_SAMPLE_NUM;
        Current->current_offset_Ic = (float)g_offset_Ic / (float)CURRENT_SAMPLE_NUM;
    
        g_current_sample_num = 0;
        g_offset_Ia = MOTOR_PARA_RESET;
        g_offset_Ic = MOTOR_PARA_RESET;
        Current->g_current_offset_state = true;
    }
}
/**
 * @fn      float GetPhaseCurrent(MotorFOC_t *motor, uint8_t phase_flag)
 * @param   Motor_t *motor, uint8_t phase_flag
 * @return  voltage / (CURRENT_AMP_GAIN * CURRENT_SHUNT_OHM)
 * @brief   this function gather real phase current when the motor is running
 */
static float GetPhaseCurrent(Current_t *Current, uint8_t phase_flag)
{
    uint16_t adc = 0;
    float offset_voltage = 0.0f;

    if (0U == phase_flag)
    {
        offset_voltage = Current->current_offset_Ia;
        adc = Current->current_adc_a;
    }
    else if (1U == phase_flag)
    {
        offset_voltage = Current->current_offset_Ic;
        adc = Current->current_adc_c;
    }
    else {
        return MOTOR_PARA_RESET;
    }

    float voltage = ((float)adc - offset_voltage) * ADC_REF_VOLTAGE / ADC_FULL_SCALE;
    
    return (voltage / (CURRENT_AMP_GAIN * CURRENT_SHUNT_OHM));
}
static float GetVoltageBus(Current_t *Current)
{
    float v_adc = Current->adc_bus * ADC_REF_VOLTAGE / ADC_FULL_SCALE;
    
    return v_adc / VBUS_DIV_RATIO;
}
static void GetTempture(Temperature_t *temp)
{
    temp->curr_resistor = temp->resistor_other * (ADC_FULL_SCALE - temp->adc_value) / temp->adc_value;
    temp->ln_value = logf(temp->curr_resistor / temp->resistor_ref);
    temp->curr_temp = 1.0f / (1.0f / temp->temperature_ref + 1.0f / temp->B_value * temp->ln_value) - K_TEMPERATURE;
}
static float GetPwmPeriod(void)
{
    uint32_t pclk2 = HAL_RCC_GetPCLK2Freq();
    uint32_t tim_clk = pclk2;
    /* APB 预分频 >1 时，TIMxCLK = PCLK × 2（G4 规则，用 LL 库读取） */
    if (LL_RCC_GetAPB2Prescaler() != LL_RCC_APB2_DIV_1)
    {
        tim_clk = pclk2 * 2U;
    }
    /* 中心对齐：完整 PWM 周期 = 2 × (PSC+1) × (ARR+1) / TIM1CLK */
    float pwm_period = 2.0f * (float)(htim1.Init.Prescaler + 1U)
                            * (float)(htim1.Init.Period + 1U)
                            / (float)tim_clk;
    /* ×(RCR+1) 得到实际更新事件周期 = FOC 执行节拍 */
    return pwm_period * (float)(htim1.Init.RepetitionCounter + 1U);
}
static API_Sample_t SampleInterface = {
    .GetOffsetCurrent = GetOffsetCurrent,
    .GetPhaseCurrent = GetPhaseCurrent,
    .GetPwmPeriod = GetPwmPeriod,
    .GetTempture = GetTempture,
    .GetVoltageBus = GetVoltageBus,
};

void __PWM_Register__(g_MotorInterface_t *g_API_Interface)
{
    if (g_API_Interface != NULL)
        g_API_Interface->Sample = &SampleInterface;
}