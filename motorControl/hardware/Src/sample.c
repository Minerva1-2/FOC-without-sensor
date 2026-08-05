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

static float g_offset_Ia, g_offset_Ic;
static uint16_t g_current_sample_num;

static bool g_current_offset_state = false;

/**
 * @fn      void GetOffsetCurrent(MotorFOC_t *motor)
 * @param   Motor_t *motor
 * @return  null
 * @brief   get the offset voltage when the motor is static
 */
void GetOffsetCurrent(Motor_t *motor)
{
    if (motor->State != MOTOR_STOP)
        return;
    if (g_current_offset_state)
        return;

	g_offset_Ia += motor->Current.current_adc_a;
    g_offset_Ic += motor->Current.current_adc_c;

    g_current_sample_num++;

    if (g_current_sample_num >= CURRENT_SAMPLE_NUM)
    {
        motor->Current.current_offset_Ia = (float)g_offset_Ia / (float)CURRENT_SAMPLE_NUM;
        motor->Current.current_offset_Ic = (float)g_offset_Ic / (float)CURRENT_SAMPLE_NUM;
    
        g_current_sample_num = 0;
        g_offset_Ia = MOTOR_PARA_RESET;
        g_offset_Ic = MOTOR_PARA_RESET;
        g_current_offset_state = true;
    }
}
/**
 * @fn      float GetPhaseCurrent(MotorFOC_t *motor, uint8_t phase_flag)
 * @param   Motor_t *motor, uint8_t phase_flag
 * @return  voltage / (CURRENT_AMP_GAIN * CURRENT_SHUNT_OHM)
 * @brief   this function gather real phase current when the motor is running
 */
float GetPhaseCurrent(Motor_t *motor, uint8_t phase_flag)
{
    uint16_t adc = 0;
    float offset_voltage = 0.0f;

    if (0U == phase_flag)
    {
        offset_voltage = motor->Current.current_offset_Ia;
        adc = motor->Current.current_adc_a;
    }
    else if (1U == phase_flag)
    {
        offset_voltage = motor->Current.current_offset_Ic;
        adc = motor->Current.current_adc_c;
    }
    else {
        return MOTOR_PARA_RESET;
    }

    float voltage = ((float)adc - offset_voltage) * ADC_REF_VOLTAGE / ADC_FULL_SCALE;
    
    return (voltage / (CURRENT_AMP_GAIN * CURRENT_SHUNT_OHM));
}
