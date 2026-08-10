#include "globalControl.h"
#include "motorPara.h"
#include "sample.h"

static uint16_t g_adc_buf[SAMPLE_BUFFER];

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance == ADC2)
    {
        Motor_t *motor = GetMotorStruct();

        motor->Current.adc_bus = g_adc_buf[0];
        motor->Current.voltage_bus = GetVoltageBus(motor);
    }
}
void MotorParaInit(Motor_t *motor)
{
    HAL_ADC_Start_DMA(&hadc2, (uint32_t *)g_adc_buf, 3);
}