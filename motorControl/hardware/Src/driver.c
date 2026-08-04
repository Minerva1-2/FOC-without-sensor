#include "driver.h"
/**
 * @fn  void MotorDriverEnable(void)
 * @brief   mos enable
 * @param   null
 * @return  null
 */
void MotorDriverEnable(void)
{
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);     //A
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);     //B
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET);     //C
}
/**
 * @fn  void MotorDriverDisable(void)
 * @brief   mos disable
 * @param   null
 * @return  null
 */
void MotorDriverDisable(void)
{
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);
}