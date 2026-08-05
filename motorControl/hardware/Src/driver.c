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
/**
 * @fn  void LedON(Motor_t *motor)
 * @brief   R:PC15, G:PC14, B:PC13; 0:Led on, 1:Led off
 * @param   Motor_t *motor
 * @return  null
 */
void LedON(Motor_t *motor)
{
    if (MOTOR_RUN == motor->State)
    {
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_SET);
    }
    else if (MOTOR_STOP == motor->State)
    {
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_SET);
    }
    else if (MOTOR_FAULT == motor->State)
    {
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_SET);
    }
}
/**
 * @fn  void LedOFF(void)
 * @brief   turn off the whole Leds
 * @param   null
 * @return  null
 */
void LedOFF(void)
{
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_SET);
}