/*
 * @Author: Minerva1-2 18993035310@163.com
 * @Date: 2026-08-15 10:58:38
 * @LastEditors: Minerva1-2 18993035310@163.com
 * @LastEditTime: 2026-08-15 10:59:51
 * @FilePath: \MDK-ARMd:\cubemx\project\keil\FOC-without-sensor\motorControl\hardware\Src\driver.c
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "driver.h"
/**
 * @fn  void MotorDriverEnable(void)
 * @brief   EG2104 enable
 * @param   null
 * @return  null
 */
static void MotorDriverEnable(void)
{
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET); // A
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET); // B
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET); // C
}
/**
 * @fn  void MotorDriverDisable(void)
 * @brief   EG2104 disable
 * @param   null
 * @return  null
 */
static void MotorDriverDisable(void)
{
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);
}
/**
 * @fn  void LedControl(State_t state)
 * @brief   R:PC15, G:PC14, B:PC13; 0:Led on, 1:Led off
 * @param   State_t state
 * @return  null
 */
static void LedControl(MOTION_STATE state)
{
    if (state == TACC_UNIFORM)
    {
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET); // ALL
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_RESET);
    }
    else if (state == TACC_DECELERATE)
    {
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET); // Blue
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_SET);
    }
    else if (state == TACC_ACCELERATE)
    {
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_RESET); // Green
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_SET);
    }
    else
    {
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_RESET); //Red
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_SET);
    }
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
    if (PWMValue_A > period)
        PWMValue_A = period;
    if (PWMValue_B > period)
        PWMValue_B = period;
    if (PWMValue_C > period)
        PWMValue_C = period;

    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, PWMValue_A);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, PWMValue_B);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, PWMValue_C);
}
static API_Driver_t DriverInterface = {
    .MotorDriverEnable = MotorDriverEnable,
    .MotorDriverDisable = MotorDriverDisable,
    .LedControl = LedControl,
    .SetPWMValue = SetPWMValue,
};

void Driver_Register(g_MotorInterface_t *iface)
{
    if (iface != NULL)
        iface->Driver = &DriverInterface;
}