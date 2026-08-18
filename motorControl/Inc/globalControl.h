#ifndef __GLOBALCONTROL_H
#define __GLOBALCONTROL_H

#include <stdio.h>
#include "motorPara.h"
#include "observer.h"
/**
 * @brief   SVPWM control method sector define
 * @attention   SVPWM_SECTOR_METHOD : Seven stage SVPWM
 *              SVPWM_ZERO_SQUENCE  : Zero squence SVPWM
 */
#define SVPWM_ZERO_SQUENCE

#define SAMPLE_BUFFER               3
#define TX_BUFFER_SIZE              128
#define PWM_ARR_ZERO                (0U)
#define MOTOR_SEPPD_COEFFICIENT     (1000U)           // 波轮电位器对应速度的一次函数系数

void TemperatureInit(Temperature_t *temp);
void MotorParaInit(Motor_t *motor);
const char *StateName(State_t state);
void TxMotorData(Motor_t *motor, Temperature_t *temp);

#endif
