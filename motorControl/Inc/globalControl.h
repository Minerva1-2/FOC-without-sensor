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

/* 预定位参数：进入开环强拖前，固定电角度施加 Id 励磁，把转子吸到已知位置 */
#define ALIGN_ID_REF                (0.2f)            /* 预定位励磁电流(A)，1A 电源下有余量 */
#define ALIGN_ANGLE                 (0.0f)            /* 预定位电角度(rad) */
#define ALIGN_TIME_MS               (500U)            /* 预定位保持时间(ms) */

void TemperatureInit(Temperature_t *temp);
void MotorParaInit(Motor_t *motor);
void FuncRegister(void);
const char *StateName(State_t state);
void TxMotorData(Motor_t *motor, Temperature_t *temp);

#endif
