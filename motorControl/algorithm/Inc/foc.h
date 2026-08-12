#ifndef __FOC_H
#define __FOC_H

#include <math.h>
#include <stdint.h>
#include "motorPara.h"
#include "globalControl.h"
#include "stm32g4xx.h"

#define SQRT3_DIV_TWO       (0.86602540378f)    // √3 / 2
#define ONE_DIV_SQRT3       (0.57735026919f)    // 1 / √3
#define SPEED_LPF_ALPHA     (0.01f)  /* 一阶低通系数：α≈2π×fc×Ts，fc≈16Hz@Ts=100µs */

void Clark(Motor_t *motor);
void Park(Motor_t *motor);
void PID(Motor_t *motor);
void AntiPark(Motor_t *motor);
void SVPWM(Motor_t *motor);
float PIDCalc(PID_t *pid, float aimValue, float nowValue);
void SpeedFeedbackFiltInit(float omega_mech);
void FuncRegister(void (*UserFuncPID)(Motor_t *motor),
                  void (*UserFuncSVPWM)(Motor_t *motor));

#endif
