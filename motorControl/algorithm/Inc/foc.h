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

void Clark(FOC_t *foc);                                        /* 三相电流 → αβ 电流 */
void Park(FOC_t *foc);                                         /* αβ 电流 → dq 电流 */
void AntiPark(FOC_t *foc);                                     /* dq 电压 → αβ 电压 */
float PIDCalc(PID_t *pid, float aimValue, float nowValue);     /* 通用位置式 PID(抗积分饱和) */
void PID(PID_t * PID, FOC_t *FOC, PLL_t *PLL);
void SVPWM(FOC_t *FOC, Current_t *Current, PWM_t *PWM);

#endif
