#ifndef __FOC_H
#define __FOC_H

#include <math.h>
#include <stdint.h>
#include "motorPara.h"
#include "globalControl.h"

#define SQRT3_DIV_TWO   (0.86602540378f)    // √3 / 2
#define ONE_DIV_SQRT3   (0.57735026919f)    // 1 / √3

void Clark(Motor_t *motor);
void Park(Motor_t *motor);
void Pid(Motor_t *motor);
void AntiPark(Motor_t *motor);

#if (defined (SVPWM_SECTOR_METHOD))
    void SVPWMSectorMethod(Motor_t *motor);
#else
    void SVPWM_ZeroSequence(Motor_t *motor);
#endif

#endif
