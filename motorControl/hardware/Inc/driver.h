#ifndef __DRIVER_H
#define __DRIVER_H

#include "gpio.h"
#include "globalControl.h"

void MotorDriverEnable(void);
void MotorDriverDisable(void);
void LedON(Motor_t *motor);
void LedOFF(void);

#endif
