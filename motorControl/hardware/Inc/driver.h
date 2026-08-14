#ifndef __DRIVER_H
#define __DRIVER_H

#include "gpio.h"
#include "globalControl.h"
#include "tim.h"

typedef struct
{
    void (*MotorDriverEnable)(void);
    void (*MotorDriverDisable)(void);
    void (*LEDControl)(State_t state);
    void (*SetPWMValue)(uint32_t PWMValue_A, uint32_t PWMValue_B, uint32_t PWMValue_C);
}Driver_t;

const Driver_t Driver;

#endif
