#ifndef __CURRENT_H
#define __CURRENT_H

#include <stdint.h>
#include <stdbool.h>
#include "motorPara.h"

#define CURRENT_SAMPLE_NUM      (1024U)
#define CURRENT_AMP_GAIN        (10U)

#define CURRENT_SHUNT_OHM       (0.005f)
#define ADC_REF_VOLTAGE         (3.3f)
#define ADC_FULL_SCALE       	(4095.0f)
#define RAISE_VOLTAGE           (1.65f)
#define MOTOR_PARA_RESET        (0.0f)

void GetOffsetCurrent(Motor_t *motor);
float GetPhaseCurrent(Motor_t *motor, uint8_t phase);

#endif
