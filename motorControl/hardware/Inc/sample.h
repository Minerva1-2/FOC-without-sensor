#ifndef __SAMPLE_H
#define __SAMPLE_H

#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include "motorPara.h"

#define CURRENT_SAMPLE_NUM      (1024U)          // current sample times
#define CURRENT_AMP_GAIN        (10U)
#define ADC_REF_VOLTAGE         (3.3f)
#define ADC_FULL_SCALE       	(4095.0f)
#define CURRENT_SHUNT_OHM       (0.005f)
#define RAISE_VOLTAGE           (1.65f)
#define MOTOR_PARA_RESET        (0.0f)
#define VBUS_DIV_RATIO          (0.0449f)        // partial pressure coefficient
#define NTC_R_REF               (10000.0f)
#define NTC_B                   (0.0f)
#define NTC_R25                 (0.0f)

void GetOffsetCurrent(Motor_t *motor);
float GetPhaseCurrent(Motor_t *motor, uint8_t phase);
float GetVoltageBus(Motor_t *motor);

#endif
