#ifndef __GLOBALCONTROL_H
#define __GLOBALCONTROL_H

#include "motorPara.h"
#include "adc.h"
/**
 * @brief   SVPWM control method sector define
 * @attention   SVPWM_SECTOR_METHOD : Seven stage SVPWM
 *              SVPWM_ZERO_SQUENCE  : Zero squence SVPWM
 */
#define SVPWM_ZERO_SQUENCE
#define SAMPLE_BUFFER   3

extern ADC_HandleTypeDef hadc2;

#endif
