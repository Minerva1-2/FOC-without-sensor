#ifndef __GLOBALCONTROL_H
#define __GLOBALCONTROL_H

#include "motorPara.h"
/**
 * @brief   SVPWM control method sector define
 * @attention   SVPWM_SECTOR_METHOD : Seven stage SVPWM
 *              SVPWM_ZERO_SQUENCE  : Zero squence SVPWM
 */
#define SVPWM_ZERO_SQUENCE
#define SAMPLE_BUFFER       3
#define CURRENT_FLAG_Ia     (0U)
#define CURRENT_FLAG_Ic     (1U)

void TemperatureInit(Temperature_t *temp);
void MotorParaInit(Motor_t *motor);

#endif
