#ifndef __GLOBALCONTROL_H
#define __GLOBALCONTROL_H

#include "motorPara.h"

/**
 * @brief   SVPWM control method sector define
 * @attention   SVPWM_SECTOR_METHOD : Seven stage SVPWM
 *              SVPWM_ZERO_SQUENCE  : Zero squence SVPWM
 */
#define SVPWM_ZERO_SQUENCE
/**
 * @brief   FOC sensor sector define
 * @attention   FOC_WITHOUT_SENSOR : FOC without sensor
 *              FOC_WITH_SENSOR    : FOC with sensor
 */
#define FOC_WITHOUT_SENSOR

#endif
