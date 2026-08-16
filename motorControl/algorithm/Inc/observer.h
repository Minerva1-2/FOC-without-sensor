#ifndef __OBSEVER_H
#define __OBSEVER_H

#include <math.h>
#include "motorPara.h"
#include "globalControl.h"

#define PI              (3.14159265f)
#define TWO_PI          (6.28318530f)
#define OMEGA_MAX       (1500.f)
#define PLL_E_MIN_SQ    (0.25f)
#define SMO_LPF_ALPHA   (0.1f)

void __PWM_Register__(g_MotorInterface_t *g_API_Interface);
#endif
