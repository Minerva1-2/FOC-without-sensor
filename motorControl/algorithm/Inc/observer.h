#ifndef __OBSEVER_H
#define __OBSEVER_H

#include <math.h>
#include "motorPara.h"
#include "globalControl.h"

#define OMEGA_MAX       (1500.f)
#define PLL_E_MIN_SQ    (0.25f)
#define SMO_LPF_ALPHA   (0.1f)

void Observer_Register(g_MotorInterface_t *iface);
#endif
