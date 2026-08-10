#ifndef __OBSEVER_H
#define __OBSEVER_H

#include <math.h>
#include "motorPara.h"
#include "globalControl.h"

#define PI             (3.14159265f)
#define TWO_PI         (6.28318530f)
#define OMEGA_MAX      (1500.f)

void ObserverSMO(Motor_t *motor);
void PLL(Motor_t *motor);

#endif
