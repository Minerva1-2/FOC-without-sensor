#ifndef __STATUESWITCH_H
#define __STATUESWITCH_H

#include "motorPara.h"

#define CURRENT_FLAG_Ia             (0U)
#define CURRENT_FLAG_Ic             (1U)

void FOC_Register(g_MotorInterface_t *iface);

#endif
