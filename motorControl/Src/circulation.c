#include "motorPara.h"

static Motor_t motor = {0};

Motor_t *GetMotorStruct(void)
{
    return &motor;
}