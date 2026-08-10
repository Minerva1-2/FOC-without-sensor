#include "motorPara.h"

static Temperature_t temp = {0};
static Motor_t motor = {0};

Temperature_t *GetTemperatureStruct(void)
{
    return &temp;
}
Motor_t *GetMotorStruct(void)
{
    return &motor;
}