#include <string.h>
#include "motorPara.h"

static Temperature_t *temperature = {0};
static Motor_t *motor = {0};

Temperature_t *GetTempStruct(void)
{
    return temperature;
} 

Motor_t *GetMotorStruct(void)
{
    return motor;
} 

g_MotorInterface_t g_API_Interface = {
    .Driver = NULL,
    .FOC = NULL,
    .Observer = NULL,
    .Sample = NULL,
    .TAccDec = NULL,
};

