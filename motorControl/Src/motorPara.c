#include "motorPara.h"

static PID_t pid = {0};
static FOC_t foc = {0};
static SMO_t smo = {0};
static PLL_t pll = {0};
static Current_t current = {0};
static PWM_t pwm = {0};
static Temperature_t temp = {0};
static Motor_t motor = {0};

PID_t *GetPIDStruct(void)
{
    return &pid;
}
FOC_t *GetFOCStruct(void)
{
    return &foc;
}
SMO_t *GetSMOStruct(void)
{
    return &smo;
}
PLL_t *GetPLLStruct(void)
{
    return &pll;
}
Current_t *GetCurrentStruct(void)
{
    return &current;
}
PWM_t *GetPWMStruct(void)
{
    return &pwm;
}
Temperature_t *GetTemperatureStruct(void)
{
    return &temp;
}
Motor_t *GetMotorStruct(void)
{
    return &motor;
}
