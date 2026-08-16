#include <string.h>
#include "motorPara.h"

static Temperature_t *temperature = {0};
static Motor_t *motor = {0};

State_t *GetStateStruct(void)
{
    return &motor->State;
}
Error_t *GetErrorStruct(void)
{
    return &motor->Error;
} 
PID_t *GetPIDIdStruct(void)
{
    return &motor->PID_Id;
}
PID_t *GetPIDIqStruct(void)
{
    return &motor->PID_Iq;
}
FOC_t *GetFOCStruct(void)
{
    return &motor->FOC;
} 
SMO_t *GetSMOStruct(void)
{
    return &motor->SMO;
}
PLL_t *GetPLLStruct(void)
{
    return &motor->PLL;
} 
Current_t *GetCurrentStruct(void)
{
    return &motor->Current;
}
Temperature_t *GetTempStruct(void)
{
    return temperature;
} 

Motor_t *GetMotorStruct(void)
{
    return motor;
} 

const g_MotorInterface_t g_Interface_API = {
    .Driver = NULL,
    .FOC = NULL,
    .Observer = NULL,
    .Sample = NULL,
};

