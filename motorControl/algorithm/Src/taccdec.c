#include <string.h>
#include "taccdec.h"

// void TAccDec_Init(TAccDec_t *t, float acc, float ts)
// {
//     t->TargetSpeed = 0.0f;
//     t->SpeedOut = 0.0f;
//     t->AccSpeed = acc;
//     t->Ts = ts;
// }
/**
 * @brief   梯形速度曲线
 * @attention   该函数传入目标转速为机械角速度，反映电机实际转速；输出转速为电角速度，该输出转速提供给速度环使用，因此为电角速度
 * @param   Motor_t *motor
 * @return  null
 */
static void TAccDec_Update(Motor_t *motor)
{
    // 计算目标速度的每周期目标位置增量
    motor->TAccDec.SpeedTargetIncrement = motor->TAccDec.TargetSpeed / 60.0f * motor->pwm_period;   /* 每周期速度步长(rad/s) = 加速度 × 周期 */
    // 计算实际每个周期能增加的速度步长
    motor->TAccDec.SpeedIncrement = motor->TAccDec.AccSpeed / 60.0f * motor->pwm_period * motor->pwm_period;
    // 根据当前积累位置增量与目标位置增量的关系，调整加速度方向
    if (motor->TAccDec.SpeedChangeIncrement < motor->TAccDec.SpeedTargetIncrement)
    {
        motor->TAccDec.SpeedChangeIncrement += motor->TAccDec.SpeedIncrement;            /* 加速：每周期增加一个速度步长 */
        if (motor->TAccDec.SpeedChangeIncrement > motor->TAccDec.SpeedTargetIncrement)
        {
            motor->TAccDec.SpeedChangeIncrement = motor->TAccDec.SpeedTargetIncrement;      /* 超过目标速度步长，则钳位至目标速度步长 */
        }
    }
    else if (motor->TAccDec.SpeedChangeIncrement > motor->TAccDec.SpeedTargetIncrement)
    {
        motor->TAccDec.SpeedChangeIncrement -= motor->TAccDec.SpeedIncrement;            /* 减速：每周期减少一个速度步长 */
        if (motor->TAccDec.SpeedChangeIncrement < motor->TAccDec.SpeedTargetIncrement)
        {
            motor->TAccDec.SpeedChangeIncrement = motor->TAccDec.SpeedTargetIncrement;
        }            /* 减速：每周期减少一个速度步长 */ 
    }
    // 当前周期速度变化量的增量
    motor->TAccDec.SpeedIincrementDelta = fabsf(motor->TAccDec.SpeedChangeIncrement) - motor->TAccDec.SpeedIncrementLast;
    // 保存这一周期的速度变化增量
    motor->TAccDec.SpeedIncrementLast = fabsf(motor->TAccDec.SpeedChangeIncrement);
    //电机状态设置：motor->TAccDec.SpeedIincrementDelta为当前周期速度变化量的增量，大于0则加速，小于0则减速，等于0则匀速
    motor->State = motor->TAccDec.SpeedIincrementDelta > 0 ? TACC_ACCELERATE :
                    (motor->TAccDec.SpeedIincrementDelta < 0 ? TACC_DECELERATE : TACC_UNIFORM);
    // 输出转速：由于该转速给到速度环，因此乘以极对数转换为电角速度使用
    motor->TAccDec.SpeedOut = motor->TAccDec.SpeedChangeIncrement * 60.0f / motor->pwm_period * MOTOR_POLE_PAIRS;
}

static API_TAccDec_t TAccDecInterface = {
    .TAccDec_Update = TAccDec_Update,
};

void TAccDec_Register(g_MotorInterface_t *iface)
{
    if (iface != NULL)
        iface->TAccDec = &TAccDecInterface;
}