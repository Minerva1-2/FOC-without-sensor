/*
 * @Author: Minerva1-2 18993035310@163.com
 * @Date: 2026-08-17 20:33:42
 * @LastEditors: Minerva1-2 18993035310@163.com
 * @LastEditTime: 2026-08-17 20:34:41
 * @FilePath: \MDK-ARMd:\cubemx\project\keil\FOC-without-sensor\motorControl\algorithm\Src\taccdec.c
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include <string.h>
#include "taccdec.h"

/**
 * @brief   梯形速度曲线
 * @attention   该函数传入目标转速为机械角速度，反映电机实际转速；输出转速为电角速度，该输出转速提供给速度环使用，因此为电角速度
 * @param   Motor_t *motor
 * @return  null
 */
static void TAccDec_Update(TAccDec_t *TAccDec)
{
    // 计算目标速度的每周期目标位置增量
    TAccDec->SpeedTargetIncrement = TAccDec->TargetSpeed / 60.0f * TAccDec->Ts;   /* 每周期速度步长(rad/s) = 加速度 × 周期 */
    // 计算实际每个周期能增加的速度步长
    TAccDec->SpeedIncrement = TAccDec->AccSpeed / 60.0f * TAccDec->Ts * TAccDec->Ts;
    // 根据当前积累位置增量与目标位置增量的关系，调整加速度方向
    if (TAccDec->SpeedChangeIncrement < TAccDec->SpeedTargetIncrement)
    {
        TAccDec->SpeedChangeIncrement += TAccDec->SpeedIncrement;            /* 加速：每周期增加一个速度步长 */
        if (TAccDec->SpeedChangeIncrement > TAccDec->SpeedTargetIncrement)
        {
            TAccDec->SpeedChangeIncrement = TAccDec->SpeedTargetIncrement;      /* 超过目标速度步长，则钳位至目标速度步长 */
        }
    }
    else if (TAccDec->SpeedChangeIncrement > TAccDec->SpeedTargetIncrement)
    {
        TAccDec->SpeedChangeIncrement -= TAccDec->SpeedIncrement;            /* 减速：每周期减少一个速度步长 */
        if (TAccDec->SpeedChangeIncrement < TAccDec->SpeedTargetIncrement)
        {
            TAccDec->SpeedChangeIncrement = TAccDec->SpeedTargetIncrement;
        }            /* 减速：每周期减少一个速度步长 */ 
    }
    // 当前周期速度变化量的增量
    TAccDec->SpeedIincrementDelta = fabsf(TAccDec->SpeedChangeIncrement) - TAccDec->SpeedIncrementLast;
    // 保存这一周期的速度变化增量
    TAccDec->SpeedIncrementLast = fabsf(TAccDec->SpeedChangeIncrement);
    //电机状态设置：TAccDec->SpeedIincrementDelta为当前周期速度变化量的增量，大于0则加速，小于0则减速，等于0则匀速
    TAccDec->State = TAccDec->SpeedIincrementDelta > 0 ? TACC_ACCELERATE :
                    (TAccDec->SpeedIincrementDelta < 0 ? TACC_DECELERATE : TACC_UNIFORM);
    // 输出转速：由于该转速给到速度环，因此乘以极对数转换为电角速度使用
    TAccDec->SpeedOut = TAccDec->SpeedChangeIncrement * 60.0f / TAccDec->Ts * MOTOR_POLE_PAIRS;
}

static API_TAccDec_t TAccDecInterface = {
    .TAccDec_Update = TAccDec_Update,
};

void TAccDec_Register(g_MotorInterface_t *iface)
{
    if (iface != NULL)
        iface->TAccDec = &TAccDecInterface;
}