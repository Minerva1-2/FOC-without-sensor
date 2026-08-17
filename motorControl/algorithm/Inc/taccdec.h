#ifndef __TACCDEC_H
#define __TACCDEC_H

#include <math.h>
#include <stdint.h>
#include "motorPara.h"

/* 梯形加减速（T 型速度曲线规划），单位：电气角速度 rad/s。
   每周期按恒定加速度 AccSpeed 逼近 TargetSpeed，到达后钳位（防超调）。
   速度-时间曲线呈梯形：恒加速段 → 匀速段 → 恒减速段。 */
void TAccDec_Register(g_MotorInterface_t *iface);

#endif /* __TACCDEC_H */
