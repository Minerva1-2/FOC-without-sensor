/*
 * @Author: Minerva1-2 18993035310@163.com
 * @Date: 2026-08-15 10:58:38
 * @LastEditors: Minerva1-2 18993035310@163.com
 * @LastEditTime: 2026-08-15 13:51:09
 * @FilePath: \MDK-ARMd:\cubemx\project\keil\FOC-without-sensor\motorControl\algorithm\Inc\foc.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#ifndef __FOC_H
#define __FOC_H

#include <math.h>
#include <stdint.h>
#include "motorPara.h"
#include "globalControl.h"
#include "stm32g4xx.h"

#define SQRT3_DIV_TWO       (0.86602540378f)    // √3 / 2
#define ONE_DIV_SQRT3       (0.57735026919f)    // 1 / √3
#define SPEED_LPF_ALPHA     (0.01f)  /* 一阶低通系数：α≈2π×fc×Ts，fc≈16Hz@Ts=100µs */

void FOC_Register(g_MotorInterface_t *iface);

#endif
