/*
 * @Author: Minerva1-2 18993035310@163.com
 * @Date: 2026-08-15 10:58:38
 * @LastEditors: Minerva1-2 18993035310@163.com
 * @LastEditTime: 2026-08-16 20:47:31
 * @FilePath: \MDK-ARMd:\cubemx\project\keil\FOC-without-sensor\motorControl\Src\motorPara.c
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
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

const g_MotorInterface_t g_API_Interface = {
    .Driver = NULL,
    .FOC = NULL,
    .Observer = NULL,
    .Sample = NULL,
    .TAccDec = NULL,
};

