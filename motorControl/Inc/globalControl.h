#ifndef __GLOBALCONTROL_H
#define __GLOBALCONTROL_H

#include "motorPara.h"
#include "observer.h"
/**
 * @brief   SVPWM control method sector define
 * @attention   SVPWM_SECTOR_METHOD : Seven stage SVPWM
 *              SVPWM_ZERO_SQUENCE  : Zero squence SVPWM
 */
#define SVPWM_ZERO_SQUENCE

#define SAMPLE_BUFFER       3
#define CURRENT_FLAG_Ia     (0U)
#define CURRENT_FLAG_Ic     (1U)
#define PWM_ARR_ZERO        (0U)
#define ALIGN_VOLTAGE   (1.5f)                  /* 预定位电压幅值(V)，按母线电压/负载调节 */
#define ALIGN_ANGLE     (0.0f)                  /* 预定位电角度(rad)，通常取 0 */
#define ALIGN_TIME_MS   (500U)                  /* 预定位保持时间(ms)，重负载可加大 */
#define OPENLOOP_OMEGA_START   (2.0f * PI)      /* 起始电角速度：2Hz 电气 */
#define OPENLOOP_OMEGA_END     (20.0f * PI)     /* 切换电角速度：20Hz 电气 */
#define OPENLOOP_ACCEL         (40.0f * PI)     /* 角加速度：每秒爬 20Hz（可调） */
#define OPENLOOP_VOLTAGE_START (1.5f)           /* 起始电压，与预定位电压衔接 */
#define OPENLOOP_VOLTAGE_END   (6.0f)           /* 切换时电压 */
#define OPENLOOP_STABLE_MS     (200U)           /* 到达目标转速后的稳定时间 */

void MotorAlignStart(void);
void TemperatureInit(Temperature_t *temp);
void MotorParaInit(Motor_t *motor);

#endif
