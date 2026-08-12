#ifndef __GLOBALCONTROL_H
#define __GLOBALCONTROL_H

#include <stdio.h>
#include "motorPara.h"
#include "observer.h"
/**
 * @brief   SVPWM control method sector define
 * @attention   SVPWM_SECTOR_METHOD : Seven stage SVPWM
 *              SVPWM_ZERO_SQUENCE  : Zero squence SVPWM
 */
#define SVPWM_ZERO_SQUENCE

#define SAMPLE_BUFFER                 3
#define TX_BUFFER_SIZE              128
#define CURRENT_FLAG_Ia             (0U)
#define CURRENT_FLAG_Ic             (1U)
#define PWM_ARR_ZERO                (0U)
/* 电流闭环强拖参数（参照盛浩开发板：无感强拖励磁电流仅需 0.05~0.1A，
   1A 电源即可启动；原 1A 把电源吃满导致 Vbus 跌到 7V、电机转不起来） */
#define ALIGN_ID_REF                (0.15f)          /* 预定位励磁电流(A)：磁极对齐，0.1~0.2A 足够 */
#define ALIGN_ANGLE                 (0.0f)           /* 预定位电角度(rad)，通常取 0 */
#define ALIGN_TIME_MS               (500U)           /* 预定位保持时间(ms)，重负载可加大 */
#define OPENLOOP_ID_REF             (0.15f)          /* 开环强拖励磁电流(A)：旋转磁场牵引转子 */
#define OPENLOOP_OMEGA_START        (2.0f * PI)      /* 起始电角速度：2Hz 电气 */
#define OPENLOOP_OMEGA_END          (80.0f * PI)     /* 切换电角速度：40Hz 电气（7对极≈343rpm）。
                                                        低速切换反电动势太小、SMO 不可靠，易在切换瞬间失控 */
#define OPENLOOP_ACCEL              (60.0f * PI)     /* 角加速度：每秒爬 20Hz（可调） */
#define OPENLOOP_STABLE_MS          (200U)           /* 到达目标转速后的稳定时间 */
#define MOTOR_SEPPD_COEFFICIENT     (1000U)           // 波轮电位器对应速度的一次函数系数
/* 开环→闭环切换收敛判据（仿盛浩板 StrongDragToObs：速度与角度同时收敛才切换） */
#define OPENLOOP_OBS_SPEED_ERR      (2.0f * PI)      /* 观测电角速度与开环给定误差容差(rad/s) */
#define OPENLOOP_OBS_OK_CNT         (100U)           /* 速度收敛需连续满足的周期数 */
#define OPENLOOP_OBS_THETA_ERR      (0.25f)          /* 观测电角度与开环角度误差容差(rad)≈14.3° */
#define OPENLOOP_OBS_ANGLE_OK_CNT   (200U)           /* 角度收敛需连续满足的周期数 */
#define CLOSE_LOOP_INIT_IQ          (0.1f)           /* 切入闭环时速度环初始输出/积分(A)：从强拖电流平滑接力，避免 Iq 阶跃 */
#define RUN_MIN_OMEGA               (20.0f * PI)     /* 闭环最低电角速度：低于 10Hz 电气判失速（原 5Hz 太低，低速反电动势不可靠） */
#define RUN_STALL_CNT               (1000U)
/* 母线欠压保护：低于阈值持续 BUS_UNDERVOLTAGE_CNT 个控制周期(50µs)即停机，切断 SVPWM 正反馈 */
#define BUS_UNDERVOLTAGE_THRESHOLD  (12.0f)          /* V，24V 电源拉低一半即判欠压 */
#define BUS_UNDERVOLTAGE_CNT        (100U)           /* 5ms 持续确认，抗启动/切换瞬态 */

void MotorAlignStart(void);
void TemperatureInit(Temperature_t *temp);
void MotorParaInit(Motor_t *motor);
const char *StateName(State_t state);
void TxMotorData(Motor_t *motor, Temperature_t *temp);
void MotorSpedControl(Motor_t *motor);

#endif
