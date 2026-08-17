#ifndef __SAMPLE_H
#define __SAMPLE_H

#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include "tim.h"
#include "motorPara.h"

#define CURRENT_SAMPLE_NUM      (1024U)          // current sample times
#define CURRENT_AMP_GAIN        (10U)
#define ADC_REF_VOLTAGE         (3.3f)
#define ADC_FULL_SCALE       	(4095.0f)
#define CURRENT_SHUNT_OHM       (0.005f)
#define RAISE_VOLTAGE           (1.65f)
#define MOTOR_PARA_RESET        (0.0f)
#define VBUS_DIV_RATIO          (0.0449f)        // partial pressure coefficient
#define NTC_R_REF               (10000.0f)
#define NTC_B                   (0.0f)
#define NTC_R25                 (0.0f)
#define K_TEMPERATURE           (273.15f)       // 开式温度
#define TEMP_SAMPLE_NUM         (1000U)         // 温度计算节流：约每 1000 次 ADC 完成回调算一次

void Sample_Register(g_MotorInterface_t *iface);

#endif
