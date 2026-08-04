#ifndef __MOTORPARA_H
#define __MOTORPARA_H

#include <stdint.h>
#include "globalControl.h"

/* 电机状态参数 */
typedef enum {
    MOTOR_RUN,
    MOTOR_STOP,
    MOTOR_FAULT,
}State_t;
/* PID参数 */
typedef struct
{
    float p;
    float i;
    float d;
    float OutputMin;                                // min output limiting
    float OutputMax;                                // max output limiting
    int32_t lastError;                              // last error
    int32_t prevError;                              // the first two times error
}PID_t;
/* 电气参数 */
typedef struct
{
    float Ia, Ib, Ic;                               // phase current value
    float I_alpha, I_beta;                          // static coordinate system current value
    float V_alpha, V_beta; 
    float Iq, Id;                                   // rotating coordinate system current value
    float Vq, Vd;
    float angle;                                    // rotor angle
} FOC_t;
/* current sample struct */
typedef struct 
{
    uint16_t current_adc_a;                         // ADC sampled current raw value
    uint16_t current_adc_c;                         // offset current value
    float current_offset_Ia;
    float current_offset_Ic;
    float voltage_bus;                              // bus votage
}Current_t;
typedef struct
{
    float Duty_a;
    float Duty_b;
    float Duty_c;
    #if (defined (SVPWM_SECTOR_METHOD))
        float pwm_period;                           // pwm period
    #endif
}PWM_t;
/* 电机结构体 */
typedef struct 
{
    State_t State;
    Current_t Current;                              // motor state
    PID_t PID;                                      // motor PID parameter
    FOC_t FOC;                                      // motor FOC parameter
    PWM_t PWM;
}Motor_t;

Motor_t *GetMotorStruct(void);

#endif
