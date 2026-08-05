#ifndef __MOTORPARA_H
#define __MOTORPARA_H

#include <stdint.h>
#include "globalControl.h"

#define _constrain(output, max, min)      (output > max ? max : (output < min ? min : output))

#define MOTOR_POLE_PAIRS    (7U)
#define MOTOR_PHASE_R       (0.085f)
#define MOTOR_PHASE_L       (0.000025f)

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
typedef struct
{
    float I_alpha_hat, I_beta_hat;                  // observe current
    float e_alpha_hat, e_beta_hat;                  // estimate reverse electromotive force (filtering)
    float e_alpha_raw, e_beta_raw;                  // smo wsitch output(no filtering)
    float K_slide;                                  // smo gain K
    float wc;                                       // low-pass filter cutoff angular frequency
}SMO_t;
typedef struct
{
    float theta_hat, omerga_hat;                    // PLL output:estimate angle and electrical angular velocity
    float theta_error;                              // angle error
    float p, i;                                     // PLL PID parameter     
    float integral;                                 // PLL integral parameter
}PLL_t;
/* current sample struct */
typedef struct 
{
    uint16_t current_adc_a;                         // ADC sampled current raw value
    uint16_t current_adc_c;
    float current_offset_Ia;                        // offset current value
    float current_offset_Ic;
    float voltage_bus;                              // bus votage
}Current_t;
typedef struct
{
    float Duty_a;                                   // PWM output value
    float Duty_b;
    float Duty_c;
    float pwm_period;                               // pwm period
}PWM_t;
/* 电机结构体 */
typedef struct 
{
    State_t State;
    Current_t Current;                              // motor state
    PID_t PID;                                      // motor PID parameter
    FOC_t FOC;                                      // motor FOC parameter
    SMO_t SMO;                                      // motor SMO parameter
    PLL_t PLL;                                      // motor PLL parameter
    PWM_t PWM;                                      // motor Pwm parameter
}Motor_t;

Motor_t *GetMotorStruct(void);

#endif
