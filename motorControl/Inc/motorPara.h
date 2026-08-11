#ifndef __MOTORPARA_H
#define __MOTORPARA_H

#include <stdint.h>
#include <stdbool.h>

#define _constrain(output, max, min)      (output > max ? max : (output < min ? min : output))

#define MOTOR_POLE_PAIRS    (7U)
#define MOTOR_PHASE_R       (0.085f)
#define MOTOR_PHASE_L       (0.000025f)

/* 电机状态参数 */
typedef enum {
    MOTOR_RUN,
    MOTOR_STOP,
    MOTOR_ALIGN,
    MOTOR_OPENLOOP,
}State_t;
/* PID参数 */
typedef struct
{
    float p;
    float i;
    float d;
    float OutputMin;                                // min output limiting
    float OutputMax;                                // max output limiting
    float Output;
    int32_t lastError;                              // last error
    int32_t prevError;                              // the first two times error
    int32_t nowValue;
    int32_t aimValue;
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
    uint16_t adc_postion;                           // 波轮电位器 ADC 原始值
    float current_offset_Ia;                        // offset current value
    float current_offset_Ic;
    float current_phase_Ia;
    float current_phase_Ib;
    float current_phase_Ic;
    float voltage_bus;                              // bus votage value
    float adc_bus;                                  // bus adc value
    float pot_ratio;                                // 波轮电位器归一化比例 0~1
    bool g_current_offset_state;                    // 偏置电流采样状态
}Current_t;
typedef struct
{
    float Duty_a;                                   // PWM output value
    float Duty_b;
    float Duty_c;
    float pwm_period;                               // pwm period
}PWM_t;
typedef struct
{
    uint16_t adc_value;                             // adc采样数据
    float B_value;                                  // B值
    float temperature_ref;                          // 参考温度值
    float resistor_ref;                             // 参考温度下的阻值
    float curr_resistor;                            // 当前阻值
    float ln_value;                                 // 自然对数值
    float resistor_other;                           // 分压电阻阻值
    float curr_temp;                                // 当前温度
}Temperature_t;
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

Temperature_t *GetTemperatureStruct(void);
Motor_t *GetMotorStruct(void);

#endif
