#ifndef __MOTORPARA_H
#define __MOTORPARA_H

#include <stdint.h>
#include <stdbool.h>

#define _constrain(output, max, min)      (output > max ? max : (output < min ? min : output))

#define MOTOR_POLE_PAIRS    (7U)
#define MOTOR_PHASE_R       (0.206f)
#define MOTOR_PHASE_L       (0.000046f)

/**
 * @brief   电机参数结构体
 * @attention   该结构体只可在中断中写入，其他任何文件中都为只读状态，即该结构体
 *              为单一写入口、多读出口结构体，防止参数被多处修改
 *  */
typedef enum 
{
    MOTOR_OPENLOOP_CURRENT_OPEN,        /* 开环强拖：速度、电流开环 */   
    MOTOR_OPENLOOP_CURRENT_CLOSE,       /* 开环强拖：速度开环、电流闭环，旋转磁场牵引转子 */
    MOTOR_RUN,                          /* 闭环运行：SMO+PLL 速度/电流双闭环 */
    MOTOR_STOP,                         /* 停机：零占空比 */
}State_t;
/**
 * 电机错误状态
 */
typedef enum
{
    MOTOR_MODE_CHANGE_ERROR,
}Error_t;
/* PID参数 */
typedef struct
{
    float p;
    float i;
    float d;
    float OutputMin;                                // 输出限幅
    float OutputMax;
    float Output;
    float lastError;                              // last error
    float prevError;                              // the first two times error
    float nowValue;
    volatile float aimValue;
    float integral;
}PID_t;
/* 电气参数 */
typedef struct
{
    float Ia, Ib, Ic;                               // 相电流值
    float Va, Vb, Vc;
    float I_alpha, I_beta;                          // clark变换之后的电流值
    float V_alpha, V_beta; 
    float Iq, Id;                                   // park变换后的数值
    float Vq, Vd;
    float angle;                                    // 电角度
} FOC_t;
typedef struct
{
    float I_alpha_hat, I_beta_hat;                  // 观测器预估电流
    float e_alpha_hat, e_beta_hat;                  // 观测器预估反电动势
    float e_alpha_raw, e_beta_raw;                  // 观测器实际输出反电动势
    float K_slide;                                  // 滑膜增益
    float wc;                                       // 低通滤波截止频率
}SMO_t;
typedef struct
{
    float theta_hat, omerga_hat;                    // 鉴相器预估电角度、电角速度
    float theta_error;                              // 角度误差
    float p, i;                                     // pid参数     
    float integral;                                 // 积分数值
}PLL_t;
/* current sample struct */
typedef struct 
{
    uint16_t current_adc_a;                         // adc电流采样实际值
    uint16_t current_adc_c;
    uint16_t adc_postion;                           // 波轮电位器 ADC 原始值
    float current_offset_Ia;                        // 相电流数值
    float current_offset_Ic;
    float current_phase_Ia;
    float current_phase_Ib;
    float current_phase_Ic;
    float voltage_bus;                              // 母线电压数值
    float adc_bus;                                  // 母线adc数值
    float pot_ratio;                                // 波轮电位器归一化比例 0~1
    bool g_current_offset_state;                    // 偏置电流采样状态
}Current_t;
typedef struct
{
    float Duty_a;                                   // 占空比数值
    float Duty_b;
    float Duty_c;
    float pwm_period;                               // PWM运行时间
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
    Error_t Error;
    Current_t Current;
    PID_t PID_Speed;
    PID_t PID_Iq;
    PID_t PID_Id;
    FOC_t FOC;
    SMO_t SMO;
    PLL_t PLL;
    PWM_t PWM;
}Motor_t;

PID_t *GetPIDStruct(void);
FOC_t *GetFOCStruct(void);
SMO_t *GetSMOStruct(void);
PLL_t *GetPLLStruct(void);
Current_t *GetCurrentStruc(void);
PWM_t *GetPWMStruct(void);
Temperature_t *GetTemperatureStruct(void);
Motor_t *GetMotorStruct(void);

#endif
