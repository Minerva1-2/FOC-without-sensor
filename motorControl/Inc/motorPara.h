#ifndef __MOTORPARA_H
#define __MOTORPARA_H

#include <stdint.h>
#include <stdbool.h>

#define _constrain(output, max, min)      (output > max ? max : (output < min ? min : output))

#define MOTOR_POLE_PAIRS    (7U)
#define MOTOR_PHASE_R       (0.206f)
#define MOTOR_PHASE_L       (0.000046f)
#define PI                  (3.14159265f)
#define TWO_PI              (6.28318530f)
#define SPEED_LPF_ALPHA     (0.1f)              /* 一阶低通系数 */
#define SQRT3_DIV_TWO       (0.86602540378f)    // √3 / 2
#define ONE_DIV_SQRT3       (0.57735026919f)    // 1 / √3
/**********************************************************************************************************/
/*******************************************电机结构体******************************************************/
/**********************************************************************************************************/
typedef enum 
{
    MOTOR_RUN,                          /* 闭环运行：SMO+PLL 速度/电流双闭环 */
    MOTOR_STOP,                         /* 停机：零占空比 */
}State_t;

typedef enum 
{
    OPEN_LOOP,
    CLOSE_LOOP,
}GeneralMode_t;

typedef enum 
{
    TACC_UNIFORM,                       /* 匀速（已到达目标） */
    TACC_ACCELERATE,                    /* 加速 */
    TACC_DECELERATE,                    /* 减速 */
}MOTION_STATE;

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
    int8_t SpeedCalculateCnt;
}PID_t;
/* 电气参数 */
typedef struct
{
    float Ia, Ib, Ic;                               // 相电流值
    float Va, Vb, Vc;
    float I_alpha, I_beta;                          // clark变换之后的电流值
    float V_alpha, V_beta; 
    float Iq, Id;                                   // park变换后的数值
    float Iq_ref, Id_ref;
    float Vq, Vd;
    float Voltage_bus;                              // 母线电压
    float angle;                                    // 电角度
    float DutyCycleA;                                   // 占空比数值
    float DutyCycleB;
    float DutyCycleC;
    float Ts;
} FOC_t;
typedef struct
{
    float I_alpha_hat, I_beta_hat;                  // 观测器预估电流
    float e_alpha_hat, e_beta_hat;                  // 观测器预估反电动势
    float e_alpha_raw, e_beta_raw;                  // 观测器实际输出反电动势
    float V_alpha, V_beta;
    float I_alpha, I_beta;
    float K_slide;                                  // 滑膜增益
    float wc;                                       // 低通滤波截止频率
    float Ts;
}SMO_t;
typedef struct
{
    float theta_hat, omerga_hat;                    // 鉴相器预估电角度、电角速度
    float theta_error;                              // 角度误差
    float p, i;                                     // pid参数     
    float integral;                                 // 积分数值
    float e_alpha_hat, e_beta_hat;                  // 观测器预估反电动势
    float theta_hat_lpf, omerga_hat_lpf;
    float Ts;
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

typedef struct {
    float SpeedTargetIncrement;
    float SpeedChangeIncrement;
    float SpeedIincrementDelta;
    float SpeedIncrementLast;
    float SpeedIncrement;
    float TargetSpeed;      /* 目标电气角速度(rad/s)，应用层每周期更新 */
    float SpeedOut;         /* 输出电气角速度(rad/s)，作为速度环目标 */
    float AccSpeed;         /* 加速度(rad/s²)，加减速共用 */
    float Ts;
    MOTION_STATE State;
} TAccDec_t;

/* 电角度发生器：开环强拖阶段产生旋转电角度。
   角速度：电气 rpm（可与 TAccDec.SpeedOut 直接对接，无需换算），恒定加速度斜坡；
   角度：标幺值 theta_pu(0~1) 循环积分（0~1 对应 0~360° 电角度），输出时 ×2π 转 rad。 */
typedef struct {
    float theta_pu;         /* 开环电角度标幺值(0~1)，对应 0~360° */
    float theta_map;        // 开环角度映射
    float omega;            /* 当前开环电角速度(电气 rpm) */
    float omega_start;      /* 起始电角速度(电气 rpm) */
    float omega_end;        /* 目标电角速度(电气 rpm)：达到后匀速 */
    float accel;            /* 角加速度(电气 rpm/s) */
    float Ts;
} EAngle_t;

typedef struct
{
    GeneralMode_t LastGeneralMode;
    GeneralMode_t GeneralMode;
    MOTION_STATE MotionState;
    uint16_t CloseRunTime;
    float OpenCurr;
    float OpenCurrLast;
    float OpenCurrMax;
    int16_t CheckCnt;
    float ThetaRef;
    float ThetaObs;
    float ThetaErr;
    float OpenSpeed;
    float CloseSpeed;
    float EleOpenSpeedAbs;
    float EleCloseSpeedAbs;
    float SpeedErr;
    float SpeedErrFlt;
    uint8_t ErrFlag;
    uint16_t ErrCnt;
    uint16_t ErrTimes;
    float LastTargetSpeed;
    float OpenToCloseSwitchSpeed;
    float CloseToOpenSwitchSpeed;
    float CloseMinSpeed;
    float CurrChangeRate;
    float ObsMag;
    uint8_t AlignFlag;      /* 预定位完成标志：0=预定位进行中，1=完成 */
    uint16_t AlignCnt;      /* 预定位计时(控制周期数) */
}ModeChange_t;

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
    float pwm_period;                               // PWM运行时间

    State_t State;
    Current_t Current;
    PID_t PID_Speed;
    PID_t PID_Iq;
    PID_t PID_Id;
    FOC_t FOC;
    SMO_t SMO;
    PLL_t PLL;
    TAccDec_t TAccDec;
    EAngle_t EAngle;
    ModeChange_t ModeChange;                        // 电机状态切换
}Motor_t;
/*************************************************************************************************************/
/************************************************函数接口******************************************************/
/*************************************************************************************************************/
typedef struct
{
    void (*MotorDriverEnable)(void);
    void (*MotorDriverDisable)(void);
    void (*LedControl)(MOTION_STATE state);
    void (*SetPWMValue)(uint32_t PWMValue_A, uint32_t PWMValue_B, uint32_t PWMValue_C);
}API_Driver_t;

typedef struct
{
    void (*Clark)(FOC_t *FOC);
    void (*Park)(FOC_t *FOC);
    void (*PID)(PID_t *PID);
    void (*AntiPark)(FOC_t *FOC);
    void (*SVPWM)(FOC_t *FOC);
}API_FOC_t;

typedef struct
{
    void (*ObserverSMO)(SMO_t *SMO);
    void (*PLL)(PLL_t *PLL);
    void (*EAngle_Update)(EAngle_t *EAngle);
}API_Observer_t;

typedef struct
{
    void (*GetOffsetCurrent)(Current_t *Current);
    float (*GetPhaseCurrent)(Current_t *Current, uint8_t phase_flag);
    float (*GetVoltageBus)(Current_t *Current);
    void (*GetTempture)(Temperature_t *temp);
    float (*GetPwmPeriod)(void);
}API_Sample_t;

typedef struct
{
    void (*TAccDec_Update)(TAccDec_t *TAccDec);
}API_TAccDec_t;

typedef struct
{
    void (*StrongDragCurrentOpenLoop)(Motor_t *motor);
    void (*StrongDragCurrentCloseLoop)(Motor_t *motor);
    void (*StrongDragSmoSpeedCurrentLoop)(Motor_t *motor);
}API_Switch_t;

typedef struct
{
    API_Driver_t *Driver;
    API_FOC_t *FOC;
    API_Observer_t *Observer;
    API_Sample_t *Sample;
    API_TAccDec_t *TAccDec;
    API_Switch_t *Switch;
}g_MotorInterface_t;

extern g_MotorInterface_t g_API_Interface;

Temperature_t *GetTempStruct(void);
Motor_t *GetMotorStruct(void);

#endif
