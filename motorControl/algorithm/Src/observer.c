/*
 * @Author: Minerva1-2 18993035310@163.com
 * @Date: 2026-08-12 21:41:41
 * @LastEditors: Minerva1-2 18993035310@163.com
 * @LastEditTime: 2026-08-12 22:39:11
 * @Description: 滑膜观测器 + PLL，与 Motor_t 解耦：只操作传入的子结构体
 */
#include "observer.h"
/**
 * @brief   get esimation counter electromotive force
 * @param   smo: 观测器状态；foc: 电压(输入)/电流(反馈)；pwm_period: 控制周期(s)
 * @return  null
 */
static void ObserverSMO(SMO_t *SMO)
{
    // 电流误差
    float Ts = SMO->Ts;
    float R_over_L = MOTOR_PHASE_R / MOTOR_PHASE_L;
    float inv_L = 1.0f / MOTOR_PHASE_L;
    // current observer
    SMO->I_alpha_hat += Ts * (-R_over_L * SMO->I_alpha_hat + (SMO->V_alpha - SMO->e_alpha_raw) * inv_L);
    SMO->I_beta_hat += Ts * (-R_over_L * SMO->I_beta_hat + (SMO->V_beta - SMO->e_beta_raw) * inv_L);
    // 滑膜切换函数
    float err_a = SMO->I_alpha_hat - SMO->I_alpha;
    float err_b = SMO->I_beta_hat - SMO->I_beta;
    float sgn_a = (err_a > 1.0f) ? SMO->K_slide : (err_a < -1.0f) ? -SMO->K_slide
                                                                  : SMO->K_slide * err_a;
    float sgn_b = (err_b > 1.0f) ? SMO->K_slide : (err_b < -1.0f) ? -SMO->K_slide
                                                                  : SMO->K_slide * err_b;
    // 反电动势
    SMO->e_alpha_raw = sgn_a;
    SMO->e_beta_raw = sgn_b;

    SMO->e_alpha_hat += SMO_LPF_ALPHA * (sgn_a - SMO->e_alpha_hat);
    SMO->e_beta_hat += SMO_LPF_ALPHA * (sgn_b - SMO->e_beta_hat);
}
/**
 * @brief   get esimation angle
 * @param   pll: PLL 状态；smo: 反电动势输入；foc: 输出角度写入 motor->FOC.angle；pwm_period: 控制周期(s)
 * @return  null
 */
static void PLL(PLL_t *PLL)
{
    float e_alpha = PLL->e_alpha_hat;
    float e_beta = PLL->e_beta_hat;
    float E = sqrtf(e_alpha * e_alpha + e_beta * e_beta);
    // get the theta error when the angle more than 5 angle
    PLL->theta_error = (-e_alpha * cosf(PLL->theta_hat) - e_beta * sinf(PLL->theta_hat)) / E;

    PLL->integral += PLL->i * PLL->theta_error * PLL->Ts;
    PLL->integral = _constrain(PLL->integral, OMEGA_MAX, -OMEGA_MAX);
    PLL->omerga_hat = PLL->p * PLL->theta_error + PLL->integral;
    PLL->omerga_hat_lpf += SPEED_LPF_ALPHA * (PLL->omerga_hat - PLL->omerga_hat_lpf);
    PLL->omerga_hat_lpf = _constrain(PLL->omerga_hat_lpf, OMEGA_MAX, -OMEGA_MAX);
    // get estimation angle
    PLL->theta_hat += PLL->omerga_hat_lpf * PLL->Ts;
    PLL->theta_hat_lpf += SPEED_LPF_ALPHA * (PLL->theta_hat - PLL->theta_hat_lpf);
    // Angle normalization
    if (PLL->theta_hat_lpf > PI)
        PLL->theta_hat_lpf -= TWO_PI;
    if (PLL->theta_hat_lpf < -PI)
        PLL->theta_hat_lpf += TWO_PI;
    // output estimation angle
}
/**
 * @brief   电角度发生器：开环角速度斜坡 + 角度积分
 * @param   Motor_t *motor
 * @return  null
 * @note    角速度口径：电气 rpm（omega_end 可直接接 TAccDec.SpeedOut）；
 *          角度为标幺值 theta_pu(0~1)，输出时 ×2π 转 rad 给 FOC。
 */
static void EAngle_Update(EAngle_t *EAngle)
{
    /* 1) 角速度梯形斜坡（电气 rpm 口径，每周期步长 = accel × Ts） */
    float step = EAngle->accel * EAngle->Ts;
    float err = EAngle->omega_end - EAngle->omega;

    if (err > step)
        EAngle->omega += step;
    else if (err < -step)
        EAngle->omega -= step;
    else
        EAngle->omega = EAngle->omega_end;

    /* 2) 标幺电角度积分（参照盛浩）：theta_pu += Ts × omega(rpm) × (1/60)
          rpm/60 = 转/秒，×Ts = 每周期转数，小数部分累积即 0~1 标幺电角度 */
    EAngle->theta_pu += EAngle->Ts * EAngle->omega * 0.0166666f;

    /* 3) 归一化到 [0, 1)：角度循环累加 */
    if (EAngle->theta_pu >= 1.0f)
        EAngle->theta_pu -= 1.0f;
    else if (EAngle->theta_pu < 0.0f)
        EAngle->theta_pu += 1.0f;
    // 将角度映射至-π-π
    EAngle->theta_map = PI * (2 * EAngle->theta_pu - 1.0f);
}
static API_Observer_t ObserverInterface = {
    .ObserverSMO = ObserverSMO,
    .PLL = PLL,
    .EAngle_Update = EAngle_Update,
};

void Observer_Register(g_MotorInterface_t *iface)
{
    if (iface != NULL)
        iface->Observer = &ObserverInterface;
}