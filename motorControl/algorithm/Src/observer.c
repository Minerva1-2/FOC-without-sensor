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
static void ObserverSMO(Motor_t *motor)
{
    // 电流误差
    float Ts = motor->pwm_period;
    float R_over_L = MOTOR_PHASE_R / MOTOR_PHASE_L;
    float inv_L = 1.0f / MOTOR_PHASE_L;
    // current observer
    motor->SMO.I_alpha_hat += Ts * (-R_over_L * motor->SMO.I_alpha_hat + (motor->FOC.V_alpha - motor->SMO.e_alpha_raw) * inv_L);
    motor->SMO.I_beta_hat += Ts * (-R_over_L * motor->SMO.I_beta_hat + (motor->FOC.V_beta - motor->SMO.e_beta_raw) * inv_L);
    // 滑膜切换函数
    float err_a = motor->SMO.I_alpha_hat - motor->FOC.I_alpha;
    float err_b = motor->SMO.I_beta_hat - motor->FOC.I_beta;
    float sgn_a = (err_a > 1.0f) ? motor->SMO.K_slide : (err_a < -1.0f) ? -motor->SMO.K_slide
                                                                   : motor->SMO.K_slide * err_a;
    float sgn_b = (err_b > 1.0f) ? motor->SMO.K_slide : (err_b < -1.0f) ? -motor->SMO.K_slide
                                                                   : motor->SMO.K_slide * err_b;
    // 反电动势
    motor->SMO.e_alpha_raw = sgn_a;
    motor->SMO.e_beta_raw = sgn_b;

    motor->SMO.e_alpha_hat += SMO_LPF_ALPHA * (sgn_a - motor->SMO.e_alpha_hat);
    motor->SMO.e_beta_hat += SMO_LPF_ALPHA * (sgn_b - motor->SMO.e_beta_hat);
}
/**
 * @brief   get esimation angle
 * @param   pll: PLL 状态；smo: 反电动势输入；foc: 输出角度写入 motor->FOC.angle；pwm_period: 控制周期(s)
 * @return  null
 */
static void PLL(Motor_t *motor)
{
    float e_alpha = motor->SMO.e_alpha_hat;
    float e_beta = motor->SMO.e_beta_hat;
    float E = sqrtf(motor->SMO.e_alpha_hat * motor->SMO.e_alpha_hat + motor->SMO.e_beta_hat * motor->SMO.e_beta_hat);

    if (E > PLL_E_MIN_SQ)
    {
        // get the theta error when the angle more than 5 angle
        motor->PLL.theta_error = (-e_alpha * cosf(motor->PLL.theta_hat) - e_beta * sinf(motor->PLL.theta_hat)) / E;

        motor->PLL.integral += motor->PLL.i * motor->PLL.theta_error * motor->pwm_period;
        motor->PLL.integral = _constrain(motor->PLL.integral, OMEGA_MAX, -OMEGA_MAX);
        motor->PLL.omerga_hat = motor->PLL.p * motor->PLL.theta_error +motor->PLL.integral;
        motor->PLL.omerga_hat = _constrain(motor->PLL.omerga_hat, OMEGA_MAX, -OMEGA_MAX);
    }
    else
    {
        // 观测器不可用
        motor->PLL.theta_error = 0.0f;
    }
    // get estimation angle
    motor->PLL.theta_hat += motor->PLL.omerga_hat * motor->pwm_period;
    // Angle normalization
    if (motor->PLL.theta_hat > PI)
        motor->PLL.theta_hat -= TWO_PI;
    if (motor->PLL.theta_hat < -PI)
        motor->PLL.theta_hat += TWO_PI;
    // output estimation angle
    motor->FOC.angle = motor->PLL.theta_hat;
}
/**
 * @brief   电角度发生器：开环角速度斜坡 + 角度积分
 * @param   Motor_t *motor
 * @return  null
 * @note    角速度口径：电气 rpm（omega_end 可直接接 TAccDec.SpeedOut）；
 *          角度为标幺值 theta_pu(0~1)，输出时 ×2π 转 rad 给 FOC。
 */
static void EAngle_Update(Motor_t *motor)
{
    /* 1) 角速度梯形斜坡（电气 rpm 口径，每周期步长 = accel × Ts） */
    float step = motor->EAngle.accel * motor->pwm_period;
    float err = motor->EAngle.omega_end - motor->EAngle.omega;

    if (err > step)
         motor->EAngle.omega += step;
    else if (err < -step)
         motor->EAngle.omega -= step;
    else
        motor->EAngle.omega = motor->EAngle.omega_end;

    /* 2) 标幺电角度积分（参照盛浩）：theta_pu += Ts × omega(rpm) × (1/60)
          rpm/60 = 转/秒，×Ts = 每周期转数，小数部分累积即 0~1 标幺电角度 */
    motor->EAngle.theta_pu += motor->pwm_period * motor->EAngle.omega * 0.0166666f;

    /* 3) 归一化到 [0, 1)：角度循环累加 */
    if (motor->EAngle.theta_pu >= 1.0f)
         motor->EAngle.theta_pu -= 1.0f;
    else if (motor->EAngle.theta_pu < 0.0f)
         motor->EAngle.theta_pu += 1.0f;
    // 将角度映射至-π-π
    motor->FOC.angle = PI * (2 * motor->EAngle.theta_pu - 1.0f);
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