/*
 * @Author: Minerva1-2 18993035310@163.com
 * @Date: 2026-08-12 21:41:41
 * @LastEditors: Minerva1-2 18993035310@163.com
 * @LastEditTime: 2026-08-12 22:39:11
 * @Description: 滑膜观测器 + PLL，与 Motor_t 解耦：只操作传入的子结构体
 */
#include "observer.h"
/**
 * @fn  void ObserverSMO(SMO_t *smo, FOC_t *foc, float pwm_period)
 * @brief   get esimation counter electromotive force
 * @param   smo: 观测器状态；foc: 电压(输入)/电流(反馈)；pwm_period: 控制周期(s)
 * @return  null
 */
void ObserverSMO(Motor_t *motor)
{
    // 电流误差
    float Ts = motor->PWM.pwm_period;
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
 * @fn  void PLL(PLL_t *pll, SMO_t *smo, FOC_t *foc, float pwm_period)
 * @brief   get esimation angle
 * @param   pll: PLL 状态；smo: 反电动势输入；foc: 输出角度写入 motor->FOC.angle；pwm_period: 控制周期(s)
 * @return  null
 */
void PLL(Motor_t *motor)
{
    float e_alpha = motor->SMO.e_alpha_hat;
    float e_beta = motor->SMO.e_beta_hat;
    float E = sqrtf(motor->SMO.e_alpha_hat * motor->SMO.e_alpha_hat + motor->SMO.e_beta_hat * motor->SMO.e_beta_hat);

    if (E > PLL_E_MIN_SQ)
    {
        // get the theta error when the angle more than 5 angle
        motor->PLL.theta_error = (-e_alpha * cosf(motor->PLL.theta_hat) - e_beta * sinf(motor->PLL.theta_hat)) / E;

        motor->PLL.integral += motor->PLL.i * motor->PLL.theta_error * motor->PWM.pwm_period;
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
    motor->PLL.theta_hat += motor->PLL.omerga_hat * motor->PWM.pwm_period;
    // Angle normalization
    if (motor->PLL.theta_hat > PI)
        motor->PLL.theta_hat -= TWO_PI;
    if (motor->PLL.theta_hat < -PI)
        motor->PLL.theta_hat += TWO_PI;
    // output estimation angle
    motor->FOC.angle = motor->PLL.theta_hat;
}
