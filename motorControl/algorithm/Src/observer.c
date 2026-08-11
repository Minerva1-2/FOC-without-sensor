#include "observer.h"
/**
 * @fn  void ObserverSMO(Motor_t *motor)
 * @brief   get esimation counter electromotive force
 * @param   Motor_t *motor
 * @return  null
 */
void ObserverSMO(Motor_t *motor)
{
    // switch function
    float sgn_a = (motor->SMO.I_alpha_hat - motor->FOC.I_alpha) > 0.0f ? 1.0f : 0.0f;
    float sgn_b = (motor->SMO.I_beta_hat - motor->FOC.I_beta) > 0.0f ? 1.0f : 0.0f;
    // current observer
    motor->SMO.I_alpha_hat += motor->PWM.pwm_period * ((-MOTOR_PHASE_R / MOTOR_PHASE_L) * motor->SMO.I_alpha_hat
                            + (1.0f / MOTOR_PHASE_L) * motor->FOC.V_alpha
                            - (1.0f / MOTOR_PHASE_L) * motor->SMO.K_slide * sgn_a);
    motor->SMO.I_beta_hat += motor->PWM.pwm_period * ((-MOTOR_PHASE_R / MOTOR_PHASE_L) * motor->SMO.I_beta_hat
                            + (1.0f / MOTOR_PHASE_L) * motor->FOC.V_beta
                            - (1.0f / MOTOR_PHASE_L) * motor->SMO.K_slide * sgn_b);
    // counter electromotive force
    motor->SMO.e_alpha_raw = motor->SMO.K_slide * sgn_a;
    motor->SMO.e_beta_raw = motor->SMO.K_slide * sgn_b;
    // low pass filtering
    motor->SMO.e_alpha_hat += motor->SMO.wc * motor->PWM.pwm_period
                            * (motor->SMO.e_alpha_raw - motor->SMO.e_alpha_hat);
    motor->SMO.e_beta_hat += motor->SMO.wc * motor->PWM.pwm_period
                            * (motor->SMO.e_beta_raw - motor->SMO.e_beta_hat);
}
/**
 * @fn  void PLL(Motor_t *motor)
 * @brief   get esimation angle
 * @param   Motor_t *motor
 * @return  null
 */
void PLL(Motor_t *motor)
{
    // get theta error
    float E = sqrtf(motor->SMO.e_alpha_hat * motor->SMO.e_alpha_hat + motor->SMO.e_beta_hat * motor->SMO.e_beta_hat);

    if (E > 0.001f)
    {
        // get the theta error when the angle more than 5 angle
        motor->PLL.theta_error = (-motor->SMO.e_alpha_hat * cosf(motor->PLL.theta_hat)
                                - motor->SMO.e_beta_hat * sinf(motor->PLL.theta_hat)) / E;
    }
    else
    {
        // sin(theta_e - theta) ~ (theta_e - theta) when the theta less than 5 angle
        motor->PLL.theta_error = 0.0f;
    }
    // integral accumulation
    motor->PLL.integral += motor->PLL.i * motor->PLL.theta_error * motor->PWM.pwm_period;
    motor->PLL.integral = _constrain(motor->PLL.integral, OMEGA_MAX, -OMEGA_MAX);
    // electrial speed
    motor->PLL.omerga_hat = motor->PLL.p * motor->PLL.theta_error + motor->PLL.integral;
    motor->PLL.omerga_hat = _constrain(motor->PLL.omerga_hat, OMEGA_MAX, -OMEGA_MAX);
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