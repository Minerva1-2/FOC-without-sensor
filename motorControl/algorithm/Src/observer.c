#include "observer.h"

#if (defined (FOC_WITHOUT_SENSOR))
    void ObserverSMO(Motor_t *motor)
    {
        // switch function
        float sgn_a = (motor->SMO.I_alpha_hat - motor->FOC.I_alpha) > 0.0f ? 1.0f : 0.0f;
        float sgn_b = (motor->SMO.I_beta_hat - motor->FOC.I_beta) > 0.0f ? 1.0f : 0.0f;
        // current observer
        motor->SMO.I_alpha_hat += motor->PWM.pwm_period
                                * ((-MOTOR_PHASE_R / MOTOR_PHASE_L) * motor->SMO.I_alpha_hat
                                + (1.0f / MOTOR_PHASE_L) * motor->FOC.V_alpha
                                - (1.0f / MOTOR_PHASE_L) * motor->SMO.K_slide * sgn_a);
        motor->SMO.I_beta_hat += motor->PWM.pwm_period
                                * ((-MOTOR_PHASE_R / MOTOR_PHASE_L) * motor->SMO.I_beta_hat
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
    void PLL(Motor_t *motor)
    {

    }
#elif (defined (FOC_WITH_SENSOR))
// observer method
#endif