#include "foc.h"
#include "globalControl.h"
/**
 * @fn  void Clark(Motor_t *motor)
 * @brief   make the motor's three phase currents become the two phase currents
 * @param   Motor_t *motor
 * @return  null
 */
void Clark(Motor_t *motor)
{   
    motor->FOC.I_alpha = motor->FOC.Ia;
    motor->FOC.I_beta = (motor->FOC.Ia + 2.0f * motor->FOC.Ib) / sqrtf(3.0f);
}
/**
 * @fn  void Park(Motor_t *motor)
 * @brief   make the motor's two phase currents become the spin two phase currents
 * @param   Motor_t *motor
 * @return  null
 */
void Park(Motor_t *motor)
{
    float sin_angle = sinf(motor->FOC.angle);
    float cos_angle = cosf(motor->FOC.angle);
    
    motor->FOC.Id = motor->FOC.I_alpha * cos_angle + motor->FOC.I_beta * sin_angle;
    motor->FOC.Iq = -(motor->FOC.I_alpha * sin_angle) + motor->FOC.I_beta * cos_angle;
}
/**
 * @fn  float pid(Motor_t *motor, int32_t nowValue, int32_t aimValue)
 * @brief   incremental PID control
 * @param   Motor_t *motor, int32_t nowValue, int32_t aimValue
 * @return  Output
 */
float Pid(Motor_t *motor, int32_t nowValue, int32_t aimValue)
{
    int32_t iError;     // current error
    float Output;       // speed output

    iError = aimValue - nowValue;

    Output = (motor->PID.p * iError)
            - (motor->PID.i * motor->PID.lastError)
            + (motor->PID.d * motor->PID.prevError);

    motor->PID.prevError = motor->PID.lastError;
    motor->PID.lastError = iError;
    //output limiting
    return _constrain(Output, motor->PID.OutputMax, motor->PID.OutputMin);
}
/**
 * @fn  void AntiPark(Motor_t *motor)
 * @brief   make the motor's two spin phase currents become the static phase currents
 * @param   Motor_t *motor
 * @return  null
 */
void AntiPark(Motor_t *motor)
{
    float sin_el = sinf(motor->FOC.angle);
    float cos_el = cosf(motor->FOC.angle);
    
    motor->FOC.V_alpha = motor->FOC.Vd * cos_el - motor->FOC.Vq * sin_el; 
    motor->FOC.V_beta = motor->FOC.Vd * sin_el + motor->FOC.Vq * cos_el;
}
/**
 * @brief   this code include two SVPWM methods.
 *          defalut method is seven stage SVPWM,if you want to change the method,
 *          please change the SVPWM define in "globalControl.h".
 */
#if (defined (SVPWM_SECTOR_METHOD))
    /**
     * @fn  void SVPWMSectorMethod(Motor_t *motor)
     * @brief   make the motor's two sine phase currents become the three sine phase currents(seven stage SVPWM)
     * @param   Motor_t *motor
     * @return  null
     */
    void SVPWMSectorMethod(Motor_t *motor)
    {
        float T1, T2;
        float Ta, Tb, Tc;

        float V_alpha = motor->FOC.V_alpha;
        float V_beta = motor->FOC.V_beta;
        float V_dc = motor->Current.voltage_bus;
        float Ts = motor->PWM.pwm_period;

        float U1 = V_beta;
        float U2 = SQRT3_DIV_TWO * V_alpha - 0.5f * V_beta;
        float U3 = -SQRT3_DIV_TWO * V_alpha - 0.5f * V_beta;
        // section judge
        uint8_t A = U1 > 0.0f ? 1 : 0;
        uint8_t B = U2 > 0.0f ? 1 : 0;
        uint8_t C = U3 > 0.0f ? 1 : 0;
        uint8_t section = (A << 2) | (B << 1) | C;
        // useful duty time
        float K = ONE_DIV_SQRT3 * Ts / V_dc;
        float X = V_beta * K;
        float Y = (SQRT3_DIV_TWO * V_alpha + 0.5f * V_beta) * K;
        float Z = (-SQRT3_DIV_TWO * V_alpha + 0.5f * V_beta) * K;
        // pwm useful time
        switch (section)
        {
            case 3: T1 = -Z;    T2 = X;     break;
            case 1: T1 = Z;     T2 = Y;     break;
            case 5: T1 = X;     T2 = -Y;    break;
            case 4: T1 = -X;    T2 = Z;     break;
            case 6: T1 = -Y;    T2 = -Z;    break;
            case 2: T1 = Y;     T2 = -X;    break;
            default:T1 = 0.0f;  T2 = 0.0f;  break;
        }
        // overmodulation limiting
        if ((T1 + T2) > Ts)
        {
            float scale = Ts / (T1 + T2);
            T1 *= scale;
            T2 *= scale;
        }
        // zero vector time
        float T0 = (Ts -T1 - T2) * 0.5f;
        // seven sector svpwm
        switch (section)
        {
            /* sction I (U4-U6): 000→100→110→111→110→100→000 */
            case 3:
                Ta = T0 + T1 + T2;
                Tb = T0 + T2;
                Tc = T0;
                break;
            /* sction II (U2-U6): 000→010→110→111→110→010→000 */
            case 1:
                Ta = T0 + T1;
                Tb = T0 + T1 + T2;
                Tc = T0;
                break;
            /* sction III (U2-U3): 000→010→011→111→011→010→000 */
            case 5:
                Ta = T0;
                Tb = T0 + T1 + T2;
                Tc = T0 + T2;
                break;
            /* sction IV (U1-U3): 000→001→011→111→011→001→000 */
            case 4:
                Ta = T0;
                Tb = T0 + T1;
                Tc = T0 + T1 + T2;
                break;
            /* sction V (U1-U5): 000→001→101→111→101→001→000 */
            case 6:
                Ta = T0 + T2;
                Tb = T0;
                Tc = T0 + T1 + T2;
                break;
            /* sction VI (U4-U5): 000→100→101→111→101→100→000 */
            case 2:
                Ta = T0 + T1 + T2;
                Tb = T0;
                Tc = T0 + T1;
                break;
            default:
                Ta = Tb = Tc = T0;
                break;
        }
        // duty output
        motor->PWM.Duty_a = Ta / Ts;
        motor->PWM.Duty_b = Tb / Ts;
        motor->PWM.Duty_c = Tc / Ts;
    }
#else
    /**
     * @fn  void SVPWM_ZeroSequence(Motor_t *motor)
     * @brief   make the motor's two sine phase currents become the three sine phase currents(zero squence SVPWM)
     * @param   Motor_t *motor
     * @return  null
     */
    void SVPWM_ZeroSequence(Motor_t *motor)
    {
        // Anti Clark
        float Va = motor->FOC.V_alpha;
        float Vb = -0.5f * motor->FOC.V_alpha + SQRT3_DIV_TWO * motor->FOC.V_beta;
        float Vc = -0.5f * motor->FOC.V_alpha - SQRT3_DIV_TWO * motor->FOC.V_beta;
        // find max and min vlotage 
        float Vmax = Va;
        if(Vb > Vmax)   Vmax = Vb;
        if(Vc > Vmax)   Vmax = Vc;

        float Vmin = Va;
        if (Vb < Vmin) Vmin = Vb;
        if (Vc < Vmin) Vmin = Vc;
        // Zero sequence injection
        float V_offset = (Vmax + Vmin) * 0.5f;

        Va -= V_offset;
        Vb -= V_offset;
        Vc -= V_offset;
        // Zero axis translation
        float V_peak = (Vmax - Vmin) * 0.5f;
        float V_limit = motor->Current.voltage_bus * ONE_DIV_SQRT3;
        // output limiting
        if (V_peak > V_limit)
        {
            float scale = V_limit / V_peak;

            Va *= scale;
            Vb *= scale;
            Vc *= scale;
        }
        // output duty
        motor->PWM.Duty_a = 0.5f + Va / motor->Current.voltage_bus;
        motor->PWM.Duty_b = 0.5f + Vb / motor->Current.voltage_bus;
        motor->PWM.Duty_c = 0.5f + Vc / motor->Current.voltage_bus;
    }
#endif
