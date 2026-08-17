#include "statueSwitch.h"

static void StrongDragCurrentOpenLoop(Motor_t *motor)
{
    // 外部波轮电位器指定速度(速度为机械角速度)
    motor->TAccDec.TargetSpeed = MOTOR_SEPPD_COEFFICIENT * motor->Current.pot_ratio;
    // 梯形速度曲线生成(传入机械角速度，函数内部转化为电角速度给到速度环)
    g_API_Interface.TAccDec->TAccDec_Update(motor);
    // 将梯形速度曲线输出设置电角度发生器目标速度
    motor->EAngle.omega_end = motor->TAccDec.SpeedOut;
    g_API_Interface.Observer->EAngle_Update(motor);
    // 反Park变换
    g_API_Interface.FOC->AntiPark(motor);
    /**SVPWM实现(此处实现了两种方式SVPWM调制方式，即零序注入以及七段式，默认为零序注入，若想使用七段式调制，
     * 则在F:\Project\motor\foc_sensorless\motorControl\Inc\globalControl.h)修改SVPWM_ZERO_SQUENCE为SVPWM_SECTOR_METHOD即可
     */ 
    motor->FOC.Voltage_bus = motor->Current.voltage_bus;
    g_API_Interface.FOC->SVPWM(motor);
}
static void StrongDragCurrentCloseLoop(Motor_t *motor)
{
    
}