#include <string.h>
#include <math.h>
#include "statueSwitch.h"
#include "globalControl.h"

/**
 * @brief   电机运行状态改变函数
 * @param   ModeChange_t *ModeChange
 * @return  null
 * @note    该函数通过对电机运行状态的判断以及电机运行速度进行状态改变
 */
static void StrongDragToObserverCal(ModeChange_t *ModeChange)
{
    // 获取角度误差，将误差钳位到-Π到Π之间
    ModeChange->ThetaErr = ModeChange->ThetaRef - ModeChange->ThetaObs;
    
    ModeChange->ThetaErr = ModeChange->ThetaErr > PI ? ModeChange->ThetaErr - TWO_PI : ModeChange->ThetaErr;
    ModeChange->ThetaErr = ModeChange->ThetaErr < -PI ? ModeChange->ThetaErr + TWO_PI : ModeChange->ThetaErr;
    // 获取速度误差，进行一阶低通滤波
    ModeChange->SpeedErr = ModeChange->OpenSpeed - ModeChange->CloseSpeed;
    ModeChange->SpeedErr += SPEED_LPF_ALPHA * (ModeChange->SpeedErrFlt - ModeChange->SpeedErr);
    // 将开环速度以及闭环速度取绝对值，用于比较大小
    ModeChange->EleOpenSpeedAbs = fabsf(ModeChange->OpenSpeed);
    ModeChange->EleCloseSpeedAbs = fabsf(ModeChange->CloseSpeed);
    // 用于计算角度收敛的计时器
    ModeChange->CheckCnt = fabsf(ModeChange->ThetaErr) <= (PI / 10.0f) ? ModeChange->CheckCnt + 1 : ModeChange->CheckCnt - 1;
    // 闭环模式下，闭环计数器增加(限制为0—40000)
    if (ModeChange->MotionState == CLOSE_LOOP)
    {
        ModeChange->CloseRunTime++;
        ModeChange->CloseRunTime = _constrain(ModeChange->CloseRunTime, 0, 40000);
    }
    // 电机需要改变的电流步长（最大开环电流的0.03%）
    float curr_change_unit = ModeChange->OpenCurrMax * 0.0003f;
     /**
     ***************************** 一以下条件判断用于电机状态之间的转换，动态调整开环强拖电流的大小**************************************
      * 情况一：电机没有减速 && 开环速度绝对值 > 开环至闭环切换的速度 && 角度未完全收敛
      * 此时电机已经有足够的速度，所以逐渐减小电流准备切换至闭环运行
      */
    if ((ModeChange->MotionState != TACC_DECELERATE)
        && (ModeChange->EleOpenSpeedAbs >= ModeChange->OpenToCloseSwitchSpeed)
        && (ModeChange->CheckCnt <= 30))
    {
        ModeChange->OpenCurr = fabsf(ModeChange->OpenCurr) - curr_change_unit;
        ModeChange->OpenCurr = _constrain(ModeChange->OpenCurr, ModeChange->OpenCurrMax, 0.08f);
    }
    /**
     * 情况二：电机没有减速 && 开环速度 < 开环至闭环切换的速度
     * 此时电机没有足够的速度，因此通过增加电流的方式增加电机的速度
     */
    else if ((ModeChange->MotionState != TACC_DECELERATE)
            && (ModeChange->EleOpenSpeedAbs < ModeChange->OpenToCloseSwitchSpeed))
    {
        ModeChange->OpenCurr = fabsf(ModeChange->OpenCurr) + 0.01f;
        ModeChange->OpenCurr = _constrain(ModeChange->OpenCurr, ModeChange->OpenCurrMax, 0.05f);
    }
    /**
     * 情况三：电机减速 && 开环速度处于[闭环最低速度, 开环至闭环切换的速度 + 5]区间
     * 此时电机需要加速，因此通过略微增加电流的方式防止电机失步
     */
    else if ((ModeChange->MotionState == TACC_DECELERATE)
            && ((ModeChange->CloseMinSpeed < ModeChange->EleOpenSpeedAbs)
            && (ModeChange->OpenToCloseSwitchSpeed + 5 > ModeChange->EleOpenSpeedAbs)))
    {
        ModeChange->OpenCurr = fabsf(ModeChange->OpenCurr) + 0.001f;
        ModeChange->OpenCurr = _constrain(ModeChange->OpenCurr, ModeChange->OpenCurrMax, 0.05f);
    }
    /**
     * 情况四：电机减速 && 开环速度 < 闭环最低运行速度
     * 此时电机无法满足滑膜观测器运行速度，因此减小电流避免低速电流过大电机出现抖动
     */
    else if ((ModeChange->MotionState == TACC_DECELERATE)
            && (ModeChange->EleOpenSpeedAbs < ModeChange->CloseMinSpeed))
    {
        ModeChange->OpenCurr = fabsf(ModeChange->OpenCurr) - curr_change_unit;
        ModeChange->OpenCurr = _constrain(ModeChange->OpenCurr, ModeChange->OpenCurrMax, 0.08f);
    }
    /**
     * 以下判断切换条件以及错误检测
     * 情况一：电机开环运行 && 闭环速度 < 最低变换运行速度 && 闭环运行周期 > 20000
     * 此时电机在闭环速度下速度过低，观测器可能失效或者电机失步，触发错误标志
     */
    if ((ModeChange->MotionState == OPEN_LOOP)
        && (ModeChange->EleCloseSpeedAbs < ModeChange->CloseMinSpeed)
        && (ModeChange->CloseRunTime > 20000))
    {
        ModeChange->ErrFlag = 1;
        ModeChange->ErrCnt++;
        ModeChange->CloseRunTime = 0;
    }
    /**
     * 情况二：电机开环运行 && 开环速度 > 开环至闭环切换速度
     * 此时电机正常运行，速度达到要求，切换至闭环
     */
    else if ((ModeChange->GeneralMode == OPEN_LOOP)
        && (ModeChange->EleOpenSpeedAbs > ModeChange->OpenToCloseSwitchSpeed))
    {
        ModeChange->ErrFlag = 0;
        ModeChange->GeneralMode = CLOSE_LOOP;
    }
    /**
     * 情况三：开环速度 < 闭环至开环切换速度
     * 此时电机在速度不满足闭环运行条件，观测器可能失效，强制切换为开环运行
     */
    else if (ModeChange->EleOpenSpeedAbs < ModeChange->CloseToOpenSwitchSpeed)
    {
        ModeChange->CloseRunTime = 0;
        ModeChange->GeneralMode = OPEN_LOOP;
    }
    // 限制角度收敛计数器
    ModeChange->CheckCnt = _constrain(ModeChange->CheckCnt, 41, 0);
}
/**
 * @brief   闭环运行
 * @param   Motor_t *motor
 * @return  null
 */
static void StrongDragSmoSpeedCurrentLoop(Motor_t *motor)
{
    /* 预定位：固定电角度 + Id 励磁，转子吸到已知位置 */
    if (motor->ModeChange.AlignFlag == 0)
    {
        motor->FOC.angle = ALIGN_ANGLE;   /* 固定电角度，转子被吸向 d 轴 */

        /* Id 电流闭环：励磁电流，Iq=0（只建立磁场、不输出转矩） */
        motor->FOC.Ia = g_API_Interface.Sample->GetPhaseCurrent(&motor->Current, CURRENT_FLAG_Ia);
        motor->FOC.Ic = g_API_Interface.Sample->GetPhaseCurrent(&motor->Current, CURRENT_FLAG_Ic);

        motor->FOC.Ib = -motor->FOC.Ia - motor->FOC.Ic;
        g_API_Interface.FOC->Clark(&motor->FOC);
        g_API_Interface.FOC->Park(&motor->FOC);

        motor->PID_Id.aimValue = ALIGN_ID_REF;
        motor->PID_Iq.aimValue = 0.0f;
        motor->PID_Id.nowValue = motor->FOC.Id;
        motor->PID_Iq.nowValue = motor->FOC.Iq;

        g_API_Interface.FOC->PID(&motor->PID_Id);
        g_API_Interface.FOC->PID(&motor->PID_Iq);

        motor->FOC.Vd = motor->PID_Id.Output;
        motor->FOC.Vq = motor->PID_Iq.Output;
        g_API_Interface.FOC->AntiPark(&motor->FOC);

        motor->FOC.Voltage_bus = motor->Current.voltage_bus;
        g_API_Interface.FOC->SVPWM(&motor->FOC);
        /* 计时完成 → 结束预定位，开环角度从预定位角度衔接（theta_map=PI*(2*theta_pu-1)，反解 theta_pu） */
        if (++motor->ModeChange.AlignCnt >= (uint16_t)(ALIGN_TIME_MS * 0.001f / motor->pwm_period))
        {
            motor->ModeChange.AlignFlag = 1;
            motor->ModeChange.AlignCnt  = 0;
            motor->EAngle.theta_pu = (ALIGN_ANGLE / PI + 1.0f) * 0.5f;   /* θ=0 → theta_pu=0.5 → theta_map=0 */
            motor->EAngle.omega    = motor->EAngle.omega_start;          /* 角速度从起始值爬升 */
        }
        return;   /* 预定位期间不进入强拖/SMO */
    }

    // 滑膜增益为母线电压的1 / √3
    motor->SMO.K_slide = motor->Current.voltage_bus * ONE_DIV_SQRT3;
    // 设置梯形曲线速度为目标速度
    motor->TAccDec.TargetSpeed = motor->PID_Speed.aimValue;
    g_API_Interface.TAccDec->TAccDec_Update(&motor->TAccDec);
    // 将梯形曲线输出速度作为电角度发生器的目标角速度
    motor->EAngle.omega_end = motor->TAccDec.SpeedOut;
    g_API_Interface.Observer->EAngle_Update(&motor->EAngle);
    // 获取变换后的电流、电压，交给滑膜观测器
    motor->SMO.I_alpha = motor->FOC.I_alpha;
    motor->SMO.I_beta = motor->FOC.I_beta;
    motor->SMO.V_alpha = motor->FOC.V_alpha;
    motor->SMO.V_beta = motor->FOC.V_beta;
    g_API_Interface.Observer->ObserverSMO(&motor->SMO);
    // 将滑膜观测器的预估反电势赋值给鉴相器进行计算
    motor->PLL.e_alpha_hat = motor->SMO.e_alpha_hat;
    motor->PLL.e_beta_hat = motor->SMO.e_beta_hat;
    g_API_Interface.Observer->PLL(&motor->PLL);
    // 获取梯形速度曲线输出速度、机械速度(作为闭环速度)、电角度发生器映射角度、一阶滤波角度、运行状态
    motor->ModeChange.OpenSpeed = motor->TAccDec.SpeedOut;
    motor->ModeChange.CloseSpeed = motor->PLL.omerga_hat_lpf / TWO_PI * 60.0f;
    motor->ModeChange.ThetaRef = motor->EAngle.theta_map;
    motor->ModeChange.ThetaObs = motor->PLL.theta_hat_lpf;
    motor->ModeChange.MotionState = motor->TAccDec.State;
    // 电机运行状态切换
    StrongDragToObserverCal(&motor->ModeChange);
    // 错误检测与开、闭环模式切换
    if (motor->ModeChange.ErrFlag != 0 )
    {
        if (motor->ModeChange.ErrTimes == 0)
        {
            motor->ModeChange.LastTargetSpeed = motor->TAccDec.TargetSpeed;
            motor->TAccDec.TargetSpeed = 0;
            motor->TAccDec.SpeedOut = 0;
            motor->TAccDec.SpeedTargetIncrement = 0;
            motor->TAccDec.SpeedChangeIncrement = 0;

            motor->ModeChange.GeneralMode = OPEN_LOOP; 
        }
        motor->ModeChange.ErrTimes++;
        // 错误周期大于5000，进行恢复
        if (motor->ModeChange.ErrTimes > 5000)
        {
            motor->ModeChange.ErrTimes = 0;
            motor->ModeChange.ErrFlag = 0;
            motor->TAccDec.TargetSpeed = motor->ModeChange.LastTargetSpeed;
        }
    }
    else
    {
        if (motor->ModeChange.MotionState == CLOSE_LOOP)
        {
            motor->EAngle.theta_map = motor->PLL.theta_hat_lpf;
        }
    }
    // PID速度环(2分频)
    motor->PID_Speed.SpeedCalculateCnt++;
    if (motor->PID_Speed.SpeedCalculateCnt >= 2)
    {
        motor->PID_Speed.aimValue = motor->TAccDec.SpeedOut;
        motor->PID_Speed.nowValue = motor->PLL.omerga_hat_lpf / MOTOR_POLE_PAIRS * 60.0f;

        if (motor->ModeChange.GeneralMode == CLOSE_LOOP)
        {
            g_API_Interface.FOC->PID(&motor->PID_Speed);

            if (motor->ModeChange.LastGeneralMode == OPEN_LOOP)
            {
                motor->PID_Speed.integral = motor->ModeChange.OpenCurr;
                motor->PID_Speed.aimValue = motor->ModeChange.OpenCurr;
            }
        }
    }
    // 记录电机运行状态
    motor->ModeChange.LastGeneralMode = motor->ModeChange.GeneralMode;

    motor->FOC.Ia = motor->Current.current_phase_Ia;
    motor->FOC.Ic = motor->Current.current_phase_Ic;
    motor->FOC.Ib = -motor->FOC.Ia - motor->FOC.Ic;
    // clark变换
    g_API_Interface.FOC->Clark(&motor->FOC);
    // 开环模式下，角度由电角度发生器提供，闭环模式下由鉴相器提供
    if (motor->ModeChange.GeneralMode == OPEN_LOOP)
    {
        motor->FOC.angle = motor->EAngle.theta_map;
    }
    else if (motor->ModeChange.GeneralMode == CLOSE_LOOP)
    {
        motor->FOC.angle = motor->PLL.theta_hat_lpf;
    }
    // park变换
    g_API_Interface.FOC->Park(&motor->FOC);
    // 电流环(目标数值进行一阶低通滤波)
    motor->PID_Id.aimValue = SPEED_LPF_ALPHA * motor->FOC.Id + (1 - SPEED_LPF_ALPHA) * motor->FOC.Id;
    motor->PID_Iq.aimValue = SPEED_LPF_ALPHA * motor->FOC.Iq + (1 - SPEED_LPF_ALPHA) * motor->FOC.Iq;
    // 模式转换
    if (motor->ModeChange.GeneralMode == OPEN_LOOP)
    {
        // 目标速度过小：清除所有数据
        if (motor->PID_Speed.aimValue < 10.0f)
        {
            motor->PID_Speed.aimValue = 0.0f;
            motor->FOC.Id = 0.0f;
            motor->FOC.Vq = 0.0f;
            motor->SMO.e_alpha_raw = 0.0f;
            motor->SMO.e_beta_raw = 0.0f;
            motor->PLL.omerga_hat_lpf = 0.0f;
        }
        else
        {
            // 正常运行，将开环电流值作为电流环目标电流，固定强拖电流
            motor->PID_Id.aimValue = 0.0f;
            motor->PID_Iq.aimValue = motor->ModeChange.OpenCurr;
        }
    }
    else
    {
        // 速度环接管
        motor->PID_Id.aimValue = 0.0f;
        motor->PID_Iq.aimValue = motor->PID_Speed.Output;
    }
    // 电流环
    motor->PID_Id.nowValue = motor->FOC.Id_ref;
    motor->PID_Iq.nowValue = motor->FOC.Iq_ref;
    g_API_Interface.FOC->PID(&motor->PID_Id);
    g_API_Interface.FOC->PID(&motor->PID_Iq);

    motor->FOC.Vd = motor->PID_Id.Output;
    motor->FOC.Vq = motor->PID_Iq.Output;
    // park反变换
    g_API_Interface.FOC->AntiPark(&motor->FOC);
    // SVPWM调制
    motor->FOC.Voltage_bus = motor->Current.voltage_bus;
    g_API_Interface.FOC->SVPWM(&motor->FOC);
}
/**************************************************************************************************************************/
/*******************以下两个函数用于ADC还未准备时进行强拖启动，该项目不使用这两个函数,在此保留调用接口方便使用*********************/
/**************************************************************************************************************************/
/**
 * @brief   电流开环强拖启动
 * @param   Motor_t *motor
 * @return  null
 */
static void StrongDragCurrentOpenLoop(Motor_t *motor)
{
    // 外部波轮电位器指定速度(速度为机械角速度)
    motor->TAccDec.TargetSpeed = motor->PID_Speed.aimValue;
    // 梯形速度曲线生成(传入机械角速度，函数内部转化为电角速度给到速度环)
    g_API_Interface.TAccDec->TAccDec_Update(&motor->TAccDec);
    // 将梯形速度曲线输出设置电角度发生器目标速度
    motor->EAngle.omega_end = motor->TAccDec.SpeedOut;
    g_API_Interface.Observer->EAngle_Update(&motor->EAngle);
    // 反Park变换
    g_API_Interface.FOC->AntiPark(&motor->FOC);
    /**SVPWM实现(此处实现了两种方式SVPWM调制方式，即零序注入以及七段式，默认为零序注入，若想使用七段式调制，
     * 则在F:\Project\motor\foc_sensorless\motorControl\Inc\globalControl.h)修改SVPWM_ZERO_SQUENCE为SVPWM_SECTOR_METHOD即可
     */
    motor->FOC.Voltage_bus = motor->Current.voltage_bus;
    g_API_Interface.FOC->SVPWM(&motor->FOC);
}
/**
 * @brief   电流闭环强拖启动
 * @param   Motor_t *motor
 * @return  null
 */
static void StrongDragCurrentCloseLoop(Motor_t *motor)
{
    motor->TAccDec.TargetSpeed = motor->PID_Speed.aimValue;
    g_API_Interface.TAccDec->TAccDec_Update(&motor->TAccDec);

    motor->EAngle.omega_end = motor->TAccDec.SpeedOut;
    g_API_Interface.Observer->EAngle_Update(&motor->EAngle);
    // 获取相电流
    motor->FOC.Ia = g_API_Interface.Sample->GetPhaseCurrent(&motor->Current, CURRENT_FLAG_Ia);
    motor->FOC.Ic = g_API_Interface.Sample->GetPhaseCurrent(&motor->Current, CURRENT_FLAG_Ic);
    motor->FOC.Ib = -motor->FOC.Ia - motor->FOC.Ic;

    g_API_Interface.FOC->Clark(&motor->FOC);
    g_API_Interface.FOC->Park(&motor->FOC);
    // 对D、Q轴电流进行一阶低通滤波
    motor->FOC.Id_ref = motor->FOC.Id * SPEED_LPF_ALPHA + motor->FOC.Id_ref * (1 - SPEED_LPF_ALPHA);
    motor->FOC.Iq_ref = motor->FOC.Iq * SPEED_LPF_ALPHA + motor->FOC.Iq_ref * (1 - SPEED_LPF_ALPHA);
    motor->PID_Id.nowValue = motor->FOC.Id_ref;
    motor->PID_Id.nowValue = motor->FOC.Iq_ref;
    // PID控制
    g_API_Interface.FOC->PID(&motor->PID_Id);
    g_API_Interface.FOC->PID(&motor->PID_Iq);
    // 输出PID计算后的电压值
    motor->FOC.Vd = motor->PID_Id.Output;
    motor->FOC.Vq = motor->PID_Iq.Output;
    // 反Park变换
    g_API_Interface.FOC->AntiPark(&motor->FOC);
    // 获取母线电压值
    motor->FOC.Voltage_bus = motor->Current.voltage_bus;
    g_API_Interface.FOC->SVPWM(&motor->FOC);
}
/*************************************************************************************************************************/
/**********************************************************函数注册接口****************************************************/
/*************************************************************************************************************************/
static API_Switch_t SwitchInterface = {
    .StrongDragCurrentOpenLoop = StrongDragCurrentOpenLoop,
    .StrongDragCurrentCloseLoop = StrongDragCurrentCloseLoop,
    .StrongDragSmoSpeedCurrentLoop = StrongDragSmoSpeedCurrentLoop,
};
void Switch_Register(g_MotorInterface_t *iface)
{
    if (iface != NULL)
        iface->Switch = &SwitchInterface;
}