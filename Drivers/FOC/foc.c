/********************************************
	FOC 控制模块
	功能：管理电压输出、速度/位置控制、编码器采样、对齐与故障停机
	输入：外部周期调用 FOC_Update，并通过目标设置接口给定控制目标
	输出：SVPWM 电压输出及 FOC_GetFeedback 反馈快照
	说明：Start/Update/Stop 和目标设置接口由电机任务调用；模块不依赖 FreeRTOS
*******************************************/
#include "foc.h"
#include "SpeedLoop.h"
#include "PositionLoop.h"
#include "AS5600.h"

#define MOTOR_VBUS 12.0f    /* 驱动母线电压（V），用于 SVPWM 占空比换算 */
extern int Sensor_DIR;     /* 编码器方向符号，参与机械角到电角度的换算 */
extern int Motor_PP;       /* 电机极对数，不是磁极总数 */

float electrical_offset = 0.0f;    /* 电角度零偏（rad），由静态对齐标定 */
static float applied_uq = 0.0f;    /* 最近一次限幅后的 q 轴电压（V） */
static float applied_ud = 0.0f;    /* 最近一次限幅后的 d 轴电压（V） */
static float active_speed_reference = 0.0f;    /* 当前生效的机械速度参考（rad/s） */

/* 运行数据由 FOC 模块持有，RTOS、OLED 和串口通过反馈接口读取。 */
static float motor_angle;    /* 多圈累计机械位置（rad），仅用于位置反馈 */
static float motor_speed;    /* 编码器驱动输出的机械角速度（rad/s） */
static float motor_electrical_angle;    /* 由单圈机械角度得到的电角度（rad） */
static FOC_State_t motor_state = FOC_STATE_BOOT;    /* 当前启动、运行或故障状态 */
static uint32_t sensor_fail_count;    /* 累计传感器读取失败次数，重新启动时不清零 */
static uint32_t stage_started_ms;    /* 当前等待/对齐阶段的起始毫秒时间 */
static uint32_t last_sample_ms;    /* 上次运行采样周期的毫秒时间 */
static uint32_t sensor_status;         /* 最近一次读取的 HAL 返回值 */
static uint32_t sensor_error;          /* 故障发生时的 I2C 错误位掩码 */
static uint32_t sensor_fault_stage;    /* 1=首次采样，2=对齐后采样，3=运行采样 */
static uint8_t alignment_applied;    /* 0：等待首次采样；1：已施加对齐电压 */

/********************************************
	逆 Clarke 变换
	功能：将静止坐标系 α、β 轴电压转换为 a、b、c 三相电压
	输入：cu，使用 cu->alpha、cu->beta（V）
	输出：更新 cu->a、cu->b、cu->c（V）；无返回值
*******************************************/
void Inv_Clark_transformer(FOC_Vector_t *cu)
{
    cu->a = cu->alpha;
    cu->b = 0.8660254f * cu->beta - 0.5f * cu->alpha;
    cu->c = -0.8660254f * cu->beta - 0.5f * cu->alpha;
}

/********************************************
	逆 Park 变换
	功能：将旋转坐标系 d、q 轴电压转换为静止坐标系 α、β 轴电压
	输入：cu，使用 cu->d、cu->q（V）；theta_e，电角度（rad）
	输出：更新 cu->alpha、cu->beta（V）；无返回值
*******************************************/
void Inv_Pake_transformer(FOC_Vector_t *cu, float theta_e)
{
    float sine = sinf(theta_e);
    float cosine = cosf(theta_e);
    cu->alpha = cu->d * cosine - cu->q * sine;
    cu->beta = cu->q * cosine + cu->d * sine;
}

/********************************************
	FOC 电压输出
	功能：对 dq 电压矢量限幅，经逆 Park、逆 Clarke 变换后调用 SVPWM
	输入：ud、uq，d/q 轴目标电压（V）；theta_e，电角度（rad）
	输出：记录限幅后电压，并通过 SVPWM 更新 TIM1 的 CCR1、CCR2、CCR3
	说明：限制的是 sqrt(ud²+uq²)，超限时同比缩放，保持电压矢量方向
*******************************************/
void FOC_SetVoltage(float ud, float uq, float theta_e)
{
    /* 1. 限制 dq 电压矢量的总幅值，避免两个轴同时输出时突破限压。 */
    FOC_Vector_t voltage = {0};
    float magnitude = sqrtf(ud * ud + uq * uq);
    if (magnitude > FOC_VOLTAGE_LIMIT)
    {
        float scale = FOC_VOLTAGE_LIMIT / magnitude;
        ud *= scale;
        uq *= scale;
    }
    /* 2. 反馈记录最终采用的电压，而不是限幅前的请求值。 */
    applied_ud = ud;
    applied_uq = uq;
    voltage.d = ud;
    voltage.q = uq;
    /* 3. dq -> αβ -> abc -> SVPWM；CCR 比较值在 SVPWM 模块内更新。 */
    Inv_Pake_transformer(&voltage, theta_e);
    Inv_Clark_transformer(&voltage);
    SVPWM_SetPhaseVoltages(voltage.a, voltage.b, voltage.c, MOTOR_VBUS);
}

/********************************************
	施加对齐电压
	功能：在给定电角度上施加 d 轴电压，用于转子静态对齐
	输入：ud，对齐电压（V）；theta，固定电角度（rad）
	输出：通过 FOC_SetVoltage 输出 ud，q 轴电压固定为 0；无返回值
*******************************************/
void FOC_SetAlignVoltage(float ud, float theta)
{
    FOC_SetVoltage(ud, 0.0f, theta);
}

/********************************************
	标定电角度零偏
	功能：将对齐后的编码器机械角度换算为电角度零偏
	输入：mechanical_angle，对齐完成后的单圈机械角度（rad）
	输出：更新 electrical_offset，范围为 [0, 2π)；无返回值
	说明：当前启动流程在固定电角度 0 上对齐，须等转子稳定后再调用
*******************************************/
void FOC_CalibrateZero(float mechanical_angle)
{
    /* 编码器方向 × 极对数 × 单圈机械角，并归一化到 [0, 2π)。 */
    electrical_offset = fmodf(Sensor_DIR * Motor_PP * mechanical_angle,
                              6.2831853f);
    if (electrical_offset < 0.0f) electrical_offset += 6.2831853f;
}

static SpeedLoop_t speed_loop;    /* 速度 PI 参数和积分状态 */
static PositionLoop_t position_loop;    /* 位置 P 参数和输出速度限制 */
static FOC_ControlMode_t control_mode = FOC_MODE_SPEED;    /* 当前控制模式 */
static float target_voltage = 0.7f;    /* 电压模式 q 轴目标（V） */
static float target_speed = 20.0f;    /* 速度模式目标（rad/s） */
static float target_position = 0.0f;    /* 位置模式绝对目标（累计机械 rad） */

/********************************************
	初始化控制参数
	功能：初始化速度 PI、位置 P，并恢复默认控制模式与目标值
	输入：无
	输出：默认速度模式，速度目标 20 rad/s，电压目标 0.7 V
	说明：仅初始化控制器；上电启动与传感器对齐由 FOC_Start/FOC_Update 完成
*******************************************/
void FOC_Init(void)
{
    /* 参考 HelloWord 惯性模式：Kp=0.10、Ki=0，仅使用比例控制。 */
    SpeedLoop_Init(&speed_loop, 0.10f, 0.0f, FOC_VOLTAGE_LIMIT);
    /* 位置 P：Kp=2.0；输出的目标速度限制为 ±20 rad/s。 */
    PositionLoop_Init(&position_loop, 2.0f, 20.0f);
    control_mode = FOC_MODE_SPEED;
    target_voltage = 0.7f;
    target_speed = 20.0f;
    target_position = 0.0f;
    active_speed_reference = 0.0f;
}

/********************************************
	设置控制模式
	功能：选择电压、速度或位置控制；模式变化时清除速度环积分
	输入：mode，FOC_MODE_VOLTAGE / FOC_MODE_SPEED / FOC_MODE_POSITION
	输出：更新 control_mode；无返回值
	说明：只切换控制模式，不负责使能或启动电机
*******************************************/
void FOC_SetControlMode(FOC_ControlMode_t mode)
{
    if (mode != control_mode) SpeedLoop_Reset(&speed_loop);
    control_mode = mode;
}

/********************************************
	设置电压模式目标
	功能：保存电压模式下的 q 轴目标电压
	输入：voltage，q 轴目标电压（V，可正可负）
	输出：更新 target_voltage；无返回值
	说明：实际输出在后续控制周期中执行，最终经过 FOC_SetVoltage 限幅
*******************************************/
void FOC_SetTargetVoltage(float voltage)
{
    target_voltage = voltage;
}

/********************************************
	设置速度目标
	功能：保存速度模式下的机械角速度目标
	输入：speed_rad_s，机械角速度（rad/s，不是 rpm）
	输出：更新 target_speed；无返回值
*******************************************/
void FOC_SetTargetSpeed(float speed_rad_s)
{
    target_speed = speed_rad_s;
}

/********************************************
	设置位置目标
	功能：保存位置模式下的多圈绝对机械位置
	输入：position_rad，与反馈角度使用同一原点的累计机械角度（rad）
	输出：更新 target_position；无返回值
	说明：这是绝对位置目标，不是单次相对转动量
*******************************************/
void FOC_SetTargetPosition(float position_rad)
{
    target_position = position_rad;
}

/********************************************
	执行一次电机控制
	功能：按模式选择直接电压、速度 PI 或位置 P 加速度 PI 串级控制
	输入：theta_e，单圈电角度（rad）；mechanical_angle，累计机械角度（rad）；
	      mechanical_speed，机械角速度（rad/s）；dt，本次控制周期（s）
	输出：生成 q 轴电压并调用 FOC_SetVoltage；d 轴电压设为 0
	说明：本函数不读取编码器；正常运行由 FOC_Update 采样后调用
*******************************************/
void FOC_MotorControl(float theta_e, float mechanical_angle,
                      float mechanical_speed, float dt)
{
    float q_voltage = 0.0f;
    float speed_reference;

    active_speed_reference = 0.0f;
    switch (control_mode)
    {
        /* 电压模式：直接给定 Uq，不保留速度环积分。 */
        case FOC_MODE_VOLTAGE:
            SpeedLoop_Reset(&speed_loop);
            q_voltage = target_voltage;
            break;

        /* 速度模式：速度误差 -> 速度 PI -> Uq。 */
        case FOC_MODE_SPEED:
            active_speed_reference = target_speed;
            q_voltage = SpeedLoop_Update(&speed_loop, target_speed,
                                         mechanical_speed, dt);
            break;

        /* 位置模式：位置误差 -> 位置 P -> 目标速度 -> 速度 PI -> Uq。 */
        case FOC_MODE_POSITION:
            speed_reference = PositionLoop_Update(&position_loop, target_position,
                                                    mechanical_angle);
            active_speed_reference = speed_reference;
            q_voltage = SpeedLoop_Update(&speed_loop, speed_reference,
                                         mechanical_speed, dt);
            break;

        /* 未识别的模式保持零电压请求，并清除速度积分。 */
        default:
            SpeedLoop_Reset(&speed_loop);
            break;
    }
    FOC_SetVoltage(0.0f, q_voltage, theta_e);
}

/********************************************
	启动 FOC 对齐流程
	功能：关闭驱动并初始化控制器，再以零电压使能驱动，进入对齐状态
	输入：now_ms，当前 uint32_t 毫秒时间，例如 HAL_GetTick()
	输出：状态切换为 FOC_STATE_ALIGNING；无返回值
	说明：非阻塞接口，后续必须周期调用 FOC_Update；会恢复默认控制参数和目标
*******************************************/
void FOC_Start(uint32_t now_ms)
{
    /* 先关闭驱动，防止重启初始化期间保留上一周期的主动输出。 */
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15, GPIO_PIN_RESET);
    FOC_Init();
    motor_speed = 0.0f;
    motor_electrical_angle = 0.0f;
    alignment_applied = 0U;
    sensor_status = HAL_OK;
    sensor_error = HAL_I2C_ERROR_NONE;
    sensor_fault_stage = 0U;
    stage_started_ms = now_ms;
    last_sample_ms = now_ms;
    motor_state = FOC_STATE_ALIGNING;
    FOC_SetVoltage(0.0f, 0.0f, 0.0f);
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15, GPIO_PIN_SET);
}

/********************************************
	停止电机输出
	功能：拉低 PE15 关闭驱动，清零 dq 输出与速度环积分
	输入：无
	输出：状态切换为 FOC_STATE_STOPPED；无返回值
	说明：停止主动驱动，不保证转子立即停止；重新启动需要显式调用 FOC_Start
*******************************************/
void FOC_Stop(void)
{
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15, GPIO_PIN_RESET);
    FOC_SetVoltage(0.0f, 0.0f, motor_electrical_angle);
    SpeedLoop_Reset(&speed_loop);
    active_speed_reference = 0.0f;
    motor_state = FOC_STATE_STOPPED;
}

/********************************************
	处理角度传感器故障
	功能：读取失败时停止驱动，记录失败原因并进入故障状态
	输入：stage，1=首次采样，2=对齐后采样，3=运行采样
	输出：失败次数加 1，状态切换为 FOC_STATE_SENSOR_FAULT；无返回值
	说明：后续 FOC_Update 不再自动读取或启动电机，等待显式重新启动
*******************************************/
static void FOC_SensorFault(uint32_t stage)
{
    /* 在停机前保存本次失败原因，故障状态下不再读取，方便串口查看。 */
    sensor_error = HAL_I2C_GetError(&hi2c2);
    sensor_fault_stage = stage;
    FOC_Stop();
    sensor_fail_count++;
    motor_state = FOC_STATE_SENSOR_FAULT;
}

/********************************************
	FOC 周期更新
	功能：推进启动对齐状态机；正常运行时读取编码器并执行一次电机控制
	输入：now_ms，当前 uint32_t 毫秒时间，与 FOC_Start 使用同一时钟
	输出：返回 1：成功完成运行控制周期；返回 0：等待、对齐、停止、故障或未到采样时刻
	说明：建议每 1 ms 调用；时间单位在内部换算为秒，I2C 读取仍可能等待至超时
*******************************************/
uint8_t FOC_Update(uint32_t now_ms)
{
    uint32_t elapsed_ms;
    float dt;

    if (motor_state == FOC_STATE_ALIGNING)
    {
        /* 无符号时间差允许毫秒计数跨越 uint32_t 回绕点。 */
        elapsed_ms = now_ms - stage_started_ms;
        /* 阶段一：零电压等待 100 ms，确认传感器可读后施加对齐电压。 */
        if (!alignment_applied)
        {
            if (elapsed_ms < 100U) return 0U;
            sensor_status = as5600_read_byte(&motor_angle, &motor_speed, 0.002f);
            if (sensor_status != HAL_OK)
            {
                FOC_SensorFault(1U);
                return 0U;
            }
            FOC_SetAlignVoltage(0.7f, 0.0f);
            stage_started_ms = now_ms;
            alignment_applied = 1U;
            return 0U;
        }

        /* 阶段二：保持 0.7 V 对齐 3000 ms；等待通过多次调用推进。 */
        if (elapsed_ms < 3000U) return 0U;
        /* 沿用原启动代码的 0.5 s 滤波步长，此值不是本阶段实际等待时间。 */
        sensor_status = as5600_read_byte(&motor_angle, &motor_speed, 0.5f);
        if (sensor_status != HAL_OK)
        {
            FOC_SensorFault(2U);
            return 0U;
        }
        /* 换相始终使用有界单圈角度，避免累计位置变大影响电角度精度。 */
        FOC_CalibrateZero(AS5600_GetWrappedAngle());
        motor_electrical_angle = FOC_GetElectricalAngle(AS5600_GetWrappedAngle());
        /* 预置位置目标为当前角度再加一圈；默认仍按速度模式运行。 */
        FOC_SetTargetPosition(motor_angle + 6.2831853f);
        FOC_SetVoltage(0.0f, 0.0f, motor_electrical_angle);
        last_sample_ms = now_ms;
        motor_state = FOC_STATE_RUNNING;
        return 0U;
    }

    /* 阶段三：只有运行状态才采样和控制，停止或故障不会自动重启。 */
    if (motor_state != FOC_STATE_RUNNING) return 0U;
    elapsed_ms = now_ms - last_sample_ms;
    /* 同一毫秒重复调用时跳过，避免 dt=0。 */
    if (elapsed_ms == 0U) return 0U;
    last_sample_ms = now_ms;
    dt = (float)elapsed_ms * 0.001f;
    /* 保留原来的异常周期处理：超过 10 ms 时按 1 ms 计算。 */
    if (dt > 0.01f) dt = 0.001f;

    /* 运行时传感器读取失败立即进入停机路径，不使用本次失效数据。 */
    sensor_status = as5600_read_byte(&motor_angle, &motor_speed, dt);
    if (sensor_status != HAL_OK)
    {
        FOC_SensorFault(3U);
        return 0U;
    }
    motor_electrical_angle = FOC_GetElectricalAngle(AS5600_GetWrappedAngle());
    FOC_MotorControl(motor_electrical_angle, motor_angle, motor_speed, dt);
    return 1U;
}

/********************************************
	读取 FOC 反馈快照
	功能：复制参考值、实际电压、机械位置/速度、电角度、运行状态和失败次数
	输入：feedback，调用方提供的 FOC_Feedback_t 存储地址；NULL 时直接返回
	输出：填充 *feedback；无返回值
	说明：只在复制数据期间短暂关闭中断，完成后恢复原中断状态；可供显示和通信任务调用
*******************************************/
void FOC_GetFeedback(FOC_Feedback_t *feedback)
{
    uint32_t irq_state;

    if (feedback == NULL) return;
    /* 临界区只复制反馈数据，不在关中断期间执行 I2C 读取或 PWM 计算。 */
    irq_state = __get_PRIMASK();
    __disable_irq();
    feedback->speed_reference = active_speed_reference;
    feedback->position_reference = target_position;
    feedback->applied_uq = applied_uq;
    feedback->applied_ud = applied_ud;
    feedback->mode = control_mode;
    feedback->mechanical_angle = motor_angle;
    feedback->mechanical_speed = motor_speed;
    feedback->mechanical_acceleration = AS5600_GetAcceleration();
    feedback->electrical_angle = motor_electrical_angle;
    feedback->state = motor_state;
    feedback->sensor_fail_count = sensor_fail_count;
    feedback->sensor_status = sensor_status;
    feedback->sensor_error = sensor_error;
    feedback->sensor_fault_stage = sensor_fault_stage;
    __set_PRIMASK(irq_state);
}
