#include "haptic.h"

/* 供串口及后续 Linux 通信读取；切歌使用累计次数，避免漏掉单周期事件。 */
volatile float haptic_volume = 50.0f;
volatile uint32_t haptic_next_song_count = 0U;
volatile uint32_t haptic_previous_song_count = 0U;

static float last_uq=0;

/********************************************
    惯性滚轮助力试验
    功能：记录拨动峰值，按 5 rad/s² 生成减速目标，仅输出同向助力
    输入：feedback，机械速度 rad/s、机械加速度 rad/s²；dt，周期 s
    输出：电压模式 Uq，正反方向统一限制为 ±0.5 V
    说明：保留单函数和原有次数计数写法；续拨仍使用简化的速度差判断
*******************************************/
void Roller(const FOC_Feedback_t *feedback, float dt)
{

    static float max_speed = 0.0f;
    static float target_speed = 0.0f;
    static int count = 0;
    static float step = 0.0f;
    static float time = 0.0f;
    static int num = 0;
    float speed = feedback->mechanical_speed;
    float reference;
    float uq = 0.0f;

    /* 手感函数直接计算 P 输出，不让速度环在未触发时主动追零速。 */
    FOC_SetControlMode(FOC_MODE_VOLTAGE);

    if (fabsf(speed) > 1.0f)
    {
        /* 同方向比较绝对值记录峰值；换向时改用新方向的速度。 */
        if (speed * max_speed < 0.0f || fabsf(speed) > fabsf(max_speed))
				{
					max_speed = speed;
					return;
				}	
        /* 正反向加速都可以触发；减速时不重新记录滑行目标。 */
        if ((count == 0 ||
						 fabsf(speed) > fabsf(target_speed) + 0.3f) &&  //0.3目前为较好数值，小之后会阻尼感变大
						speed * feedback->mechanical_acceleration > 0.0f)
				{
						target_speed = speed;
						time = fabsf(target_speed) / 15.0f;
						count = (int)(time / dt);
						step = target_speed / count;
						num = 0;
				}

        if (count > 0 && num < count)
        {
            /* 从保存的初始目标减去累计变化量，不从实际速度重复减速。 */
            reference = target_speed - step * num;
            num++;
            uq = 0.1f * (reference - speed);

            /* 只保留与实际转动同向的助力，反向制动请求直接归零。 */
            if (uq * speed < 0.0f)
                uq = 0.0f;

            uq = fmaxf(-1.0f, fminf(uq, 1.0f));
						uq = 0.5f*last_uq+0.5f*uq;
						last_uq = uq;
						
        }
        /* 未触发或本轮次数用完时，uq 保持零，不再主动制动。 */
    }
    else
    {
        /* 近零速清掉旧记录，避免下一次拨动继续使用上一轮的目标。 */
        max_speed = 0.0f;
        target_speed = 0.0f;
        count = 0;
        step = 0.0f;
        time = 0.0f;
        num = 0;
    }

    FOC_SetTargetVoltage(uq);
}


/********************************************
    三档卡扣旋钮
    功能：实现间隔 60° 的三档吸附及两端虚拟限位
    输入：feedback，FOC 反馈，机械角度单位 rad
    输出：设置位置环目标
    说明：首次调用记录原点；合法行程为原点 ±60°
*******************************************/
void Snap_fastener(const FOC_Feedback_t *feedback)
{
    static uint8_t initialized = 0U;
    static float origin_angle = 0.0f;

    const float step = 1.0471976f*1.5f;  /* 相邻档位间隔：60° */
    float relative_angle;
    float target_angle;

    /* 只记录一次原点，转动过程中不移动限位边界。 */
    if (!initialized)
    {
        origin_angle = feedback->mechanical_angle;
        initialized = 1U;
    }

    relative_angle = feedback->mechanical_angle - origin_angle;

    /* 超过半格，切换到相邻档位。 */
    if (relative_angle > step * 0.5f)
    {
        target_angle = origin_angle + step;  /* 右档：+60° */
    }
    else if (relative_angle < -step * 0.5f)
    {
        target_angle = origin_angle - step;  /* 左档：-60° */
    }
    else
    {
        target_angle = origin_angle;         /* 中档：0° */
    }

    /*
     * 实际角度即使超过 ±60°，目标也不再向外移动。
     * 保留真实位置反馈，让位置环产生朝端点的回拉。
     */
    FOC_SetControlMode(FOC_MODE_POSITION);
    FOC_SetTargetPosition(target_angle);
}

/********************************************
    连续音量旋钮
    功能：行程内自由转动、连续调节音量，两端使用位置环虚拟限位
    输入：feedback，累计机械角度（rad）
    输出：返回当前音量百分比（0~100 的 float），不产生档位吸附
    说明：总行程 600°；首次调用的位置对应 50%，两端为 ±300°
*******************************************/
float Volume_Control(const FOC_Feedback_t *feedback)
{
    static uint8_t initialized = 0U;
    static float minimum_angle = 0.0f;
    const float travel = 10.4719755f;  /* 0~100% 对应的总行程：600° */
    float maximum_angle;
    float angle = feedback->mechanical_angle;

    /* 初始位置对应 50%，向两侧各保留一半行程。 */
    if (!initialized)
    {
        minimum_angle = angle - travel * 0.5f;
        initialized = 1U;
    }

    maximum_angle = minimum_angle + travel;

    if (angle < minimum_angle)
    {
        /* 越过 0% 端点，保持真实角度反馈，由位置环向内回拉。 */
        haptic_volume = 0.0f;
        FOC_SetControlMode(FOC_MODE_POSITION);
        FOC_SetTargetPosition(minimum_angle);
    }
    else if (angle > maximum_angle)
    {
        /* 越过 100% 端点，目标固定在上限，不随越界移动。 */
        haptic_volume = 100.0f;
        FOC_SetControlMode(FOC_MODE_POSITION);
        FOC_SetTargetPosition(maximum_angle);
    }
    else
    {
        /* 行程内不吸附、不追零速；角度直接线性换算为音量。 */
        haptic_volume = (angle - minimum_angle) * 100.0f / travel;
        FOC_SetControlMode(FOC_MODE_VOLTAGE);
        FOC_SetTargetVoltage(0.0f);
    }
    return haptic_volume;
}

/********************************************
    回中切歌旋钮
    功能：正向超过 60° 切下一首，反向超过 60° 切上一首
    输入：feedback，累计机械角度（rad）、机械速度（rad/s）
    输出：本次触发返回 +1 / -1，未触发返回 0；持续设置回中目标
    说明：首次调用记录中心；一次拨动只触发一次
          回到中心 ±8° 且速度小于 0.5 rad/s 后重新解锁
          位置环始终向中心回拉，松手后自动回中，无需阻塞等待
*******************************************/
int8_t Song_Control(const FOC_Feedback_t *feedback)
{
    static uint8_t initialized = 0U;
    static uint8_t ready = 1U;
    static float center_angle = 0.0f;
    const float trigger_angle = 1.0471976f;  /* 切歌阈值：60° */
    const float reset_angle = 0.1396263f;    /* 回中解锁范围：±8° */
    float relative_angle;
    int8_t event = 0;

    if (!initialized)
    {
        center_angle = feedback->mechanical_angle;
        initialized = 1U;
    }
    relative_angle = feedback->mechanical_angle - center_angle;

    /* 回中停稳后才能再切，避免快速经过中心时立即解锁。 */
    if (!ready && fabsf(relative_angle) < reset_angle &&
        fabsf(feedback->mechanical_speed) < 0.5f)
        ready = 1U;

    if (ready)
    {
        if (relative_angle > trigger_angle)
        {
            event = 1;
            haptic_next_song_count++;
            ready = 0U;
        }
        else if (relative_angle < -trigger_angle)
        {
            event = -1;
            haptic_previous_song_count++;
            ready = 0U;
        }
    }

    FOC_SetControlMode(FOC_MODE_POSITION);
    FOC_SetTargetPosition(center_angle);
    return event;
}
