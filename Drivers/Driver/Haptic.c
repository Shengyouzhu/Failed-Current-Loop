#include "haptic.h"

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
						 fabsf(speed) > fabsf(target_speed) + 0.3f) &&
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
