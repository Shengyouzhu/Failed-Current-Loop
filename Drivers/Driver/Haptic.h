#ifndef _HAPTIC_H_
#define _HAPTIC_H_
#include "foc.h"

/* 速度单位 rad/s；dt 为实际调用周期，单位 s。 */
void Roller(const FOC_Feedback_t *feedback, float dt);

void Snap_fastener(const FOC_Feedback_t *feedback);
/* 音量返回 0~100；切歌仅在触发的当次返回 +1 / -1，其余返回 0。
 * 与 Roller、Snap_fastener 一样，在电机任务中每周期只调用一种手感。
 * 新函数首次调用记录原点，后续调用保持该原点。
 */
float Volume_Control(const FOC_Feedback_t *feedback);
int8_t Song_Control(const FOC_Feedback_t *feedback);

extern volatile float haptic_volume;
extern volatile uint32_t haptic_next_song_count;
extern volatile uint32_t haptic_previous_song_count;
#endif
