#ifndef VOFA_TELEMETRY_H
#define VOFA_TELEMETRY_H

#include "main.h"

/* 19 JustFloat channels, sent every 2 ms on USART1 at 460800 baud.
 *  0  Iq target [A]       1  Iq filtered for PI [A]
 *  2  Id measured [A]     3  Uq applied [V]
 *  4  Ud applied [V]      5  Ia [A]
 *  6  Ib [A]              7  Ic [A]
 *  8  Ia+Ib+Ic [A]        9  electrical angle [rad]
 * 10  mechanical angle [rad, unwrapped]
 * 11  mechanical speed [rpm]
 * 12  ADC injected samples since previous frame (about 40 per 2 ms)
 * 13  motor state: 0 boot, 1 current calibration, 2 alignment,
 *                  3 closed loop, 4 sensor/calibration fault
 * 14  AS5600 read failure count since boot
 * 15  reset reason bitmask: BOR=1, pin=2, power-on=4, software=8,
 *      independent watchdog=16, window watchdog=32, low-power=64
 * 16  ADC samples since the last successful electrical-angle update
 * 17  reserved, always zero (current-sample rejection removed)
 * 18  Iq before d/q low-pass [A] (after five-sample phase average)
 */
#define VOFA_TELEMETRY_PERIOD_MS  2U
#define VOFA_TELEMETRY_CHANNELS   19U

HAL_StatusTypeDef VOFA_TelemetryInit(void);
void VOFA_TelemetrySend(void);

/* Optional Watch variables for diagnosing the telemetry link. */
extern volatile uint32_t vofa_frames_started;
extern volatile uint32_t vofa_frames_skipped;
extern volatile uint32_t vofa_errors;

#endif
