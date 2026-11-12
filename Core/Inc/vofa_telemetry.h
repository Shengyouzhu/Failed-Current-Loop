#pragma once
#include "main.h"

/* USART1 460800 8N1; VOFA JustFloat; 16 channels every 2 ms.
 * I0 effective target speed [rpm]     I1 actual speed [rpm]
 * I2 applied Uq [V]                  I3 target position [turns]
 * I4 actual position [turns]         I5 position error [turns, position mode only]
 * I6 electrical angle [rad]          I7 mode: 1 voltage, 2 speed, 3 position
 * I8 motor state: 0 boot, 1 stopped, 2 alignment, 3 running, 4 fault
 * I9 sensor read failure count
 * I10 last sensor HAL status: 0 OK, 1 ERROR, 2 BUSY, 3 TIMEOUT
 * I11 latched I2C error bitmask: 1 BERR, 2 ARLO, 4 AF/NACK, 32 TIMEOUT
 * I12 failure stage: 0 none, 1 first read, 2 after alignment, 3 running
 * I13 volume [%], initial 50; updated by Volume_Control
 * I14 next-song event count          I15 previous-song event count
 * Event counts persist across telemetry frames; inactive modes do not update them.
 * Position values are absolute, unwrapped mechanical turns.
 */
#define VOFA_TELEMETRY_PERIOD_MS 2U
#define VOFA_TELEMETRY_CHANNELS 16U

HAL_StatusTypeDef VOFA_TelemetryInit(void);
void VOFA_TelemetrySend(void);
extern volatile uint32_t vofa_frames_started;
extern volatile uint32_t vofa_frames_skipped;
extern volatile uint32_t vofa_errors;
