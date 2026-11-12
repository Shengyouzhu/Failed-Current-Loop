# STM32 FOC haptic knob

This STM32F407VETx project uses an AS5600 angle sensor and DRV8316 three-phase driver. The motor task applies a voltage vector through SVPWM. The current sensing and current PI experiment has been removed from this working tree; the previous version remains available in Git history at commit `118d21f`.

## Modes

The motor task calls `FOC_Start(HAL_GetTick())` once and `FOC_Update(HAL_GetTick())` every 1 ms. `Drivers/FOC/foc.c` owns the AS5600 readings, electrical-angle calculation, mode dispatch, and motor state. No FreeRTOS types or delays are required by FOC.

Startup is a non-blocking state machine: zero output for 100 ms, sensor check, 0.7 V d-axis alignment for 3000 ms, sensor check and zero calibration, then running. Existing sensor filter, PI gains, limits and startup targets are preserved. `FOC_Update` returns 1 only when a running control cycle succeeds. `FOC_Stop()` zeros output, disables PE15, and clears the speed integral. Sensor errors latch state 4 and count once; only an explicit `FOC_Start` restarts alignment.

Call Start/Update/Stop and target setters from the motor task. Other tasks read `FOC_GetFeedback()`, which provides a short interrupt-protected snapshot of references, applied voltages, measured angles/speed, state and failure count. The OLED and VOFA no longer read motor globals from `freertos.c`.

Control selection remains:

- `FOC_SetControlMode(FOC_MODE_VOLTAGE)` and `FOC_SetTargetVoltage(volts)` select fixed q-axis voltage.
- `FOC_SetControlMode(FOC_MODE_SPEED)` and `FOC_SetTargetSpeed(rad_s)` select speed PI control. The startup target is 20 rad/s.
- `FOC_SetControlMode(FOC_MODE_POSITION)` and `FOC_SetTargetPosition(radians)` select position P followed by speed PI. The target is an absolute unwrapped mechanical angle; startup initializes it to one turn beyond the aligned position.

`SpeedLoop.c/.h` contain the speed PI and its integral state; `PositionLoop.c/.h` contain the position P and speed limit. Gains are initialized in `FOC_Init`. Mode changes reset the speed integral.

The q/d voltage vector is limited to 1.5 V by `FOC_VOLTAGE_LIMIT` in `Drivers/FOC/foc.h`. The AS5600 read failure path disables the driver. Gains and targets are experimental values; haptic behavior is still being tuned on hardware.

Open `MDK-ARM/STM32project.uvprojx` in Keil MDK 5 to build. CubeMX configuration is `STM32project.ioc`.

## Serial feedback

USART1 (PA9 TX, PA10 RX), 460800 baud, 8N1. Select JustFloat in VOFA. A separate task sends 13 channels every 2 ms using DMA. The channel map and units are documented in `Core/Inc/vofa_telemetry.h`. Observe I0/I1 for target/actual speed, I2 for applied Uq, and I3/I4 for target/actual position. I7 reports the control mode number and I8 reports the run/fault state.

## Version v0.2.0 — voltage-driven knob experiment

This snapshot preserves the current single-function roller implementation after reverting the 300 ms gesture-interval experiment. It is a development checkpoint, not a finished haptic controller.

- `Drivers/Driver/Haptic.c/.h`: `Roller(feedback, dt)` records motion and generates a linearly decreasing speed reference with nominal deceleration 15 rad/s². It computes proportional voltage assistance (`Kp = 0.1`), suppresses requested braking voltage, limits the request to ±1 V, and retains the existing 0.5/0.5 voltage filter. The early return during peak updates is retained as part of this snapshot.
- The motor task starts with a zero speed target and calls `Roller` after each successful FOC update. Roller selects voltage mode; its internal speed reference is not the FOC speed-loop target reported on I0.
- AS5600 position is accumulated in integer counts; commutation uses a bounded single-turn angle. Mechanical speed and acceleration are available in rad/s and rad/s².
- Speed PI and position P controllers are separate modules. This version removes the earlier current-sampling/current-loop experiment; the original source remains at commit `118d21f`.
- VOFA channels I10–I12 report sensor HAL status, latched I2C error bits, and fault stage. The complete I0–I12 map is in `Core/Inc/vofa_telemetry.h`.

Known tuning work remains around low-speed assistance, repeated flick detection, and smooth coasting. This version is archived without changing the current motor-control behavior.
