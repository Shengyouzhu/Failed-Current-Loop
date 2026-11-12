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

`SpeedLoop.c/.h` contain the speed PI and its integral state; `PositionLoop.c/.h` contain the position P and speed limit. Gains are initialized in `FOC_Init`. Mode changes reset the speed integral and filtered voltage output.

The q/d voltage vector is limited to 1.5 V by `FOC_VOLTAGE_LIMIT` in `Drivers/FOC/foc.h`. The AS5600 read failure path disables the driver. Gains and targets are experimental values; haptic behavior is still being tuned on hardware.

Open `MDK-ARM/STM32project.uvprojx` in Keil MDK 5 to build. CubeMX configuration is `STM32project.ioc`.

## Serial feedback

USART1 (PA9 TX, PA10 RX), 460800 baud, 8N1. Select JustFloat in VOFA. A separate task sends 16 channels every 2 ms using DMA. The channel map and units are documented in `Core/Inc/vofa_telemetry.h`. Observe I0/I1 for target/actual speed, I2 for applied Uq, and I3/I4 for target/actual position. I7 reports the control mode number and I8 reports the run/fault state.

## Version v0.3.0 — detents, continuous volume and centered song control

The current motor task calls `Volume_Control(&feedback)` after each successful FOC update. Use exactly one haptic function per cycle; the other calls are left commented in `Core/Src/freertos.c` for manual selection.

- `Volume_Control`: continuous 0–100% volume, initially 50%, over 600° of mechanical travel (±300° from its first-call position). The interior uses zero q-axis voltage without detent attraction. Beyond either endpoint, position control pulls toward the fixed boundary. The function returns a floating-point percentage.
- `Song_Control`: positive rotation beyond 60° generates one next-track event (`+1`); negative rotation beyond 60° generates one previous-track event (`-1`). Position control always pulls toward the recorded center. Return within ±8° and speed below 0.5 rad/s are required to rearm. Other calls return zero.
- `Snap_fastener`: three position-controlled detents. The current tuned spacing is 90° (`1.0471976f * 1.5f`), with fixed endpoint targets at ±90° relative to the first-call position. Some inline comments still describe the original 60° setting; the code value is the active setting.
- `Roller`: the earlier single-function assisted roller remains available.
- The speed/position controller now low-pass filters its clamped Uq output. This snapshot uses a 4 ms time constant in `SpeedLoop.h`; zero bypasses the filter. Controller reset clears filter history. Position Kp is 25, target-speed limit is ±20 rad/s, speed Kp is 0.1, Ki is zero, and the final dq voltage limit remains 1.5 V.
- VOFA now sends 16 JustFloat channels every 2 ms at 460800 baud. I0–I12 retain their previous meanings; I13 is volume percentage, I14 is the cumulative next-track count, and I15 is the cumulative previous-track count. Counters preserve events between telemetry frames.

All haptic origins are recorded on first use and remain fixed afterward. End stops are virtual restoring forces, not mechanical barriers. These functions produce STM32-side values/events; Linux/Windows volume and media control integration is pending. Hardware feel is still being tuned.

## Version v0.2.0 — voltage-driven knob experiment

This snapshot preserves the current single-function roller implementation after reverting the 300 ms gesture-interval experiment. It is a development checkpoint, not a finished haptic controller.

- `Drivers/Driver/Haptic.c/.h`: `Roller(feedback, dt)` records motion and generates a linearly decreasing speed reference with nominal deceleration 15 rad/s². It computes proportional voltage assistance (`Kp = 0.1`), suppresses requested braking voltage, limits the request to ±1 V, and retains the existing 0.5/0.5 voltage filter. The early return during peak updates is retained as part of this snapshot.
- The motor task starts with a zero speed target and calls `Roller` after each successful FOC update. Roller selects voltage mode; its internal speed reference is not the FOC speed-loop target reported on I0.
- AS5600 position is accumulated in integer counts; commutation uses a bounded single-turn angle. Mechanical speed and acceleration are available in rad/s and rad/s².
- Speed PI and position P controllers are separate modules. This version removes the earlier current-sampling/current-loop experiment; the original source remains at commit `118d21f`.
- VOFA channels I10–I12 report sensor HAL status, latched I2C error bits, and fault stage. The complete I0–I12 map is in `Core/Inc/vofa_telemetry.h`.

Known tuning work remains around low-speed assistance, repeated flick detection, and smooth coasting. This version is archived without changing the current motor-control behavior.
