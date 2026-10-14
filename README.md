# Modbus motor control

The Delta HMI is the Modbus RTU master. Configure slave **17**, **9600 baud, 8 data bits, even parity, 1 stop bit (8E1)** on USART1 (PA9 TX, PA10 RX). The STM32 uses a 9-bit UART word length because parity occupies one bit.

| Data | Zero-based wire address | Function codes | Meaning |
| --- | --- | --- | --- |
| Motor enable (coil) | 0 | 01 read, 05/15 write | 0 stop, 1 run |
| Motor direction (coil) | 1 | 01 read, 05/15 write | 0 forward, 1 backward |
| Motion tracking enable (coil) | 2 | 01 read, 05/15 write | 0 pause tracking, 1 track |
| Zero motion estimates (coil) | 3 | 01 read, 05/15 write | Write 1 to zero; firmware clears to 0 on completion |
| Discrete inputs | 100–115 | 02 read | Reserved; initially zero |
| Estimated speed | 200–201 | 04 read | Signed 32-bit pulses/second |
| Estimated position | 202–205 | 04 read | Signed 64-bit pulse position from boot or last zero |
| Holding registers | 300–301 | 03 read, 06/16 write | Separate storage; does not control the motor |

Set direction coil 1, then enable coil 0. Stop by clearing coil 0. FC15 can update both coils together. Coil 2 enables pulse speed/position tracking and does not affect motor control. Both directions count while enabled: forward adds, backward subtracts. Disabling tracking retains position, ignores further travel, and reports zero speed. Re-enabling resumes from that retained position; movement during the pause is lost. Position starts at zero on boot, with no persistence.

Set coil 3 to 1 to zero the pulse position and speed. The tracking task resets its counters and speed sampling baseline, publishes zeros to registers 200–205, then clears only coil 3 under the same Modbus semaphore. Read coil 3 returning to 0 as completion acknowledgement. The request works while tracking is paused or the motor is stopped. While moving, tracking resumes immediately from zero and the next sample can be nonzero; zeroing does not stop the motor. Normally completion takes up to 100 ms; semaphore contention leaves the request pending until a successful acquisition.

`MotorControlTask` copies the coil word while holding the Modbus semaphore, then releases it before controlling GPIO/PWM or delaying. It polls every 10 RTOS ticks. Starting and reversing wait 10 ticks after setting direction, then re-read the enable/direction bits before starting PWM. A changed command or failed semaphore acquisition during this recheck leaves the motor disabled until a later poll. If the initial read fails while running, the current motion continues until a successful read.

The board starts stopped and checks commands every 10 RTOS ticks (10 ms with the current configuration). Forward drives PB10 high, backward drives it low; PB11 is treated as active-high enable, matching the previous firmware. TIM3 channel 1 (PA6) generates 100 Hz pulses with 50% duty cycle. PWM mode 2 starts each cycle low; its compare interrupt matches the rising STEP edge. Every restart resets the timer counter and clears stale compare/pending interrupt state. Confirm these polarities with the motor driver before testing.

A direction change disables the driver and stops PWM for one poll cycle, then re-reads the command before starting in the new direction. This is an electrical pause, not a calibrated deceleration or brake. Verify the driver's timing and mechanical stopping requirements before operating under load.

Commands remain latched until overwritten: disconnecting the HMI does not stop motion. The library's timeout is not a slave communication-loss watchdog. Use the physical stop/interlocks required by the machine; communication-loss stopping is not implemented here.

`EN_Port = NULL` assumes a serial interface that needs no firmware-controlled RS-485 direction pin (for example, an automatic-direction transceiver). Never connect STM32 UART pins directly to RS-485 signals. Configure the appropriate transceiver and DE pin if your hardware needs it.

`MotionTrackingTask` samples every 100 RTOS ticks (100 ms at the current 1 kHz tick rate), calculates signed speed using actual elapsed milliseconds, and publishes all six registers together under the Modbus semaphore. Speed uses integer pulses/second, truncating toward zero, averaged over the sampling interval; stopped motion or disabled tracking reports zero. Net forward/backward pulses within one interval can cancel. A failed semaphore acquisition leaves registers unchanged until a later sample publishes the latest position and rate; pulse accounting continues.

Read addresses 200–205 together with FC4 for a consistent snapshot. Multi-register values use two’s complement, most significant 16-bit word first. For example, speed -100 is `FFFF FF9C`, and position -1 is `FFFF FFFF FFFF FFFF`. Coil 2 changes take effect when the motor task successfully polls it, normally within 10 ms; motor startup also rechecks it after the direction pause. HMI readings normally update within 100 ms after that. Semaphore contention can extend either delay.

TIM3 IRQ priority 5 allows short FreeRTOS critical sections to protect the counter and account for pending edges at stop or tracking changes. The ISR performs only integer counting. Confirm the driver steps on the rising edge of PA6, and compare PA6 pulses against the counter on hardware, including starts, stops, and reversals.

These values estimate commanded travel, not measured motion: missed steps, driver faults, and rope slip are not detected. The 1.8-degree rating means 200 full steps per motor revolution, but driver microstepping determines pulses per revolution. Physical units require driver calibration plus rope-contact circumference/gearing (or measured travel per pulse). With that calibration, position in mm is pulse position × mm/pulse, and speed in mm/s is pulses/s × mm/pulse.

Run the host checks with `python3 tests/test_motor_control.py` and `python3 tests/test_motion_tracking.py`. Build firmware with `cmake --build --preset Debug` using the STM32 toolchain. The RTOS heap is 6144 bytes to accommodate the motor control and motion tracking tasks, default task, Modbus, idle, and timer tasks.
