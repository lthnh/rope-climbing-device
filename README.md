# Modbus motor control

The Delta HMI is the Modbus RTU master. Configure slave **17**, **9600 baud, 8 data bits, even parity, 1 stop bit (8E1)** on USART1 (PA9 TX, PA10 RX). The STM32 uses a 9-bit UART word length because parity occupies one bit.

| Data | Zero-based wire address | Function codes | Meaning |
| --- | --- | --- | --- |
| Motor command (holding register, commonly 40001) | 0 | 03 read, 06/16 write | 0 stop, 1 forward, 2 backward; any other value stops |
| Measured speed (input register, commonly 30001) | 0 | 04 read | Reserved RPM value; always 0 until feedback is implemented |

Use word-value buttons: Forward writes 1, Backward writes 2, Stop writes 0 to the command register. For hold-to-run buttons, configure press to write the direction and release to write 0 to that same register. The HMI driver's address notation may differ; the wire address is 0. Use FC04 for the speed display. Coil writes do not control the motor.

The application `ModbusCommTask` initializes and starts Modbus, reads the HMI command register under the Modbus semaphore, validates it, and sends the latest command through a single-slot FreeRTOS queue. `MotorControlTask` receives commands from that queue and controls the GPIO/PWM outputs. Both tasks poll every 10 RTOS ticks. The queue keeps the latest command rather than accumulating stale commands.

The board starts stopped and checks commands every 10 RTOS ticks (10 ms with the current configuration). Forward drives PB10 high, backward drives it low; PB11 is treated as active-high enable, matching the previous firmware. TIM3 channel 1 (PA6) retains the existing fixed pulse frequency and duty cycle. Confirm these polarities with the motor driver before testing.

A direction change disables the driver and stops PWM for one poll cycle, then re-reads the command before starting in the new direction. This is an electrical pause, not a calibrated deceleration or brake. Verify the driver's timing and mechanical stopping requirements before operating under load.

Commands remain latched until overwritten: disconnecting the HMI does not stop motion. The library's timeout is not a slave communication-loss watchdog. Use the physical stop/interlocks required by the machine; communication-loss stopping is not implemented here.

`EN_Port = NULL` assumes a serial interface that needs no firmware-controlled RS-485 direction pin (for example, an automatic-direction transceiver). Never connect STM32 UART pins directly to RS-485 signals. Configure the appropriate transceiver and DE pin if your hardware needs it.

Speed feedback is deliberately deferred; zero does not confirm that the shaft is stationary. Later, update `motorSpeedRpm[0]` with measured RPM while holding `modbusHandler.ModBusSphrHandle`.

Run the host control check with `python3 tests/test_motor_control.py`. Build firmware with `cmake --build --preset Debug` using the STM32 toolchain. The RTOS heap is 6144 bytes to accommodate the application, Modbus, idle, and timer tasks.
