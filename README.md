# Modbus motor control

The Delta HMI is the Modbus RTU master. Configure slave **17**, **9600 baud, 8 data bits, even parity, 1 stop bit (8E1)** on USART1 (PA9 TX, PA10 RX). The STM32 uses a 9-bit UART word length because parity occupies one bit.

| Data | Zero-based wire address | Function codes | Meaning |
| --- | --- | --- | --- |
| Motor enable (coil) | 0 | 01 read, 05/15 write | 0 stop, 1 run |
| Motor direction (coil) | 1 | 01 read, 05/15 write | 0 forward, 1 backward |
| Discrete inputs | 100–115 | 02 read | Reserved; initially zero |
| Input registers | 200–201 | 04 read | Reserved; initially zero |
| Holding registers | 300–301 | 03 read, 06/16 write | Separate storage; does not control the motor |

Set direction coil 1, then enable coil 0. Stop by clearing coil 0. FC15 can update both coils together. Coil 2 is reserved for future speed monitoring and does not affect motor control.

`MotorControlTask` copies the coil word while holding the Modbus semaphore, then releases it before controlling GPIO/PWM or delaying. It polls every 10 RTOS ticks. Starting and reversing wait 10 ticks after setting direction, then re-read the enable/direction bits before starting PWM. A changed command or failed semaphore acquisition during this recheck leaves the motor disabled until a later poll. If the initial read fails while running, the current motion continues until a successful read.

The board starts stopped and checks commands every 10 RTOS ticks (10 ms with the current configuration). Forward drives PB10 high, backward drives it low; PB11 is treated as active-high enable, matching the previous firmware. TIM3 channel 1 (PA6) retains the existing fixed pulse frequency and duty cycle. Confirm these polarities with the motor driver before testing.

A direction change disables the driver and stops PWM for one poll cycle, then re-reads the command before starting in the new direction. This is an electrical pause, not a calibrated deceleration or brake. Verify the driver's timing and mechanical stopping requirements before operating under load.

Commands remain latched until overwritten: disconnecting the HMI does not stop motion. The library's timeout is not a slave communication-loss watchdog. Use the physical stop/interlocks required by the machine; communication-loss stopping is not implemented here.

`EN_Port = NULL` assumes a serial interface that needs no firmware-controlled RS-485 direction pin (for example, an automatic-direction transceiver). Never connect STM32 UART pins directly to RS-485 signals. Configure the appropriate transceiver and DE pin if your hardware needs it.

Speed feedback is deliberately deferred; zero does not confirm that the shaft is stationary. Later, update `modbusInputRegistersData[0]` with measured RPM while holding `modbusHandle.ModBusSphrHandle`.

Run the host control check with `python3 tests/test_motor_control.py`. Build firmware with `cmake --build --preset Debug` using the STM32 toolchain. The RTOS heap is 6144 bytes to accommodate the application, Modbus, idle, and timer tasks.
