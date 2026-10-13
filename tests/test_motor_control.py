"""Run the real motor task with mocked HAL/RTOS calls: python3 tests/test_motor_control.py."""
from pathlib import Path
import subprocess
import tempfile

source = (Path(__file__).resolve().parents[1] / "Core/Src/main.c").read_text()
start = source.index("void MotorControlTask(void *argument)\n{")
end = source.index("/* USER CODE END 0 */", start)
defines = "\n".join(line for line in source.splitlines() if line.startswith("#define MOTOR_"))
harness = r'''
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#define GPIOB 0
#define CW_PIN 10
#define EN_PIN 11
#define GPIO_PIN_SET 1
#define GPIO_PIN_RESET 0
#define TIM_CHANNEL_1 1
#define HAL_OK 0
#define osOK 0
static int htim3, enabled, pwm, direction, step, starts;
static int motorCommandQueue;
static void Error_Handler(void) { abort(); }
static int xQueueReceive(int queue, uint16_t *command, int timeout) {
  (void)queue; assert(timeout == 0);
  const uint16_t commands[] = {0, 1, 1, 2, 2, 0, 0, 2, 1, 1, 0, 1};
  if (step == 0 || step == 2) return 0;
  *command = commands[step]; return 1;
}
static void HAL_GPIO_WritePin(int port, int pin, int value) {
  (void)port;
  if (pin == CW_PIN) { assert(!enabled && !pwm); direction = value; }
  else { enabled = value; if (value) assert(pwm); }
}
static int HAL_TIM_PWM_Start(int *timer, int channel) {
  (void)timer; (void)channel; assert(!enabled && !pwm); pwm = 1; starts++; return HAL_OK;
}
static int HAL_TIM_PWM_Stop(int *timer, int channel) {
  (void)timer; (void)channel; assert(!enabled); pwm = 0; return HAL_OK;
}
static void osDelay(int ticks) {
  assert(ticks == 10);
  const int running[] = {0, 1, 1, 0, 1, 0, 0, 1, 0, 1, 0, 1};
  const int directions[] = {0, 1, 1, 1, 0, 0, 0, 0, 0, 1, 1, 1};
  assert(enabled == running[step] && pwm == running[step]);
  assert(direction == directions[step]);
  if (++step == 12) { assert(starts == 5); exit(0); }
}
'''
with tempfile.TemporaryDirectory() as directory:
    test = Path(directory) / "motor_test.c"
    binary = Path(directory) / "motor_test"
    test.write_text(defines + "\n" + harness + source[start:end]
                    + "\nint main(void) { MotorControlTask(NULL); }\n")
    subprocess.run(["gcc", "-std=c11", "-Wall", "-Wextra", "-Wno-unused-parameter",
                    "-Werror", str(test), "-o", str(binary)], check=True)
    subprocess.run([str(binary)], check=True)
# Exercise command parsing and queue publication independently of motor outputs.
comm_start = source.index("void ModbusCommTask(void *argument)\n{")
comm_end = source.index("void MotorControlTask(void *argument)\n{", comm_start)
comm_harness = r'''
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#define osOK 0
static int motorCommandQueue, step, locked, initialized, started;
static uint16_t motorCommand[1];
static struct { int ModBusSphrHandle; } modbusHandler;
static void Modbus_Comm_Init(void) { assert(!initialized); initialized = 1; }
static void ModbusStart(void *handler) {
  (void)handler; assert(initialized); started = 1;
}
static int osSemaphoreAcquire(int handle, int timeout) {
  (void)handle; assert(timeout == 0 && started && !locked);
  if (step == 4) return 1;
  locked = 1; return osOK;
}
static void osSemaphoreRelease(int handle) {
  (void)handle; assert(locked); locked = 0;
}
static int xQueueOverwrite(int queue, const uint16_t *command) {
  (void)queue; assert(!locked);
  const uint16_t expected[] = {0, 1, 2, 0, 0, 1};
  assert(*command == expected[step]); return 1;
}
static void osDelay(int ticks) {
  assert(ticks == 10);
  if (++step == 6) exit(0);
  const uint16_t raw[] = {0, 1, 2, 65535, 2, 1};
  motorCommand[0] = raw[step];
}
'''
with tempfile.TemporaryDirectory() as directory:
    test = Path(directory) / "comm_test.c"
    binary = Path(directory) / "comm_test"
    test.write_text(defines + "\n" + comm_harness + source[comm_start:comm_end]
                    + "\nint main(void) { ModbusCommTask(NULL); }\n")
    subprocess.run(["gcc", "-std=c11", "-Wall", "-Wextra", "-Wno-unused-parameter",
                    "-Werror", str(test), "-o", str(binary)], check=True)
    subprocess.run([str(binary)], check=True)
print("Command parsing/queue publication and motor startup, stop, reversal, and empty-queue checks passed.")
