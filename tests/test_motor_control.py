"""Run the real coil-controlled motor task with mocked HAL/RTOS calls."""
from pathlib import Path
import subprocess
import tempfile

source = (Path(__file__).resolve().parents[1] / 'Core/Src/main.c').read_text()
start = source.index('void MotorControlTask(void *argument)\n{')
end = source.index('/* USER CODE END 0 */', start)
harness = r'''
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#define MOTOR_EN (1u << 0u)
#define MOTOR_CW (1u << 1u)
#define READ_BIT(REG, BIT) ((REG) & (BIT))
#define GPIOB 0
#define CW_PIN 10
#define EN_PIN 11
#define GPIO_PIN_RESET 0
#define GPIO_PIN_SET 1
#define TIM_CHANNEL_1 1
#define HAL_OK 0
#define pdTRUE 1
typedef int GPIO_PinState;
static int htim3, enabled, direction, pwm, step, locked, pause, reads, fault;
static uint16_t modbusCoilsData[1];
static struct { int ModBusSphrHandle; } modbusHandle;
static const uint16_t commands[] = {0,1,1,3,3,0,3,1,5,0,0,1,1};
static void Error_Handler(const char *file, int line) {
  (void)file; (void)line; assert(fault && !enabled && !locked); exit(0);
}
static int xSemaphoreTake(int handle, int timeout) {
  (void)handle; assert(timeout == 0 && !locked);
  if (step == 9 || (step == 11 && reads++ == 1)) return 0;
  locked = 1; return pdTRUE;
}
static void xSemaphoreGive(int handle) { (void)handle; assert(locked); locked = 0; }
static void HAL_GPIO_WritePin(int port, int pin, int value) {
  (void)port; assert(!locked);
  if (pin == CW_PIN) { assert(!enabled && !pwm); direction = value; pause = 1; }
  else { enabled = value; if (value) assert(pwm); }
}
static int HAL_TIM_PWM_Start(int *timer, int channel) {
  (void)timer; (void)channel; assert(!locked && !enabled && !pwm);
  if (fault == 1) return 1;
  pwm = 1; return HAL_OK;
}
static int HAL_TIM_PWM_Stop(int *timer, int channel) {
  (void)timer; (void)channel; assert(!locked && !enabled && pwm);
  if (fault == 2) return 1;
  pwm = 0; return HAL_OK;
}
static void osDelay(int ticks) {
  assert(ticks == 10 && !locked);
  if (pause) {
    pause = 0; assert(!enabled && !pwm);
    if (step == 3) modbusCoilsData[0] = 0;
    if (step == 6) modbusCoilsData[0] = 1;
    return;
  }
  const int running[] = {0,1,1,0,1,0,0,1,1,1,0,0,1};
  const int directions[] = {0,1,1,0,0,0,0,1,1,1,1,1,1};
  assert(enabled == running[step] && pwm == running[step]);
  assert(direction == directions[step]);
  if (++step == 13) { assert(!fault); exit(0); }
  reads = 0; modbusCoilsData[0] = commands[step];
}
'''
with tempfile.TemporaryDirectory() as directory:
    test = Path(directory) / 'motor.c'
    binary = Path(directory) / 'motor'
    test.write_text(harness + source[start:end] + r'''
int main(int argc, char **argv) {
  assert(argc == 2); fault = atoi(argv[1]);
  modbusCoilsData[0] = commands[0]; MotorControlTask(NULL);
}
''')
    subprocess.run(['gcc', '-std=c11', '-Wall', '-Wextra', '-Werror',
                    '-Wno-unused-parameter', str(test), '-o', str(binary)], check=True)
    for fault in (0, 1, 2):
        subprocess.run([str(binary), str(fault)], check=True)
print('Coil startup, restart, reversal, pause updates, lock failures, and PWM failures passed.')
