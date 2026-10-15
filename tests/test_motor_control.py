"""Run the real coil-controlled motor task with mocked HAL/RTOS calls."""
from pathlib import Path
import subprocess
import tempfile

source = (Path(__file__).resolve().parents[1] / 'Core/Src/main.c').read_text()
start = source.index('static void RecordMotionPulse(void)\n{')
end = source.index('void MotionTrackingTask(void *argument)\n{', start)
harness = r'''
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#define MOTOR_EN (1u << 0u)
#define MOTOR_CW (1u << 1u)
#define MOTION_TRACK_EN (1u << 2u)
#define TIM3 3
#define TIM3_IRQn 3
#define TIM_FLAG_CC1 1
#define HAL_TIM_ACTIVE_CHANNEL_1 1
#define RESET 0
#define READ_BIT(REG, BIT) ((REG) & (BIT))
#define GPIOB 0
#define CW_PIN 10
#define EN_PIN 11
#define GPIO_PIN_RESET 0
#define GPIO_PIN_SET 1
#define TIM_CHANNEL_1 1
#define TIM_EVENTSOURCE_UPDATE 1
#define HAL_OK 0
#define pdTRUE 1
typedef int GPIO_PinState;
typedef struct { int Instance, Channel, counter, pending; } TIM_HandleTypeDef;
static TIM_HandleTypeDef htim3 = {.Instance = TIM3};
static int enabled, direction, pwm, step, locked, pause, reads, fault, critical;
static int prescaler;
static const uint16_t frequencies[] = {100,100,700,700,1,100,100,100,100,100,100,100,65535};
static uint16_t modbusHoldingRegistersData[2] = {100,0};
static uint32_t HAL_RCC_GetPCLK1Freq(void) { return 8000000; }
#define __HAL_TIM_SET_PRESCALER(timer, value) (prescaler = (value))
static int HAL_TIM_GenerateEvent(TIM_HandleTypeDef *timer, int event) {
  (void)timer; (void)event; assert(critical && !pwm && !enabled); return HAL_OK;
}
static volatile bool motionTrackingEnabled, motionMotorRunning;
static volatile int32_t motionDirection = 1, motionIntervalPulses;
static volatile uint64_t motionPosition;
#define taskENTER_CRITICAL() (++critical)
#define taskEXIT_CRITICAL() (--critical)
#define __HAL_TIM_GET_FLAG(timer, flag) ((timer)->pending)
#define __HAL_TIM_CLEAR_FLAG(timer, flag) ((timer)->pending = 0)
#define __HAL_TIM_SET_COUNTER(timer, value) ((timer)->counter = (value))
static void HAL_NVIC_ClearPendingIRQ(int irq) { (void)irq; assert(critical); }
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
  if (pin == CW_PIN) { assert(!enabled && !pwm); direction = value; pause = 1; htim3.pending = 1; }
  else { enabled = (value == GPIO_PIN_RESET); if (enabled) assert(pwm); }
}
static int HAL_TIM_PWM_Start_IT(TIM_HandleTypeDef *timer, int channel) {
  (void)timer; (void)channel; assert(!locked && !enabled && !pwm);
  if (fault == 1) return 1;
  assert(critical && !timer->pending && timer->counter == 0);
  assert(prescaler == ((step == 2 || step == 12) ? 113 : 799));
  pwm = 1; return HAL_OK;
}
static int HAL_TIM_PWM_Stop_IT(TIM_HandleTypeDef *timer, int channel) {
  (void)timer; (void)channel; assert(!locked && !enabled && pwm);
  if (fault == 2) return 1;
  pwm = 0; return HAL_OK;
}
static void FlushMotionPulse(void);
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim);
static void osDelay(int ticks) {
  assert(ticks == 10 && !locked && !critical);
  if (pause) {
    pause = 0; assert(!enabled && !pwm);
    if (step == 3) modbusCoilsData[0] = 0;
    if (step == 6) modbusCoilsData[0] = 1;
    if (step == 11) modbusHoldingRegistersData[0] = 200;
    if (fault == 3 && step == 1) modbusHoldingRegistersData[0] = 700;
    return;
  }
  if (fault == 3 && step == 1) {
    assert(!enabled && !pwm && !motionMotorRunning); exit(0);
  }
  const int running[] = {0,1,1,0,1,0,0,1,1,1,0,0,1};
  const int directions[] = {0,1,1,0,0,0,0,1,1,1,1,1,1};
  assert(enabled == running[step] && pwm == running[step]);
  assert(direction == directions[step]);
  assert(motionMotorRunning == (bool)pwm);
  if (pwm) assert(motionDirection == (direction ? 1 : -1));
  if (step >= 10) assert(motionPosition == 4);
  if (pwm) {
    taskENTER_CRITICAL();
    FlushMotionPulse();
    taskEXIT_CRITICAL();
    htim3.Channel = HAL_TIM_ACTIVE_CHANNEL_1;
    HAL_TIM_PWM_PulseFinishedCallback(&htim3);
    htim3.pending = 1;
  }
  if (++step == 13) { assert(!fault); exit(0); }
  reads = 0; modbusCoilsData[0] = commands[step];
  modbusHoldingRegistersData[0] = frequencies[step];
}
'''
def main():
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
        for fault in (0, 1, 2, 3):
            subprocess.run([str(binary), str(fault)], check=True)
    print('Coil control, frequency changes/clamping, pause updates, lock failures, and PWM failures passed.')

if __name__ == '__main__':
    main()
