"""Exercise the real PWM counter and tracking task with mocked HAL/RTOS."""
from pathlib import Path
import subprocess
import tempfile

source = (Path(__file__).resolve().parents[1] / 'Core/Src/main.c').read_text()
helpers = source[source.index('static void RecordMotionPulse(void)\n{'):
                 source.index('static uint16_t SpeedFrequency(uint16_t speed)\n{')]
task = source[source.index('void MotionTrackingTask(void *argument)\n{'):
              source.index('/* USER CODE END 0 */')]
calibration = source[source.index('#define WHEEL_RADIUS_MM'):source.index('/* USER CODE END PD */')]
harness = r'''
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#define TIM3 3
#define TIM_FLAG_CC1 1
#define HAL_TIM_ACTIVE_CHANNEL_1 1
#define MOTION_TRACK_EN 4
#define MOTION_ZERO 8
#define RESET 0
#define pdTRUE 1
typedef struct { int Instance, Channel, pending; } TIM_HandleTypeDef;
static TIM_HandleTypeDef htim3 = {TIM3, HAL_TIM_ACTIVE_CHANNEL_1, 0};
static volatile bool motionTrackingEnabled, motionMotorRunning;
static volatile int32_t motionDirection = 1, motionIntervalPulses;
static volatile uint64_t motionPosition;
static volatile uint64_t travelRemainingPulses;
static volatile bool travelComplete, travelStopFailed;
#define GPIOB 0
#define EN_PIN 11
#define GPIO_PIN_SET 1
#define TIM_CHANNEL_1 1
#define HAL_OK 0
static void HAL_GPIO_WritePin(int port, int pin, int value) { assert(0); }
static int HAL_TIM_PWM_Stop_IT(TIM_HandleTypeDef *timer, int channel) { assert(0); return HAL_OK; }
static uint16_t modbusInputRegistersData[6];
static uint16_t modbusCoilsData[1];
static struct { int ModBusSphrHandle; } modbusHandle;
static int critical, locked, sample = -1;
static uint32_t now;
#define taskENTER_CRITICAL() (++critical)
#define taskEXIT_CRITICAL() (--critical)
#define __HAL_TIM_GET_FLAG(timer, flag) ((timer)->pending)
#define __HAL_TIM_CLEAR_FLAG(timer, flag) ((timer)->pending = 0)
static uint32_t HAL_GetTick(void) { return now; }
static uint32_t osKernelGetTickFreq(void) { return 1000; }
static void osDelay(uint32_t ticks);
static const int32_t expectedSpeed[] = {70,-14,0,0,0,70,0,-706,0,-70,0,-70,0,0,35,0,70,0,0,70,0,1413,-1413,0,0};
static const int64_t expectedPosition[] = {7,3,3,10,17,24,24,-45,-46,-53,-53,-60,-60,0,3,0,7,14,0,7,0,141,0,6519617512269410304, -6519617512269410304};
static uint64_t publishedPosition(void) {
  uint64_t result = 0;
  for (int i = 2; i < 6; ++i) result = (result << 16) | modbusInputRegistersData[i];
  return result;
}
static int xSemaphoreTake(int handle, int timeout) {
  (void)handle; assert(timeout == 0 && !locked && !critical);
  assert(sample != 8); // A zero-length sample must not publish or discard pulses.
  if (sample == 3 || sample == 4 || sample == 17) {
    assert(publishedPosition() == (sample == 17 ? 7 : 3));
    if (sample == 17) assert(modbusCoilsData[0] & MOTION_ZERO);
    return 0;
  }
  locked = 1; return pdTRUE;
}
static void xSemaphoreGive(int handle) {
  (void)handle; assert(locked && !critical); locked = 0;
  uint32_t speed = ((uint32_t)modbusInputRegistersData[0] << 16) | modbusInputRegistersData[1];
  assert(speed == (uint32_t)expectedSpeed[sample]);
  assert(publishedPosition() == (uint64_t)expectedPosition[sample]);
  assert(!(modbusCoilsData[0] & MOTION_ZERO));
  if (sample >= 13) assert(modbusCoilsData[0] == (sample < 14 || sample == 20 ? 0x8003 : 0x8007));
  if (sample == 13 || sample == 15 || sample == 18 || sample == 20) {
    assert(motionPosition == 0 && motionIntervalPulses == 0 && !htim3.pending);
  }
}
'''
scenario = r'''
static void pulses(int count) {
  while (count--) HAL_TIM_PWM_PulseFinishedCallback(&htim3);
}
static void osDelay(uint32_t ticks) {
  assert(ticks == 100 && !critical && !locked);
  if (++sample == 25) exit(0);
  now += 100;
  switch (sample) {
    case 0:
      motionMotorRunning = true; SetMotionTracking(MOTION_TRACK_EN);
      pulses(9); htim3.pending = 1; break;
    case 1:
      now += 150; motionDirection = -1; pulses(5); break;
    case 2:
      SetMotionTracking(0); pulses(20); break;
    case 3:
      motionDirection = 1; SetMotionTracking(MOTION_TRACK_EN); pulses(10); break;
    case 4: case 5: pulses(10); break;
    case 6: motionMotorRunning = false; pulses(10); break;
    case 7: motionMotorRunning = true; motionDirection = -1; pulses(100); break;
    case 8: now -= 100; pulses(1); break;
    case 9: pulses(9); break;
    case 10: now = UINT32_MAX - 49; break;
    case 11: assert(now == 50); pulses(10); break;
    case 12:
      htim3.pending = 1; SetMotionTracking(0); pulses(10); break;
    case 13: modbusCoilsData[0] = 0x8003 | MOTION_ZERO; break;
    case 14:
      modbusCoilsData[0] = 0x8007; SetMotionTracking(MOTION_TRACK_EN);
      motionDirection = 1; pulses(5); break;
    case 15:
      pulses(5); htim3.pending = 1; modbusCoilsData[0] |= MOTION_ZERO; break;
    case 16: case 19: pulses(10); break;
    case 17: pulses(10); modbusCoilsData[0] |= MOTION_ZERO; break;
    case 18: pulses(9); htim3.pending = 1; break;
    case 20:
      motionMotorRunning = false; SetMotionTracking(0);
      modbusCoilsData[0] = 0x8003 | MOTION_ZERO; break;
    case 21:
      motionMotorRunning = true; SetMotionTracking(MOTION_TRACK_EN);
      modbusCoilsData[0] = 0x8007; pulses(200); break;
    case 22: motionDirection = -1; pulses(200); break;
    case 23: motionPosition = INT64_MAX; break;
    case 24: motionPosition = UINT64_C(1) << 63; break;
  }
}
int main(void) {
  // Channel filtering, stopped/disabled pulses, 64-bit carry and negative position.
  pulses(10); assert(motionPosition == 0);
  motionMotorRunning = true; pulses(10); assert(motionPosition == 0);
  SetMotionTracking(MOTION_TRACK_EN);
  TIM_HandleTypeDef other = {4, HAL_TIM_ACTIVE_CHANNEL_1, 0};
  HAL_TIM_PWM_PulseFinishedCallback(&other);
  other.Instance = TIM3; other.Channel = 2;
  HAL_TIM_PWM_PulseFinishedCallback(&other); assert(motionPosition == 0);
  motionPosition = UINT32_MAX; pulses(1); assert(motionPosition == UINT64_C(1) << 32);
  motionPosition = 0; motionDirection = -1; pulses(1); assert(motionPosition == UINT64_MAX);
  SetMotionTracking(0); assert(motionPosition == UINT64_MAX);
  motionPosition = 0; motionIntervalPulses = 0; motionDirection = 1;
  MotionTrackingTask(NULL);
}
'''
with tempfile.TemporaryDirectory() as directory:
    test = Path(directory) / 'motion.c'
    binary = Path(directory) / 'motion'
    test.write_text(calibration + harness + helpers + task + scenario)
    subprocess.run(['gcc', '-std=c11', '-Wall', '-Wextra', '-Werror',
                    '-Wno-unused-parameter', '-fsanitize=undefined',
                    str(test), '-o', str(binary)], check=True)
    subprocess.run([str(binary)], check=True)
print('Wheel conversion, signed millimetre distance/speed, extreme positions, pulse tracking, and zeroing passed.')
