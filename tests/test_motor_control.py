"""Run the real speed/travel motor task with mocked HAL/RTOS calls."""
from pathlib import Path
import subprocess
import tempfile

source = (Path(__file__).resolve().parents[1] / 'Core/Src/main.c').read_text()
code = source[source.index('static void RecordMotionPulse(void)\n{'):
              source.index('void MotionTrackingTask(void *argument)\n{')]
calibration = source[source.index('#define WHEEL_RADIUS_MM'):source.index('/* USER CODE END PD */')]
harness = r'''
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <setjmp.h>
#define MOTOR_EN 1
#define MOTOR_CW 2
#define MOTION_TRACK_EN 4
#define TIM3 3
#define TIM3_IRQn 3
#define TIM_FLAG_CC1 1
#define HAL_TIM_ACTIVE_CHANNEL_1 1
#define RESET 0
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
static TIM_HandleTypeDef htim3 = {TIM3, 1, 0, 0};
static volatile bool motionTrackingEnabled, motionMotorRunning;
static volatile int32_t motionDirection = 1, motionIntervalPulses;
static volatile uint64_t motionPosition, travelRemainingPulses;
static volatile bool travelComplete, travelStopFailed;
static uint16_t modbusHoldingRegistersData[5], modbusCoilsData[1];
static struct { int ModBusSphrHandle; } modbusHandle;
static int enabled, direction, pwm, locked, critical, pause, step, reads;
static int prescaler, fault, scenario;
static jmp_buf done;
#define taskENTER_CRITICAL() (++critical)
#define taskEXIT_CRITICAL() (--critical)
#define __HAL_TIM_GET_FLAG(timer, flag) ((timer)->pending)
#define __HAL_TIM_CLEAR_FLAG(timer, flag) ((timer)->pending = 0)
#define __HAL_TIM_SET_COUNTER(timer, value) ((timer)->counter = (value))
#define __HAL_TIM_SET_PRESCALER(timer, value) (prescaler = (value))
static uint32_t HAL_RCC_GetPCLK1Freq(void) { return 8000000; }
static int HAL_TIM_GenerateEvent(TIM_HandleTypeDef *timer, int event) {
  assert(critical && !pwm && !enabled); return fault == 4 ? 1 : HAL_OK;
}
static void HAL_NVIC_ClearPendingIRQ(int irq) { assert(critical); }
static void HAL_GPIO_WritePin(int port, int pin, int value) {
  assert(!locked);
  if (pin == CW_PIN) { assert(!enabled && !pwm); direction = value; pause = 1; htim3.pending = 1; }
  else { enabled = value == GPIO_PIN_RESET; if (enabled) assert(pwm); }
}
static int HAL_TIM_PWM_Start_IT(TIM_HandleTypeDef *timer, int channel) {
  assert(!locked && !enabled && !pwm && critical);
  assert(!timer->pending && timer->counter == 0);
  if (fault == 1) return 1;
  pwm = 1; return HAL_OK;
}
static int HAL_TIM_PWM_Stop_IT(TIM_HandleTypeDef *timer, int channel) {
  assert(!locked && !enabled);
  if (scenario == 0 && step == 18 && pwm) timer->pending = 1;
  if (fault == 2 || fault == 3) return 1;
  pwm = 0; return HAL_OK;
}
static void Error_Handler(const char *file, int line) {
  assert(fault && !enabled && !locked); longjmp(done, 2);
}
static int xSemaphoreTake(int handle, int timeout) {
  assert(!locked && !critical && timeout == 0);
  ++reads;
  if (scenario == 2 && ((step == 1 && reads == 2) || step == 3)) return 0;
  if (scenario == 3 && (step == 1 || step == 2)) return 0;
  locked = 1; return pdTRUE;
}
static void xSemaphoreGive(int handle) { assert(locked); locked = 0; }
static void osDelay(int ticks);
'''
scenario_code = r'''
static void command(uint16_t coils, uint16_t speed, uint64_t distance) {
  modbusCoilsData[0] = coils; modbusHoldingRegistersData[0] = speed;
  for (int i = 0; i < 4; ++i) modbusHoldingRegistersData[1+i] = distance >> (48-16*i);
}
static void pulse(void) { HAL_TIM_PWM_PulseFinishedCallback(&htim3); }
static void osDelay(int ticks) {
  assert(ticks == 10 && !locked && !critical);
  if (pause) {
    pause = 0; assert(!enabled && !pwm);
    if (scenario == 2 && step == 0) modbusHoldingRegistersData[0] = 200;
    if (scenario == 2 && step == 4) modbusCoilsData[0] = 0;
    if (scenario == 2 && step == 6) modbusCoilsData[0] = 3;
    return;
  }
  if (scenario == 0) {
    switch (step) {
      case 0: assert(!pwm); command(0x8005,100,10); break;
      case 1:
        assert(pwm && enabled && direction == 1 && prescaler == 566);
        assert(travelRemainingPulses == 14); pulse(); pulse();
        assert(motionPosition == 2 && travelRemainingPulses == 12);
        command(0x8003,200,(uint64_t)-99); break; // Edits do not retarget the move.
      case 2:
        assert(pwm && direction == 1 && prescaler == 282 && travelRemainingPulses == 12);
        pulse(); assert(travelRemainingPulses == 11 && motionPosition == 2);
        command(0x8003,0,0); break;
      case 3:
        assert(!pwm && !enabled && travelRemainingPulses == 11);
        command(0x8003,65535,0); break;
      case 4:
        assert(pwm && direction == 1 && prescaler == 113 && travelRemainingPulses == 11);
        // Telemetry zero has no effect on travel.
        motionPosition = 0; motionIntervalPulses = 0;
        for (int i = 0; i < 10; ++i) { pulse(); assert(pwm); }
        pulse(); assert(!pwm && !enabled && travelComplete && !motionMotorRunning);
        pulse(); assert(travelRemainingPulses == 0); break;
      case 5:
        assert(modbusCoilsData[0] == 0x8002 && !pwm && !travelComplete);
        command(5,1,(uint64_t)-1); break;
      case 6:
        assert(pwm && direction == 0 && prescaler == 799 && travelRemainingPulses == 1);
        pulse(); assert(!pwm && motionPosition == UINT64_MAX); break;
      case 7:
        assert(modbusCoilsData[0] == 4); command(1,100,10); break;
      case 8: assert(pwm); pulse(); command(0,100,10); break;
      case 9: assert(!pwm && !travelRemainingPulses); command(1,100,10); break;
      case 10: assert(pwm && travelRemainingPulses == 14); command(0,100,0); break;
      case 11: assert(!pwm); command(1,100,0); break;
      case 12: assert(pwm && direction == 1 && !travelRemainingPulses); command(3,100,0); break;
      case 13: assert(pwm && direction == 0); command(0,100,0); break;
      case 14: assert(!pwm); command(1,100,UINT64_C(1)<<63); break;
      case 15:
        assert(pwm && direction == 0 && travelRemainingPulses > (uint64_t)INT64_MAX);
        command(0,100,0); break;
      case 16: assert(!pwm); command(1,100,1); break;
      case 17:
        assert(pwm && travelRemainingPulses == 1);
        command(1,200,1); break; // Final edge arrives during the stop call.
      case 18: assert(!pwm && travelComplete && !travelRemainingPulses); break;
      case 19: assert(!pwm && !(modbusCoilsData[0] & MOTOR_EN)); longjmp(done, 1);
    }
  } else if (scenario == 1) {
    assert(fault);
    if (step == 0) {
      assert(pwm); if (fault == 3) { pulse(); assert(travelStopFailed); }
      else command(0,100,0);
    } else assert(0);
  } else if (scenario == 3) {
    if (step == 0) { assert(pwm); pulse(); assert(travelComplete && !pwm); }
    else if (step < 3) assert(!pwm && travelComplete && (modbusCoilsData[0] & MOTOR_EN));
    else { assert(!pwm && !travelComplete && modbusCoilsData[0] == 0x8004); longjmp(done,1); }
  } else {
    switch (step) {
      case 0: assert(!pwm); break; // Speed changed during startup.
      case 1: assert(!pwm); break; // Startup recheck could not lock.
      case 2: assert(pwm && prescaler == 282); break;
      case 3: assert(pwm); command(3,100,0); break; // Failed read retains motion.
      case 4: assert(!pwm); break; // Disable during reversal pause.
      case 5: assert(!pwm); command(1,100,0); break;
      case 6: assert(!pwm); break; // Direction changed during startup.
      case 7: assert(pwm && direction == 0); longjmp(done,1);
    }
  }
  ++step; reads = 0;
}
int main(int argc, char **argv) {
  assert(argc == 3); scenario = atoi(argv[1]); fault = atoi(argv[2]);
  assert(SpeedFrequency(0) == 0 && SpeedFrequency(1) == 100);
  assert(SpeedFrequency(100) == 141 && SpeedFrequency(200) == 283);
  assert(SpeedFrequency(65535) == 700);
  command(0,100,UINT64_MAX); assert(ReadTravelCommand() == UINT64_MAX);
  command(scenario ? 1 : 0,100,fault == 3 ? 1 : 0);
  if (scenario == 3) command(0x8005,100,1);
  int result = setjmp(done);
  if (!result) MotorControlTask(NULL);
  assert(result == (fault ? 2 : 1));
  return 0;
}
'''
with tempfile.TemporaryDirectory() as directory:
    test = Path(directory) / 'motor.c'
    binary = Path(directory) / 'motor'
    test.write_text(calibration + harness + code + scenario_code)
    subprocess.run(['gcc', '-std=c11', '-Wall', '-Wextra', '-Werror',
                    '-Wno-unused-parameter', '-fsanitize=undefined',
                    str(test), '-o', str(binary)], check=True)
    for scenario, fault in ((0,0), (2,0), (3,0), (1,1), (1,2), (1,3), (1,4)):
        subprocess.run([str(binary), str(scenario), str(fault)], check=True)
print('Speed conversion, finite travel, pulse stopping, command races, lock and HAL failures passed.')
