#include "cmsis_os.h"
#include "io/led/led.hpp"

sp::LED led(&htim5);

namespace
{
constexpr uint32_t kStepMs = 20;                       // 50 Hz 刷新
constexpr uint32_t kStepsPerSeg = 1000 / kStepMs;      // 每段 1 s
constexpr uint32_t kStepsPerCycle = kStepsPerSeg * 3;  // 红→绿→蓝 共 3 s
constexpr uint32_t kFlashSteps = 2;                    // 每段开头 40 ms 白光 = 1 Hz 心跳

void render(uint32_t step)
{
  const uint32_t phase = step % kStepsPerCycle;
  const uint32_t in_seg = phase % kStepsPerSeg;

  if (in_seg < kFlashSteps) {  // 心跳：白
    led.set(1.0f, 1.0f, 1.0f);
    return;
  }

  const float t = static_cast<float>(in_seg) / static_cast<float>(kStepsPerSeg);

  switch ((phase / kStepsPerSeg) % 3) {
    case 0:
      led.set(1.0f - t, t, 0.0f);
      break;  // 红 → 绿
    case 1:
      led.set(0.0f, 1.0f - t, t);
      break;  // 绿 → 蓝
    default:
      led.set(t, 0.0f, 1.0f - t);
      break;  // 蓝 → 红
  }
}
}  // namespace

extern "C" void led_task()
{
  led.start();

  while (true) {
    const uint32_t now = osKernelSysTick();
    const uint32_t step = now / kStepMs;

    render(step);

    osDelay((step + 1) * kStepMs - now);  // 对齐到下一个刷新边界
  }
}