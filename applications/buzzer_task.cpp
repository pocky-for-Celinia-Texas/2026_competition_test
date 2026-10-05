#include "cmsis_os.h"
#include "io/buzzer/buzzer.hpp"

// C板
sp::Buzzer buzzer(&htim4, TIM_CHANNEL_3, 84e6);

// 达妙
// sp::Buzzer buzzer(&htim12, TIM_CHANNEL_2, 240e6);

namespace
{
// ---------- 音名频率表(Hz)，要别的音按 f = 440 * 2^((n-69)/12) 自己加 ----------
constexpr uint16_t NOTE_REST = 0;  // 休止符: 只用来占时间，绝不能传给 set()（会除零）
constexpr uint16_t NOTE_C4 = 262, NOTE_D4 = 294, NOTE_E4 = 330, NOTE_F4 = 349, NOTE_G4 = 392,
                   NOTE_A4 = 440, NOTE_B4 = 494;
constexpr uint16_t NOTE_C5 = 523, NOTE_D5 = 587, NOTE_E5 = 659, NOTE_F5 = 698, NOTE_G5 = 784,
                   NOTE_A5 = 880, NOTE_B5 = 988;
constexpr uint16_t NOTE_C6 = 1047;

// ---------- 开机音乐：想换曲子只改这张表 ----------
// 每个音符 = {频率, 发声时长(ms)}，频率写 NOTE_REST 就是停顿。
// 例：上行琶音 -> {NOTE_C5,120},{NOTE_E5,120},{NOTE_G5,120},{NOTE_C6,300}
// 例：三声短鸣 -> {NOTE_C5,100},{NOTE_REST,100} 重复三次
struct Note
{
  uint16_t hz;
  uint16_t ms;
};

constexpr Note kMusic[] = {
  {NOTE_E5, 300}, {NOTE_E5, 300}, {NOTE_F5, 300}, {NOTE_G5, 300},  // 欢乐颂
  {NOTE_G5, 300}, {NOTE_F5, 300}, {NOTE_E5, 300}, {NOTE_D5, 300}, {NOTE_C5, 300},
  {NOTE_C5, 300}, {NOTE_D5, 300}, {NOTE_E5, 300}, {NOTE_E5, 450}, {NOTE_D5, 150},
};
constexpr uint32_t kMusicCount = sizeof(kMusic) / sizeof(kMusic[0]);
constexpr float kDuty = 0.5f;        // 占空比 0.5 时声音最响
constexpr uint32_t kNoteGapMs = 20;  // 音符之间的断音，避免同音高粘成一片
}  // namespace

extern "C" void buzzer_task()
{
  // 接着播放开机音乐，改上面的 kMusic 就能换曲子
  for (uint32_t i = 0; i < kMusicCount; i++) {
    if (kMusic[i].hz == NOTE_REST) {  // 停顿：只等待，不要调 set()
      osDelay(kMusic[i].ms);
      continue;
    }

    buzzer.set(kMusic[i].hz, kDuty);  // 每个音都要重设频率
    buzzer.start();
    osDelay(kMusic[i].ms);
    buzzer.stop();
    osDelay(kNoteGapMs);
  }

  while (true) {
    osDelay(100);  // 想更省事可以换成 osDelay(osWaitForever)
  }
}
