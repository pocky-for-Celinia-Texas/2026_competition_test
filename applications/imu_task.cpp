#include "cmsis_os.h"
#include "io/bmi088/bmi088.hpp"
#include "io/plotter/plotter.hpp"
#include "tools/mahony/mahony.hpp"

// ===================== IMU 加热 =====================
// 默认开启：PWM 走 PF6 / TIM10_CH1（C 板），温度反馈用 BMI088 自带温度传感器。
// 依赖 sp_middleware/tools/pid/pid.cpp，需已在 CMakeLists.txt 的 target_sources 里。
// 没有接加热电阻、或 PF6 另有用途时改成 0。
#define IMU_HEAT_ENABLE 0

#if IMU_HEAT_ENABLE
#include "tim.h"
#include "tools/pid/pid.hpp"
#endif

namespace
{
// --------------------- 任务参数 ---------------------
constexpr uint32_t kPeriodMs = 1;         // 任务周期 1 ms
constexpr float kDt = kPeriodMs * 1e-3f;  // Mahony/PID 的 dt，必须与任务周期严格一致

// 传感器系{b} -> 机器人系{a} 的旋转矩阵（readme 里的 r_ab）
// 这是 C 板横装在云台上、CAN 一侧朝前时的取值；三轴符号不对就改这里，别动驱动
constexpr float kRAb[3][3] = {{0.0f, -1.0f, 0.0f}, {1.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 1.0f}};

#if IMU_HEAT_ENABLE
// --------------------- 加热参数 ---------------------
constexpr float kImuTemp = 50.0f;     // 目标温度 ℃
constexpr float kImuTempKp = 400.0f;  // 发热丝功率不同需要微调
constexpr float kImuTempKi = 0.0f;
constexpr float kImuTempKd = 0.0f;
constexpr float kImuTempMaxDuty = 0.4f;  // PID 输出上限占空比（= 2000/5000），防止一直全功率烘烤
constexpr float kImuTempMaxIOut = 0.0f;
constexpr float kImuTempBand = 10.0f;    // 进入 PID 调节的温差，单位 ℃
constexpr float kImuTempPreheat = 0.8f;  // 温差还很大时的预热占空比
constexpr float kImuTempCutOff = 60.0f;  // 过温保护阈值，超过就断加热
constexpr float kImuTempAlpha = 0.1f;    // 温度反馈一阶滤波系数，1 表示不滤波

// PWM 满量程 = ARR + 1，即 100% 占空比对应的 CCR。
// 计数频率固定 1MHz，所以 PWM 频率 = 1MHz / kImuTempPwmFullScale = 200Hz。
constexpr uint32_t kImuTempPwmFullScale = 5000;
constexpr uint32_t kImuTempTickHz = 1000000;
#endif
}  // namespace

// --------------------- 对象 ---------------------
// C 板: SPI1, CSB1 = PA4(加速度计), CSB2 = PB0(陀螺仪)
sp::BMI088 bmi088(&hspi1, GPIOA, GPIO_PIN_4, GPIOB, GPIO_PIN_0, kRAb);
// 达妙: sp::BMI088 bmi088(&hspi2, GPIOC, GPIO_PIN_0, GPIOC, GPIO_PIN_3, kRAb);

// 姿态解算（四元数 + 欧拉角），dt 必须等于 kDt
sp::Mahony imu(kDt);

// 上位机波形：USART1（921600 8N1 + DMA），帧格式 0xAA 0xBB + 长度 + float32 小端
// 注意：一个 UART 只能有一个 Plotter。若工程里已经有 plot_task.cpp 的 plotter(&huart1)，
//       请只保留一个，或把电机波形和 IMU 数据合并到同一个任务里发送。
sp::Plotter plotter(&huart1);

#if IMU_HEAT_ENABLE
sp::PID imu_temp_pid(
  kDt, kImuTempKp, kImuTempKi, kImuTempKd, kImuTempMaxDuty * kImuTempPwmFullScale, kImuTempMaxIOut,
  1.0f);

namespace
{
bool heater_ready = false;

// APB2 分频系数不为 1 时，定时器时钟是 PCLK2 的两倍（C 板 168MHz）
uint32_t heater_timer_clock_hz()
{
  uint32_t pclk2 = HAL_RCC_GetPCLK2Freq();
  return (pclk2 == HAL_RCC_GetHCLKFreq()) ? pclk2 : pclk2 * 2U;
}

// 显式配置 TIM10，不依赖 CubeMX 里 Period 的默认值：
// 计数 1MHz、满量程 kImuTempPwmFullScale（200Hz PWM），并把初始占空比清零
void heater_pwm_init()
{
  __HAL_TIM_SET_PRESCALER(&htim10, heater_timer_clock_hz() / kImuTempTickHz - 1U);
  __HAL_TIM_SET_AUTORELOAD(&htim10, kImuTempPwmFullScale - 1U);
  __HAL_TIM_SET_COUNTER(&htim10, 0);
  __HAL_TIM_SET_COMPARE(&htim10, TIM_CHANNEL_1, 0);

  heater_ready = (HAL_TIM_PWM_Start(&htim10, TIM_CHANNEL_1) == HAL_OK);
}
}  // namespace

// 陀螺仪温度控制：温度反馈用 BMI088 自带的温度传感器，PWM 输出到 PF6(TIM10_CH1) 驱动加热电阻
// temp: 摄氏度；输出 CCR 取值 [0, kImuTempPwmFullScale]
void imu_temp_control(float temp)
{
  static float temp_fdb = 0.0f;
  static bool temp_fdb_ready = false;

  if (!heater_ready) return;

  // 温度是慢变量，实际循环周期与 kDt 的偏差对加热控制可忽略；
  // 这里做一阶滤波，抑制温度读数跳变带来的占空比抖动
  if (!temp_fdb_ready) {
    temp_fdb = temp;
    temp_fdb_ready = true;
  }
  else {
    temp_fdb += kImuTempAlpha * (temp - temp_fdb);
  }

  float ccr = 0.0f;

  if (temp_fdb >= kImuTempCutOff) {
    ccr = 0.0f;  // 过温保护：断加热并清掉 PID 状态
    imu_temp_pid.clear();
  }
  else if (kImuTemp - temp_fdb > kImuTempBand) {
    ccr = kImuTempPreheat * kImuTempPwmFullScale;  // 离目标还远，先全速预热
  }
  else {
    imu_temp_pid.calc(kImuTemp, temp_fdb);  // PID 内部已按 max_out 限幅
    ccr = imu_temp_pid.out;
  }

  // 输出钳位：负值转 uint16_t 会溢出成满占空比
  if (ccr < 0.0f) ccr = 0.0f;
  if (ccr > static_cast<float>(kImuTempPwmFullScale)) {
    ccr = static_cast<float>(kImuTempPwmFullScale);
  }

  __HAL_TIM_SET_COMPARE(&htim10, TIM_CHANNEL_1, static_cast<uint16_t>(ccr));
}
#endif

extern "C" void imu_task()
{
  bmi088.init();  // 内部一直重试，卡在这里 = SPI 引脚/片选/模式配错了

#if IMU_HEAT_ENABLE
  heater_pwm_init();  // C 板加热 PWM: PF6 / TIM10_CH1（1MHz 计数，满量程 5000 → 200Hz）
#endif

  while (true) {
    bmi088.update();
    imu.update(bmi088.acc, bmi088.gyro);  // 注意：别再直接把 bmi088.gyro 传进去

#if IMU_HEAT_ENABLE
    imu_temp_control(bmi088.temp);
#endif

    // 三轴加速度 + 三轴角速度：一帧 6 通道 27 字节 ≈ 0.29 ms @921600，1 kHz 发送很安全
    // 想连姿态角一起看就换成：
    plotter.plot(imu.roll, imu.pitch, imu.yaw, bmi088.gyro[0], bmi088.gyro[1], bmi088.gyro[2]);
    //plotter.plot(
    //  bmi088.acc[0], bmi088.acc[1], bmi088.acc[2], bmi088.gyro[0], bmi088.gyro[1], bmi088.gyro[2]);

    osDelay(kPeriodMs);
  }
}
