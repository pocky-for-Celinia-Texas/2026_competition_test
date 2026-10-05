#include "cmsis_os.h"
#include "io/bmi088/bmi088.hpp"
#include "io/plotter/plotter.hpp"
#include "tools/mahony/mahony.hpp"

// ===================== 可选：IMU 加热 =====================
// 改成 1 时，要把 sp_middleware/tools/pid/pid.cpp 也加进 CMakeLists.txt
#define IMU_HEAT_ENABLE 0

#if IMU_HEAT_ENABLE
#include "tim.h"
#include "tools/pid/pid.hpp"
#endif

namespace
{
// --------------------- 任务参数 ---------------------
constexpr uint32_t kPeriodMs = 1;                  // 任务周期 1 ms
constexpr float    kDt       = kPeriodMs * 1e-3f;  // Mahony/PID 的 dt，必须与任务周期严格一致

// 传感器系{b} -> 机器人系{a} 的旋转矩阵（readme 里的 r_ab）
// 这是 C 板横装在云台上、CAN 一侧朝前时的取值；三轴符号不对就改这里，别动驱动
constexpr float kRAb[3][3] = {{0.0f, -1.0f, 0.0f}, {1.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 1.0f}};

#if IMU_HEAT_ENABLE
// --------------------- 加热参数 ---------------------
constexpr float    kImuTemp        = 50.0f;    // 目标温度 ℃
constexpr float    kImuTempKp      = 400.0f;   // 发热丝功率不同需要微调
constexpr float    kImuTempKi      = 0.0f;
constexpr float    kImuTempKd      = 0.0f;
constexpr float    kImuTempMaxOut  = 2000.0f;  // 限制最大输出，防止静态积分时全功率烘烤
constexpr float    kImuTempMaxIOut = 0.0f;
constexpr uint16_t kImuTempPwmFull = 4000;     // 温差还很大时的预热占空比（满量程 5000）
constexpr float    kImuTempBand    = 10.0f;    // 进入 PID 调节的温差
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
  kDt, kImuTempKp, kImuTempKi, kImuTempKd, kImuTempMaxOut, kImuTempMaxIOut, 1.0f);

// 陀螺仪温度控制：温度反馈用 BMI088 自带的温度传感器，PWM 输出到 PF6(TIM10_CH1) 驱动加热电阻
void imu_temp_control(float temp)
{
  uint16_t temp_pwm = 0;

  if (kImuTemp - temp > kImuTempBand) {
    temp_pwm = kImuTempPwmFull;  // 离目标还远，先全速预热
  }
  else {
    imu_temp_pid.calc(kImuTemp, temp);

    // 输出下限钳位：负值转 uint16_t 会溢出成满占空比
    if (imu_temp_pid.out < 0.0f) imu_temp_pid.out = 0.0f;
    temp_pwm = static_cast<uint16_t>(imu_temp_pid.out);
  }

  __HAL_TIM_SET_COMPARE(&htim10, TIM_CHANNEL_1, temp_pwm);
}
#endif

extern "C" void imu_task()
{
  bmi088.init();  // 内部一直重试，卡在这里 = SPI 引脚/片选/模式配错了

#if IMU_HEAT_ENABLE
  HAL_TIM_PWM_Start(&htim10, TIM_CHANNEL_1);  // C 板加热 PWM: PF6 / TIM10_CH1
#endif

  while (true) {
    bmi088.update();                      // 阻塞式 SPI 读，单位 m/s²、rad/s
    imu.update(bmi088.acc, bmi088.gyro);  // Mahony 姿态解算

#if IMU_HEAT_ENABLE
    imu_temp_control(bmi088.temp);
#endif

    // 三轴加速度 + 三轴角速度：一帧 6 通道 27 字节 ≈ 0.29 ms @921600，1 kHz 发送很安全
    // 想连姿态角一起看就换成：
    // plotter.plot(imu.roll, imu.pitch, imu.yaw,
    //              bmi088.gyro[0], bmi088.gyro[1], bmi088.gyro[2]);
    plotter.plot(
      bmi088.acc[0], bmi088.acc[1], bmi088.acc[2], bmi088.gyro[0], bmi088.gyro[1],
      bmi088.gyro[2]);

    osDelay(kPeriodMs);
  }
}
