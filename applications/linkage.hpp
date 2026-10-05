#ifndef APPLICATIONS_LINKAGE_HPP
#define APPLICATIONS_LINKAGE_HPP

// 4.4.1.2 姿态与电机联动：共享对象与常量
//
// 定义成 inline 变量，任何 include 它的任务都拿到同一个对象（A/B 电机、CAN1、左拨杆比例表）。

#include "io/can/can.hpp"
#include "io/dbus/dbus.hpp"
#include "motor/rm_motor/rm_motor.hpp"
#include "tools/mahony/mahony.hpp"

namespace sp_app
{

// ------------------- 需要按你的硬件确认 -------------------
// 两台 6020 的电机 ID（拨码设定）。C 板 CAN1 的 ID 分组为 1~4 / 5~8：
// 同组共用一个 8 字节控制帧，跨组会各发一帧（代码里已处理）。
inline constexpr uint8_t kMotorIdA = 1;  // A 电机：与 C 板 1:1
inline constexpr uint8_t kMotorIdB = 2;  // B 电机：比例由左拨杆决定

// ------------------- 联动比例（左拨杆档位 -> k）-------------------
// (thetaB - thetaA) = k * (thetaA - thetaC)，thetaC 为 C 板 yaw
inline constexpr float kRatioDown = 0.5f;  // 下档 1 : 0.5
inline constexpr float kRatioMid = -1.0f;  // 中档 1 : -1（负号 = 反向）
inline constexpr float kRatioUp = 3.0f;    // 上档 1 : 3

inline float RatioOf(sp::DBusSwitchMode sw)
{
  switch (sw) {
    case sp::DBusSwitchMode::DOWN:
      return kRatioDown;
    case sp::DBusSwitchMode::MID:
      return kRatioMid;
    default:
      return kRatioUp;
  }
}

// ------------------- 共享对象（工程级）-------------------
// 注意：remote / imu 是全局命名空间里的对象（定义在 uart_task.cpp / imu_task.cpp），
// 它们的 extern 声明写在文件末尾、namespace sp_app 之外，避免变成 sp_app::remote 找不到定义。

inline sp::CAN can1(&hcan1);
inline sp::RM_Motor motor_a(kMotorIdA, sp::RM_Motors::GM6020);
inline sp::RM_Motor motor_b(kMotorIdB, sp::RM_Motors::GM6020);

}  // namespace sp_app

// ------------------- 共享对象（全局命名空间）-------------------
// 这两个对象分别定义在 applications/uart_task.cpp 与 applications/imu_task.cpp 里，
// 这里只做声明，让联动任务共用同一个实例。
// 必须放在 namespace sp_app 之外：它们本身就是全局对象。
extern sp::DBus remote;
extern sp::Mahony imu;

#endif  // APPLICATIONS_LINKAGE_HPP
