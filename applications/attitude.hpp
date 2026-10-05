#ifndef APPLICATIONS_ATTITUDE_HPP
#define APPLICATIONS_ATTITUDE_HPP

// 姿态数据工具：把 Mahony 输出的 yaw（±π）展成连续角度
//
// 联动任务需要"能超过 ±180° 的 yaw"：中档 k=3 时 C 板转 60° 电机要转 180°，
// 再多转一点 yaw 就会从 +π 跳到 -π，直接用会让电机目标瞬间反向甩一大圈。
//
// imu 对象本身定义在 applications/imu_task.cpp 里，声明放在 applications/linkage.hpp。

#include <cmath>

namespace sp_app
{

constexpr float PI = 3.14159265358979f;
constexpr float TWO_PI = 2.0f * PI;

// 把任意角折算到 (-π, π]
inline float WrapToPi(float rad)
{
  while (rad > PI) rad -= TWO_PI;
  while (rad <= -PI) rad += TWO_PI;
  return rad;
}

// 逐帧解缠：跳变超过 π 就记一圈
class YawUnwrapper
{
public:
  float Update(float yaw_now)
  {
    if (!inited_) {
      inited_ = true;
      last_ = yaw_now;
      current_ = yaw_now;
      return current_;
    }

    const float d = yaw_now - last_;
    if (d > PI)
      turns_ -= TWO_PI;
    else if (d < -PI)
      turns_ += TWO_PI;
    last_ = yaw_now;
    current_ = yaw_now + turns_;

    return current_;
  }

  // 读当前连续 yaw（不推进）。用于"先判断是否被人掰动、再决定推不推进"的顺序。
  float Current() const { return current_; }

  // 复位/重新对零时调用：以当前 yaw 作为新的连续起点
  void Reset(float yaw_now)
  {
    inited_ = true;
    last_ = yaw_now;
    turns_ = 0.0f;
    current_ = yaw_now;
  }

private:
  bool inited_ = false;
  float last_ = 0.0f;
  float turns_ = 0.0f;
  float current_ = 0.0f;
};

inline YawUnwrapper yaw_unwrapper;

// 读当前连续 yaw（不推进）
inline float YawEstimate() { return yaw_unwrapper.Current(); }

}  // namespace sp_app

#endif  // APPLICATIONS_ATTITUDE_HPP
