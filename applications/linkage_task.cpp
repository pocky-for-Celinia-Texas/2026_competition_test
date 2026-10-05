#include "cmsis_os.h"

#include <cmath>

#include "attitude.hpp"
#include "linkage.hpp"
#include "tools/pid/pid.hpp"

// ============================================================================
// 4.4.1.2 姿态与电机联动
//
// 右拨杆（remote.sw_r）：下档 = 失能（两台电机无力）
//                        中档 = 姿态联动
//                        上档 = 复位（两台电机的指向标箭头对齐 C 板箭头）
// 左拨杆（remote.sw_l）决定 B 电机比例：下档 1:0.5 / 中档 1:-1 / 上档 1:3
//
// 数学模型（详见同目录《设计说明_姿态联动.md》第二节）：
//
//     thetaA = c1 + (psi - psi_ref)          A 与 C 板 1:1
//     thetaB = c2 + k * (psi - psi_ref)      B 与 C 板 1:k（应用里 k 含负号即反向）
//
// 其中 psi 是 C 板 imu 的 yaw（已连续化），c1/c2 是参考姿态下两台电机的角度。
// 两个目标都只与 psi 有关、彼此解耦，因此：
//   · C 板转 dpsi  ->  A 转 dpsi，B 转 k*dpsi            （题目要求 1、4）
//   · 手动转 A 角度 d（C 板不动）：
//       c1 变成 c1+d，由 thetaB - thetaA = k(thetaA - psi) 得 c2 变成 c2+k*d
//       -> B 跟 k*d，且轴线关系 thetaB - thetaA = k(thetaA - psi) 始终成立   （题目要求 2）
//   · 手动转 B 角度 d（C 板不动）：A 跟 d/k（|k|≠1 时），即 1/k 比例               （题目要求 2）
//   · 手动结束后 c1/c2 就停在新的数值上，不会回到原零点                        （题目要求 3）
//
// ★ 为什么不用"两个目标同步平移 dpsi"的写法：那样在 k = -1 时会丢掉 1:-1 的耦合
//   （dpsi 平移无法同时满足 thetaB-thetaA = k(thetaA-psi)），本写法对任意 k 都成立。
//   同时 k = -1 时矩阵奇异（det = 1+k = 0），"手动转 B 让 A 跟随"在数学上无解，
//   代码里在这种情况下保持 A 不动，见 Update() 里的说明。
// ============================================================================

namespace
{
// imu / remote 是全局对象（分别定义在 imu_task.cpp、uart_task.cpp），这里显式声明可见
using ::imu;
using ::remote;

// ------------------------- 控制参数 -------------------------
constexpr uint32_t kPeriodMs = 1;  // 与 imu_task 同为 1 ms
constexpr float kDt = kPeriodMs * 1e-3f;

// 位置环（rad -> rad/s）与速度环（rad/s -> N·m）串级，输出转矩
constexpr float kPosKp = 25.0f;      // 先小后大：抖/叫就减半，跟不上就加倍
constexpr float kPosMaxOut = 12.0f;  // 限速 rad/s

constexpr float kSpdKp = 1.2f;
constexpr float kSpdKi = 0.3f;
constexpr float kSpdMaxOut = 1.5f;   // 转矩上限 N·m（6020 额定 0.741*3 ≈ 2.22 N·m）
constexpr float kSpdMaxIOut = 0.5f;

// ------------------------- 手动转动检测 -------------------------
// 位置误差超过阈值 -> 认为人在掰电机：这一帧不出力，并按当前角度更新参考姿态。
// 进/出用两个阈值（迟滞），避免在阈值附近来回抖。
constexpr float kManualEnterRad = 0.35f;  // ≈ 20°
constexpr float kManualExitRad = 0.10f;   // ≈ 5.7°

// 位置串级控制：目标角度（rad，连续）-> 转矩（N·m）
class Joint
{
public:
  Joint()
  : pos_pid_(kDt, kPosKp, 0.0f, 0.0f, kPosMaxOut, 0.0f, 1.0f),
    spd_pid_(kDt, kSpdKp, kSpdKi, 0.0f, kSpdMaxOut, kSpdMaxIOut, 1.0f)
  {
  }

  float Update(sp::RM_Motor & motor, float target)
  {
    pos_pid_.calc(target, motor.angle);        // 位置误差 -> 期望角速度
    spd_pid_.calc(pos_pid_.out, motor.speed);  // 角速度误差 -> 转矩
    return spd_pid_.out;
  }

  void Reset()
  {
    pos_pid_.clear();
    spd_pid_.clear();
  }

private:
  sp::PID pos_pid_;
  sp::PID spd_pid_;
};

Joint joint_a;
Joint joint_b;

class Linkage
{
public:
  // 上电：以"两台电机角度的中点 = C 板 yaw"作为初始参考姿态
  void Init()
  {
    const float mid = 0.5f * (sp_app::motor_a.angle + sp_app::motor_b.angle);
    sp_app::yaw_unwrapper.Reset(imu.yaw);
    psi_ref_ = imu.yaw;
    SetRef(mid, mid);
    manual_ = false;
    reset_latched_ = false;
  }

  void Update()
  {
    const auto right = remote.sw_r;

    // ---------------- 右拨杆下档：失能 ----------------
    if (right == sp::DBusSwitchMode::DOWN) {
      ResetLatches();
      joint_a.Reset();
      joint_b.Reset();
      sp_app::motor_a.cmd(0.0f);
      sp_app::motor_b.cmd(0.0f);
      Send();
      return;
    }

    // 左拨杆换档：k 一变，映射立刻改变。用"保持两台电机当前角度不动"的参考姿态重建，
    // 这样不会因为换档产生目标跳变（换档后 C 板再转动即按新比例联动）。
    const float k = sp_app::RatioOf(remote.sw_l);
    if (k != k_) {
      k_ = k;
      RebaseToCurrent();
    }

    // ---------------- 右拨杆上档：复位 ----------------
    // 复位要先于"推进 yaw"处理，这样复位瞬间用的就是当前姿态（也便于失能/复位时冻结解缠）
    if (right == sp::DBusSwitchMode::UP) {
      manual_ = false;

      const float psi = sp_app::yaw_unwrapper.Update(imu.yaw);

      // 复位只在"刚拨到上档"那一刻执行一次：之后保持，避免把电机顶在目标上一直用力。
      if (!reset_latched_) {
        reset_latched_ = true;

        // 复位姿态：两台电机都转到 C 板当前 yaw（thetaA = thetaB = psi），
        // 并把这一姿态记为新的参考零点。由 thetaA=c1、thetaB=c2 得 c1=c2=psi。
        psi_ref_ = psi;
        SetRef(psi, psi);
        joint_a.Reset();
        joint_b.Reset();
      }

      Command(psi);
      return;
    }

    reset_latched_ = false;

    // ---------------- 右拨杆中档：姿态联动 ----------------
    // 判断顺序很重要：先用"上一帧的 yaw 估计"算误差决定 manual_，
    // 再推进 yaw 估计并输出。否则人会掰不动电机（目标跟着他跑），
    // 而且"先 Update 再 Reset"会让解缠状态和 psi_ref 不一致、注入一个假跳变。
    const float psi_prev = sp_app::YawEstimate();
    const float err_a = std::fabs(TargetA(psi_prev) - sp_app::motor_a.angle);
    const float err_b = std::fabs(TargetB(psi_prev) - sp_app::motor_b.angle);

    if (manual_) {
      if (err_a < kManualExitRad && err_b < kManualExitRad) manual_ = false;
    }
    else if (err_a > kManualEnterRad || err_b > kManualEnterRad) {
      manual_ = true;
    }

    if (manual_) {
      // 人在掰电机：冻结 yaw 解缠（不推进），把参考姿态平移到当前姿态：
      //     psi_ref = 当前 yaw,   thetaA_target = thetaA_meas,  thetaB_target = thetaB_meas
      // 两条轴线关系在 psi_ref = psi 时退化为 thetaA = c1、thetaB = c2，联立可得：
      //   · 手动转 A 角度 d -> c1 = thetaA_meas，且 c2 = (1+k)*c1  ->  B 跟 k*d（方向由 k 符号决定）
      //   · 手动转 B 角度 d -> c2 = thetaB_meas，且 c1 = c2/(1+k)  ->  A 跟 d/k（|k|≠1）
      //   · 松手后停在新的 c1/c2 上，不回正
      const float psi = sp_app::YawEstimate();  // 与上一帧一致（未推进）
      psi_ref_ = psi;

      const float c1 = sp_app::motor_a.angle;
      const float c2 = sp_app::motor_b.angle;

      if (std::fabs(1.0f + k_) > 1e-3f) {
        // 哪台电机被人掰动，就以哪台的测量值为准（避免另一台的噪声把参考拽跑）
        const bool a_moved = std::fabs(c1 - a_ref_) > std::fabs(c2 - b_ref_);
        if (a_moved)
          SetRef(c1, (1.0f + k_) * c1);
        else {
          const float c1_from_b = c2 / (1.0f + k_);
          SetRef(c1_from_b, (1.0f + k_) * c1_from_b);
        }
      }
      else {
        // k = -1（1+k = 0，矩阵奇异）：两条关系线性相关，只能确定 c2 = -c1 这一条，
        // "手动转 B 让 A 跟随"在数学上无解 -> 保持 A 的参考按实测更新（A 若被掰则就地保持）。
        a_ref_ = c1;
      }

      sp_app::motor_a.cmd(0.0f);
      sp_app::motor_b.cmd(0.0f);
      Send();
      return;
    }

    Command(sp_app::yaw_unwrapper.Update(imu.yaw));
  }

private:
  float TargetA(float psi) const { return a_ref_ + (psi - psi_ref_); }
  float TargetB(float psi) const { return b_ref_ + k_ * (psi - psi_ref_); }

  void SetRef(float c1, float c2)
  {
    a_ref_ = c1;
    b_ref_ = c2;
  }

  // 重建参考姿态：保持两台电机当前角度不动（目标不变），psi_ref 取当前 yaw
  //
  // ★ psi_ref_ 必须与 TargetA/TargetB 的 psi 用同一个坐标系（连续 yaw）。
  //   这里曾写成 imu.yaw（±π 缠绕值），而 Command() 传入的是 Update() 解缠后的连续值，
  //   一旦 C 板 yaw 累计转过 ±180°，psi - psi_ref_ 就会整体偏 2π 的整数倍：
  //     · 换档瞬间两台电机目标各跳一整圈；
  //     · 更糟的是手动检测用 TargetA(psi_prev) - motor_a.angle 算误差，
  //       那个 2π 常量偏置会让误差每帧都超过 kManualEnterRad，电机被永久判定为"人在掰"、
  //       一直输出 0 转矩而完全不响应遥控。
  //   改用 YawEstimate()（只读不推进，与 Update() 里的 psi 同源）后两个坐标系才统一。
  void RebaseToCurrent()
  {
    psi_ref_ = sp_app::YawEstimate();
    SetRef(sp_app::motor_a.angle, sp_app::motor_b.angle);
  }

  void ResetLatches()
  {
    manual_ = false;
    reset_latched_ = false;
  }

  void Command(float psi)
  {
    // 反馈丢失保护：100 ms 内没有反馈就不输出转矩，避免失控
    if (!sp_app::motor_a.is_alive(osKernelSysTick()) ||
        !sp_app::motor_b.is_alive(osKernelSysTick())) {
      sp_app::motor_a.cmd(0.0f);
      sp_app::motor_b.cmd(0.0f);
      Send();
      return;
    }

    sp_app::motor_a.cmd(joint_a.Update(sp_app::motor_a, TargetA(psi)));
    sp_app::motor_b.cmd(joint_b.Update(sp_app::motor_b, TargetB(psi)));
    Send();
  }

  void Send()
  {
    sp_app::motor_a.write(sp_app::can1.tx_data);
    sp_app::motor_b.write(sp_app::can1.tx_data);  // 同组 ID 共用一个 8 字节帧
    sp_app::can1.send(sp_app::motor_a.tx_id);

    if (sp_app::motor_b.tx_id != sp_app::motor_a.tx_id) {
      sp_app::motor_a.write(sp_app::can1.tx_data);
      sp_app::motor_b.write(sp_app::can1.tx_data);
      sp_app::can1.send(sp_app::motor_b.tx_id);
    }
  }

  float k_ = sp_app::kRatioDown;   // 当前生效的比例（左拨杆）
  float psi_ref_ = 0.0f;           // 参考姿态下的 C 板 yaw
  float a_ref_ = 0.0f;             // 参考姿态下的 A 电机角度 c1
  float b_ref_ = 0.0f;             // 参考姿态下的 B 电机角度 c2
  bool manual_ = false;            // 正在被人手动转动
  bool reset_latched_ = false;     // 上档复位只执行一次
};

Linkage linkage;
}  // namespace

extern "C" void linkage_task()
{
  sp_app::can1.config();
  sp_app::can1.start();

  // 等两台电机都有反馈，再确定初始参考姿态
  while (!sp_app::motor_a.is_open() || !sp_app::motor_b.is_open()) {
    osDelay(1);
  }

  linkage.Init();

  // 上电先把两台电机置于失能（右拨杆不在下档时才出力）
  sp_app::motor_a.cmd(0.0f);
  sp_app::motor_b.cmd(0.0f);

  while (true) {
    linkage.Update();
    osDelay(kPeriodMs);
  }
}

// CAN1 接收中断：把两台 6020 的反馈分发到各自的电机对象
extern "C" void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef * hcan)
{
  const uint32_t stamp_ms = osKernelSysTick();

  while (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0) > 0) {
    if (hcan == &hcan1) {
      sp_app::can1.recv();

      if (sp_app::can1.rx_id == sp_app::motor_a.rx_id)
        sp_app::motor_a.read(sp_app::can1.rx_data, stamp_ms);
      if (sp_app::can1.rx_id == sp_app::motor_b.rx_id)
        sp_app::motor_b.read(sp_app::can1.rx_data, stamp_ms);
    }
  }
}
