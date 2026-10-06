#include <cmath>

#include "attitude.hpp"
#include "cmsis_os.h"
#include "linkage.hpp"
#include "tools/pid/pid.hpp"

// ============================================================================
// 4.4.1.2 姿态与电机联动
//
// 右拨杆（remote.sw_r）：下档 = 失能（两台电机无力）
//                        中档 = 姿态联动
//                        上档 = 复位（两臂箭头方向对齐 C 板箭头；停在上档时持续跟随对齐）
// 左拨杆（remote.sw_l）决定 B 电机比例：下档 1:0.5 / 中档 1:-1 / 上档 1:3
//
// ★ 复位为什么不能直接把目标设成 psi：psi 的零点由 Mahony 上电时的朝向决定，
//   而 motor.angle 是电机编码器的绝对角度，两者相差一个固定的安装偏移 off
//   （对齐时 目标角 = psi + off）。off 在 Init() 里按"上电姿态即对齐姿态"记录，
//   所以上电前要把两臂箭头掰到与 C 板箭头方向一致。
//
// 数学模型（详见同目录《设计说明_姿态联动.md》第二节）：
//
//     thetaA = c1 + (psi - psi_ref)          A 与 C 板 1:1
//     thetaB = c2 + k * (psi - psi_ref)      B 与 C 板 1:k（应用里 k 含负号即反向）
//
// 其中 psi 是 C 板 imu 的 yaw（已连续化），c1/c2 是参考姿态下两台电机的角度。
//
// ★ 这组目标的不变量是：thetaB - k*thetaA = c2 - k*c1 = 常数（与 psi 无关）。
//   由它同时推出题目四条要求：
//   · C 板转 dpsi  ->  A 转 dpsi，B 转 k*dpsi                            （要求 1、4）
//   · 手动转 A 角度 d（C 板不动）：为保持不变量，B 必须转 k*d              （要求 2）
//   · 手动转 B 角度 d（C 板不动）：为保持不变量，A 必须转 d/k              （要求 2）
//   · 松手后 c1/c2 停在新的数值上，不回正                                （要求 3）
//
//   ★ 另一条常被写出来的关系 thetaB - thetaA = k(thetaA - psi) 与
//     "A 与 C 板 1:1、B 与 C 板 1:k" 是矛盾的：把 thetaA = psi + 常数代入右边
//     得到常数，而左边 = (k-1)*psi + 常数，要恒等必须 k = 1。
//     所以代码按上面的不变量实现。若题目原文写的确实是那条关系，需要把 TargetB
//     里的 k 换成 (1+k)（此时 B 相对 C 板是 1:(1+k)，手动掰 A 时 B 跟 (1+k)d）。
// ============================================================================

namespace
{
// imu / remote 是全局对象（分别定义在 imu_task.cpp、uart_task.cpp），这里显式声明可见
using ::imu;
using ::remote;

constexpr bool kNoRemote = false;  // 调试用：true 时忽略右拨杆，强制中档联动

// ------------------------- 控制参数 -------------------------
constexpr uint32_t kPeriodMs = 1;  // 与 imu_task 同为 1 ms
constexpr float kDt = kPeriodMs * 1e-3f;

// 位置环（rad -> rad/s）与速度环（rad/s -> N·m）串级，输出转矩
constexpr float kPosKp = 4.0f;      // 先小后大：抖/叫就减半，跟不上就加倍
constexpr float kPosMaxOut = 4.0f;  // 限速 rad/s

constexpr float kSpdKp = 0.02f;
constexpr float kSpdKi = 0.3f;
constexpr float kSpdMaxOut = 0.3f;  // 转矩上限 N·m（6020 额定 0.741*3 ≈ 2.22 N·m）
constexpr float kSpdMaxIOut = 0.1f;

// ------------------------- 摩擦补偿与保持 -------------------------
// 纯串级 PID 在静止/极低速时给不出克服静摩擦所需的力矩：位置环只能靠"留一点静差"
// 让 P 项凑出力来，所以复位总差那么一点点。对策分两段：
//   1) 接近目标途中：按误差方向补一个库仑摩擦前馈，帮它起转、补掉摩擦滞后；
//   2) 已到目标附近：把力矩"锁存"住（冻结速度环积分），不再让积分继续蓄力。
// ★ 第 2 条是关键：不锁存的话，积分在到位后会顶在饱和值上把机构推过目标，
//   误差反向后积分再退回来 —— 表现就是"临近目标点时的低频抖动"。
//   kFrictionTorque : 前馈满值(N·m)。太小 -> 复位留静差；太大 -> 到位前来回蹭。
//                     调法：从 0.03 起每次 +0.01 直到静差消失，开始蹭就回退一档。
//   kFrictionRampRad: 误差到这个大小前馈给满，再靠近就线性回收（避免"到点了还在推"）。
//   kHoldRad        : 进入"保持区"的误差窗口，也是复位精度的上限（0.01 rad ≈ 0.57°）。
//   kHoldSpeedRad   : 还要基本停住才算进入保持区（GM6020 转速反馈 1 rpm ≈ 0.105 rad/s，
//                     阈值要高于静止时的 rpm 噪声）。
constexpr float kFrictionTorque = 0.05f;     // N·m
constexpr float kFrictionRampRad = 0.03f;    // rad ≈ 1.7°
constexpr float kHoldRad = 0.01f;            // rad ≈ 0.57°
constexpr float kHoldSpeedRad = 0.2f;        // rad/s ≈ 2 rpm

// ------------------------- 手动转动检测 -------------------------
// 判据：C 板基本没动、却有一台电机在动、而且离目标很远 -> 人在掰电机。
// 只看"目标 - 实测"误差是不够的：把 C 板转快了位置环也会滞后出大误差；
// 那种情况由 kPsiStillRad（C 板没动）和"连续两个窗口"一起过滤掉。
constexpr uint32_t kSlowFrames = 100;     // 慢采样窗口：100 帧 = 100 ms
constexpr float kPsiStillRad = 0.02f;     // 100 ms 内 psi 变化 < 此值 => C 板没动
constexpr float kStillRad = 0.02f;        // 100 ms 内电机角变化 < 此值 => 电机没动
constexpr float kManualEnterRad = 0.35f;  // 目标与实测差 > 此值(≈20°) 才算被拉走
constexpr uint32_t kManualConfirm = 2;    // 连续 2 个窗口(200 ms)满足才判为人手

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
    const float err = target - motor.angle;

    pos_pid_.calc(target, motor.angle);        // 位置误差 -> 期望角速度
    spd_pid_.calc(pos_pid_.out, motor.speed);  // 角速度误差 -> 转矩

    // 摩擦前馈：满值 kFrictionTorque，误差小于 kFrictionRampRad 时线性回收
    float ramp = err / kFrictionRampRad;
    if (ramp > 1.0f) ramp = 1.0f;
    if (ramp < -1.0f) ramp = -1.0f;
    const float ff = kFrictionTorque * ramp;

    // 保持区：误差很小且基本停住 -> 锁存力矩、冻结速度环积分，不再蓄力（见文件顶部说明）
    const bool holding = std::fabs(err) < kHoldRad && std::fabs(motor.speed) < kHoldSpeedRad;

    float out;
    if (holding) {
      spd_pid_.data.iout = hold_torque_;  // 冻结积分，退出保持区时不会突变
      out = hold_torque_;
    }
    else {
      out = spd_pid_.out + ff;
      hold_torque_ = out;  // 记住"能推得动"的力矩，进入保持区时直接用它顶住
    }

    if (out > kSpdMaxOut) out = kSpdMaxOut;
    if (out < -kSpdMaxOut) out = -kSpdMaxOut;
    return out;
  }

  void Reset()
  {
    pos_pid_.clear();
    spd_pid_.clear();
    hold_torque_ = 0.0f;
  }

private:
  sp::PID pos_pid_;
  sp::PID spd_pid_;
  float hold_torque_ = 0.0f;  // 保持区锁存的力矩（N·m）
};

Joint joint_a;
Joint joint_b;

class Linkage
{
public:
  // 上电：以当前实测姿态为参考（首帧误差为 0，不会被误判成"人在掰"）
  void Init()
  {
    k_ = sp_app::RatioOf(remote.sw_l);
    sp_app::yaw_unwrapper.Reset(imu.yaw);
    psi_ref_ = imu.yaw;
    SetRef(sp_app::motor_a.angle, sp_app::motor_b.angle);

    // 复位用的"箭头对齐"偏移：两臂箭头与 C 板箭头方向一致时，目标角 = psi + off。
    // 这里把"上电瞬间的姿态"当作已经对齐的姿态，所以上电前要（电机未通电、可用手掰）
    // 把两臂箭头掰到与 C 板箭头方向一致。若上电姿态不可重复，把下面两行换成手测常量。
    off_a_ = a_ref_ - psi_ref_;
    off_b_ = b_ref_ - psi_ref_;

    manual_ = false;
    held_a_ = true;
    reset_latched_ = false;
    mid_active_ = false;
    manual_cnt_ = 0;
    slow_cnt_ = 0;
    a_slow_ = sp_app::motor_a.angle;
    b_slow_ = sp_app::motor_b.angle;
    psi_slow_ = psi_ref_;
  }

  void Update()
  {
    const auto right = kNoRemote ? sp::DBusSwitchMode::MID : remote.sw_r;

    // 解缠器每帧都推进：任何分支下 psi 的连续性与参考系都保持一致
    const float psi = sp_app::yaw_unwrapper.Update(imu.yaw);

    // ---------------- 右拨杆下档：失能 ----------------
    if (right == sp::DBusSwitchMode::DOWN) {
      manual_ = false;
      reset_latched_ = false;
      mid_active_ = false;
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
      RebaseToCurrent(psi);
    }

    // ---------------- 右拨杆上档：复位 ----------------
    if (right == sp::DBusSwitchMode::UP) {
      manual_ = false;
      mid_active_ = false;

      // 复位只在"刚拨到上档"那一刻清一次 PID 状态，避免换档冲击。
      if (!reset_latched_) {
        reset_latched_ = true;
        joint_a.Reset();
        joint_b.Reset();
      }

      // 复位姿态：两臂箭头方向与 C 板箭头一致 -> 目标角 = psi + off（off 见 Init()）。
      // 每帧都按当前 psi 重算，所以停在上档期间转动 C 板，两臂箭头始终跟着对齐。
      // ★ 不能写 SetRef(psi, psi)：psi 的零点由 Mahony 上电时的朝向决定，
      //   和电机编码器的零点差一个安装偏移 off，直接相等会让两臂偏掉 off。
      psi_ref_ = psi;
      SetRef(psi + off_a_, psi + off_b_);
      Command(psi_ref_);
      return;
    }

    reset_latched_ = false;

    // ---------------- 右拨杆中档：姿态联动 ----------------
    // 刚进入中档：以实测姿态为参考，避免切换瞬间目标跳变（也保证首帧误差为 0）
    if (!mid_active_) {
      mid_active_ = true;
      RebaseToCurrent(psi);
    }

    // 慢采样窗口（100 ms）：判断 C 板有没有动、电机有没有动
    bool win = false;
    float dA = 0.0f, dB = 0.0f, dpsi = 0.0f;
    if (++slow_cnt_ >= kSlowFrames) {
      slow_cnt_ = 0;
      win = true;
      dA = std::fabs(sp_app::motor_a.angle - a_slow_);
      dB = std::fabs(sp_app::motor_b.angle - b_slow_);
      dpsi = std::fabs(psi - psi_slow_);
      a_slow_ = sp_app::motor_a.angle;
      b_slow_ = sp_app::motor_b.angle;
      psi_slow_ = psi;
    }

    const float err_a = std::fabs(TargetA(psi) - sp_app::motor_a.angle);
    const float err_b = std::fabs(TargetB(psi) - sp_app::motor_b.angle);

    if (manual_) {
      // 松手判定：一个窗口内两台电机都几乎不动
      if (win && dA < kStillRad && dB < kStillRad) {
        manual_ = false;
        psi_ref_ = psi;  // 参考角保持不动，只把 yaw 参考挪到当前值
      }
    }
    else if (
      win && dpsi < kPsiStillRad && (dA > kStillRad || dB > kStillRad) &&
      (err_a > kManualEnterRad || err_b > kManualEnterRad)) {
      // C 板没动、电机却在动、而且离目标很远 -> 人在掰电机。
      // 连续两个窗口都满足才算，避免位置环追赶 C 板时的滞后被误判成手掰。
      if (++manual_cnt_ >= kManualConfirm) {
        manual_cnt_ = 0;
        manual_ = true;
        held_a_ = (dA >= dB);
        c_inv_ = b_ref_ - k_ * a_ref_;  // 锁定不变量 thetaB = k*thetaA + c_inv_
      }
    }
    else if (win) {
      manual_cnt_ = 0;
    }

    if (manual_) {
      // 人在掰电机：被掰的那台不出力（0 转矩），另一台照常按 k 倍关系跟过去
      if (
        !sp_app::motor_a.is_alive(osKernelSysTick()) ||
        !sp_app::motor_b.is_alive(osKernelSysTick())) {
        sp_app::motor_a.cmd(0.0f);
        sp_app::motor_b.cmd(0.0f);
        Send();
        return;
      }

      // psi_ref_ = psi 时 TargetA = a_ref_、TargetB = b_ref_，所以直接拿参考角当目标
      psi_ref_ = psi;
      if (held_a_) {
        a_ref_ = sp_app::motor_a.angle;  // 接受人的位置
        b_ref_ = k_ * a_ref_ + c_inv_;   // B 跟 k 倍
      }
      else {
        b_ref_ = sp_app::motor_b.angle;   // 接受人的位置
        a_ref_ = (b_ref_ - c_inv_) / k_;  // A 跟 1/k 倍（k = 0.5 / -1 / 3 都非零）
      }

      if (held_a_) {
        sp_app::motor_a.cmd(0.0f);  // 被掰的 A 自由
        sp_app::motor_b.cmd(joint_b.Update(sp_app::motor_b, b_ref_));
      }
      else {
        sp_app::motor_b.cmd(0.0f);  // 被掰的 B 自由
        sp_app::motor_a.cmd(joint_a.Update(sp_app::motor_a, a_ref_));
      }
      Send();
      return;
    }

    Command(psi);
  }

private:
  float TargetA(float psi) const { return a_ref_ + (psi - psi_ref_); }
  float TargetB(float psi) const { return b_ref_ + k_ * (psi - psi_ref_); }

  void SetRef(float c1, float c2)
  {
    a_ref_ = c1;
    b_ref_ = c2;
  }

  // 重建参考姿态：保持两台电机当前角度不动（目标不变），psi_ref 取当前连续 yaw。
  //
  // ★ psi_ref_ 必须与 TargetA/TargetB 的 psi 用同一个坐标系（连续 yaw）。这里传进来的
  //   psi 就是 Update() 顶部 Update(imu.yaw) 的结果，与 Command() 用的是同一个值。
  //   若改用 imu.yaw（±π 缠绕值），一旦 C 板 yaw 累计转过 ±180°，psi - psi_ref_ 会整体
  //   偏 2π 的整数倍：换档瞬间两台电机目标各跳一整圈，手动检测也会被恒定偏置打亮。
  void RebaseToCurrent(float psi)
  {
    psi_ref_ = psi;
    SetRef(sp_app::motor_a.angle, sp_app::motor_b.angle);

    // 同步慢采样窗口状态，避免刚重建参考就被当成"电机在动"
    psi_slow_ = psi;
    a_slow_ = sp_app::motor_a.angle;
    b_slow_ = sp_app::motor_b.angle;
    slow_cnt_ = 0;
    manual_cnt_ = 0;
  }

  void Command(float psi)
  {
    // 反馈丢失保护：100 ms 内没有反馈就不输出转矩，避免失控
    if (
      !sp_app::motor_a.is_alive(osKernelSysTick()) ||
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

  float k_ = sp_app::kRatioDown;  // 当前生效的比例（左拨杆）
  float psi_ref_ = 0.0f;          // 参考姿态下的 C 板 yaw
  float a_ref_ = 0.0f;            // 参考姿态下的 A 电机角度 c1
  float b_ref_ = 0.0f;            // 参考姿态下的 B 电机角度 c2
  float c_inv_ = 0.0f;            // 手动期间锁定的不变量：thetaB = k*thetaA + c_inv_
  float off_a_ = 0.0f;            // 箭头对齐偏移：复位时 A 的目标角 = psi + off_a_
  float off_b_ = 0.0f;            // 箭头对齐偏移：复位时 B 的目标角 = psi + off_b_
  bool manual_ = false;           // 正在被人手动转动
  bool held_a_ = true;            // 手动时被掰的是 A（否则是 B）
  bool reset_latched_ = false;    // 上档复位只执行一次
  bool mid_active_ = false;       // 是否已在中档（用于进入瞬间重建参考）
  uint32_t manual_cnt_ = 0;       // 连续满足手动条件的窗口数
  uint32_t slow_cnt_ = 0;         // 慢采样计数
  float a_slow_ = 0.0f;
  float b_slow_ = 0.0f;
  float psi_slow_ = 0.0f;
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
  sp_app::motor_a.cmd(0.0f);  /////
  sp_app::motor_b.cmd(0.0f);  ////

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
