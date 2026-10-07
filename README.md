# 项目名称 2026_competition_test

同济大学 SuperPower 战队 2026 赛季 C 板（STM32F407IGH6）电控工程第一阶段任务要求代码。

当前实现的任务全部挂在 FreeRTOS 下，用一个 DT7/DR16 遥控器同时驱动：LED 灯效、开机音乐、BMI088 姿态解算、
上位机波形，以及 ** 姿态与电机联动**。

---

## 1. 主要任务

| 编号 | 任务 | 实现文件 | 说明 |
| --- | --- | --- | --- |
| 1 | LED 灯效 | [led_task.cpp](applications/led_task.cpp) | 50 Hz 刷新，红→绿→蓝循环渐变，每段开头 40 ms 白闪做心跳 |
| 2 | 开机音乐 | [buzzer_task.cpp](applications/buzzer_task.cpp) | 音名频率表 + `{频率, 时长}` 曲谱，开机播《欢乐颂》后挂起 |
| 3 | IMU 姿态解算 | [imu_task.cpp](applications/imu_task.cpp) | 1 kHz：BMI088 读值 → Mahony 四元数/欧拉角 → 6个波形输出；含加热 PID （尚未实测暂不可用）|
| 4 | 遥控器 DBUS | [uart_task.cpp](applications/uart_task.cpp) | USART3 + DMA + 空闲中断收 18 字节，解析摇杆/拨杆/键鼠 |
| 5 | **姿态与电机联动** | [linkage_task.cpp](applications/linkage_task.cpp)<br>[linkage.hpp](applications/linkage.hpp)<br>[attitude.hpp](applications/attitude.hpp) | 1 kHz 串级 PID 控两台 GM6020 电机，按 C 板 yaw 比例联动，支持手动掰动跟随与复位对齐 |

### 5.1 姿态联动契约

```
thetaA = c1 + (psi - psi_ref)         A 与 C 板 1:1
thetaB = c2 + k * (psi - psi_ref)     B 与 C 板 1:k
```

不变量 `thetaB - k*thetaA = 常数`，由此同时满足题目的四条要求：C 板转 `dpsi` 时 A 转 `dpsi`、B 转 `k*dpsi`；
手动掰 A 转 `d` 则 B 跟 `k*d`（掰 B 则 A 跟 `d/k`）；松手后停在新的参考角，不回正。

右拨杆 `sw_r`：**下档 = 失能（0 转矩）**、**中档 = 姿态联动**、**上档 = 复位**（两臂箭头方向对齐 C 板R标，停在上档期间持续跟随）；
左拨杆 `sw_l` 选比例 `k`：下档 `0.5`、中档 `-1`（反向）、上档 `3`（见 [linkage.hpp](applications/linkage.hpp)）。

### 1.2 目录

```
applications/          本工程的应用层（任务 + 参数，作业主要改动区）
Core/                  CubeMX 生成的 HAL 初始化、中断、FreeRTOS 任务创建
Drivers/               STM32F4 HAL + CMSIS（勿改）
Middlewares/           FreeRTOS 内核（勿改）
sp_middleware/         战队中间件 submodule，命名空间 sp::
cmake/ stm32cubemx/    工具链与 CubeMX 源文件清单
build/Debug/           构建产物（已 gitignore）
```

---

## 2. 硬件约定（C 板）

| 功能 | 外设 / 引脚 | 参数 |
| --- | --- | --- |
| LED | TIM5 CH1/2/3 → PH10/PH11/PH12 | ARR=65535，软件 PWM 调光 |
| 蜂鸣器 | TIM4_CH3 → PD14 | 定时器时钟 84 MHz |
| BMI088 | SPI1，CSB1=PA4(加速度计)、CSB2=PB0(陀螺仪) | 10.5 Mbit/s，模式 3 |
| IMU 加热（默认关闭） | TIM10_CH1 → PF6 | 计数 1 MHz，满量程 5000 → 200 Hz PWM |
| 上位机波形 | USART1 → PA9/PA10 + DMA2_Stream7 | 921600 8N1，帧 `0xAA 0xBB` + 长度 + float32 小端 |
| 遥控器 DBUS | USART3_RX = PC11（经板上反相器 Q10） | 100 kbps，偶校验，DMA1_Stream1 + 空闲中断 |
| 电机 CAN | CAN1 → PD0/PD1，`CAN_MODE_NORMAL` | 1 Mbps（Prescaler=3 / BS1=10TQ / BS2=3TQ） |
| 电机 ID | A = `1`，B = `2`（拨码设定） | GM6020，转矩控制，`±0.3 N·m` |

系统时钟：HSE 12 MHz → PLL → **168 MHz**，APB1 = 42 MHz、APB2 = 84 MHz，HAL 时基用 TIM14。

FreeRTOS 任务（[freertos.c](Core/Src/freertos.c)）：

| 任务 | 优先级 | 栈 | 周期 |
| --- | --- | --- | --- |
| uarttask | AboveNormal | 512 | 10 ms（收帧靠中断） |
| imutask | Normal | 512 | 1 ms |
| linkagetask | Normal | 512 | 1 ms |
| defaultTask | Normal | 128 | 1 ms（空转） |
| LEDTask / buzzertask | Low | 128 | 20 ms / 事件式 |


---

## 3. 操作注意事项（重要）

### 3.1 上电前

1. **先把两电机R标掰到与 C 板R标方向一致再上电。** 复位功能的安装偏移 `off_a_/off_b_` 是在
   `Linkage::Init()` 里按「上电姿态即对齐姿态」记录的（见 [linkage_task.cpp:159-163](applications/linkage_task.cpp#L159-L163)）。
   同时 yaw 零点由 Mahony 上电时的朝向决定，上电姿态不可重复则复位必然带固定偏差 —— 这种情况必须把
   `off_a_/off_b_` 换成人手实测的常量。
2. **接通电源前确认遥控器右拨杆不在中档/上档。** 上电后 `linkage_task` 会先等两台电机都有 CAN 反馈，
   再执行 `cmd(0.0f)` 置于失能；但只要右拨杆在中档就会立刻进入闭环出力。
3. 遥控器（DT7）先开机并与接收机配对（绿灯常亮），接收机 `DATA` 用一根 3-Pin 线接 C 板 DBUS 口（棕线在外）。
   **不要热插拔 DBUS 线**，误接/反接会经反相电路打进 USART3。

### 3.2 运行中

4. **调试连杆前先让机构空载、电机不装负载。** 参数是按 GM6020 额定转矩 `±0.3 N·m` 给的，
   带负载首次上电容易直接顶到机械限位。
5. 右拨杆下档 = 两台电机真的无力（自由状态），不是「刹车保持」；机构带重力负载时会自由掉落。
6. 左拨杆换档瞬间不会跳变目标（内部 `RebaseToCurrent` 重建参考姿态），但换档后 C 板再转动才按新比例联动。
7. 手动掰动检测依赖「C 板不动 + 电机在动 + 误差 > 0.35 rad」，连续 2 个 100 ms 窗口才确认；
   快速甩动 C 板时位置环滞后可能被误判，这也是里面对 `kPsiStillRad` / `kManualConfirm` 过滤的原因，
   调参时不要随手改小。
8. 任意一台电机超过 100 ms 没有反馈，联动任务会把两台电机转矩清零（`is_alive` 保护），
   现象就是「动一下停一下」，先查 CAN 接线与电机 ID 拨码。

### 3.3 改代码时

9. **新增 `.cpp` 必须同时改 [CMakeLists.txt](CMakeLists.txt) 的 `target_sources`**，
头文件统一用 `sp_middleware/` 作为包含根（`#include "io/led/led.hpp"`）。
10. **CubeMX 重新生成（`KeepUserCode=true`）只保留 `USER CODE BEGIN/END` 之间的内容。**
    新增任务必须用 `As external` 声明（如 `extern "C" void my_task()`），否则会被生成代码清掉；
    任务函数签名与 `cmsis_os` 的 `void const * argument` 形式不同也能链接通过（ARM ABI 下多余参数被忽略）。
11. 用 PID / Mahony 的任务，其 `kDt` 必须与任务周期严格一致：`imu_task` 与 `linkage_task` 都是
    `kPeriodMs = 1` 配 `kDt = 1e-3f`（见 [linkage_task.cpp:51-52](applications/linkage_task.cpp#L51-L52)），
    改了 `osDelay` 就要同步改 `kDt`，否则微分/积分项全都对不上。
12. `IMU_HEAT_ENABLE` 默认 **0**（见 [imu_task.cpp:10](applications/imu_task.cpp#L10)）：没接加热电阻或 PF6 另有用途时保持 0；
    打开后过温保护阈值 60 ℃、PID 输出限幅 0.4 占空比，此功能尚未调试过，暂不可用。
13. **一个 UART 只能有一个 `sp::Plotter`。** 目前只有 `imu_task.cpp` 用了 `&huart1`；
    若再加电机波形任务，必须合并到同一处发送，否则 DMA 发送会互相踩。
14. 曲谱表 `kMusic` 里 `NOTE_REST = 0` 只用于占时间，**绝不能传给 `buzzer.set()`**（会除零）。
15. BMI088 的坐标系矩阵 `kRAb` 写在 [imu_task.cpp:25](applications/imu_task.cpp#L25)：
    三轴符号不对时**只改这个矩阵，不要动驱动**。`bmi088.gyro` 是原始值，
    姿态解算必须传 `imu.update(bmi088.acc, bmi088.gyro)`，注意别和已处理过的量混用。
16. yaw 要做「超过 ±180°」的连续化处理（[attitude.hpp](applications/attitude.hpp) 的 `YawUnwrapper`）：
    `k = 3` 时 C 板转 60° 电机就要转 180°，直接用 `imu.yaw` 会在 ±π 处整圈跳变。
    任何新代码里 `psi` 都必须与 `TargetA/TargetB` 用同一个连续坐标系。
17. `build/` 已在 [.gitignore](.gitignore) 中；`compile_commands.json` 生成在 `build/Debug/` 下供 clangd 使用。
    格式化配置见 [.clang-format](.clang-format)，保存时自动格式化（C 文件除外，见 [settings.json](.vscode/settings.json)）。

---

## 4. 相关文档

- 中间件说明：[sp_middleware/readme.md](sp_middleware/readme.md)   [原地址](https://github.com/TongjiSuperPower/sp_middleware.git)
- 代码中带 ★ 的注释段落记录了姿态联动的数学模型与实现取舍，改联动逻辑前建议通读
  [linkage_task.cpp](applications/linkage_task.cpp) 顶部注释块。
