#include "cmsis_os.h"
#include "io/dbus/dbus.hpp"

// 4. 遥控器控制：C 板 + DT7 遥控器 / DR16 接收机（DBUS）
//
// C 板上执行这条链路的部分（详见作业目录 README_遥控器DBUS_现有工程.md）：
//   DBUS 接口(Pin-C8) → 板上反相电路 Q10(PMBT3904)/R109 → USART3_RX = PC11
//   → USART3（100 kbps，8 数据位 + 偶校验 + 1 停止位）→ DMA1 Stream1/Channel4
//   → 空闲中断(IDLE) → HAL_UARTEx_RxEventCallback → sp::DBus → 本任务/其它任务读 remote
//
// 硬件只需一根 3-Pin 线：DR16 接收机 DATA → C 板 DBUS 接口，DT7 开机并与接收机配对（绿灯常亮）。

// C板：DBUS 经板上反相器接到 USART3
sp::DBus remote(&huart3);

// 达妙板：sp::DBus remote(&huart5, false);

extern "C" void uart_task()
{
  remote.request();  // 启动第一次「最多 18 字节 + 空闲中断」接收

  while (true) {
    // 使用调试(f5)查看 remote 内部变量的变化：
    //   ch_rh/ch_rv/ch_lh/ch_lv：右水平/右垂直/左水平/左垂直摇杆，[-1, 1]
    //   ch_lu：拨轮（数据链路上是鼠标 Z 轴）
    //   sw_r/sw_l：右/左三位开关，UP / MID / DOWN
    //   mouse.vx/vy/vs、mouse.left/right、keys.*：键鼠
    //   is_open()：是否收到过帧；is_alive(osKernelSysTick())：100 ms 内是否有新帧
    osDelay(10);
  }
}

// 一帧收完（空闲中断）或出错时都会被调用：解析 + 重新武装接收
extern "C" void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef * huart, uint16_t Size)
{
  auto stamp_ms = osKernelSysTick();

  if (huart == &huart3) {
    remote.update(Size, stamp_ms);
    remote.request();
  }
}

extern "C" void HAL_UART_ErrorCallback(UART_HandleTypeDef * huart)
{
  if (huart == &huart3) {
    remote.request();
  }
}