# Unitree_H7_Dual_RS485

基于 STM32H723VGTx 的双路 RS485 关节电机驱动工程，通过两路 4 Mbit/s 半双工 RS485 总线控制 4 个宇树 **GO-M8010-6** 关节电机（轮腿机器人），电机 ID 为 0~3。

- 控制器：STM32H723VGTx（Cortex-M7）
- 开发环境：Keil MDK（`MDK-ARM/Unitree_H7_Dual_RS485.uvprojx`），CubeMX 工程 `Unitree_H7_Dual_RS485.ioc`
- 通信协议：宇树 GO-M8010-6 串口协议（`BSP/Unitree/`）

## 硬件资源分配

| 功能 | 外设 | 引脚 | 说明 |
|---|---|---|---|
| RS485 总线 1 | USART2 | PD5 TX / PD6 RX / PD4 DE | 4 Mbit/s，硬件 DE，DMA1_Stream0 接收 |
| RS485 总线 2 | USART3 | PD8 TX / PD9 RX / PB14 DE | 4 Mbit/s，硬件 DE，DMA1_Stream1 接收 |
| 调试串口 | USART1 | PA9 TX / PA10 RX | 921600 8N1 |
| 电机电源 1 | GPIO | PC13 (`POWER_OUT1`) | 高电平打开 |
| 电机电源 2 | GPIO | PC14 (`POWER_OUT2`) | 高电平打开 |
| 5V 电源 | GPIO | PC15 (`POWER_5V`) | 上电即置高 |

电机与总线、ID 的对应关系：

```
USART2 ── 电机 ID 0
       └─ 电机 ID 1
USART3 ── 电机 ID 2
       └─ 电机 ID 3
```

## 目录结构

```
Core/
  Inc/ , Src/             CubeMX 生成的 HAL 初始化代码（main.c、usart.c、dma.c ...）
BSP/
  Unitree/                宇树官方通信协议（gom_protocol.c/h、crc_ccitt.c/h）
  Motor/                  电机任务：DMA 接收 + 反馈解析
    motor_task.c/h
MDK-ARM/                  Keil MDK 工程，编译输出在 Unitree_H7_Dual_RS485/
Drivers/                  STM32H7 HAL 驱动与 CMSIS
```

## 通信协议速查

| 方向 | 长度 | 帧格式 |
|---|---|---|
| 主机 → 电机（控制帧） | 17 Byte | `FE EE` + mode(1B) + 期望量(12B) + CRC16(2B) |
| 电机 → 主机（反馈帧） | 16 Byte | `FD EE` + mode(1B) + 反馈(11B) + CRC16(2B) |

- **mode**：低 4 位为电机 ID（15 = 广播），3 位为工作模式（0 锁定/阻尼、1 FOC 闭环、2 编码器标定）。
- **控制量**：`tor_des`(q8)、`spd_des`(q8)、`pos_des`(q15)、`k_pos`(q15)、`k_spd`(q15)。
- **反馈量**：`torque`(q8)、`speed`(q8)、`pos`(q15)、温度、错误码 `MError`、足端力传感器。
- 用户侧只需操作物理量（N·m、rad/s、rad），单位换算/打包/CRC 由协议层完成：
  - `modify_data()`：`MotorCmd_t` → 17 Byte 发送帧；
  - `extract_data()`：16 Byte 接收帧 → `MotorData_t`。

## 软件结构

### 电机任务（`BSP/Motor/motor_task.c`）

接收链路：

1. `MotorTask_Init()` 初始化 4 个电机结构体（ID 0~3、FOC 闭环、指令量清零），并分别对 USART2 / USART3 调用 `HAL_UARTEx_ReceiveToIdle_DMA()`，**按固定 16 Byte 长度**开启接收（DMA 为 Normal 模式）；
2. 串口空闲/接收完成进入 `HAL_UARTEx_RxEventCallback()`，先判断是哪路串口，再检查：
   - 本次收到的字节数必须是 16；
   - 包头必须是 `0xFD 0xEE`；
   - 帧内 ID 必须属于该串口（USART2：0/1，USART3：2/3）；
3. 校验通过后调用宇树官方解包函数 `extract_data()`（再校验 CRC，并换算成 `Pos/W/T/Temp/MError` 等物理量）；
4. Normal 模式 DMA 收完即停，回调里立即重新 arm 接收。

串口出错时经 `HAL_UART_ErrorCallback()` 清 ORE 标志并重启接收。

### 对外接口（`motor_task.h`）

```c
void         MotorTask_Init(void);           /* 初始化电机 + 开启两路 DMA 接收 */
MotorData_t *MotorTask_GetData(uint8_t id);  /* 读取电机反馈（id: 0~3） */
MotorCmd_t  *MotorTask_GetCmd(uint8_t id);   /* 取得指令结构体（发送部分使用） */
```

读取反馈示例：

```c
MotorData_t *m = MotorTask_GetData(0);
if (m->correct) {
    /* m->Pos (rad)、m->W (rad/s)、m->T (N.m)、m->Temp (℃)、m->MError */
}
```

### 当前进度

- [x] 双路 RS485 DMA 接收、帧校验、官方解包
- [ ] 控制帧发送（`modify_data()` + `HAL_UART_Transmit` / DMA 发送）
- [ ] 控制循环（轮腿机器人所需的状态机 / PD 控制）

## 编译与烧录

1. 用 Keil MDK 打开 `MDK-ARM/Unitree_H7_Dual_RS485.uvprojx`，`Build`（F7）；
2. 输出在 `MDK-ARM/Unitree_H7_Dual_RS485/`：`.axf`（调试/下载）、`.hex`（烧录）；
3. 也可在 VS Code 中用 Keil Assistant 插件编译下载。

## 使用注意事项

- 示例工程建议单电机 **1000 Hz** 收发频率，总线最高轮询频率 3000 Hz，过高会影响电机正常工作。
- 电机 `MError != 0` 表示进入错误状态，电机自动切到阻尼模式且不再响应运动指令；机器人系统应据此做保护处理（例如让其余电机进入阻尼状态）。
- 修改 CubeMX 配置时，自定义代码都在 `USER CODE BEGIN/END` 区间或 `BSP/` 目录内，不会被覆盖。
- 仓库忽略 Keil 编译中间产物（`.o/.d/.crf/...`）与 `.vscode/`，保留最终 `.axf/.hex` 输出，详见 `.gitignore`。
