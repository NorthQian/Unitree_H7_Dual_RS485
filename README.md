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

1. `MotorTask_Init()` 初始化 4 个电机结构体（ID 0~3、FOC 闭环、指令量清零），并分别对 USART2 / USART3 调用 `HAL_UARTEx_ReceiveToIdle_DMA()`，接收缓冲 64 Byte（DMA 为 Normal 模式）；
2. 串口空闲/接收完成进入 `HAL_UARTEx_RxEventCallback()`，先判断是哪路串口，再检查：
   - 本次收到的字节数必须是 16；
   - 包头必须是 `0xFD 0xEE`；
   - 帧内 ID 必须属于该串口（USART2：0/1，USART3：2/3）；
   - 任一不符直接丢弃；
3. 通过后调用宇树官方解包函数 `extract_data()`（校验 CRC，并换算成 `pos/vel/tor/temp/error`）；CRC 错则 `correct = 0` 且 `rx_bad++`；
4. Normal 模式 DMA 收完即停，回调里立即重新 arm 接收。

串口出错时经 `HAL_UART_ErrorCallback()` 清 ORE 标志并重启接收。

每个电机对应一个扁平结构体 `Unitree_Motor_t`（`motor[id]`），只存上层用得到的物理量：

```c
typedef struct
{
    uint8_t id;                          /* 电机 ID: 0 ~ 3 */

    float   pos_des, vel_des, tor_des;   /* 指令：rad, rad/s, N.m */
    float   kp, kd;                      /* 刚度 / 阻尼 */
    uint8_t mode;                        /* 0 阻尼 / 1 FOC */

    float   pos, vel, tor;               /* 反馈：输出轴侧，pos ∈ [0, 2π) */
    int8_t  temp;
    uint8_t error;                       /* MError */
    uint8_t correct;                     /* 1: 最近一帧反馈有效 */
    uint32_t rx_bad;                     /* CRC 错误累计 */
} Unitree_Motor_t;
```

调试时 watch `motor[0]` 展开即可看到全部信息。17/16 Byte 的原始帧不存进电机结构体，需要时看 `rx_usart2` / `rx_usart3`（最后收到的帧）或 `tx_cmd.motor_send_data`（最后发出的帧）。

### 对外接口（`motor_task.h`）

```c
void            MotorTask_Init(void);          /* 初始化电机 + 开启两路 DMA 接收 */
const Unitree_Motor_t  *MotorTask_GetMotor(uint8_t id);/* 读取电机状态（id: 0~3，只读） */

/* MIT 力位混合发送（阻塞 17 Byte，约 42.5 us/帧） */
HAL_StatusTypeDef MotorTask_SetMit(uint8_t id, float pos, float vel,
                                   float kp, float kd, float tq);
HAL_StatusTypeDef MotorTask_SendMit(uint8_t id, float pos, float vel,
                                    float kp, float kd, float tq);
HAL_StatusTypeDef MotorTask_SendAll(void);
```

读取反馈示例：

```c
const Unitree_Motor_t *m = MotorTask_GetMotor(0);
if (m->correct) {
    /* m->pos (rad)、m->vel (rad/s)、m->tor (N.m)、m->temp (℃)、m->error */
}
```

发送 MIT 控制帧（mode 1：力位混合）：

```c
/* 单个电机：位置 0 rad、速度 0 rad/s、kp=20、kd=0.5、前馈力矩 0 N.m */
(void)MotorTask_SendMit(0, 0.0f, 0.0f, 20.0f, 0.5f, 0.0f);

/* 只更新缓存，再统一发送（便于多电机同步） */
(void)MotorTask_SetMit(0, 0.0f, 0.0f, 20.0f, 0.5f, 0.0f);
(void)MotorTask_SetMit(1, 0.0f, 0.0f, 20.0f, 0.5f, 0.0f);
(void)MotorTask_SendAll();
```

- 接口为**阻塞发送**，单帧 17 Byte 在 4 Mbit/s 下约 42.5 us，只能在主循环/线程上下文调用，不要在中断里调用。
- 发送期间默认关闭接收机（`MOTOR_TX_GATE_RX=1`），防止半双工回显破坏固定 16 Byte 的反馈帧；RX DMA 始终保持 arm，无需手动重开。
- 同一路总线上两帧需错开时间（建议每总线每 tick 只发一帧，或间隔 ≥200 us），背靠背发送会与上一台电机的反馈帧冲突。

### 当前进度

- [x] 双路 RS485 DMA 接收、帧校验、官方解包
- [x] 控制帧发送（`modify_data()` + `HAL_UART_Transmit` 阻塞发送，MIT 接口）
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
