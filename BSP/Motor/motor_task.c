/**
 * @file    motor_task.c
 * @brief   Unitree GO-M8010-6 电机任务（收发）
 *
 * 数据结构：
 *   每个电机一个 Unitree_Motor_t（motor[id]），只存上层需要的**物理量**：
 *   5 个指令量 + 3 个反馈量 + 模式/温度/错误/有效标志。
 *   协议层要求整帧结构体：发送借用模块内的 tx_cmd 组帧，接收在中断里用栈上
 *   的 MotorData_t 解包后再拷出物理量。
 *
 * 接收思路：
 *   主机发送 17 Byte 控制帧后，GO-M8010-6 固定返回 16 Byte 反馈帧。
 *   每路 RS485 用 HAL_UARTEx_ReceiveToIdle_DMA() 开启接收（缓冲 64 Byte），
 *   串口空闲/接收完成时进入 HAL_UARTEx_RxEventCallback：
 *     1. 判断是哪路串口（USART2 挂 ID 0/1，USART3 挂 ID 2/3）；
 *     2. 长度必须是 16、包头必须是 0xFD 0xEE、ID 必须属于该串口，否则丢弃；
 *     3. 交给宇树官方解包函数 extract_data()（CRC 校验 + 物理量换算），
 *        再把物理量拷进 motor[id]。
 *
 * 发送思路：
 *   MotorTask_SendMit() 先填 motor[id] 的指令量，再拷进 tx_cmd、modify_data()
 *   打包成 17 Byte，按 ID 选 USART2/USART3 用 HAL_UART_Transmit() 阻塞发送。
 */

#include "motor_task.h"
#include "usart.h"
#include <math.h>
#include <string.h>

/* ------------------------------ 私有宏 ------------------------------ */
/* 电机反馈帧固定 16 Byte：0xFD 0xEE | mode(含 ID) | 11 Byte 反馈 | CRC16 */
#define MOTOR_FRAME_SIZE     ((uint16_t)sizeof(RIS_MotorData_t))
/* 接收缓冲留余量；帧长仍按 16 Byte 判定，超长帧直接丢弃 */
#define MOTOR_RX_BUF_SIZE    64U
#define MOTOR_FRAME_HEAD0    0xFDU               /* 反馈帧包头第 1 字节 */
#define MOTOR_FRAME_HEAD1    0xEEU               /* 反馈帧包头第 2 字节 */

/* 控制帧固定 17 Byte：0xFE 0xEE | mode(含 ID) | 12 Byte 期望量 | CRC16 */
#define MOTOR_CMD_FRAME_SIZE ((uint16_t)sizeof(RIS_ControlData_t))
#define MOTOR_BUS_IDS        2U                  /* 每路总线挂 2 台电机 */
#define MOTOR_TWO_PI         6.28318f            /* 2π，位置归一化用 */

/* 编译期检查：打包结构必须正好 17 Byte，否则协议错位 */
typedef char motor_cmd_frame_size_check[(sizeof(RIS_ControlData_t) == 17U) ? 1 : -1];

/* ------------------------------ 私有变量 ------------------------------ */
/* 四个电机，下标即电机 ID（不加 static，方便调试器直接 watch motor[0]） */
Unitree_Motor_t motor[MOTOR_NUM];

/* 两路 RS485 各自的 DMA 接收缓冲，可容纳多帧（帧长 16 Byte） */
static uint8_t rx_usart2[MOTOR_RX_BUF_SIZE];     /* USART2：电机 ID 0、1 */
static uint8_t rx_usart3[MOTOR_RX_BUF_SIZE];     /* USART3：电机 ID 2、3 */

/* 打包用临时结构：只在主循环阻塞发送期间使用 */
static MotorCmd_t  tx_cmd;

/* ------------------------------ 私有函数声明 ------------------------------ */
static void MotorTask_StartRx(UART_HandleTypeDef *huart, uint8_t *buf);
static void MotorTask_HandleFrame(uint8_t *buf, uint16_t Size, uint8_t id_base);
static void MotorTask_ErrorRecover(UART_HandleTypeDef *huart, uint8_t *buf);
static UART_HandleTypeDef *MotorTask_GetUart(uint8_t id);
static uint8_t MotorTask_ValueValid(float v);
static HAL_StatusTypeDef MotorTask_Transmit(const Unitree_Motor_t *m);

/* ------------------------------ 函数实现 ------------------------------ */

/**
 * @brief  初始化电机结构体并开启 USART2、USART3 的 DMA 接收
 */
void MotorTask_Init(void)
{
    uint8_t i;

    for (i = 0; i < MOTOR_NUM; i++)
    {
        motor[i].id      = i;      /* 电机 ID: 0,1,2,3 */
        motor[i].mode    = 1U;     /* 1: FOC 闭环控制 (0: 阻尼/锁定, 2: 编码器标定) */
        motor[i].pos_des = 0.0f;   /* 期望位置 (rad) */
        motor[i].vel_des = 0.0f;   /* 期望速度 (rad/s) */
        motor[i].tor_des = 0.0f;   /* 前馈力矩 (N.m) */
        motor[i].kp      = 0.0f;   /* 位置刚度 (0 ~ 25.599) */
        motor[i].kd      = 0.0f;   /* 速度阻尼 (0 ~ 25.599) */
        motor[i].correct = 0U;     /* 尚未收到有效反馈 */
        motor[i].rx_bad  = 0U;
    }

    MotorTask_StartRx(&huart2, rx_usart2);
    MotorTask_StartRx(&huart3, rx_usart3);
}

/**
 * @brief  获取某个电机的状态（只读）
 */
const Unitree_Motor_t *MotorTask_GetMotor(uint8_t id)
{
    if (id >= MOTOR_NUM)
    {
        return NULL;
    }
    return &motor[id];
}

/* ------------------------------ 发送实现 ------------------------------ */

/**
 * @brief  ID 到总线的映射：ID 0/1 -> USART2，ID 2/3 -> USART3
 */
static UART_HandleTypeDef *MotorTask_GetUart(uint8_t id)
{
    return (id < MOTOR_BUS_IDS) ? &huart2 : &huart3;
}

/**
 * @brief  参数合法性：排除 NaN 与 ±Inf
 * @note   modify_data() 的 SATURATE 宏对 NaN 无效（比较恒为假），必须提前拦截
 */
static uint8_t MotorTask_ValueValid(float v)
{
    return ((v == v) && (v >= -1.0e9f) && (v <= 1.0e9f)) ? 1U : 0U;
}

/**
 * @brief  组帧（modify_data）并阻塞发送 17 Byte 控制帧
 * @note   MOTOR_TX_GATE_RX=1 时在发送期间清 USART_CR1_RE，避免半双工回显
 *         把固定 16 Byte 的反馈窗口冲乱；RX DMA 全程保持 arm，无需重开。
 *         只能在主循环/线程上下文调用，不可在中断里调用。
 */
static HAL_StatusTypeDef MotorTask_Transmit(const Unitree_Motor_t *m)
{
    UART_HandleTypeDef *huart = MotorTask_GetUart(m->id);
    HAL_StatusTypeDef st;

    /* 物理量 -> 协议层结构体（modify_data 内部做饱和、Q8/Q15 换算和 CRC） */
    tx_cmd.id   = m->id;
    tx_cmd.mode = m->mode;
    tx_cmd.Pos  = m->pos_des;
    tx_cmd.W    = m->vel_des;
    tx_cmd.T    = m->tor_des;
    tx_cmd.K_P  = m->kp;
    tx_cmd.K_W  = m->kd;
    modify_data(&tx_cmd);

#if (MOTOR_TX_GATE_RX != 0)
    ATOMIC_CLEAR_BIT(huart->Instance->CR1, USART_CR1_RE);
#endif

    st = HAL_UART_Transmit(huart, (const uint8_t *)&tx_cmd.motor_send_data,
                           MOTOR_CMD_FRAME_SIZE, MOTOR_TX_TIMEOUT_MS);

#if (MOTOR_TX_GATE_RX != 0)
    ATOMIC_SET_BIT(huart->Instance->CR1, USART_CR1_RE);
#endif

    return st;
}

/**
 * @brief  只更新指令缓存，不发送
 */
HAL_StatusTypeDef MotorTask_SetMit(uint8_t id, float pos, float vel,
                                   float kp, float kd, float tq)
{
    Unitree_Motor_t *m;

    if (id >= MOTOR_NUM)
    {
        return HAL_ERROR;
    }
    if ((MotorTask_ValueValid(pos) == 0U) || (MotorTask_ValueValid(vel) == 0U) ||
        (MotorTask_ValueValid(kp) == 0U)  || (MotorTask_ValueValid(kd) == 0U)  ||
        (MotorTask_ValueValid(tq) == 0U))
    {
        return HAL_ERROR;
    }

    m = &motor[id];
    m->mode    = 1U;       /* 1: FOC 闭环（MIT 力位混合） */
    m->pos_des = pos;
    m->vel_des = vel;
    m->kp      = kp;
    m->kd      = kd;
    m->tor_des = tq;
    return HAL_OK;
}

/**
 * @brief  设置并立即发送一个电机的 MIT 指令
 */
HAL_StatusTypeDef MotorTask_SendMit(uint8_t id, float pos, float vel,
                                    float kp, float kd, float tq)
{
    HAL_StatusTypeDef st = MotorTask_SetMit(id, pos, vel, kp, kd, tq);

    return (st != HAL_OK) ? st : MotorTask_Transmit(&motor[id]);
}

/**
 * @brief  依次发送 ID 0~3 当前缓存的指令
 */
HAL_StatusTypeDef MotorTask_SendAll(void)
{
    HAL_StatusTypeDef st = HAL_OK;
    HAL_StatusTypeDef s;
    uint8_t id;

    for (id = 0U; id < MOTOR_NUM; id++)
    {
        s = MotorTask_Transmit(&motor[id]);
        if (s != HAL_OK)
        {
            st = s;            /* 返回第一个错误，但继续发完其余电机 */
        }
    }
    return st;
}

/**
 * @brief  开启/重启某一路串口的 DMA 空闲中断接收（按 16 Byte 长度接收）
 * @param  huart 串口句柄
 * @param  buf   该串口对应的 DMA 接收缓冲
 */
static void MotorTask_StartRx(UART_HandleTypeDef *huart, uint8_t *buf)
{
    if (HAL_UARTEx_ReceiveToIdle_DMA(huart, buf, MOTOR_RX_BUF_SIZE) == HAL_OK)
    {
        /* 只使用空闲(IDLE)/传输完成(TC)事件；关掉半传输中断，
           否则半包数据也会进回调 */
        __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);
    }
}

/**
 * @brief  校验并解析一帧反馈（中断上下文）
 * @param  buf     该串口的 DMA 接收缓冲
 * @param  Size    本次收到的字节数
 * @param  id_base 该串口上的起始电机 ID（USART2: 0，USART3: 2，即挂 id_base/id_base+1）
 * @note   长度/包头/ID 任一不符直接丢弃；CRC 由 extract_data() 校验。
 */
static void MotorTask_HandleFrame(uint8_t *buf, uint16_t Size, uint8_t id_base)
{
    MotorData_t rx;              /* 栈上解包 */
    Unitree_Motor_t *m;
    uint8_t id;

    /* 1. 长度必须是 16 Byte */
    if (Size != MOTOR_FRAME_SIZE)
    {
        return;
    }

    /* 2. 包头必须是 0xFD 0xEE */
    if ((buf[0] != MOTOR_FRAME_HEAD0) || (buf[1] != MOTOR_FRAME_HEAD1))
    {
        return;
    }

    /* 3. ID 必须是本总线上的电机 */
    id = (uint8_t)(buf[2] & 0x0FU);
    if ((id < id_base) || (id > (uint8_t)(id_base + 1U)))
    {
        return;
    }

    /* 4. 解包：extract_data() 内部校验 CRC 并换算物理量 */
    rx.correct = 0U;
    rx.bad_msg = 0U;
    memcpy(&rx.motor_recv_data, buf, MOTOR_FRAME_SIZE);
    extract_data(&rx);

    m = &motor[id];
    if (rx.correct == 0U)        /* CRC 错 */
    {
        m->correct = 0U;
        m->rx_bad++;
        return;
    }

    /* 5. CRC 正确，换算到输出轴(关节)侧并更新 */
    m->correct = 1U;

    /* 转子侧角度 -> 输出轴角度，再归一化到 [0, 2π) */
    m->pos = rx.Pos / MOTOR_GEAR_RATIO;
    m->pos = fmodf(m->pos, MOTOR_TWO_PI);
    if (m->pos < 0.0f)
    {
        m->pos += MOTOR_TWO_PI;
    }

    m->vel   = rx.W / MOTOR_GEAR_RATIO;    /* 转速：转子 -> 输出轴 */
    m->tor   = rx.T * MOTOR_GEAR_RATIO;    /* 力矩：转子 -> 输出轴 */
    m->temp  = (int8_t)rx.Temp;
    m->error = (uint8_t)rx.MError;
}

/**
 * @brief  串口出错后清标志并重启接收
 */
static void MotorTask_ErrorRecover(UART_HandleTypeDef *huart, uint8_t *buf)
{
    if ((huart->ErrorCode & HAL_UART_ERROR_ORE) != 0U)
    {
        __HAL_UART_CLEAR_OREFLAG(huart);
    }

    /* 出错后 HAL 会终止接收并把状态置为 BUSY/READY，这里强制回到 READY 再重新开启 */
    huart->RxState = HAL_UART_STATE_READY;
    __HAL_UNLOCK(huart);

    MotorTask_StartRx(huart, buf);
}

/**
 * @brief  DMA 空闲/传输完成回调：先判断是哪路串口，再处理（中断上下文）
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart->Instance == USART2)
    {
        MotorTask_HandleFrame(rx_usart2, Size, 0U);      /* USART2 挂 ID 0/1 */
        MotorTask_StartRx(&huart2, rx_usart2);           /* 收完即停，必须重新开启 */
    }
    else if (huart->Instance == USART3)
    {
        MotorTask_HandleFrame(rx_usart3, Size, 2U);      /* USART3 挂 ID 2/3 */
        MotorTask_StartRx(&huart3, rx_usart3);
    }
}

/**
 * @brief  串口错误回调
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2)
    {
        MotorTask_ErrorRecover(&huart2, rx_usart2);
    }
    else if (huart->Instance == USART3)
    {
        MotorTask_ErrorRecover(&huart3, rx_usart3);
    }
}
