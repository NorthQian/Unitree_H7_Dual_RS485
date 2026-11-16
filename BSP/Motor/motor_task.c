/**
 * @file    motor_task.c
 * @brief   Unitree GO-M8010-6 电机任务（接收部分）
 *
 * 接收思路：
 *   主机发送 17 Byte 控制帧后，GO-M8010-6 固定返回 16 Byte 反馈帧，
 *   因此每路 RS485 用 HAL_UARTEx_ReceiveToIdle_DMA() 按 16 Byte 长度开启接收，
 *   串口空闲/接收完成时进入 HAL_UARTEx_RxEventCallback：
 *     1. 先判断是哪路串口发来的（USART2 上挂 ID 0/1，USART3 上挂 ID 2/3）；
 *     2. 判断收到的字节数是否为 16；
 *     3. 判断包头是否为 0xFD 0xEE、ID 是否属于该串口；
 *     4. 都符合后交给宇树官方解包函数 extract_data()（CRC 校验 + 物理量换算）。
 *
 * 发送部分暂未实现。
 */

#include "motor_task.h"
#include "usart.h"
#include <string.h>

/* ------------------------------ 私有宏 ------------------------------ */
/* 电机反馈帧固定 16 Byte：0xFD 0xEE | mode(含 ID) | 11 Byte 反馈 | CRC16 */
#define MOTOR_FRAME_SIZE     ((uint16_t)sizeof(RIS_MotorData_t))
#define MOTOR_FRAME_HEAD0    0xFDU               /* 反馈帧包头第 1 字节 */
#define MOTOR_FRAME_HEAD1    0xEEU               /* 反馈帧包头第 2 字节 */

/* ------------------------------ 私有变量 ------------------------------ */
/* 四个电机的指令结构体，下标即电机 ID（发送部分后续使用） */
static MotorCmd_t  motor_cmd[MOTOR_NUM];
/* 四个电机的反馈结构体，下标即电机 ID */
static MotorData_t motor_data[MOTOR_NUM];

/* 两路 RS485 各自的 DMA 接收缓冲，一帧正好 16 Byte */
static uint8_t rx_usart2[MOTOR_FRAME_SIZE];      /* USART2：电机 ID 0、1 */
static uint8_t rx_usart3[MOTOR_FRAME_SIZE];      /* USART3：电机 ID 2、3 */

/* ------------------------------ 私有函数声明 ------------------------------ */
static void MotorTask_StartRx(UART_HandleTypeDef *huart, uint8_t *buf);
static void MotorTask_HandleFrame(uint8_t *buf, uint16_t Size, uint8_t id_min, uint8_t id_max);
static void MotorTask_ErrorRecover(UART_HandleTypeDef *huart, uint8_t *buf);

/* ------------------------------ 函数实现 ------------------------------ */

/**
 * @brief  初始化电机结构体并开启 USART2、USART3 的 DMA 接收
 */
void MotorTask_Init(void)
{
    uint8_t i;

    for (i = 0; i < MOTOR_NUM; i++)
    {
        motor_cmd[i].id   = i;     /* 电机 ID: 0,1,2,3 */
        motor_cmd[i].mode = 1;     /* 1: FOC 闭环控制 (0: 锁定, 2: 编码器标定) */
        motor_cmd[i].K_P  = 0.0f;  /* 关节刚度系数 (0 ~ 25.599) */
        motor_cmd[i].K_W  = 0.0f;  /* 关节阻尼系数 (0 ~ 25.599) */
        motor_cmd[i].Pos  = 0.0f;  /* 期望位置 (rad) */
        motor_cmd[i].W    = 0.0f;  /* 期望速度 (rad/s) */
        motor_cmd[i].T    = 0.0f;  /* 期望力矩 (N.m) */
    }

    MotorTask_StartRx(&huart2, rx_usart2);
    MotorTask_StartRx(&huart3, rx_usart3);
}

/**
 * @brief  获取电机指令结构体指针
 */
MotorCmd_t *MotorTask_GetCmd(uint8_t id)
{
    if (id >= MOTOR_NUM)
    {
        return NULL;
    }
    return &motor_cmd[id];
}

/**
 * @brief  获取电机反馈结构体指针
 */
MotorData_t *MotorTask_GetData(uint8_t id)
{
    if (id >= MOTOR_NUM)
    {
        return NULL;
    }
    return &motor_data[id];
}

/**
 * @brief  开启/重启某一路串口的 DMA 空闲中断接收（按 16 Byte 长度接收）
 * @param  huart 串口句柄
 * @param  buf   该串口对应的 DMA 接收缓冲
 */
static void MotorTask_StartRx(UART_HandleTypeDef *huart, uint8_t *buf)
{
    if (HAL_UARTEx_ReceiveToIdle_DMA(huart, buf, MOTOR_FRAME_SIZE) == HAL_OK)
    {
        /* 只使用空闲(IDLE)/传输完成(TC)事件；关掉半传输中断，
           否则半包数据也会进回调 */
        __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);
    }
}

/**
 * @brief  校验并解析刚收到的一帧反馈（中断上下文）
 * @param  buf    该串口的 DMA 接收缓冲
 * @param  Size   DMA 实际收到的字节数
 * @param  id_min 该串口上最小电机 ID
 * @param  id_max 该串口上最大电机 ID
 */
static void MotorTask_HandleFrame(uint8_t *buf, uint16_t Size, uint8_t id_min, uint8_t id_max)
{
    uint8_t id;

    /* 1. 长度必须是 16 Byte，否则丢弃 */
    if (Size != MOTOR_FRAME_SIZE)
    {
        return;
    }

    /* 2. 校验包头 0xFD 0xEE */
    if ((buf[0] != MOTOR_FRAME_HEAD0) || (buf[1] != MOTOR_FRAME_HEAD1))
    {
        return;
    }

    /* 3. 取出 ID（mode 字节低 4 位），必须是该串口上的电机 */
    id = (uint8_t)(buf[2] & 0x0FU);
    if ((id < id_min) || (id > id_max))
    {
        return;
    }

    /* 4. 官方解包函数：再校验一次包头/CRC，并换算 rad、rad/s、N.m 等物理量 */
    memcpy(&motor_data[id].motor_recv_data, buf, MOTOR_FRAME_SIZE);
    extract_data(&motor_data[id]);
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
        MotorTask_HandleFrame(rx_usart2, Size, 0, 1);   /* USART2：ID 0/1 */
        MotorTask_StartRx(&huart2, rx_usart2);          /* Normal 模式收完即停，必须重新开启 */
    }
    else if (huart->Instance == USART3)
    {
        MotorTask_HandleFrame(rx_usart3, Size, 2, 3);   /* USART3：ID 2/3 */
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
