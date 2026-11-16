/**
 * @file    motor_task.h
 * @brief   Unitree GO-M8010-6 电机任务（接收部分）
 *          两路 RS485：USART2 挂 ID 0/1，USART3 挂 ID 2/3
 *          DMA + 空闲中断按 16 Byte 接收，回调里直接校验并解析反馈帧
 */
#ifndef __MOTOR_TASK_H
#define __MOTOR_TASK_H

#include "main.h"
#include "gom_protocol.h"

/* 电机数量：ID 0 ~ 3 */
#define MOTOR_NUM          4

/**
 * @brief  初始化电机结构体（ID 0~3，FOC 闭环，指令量清零）并开启各路 DMA 接收
 */
void MotorTask_Init(void);

/**
 * @brief  取得某个电机的指令结构体（发送部分后续实现时使用）
 * @param  id 电机 ID (0 ~ MOTOR_NUM-1)，越界返回 NULL
 */
MotorCmd_t *MotorTask_GetCmd(uint8_t id);

/**
 * @brief  取得某个电机的反馈结构体
 * @param  id 电机 ID (0 ~ MOTOR_NUM-1)，越界返回 NULL
 */
MotorData_t *MotorTask_GetData(uint8_t id);

#endif /* __MOTOR_TASK_H */
