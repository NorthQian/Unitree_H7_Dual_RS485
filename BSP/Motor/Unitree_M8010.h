/**
 * @file    Unitree_M8010.h
 * @brief   Unitree GO-M8010-6 电机驱动（MIT 收发）
 *          两路 RS485：USART2 挂 ID 0/1，USART3 挂 ID 2/3
 *          DMA + 空闲中断按 16 Byte 接收，回调里直接校验并解析反馈帧
 */
#ifndef __UNITREE_M8010_H
#define __UNITREE_M8010_H

#include "main.h"
#include "gom_protocol.h"

/* 电机数量：ID 0 ~ 3 */
#define MOTOR_NUM          4

/* ---------------------- 减速比（GO-M8010-6 行星减速器） ---------------------- */
#define MOTOR_GEAR_RATIO   6.33f    /* 减速比 6.33 : 1 */

/* 关节侧 -> 电机(转子)侧：位置/速度乘减速比，力矩除减速比 */
#define MOTOR_JOINT_POS_TO_ROTOR(p)     ((p) * MOTOR_GEAR_RATIO)
#define MOTOR_JOINT_VEL_TO_ROTOR(v)     ((v) * MOTOR_GEAR_RATIO)
#define MOTOR_JOINT_TORQUE_TO_ROTOR(t)  ((t) / MOTOR_GEAR_RATIO)

/* 电机(转子)侧 -> 关节侧：位置/速度除减速比，力矩乘减速比 */
#define MOTOR_ROTOR_POS_TO_JOINT(p)     ((p) / MOTOR_GEAR_RATIO)
#define MOTOR_ROTOR_VEL_TO_JOINT(v)     ((v) / MOTOR_GEAR_RATIO)
#define MOTOR_ROTOR_TORQUE_TO_JOINT(t)  ((t) * MOTOR_GEAR_RATIO)

/**
 * @brief 单个电机的全部信息（下标即电机 ID，调试 watch motor[0] 即可）
 * @note  指令是**电机(转子)侧**量（输出轴量需 × 减速比后再传入）；
 *        反馈已由驱动层换算到**输出轴(关节)侧**，pos 归一化到 [0, 2π)
 */
typedef struct
{
    uint8_t id;         /* 电机 ID (0 ~ 3)         */

    /* ---- 指令 ---- */
    float   pos_des;    /* 期望位置 (rad)          */
    float   vel_des;    /* 期望速度 (rad/s)        */
    float   tor_des;    /* 前馈力矩 (N.m)          */
    float   kp;         /* 位置刚度 (0 ~ 25.599)   */
    float   kd;         /* 速度阻尼 (0 ~ 25.599)   */
    uint8_t mode;       /* 0: 阻尼/锁定  1: FOC 闭环 */

    /* ---- 反馈 ---- */
    float   pos;        /* 输出轴位置 (rad, 0~2π)  */
    float   vel;        /* 输出轴速度 (rad/s)      */
    float   tor;        /* 输出轴力矩 (N.m)        */
    int8_t  temp;       /* 电机温度 (℃)           */
    uint8_t error;      /* MError: 0 正常，非 0 为故障码 */
    uint8_t correct;    /* 1: 最近一帧反馈有效     */
    uint32_t rx_bad;    /* CRC 错误累计            */
} Unitree_Motor_t;

/**
 * @brief  初始化电机结构体（ID 0~3，FOC 闭环，指令量清零）并开启各路 DMA 接收
 */
void Unitree_M8010_Init(void);

/**
 * @brief  取得某个电机的状态（只读；指令请用 Unitree_M8010_SetMit/SendMit 下发）
 * @param  id 电机 ID (0 ~ MOTOR_NUM-1)，越界返回 NULL
 */
const Unitree_Motor_t *Unitree_M8010_GetMotor(uint8_t id);

/* ---------------------- 控制帧发送（MIT 力位混合） ---------------------- */
#define MOTOR_TX_TIMEOUT_MS  2U     /* 17 Byte @4 Mbit/s ≈ 42.5 us，2 ms 余量充足 */
#define MOTOR_TX_GATE_RX     1      /* 1: 发送期间关闭接收机，抑制半双工回显 */

/**
 * @brief  只更新某电机指令缓存，不发送（mode 固定 1: FOC 闭环）
 * @param  id    电机 ID (0 ~ MOTOR_NUM-1)
 * @param  pos   期望位置 (rad)
 * @param  vel   期望速度 (rad/s)
 * @param  kp    位置刚度 (0 ~ 25.599)
 * @param  kd    速度阻尼 (0 ~ 25.599)
 * @param  tq    前馈力矩 (N.m)
 * @retval HAL_OK；id 越界或参数为 NaN/Inf 返回 HAL_ERROR
 * @note   范围饱和由 modify_data() 完成；仅主循环/线程上下文调用
 * @note   协议里的 q/dq/tau 都是**电机(转子)侧**量，关节侧要先换算：
 *         位置/速度 用 MOTOR_JOINT_*_TO_ROTOR()，力矩用 MOTOR_JOINT_TORQUE_TO_ROTOR()
 */
HAL_StatusTypeDef Unitree_M8010_SetMit(uint8_t id, float pos, float vel,
                                   float kp, float kd, float tq);

/**
 * @brief  设置并立即发送一个电机的 MIT 指令（最常用接口）
 * @retval HAL_OK / HAL_ERROR / HAL_BUSY / HAL_TIMEOUT
 */
HAL_StatusTypeDef Unitree_M8010_SendMit(uint8_t id, float pos, float vel,
                                    float kp, float kd, float tq);

/**
 * @brief  依次发送 ID 0~3 当前缓存的指令（配合 Unitree_M8010_SetMit 做整机周期发送）
 * @warning 同一路总线上两帧背靠背只间隔约 42.5 us，可能与上一台电机的
 *          反馈帧冲突；闭环运行时请把不同电机的发送时间错开。
 */
HAL_StatusTypeDef Unitree_M8010_SendAll(void);

#endif /* __UNITREE_M8010_H */
