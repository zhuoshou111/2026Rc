#ifndef __CHASSIS_MECANUM_H
#define __CHASSIS_MECANUM_H

#ifdef __cplusplus
extern "C" {
#endif

/*======================================================================*/
/*  四轮麦克纳姆轮定点全向移动 — 步进电机纯算法（与驱动无关）            */
/*======================================================================*/

#include <stdint.h>

/*========== 底盘几何 + 电机参数 ==========*/
typedef struct {
    float wheel_radius;       /* 轮子半径 (mm)                           */
    float half_length;        /* 底盘半长，X方向 中心→轮子 (mm)          */
    float half_width;         /* 底盘半宽，Y方向 中心→轮子 (mm)          */
    int32_t steps_per_rev;    /* 轮子转一圈的脉冲数（含细分数）            */
                              /* 例：200步电机 + 16细分 → 3200           */
} ChassisConfig;

/*========== 位姿（世界坐标系）==========*/
typedef struct {
    float x;                  /* X 坐标 (mm)      */
    float y;                  /* Y 坐标 (mm)      */
    float theta;              /* 朝向角 (rad)     */
} ChassisPose;

/*========== 四轮脉冲增量（输出值）==========*/
typedef struct {
    int32_t step_fl;          /* 前左轮 (Front-Left)   正=前进     */
    int32_t step_fr;          /* 前右轮 (Front-Right)  正=前进     */
    int32_t step_rl;          /* 后左轮 (Rear-Left)    正=前进     */
    int32_t step_rr;          /* 后右轮 (Rear-Right)   正=前进     */
} ChassisSteps;

/*========== 四轮当前累计脉冲（里程用）==========*/
typedef struct {
    int32_t total_fl;         /* 前左轮累计脉冲 */
    int32_t total_fr;         /* 前右轮累计脉冲 */
    int32_t total_rl;         /* 后左轮累计脉冲 */
    int32_t total_rr;         /* 后右轮累计脉冲 */
} ChassisOdom;

/*========== 函数接口 ==========*/

/**
 * @brief  初始化底盘参数（必须最先调用）
 * @param  cfg  底盘几何 + 步进电机参数
 */
void CHASSIS_Init(const ChassisConfig *cfg);

/**
 * @brief  定点全向移动：当前位姿 → 目标位姿，解算四轮脉冲增量
 * @param  cur   当前位姿 (x, y, theta)
 * @param  tgt   目标位姿 (x, y, theta)
 * @param  out   输出各轮所需脉冲增量（正=前进，负=后退）
 * @note   直接由位移差一步算出，无中间速度量
 */
void CHASSIS_MoveTo(const ChassisPose *cur, const ChassisPose *tgt, ChassisSteps *out);

/**
 * @brief  正运动学：累计脉冲 → 推算当前位姿（里程计）
 * @param  odom     四轮累计脉冲（自上次复位起）
 * @param  ref_pose 上次已知位姿（作为累加基准）
 * @param  out      输出的推算位姿
 */
void CHASSIS_Odom(const ChassisOdom *odom, const ChassisPose *ref_pose, ChassisPose *out);

/**
 * @brief  脉冲增量累加到里程计
 */
void CHASSIS_OdomAdd(ChassisOdom *odom, const ChassisSteps *delta);

/**
 * @brief  里程计清零
 */
void CHASSIS_OdomReset(ChassisOdom *odom);

#ifdef __cplusplus
}
#endif

#endif /* __CHASSIS_MECANUM_H */
