#ifndef __CHASSIS_DRIVER_H
#define __CHASSIS_DRIVER_H

#ifdef __cplusplus
extern "C" {
#endif

/*======================================================================*/
/*  底盘驱动桥接层 — 连接 chassis_mecanum 算法 ↔ 电机驱动层 (TIM3)        */
/*                                                                      */
/*  硬件拓扑:                                                             */
/*    FL(MOTOR3/TIM3_CH1/PC6/DIR1) ─── FR(MOTOR4/TIM3_CH2/PC7/DIR2)     */
/*    RL(MOTOR7/TIM3_CH3/PC8/DIR3) ─── RR(MOTOR8/TIM3_CH4/PC9/DIR4)     */
/*                                                                      */
/*  局限: 四轮共用一个定时器 (TIM3), 每次移动所有轮子脉冲数相同。           */
/*        纯平移和纯旋转是精确的，平移+旋转组合移动会引入微小误差。          */
/*======================================================================*/

#include "chassis_mecanum.h"
#include <stdint.h>

/*========== 初始化 ==========*/

/**
 * @brief  初始化底盘驱动（在系统初始化阶段调用一次）
 * @param  cfg  底盘几何参数 + 电机参数
 * @note   必须先调用 CHASSIS_Init() 再调用此函数
 */
void CHASSIS_DRIVER_Init(const ChassisConfig *cfg);

/*========== 绝对移动 ==========*/

/**
 * @brief  定点全向移动：从当前位置走到世界坐标 (x_mm, y_mm, theta_rad)
 * @param  x_mm       目标 X 坐标 (mm)
 * @param  y_mm       目标 Y 坐标 (mm)
 * @param  theta_rad  目标朝向角 (rad), 0=正前方
 * @note   阻塞式，完成后自动更新里程计并调用 MOTOR_Align() 陀螺仪校准
 */
void CHASSIS_DRIVER_GoTo(float x_mm, float y_mm, float theta_rad);

/**
 * @brief  旋转到指定绝对角度
 * @param  theta_rad  目标朝向角 (rad)
 * @note   内部调用 CHASSIS_DRIVER_GoTo(x, y, theta)
 */
void CHASSIS_DRIVER_TurnTo(float theta_rad);

/*========== 相对移动 ==========*/

/**
 * @brief  相对移动：在底盘坐标系下平移 (dx_mm, dy_mm)，不旋转
 * @param  dx_mm  底盘前方位移 (mm), 正=前进, 负=后退
 * @param  dy_mm  底盘左方位移 (mm), 正=左移, 负=右移
 * @note   阻塞式，角度不变，内部会先转为世界坐标再调用 GoTo
 */
void CHASSIS_DRIVER_MoveRel(float dx_mm, float dy_mm);

/**
 * @brief  相对旋转：原地旋转 dtheta_rad
 * @param  dtheta_rad  旋转角 (rad), 正=逆时针
 * @note   阻塞式，位置不变
 */
void CHASSIS_DRIVER_TurnRel(float dtheta_rad);

/*========== 位姿管理 ==========*/

/**
 * @brief  重置当前位姿（通常用于设定原点）
 * @param  x_mm, y_mm, theta_rad  新的当前位姿
 */
void CHASSIS_DRIVER_ResetPose(float x_mm, float y_mm, float theta_rad);

/**
 * @brief  获取当前估计位姿（里程计推算）
 * @param  out  输出当前位姿 (x_mm, y_mm, theta_rad)
 */
void CHASSIS_DRIVER_GetPose(ChassisPose *out);

/**
 * @brief  获取里程计原始脉冲累计值
 * @param  out  输出四轮累计脉冲
 */
void CHASSIS_DRIVER_GetOdom(ChassisOdom *out);

#ifdef __cplusplus
}
#endif

#endif /* __CHASSIS_DRIVER_H */
