/*======================================================================*/
/*  底盘驱动桥接层 — 连接 chassis_mecanum 算法 ↔ 电机驱动层                */
/*                                                                      */
/*  调用链:                                                              */
/*    CHASSIS_DRIVER_GoTo()                                              */
/*      → CHASSIS_MoveTo()       (逆运动学: 位姿差 → 四轮脉冲增量)        */
/*      → _SetDirection()       (设置 DIR1~4 独立方向)                   */
/*      → MSD_Move()            (TIM3 统一驱动四轮, 取 max 脉冲)          */
/*      → _WaitComplete()       (阻塞等待 stepPosition == distance)      */
/*      → CHASSIS_OdomAdd()     (累加里程计)                             */
/*      → CHASSIS_Odom()        (更新当前位姿)                            */
/*      → MOTOR_Align()         (陀螺仪角度校准)                          */
/*                                                                      */
/*  轮→电机映射:                                                          */
/*    FL(前左) → MOTOR3 → DIR1(PG7) → TIM3_CH1(PC6)                     */
/*    FR(前右) → MOTOR4 → DIR2(PG8) → TIM3_CH2(PC7)                     */
/*    RL(后左) → MOTOR7 → DIR3(PA8) → TIM3_CH3(PC8)                     */
/*    RR(后右) → MOTOR8 → DIR4(PC1) → TIM3_CH4(PC9)                     */
/*======================================================================*/

#include "chassis_driver.h"
#include "contral.h"        /* MSD_Move, DIR1~4, stepPosition, distance, MOTOR_Align */
#include "delay.h"          /* delay_ms */
#include <stdlib.h>         /* abs */

/* ---- 模块级全局变量 ---- */
static ChassisPose  g_pose;         /* 当前估计位姿 (世界坐标) */
static ChassisOdom  g_odom;         /* 累计脉冲里程计          */

/*======================================================================*/
/*  _SetDirection — 根据四轮脉冲增量设置 DIR1~4                          */
/*                                                                      */
/*  方向映射规则:                                                        */
/*    DIR1/4: CW=HIGH/SET, CCW=LOW/RESET                                */
/*    DIR2/3: CW=LOW/RESET,  CCW=HIGH/SET  (硬件反相, DIRx内部已处理)    */
/*                                                                      */
/*  因此: 脉冲>0 → CW(前进), 脉冲<0 → CCW(后退), 脉冲=0 → 不动           */
/*======================================================================*/
static void _SetDirection(const ChassisSteps *steps)
{
    /* 前左 FL → DIR1 */
    if      (steps->step_fl > 0) DIR1(CW);
    else if (steps->step_fl < 0) DIR1(CCW);

    /* 前右 FR → DIR2 */
    if      (steps->step_fr > 0) DIR2(CW);
    else if (steps->step_fr < 0) DIR2(CCW);

    /* 后左 RL → DIR3 */
    if      (steps->step_rl > 0) DIR3(CW);
    else if (steps->step_rl < 0) DIR3(CCW);

    /* 后右 RR → DIR4 */
    if      (steps->step_rr > 0) DIR4(CW);
    else if (steps->step_rr < 0) DIR4(CCW);
}

/*======================================================================*/
/*  _MaxAbs4 — 返回四个 int32_t 中绝对值的最大值                          */
/*======================================================================*/
static uint32_t _MaxAbs4(int32_t a, int32_t b, int32_t c, int32_t d)
{
    uint32_t ua = (a >= 0) ? (uint32_t)a : (uint32_t)(-a);
    uint32_t ub = (b >= 0) ? (uint32_t)b : (uint32_t)(-b);
    uint32_t uc = (c >= 0) ? (uint32_t)c : (uint32_t)(-c);
    uint32_t ud = (d >= 0) ? (uint32_t)d : (uint32_t)(-d);

    uint32_t max = ua;
    if (ub > max) max = ub;
    if (uc > max) max = uc;
    if (ud > max) max = ud;
    return max;
}

/*======================================================================*/
/*  _WaitComplete — 阻塞等待 TIM3 电机运动结束                            */
/*======================================================================*/
static void _WaitComplete(void)
{
    /* 轮询直到 stepPosition 达到 distance (由 TIM3 ISR 递增) */
    while (1) {
        if ((uint32_t)stepPosition >= (uint32_t)distance) {
            break;
        }
    }
}

/*======================================================================*/
/*  _DriveAndWait — 驱动 + 等待完成                                      */
/*======================================================================*/
static void _DriveAndWait(const ChassisSteps *steps)
{
    uint32_t max_steps;
    uint32_t target_distance;

    /* 1. 设置方向 */
    _SetDirection(steps);

    /* 2. 取最大脉冲数（四轮共用 TIM3, 必须同步） */
    max_steps = _MaxAbs4(steps->step_fl, steps->step_fr,
                         steps->step_rl, steps->step_rr);

    if (max_steps == 0) return;  /* 无需移动 */

    target_distance = max_steps;

    /* 3. 清零步数计数器并启动 S曲线电机运动 */
    stepPosition = 0;
    distance     = (int)target_distance;
    MSD_Move((signed int)target_distance, 28, 28, 48);

    /* 4. 阻塞等待完成 */
    _WaitComplete();

    /* 5. 更新里程计（使用实际驱动的脉冲，即 max_steps）*/
    {
        ChassisSteps actual;
        /* 记录各轮实际走的脉冲: 符号保持, 幅度统一为 max_steps */
        actual.step_fl = (steps->step_fl >= 0) ? (int32_t)max_steps : -(int32_t)max_steps;
        actual.step_fr = (steps->step_fr >= 0) ? (int32_t)max_steps : -(int32_t)max_steps;
        actual.step_rl = (steps->step_rl >= 0) ? (int32_t)max_steps : -(int32_t)max_steps;
        actual.step_rr = (steps->step_rr >= 0) ? (int32_t)max_steps : -(int32_t)max_steps;

        CHASSIS_OdomAdd(&g_odom, &actual);
    }

    /* 6. 正运动学更新位姿 */
    {
        ChassisPose ref = g_pose;
        CHASSIS_Odom(&g_odom, &ref, &g_pose);
    }
}

/*======================================================================*/
/*  PUBLIC API                                                          */
/*======================================================================*/

/**
 * @brief  初始化底盘驱动
 */
void CHASSIS_DRIVER_Init(const ChassisConfig *cfg)
{
    /* 委托底盘算法模块初始化 */
    CHASSIS_Init(cfg);

    /* 位姿归零 */
    g_pose.x     = 0.0f;
    g_pose.y     = 0.0f;
    g_pose.theta = 0.0f;

    /* 里程计归零 */
    CHASSIS_OdomReset(&g_odom);
}

/**
 * @brief  定点全向移动: 当前位置 → 目标位姿
 */
void CHASSIS_DRIVER_GoTo(float x_mm, float y_mm, float theta_rad)
{
    ChassisPose tgt;
    ChassisSteps delta;

    tgt.x     = x_mm;
    tgt.y     = y_mm;
    tgt.theta = theta_rad;

    /* 逆运动学解算 */
    CHASSIS_MoveTo(&g_pose, &tgt, &delta);

    /* 驱动电机并等待 */
    _DriveAndWait(&delta);

    /* 陀螺仪角度校准 (利用外部 global_angle / base_angle) */
    MOTOR_Align();

    /* 校准后同步位姿角度 = 目标角度 (陀螺仪校准已修正物理偏差) */
    g_pose.theta = theta_rad;

    /* 如果有平移，用里程计结果覆盖 */
    {
        ChassisPose ref = g_pose;
        ref.theta = theta_rad;  /* 角度以陀螺仪为准 */
        CHASSIS_Odom(&g_odom, &ref, &g_pose);
    }
}

/**
 * @brief  旋转到绝对角度
 */
void CHASSIS_DRIVER_TurnTo(float theta_rad)
{
    CHASSIS_DRIVER_GoTo(g_pose.x, g_pose.y, theta_rad);
}

/**
 * @brief  相对平移 (底盘坐标系)
 */
void CHASSIS_DRIVER_MoveRel(float dx_mm, float dy_mm)
{
    float c = cosf(g_pose.theta);
    float s = sinf(g_pose.theta);

    /* 底盘坐标系 → 世界坐标系 */
    float wx = g_pose.x + dx_mm * c - dy_mm * s;
    float wy = g_pose.y + dx_mm * s + dy_mm * c;

    CHASSIS_DRIVER_GoTo(wx, wy, g_pose.theta);
}

/**
 * @brief  相对旋转 (原地)
 */
void CHASSIS_DRIVER_TurnRel(float dtheta_rad)
{
    CHASSIS_DRIVER_GoTo(g_pose.x, g_pose.y, g_pose.theta + dtheta_rad);
}

/**
 * @brief  重置当前位姿
 */
void CHASSIS_DRIVER_ResetPose(float x_mm, float y_mm, float theta_rad)
{
    g_pose.x     = x_mm;
    g_pose.y     = y_mm;
    g_pose.theta = theta_rad;
    CHASSIS_OdomReset(&g_odom);
}

/**
 * @brief  获取当前位姿
 */
void CHASSIS_DRIVER_GetPose(ChassisPose *out)
{
    if (out) *out = g_pose;
}

/**
 * @brief  获取里程计原始脉冲
 */
void CHASSIS_DRIVER_GetOdom(ChassisOdom *out)
{
    if (out) *out = g_odom;
}
