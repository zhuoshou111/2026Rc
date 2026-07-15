/*======================================================================*/
/*  四轮麦克纳姆轮定点全向移动 — 步进电机纯算法                           */
/*                                                                      */
/*  轮序布局（俯视图）：                                                */
/*                                                                      */
/*        FRONT                                                          */
/*    FL ───── FR      FL: 左前    FR: 右前                             */
/*    │    ·    │       RL: 左后    RR: 右后                             */
/*    │  (0,0)  │       · = 底盘几何中心                                */
/*    RL ───── RR                                                        */
/*                                                                      */
/*  轮子弧长 → 脉冲公式：                                              */
/*    pulse = arc_mm / (2 * PI * R) * steps_per_rev                     */
/*                                                                      */
/*  逆运动学（位移形式）：                                              */
/*    ΔL_FL = Δx - Δy - Δθ*(Lx+Ly)    (mm)                              */
/*    ΔL_FR = Δx + Δy + Δθ*(Lx+Ly)    (mm)                              */
/*    ΔL_RL = Δx + Δy - Δθ*(Lx+Ly)    (mm)                              */
/*    ΔL_RR = Δx - Δy + Δθ*(Lx+Ly)    (mm)                              */
/*                                                                      */
/*  正运动学（里程推算）：                                              */
/*    ΔL_i = pulse_i / steps_per_rev * 2*PI*R   (每个轮子的弧长)        */
/*    Δx = (ΔL_FL + ΔL_FR + ΔL_RL + ΔL_RR) / 4                          */
/*    Δy = (-ΔL_FL + ΔL_FR + ΔL_RL - ΔL_RR) / 4                         */
/*    Δθ = (-ΔL_FL + ΔL_FR - ΔL_RL + ΔL_RR) / (4*(Lx+Ly))              */
/*======================================================================*/

#include "chassis_mecanum.h"
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

/* ---- 模块级全局变量 ---- */
static ChassisConfig g_cfg;
static float g_mm_per_step;    /* 每个脉冲对应的轮子弧长 (mm) */
static float g_step_per_mm;    /* 每 mm 对应的脉冲数         */

/*======================================================================*/
/*  CHASSIS_Init                                                        */
/*======================================================================*/
void CHASSIS_Init(const ChassisConfig *cfg)
{
    g_cfg = *cfg;

    /* 预计算：一圈弧长 / 一圈脉冲 = mm/step */
    float circum = 2.0f * (float)M_PI * g_cfg.wheel_radius;
    g_mm_per_step = circum / (float)g_cfg.steps_per_rev;
    g_step_per_mm = (float)g_cfg.steps_per_rev / circum;
}

/*======================================================================*/
/*  arc_to_steps — 轮子弧长(mm) → 脉冲数                                */
/*======================================================================*/
static int32_t arc_to_steps(float arc_mm)
{
    /* 四舍五入取整，保证累计不丢步 */
    return (int32_t)(arc_mm * g_step_per_mm + ((arc_mm >= 0.0f) ? 0.5f : -0.5f));
}

/*======================================================================*/
/*  steps_to_arc — 脉冲数 → 轮子弧长(mm)                                */
/*======================================================================*/
static float steps_to_arc(int32_t steps)
{
    return (float)steps * g_mm_per_step;
}

/*======================================================================*/
/*  CHASSIS_MoveTo — 定点全向移动核心                                   */
/*======================================================================*/
void CHASSIS_MoveTo(const ChassisPose *cur, const ChassisPose *tgt, ChassisSteps *out)
{
    /* 1. 世界坐标系下的位移差 */
    float dx_w = tgt->x - cur->x;
    float dy_w = tgt->y - cur->y;
    float dth  = tgt->theta - cur->theta;

    /* 2. 将位移旋转到当前底盘坐标系 */
    float c = cosf(cur->theta);
    float s = sinf(cur->theta);
    float dx =  dx_w * c + dy_w * s;   /* 底盘前方 */
    float dy = -dx_w * s + dy_w * c;   /* 底盘左方 */

    /* 3. 逆运动学：底盘位移 → 各轮弧长 */
    float L = g_cfg.half_length + g_cfg.half_width;

    float arc_fl = dx - dy - dth * L;  /* 前左 */
    float arc_fr = dx + dy + dth * L;  /* 前右 */
    float arc_rl = dx + dy - dth * L;  /* 后左 */
    float arc_rr = dx - dy + dth * L;  /* 后右 */

    /* 4. 弧长 → 脉冲 */
    out->step_fl = arc_to_steps(arc_fl);
    out->step_fr = arc_to_steps(arc_fr);
    out->step_rl = arc_to_steps(arc_rl);
    out->step_rr = arc_to_steps(arc_rr);
}

/*======================================================================*/
/*  CHASSIS_Odom — 里程计推算                                           */
/*======================================================================*/
void CHASSIS_Odom(const ChassisOdom *odom, const ChassisPose *ref_pose, ChassisPose *out)
{
    float arc_fl = steps_to_arc(odom->total_fl);
    float arc_fr = steps_to_arc(odom->total_fr);
    float arc_rl = steps_to_arc(odom->total_rl);
    float arc_rr = steps_to_arc(odom->total_rr);

    float L = g_cfg.half_length + g_cfg.half_width;

    /* 底盘坐标系下的位移增量 */
    float dx_local =  (arc_fl + arc_fr + arc_rl + arc_rr) / 4.0f;
    float dy_local =  (-arc_fl + arc_fr + arc_rl - arc_rr) / 4.0f;
    float dth      =  (-arc_fl + arc_fr - arc_rl + arc_rr) / (4.0f * L);

    /* 旋转回世界坐标系 */
    float mid_theta = ref_pose->theta + dth / 2.0f;  /* 用中值角度近似 */
    float c = cosf(mid_theta);
    float s = sinf(mid_theta);

    out->x     = ref_pose->x     + dx_local * c - dy_local * s;
    out->y     = ref_pose->y     + dx_local * s + dy_local * c;
    out->theta = ref_pose->theta + dth;
}

/*======================================================================*/
/*  CHASSIS_OdomAdd                                                     */
/*======================================================================*/
void CHASSIS_OdomAdd(ChassisOdom *odom, const ChassisSteps *delta)
{
    odom->total_fl += delta->step_fl;
    odom->total_fr += delta->step_fr;
    odom->total_rl += delta->step_rl;
    odom->total_rr += delta->step_rr;
}

/*======================================================================*/
/*  CHASSIS_OdomReset                                                   */
/*======================================================================*/
void CHASSIS_OdomReset(ChassisOdom *odom)
{
    odom->total_fl = 0;
    odom->total_fr = 0;
    odom->total_rl = 0;
    odom->total_rr = 0;
}
