/*
* navigation.c
*
*  Created on: 2026年8月10日
*      Author: 陈信沅
*
*  适配麦克纳姆轮全向小车导航
*  1. 导航位姿(Nav_Odom_t)依然由chassis.c传递至此，统一管理世界坐标X,Y,Theta
*  2. IMU融合：使用IMU原始角度直接替换轮速积分的航向角，消除漂移
*  3. 路径记录：由存储"里程+yaw"改为存储"X,Y,Theta"路径点，支持横移/斜走记录
*  4. 路径跟踪：麦克纳姆全向控制，同时输出vx/vy/omega，利用横移修正侧向偏差
*/

#include "stm32f4xx_hal.h"
#include "navigation.h"
#include "kalman.h"
#include "Chassis.h"
#include "flash.h"
#include <math.h>
#include <string.h>

/*================== 全局变量 ==================*/

Nav_Odom_t Nav_Odom;
NavPathPoint_t Nav_Path[NAV_MAX_PATH_POINTS];
uint16_t Nav_PathCount = 0;

Nag N;

// 兼容旧接口，但不再作为主要存储
int32_t Nav_read[Read_MaxSize];

/*================== Flash 页面大小修正 ==================*/
#define FLASH_PAGE_SIZE  2048
#define NAV_LOOK_AHEAD_DIST 0.05f

/*================== 导航位姿估计 ==================*/

/**
 * @brief 角度归一化到 [-PI, +PI]
 */
static float nav_angle_norm(float angle)
{
    while (angle > NAV_PI)  angle -= 2.0f * NAV_PI;
    while (angle < -NAV_PI) angle += 2.0f * NAV_PI;
    return angle;
}

/**
 * @brief 底盘坐标系速度 -> 世界坐标系速度
 */
static void nav_body_to_world(float vx, float vy, float theta,
                               float *vx_w, float *vy_w)
{
    float c = cosf(theta);
    float s = sinf(theta);
    *vx_w = vx * c - vy * s;
    *vy_w = vx * s + vy * c;
}

void Nav_Odom_Init(void)
{
    memset(&Nav_Odom, 0, sizeof(Nav_Odom_t));
    Nav_Odom.last_tick_ms = HAL_GetTick();
    Nav_Odom.is_inited = 1;
    Nav_Odom.use_imu_fusion = 1; //使用imu融合(默认使用imu融合)
}

void Nav_Odom_Reset(float x, float y, float theta)
{
    Nav_Odom.X = x;
    Nav_Odom.Y = y;
    Nav_Odom.Theta = theta;
    Nav_Odom.Vx_body = 0.0f;
    Nav_Odom.Vy_body = 0.0f;
    Nav_Odom.Omega = 0.0f;
    Nav_Odom.Vx_world = 0.0f;
    Nav_Odom.Vy_world = 0.0f;
    Nav_Odom.mileage = 0.0f;
    Nav_Odom.last_tick_ms = HAL_GetTick();
}

Nav_Odom_t* Nav_Get_Odom(void)
{
    return &Nav_Odom;
}

/**
 * @brief 
 * @note  
 *
 * 【流程】
 *  1. 调用 Chassis_GetBodySpeed() 获取底盘坐标系速度 (vx_body, vy_body)
 *  2. 调用 Kalman_GetYawRad() / Kalman_GetYawOmegaRad() 获取IMU数据
 *  3. 融合: Theta/Omega 使用IMU (无漂移), 位置由轮速积分
 *  4. 坐标转换: 底盘速度 -> 世界坐标系速度 (使用融合后的Theta)
 *  5. 积分更新 X, Y, mileage
 */
void Nav_Odom_Update(void)
{
    if (!Nav_Odom.is_inited) {
        Nav_Odom_Init();
        return;
    }

    uint32_t now_ms = HAL_GetTick();
    float dt = (now_ms - Nav_Odom.last_tick_ms) * 0.001f;//单位换算ms-->s

    if (dt > 0.1f || dt <= 0.0f) {
        Nav_Odom.last_tick_ms = now_ms;
        return;
    }// 过长的dt可能是系统卡顿或首次调用，直接跳过积分

    // 1.获取底盘坐标系速度
    float vx_body, vy_body, omega_wheel;
    Chassis_GetBodySpeed(&vx_body, &vy_body, &omega_wheel);

    Nav_Odom.Vx_body = vx_body;
    Nav_Odom.Vy_body = vy_body;
    // 2.获取imu数据
    float theta_imu = Kalman_GetYawRad();
    float omega_imu = Kalman_GetYawOmegaRad();

    Nav_Odom.Theta_imu = theta_imu;
    Nav_Odom.Omega_imu = omega_imu;

    // 3. 融合数据（角速度和角度用imu获取）
    if (Nav_Odom.use_imu_fusion) {
        Nav_Odom.Theta = theta_imu;
        Nav_Odom.Omega = omega_imu;
    } else {
        Nav_Odom.Omega = omega_wheel;
        Nav_Odom.Theta = nav_angle_norm(Nav_Odom.Theta + omega_wheel * dt);
    }

    // 4. 坐标转换: 底盘 -> 世界
    float vx_world, vy_world;
    nav_body_to_world(vx_body, vy_body, Nav_Odom.Theta, &vx_world, &vy_world);

    Nav_Odom.Vx_world = vx_world;
    Nav_Odom.Vy_world = vy_world;

    // 5. 积分更新位姿
    float dx = vx_world * dt;
    float dy = vy_world * dt;

    Nav_Odom.X += dx;
    Nav_Odom.Y += dy;
    Nav_Odom.mileage += sqrtf(dx * dx + dy * dy);

    Nav_Odom.last_tick_ms = now_ms;
}

/*================== 路径记录 ==================*/

/**
 * @brief 路径记录更新
 * @note  每行走 NAV_PATH_POINT_DIST (5cm) 记录一个 (X,Y,Theta) 路径点
 *
 * 【与旧代码区别】
 *  旧: 每行走固定里程记录一个 yaw 角 (假设全程前进)
 *  新: 使用欧式距离判断，记录完整位姿 (X,Y,Theta), 支持横移/斜走
 */
void Run_Nag_Save(void)
{
    Nav_Odom_t* odom = &Nav_Odom;

    // 计算与上一个记录点的欧式距离
    float dx = odom->X - N.last_record_X;
    float dy = odom->Y - N.last_record_Y;
    float dist = sqrtf(dx * dx + dy * dy);

    if (dist >= NAV_PATH_POINT_DIST) {
        if (Nav_PathCount >= NAV_MAX_PATH_POINTS) {
            // 路径点已满，标记结束
            N.End_f = 1;
            return;
        }

        // 记录新路径点到 RAM
        Nav_Path[Nav_PathCount].X = odom->X;
        Nav_Path[Nav_PathCount].Y = odom->Y;
        Nav_Path[Nav_PathCount].Theta = odom->Theta;

        // 同时写入 flash_union_buffer (按页组织)
        uint16_t page_offset = Nav_PathCount % NAV_POINTS_PER_PAGE;
        flash_union_buffer[page_offset * 3 + 0] = *(uint32_t*)&odom->X;
        flash_union_buffer[page_offset * 3 + 1] = *(uint32_t*)&odom->Y;
        flash_union_buffer[page_offset * 3 + 2] = *(uint32_t*)&odom->Theta;

        Nav_PathCount++;
        N.Save_index = Nav_PathCount;
        N.last_record_X = odom->X;
        N.last_record_Y = odom->Y;

        // 当前页面写满，写入Flash (结束时的写入由 Nag_Run/Nag_Read 统一处理)
        if ((Nav_PathCount % NAV_POINTS_PER_PAGE) == 0) {
            flash_Nag_Write();
        }
    }
}

/*================== 路径跟踪 ==================*/

/**
 * @brief 找到距离当前位置最近的路径点索引
 */
static uint16_t nav_find_nearest_point(float x, float y)
{
    uint16_t nearest_idx = N.track_target_idx;
    float min_dist_sq = 1e10f;

    // 从当前跟踪索引附近搜索，避免回环
    uint16_t start = N.track_target_idx;
    uint16_t end = Nav_PathCount;
    if (end > start + 50) end = start + 50; // 最多向前看50个点

    for (uint16_t i = start; i < end; i++) {
        float dx = Nav_Path[i].X - x;
        float dy = Nav_Path[i].Y - y;
        float dist_sq = dx * dx + dy * dy;
        if (dist_sq < min_dist_sq) {
            min_dist_sq = dist_sq;
            nearest_idx = i;
        }
    }
    return nearest_idx;
}

/**
 * @brief 找到前视目标点索引
 * @note  从最近点向前搜索，找到距离当前位置大于等于 NAV_LOOK_AHEAD_DIST 的点
 */
static uint16_t nav_find_lookahead_point(uint16_t nearest_idx, float x, float y)
{
    float look_ahead_sq = NAV_LOOK_AHEAD_DIST * NAV_LOOK_AHEAD_DIST;

    for (uint16_t i = nearest_idx; i < Nav_PathCount; i++) {
        float dx = Nav_Path[i].X - x;
        float dy = Nav_Path[i].Y - y;
        float dist_sq = dx * dx + dy * dy;
        if (dist_sq >= look_ahead_sq) {
            return i;
        }
    }
    // 如果到终点都不足前视距离，返回最后一个点
    return (Nav_PathCount > 0) ? (Nav_PathCount - 1) : 0;
}

/**
 * @brief 路径跟踪控制更新 (麦克纳姆全向)
 * @note  计算位置偏差和角度偏差，输出 vx/vy/omega 三个控制量
 *
 * 【控制策略】
 *  1. 找到最近路径点 + 前视点
 *  2. 计算世界坐标系偏差 (dx, dy, dtheta)
 *  3. 转换到底盘坐标系偏差:
 *       err_long =  dx*cosθ + dy*sinθ   (车头方向偏差)
 *       err_lat  = -dx*sinθ + dy*cosθ   (侧向偏差, 麦克纳姆可直接横移修正)
 *  4. P控制输出:
 *       vy = KP_LONG * err_long
 *       vx = KP_LAT  * err_lat          -> 麦克纳姆优势: 直接横移!
 *       omega = KP_ANGLE * dtheta
 *  5. 速度限幅后输出
 */
void Run_Nag_GPS(void)
{
    Nav_Odom_t* odom = &Nav_Odom;

    if (Nav_PathCount == 0 || N.track_target_idx >= Nav_PathCount) {
        // 无路径或已经跑完
        N.Nag_Stop_f = true;
        N.ctrl_vx = 0.0f;
        N.ctrl_vy = 0.0f;
        N.ctrl_omega = 0.0f;
        return;
    }

    // 1. 找到最近点和前视点
    uint16_t nearest_idx = nav_find_nearest_point(odom->X, odom->Y);
    uint16_t target_idx = nav_find_lookahead_point(nearest_idx, odom->X, odom->Y);
    N.track_target_idx = nearest_idx;  // 更新当前进度

    NavPathPoint_t* target = &Nav_Path[target_idx];

    // 2. 计算世界坐标系偏差
    float dx = target->X - odom->X;
    float dy = target->Y - odom->Y;
    float dtheta = target->Theta - odom->Theta;
    dtheta = nav_angle_norm(dtheta);

    // 判断是否到达终点
    float dist_to_end_x = Nav_Path[Nav_PathCount - 1].X - odom->X;
    float dist_to_end_y = Nav_Path[Nav_PathCount - 1].Y - odom->Y;
    float dist_to_end = sqrtf(dist_to_end_x * dist_to_end_x + dist_to_end_y * dist_to_end_y);

    if (dist_to_end < NAV_ARRIVE_DIST && target_idx >= Nav_PathCount - 1) {
        N.Nag_Stop_f = true;
        N.ctrl_vx = 0.0f;
        N.ctrl_vy = 0.0f;
        N.ctrl_omega = 0.0f;
        N.Final_Out = 0.0f;
        return;
    }

    // 3. 转换到底盘坐标系偏差
    float c = cosf(odom->Theta);
    float s = sinf(odom->Theta);
    float err_long =  dx * c + dy * s;   // 纵向偏差 (车头方向)
    float err_lat  = -dx * s + dy * c;   // 侧向偏差 (垂直车头)

    // 4. P控制计算控制量
    float vy_cmd = NAV_KP_LONG * err_long;
    float vx_cmd = NAV_KP_LAT  * err_lat;
    float omega_cmd = NAV_KP_ANGLE * dtheta;

    // 5. 速度限幅
    float v_norm = sqrtf(vx_cmd * vx_cmd + vy_cmd * vy_cmd);
    if (v_norm > NAV_MAX_VEL) {
        float scale = NAV_MAX_VEL / v_norm;
        vx_cmd *= scale;
        vy_cmd *= scale;
    }

    if (omega_cmd >  NAV_MAX_OMEGA) omega_cmd =  NAV_MAX_OMEGA;
    if (omega_cmd < -NAV_MAX_OMEGA) omega_cmd = -NAV_MAX_OMEGA;

    // 6. 输出到导航结构体
    N.ctrl_vx = vx_cmd;
    N.ctrl_vy = vy_cmd;
    N.ctrl_omega = omega_cmd;
    N.Final_Out = dtheta;  // 兼容旧接口
}

/*================== 导航主控 ==================*/

void Nag_Run(void)
{
    if (N.End_f == 1 && N.Save_state == 0) {
        // 记录结束，保存最后数据
        flash_Nag_Write();
        N.Save_state = 1;
        return;
    }

    if (N.Nag_Stop_f) {
        N.ctrl_vx = 0.0f;
        N.ctrl_vy = 0.0f;
        N.ctrl_omega = 0.0f;
        N.Final_Out = 0.0f;
        Chassis_stop();
        return;
    }

    // 根据运行模式执行记录或者跟踪
    if (N.Save_state == 0 && N.End_f == 0) {
        // 记录模式
        Run_Nag_Save();
    } else {
        // 跟踪/回放模式
        Run_Nag_GPS();
        // 直接输出控制量到底盘 (麦克纳姆全向控制)
        Chassis_Move(N.ctrl_vx, N.ctrl_vy, N.ctrl_omega);
    }
}

void Nag_Read(void)
{
    switch(N.End_f)
    {
        case 0: Run_Nag_Save();
            break;
        case 1:
            flash_Nag_Write();
            N.End_f++;
            break;
        case 2:
            N.End_f++;
            break;
    }
}

/*================== 初始化 ==================*/

void Init_Nag(void)
{
    memset(&N, 0, sizeof(N));
    N.Flash_page_index = Nag_Start_Page * FLASH_PAGE_SIZE;  // 修正: 字节偏移
    flash_buffer_clear();
    Nav_PathCount = 0;
    N.last_record_X = 0.0f;
    N.last_record_Y = 0.0f;
    N.track_target_idx = 0;
}

/*================== 系统调度 ==================*/

void Nag_System(void)
{
    if (!N.Nag_SystemRun_Index || N.Nag_Stop_f) return;

    switch(N.Nag_SystemRun_Index)
    {
        case 1: Nag_Read();
            break;
        case 3: Nag_Run();
            break;
    }
}

/*================== Flash读写适配 ==================*/

/**
 * @brief 从Flash加载路径点到RAM
 * @note 一次性读取所有页面的路径点，解析存入 Nav_Path[] 数组
 */
void NagFlashRead(void)
{
    if (N.Save_state) return;

    // 第0页面存放 Save_index (总点数), 先读出来
    flash_buffer_clear();
    flash_read_page_to_buffer(0, FLASH_PAGE_SIZE);
    N.Save_index = flash_union_buffer[MAX_SIZE + 2];  // 存放在第102位置
    uint16_t total_points = N.Save_index;
    if (total_points > NAV_MAX_PATH_POINTS) total_points = NAV_MAX_PATH_POINTS;

    flash_buffer_clear();

    uint16_t points_loaded = 0;
    uint16_t page = Nag_Start_Page;  // 页号从45开始递减

    while (points_loaded < total_points && page >= Nag_End_Page) {
        uint32_t offset = page * FLASH_PAGE_SIZE;

        if (!flash_check(FLASH_USER_START_ADDR + offset)) {
            // 该页面有数据，读取
            flash_read_page_to_buffer(offset, FLASH_PAGE_SIZE);

            // 解析路径点 (每个点3个float)
            for (uint16_t i = 0; i < NAV_POINTS_PER_PAGE && points_loaded < total_points; i++) {
                Nav_Path[points_loaded].X     = *(float*)&flash_union_buffer[i * 3 + 0];
                Nav_Path[points_loaded].Y     = *(float*)&flash_union_buffer[i * 3 + 1];
                Nav_Path[points_loaded].Theta = *(float*)&flash_union_buffer[i * 3 + 2];
                points_loaded++;
            }
            flash_buffer_clear();
        }
        page--;
    }

    Nav_PathCount = points_loaded;
    N.Save_state = 1;
    N.Nag_SystemRun_Index++;
}

/*================== Flash读写 (详见 flash.c) ==================*/
// flash_Nag_Write() 和 flash_Nag_Read() 在 flash.c 中实现,
// 已修正页偏移bug并适配新路径点格式