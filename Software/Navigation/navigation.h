/*
 * nagivation.h
 *
 *      Author: 陈信沅
 */
#include <stdbool.h>
#include <stdint.h>

#ifndef _NAVIGATION_H_
#define _NAVIGATION_H_


#define NAV_PI  3.14159265f

typedef struct {
    float X;        //  (m)
    float Y;        //  (m)
    float Theta;    // (rad)世界坐标

    float Vx_body;  //  
    float Vy_body;  //  (m/s)
    float Omega;    //  (rad/s), 

    float Vx_world; // 
    float Vy_world; //  (m/s)

    float mileage;      //  (m)
    float Theta_imu;    //  (rad)
    float Omega_imu;    // (rad/s)

    uint32_t last_tick_ms;
    uint8_t  is_inited;
    uint8_t  use_imu_fusion; //
} Nav_Odom_t;

extern Nav_Odom_t Nav_Odom;

void Nav_Odom_Init(void);
void Nav_Odom_Update(void);
void Nav_Odom_Reset(float x, float y, float theta);
Nav_Odom_t* Nav_Get_Odom(void);

/*================== 路径点 ==================*/

#define NAV_PATH_POINT_DIST  0.05f   // 路径点间隔 5cm
#define NAV_MAX_PATH_POINTS  2000    // 最大支持的路径点数 (约100m)
#define NAV_POINTS_PER_PAGE  150     // 每Flash页存储150个点 (150*3*4=1800 bytes)

typedef struct {
    float X;
    float Y;
    float Theta;
} NavPathPoint_t;

extern NavPathPoint_t Nav_Path[NAV_MAX_PATH_POINTS];
extern uint16_t Nav_PathCount;  // 当前有效路径点数

/*================== 导航控制参数 ==================*/

#define NAV_KP_LONG   2.0f   // 纵向偏差增益 (车头方向)
#define NAV_KP_LAT    3.0f   // 侧向偏差增益 (垂直车头方向,麦轮横移使用)
#define NAV_KP_ANGLE  2.5f   // 角度偏差增益
#define NAV_MAX_VEL   0.5f   // 导航输出速度限幅 (m/s)
#define NAV_MAX_OMEGA 2.0f   // 导航输出角速度限幅 (rad/s)
#define NAV_ARRIVE_DIST 0.03f // 到达判定距离 (3cm)

/*================== Flash 相关 ==================*/

#define MaxSize 500    //flash存储点数组数据页数 (兼容旧定义，实际使用NAV_POINTS_PER_PAGE)

#define Read_MaxSize 10000//读取预设值

#define Nag_End_Page 1
#define Nag_Start_Page 45

/*================== 导航状态结构体 ==================*/

typedef struct{
    // 控制输出 (供上层或直接调用底盘)
    float Final_Out;    // 角度偏差 (兼容旧接口, rad)
    float ctrl_vx;      // 侧向速度输出 (m/s)
    float ctrl_vy;      // 纵向速度输出 (m/s)
    float ctrl_omega;   // 旋转角速度输出 (rad/s)

    // 路径跟踪状态
    bool Nag_Stop_f;
    uint8_t Flash_read_f;
    uint16_t size;
    uint16_t Run_index;
    uint16_t Save_count;
    uint16_t Save_index;    // 总路径点数
    uint8_t Save_state;
    uint8_t End_f;

    // Flash相关
    uint8_t Flash_page_index;
    uint8_t Flash_Save_Page_Index;
    uint8_t Nag_SystemRun_Index;

    // 记录/跟踪辅助
    float last_record_X;
    float last_record_Y;
    uint16_t track_target_idx;  // 当前跟踪的目标点索引
} Nag;

extern Nag N;

/*================== 导航接口 ==================*/

void Nag_Run(void);         // 导航主控函数 (记录或跟踪)
void Run_Nag_GPS(void);     // 路径跟踪更新
void Run_Nag_Save(void);    // 路径记录更新
void Nag_Read(void);        // 导航状态机
void Init_Nag(void);        // 导航初始化
void Nag_System(void);      // 导航系统调度
void NagFlashRead(void);    // 从Flash加载路径

#endif /* _NAVIGATION_H_ */
