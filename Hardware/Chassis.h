#ifndef __ASFL_CAR_CHASSIS_H__
#define __ASFL_CAR_CHASSIS_H__
#include "main.h"

/*================== 常量定义 ==================*/
#define CHASSIS_PI          3.14159265f
#define CHASSIS_WIDTH       0.160f       // 左右轮距 (m)
#define CHASSIS_LENGTH      0.1725f       // 前后轮距 (m)
#define CHASSIS_WHEEL_DIA   0.08f       // 轮子直径 (m)

/* Number of complete four-motor updates rejected because the TX queue was busy. */
extern volatile uint32_t Chassis_TxRejectedBatches;

/*================== 底盘运动控制接口 ==================*/

/**
 * @brief 初始化底盘
 * @param _USART 串口句柄, 波特率需为 115200
 */
void Chassis_Init(UART_HandleTypeDef *_USART);

/**
 * @brief Service the motor TX queue.
 * @note No motor feedback is received; this only advances queued TX frames.
 */
void Chassis_Process(void);

/**
 * @brief 设置默认速度 (m/s), 用于位置控制
 */
void Chassis_Setdefaultspeed(float v);

/**
 * @brief 底盘速度控制 (麦轮精确解算, 统一接口)
 * @param vx    横向速度 (m/s), 右侧为正
 * @param vy    纵向速度 (m/s), 前方为正
 * @param omega 旋转角速度 (rad/s), 逆时针为正
 *
 * @note  这是统一的速度控制接口
 */
void Chassis_Move(float vx, float vy, float omega);

/**
 * @brief 差速循迹速度模式
 * @param forward_speed 前进线速度 (m/s)，前方为正
 * @param pid_output    灰度 PID 输出，作为左右轮转速差 (RPM)
 * @note  pid_output 为正时向右修正；左侧为电机 1/3，右侧为电机 2/4。
 */
void Chassis_TrackDifferential(float forward_speed, float pid_output);

/**
 * @brief 底盘位置控制 (麦轮精确解算, 统一接口)
 * @param sx        横向位移 (m), 右侧为正
 * @param sy        纵向位移 (m), 前方为正
 * @param theta_deg 旋转角度 (deg), 逆时针为正
 * @retval 预计执行时间 (ms)；发送队列无法接收完整命令组时返回 -1
 */
float Chassis_MovePos(float sx, float sy, float theta_deg);

/**
 * @brief 停止底盘
 */
void Chassis_stop(void);

/**
 * @brief 直接设置四轮速度 (底层接口)
 */
void Chassis_Set4MotorSpeed(float v1, float v2, float v3, float v4);

/*================== 状态查询 ==================*/

/**
 * @brief 获取单个电机线速度
 */
float Chassis_GetLinearSpeed(uint8_t motor_ID);

/**
 * @brief 获取四轮线速度到 Car 结构体
 */
void Chassis_GetSpeed(void);

/**
 * @brief 实时正解算获取底盘坐标系速度
 * @param vx    输出底盘横向速度 (m/s), 右侧为正
 * @param vy    输出底盘纵向速度 (m/s), 前方为正
 * @param omega 输出旋转角速度 (rad/s), 逆时针为正
 *
 * @note  该函数实时读取四轮速度并做正解算, 不依赖任何全局状态。
 *        返回的是底盘坐标系下的瞬时速度, 供惯导模块调用。
 */
void Chassis_GetBodySpeed(float *vx, float *vy, float *omega);

/*================== 测试/任务函数 ==================*/

void Chassis_Test(void);


/*================== 状态机辅助函数 ==================*/
void Chassis_OpenLoopMove(float vx, float vy, float omega, uint32_t duration_ms);
uint32_t Chassis_MovePosBlocking(float sx, float sy, float theta_deg);
uint32_t Chassis_MovePosBlocking_Set_V(float sx, float sy, float theta_deg, float v);
void Chassis_StrafeLeftUntilLine(float speed, uint32_t timeout_ms);
void Chassis_StrafeRightUntilLine(float speed, uint32_t timeout_ms);


/*================== 对圆心(速度控制)模式 ==================*/
/* 对圆心用速度控制模式：上位机每隔固定时间下发 x/y 方向速度(m/s)，
 * 收到完成帧 AA 00 0A 后立刻停车。
 * 注释掉本宏则使用原来的位置控制模式（上位机回传 x/y 位置偏差，Control_AlignTest）。 */
#define CIRCLE_ALIGN_SPEED_MODE

/* 对圆心速度控制的两种实现方式（二选一，用于对比效果）：
 *   定义   CIRCLE_ALIGN_USE_ANGLE_HOLD -> 用 Control_AngleHoldMove（速度控制 + 保持航向）
 *   注释掉                           -> 用 Chassis_Move（纯速度控制，不纠航向） */
#define CIRCLE_ALIGN_USE_ANGLE_HOLD

/**
 * @brief 对圆心 - 速度控制模式（阻塞式与上位机通信）
 * @param max_speed_mps 上位机速度指令限幅(m/s)，防止数值异常导致飞车
 * @param hold_yaw_deg  对圆心期间要保持的绝对航向(deg)，通常与前面的 Control_StaticTurn 目标一致
 * @note  本机每 100ms 发一次请求帧 AA 02 0A，上位机收到后回传 x/y 方向速度(m/s)
 *        （帧格式 AA 08 <vx:4B> <vy:4B> 0A；vx/vy 为机体(相机)坐标系，右/前为正）；
 *        速度输出由 CIRCLE_ALIGN_USE_ANGLE_HOLD 宏选择：
 *          定义 -> Control_AngleUpdate 角度环输出 omega + 直接下发 vx/vy（机体坐标系）；
 *          注释 -> Chassis_Move（纯速度，hold_yaw_deg 忽略）。
 *        收到完成帧 AA 00 0A（中断把 circle_flag 置 0）后立刻 Chassis_stop() 并回传 CIRCLE_DONE。
 */
void Circle_AlignBySpeed(float max_speed_mps, float hold_yaw_deg);

#endif
