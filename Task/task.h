#ifndef __TASK_H__
#define __TASK_H__

#include "main.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ==================== 比赛任务搬运方案（二维码 → 搬运计划） ====================
 *
 * 规则依据: 《2026年度(专项赛省赛)赛事规则(工程竞技类赛项车型智能搬运项目)》
 *   任务1: 5个颜色物块(黑/白/红/绿/蓝)，二维码编号 1~16，方案表见规则附录A
 *   任务2: 3个奖杯(A/B/C)，二维码编号 1~6，方案表见规则附录B
 *
 * 使用说明:
 *   调用者输入【二维码编号】和【现场实际摆放顺序】：
 *     - 任务1 颜色物块: 从左到右读入(如 "蓝白黑红绿" 或字母 "UWBGR")
 *     - 任务2 奖杯:     从右到左读入(如 "CBA")
 *   函数按照二维码方案表，计算每一步该去几号槽位抓取，填入 plan[]。
 *
 *   颜色字母表示(输入/打印通用): B=黑  W=白  R=红  G=绿  U=蓝
 *   (也可直接输入中文"黑白红绿蓝"，需UTF-8编码，与本工程文件编码一致)
 */

#define TASK1_QR_MAX     16U   /* 任务1 二维码编号范围 1~16 */
#define TASK2_QR_MAX     6U    /* 任务2 二维码编号范围 1~6  */

/* 返回值 */
#define TASK_PLAN_OK          0U   /* 计算成功 */
#define TASK_PLAN_ERR_QR      1U   /* 二维码编号越界 */
#define TASK_PLAN_ERR_INPUT   2U   /* 摆放顺序非法(数量/未知字符/重复) */

/* 任务1: 根据二维码编号 + 现场颜色摆放顺序(从左到右)，计算搬运(抓取)方案
 * 输入:
 *   qr_code             : 任务1二维码编号 1~16
 *   colors_left_to_right: 5个颜色物块现场摆放顺序(从左到右)
 *                         支持中文"黑白红绿蓝"或字母"BWRGU"，字母不区分大小写
 * 输出:
 *   plan[5]: plan[i] = 方案表第 i 个颜色当前所在的槽位号(1~5)
 *            即机器人依次去 plan[0]→plan[1]→...→plan[4] 号槽抓取，
 *            抓到的颜色顺序正好等于二维码方案表的搬运顺序。
 * 返回: TASK_PLAN_OK / TASK_PLAN_ERR_QR / TASK_PLAN_ERR_INPUT
 */
uint8_t Task1_QRPlan(uint8_t qr_code, const char *colors_left_to_right, uint8_t plan[5]);

/* 便捷版: 直接输入每个槽位(1~5, 从左到右)的颜色编号数组，计算搬运(抓取)方案
 * 颜色编号: 0=黑(B) 1=白(W) 2=红(R) 3=绿(G) 4=蓝(U)
 * slot_colors[i] = 槽位 i+1 的颜色编号 (i=0~4 对应从左到右)
 * 输出 plan[5] 与 Task1_QRPlan 相同: plan[i] = 第 i 步去几号槽抓
 * 返回: TASK_PLAN_OK / TASK_PLAN_ERR_QR / TASK_PLAN_ERR_INPUT
 */
uint8_t Task1_QRPlanBySlotColors(uint8_t qr_code, const uint8_t slot_colors[5], uint8_t plan[5]);

/* 任务2: 根据二维码编号 + 现场奖杯摆放顺序(从右到左)，计算搬运(抓取)方案
 * 输入:
 *   qr_code          : 任务2二维码编号 1~6
 *   abc_right_to_left: 3个奖杯现场摆放顺序(从右到左)，如 "CBA"(右=C 中=B 左=A)
 * 输出:
 *   plan[3]: plan[i] = 方案表第 i 个奖杯当前所在的槽位号(1~3)
 * 说明: 放置目标固定为 A→冠军领奖台, B→亚军领奖台, C→季军领奖台
 * 返回: TASK_PLAN_OK / TASK_PLAN_ERR_QR / TASK_PLAN_ERR_INPUT
 */
uint8_t Task2_QRPlan(uint8_t qr_code, const char *abc_right_to_left, uint8_t plan[3]);

/* 取任务2方案表第 step 步对应的奖杯编号(0=A 1=B 2=C)；参数非法返回 0xFF
 * 用于执行层知道"这一步抓到的奖杯该放到哪个领奖台(A->冠军 B->亚军 C->季军)"
 */
uint8_t Task2_GetSchemeTrophy(uint8_t qr_code, uint8_t step);

/* 一键计算任务1 + 任务2 搬运方案 */
uint8_t Task_BuildPlan(uint8_t qr_task1, const char *colors_left_to_right,
                       uint8_t qr_task2, const char *abc_right_to_left,
                       uint8_t plan1[5], uint8_t plan2[3]);

/* 调试打印: 内部计算并在串口打印任务1/任务2搬运方案(依赖 uart_printf) */
void Task_PrintPlan(uint8_t qr_task1, const char *colors_left_to_right,
                    uint8_t qr_task2, const char *abc_right_to_left);

/* 演示: 打印一组示例方案，方便上电后快速验证 */
void Task_TestPlan(void);

/* ===== 任务1: 依次走5个点的固定移动距离（定义在 task.c，供状态机使用） ===== */
extern float task1_move_distance_X_m[5];   /* x 移动距离 (m) */
extern float task1_move_distance_Y_m[5];   /* y 移动距离 (m) */
extern bool is_align[5];                   /* true=对十字, false=对圆心 */

#ifdef __cplusplus
}
#endif

#endif /* __TASK_H__ */
