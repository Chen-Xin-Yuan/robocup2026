#include "task.h"
#include "usart_sent.h"   /* uart_printf 调试打印 */

#include <string.h>
#include <stdbool.h>

float task1_move_distance_X_m[5] = 
{
    -0.40f,
    0.12f,
    0.58f,
    0.01f,
    0.45f
};
float task1_move_distance_Y_m[5] = 
{
    0.42f,
    0.38f,
    -0.22f,
    0.19f,
    -0.02f
};

float task2_move_distance_X_m[3] = 
{
    -0.15f,
    -0.42f,
    -0.22f
};
float task2_move_distance_Y_m[3] = 
{
    0.0f,
    -0.58f,
    0.0f
};

bool is_align[5]=
{
    false,
    false,
    false,
    false,
    false
};


/* ==================== 方案表（来源于规则附录） ==================== */

/* 任务1 二维码对应搬运方案表(附录A): 每一行 = 该二维码编号对应的搬运顺序
 * 颜色编号: 0=黑(B) 1=白(W) 2=红(R) 3=绿(G) 4=蓝(U)
 */
static const uint8_t task1_scheme[16][5] = {
    {0, 1, 2, 3, 4},  /*  1: 黑 白 红 绿 蓝 */
    {1, 0, 2, 3, 4},  /*  2: 白 黑 红 绿 蓝 */
    {1, 0, 3, 2, 4},  /*  3: 白 黑 绿 红 蓝 */
    {4, 1, 0, 2, 3},  /*  4: 蓝 白 黑 红 绿 */
    {1, 2, 4, 0, 3},  /*  5: 白 红 蓝 黑 绿 */
    {0, 2, 4, 1, 3},  /*  6: 黑 红 蓝 白 绿 */
    {4, 3, 0, 1, 2},  /*  7: 蓝 绿 黑 白 红 */
    {3, 1, 4, 0, 2},  /*  8: 绿 白 蓝 黑 红 */
    {1, 3, 0, 4, 2},  /*  9: 白 绿 黑 蓝 红 */
    {0, 2, 4, 3, 1},  /* 10: 黑 红 蓝 绿 白 */
    {2, 4, 3, 0, 1},  /* 11: 红 蓝 绿 黑 白 */
    {3, 2, 0, 4, 1},  /* 12: 绿 红 黑 蓝 白 */
    {1, 2, 4, 3, 0},  /* 13: 白 红 蓝 绿 黑 */
    {2, 3, 1, 4, 0},  /* 14: 红 绿 白 蓝 黑 */
    {4, 1, 3, 2, 0},  /* 15: 蓝 白 绿 红 黑 */
    {3, 4, 2, 1, 0},  /* 16: 绿 蓝 红 白 黑 */
};

/* 任务2 二维码对应搬运方案表(附录B): 每一行 = 该二维码编号对应的搬运顺序
 * 字母编号: 0=A 1=B 2=C
 */
static const uint8_t task2_scheme[6][3] = {
    {0, 1, 2},  /* 1: A B C */
    {0, 2, 1},  /* 2: A C B */
    {1, 0, 2},  /* 3: B A C */
    {1, 2, 0},  /* 4: B C A */
    {2, 0, 1},  /* 5: C A B */
    {2, 1, 0},  /* 6: C B A */
};

/* ==================== 颜色/字母 解析工具 ==================== */

/* 颜色字母表: B=黑 W=白 R=红 G=绿 U=蓝 */
static const char task1_color_letter[5] = {'B', 'W', 'R', 'G', 'U'};

/* 任务2 奖杯放置目标(打印用): 0=A->champion 1=B->runner-up 2=C->third */
static const char *const task2_target_en[3] = {"champion",
                                               "runner-up",
                                               "third"};

/* 将一个颜色字符(ASCII字母或UTF-8中文)解析为颜色编号(0~4)，并更新读取位置
 * 返回 0~4 成功; 0xFF 未知字符/编码不完整
 */
static uint8_t task1_char_to_color(const uint8_t *s, const uint8_t **next)
{
    /* 中文颜色 UTF-8 字节: 黑=E9BB91 白=E799BD 红=E7BAA2 绿=E7BBBF 蓝=E8939D */
    static const uint8_t cn_utf8[5][3] = {
        {0xE9, 0xBB, 0x91},  /* 黑 */
        {0xE7, 0x99, 0xBD},  /* 白 */
        {0xE7, 0xBA, 0xA2},  /* 红 */
        {0xE7, 0xBB, 0xBF},  /* 绿 */
        {0xE8, 0x93, 0x9D},  /* 蓝 */
    };
    uint8_t ch = *s;
    uint8_t i;

    /* ASCII 字母(不区分大小写) */
    if (ch < 0x80U) {
        for (i = 0U; i < 5U; i++) {
            uint8_t upper = (uint8_t)task1_color_letter[i];
            uint8_t lower = (uint8_t)(upper + ('a' - 'A'));
            if ((ch == upper) || (ch == lower)) {
                *next = s + 1;
                return i;
            }
        }
        *next = s + 1;
        return 0xFFU;
    }

    /* UTF-8 中文(3字节)，先确认字节足够 */
    if ((s[1] == 0U) || (s[2] == 0U)) {
        *next = s + 1;
        return 0xFFU;
    }
    for (i = 0U; i < 5U; i++) {
        if ((s[0] == cn_utf8[i][0]) && (s[1] == cn_utf8[i][1]) && (s[2] == cn_utf8[i][2])) {
            *next = s + 3;
            return i;
        }
    }
    *next = s + 1;
    return 0xFFU;
}

/* 解析任务1颜色顺序字符串为5个颜色编号(0~4)
 * 支持: 字母"BWRGU"(大小写均可) 或 中文"黑白红绿蓝"(UTF-8)，可含空格/逗号分隔
 * 返回 0=成功; 非0=失败
 */
static uint8_t task1_parse_colors(const char *str, uint8_t out[5])
{
    const uint8_t *s = (const uint8_t *)str;
    uint8_t n = 0U;
    uint8_t seen = 0U;
    uint8_t c;

    while (*s != 0U) {
        if ((*s == ' ') || (*s == '\t') || (*s == ',')) {
            s++;
            continue;
        }
        if (n >= 5U) {
            return 1U;   /* 超过5个颜色 */
        }
        c = task1_char_to_color(s, &s);
        if (c >= 5U) {
            return 1U;   /* 未知颜色 */
        }
        if ((seen & (uint8_t)(1U << c)) != 0U) {
            return 1U;   /* 颜色重复 */
        }
        seen |= (uint8_t)(1U << c);
        out[n++] = c;
    }
    if (n != 5U) {
        return 1U;   /* 颜色数量不足5个 */
    }
    return 0U;
}

/* 解析任务2奖杯顺序字符串为3个字母编号(0=A 1=B 2=C)
 * 返回 0=成功; 非0=失败
 */
static uint8_t task2_parse_abc(const char *str, uint8_t out[3])
{
    const uint8_t *s = (const uint8_t *)str;
    uint8_t n = 0U;
    uint8_t seen = 0U;
    uint8_t c;

    while (*s != 0U) {
        if ((*s == ' ') || (*s == '\t') || (*s == ',')) {
            s++;
            continue;
        }
        if (n >= 3U) {
            return 1U;   /* 超过3个奖杯 */
        }
        if ((*s >= 'A') && (*s <= 'C')) {
            c = (uint8_t)(*s - 'A');
        } else if ((*s >= 'a') && (*s <= 'c')) {
            c = (uint8_t)(*s - 'a');
        } else {
            return 1U;   /* 未知字母 */
        }
        s++;
        if ((seen & (uint8_t)(1U << c)) != 0U) {
            return 1U;   /* 字母重复 */
        }
        seen |= (uint8_t)(1U << c);
        out[n++] = c;
    }
    if (n != 3U) {
        return 1U;   /* 奖杯数量不足3个 */
    }
    return 0U;
}

/* ==================== 对外接口 ==================== */

/* 内部: 由槽位颜色编号数组(从左到右) + 二维码编号，计算搬运方案
 * observed[j] = 槽位 j+1 的颜色编号(0=黑 1=白 2=红 3=绿 4=蓝)
 * plan[i] = 第 i 步去的槽位号(1~5)
 * 返回: TASK_PLAN_OK / TASK_PLAN_ERR_QR / TASK_PLAN_ERR_INPUT
 */
static uint8_t task1_build_plan(uint8_t qr_code, const uint8_t observed[5], uint8_t plan[5])
{
    uint8_t i;
    uint8_t j;
    const uint8_t *scheme;

    if ((qr_code < 1U) || (qr_code > TASK1_QR_MAX)) {
        return TASK_PLAN_ERR_QR;
    }
    if ((observed == NULL) || (plan == NULL)) {
        return TASK_PLAN_ERR_INPUT;
    }

    scheme = task1_scheme[qr_code - 1U];
    for (i = 0U; i < 5U; i++) {
        plan[i] = 0U;
        for (j = 0U; j < 5U; j++) {
            if (observed[j] == scheme[i]) {
                plan[i] = (uint8_t)(j + 1U);   /* 槽位号 1~5 */
                break;
            }
        }
        if (plan[i] == 0U) {
            return TASK_PLAN_ERR_INPUT;   /* 方案要求的颜色不在现场槽位里 */
        }
    }
    return TASK_PLAN_OK;
}

uint8_t Task1_QRPlan(uint8_t qr_code, const char *colors_left_to_right, uint8_t plan[5])
{
    uint8_t observed[5];

    if ((plan == NULL) || (colors_left_to_right == NULL)) {
        return TASK_PLAN_ERR_INPUT;
    }
    if (task1_parse_colors(colors_left_to_right, observed) != 0U) {
        return TASK_PLAN_ERR_INPUT;
    }
    return task1_build_plan(qr_code, observed, plan);
}

/* 便捷版: 直接输入每个槽位(1~5, 从左到右)的颜色编号数组，计算搬运(抓取)方案
 * 颜色编号: 0=黑(B) 1=白(W) 2=红(R) 3=绿(G) 4=蓝(U)
 * slot_colors[i] = 槽位 i+1 的颜色编号 (i=0~4 对应从左到右)
 * 输出 plan[5] 与 Task1_QRPlan 相同: plan[i] = 第 i 步去几号槽抓
 * 返回: TASK_PLAN_OK / TASK_PLAN_ERR_QR / TASK_PLAN_ERR_INPUT
 */
uint8_t Task1_QRPlanBySlotColors(uint8_t qr_code, const uint8_t slot_colors[5], uint8_t plan[5])
{
    uint8_t seen = 0U;
    uint8_t i;
    uint8_t c;

    if (slot_colors == NULL) {
        return TASK_PLAN_ERR_INPUT;
    }
    for (i = 0U; i < 5U; i++) {
        c = slot_colors[i];
        if (c >= 5U) {
            return TASK_PLAN_ERR_INPUT;   /* 颜色编号越界 */
        }
        if ((seen & (uint8_t)(1U << c)) != 0U) {
            return TASK_PLAN_ERR_INPUT;   /* 颜色重复 */
        }
        seen |= (uint8_t)(1U << c);
    }
    return task1_build_plan(qr_code, slot_colors, plan);
}

uint8_t Task2_QRPlan(uint8_t qr_code, const char *abc_right_to_left, uint8_t plan[3])
{
    uint8_t observed[3];
    uint8_t i;
    uint8_t j;
    const uint8_t *scheme;

    if ((qr_code < 1U) || (qr_code > TASK2_QR_MAX)) {
        return TASK_PLAN_ERR_QR;
    }
    if ((plan == NULL) || (abc_right_to_left == NULL)) {
        return TASK_PLAN_ERR_INPUT;
    }
    if (task2_parse_abc(abc_right_to_left, observed) != 0U) {
        return TASK_PLAN_ERR_INPUT;
    }

    scheme = task2_scheme[qr_code - 1U];
    for (i = 0U; i < 3U; i++) {
        for (j = 0U; j < 3U; j++) {
            if (observed[j] == scheme[i]) {
                plan[i] = (uint8_t)(j + 1U);   /* 槽位号 1~3 */
                break;
            }
        }
    }
    return TASK_PLAN_OK;
}

uint8_t Task2_GetSchemeTrophy(uint8_t qr_code, uint8_t step)
{
    if ((qr_code < 1U) || (qr_code > TASK2_QR_MAX) || (step >= 3U)) {
        return 0xFFU;
    }
    return task2_scheme[qr_code - 1U][step];
}

uint8_t Task_BuildPlan(uint8_t qr_task1, const char *colors_left_to_right,
                       uint8_t qr_task2, const char *abc_right_to_left,
                       uint8_t plan1[5], uint8_t plan2[3])
{
    uint8_t ret1 = Task1_QRPlan(qr_task1, colors_left_to_right, plan1);
    uint8_t ret2 = Task2_QRPlan(qr_task2, abc_right_to_left, plan2);

    if (ret1 != TASK_PLAN_OK) {
        return ret1;
    }
    if (ret2 != TASK_PLAN_OK) {
        return ret2;
    }
    return TASK_PLAN_OK;
}

void Task_PrintPlan(uint8_t qr_task1, const char *colors_left_to_right,
                    uint8_t qr_task2, const char *abc_right_to_left)
{
    uint8_t plan1[5];
    uint8_t plan2[3];
    uint8_t i;
    uint8_t c;

    uart_printf("\r\n======== TRANSPORT PLAN ========\r\n");

    if (Task1_QRPlan(qr_task1, colors_left_to_right, plan1) == TASK_PLAN_OK) {
        uart_printf("Task1: QR=%u  layout(left->right)=%s\r\n", qr_task1, colors_left_to_right);
        uart_printf("  scheme(transport order):");
        for (i = 0U; i < 5U; i++) {
            uart_printf(" %c", task1_color_letter[task1_scheme[qr_task1 - 1U][i]]);
        }
        uart_printf("\r\n");
        uart_printf("  grab slot order:");
        for (i = 0U; i < 5U; i++) {
            c = task1_scheme[qr_task1 - 1U][i];
            uart_printf("  slot%u(%c)", plan1[i], task1_color_letter[c]);
            if (i < 4U) {
                uart_printf(" ->");
            }
        }
        uart_printf("\r\n");
    } else {
        uart_printf("Task1: input error(QR=%u layout=%s)\r\n",
                    qr_task1, (colors_left_to_right != NULL) ? colors_left_to_right : "NULL");
    }

    if (Task2_QRPlan(qr_task2, abc_right_to_left, plan2) == TASK_PLAN_OK) {
        uart_printf("Task2: QR=%u  layout(right->left)=%s\r\n", qr_task2, abc_right_to_left);
        uart_printf("  scheme(transport order):");
        for (i = 0U; i < 3U; i++) {
            uart_printf(" %c", (char)('A' + task2_scheme[qr_task2 - 1U][i]));
        }
        uart_printf("\r\n");
        uart_printf("  grab slot order:");
        for (i = 0U; i < 3U; i++) {
            c = task2_scheme[qr_task2 - 1U][i];
            uart_printf("  slot%u(%c->%s)", plan2[i], (char)('A' + c), task2_target_en[c]);
            if (i < 2U) {
                uart_printf(" ->");
            }
        }
        uart_printf("\r\n");
    } else {
        uart_printf("Task2: input error(QR=%u layout=%s)\r\n",
                    qr_task2, (abc_right_to_left != NULL) ? abc_right_to_left : "NULL");
    }

    uart_printf("============================\r\n");
}

void Task_TestPlan(void)
{
    /* 示例: 任务1二维码=1, 现场摆放(左->右)=蓝 白 黑 红 绿
     *       任务2二维码=3, 现场摆放(右->左)=C B A
     */
    Task_PrintPlan(1U, "蓝白黑红绿", 3U, "CBA");

    /* 再演示一组字母输入 */
    Task_PrintPlan(5U, "W R U B G", 2U, "ACB");
}
