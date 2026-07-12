


#include "ZDTstepmotor.h"
#include "math.h"
void Step_ZDT_Init(StepMotorZDT_t *zdt_mot,  uint32_t id ,UART_HandleTypeDef *_USART,int8_t _dir,
     float _wheel_diameter, bool _have_pub_permission)
{
	zdt_mot->motor_controller_t.id=id;
	zdt_mot->_USART=_USART;
	zdt_mot->_dir=_dir;
	zdt_mot->_wheel_diameter=_wheel_diameter;
	zdt_mot->_have_pub_permission=_have_pub_permission;
}

//第一部分：装填命令
/**
 * @brief    位置模式
 * @param    addr：电机地址
 * @param    dir ：方向        ，0为CW，其余值为CCW
 * @param    vel ：速度(RPM)   ，范围0 - 5000RPM
 * @param    acc ：加速度      ，范围0 - 255，注意：0是直接启动
 * @param    clk ：脉冲数      ，范围0- (2^32 - 1)个
 * @param    raF ：相位/绝对标志，false为相对运动，true为绝对值运动
 * @param    snF ：多机同步标志 ，false为不启用，true为启用
 * @retval   地址 + 功能码 + 命令状态 + 校验字节
 */
static uint8_t Step_Pos_Control(uint8_t *cmd, uint8_t addr, uint8_t dir, uint16_t vel, uint8_t acc, 
    uint32_t clk, bool raF, bool snF)
{
    // uint8_t cmd[16] = {0};

    // 装载命令
    cmd[0] = addr;                 // 地址
    cmd[1] = 0xFD;                 // 功能码
    cmd[2] = dir;                  // 方向
    cmd[3] = (uint8_t)(vel >> 8);  // 速度(RPM)高8位字节
    cmd[4] = (uint8_t)(vel >> 0);  // 速度(RPM)低8位字节
    cmd[5] = acc;                  // 加速度，注意：0是直接启动
    cmd[6] = (uint8_t)(clk >> 24); // 脉冲数(bit24 - bit31)
    cmd[7] = (uint8_t)(clk >> 16); // 脉冲数(bit16 - bit23)
    cmd[8] = (uint8_t)(clk >> 8);  // 脉冲数(bit8  - bit15)
    cmd[9] = (uint8_t)(clk >> 0);  // 脉冲数(bit0  - bit7 )
    cmd[10] = raF;                 // 相位/绝对标志，false为相对运动，true为绝对值运动
    cmd[11] = snF;                 // 多机同步运动标志，false为不启用，true为启用
    cmd[12] = 0x6B;                // 校验字节

    // 返回命令长度
    return (uint8_t)13;
}

/**
 * @brief    速度模式
 * @param    addr：电机地址
 * @param    dir ：方向       ，0为CW，其余值为CCW
 * @param    vel ：速度       ，范围0 - 5000RPM
 * @param    acc ：加速度     ，范围0 - 255，注意：0是直接启动
 * @param    snF ：多机同步标志，false为不启用，true为启用
 * @retval   地址 + 功能码 + 命令状态 + 校验字节
 */
static uint8_t Step_Vel_Control(uint8_t *cmd, uint8_t addr, uint8_t dir, uint16_t vel, uint8_t acc, bool snF)
{
    // uint8_t cmd[16] = {0};

    // 装载命令
    cmd[0] = addr;                // 地址
    cmd[1] = 0xF6;                // 功能码
    cmd[2] = dir;                 // 方向
    cmd[3] = (uint8_t)(vel >> 8); // 速度(RPM)高8位字节
    cmd[4] = (uint8_t)(vel >> 0); // 速度(RPM)低8位字节
    cmd[5] = acc;                 // 加速度，注意：0是直接启动
    cmd[6] = snF;                 // 多机同步运动标志
    cmd[7] = 0x6B;                // 校验字节

    // 发送命令
    return (uint8_t)8;
}

/**
 * @brief    多机同步运动
 * @param    addr  ：电机地址
 * @retval   地址 + 功能码 + 命令状态 + 校验字节
 */
static uint8_t Step_Synchronous_motion(uint8_t *cmd, uint8_t addr)
{
    // uint8_t cmd[16] = {0};

    // 装载命令
    cmd[0] = addr; // 地址
    cmd[1] = 0xFF; // 功能码
    cmd[2] = 0x66; // 辅助码
    cmd[3] = 0x6B; // 校验字节
    return (4);
}

/**
 * @brief 发送读取实际转速命令 (Emm_V5 功能码 0x33)
 * @param zdt_motor 电机结构体指针
 *
 * 【协议帧】发送: [地址 1B] [0x35 1B] [0x00 1B] [校验 0x6B]
 * 【回传帧】     : [地址 1B] [0x35 1B] [转速高8位] [转速低8位] [校验 0x6B] (共5字节)
 *
 * @note  发送后驱动器立即回传, 需在 USART1 IDLE 中断中接收解析。
 *        4个电机共用一条 UART 总线, 不可同时查询, 必须轮询。
 */
void ZDT_Query_RPM(StepMotorZDT_t *zdt_motor)
{
    uint8_t cmd[6];
    cmd[0] = zdt_motor->motor_controller_t.id;  // 电机地址
    cmd[1] = 0x11;                              
    cmd[2] = 0x18;                             
    cmd[3] = 0x32;                              // 读取转速功能码
    cmd[4] = 0x00;                              // 固定辅助码
    cmd[5] = 0x6B;                              // 校验字节

    HAL_UART_Transmit(zdt_motor->_USART, cmd, 6, 100);
//    HAL_Delay(1);                               // 避免总线冲突,避免粘包（*标记*但会堵死，先去掉）
    zdt_motor->_last_query_tick = HAL_GetTick();
}

/**
 * @brief 发送读取编码器命令 (Emm_V5 功能码 0x30)
 * @param zdt_motor 电机结构体指针
 *
 * 【协议帧】发送: [地址 1B] [0x30 1B] [0x00 1B] [校验 0x6B]
 * 【回传帧】     : [地址 1B] [0x30 1B] [进位高] [进位低] [编码器3][2][1][0] [校验 0x6B] (共9字节)
 */
void ZDT_Query_Encoder(StepMotorZDT_t *zdt_motor)
{
    uint8_t cmd[4];
    cmd[0] = zdt_motor->motor_controller_t.id;
    cmd[1] = 0x30;
    cmd[2] = 0x00;
    cmd[3] = 0x6B;

    HAL_UART_Transmit(zdt_motor->_USART, cmd, 4, 100);
//    HAL_Delay(1);
    zdt_motor->_last_query_tick = HAL_GetTick();
}


//第二部分：

/**
 * @brief    设置速度
 * @param    
 * @retval   
 */
void set_speed_target(StepMotorZDT_t *zdt_motor, float target)
{
    zdt_motor->motor_controller_t.set.velocity = target;
    zdt_motor->_target_rpm = (int16_t)(target * 60.0 / (3.14 * (zdt_motor->_wheel_diameter))); // 转化为转速(RPM)

    uint8_t len;
	//确定正反转，同时装载命令
    if (zdt_motor->_target_rpm > 0)
    {
        len = Step_Vel_Control(zdt_motor->_cmd_buffer, zdt_motor->motor_controller_t.id, zdt_motor->_dir, 
            (uint16_t)(zdt_motor->_target_rpm), 0, true);
    }
    else
    { 
        int dir_trans = zdt_motor->_dir == 0 ? 1 : 0;
        len = Step_Vel_Control(zdt_motor->_cmd_buffer, zdt_motor->motor_controller_t.id, dir_trans, 
            (uint16_t)(-zdt_motor->_target_rpm), 0, true);
    }
	//发送命令（串口orDMA）
//    HAL_UART_Transmit(zdt_motor->_USART, zdt_motor->_cmd_buffer, len, 1000); // 发送数据到电机
     HAL_UART_Transmit_DMA(zdt_motor->_USART, zdt_motor->_cmd_buffer, len); // 发送数据到电机
    HAL_Delay(1);                                                            // 傻逼电机需要延迟避免重包  //hahaha  //666
	
    if (zdt_motor->_have_pub_permission)
    {
        // 发布同步信号
        len = Step_Synchronous_motion(zdt_motor->_cmd_buffer, 0);                // 装载命令
//        HAL_UART_Transmit(zdt_motor->_USART, zdt_motor->_cmd_buffer, len, 1000); // 发送数据到电机
         HAL_UART_Transmit_DMA(zdt_motor->_USART, zdt_motor->_cmd_buffer, len); // 发送数据到电机
        HAL_Delay(1);
    }
}


/**
 * @brief    设置电机以指定速度运动到目标位置
 * @param    zdt_motor: 电机控制结构体指针
 * @param    target_speed: 目标线速度(m/s)
 * @param    target_pos: 目标位置(m)
 * @note     使用位置控制模式，通过计算将线速度和位置转换为RPM和脉冲数
 */
void set_speed_pos_target(StepMotorZDT_t *zdt_motor, float target_speed, float target_pos)
{
    // 计算目标转速(RPM)
    zdt_motor->_target_rpm = (int16_t)(target_speed * 60 / (3.14 * zdt_motor->_wheel_diameter));
    
    // 计算目标位置对应的脉冲数
    // 假设每转脉冲数为固定值(需要根据实际电机参数调整)
    const uint32_t pulses_per_revolution = 200 * 16; // 示例: 200步/转，16细分，步进角是1.8°（1.8*200=360）所以每微步旋转0.1125°
    uint32_t target_pulses = (uint32_t)(fabsf(target_pos) / (3.14 * zdt_motor->_wheel_diameter) * pulses_per_revolution);
    
    uint8_t len;
    int8_t dir = zdt_motor->_dir;
    
    // 处理方向
    if (zdt_motor->_target_rpm < 0)
    {
        // 反转时调整方向
        dir = (zdt_motor->_dir == 0) ? 1 : 0;
        zdt_motor->_target_rpm = -zdt_motor->_target_rpm;
    }
    
    // 发送位置控制命令
    len = Step_Pos_Control(zdt_motor->_cmd_buffer, 
                          zdt_motor->motor_controller_t.id, 
                          dir,
                          (uint16_t)zdt_motor->_target_rpm, 
                          0, // 加速度设为0直接启动
                          target_pulses, 
                          false, // 使用相对位置模式
                          true); // 不使用同步    //*标记*
    
//    HAL_UART_Transmit(zdt_motor->_USART, zdt_motor->_cmd_buffer, len, 1000);
    HAL_UART_Transmit_DMA(zdt_motor->_USART, zdt_motor->_cmd_buffer, len);
    HAL_Delay(1);
    // 如果有发布权限，发送同步信号
    if (zdt_motor->_have_pub_permission)
    {
        len = Step_Synchronous_motion(zdt_motor->_cmd_buffer, 0);
//        HAL_UART_Transmit(zdt_motor->_USART, zdt_motor->_cmd_buffer, len, 1000);
         HAL_UART_Transmit_DMA(zdt_motor->_USART, zdt_motor->_cmd_buffer, len);
        HAL_Delay(1);
    }
    
    // 更新控制器设置//*标记*
    zdt_motor->motor_controller_t.set.velocity = target_speed;
    zdt_motor->motor_controller_t.set.deg_pos = target_pos;
}

/* 外部全局电机定义 (定义在 Chassis.c) */
extern StepMotorZDT_t Motor1, Motor2, Motor3, Motor4;


/**
 * @brief 轮询查询4个电机的实际转速
 * @note  每次调用只查询一个电机, 4次调用覆盖全部电机。
 *        适用于 10ms 周期任务, 确保总线不会冲突。
 *        查询顺序: Motor1 → Motor2 → Motor3 → Motor4 → Motor1 ...
 */
void ZDT_Query_All_Motor_RPM(void)
{
    static uint8_t idx = 0;
    StepMotorZDT_t *motors[4] = {&Motor1, &Motor2, &Motor3, &Motor4};

    ZDT_Query_RPM(motors[idx]);

//    idx++;
//    if (idx >= 4) idx = 0;
}

/**
 * @brief 解析 Emm_V5 驱动器回传帧
 * @param data  USART1 DMA 接收缓冲区指针
 * @param len   本次 IDLE 中断接收到的总字节数
 *
 * 【解析逻辑】
 *  1. 以 0x6B 为帧尾, 将粘连的多帧分割开
 *  2. 根据帧首字节(地址)匹配到 Motor1~Motor4
 *  3. 根据功能码提取数据:
 *       0x33 → 实际转速 (RPM), 换算为线速度 m/s
 *       0x30 → 编码器累计值 (32位 + 进位)
 *
 * @note  必须在 USART1 IDLE 中断里调用, 解析完立即释放缓冲区。
 */
void ZDT_Parse_Frame(uint8_t *data, uint8_t len)
{
    uint8_t i = 0;

    while (i < len) {
        /* Step 1: 找帧尾 0x6B */
        uint8_t frame_end = i;
        while (frame_end < len && data[frame_end] != 0x6B) {
            frame_end++;
        }
        if (frame_end >= len) {
            break;  /* 剩余数据没有完整帧尾, 丢弃 */
        }

        uint8_t frame_len = frame_end - i + 1;
        if (frame_len < 3) {
            i = frame_end + 1;
            continue;  /* 帧太短, 跳过 */
        }

        /* Step 2: 提取地址和功能码 */
        uint8_t addr = data[i];
        uint8_t func = data[i + 1];
        StepMotorZDT_t *mot = NULL;

        if      (addr == Motor1.motor_controller_t.id) mot = &Motor1;
        else if (addr == Motor2.motor_controller_t.id) mot = &Motor2;
        else if (addr == Motor3.motor_controller_t.id) mot = &Motor3;
        else if (addr == Motor4.motor_controller_t.id) mot = &Motor4;

        /* Step 3: 根据功能码解析数据 */
        if (mot != NULL) {
            if (func == 0x33 && frame_len >= 5) {
                /* 读取转速回传: [addr][0x33][rpm_h][rpm_l][0x6B] */
                int16_t rpm = (int16_t)((data[i + 2] << 8) | data[i + 3]);
                mot->_real_rpm = rpm;
                /* RPM → m/s: v = rpm * π * d / 60 */
                mot->_real_linear_speed = (float)rpm * 3.14159265f * mot->_wheel_diameter / 60.0f;
            }
            else if (func == 0x30 && frame_len >= 9) {
                /* 读取编码器回传: [addr][0x30][carry_h][carry_l][enc3][enc2][enc1][enc0][0x6B] */
                int16_t carry = (int16_t)((data[i + 2] << 8) | data[i + 3]);
                int32_t enc   = ((int32_t)data[i + 4] << 24) |
                                ((int32_t)data[i + 5] << 16) |
                                ((int32_t)data[i + 6] << 8)  |
                                data[i + 7];
                mot->_encoder_value = (int32_t)carry * 65536 + enc;
            }
        }

        i = frame_end + 1;  /* 跳到下一帧 */
    }
}

/*================== 速度查询 ==================*/

/**
 * @brief 获取电机实际线速度 (优先使用真实反馈)
 * @retval 电机线速度 (m/s)
 *
 * 【优先级】
 *  1. 如果已经收到过 Emm_V5 回传的实际转速 (_real_rpm 非零 或 _real_linear_speed 非零),
 *     返回 _real_linear_speed (真实速度)。
 *  2. 如果尚未收到回传 (比如刚上电还没查询过),
 *     fallback 到 _target_rpm (指令速度), 保证兼容旧代码。
 *
 * @note  要获得真实速度, 必须先在定时任务中周期性调用 ZDT_Query_All_Motor_RPM()。
 */
float get_linear_speed(StepMotorZDT_t *zdt_motor)
{
    
//    if (zdt_motor->_real_rpm != 0 || zdt_motor->_real_linear_speed != 0.0f) {
		
        return zdt_motor->_real_linear_speed;
//    }

//    return zdt_motor->_target_rpm * 3.14159265f * zdt_motor->_wheel_diameter / 60.0f;
}
