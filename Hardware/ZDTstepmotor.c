#include "ZDTstepmotor.h"
#include "Emm_V5.h"
#include "math.h"

volatile uint32_t ZDT_TxRejectedCommands = 0U;

/**
 * @brief Validate USART1 ownership and reserve enough visible queue capacity.
 *
 * A publisher motor submits its own command followed by the synchronization
 * trigger, so both slots must be available before either frame is queued.
 */
static bool ZDT_TxCanSubmit(const StepMotorZDT_t *zdt_motor, uint8_t frame_count)
{
    if ((zdt_motor == NULL) || (zdt_motor->_USART != &huart1) ||
        (Emm_V5_TxGetFreeSlots() < frame_count)) {
        ZDT_TxRejectedCommands++;
        return false;
    }

    return true;
}

void Step_ZDT_Init(StepMotorZDT_t *zdt_mot,  uint8_t id ,UART_HandleTypeDef *_USART,int8_t _dir,
     float _wheel_diameter, bool _have_pub_permission)
{
	zdt_mot->motor_controller_t.id=id;
	zdt_mot->_USART=_USART;
	zdt_mot->_dir=_dir;
	zdt_mot->_wheel_diameter=_wheel_diameter;
	zdt_mot->_have_pub_permission=_have_pub_permission;
}

//第一部分：装填命令
// /**
//  * @brief    位置模式
//  * @param    addr：电机地址
//  * @param    dir ：方向        ，0为CW，其余值为CCW
//  * @param    vel ：速度(RPM)   ，范围0 - 5000RPM
//  * @param    acc ：加速度      ，范围0 - 255，注意：0是直接启动
//  * @param    clk ：脉冲数      ，范围0- (2^32 - 1)个
//  * @param    raF ：相位/绝对标志，false为相对运动，true为绝对值运动
//  * @param    snF ：多机同步标志 ，false为不启用，true为启用
//  * @retval   地址 + 功能码 + 命令状态 + 校验字节
//  */
// static uint8_t Step_Pos_Control(uint8_t *cmd, uint8_t addr, uint8_t dir, uint16_t vel, uint8_t acc, 
//     uint32_t clk, bool raF, bool snF)
// {
//     // uint8_t cmd[16] = {0};

//     // 装载命令
//     cmd[0] = addr;                 // 地址
//     cmd[1] = 0xFD;                 // 功能码
//     cmd[2] = dir;                  // 方向
//     cmd[3] = (uint8_t)(vel >> 8);  // 速度(RPM)高8位字节
//     cmd[4] = (uint8_t)(vel >> 0);  // 速度(RPM)低8位字节
//     cmd[5] = acc;                  // 加速度，注意：0是直接启动
//     cmd[6] = (uint8_t)(clk >> 24); // 脉冲数(bit24 - bit31)
//     cmd[7] = (uint8_t)(clk >> 16); // 脉冲数(bit16 - bit23)
//     cmd[8] = (uint8_t)(clk >> 8);  // 脉冲数(bit8  - bit15)
//     cmd[9] = (uint8_t)(clk >> 0);  // 脉冲数(bit0  - bit7 )
//     cmd[10] = raF;                 // 相位/绝对标志，false为相对运动，true为绝对值运动
//     cmd[11] = snF;                 // 多机同步运动标志，false为不启用，true为启用
//     cmd[12] = 0x6B;                // 校验字节

//     // 返回命令长度
//     return (uint8_t)13;
// }

// /**
//  * @brief    速度模式
//  * @param    addr：电机地址
//  * @param    dir ：方向       ，0为CW，其余值为CCW
//  * @param    vel ：速度       ，范围0 - 5000RPM
//  * @param    acc ：加速度     ，范围0 - 255，注意：0是直接启动
//  * @param    snF ：多机同步标志，false为不启用，true为启用
//  * @retval   地址 + 功能码 + 命令状态 + 校验字节
//  */
// static uint8_t Step_Vel_Control(uint8_t *cmd, uint8_t addr, uint8_t dir, uint16_t vel, uint8_t acc, bool snF)
// {
//     // uint8_t cmd[16] = {0};

//     // 装载命令
//     cmd[0] = addr;                // 地址
//     cmd[1] = 0xF6;                // 功能码
//     cmd[2] = dir;                 // 方向
//     cmd[3] = (uint8_t)(vel >> 8); // 速度(RPM)高8位字节
//     cmd[4] = (uint8_t)(vel >> 0); // 速度(RPM)低8位字节
//     cmd[5] = acc;                 // 加速度，注意：0是直接启动
//     cmd[6] = snF;                 // 多机同步运动标志
//     cmd[7] = 0x6B;                // 校验字节

//     // 发送命令
//     return (uint8_t)8;
// }

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
 * @brief 发送读取编码器命令 (Emm_V5 功能码 0x30)
 * @param zdt_motor 电机结构体指针
 *
 * 【协议帧】发送: [地址 1B] [0x30 1B] [0x00 1B] [校验 0x6B]
 * 【回传帧】     : [地址 1B] [0x30 1B] [进位高] [进位低] [编码器3][2][1][0] [校验 0x6B] (共9字节)
 */
void ZDT_Query_Encoder(StepMotorZDT_t *zdt_motor)
{
    uint8_t cmd[4];

    if (!ZDT_TxCanSubmit(zdt_motor, 1U)) return;

    cmd[0] = zdt_motor->motor_controller_t.id;
    cmd[1] = 0x30;
    cmd[2] = 0x00;
    cmd[3] = 0x6B;

    if (Emm_V5_Transmit_Frame(zdt_motor->_USART, cmd, 4U) == HAL_OK) {
        // DMA队列已经复制命令，不需要额外延时。
    } else {
        ZDT_TxRejectedCommands++;
    }
}


//第二部分：

/**
 * @brief    设置电机速度（但实际上这个函数需要反复执行，否则速度会略低于理论值）
 * @param    
 * @retval   无
 */
void set_speed_target(StepMotorZDT_t *zdt_motor, float target)
{
    uint8_t required_frames;

    if (zdt_motor == NULL) {
        ZDT_TxRejectedCommands++;
        return;
    }
    required_frames = zdt_motor->_have_pub_permission ? 2U : 1U;
    if (!ZDT_TxCanSubmit(zdt_motor, required_frames)) return;

    zdt_motor->motor_controller_t.tar_velocity = target;
    zdt_motor->motor_controller_t.tar_rpm = (int16_t)(target * 60.0f / (3.14f * zdt_motor->_wheel_diameter)); // 转化为转速(RPM)
    //正反转
    if (zdt_motor->motor_controller_t.tar_rpm > 0)
    {
        Emm_V5_Vel_Control(zdt_motor->motor_controller_t.id,
                           zdt_motor->_dir,
                           (uint16_t)(zdt_motor->motor_controller_t.tar_rpm),
                           0,
                           true);//true表示多机同步运动
    }
    else
    {
        int dir_trans = zdt_motor->_dir == 0 ? 1 : 0;
        Emm_V5_Vel_Control(zdt_motor->motor_controller_t.id,
                           dir_trans,
                           (uint16_t)(-zdt_motor->motor_controller_t.tar_rpm),
                           0,
                           true);//true表示多机同步运动
    }
    //发布权限
    if (zdt_motor->_have_pub_permission)
    {
        uint8_t len = Step_Synchronous_motion(zdt_motor->_cmd_buffer, 0);                // 装载命令
        Emm_V5_Transmit_Frame(zdt_motor->_USART, zdt_motor->_cmd_buffer, len);
        // 发送同步信号，触发运动
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
    uint8_t required_frames;

    if (zdt_motor == NULL) {
        ZDT_TxRejectedCommands++;
        return;
    }
    required_frames = zdt_motor->_have_pub_permission ? 2U : 1U;
    if (!ZDT_TxCanSubmit(zdt_motor, required_frames)) return;

    // 计算目标转速(RPM)
    zdt_motor->motor_controller_t.tar_rpm = (int16_t)(target_speed * 60.0f / (3.14f * zdt_motor->_wheel_diameter));
    
    // 计算目标位置对应的脉冲数
    // 假设每转脉冲数为固定值(需要根据实际电机参数调整)
    const uint32_t pulses_per_revolution = 200 * 16; // 示例: 200步/转，16细分，步进角是1.8°（1.8*200=360）所以每微步旋转0.1125°
    uint32_t target_pulses = (uint32_t)(fabsf(target_pos) / (3.14f * zdt_motor->_wheel_diameter) * pulses_per_revolution);
    
    int8_t dir = zdt_motor->_dir;
    
    // 处理方向
    if (zdt_motor->motor_controller_t.tar_rpm < 0)
    {
        // 反转时调整方向
        dir = (zdt_motor->_dir == 0) ? 1 : 0;
        // dir = (zdt_motor->_dir == 0) ? 1 : 0;
        zdt_motor->motor_controller_t.tar_rpm = -zdt_motor->motor_controller_t.tar_rpm;
    }
    
    // 发送位置控制命令
    Emm_V5_Pos_Control(zdt_motor->motor_controller_t.id,
                       dir,
                       (uint16_t)zdt_motor->motor_controller_t.tar_rpm,
                       0, // 加速度设为0直接启动
                       target_pulses,
                       false, // 使用相对位置模式
                       true); // 多机同步标志
    
    // 如果有发布权限，发送同步信号
    if (zdt_motor->_have_pub_permission)
    {
        uint8_t len = Step_Synchronous_motion(zdt_motor->_cmd_buffer, 0);
        Emm_V5_Transmit_Frame(zdt_motor->_USART, zdt_motor->_cmd_buffer, len);
    }
    
    // 更新控制器设置//*标记*
    zdt_motor->motor_controller_t.tar_velocity = target_speed;
    zdt_motor->motor_controller_t.tar_deg_pos = target_pos;
}



/*================== 速度查询 ==================*/

/**
 * @brief 获取电机当前指令线速度
 * @retval 电机线速度 (m/s)
 * @note 不读取电机串口回传，返回最近一次下发的目标速度。
 */
float get_linear_speed(StepMotorZDT_t *zdt_motor)
{
    if (zdt_motor == NULL) {
        return 0.0f;
    }

    return zdt_motor->motor_controller_t.tar_velocity;
}
