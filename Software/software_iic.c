#include "software_iic.h"
/* 基本时序操作 */
static void Soft_IIC_Delay(void)
{
    Delay_us(10);  // 根据实际I2C速度调整延时
}
/* 软件I2C基础函数 */
void Soft_IIC_Start(void)
{
    SDA_HIGH();
    SCL_HIGH();
    Soft_IIC_Delay();
    SDA_LOW();
    Soft_IIC_Delay();
    SCL_LOW();
}

void Soft_IIC_Stop(void)
{
    SDA_LOW();
    Soft_IIC_Delay();
    SCL_HIGH();
    Soft_IIC_Delay();
    SDA_HIGH();
    Soft_IIC_Delay();
}

unsigned char Soft_IIC_WaitAck(void)
{
    unsigned char ack;
    SDA_HIGH();
    SCL_HIGH();
    Soft_IIC_Delay();
    ack = READ_SDA();
    SCL_LOW();
    Soft_IIC_Delay();
    return ack;
}

void Soft_IIC_SendAck(void)
{
    SDA_LOW();
    SCL_HIGH();
    Soft_IIC_Delay();
    SCL_LOW();
    SDA_HIGH();
}

void Soft_IIC_SendNAck(void)
{
    SDA_HIGH();
    SCL_HIGH();
    Soft_IIC_Delay();
    SCL_LOW();
}

unsigned char Soft_IIC_SendByte(unsigned char dat)
{
    for(unsigned char i = 0; i < 8; i++) {
        (dat & 0x80) ? SDA_HIGH() : SDA_LOW();
        dat <<= 1;
        SCL_HIGH();
        Soft_IIC_Delay();
        SCL_LOW();
        Soft_IIC_Delay();
    }
    return Soft_IIC_WaitAck();
}

unsigned char Soft_IIC_RecvByte(void)
{
    unsigned char dat = 0;
    SDA_HIGH();
    /* 接收数据前切换SDA为输入模式 */
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = SDA_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;        // 关键修改！
    GPIO_InitStruct.Pull = GPIO_PULLUP;            // 保持上拉
    HAL_GPIO_Init(SDA_PORT, &GPIO_InitStruct);
    for(unsigned char i = 0; i < 8; i++) {
        dat <<= 1;
        SCL_HIGH();
        Soft_IIC_Delay();
        if(READ_SDA()) dat |= 0x01;
        SCL_LOW();
        Soft_IIC_Delay();
    }
		
		GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    HAL_GPIO_Init(SDA_PORT, &GPIO_InitStruct);
    return dat;
}

/* 应用层函数改写 */
unsigned char Soft_IIC_ReadByte(unsigned char Salve_Address)
{
    unsigned char dat;
    
    Soft_IIC_Start();
    Soft_IIC_SendByte(Salve_Address | 0x01);  // 读模式
    dat = Soft_IIC_RecvByte();
    Soft_IIC_SendNAck();
    Soft_IIC_Stop();
    
    return dat;
}

unsigned char Soft_IIC_ReadBytes(unsigned char Salve_Address, unsigned char Reg_Address, 
                          unsigned char *Result, unsigned char len)
{
    Soft_IIC_Start();
    if(Soft_IIC_SendByte(Salve_Address & 0xFE)) {  // 写模式
        Soft_IIC_Stop();
        return 0;
    }
    if(Soft_IIC_SendByte(Reg_Address)) {
        Soft_IIC_Stop();
        return 0;
    }
    Soft_IIC_Start();
    if(Soft_IIC_SendByte(Salve_Address | 0x01)) {  // 读模式
        Soft_IIC_Stop();
        return 0;
    }
    
    for(unsigned char i = 0; i < len; i++) {
        Result[i] = Soft_IIC_RecvByte();
        (i == len-1) ? Soft_IIC_SendNAck() : Soft_IIC_SendAck();
    }
    Soft_IIC_Stop();
    return 1;
}

unsigned char Soft_IIC_WriteByte(unsigned char Salve_Address, unsigned char Reg_Address, 
                          unsigned char data)
{
    Soft_IIC_Start();
    if(Soft_IIC_SendByte(Salve_Address & 0xFE)) {  // 写模式
        Soft_IIC_Stop();
        return 0;
    }
    if(Soft_IIC_SendByte(Reg_Address)) {
        Soft_IIC_Stop();
        return 0;
    }
    if(Soft_IIC_SendByte(data)) {
        Soft_IIC_Stop();
        return 0;
    }
    Soft_IIC_Stop();
    return 1;
}

unsigned char Soft_IIC_WriteBytes(unsigned char Salve_Address, unsigned char Reg_Address,
                           unsigned char *data, unsigned char len)
{
    Soft_IIC_Start();
    if(Soft_IIC_SendByte(Salve_Address & 0xFE)) {
        Soft_IIC_Stop();
        return 0;
    }
    if(Soft_IIC_SendByte(Reg_Address)) {
        Soft_IIC_Stop();
        return 0;
    }
    
    for(unsigned char i = 0; i < len; i++) {
        if(Soft_IIC_SendByte(data[i])) {
            Soft_IIC_Stop();
            return 0;
        }
    }
    Soft_IIC_Stop();
    return 1;
}
unsigned char Soft_IIC_Ping(void)
{
	unsigned char dat;
	Soft_IIC_ReadBytes(Color_Adress<<1,PING,&dat,1);
	if(dat==PING_OK)
	{
			return 0;
	}	
	else return 1;
}
unsigned char Soft_IIC_Get_Error(void)
{
	unsigned char dat;
	Soft_IIC_ReadBytes(Color_Adress<<1,Error,&dat,1);
	return dat;
}
unsigned char Soft_IIC_Get_RGB(unsigned char * Result,unsigned char len)
{
	if(Soft_IIC_ReadBytes(Color_Adress<<1,RGB_Reg,Result,len))return 1;
	else return 0;
}
unsigned char Soft_IIC_Get_HSL(unsigned char * Result,unsigned char len)
{
	if(Soft_IIC_ReadBytes(Color_Adress<<1,HSL_Reg,Result,len))return 1;
	else return 0;
}

