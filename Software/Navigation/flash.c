#include "flash.h"
#include "navigation.h"

//�����ؼ���ɣ�flash <--> buffer <--> varible  ���������뻺������������������flash
uint32_t flash_union_buffer[FLASH_BUFFER_SIZE] = {0};
// N_TypeDef N = {0};

void flash_buffer_clear(void)
{
    for(uint32_t i=0; i<FLASH_BUFFER_SIZE; i++)
        flash_union_buffer[i] = 0;
}

// ���ܣ���� 2048 �ֽڵ� Flash �Ƿ�Ϊ�հף�ȫ 0xFF��
// ������check_addr = Ҫ���� Flash ��ʼ��ַ
// ���أ�1=ȫ�գ�0=������
uint8_t flash_check(uint32_t check_addr)
{
    for(uint32_t i=0; i<2048/4; i++)
    {
        if(*(uint32_t*)(check_addr + i*4) != 0xFFFFFFFF)
            return 0;
    }
    return 1;
}

// ===================== F407 ��ȷ�������� =====================
void flash_erase_data_sector(void)
{
    FLASH_EraseInitTypeDef EraseInitStruct = {0};
    uint32_t PAGEError = 0;

    HAL_FLASH_Unlock();

    EraseInitStruct.TypeErase   = FLASH_TYPEERASE_SECTORS;
    EraseInitStruct.Banks       = FLASH_BANK_1;
    EraseInitStruct.Sector      = FLASH_USER_SECTOR;  // ����11
    EraseInitStruct.NbSectors   = 1;
    EraseInitStruct.VoltageRange = FLASH_VOLTAGE_RANGE_3;

    HAL_FLASHEx_Erase(&EraseInitStruct, &PAGEError);

    HAL_FLASH_Lock();
}

void flash_write_page_from_buffer(uint32_t offset_addr, uint32_t len)
{
    uint32_t addr = FLASH_USER_START_ADDR + offset_addr;

    HAL_FLASH_Unlock();
    for(uint32_t i=0; i<len/4; i++)
    {
        HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr, flash_union_buffer[i]);
        addr += 4;
    }
    HAL_FLASH_Lock();
}

void flash_read_page_to_buffer(uint32_t offset_addr, uint32_t len)
{
    uint32_t addr = FLASH_USER_START_ADDR + offset_addr;
    for(uint32_t i=0; i<len/4; i++)
    {
        flash_union_buffer[i] = *(uint32_t*)addr;
        addr += 4;
    }
}

// ===================== ���ҵ���߼� =====================
//-------------------------------------------------------------------------------------------------------------------
// �������     �����ݻ�����������ݣ�д�뵽 Flash ��ָ��ҳ
// ����˵��     ���ڴ洢������·���� (X,Y,Theta), ��ҳ150��
// ���ز���     void
// ʹ��ʾ��
// ��ע��Ϣ     ����ԭ����ҳƫ��bug��ҳ��Ҫ��2048�����ֽ�ƫ��
//-------------------------------------------------------------------------------------------------------------------
void flash_Nag_Write(void)
{
    // N.Flash_page_index �������ֽ�ƫ�� (�� Init_Nag ��ʼ��ʱ�� Nag_Start_Page * 2048 ����)
    uint32_t offset = N.Flash_page_index;

    if(flash_check(FLASH_USER_START_ADDR + offset))
        flash_erase_data_sector();//��������ݾͲ���

    flash_write_page_from_buffer(offset, 2048);

    if(N.End_f == 1)
    {
        // ��������·������浽��0ҳ������λ�� (MAX_SIZE+2 = 102)
        flash_union_buffer[MAX_SIZE + 2] = N.Save_index;
        flash_write_page_from_buffer(0, 2048);
    }

    flash_buffer_clear();
    N.Flash_page_index -= 2048;  // ��һҳ (�ֽ�ƫ���ݼ�)
}

void flash_Nag_Read(void)
{
    static uint8_t Index_R_f = 0;
    flash_buffer_clear();

    if(Index_R_f == 0)
    {
        // ��ȡ��0ҳ��������·���� (������1023λ��)
        flash_read_page_to_buffer(0, 2048);
        N.Save_index = flash_union_buffer[FLASH_BUFFER_SIZE - 1];
        Index_R_f = 1;
        flash_buffer_clear();
    }

    uint32_t offset = N.Flash_page_index;
    if(!flash_check(FLASH_USER_START_ADDR + offset))
    {
        flash_read_page_to_buffer(offset, 2048);
    }
}
