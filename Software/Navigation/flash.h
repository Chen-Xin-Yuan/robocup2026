#ifndef __FLASH_H
#define __FLASH_H

#include "stm32f4xx_hal.h"

// ===================== F407 官方正确配置 =====================
#define FLASH_USER_START_ADDR   0x080E0000    // 扇区11起始地址
#define FLASH_USER_SECTOR       FLASH_SECTOR_11 // 官方宏
#define MAX_SIZE                100
#define FLASH_BUFFER_SIZE       1024
// ==============================================================

// typedef struct {
//     uint32_t Flash_page_index;
//     uint32_t Save_index;
//     uint8_t  End_f;
// } N_TypeDef;

extern uint32_t flash_union_buffer[FLASH_BUFFER_SIZE];

void flash_buffer_clear(void);
uint8_t flash_check(uint32_t check_addr);
void flash_erase_data_sector(void);
void flash_write_page_from_buffer(uint32_t offset_addr, uint32_t len);
void flash_read_page_to_buffer(uint32_t offset_addr, uint32_t len);

void flash_Nag_Write(void);
void flash_Nag_Read(void);

#endif
