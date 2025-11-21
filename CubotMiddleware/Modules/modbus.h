#ifndef __MODBUS__H_
#define __MODBUS__H_
#include "stdint.h"
typedef struct
{
    uint8_t TXbuf[64];
    uint8_t TXcount;
    
    
    
}MODBUS_T;
void MODBUS06(uint8_t address,uint16_t reg,uint16_t value);
static void CRCcheck(void);
uint16_t  CRC16modebus(uint8_t *p_buf,uint16_t len);
#endif