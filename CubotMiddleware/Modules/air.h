#ifndef __AIR_H_
#define __AIR_H_
#include "stm32h7xx_hal.h"
#include "tim.h"
#include "dr16.h"

#define open 1
#define close 0
void Vacuum_pump_main(uint8_t order);//主真空泵
void Vacuum_pump_vice(uint8_t order);//副真空泵	
void air_init();
void valve_fetch(int8_t state);
void arm_fetch(int8_t state);
void arm_relese(int8_t state);
void air_control(RC_Ctrl *rc_ctrl);
#endif