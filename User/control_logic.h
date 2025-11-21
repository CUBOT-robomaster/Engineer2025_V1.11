#ifndef CONTROLLOGIC_H_
#define CONTROLLOGIC_H_
#include "stm32h7xx_hal.h"
#include "dr16.h"
#include "motor.h"
	
#define AtR 0.0174532f
#define RtA 57.296083f

uint8_t CAN1_rxCallBack(CAN_RxBuffer* rxBuffer);

uint8_t CAN2_rxCallBack(CAN_RxBuffer* rxBuffer);
extern int8_t DM_restart_flag;
void robot_offline_protection(void );
void TIM14_Task();

#endif



