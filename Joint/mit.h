#ifndef _MIT_H
#define _MIT_H
#include <driver_can.h>
#include "stm32h7xx_hal.h"

typedef struct
{
  uint8_t CAN_ID;
  uint8_t id;
  uint8_t Error;
  float pos;
  float vel;
  float T;
  int8_t T_mos;//驱动mos温度
  int8_t T_Rotor;//电机线圈温度
	
}DM_motor;



void ctrl_motor1(CAN_Object* hcan,DM_motor *Dm_motor, float _pos, float _vel, float _KP, float _KD, float _torq);
void ctrl_motor2(CAN_Object* hcan,DM_motor *Dm_motor, float _pos, float _vel);
void ctrl_motor3(CAN_Object* hcan,DM_motor *Dm_motor, float _vel);
void start_motor(CAN_Object* hcan,DM_motor *Dm_motor);
void lock_motor(CAN_Object* hcan,DM_motor *Dm_motor);
void DM_init();
void DM_Restart();

CAN_RxBuffer MitRxCallback(CAN_Object canx, CAN_RxBuffer rxBuffer);


#endif
