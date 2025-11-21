#ifndef __DM4310__H_
#define __DM4310__H_
#include "stdint.h"
#include "driver_can.h"
typedef struct 
{
    uint16_t id;
    uint16_t position;
    uint16_t velocity;
    uint16_t  Kp;
    uint16_t  Kd;
    uint16_t  torque;
}MIT4310_control;
void MIT4310FillData(MIT4310_control*mit,uint16_t id,uint16_t position,uint16_t velocity,uint16_t Kp,uint16_t Kd,uint16_t torque);
void motor4310MITcontrol(CAN_Object*hcan,MIT4310_control*mit);
void mid4310motor_start(CAN_Object*hcan,uint16_t id);
extern CAN_TxBuffer dm4310;
extern MIT4310_control mit4310_control;


#endif