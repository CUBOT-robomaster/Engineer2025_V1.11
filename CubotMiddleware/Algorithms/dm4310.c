#include "dm4310.h"
#include "driver_can.h"
#include "stdint.h"
 CAN_TxBuffer dm4310;
 CAN_TxBuffer motorstart;
 MIT4310_control mit4310_control;
void MIT4310FillData(MIT4310_control*mit,uint16_t id,uint16_t position,uint16_t velocity,uint16_t Kp,uint16_t Kd,uint16_t torque)
{
    mit->id=id;
    mit->position=position;
    mit->velocity=velocity;
    mit->Kp=Kp;
    mit->Kd=Kd;
    mit->torque=torque;
}
void motor4310MITcontrol(CAN_Object*hcan,MIT4310_control*mit)
{
    dm4310.Identifier=mit->id;
    dm4310.Data[0]=(mit->position>>8)&0xff;
    dm4310.Data[1]=mit->position&0xff;
    dm4310.Data[2]=(mit->velocity>>4)&0xff;
    dm4310.Data[3]=(mit->velocity&0x0f)|((mit->Kp>>8)&0x0f);
    dm4310.Data[4]=mit->Kp&0xff;
    dm4310.Data[5]=(mit->Kd>>4)&0xff;
    dm4310.Data[6]=(mit->Kd&0x0f)|((mit->torque>>8)&0x0f);
    dm4310.Data[7]=mit->torque&0xff;
    CAN_Send(hcan,&dm4310);
}
void mid4310motor_start(CAN_Object*hcan,uint16_t id)
{
    dm4310.Identifier=id;
    dm4310.Data[0]=0xFF;
    dm4310.Data[1]=0xFF;
    dm4310.Data[2]=0xFF;
    dm4310.Data[3]=0xFF;
    dm4310.Data[4]=0xFF;
    dm4310.Data[5]=0xFF;
    dm4310.Data[6]=0xFF;
    dm4310.Data[7]=0xFC;
    CAN_Send(hcan,&motorstart);
    
}


