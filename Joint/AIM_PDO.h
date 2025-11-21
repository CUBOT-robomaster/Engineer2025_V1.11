#ifndef __AIM_PDO_H__
#define __AIM_PDO_H__
#include "driver_can.h"

typedef struct
{
   CAN_TxBuffer RPDO3_speed;//速度控制
   CAN_TxBuffer RPDO1_pos;//位置模式(绝对位置+立即执行)+位置模式+目标位置，可接收位置状态
   CAN_TxBuffer RPDO2_pos;//位置模式(获取当前位置+设置梯形速度)
}RPDO_COM;


typedef struct
{
   uint32_t RPDO1_ID;
   uint32_t RPDO2_ID;
   float pos;
   uint16_t state;//0x473到达给定位置,0x037未到达
	
}TPDO_COM;

extern RPDO_COM pdo_com;

void RPDO1_pos(float Pos);//位置模式(绝对位置+立即执行)+位置模式+目标位置
void RPDO2_pos(int speed );//位置模式(目标位置+梯形速度)
void YZ_AIM_init();//角度制

CAN_RxBuffer AIMRxCallback(CAN_Object canx, CAN_RxBuffer rxBuffer);
#endif