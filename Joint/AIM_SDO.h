#ifndef __AIM_SDO_H__
#define __AIM_SDO_H__
#include "driver_can.h"

typedef struct
{
   CAN_TxBuffer SDO_control;//控制段
   CAN_TxBuffer SDO_set;//设置工作模式
   CAN_TxBuffer SDO_pos_read;//读取当前位置
   CAN_TxBuffer SDO_back;//回原点
}SDO_COM;


extern SDO_COM sdo_com;

void get_zero();
void pos_set();//设位置模式函数
void  motor_start();//启动+电压输出+允许急停+允许操作

#endif