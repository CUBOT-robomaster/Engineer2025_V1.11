#include "driver_can.h"
#include "AIM_SDO.h"

//CAN_TxBuffer SDO_control={
//	.Identifier = 0x601,
//	.Data[0]=0x2B,
//	.Data[1]=0x40,
//	.Data[2]=0x60,
//	.Data[3]=0x00,
//	.Data[4]=0x0F,
//	.Data[5]=0x0F,
//	.Data[6]=0x00,
//	.Data[7]=0x00
//};
//CAN_TxBuffer SDO_pos_set={
//	.Identifier = 0x601,
//	.Data[0]=0x2F,
//	.Data[1]=0x60,
//	.Data[2]=0x60,
//	.Data[3]=0x00,
//	.Data[4]=0x01
//};
//CAN_TxBuffer SDO_pos_read={
//	.Identifier = 0x601,
//	.Data[0]=0x40,
//	.Data[1]=0x64,
//	.Data[2]=0x60,
//	.Data[3]=0x00,
//};


SDO_COM sdo_com=        //初始化数据
{
	.SDO_control.Identifier = 0x601,
	.SDO_control.Data[0]=0x2B,
	.SDO_control.Data[1]=0x40,
	.SDO_control.Data[2]=0x60,
	.SDO_control.Data[3]=0x00,
	.SDO_control.Data[4]=0x0F,
	.SDO_control.Data[5]=0x00,
	.SDO_control.Data[6]=0x00,
	.SDO_control.Data[7]=0x00,
	
	.SDO_set.Identifier = 0x601,
	.SDO_set.Data[0]=0x2F,
	.SDO_set.Data[1]=0x60,
	.SDO_set.Data[2]=0x60,
	.SDO_set.Data[3]=0x00,
	.SDO_set.Data[4]=0x01,
	
	.SDO_pos_read.Identifier = 0x601,
	.SDO_pos_read.Data[0]=0x40,
	.SDO_pos_read.Data[1]=0x64,
	.SDO_pos_read.Data[2]=0x60,
	.SDO_pos_read.Data[3]=0,
	
	.SDO_back.Identifier = 0x601,
	.SDO_back.Data[0]=0x2F,
	.SDO_back.Data[1]=0x60,
	.SDO_back.Data[2]=0x60,
	.SDO_back.Data[3]=0X00,
	.SDO_back.Data[4]=0X06,
	

};
void get_zero()//找原点函数
{
    CAN_Send (&can2,&sdo_com.SDO_back);
	CAN_Send (&can2,&sdo_com.SDO_back);
   sdo_com.SDO_set.Data[4]=0X06;//设置为找原点模式
   CAN_Send (&can2,&sdo_com.SDO_set);
}

void  motor_start()//启动+电压输出+允许急停+允许操作
{
   CAN_Send (&can2,&sdo_com.SDO_control);
}


void pos_set()//设位置模式函数
{
     sdo_com.SDO_set.Data[4]=0X01;//设置为位置模式
     CAN_Send (&can2,&sdo_com.SDO_set);
}









