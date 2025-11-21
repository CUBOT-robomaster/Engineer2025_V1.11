#include "motor.h"
#include "dr16.h"
#include "driver_timer.h"
#include "driver_can.h"
#include "interboard.h"
#include "auto.h"
#include "servo.h"
#include "vt13.h"
#include "inverse.h"

int8_t speed_sence = 0;//底盘速度
CAN_TxBuffer DR16Data=
{
  .Identifier=0x101
};
CAN_TxBuffer DR16Data1=
{
  .Identifier=0x102
};
CAN_TxBuffer MouseData=
{
  .Identifier=0x103
};
CAN_TxBuffer VT13Data=
{
  .Identifier=0x104
};
CAN_TxBuffer VT13Data1=
{
  .Identifier=0x105
};
CAN_TxBuffer VT13Data2=
{
  .Identifier=0x106
};
/**
  * @brief   DR16拨杆数据can放入缓存区
  */
void DR16PaddleFillData(RC_Ctrl *rc_ctrl)
{
    DR16Data.Data[0] = rc_ctrl->rc.ch0>>8;
    DR16Data.Data[1] = rc_ctrl->rc.ch0& 0xff;
    DR16Data.Data[2] = rc_ctrl->rc.ch1>>8;
    DR16Data.Data[3] = rc_ctrl->rc.ch1& 0xff;
    DR16Data.Data[4] = rc_ctrl->rc.ch2>>8;
    DR16Data.Data[5] = rc_ctrl->rc.ch2& 0xff;
    DR16Data.Data[6] = rc_ctrl->rc.ch3>>8;
    DR16Data.Data[7] = rc_ctrl->rc.ch3& 0xff;
}
/**
  * @brief   云台和遥控器部分数据放入缓存中
  */
  int angle;
  float a;
void DR16keyFillData(RC_Ctrl *rc_ctrl)
{ 
    uint8_t s1ands1 =( (auto_flag << 4)|(get_double_gold_manual_flag<<6) ) | (move_back_flag);
    uint8_t s1ands2 =( rc_Ctrl.rc.s1 << 4 ) | (rc_Ctrl.rc.s2);
    DR16Data1.Data[0] = rc_ctrl->keyboard.keyboard14;
    DR16Data1.Data[1] = rc_ctrl->keyboard.keyboard15;
    DR16Data1.Data[2] = rc_ctrl->rc.sw>>8;
    DR16Data1.Data[3] = rc_ctrl->rc.sw&0xff;
    DR16Data1.Data[4] = rc_ctrl->isOnline;
    DR16Data1.Data[5] = s1ands1;
    DR16Data1.Data[6] = move_angle;
    DR16Data1.Data[7] = s1ands2;
 
}
/**
  * @brief   鼠标数据放入缓存中，加入自动位移量
  */
void DR16MouseFillData(RC_Ctrl *rc_ctrl)
{ 
    MouseData.Data[0] = rc_ctrl->mouse.x>>8;
    MouseData.Data[1] = rc_ctrl->mouse.x;
    MouseData.Data[2] = rc_ctrl->mouse.y>>8;
    MouseData.Data[3] = rc_ctrl->mouse.y;
    MouseData.Data[4] = (rc_ctrl->mouse.press_l<<4)|rc_ctrl->mouse.press_r;
    MouseData.Data[5] = chassis_x;
    MouseData.Data[6] = (chassis_x>>8);
    MouseData.Data[7] = 0;
 
}
/**
  * @brief   VT13拨杆数据can放入缓存区
  */
void VT13PaddleFillData(VT13 *rc_ctrl)
{
    VT13Data.Data[0] = rc_ctrl->rc.ch0>>8;
    VT13Data.Data[1] = rc_ctrl->rc.ch0& 0xff;
    VT13Data.Data[2] = rc_ctrl->rc.ch1>>8;
    VT13Data.Data[3] = rc_ctrl->rc.ch1& 0xff;
    VT13Data.Data[4] = rc_ctrl->rc.ch2>>8;
    VT13Data.Data[5] = rc_ctrl->rc.ch2& 0xff;
    VT13Data.Data[6] = rc_ctrl->rc.ch3>>8;
    VT13Data.Data[7] = rc_ctrl->rc.ch3& 0xff;
}
/**
  * @brief   键盘及遥控器部分数据放入缓存中
  */
void VT13keyFillData(VT13 *rc_ctrl)
{ 
    uint8_t s1ands1 = rc_ctrl->rc.go_home|(rc_ctrl->rc.Fn<<1)|(rc_ctrl->rc.change<<2)|(rc_ctrl->rc.photo<<3)|(rc_ctrl->rc.mode_sw<<4);
    uint8_t s1ands2 =0;
    VT13Data1.Data[0] = rc_ctrl->keyboard.keyboard14;
    VT13Data1.Data[1] = rc_ctrl->keyboard.keyboard15;
    VT13Data1.Data[2] = rc_ctrl->rc.sw>>8;
    VT13Data1.Data[3] = rc_ctrl->rc.sw&0xff;
    VT13Data1.Data[4] = rc_ctrl->isOnline;
    VT13Data1.Data[5] = s1ands1;
    VT13Data1.Data[6] = move_angle;
    VT13Data1.Data[7] = s1ands2;
 
}
/**
  * @brief   鼠标数据放入缓存中
  */
void VT13MouseFillData(VT13 *rc_ctrl)
{ 
    VT13Data2.Data[0] = rc_ctrl->mouse.x>>8;
    VT13Data2.Data[1] = rc_ctrl->mouse.x;
    VT13Data2.Data[2] = rc_ctrl->mouse.y>>8;
    VT13Data2.Data[3] = rc_ctrl->mouse.y;
    VT13Data2.Data[4] = (rc_ctrl->mouse.press_l<<4)|rc_ctrl->mouse.press_r;
    VT13Data2.Data[5] = chassis_y;
    VT13Data2.Data[6] = (chassis_y>>8);
    VT13Data2.Data[7] = 0x00;
 
}
void board_conmmunity()
{
	if(tim14.ClockTime%5==0)
	{
		DR16PaddleFillData(&rc_Ctrl);
		DR16keyFillData(&rc_Ctrl);
		DR16MouseFillData(&rc_Ctrl);
		CAN_Send(&can2,&DR16Data);
		CAN_Send(&can2,&DR16Data1);
		CAN_Send(&can2,&MouseData);
	}
	if(tim14.ClockTime%5==1)
	{
		VT13PaddleFillData(&vT13);
		VT13keyFillData(&vT13);
		VT13MouseFillData(&vT13);
		CAN_Send(&can2,&VT13Data);
		CAN_Send(&can2,&VT13Data1);
		CAN_Send(&can2,&VT13Data2);
	}
	
}
int8_t move_x_ok;
void BoardCallBack(CAN_Object canx, CAN_RxBuffer rxBuffer)
{
	uint8_t rx[8];
	if(rxBuffer.Header.Identifier == 0x107)
	{
		rx[0] = rxBuffer.Data[0];
		move_x_ok = rx[0];
	}
}
