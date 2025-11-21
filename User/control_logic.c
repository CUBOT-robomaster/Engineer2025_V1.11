#include "usart.h"
#include "hardware_config.h"
#include "control_logic.h"
#include "motor.h"
#include "dr16.h"
#include "usart.h"
#include "driver_timer.h"
#include "referee.h"
#include "check.h"
#include "gold.h"
#include "servo.h"
#include "air.h"
#include "mit.h"
#include "interboard.h"
#include "joint_control.h"
#include "A1.h"
#include "A1_stm32.h"
#include "AIM_SDO.h"
#include "AIM_PDO.h"
#include "auto.h"
#include "interaction_image.h"
#include "inverse.h"
#include "vision_community.h"
#include "vt13.h"
//< TIM14的触发频率在CubeMX中被配置为1000Hz

CAN_RxBuffer Rxbuffer;
float target_speed=0;
int8_t DM_restart_flag = 0;

float target_x[3] ={-498,0,440},target_angle[3] = {0,0,180};

/**
  * @brief 遥控器保护
  */
  void robot_offline_protection()
{
    RobotOnlineState(&check_robot_state);
    if( check_robot_state.usart_state.Check_receiver>50&&vT13.rc.mode_sw==0)
	{
		rc_Ctrl.isOnline=0;//遥控离线
	}
	else
	{
		rc_Ctrl.isOnline=1;
	}
	if( check_robot_state.usart_state.Check_vision>1000)
	{
		Vision_t.online = 0;//相机离线
	}
	else
	{
		Vision_t.online=1;
	}
	if( check_robot_state.usart_state.Check_custom>1000)
	{
		Custom.image_recv.Coordinate.isonline = 0;//自定义控制器离线
	}
	else
	{
		Custom.image_recv.Coordinate.isonline = 1;
	}
	
}
void TIM14_Task()
{
	tim14.ClockTime++;
    robot_offline_protection();//遥控保护
	board_conmmunity();//双板通信
	UI_show();
	if(tim14.ClockTime<500)//在这做一些必要的初始化
	{ 
		YZ_AIM_init();//
		A1_init();		//A1 初始化
		ServoSM45BL_init();
		
	}
    if(tim14.ClockTime>=500&&tim14.ClockTime<=2000)
	{
			start_motor(&can1,&Manipulator.Dm_4310_pitch3);
			start_motor(&can1,&Manipulator.Dm_4310_roll2);
			start_motor(&can1,&Manipulator.Dm_4310_roll1);
			start_motor(&can2,&Manipulator.Dm_8006_yaw);
			DM_init();//使能达妙电机并初始化
	}
	
	if(tim14.ClockTime>2000&&tim14.ClockTime<=5000)
	{
		roll1_reset();//roll1复位
	}
	if(tim14.ClockTime>=2000)
   {
        //imagetrans_recv_datas_modify(recv_Buffer,&Custom);//图传传输数据解算
	    joint_Ctrl(&Manipulator); //机械臂关节控制(总)
     	get_current_angle(&Manipulator);//获取逆解坐标系下的关节角度                  
        air_control(&rc_Ctrl);//气泵
	    servo_control();//飞特舵机控制
	    watch_control( &rc_Ctrl);//图传视角控制
	    auto_judge();//自动模式判断
	    auto_mode();//自动模式执行
        Angle_add(&Goldmining_t,&rc_Ctrl);//取矿机构控制，建议角度累加计算和控制分开
		if(tim14.ClockTime>4000)
			gold_reset(&Goldmining_t);//取矿机构复位
		if(tim14.ClockTime%10 == 0)
		{
			inverse_test(&Manipulator);//自动兑矿执行函数
			//vision_coil_test(target_angle,target_x);//测试机械臂逆解函数
		}
		//Usart8DmaPrintf("%f,%f\n",Vision_t.matrix_data[2],Vision_t.matrix_data[4]);
	}
   

    if(rc_Ctrl.isOnline==0)
{
	if(tim14.ClockTime>10000)
	{
		MotorFillData(&Goldmining_t.Motors2006.goldmotor[0],0);
        MotorFillData(&Goldmining_t.Motors2006.goldmotor[1],0);
        MotorFillData(&Goldmining_t.Motors2006.goldmotor[2],0);
        MotorFillData(&Goldmining_t.Motors2006.goldmotor[3],0);
		//进入阻尼模式
		Angle_add_init(&Goldmining_t);
//		if(rc_Ctrl.key_shift== 1&& rc_Ctrl.key_G == 1)
//		{
//			DM_restart_flag =1;
//			joint_reset_flag = 1;
//			DM_Restart();
//		}//用来手动使能达妙，该方法没有用
		if(joint_reset_flag == 0)//不执行手动使能达妙操作执行
		{
			A1_init();		//A1 初始化
		    DM_init();
		
			if(tim14.ClockTime%2==0)
			{
				ctrl_motor1(&can1,&Manipulator.Dm_4310_pitch3,0,0,0,0.3,0);
				ctrl_motor1(&can1,&Manipulator.Dm_4310_roll2,0,0,0,0.1,0);
				
				ctrl_motor1(&can2,&Manipulator.Dm_8006_yaw,0,0,0,1.5,0);
			}
			if(tim14.ClockTime%2==1)
			{
				ctrl_motor1(&can1,&Manipulator.Dm_4310_roll1,0,0,0,0.3,0);
			}
		}
	}
}
      if(tim14.ClockTime%2==0)
	  A1_send(&huart4, &Motor_zero_send);			//A1发送函数
      

	  if(tim14.ClockTime%2==1)
	   MotorCanOutput(can2, 0x200);
}


/**
  * @brief  CAN1接收中断回调
  */
uint8_t CAN1_rxCallBack(CAN_RxBuffer* rxBuffer)
{
	MotorRxCallback(can1, (*rxBuffer)); 
	MitRxCallback(can1, (*rxBuffer));
    Rxbuffer =AIMRxCallback(can1, (*rxBuffer));		
	return 0;
}

/**
  * @brief  CAN2接收中断回调
  */
uint8_t CAN2_rxCallBack(CAN_RxBuffer* rxBuffer)
{	
	BoardCallBack(can2, (*rxBuffer));	
	MitRxCallback(can2, (*rxBuffer));
	MotorRxCallback(can2, (*rxBuffer)); 

	return 0;
}
