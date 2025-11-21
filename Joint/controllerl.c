#include "controllerl.h"
#include "interaction_image.h"
#include "joint_control.h"
#include "mit.h"
#include "dr16.h"
#include "A1_stm32.h"
#include "motor.h"
#include "driver_timer.h"
#include "AIM_PDO.h"
#include "servo.h"
#include "auto.h"
#include "vt13.h"
#include "filter.h"
controller_data Controller;
int CC_start_timer=0;
int CC_back_timer=0;
int8_t CC_back_flag=0;
/**
  * @brief 自定义控制器初始化函数
  */
void Customer_init(){
	Controller.roll2_ccinit = Custom.image_recv.Coordinate.roll2;
	Controller.roll2_init = Manipulator.roll2_deg.angle;
	Controller.pitch3_ccinit = Custom.image_recv.Coordinate.pitch3;
	Controller.pitch3_init = Manipulator.pitch3_deg.angle;
	Controller.pitch2_ccinit = Custom.image_recv.Coordinate.pitch2;
	Controller.pitch2_init = Manipulator.pitch2_deg.angle;
	Controller.roll1_ccinit = Custom.image_recv.Coordinate.roll1;
	Controller.roll1_init = Manipulator.roll1_deg.angle;
	Controller.pitch1_ccinit = Custom.image_recv.Coordinate.pitch1;
	Controller.pitch1_init = Manipulator.pitch1_deg.angle;
	Controller.yaw_ccinit = Custom.image_recv.Coordinate.yaw1;
	Controller.yaw_init = Manipulator.yaw_deg.angle;
	//低通滤波器的参数也要更新，否则lastoutput还保留关闭前的值，再启动计算会跳
	LPF_joint_cc_pitch3.last_output=Custom.image_recv.Coordinate.pitch3-Controller.pitch3_ccinit;
    LPF_joint_cc_roll2.last_output=Custom.image_recv.Coordinate.roll2-Controller.roll2_ccinit;
	LPF_joint_cc_pitch2.last_output=Custom.image_recv.Coordinate.pitch2-Controller.pitch2_ccinit;
	LPF_joint_cc_pitch1.last_output=Custom.image_recv.Coordinate.pitch1-Controller.pitch1_ccinit;
	LPF_joint_cc_yaw.last_output=Custom.image_recv.Coordinate.yaw1-Controller.yaw_ccinit;
	LPF_joint_cc_roll1.last_output=Custom.image_recv.Coordinate.roll1-Controller.roll1_ccinit;

}

/**
  * @brief 关节控制函数-自定义控制器
  */
void Joint_control_cc()
{
	if(associate_stretch_left_flag == 0 && associate_stretch_right_flag == 0 &&associate_arm_left_flag ==0)
	{
		if(move_angle==0&&rc_Ctrl.key_F == 1&&rc_Ctrl.key_ctrl==0&&rc_Ctrl.key_shift==0)
		Custom.image_recv.Coordinate.cc_control_flag =1;
		if(move_angle==0&&rc_Ctrl.key_V == 1&&rc_Ctrl.key_ctrl==0&&rc_Ctrl.key_shift==0)
		Custom.image_recv.Coordinate.cc_control_flag =0;
		if(move_angle==0&&vT13.key_F == 1&&vT13.key_ctrl==0&&vT13.key_shift==0)
		Custom.image_recv.Coordinate.cc_control_flag =1;
		if(move_angle==0&&vT13.key_V == 1&&vT13.key_ctrl==0&&vT13.key_shift==0)
		Custom.image_recv.Coordinate.cc_control_flag =0;
	}
	
	if(Custom.image_recv.Coordinate.cc_control_flag == 1&& move_angle==0)
	{	
    CC_back_flag=1;//回收标志位，表示可以收回
		CC_start_timer++;

		if(CC_start_timer<=5){
			Customer_init();//更新初始化自定义控制器启动时各关节角度，否则沿用上一次的值会有很大跳变值
		}
		if(CC_start_timer > 5)
		{
			Customer_controller();//开始更新机械臂目标角度
		}
	}
		

	if(Custom.image_recv.Coordinate.cc_control_flag == 0 && CC_back_flag==1)//关闭自定义控制器，需要判断是不是启动过
	{
		CC_start_timer=0;
		CC_back_timer++;
		if(CC_back_timer==100)
		{
		  servo_move(4,50,620);
		  Manipulator.yaw_deg.angle_target=0;
		  Manipulator.roll1_deg.angle_target=0;
		  Manipulator.roll2_deg.angle_target=0;
		  Manipulator.pitch3_deg.angle_target=0;
		} 
		if(CC_back_timer>=500)
		{
		  servo_move(2,30,290);
		  Manipulator.pitch1_deg.angle_target=0;
		  Manipulator.pitch2_deg.angle_target=0;
		 roll2_feed = 0;
		  roll1_feed = 0;
	      pitch3_feed = 30;
			pitch2_feed =20;
			pitch1_feed = -30;
			CC_back_timer=0;
		    CC_back_flag=0;
		}
	}
}


/**
  * @brief 自定义控制器角度映射
  */
void Customer_controller()
{
	Controller.roll2=LPFilter(Custom.image_recv.Coordinate.roll2-Controller.roll2_ccinit,&LPF_joint_cc_roll2);
	Controller.pitch3=LPFilter(Custom.image_recv.Coordinate.pitch3-Controller.pitch3_ccinit,&LPF_joint_cc_pitch3)*1.1;
	//Controller.pitch2=LPFilter(Custom.image_recv.Coordinate.pitch2-Controller.pitch2_ccinit,&LPF_joint_cc_pitch2)*1.2;
	//不敢用低通滤波了，会突然挑一个很大的值，去年也有，没细找原因
	//在低通滤波输出计算中限制了跳变，pitch1目前没出现，pitch2应该也可以使用了
	Controller.pitch2=(Custom.image_recv.Coordinate.pitch2-Controller.pitch2_ccinit)*1.2;
	Controller.roll1=LPFilter(Custom.image_recv.Coordinate.roll1-Controller.roll1_ccinit,&LPF_joint_cc_roll1)*1.1;
	Controller.pitch1=LPFilter(Custom.image_recv.Coordinate.pitch1-Controller.pitch1_ccinit,&LPF_joint_cc_pitch1)*1.1;
	Controller.yaw=LPFilter(Custom.image_recv.Coordinate.yaw1-Controller.yaw_ccinit,&LPF_joint_cc_yaw)*1.5;
	//每个关节变化乘一个系数，是为了加大控制范围
}


