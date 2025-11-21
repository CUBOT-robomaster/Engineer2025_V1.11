#include "joint_control.h"
#include "mit.h"
#include "dr16.h"
#include "A1_stm32.h"
#include "motor.h"
#include "driver_timer.h"
#include "AIM_PDO.h"
#include "interaction_image.h"
#include "controllerl.h"
#include "velocity_pl.h"
#include "auto.h"
#include "vt13.h"
/**
  * @brief 关节控制模块
  */
  
int8_t joint_reset_flag=1;  //roll1复位标志位
int16_t joint_reset_count=1;  //roll1复位标志位
/**
* @brief 初始角度roll2:0.46，pitch3:1.32
  */  
Manipulator_t Manipulator=
{
	//初始化电机ID
   .Dm_4310_pitch3.CAN_ID=0x01,
   .Dm_4310_roll2.CAN_ID=0x02,
   .Dm_8006_yaw.CAN_ID=0x03,
   .Dm_4310_roll1.CAN_ID=0x04,
	//初始化目标角度
   .roll2_deg.angle_target=0,
   .pitch3_deg.angle_target=0,
   .yaw_deg.angle_target=0,
	//初始化运动量
   .roll1_deg.velocity=0,
   .roll1_deg.max_velocity=0.16,
   .roll1_deg.min_velocity=0.06,
   .roll1_deg.Accel=0.003,
   .roll2_deg.velocity=0,
   .roll2_deg.max_velocity=0.09,
   .roll2_deg.min_velocity=0.03,
   .roll2_deg.Accel=0.0015,
   .pitch1_deg.velocity=0,
   .pitch1_deg.max_velocity=0.05,
   .pitch1_deg.min_velocity=0.01,
   .pitch1_deg.Accel=0.0008,
   .pitch2_deg.velocity=0,
   .pitch2_deg.max_velocity=0.05,
   .pitch2_deg.min_velocity=0.025,
   .pitch2_deg.Accel=0.0002,
   .pitch3_deg.velocity=0,
   .pitch3_deg.max_velocity=0.09,
   .pitch3_deg.min_velocity=0.03,
   .pitch3_deg.Accel=0.00012,
   .yaw_deg.velocity=0,
   .yaw_deg.max_velocity=0.09,
   .yaw_deg.min_velocity=0.03,
   .yaw_deg.Accel=0.00012

};

extern struct joint_deg Joint_deg;
int inverse_count=0;
int8_t inverse_flag , inverse_success_flag;//逆解标志位
/**
* @brief 关节初始化
  */ 
void joint_init()
{
	    //YZ_AIM_init();
		A1_init();		//A1 初始化
	    DM_init();
	    
}

/**
* @brief 关节限制函数。防止堵转
  */ 
float joints_limit(float target_angle,float min_angle,float max_angle)
{
	if(target_angle > max_angle)
		target_angle = max_angle;
	if(target_angle < min_angle)
		target_angle = min_angle;
	return target_angle;
}
/**
* @brief roll2关节控制
  */ 
void roll2_Ctrl(Manipulator_t* manipulator)
{
	if(rc_Ctrl.rc.ch0>=1124&&(rc_Ctrl.rc.s1==2))
		manipulator->roll2_deg.angle_target+=0.06;
	else if(rc_Ctrl.rc.ch0<=924&&(rc_Ctrl.rc.s1==2))
		manipulator->roll2_deg.angle_target-=0.06;
	
	if((rc_Ctrl.key_ctrl ==1||vT13.key_ctrl ==1) &&(rc_Ctrl.key_D==1||vT13.key_D==1))
		manipulator->roll2_deg.angle_target+=0.03;
	else if((rc_Ctrl.key_ctrl ==1||vT13.key_ctrl ==1) &&(rc_Ctrl.key_A==1||vT13.key_A ==1))
		manipulator->roll2_deg.angle_target-=0.03;//微调roll2.视觉兑矿用
//	if(manipulator->roll2_deg.angle_target+Manipulator.roll2_deg.angle_init>manipulator->roll2_deg.angle+0.06)
//		manipulator->roll2_deg.angle+=0.06;
//	else if(manipulator->roll2_deg.angle_target+Manipulator.roll2_deg.angle_init<manipulator->roll2_deg.angle-0.06)
//		manipulator->roll2_deg.angle-=0.06;
	manipulator->roll2_deg.angle_target = joints_limit(manipulator->roll2_deg.angle_target,-180,180);
	if(manipulator->roll2_deg.angle_target+Manipulator.roll2_deg.angle_init>manipulator->roll2_deg.angle+0.01)
		manipulator->roll2_deg.angle+=velocity_plan(&manipulator->roll2_deg);
	else if(manipulator->roll2_deg.angle_target+Manipulator.roll2_deg.angle_init<manipulator->roll2_deg.angle-0.01)
		manipulator->roll2_deg.angle-=velocity_plan(&manipulator->roll2_deg);
	manipulator->roll2_deg.rad=manipulator->roll2_deg.angle*AtR;
	
}

/**
* @brief pitch3关节控制
  */ 
void pitch3_Ctrl(Manipulator_t* manipulator)
{
	if(rc_Ctrl.rc.ch1>=1124&&rc_Ctrl.rc.s1==2)
		manipulator->pitch3_deg.angle_target+=0.04;
	else if(rc_Ctrl.rc.ch1<=924&&rc_Ctrl.rc.s1==2)
		manipulator->pitch3_deg.angle_target-=0.04;
//	if(manipulator->pitch3_deg.angle_target+Manipulator.pitch3_deg.angle_init>manipulator->pitch3_deg.angle+0.04)
//		manipulator->pitch3_deg.angle+=0.04;
//	else if(manipulator->pitch3_deg.angle_target+Manipulator.pitch3_deg.angle_init<manipulator->pitch3_deg.angle-0.04)
//		manipulator->pitch3_deg.angle-=0.04;
	manipulator->pitch3_deg.angle_target = joints_limit(manipulator->pitch3_deg.angle_target,-125,125);

	if(manipulator->pitch3_deg.angle_target + manipulator->pitch3_deg.angle_init > manipulator->pitch3_deg.angle + 0.01)
		manipulator->pitch3_deg.angle += velocity_plan(&manipulator->pitch3_deg);
	else if(manipulator->pitch3_deg.angle_target + manipulator->pitch3_deg.angle_init < manipulator->pitch3_deg.angle - 0.01)
		manipulator->pitch3_deg.angle -= velocity_plan(&manipulator->pitch3_deg); 
	manipulator->pitch3_deg.rad=manipulator->pitch3_deg.angle*AtR;
	
}

/**
* @brief pitch2关节控制
  */ 
void pitch2_Ctrl(Manipulator_t* manipulator)
{
	if(rc_Ctrl.rc.ch2>=1124&&rc_Ctrl.rc.s1==2)
		manipulator->pitch2_deg.angle_target+=0.01;
	else if(rc_Ctrl.rc.ch2<=924&&rc_Ctrl.rc.s1==2)
		manipulator->pitch2_deg.angle_target-=0.01;
	if(manipulator->pitch2_deg.angle_target<0)
		manipulator->pitch2_deg.angle_target=0;
//	if(manipulator->pitch2_deg.angle_target+manipulator->pitch2_deg.angle_init>manipulator->pitch2_deg.angle+0.018)
//		manipulator->pitch2_deg.angle+=0.018;
//	if(manipulator->pitch2_deg.angle_target+manipulator->pitch2_deg.angle_init<manipulator->pitch2_deg.angle-0.018)
//		manipulator->pitch2_deg.angle-=0.018;
	
	manipulator->pitch2_deg.angle_target = joints_limit(manipulator->pitch2_deg.angle_target,0,140);

	if(manipulator->pitch2_deg.angle_target+manipulator->pitch2_deg.angle_init>manipulator->pitch2_deg.angle+0.01)
		manipulator->pitch2_deg.angle+=velocity_plan(&manipulator->pitch2_deg);
	if(manipulator->pitch2_deg.angle_target+manipulator->pitch2_deg.angle_init<manipulator->pitch2_deg.angle-0.01)
		manipulator->pitch2_deg.angle-=velocity_plan(&manipulator->pitch2_deg);
	
	manipulator->pitch2_deg.rad=(manipulator->pitch2_deg.angle*AtR*9.1);
	
}
/**
* @brief roll1关节控制
  */ 
void roll1_Ctrl(Manipulator_t* manipulator)
{
	if(rc_Ctrl.rc.sw>=1124&&(rc_Ctrl.rc.s1==2))
		manipulator->roll1_deg.angle_target+=0.05;
	else if(rc_Ctrl.rc.sw<=924&&(rc_Ctrl.rc.s1==2))
		manipulator->roll1_deg.angle_target-=0.05;

//	if(manipulator->roll1_deg.angle_target+Manipulator.roll1_deg.angle_init>manipulator->roll1_deg.angle+0.06)
//		manipulator->roll1_deg.angle+=0.06;
//	else if(manipulator->roll1_deg.angle_target+Manipulator.roll1_deg.angle_init<manipulator->roll1_deg.angle-0.06)
//		manipulator->roll1_deg.angle-=0.06;
	manipulator->roll1_deg.angle_target = joints_limit(manipulator->roll1_deg.angle_target,-130,130);

	if(manipulator->roll1_deg.angle_target+Manipulator.roll1_deg.angle_init>manipulator->roll1_deg.angle+0.01)
		manipulator->roll1_deg.angle+=velocity_plan(&manipulator->roll1_deg);
	else if(manipulator->roll1_deg.angle_target+Manipulator.roll1_deg.angle_init<manipulator->roll1_deg.angle-0.01)
		manipulator->roll1_deg.angle-=velocity_plan(&manipulator->roll1_deg);
	manipulator->roll1_deg.rad=manipulator->roll1_deg.angle*AtR;
}
 
/**
* @brief pitch1关节控制,升高，角度减小
  */ 
void pitch1_Ctrl(Manipulator_t* manipulator)
{
	if(rc_Ctrl.rc.ch3>=1124&&rc_Ctrl.rc.s1==2)
		manipulator->pitch1_deg.angle_target-=0.01;
	else if(rc_Ctrl.rc.ch3<=924&&rc_Ctrl.rc.s1==2)
		manipulator->pitch1_deg.angle_target+=0.01;
//	if(manipulator->pitch1_deg.angle_target+manipulator->pitch1_deg.angle_init>manipulator->pitch1_deg.angle+0.015)
//		manipulator->pitch1_deg.angle+=0.015;
//	else if(manipulator->pitch1_deg.angle_target+manipulator->pitch1_deg.angle_init<manipulator->pitch1_deg.angle-0.015)
//		manipulator->pitch1_deg.angle-=0.015;
	manipulator->pitch1_deg.angle_target = joints_limit(manipulator->pitch1_deg.angle_target,-130,0);
	if(manipulator->pitch1_deg.angle_target>0)
		manipulator->pitch1_deg.angle_target=0; 
	if(manipulator->pitch1_deg.angle_target+manipulator->pitch1_deg.angle_init>manipulator->pitch1_deg.angle+0.01)
		manipulator->pitch1_deg.angle+=velocity_plan(&manipulator->pitch1_deg);
	else if(manipulator->pitch1_deg.angle_target+manipulator->pitch1_deg.angle_init<manipulator->pitch1_deg.angle-0.01)
		manipulator->pitch1_deg.angle-=velocity_plan(&manipulator->pitch1_deg); 

}

/**
* @brief yaw关节控制,升高，角度减小

  */ 
void yaw_Ctrl(Manipulator_t* manipulator)
{
	if(rc_Ctrl.rc.sw>=1124&&rc_Ctrl.rc.s1==2)
		manipulator->yaw_deg.angle_target+=0.01;
	else if(rc_Ctrl.rc.sw<=924&&rc_Ctrl.rc.s1==2)
		manipulator->yaw_deg.angle_target-=0.01;
//	if(manipulator->yaw_deg.angle_target+Manipulator.yaw_deg.angle_init>manipulator->yaw_deg.angle+0.025)
//		manipulator->yaw_deg.angle+=0.025;
//	else if(manipulator->yaw_deg.angle_target+Manipulator.yaw_deg.angle_init<manipulator->yaw_deg.angle-0.025)
//		manipulator->yaw_deg.angle-=0.025;
	if( manipulator->pitch1_deg.angle >-22)
		manipulator->yaw_deg.angle_target = joints_limit(manipulator->yaw_deg.angle_target,-43,64);
	if(manipulator->yaw_deg.angle_target+Manipulator.yaw_deg.angle_init>manipulator->yaw_deg.angle+0.01)
		manipulator->yaw_deg.angle+=velocity_plan(&manipulator->yaw_deg);
	else if(manipulator->yaw_deg.angle_target+Manipulator.yaw_deg.angle_init<manipulator->yaw_deg.angle-0.01)
		manipulator->yaw_deg.angle-=velocity_plan(&manipulator->yaw_deg);
	manipulator->yaw_deg.rad=manipulator->yaw_deg.angle*AtR;
}
/**
* @brief rool1复位，确保roll1转到正确位置
  */ 
void roll1_reset()
{
	joint_reset_count ++;
	if(joint_reset_count == 10)
	Manipulator.pitch2_deg.angle_target = 5.8;
	if(joint_reset_count == 1000)
	{
		Manipulator.roll1_deg.angle_init = 1.44*RtA;
	}
	if(joint_reset_count == 1500)
		Manipulator.pitch2_deg.angle_target = 0;
	if(joint_reset_count == 2500)
	   joint_reset_flag = 0;
}
/**
* @brief 自定义控制器关节
  */ 

double v1,v2;
double Holder_TD(float Expect,float r,float h)
{
    double fh= -r * r * (v1 - Expect) - 2 * r * v2;
    v1 += v2 * h;
    v2 += fh * h;
        return v1;
}
void controller_control(Manipulator_t* manipulator)
{
	if(CC_start_timer > 1000)
	{
		//roll2
		manipulator->roll2_deg.angle_target =-Controller.roll2 + roll2_feed;
		manipulator->roll2_deg.angle_target = joints_limit(manipulator->roll2_deg.angle_target,-180,180);


		manipulator->roll2_deg.angle = manipulator->roll2_deg.angle_target+Manipulator.roll2_deg.angle_init;
		manipulator->roll2_deg.rad=manipulator->roll2_deg.angle*AtR;
		
		//pitch3
		manipulator->pitch3_deg.angle_target=-Controller.pitch3+pitch3_feed;
		manipulator->pitch3_deg.angle_target = joints_limit(manipulator->pitch3_deg.angle_target,-112,109);


		manipulator->pitch3_deg.angle = manipulator->pitch3_deg.angle_target + manipulator->pitch3_deg.angle_init;
		manipulator->pitch3_deg.rad=manipulator->pitch3_deg.angle*AtR;
		
		//pitch2
		//低通滤波的
//		manipulator->pitch2_deg.angle_target=-Controller.pitch2+pitch2_feed;
//		manipulator->pitch2_deg.angle_target = joints_limit(manipulator->pitch2_deg.angle_target,0,135);

//		if(manipulator->pitch2_deg.angle_target<0)
//		  manipulator->pitch2_deg.angle_target=0;

//		manipulator->pitch2_deg.angle=manipulator->pitch2_deg.angle_target+manipulator->pitch2_deg.angle_init;
//		manipulator->pitch2_deg.rad=(manipulator->pitch2_deg.angle*AtR*9.1);

		manipulator->pitch2_deg.angle_target=-Controller.pitch2+pitch2_feed;
		manipulator->pitch2_deg.angle_target = joints_limit(manipulator->pitch2_deg.angle_target,0,135);

		if(manipulator->pitch2_deg.angle_target<0)
		manipulator->pitch2_deg.angle_target=0;
		if(manipulator->pitch2_deg.angle_target+manipulator->pitch2_deg.angle_init>manipulator->pitch2_deg.angle+0.04)
			manipulator->pitch2_deg.angle+=0.035;
		if(manipulator->pitch2_deg.angle_target+manipulator->pitch2_deg.angle_init<manipulator->pitch2_deg.angle-0.04)
			manipulator->pitch2_deg.angle-=0.035;
		
		manipulator->pitch2_deg.rad=(manipulator->pitch2_deg.angle*AtR*9.1);
	//	
		//roll1
		manipulator->roll1_deg.angle_target=-Controller.roll1 + roll1_feed;
		manipulator->roll1_deg.angle_target = joints_limit(manipulator->roll1_deg.angle_target,-110,110);



		manipulator->roll1_deg.angle = manipulator->roll1_deg.angle_target+Manipulator.roll1_deg.angle_init;
		manipulator->roll1_deg.rad=manipulator->roll1_deg.angle*AtR;
		
		//pitch1
		manipulator->pitch1_deg.angle_target=Controller.pitch1+pitch1_feed;
		if(manipulator->pitch1_deg.angle_target>0)
			manipulator->pitch1_deg.angle_target=0;

		
		manipulator->pitch1_deg.angle =manipulator->pitch1_deg.angle_target+manipulator->pitch1_deg.angle_init;
		
		//yaw
		manipulator->yaw_deg.angle_target=Controller.yaw+yaw_feed;
		manipulator->yaw_deg.angle_target = joints_limit(manipulator->yaw_deg.angle_target,-45,45);
		manipulator->yaw_deg.angle = manipulator->yaw_deg.angle_target+Manipulator.yaw_deg.angle_init;
		manipulator->yaw_deg.rad=manipulator->yaw_deg.angle*AtR;
	}
	else  //前1000秒先使用梯形速度到达目标位置，直接使用低通滤波赋值会有一个执行角度跳变
	{
		//roll2
		manipulator->roll2_deg.angle_target=-Controller.roll2+roll2_feed;
		manipulator->roll2_deg.angle_target = joints_limit(manipulator->roll2_deg.angle_target,-180,180);

		if(manipulator->roll2_deg.angle_target+Manipulator.roll2_deg.angle_init>manipulator->roll2_deg.angle+0.3)
			manipulator->roll2_deg.angle+=0.2;
		else if(manipulator->roll2_deg.angle_target+Manipulator.roll2_deg.angle_init<manipulator->roll2_deg.angle-0.3)
			manipulator->roll2_deg.angle-=0.2;
		manipulator->roll2_deg.rad=manipulator->roll2_deg.angle*AtR;
		
		//pitch3
		manipulator->pitch3_deg.angle_target=-Controller.pitch3+pitch3_feed;
		manipulator->pitch3_deg.angle_target = joints_limit(manipulator->pitch3_deg.angle_target,-112,109);

		if(manipulator->pitch3_deg.angle_target + manipulator->pitch3_deg.angle_init > manipulator->pitch3_deg.angle + 0.2)
			manipulator->pitch3_deg.angle += 0.15;
		else if(manipulator->pitch3_deg.angle_target + manipulator->pitch3_deg.angle_init < manipulator->pitch3_deg.angle - 0.2)
			manipulator->pitch3_deg.angle -= 0.15; 
		manipulator->pitch3_deg.rad=manipulator->pitch3_deg.angle*AtR;
		
		//pitch2
		manipulator->pitch2_deg.angle_target=-Controller.pitch2+pitch2_feed;
		manipulator->pitch2_deg.angle_target = joints_limit(manipulator->pitch2_deg.angle_target,0,135);

		if(manipulator->pitch2_deg.angle_target<0)
		  manipulator->pitch2_deg.angle_target=0;
		if(manipulator->pitch2_deg.angle_target+manipulator->pitch2_deg.angle_init>manipulator->pitch2_deg.angle+0.045)
			manipulator->pitch2_deg.angle+=0.035;
		if(manipulator->pitch2_deg.angle_target+manipulator->pitch2_deg.angle_init<manipulator->pitch2_deg.angle-0.045)
			manipulator->pitch2_deg.angle-=0.035;
		manipulator->pitch2_deg.rad=(manipulator->pitch2_deg.angle*AtR*9.1);
		
		//roll1
		manipulator->roll1_deg.angle_target=-Controller.roll1 + roll1_feed;
		manipulator->roll1_deg.angle_target = joints_limit(manipulator->roll1_deg.angle_target,-110,110);

		if(manipulator->roll1_deg.angle_target+Manipulator.roll1_deg.angle_init>manipulator->roll1_deg.angle+0.2)
			manipulator->roll1_deg.angle+=0.15;
		else if(manipulator->roll1_deg.angle_target+Manipulator.roll1_deg.angle_init<manipulator->roll1_deg.angle-0.2)
			manipulator->roll1_deg.angle-=0.15;

		manipulator->roll1_deg.rad=manipulator->roll1_deg.angle*AtR;
		
		//pitch1
		manipulator->pitch1_deg.angle_target=Controller.pitch1+pitch1_feed;
		if(manipulator->pitch1_deg.angle_target>0)
			manipulator->pitch1_deg.angle_target=0;
		if(manipulator->pitch1_deg.angle_target+manipulator->pitch1_deg.angle_init > manipulator->pitch1_deg.angle+0.045)
			manipulator->pitch1_deg.angle+=0.04;
		else if(manipulator->pitch1_deg.angle_target+manipulator->pitch1_deg.angle_init<manipulator->pitch1_deg.angle-0.045)
			manipulator->pitch1_deg.angle-=0.04;  
			
		//yaw
		manipulator->yaw_deg.angle_target=Controller.yaw+yaw_feed;
		manipulator->yaw_deg.angle_target = joints_limit(manipulator->yaw_deg.angle_target,-45,45);
		if(manipulator->yaw_deg.angle_target+Manipulator.yaw_deg.angle_init>manipulator->yaw_deg.angle+0.025)
			manipulator->yaw_deg.angle+=0.04;
		else if(manipulator->yaw_deg.angle_target+Manipulator.yaw_deg.angle_init<manipulator->yaw_deg.angle-0.025)
			manipulator->yaw_deg.angle-=0.04;
		manipulator->yaw_deg.rad=manipulator->yaw_deg.angle*AtR;
	
	}
	
}
/**
* @brief 逆解开始函数
  */ 
void inverse_get()
{
	inverse_count++;
	/*发送开始逆解标志位*/
	
	/*根据接收数据校验开始逆解*/
	
	/*判断逆解完成标志*/
	
	/*赋值逆解角度*/
	
}
/**
* @brief 逆解执行函数
  */ 
void inverse_control(Manipulator_t* manipulator)
{	
//	if(manipulator->pitch1_deg.angle_target+manipulator->pitch1_deg.angle_init>manipulator->pitch1_deg.angle+0.01)
//		manipulator->pitch1_deg.angle+=velocity_plan(manipulator->pitch1_deg);
//	else if(manipulator->pitch1_deg.angle_target+manipulator->pitch1_deg.angle_init<manipulator->pitch1_deg.angle-0.01)
//		manipulator->pitch1_deg.angle-=velocity_plan(manipulator->pitch1_deg); 
//	if(manipulator->pitch1_deg.angle_target>0)
//		manipulator->pitch1_deg.angle_target=0;
	
//	if(manipulator->pitch3_deg.angle_target + manipulator->pitch3_deg.angle_init > manipulator->pitch3_deg.angle + 0.01)
//		manipulator->pitch3_deg.angle += velocity_plan(manipulator->pitch3_deg);
//	else if(manipulator->pitch3_deg.angle_target + manipulator->pitch3_deg.angle_init < manipulator->pitch3_deg.angle - 0.01)
//		manipulator->pitch3_deg.angle -= velocity_plan(manipulator->pitch3_deg); 
//	if(manipulator->pitch3_deg.angle_target > 0)
//		manipulator->pitch3_deg.angle_target = 0;
}
/**
* @brief 关节执行函数
  */ 
void joint_Ctrl(Manipulator_t* manipulator)
{
	
	if(rc_Ctrl.isOnline==1 && joint_reset_flag ==0)
	{
		if(Custom.image_recv.Coordinate.cc_control_flag == 0)
		{
			roll2_Ctrl(manipulator);
			pitch3_Ctrl(manipulator);
			pitch2_Ctrl(manipulator);	
			roll1_Ctrl(manipulator);
			pitch1_Ctrl(manipulator);
			yaw_Ctrl(manipulator);
		}
		
	   Joint_control_cc();
       if(Custom.image_recv.Coordinate.cc_control_flag == 1)//自定义控制器
	   {
	       controller_control(manipulator);
	   }
	   
	  //最终输出
	  if(tim14.ClockTime%2==0)
	  {
		  
		 ctrl_motor1(&can1,&manipulator->Dm_4310_pitch3,manipulator->pitch3_deg.rad,0,40,0.8,0);
	     ctrl_motor1(&can1,&manipulator->Dm_4310_roll2,manipulator->roll2_deg.rad,0,30,0.6,0);
		 ctrl_motor1(&can2,&manipulator->Dm_8006_yaw,manipulator->yaw_deg.rad,0,85,1.6,0);
		 
         Motor_DegPositionMode(&Motor_zero_send, manipulator->pitch2_deg.rad);
	  }
	    if(tim14.ClockTime%2==1){
			RPDO1_pos(manipulator->pitch1_deg.angle);//位置模式(绝对位置+立即执行)+位置模式+目标位置 
		
			ctrl_motor1(&can1,&manipulator->Dm_4310_roll1,manipulator->roll1_deg.rad,0,40,0.7,0);
			}
    }
	if(joint_reset_flag == 1)
	{
		if(tim14.ClockTime%2==0)
		{
			pitch1_Ctrl(manipulator);
			pitch2_Ctrl(manipulator);
			roll1_Ctrl(manipulator);
			RPDO1_pos(manipulator->pitch1_deg.angle);//位置模式(绝对位置+立即执行)+位置模式+目标位置 
			ctrl_motor1(&can1,&manipulator->Dm_4310_roll1,manipulator->roll1_deg.rad,0,30,0.6,0);
			Motor_DegPositionMode(&Motor_zero_send, manipulator->pitch2_deg.rad);
		}
	}
	
}

