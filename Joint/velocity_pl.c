#include "joint_control.h"
#include "mit.h"
#include "dr16.h"
#include "A1_stm32.h"
#include "motor.h"
#include "driver_timer.h"
#include "AIM_PDO.h"
#include "interaction_image.h"
#include "controllerl.h"


//Manipulator_t Manipulator=
//{
//	//初始化电机运动量
//   .roll1_deg.velocity=0,
//   .roll1_deg.max_velocity=0.07,
//   .roll1_deg.min_velocity=0.03,
//   .roll1_deg.Accel=0.004,
//   .roll2_deg.velocity=0,
//   .roll2_deg.max_velocity=0.07,
//   .roll2_deg.min_velocity=0.03,
//   .roll2_deg.Accel=0.004,
//   .pitch1_deg.velocity=0,
//   .pitch1_deg.max_velocity=0.04,
//   .pitch1_deg.min_velocity=0.01,
//   .pitch1_deg.Accel=0.002,
//   .pitch2_deg.velocity=0,
//   .pitch2_deg.max_velocity=0.025,
//   .pitch2_deg.min_velocity=0.01,
//   .pitch2_deg.Accel=0.0015,
//   .pitch3_deg.velocity=0,
//   .pitch3_deg.max_velocity=0.06,
//   .pitch3_deg.min_velocity=0.02,
//   .pitch3_deg.Accel=0.004,
//   .yaw_deg.velocity=0,
//   .yaw_deg.max_velocity=0.04,
//   .yaw_deg.min_velocity=0.01,
//   .yaw_deg.Accel=0.003
//   
//};
float speed;
/**
  * @brief 关节角速度规划
  */
  
float f_abs(float angle)
{
	if(angle >= 0)
		return angle;
	else 
		return -1*angle;
}
float velocity_plan(joint_angle_t *joint)
{
	if(fabs((joint->angle_target + joint->angle_init) - joint->angle)>=10)
		joint->velocity +=   joint->Accel;

	else if(fabs((joint->angle_target + joint->angle_init) - joint->angle)<10)
		joint->velocity -=  joint->Accel;
	
	if(joint->velocity > joint->max_velocity)
		joint->velocity = joint->max_velocity;
	if(joint->velocity <joint->min_velocity)
		joint->velocity = joint->min_velocity;
	return joint->velocity;
}	 











