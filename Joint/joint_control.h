#ifndef __JOINT_CONTROL_H_
#define __JOINT_CONTROL_H_
#include "mit.h"
#include "motor.h"
#include "pid.h"

struct Manipulator_motor_information{
    float Target_Angle;
    float Can_Angle;
    float Last_Can_Angle;
    float Can_Angle_speed;
    float angle;
    float angle_last;
    int16_t Can_Speed_Feedback;
};
typedef struct
{
 struct Manipulator_motor_information roll1;
 Motor motor6020;
 BasePID_Object turnpid[4];
 
 
 }motor6020_t;
typedef struct
{
	
	float rad;
	float rad_init;
	float rad_target;
	float angle;
	float angle_add;
	float angle_init;
	float angle_target;
	float velocity;//速度
	float max_velocity;//最小速度
	float min_velocity;//最小速度
	
	float Accel;//加速度
	//float angle_com;//转化坐标系中的角度补充
    
}joint_angle_t;

typedef struct
{
	DM_motor  Dm_4310_pitch3;
	DM_motor  Dm_4310_roll2;
	DM_motor  Dm_8006_yaw;
	DM_motor  Dm_4310_roll1;
    joint_angle_t roll2_deg;
    joint_angle_t pitch3_deg;
	joint_angle_t pitch2_deg;
	joint_angle_t roll1_deg;
	joint_angle_t pitch1_deg;
	joint_angle_t yaw_deg;
}Manipulator_t;

extern int8_t   auto_flag;//进入自动阶段标志位，防止再去执行其他自动阶段
extern int8_t   move_back_flag;//需要把后退发给底盘的标志


extern Manipulator_t  Manipulator;
extern int8_t joint_reset_flag;  

void joint_Ctrl(Manipulator_t* manipulator);
void manipulator_init(Manipulator_t* manipulator,BasePID_Object roll1_speed,BasePID_Object roll1_angle);
void angle_agular(Manipulator_t* manipulator);
void joint_init();
void roll1_reset();

#endif