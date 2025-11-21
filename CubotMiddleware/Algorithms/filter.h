#ifndef __FILTER_H__
#define __FILTER_H__

#include "main.h"

struct LowPassFilter_Info{	
	float filter_coefficient;
	float last_output;
	float output;
	float sampling;	
};

float LPFilter(float sampling ,struct LowPassFilter_Info *LPF);

//extern struct LowPassFilter_Info LPF_pitch_speed;
//extern struct LowPassFilter_Info LPF_pitch_vision;
//extern struct LowPassFilter_Info LPF_yaw_speed;
//extern struct LowPassFilter_Info LPF_yaw_vision;
//extern struct LowPassFilter_Info LPF_none;
//extern struct LowPassFilter_Info LPF_pitch_mpu;
//extern struct LowPassFilter_Info LPF_yaw_mpu;
extern struct LowPassFilter_Info LPF_joint_cc_pitch2;
extern struct LowPassFilter_Info LPF_joint_cc_pitch1;
extern struct LowPassFilter_Info LPF_joint_cc_pitch3;
extern struct LowPassFilter_Info LPF_joint_cc_roll2;
extern struct LowPassFilter_Info LPF_joint_cc_yaw;
extern struct LowPassFilter_Info LPF_joint_cc_roll1;
#endif
