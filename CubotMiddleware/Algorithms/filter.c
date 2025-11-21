#include "filter.h"


struct LowPassFilter_Info LPF_joint_cc_pitch1={
  .filter_coefficient=0.0045f,
	.last_output=0,
};
struct LowPassFilter_Info LPF_joint_cc_pitch2={
  .filter_coefficient=0.0035f,
	.last_output=0,
};
struct LowPassFilter_Info LPF_joint_cc_pitch3={
  .filter_coefficient=0.009f,
	.last_output=0,
};
struct LowPassFilter_Info LPF_joint_cc_roll2={
  .filter_coefficient=0.009f,
	.last_output=0,
};

struct LowPassFilter_Info LPF_joint_cc_roll1={
  .filter_coefficient=0.009f,
	.last_output=0,
};
struct LowPassFilter_Info LPF_joint_cc_yaw={
  .filter_coefficient=0.0045f,
	.last_output=0,
};

float LPFilter(float sampling ,struct LowPassFilter_Info *LPF){
	//Ò»½×µÍÍ¨ÂË²¨Æ÷£ºp(n) = c¡¤q(n) + (1 - c)¡¤p(n - 1) 
	(*LPF).sampling =sampling;
	
	(*LPF).output=(*LPF).filter_coefficient *(*LPF).sampling +(1-(*LPF).filter_coefficient)*(*LPF).last_output;
	
	if(fabs((*LPF).output - (*LPF).last_output) > 8 )
	{
		(*LPF).output = (*LPF).last_output;
	}
	(*LPF).last_output =(*LPF).output ;
	
	return (*LPF).output ;
};
