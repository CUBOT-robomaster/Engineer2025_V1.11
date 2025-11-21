#ifndef __CONTROLLERL_H_
#define __CONTROLLERL_H_

#include "driver_timer.h"

typedef struct {
	float yaw_ccinit;
	float pitch1_ccinit;
	float pitch2_ccinit;	
	float pitch2_limit_d;
	float roll1_ccinit;
	float pitch3_ccinit;
	float roll2_ccinit;
	float yaw_init;
	float pitch1_init;
	float pitch2_init;	
	float roll1_init;
	float pitch3_init;
	float roll2_init;
	float yaw;
	float pitch1;
	float pitch2;	
	float roll1;
	float pitch3;
	float roll2;
	
}controller_data;
extern controller_data Controller;
extern int CC_start_timer ;
extern int8_t CC_back_flag;

void Customer_controller();
void Joint_control_cc();
#endif