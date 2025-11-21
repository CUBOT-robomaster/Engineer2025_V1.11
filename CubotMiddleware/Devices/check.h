#ifndef __CHECK_H
#define __CHECK_H

#include "stdint.h"
#include "referee.h"

typedef struct
{
	struct
	{
	  uint16_t Check_receiver;
	  uint16_t Check_vision;
	  uint16_t Check_referee;
	  uint16_t Check_custom;
	}usart_state;	
}Check_Robot_State;

void RobotOnlineState(Check_Robot_State *CheckRobotState);

extern Check_Robot_State check_robot_state;

#endif  
