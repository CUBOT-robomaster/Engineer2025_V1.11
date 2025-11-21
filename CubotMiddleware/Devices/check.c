#include "check.h"
#include "referee.h"

Check_Robot_State check_robot_state;

/**
  *读取机器人状态，各部分是否在线，
	*/
	
void RobotOnlineState(Check_Robot_State *CheckRobotState)
{
	CheckRobotState->usart_state.Check_receiver++;
	CheckRobotState->usart_state.Check_vision++;
	CheckRobotState->usart_state.Check_referee++;
	CheckRobotState->usart_state.Check_custom++;
	       

	if(CheckRobotState->usart_state.Check_receiver > 100)
		CheckRobotState->usart_state.Check_receiver = 100;                                                                 
	if(CheckRobotState->usart_state.Check_vision > 3000)
		CheckRobotState->usart_state.Check_vision = 3000; 
	if(CheckRobotState->usart_state.Check_custom > 2000)
	CheckRobotState->usart_state.Check_custom = 2000; 

}