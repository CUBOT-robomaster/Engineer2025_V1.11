#include "air.h"
#include "gpio.h"
#include "dr16.h"
#include "vt13.h"
/**
  * @brief 气路模块，继电器控制，气泵调节
  */
 uint16_t key_R_last=0;//KEY_R标记上一次数值，用于判断
 uint16_t key_R_last1=0;//KEY_R标记上一次数值，用于判断
void Vacuum_pump_main(uint8_t order)//主真空泵(取矿机构)
{
	if(order == 1)
	{
		if(rc_Ctrl .rc.s2 == 2)
			TIM1->CCR3 = 5000;
		else
			TIM1->CCR3 = 0;
		
	}
		
	else if(order == 0)
		TIM1->CCR3 = 10000;
}

void Vacuum_pump_vice(uint8_t order)//副真空泵（机械臂）
{
	if(order == 1)
	{
			if(rc_Ctrl .rc.s2 == 2)
				TIM1->CCR4 = 5000;
			else 
				TIM1->CCR4 = 0;
	}
		
	else if(order == 0)
		TIM1->CCR4 = 10000;
}


void arm_fetch(int8_t state)
{
	if(state==0)
    HAL_GPIO_WritePin(GPIOD,GPIO_PIN_7,GPIO_PIN_RESET);
	if(state==1)
    HAL_GPIO_WritePin(GPIOD,GPIO_PIN_7,GPIO_PIN_SET);
}

void valve_fetch(int8_t state)
{
	if(state==0)
    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_3,GPIO_PIN_RESET);
	if(state==1)
    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_3,GPIO_PIN_SET);
}
void arm_relese(int8_t state)
{
	if(state==0)
    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_4,GPIO_PIN_RESET);
	if(state==1)
    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_4,GPIO_PIN_SET);
}

//气路初始化
void air_init()
{
	valve_fetch(close);
	arm_fetch(close);
	arm_relese(close);
    Vacuum_pump_main(close);
	Vacuum_pump_vice(close);

}

void air_control(RC_Ctrl *rc_ctrl)
{
	if(rc_ctrl->key_R_flag==key_R_last+1)
	{
		Vacuum_pump_vice(close);
		arm_relese(open);
		key_R_last=rc_ctrl->key_R_flag;
	}
	if(vT13.key_R_flag==key_R_last1+1)
	{
		Vacuum_pump_vice(close);
		arm_relese(open);
		key_R_last1=vT13.key_R_flag;
	}
}