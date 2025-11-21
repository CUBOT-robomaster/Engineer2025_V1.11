#include "servo.h"
#include "stdint.h"
#include "stm32h7xx_hal.h"
#include "usart.h"
#include "modbus.h"
#include "driver_timer.h"
#include "joint_control.h"
#include "vt13.h"

#define SERVO_Position_ADDRESS    128    //位置控制地址（查阅飞特SM45BL舵机内存表）
#define SERVO_Velocity_ADDRESS    131    //速度控制地址（查阅飞特SM45BL舵机内存表）
#define SERVO_Accelertion_ADDRESS  130   //加速度控制地址（查阅飞特SM45BL舵机内存表）
uint16_t  position_right=2820;//右侧
uint16_t  position_left=1000;
uint16_t  velocity1=30;
uint16_t  velocity2=30;
uint16_t  acceleration1=15;
uint16_t  acceleration2=15;

//幻尔总线舵机
uint8_t buf[10];
int16_t test_control=500;
int16_t test_move=0;

/*目的是为了只发一次*/
int8_t servo_image_judge1=0;
int8_t servo_image_judge2=0;//yaw控制标志位
int8_t servo_image_judge3=0;
int8_t servo_image_judge4=0;//lift控制标志位

int8_t servo_id=0;
float move_angle=90;//运动视角角度

uint8_t  servo_check_number(uint8_t buf[])
{   uint8_t i;
    uint16_t temp=0;
    for(i=2;i<buf[3]+2;i++){
    temp+=buf[i];
    
    }
    temp=~temp;
    i=(uint8_t)temp;
return i;
}
void servo_move(uint16_t id,uint16_t time,int16_t angle)
{   
    buf[0]=buf[1]=0x55;
    buf[2]=id;
    buf[3]=SERVO_MOVE_TIME_WRITE_Lenth;
    buf[4]=SERVO_MOVE_TIME_WRITE_Commandvalue;
    buf[5]=((int8_t )(angle));
    buf[6]=((int8_t )((angle)>>8));
    buf[7]=((int8_t )(time));
    buf[8]=((int8_t )(time>>8));
    buf[9]=servo_check_number(buf);
    HAL_UART_Transmit_DMA(&huart7,(unsigned char*)buf,10);
}

void servo_move_start(void)
{	
	servo_move(2,30,test_control);
}
	
	
void servo_constant_move(uint16_t id,int16_t speed)
{
    buf[0]=buf[1]=0x55;
    buf[2]=id;
    buf[3]=SERVO_OR_MOTOR_MODE_WRITE_Lenth;
    buf[4]=SERVO_OR_MOTOR_MODE_WRITE_Commandvalue;
    buf[5]=1;
    buf[6]=NULL;
    buf[7]=((int8_t )(speed));
    buf[8]=((int8_t )((speed)>>8));
    buf[9]=servo_check_number(buf);
    HAL_UART_Transmit_DMA(&huart7,(unsigned char*)buf,10);
}

 void servo_constant_move_start(void)
{
    servo_constant_move(3,test_move);
}
	

void watch_turn()
{
	
		//初始位置250 兑矿630
       if(move_angle==90)
        {  
            if(servo_image_judge1==0)
            {
				servo_image_judge1=1;
				servo_image_judge2=0;
				Manipulator.yaw_deg.angle_target=0;
				servo_move(4,50,1010);
			}
        
        }
        if(move_angle==0)
        {
            if(servo_image_judge2==0)
			{
				servo_image_judge2=1;
				servo_image_judge1=0;
				Manipulator.yaw_deg.angle_target=0;
				servo_move(4,50,640);
			}
        }
		   
		
		if(rc_Ctrl.key_Z==1&&rc_Ctrl.key_ctrl==0&&rc_Ctrl.key_shift==0)
        {   
			if(servo_image_judge2==1)
			{
				servo_move(4,50,790);
			}
			if(servo_image_judge1==1)
			{
				Manipulator.yaw_deg.angle_target=0;
				servo_move(4,50,1010);
			}
		}
		else if(rc_Ctrl.key_C==1&&rc_Ctrl.key_ctrl==0&&rc_Ctrl.key_shift==0)
        {   
			if(servo_image_judge2==1)
			{
				servo_move(4,50,490);
			}
			if(servo_image_judge1==1)
			{
				
				Manipulator.yaw_deg.angle_target=43;
				servo_move(4,50,845);
			}
		}
		else if(rc_Ctrl.key_X==1&&rc_Ctrl.key_ctrl==0&&rc_Ctrl.key_shift==0)
		{
			if(servo_image_judge2==1)
			{
				servo_move(4,50,640);
			}
		}
	
		if(vT13.key_Z==1&&vT13.key_ctrl==0&&vT13.key_shift==0)
        {   
			if(servo_image_judge2==1)
			{
				servo_move(4,50,790);
			}
			if(servo_image_judge1==1)
			{
				Manipulator.yaw_deg.angle_target=0;
				servo_move(4,50,1010);
			}
		}
		else if(vT13.key_C==1&&vT13.key_ctrl==0&&vT13.key_shift==0)
        {   
			if(servo_image_judge2==1)
			{
				servo_move(4,50,490);
			}
			if(servo_image_judge1==1)
			{
				
				Manipulator.yaw_deg.angle_target=43;
				servo_move(4,50,845);
			}
		}
		else if(vT13.key_X==1&&vT13.key_ctrl==0&&vT13.key_shift==0)
		{
			if(servo_image_judge2==1)
			{
				servo_move(4,50,640);
			}
		}
}

void watch_lift()
{
	
		
       if(move_angle==90)
        {  
            if(servo_image_judge3==0)
            {
				servo_image_judge3=1;
				servo_image_judge4=0;
                servo_move(2,30,455);
			}
           
        }
        if(move_angle==0)
        {
            if(servo_image_judge4==0)
			{
				servo_image_judge4=1;
				servo_image_judge3=0;
                servo_move(2,30,290);
				// servo_move(2,30,475);
				
			}
        }
		if(rc_Ctrl.key_Z==1&&rc_Ctrl.key_ctrl==0&&rc_Ctrl.key_shift==0)
        {   
				servo_move(2,30,455);
		}
		if(rc_Ctrl.key_C==1&&rc_Ctrl.key_ctrl==0&&rc_Ctrl.key_shift==0)
        {   
				servo_move(2,30,455);
		}
		if(rc_Ctrl.key_X==1&&rc_Ctrl.key_ctrl==0&&rc_Ctrl.key_shift==0)
        {   
				servo_move(2,30,290);
		}
		   
}		

void watch_control(RC_Ctrl* rc_ctrl)
{
	if(vT13.rc.mode_sw ==0){
	if(rc_ctrl->key_E_flag%2==1)
    {
		move_angle=0;//兑矿
    }
		
    if(rc_ctrl->key_E_flag%2==0)
    {
		move_angle=90;//运动
    }}
	if(vT13.rc.mode_sw !=0){
	if(vT13.key_E_flag%2==1)
    {
		move_angle=0;//兑矿
    }
		
    if(vT13.key_E_flag%2==0)
    {
		move_angle=90;//运动
    }}
	if(tim14.ClockTime%100==30)
	watch_turn();
	if(tim14.ClockTime%100==60)
	watch_lift();	
		

}	

void watch_angle_init()
{
	servo_move(4,30,250);
}	


//飞特SM45BL舵机
    void servo_Position_control(uint8_t id,uint16_t position)
    {
        MODBUS06(id,SERVO_Position_ADDRESS,position);
        
    }
    void servo_Velocity_control(uint8_t id,uint16_t velocity)
    {
         MODBUS06(id,SERVO_Velocity_ADDRESS,velocity);
    }
    void servo_Accelertion_control(uint8_t id,uint16_t accelertion)
    {
        MODBUS06(id,SERVO_Accelertion_ADDRESS,accelertion);
    }

void servo_control()
{
   //飞特舵机控制（之后需要传入参数）
   if(tim14.ClockTime%120==0)
   {
	   servo_Position_control(1,position_right);//1600平行，500最上，2600最下
   }
   if(tim14.ClockTime%120==20)//
   {
       servo_Position_control(2,position_left);//600平行，260最上，940最下
   }
   if(tim14.ClockTime%120==40)
   {
       servo_Velocity_control(1,velocity1);
   }
   if(tim14.ClockTime%120==60)
   {
       servo_Velocity_control(2,velocity2);
   }
   if(tim14.ClockTime%120==80)
   {
       servo_Accelertion_control(1,acceleration1);
   }
   if(tim14.ClockTime%120==100)
   {
       servo_Accelertion_control(2,acceleration2);
   }	
	
}
void ServoSM45BL_init()
{
        if(tim14.ClockTime%1000==100)
		{
		  servo_Position_control(1,2820);//1600平行，500最上，2600最下
		}
		if(tim14.ClockTime%1000==200)//
		{
		  servo_Position_control(2,1000);//600平行，260最上，940最下
		}
        if(tim14.ClockTime%1000==300)//
		{
		  servo_Velocity_control(1,40);
		}
        if(tim14.ClockTime%1000==400)//
		{
		  servo_Velocity_control(2,40);
		}
        if(tim14.ClockTime%1000==500)//
		{
		  servo_Velocity_control(1,20);
		}
        if(tim14.ClockTime%1000==600)//
		{
		  servo_Velocity_control(2,20);
		}

}	
	