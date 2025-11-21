#include "hardware_config.h"
#include "control_logic.h"
#include "motor.h"
#include "dr16.h"
#include "driver_timer.h"
#include "pid.h"
#include "gold.h"
#include "servo.h"                                                                                                                               
#include "A1_stm32.h"
#include "joint_control.h"
#include "air.h"
#include "interaction_image.h"
#include "vision_community.h"
#include "referee.h"
#include "vt13.h"
#include "inverse.h"
//取矿机构
BasePID_Object motor_strechout_speed;//拉伸向外速度
BasePID_Object motor_strechout_angle;//拉伸向外角度
BasePID_Object motor_strechup_speed;//拉伸向上速度
BasePID_Object motor_strechup_angle;//拉伸向上角度

Goldmining Goldmining_t;
//roll16020
BasePID_Object roll1_speed_pid;//向左偏航速度
BasePID_Object roll1_angle_pid;//向左偏航角度
/**
  * @brief  初始化指令合集
  */
void HardwareConfig(void)
{
    //1
    UARTx_Init(&huart1, DR16_callback);
    UART_ENABLE_IT(&uart1, &uart1_buffer);
    UART_Receive_DMA(&uart1, &uart1_buffer);

	UARTx_Init(&huart3, Referee_callback);	//A1-485串口回调
    UART_ENABLE_IT(&uart3, &uart3_buffer);
	UART_Receive_DMA(&uart3, &uart3_buffer);
	
	UARTx_Init(&huart4, A1_fake_callback);	//A1-485串口回调
    UART_ENABLE_IT(&uart4, &uart4_buffer);
	UART_Receive_DMA(&uart4, &uart4_buffer);
	
	UARTx_Init(&huart2, imagetrans_callback);	//图传接收串口回调
    UART_ENABLE_IT(&uart2, &uart2_buffer);
	UART_Receive_DMA(&uart2, &uart2_buffer);
	
	UARTx_Init(&huart6, vision_callback);	//图传接收串口回调
    UART_ENABLE_IT(&uart6, &uart6_buffer);
	UART_Receive_DMA(&uart6, &uart6_buffer);
    //PID
    BasePID_Init(&motor_strechout_speed,3,0,-10,0);
    BasePID_Init(&motor_strechout_angle,6,0,-100,0);
    BasePID_Init(&motor_strechup_speed,3,0,-10,0);
    BasePID_Init(&motor_strechup_angle,6,0,-100,0);
	//取矿机构初始化
	Goldmining_Init(&Goldmining_t,motor_strechout_speed,motor_strechup_speed,motor_strechout_angle,motor_strechup_angle,CAN2);

	DR16Init(&rc_Ctrl);
	VT13Init( &vT13);
	
    CANx_Init(&hfdcan1, CAN1_rxCallBack);
    CAN_Open (&can1);
    CANx_Init(&hfdcan2, CAN2_rxCallBack);
    CAN_Open (&can2 );
	
	Vision_coil_init(&Vision_coil_t);//视觉坐标系逆解矩阵初始化
	inverse_coil_init();//机械臂逆解算矩阵初始化
	
    TIMx_Init(&htim14, TIM14_Task);//链接定时器回调
	TIM_Open(&tim14);
	
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_4);
	air_init();//气路模块初始化

}








