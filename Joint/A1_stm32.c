#include "A1_stm32.h"
#include "hardware_config.h"
#include "control_logic.h"
#include <math.h>
#include <string.h>
#include "joint_control.h"

#define abs(x) ((x>=0)?(x):(-x))
#define AtR 0.0174532f
#define RtD 57.2957795f

float A1_step = 0.008;
float pos_target1 = 0;
float pos_target2 = 0;
float pos_target2_last = 0;
float pos_target1_last = 0;
Pitch_acce pitch1_acce;
Pitch_acce pitch2_acce;


//< 收发数组

uint8_t Usart4_RxBuffer[USART_RXBUF_SIZE]__attribute__((at(0x24001000)));
uint8_t Usart4_TxBuffer[USART_TXBUF_SIZE]__attribute__((at(0x24001800)));


UART_RxBuffer uart4_buffer={
	.Data = Usart4_RxBuffer,
	.Size = USART_RXBUF_SIZE
};

//电机收发结构体变量声明
MOTOR_send Motor_zero_send = 
{
	.motor_send_data=
	{
		0
	},
	.hex_len=34, 
	.id     =MOTOR_ZERO_ID,  //0xbb为广播
	.mode   =10, //闭环伺服控制
	.T      =0,   //># 单位：Nm, |T|<128
	.W      =0, //># 单位：rad/s, |W|<256
	.Pos    =0,   //># 单位：rad, |Pos|<823549 减速比：9.1
	.K_P    =0.2,   //># 0<K_P<16
	.K_W    =3    //># 0<K_W<32
};


//电机收发结构体变量声明
MOTOR_send Motor_one_send = 
{
	.motor_send_data=
	{
		0
	},
	.hex_len=34, 
	.id     =MOTOR_ONE_ID,  //0xbb为广播
	.mode   =10, //闭环伺服控制
	.T      =0,   //># 单位：Nm, |T|<128
	.W      =0, //># 单位：rad/s, |W|<256
	.Pos    =0,   //># 单位：rad, |Pos|<823549 减速比：9.1
	.K_P    =0.2,   //># 0<K_P<16
	.K_W    =3    //># 0<K_W<32
};


MOTOR_recv Motor_zero_receive;
MOTOR_recv Motor_one_receive;

/**
  * @brief A1初始化函数（pitch1，pitch2）
  */
void A1_init()
{
	A1_DampingMode(&Motor_zero_send);		//进入阻尼模式
	
//	Motor_zero_send.Pos_target = Motor_zero_receive.Pos;	//读取返回值作为目标值
//	Motor_one_send.Pos_target = Motor_one_receive.Pos;
	
	Motor_one_send.T = 0;
	Motor_zero_send.T = 0;
	
	Manipulator.pitch2_deg.angle_init = Motor_zero_receive.Pos*RtA*0.11;		//pitch1读取阻尼模式位置初始化
	Manipulator.pitch2_deg.angle = Manipulator.pitch2_deg.angle_init;
	Manipulator.pitch2_deg.rad_init = Motor_zero_receive.Pos*0.11;
	Manipulator.pitch2_deg.rad = Manipulator.pitch2_deg.rad_init;
	Motor_one_send.acce = 0.00001;	//加速度驱动器参数初始化
	Motor_one_send.v_max = 0.003;
		
	Motor_zero_send.Pos_target=0;
	
}
	

/**
  * @brief A1发送函数
  */
uint8_t A1_send(UART_HandleTypeDef* huart_x, MOTOR_send* pMotor_s)
{

	if(huart_x == &huart4)
	{
		modify_data(pMotor_s,Usart4_TxBuffer);
		HAL_UART_Transmit_DMA(huart_x,Usart4_TxBuffer,USART_TXBUF_SIZE);
	}
	return 1;
}

/**
  * @brief 电机进入零力矩模式
  */
void Motor_ZeroTorque(MOTOR_send* pMotor_s)
{
	pMotor_s->T = 0;
	pMotor_s->W = 0;
	pMotor_s->Pos = 0;
	pMotor_s->K_P = 0;
	pMotor_s->K_W = 0;
}


/**
  * @brief A1阻尼模式
  */
void A1_DampingMode(MOTOR_send* pMotor_s)
{
	pMotor_s->T = 0;
	pMotor_s->W = 0;
	pMotor_s->Pos = 0;
	pMotor_s->K_P = 0;
	pMotor_s->K_W = 3;
}


/**
  * @brief 电机进入速度控制模式（一般不用）
  */
void Motor_VelocityMode(MOTOR_send* pMotor_s, float velocity)
{
	pMotor_s->T = 0;
	pMotor_s->W = velocity * 9.1;	 //< 单位：rad/s,
	pMotor_s->Pos = 0;
	pMotor_s->K_P = 0;
	pMotor_s->K_W = 3;
}

/**
  * @brief 电机进入位置控制模式，控制电机输出轴的角度
  */
void Motor_DegPositionMode(MOTOR_send* pMotor_s, float rad) 
{
	pMotor_s->T = 0;
	pMotor_s->W = 0;	 										//< 单位：rad/s,
	pMotor_s->Pos = rad ;   //< 单位是rad，需要先将角度转换为弧度，再将弧度乘减速比得到出轴的角度
	pMotor_s->K_P = 0.2;   //< Kp < 16
	pMotor_s->K_W = 3;		 //< Kd < 32
}

///**
//  * @brief 
//  */
//void A1_ag_control01(MOTOR_send* pMotor_s)
//{	
//	pitch1_acce.error = joint_deg.pitch1 * AtR * 9.1f - pos_target1;
//	if(pitch1_acce.error >= 0)
//	{
//		if((pitch1_acce.error >= 0.245)&&(pitch1_acce.step <= 0.007))
//			pitch1_acce.step += 0.0001;
//		else if((pitch1_acce.error <= 0.245)&&(pitch1_acce.step >= 0))
//			pitch1_acce.step -= 0.0001;
//	}
//	if(pitch1_acce.error < 0)
//	{
//		if((pitch1_acce.error <= -0.245)&&(pitch1_acce.step >= -0.007))
//			pitch1_acce.step -= 0.0001;
//		else if((pitch1_acce.error >= -0.245)&&(pitch1_acce.step <= 0))
//			pitch1_acce.step += 0.0001;
//	}
//	
//	 pos_target1 += pitch1_acce.step;
//	
//	pMotor_s->Pos_target = joint_limit.pitch1_init +  pos_target1;
//	pMotor_s->Pos = pMotor_s->Pos_target;
//	pMotor_s->K_P = 0.2;   //< Kp < 16
//	pMotor_s->K_W = 3;		 //< Kd < 32
//}



//void A1_ag_control02(MOTOR_send* pMotor_s)
//{	
//	
//	pitch2_acce.error = joint_deg.pitch2_single * AtR * 9.1f - pos_target2;
//	if(pitch2_acce.error >= 0)
//	{
//		if((pitch2_acce.error >= 0.245)&&(pitch2_acce.step <= 0.007))
//			pitch2_acce.step += 0.0001;
//		else if((pitch2_acce.error <= 0.245)&&(pitch2_acce.step >= 0))
//			pitch2_acce.step -= 0.0001;
//	}
//	if(pitch2_acce.error < 0)
//	{
//		if((pitch2_acce.error <= -0.245)&&(pitch2_acce.step >= -0.007))
//			pitch2_acce.step -= 0.0001;
//		else if((pitch2_acce.error >= -0.245)&&(pitch2_acce.step <= 0))
//			pitch2_acce.step += 0.0001;
//	}

//	pos_target2 += pitch2_acce.step;
//	
//	pMotor_s->Pos_target = joint_limit.pitch2_limit_d + pos_target2;
//	pMotor_s->W = 0.01;
//	pMotor_s->Pos = pMotor_s->Pos_target;
//	pMotor_s->K_P = 0.2;   //< Kp < 16
//	pMotor_s->K_W = 3;		 //< Kd < 32
//}

//void A1_reset_control(MOTOR_send* pMotor_s)
//{	
//	
//	if(pMotor_s->Pos_target - joint_limit.pitch2_init  < joint_deg.pitch2_single * AtR * 9.1f)
//		pMotor_s->Pos_target += A1_step;
//	else if(pMotor_s->Pos_target- joint_limit.pitch2_init > joint_deg.pitch2_single * AtR * 9.1f)
//		pMotor_s->Pos_target -= A1_step;
//	else if(abs((pMotor_s->Pos_target- joint_limit.pitch2_init) - joint_deg.pitch2_single * AtR * 9.1f)<=0.05)
//		pMotor_s->Pos_target = joint_deg.pitch2_single * AtR * 9.1f+ joint_limit.pitch2_init;
//	
//	pMotor_s->W = 0.01;
//	pMotor_s->Pos =   pMotor_s->Pos_target;
//	pMotor_s->K_P = 0.2;   //< Kp < 16
//	pMotor_s->K_W = 3;		 //< Kd < 32
//}
/**
  * @brief A1第一关节遥控控制（ing）
  */
//void A1_rc_mode01(MOTOR_send* pMotor_s, Control_flag* c_flag, RC_Ctrl* rc_ctrl)
//{
//	
//	if(rc_ctrl->rc.ch2-1024>150)	
//	{
//		c_flag->fwd_flag = 1;
//	}
//	else if(rc_ctrl->rc.ch2-1024<-150)
//	{
//		c_flag->back_flag = 1;
//	}
//	else	{
//		c_flag->back_flag = 0;
//		c_flag->fwd_flag = 0;
//	}

//	
//	Step_calculate(pMotor_s, c_flag);
//	
//	joint_deg.pitch1 += 5*pMotor_s->step;	//速度提升系数
//	if(joint_deg.pitch1>=95)
//		joint_deg.pitch1=95;
//	pMotor_s->Pos_target = joint_deg.pitch1 * AtR * 9.1f;
//	pMotor_s->W = 0.01;
//	pMotor_s->Pos = joint_limit.pitch1_init + pMotor_s->Pos_target;
//	pMotor_s->K_P = 0.2;   //< Kp < 16
//	pMotor_s->K_W = 3;		 //< Kd < 32
//	
//}


///**
//  * @brief A1第二关节遥控控制（ing）
//  */
//void A1_rc_mode02(MOTOR_send* pMotor_s, Control_flag* c_flag, RC_Ctrl* rc_ctrl)
//{
//	
//	if(rc_ctrl->rc.ch3-1024>150)
//	{
//		c_flag->fwd_flag = 1;
//	}
//	else if(rc_ctrl->rc.ch3-1024<-150)
//	{
//		c_flag->back_flag = 1;
//	}
//	else
//	{
//		c_flag->back_flag = 0;
//		c_flag->fwd_flag = 0;
//	}
//	
//	Step_calculate(pMotor_s, c_flag);
//	
//	joint_deg.pitch2_single += 5*pMotor_s->step;
//	pMotor_s->Pos_target = joint_deg.pitch2_single * AtR * 9.1f;
//	pMotor_s->W = 0.01;
//	pMotor_s->Pos = joint_limit.pitch2_limit_d + pMotor_s->Pos_target;
//	pMotor_s->K_P = 0.2;   //< Kp < 16
//	pMotor_s->K_W = 3;		 //< Kd < 32
//	
//}


///**
//  * @brief 自定义控制器-Pitch1
//  */
//void A1_cc_Pitch1(MOTOR_send* pMotor_s)
//{
//	
//	pMotor_s->Pos_target = joint_deg.pitch1_LPF1 * AtR * 9.1f;
//	pMotor_s->W = 0.01;
//	if(pMotor_s->Pos_target>150* AtR * 9.1f)
//		pMotor_s->Pos_target=pos_target1_last;
//	if(pMotor_s->Pos_target<-150* AtR * 9.1f)
//		pMotor_s->Pos_target=pos_target1_last;
//	pos_target1_last=pMotor_s->Pos_target;
//	pMotor_s->Pos = joint_limit.pitch1_init + pMotor_s->Pos_target;	
//	pMotor_s->K_P = 0.2;   //< Kp < 16
//	pMotor_s->K_W = 3;		 //< Kd < 32
//}

///**
//  * @brief 自定义控制器-Pitch2
//  */
//void A1_cc_Pitch2(MOTOR_send* pMotor_s)
//{
//	
//	pMotor_s->Pos_target = joint_deg.pitch2_single * AtR * 9.1f;	//由差值角度换算为减速后的电机弧度
//	
//	if(pMotor_s->Pos_target>150* AtR * 9.1f)
//		pMotor_s->Pos_target=pos_target2_last;
//	if(pMotor_s->Pos_target<-150* AtR * 9.1f)
//		pMotor_s->Pos_target=pos_target2_last;
//	
//	pos_target2_last=pMotor_s->Pos_target;
//	
//	pMotor_s->W = 0.01;
//	pMotor_s->Pos = joint_limit.pitch2_limit_d + pMotor_s->Pos_target;
//	
//	pMotor_s->K_P = 0.2;   //< Kp < 16
//	pMotor_s->K_W = 3;		 //< Kd < 32

//	//pMotor_s->Pos = LPFilter(pMotor_s->Pos ,&LPF_joint_pitch2_pos);
//	
//}



//void A1_cc_Pitch1_back(MOTOR_send* pMotor_s)
//{
//	if(pMotor_s->Pos_target < joint_deg.pitch1 * AtR * 9.1f)
//		pMotor_s->Pos_target += A1_step;
//	else if(pMotor_s->Pos_target > joint_deg.pitch1 * AtR * 9.1f)
//		pMotor_s->Pos_target -= A1_step;
//	else if(abs(pMotor_s->Pos_target - joint_deg.pitch1 * AtR * 9.1f)<=0.05)
//		pMotor_s->Pos_target = joint_deg.pitch1 * AtR * 9.1f;
//	
//	pMotor_s->W = 0.01;
//	pMotor_s->Pos = joint_limit.pitch1_init + pMotor_s->Pos_target;
//	pMotor_s->K_P = 0.2;   //< Kp < 16
//	pMotor_s->K_W = 3;		 //< Kd < 32
//}

//void A1_cc_Pitch2_back(MOTOR_send* pMotor_s)
//{
//	if(pMotor_s->Pos_target < joint_deg.pitch2_single * AtR * 9.1f)
//		pMotor_s->Pos_target += A1_step;
//	else if(pMotor_s->Pos_target > joint_deg.pitch2_single * AtR * 9.1f)
//		pMotor_s->Pos_target -= A1_step;
//	else if(abs(pMotor_s->Pos_target - joint_deg.pitch2_single * AtR * 9.1f)<=0.05)
//		pMotor_s->Pos_target = joint_deg.pitch2_single * AtR * 9.1f;
//	
//	pMotor_s->W = 0.01;
//	pMotor_s->Pos = joint_limit.pitch2_limit_d + pMotor_s->Pos_target;
//	pMotor_s->K_P = 0.2;   //< Kp < 16
//	pMotor_s->K_W = 3;		 //< Kd < 32
//}


/**
  * @brief 电机进入力矩控制模式，空载安全力矩为0.05 单位Nm T<128
  */
void Motor_TorqueMode(MOTOR_send* pMotor_s, float tau) 
{
	pMotor_s->T = tau;
	pMotor_s->W = 0;	 	
	pMotor_s->Pos = 0;  
	pMotor_s->K_P = 0;
	pMotor_s->K_W = 0;
}


//ID读取回调函数，当前使用串口3
MOTOR_recv* Motor_receive_callback(uint8_t ID, UART_HandleTypeDef* huart)
{
	
	if(huart == &huart4)
	{
		switch(ID)
		{
			case 0:
				return &(Motor_zero_receive);
			case 1:
				return &(Motor_one_receive);
			default:
				return NULL;
		}
		
	}
}

//串口空闲中断中调用
void Motor_receive_test(UART_HandleTypeDef* huart)
{
	//使用串口5 注册回调函数 注册接收数组
//	if(huart == &huart5)
//		A1_data_receive(huart, Motor_receive_callback, Usart5_RxBuffer, USART_RXBUF_SIZE);

	if(huart == &huart4)
		A1_data_receive(huart, Motor_receive_callback, Usart4_RxBuffer, USART_RXBUF_SIZE);

}



//stm32 HAL库 A1电机通信驱动
//调用接收函数时，需要注册回调函数 回调函数参数为接收到的电机ID 返回对应电机的结构体

uint8_t A1_data_send(UART_HandleTypeDef* huart_x,MOTOR_send* pMotor_s,uint8_t* pBuffer,uint8_t len)
{
	//调用A1协议封包函数 将数据装入发送协议包结构体
	modify_data(pMotor_s,pBuffer);
	// 发送
	HAL_UART_Transmit_DMA(huart_x,pBuffer,len);
	// 执行成功
	return 1;
}


uint8_t A1_data_receive(UART_HandleTypeDef* huart_x,MOTOR_recv* callback_fun(uint8_t, UART_HandleTypeDef*),uint8_t* pBuffer,uint8_t len)
{ 
	uint8_t receive_id;
	MOTOR_recv* pMotor_r;
	
//	if((__HAL_UART_GET_FLAG(huart_x, UART_FLAG_IDLE) != RESET))
//	{ 
		__HAL_UART_CLEAR_IDLEFLAG(huart_x);
		__HAL_UART_CLEAR_OREFLAG(huart_x);
		HAL_UART_DMAStop(huart_x);
		//解包 将pBuffer中的数据解入pMotor_r
		receive_id = check_data_forID(pBuffer);
		//执行回调函数，获取对应ID的结构体指针
		pMotor_r = callback_fun(receive_id, huart_x);
		//将数据包数据解到结构体中
		extract_data(pMotor_r,pBuffer);
		
//		if(huart_x == &huart4)
//			LegAnglePreprocess(&leftLeg, Motor_lzero_receive.Pos, Motor_lone_receive.Pos, Motor_lzero_receive.W, Motor_lone_receive.W,Motor_lzero_receive.T, Motor_lone_receive.T);
//		if(huart_x == &huart5)
//			LegAnglePreprocess(&rightLeg, Motor_rzero_receive.Pos, Motor_rone_receive.Pos, Motor_rzero_receive.W, Motor_rone_receive.W,Motor_rzero_receive.T,Motor_rone_receive.T );
		
		//DMA 接收
		HAL_UART_DMAResume(huart_x);
		HAL_UART_Receive_DMA(huart_x, pBuffer, len);
			return 1;
//	}
}



uint8_t A1_fake_callback(uint8_t * recBuffer, uint16_t len)
{
	Motor_receive_test(&huart4);
	return 0;
}

