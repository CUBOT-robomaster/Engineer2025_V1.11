#ifndef __A132_H_
#define __A132_H_
#include "A1.h"
#include "stm32h7xx.h"
#include "driver_usart.h"
#include "control_logic.h"
#include "usart.h"
//#include "five_link_leg.h"


#define MOTOR_ZERO_ID 0
#define MOTOR_ONE_ID 1
#define MOTOR_TWO_ID 1

#define USART_RXBUF_SIZE 88
#define USART_TXBUF_SIZE 34


typedef struct {
	float error;
	float step;
}Pitch_acce;


void Motor_init(MOTOR_send* pMotor_s,uint8_t ID, UART_HandleTypeDef* huart, float pos);
void Motor_receive_test(UART_HandleTypeDef* huart);

//void Motor_send_test();
uint8_t A1_send(UART_HandleTypeDef* huart_x, MOTOR_send* pMotor_s);
uint8_t A1_data_send(UART_HandleTypeDef* huart_x,MOTOR_send* pMotor_s,uint8_t* pBuffer,uint8_t len);
uint8_t A1_data_receive(UART_HandleTypeDef* huart_x,MOTOR_recv* callback_fun(uint8_t, UART_HandleTypeDef*),uint8_t* pBuffer,uint8_t len);
uint8_t A1_fake_callback(uint8_t * recBuffer, uint16_t len);

extern UART_RxBuffer uart4_buffer;
extern MOTOR_send Motor_zero_send;
extern MOTOR_send Motor_one_send;
//宇树电机数据接收结构体
extern MOTOR_recv Motor_zero_receive;
extern MOTOR_recv Motor_one_receive;
extern MOTOR_recv Motor_rzero_receive;
extern MOTOR_recv Motor_rone_receive;
extern float pos_target2_last ;
extern float pos_target1_last ;


void Motor_RadPositionMode(MOTOR_send* pMotor_s, float pos);
void Motor_ZeroTorque(MOTOR_send* pMotor_s);
void Motor_VelocityMode(MOTOR_send* pMotor_s, float velocity);
void A1_DampingMode(MOTOR_send* pMotor_s);
void Motor_DegPositionMode(MOTOR_send* pMotor_s, float rad) ;

//使用中的A1模式封装函数
void A1_init(void);

//void A1_rc_mode01(MOTOR_send* pMotor_s, Control_flag* c_flag, RC_Ctrl* rc_ctrl);
//void A1_rc_mode02(MOTOR_send* pMotor_s, Control_flag* c_flag, RC_Ctrl* rc_ctrl);

//void A1_TP_control(MOTOR_send* pMotor_s, float pos);
//void A1_rc_mixMode(MOTOR_send* pMotor_s, Control_flag* c_flag, RC_Ctrl* rc_ctrl, float torque, float pos_target);
void A1_unfollowMode(MOTOR_send* pMotor_s, float torque, RC_Ctrl* rc_ctrl);

void A1_cc_Pitch1(MOTOR_send* pMotor_s);
void A1_cc_Pitch2(MOTOR_send* pMotor_s);

void A1_ag_control01(MOTOR_send* pMotor_s);
void A1_ag_control02(MOTOR_send* pMotor_s);

void A1_cc_Pitch1_back(MOTOR_send* pMotor_s);
void A1_cc_Pitch2_back(MOTOR_send* pMotor_s);

void A1_reset_control(MOTOR_send* pMotor_s);
void A1_init();

extern float roll_angle_raw;
extern float pos_target1;
extern float pos_target2;
#endif

