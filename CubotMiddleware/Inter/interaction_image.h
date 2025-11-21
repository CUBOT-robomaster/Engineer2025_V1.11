#ifndef INTERACTION_IMAGE_H__
#define INTERACTION_IMAGE_H__

#include "stm32h7xx.h"
#include "cmsis_armcc.h"
#include "interaction.h"
#include "driver_usart.h"
typedef float float32_t;
#define controller_data_lenth  30 //自定义控制器数据长度
#define rc_data_lenth  17 //遥控数据长度
#define USART2_RXBUF_SIZE 7+controller_data_lenth+2+5//取最大
#define USART2_TXBUF_SIZE 7+controller_data_lenth+2//取最大

extern UART_RxBuffer uart2_buffer;
uint8_t imagetrans_callback(uint8_t * recBuffer, uint16_t len);


typedef __packed struct //自定义发送结构体
{
 int sx;
 int sy;
 int sz;
int16_t x_speed;
int16_t y_speed;
int16_t z_speed;
int16_t yaw;
 int16_t pitch;
 int16_t roll;
	
}coordinates_send;

typedef __packed struct //自定义控制器发送结构体
{
	int yaw;
	int pitch1;
	int pitch2;
	int roll1;
	int pitch3;
	int roll2;
	
}angledata_send;

typedef __packed struct //自定义接收结构体
{
	float yaw1;
	float pitch1;
	float pitch2;
	float roll1;
	float pitch3;
	float roll2;
	uint8_t arm_fetch_flag;
	uint8_t cc_control_flag;
	uint8_t  cc_s2;
	uint8_t  cc_s3;
	uint8_t  cc_circle;
	uint8_t isonline;
}coordinates_recv;


typedef struct
{
	__packed struct
	{
		//< frame_header(5-byte) 帧头部分
		frame_header_t image_frame_header;  
		//< cmd_id(2-byte) 命令码
		uint16_t cmd_id;
		//< 自定义数据(-byte) 
		uint8_t check;//映射状态send_flag
	//	 coordinates_send Coordinate;//坐标
		angledata_send angle_send;
		uint8_t key_send;
		uint8_t other_data[4];
		//< frame_tail(2-byte，CRC16，整包校验)
		uint8_t CRC16[2];
	
	}image_send;
	__packed struct
	{
		coordinates_recv Coordinate;//坐标
	}image_recv;

}custom_robot_data_t;

uint16_t Get_CRC16_Check(uint8_t *pchMessage,uint32_t dwLength,uint16_t wCRC);
extern unsigned char recv_Buffer[USART2_RXBUF_SIZE];

extern custom_robot_data_t Custom;//自定义控制器

extern int image_count;
#endif