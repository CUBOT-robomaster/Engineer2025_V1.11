#ifndef VISION_COMMUNITY
#define VISION_COMMUNITY

#include "stm32h7xx_hal.h"
#include "usart.h"
#include "driver_usart.h"
#define vision_rxBufferLengh 30	 //< dr16接收缓存区数据长度
typedef struct 
{
	float yaw;
	float roll;
	float pitch;
	float x;
	float y;
	float z;

}Pose;
typedef struct
{
	float matrix_data[9];
	float move[3];
	Pose    pose;
	int8_t online;
}Vision;

uint8_t vision_callback(uint8_t * recBuffer, uint16_t len);
extern UART_RxBuffer uart6_buffer;
extern Vision Vision_t;

#endif