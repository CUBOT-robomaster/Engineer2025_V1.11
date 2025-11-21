
#include "vision_community.h"
#include "check.h"
#include "inverse.h"
Vision Vision_t;
uint8_t Vidion_recData[vision_rxBufferLengh];

	/**
		* @brief  创建dr16串口缓存区数据结构
		*/
UART_RxBuffer uart6_buffer={
	.Data = Vidion_recData,
	.Size = vision_rxBufferLengh
};

int16_t matrix_data[9];
int16_t move[3];
int16_t x_com_t=-90;
int inver_enable_count = 0;
uint8_t vision_callback(uint8_t * recBuffer, uint16_t len)
{
	int16_t Header = recBuffer[1]<<8|recBuffer[0];
	int16_t Tail = recBuffer[27]<<8|recBuffer[26];
	if(Header == 0x85)
		check_robot_state.usart_state.Check_vision = 0;
	if(Header == 0x00AA && Tail == 0x00DD)
	{
		check_robot_state.usart_state.Check_vision = 0;
		inver_enable_count = 0;
		inver_enable = 1;
		matrix_data[0] =   (int16_t)((recBuffer[3]<<8|recBuffer[2]));
		matrix_data[1] =   (int16_t)(recBuffer[5]<<8|recBuffer[4]);
		matrix_data[2] =   (int16_t)(recBuffer[7]<<8|recBuffer[6]);
		matrix_data[3] =   (int16_t)(recBuffer[9]<<8|recBuffer[8]);
		matrix_data[4] =   (int16_t)(recBuffer[11]<<8|recBuffer[10]);
		matrix_data[5] =   (int16_t)(recBuffer[13]<<8|recBuffer[12]);
		matrix_data[6] =   (int16_t)(recBuffer[15]<<8|recBuffer[14]);
		matrix_data[7] =   (int16_t)(recBuffer[17]<<8|recBuffer[16]);
		matrix_data[8] =   (int16_t)( recBuffer[19]<<8|recBuffer[18]);
		move[0] =  (int16_t)(recBuffer[21]<<8|recBuffer[20]);
		move[1] =  (int16_t)(recBuffer[23]<<8|recBuffer[22]);
		move[2] =  (int16_t)(recBuffer[25]<<8|recBuffer[24]);
		
		Vision_t.matrix_data[0] = (float)(matrix_data[0])/100;
		Vision_t.matrix_data[1] = (float)(matrix_data[1])/100;
		Vision_t.matrix_data[2] = (float)(matrix_data[2])/100;
		Vision_t.matrix_data[3] = (float)(matrix_data[3])/100;
		Vision_t.matrix_data[4] = (float)(matrix_data[4])/100;
		Vision_t.matrix_data[5] = (float)(matrix_data[5])/100;
		Vision_t.matrix_data[6] = (float)(matrix_data[6])/100;
		Vision_t.matrix_data[7] = (float)(matrix_data[7])/100;
		Vision_t.matrix_data[8] = (float)(matrix_data[8])/100;
		
		Vision_t.move[0] = (float)(move[0])+x_com_t;
		Vision_t.move[1] = (float)(move[1]);
		Vision_t.move[2] = (float)(move[2]);
		
	}
	else
		inver_enable_count++;
	if(inver_enable_count > 20)
		inver_enable = 0;
	
}