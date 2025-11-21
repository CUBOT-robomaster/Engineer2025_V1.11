#include "interaction_image.h"
#include "string.h"
#include "usart.h"
#include "vt13.h"
#include "check.h"
custom_robot_data_t Custom;//自定义控制器

unsigned char Usart2_RxBuffer[USART2_RXBUF_SIZE]__attribute__((at(0x24006400)));
unsigned char Usart2_TxBuffer[USART2_TXBUF_SIZE]__attribute__((at(0x24006800)));

UART_RxBuffer uart2_buffer={
	.Data = Usart2_RxBuffer,
	.Size = USART2_RXBUF_SIZE
};
uint8_t ref_image_packge[10][40];  //最多一次收50个包
/**
  * @brief  图传接收数据,测试,在定时中断才处理数据
	*/
void imagetrans_recv_datas_copy(uint8_t *pdata,uint8_t * pBuffer)//只复制
{
	memcpy(pBuffer,pdata,USART2_RXBUF_SIZE);
}
void controller_recv_datas_modify(uint8_t *pdata,custom_robot_data_t* Custom )
{  
	/*自定义控制器（工程）*/
	uint16_t cmd_id;
	cmd_id=*(pdata+6)<<8|*(pdata+5); 
	if(cmd_id==0x0302)
	{	
		check_robot_state.usart_state.Check_custom = 0;
		int yaw_1,pitch_1,pictch2,roll_1,pitch_3,roll_2;
		yaw_1=*(pdata+8)|*(pdata+9)<<8|*(pdata+10)<<16|*(pdata+11)<<24;
		pitch_1=*(pdata+12)|*(pdata+13)<<8|*(pdata+14)<<16|*(pdata+15)<<24;
		pictch2=*(pdata+16)|*(pdata+17)<<8|*(pdata+18)<<16|*(pdata+19)<<24;
		roll_1=*(pdata+20)|*(pdata+21)<<8|*(pdata+22)<<16|*(pdata+23)<<24;
		pitch_3=*(pdata+24)|*(pdata+25)<<8|*(pdata+26)<<16|*(pdata+27)<<24;
		roll_2=*(pdata+28)|*(pdata+29)<<8|*(pdata+30)<<16|*(pdata+31)<<24;
		Custom->image_recv.Coordinate.arm_fetch_flag=*(pdata+7);
		//Custom->image_recv.Coordinate.cc_control_flag=*(pdata+32)>>6||((*(pdata+32)>>4)&0x01);
		Custom->image_recv.Coordinate.cc_circle=*(pdata+32)&0x01;
		Custom->image_recv.Coordinate.yaw1=(float)yaw_1/1000;
		Custom->image_recv.Coordinate.pitch1=(float)pitch_1/1000;
		Custom->image_recv.Coordinate.pitch2=(float)pictch2/1000;
		Custom->image_recv.Coordinate.roll1=(float)roll_1/1000;
		Custom->image_recv.Coordinate.pitch3=(float)pitch_3/1000;
		Custom->image_recv.Coordinate.roll2=(float)roll_2/1000;
	}

}

uint8_t image_command(uint8_t *data, uint16_t len) 
{
   
    static uint32_t Verify_CRC8_OK;
    static uint32_t Verify_CRC16_OK;
    uint16_t i = 0;
    uint16_t pack_size = 0;

    while (pack_size < len) 
	{
		if (*data != 0xA5) 
		{
			data++;
			pack_size++;
			continue;
		}

		// 检查剩余数据是否足够解析头部
		if (pack_size + 7 > len) 
			break;

		// 解析 data_len（避免非对齐访问）
		uint16_t data_len;
		memcpy(&data_len, data + 1, 2);
		Custom.image_send.image_frame_header.data_length = data_len;

		// 检查数据包完整性
		uint16_t total_packet_size = 7 + data_len + 2;
		if (pack_size + total_packet_size > len)
			break;

		// 校验 CRC
		Verify_CRC8_OK = Verify_CRC8_Check_Sum(data, frame_header_len);
		Verify_CRC16_OK = Verify_CRC16_Check_Sum(data, total_packet_size);

		if (Verify_CRC8_OK && Verify_CRC16_OK) 
		{
			if (i < 200) 
			{
				memcpy(ref_image_packge[i], data, total_packet_size);
				controller_recv_datas_modify(ref_image_packge[i],&Custom);
				i++;
				if(i>25)
					i=25;
			}
		}

		data += total_packet_size;
		pack_size += total_packet_size;

		if (pack_size >= 150)
			break;
    }
    return 0;
}

	
/*接收回调*/
uint8_t imagetrans_callback(uint8_t * recBuffer, uint16_t len)
{
	
	controller_recv_datas_modify(recBuffer,&Custom);	
	vt13_recv_datas_modify(recBuffer,&vT13);
	//image_command(recBuffer,len);
//	imagetrans_recv_datas_copy(Usart2_RxBuffer,recv_Buffer);//测试,在定时中断才处理数据
	return 0;
}

