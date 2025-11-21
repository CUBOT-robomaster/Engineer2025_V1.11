#include "driver_can.h"
#include "AIM_PDO.h"
#include "stm32h7xx_hal.h"
#include "joint_control.h"
RPDO_COM pdo_com=  //初始化数据协议
{
	.RPDO3_speed.Identifier = 0x401,
	.RPDO3_speed.Data[0]=0x0F,
	.RPDO3_speed.Data[1]=0x00,
	.RPDO3_speed.Data[2]=0x03,
	.RPDO3_speed.Data[3]=0x58,
	.RPDO3_speed.Data[4]=0x01,
	.RPDO3_speed.Data[5]=0x00,
	.RPDO3_speed.Data[6]=0x00,
	.RPDO3_speed.Data[7]=0x00,
	
	.RPDO1_pos.Identifier = 0x201,
	.RPDO1_pos.Data[0]=0x2F,
	.RPDO1_pos.Data[1]=0x00,
	.RPDO1_pos.Data[2]=0x01,
	.RPDO1_pos.Data[3]=0x00,
	.RPDO1_pos.Data[4]=0x00,
	.RPDO1_pos.Data[5]=0x00,
	.RPDO1_pos.Data[6]=0x00,
	.RPDO1_pos.Data[7]=0x00,
	
	.RPDO2_pos.Identifier = 0x301,
	.RPDO2_pos.Data[0]=0x00,
	.RPDO2_pos.Data[1]=0x00,
	.RPDO2_pos.Data[2]=0x00,
	.RPDO2_pos.Data[3]=0x00,
};


TPDO_COM MOTOR_ONE=
{
	.RPDO1_ID=0x181,
	.RPDO2_ID = 0x281
};//取决于使用PDO几
int init_count = 0;
void YZ_AIM_init()//角度制
{
//	init_count ++ ;
//	Manipulator.pitch1_deg.angle_init=MOTOR_ONE.pos -3;
//	Manipulator.pitch1_deg.angle=MOTOR_ONE.pos - 3;
	//Manipulator.pitch1_deg.angle_init=-91.78;
	//Manipulator.pitch1_deg.angle=-91.78;

	Manipulator.pitch1_deg.angle_init=-77.78;
	Manipulator.pitch1_deg.angle=-77.78;
	//Manipulator.pitch1_deg.angle_target=-76.78;
}

int pos_send;
void RPDO1_pos(float Pos)//位置模式(绝对位置+立即执行)+位置模式+目标位置
{
	//一圈大约0—1670000
	int pos=(int)(Pos*4639);
	pos_send=(int)(Pos*4639);
    pdo_com.RPDO1_pos.Data[3]=(uint8_t)(pos);
    pdo_com.RPDO1_pos.Data[4]=(uint8_t)(pos>>8);
	pdo_com.RPDO1_pos.Data[5]=(uint8_t)(pos>>16);
    pdo_com.RPDO1_pos.Data[6]=(uint8_t)(pos>>24);

	CAN_Send (&can1,&pdo_com.RPDO1_pos);
}

void RPDO2_pos( int speed )//位置模式(获取当前位置+设置梯形速度)
{
	//一圈大约0—1670000
	pdo_com.RPDO2_pos.Data[4]=(uint8_t)(speed);
    pdo_com.RPDO2_pos.Data[5]=(uint8_t)(speed>>8);
	pdo_com.RPDO2_pos.Data[6]=(uint8_t)(speed>>16);
    pdo_com.RPDO2_pos.Data[7]=(uint8_t)(speed>>24);
	
	CAN_Send (&can1,&pdo_com.RPDO2_pos);
	
}

int POS;

CAN_RxBuffer AIMRxCallback(CAN_Object canx, CAN_RxBuffer rxBuffer)
{
	
	if(rxBuffer.Header.Identifier== MOTOR_ONE.RPDO1_ID||rxBuffer.Header.Identifier== MOTOR_ONE.RPDO2_ID)
	{
	  POS=(int)(rxBuffer.Data[0]|rxBuffer.Data[1]<<8|rxBuffer.Data[2]<<16|rxBuffer.Data[3]<<24);
	  MOTOR_ONE.state=(int)(rxBuffer.Data[4]|rxBuffer.Data[5]<<8);
	  MOTOR_ONE.pos=POS*0.000215564f;
		return  rxBuffer;
	}
	
 

}
