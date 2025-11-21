#ifndef VT13_H__
#define VT13_H__

#include "stm32h7xx.h"
#include "cmsis_armcc.h"
#include "driver_usart.h"
#include "dr16.h"

#define Key_Filter_Num_vT13 2       //< 按键检测消抖滤波时间(ms)
typedef struct { 
 struct {       
	 uint16_t ch0;       
	 uint16_t ch1;       
	 uint16_t ch2;      
	 uint16_t ch3;
	 uint16_t sw; 
	 uint8_t mode_sw;
	 uint8_t go_home;
	 uint8_t Fn;
	 uint8_t change;
	     
	 uint8_t photo;
	}rc; 

 struct {       
	 int16_t x;       
	 int16_t y;       
	 int16_t z;         
	 uint8_t press_l;     
	 uint8_t press_m;    
	 uint8_t press_r; 
	 uint8_t press_l_flag;  
	 uint8_t press_m_flag; 	 
	 uint8_t press_r_flag; 
 }mouse; 

	struct {     
		uint8_t keyboard14;
        uint8_t keyboard15;	
		uint8_t s1ands1;
		uint8_t slands2;
	}keyboard;
	
	uint8_t  key[18];   
	uint8_t  keyflag[18];	
	uint8_t  isUnpackaging;  	 //< 解算状态标志位，解算过程中不读取数据
	uint8_t  isOnline;
	uint32_t onlineCheckCnt;
	float Chassis_Z_Integ;
	float Chassis_Y_Integ;//斜坡积分变量
    float Chassis_X_Integ;
}VT13; 
void vt13_recv_datas_modify(uint8_t *pdata,VT13* vt13 );
void PC_keybroad_filter_vt13(VT13* RC_Ctl);
void KeyBoard_DataUnpack_vt13(VT13 *rc_ctrl);
extern VT13 vT13;
#endif