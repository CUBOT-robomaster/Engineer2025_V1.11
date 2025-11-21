#ifndef __SERVO_H__
#define __SERVO_H__
#define SERVO_MOVE_TIME_WRITE_Commandvalue   1
#define SERVO_MOVE_TIME_WRITE_Lenth   7
#define SERVO_OR_MOTOR_MODE_WRITE_Commandvalue   29
#define SERVO_OR_MOTOR_MODE_WRITE_Lenth   7
#include "stdint.h"
#include "dr16.h"
uint8_t  servo_check_number(uint8_t buf[]);
void servo_move(uint16_t id,uint16_t time,int16_t angle);
void servo_move_start(void );
void servo_constant_move(uint16_t id,int16_t speed);
 void servo_constant_move_start(void);
void servo_Position_control(uint8_t id,uint16_t position);
void servo_Velocity_control(uint8_t id,uint16_t velocity);
void servo_Accelertion_control(uint8_t id,uint16_t accelertion);
void ServoSM45BL_init();
void servo_control();

extern uint16_t  position_right;
extern uint16_t  position_left;
extern uint16_t  velocity1;
extern uint16_t  velocity2;
extern uint16_t  acceleration1;
extern uint16_t  acceleration2;
extern float move_angle;
void watch_control(RC_Ctrl* rc_ctrl);
void watch_angle_init();

#endif
