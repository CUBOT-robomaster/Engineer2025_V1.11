#ifndef _GOLD_H
#define _GOLD_H
#include "stdint.h"
#include "motor.h"
#include "pid.h"
#include "dr16.h"
struct Goldmining_motor_information{
    float Target_Angle;
    float Can_Angle;
    float Last_Can_Angle;
    float Can_Angle_speed;
    float angle;
    float angle_last;
    int16_t Can_Speed_Feedback;
};
typedef struct{
    struct Goldmining_motor_information stretchout[2];
    struct Goldmining_motor_information stretchup[2];
 struct {
    Motor goldmotor[4];
    BasePID_Object turnpid[4];
}Motors2006;
}Goldmining;

void Goldmining_Init(Goldmining*goldmining,BasePID_Object motor_strechout_speed_pid,BasePID_Object motor_strechup_speed_pid,BasePID_Object motor_strechout_angle_pid,BasePID_Object motor_strechup_angle_pid,CanNumber canx);
void Angle_add(Goldmining *goldmining,RC_Ctrl *rc_ctrl);
void Auto_left_up_goldmining(Goldmining *goldmining,float  auto_target,float  auto_param);
void Auto_right_up_goldmining(Goldmining *goldmining,float  auto_target,float  auto_param);
void Auto_left_out_goldmining(Goldmining *goldmining,float  auto_target,float  auto_param);
void Auto_right_out_goldmining(Goldmining *goldmining,float  auto_target,float  auto_param);
void gold_reset(Goldmining*goldmining);//取矿机构复位函数
void Angle_add_init(Goldmining *goldmining);//初始化last值，保证累加器从0开始

extern int8_t gold_reset_flag;

#endif
