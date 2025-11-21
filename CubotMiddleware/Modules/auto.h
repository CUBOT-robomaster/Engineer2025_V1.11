#ifndef __AUTO__H_
#define __AUTO__H_
#include "gold.h"

void auto_judge();//判断自动模式
void auto_mode();


extern int8_t   auto_flag;//进入自动阶段标志位，防止再去执行其他自动阶段
extern int8_t   move_back_flag;//需要把后退发给底盘的标志
extern float roll2_feed;
extern float roll1_feed;
extern float pitch3_feed;
extern float pitch2_feed;
extern float pitch1_feed;
extern float yaw_feed ;

extern uint8_t get_double_gold_manual_flag;//判断是否在手动取矿
extern int8_t   associate_stretch_right_flag;//交接右侧取矿机构标志位
extern int8_t   associate_stretch_left_flag;//交接右侧取矿机构标志位
extern int8_t   associate_arm_left_flag;//交接右侧取矿机构标志位


extern int16_t  associate_stretch_count;//交接时间戳
extern uint8_t associate_stretch_auto_flag;//自动判断交接标志

#endif
