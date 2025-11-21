#ifndef INVERSE_H_
#define INVERSE_H_

#include "vision_community.h"
#include "joint_control.h"
#include "matrix.h"

#define sizeof_float 32
#define direction_LEFT 0
#define direction_RIGHT 1
struct T_M
{
    float data[16];

};
struct joint_deg
{
   float deg_yaw;
   float deg_pitch1;
   float deg_pitch2;
   float deg_pitch3;
   float deg_roll1;
   float deg_roll2;
   
   float current_deg_yaw;
   float current_deg_pitch1;
   float current_deg_pitch2;
   float current_deg_pitch3;
   float current_deg_roll1;
   float current_deg_roll2;
	

};
struct intern_variable//中间变量
{
	float p2[3];
	float p3[3];
    float l_c;
	float theta3;
	float theta4;
	float theta5;
	float theta6;
	float P1_1P2[3];
	float P1P2[3];
	float P2P3[3];
	
	
	float32_t *ATMA[6];
	float32_t *BTMB;
	float32_t *T_out_data[2];
	float32_t *TM;
	float32_t *TM_inverse;
	float32_t *RL;
	
	mat T[6];
	mat T_out[2];
	mat R_out;
	mat T_M;
	mat T_M_inverse;
	mat R_L;

	
};

typedef struct vs_c {
	void (*get_photo_coordinate)(struct vs_c *vs);
	float* (*get_taget_coordinate)(struct vs_c *vs);
	
	mat rotate;//旋转矩阵 将物体坐标系变换到机械臂的物体坐标系  
	mat object;//识别的物体姿态矩阵  
	mat camera_coor[2];//相机相对基坐标坐标系
	mat object_coor;//物体相对相机坐标系
	mat object_coor_base;//物体在基坐标系的坐标
	mat R[4];//物体在基坐标系的坐标
	
	float *rotate_data;
	float *object_data;
	float *camera_coor_data[2];
	float *object_coor_data;
	float *object_coor_base_data;
	
	float *R_data[4];
	
	
}Vision_coil;

void inverse_coil(float target_angle_p,float target_angle_y,float target_angle_r,
                  float target_x,float target_y,float target_z);
float yaw_coil();
float pitch1_coil();
float pitch2_coil();
float pitch3_coil();
float roll1_coil();
float roll2_coil_part1();
float roll2_coil_part2();
float roll2_coil_part();



struct T_M Attitude_description(float rad_p,float rad_y,float rad_r,float x,float y,float z);
float deg2rad(float angle);
float rad2deg(float rad);
float space_distance(float* p1,float *p2);
float get_dot(float *N1 , float *N2);
float get_norm(float *n);
void Vision_coil_init(Vision_coil *vision_coil_t);
void inverse_coil_init();


void get_photo_coordinate(Vision_coil *vision_coil_t);
float *get_taget_coordinate(Vision_coil *vision_coil_t);


void get_current_angle(Manipulator_t* manipulator);
void inverse_test(Manipulator_t* manipulator);
void judge_direction();
void vision_coil_test();

extern Vision_coil Vision_coil_t;
extern struct joint_deg Joint_deg;
extern int8_t direction_t;
extern int16_t chassis_x;
extern int16_t chassis_y; 
extern int8_t inver_enable;//相机识别到才能执行自动兑矿
extern int8_t inver_start ;//需要两次自动兑矿，判断是否需要下次


#endif