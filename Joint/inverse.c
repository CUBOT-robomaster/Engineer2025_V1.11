/**************************************************************
文件名	: 	 inverse.c
作者	   : nico
版本	   : V1.0
描述	   : 机械臂几何解逆解
其他	   : 无
日志	   : 初版V1.0 2025/2/19创建
**************************************************************/
#include "inverse.h"
#include "arm_math.h"
#include "math.h"
#include "matrix.h"
#include "dr16.h"
#include "air.h"
#include "stdlib.h"
#include "interboard.h"
#include "auto.h"
#include "vt13.h"

#define L1  330.f
#define L2  357.91f
#define L3  85.f
#define L_link2  357.58f
#define theta_r  85.03f
#define theta_p  1.56f
#define off_z  101.f        //p0点和基座起点沿z轴的偏距
#define off_x  58.34f    //p0点和基座起点沿x轴负方向的偏距
#define r2d   57.2957805f
#define d2r   0.0174533f
#define abs(x) ((x)>0? (x):(-(x)))

#define second_trans 1 //是否需要二次自动兑
struct  joint_deg Joint_deg;
struct T_M  T_m;
struct intern_variable intern_variable_t;//中间变量保存

Vision_coil Vision_coil_t;

float matrix_test[8];//观察矩阵值
int8_t if_trans_flag = 0;//是否兑矿标志位
float y_com_chassis = 0;;
float theta_yaw;
//给定目标6个状态求解
void inverse_coil(float target_angle_p,float target_angle_y,float target_angle_r,
	float target_x,float target_y,float target_z)
{
	T_m = Attitude_description(deg2rad(target_angle_p),deg2rad(target_angle_y),
	deg2rad(target_angle_r),target_x,target_y,target_z);
	Joint_deg.deg_yaw    = yaw_coil();
    Joint_deg.deg_pitch1 = pitch1_coil();
	Joint_deg.deg_pitch2 = pitch2_coil();
	judge_direction();
	Joint_deg.deg_pitch3 = pitch3_coil();
	Joint_deg.deg_roll1  = roll1_coil();
//	roll2_coil_part1();
//	Joint_deg.deg_roll2  = roll2_coil_part2();
	
	Joint_deg.deg_roll2 = roll2_coil_part();
}
float vision_x,vision_y,vision_z;
/*使用*/
void visiondata_inverse_coil()
{
	float*TM = get_taget_coordinate(&Vision_coil_t);
	for(int8_t i = 0; i<16 ;i ++)
		T_m.data[i] = TM[i];
	float x_comt;
	x_comt = TM[2]*250;
	T_m.data[3]  = T_m.data[3] - x_comt;
	if(T_m.data[3] < -500)
		T_m.data[3] = -500;
	T_m.data[7]  = -T_m.data[7] - TM[6]*250;
	T_m.data[11] = T_m.data[11] - TM[10]*250;
	vision_x =	T_m.data[3];
	vision_y =	T_m.data[7];
	vision_z =	T_m.data[11];
	Joint_deg.deg_yaw    = yaw_coil();
    Joint_deg.deg_pitch1 = pitch1_coil();
	Joint_deg.deg_pitch2 = pitch2_coil();
	Joint_deg.deg_pitch3 = pitch3_coil();
	Joint_deg.deg_roll1  = roll1_coil();
	
//	roll2_coil_part1();
//	Joint_deg.deg_roll2  = roll2_coil_part2();
	
	Joint_deg.deg_roll2 = roll2_coil_part();

}

void redemption_inverse_coil(float x,float y,float z)
{

	T_m.data[3] = x;
	T_m.data[7] = y;
	T_m.data[11] = z;
	Joint_deg.deg_yaw    = yaw_coil();
    Joint_deg.deg_pitch1 = pitch1_coil();
	Joint_deg.deg_pitch2 = pitch2_coil();
	Joint_deg.deg_pitch3 = pitch3_coil();
	Joint_deg.deg_roll1  = roll1_coil();
//	roll2_coil_part1();
//	Joint_deg.deg_roll2  = roll2_coil_part2();
	
	Joint_deg.deg_roll2 = roll2_coil_part();

}

//位姿描述函数(已验证)
struct T_M Attitude_description(float rad_p,float rad_y,float rad_r,float x,float y,float z)
{
    struct T_M  T_mm;
	rad_r = -rad_r;
	rad_y = -rad_y;
	rad_p = -rad_p;
    T_mm.data[0]=arm_cos_f32(rad_p)*arm_cos_f32(rad_y);
    T_mm.data[1]=arm_sin_f32(rad_r)*arm_sin_f32(rad_p)*arm_cos_f32(rad_y)-arm_cos_f32(rad_r)*arm_sin_f32(rad_y);
    T_mm.data[2]=arm_cos_f32(rad_r)*arm_sin_f32(rad_p)*arm_cos_f32(rad_y)+arm_sin_f32(rad_r)*arm_sin_f32(rad_y);
	T_mm.data[3]=x;
	T_mm.data[4]=arm_cos_f32(rad_p)*arm_sin_f32(rad_y);
    T_mm.data[5]=arm_sin_f32(rad_r)*arm_sin_f32(rad_p)*arm_sin_f32(rad_y)+arm_cos_f32(rad_r)*arm_cos_f32(rad_y);
    T_mm.data[6]=arm_cos_f32(rad_r)*arm_sin_f32(rad_p)*arm_sin_f32(rad_y)-arm_sin_f32(rad_r)*arm_cos_f32(rad_y);
	T_mm.data[7]=y;
	T_mm.data[8]=-arm_sin_f32(rad_p);
	T_mm.data[9]=arm_sin_f32(rad_r)*arm_cos_f32(rad_p);
	T_mm.data[10]=arm_cos_f32(rad_r)*arm_cos_f32(rad_p);
	T_mm.data[11]=z;
	T_mm.data[12]=0;
	T_mm.data[13]=0;
	T_mm.data[14]=0;
	T_mm.data[15]=1;
	
	return T_mm;
}

float theta_yaw = 0;
//求解yaw(已验证)
float yaw_coil()
{
    float joint_deg_yaw;
	float n_z[3];//提取向量
	float pos[3];//提取位置信息
	float Atti[3];//末端向量
	float control_point[3];//pitch3点
	n_z[0] = T_m.data[2];   //matlab:n_z= T_M(1:3, 3);   
	n_z[1] = T_m.data[6];
	n_z[2] = T_m.data[10];
	pos[0] = T_m.data[3];   //matlab:n_z= T_M(1:3, 4)
	pos[1] = T_m.data[7];
	pos[2] = T_m.data[11];
	if(if_trans_flag == 1)
	{
		Atti[0] = -(L3+50) * n_z[0];
		Atti[1] = -(L3+50) * n_z[1];
		Atti[2] = -(L3+50) * n_z[2];
	}
	else
	{
		Atti[0] = -L3 * n_z[0];
		Atti[1] = -L3 * n_z[1];
		Atti[2] = -L3 * n_z[2];
	}
	control_point[0] = pos[0] + Atti[0];
	control_point[1] = pos[1] + Atti[1];
	control_point[2] = pos[2] + Atti[2];
	
	intern_variable_t.p2[0] = control_point[0];
	intern_variable_t.p2[1] = control_point[1];
	intern_variable_t.p2[2] = control_point[2];
	intern_variable_t.p3[0] = pos[0];
	intern_variable_t.p3[1] = pos[1];
	intern_variable_t.p3[2] = pos[2];
	
    joint_deg_yaw= atan2(control_point[1],abs(control_point[0]));
	joint_deg_yaw = rad2deg(joint_deg_yaw);
	
	
	return joint_deg_yaw;
}

//求解pitch1(已验证)
float pitch1_coil()
{
	float joint_deg_pitch1;
	float p0[3];//P0点
	float theta1,theta2,theta3;
	p0[0] = -off_x*arm_cos_f32(deg2rad(Joint_deg.deg_yaw));
	p0[1] = off_x*arm_sin_f32(deg2rad(Joint_deg.deg_yaw));
	p0[2] = off_z;
	intern_variable_t.l_c = space_distance(p0,intern_variable_t.p2);
    theta1 = asinf((intern_variable_t.p2[2] - p0[2])/intern_variable_t.l_c);
	theta2 = acosf((L1*L1 + intern_variable_t.l_c*intern_variable_t.l_c - L2*L2)/(2*L1*intern_variable_t.l_c));
	theta3 = rad2deg(theta1 + theta2);
	intern_variable_t.theta3 = theta3;
	joint_deg_pitch1 = theta3 - 90;
	return  joint_deg_pitch1;
}

//求解pitch2(已验证)
float pitch2_coil()
{
	float joint_deg_pitch2;
	float theta4;
	theta4 = acosf((L1*L1 + L2*L2 - intern_variable_t.l_c*intern_variable_t.l_c)/(2*L1*L2));
	theta4 = rad2deg(theta4);
	intern_variable_t.theta4 = theta4;
	joint_deg_pitch2 = theta4-90;
	return  joint_deg_pitch2;
}


//求解pitch3(已验证)
float pitch3_coil()
{
	float joint_deg_pitch3;
    float p1[3],p1_1[3];
	float dis1,dis2,dis3;
	float theta5,theta6;
	float P1P2[3],P2P3[3],P1_1P2[3];
	dis1 = L1 * arm_cos_f32(deg2rad(intern_variable_t.theta3));
	p1[0] = -(dis1+off_x)*arm_cos_f32 (deg2rad(Joint_deg.deg_yaw));
	p1[1] = (dis1+off_x)*arm_sin_f32 (deg2rad(Joint_deg.deg_yaw));
	p1[2] = L1 * arm_sin_f32(deg2rad(intern_variable_t.theta3)) + off_z;
	
	P1P2[0] = intern_variable_t.p2[0] - p1[0];
	P1P2[1] = intern_variable_t.p2[1] - p1[1];
	P1P2[2] = intern_variable_t.p2[2] - p1[2];
	
	P2P3[0] = intern_variable_t.p3[0] - intern_variable_t.p2[0];
	P2P3[1] = intern_variable_t.p3[1] - intern_variable_t.p2[1];
	P2P3[2] = intern_variable_t.p3[2] - intern_variable_t.p2[2];
	
	theta5 = 360-intern_variable_t.theta4-intern_variable_t.theta3-90;
	intern_variable_t.theta5 = theta5;
	dis2 = L_link2 * arm_cos_f32(deg2rad(theta5));
	dis3 = L_link2 * arm_sin_f32(deg2rad(theta5));
	
	//机械臂pitch2偏移，旧车才有
//	P1_1P2[0] = -dis3 * arm_cos_f32(deg2rad(Joint_deg.deg_yaw));
//	P1_1P2[1] = dis3 * arm_sin_f32(deg2rad(Joint_deg.deg_yaw));
//	P1_1P2[2] = dis2;
//	
	intern_variable_t.P1P2[0] = P1P2[0];
	intern_variable_t.P1P2[1] = P1P2[1];
	intern_variable_t.P1P2[2] = P1P2[2];
	
	intern_variable_t.P2P3[0] = P2P3[0];
	intern_variable_t.P2P3[1] = P2P3[1];
	intern_variable_t.P2P3[2] = P2P3[2];
	
	theta6 = acosf(get_dot(P1P2,P2P3)/(get_norm(P1P2)*get_norm(P2P3)));
	theta6 = rad2deg(theta6);
	intern_variable_t.theta6 = theta6;
	joint_deg_pitch3 = 90 - theta6;
	
	if(!direction_t)
	{
		if(if_trans_flag)
			joint_deg_pitch3 = 180 - joint_deg_pitch3  - 6;//箭头朝左
	}
	else 
	{
		joint_deg_pitch3 = joint_deg_pitch3 ;
	} //箭头朝右
	
	return  joint_deg_pitch3;
}


/*
求解roll1(已验证)
罗格里德斯反解roll角度
*/
float l_BD,dx,BE[3],BC[3];
float roll1_coil()
{
//	float joint_deg_roll1;
//  float theta7,dis4,dis5;
//  float i1,j1,k1,P2P3_1[3];
//	float v1,v2,v3,v4,v5,v6;
//	float C,C1,K1,K2,K3,K4,K5,K6;
//	float P1,P2,P3,P4,P5,P6;
//	float Q1,Q2,Q3,Q4,Q5;
//	theta7 = intern_variable_t.theta6 - intern_variable_t.theta5;
//	dis4 = L3*arm_cos_f32(deg2rad(theta7));
//	dis5 = L3*arm_sin_f32(deg2rad(theta7));
//	
//	P2P3_1[0] = -dis5*arm_cos_f32(deg2rad(Joint_deg.deg_yaw));
//	P2P3_1[1] = dis5*arm_sin_f32(deg2rad(Joint_deg.deg_yaw));
//	P2P3_1[2] = -dis4;
//	
//	i1 = intern_variable_t.P1_1P2[0];
//	j1 = intern_variable_t.P1_1P2[1];
//	k1 = intern_variable_t.P1_1P2[2];
//	v1 = P2P3_1[0];
//	v2 = P2P3_1[1];
//	v3 = P2P3_1[2];
//	v4 = intern_variable_t.P2P3[0];
//	v5 = intern_variable_t.P2P3[1];
//	v6 = intern_variable_t.P2P3[2];
//	C1 = (i1 * i1 + j1 * j1 + k1 * k1);
//	arm_sqrt_f32(C1,&C);
//	K1 =  (j1 * j1 * v1)/C1;
//	K2 =  (k1 * k1 * v1)/C1;
//	K3 = -(k1 * v2 )/ C;
//	K4 = -(i1 * j1 * v2)/C1;
//	K5 = (j1 * v3 )/C;
//	K6 = -(i1 * k1 *v3 )/C1;
//	P1 =  (k1 * v1)/ C;
//	P2 = -(i1 * j1 * v1)/C1;
//	P3 =  (i1 * i1 * v2)/C1;
//	P4 =  (k1 * k1 * v2)/C1;
//	P5 = -(i1 * v3)/C;
//	P6 = -(j1 * k1 * v3)/C1;
//	Q1 =  (v5 - v2)/(P2 + P3 + P4 + P6);
//	Q2 =  (P1 + P5)/(P2 + P3 + P4 + P6);
//	Q3 =  K1 + K2 + K4 + K6;
//	Q4 =  K3 + K5 - Q2 * Q3;
//	Q5 = (v4 - v1) - (Q1 * Q3);
//  joint_deg_roll1 = rad2deg(asinf(Q5/Q4));
	
//以上特定情况下roll1计算不对，以下另一种方法计算,该方法只能计算角度差值绝对值，后面需要根据你的方向确定正负
	/*需要一二连杆平面计算roll=0时的末端坐标和向量，先计算更换平面后末端向量投影到二连杆延长向量的点与p2的向量*/
	float joint_deg_roll1;
	float Lp[3];//更换平面后末端向量投影到二连杆延长向量的点与p2的向量
	float theta7,theta8;
	float B[3],D[3],BD[3];
	
	/*先计算更换平面后末端向量投影到二连杆延长向量的点与p2的向量*/
	Lp[0] = L3*arm_cos_f32(deg2rad(intern_variable_t.theta6))*(intern_variable_t.P1P2[0])/L_link2;
	Lp[1] = L3*arm_cos_f32(deg2rad(intern_variable_t.theta6))*(intern_variable_t.P1P2[1])/L_link2;
	Lp[2] = L3*arm_cos_f32(deg2rad(intern_variable_t.theta6))*(intern_variable_t.P1P2[2])/L_link2;
	/*计算出投影点*/
	B[0] = intern_variable_t.p2[0]+ Lp[0];
	B[1] = intern_variable_t.p2[1]+ Lp[1];
	B[2] = intern_variable_t.p2[2]+ Lp[2];
	/*计算更换平面后末端向量与水平夹角*/
	theta7 = (intern_variable_t.theta6 - intern_variable_t.theta4 - intern_variable_t.theta3 +180);
	/*计算更换平面后末端点相对p2点的坐标，该过程省略，直接下一步代入*/
	//	D_P2[0] = L3*arm_cos_f32(deg2rad(theta7));
	//	D_P2[1] = 0;
	//	D_P2[2] = L3*arm_sin_f32(deg2rad(theta7));
	/*计算更换平面后roll1=0末端向量*/
	D[0] = intern_variable_t.p2[0] - L3*arm_cos_f32(deg2rad(theta7));
	D[1] = intern_variable_t.p2[1] - 0;
	D[2] = intern_variable_t.p2[2] - L3*arm_sin_f32(deg2rad(theta7));
	BD[0]  = B[0] - D[0];
	BD[1]  = B[1] - D[1];
	BD[2]  = B[2] - D[2];
	/*将平面旋转回来，继续求roll1=0时末端实际向量*/
	theta8 = 180-intern_variable_t.theta6-intern_variable_t.theta5;
	l_BD = get_norm(BD);
	dx = l_BD*arm_sin_f32(deg2rad(theta8));
	
	//	Ex = B[0] - dx*arm_cos_f32(deg2rad(Joint_deg.deg_yaw));
	//	Ey = B[1] - dx*arm_sin_f32(deg2rad(Joint_deg.deg_yaw));
	//	E[0] = B[0] - dx*arm_cos_f32(deg2rad(Joint_deg.deg_yaw));
	//	E[1] = B[1] - dx*arm_sin_f32(deg2rad(Joint_deg.deg_yaw));
	//	E[2] = D[2];
	//	
	//	BE[0] = E[0] - B[0];
	//	BE[1] = E[1] - B[1];
	//	BE[2] = E[2] - B[2];
	
	/*实际roll=0时末端向量*/
	BE[0] = - dx*arm_cos_f32(deg2rad(Joint_deg.deg_yaw));
	BE[1] = + dx*arm_sin_f32(deg2rad(Joint_deg.deg_yaw));
	BE[2] = D[2] - B[2];
	/*目标位姿的末端向量*/
	BC[0] = T_m.data[3] - B[0];
	BC[1] = T_m.data[7] - B[1];
	BC[2] = T_m.data[11] - B[2];

	joint_deg_roll1 = rad2deg(acosf(get_dot(BE,BC)/(get_norm(BE)*get_norm(BC))));
	if(!direction_t)
	{
		if(!if_trans_flag)
			joint_deg_roll1 = -joint_deg_roll1;//不兑矿
		else
			joint_deg_roll1 = 180 - (joint_deg_roll1 + 1) ;//箭头朝左，这么做是为了这个方向兑时能把rfid面朝下
	}
	else
	{
		if(!if_trans_flag)
			joint_deg_roll1 = joint_deg_roll1 ;
		else
			joint_deg_roll1 = joint_deg_roll1 ;
	}
    return joint_deg_roll1;
}

void inverse_coil_init()
{
	for(int i=0;i<6;i++)
	{
		intern_variable_t.ATMA[i] = (float *)malloc(sizeof_float * 4*4);
		memset(intern_variable_t.ATMA[i], 0, sizeof_float * 4 * 4);
		Matrix_Init(&intern_variable_t.T[i],4,4,(float *)intern_variable_t.ATMA[i]);
	}
	
	intern_variable_t.T_out_data[0] = (float *)malloc(sizeof_float * 4*4);
	memset(intern_variable_t.T_out_data[0], 0, sizeof_float * 4 * 4);
	Matrix_Init(&intern_variable_t.T_out[0],4,4,(float *)intern_variable_t.T_out_data[0]);
	
	intern_variable_t.T_out_data[1] = (float *)malloc(sizeof_float * 4*4);
	memset(intern_variable_t.T_out_data[1], 0, sizeof_float * 4 * 4);
	Matrix_Init(&intern_variable_t.T_out[1],4,4,(float *)intern_variable_t.T_out_data[1]);
	
	intern_variable_t.BTMB = (float *)malloc(sizeof_float * 3*3);
	memset(intern_variable_t.BTMB, 0, sizeof_float * 3 * 3);
	Matrix_Init(&intern_variable_t.R_out,3,3,(float *)intern_variable_t.BTMB);
	
	intern_variable_t.TM = (float *)malloc(sizeof_float * 3*3);
	memset(intern_variable_t.TM, 0, sizeof_float * 3 * 3);
	Matrix_Init(&intern_variable_t.T_M,3,3,(float *)intern_variable_t.TM);
	
	intern_variable_t.TM_inverse = (float *)malloc(sizeof_float * 3*3);
	memset(intern_variable_t.TM_inverse, 0, sizeof_float * 3 * 3);
	Matrix_Init(&intern_variable_t.T_M_inverse,3,3,(float *)intern_variable_t.TM_inverse);
	
	intern_variable_t.RL = (float *)malloc(sizeof_float * 3*3);
	memset(intern_variable_t.RL, 0, sizeof_float * 3 * 3);
	Matrix_Init(&intern_variable_t.R_L,3,3,(float *)intern_variable_t.RL);

}

float roll2_coil_part()
{    
		float joint_deg_roll2;
		arm_status status;
		DH2TM_t(0	,0		,-Joint_deg.deg_yaw			,off_z	,intern_variable_t.ATMA[0]);
		DH2TM_t(90	,-off_x	,90-Joint_deg.deg_pitch1	,0		,intern_variable_t.ATMA[1]);
		DH2TM_t(0	,330	,180-Joint_deg.deg_pitch2	,0		,intern_variable_t.ATMA[2]);
		DH2TM_t(90	,-9.5	,0+Joint_deg.deg_roll1		,357.91	,intern_variable_t.ATMA[3]);
		DH2TM_t(-90	,0		,90-Joint_deg.deg_pitch3	,0		,intern_variable_t.ATMA[4]);
		DH2TM_t(90	,0		,0,82,intern_variable_t.ATMA[5]);

		//intern_variable_t.T_out[0]=matrix_mult_multi_4(T,6); //位姿矩阵
		arm_mat_mult_f32(&intern_variable_t.T[0],&intern_variable_t.T[1],&intern_variable_t.T_out[0]);
		arm_mat_mult_f32(&intern_variable_t.T_out[0],&intern_variable_t.T[2],&intern_variable_t.T_out[1]);
		arm_mat_mult_f32(&intern_variable_t.T_out[1],&intern_variable_t.T[3],&intern_variable_t.T_out[0]);
		arm_mat_mult_f32(&intern_variable_t.T_out[0],&intern_variable_t.T[4],&intern_variable_t.T_out[1]);
		arm_mat_mult_f32(&intern_variable_t.T_out[1],&intern_variable_t.T[5],&intern_variable_t.T_out[0]);

		intern_variable_t.BTMB[0] = intern_variable_t.T_out[0].pData[0];
		intern_variable_t.BTMB[1] = intern_variable_t.T_out[0].pData[1];
		intern_variable_t.BTMB[2] = intern_variable_t.T_out[0].pData[2];
		intern_variable_t.BTMB[3] = intern_variable_t.T_out[0].pData[4];
		intern_variable_t.BTMB[4] = intern_variable_t.T_out[0].pData[5];
		intern_variable_t.BTMB[5] = intern_variable_t.T_out[0].pData[6];
		intern_variable_t.BTMB[6] = intern_variable_t.T_out[0].pData[8];
		intern_variable_t.BTMB[7] = intern_variable_t.T_out[0].pData[9];
		intern_variable_t.BTMB[8] = intern_variable_t.T_out[0].pData[10];

		intern_variable_t.TM[0] = T_m.data[0];
		intern_variable_t.TM[1] = T_m.data[1];
		intern_variable_t.TM[2] = T_m.data[2];
		intern_variable_t.TM[3] = T_m.data[4];
		intern_variable_t.TM[4] = T_m.data[5];
		intern_variable_t.TM[5] = T_m.data[6];
		intern_variable_t.TM[6] = T_m.data[8];
		intern_variable_t.TM[7] = T_m.data[9];
		intern_variable_t.TM[8] = T_m.data[10];

		status = arm_mat_inverse_f32(&intern_variable_t.T_M, &intern_variable_t.T_M_inverse);//求逆
		if(status != ARM_MATH_SUCCESS)
		return 0;

		status = arm_mat_mult_f32(&intern_variable_t.R_out,&intern_variable_t.T_M_inverse,&intern_variable_t.R_L);//与求逆矩阵相乘
		if(status != ARM_MATH_SUCCESS)
		return 0;

		joint_deg_roll2 = -atan2(-intern_variable_t.R_L.pData[6], sqrt(intern_variable_t.R_L.pData[0]*intern_variable_t.R_L.pData[0] + 
						intern_variable_t.R_L.pData[3]*intern_variable_t.R_L.pData[3]));
		joint_deg_roll2 = rad2deg(joint_deg_roll2);
		if(!direction_t)
			joint_deg_roll2 =  - joint_deg_roll2;
		else 
		{
			//if(if_trans_flag)//在兑矿
			//joint_deg_roll2 += 3;
		} //箭头朝右
		return joint_deg_roll2;
	  
}
/*
求解roll2()
*/
float roll2_coil_part1()
{    
	  arm_matrix_instance_f32 T[6];
      float32_t ATMA[6][16];
	  arm_matrix_instance_f32 R[6];
	  float32_t BTMB[9];
      T[0] = DH2TM(0	,0		,-Joint_deg.deg_yaw			,off_z	,ATMA[0]);
	  T[1] = DH2TM(90	,-off_x	,90-Joint_deg.deg_pitch1	,0		,ATMA[1]);
	  T[2] = DH2TM(0	,330	,180-Joint_deg.deg_pitch2	,0		,ATMA[2]);
	  T[3] = DH2TM(90	,-9.5	,0+Joint_deg.deg_roll1		,357.91	,ATMA[3]);
	  T[4] = DH2TM(-90	,0		,90-Joint_deg.deg_pitch3	,0		,ATMA[4]);
	  T[5] = DH2TM(90	,0		,0,132,ATMA[5]);
	  intern_variable_t.T_out[0]=matrix_mult_multi_4(T,6); //位姿矩阵
	
	  
	  BTMB[0] = intern_variable_t.T_out[0].pData[0];
	  BTMB[1] = intern_variable_t.T_out[0].pData[1];
	  BTMB[2] = intern_variable_t.T_out[0].pData[2];
	  BTMB[3] = intern_variable_t.T_out[0].pData[4];
	  BTMB[4] = intern_variable_t.T_out[0].pData[5];
	  BTMB[5] = intern_variable_t.T_out[0].pData[6];
	  BTMB[6] = intern_variable_t.T_out[0].pData[8];
	  BTMB[7] = intern_variable_t.T_out[0].pData[9];
	  BTMB[8] = intern_variable_t.T_out[0].pData[10];
	  
	
	  arm_mat_init_f32(&intern_variable_t.R_out, 3, 3,(float32_t *)BTMB );	//旋转矩阵
	  
	  
//    matrix_test[0] = intern_variable_t.R_out.pData[0];
//	  matrix_test[1] = intern_variable_t.R_out.pData[1];
//	  matrix_test[2] = intern_variable_t.R_out.pData[2];
//	  matrix_test[3] = intern_variable_t.R_out.pData[3];
//	  matrix_test[4] = intern_variable_t.R_out.pData[4];
//	  matrix_test[5] = intern_variable_t.R_out.pData[5];
//	  matrix_test[6] = intern_variable_t.R_out.pData[6];
//	  matrix_test[7] = intern_variable_t.R_out.pData[7];

}
float joint_deg_roll2;
float roll2_coil_part2()
{ 
	arm_status status;
	arm_matrix_instance_f32 R_L;
	//创建需要矩阵及变量
	float32_t TM[9]={
	T_m.data[0],T_m.data[1],T_m.data[2],
	T_m.data[4],T_m.data[5],T_m.data[6],
	T_m.data[8],T_m.data[9],T_m.data[10],
	};//T_M姿态矩阵
	arm_matrix_instance_f32 T_M;
	float32_t TM_inverse[9];
	arm_matrix_instance_f32 T_M_inverse;
	float32_t RL[9];
	
	//初始化
	arm_mat_init_f32(&T_M, 3, 3,(float32_t *)TM );
	arm_mat_init_f32(&T_M_inverse, 3, 3,(float32_t *)TM_inverse );
	arm_mat_init_f32(&R_L, 3, 3,(float32_t *)RL );
	//运算
	status = arm_mat_inverse_f32(&T_M, &T_M_inverse);//求逆
	if(status != ARM_MATH_SUCCESS)
		return 0;
	
	status = arm_mat_mult_f32(&intern_variable_t.R_out,&T_M_inverse,&R_L);//与求逆矩阵相乘
	if(status != ARM_MATH_SUCCESS)
		return 0;
	
	joint_deg_roll2 = -atan2(-R_L.pData[6], sqrt(R_L.pData[0]*R_L.pData[0] + R_L.pData[3]*R_L.pData[3]));
	joint_deg_roll2 = rad2deg(joint_deg_roll2);
	if(!direction_t)
		joint_deg_roll2 =  - joint_deg_roll2;
	else 
		{
		} //箭头朝右
	return joint_deg_roll2;
}

/*---------------------------------------------------------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------------------------------------------------------*/

float32_t object[16]={
-0.9936 ,  -0.0279  , -0.0387   ,    300,
0.0271  , -0.9996 ,   0.0066 , 320.0000,
-0.1098 ,   0.0055 ,   0.9992 , 149.0000,
	 0  ,       0 ,        0  ,  1.0000
};
float32_t Rc[16] ={
 0,     0,     1.0,     0,
 1.0,     0,     0,     0,
 0,     1.0,     0,     0,
 0,     0,     0,     1.0
};//旋转矩阵 将物体坐标系变换到机械臂的物体坐标系  


	
//初始化坐标系转换需要的各矩阵
void Vision_coil_init(Vision_coil *vision_coil_t)
{
	vision_coil_t->camera_coor_data[0] = (float *)malloc(sizeof_float * 4*4);
	memset(vision_coil_t->camera_coor_data[0], 0, sizeof_float * 4 * 4);
	Matrix_Init(&vision_coil_t->camera_coor[0],4,4,(float *)vision_coil_t->camera_coor_data[0]);
	
	vision_coil_t->camera_coor_data[1] = (float *)malloc(sizeof_float * 4*4);
	memset(vision_coil_t->camera_coor_data[1], 0, sizeof_float * 4 * 4);
	Matrix_Init(&vision_coil_t->camera_coor[1],4,4,(float *)vision_coil_t->camera_coor_data[1]);
	
	vision_coil_t->object_data = (float *)malloc(sizeof_float * 4*4);
	memset(vision_coil_t->object_data, 0, sizeof_float * 4 * 4);
	Matrix_Init(&vision_coil_t->object,4,4,(float *)vision_coil_t->object_data);
	
	vision_coil_t->object_coor_base_data = (float *)malloc(sizeof_float * 4*4);
	memset(vision_coil_t->object_coor_base_data, 0, sizeof_float * 4 * 4);
	Matrix_Init(&vision_coil_t->object_coor_base,4,4,(float *)vision_coil_t->object_coor_base_data);
	
	vision_coil_t->object_coor_data = (float *)malloc(sizeof_float * 4*4);
	memset(vision_coil_t->object_coor_data, 0, sizeof_float * 4 * 4);
	Matrix_Init(&vision_coil_t->object_coor,4,4,(float *)vision_coil_t->object_coor_data);
	
	vision_coil_t->rotate_data = (float *)malloc(sizeof_float * 4*4);
	memset(vision_coil_t->rotate_data, 0, sizeof_float * 4 * 4);
	Matrix_Init(&vision_coil_t->rotate,4,4,(float *)vision_coil_t->rotate_data);
	
	//DH表旋转矩阵
	vision_coil_t->R_data[0] = (float *)malloc(sizeof_float * 4*4);
	memset(vision_coil_t->R_data[0], 0, sizeof_float * 4 * 4);
	Matrix_Init(&vision_coil_t->R[0],4,4,(float *)vision_coil_t->R_data[0]);
	
	vision_coil_t->R_data[1] = (float *)malloc(sizeof_float * 4*4);
	memset(vision_coil_t->R_data[1], 0, sizeof_float * 4 * 4);
	Matrix_Init(&vision_coil_t->R[1],4,4,(float *)vision_coil_t->R_data[1]);
	
	vision_coil_t->R_data[2] = (float *)malloc(sizeof_float * 4*4);
	memset(vision_coil_t->R_data[2], 0, sizeof_float * 4 * 4);
	Matrix_Init(&vision_coil_t->R[2],4,4,(float *)vision_coil_t->R_data[2]);
	
	vision_coil_t->R_data[3] = (float *)malloc(sizeof_float * 4*4);
	memset(vision_coil_t->R_data[3], 0, sizeof_float * 4 * 4);
	Matrix_Init(&vision_coil_t->R[3],4,4,(float *)vision_coil_t->R_data[3]);
	
	
	memcpy(vision_coil_t->object_data, object, sizeof(object));
	memcpy(vision_coil_t->rotate_data, Rc, sizeof(Rc));
	//函数指针
	Vision_coil_t.get_photo_coordinate = get_photo_coordinate;
	Vision_coil_t.get_taget_coordinate = get_taget_coordinate;
	
}

// 获取相机相对基坐标系位姿
void get_photo_coordinate(Vision_coil *vision_coil_t)
{
//	arm_matrix_instance_f32 R[4];
//	float32_t RT[4][16];
//	R[0] = DH2TM(0	,0		,-Joint_deg.current_deg_yaw			,off_z	,RT[0]);
//	R[1] = DH2TM(90	,-off_x	,90-Joint_deg.current_deg_pitch1	,0		,RT[1]);
//	R[2] = DH2TM(0	,330	,103-Joint_deg.current_deg_pitch2	,0		,RT[2]);
//	R[3] = DH2TM(90	,120	,-90 ,0	,RT[3]);
//	vision_coil_t->camera_coor[0]=matrix_mult_multi_4(R,4); //旋转矩阵
	
	DH2TM_t(0	,0		,-Joint_deg.current_deg_yaw			,off_z	,vision_coil_t->R_data[0]);
	DH2TM_t(90	,-off_x	,90-Joint_deg.current_deg_pitch1	,0	,vision_coil_t->R_data[1]);
	DH2TM_t(0	,330	,102-Joint_deg.current_deg_pitch2	,0	,vision_coil_t->R_data[2]);
	DH2TM_t(90	,120	,-90 ,0	,vision_coil_t->R_data[3]);
	
	arm_mat_mult_f32(&vision_coil_t->R[0],&vision_coil_t->R[1],&vision_coil_t->camera_coor[0]);
	arm_mat_mult_f32(&vision_coil_t->camera_coor[0],&vision_coil_t->R[2],&vision_coil_t->camera_coor[1]);
	arm_mat_mult_f32(&vision_coil_t->camera_coor[1],&vision_coil_t->R[3],&vision_coil_t->camera_coor[0]);
	
}

//根据返回位姿矩阵求解目标位姿矩阵
/*最终塞进兑换槽目标位置量为（Dx，Dy，Dz）。为防止超出机械臂活动范围，主要是前后（x方向）和左右的（y方向）。
于是我们令Dx和Dy为一个固定值，Dz取识别到的数据。最后使用底盘来补偿剩下的位移。根据识别出物体的实际位
姿沿兑换口朝向的向量（我们定义为Z轴向量，与目标机械臂末端方向相同），取沿该向量模长为D（我们选择300mm）的点，
以此来确定兑换的一个中间点。到达该中点后，使机械臂末端沿其z轴向量反向运动使矿石进入兑换槽中*/
int16_t chassis_x,chassis_y; 
float z_com=20,x_com,theta_roll;
float *get_taget_coordinate(Vision_coil *vision_coil_t)
{
    get_photo_coordinate(&Vision_coil_t);

	/*视觉数*/
	vision_coil_t->object_data[0] = Vision_t.matrix_data[0];
	vision_coil_t->object_data[1] = Vision_t.matrix_data[1];
	vision_coil_t->object_data[2] = Vision_t.matrix_data[2];
	vision_coil_t->object_data[4] = Vision_t.matrix_data[3];
	vision_coil_t->object_data[5] = Vision_t.matrix_data[4];
	vision_coil_t->object_data[6] = Vision_t.matrix_data[5];
	vision_coil_t->object_data[8] = Vision_t.matrix_data[6];
	vision_coil_t->object_data[9] = Vision_t.matrix_data[7];
	vision_coil_t->object_data[10] = Vision_t.matrix_data[8];
	
//	//位置量
//	vision_coil_t->object_data[3]  = Vision_t.move[0]*0-(300*Vision_t.matrix_data[0]);
//	vision_coil_t->object_data[7]  = Vision_t.move[1]*0+380;
//	/*z补偿*/
//	theta_roll = -asin(T_m.data[8]);
//	//theta_roll = atan2(Vision_t.matrix_data[7],Vision_t.matrix_data[8]);
	if(direction_t)
	{
		if(Vision_t.move[2]>50)
			z_com = -25;//朝右
		else
			z_com = -25;//朝右
	}
	else
	{
		if(Vision_t.move[2]>50)
			z_com = -10 ;//朝左
		else
			z_com = -10;//朝左
	}
//	
//	vision_coil_t->object_data[11] = Vision_t.move[2]+z_com;
//	
//	chassis_x=Vision_t.move[1]-410;
//	chassis_y=Vision_t.move[0]-300*Vision_t.matrix_data[0];

//	Matrix_Multiply(&vision_coil_t->object,&vision_coil_t->rotate,&vision_coil_t->object_coor);
//	Matrix_Multiply(&vision_coil_t->camera_coor[0],&vision_coil_t->object_coor,&vision_coil_t->object_coor_base);
//	matrix_test[0] = vision_coil_t->object_coor_base.pData[0];
//	matrix_test[1] = vision_coil_t->object_coor_base.pData[1];
//	matrix_test[2] = vision_coil_t->object_coor_base.pData[2];
//	matrix_test[3] = vision_coil_t->object_coor_base.pData[6];
//	matrix_test[4] = vision_coil_t->object_coor_base.pData[10];
//	matrix_test[5] = vision_coil_t->object_coor_base.pData[11];
//	matrix_test[6] = vision_coil_t->object_coor_base.pData[1];
//	matrix_test[7] = vision_coil_t->object_coor_base.pData[11];

	vision_coil_t->object_data[3]  = Vision_t.move[0];
	vision_coil_t->object_data[7]  = Vision_t.move[1];
	vision_coil_t->object_data[11] = Vision_t.move[2]+z_com;
	Matrix_Multiply(&vision_coil_t->object,&vision_coil_t->rotate,&vision_coil_t->object_coor);
	Matrix_Multiply(&vision_coil_t->camera_coor[0],&vision_coil_t->object_coor,&vision_coil_t->object_coor_base);

	if(direction_t)
		chassis_x=-vision_coil_t->object_coor_base.pData[3]-455;
	else
		chassis_x=-vision_coil_t->object_coor_base.pData[3]-455;//左边
	theta_yaw = atan2(vision_coil_t->object_coor_base.pData[4],vision_coil_t->object_coor_base.pData[0]);
	//y_com_chassis = 144*arm_sin_f32(theta_yaw);
	if(direction_t)//朝右
	{
		chassis_y=vision_coil_t->object_coor_base.pData[7]-350*vision_coil_t->object_coor_base.pData[6];
		vision_coil_t->object_coor_base.pData[3]  = -430;
		vision_coil_t->object_coor_base.pData[7]  = 0-300*vision_coil_t->object_coor_base.pData[6];
	}
	else
	{
		chassis_y=vision_coil_t->object_coor_base.pData[7]-300*vision_coil_t->object_coor_base.pData[6];
		vision_coil_t->object_coor_base.pData[3]  = -430;
		vision_coil_t->object_coor_base.pData[7]  = 0-250*vision_coil_t->object_coor_base.pData[6];

	}

	return vision_coil_t->object_coor_base.pData;
		   
}


// 获取坐标系下各关节角度
void get_current_angle(Manipulator_t* manipulator)
{
//	Joint_deg.current_deg_pitch1 = manipulator->pitch1_deg.angle_target + 75;
//	Joint_deg.current_deg_pitch2 = manipulator->pitch2_deg.angle_target - 80;
//	Joint_deg.current_deg_pitch3 = manipulator->pitch3_deg.angle_target + 87;
//	Joint_deg.current_deg_yaw = -manipulator->yaw_deg.angle_target;
//	Joint_deg.current_deg_roll1 = manipulator->roll1_deg.angle_target;
//	Joint_deg.current_deg_roll2 = -manipulator->roll2_deg.angle_target;
	
	Joint_deg.current_deg_pitch1 = manipulator->pitch1_deg.angle_target + 77;
	Joint_deg.current_deg_pitch2 = manipulator->pitch2_deg.angle_target - 80;
	Joint_deg.current_deg_pitch3 = manipulator->pitch3_deg.angle_target + 84;
	Joint_deg.current_deg_yaw = -manipulator->yaw_deg.angle_target;
	Joint_deg.current_deg_roll1 = manipulator->roll1_deg.angle_target;
	Joint_deg.current_deg_roll2 = -manipulator->roll2_deg.angle_target;
}


// 将逆解角度转换为控制角度
float roll2_com=0;//需要调整roll2水平
void taget_deg_2_control_deg(Manipulator_t* manipulator)
{
//	manipulator->pitch1_deg.angle_target = Joint_deg.deg_pitch1 - 75;
//	manipulator->pitch2_deg.angle_target = Joint_deg.deg_pitch2 + 80;
//	manipulator->pitch3_deg.angle_target = Joint_deg.deg_pitch3 - 87;
//	manipulator->yaw_deg.angle_target    = -Joint_deg.deg_yaw;
//	manipulator->roll1_deg.angle_target  = Joint_deg.deg_roll1;
//	manipulator->roll2_deg.angle_target  = -Joint_deg.deg_roll2 + roll2_com;
	
	manipulator->pitch1_deg.angle_target = Joint_deg.deg_pitch1 - 77;
	manipulator->pitch2_deg.angle_target = Joint_deg.deg_pitch2 + 80;
	manipulator->pitch3_deg.angle_target = Joint_deg.deg_pitch3 - 84;
	manipulator->yaw_deg.angle_target    = -Joint_deg.deg_yaw;
	manipulator->roll1_deg.angle_target  = Joint_deg.deg_roll1;
	manipulator->roll2_deg.angle_target  = -Joint_deg.deg_roll2 + roll2_com;
}

int8_t inver_start_test=0;
void vision_coil_test(float *target_angle,float *target_x)
{
	//get_current_angle(&Manipulator);
	if_trans_flag = 0;
	inverse_coil(target_angle[0],target_angle[1],target_angle[2],target_x[0],target_x[1],target_x[2]);
	taget_deg_2_control_deg(&Manipulator);
	
}



int8_t inver_enable=0;//相机识别到才能执行自动兑矿
int8_t inver_start=0;
int16_t inver_count=0;
int8_t inver_start_next=0;
int8_t inver_start_next_rc=0;
int inver_first_count=0;
int8_t inverse_next = 1;//需要两次自动兑矿，判断是否需要下次

/*遥控自动兑矿标志位判断*/
void auto_exchange_judge()
{
//	//遥控
//	if(inver_start_next_rc == 0 &&rc_Ctrl.rc.s1 == 1)
//	{
//		inver_start_next_rc = 1;
//		inver_start = 1;
//	}
//	
//	if(inver_start_next_rc == 2 &&rc_Ctrl.rc.s1 == 1)
//	{	
//		inver_start_next_rc = 3;
//		if(inver_enable==1&&Vision_t.online ==1)
//			inver_start = 2;
//		else
//			inver_start = 0;
//	}
//	
//	if(inver_start_next_rc == 1 &&rc_Ctrl.rc.s1 == 3)
//	{
//		inver_start_next_rc = 2;
//	}
//	if(inver_start_next_rc == 3 &&rc_Ctrl.rc.s1 == 3)
//		inver_start_next_rc = 0;
	//键盘
	if(((rc_Ctrl.key_Q ==1 && rc_Ctrl.key_shift ==1)||(vT13.key_Q ==1 && vT13.key_shift ==1)) && inver_start_next == 0)
	{
		inver_start_next = 1;
		inver_start = 1;
	}
	
	if(((rc_Ctrl.key_Q ==0 && rc_Ctrl.key_shift ==0)&&(vT13.key_Q ==0 && vT13.key_shift ==0)) && inver_start_next == 1)
	{
		inver_start_next = 2;
	}
	
	if(((rc_Ctrl.key_Q ==1 && rc_Ctrl.key_shift ==1)||(vT13.key_Q ==1 && vT13.key_shift ==1)) && inver_start_next ==2)
	{
		inver_start_next = 3;
		if(inver_enable==1)
			inver_start = 2;
		else
			inver_start = 0;
	}

	
	//取消前置
	if((rc_Ctrl.key_S ==1 && rc_Ctrl.key_shift ==1)||(vT13.key_S ==1 && vT13.key_shift ==1))
	{
		inver_start = 0;
		inver_start_next = 0;
		inver_count = 0;
		inver_first_count = 0;
		inverse_next = 1;
		//机械臂收回在取矿前置取消动作
	}
	
}


float target_sum[6];
float target_averagedeg_check[6];
int8_t average_count = 0;

void inverse_test(Manipulator_t* manipulator)//验证逆解正确
{
	auto_exchange_judge();
//	inverse_coil(-76.16,98,-94,-212,y,z);
//	if(inver_start == 1)
//	{
//		taget_deg_2_control_deg(manipulator);
//	}
	if(inver_start == 1)//识别前置
	{   
		inver_first_count ++;
		rc_Ctrl.key_E_flag =1;
		Manipulator.pitch1_deg.angle_target=-55;
		Manipulator.pitch2_deg.angle_target=78;
		Manipulator.pitch3_deg.angle_target=45;
		Manipulator.yaw_deg.angle_target=0;
		Manipulator.roll1_deg.angle_target=0;
		if(inverse_next ==0)
			Manipulator.roll2_deg.angle_target=0;
		roll2_com = Manipulator.roll2_deg.angle_target;
		get_current_angle(&Manipulator);
		if(inver_first_count <10)
		{
			Vacuum_pump_vice(open);
			arm_relese(close);
			if_trans_flag =1;
		}
		if(inver_first_count>150&&inver_first_count%2 == 0)
		{
				//average_count ++;
				visiondata_inverse_coil();//获取视觉逆解数据并转换到基坐标系，求解目标当前坐标系下的目标角度
//				target_sum[0] +=  Joint_deg.deg_yaw;
//				target_sum[1] +=  Joint_deg.deg_pitch1;
//				target_sum[2] +=  Joint_deg.deg_pitch2;
//				target_sum[3] +=  Joint_deg.deg_roll1;
//				target_sum[4] +=  Joint_deg.deg_pitch3;
//				target_sum[5] +=  Joint_deg.deg_roll2;
//			if(average_count ==50)
//			{
//				target_averagedeg_check[0] = target_sum[0]/average_count;
//				target_averagedeg_check[1] = target_sum[1]/average_count;
//				target_averagedeg_check[2] = target_sum[2]/average_count;
//				target_averagedeg_check[3] = target_sum[3]/average_count;
//				target_averagedeg_check[4] = target_sum[4]/average_count;
//				target_averagedeg_check[5] = target_sum[5]/average_count;
//				average_count = 0;
//				target_sum[0] = 0;
//				target_sum[1] = 0;
//				target_sum[2] = 0;
//				target_sum[3] = 0;
//				target_sum[4] = 0;
//				target_sum[5] = 0;
//			}
			judge_direction();//判断方向
		};//提前求解,告知操作手，判断可否执行下一步
	}
	if(inver_start == 2)//识别后
	{
		inver_first_count = 0;
		inver_count ++;
		if(inver_count> 130&&inver_count <150)
		{
			average_count ++;
			visiondata_inverse_coil();//获取视觉逆解数据并转换到基坐标系，求解目标当前坐标系下的目标角度
			target_sum[0] +=  Joint_deg.deg_yaw;
			target_sum[1] +=  Joint_deg.deg_pitch1;
			target_sum[2] +=  Joint_deg.deg_pitch2;
			target_sum[3] +=  Joint_deg.deg_roll1;
			target_sum[4] +=  Joint_deg.deg_pitch3;
			target_sum[5] +=  Joint_deg.deg_roll2;
		}
		if(inver_count == 150)
		{
			Joint_deg.deg_yaw = target_sum[0]/average_count;
			Joint_deg.deg_pitch1 = target_sum[1]/average_count;
			Joint_deg.deg_pitch2 = target_sum[2]/average_count;
			Joint_deg.deg_roll1 = target_sum[3]/average_count;
			Joint_deg.deg_pitch3 = target_sum[4]/average_count;
			Joint_deg.deg_roll2 = target_sum[5]/average_count;
			average_count = 0;
			target_sum[0] = 0;
			target_sum[1] = 0;
			target_sum[2] = 0;
			target_sum[3] = 0;
			target_sum[4] = 0;
			target_sum[5] = 0;
			Manipulator.pitch3_deg.velocity = 0.05;
			Manipulator.yaw_deg.velocity = 0.05;
			
		}
			judge_direction();//判断方向
		if(inver_count >=150) 
		{
			
			inver_start_next = 0;
			inver_count =100;
			inver_start =3;
		}
	}
	if(inver_start == 3)//识别后平移
	{
		inver_count ++;
		if(inver_count ==110)
		{
			auto_flag =1;
			move_back_flag = 4;//右平移（0.3m/s）
		}
		if(inver_count >110&&move_back_flag == 4&&move_x_ok == 1)
		{
			move_back_flag = 5;//前进（0.3m/s）
			taget_deg_2_control_deg(manipulator);
		}
		if(inver_count >110&&move_back_flag == 5 && move_x_ok == 0)
		{
			move_back_flag = 0;
			auto_flag =0; 
			inver_count =310;
			inver_start =4;
			//inver_count =0;
			//inver_start =0;
		}
//		roll2_feed = Manipulator.roll2_deg.angle_target;
//		roll1_feed = Manipulator.roll1_deg.angle_target;
//		pitch3_feed = Manipulator.pitch3_deg.angle_target;
//		pitch1_feed = Manipulator.pitch1_deg.angle_target;
//		pitch2_feed = Manipulator.pitch2_deg.angle_target;
//		yaw_feed = Manipulator.yaw_deg.angle_target;
		if(inver_count > 1000) inver_count = 1000;
	}
	
	if(inver_start == 4)//识别后兑换
	{
		inver_count ++;
		if(inver_count >= 360&&inver_count < 581)
		{
			if(direction_t)
				vision_x += 1.75*T_m.data[2];
			else
				vision_x += 1.75*T_m.data[2];
			vision_y += 1.5*T_m.data[6];
			vision_z += 1.5*T_m.data[10];
		}
		if(inver_count >630)
		{
			Manipulator.pitch3_deg.velocity = 0.0;
			Manipulator.yaw_deg.velocity = 0.0;
			if(inverse_next == 1 && second_trans)
				inver_start =5;//准备下一次自动对矿
			else
				inver_start =6;//兑换完毕
			Vacuum_pump_vice(close);
			arm_relese(open);
		}
		get_current_angle(&Manipulator);
		redemption_inverse_coil(vision_x,vision_y,vision_z);
		taget_deg_2_control_deg(manipulator);
	}
	
	if(inver_start == 5)//兑换完成
	{
		inver_count ++;
		if(inver_count == 650)
			Manipulator.yaw_deg.angle_target = 0;
		if(inver_count == 740)
		{
			Manipulator.roll2_deg.angle_target = 0;
			roll2_com = 0;
			Manipulator.roll1_deg.angle_target = 0;
			Manipulator.pitch3_deg.angle_target = 0;
		}
		if(inver_count == 780)
		{
			Manipulator.pitch1_deg.angle_target = 0;
			Manipulator.pitch2_deg.angle_target = 0;
		}
		if(inver_count == 800)
		{
			auto_flag =1;
			move_back_flag = 1;
		}
		if(inver_count == 850)
			associate_stretch_auto_flag = 1;
		if(inver_count == 950)
		{
			auto_flag =0;
			move_back_flag = 0;
			chassis_y = -chassis_y;
		}
		if(inver_count ==960)
		{
			auto_flag =1;
			move_back_flag = 4;
		}
		if(inver_count>1000&&move_back_flag == 4&&move_x_ok ==1)
		{
			inverse_next =0;
			move_back_flag = 0;
		}
		if(inver_count>1350)
		{
			associate_stretch_right_flag=0;
			associate_stretch_count=0;
			auto_flag =0;
			inver_start =1;
			inver_count = 0;
			inver_start_next = 2;
			inver_start_next_rc = 2;
		}
	}
	
	
		if(inver_start == 6)//兑换完成
	{
		inver_count ++;
		if(inver_count ==650)
			Manipulator.yaw_deg.angle_target = 0;
		if(inver_count == 740)
		{
			Manipulator.roll2_deg.angle_target = 0;
			roll2_com = 0;
			Manipulator.roll1_deg.angle_target = 0;
			Manipulator.pitch3_deg.angle_target = 0;
		}
		if(inver_count == 780)
		{
			Manipulator.pitch1_deg.angle_target = 0;
			Manipulator.pitch2_deg.angle_target = 0;
		}
		if(inver_count == 820)
		{
			inverse_next =1;
			auto_flag =1;
			move_back_flag = 1;
		}
		if(inver_count == 900)
		{
			inver_start =0;
			inver_count = 0;
			auto_flag =0;
			move_back_flag = 0;
		}	
	}
	
		
}


//角度转弧度函数
float deg2rad(float angle)
{
    return angle*d2r;
}
//弧度转角度函数
float rad2deg(float rad)
{
    return rad*r2d;
}
//计算空间两点距离
float space_distance(float* p1,float *p2)
{
	float dis,x2,y2,z2;
	x2=(p1[0]-p2[0])*(p1[0]-p2[0]);
	y2=(p1[1]-p2[1])*(p1[1]-p2[1]);
	z2=(p1[2]-p2[2])*(p1[2]-p2[2]);
	arm_sqrt_f32(x2+y2+z2,&dis);//求方根
	return dis;
}
//计算两个向量的点积
float get_dot(float *N1 , float *N2)
{
	float dot;
	dot = N1[0]*N2[0] + N1[1]*N2[1] + N1[2]*N2[2];
	return dot;
	
	
}
//计算三维空间向量模长
float get_norm(float *n)
{
	float norm,norm1,x2,y2,z2;
	x2= n[0] * n[0];
	y2= n[1] * n[1];
	z2= n[2] * n[2];
	arm_sqrt_f32(x2+y2+z2,&norm);//求方根
	return norm;

}

//判断兑换口朝向
int8_t direction_t = 1;
void judge_direction()
{
	if(T_m.data[6] > 0.4)
		direction_t = direction_LEFT;
	else if(T_m.data[6] < -0.4)
		direction_t = direction_RIGHT;
	
}