/**
 ******************************************************************************
 * @file    matrix.c/h
 * @brief   Matrix/vector calculation. 矩阵/向量运算(二次封装)
 * @author  
 ******************************************************************************
 * Copyright (c) 2023 Team CUBOT
 * All rights reserved.
 ******************************************************************************
 */
#include "stm32h750xx.h"
#include "arm_math.h"
#include "inverse.h"
#include "stdio.h"
#include "stdlib.h"
/**
  * @brief多个4X4矩阵相乘封装
  */	
arm_matrix_instance_f32 matrix_mult_multi_4(arm_matrix_instance_f32 * matrix_input ,int mult_size)
{
	arm_matrix_instance_f32 matrix_out[2];
	float32_t ATMA_f32[2][16];
	arm_mat_init_f32(&matrix_out[0], 4, 4,(float32_t *)ATMA_f32[0] );
	arm_mat_init_f32(&matrix_out[1], 4, 4,(float32_t *)ATMA_f32[1] );
	int8_t multi_change=0;
	arm_mat_mult_f32(&matrix_input[0],&matrix_input[1],&matrix_out[multi_change]);
	for(int i=2;i <mult_size;i++)
	{
		arm_mat_mult_f32(&matrix_out[multi_change],&matrix_input[i],&matrix_out[(multi_change^1)]);
		multi_change = multi_change^1;
		
	}
	return matrix_out[multi_change];

}
/**
  * @brief多个3X3矩阵相乘封装
  */	
arm_matrix_instance_f32 matrix_mult_multi_3(arm_matrix_instance_f32 * matrix_input ,int mult_size)
{
	arm_matrix_instance_f32 matrix_out[2];
	float32_t ATMA_f32[2][9];
	arm_mat_init_f32(&matrix_out[0], 3, 3,(float32_t *)ATMA_f32[0] );
	arm_mat_init_f32(&matrix_out[1], 3, 3,(float32_t *)ATMA_f32[1] );
	int8_t multi_change=0;
	arm_mat_mult_f32(&matrix_input[0],&matrix_input[1],&matrix_out[multi_change]);
	for(int i=2;i <mult_size;i++)
	{
		arm_mat_mult_f32(&matrix_out[multi_change],&matrix_input[i],&matrix_out[(multi_change^1)]);
		multi_change = multi_change^1;
		
	}
	return matrix_out[multi_change];

}

arm_matrix_instance_f32 matrix_multi(arm_matrix_instance_f32 * matrix_input1 ,arm_matrix_instance_f32 * matrix_input2,int16_t num,int16_t col)
{
	arm_matrix_instance_f32 matrix_out;
	float32_t TM_f32[16];//为什么这个就可以呢，而上面的函数不行
	arm_mat_init_f32(&matrix_out, 4, 4,(float32_t * ) TM_f32);
	arm_mat_mult_f32(matrix_input1,matrix_input2,&matrix_out);
	return matrix_out;

}
 /**
  * @brief通过DH表转换成旋转矩阵
  经验证，仅仅创建局部变量传参获取矩阵后面的矩阵会改掉前面的矩阵，需要创建不同的float32_t *ATMA传进来才正常
  */	 
arm_matrix_instance_f32 DH2RM(float alpha,float a,float theta, float d,float32_t *ATMA)
{
	float32_t ATMA_f32[9]={
	
	arm_cos_f32(deg2rad(theta)),-arm_sin_f32(deg2rad(theta)),0,
	arm_sin_f32(deg2rad(theta))*arm_cos_f32(deg2rad(alpha)),arm_cos_f32(deg2rad(theta))*arm_cos_f32(deg2rad(alpha)),-arm_sin_f32(deg2rad(alpha)),
	arm_sin_f32(deg2rad(theta))*arm_sin_f32(deg2rad(alpha)),arm_cos_f32(deg2rad(theta))*arm_sin_f32(deg2rad(alpha)),arm_cos_f32(deg2rad(alpha))
	};
	for(int i=0;i<9;i++)
	ATMA[i] = ATMA_f32[i];
	arm_matrix_instance_f32 matrix_out;
	arm_mat_init_f32(&matrix_out, 3, 3,(float32_t *)ATMA );	
	return matrix_out;
}
/**
  * @brief将DH表转换成位姿矩阵
  经验证，仅仅创建局部变量传参获取矩阵后面的矩阵会改掉前面的矩阵，需要创建不同的float32_t *ATMA传进来才正常
  */	
arm_matrix_instance_f32 DH2TM(float alpha,float a,float theta, float d,float32_t *ATMA)
{
	float32_t ATMA_f32[16]={
	
	arm_cos_f32(deg2rad(theta)),-arm_sin_f32(deg2rad(theta)),0,a,
	arm_sin_f32(deg2rad(theta))*arm_cos_f32(deg2rad(alpha)),arm_cos_f32(deg2rad(theta))*arm_cos_f32(deg2rad(alpha)),-arm_sin_f32(deg2rad(alpha)),-d*arm_sin_f32(deg2rad(alpha)),
	arm_sin_f32(deg2rad(theta))*arm_sin_f32(deg2rad(alpha)),arm_cos_f32(deg2rad(theta))*arm_sin_f32(deg2rad(alpha)),arm_cos_f32(deg2rad(alpha)),d*arm_cos_f32(deg2rad(alpha)),0,0,0,1
	};
	for(int i=0;i<16;i++)
	ATMA[i] = ATMA_f32[i];
	arm_matrix_instance_f32 matrix_out;
	arm_mat_init_f32(&matrix_out, 4, 4,(float32_t *)ATMA );	
	return matrix_out;
}

//DH表转旋转矩阵(数组)
float DH2TM_t(float alpha,float a,float theta, float d,float *ATMA_f32)
{
	float32_t ATMA_f32_t[16]={
	
	arm_cos_f32(deg2rad(theta)),-arm_sin_f32(deg2rad(theta)),0,a,
	arm_sin_f32(deg2rad(theta))*arm_cos_f32(deg2rad(alpha)),arm_cos_f32(deg2rad(theta))*arm_cos_f32(deg2rad(alpha)),-arm_sin_f32(deg2rad(alpha)),-d*arm_sin_f32(deg2rad(alpha)),
	arm_sin_f32(deg2rad(theta))*arm_sin_f32(deg2rad(alpha)),arm_cos_f32(deg2rad(theta))*arm_sin_f32(deg2rad(alpha)),arm_cos_f32(deg2rad(alpha)),d*arm_cos_f32(deg2rad(alpha)),0,0,0,1
	};
	memcpy(ATMA_f32, ATMA_f32_t, sizeof(ATMA_f32_t));

}