/**
 ******************************************************************************
 * @file    matrix.c/h
 * @brief   Matrix/vector calculation. 矩阵/向量运算
 * @author  Nico ly
 ******************************************************************************
 * Copyright (c) 2023 Team CUBOT
 * All rights reserved.
 ******************************************************************************
 */

#ifndef MATRIX_H
#define MATRIX_H

#include "arm_math.h"

// 若运算速度不够,可以使用q31代替f32,但是精度会降低
#define mat              arm_matrix_instance_f32
#define Matrix_Init      arm_mat_init_f32
#define Matrix_Add       arm_mat_add_f32
#define Matrix_Subtract  arm_mat_sub_f32
#define Matrix_Multiply  arm_mat_mult_f32
#define Matrix_Transpose arm_mat_trans_f32
#define Matrix_Inverse   arm_mat_inverse_f32

arm_matrix_instance_f32 DH2TM(float alpha,float a,float theta, float d,float32_t *ATMA);
arm_matrix_instance_f32 matrix_mult_multi_4(arm_matrix_instance_f32 * matrix_input ,int mult_size);
arm_matrix_instance_f32 DH2RM(float alpha,float a,float theta, float d,float32_t *ATMA);
arm_matrix_instance_f32 matrix_mult_multi_3(arm_matrix_instance_f32 * matrix_input ,int mult_size);
arm_matrix_instance_f32 matrix_multi(arm_matrix_instance_f32 * matrix_input1 ,arm_matrix_instance_f32 * matrix_input2,int16_t num,int16_t col);

float DH2TM_t(float alpha,float a,float theta, float d,float *ATMA_f32);











#endif  // MATRIX_H
