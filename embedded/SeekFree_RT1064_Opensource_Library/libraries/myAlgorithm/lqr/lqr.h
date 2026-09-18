#ifndef LQR_H
#define LQR_H

#include <stdint.h>
#include "math.h"
#include <stdlib.h>
#include "arm_math.h"
#include "FreeRTOS.h"
#include "task.h"

#ifdef portSTACK_GROWTH  // FreeRTOS 存在时使用 pvPortMalloc
#define user_malloc pvPortMalloc
#else
#define user_malloc malloc
#endif

#define mat arm_matrix_instance_f32
#define Mat_Init arm_mat_init_f32
#define Mat_Add arm_mat_add_f32
#define Mat_Sub arm_mat_sub_f32
#define Mat_Mult arm_mat_mult_f32
#define Mat_Tran arm_mat_trans_f32
#define Mat_Inv arm_mat_inverse_f32

extern uint16_t sizeof_float, sizeof_double;

typedef struct {

    uint16_t num_states,num_outputs;

    mat K,x_d,x,u,e;

    float *K_data,*x_d_data,*x_data,*u_data,*e_data;
}lqr_t;

void lqr_init(lqr_t* lqr,uint16_t num_states,uint16_t num_outputs,
               float* K_data,float* x_d_data,float* x_data,float* u_data,float* e_data);
               
void lqr_update(lqr_t* lqr);

void lqr_setK(lqr_t* lqr,float *K_data);

void lqr_setx_d(lqr_t* lqr,float *x_d_data);

void lqr_setx(lqr_t* lqr,float *x_data);

#endif

