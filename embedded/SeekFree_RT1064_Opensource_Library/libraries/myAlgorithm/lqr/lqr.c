#include "lqr.h"

void lqr_init(lqr_t* lqr,uint16_t num_states,uint16_t num_outputs,
               float* K_data,float* x_d_data,float* x_data,float* u_data,float* e_data)
{
    lqr->num_states = num_states;
    lqr->num_outputs = num_outputs;

    lqr->K_data = K_data;
    lqr->x_d_data = x_d_data;
    lqr->x_data = x_data;
    lqr->u_data = u_data;
    lqr->e_data = e_data;

    Mat_Init(&lqr->K, num_outputs, num_states, lqr->K_data);
    Mat_Init(&lqr->x_d, num_states, 1, lqr->x_d_data);
    Mat_Init(&lqr->x, num_states, 1, lqr->x_data);
    Mat_Init(&lqr->u, num_outputs, 1, lqr->u_data);
    Mat_Init(&lqr->e, num_outputs, 1, lqr->e_data);
}

void lqr_update(lqr_t* lqr){
    Mat_Sub(&lqr->x_d,&lqr->x,&lqr->e);
    Mat_Mult(&lqr->K,&lqr->e,&lqr->u);
}

void lqr_setK(lqr_t *lqr, float *K_data){
    lqr->K_data = K_data;
    Mat_Init(&lqr->K, lqr->num_outputs, lqr->num_states, lqr->K_data);
}

void lqr_setx_d(lqr_t *lqr, float *x_d_data){
    lqr->x_d_data = x_d_data;
    Mat_Init(&lqr->x_d, lqr->num_states, 1, lqr->x_d_data);
}

void lqr_setx(lqr_t *lqr, float *x_data){
    lqr->x_data = x_data;
    Mat_Init(&lqr->x, lqr->num_states, 1, lqr->x_data);
}

float Poly_Cal(float* coeffs, float x, uint8_t degree) {
    float result = 0.0f;
    for (uint8_t i = 0; i <= degree; i++) {
        result += coeffs[i] * powf(x, degree - i);
    }
    return result;
}
