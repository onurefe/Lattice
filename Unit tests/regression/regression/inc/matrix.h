#ifndef __MATRIX_H
#define __MATRIX_H

#include "global.h"

#ifdef __cplusplus
extern "C"
{
#endif

/* Exported definitions ----------------------------------------------------*/
#define MAX_SQUARE_MATRIX_SIZE 5

    /* Exported types ----------------------------------------------------------*/
    typedef struct
    {
        float *arr;
        uint16_t m;
        uint16_t n;
    } Matrix_t;

    /* Exported functions ------------------------------------------------------*/
    extern void Matrix_Transpose(Matrix_t *A, Matrix_t *R);
    extern Bool_t Matrix_Multiply(Matrix_t *A, Matrix_t *B, Matrix_t *R);
    extern Bool_t Matrix_Inverse(Matrix_t *A, Matrix_t *R);

#ifdef __cplusplus
}
#endif

#endif