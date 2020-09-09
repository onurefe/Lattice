#ifndef __COMPLEX_H
#define __COMPLEX_H

#include "global.h"

/* Exported types ----------------------------------------------------------*/
typedef struct
{
    float real;
    float img;
} Complex_t;

/* Exported functions ------------------------------------------------------*/
inline void Complex_Set(Complex_t *a, float real, float img)
{
    a->real = real;
    a->img = img;
}

inline float Complex_NormSqr(Complex_t *a)
{
    return (a->real * a->real + a->img * a->img);
}

inline void Complex_PolyCalc(float x, Complex_t *coeffs, Complex_t *result, uint8_t degree)
{
    result->real = 0.0f;
    result->img = 0.0f;

    for (int8_t i = degree; i >= 0; i--)
    {
        result->real = result->real * x + coeffs[i].real;
        result->img = result->img * x + coeffs[i].img;
    }
}

inline void Complex_Divide(Complex_t *dividend, Complex_t *divisor, Complex_t *result)
{
    float temp;
    temp = (dividend->real * divisor->real + dividend->img * divisor->img) /
           Complex_NormSqr(divisor);
    result->img = (dividend->img * divisor->real - dividend->real * divisor->img) /
                  Complex_NormSqr(divisor);
    result->real = temp;
}

inline void Complex_Multiply(Complex_t *a, Complex_t *b, Complex_t *result)
{
    float temp;

    temp = a->real * b->real - a->img * b->img;
    result->img = a->real * b->img + a->img * b->real;
    result->real = temp;
}

inline void Complex_MultiplyReal(Complex_t *a, float value, Complex_t *result)
{
    result->real = a->real * value;
    result->img = a->img * value;
}

inline void Complex_Conjugate(Complex_t *a, Complex_t *result)
{
    result->img = -a->img;
}

inline void Complex_Copy(Complex_t *src, Complex_t *dest)
{
    dest->real = src->real;
    dest->img = src->img;
}

inline void Complex_CopyArr(Complex_t *src, Complex_t *dest, uint16_t elements)
{
    for (uint16_t i = 0; i < elements; i++)
    {
        dest[i].real = src[i].real;
        dest[i].img = src[i].img;
    }
}

#endif