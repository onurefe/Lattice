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
/***
 * @brief Sets a complex number.
 * 
 * @param a: Pointer to the complex number to be set.
 * @param real: Real part of the number.
 * @param img: Imaginary part of the number.
 */
inline void Complex_Set(Complex_t *a, float real, float img)
{
    a->real = real;
    a->img = img;
}

/***
 * @brief Takes the square norm of a complex number.
 * 
 * @param a: Complex number.
 * 
 * @retval |a|^2
 */
inline float Complex_NormSqr(Complex_t *a)
{
    return (a->real * a->real + a->img * a->img);
}

/***
 * @brief Calculates the value of a complex polynomial.
 * 
 * @param x: Input.
 * @param coeffs: Complex coefficients of the polynomial.
 * @param result: Pointer to return the result.
 * @param degree: Degree of the polynomial.
 */
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

/***
 * @brief Divides two complex numbers.
 * 
 * @param dividend: Pointer to the number to be divided.
 * @param divisor: Pointer to the dividing number.
 * @param result: Pointer to the result.
 */ 
inline void Complex_Divide(Complex_t *dividend, Complex_t *divisor, Complex_t *result)
{
    float temp;
    temp = (dividend->real * divisor->real + dividend->img * divisor->img) /
           Complex_NormSqr(divisor);
    result->img = (dividend->img * divisor->real - dividend->real * divisor->img) /
                  Complex_NormSqr(divisor);
    result->real = temp;
}

/***
 * @brief Multiplies two complex numbers. 
 * 
 * @param a: Pointer to the first complex number.
 * @param b: Pointer to the second complex number.
 * @param result: Pointer to the result.
 */
inline void Complex_Multiply(Complex_t *a, Complex_t *b, Complex_t *result)
{
    float temp;

    temp = a->real * b->real - a->img * b->img;
    result->img = a->real * b->img + a->img * b->real;
    result->real = temp;
}

/***
 * @brief Multiplies a complex number with a real number.
 * 
 * @param a: Pointer to the complex number to be multiplied.
 * @param value: Pointer to the real number to multiply.
 * @param result: Pointer to the result
 */
inline void Complex_MultiplyReal(Complex_t *a, float value, Complex_t *result)
{
    result->real = a->real * value;
    result->img = a->img * value;
}

/***
 * @brief Takes the complex conjugate of a complex number.
 * 
 * @param a: Pointer to the complex number to be conjugated.
 * @param result: Pointer to the result.
 */
inline void Complex_Conjugate(Complex_t *a, Complex_t *result)
{
    result->img = -a->img;
}

/***
 * @brief Copies a complex number.
 * 
 * @param src: Pointer to the source memory.
 * @param dest: Pointer to the destination memory.
 */
inline void Complex_Copy(Complex_t *src, Complex_t *dest)
{
    dest->real = src->real;
    dest->img = src->img;
}

/***
 * @brief Copies array of complex numbers.
 * 
 * @param src: Pointer to the source array memory.
 * @param dest: Pointer to the destination array memory.
 * @param elements: Number of elements(array length).
 */
inline void Complex_CopyArr(Complex_t *src, Complex_t *dest, uint16_t elements)
{
    for (uint16_t i = 0; i < elements; i++)
    {
        dest[i].real = src[i].real;
        dest[i].img = src[i].img;
    }
}

#endif