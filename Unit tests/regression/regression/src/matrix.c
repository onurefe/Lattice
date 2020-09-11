#include "../inc/matrix.h"

#define SIZE MAX_SQUARE_MATRIX_SIZE

/* Exported functions ------------------------------------------------------*/
/***
 * @brief Multiplies two matrices and puts the result in R.
 * 
 * @param A: Pointer to the first matrix.
 * @param B: Pointer to the second matrix.
 * @param R: Pointer to resultant matrix.
 * 
 * @retval: Operation result. TRUE or FALSE(for invalid operation).
 */
Bool_t Matrix_Multiply(Matrix_t *A, Matrix_t *B, Matrix_t *R)
{
    uint16_t i, j, k;

    if (A->n == B->m)
    {
        R->m = A->m;
        R->n = B->n;
    }
    else
    {
        return FALSE;
    }

    for (i = 0; i < R->m; i++)
    {
        for (j = 0; j < R->n; j++)
        {
            R->arr[R->n * i + j] = 0;

            for (k = 0; k < A->n; k++)
            {
                R->arr[R->n * i + j] += (A->arr[A->n * i + k] * B->arr[B->n * k + j]);
            }
        }
    }

    return TRUE;
}

/***
 * @brief Transposes given matrix.
 * 
 * @param A: Input matrix.
 * @param R: Output matrix.
 */
void Matrix_Transpose(Matrix_t *A, Matrix_t *R)
{
    float temp;
    uint16_t i, j;

    for (i = 0; i < A->m; i++)
    {
        if (A->arr != R->arr)
        {
            for (j = 0; j < A->n; j++)
            {
                R->arr[A->m * j + i] = A->arr[A->n * i + j];
            }
        }
        else
        {
            for (j = i + 1; j < A->n; j++)
            {
                temp = R->arr[A->m * j + i];
                R->arr[A->m * j + i] = A->arr[A->n * i + j];
                A->arr[A->n * i + j] = temp;
            }
        }
    }

    R->m = A->n;
    R->n = A->m;
}

/***
 * @brief Takes the multiplicative inverse of a square matrix.
 * 
 * @param A: Matrix to be inverted.
 * @param R: Resultant matrix.
 * 
 * @retval TRUE or FALSE.
 */
Bool_t Matrix_Inverse(Matrix_t *A, Matrix_t *R)
{
    float ratio;
    uint16_t i, j, k, n;
    float W[SIZE * SIZE];

    // Ensure that the given matrix is square matrix.
    if (A->m != A->n)
    {
        return FALSE;
    }
    else
    {
        n = A->m;
    }

    /* Copying the input to store and creating Augmenting Identity Matrix of Order n */
    for (i = 0; i < n; i++)
    {
        for (j = 0; j < n; j++)
        {
            W[n * i + j] = A->arr[n * i + j];

            if (i == j)
            {
                R->arr[n * i + j] = 1;
            }
            else
            {
                R->arr[n * i + j] = 0;
            }
        }
    }

    /* Applying Gauss Jordan Elimination */
    for (i = 0; i < n; i++)
    {
        if (W[n * i + i] == 0.0)
        {
            W[n * i + i] = __FLT_MIN__;
        }

        for (j = 0; j < n; j++)
        {
            if (i != j)
            {
                ratio = W[n * j + i] / W[n * i + i];
                for (k = 0; k < n; k++)
                {
                    W[n * j + k] = W[n * j + k] - ratio * W[n * i + k];
                    R->arr[n * j + k] = R->arr[n * j + k] - ratio * R->arr[n * i + k];
                }
            }
        }
    }

    /* Row Operation to Make Principal Diagonal to 1 */
    for (i = 0; i < n; i++)
    {
        for (j = 0; j < n; j++)
        {
            R->arr[n * i + j] = R->arr[n * i + j] / W[n * i + i];
        }
    }

    return TRUE;
}