
#include "MagnetoCalibration.h"
#include <math.h>
#include <string.h>

#define FLT_EPSILON 1.1920929e-07F  /* 1E-5 */
#define fabsf(a) (a>=0.0f)?(a):(-a)
//#define PX4_ISFINITE(x) 1
//#define sqrtf sqrt

int PX4_ISFINITE(float a)
{
    unsigned int b;

    memcpy((void*)&b, (void*)&a, 4);
    b &= 0x7fffffff;
    if (0x7f800000 == b)
    {
        return 0;  // infinite
    }
    else
    {
        return 1;  // finite
    }
}

int run_lm_sphere_fit(const float x[], const float y[], const float z[], float *_fitness, float *_sphere_lambda,
    int size, float *offset_x, float *offset_y, float *offset_z,
    float *sphere_radius, float *diag_x, float *diag_y, float *diag_z, float *offdiag_x, float *offdiag_y, float *offdiag_z);

int run_lm_ellipsoid_fit(const float x[], const float y[], const float z[], float *_fitness, float *_sphere_lambda,
    int size, float *offset_x, float *offset_y, float *offset_z,
    float *sphere_radius, float *diag_x, float *diag_y, float *diag_z, float *offdiag_x, float *offdiag_y, float *offdiag_z);

/*
*    Does matrix multiplication of two regular/square matrices
*
*    @param     A,           Matrix A
*    @param     B,           Matrix B
*    @param     n,           dimemsion of square matrices
*    @returns                multiplied matrix i.e. A*B
*/

void mat_mul(float *A, float *B, int n, float *C)
{
    int i, j, k;
    for (i = 0; i < n; i++) {
        for (j = 0; j < n; j++) {
            C[i * n + j] = 0.0f;
        }
    }
    for (i = 0; i < n; i++) {
        for (j = 0; j < n; j++) {
            for (k = 0; k < n; k++) {
                C[i * n + j] += A[i * n + k] * B[k * n + j];
            }
        }
    }
}

static void swap(float *a, float *b)
{
    float c;
    c = *a;
    *a = *b;
    *b = c;
}

/*
*    calculates pivot matrix such that all the larger elements in the row are on diagonal
*
*    @param     A,           input matrix matrix
*    @param     pivot
*    @param     n,           dimenstion of square matrix
*    @returns                false = matrix is Singular or non positive definite, true = matrix inversion successful
*/

static void mat_pivot(float *A, float *pivot, int n)
{
    int i, j, max_j, k;

    for (i = 0; i < n; i++) {
        for (j = 0; j < n; j++) {
            pivot[i * n + j] = (i == j);
        }
    }

    for (i = 0; i < n; i++) {
        max_j = i;

        for (j = i; j < n; j++) {
            if (fabsf(A[j * n + i]) > fabsf(A[max_j * n + i])) {
                max_j = j;
            }
        }

        if (max_j != i) {
            for (k = 0; k < n; k++) {
                swap(&pivot[i * n + k], &pivot[max_j * n + k]);
            }
        }
    }
}

/*
*    calculates matrix inverse of Lower trangular matrix using forward substitution
*
*    @param     L,           lower triangular matrix
*    @param     out,         Output inverted lower triangular matrix
*    @param     n,           dimension of matrix
*/

static void mat_forward_sub(float *L, float *out, int n)
{
    // Forward substitution solve LY = I
    int i, j, k;
    for (i = 0; i < n; i++) {
        out[i * n + i] = 1 / L[i * n + i];

        for (j = i + 1; j < n; j++) {
            for (k = i; k < j; k++) {
                out[j * n + i] -= L[j * n + k] * out[k * n + i];
            }

            out[j * n + i] /= L[j * n + j];
        }
    }
}

/*
*    calculates matrix inverse of Upper trangular matrix using backward substitution
*
*    @param     U,           upper triangular matrix
*    @param     out,         Output inverted upper triangular matrix
*    @param     n,           dimension of matrix
*/

static void mat_back_sub(float *U, float *out, int n)
{
    // Backward Substitution solve UY = I
    int i, j, k;
    for (i = n - 1; i >= 0; i--) {
        out[i * n + i] = 1 / U[i * n + i];

        for (j = i - 1; j >= 0; j--) {
            for (k = i; k > j; k--) {
                out[j * n + i] -= U[j * n + k] * out[k * n + i];
            }

            out[j * n + i] /= U[j * n + j];
        }
    }
}

/*
*    Decomposes square matrix into Lower and Upper triangular matrices such that
*    A*P = L*U, where P is the pivot matrix
*    ref: http://rosettacode.org/wiki/LU_decomposition
*    @param     U,           upper triangular matrix
*    @param     out,         Output inverted upper triangular matrix
*    @param     n,           dimension of matrix
*/

static void mat_LU_decompose(float *A, float *L, float *U, float *P, int n)
{
    float APrime[9 * 9];
    int i, j, k;

    if (n > 9)
    {
        return;
    }
    for (i = 0; i < n; i++) {
        for (j = 0; j < n; j++) {
            L[j * n + i] = 0.0f;
            U[j * n + i] = 0.0f;
            P[j * n + i] = 0.0f;
        }
    }
    mat_pivot(A, P, n);

    mat_mul(P, A, n, APrime);

    for (i = 0; i < n; i++) {
        L[i * n + i] = 1;
    }

    for (i = 0; i < n; i++) {
        for (j = 0; j < n; j++) {
            if (j <= i) {
                U[j * n + i] = APrime[j * n + i];

                for (k = 0; k < j; k++) {
                    U[j * n + i] -= L[j * n + k] * U[k * n + i];
                }
            }

            if (j >= i) {
                L[j * n + i] = APrime[j * n + i];

                for (k = 0; k < i; k++) {
                    L[j * n + i] -= L[j * n + k] * U[k * n + i];
                }

                L[j * n + i] /= U[i * n + i];
            }
        }
    }
}

/*
*    matrix inverse code for any square matrix using LU decomposition
*    inv = inv(U)*inv(L)*P, where L and U are triagular matrices and P the pivot matrix
*    ref: http://www.cl.cam.ac.uk/teaching/1314/NumMethods/supporting/mcmaster-kiruba-ludecomp.pdf
*    @param     m,           input 4x4 matrix
*    @param     inv,      Output inverted 4x4 matrix
*    @param     n,           dimension of square matrix
*    @returns                false = matrix is Singular, true = matrix inversion successful
*/
int mat_inverse(float *A, float *inv, int n)
{
    float L[9 * 9], U[9 * 9], P[9 * 9];
    int ret = 1;
    int i, j;
    float L_inv[9 * 9];
    float U_inv[9 * 9];
    float inv_unpivoted[9 * 9];
    float inv_pivoted[9 * 9];

    if (n > 9)
    {
        return 0;
    }
    mat_LU_decompose(A, L, U, P, n);


    for (i = 0; i < n; i++) {
        for (j = 0; j < n; j++) {
            L_inv[j * n + i] = 0.0f;
        }
    }
    mat_forward_sub(L, L_inv, n);

    for (i = 0; i < n; i++) {
        for (j = 0; j < n; j++) {
            U_inv[j * n + i] = 0.0f;
        }
    }
    mat_back_sub(U, U_inv, n);


    mat_mul(U_inv, L_inv, n, inv_unpivoted);
    mat_mul(inv_unpivoted, P, n, inv_pivoted);

    //check sanity of results
    for (i = 0; i < n; i++) {
        for (j = 0; j < n; j++) {
            if (!PX4_ISFINITE(inv_pivoted[i * n + j])) {
                ret = 0;
            }
        }
    }

    for (i = 0; i < n; i++) {
        for (j = 0; j < n; j++) {
            inv[j * n + i] = inv_pivoted[j * n + i];
        }
    }
    return ret;
}

/* The difference between 1 and the least value greater than 1 that is
* representable in the given floating-point type, b1-p.
*/


int sphere_fit_least_squares(const float x[], const float y[], const float z[],
    int size, int max_iterations, float delta, float *sphere_x, float *sphere_y, float *sphere_z,
    float *sphere_radius)
{

    float x_sumplain = 0.0f;
    float x_sumsq = 0.0f;
    float x_sumcube = 0.0f;

    float y_sumplain = 0.0f;
    float y_sumsq = 0.0f;
    float y_sumcube = 0.0f;

    float z_sumplain = 0.0f;
    float z_sumsq = 0.0f;
    float z_sumcube = 0.0f;

    float xy_sum = 0.0f;
    float xz_sum = 0.0f;
    float yz_sum = 0.0f;

    float x2y_sum = 0.0f;
    float x2z_sum = 0.0f;
    float y2x_sum = 0.0f;
    float y2z_sum = 0.0f;
    float z2x_sum = 0.0f;
    float z2y_sum = 0.0f;
    int n = 0, i;
    float x2, y2, z2;
    float x_sum;        //sum( X[n] )
    float x_sum2;    //sum( X[n]^2 )
    float x_sum3;    //sum( X[n]^3 )
    float y_sum;        //sum( Y[n] )
    float y_sum2;    //sum( Y[n]^2 )
    float y_sum3;    //sum( Y[n]^3 )
    float z_sum;        //sum( Z[n] )
    float z_sum2;    //sum( Z[n]^2 )
    float z_sum3;    //sum( Z[n]^3 )

    float XY;        //sum( X[n] * Y[n] )
    float XZ;        //sum( X[n] * Z[n] )
    float YZ;        //sum( Y[n] * Z[n] )
    float X2Y;    //sum( X[n]^2 * Y[n] )
    float X2Z;    //sum( X[n]^2 * Z[n] )
    float Y2X;    //sum( Y[n]^2 * X[n] )
    float Y2Z;    //sum( Y[n]^2 * Z[n] )
    float Z2X;    //sum( Z[n]^2 * X[n] )
    float Z2Y;    //sum( Z[n]^2 * Y[n] )

                                   //Reduction of multiplications
    float F0, F1, F2, F3, F4;

    //Set initial conditions:
    float A, B, C;

    //First iteration computation:
    float A2, B2, C2, QS, QB;

    //Set initial conditions:
    float Rsq;

    //First iteration computation:
    float Q0,Q1, Q2;
    float aA, aB, aC, nA, nB, nC, dA, dB, dC;

    for (i = 0; i < size; i++) {

        x2 = x[i] * x[i];
        y2 = y[i] * y[i];
        z2 = z[i] * z[i];

        x_sumplain += x[i];
        x_sumsq += x2;
        x_sumcube += x2 * x[i];

        y_sumplain += y[i];
        y_sumsq += y2;
        y_sumcube += y2 * y[i];

        z_sumplain += z[i];
        z_sumsq += z2;
        z_sumcube += z2 * z[i];

        xy_sum += x[i] * y[i];
        xz_sum += x[i] * z[i];
        yz_sum += y[i] * z[i];

        x2y_sum += x2 * y[i];
        x2z_sum += x2 * z[i];

        y2x_sum += y2 * x[i];
        y2z_sum += y2 * z[i];

        z2x_sum += z2 * x[i];
        z2y_sum += z2 * y[i];
    }

    //
    //Least Squares Fit a sphere A,B,C with radius squared Rsq to 3D data
    //
    //    P is a structure that has been computed with the data earlier.
    //    P.npoints is the number of elements; the length of X,Y,Z are identical.
    //    P's members are logically named.
    //
    //    X[n] is the x component of point n
    //    Y[n] is the y component of point n
    //    Z[n] is the z component of point n
    //
    //    A is the x coordiante of the sphere
    //    B is the y coordiante of the sphere
    //    C is the z coordiante of the sphere
    //    Rsq is the radius squared of the sphere.
    //
    //This method should converge; maybe 5-100 iterations or more.
    //
    x_sum = x_sumplain / size;        //sum( X[n] )
    x_sum2 = x_sumsq / size;    //sum( X[n]^2 )
    x_sum3 = x_sumcube / size;    //sum( X[n]^3 )
    y_sum = y_sumplain / size;        //sum( Y[n] )
    y_sum2 = y_sumsq / size;    //sum( Y[n]^2 )
    y_sum3 = y_sumcube / size;    //sum( Y[n]^3 )
    z_sum = z_sumplain / size;        //sum( Z[n] )
    z_sum2 = z_sumsq / size;    //sum( Z[n]^2 )
    z_sum3 = z_sumcube / size;    //sum( Z[n]^3 )

    XY = xy_sum / size;        //sum( X[n] * Y[n] )
    XZ = xz_sum / size;        //sum( X[n] * Z[n] )
    YZ = yz_sum / size;        //sum( Y[n] * Z[n] )
    X2Y = x2y_sum / size;    //sum( X[n]^2 * Y[n] )
    X2Z = x2z_sum / size;    //sum( X[n]^2 * Z[n] )
    Y2X = y2x_sum / size;    //sum( Y[n]^2 * X[n] )
    Y2Z = y2z_sum / size;    //sum( Y[n]^2 * Z[n] )
    Z2X = z2x_sum / size;    //sum( Z[n]^2 * X[n] )
    Z2Y = z2y_sum / size;    //sum( Z[n]^2 * Y[n] )

                                  //Reduction of multiplications
    F0 = x_sum2 + y_sum2 + z_sum2;
    F1 = 0.5f * F0;
    F2 = -8.0f * (x_sum3 + Y2X + Z2X);
    F3 = -8.0f * (X2Y + y_sum3 + Z2Y);
    F4 = -8.0f * (X2Z + Y2Z + z_sum3);

    //Set initial conditions:
    A = x_sum;
    B = y_sum;
    C = z_sum;

    //First iteration computation:
    A2 = A * A;
    B2 = B * B;
    C2 = C * C;
    QS = A2 + B2 + C2;
    QB = -2.0f * (A * x_sum + B * y_sum + C * z_sum);

    //Set initial conditions:
    Rsq = F0 + QB + QS;

    //First iteration computation:
    Q0 = 0.5f * (QS - Rsq);
    Q1 = F1 + Q0;
    Q2 = 8.0f * (QS - Rsq + QB + F0);

    //Iterate N times, ignore stop condition.
    n = 0;

    while (n < max_iterations) {
        n++;

        //Compute denominator:
        aA = Q2 + 16.0f * (A2 - 2.0f * A * x_sum + x_sum2);
        aB = Q2 + 16.0f * (B2 - 2.0f * B * y_sum + y_sum2);
        aC = Q2 + 16.0f * (C2 - 2.0f * C * z_sum + z_sum2);
        aA = (fabsf(aA) < FLT_EPSILON) ? 1.0f : aA;
        aB = (fabsf(aB) < FLT_EPSILON) ? 1.0f : aB;
        aC = (fabsf(aC) < FLT_EPSILON) ? 1.0f : aC;

        //Compute next iteration
        nA = A - ((F2 + 16.0f * (B * XY + C * XZ + x_sum * (-A2 - Q0) + A * (x_sum2 + Q1 - C * z_sum - B * y_sum))) / aA);
        nB = B - ((F3 + 16.0f * (A * XY + C * YZ + y_sum * (-B2 - Q0) + B * (y_sum2 + Q1 - A * x_sum - C * z_sum))) / aB);
        nC = C - ((F4 + 16.0f * (A * XZ + B * YZ + z_sum * (-C2 - Q0) + C * (z_sum2 + Q1 - A * x_sum - B * y_sum))) / aC);

        //Check for stop condition
        dA = (nA - A);
        dB = (nB - B);
        dC = (nC - C);

        if ((dA * dA + dB * dB + dC * dC) <= delta) { break; }

        //Compute next iteration's values
        A = nA;
        B = nB;
        C = nC;
        A2 = A * A;
        B2 = B * B;
        C2 = C * C;
        QS = A2 + B2 + C2;
        QB = -2.0f * (A * x_sum + B * y_sum + C * z_sum);
        Rsq = F0 + QB + QS;
        Q0 = 0.5f * (QS - Rsq);
        Q1 = F1 + Q0;
        Q2 = 8.0f * (QS - Rsq + QB + F0);
    }

    *sphere_x = A;
    *sphere_y = B;
    *sphere_z = C;
    *sphere_radius = sqrtf(Rsq);

    return 0;
}


int run_lm_sphere_fit(const float x[], const float y[], const float z[], float *_fitness, float *_sphere_lambda,
    int size, float *offset_x, float *offset_y, float *offset_z,
    float *sphere_radius, float *diag_x, float *diag_y, float *diag_z, float *offdiag_x, float *offdiag_y, float *offdiag_z)
{
    //Run Sphere Fit using Levenberg Marquardt LSq Fit
    const float lma_damping = 10.0f;
    int _samples_collected = size;
    float fitness = *_fitness;
    float fit1 = 0.0f, fit2 = 0.0f;

    float JTJ[4][4];
    float JTJ2[4][4];
    float JTFI[4] = { 0.0f,0.0f,0.0f,0.0f };
    float residual = 0.0f;
    int i, j, k, row, col;
    float A, B, C, length, sphere_jacob[4];
    float fit1_params[4];
    float fit2_params[4];

    // Gauss Newton Part common for all kind of extensions including LM
    for (k = 0; k < _samples_collected; k++) {

        //Calculate Jacobian
        A = (*diag_x    * (x[k] - *offset_x)) + (*offdiag_x * (y[k] - *offset_y)) + (*offdiag_y * (z[k] - *offset_z));
        B = (*offdiag_x * (x[k] - *offset_x)) + (*diag_y    * (y[k] - *offset_y)) + (*offdiag_z * (z[k] - *offset_z));
        C = (*offdiag_y * (x[k] - *offset_x)) + (*offdiag_z * (y[k] - *offset_y)) + (*diag_z    * (z[k] - *offset_z));
        length = sqrtf(A * A + B * B + C * C);

        // 0: partial derivative (radius wrt fitness fn) fn operated on sample
        sphere_jacob[0] = 1.0f;
        // 1-3: partial derivative (offsets wrt fitness fn) fn operated on sample
        sphere_jacob[1] = 1.0f * (((*diag_x    * A) + (*offdiag_x * B) + (*offdiag_y * C)) / length);
        sphere_jacob[2] = 1.0f * (((*offdiag_x * A) + (*diag_y    * B) + (*offdiag_z * C)) / length);
        sphere_jacob[3] = 1.0f * (((*offdiag_y * A) + (*offdiag_z * B) + (*diag_z    * C)) / length);
        residual = *sphere_radius - length;

        for (i = 0; i < 4; i++) {
            // compute JTJ
            for (j = 0; j < 4; j++) {
                JTJ[i][j] += sphere_jacob[i] * sphere_jacob[j];
                JTJ2[i][j] += sphere_jacob[i] * sphere_jacob[j]; //a backup JTJ for LM
            }

            JTFI[i] += sphere_jacob[i] * residual;
        }
    }


    //------------------------Levenberg-Marquardt-part-starts-here---------------------------------//
    //refer: http://en.wikipedia.org/wiki/Levenberg%E2%80%93Marquardt_algorithm#Choice_of_damping_parameter
    fit1_params[0] = *sphere_radius;
    fit1_params[1] = *offset_x;
    fit1_params[2] = *offset_y;
    fit1_params[3] = *offset_z;
    for (i = 0; i < 4; i++) {
        fit2_params[i] = fit1_params[i];
    }

    for (i = 0; i < 4; i++) {
        JTJ[i][i] += *_sphere_lambda;
        JTJ2[i][i] += *_sphere_lambda / lma_damping;
    }

    if (!mat_inverse(&JTJ[0][0], &JTJ[0][0], 4)) {
        return -1;
    }

    if (!mat_inverse(&JTJ2[0][0], &JTJ2[0][0], 4)) {
        return -1;
    }

    for (row = 0; row < 4; row++) {
        for (col = 0; col < 4; col++) {
            fit1_params[row] -= JTFI[col] * JTJ[row][col];
            fit2_params[row] -= JTFI[col] * JTJ2[row][col];
        }
    }

    //Calculate mean squared residuals
    for (k = 0; k < _samples_collected; k++) {
        A = (*diag_x    * (x[k] - fit1_params[1])) + (*offdiag_x * (y[k] - fit1_params[2])) + (*offdiag_y *
            (z[k] + fit1_params[3]));
        B = (*offdiag_x * (x[k] - fit1_params[1])) + (*diag_y    * (y[k] - fit1_params[2])) + (*offdiag_z *
            (z[k] + fit1_params[3]));
        C = (*offdiag_y * (x[k] - fit1_params[1])) + (*offdiag_z * (y[k] - fit1_params[2])) + (*diag_z    *
            (z[k] - fit1_params[3]));
        length = sqrtf(A * A + B * B + C * C);
        residual = fit1_params[0] - length;
        fit1 += residual * residual;

        A = (*diag_x    * (x[k] - fit2_params[1])) + (*offdiag_x * (y[k] - fit2_params[2])) + (*offdiag_y *
            (z[k] - fit2_params[3]));
        B = (*offdiag_x * (x[k] - fit2_params[1])) + (*diag_y    * (y[k] - fit2_params[2])) + (*offdiag_z *
            (z[k] - fit2_params[3]));
        C = (*offdiag_y * (x[k] - fit2_params[1])) + (*offdiag_z * (y[k] - fit2_params[2])) + (*diag_z    *
            (z[k] - fit2_params[3]));
        length = sqrtf(A * A + B * B + C * C);
        residual = fit2_params[0] - length;
        fit2 += residual * residual;
    }

    fit1 = sqrtf(fit1) / _samples_collected;
    fit2 = sqrtf(fit2) / _samples_collected;

    if (fit1 > *_fitness && fit2 > *_fitness) {
        *_sphere_lambda *= lma_damping;

    }
    else if (fit2 < *_fitness && fit2 < fit1) {
        *_sphere_lambda /= lma_damping;
        for (i = 0; i < 4; i++) {
            fit1_params[i] = fit2_params[i];
        }
        fitness = fit2;

    }
    else if (fit1 < *_fitness) {
        fitness = fit1;
    }

    //--------------------Levenberg-Marquardt-part-ends-here--------------------------------//

    if (PX4_ISFINITE(fitness) && fitness < *_fitness) {
        *_fitness = fitness;
        *sphere_radius = fit1_params[0];
        *offset_x = fit1_params[1];
        *offset_y = fit1_params[2];
        *offset_z = fit1_params[3];
        return 0;
    }
    else {
        return -1;
    }
}

int run_lm_ellipsoid_fit(const float x[], const float y[], const float z[], float *_fitness, float *_sphere_lambda,
    int size, float *offset_x, float *offset_y, float *offset_z,
    float *sphere_radius, float *diag_x, float *diag_y, float *diag_z, float *offdiag_x, float *offdiag_y, float *offdiag_z)
{
    //Run Sphere Fit using Levenberg Marquardt LSq Fit
    const float lma_damping = 10.0f;
    int _samples_collected = size;
    float fitness = *_fitness;
    float fit1 = 0.0f;
    float fit2 = 0.0f;

    float JTJ[81];
    float JTJ2[81];
    float JTFI[9];
    float residual = 0.0f;
    float ellipsoid_jacob[9];
    float A, B, C, length;
    int i, j, k, row, col;
    float fit1_params[9];
    float fit2_params[9];

    for (i = 0; i < 81; i++)
    {
        JTJ[i] = 0.0f;
        JTJ2[i] = 0.0f;
    }
    for (i = 0; i < 9; i++)
    {
        JTFI[i] = 0.0f;
    }


    // Gauss Newton Part common for all kind of extensions including LM
    for (k = 0; k < _samples_collected; k++) {

        //Calculate Jacobian
        A = (*diag_x    * (x[k] - *offset_x)) + (*offdiag_x * (y[k] - *offset_y)) + (*offdiag_y * (z[k] - *offset_z));
        B = (*offdiag_x * (x[k] - *offset_x)) + (*diag_y    * (y[k] - *offset_y)) + (*offdiag_z * (z[k] - *offset_z));
        C = (*offdiag_y * (x[k] - *offset_x)) + (*offdiag_z * (y[k] - *offset_y)) + (*diag_z    * (z[k] - *offset_z));
        length = sqrtf(A * A + B * B + C * C);
        residual = *sphere_radius - length;
        fit1 += residual * residual;
        // 0-2: partial derivative (offset wrt fitness fn) fn operated on sample
        ellipsoid_jacob[0] = 1.0f * (((*diag_x    * A) + (*offdiag_x * B) + (*offdiag_y * C)) / length);
        ellipsoid_jacob[1] = 1.0f * (((*offdiag_x * A) + (*diag_y    * B) + (*offdiag_z * C)) / length);
        ellipsoid_jacob[2] = 1.0f * (((*offdiag_y * A) + (*offdiag_z * B) + (*diag_z    * C)) / length);
        // 3-5: partial derivative (diag offset wrt fitness fn) fn operated on sample
        ellipsoid_jacob[3] = -1.0f * ((x[k] + *offset_x) * A) / length;
        ellipsoid_jacob[4] = -1.0f * ((y[k] + *offset_y) * B) / length;
        ellipsoid_jacob[5] = -1.0f * ((z[k] + *offset_z) * C) / length;
        // 6-8: partial derivative (off-diag offset wrt fitness fn) fn operated on sample
        ellipsoid_jacob[6] = -1.0f * (((y[k] + *offset_y) * A) + ((x[k] + *offset_x) * B)) / length;
        ellipsoid_jacob[7] = -1.0f * (((z[k] + *offset_z) * A) + ((x[k] + *offset_x) * C)) / length;
        ellipsoid_jacob[8] = -1.0f * (((z[k] + *offset_z) * B) + ((y[k] + *offset_y) * C)) / length;

        for (i = 0; i < 9; i++) {
            // compute JTJ
            for (j = 0; j < 9; j++) {
                JTJ[i * 9 + j] += ellipsoid_jacob[i] * ellipsoid_jacob[j];
                JTJ2[i * 9 + j] += ellipsoid_jacob[i] * ellipsoid_jacob[j]; //a backup JTJ for LM
            }

            JTFI[i] += ellipsoid_jacob[i] * residual;
        }
    }


    //------------------------Levenberg-Marquardt-part-starts-here---------------------------------//
    //refer: http://en.wikipedia.org/wiki/Levenberg%E2%80%93Marquardt_algorithm#Choice_of_damping_parameter
    fit1_params[0] = *offset_x;
    fit1_params[1] = *offset_y;
    fit1_params[2] = *offset_z;
    fit1_params[3] = *diag_x;
    fit1_params[4] = *diag_y;
    fit1_params[5] = *diag_z;
    fit1_params[6] = *offdiag_x;
    fit1_params[7] = *offdiag_y;
    fit1_params[8] = *offdiag_z;

    for (i = 0; i < 9; i++) {
        fit2_params[i] = fit1_params[i];
    }

    for (i = 0; i < 9; i++) {
        JTJ[i * 9 + i] += *_sphere_lambda;
        JTJ2[i * 9 + i] += *_sphere_lambda / lma_damping;
    }


    if (!mat_inverse(JTJ, JTJ, 9)) {
        return -1;
    }

    if (!mat_inverse(JTJ2, JTJ2, 9)) {
        return -1;
    }

    for (row = 0; row < 9; row++) {
        for (col = 0; col < 9; col++) {
            fit1_params[row] -= JTFI[col] * JTJ[row * 9 + col];
            fit2_params[row] -= JTFI[col] * JTJ2[row * 9 + col];
        }
    }

    //Calculate mean squared residuals
    for (k = 0; k < _samples_collected; k++) {
        A = (fit1_params[3] * (x[k] - fit1_params[0])) + (fit1_params[6] * (y[k] - fit1_params[1])) + (fit1_params[7] *
            (z[k] - fit1_params[2]));
        B = (fit1_params[6] * (x[k] - fit1_params[0])) + (fit1_params[4] * (y[k] - fit1_params[1])) + (fit1_params[8] *
            (z[k] - fit1_params[2]));
        C = (fit1_params[7] * (x[k] - fit1_params[0])) + (fit1_params[8] * (y[k] - fit1_params[1])) + (fit1_params[5] *
            (z[k] - fit1_params[2]));
        length = sqrtf(A * A + B * B + C * C);
        residual = *sphere_radius - length;
        fit1 += residual * residual;

        A = (fit2_params[3] * (x[k] - fit2_params[0])) + (fit2_params[6] * (y[k] - fit2_params[1])) + (fit2_params[7] *
            (z[k] - fit2_params[2]));
        B = (fit2_params[6] * (x[k] - fit2_params[0])) + (fit2_params[4] * (y[k] - fit2_params[1])) + (fit2_params[8] *
            (z[k] - fit2_params[2]));
        C = (fit2_params[7] * (x[k] - fit2_params[0])) + (fit2_params[8] * (y[k] - fit2_params[1])) + (fit2_params[5] *
            (z[k] - fit2_params[2]));
        length = sqrtf(A * A + B * B + C * C);
        residual = *sphere_radius - length;
        fit2 += residual * residual;
    }

    fit1 = sqrtf(fit1) / _samples_collected;
    fit2 = sqrtf(fit2) / _samples_collected;

    if (fit1 > *_fitness && fit2 > *_fitness) {
        *_sphere_lambda *= lma_damping;

    }
    else if (fit2 < *_fitness && fit2 < fit1) {
        *_sphere_lambda /= lma_damping;
        for (i = 0; i < 9; i++) {
            fit1_params[i] = fit2_params[i];
        }
        fitness = fit2;

    }
    else if (fit1 < *_fitness) {
        fitness = fit1;
    }

    //--------------------Levenberg-Marquardt-part-ends-here--------------------------------//
    if (PX4_ISFINITE(fitness) && fitness < *_fitness) {
        *_fitness = fitness;
        *offset_x = fit1_params[0];
        *offset_y = fit1_params[1];
        *offset_z = fit1_params[2];
        *diag_x = fit1_params[3];
        *diag_y = fit1_params[4];
        *diag_z = fit1_params[5];
        *offdiag_x = fit1_params[6];
        *offdiag_y = fit1_params[7];
        *offdiag_z = fit1_params[8];
        return 0;

    }
    else {
        return -1;
    }
}

// Levenberg-Marquardt
int ellipsoid_fit_least_squares(const float x[], const float y[], const float z[],
    int size, int max_iterations, float delta, float *offset_x, float *offset_y, float *offset_z,
    float *sphere_radius, float *diag_x, float *diag_y, float *diag_z, float *offdiag_x, float *offdiag_y, float *offdiag_z)
{
    float _fitness = 1.0e30f, _sphere_lambda = 1.0f, _ellipsoid_lambda = 1.0f;
    int i;

    for (i = 0; i < max_iterations; i++) {
        run_lm_sphere_fit(x, y, z, &_fitness, &_sphere_lambda,
            size, offset_x, offset_y, offset_z,
            sphere_radius, diag_x, diag_y, diag_z, offdiag_x, offdiag_y, offdiag_z);
    }

    _fitness = 1.0e30f;

    for (i = 0; i < max_iterations; i++) {
        run_lm_ellipsoid_fit(x, y, z, &_fitness, &_ellipsoid_lambda,
            size, offset_x, offset_y, offset_z,
            sphere_radius, diag_x, diag_y, diag_z, offdiag_x, offdiag_y, offdiag_z);
    }

    return 0;
}


static float MAG_MAX_OFFSET_LEN = 1.3f;   //< The maximum measurement range is ~1.9 Ga, the earth field is ~0.6 Ga, so an offset larger than ~1.3 Ga means the mag will saturate in some directions.

// Returns calibrate_return_error if any parameter is not finite
// Logs if parameters are out of range
calibrate_return check_calibration_result(float offset_x, float offset_y, float offset_z,
    float sphere_radius,
    float diag_x, float diag_y, float diag_z,
    float offdiag_x, float offdiag_y, float offdiag_z)
{
    float must_be_finite[10];
    float should_be_not_huge[3];
    float should_be_positive[4];
    int i, num_finite, num_not_huge, num_positive;

    must_be_finite[0] = offset_x;
    must_be_finite[1] = offset_y;
    must_be_finite[2] = offset_z;
    must_be_finite[3] = sphere_radius;
    must_be_finite[4] = diag_x;
    must_be_finite[5] = diag_y;
    must_be_finite[6] = diag_z;
    must_be_finite[7] = offdiag_x;
    must_be_finite[8] = offdiag_y;
    must_be_finite[9] = offdiag_z;

    should_be_not_huge[0] = offset_x;
    should_be_not_huge[1] = offset_y;
    should_be_not_huge[2] = offset_z;

    should_be_positive[0] = sphere_radius;
    should_be_positive[1] = diag_x;
    should_be_positive[2] = diag_y;
    should_be_positive[3] = diag_z;

    // Make sure every parameter is finite
    num_finite = sizeof(must_be_finite) / sizeof(*must_be_finite);
    for (i = 0; i < num_finite; ++i) {
        if (!PX4_ISFINITE(must_be_finite[i])) {
            return calibrate_return_error;
        }
    }

    // Notify if offsets are too large
    num_not_huge = sizeof(should_be_not_huge) / sizeof(*should_be_not_huge);
    for (i = 0; i < num_not_huge; ++i) {
        if (fabsf(should_be_not_huge[i]) > MAG_MAX_OFFSET_LEN) {
            break;
        }
    }

    // Notify if a parameter which should be positive is non-positive
    num_positive = sizeof(should_be_positive) / sizeof(*should_be_positive);
    for (i = 0; i < num_positive; ++i) {
        if (should_be_positive[i] <= 0.0f) {
            break;
        }
    }

    return calibrate_return_ok;
}


