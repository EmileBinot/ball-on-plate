/*
 * File: kalmanfilter.c
 *
 * MATLAB Coder version            : 5.0
 * C/C++ source code generated on  : 27-Apr-2021 17:42:28
 */

/* Include Files */
#include "kalmanfilter.h"
#include "kalmanfilter_data.h"
#include "kalmanfilter_initialize.h"
#include <math.h>
#include <string.h>

/* Variable Definitions */
static double x_est[6];
static double p_est[36];

/* Function Definitions */

/*
 * Arguments    : const double z[2]
 *                double y[2]
 * Return Type  : void
 */
void kalmanfilter(const double z[2], double y[2])
{
  int i;
  signed char Q[36];
  int k;
  double x_prd[6];
  int i1;
  double d;
  int r1;
  double S[4];
  static const signed char a[36] = { 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 1,
    0, 0, 0, 0, 1, 0, 1, 0, 0, 0, 0, 1, 0, 1, 0, 0, 0, 0, 1, 0, 1 };

  int r2;
  double b_a[36];
  static const signed char iv[36] = { 1, 0, 1, 0, 0, 0, 0, 1, 0, 1, 0, 0, 0, 0,
    1, 0, 1, 0, 0, 0, 0, 1, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1 };

  double p_prd[36];
  double a21;
  double a22_tmp;
  double B[12];
  static const signed char c_a[12] = { 1, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0 };

  double a22;
  static const signed char iv1[12] = { 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0 };

  static const short R[4] = { 1000, 0, 0, 1000 };

  double Y[12];
  double b_z[2];
  if (!isInitialized_kalmanfilter) {
    kalmanfilter_initialize();
  }

  /*    Copyright 2010 The MathWorks, Inc. */
  /*  Initialize state transition matrix */
  /*      % [x  ] */
  /*      % [y  ] */
  /*      % [Vx] */
  /*      % [Vy] */
  /*      % [Ax] */
  /*  [Ay] */
  /*  Initialize measurement matrix */
  for (i = 0; i < 36; i++) {
    Q[i] = 0;
  }

  /*  Initial state conditions */
  /*  Predicted state and covariance */
  for (k = 0; k < 6; k++) {
    Q[k + 6 * k] = 1;
    x_prd[k] = 0.0;
    for (i = 0; i < 6; i++) {
      r1 = k + 6 * i;
      x_prd[k] += (double)a[r1] * x_est[i];
      d = 0.0;
      for (i1 = 0; i1 < 6; i1++) {
        d += (double)a[k + 6 * i1] * p_est[i1 + 6 * i];
      }

      b_a[r1] = d;
    }
  }

  for (i = 0; i < 6; i++) {
    for (i1 = 0; i1 < 6; i1++) {
      d = 0.0;
      for (r1 = 0; r1 < 6; r1++) {
        d += b_a[i + 6 * r1] * (double)iv[r1 + 6 * i1];
      }

      r1 = i + 6 * i1;
      p_prd[r1] = d + (double)Q[r1];
    }
  }

  /*  Estimation */
  for (i = 0; i < 2; i++) {
    for (i1 = 0; i1 < 6; i1++) {
      d = 0.0;
      for (r1 = 0; r1 < 6; r1++) {
        d += (double)c_a[i + (r1 << 1)] * p_prd[i1 + 6 * r1];
      }

      B[i + (i1 << 1)] = d;
    }

    for (i1 = 0; i1 < 2; i1++) {
      d = 0.0;
      for (r1 = 0; r1 < 6; r1++) {
        d += B[i + (r1 << 1)] * (double)iv1[r1 + 6 * i1];
      }

      r1 = i + (i1 << 1);
      S[r1] = d + (double)R[r1];
    }
  }

  if (fabs(S[1]) > fabs(S[0])) {
    r1 = 1;
    r2 = 0;
  } else {
    r1 = 0;
    r2 = 1;
  }

  a21 = S[r2] / S[r1];
  a22_tmp = S[r1 + 2];
  a22 = S[r2 + 2] - a21 * a22_tmp;
  for (k = 0; k < 6; k++) {
    i = k << 1;
    i1 = r1 + i;
    d = (B[r2 + i] - B[i1] * a21) / a22;
    Y[i + 1] = d;
    Y[i] = (B[i1] - d * a22_tmp) / S[r1];
  }

  for (i = 0; i < 2; i++) {
    for (i1 = 0; i1 < 6; i1++) {
      B[i1 + 6 * i] = Y[i + (i1 << 1)];
    }
  }

  /*  Estimated state and covariance */
  for (i = 0; i < 2; i++) {
    d = 0.0;
    for (i1 = 0; i1 < 6; i1++) {
      d += (double)c_a[i + (i1 << 1)] * x_prd[i1];
    }

    b_z[i] = z[i] - d;
  }

  for (i = 0; i < 6; i++) {
    d = B[i + 6];
    x_est[i] = x_prd[i] + (B[i] * b_z[0] + d * b_z[1]);
    for (i1 = 0; i1 < 6; i1++) {
      r1 = i1 << 1;
      b_a[i + 6 * i1] = B[i] * (double)c_a[r1] + d * (double)c_a[r1 + 1];
    }

    for (i1 = 0; i1 < 6; i1++) {
      d = 0.0;
      for (r1 = 0; r1 < 6; r1++) {
        d += b_a[i + 6 * r1] * p_prd[r1 + 6 * i1];
      }

      r1 = i + 6 * i1;
      p_est[r1] = p_prd[r1] - d;
    }
  }

  /*  Compute the estimated measurements */
  for (i = 0; i < 2; i++) {
    d = 0.0;
    for (i1 = 0; i1 < 6; i1++) {
      d += (double)c_a[i + (i1 << 1)] * x_est[i1];
    }

    y[i] = d;
  }

  /*  of the function */
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void kalmanfilter_init(void)
{
  int i;
  for (i = 0; i < 6; i++) {
    x_est[i] = 0.0;
  }

  /*  x_est=[x,y,Vx,Vy,Ax,Ay]' */
  memset(&p_est[0], 0, 36U * sizeof(double));
}

/*
 * File trailer for kalmanfilter.c
 *
 * [EOF]
 */
