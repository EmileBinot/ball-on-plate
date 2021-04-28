/*
 * File: _coder_kalmanfilter_api.h
 *
 * MATLAB Coder version            : 5.0
 * C/C++ source code generated on  : 27-Apr-2021 17:42:28
 */

#ifndef _CODER_KALMANFILTER_API_H
#define _CODER_KALMANFILTER_API_H

/* Include Files */
#include <stddef.h>
#include <stdlib.h>
#include "tmwtypes.h"
#include "mex.h"
#include "emlrt.h"

/* Variable Declarations */
extern emlrtCTX emlrtRootTLSGlobal;
extern emlrtContext emlrtContextGlobal;

/* Function Declarations */
extern void kalmanfilter(real_T z[2], real_T y[2]);
extern void kalmanfilter_api(const mxArray * const prhs[1], int32_T nlhs, const
  mxArray *plhs[1]);
extern void kalmanfilter_atexit(void);
extern void kalmanfilter_initialize(void);
extern void kalmanfilter_terminate(void);
extern void kalmanfilter_xil_shutdown(void);
extern void kalmanfilter_xil_terminate(void);

#endif

/*
 * File trailer for _coder_kalmanfilter_api.h
 *
 * [EOF]
 */
