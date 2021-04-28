/*
 * File: kalmanfilter.h
 *
 * MATLAB Coder version            : 5.0
 * C/C++ source code generated on  : 27-Apr-2021 17:42:28
 */

#ifndef KALMANFILTER_H
#define KALMANFILTER_H

/* Include Files */
#include <stddef.h>
#include <stdlib.h>
#include "rtwtypes.h"
#include "kalmanfilter_types.h"

/* Function Declarations */
extern void kalmanfilter(const double z[2], double y[2]);
extern void kalmanfilter_init(void);

#endif

/*
 * File trailer for kalmanfilter.h
 *
 * [EOF]
 */
