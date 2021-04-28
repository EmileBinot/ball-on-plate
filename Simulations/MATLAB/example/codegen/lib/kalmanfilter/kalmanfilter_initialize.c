/*
 * File: kalmanfilter_initialize.c
 *
 * MATLAB Coder version            : 5.0
 * C/C++ source code generated on  : 27-Apr-2021 17:42:28
 */

/* Include Files */
#include "kalmanfilter_initialize.h"
#include "kalmanfilter.h"
#include "kalmanfilter_data.h"

/* Function Definitions */

/*
 * Arguments    : void
 * Return Type  : void
 */
void kalmanfilter_initialize(void)
{
  kalmanfilter_init();
  isInitialized_kalmanfilter = true;
}

/*
 * File trailer for kalmanfilter_initialize.c
 *
 * [EOF]
 */
