/*
 * File: main.c
 *
 * MATLAB Coder version            : 5.0
 * C/C++ source code generated on  : 27-Apr-2021 17:42:28
 */

/*************************************************************************/
/* This automatically generated example C main file shows how to call    */
/* entry-point functions that MATLAB Coder generated. You must customize */
/* this file for your application. Do not modify this file directly.     */
/* Instead, make a copy of this file, modify it, and integrate it into   */
/* your development environment.                                         */
/*                                                                       */
/* This file initializes entry-point function arguments to a default     */
/* size and value before calling the entry-point functions. It does      */
/* not store or use any values returned from the entry-point functions.  */
/* If necessary, it does pre-allocate memory for returned values.        */
/* You can use this file as a starting point for a main function that    */
/* you can deploy in your application.                                   */
/*                                                                       */
/* After you copy the file, and before you deploy it, you must make the  */
/* following changes:                                                    */
/* * For variable-size function arguments, change the example sizes to   */
/* the sizes that your application requires.                             */
/* * Change the example values of function arguments to the values that  */
/* your application requires.                                            */
/* * If the entry-point functions return values, store these values or   */
/* otherwise use them as required by your application.                   */
/*                                                                       */
/*************************************************************************/

/* Include Files */
#include "main.h"
#include "kalmanfilter.h"
#include "kalmanfilter_terminate.h"

/* Function Declarations */
static void argInit_2x1_real_T(double result[2]);
static double argInit_real_T(void);
static void main_kalmanfilter(void);

/* Function Definitions */

/*
 * Arguments    : double result[2]
 * Return Type  : void
 */
static void argInit_2x1_real_T(double result[2])
{
  double result_tmp;

  /* Loop over the array to initialize each element. */
  /* Set the value of the array element.
     Change this value to the value that the application requires. */
  result_tmp = argInit_real_T();
  result[0] = result_tmp;

  /* Set the value of the array element.
     Change this value to the value that the application requires. */
  result[1] = result_tmp;
}

/*
 * Arguments    : void
 * Return Type  : double
 */
static double argInit_real_T(void)
{
  return 0.0;
}

/*
 * Arguments    : void
 * Return Type  : void
 */
static void main_kalmanfilter(void)
{
  double dv[2];
  double y[2];

  /* Initialize function 'kalmanfilter' input arguments. */
  /* Initialize function input argument 'z'. */
  /* Call the entry-point 'kalmanfilter'. */
  argInit_2x1_real_T(dv);
  kalmanfilter(dv, y);
}

/*
 * Arguments    : int argc
 *                const char * const argv[]
 * Return Type  : int
 */
int main(int argc, const char * const argv[])
{
  (void)argc;
  (void)argv;

  /* The initialize function is being called automatically from your entry-point function. So, a call to initialize is not included here. */
  /* Invoke the entry-point functions.
     You can call entry-point functions multiple times. */
  main_kalmanfilter();

  /* Terminate the application.
     You do not need to do this more than one time. */
  kalmanfilter_terminate();
  return 0;
}

/*
 * File trailer for main.c
 *
 * [EOF]
 */
