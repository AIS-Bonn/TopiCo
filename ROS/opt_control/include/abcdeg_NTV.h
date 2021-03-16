/*
 * Student License - for use by students to meet course requirements and
 * perform academic research at degree granting institutions only.  Not
 * for government, commercial, or other organizational use.
 *
 * abcdeg_NTV.h
 *
 * Code generation for function 'abcdeg_NTV'
 *
 */

#ifndef ABCDEG_NTV_H
#define ABCDEG_NTV_H

/* Include files */
#include <stddef.h>
#include <stdlib.h>
#include "rtwtypes.h"
#include "opt_control_lib_types.h"

/* Function Declarations */
extern void abcdeg_NTV(double P_init, double V_init, double A_init, double
  P_wayp, double V_wayp, double A_wayp, double A_min, double J_max, double J_min,
  double T, creal_T t_data[], int t_size[2]);

#endif

/* End of code generation (abcdeg_NTV.h) */
