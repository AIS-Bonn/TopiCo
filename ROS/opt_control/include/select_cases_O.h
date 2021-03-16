/*
 * Student License - for use by students to meet course requirements and
 * perform academic research at degree granting institutions only.  Not
 * for government, commercial, or other organizational use.
 *
 * select_cases_O.h
 *
 * Code generation for function 'select_cases_O'
 *
 */

#ifndef SELECT_CASES_O_H
#define SELECT_CASES_O_H

/* Include files */
#include <stddef.h>
#include <stdlib.h>
#include "rtwtypes.h"
#include "opt_control_lib_types.h"

/* Function Declarations */
extern void b_select_cases_O(double V_init, double A_init, double P_wayp, double
  V_wayp, double A_wayp, double V_max, double V_min, double A_max, double A_min,
  double J_max, double J_min, short cases_data[], int cases_size[2]);
extern void select_cases_O(double V_init, double A_init, double V_wayp, double
  V_max, double V_min, double A_max, double A_min, double J_max, double J_min,
  short cases_data[], int cases_size[2]);

#endif

/* End of code generation (select_cases_O.h) */
