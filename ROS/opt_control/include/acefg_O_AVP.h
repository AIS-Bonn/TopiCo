/*
 * Student License - for use by students to meet course requirements and
 * perform academic research at degree granting institutions only.  Not
 * for government, commercial, or other organizational use.
 *
 * acefg_O_AVP.h
 *
 * Code generation for function 'acefg_O_AVP'
 *
 */

#ifndef ACEFG_O_AVP_H
#define ACEFG_O_AVP_H

/* Include files */
#include <stddef.h>
#include <stdlib.h>
#include "rtwtypes.h"
#include "opt_control_lib_types.h"

/* Function Declarations */
extern void acefg_O_AVP(double V_init, double A_init, double A_min, double J_max,
  double J_min, creal_T t_data[], int t_size[2]);
extern void b_acefg_O_AVP(double V_init, double A_init, double A_min, double
  J_max, double J_min, creal_T t_data[], int t_size[2]);
extern void c_acefg_O_AVP(double P_init, double V_init, double A_init, double
  P_wayp, double V_wayp, double A_wayp, double A_min, double J_max, double J_min,
  creal_T t_data[], int t_size[2]);

#endif

/* End of code generation (acefg_O_AVP.h) */
