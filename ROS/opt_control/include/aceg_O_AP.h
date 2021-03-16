/*
 * Student License - for use by students to meet course requirements and
 * perform academic research at degree granting institutions only.  Not
 * for government, commercial, or other organizational use.
 *
 * aceg_O_AP.h
 *
 * Code generation for function 'aceg_O_AP'
 *
 */

#ifndef ACEG_O_AP_H
#define ACEG_O_AP_H

/* Include files */
#include <stddef.h>
#include <stdlib.h>
#include "rtwtypes.h"
#include "opt_control_lib_types.h"

/* Function Declarations */
extern void aceg_O_AP(double J_max, double J_min, creal_T t_data[], int t_size[2]);
extern void b_aceg_O_AP(double J_max, double J_min, creal_T t_data[], int
  t_size[2]);
extern void c_aceg_O_AP(double P_init, double V_init, double A_init, double
  P_wayp, double A_wayp, double V_max, double J_max, double J_min, creal_T
  t_data[], int t_size[2]);

#endif

/* End of code generation (aceg_O_AP.h) */
