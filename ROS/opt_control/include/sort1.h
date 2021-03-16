/*
 * Student License - for use by students to meet course requirements and
 * perform academic research at degree granting institutions only.  Not
 * for government, commercial, or other organizational use.
 *
 * sort1.h
 *
 * Code generation for function 'sort1'
 *
 */

#ifndef SORT1_H
#define SORT1_H

/* Include files */
#include <stddef.h>
#include <stdlib.h>
#include "rtwtypes.h"
#include "opt_control_lib_types.h"

/* Function Declarations */
extern void b_sort(double x_data[], int x_size[1], int idx_data[], int idx_size
                   [1]);
extern void c_sort(short x_data[], int x_size[2], int idx_data[], int idx_size[2]);
extern void d_sort(short x_data[], int x_size[2], int idx_data[], int idx_size[2]);
extern void sort(creal_T x_data[], int x_size[1], int idx_data[], int idx_size[1]);

#endif

/* End of code generation (sort1.h) */
