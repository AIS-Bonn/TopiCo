/*
 * Student License - for use by students to meet course requirements and
 * perform academic research at degree granting institutions only.  Not
 * for government, commercial, or other organizational use.
 *
 * cat.h
 *
 * Code generation for function 'cat'
 *
 */

#ifndef CAT_H
#define CAT_H

/* Include files */
#include <stddef.h>
#include <stdlib.h>
#include "rtwtypes.h"
#include "opt_control_lib_types.h"

/* Function Declarations */
extern void b_cat(const emxArray_real_T *varargin_2, emxArray_real_T *y);
extern void c_cat(const emxArray_real_T *varargin_1, emxArray_real_T *y);
extern void cat(const emxArray_real_T *varargin_1, double varargin_2,
                emxArray_real_T *y);
extern void d_cat(const double varargin_1_data[], const double varargin_2_data[],
                  double y_data[], int y_size[1]);

#endif

/* End of code generation (cat.h) */
