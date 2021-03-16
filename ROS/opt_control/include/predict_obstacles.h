/*
 * Student License - for use by students to meet course requirements and
 * perform academic research at degree granting institutions only.  Not
 * for government, commercial, or other organizational use.
 *
 * predict_obstacles.h
 *
 * Code generation for function 'predict_obstacles'
 *
 */

#ifndef PREDICT_OBSTACLES_H
#define PREDICT_OBSTACLES_H

/* Include files */
#include <stddef.h>
#include <stdlib.h>
#include "rtwtypes.h"
#include "opt_control_lib_types.h"

/* Function Declarations */
extern void predict_obstacles(const emxArray_real_T *Obstacles, const
  emxArray_real_T *VP_wayp, const emxArray_real_T *AP_wayp, emxArray_real_T
  *Obstacles_predicted);

#endif

/* End of code generation (predict_obstacles.h) */
