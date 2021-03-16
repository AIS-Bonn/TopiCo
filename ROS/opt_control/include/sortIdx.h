/*
 * Student License - for use by students to meet course requirements and
 * perform academic research at degree granting institutions only.  Not
 * for government, commercial, or other organizational use.
 *
 * sortIdx.h
 *
 * Code generation for function 'sortIdx'
 *
 */

#ifndef SORTIDX_H
#define SORTIDX_H

/* Include files */
#include <stddef.h>
#include <stdlib.h>
#include "rtwtypes.h"
#include "opt_control_lib_types.h"

/* Function Declarations */
extern void b_merge(int idx_data[], short x_data[], int offset, int np, int nq,
                    int iwork_data[], short xwork_data[]);
extern void b_sortIdx(short x_data[], int x_size[1], int idx_data[], int
                      idx_size[1]);
extern void sortIdx(double x_data[], int x_size[1], int idx_data[], int
                    idx_size[1]);

#endif

/* End of code generation (sortIdx.h) */
