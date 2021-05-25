//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// cut_to_time.h
//
// Code generation for function 'cut_to_time'
//

#ifndef CUT_TO_TIME_H
#define CUT_TO_TIME_H

// Include files
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

// Type Declarations
struct cell_wrap_17;

// Function Declarations
void cut_to_time(const cell_wrap_17 *t_in, const cell_wrap_17 *J_in, double T,
                 cell_wrap_17 *t_out, cell_wrap_17 *J_out);

#endif
// End of code generation (cut_to_time.h)
