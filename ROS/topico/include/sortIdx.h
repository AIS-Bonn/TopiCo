//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// sortIdx.h
//
// Code generation for function 'sortIdx'
//

#ifndef SORTIDX_H
#define SORTIDX_H

// Include files
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

// Function Declarations
namespace coder {
namespace internal {
void merge(int idx_data[], double x_data[], int offset, int np, int nq,
           int iwork_data[], double xwork_data[]);

void sortIdx(double x_data[], const int *x_size, int idx_data[], int *idx_size);

} // namespace internal
} // namespace coder

#endif
// End of code generation (sortIdx.h)
