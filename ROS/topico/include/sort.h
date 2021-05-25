//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// sort.h
//
// Code generation for function 'sort'
//

#ifndef SORT_H
#define SORT_H

// Include files
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

// Function Declarations
namespace coder {
namespace internal {
void b_sort(double x_data[], const int *x_size, int idx_data[], int *idx_size);

void sort(double x_data[], const int *x_size, int idx_data[], int *idx_size);

} // namespace internal
} // namespace coder

#endif
// End of code generation (sort.h)
