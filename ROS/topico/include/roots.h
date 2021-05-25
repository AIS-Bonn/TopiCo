//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// roots.h
//
// Code generation for function 'roots'
//

#ifndef ROOTS_H
#define ROOTS_H

// Include files
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

// Function Declarations
namespace coder {
void b_roots(const double c[7], creal_T r_data[], int *r_size);

void roots(const double c[6], creal_T r_data[], int *r_size);

} // namespace coder

#endif
// End of code generation (roots.h)
