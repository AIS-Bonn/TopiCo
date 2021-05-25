//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// xrot.h
//
// Code generation for function 'xrot'
//

#ifndef XROT_H
#define XROT_H

// Include files
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

// Function Declarations
namespace coder {
namespace internal {
namespace blas {
void xrot(int n, double x_data[], int ix0, int incx, int iy0, int incy,
          double c, double s);

void xrot(int n, double x_data[], int ix0, int iy0, double c, double s);

} // namespace blas
} // namespace internal
} // namespace coder

#endif
// End of code generation (xrot.h)
