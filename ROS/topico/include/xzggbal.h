//
// Student License - for use by students to meet course requirements and
// perform academic research at degree granting institutions only.  Not
// for government, commercial, or other organizational use.
//
// xzggbal.h
//
// Code generation for function 'xzggbal'
//

#ifndef XZGGBAL_H
#define XZGGBAL_H

// Include files
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

// Function Declarations
namespace coder {
namespace internal {
namespace reflapack {
void xzggbal(creal_T A_data[], const int A_size[2], int *ilo, int *ihi,
             int rscale_data[], int *rscale_size);

}
} // namespace internal
} // namespace coder

#endif
// End of code generation (xzggbal.h)
