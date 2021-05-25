//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// xzhgeqz.h
//
// Code generation for function 'xzhgeqz'
//

#ifndef XZHGEQZ_H
#define XZHGEQZ_H

// Include files
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

// Function Declarations
namespace coder {
namespace internal {
namespace reflapack {
void xzhgeqz(const creal_T A_data[], const int A_size[2], int ilo, int ihi,
             int *info, creal_T alpha1_data[], int *alpha1_size,
             creal_T beta1_data[], int *beta1_size);

}
} // namespace internal
} // namespace coder

#endif
// End of code generation (xzhgeqz.h)
