//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// xzlartg.h
//
// Code generation for function 'xzlartg'
//

#ifndef XZLARTG_H
#define XZLARTG_H

// Include files
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

// Function Declarations
namespace coder {
namespace internal {
namespace reflapack {
void xzlartg(const creal_T f, const creal_T g, double *cs, creal_T *sn);

void xzlartg(const creal_T f, const creal_T g, double *cs, creal_T *sn,
             creal_T *r);

} // namespace reflapack
} // namespace internal
} // namespace coder

#endif
// End of code generation (xzlartg.h)
