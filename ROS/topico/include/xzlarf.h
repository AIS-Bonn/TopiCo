//
// Student License - for use by students to meet course requirements and
// perform academic research at degree granting institutions only.  Not
// for government, commercial, or other organizational use.
//
// xzlarf.h
//
// Code generation for function 'xzlarf'
//

#ifndef XZLARF_H
#define XZLARF_H

// Include files
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

// Function Declarations
namespace coder {
namespace internal {
namespace reflapack {
void xzlarf(int m, int n, int iv0, double tau, double C_data[], int ic0,
            int ldc, double work_data[]);

}
} // namespace internal
} // namespace coder

#endif
// End of code generation (xzlarf.h)
