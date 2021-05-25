//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// nnz.cpp
//
// Code generation for function 'nnz'
//

// Include files
#include "nnz.h"
#include "rt_nonfinite.h"

// Function Definitions
namespace coder {
int intnnz(const bool s_data[], int s_size)
{
  int n;
  n = 0;
  for (int k = 0; k < s_size; k++) {
    if (s_data[k]) {
      n++;
    }
  }
  return n;
}

} // namespace coder

// End of code generation (nnz.cpp)
