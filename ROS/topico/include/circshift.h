//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// circshift.h
//
// Code generation for function 'circshift'
//

#ifndef CIRCSHIFT_H
#define CIRCSHIFT_H

// Include files
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

// Function Declarations
namespace coder {
void circshift(double a_data[], const int a_size[2]);

void circshift(int a_data[], const int *a_size);

} // namespace coder

#endif
// End of code generation (circshift.h)
