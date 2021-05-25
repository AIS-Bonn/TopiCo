//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// find.h
//
// Code generation for function 'find'
//

#ifndef FIND_H
#define FIND_H

// Include files
#include "rtwtypes.h"
#include "coder_array.h"
#include <cstddef>
#include <cstdlib>

// Function Declarations
namespace coder {
void b_eml_find(const ::coder::array<bool, 2U> &x, int i_data[], int i_size[2]);

void eml_find(const ::coder::array<bool, 2U> &x, ::coder::array<int, 2U> &i);

} // namespace coder

#endif
// End of code generation (find.h)
