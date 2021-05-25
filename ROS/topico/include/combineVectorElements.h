//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// combineVectorElements.h
//
// Code generation for function 'combineVectorElements'
//

#ifndef COMBINEVECTORELEMENTS_H
#define COMBINEVECTORELEMENTS_H

// Include files
#include "rtwtypes.h"
#include "coder_array.h"
#include <cstddef>
#include <cstdlib>

// Function Declarations
namespace coder {
double b_combineVectorElements(const ::coder::array<double, 2U> &x);

void c_combineVectorElements(const ::coder::array<bool, 2U> &x,
                             ::coder::array<int, 1U> &y);

void combineVectorElements(const ::coder::array<bool, 2U> &x,
                           ::coder::array<int, 2U> &y);

} // namespace coder

#endif
// End of code generation (combineVectorElements.h)
