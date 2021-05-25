//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// power.h
//
// Code generation for function 'power'
//

#ifndef POWER_H
#define POWER_H

// Include files
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

// Function Declarations
namespace coder {
void b_power(const creal_T a[4], creal_T y[4]);

creal_T b_power(const creal_T a);

creal_T c_power(const creal_T a);

void c_power(const creal_T a[3], creal_T y[3]);

creal_T d_power(const creal_T a);

creal_T e_power(const creal_T a);

creal_T f_power(const creal_T a);

void power(const creal_T a[2], creal_T y[2]);

creal_T power(const creal_T a);

} // namespace coder

#endif
// End of code generation (power.h)
