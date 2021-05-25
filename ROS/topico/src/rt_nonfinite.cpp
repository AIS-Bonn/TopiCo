//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// rt_nonfinite.cpp
//
// Code generation for function 'topico_wrapper'
//

// Abstract:
//      MATLAB for code generation function to initialize non-finites,
//      (Inf, NaN and -Inf).
// Include files
#include "rt_nonfinite.h"
#include <cmath>
#include <limits>

real_T rtNaN = std::numeric_limits<real_T>::quiet_NaN();
real_T rtInf = std::numeric_limits<real_T>::infinity();
real_T rtMinusInf = -std::numeric_limits<real_T>::infinity();
real32_T rtNaNF = std::numeric_limits<real32_T>::quiet_NaN();
real32_T rtInfF = std::numeric_limits<real32_T>::infinity();
real32_T rtMinusInfF = -std::numeric_limits<real32_T>::infinity();

// Function: rtIsInf ==================================================
//  Abstract:
//  Test if value is infinite
boolean_T rtIsInf(real_T value)
{
  return ((value == rtInf || value == rtMinusInf) ? true : false);
}

// Function: rtIsInfF =================================================
//  Abstract:
//  Test if single-precision value is infinite
boolean_T rtIsInfF(real32_T value)
{
  return ((value == rtInfF || value == rtMinusInfF) ? true : false);
}

// Function: rtIsNaN ==================================================
//  Abstract:
//  Test if value is not a number
boolean_T rtIsNaN(real_T value)
{
  return ((value != value) ? true : false);
}

// Function: rtIsNaNF =================================================
//  Abstract:
//  Test if single-precision value is not a number
boolean_T rtIsNaNF(real32_T value)
{
  return ((value != value) ? true : false);
}

// End of code generation (rt_nonfinite.cpp)
