//
// Student License - for use by students to meet course requirements and
// perform academic research at degree granting institutions only.  Not
// for government, commercial, or other organizational use.
//
// rtGetNaN.cpp
//
// Code generation for function 'topico_wrapper'
//

// Abstract:
//       MATLAB for code generation function to initialize non-finite, NaN
// Include files
#include "rtGetNaN.h"
#include "rt_nonfinite.h"

// Function: rtGetNaN
// ======================================================================
//  Abstract:
// Initialize rtNaN needed by the generated code.
//  NaN is initialized as non-signaling. Assumes IEEE.
real_T rtGetNaN(void)
{
  return rtNaN;
}

// Function: rtGetNaNF
// =====================================================================
//  Abstract:
//  Initialize rtNaNF needed by the generated code.
//  NaN is initialized as non-signaling. Assumes IEEE
real32_T rtGetNaNF(void)
{
  return rtNaNF;
}

// End of code generation (rtGetNaN.cpp)
