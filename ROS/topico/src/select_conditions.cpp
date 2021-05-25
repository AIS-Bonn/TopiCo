//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// select_conditions.cpp
//
// Code generation for function 'select_conditions'
//

// Include files
#include "select_conditions.h"
#include "rt_nonfinite.h"
#include "topico_wrapper_data.h"

// Function Definitions
void select_conditions_init()
{
  static const unsigned char uv[48] = {
      11U, 0U, 0U, 7U, 19U, 0U, 3U, 0U, 27U, 15U, 23U, 31U,
      10U, 0U, 0U, 6U, 18U, 0U, 2U, 0U, 26U, 14U, 22U, 30U,
      9U,  0U, 0U, 5U, 17U, 0U, 1U, 0U, 25U, 13U, 21U, 29U,
      12U, 0U, 0U, 8U, 20U, 0U, 4U, 0U, 28U, 16U, 24U, 32U};
  for (int i = 0; i < 48; i++) {
    condlut[i] = uv[i];
  }
}

// End of code generation (select_conditions.cpp)
