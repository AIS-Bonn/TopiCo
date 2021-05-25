//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// select_type.cpp
//
// Code generation for function 'select_type'
//

// Include files
#include "select_type.h"
#include "rt_nonfinite.h"
#include "topico_wrapper_data.h"

// Function Definitions
void select_type_init()
{
  static const unsigned char uv[8] = {0U, 7U, 6U, 2U, 5U, 3U, 4U, 1U};
  for (int i = 0; i < 8; i++) {
    nlut[i] = uv[i];
  }
}

// End of code generation (select_type.cpp)
