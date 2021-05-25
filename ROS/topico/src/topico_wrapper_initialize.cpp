//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// topico_wrapper_initialize.cpp
//
// Code generation for function 'topico_wrapper_initialize'
//

// Include files
#include "topico_wrapper_initialize.h"
#include "rt_nonfinite.h"
#include "select_cases_O.h"
#include "select_cases_TA.h"
#include "select_cases_TV.h"
#include "select_conditions.h"
#include "select_type.h"
#include "topico_wrapper_data.h"

// Function Definitions
void topico_wrapper_initialize()
{
  select_type_init();
  select_conditions_init();
  select_cases_O_init();
  select_cases_TV_init();
  select_cases_TA_init();
  isInitialized_topico_wrapper = true;
}

// End of code generation (topico_wrapper_initialize.cpp)
