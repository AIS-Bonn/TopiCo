//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// select_cases_TV.h
//
// Code generation for function 'select_cases_TV'
//

#ifndef SELECT_CASES_TV_H
#define SELECT_CASES_TV_H

// Include files
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

// Function Declarations
void select_cases_TV(unsigned char n, unsigned char cond, bool b_sync_V,
                     int cases_data[], int cases_size[2]);

void select_cases_TV_init();

#endif
// End of code generation (select_cases_TV.h)
