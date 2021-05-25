//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// correct_feasibility.h
//
// Code generation for function 'correct_feasibility'
//

#ifndef CORRECT_FEASIBILITY_H
#define CORRECT_FEASIBILITY_H

// Include files
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

// Function Declarations
void correct_feasibility(double P_init, double V_init, double A_init,
                         double V_max, double V_min, double A_max, double A_min,
                         double J_max, double J_min, bool b_hard_V_lim,
                         double t[4], double J[4]);

#endif
// End of code generation (correct_feasibility.h)
