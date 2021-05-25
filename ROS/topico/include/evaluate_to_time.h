//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// evaluate_to_time.h
//
// Code generation for function 'evaluate_to_time'
//

#ifndef EVALUATE_TO_TIME_H
#define EVALUATE_TO_TIME_H

// Include files
#include "rtwtypes.h"
#include "coder_array.h"
#include <cstddef>
#include <cstdlib>

// Function Declarations
void evaluate_to_time(
    double P_init, double V_init, double A_init,
    const coder::array<double, 2U> &J_setp_struct_time,
    const coder::array<double, 2U> &J_setp_struct_signals_values, double T_in,
    double *P, double *V, double *A, double *J);

void evaluate_to_time(
    double P_init, double V_init, double A_init,
    const coder::array<double, 2U> &J_setp_struct_time,
    const coder::array<double, 2U> &J_setp_struct_signals_values, double *P,
    double *V, double *A, double *J);

void evaluate_to_time(
    double P_init, double V_init, double A_init,
    const coder::array<double, 2U> &J_setp_struct_time,
    const coder::array<double, 2U> &J_setp_struct_signals_values, double *P,
    double *V, double *A);

#endif
// End of code generation (evaluate_to_time.h)
