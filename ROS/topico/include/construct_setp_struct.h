//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// construct_setp_struct.h
//
// Code generation for function 'construct_setp_struct'
//

#ifndef CONSTRUCT_SETP_STRUCT_H
#define CONSTRUCT_SETP_STRUCT_H

// Include files
#include "rtwtypes.h"
#include "coder_array.h"
#include <cstddef>
#include <cstdlib>

// Type Declarations
struct cell_wrap_0;

struct struct0_T;

// Function Declarations
void b_construct_setp_struct(
    const double t_in2_data[], const int t_in2_size[2],
    const double J_in2_data[], const int J_in2_size[2],
    coder::array<double, 2U> &J_setp_struct_time,
    coder::array<double, 2U> &J_setp_struct_signals_values);

void construct_setp_struct(const coder::array<cell_wrap_0, 2U> &t_in2,
                           const coder::array<cell_wrap_0, 2U> &J_in2,
                           coder::array<struct0_T, 2U> &J_setp_struct);

void construct_setp_struct(
    const double t_in2_data[], const int t_in2_size[2],
    const double J_in2_data[], const int J_in2_size[2],
    coder::array<double, 2U> &J_setp_struct_time,
    coder::array<double, 2U> &J_setp_struct_signals_values);

void construct_setp_struct(
    const double t_in2[7], const double J_in2[7],
    coder::array<double, 2U> &J_setp_struct_time,
    coder::array<double, 2U> &J_setp_struct_signals_values);

#endif
// End of code generation (construct_setp_struct.h)
