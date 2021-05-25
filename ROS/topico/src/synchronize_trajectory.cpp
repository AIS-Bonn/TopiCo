//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// synchronize_trajectory.cpp
//
// Code generation for function 'synchronize_trajectory'
//

// Include files
#include "synchronize_trajectory.h"
#include "all.h"
#include "any1.h"
#include "circshift.h"
#include "construct_setp_struct.h"
#include "correct_feasibility.h"
#include "evaluate_to_time.h"
#include "minOrMax.h"
#include "nnz.h"
#include "printint.h"
#include "rt_nonfinite.h"
#include "solve_O.h"
#include "solve_T.h"
#include "topico_wrapper_data.h"
#include "topico_wrapper_internal_types.h"
#include "topico_wrapper_rtwutil.h"
#include "topico_wrapper_types.h"
#include "coder_array.h"
#include "coder_bounded_array.h"
#include "rt_nonfinite.h"
#include <stdio.h>

// Type Definitions
struct cell_wrap_15 {
  double f1[11];
};

struct cell_wrap_12 {
  coder::bounded_array<double, 4U, 2U> f1;
};

struct cell_wrap_13 {
  coder::bounded_array<double, 700U, 2U> f1;
};

struct cell_wrap_14 {
  coder::bounded_array<double, 1100U, 2U> f1;
};

struct cell_wrap_16 {
  coder::bounded_array<int, 100U, 1U> f1;
};

// Function Definitions
void synchronize_trajectory(
    coder::array<double, 1U> &P_init, coder::array<double, 1U> &V_init,
    coder::array<double, 1U> &A_init, coder::array<double, 1U> &P_wayp,
    coder::array<double, 1U> &V_wayp, coder::array<double, 1U> &A_wayp,
    const coder::array<double, 1U> &V_max,
    const coder::array<double, 1U> &V_min,
    const coder::array<double, 1U> &A_max, coder::array<double, 1U> &A_min,
    coder::array<double, 1U> &J_max, coder::array<double, 1U> &J_min,
    const coder::array<bool, 1U> &b_sync_V,
    const coder::array<bool, 1U> &b_sync_A,
    const coder::array<bool, 1U> &b_sync_J,
    const coder::array<bool, 1U> &b_sync_W,
    const coder::array<bool, 1U> &b_hard_V_lim,
    const coder::array<double, 1U> &T_catch_up,
    const coder::array<signed char, 1U> &direction,
    coder::array<cell_wrap_0, 1U> &t_out, coder::array<cell_wrap_0, 1U> &J_out,
    coder::array<int, 1U> &solution_out)
{
  static rtBoundsCheckInfo ac_emlrtBCI = {
      -1,                                                            // iFirst
      -1,                                                            // iLast
      62,                                                            // lineNo
      27,                                                            // colNo
      "t_opt",                                                       // aName
      "synchronize_trajectory",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/synchronize_trajectory.m", // pName
      0 // checkKind
  };
  static rtBoundsCheckInfo ad_emlrtBCI = {
      -1,                                                            // iFirst
      -1,                                                            // iLast
      125,                                                           // lineNo
      125,                                                           // colNo
      "V_max",                                                       // aName
      "synchronize_trajectory",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/synchronize_trajectory.m", // pName
      0 // checkKind
  };
  static rtBoundsCheckInfo ae_emlrtBCI = {
      -1,                                                            // iFirst
      -1,                                                            // iLast
      133,                                                           // lineNo
      202,                                                           // colNo
      "V_max",                                                       // aName
      "synchronize_trajectory",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/synchronize_trajectory.m", // pName
      0 // checkKind
  };
  static rtBoundsCheckInfo af_emlrtBCI = {
      -1,                                                            // iFirst
      -1,                                                            // iLast
      166,                                                           // lineNo
      223,                                                           // colNo
      "J_opt_test",                                                  // aName
      "synchronize_trajectory",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/synchronize_trajectory.m", // pName
      0 // checkKind
  };
  static rtBoundsCheckInfo ag_emlrtBCI = {
      -1,                                                            // iFirst
      -1,                                                            // iLast
      171,                                                           // lineNo
      119,                                                           // colNo
      "V_init",                                                      // aName
      "synchronize_trajectory",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/synchronize_trajectory.m", // pName
      0 // checkKind
  };
  static rtBoundsCheckInfo ah_emlrtBCI = {
      -1,                                                            // iFirst
      -1,                                                            // iLast
      175,                                                           // lineNo
      61,                                                            // colNo
      "J_tim{index_axis}",                                           // aName
      "synchronize_trajectory",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/synchronize_trajectory.m", // pName
      0 // checkKind
  };
  static rtBoundsCheckInfo ai_emlrtBCI = {
      -1,                                                            // iFirst
      -1,                                                            // iLast
      164,                                                           // lineNo
      98,                                                            // colNo
      "V_wayp",                                                      // aName
      "synchronize_trajectory",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/synchronize_trajectory.m", // pName
      0 // checkKind
  };
  static rtBoundsCheckInfo aj_emlrtBCI = {
      -1,                                                            // iFirst
      -1,                                                            // iLast
      219,                                                           // lineNo
      77,                                                            // colNo
      "solution_opt{index_axis,:}",                                  // aName
      "synchronize_trajectory",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/synchronize_trajectory.m", // pName
      0 // checkKind
  };
  static rtBoundsCheckInfo bc_emlrtBCI = {
      -1,                                                            // iFirst
      -1,                                                            // iLast
      72,                                                            // lineNo
      15,                                                            // colNo
      "t_opt",                                                       // aName
      "synchronize_trajectory",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/synchronize_trajectory.m", // pName
      0 // checkKind
  };
  static rtBoundsCheckInfo bd_emlrtBCI = {
      -1,                                                            // iFirst
      -1,                                                            // iLast
      125,                                                           // lineNo
      143,                                                           // colNo
      "V_min",                                                       // aName
      "synchronize_trajectory",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/synchronize_trajectory.m", // pName
      0 // checkKind
  };
  static rtBoundsCheckInfo be_emlrtBCI = {
      -1,                                                            // iFirst
      -1,                                                            // iLast
      133,                                                           // lineNo
      220,                                                           // colNo
      "V_min",                                                       // aName
      "synchronize_trajectory",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/synchronize_trajectory.m", // pName
      0 // checkKind
  };
  static rtBoundsCheckInfo bf_emlrtBCI = {
      -1,                                                            // iFirst
      -1,                                                            // iLast
      166,                                                           // lineNo
      207,                                                           // colNo
      "t_opt_test{index_axis}",                                      // aName
      "synchronize_trajectory",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/synchronize_trajectory.m", // pName
      0 // checkKind
  };
  static rtBoundsCheckInfo bg_emlrtBCI = {
      -1,                                                            // iFirst
      -1,                                                            // iLast
      171,                                                           // lineNo
      138,                                                           // colNo
      "A_init",                                                      // aName
      "synchronize_trajectory",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/synchronize_trajectory.m", // pName
      0 // checkKind
  };
  static rtBoundsCheckInfo bh_emlrtBCI = {
      -1,                                                            // iFirst
      -1,                                                            // iLast
      175,                                                           // lineNo
      25,                                                            // colNo
      "J",                                                           // aName
      "synchronize_trajectory",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/synchronize_trajectory.m", // pName
      0 // checkKind
  };
  static rtBoundsCheckInfo bi_emlrtBCI = {
      -1,                                                            // iFirst
      -1,                                                            // iLast
      164,                                                           // lineNo
      127,                                                           // colNo
      "A_wayp",                                                      // aName
      "synchronize_trajectory",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/synchronize_trajectory.m", // pName
      0 // checkKind
  };
  static rtBoundsCheckInfo bj_emlrtBCI = {
      -1,                                                            // iFirst
      -1,                                                            // iLast
      219,                                                           // lineNo
      21,                                                            // colNo
      "solution_out",                                                // aName
      "synchronize_trajectory",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/synchronize_trajectory.m", // pName
      0 // checkKind
  };
  static rtBoundsCheckInfo cc_emlrtBCI = {
      -1,                                                            // iFirst
      -1,                                                            // iLast
      62,                                                            // lineNo
      27,                                                            // colNo
      "J_opt",                                                       // aName
      "synchronize_trajectory",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/synchronize_trajectory.m", // pName
      0 // checkKind
  };
  static rtBoundsCheckInfo cd_emlrtBCI = {
      -1,                                                            // iFirst
      -1,                                                            // iLast
      125,                                                           // lineNo
      161,                                                           // colNo
      "A_max",                                                       // aName
      "synchronize_trajectory",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/synchronize_trajectory.m", // pName
      0 // checkKind
  };
  static rtBoundsCheckInfo ce_emlrtBCI = {
      -1,                                                            // iFirst
      -1,                                                            // iLast
      133,                                                           // lineNo
      238,                                                           // colNo
      "A_max",                                                       // aName
      "synchronize_trajectory",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/synchronize_trajectory.m", // pName
      0 // checkKind
  };
  static rtBoundsCheckInfo cf_emlrtBCI = {
      -1,                                                            // iFirst
      -1,                                                            // iLast
      166,                                                           // lineNo
      235,                                                           // colNo
      "J_opt_test{index_axis}",                                      // aName
      "synchronize_trajectory",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/synchronize_trajectory.m", // pName
      0 // checkKind
  };
  static rtBoundsCheckInfo cg_emlrtBCI = {
      -1,                                                            // iFirst
      -1,                                                            // iLast
      171,                                                           // lineNo
      157,                                                           // colNo
      "P_wayp",                                                      // aName
      "synchronize_trajectory",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/synchronize_trajectory.m", // pName
      0 // checkKind
  };
  static rtBoundsCheckInfo ch_emlrtBCI = {
      -1,                                                            // iFirst
      -1,                                                            // iLast
      176,                                                           // lineNo
      67,                                                            // colNo
      "solution_tim",                                                // aName
      "synchronize_trajectory",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/synchronize_trajectory.m", // pName
      0 // checkKind
  };
  static rtBoundsCheckInfo ci_emlrtBCI = {
      -1,                                                            // iFirst
      -1,                                                            // iLast
      163,                                                           // lineNo
      80,                                                            // colNo
      "b_sync_A",                                                    // aName
      "synchronize_trajectory",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/synchronize_trajectory.m", // pName
      0 // checkKind
  };
  static rtBoundsCheckInfo cj_emlrtBCI = {
      1,                                                             // iFirst
      100,                                                           // iLast
      201,                                                           // lineNo
      61,                                                            // colNo
      "solution_opt_test_all",                                       // aName
      "synchronize_trajectory",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/synchronize_trajectory.m", // pName
      0 // checkKind
  };
  static rtBoundsCheckInfo dc_emlrtBCI = {
      -1,                                                            // iFirst
      -1,                                                            // iLast
      76,                                                            // lineNo
      15,                                                            // colNo
      "J_opt",                                                       // aName
      "synchronize_trajectory",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/synchronize_trajectory.m", // pName
      0 // checkKind
  };
  static rtBoundsCheckInfo dd_emlrtBCI = {
      -1,                                                            // iFirst
      -1,                                                            // iLast
      125,                                                           // lineNo
      179,                                                           // colNo
      "A_min",                                                       // aName
      "synchronize_trajectory",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/synchronize_trajectory.m", // pName
      0 // checkKind
  };
  static rtBoundsCheckInfo de_emlrtBCI = {
      -1,                                                            // iFirst
      -1,                                                            // iLast
      133,                                                           // lineNo
      256,                                                           // colNo
      "A_min",                                                       // aName
      "synchronize_trajectory",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/synchronize_trajectory.m", // pName
      0 // checkKind
  };
  static rtBoundsCheckInfo df_emlrtBCI = {
      -1,                                                            // iFirst
      -1,                                                            // iLast
      166,                                                           // lineNo
      105,                                                           // colNo
      "P_init",                                                      // aName
      "synchronize_trajectory",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/synchronize_trajectory.m", // pName
      0 // checkKind
  };
  static rtBoundsCheckInfo dg_emlrtBCI = {
      -1,                                                            // iFirst
      -1,                                                            // iLast
      171,                                                           // lineNo
      176,                                                           // colNo
      "V_wayp",                                                      // aName
      "synchronize_trajectory",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/synchronize_trajectory.m", // pName
      0 // checkKind
  };
  static rtBoundsCheckInfo dh_emlrtBCI = {
      -1,                                                            // iFirst
      -1,                                                            // iLast
      176,                                                           // lineNo
      79,                                                            // colNo
      "solution_tim{index_axis}",                                    // aName
      "synchronize_trajectory",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/synchronize_trajectory.m", // pName
      0 // checkKind
  };
  static rtBoundsCheckInfo di_emlrtBCI = {
      -1,                                                            // iFirst
      -1,                                                            // iLast
      163,                                                           // lineNo
      112,                                                           // colNo
      "b_sync_J",                                                    // aName
      "synchronize_trajectory",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/synchronize_trajectory.m", // pName
      0 // checkKind
  };
  static rtBoundsCheckInfo dj_emlrtBCI = {
      -1,                                                            // iFirst
      -1,                                                            // iLast
      227,                                                           // lineNo
      38,                                                            // colNo
      "t_opt",                                                       // aName
      "synchronize_trajectory",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/synchronize_trajectory.m", // pName
      0 // checkKind
  };
  static rtBoundsCheckInfo ec_emlrtBCI = {
      -1,                                                            // iFirst
      -1,                                                            // iLast
      62,                                                            // lineNo
      27,                                                            // colNo
      "t_tim",                                                       // aName
      "synchronize_trajectory",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/synchronize_trajectory.m", // pName
      0 // checkKind
  };
  static rtBoundsCheckInfo ed_emlrtBCI = {
      -1,                                                            // iFirst
      -1,                                                            // iLast
      125,                                                           // lineNo
      197,                                                           // colNo
      "J_max",                                                       // aName
      "synchronize_trajectory",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/synchronize_trajectory.m", // pName
      0 // checkKind
  };
  static rtBoundsCheckInfo ee_emlrtBCI = {
      -1,                                                            // iFirst
      -1,                                                            // iLast
      133,                                                           // lineNo
      274,                                                           // colNo
      "J_max",                                                       // aName
      "synchronize_trajectory",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/synchronize_trajectory.m", // pName
      0 // checkKind
  };
  static rtBoundsCheckInfo ef_emlrtBCI = {
      -1,                                                            // iFirst
      -1,                                                            // iLast
      166,                                                           // lineNo
      124,                                                           // colNo
      "V_init",                                                      // aName
      "synchronize_trajectory",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/synchronize_trajectory.m", // pName
      0 // checkKind
  };
  static rtBoundsCheckInfo eg_emlrtBCI = {
      -1,                                                            // iFirst
      -1,                                                            // iLast
      171,                                                           // lineNo
      195,                                                           // colNo
      "A_wayp",                                                      // aName
      "synchronize_trajectory",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/synchronize_trajectory.m", // pName
      0 // checkKind
  };
  static rtBoundsCheckInfo eh_emlrtBCI = {
      -1,                                                            // iFirst
      -1,                                                            // iLast
      176,                                                           // lineNo
      25,                                                            // colNo
      "solution_out",                                                // aName
      "synchronize_trajectory",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/synchronize_trajectory.m", // pName
      0 // checkKind
  };
  static rtBoundsCheckInfo ei_emlrtBCI = {
      -1,                                                            // iFirst
      -1,                                                            // iLast
      163,                                                           // lineNo
      144,                                                           // colNo
      "b_sync_W",                                                    // aName
      "synchronize_trajectory",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/synchronize_trajectory.m", // pName
      0 // checkKind
  };
  static rtBoundsCheckInfo ej_emlrtBCI = {
      -1,                                                            // iFirst
      -1,                                                            // iLast
      227,                                                           // lineNo
      52,                                                            // colNo
      "t_opt{index_axis,:}",                                         // aName
      "synchronize_trajectory",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/synchronize_trajectory.m", // pName
      0 // checkKind
  };
  static rtBoundsCheckInfo fc_emlrtBCI = {
      -1,                                                            // iFirst
      -1,                                                            // iLast
      80,                                                            // lineNo
      15,                                                            // colNo
      "t_tim",                                                       // aName
      "synchronize_trajectory",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/synchronize_trajectory.m", // pName
      0 // checkKind
  };
  static rtBoundsCheckInfo fd_emlrtBCI = {
      -1,                                                            // iFirst
      -1,                                                            // iLast
      125,                                                           // lineNo
      215,                                                           // colNo
      "J_min",                                                       // aName
      "synchronize_trajectory",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/synchronize_trajectory.m", // pName
      0 // checkKind
  };
  static rtBoundsCheckInfo fe_emlrtBCI = {
      -1,                                                            // iFirst
      -1,                                                            // iLast
      133,                                                           // lineNo
      292,                                                           // colNo
      "J_min",                                                       // aName
      "synchronize_trajectory",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/synchronize_trajectory.m", // pName
      0 // checkKind
  };
  static rtBoundsCheckInfo ff_emlrtBCI = {
      -1,                                                            // iFirst
      -1,                                                            // iLast
      166,                                                           // lineNo
      143,                                                           // colNo
      "A_init",                                                      // aName
      "synchronize_trajectory",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/synchronize_trajectory.m", // pName
      0 // checkKind
  };
  static rtBoundsCheckInfo fg_emlrtBCI = {
      -1,                                                            // iFirst
      -1,                                                            // iLast
      171,                                                           // lineNo
      214,                                                           // colNo
      "V_max",                                                       // aName
      "synchronize_trajectory",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/synchronize_trajectory.m", // pName
      0 // checkKind
  };
  static rtBoundsCheckInfo fh_emlrtBCI = {
      -1,                                                            // iFirst
      -1,                                                            // iLast
      188,                                                           // lineNo
      51,                                                            // colNo
      "t_opt_test",                                                  // aName
      "synchronize_trajectory",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/synchronize_trajectory.m", // pName
      0 // checkKind
  };
  static rtBoundsCheckInfo fi_emlrtBCI = {
      -1,                                                            // iFirst
      -1,                                                            // iLast
      197,                                                           // lineNo
      65,                                                            // colNo
      "solution_opt_test",                                           // aName
      "synchronize_trajectory",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/synchronize_trajectory.m", // pName
      0 // checkKind
  };
  static rtBoundsCheckInfo fj_emlrtBCI = {
      -1,                                                            // iFirst
      -1,                                                            // iLast
      227,                                                           // lineNo
      13,                                                            // colNo
      "t",                                                           // aName
      "synchronize_trajectory",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/synchronize_trajectory.m", // pName
      0 // checkKind
  };
  static rtBoundsCheckInfo gc_emlrtBCI = {
      -1,                                                            // iFirst
      -1,                                                            // iLast
      62,                                                            // lineNo
      27,                                                            // colNo
      "J_tim",                                                       // aName
      "synchronize_trajectory",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/synchronize_trajectory.m", // pName
      0 // checkKind
  };
  static rtBoundsCheckInfo gd_emlrtBCI = {
      -1,                                                            // iFirst
      -1,                                                            // iLast
      125,                                                           // lineNo
      233,                                                           // colNo
      "b_hard_V_lim",                                                // aName
      "synchronize_trajectory",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/synchronize_trajectory.m", // pName
      0 // checkKind
  };
  static rtBoundsCheckInfo ge_emlrtBCI = {
      -1,                                                            // iFirst
      -1,                                                            // iLast
      133,                                                           // lineNo
      10,                                                            // colNo
      "t_opt",                                                       // aName
      "synchronize_trajectory",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/synchronize_trajectory.m", // pName
      0 // checkKind
  };
  static rtBoundsCheckInfo gf_emlrtBCI = {
      -1,                                                            // iFirst
      -1,                                                            // iLast
      166,                                                           // lineNo
      26,                                                            // colNo
      "P_wayp",                                                      // aName
      "synchronize_trajectory",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/synchronize_trajectory.m", // pName
      0 // checkKind
  };
  static rtBoundsCheckInfo gg_emlrtBCI = {
      -1,                                                            // iFirst
      -1,                                                            // iLast
      171,                                                           // lineNo
      232,                                                           // colNo
      "V_min",                                                       // aName
      "synchronize_trajectory",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/synchronize_trajectory.m", // pName
      0 // checkKind
  };
  static rtBoundsCheckInfo gh_emlrtBCI = {
      -1,                                                            // iFirst
      -1,                                                            // iLast
      188,                                                           // lineNo
      65,                                                            // colNo
      "t_opt_test{index_axis,:}",                                    // aName
      "synchronize_trajectory",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/synchronize_trajectory.m", // pName
      0 // checkKind
  };
  static rtBoundsCheckInfo gi_emlrtBCI = {
      -1,                                                            // iFirst
      -1,                                                            // iLast
      198,                                                           // lineNo
      95,                                                            // colNo
      "solution_opt_test",                                           // aName
      "synchronize_trajectory",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/synchronize_trajectory.m", // pName
      0 // checkKind
  };
  static rtBoundsCheckInfo gj_emlrtBCI = {
      -1,                                                            // iFirst
      -1,                                                            // iLast
      228,                                                           // lineNo
      38,                                                            // colNo
      "J_opt",                                                       // aName
      "synchronize_trajectory",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/synchronize_trajectory.m", // pName
      0 // checkKind
  };
  static rtBoundsCheckInfo hc_emlrtBCI = {
      -1,                                                            // iFirst
      -1,                                                            // iLast
      84,                                                            // lineNo
      15,                                                            // colNo
      "J_tim",                                                       // aName
      "synchronize_trajectory",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/synchronize_trajectory.m", // pName
      0 // checkKind
  };
  static rtBoundsCheckInfo hd_emlrtBCI = {
      -1,                                                            // iFirst
      -1,                                                            // iLast
      125,                                                           // lineNo
      15,                                                            // colNo
      "t_red",                                                       // aName
      "synchronize_trajectory",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/synchronize_trajectory.m", // pName
      0 // checkKind
  };
  static rtBoundsCheckInfo he_emlrtBCI = {
      -1,                                                            // iFirst
      -1,                                                            // iLast
      133,                                                           // lineNo
      30,                                                            // colNo
      "J_opt",                                                       // aName
      "synchronize_trajectory",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/synchronize_trajectory.m", // pName
      0 // checkKind
  };
  static rtBoundsCheckInfo hf_emlrtBCI = {
      -1,                                                            // iFirst
      -1,                                                            // iLast
      166,                                                           // lineNo
      45,                                                            // colNo
      "V_wayp",                                                      // aName
      "synchronize_trajectory",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/synchronize_trajectory.m", // pName
      0 // checkKind
  };
  static rtBoundsCheckInfo hg_emlrtBCI = {
      -1,                                                            // iFirst
      -1,                                                            // iLast
      171,                                                           // lineNo
      250,                                                           // colNo
      "A_max",                                                       // aName
      "synchronize_trajectory",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/synchronize_trajectory.m", // pName
      0 // checkKind
  };
  static rtBoundsCheckInfo hh_emlrtBCI = {
      -1,                                                            // iFirst
      -1,                                                            // iLast
      188,                                                           // lineNo
      21,                                                            // colNo
      "t",                                                           // aName
      "synchronize_trajectory",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/synchronize_trajectory.m", // pName
      0 // checkKind
  };
  static rtBoundsCheckInfo hi_emlrtBCI = {
      1,                                                             // iFirst
      100,                                                           // iLast
      198,                                                           // lineNo
      39,                                                            // colNo
      "solution_opt_test_all",                                       // aName
      "synchronize_trajectory",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/synchronize_trajectory.m", // pName
      0 // checkKind
  };
  static rtBoundsCheckInfo hj_emlrtBCI = {
      -1,                                                            // iFirst
      -1,                                                            // iLast
      228,                                                           // lineNo
      52,                                                            // colNo
      "J_opt{index_axis,:}",                                         // aName
      "synchronize_trajectory",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/synchronize_trajectory.m", // pName
      0 // checkKind
  };
  static rtBoundsCheckInfo ic_emlrtBCI = {
      -1,                                                            // iFirst
      -1,                                                            // iLast
      62,                                                            // lineNo
      27,                                                            // colNo
      "t_out",                                                       // aName
      "synchronize_trajectory",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/synchronize_trajectory.m", // pName
      0 // checkKind
  };
  static rtBoundsCheckInfo id_emlrtBCI = {
      -1,                                                            // iFirst
      -1,                                                            // iLast
      125,                                                           // lineNo
      9,                                                             // colNo
      "t_red",                                                       // aName
      "synchronize_trajectory",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/synchronize_trajectory.m", // pName
      0 // checkKind
  };
  static rtBoundsCheckInfo ie_emlrtBCI = {
      -1,                                                            // iFirst
      -1,                                                            // iLast
      133,                                                           // lineNo
      50,                                                            // colNo
      "solution_opt",                                                // aName
      "synchronize_trajectory",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/synchronize_trajectory.m", // pName
      0 // checkKind
  };
  static rtBoundsCheckInfo if_emlrtBCI = {
      -1,                                                            // iFirst
      -1,                                                            // iLast
      166,                                                           // lineNo
      64,                                                            // colNo
      "A_wayp",                                                      // aName
      "synchronize_trajectory",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/synchronize_trajectory.m", // pName
      0 // checkKind
  };
  static rtBoundsCheckInfo ig_emlrtBCI = {
      -1,                                                            // iFirst
      -1,                                                            // iLast
      171,                                                           // lineNo
      268,                                                           // colNo
      "A_min",                                                       // aName
      "synchronize_trajectory",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/synchronize_trajectory.m", // pName
      0 // checkKind
  };
  static rtBoundsCheckInfo ih_emlrtBCI = {
      -1,                                                            // iFirst
      -1,                                                            // iLast
      189,                                                           // lineNo
      51,                                                            // colNo
      "J_opt_test",                                                  // aName
      "synchronize_trajectory",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/synchronize_trajectory.m", // pName
      0 // checkKind
  };
  static rtBoundsCheckInfo ii_emlrtBCI = {
      1,                                                             // iFirst
      100,                                                           // iLast
      198,                                                           // lineNo
      43,                                                            // colNo
      "solution_opt_test_all",                                       // aName
      "synchronize_trajectory",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/synchronize_trajectory.m", // pName
      0 // checkKind
  };
  static rtBoundsCheckInfo ij_emlrtBCI = {
      -1,                                                            // iFirst
      -1,                                                            // iLast
      228,                                                           // lineNo
      13,                                                            // colNo
      "J",                                                           // aName
      "synchronize_trajectory",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/synchronize_trajectory.m", // pName
      0 // checkKind
  };
  static rtBoundsCheckInfo jc_emlrtBCI = {
      -1,                                                            // iFirst
      -1,                                                            // iLast
      88,                                                            // lineNo
      15,                                                            // colNo
      "t_out",                                                       // aName
      "synchronize_trajectory",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/synchronize_trajectory.m", // pName
      0 // checkKind
  };
  static rtBoundsCheckInfo jd_emlrtBCI = {
      -1,                                                            // iFirst
      -1,                                                            // iLast
      125,                                                           // lineNo
      33,                                                            // colNo
      "J_red",                                                       // aName
      "synchronize_trajectory",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/synchronize_trajectory.m", // pName
      0 // checkKind
  };
  static rtBoundsCheckInfo je_emlrtBCI = {
      -1,                                                            // iFirst
      -1,                                                            // iLast
      135,                                                           // lineNo
      33,                                                            // colNo
      "solution_opt",                                                // aName
      "synchronize_trajectory",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/synchronize_trajectory.m", // pName
      0 // checkKind
  };
  static rtBoundsCheckInfo jf_emlrtBCI = {
      -1,                                                            // iFirst
      -1,                                                            // iLast
      168,                                                           // lineNo
      113,                                                           // colNo
      "P_init",                                                      // aName
      "synchronize_trajectory",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/synchronize_trajectory.m", // pName
      0 // checkKind
  };
  static rtBoundsCheckInfo jg_emlrtBCI = {
      -1,                                                            // iFirst
      -1,                                                            // iLast
      171,                                                           // lineNo
      286,                                                           // colNo
      "J_max",                                                       // aName
      "synchronize_trajectory",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/synchronize_trajectory.m", // pName
      0 // checkKind
  };
  static rtBoundsCheckInfo jh_emlrtBCI = {
      -1,                                                            // iFirst
      -1,                                                            // iLast
      189,                                                           // lineNo
      65,                                                            // colNo
      "J_opt_test{index_axis,:}",                                    // aName
      "synchronize_trajectory",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/synchronize_trajectory.m", // pName
      0 // checkKind
  };
  static rtBoundsCheckInfo ji_emlrtBCI = {
      -1,                                                            // iFirst
      -1,                                                            // iLast
      209,                                                           // lineNo
      46,                                                            // colNo
      "t_opt",                                                       // aName
      "synchronize_trajectory",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/synchronize_trajectory.m", // pName
      0 // checkKind
  };
  static rtBoundsCheckInfo jj_emlrtBCI = {
      -1,                                                            // iFirst
      -1,                                                            // iLast
      229,                                                           // lineNo
      55,                                                            // colNo
      "solution_opt",                                                // aName
      "synchronize_trajectory",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/synchronize_trajectory.m", // pName
      0 // checkKind
  };
  static rtBoundsCheckInfo kc_emlrtBCI = {
      -1,                                                            // iFirst
      -1,                                                            // iLast
      62,                                                            // lineNo
      27,                                                            // colNo
      "J_out",                                                       // aName
      "synchronize_trajectory",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/synchronize_trajectory.m", // pName
      0 // checkKind
  };
  static rtBoundsCheckInfo kd_emlrtBCI = {
      -1,                                                            // iFirst
      -1,                                                            // iLast
      125,                                                           // lineNo
      27,                                                            // colNo
      "J_red",                                                       // aName
      "synchronize_trajectory",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/synchronize_trajectory.m", // pName
      0 // checkKind
  };
  static rtBoundsCheckInfo ke_emlrtBCI = {
      -1,                                                            // iFirst
      -1,                                                            // iLast
      137,                                                           // lineNo
      19,                                                            // colNo
      "t_opt",                                                       // aName
      "synchronize_trajectory",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/synchronize_trajectory.m", // pName
      0 // checkKind
  };
  static rtBoundsCheckInfo kf_emlrtBCI = {
      -1,                                                            // iFirst
      -1,                                                            // iLast
      168,                                                           // lineNo
      132,                                                           // colNo
      "V_init",                                                      // aName
      "synchronize_trajectory",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/synchronize_trajectory.m", // pName
      0 // checkKind
  };
  static rtBoundsCheckInfo kg_emlrtBCI = {
      -1,                                                            // iFirst
      -1,                                                            // iLast
      171,                                                           // lineNo
      304,                                                           // colNo
      "J_min",                                                       // aName
      "synchronize_trajectory",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/synchronize_trajectory.m", // pName
      0 // checkKind
  };
  static rtBoundsCheckInfo kh_emlrtBCI = {
      -1,                                                            // iFirst
      -1,                                                            // iLast
      189,                                                           // lineNo
      21,                                                            // colNo
      "J",                                                           // aName
      "synchronize_trajectory",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/synchronize_trajectory.m", // pName
      0 // checkKind
  };
  static rtBoundsCheckInfo ki_emlrtBCI = {
      -1,                                                            // iFirst
      -1,                                                            // iLast
      209,                                                           // lineNo
      60,                                                            // colNo
      "t_opt{index_axis,:}",                                         // aName
      "synchronize_trajectory",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/synchronize_trajectory.m", // pName
      0 // checkKind
  };
  static rtBoundsCheckInfo kj_emlrtBCI = {
      -1,                                                            // iFirst
      -1,                                                            // iLast
      229,                                                           // lineNo
      69,                                                            // colNo
      "solution_opt{index_axis,:}",                                  // aName
      "synchronize_trajectory",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/synchronize_trajectory.m", // pName
      0 // checkKind
  };
  static rtBoundsCheckInfo lc_emlrtBCI = {
      -1,                                                            // iFirst
      -1,                                                            // iLast
      92,                                                            // lineNo
      15,                                                            // colNo
      "J_out",                                                       // aName
      "synchronize_trajectory",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/synchronize_trajectory.m", // pName
      0 // checkKind
  };
  static rtBoundsCheckInfo ld_emlrtBCI = {
      -1,                                                            // iFirst
      -1,                                                            // iLast
      126,                                                           // lineNo
      165,                                                           // colNo
      "t_red",                                                       // aName
      "synchronize_trajectory",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/synchronize_trajectory.m", // pName
      0 // checkKind
  };
  static rtBoundsCheckInfo le_emlrtBCI = {
      -1,                                                            // iFirst
      -1,                                                            // iLast
      137,                                                           // lineNo
      13,                                                            // colNo
      "t_opt",                                                       // aName
      "synchronize_trajectory",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/synchronize_trajectory.m", // pName
      0 // checkKind
  };
  static rtBoundsCheckInfo lf_emlrtBCI = {
      -1,                                                            // iFirst
      -1,                                                            // iLast
      168,                                                           // lineNo
      151,                                                           // colNo
      "A_init",                                                      // aName
      "synchronize_trajectory",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/synchronize_trajectory.m", // pName
      0 // checkKind
  };
  static rtBoundsCheckInfo lg_emlrtBCI = {
      -1,                                                            // iFirst
      -1,                                                            // iLast
      171,                                                           // lineNo
      322,                                                           // colNo
      "t_sync",                                                      // aName
      "synchronize_trajectory",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/synchronize_trajectory.m", // pName
      0 // checkKind
  };
  static rtBoundsCheckInfo lh_emlrtBCI = {
      -1,                                                            // iFirst
      -1,                                                            // iLast
      190,                                                           // lineNo
      68,                                                            // colNo
      "solution_opt_test",                                           // aName
      "synchronize_trajectory",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/synchronize_trajectory.m", // pName
      0 // checkKind
  };
  static rtBoundsCheckInfo li_emlrtBCI = {
      -1,                                                            // iFirst
      -1,                                                            // iLast
      209,                                                           // lineNo
      21,                                                            // colNo
      "t",                                                           // aName
      "synchronize_trajectory",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/synchronize_trajectory.m", // pName
      0 // checkKind
  };
  static rtBoundsCheckInfo lj_emlrtBCI = {
      -1,                                                            // iFirst
      -1,                                                            // iLast
      229,                                                           // lineNo
      13,                                                            // colNo
      "solution_out",                                                // aName
      "synchronize_trajectory",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/synchronize_trajectory.m", // pName
      0 // checkKind
  };
  static rtBoundsCheckInfo mc_emlrtBCI = {
      -1,                                                            // iFirst
      -1,                                                            // iLast
      96,                                                            // lineNo
      9,                                                             // colNo
      "t",                                                           // aName
      "synchronize_trajectory",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/synchronize_trajectory.m", // pName
      0 // checkKind
  };
  static rtBoundsCheckInfo md_emlrtBCI = {
      -1,                                                            // iFirst
      -1,                                                            // iLast
      126,                                                           // lineNo
      183,                                                           // colNo
      "J_red",                                                       // aName
      "synchronize_trajectory",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/synchronize_trajectory.m", // pName
      0 // checkKind
  };
  static rtBoundsCheckInfo me_emlrtBCI = {
      -1,                                                            // iFirst
      -1,                                                            // iLast
      138,                                                           // lineNo
      19,                                                            // colNo
      "J_opt",                                                       // aName
      "synchronize_trajectory",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/synchronize_trajectory.m", // pName
      0 // checkKind
  };
  static rtBoundsCheckInfo mf_emlrtBCI = {
      -1,                                                            // iFirst
      -1,                                                            // iLast
      168,                                                           // lineNo
      170,                                                           // colNo
      "P_wayp",                                                      // aName
      "synchronize_trajectory",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/synchronize_trajectory.m", // pName
      0 // checkKind
  };
  static rtBoundsCheckInfo mg_emlrtBCI = {
      -1,                                                            // iFirst
      -1,                                                            // iLast
      171,                                                           // lineNo
      341,                                                           // colNo
      "b_sync_V",                                                    // aName
      "synchronize_trajectory",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/synchronize_trajectory.m", // pName
      0 // checkKind
  };
  static rtBoundsCheckInfo mh_emlrtBCI = {
      -1,                                                            // iFirst
      -1,                                                            // iLast
      190,                                                           // lineNo
      82,                                                            // colNo
      "solution_opt_test{index_axis,:}",                             // aName
      "synchronize_trajectory",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/synchronize_trajectory.m", // pName
      0 // checkKind
  };
  static rtBoundsCheckInfo mi_emlrtBCI = {
      -1,                                                            // iFirst
      -1,                                                            // iLast
      210,                                                           // lineNo
      46,                                                            // colNo
      "J_opt",                                                       // aName
      "synchronize_trajectory",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/synchronize_trajectory.m", // pName
      0 // checkKind
  };
  static rtBoundsCheckInfo mj_emlrtBCI = {
      -1,                                                            // iFirst
      -1,                                                            // iLast
      234,                                                           // lineNo
      36,                                                            // colNo
      "t_red",                                                       // aName
      "synchronize_trajectory",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/synchronize_trajectory.m", // pName
      0 // checkKind
  };
  static rtBoundsCheckInfo nb_emlrtBCI = {
      -1,                                                            // iFirst
      -1,                                                            // iLast
      54,                                                            // lineNo
      13,                                                            // colNo
      "J_max",                                                       // aName
      "synchronize_trajectory",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/synchronize_trajectory.m", // pName
      0 // checkKind
  };
  static rtBoundsCheckInfo nc_emlrtBCI = {
      -1,                                                            // iFirst
      -1,                                                            // iLast
      100,                                                           // lineNo
      9,                                                             // colNo
      "J",                                                           // aName
      "synchronize_trajectory",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/synchronize_trajectory.m", // pName
      0 // checkKind
  };
  static rtBoundsCheckInfo nd_emlrtBCI = {
      -1,                                                            // iFirst
      -1,                                                            // iLast
      126,                                                           // lineNo
      86,                                                            // colNo
      "P_init",                                                      // aName
      "synchronize_trajectory",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/synchronize_trajectory.m", // pName
      0 // checkKind
  };
  static rtBoundsCheckInfo ne_emlrtBCI = {
      -1,                                                            // iFirst
      -1,                                                            // iLast
      138,                                                           // lineNo
      13,                                                            // colNo
      "J_opt",                                                       // aName
      "synchronize_trajectory",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/synchronize_trajectory.m", // pName
      0 // checkKind
  };
  static rtBoundsCheckInfo nf_emlrtBCI = {
      -1,                                                            // iFirst
      -1,                                                            // iLast
      168,                                                           // lineNo
      189,                                                           // colNo
      "V_wayp",                                                      // aName
      "synchronize_trajectory",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/synchronize_trajectory.m", // pName
      0 // checkKind
  };
  static rtBoundsCheckInfo ng_emlrtBCI = {
      -1,                                                            // iFirst
      -1,                                                            // iLast
      171,                                                           // lineNo
      362,                                                           // colNo
      "b_sync_A",                                                    // aName
      "synchronize_trajectory",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/synchronize_trajectory.m", // pName
      0 // checkKind
  };
  static rtBoundsCheckInfo nh_emlrtBCI = {
      -1,                                                            // iFirst
      -1,                                                            // iLast
      190,                                                           // lineNo
      21,                                                            // colNo
      "solution_out",                                                // aName
      "synchronize_trajectory",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/synchronize_trajectory.m", // pName
      0 // checkKind
  };
  static rtBoundsCheckInfo ni_emlrtBCI = {
      -1,                                                            // iFirst
      -1,                                                            // iLast
      210,                                                           // lineNo
      60,                                                            // colNo
      "J_opt{index_axis,:}",                                         // aName
      "synchronize_trajectory",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/synchronize_trajectory.m", // pName
      0 // checkKind
  };
  static rtBoundsCheckInfo nj_emlrtBCI = {
      -1,                                                            // iFirst
      -1,                                                            // iLast
      234,                                                           // lineNo
      50,                                                            // colNo
      "t",                                                           // aName
      "synchronize_trajectory",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/synchronize_trajectory.m", // pName
      0 // checkKind
  };
  static rtBoundsCheckInfo ob_emlrtBCI = {
      -1,                                                            // iFirst
      -1,                                                            // iLast
      54,                                                            // lineNo
      35,                                                            // colNo
      "J_min",                                                       // aName
      "synchronize_trajectory",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/synchronize_trajectory.m", // pName
      0 // checkKind
  };
  static rtBoundsCheckInfo oc_emlrtBCI = {
      -1,                                                            // iFirst
      -1,                                                            // iLast
      62,                                                            // lineNo
      27,                                                            // colNo
      "solution_opt",                                                // aName
      "synchronize_trajectory",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/synchronize_trajectory.m", // pName
      0 // checkKind
  };
  static rtBoundsCheckInfo od_emlrtBCI = {
      -1,                                                            // iFirst
      -1,                                                            // iLast
      126,                                                           // lineNo
      105,                                                           // colNo
      "V_init",                                                      // aName
      "synchronize_trajectory",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/synchronize_trajectory.m", // pName
      0 // checkKind
  };
  static rtBoundsCheckInfo oe_emlrtBCI = {
      -1,                                                            // iFirst
      -1,                                                            // iLast
      139,                                                           // lineNo
      13,                                                            // colNo
      "solution_opt",                                                // aName
      "synchronize_trajectory",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/synchronize_trajectory.m", // pName
      0 // checkKind
  };
  static rtBoundsCheckInfo of_emlrtBCI = {
      -1,                                                            // iFirst
      -1,                                                            // iLast
      168,                                                           // lineNo
      208,                                                           // colNo
      "A_wayp",                                                      // aName
      "synchronize_trajectory",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/synchronize_trajectory.m", // pName
      0 // checkKind
  };
  static rtBoundsCheckInfo og_emlrtBCI = {
      -1,                                                            // iFirst
      -1,                                                            // iLast
      171,                                                           // lineNo
      383,                                                           // colNo
      "b_sync_J",                                                    // aName
      "synchronize_trajectory",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/synchronize_trajectory.m", // pName
      0 // checkKind
  };
  static rtBoundsCheckInfo oh_emlrtBCI = {
      -1,                                                            // iFirst
      -1,                                                            // iLast
      179,                                                           // lineNo
      60,                                                            // colNo
      "t_opt_test",                                                  // aName
      "synchronize_trajectory",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/synchronize_trajectory.m", // pName
      0 // checkKind
  };
  static rtBoundsCheckInfo oi_emlrtBCI = {
      -1,                                                            // iFirst
      -1,                                                            // iLast
      210,                                                           // lineNo
      21,                                                            // colNo
      "J",                                                           // aName
      "synchronize_trajectory",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/synchronize_trajectory.m", // pName
      0 // checkKind
  };
  static rtBoundsCheckInfo oj_emlrtBCI = {
      -1,                                                            // iFirst
      -1,                                                            // iLast
      234,                                                           // lineNo
      9,                                                             // colNo
      "t_out",                                                       // aName
      "synchronize_trajectory",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/synchronize_trajectory.m", // pName
      0 // checkKind
  };
  static rtBoundsCheckInfo pb_emlrtBCI = {
      -1,                                                            // iFirst
      -1,                                                            // iLast
      55,                                                            // lineNo
      35,                                                            // colNo
      "J_max",                                                       // aName
      "synchronize_trajectory",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/synchronize_trajectory.m", // pName
      0 // checkKind
  };
  static rtBoundsCheckInfo pc_emlrtBCI = {
      -1,                                                            // iFirst
      -1,                                                            // iLast
      105,                                                           // lineNo
      9,                                                             // colNo
      "solution_opt",                                                // aName
      "synchronize_trajectory",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/synchronize_trajectory.m", // pName
      0 // checkKind
  };
  static rtBoundsCheckInfo pd_emlrtBCI = {
      -1,                                                            // iFirst
      -1,                                                            // iLast
      126,                                                           // lineNo
      124,                                                           // colNo
      "A_init",                                                      // aName
      "synchronize_trajectory",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/synchronize_trajectory.m", // pName
      0 // checkKind
  };
  static rtBoundsCheckInfo pe_emlrtBCI = {
      -1,                                                            // iFirst
      -1,                                                            // iLast
      151,                                                           // lineNo
      61,                                                            // colNo
      "t_opt_test",                                                  // aName
      "synchronize_trajectory",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/synchronize_trajectory.m", // pName
      0 // checkKind
  };
  static rtBoundsCheckInfo pf_emlrtBCI = {
      -1,                                                            // iFirst
      -1,                                                            // iLast
      168,                                                           // lineNo
      227,                                                           // colNo
      "V_max",                                                       // aName
      "synchronize_trajectory",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/synchronize_trajectory.m", // pName
      0 // checkKind
  };
  static rtBoundsCheckInfo pg_emlrtBCI = {
      -1,                                                            // iFirst
      -1,                                                            // iLast
      171,                                                           // lineNo
      404,                                                           // colNo
      "b_sync_W",                                                    // aName
      "synchronize_trajectory",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/synchronize_trajectory.m", // pName
      0 // checkKind
  };
  static rtBoundsCheckInfo ph_emlrtBCI = {
      -1,                                                            // iFirst
      -1,                                                            // iLast
      180,                                                           // lineNo
      60,                                                            // colNo
      "J_opt_test",                                                  // aName
      "synchronize_trajectory",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/synchronize_trajectory.m", // pName
      0 // checkKind
  };
  static rtBoundsCheckInfo pi_emlrtBCI = {
      -1,                                                            // iFirst
      -1,                                                            // iLast
      211,                                                           // lineNo
      63,                                                            // colNo
      "solution_opt",                                                // aName
      "synchronize_trajectory",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/synchronize_trajectory.m", // pName
      0 // checkKind
  };
  static rtBoundsCheckInfo pj_emlrtBCI = {
      -1,                                                            // iFirst
      -1,                                                            // iLast
      235,                                                           // lineNo
      36,                                                            // colNo
      "J_red",                                                       // aName
      "synchronize_trajectory",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/synchronize_trajectory.m", // pName
      0 // checkKind
  };
  static rtBoundsCheckInfo qb_emlrtBCI = {
      -1,                                                            // iFirst
      -1,                                                            // iLast
      55,                                                            // lineNo
      14,                                                            // colNo
      "J_min",                                                       // aName
      "synchronize_trajectory",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/synchronize_trajectory.m", // pName
      0 // checkKind
  };
  static rtBoundsCheckInfo qc_emlrtBCI = {
      -1,                                                            // iFirst
      -1,                                                            // iLast
      62,                                                            // lineNo
      27,                                                            // colNo
      "solution_tim",                                                // aName
      "synchronize_trajectory",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/synchronize_trajectory.m", // pName
      0 // checkKind
  };
  static rtBoundsCheckInfo qd_emlrtBCI = {
      -1,                                                            // iFirst
      -1,                                                            // iLast
      126,                                                           // lineNo
      9,                                                             // colNo
      "P_init",                                                      // aName
      "synchronize_trajectory",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/synchronize_trajectory.m", // pName
      0 // checkKind
  };
  static rtBoundsCheckInfo qe_emlrtBCI = {
      -1,                                                            // iFirst
      -1,                                                            // iLast
      157,                                                           // lineNo
      48,                                                            // colNo
      "t_opt_test",                                                  // aName
      "synchronize_trajectory",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/synchronize_trajectory.m", // pName
      0 // checkKind
  };
  static rtBoundsCheckInfo qf_emlrtBCI = {
      -1,                                                            // iFirst
      -1,                                                            // iLast
      168,                                                           // lineNo
      245,                                                           // colNo
      "V_min",                                                       // aName
      "synchronize_trajectory",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/synchronize_trajectory.m", // pName
      0 // checkKind
  };
  static rtBoundsCheckInfo qg_emlrtBCI = {
      -1,                                                            // iFirst
      -1,                                                            // iLast
      171,                                                           // lineNo
      425,                                                           // colNo
      "direction",                                                   // aName
      "synchronize_trajectory",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/synchronize_trajectory.m", // pName
      0 // checkKind
  };
  static rtBoundsCheckInfo qh_emlrtBCI = {
      -1,                                                            // iFirst
      -1,                                                            // iLast
      181,                                                           // lineNo
      67,                                                            // colNo
      "solution_opt_test",                                           // aName
      "synchronize_trajectory",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/synchronize_trajectory.m", // pName
      0 // checkKind
  };
  static rtBoundsCheckInfo qi_emlrtBCI = {
      -1,                                                            // iFirst
      -1,                                                            // iLast
      211,                                                           // lineNo
      77,                                                            // colNo
      "solution_opt{index_axis,:}",                                  // aName
      "synchronize_trajectory",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/synchronize_trajectory.m", // pName
      0 // checkKind
  };
  static rtBoundsCheckInfo qj_emlrtBCI = {
      -1,                                                            // iFirst
      -1,                                                            // iLast
      235,                                                           // lineNo
      50,                                                            // colNo
      "J",                                                           // aName
      "synchronize_trajectory",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/synchronize_trajectory.m", // pName
      0 // checkKind
  };
  static rtBoundsCheckInfo rb_emlrtBCI = {
      -1,                                                            // iFirst
      -1,                                                            // iLast
      57,                                                            // lineNo
      13,                                                            // colNo
      "A_max",                                                       // aName
      "synchronize_trajectory",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/synchronize_trajectory.m", // pName
      0 // checkKind
  };
  static rtBoundsCheckInfo rc_emlrtBCI = {
      -1,                                                            // iFirst
      -1,                                                            // iLast
      109,                                                           // lineNo
      9,                                                             // colNo
      "solution_tim",                                                // aName
      "synchronize_trajectory",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/synchronize_trajectory.m", // pName
      0 // checkKind
  };
  static rtBoundsCheckInfo rd_emlrtBCI = {
      -1,                                                            // iFirst
      -1,                                                            // iLast
      126,                                                           // lineNo
      28,                                                            // colNo
      "V_init",                                                      // aName
      "synchronize_trajectory",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/synchronize_trajectory.m", // pName
      0 // checkKind
  };
  static rtBoundsCheckInfo re_emlrtBCI = {
      -1,                                                            // iFirst
      -1,                                                            // iLast
      157,                                                           // lineNo
      60,                                                            // colNo
      "t_opt_test{index_axis}",                                      // aName
      "synchronize_trajectory",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/synchronize_trajectory.m", // pName
      0 // checkKind
  };
  static rtBoundsCheckInfo rf_emlrtBCI = {
      -1,                                                            // iFirst
      -1,                                                            // iLast
      168,                                                           // lineNo
      263,                                                           // colNo
      "A_max",                                                       // aName
      "synchronize_trajectory",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/synchronize_trajectory.m", // pName
      0 // checkKind
  };
  static rtBoundsCheckInfo rg_emlrtBCI = {
      -1,                                                            // iFirst
      -1,                                                            // iLast
      171,                                                           // lineNo
      22,                                                            // colNo
      "t_tim",                                                       // aName
      "synchronize_trajectory",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/synchronize_trajectory.m", // pName
      0 // checkKind
  };
  static rtBoundsCheckInfo rh_emlrtBCI = {
      -1,                                                            // iFirst
      -1,                                                            // iLast
      182,                                                           // lineNo
      36,                                                            // colNo
      "t_opt_test",                                                  // aName
      "synchronize_trajectory",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/synchronize_trajectory.m", // pName
      0 // checkKind
  };
  static rtBoundsCheckInfo ri_emlrtBCI = {
      -1,                                                            // iFirst
      -1,                                                            // iLast
      211,                                                           // lineNo
      21,                                                            // colNo
      "solution_out",                                                // aName
      "synchronize_trajectory",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/synchronize_trajectory.m", // pName
      0 // checkKind
  };
  static rtBoundsCheckInfo rj_emlrtBCI = {
      -1,                                                            // iFirst
      -1,                                                            // iLast
      235,                                                           // lineNo
      9,                                                             // colNo
      "J_out",                                                       // aName
      "synchronize_trajectory",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/synchronize_trajectory.m", // pName
      0 // checkKind
  };
  static rtBoundsCheckInfo sb_emlrtBCI = {
      -1,                                                            // iFirst
      -1,                                                            // iLast
      57,                                                            // lineNo
      35,                                                            // colNo
      "A_min",                                                       // aName
      "synchronize_trajectory",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/synchronize_trajectory.m", // pName
      0 // checkKind
  };
  static rtBoundsCheckInfo sc_emlrtBCI = {
      -1,                                                            // iFirst
      -1,                                                            // iLast
      113,                                                           // lineNo
      25,                                                            // colNo
      "J_max",                                                       // aName
      "synchronize_trajectory",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/synchronize_trajectory.m", // pName
      0 // checkKind
  };
  static rtBoundsCheckInfo sd_emlrtBCI = {
      -1,                                                            // iFirst
      -1,                                                            // iLast
      126,                                                           // lineNo
      47,                                                            // colNo
      "A_init",                                                      // aName
      "synchronize_trajectory",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/synchronize_trajectory.m", // pName
      0 // checkKind
  };
  static rtBoundsCheckInfo se_emlrtBCI = {
      -1,                                                            // iFirst
      -1,                                                            // iLast
      157,                                                           // lineNo
      17,                                                            // colNo
      "T",                                                           // aName
      "synchronize_trajectory",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/synchronize_trajectory.m", // pName
      0 // checkKind
  };
  static rtBoundsCheckInfo sf_emlrtBCI = {
      -1,                                                            // iFirst
      -1,                                                            // iLast
      168,                                                           // lineNo
      281,                                                           // colNo
      "A_min",                                                       // aName
      "synchronize_trajectory",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/synchronize_trajectory.m", // pName
      0 // checkKind
  };
  static rtBoundsCheckInfo sg_emlrtBCI = {
      -1,                                                            // iFirst
      -1,                                                            // iLast
      171,                                                           // lineNo
      42,                                                            // colNo
      "J_tim",                                                       // aName
      "synchronize_trajectory",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/synchronize_trajectory.m", // pName
      0 // checkKind
  };
  static rtBoundsCheckInfo sh_emlrtBCI = {
      -1,                                                            // iFirst
      -1,                                                            // iLast
      182,                                                           // lineNo
      48,                                                            // colNo
      "t_opt_test{index_axis}",                                      // aName
      "synchronize_trajectory",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/synchronize_trajectory.m", // pName
      0 // checkKind
  };
  static rtBoundsCheckInfo si_emlrtBCI = {
      -1,                                                            // iFirst
      -1,                                                            // iLast
      217,                                                           // lineNo
      46,                                                            // colNo
      "t_opt",                                                       // aName
      "synchronize_trajectory",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/synchronize_trajectory.m", // pName
      0 // checkKind
  };
  static rtBoundsCheckInfo tb_emlrtBCI = {
      -1,                                                            // iFirst
      -1,                                                            // iLast
      58,                                                            // lineNo
      33,                                                            // colNo
      "A_min",                                                       // aName
      "synchronize_trajectory",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/synchronize_trajectory.m", // pName
      0 // checkKind
  };
  static rtBoundsCheckInfo tc_emlrtBCI = {
      -1,                                                            // iFirst
      -1,                                                            // iLast
      114,                                                           // lineNo
      13,                                                            // colNo
      "J_max",                                                       // aName
      "synchronize_trajectory",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/synchronize_trajectory.m", // pName
      0 // checkKind
  };
  static rtBoundsCheckInfo td_emlrtBCI = {
      -1,                                                            // iFirst
      -1,                                                            // iLast
      133,                                                           // lineNo
      88,                                                            // colNo
      "P_init",                                                      // aName
      "synchronize_trajectory",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/synchronize_trajectory.m", // pName
      0 // checkKind
  };
  static rtBoundsCheckInfo te_emlrtBCI = {
      -1,                                                            // iFirst
      -1,                                                            // iLast
      160,                                                           // lineNo
      22,                                                            // colNo
      "T",                                                           // aName
      "synchronize_trajectory",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/synchronize_trajectory.m", // pName
      0 // checkKind
  };
  static rtBoundsCheckInfo tf_emlrtBCI = {
      -1,                                                            // iFirst
      -1,                                                            // iLast
      168,                                                           // lineNo
      299,                                                           // colNo
      "J_max",                                                       // aName
      "synchronize_trajectory",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/synchronize_trajectory.m", // pName
      0 // checkKind
  };
  static rtBoundsCheckInfo tg_emlrtBCI = {
      -1,                                                            // iFirst
      -1,                                                            // iLast
      171,                                                           // lineNo
      62,                                                            // colNo
      "solution_tim",                                                // aName
      "synchronize_trajectory",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/synchronize_trajectory.m", // pName
      0 // checkKind
  };
  static rtBoundsCheckInfo th_emlrtBCI = {
      -1,                                                            // iFirst
      -1,                                                            // iLast
      182,                                                           // lineNo
      25,                                                            // colNo
      "t_opt_test",                                                  // aName
      "synchronize_trajectory",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/synchronize_trajectory.m", // pName
      0 // checkKind
  };
  static rtBoundsCheckInfo ti_emlrtBCI = {
      -1,                                                            // iFirst
      -1,                                                            // iLast
      217,                                                           // lineNo
      60,                                                            // colNo
      "t_opt{index_axis,:}",                                         // aName
      "synchronize_trajectory",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/synchronize_trajectory.m", // pName
      0 // checkKind
  };
  static rtBoundsCheckInfo ub_emlrtBCI = {
      -1,                                                            // iFirst
      -1,                                                            // iLast
      58,                                                            // lineNo
      13,                                                            // colNo
      "A_min",                                                       // aName
      "synchronize_trajectory",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/synchronize_trajectory.m", // pName
      0 // checkKind
  };
  static rtBoundsCheckInfo uc_emlrtBCI = {
      -1,                                                            // iFirst
      -1,                                                            // iLast
      116,                                                           // lineNo
      25,                                                            // colNo
      "J_min",                                                       // aName
      "synchronize_trajectory",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/synchronize_trajectory.m", // pName
      0 // checkKind
  };
  static rtBoundsCheckInfo ud_emlrtBCI = {
      -1,                                                            // iFirst
      -1,                                                            // iLast
      133,                                                           // lineNo
      107,                                                           // colNo
      "V_init",                                                      // aName
      "synchronize_trajectory",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/synchronize_trajectory.m", // pName
      0 // checkKind
  };
  static rtBoundsCheckInfo ue_emlrtBCI = {
      -1,                                                            // iFirst
      -1,                                                            // iLast
      160,                                                           // lineNo
      49,                                                            // colNo
      "T_catch_up",                                                  // aName
      "synchronize_trajectory",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/synchronize_trajectory.m", // pName
      0 // checkKind
  };
  static rtBoundsCheckInfo uf_emlrtBCI = {
      -1,                                                            // iFirst
      -1,                                                            // iLast
      168,                                                           // lineNo
      317,                                                           // colNo
      "J_min",                                                       // aName
      "synchronize_trajectory",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/synchronize_trajectory.m", // pName
      0 // checkKind
  };
  static rtBoundsCheckInfo ug_emlrtBCI = {
      -1,                                                            // iFirst
      -1,                                                            // iLast
      172,                                                           // lineNo
      47,                                                            // colNo
      "solution_tim",                                                // aName
      "synchronize_trajectory",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/synchronize_trajectory.m", // pName
      0 // checkKind
  };
  static rtBoundsCheckInfo uh_emlrtBCI = {
      -1,                                                            // iFirst
      -1,                                                            // iLast
      183,                                                           // lineNo
      36,                                                            // colNo
      "J_opt_test",                                                  // aName
      "synchronize_trajectory",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/synchronize_trajectory.m", // pName
      0 // checkKind
  };
  static rtBoundsCheckInfo ui_emlrtBCI = {
      -1,                                                            // iFirst
      -1,                                                            // iLast
      217,                                                           // lineNo
      21,                                                            // colNo
      "t",                                                           // aName
      "synchronize_trajectory",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/synchronize_trajectory.m", // pName
      0 // checkKind
  };
  static rtBoundsCheckInfo vb_emlrtBCI = {
      -1,                                                            // iFirst
      -1,                                                            // iLast
      62,                                                            // lineNo
      27,                                                            // colNo
      "t_red",                                                       // aName
      "synchronize_trajectory",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/synchronize_trajectory.m", // pName
      0 // checkKind
  };
  static rtBoundsCheckInfo vc_emlrtBCI = {
      -1,                                                            // iFirst
      -1,                                                            // iLast
      117,                                                           // lineNo
      13,                                                            // colNo
      "J_min",                                                       // aName
      "synchronize_trajectory",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/synchronize_trajectory.m", // pName
      0 // checkKind
  };
  static rtBoundsCheckInfo vd_emlrtBCI = {
      -1,                                                            // iFirst
      -1,                                                            // iLast
      133,                                                           // lineNo
      126,                                                           // colNo
      "A_init",                                                      // aName
      "synchronize_trajectory",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/synchronize_trajectory.m", // pName
      0 // checkKind
  };
  static rtBoundsCheckInfo ve_emlrtBCI = {
      -1,                                                            // iFirst
      -1,                                                            // iLast
      163,                                                           // lineNo
      48,                                                            // colNo
      "b_sync_V",                                                    // aName
      "synchronize_trajectory",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/synchronize_trajectory.m", // pName
      0 // checkKind
  };
  static rtBoundsCheckInfo vf_emlrtBCI = {
      -1,                                                            // iFirst
      -1,                                                            // iLast
      168,                                                           // lineNo
      26,                                                            // colNo
      "t_opt_test",                                                  // aName
      "synchronize_trajectory",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/synchronize_trajectory.m", // pName
      0 // checkKind
  };
  static rtBoundsCheckInfo vg_emlrtBCI = {
      -1,                                                            // iFirst
      -1,                                                            // iLast
      174,                                                           // lineNo
      49,                                                            // colNo
      "t_tim",                                                       // aName
      "synchronize_trajectory",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/synchronize_trajectory.m", // pName
      0 // checkKind
  };
  static rtBoundsCheckInfo vh_emlrtBCI = {
      -1,                                                            // iFirst
      -1,                                                            // iLast
      183,                                                           // lineNo
      48,                                                            // colNo
      "J_opt_test{index_axis}",                                      // aName
      "synchronize_trajectory",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/synchronize_trajectory.m", // pName
      0 // checkKind
  };
  static rtBoundsCheckInfo vi_emlrtBCI = {
      -1,                                                            // iFirst
      -1,                                                            // iLast
      218,                                                           // lineNo
      46,                                                            // colNo
      "J_opt",                                                       // aName
      "synchronize_trajectory",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/synchronize_trajectory.m", // pName
      0 // checkKind
  };
  static rtBoundsCheckInfo wb_emlrtBCI = {
      -1,                                                            // iFirst
      -1,                                                            // iLast
      64,                                                            // lineNo
      15,                                                            // colNo
      "t_red",                                                       // aName
      "synchronize_trajectory",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/synchronize_trajectory.m", // pName
      0 // checkKind
  };
  static rtBoundsCheckInfo wc_emlrtBCI = {
      -1,                                                            // iFirst
      -1,                                                            // iLast
      125,                                                           // lineNo
      68,                                                            // colNo
      "P_init",                                                      // aName
      "synchronize_trajectory",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/synchronize_trajectory.m", // pName
      0 // checkKind
  };
  static rtBoundsCheckInfo wd_emlrtBCI = {
      -1,                                                            // iFirst
      -1,                                                            // iLast
      133,                                                           // lineNo
      145,                                                           // colNo
      "P_wayp",                                                      // aName
      "synchronize_trajectory",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/synchronize_trajectory.m", // pName
      0 // checkKind
  };
  static rtBoundsCheckInfo we_emlrtBCI = {
      -1,                                                            // iFirst
      -1,                                                            // iLast
      164,                                                           // lineNo
      25,                                                            // colNo
      "direction",                                                   // aName
      "synchronize_trajectory",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/synchronize_trajectory.m", // pName
      0 // checkKind
  };
  static rtBoundsCheckInfo wf_emlrtBCI = {
      -1,                                                            // iFirst
      -1,                                                            // iLast
      168,                                                           // lineNo
      49,                                                            // colNo
      "J_opt_test",                                                  // aName
      "synchronize_trajectory",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/synchronize_trajectory.m", // pName
      0 // checkKind
  };
  static rtBoundsCheckInfo wg_emlrtBCI = {
      -1,                                                            // iFirst
      -1,                                                            // iLast
      174,                                                           // lineNo
      61,                                                            // colNo
      "t_tim{index_axis}",                                           // aName
      "synchronize_trajectory",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/synchronize_trajectory.m", // pName
      0 // checkKind
  };
  static rtBoundsCheckInfo wh_emlrtBCI = {
      -1,                                                            // iFirst
      -1,                                                            // iLast
      183,                                                           // lineNo
      25,                                                            // colNo
      "J_opt_test",                                                  // aName
      "synchronize_trajectory",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/synchronize_trajectory.m", // pName
      0 // checkKind
  };
  static rtBoundsCheckInfo wi_emlrtBCI = {
      -1,                                                            // iFirst
      -1,                                                            // iLast
      218,                                                           // lineNo
      60,                                                            // colNo
      "J_opt{index_axis,:}",                                         // aName
      "synchronize_trajectory",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/synchronize_trajectory.m", // pName
      0 // checkKind
  };
  static rtBoundsCheckInfo xb_emlrtBCI = {
      -1,                                                            // iFirst
      -1,                                                            // iLast
      62,                                                            // lineNo
      27,                                                            // colNo
      "J_red",                                                       // aName
      "synchronize_trajectory",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/synchronize_trajectory.m", // pName
      0 // checkKind
  };
  static rtBoundsCheckInfo xc_emlrtBCI = {
      -1,                                                            // iFirst
      -1,                                                            // iLast
      125,                                                           // lineNo
      87,                                                            // colNo
      "V_init",                                                      // aName
      "synchronize_trajectory",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/synchronize_trajectory.m", // pName
      0 // checkKind
  };
  static rtBoundsCheckInfo xd_emlrtBCI = {
      -1,                                                            // iFirst
      -1,                                                            // iLast
      133,                                                           // lineNo
      164,                                                           // colNo
      "V_wayp",                                                      // aName
      "synchronize_trajectory",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/synchronize_trajectory.m", // pName
      0 // checkKind
  };
  static rtBoundsCheckInfo xe_emlrtBCI = {
      -1,                                                            // iFirst
      -1,                                                            // iLast
      164,                                                           // lineNo
      69,                                                            // colNo
      "P_wayp",                                                      // aName
      "synchronize_trajectory",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/synchronize_trajectory.m", // pName
      0 // checkKind
  };
  static rtBoundsCheckInfo xf_emlrtBCI = {
      -1,                                                            // iFirst
      -1,                                                            // iLast
      168,                                                           // lineNo
      72,                                                            // colNo
      "solution_opt_test",                                           // aName
      "synchronize_trajectory",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/synchronize_trajectory.m", // pName
      0 // checkKind
  };
  static rtBoundsCheckInfo xg_emlrtBCI = {
      -1,                                                            // iFirst
      -1,                                                            // iLast
      174,                                                           // lineNo
      25,                                                            // colNo
      "t",                                                           // aName
      "synchronize_trajectory",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/synchronize_trajectory.m", // pName
      0 // checkKind
  };
  static rtBoundsCheckInfo xh_emlrtBCI = {
      -1,                                                            // iFirst
      -1,                                                            // iLast
      184,                                                           // lineNo
      43,                                                            // colNo
      "solution_opt_test",                                           // aName
      "synchronize_trajectory",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/synchronize_trajectory.m", // pName
      0 // checkKind
  };
  static rtBoundsCheckInfo xi_emlrtBCI = {
      -1,                                                            // iFirst
      -1,                                                            // iLast
      218,                                                           // lineNo
      21,                                                            // colNo
      "J",                                                           // aName
      "synchronize_trajectory",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/synchronize_trajectory.m", // pName
      0 // checkKind
  };
  static rtBoundsCheckInfo yb_emlrtBCI = {
      -1,                                                            // iFirst
      -1,                                                            // iLast
      68,                                                            // lineNo
      15,                                                            // colNo
      "J_red",                                                       // aName
      "synchronize_trajectory",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/synchronize_trajectory.m", // pName
      0 // checkKind
  };
  static rtBoundsCheckInfo yc_emlrtBCI = {
      -1,                                                            // iFirst
      -1,                                                            // iLast
      125,                                                           // lineNo
      106,                                                           // colNo
      "A_init",                                                      // aName
      "synchronize_trajectory",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/synchronize_trajectory.m", // pName
      0 // checkKind
  };
  static rtBoundsCheckInfo yd_emlrtBCI = {
      -1,                                                            // iFirst
      -1,                                                            // iLast
      133,                                                           // lineNo
      183,                                                           // colNo
      "A_wayp",                                                      // aName
      "synchronize_trajectory",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/synchronize_trajectory.m", // pName
      0 // checkKind
  };
  static rtBoundsCheckInfo ye_emlrtBCI = {
      -1,                                                            // iFirst
      -1,                                                            // iLast
      166,                                                           // lineNo
      195,                                                           // colNo
      "t_opt_test",                                                  // aName
      "synchronize_trajectory",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/synchronize_trajectory.m", // pName
      0 // checkKind
  };
  static rtBoundsCheckInfo yf_emlrtBCI = {
      -1,                                                            // iFirst
      -1,                                                            // iLast
      171,                                                           // lineNo
      100,                                                           // colNo
      "P_init",                                                      // aName
      "synchronize_trajectory",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/synchronize_trajectory.m", // pName
      0 // checkKind
  };
  static rtBoundsCheckInfo yg_emlrtBCI = {
      -1,                                                            // iFirst
      -1,                                                            // iLast
      175,                                                           // lineNo
      49,                                                            // colNo
      "J_tim",                                                       // aName
      "synchronize_trajectory",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/synchronize_trajectory.m", // pName
      0 // checkKind
  };
  static rtBoundsCheckInfo yh_emlrtBCI = {
      -1,                                                            // iFirst
      -1,                                                            // iLast
      184,                                                           // lineNo
      25,                                                            // colNo
      "solution_opt_test{index_axis}",                               // aName
      "synchronize_trajectory",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/synchronize_trajectory.m", // pName
      0 // checkKind
  };
  static rtBoundsCheckInfo yi_emlrtBCI = {
      -1,                                                            // iFirst
      -1,                                                            // iLast
      219,                                                           // lineNo
      63,                                                            // colNo
      "solution_opt",                                                // aName
      "synchronize_trajectory",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/synchronize_trajectory.m", // pName
      0 // checkKind
  };
  static rtEqualityCheckInfo c_emlrtECI = {
      -1,                                                           // nDims
      145,                                                          // lineNo
      29,                                                           // colNo
      "synchronize_trajectory",                                     // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/synchronize_trajectory.m" // pName
  };
  static rtEqualityCheckInfo d_emlrtECI = {
      -1,                                                           // nDims
      159,                                                          // lineNo
      32,                                                           // colNo
      "synchronize_trajectory",                                     // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/synchronize_trajectory.m" // pName
  };
  static rtEqualityCheckInfo e_emlrtECI = {
      -1,                                                           // nDims
      198,                                                          // lineNo
      17,                                                           // colNo
      "synchronize_trajectory",                                     // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/synchronize_trajectory.m" // pName
  };
  coder::array<cell_wrap_12, 1U> J_red;
  coder::array<cell_wrap_12, 1U> t_red;
  coder::array<cell_wrap_13, 1U> J_opt;
  coder::array<cell_wrap_13, 1U> J_opt_test;
  coder::array<cell_wrap_13, 1U> t_opt;
  coder::array<cell_wrap_13, 1U> t_opt_test;
  coder::array<cell_wrap_14, 1U> J_tim;
  coder::array<cell_wrap_14, 1U> t_tim;
  coder::array<cell_wrap_15, 1U> J;
  coder::array<cell_wrap_15, 1U> t;
  coder::array<cell_wrap_16, 1U> solution_opt;
  coder::array<cell_wrap_16, 1U> solution_opt_test;
  coder::array<cell_wrap_16, 1U> solution_tim;
  coder::array<double, 2U> t18_signals_values;
  coder::array<double, 2U> t18_time;
  coder::array<double, 1U> T;
  coder::array<double, 1U> t_sync;
  coder::array<bool, 1U> b_solution_out;
  coder::array<bool, 1U> r;
  double b_J_opt_test[7];
  double b_t_opt_test[7];
  double dv[4];
  double dv1[4];
  double idx;
  int solution_opt_test_all[100];
  int iv[2];
  int iv1[2];
  int b_i;
  int b_unnamed_idx_0;
  int c_unnamed_idx_0;
  int d_unnamed_idx_0;
  int e_unnamed_idx_0;
  int i;
  int i1;
  int index_axis;
  int loop_ub;
  int num_axes;
  int unnamed_idx_0;
  bool solution_opt_test_all_data[100];
  bool guard1 = false;
  //  ---------------------------------------------------------------------
  //  Package:    TopiCo (https://github.com/AIS-Bonn/TopiCo)
  //  Version:    2021-03-18 12:09:55
  //  Maintainer: Marius Beul (mbeul@ais.uni-bonn.de)
  //  License:    BSD
  //  ---------------------------------------------------------------------
  //  Software License Agreement (BSD License)
  //  Copyright (c) 2021, Computer Science Institute VI, University of Bonn
  //  All rights reserved.
  //  Redistribution and use in source and binary forms, with or without
  //  modification, are permitted provided that the following conditions
  //  are met:
  //
  //  * Redistributions of source code must retain the above copyright
  //    notice, this list of conditions and the following disclaimer.
  //  * Redistributions in binary form must reproduce the above
  //    copyright notice, this list of conditions and the following
  //    disclaimer in the documentation and/or other materials provided
  //    with the distribution.
  //  * Neither the name of University of Bonn, Computer Science Institute
  //    VI nor the names of its contributors may be used to endorse or
  //    promote products derived from this software without specific
  //    prior written permission.
  //
  //  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
  //  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
  //  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
  //  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
  //  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  //  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
  //  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
  //  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  //  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
  //  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
  //  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
  //  POSSIBILITY OF SUCH DAMAGE.
  //  --------------------------------------------------------------------
  num_axes = P_init.size(0) - 1;
  solution_out.set_size(P_init.size(0));
  loop_ub = P_init.size(0);
  for (i = 0; i < loop_ub; i++) {
    solution_out[i] = -1;
  }
  i = P_init.size(0);
  for (index_axis = 0; index_axis < i; index_axis++) {
    // TODO This workaround sometimes prevents numerical issues. Instead, we
    // should find symbolic solutions for the J_max == -J_min and A_max = -A_min
    // condition
    if (index_axis + 1 > J_max.size(0)) {
      rtDynamicBoundsError(index_axis + 1, 1, J_max.size(0), &nb_emlrtBCI);
    }
    if (index_axis + 1 > J_min.size(0)) {
      rtDynamicBoundsError(index_axis + 1, 1, J_min.size(0), &ob_emlrtBCI);
    }
    if (J_max[index_axis] == -J_min[index_axis]) {
      if (index_axis + 1 > J_max.size(0)) {
        rtDynamicBoundsError(index_axis + 1, 1, J_max.size(0), &pb_emlrtBCI);
      }
      if (index_axis + 1 > J_min.size(0)) {
        rtDynamicBoundsError(index_axis + 1, 1, J_min.size(0), &qb_emlrtBCI);
      }
      J_min[index_axis] = -J_max[index_axis] * 0.98;
    }
    if (index_axis + 1 > A_max.size(0)) {
      rtDynamicBoundsError(index_axis + 1, 1, A_max.size(0), &rb_emlrtBCI);
    }
    if (index_axis + 1 > A_min.size(0)) {
      rtDynamicBoundsError(index_axis + 1, 1, A_min.size(0), &sb_emlrtBCI);
    }
    if (A_max[index_axis] == -A_min[index_axis]) {
      if (index_axis + 1 > A_min.size(0)) {
        rtDynamicBoundsError(index_axis + 1, 1, A_min.size(0), &tb_emlrtBCI);
      }
      if (index_axis + 1 > A_min.size(0)) {
        rtDynamicBoundsError(index_axis + 1, 1, A_min.size(0), &ub_emlrtBCI);
      }
      A_min[index_axis] = A_min[index_axis] * 0.98;
    }
  }
  i = P_init.size(0);
  t_red.set_size(P_init.size(0));
  for (i1 = 0; i1 < i; i1++) {
    b_i = t_red.size(0) - 1;
    if (i1 > t_red.size(0) - 1) {
      rtDynamicBoundsError(i1, 0, t_red.size(0) - 1, &vb_emlrtBCI);
    }
    t_red[i1].f1.size[0] = 0;
    if (i1 > b_i) {
      rtDynamicBoundsError(i1, 0, b_i, &vb_emlrtBCI);
    }
    t_red[i1].f1.size[1] = 4;
  }
  i = P_init.size(0);
  t_red.set_size(P_init.size(0));
  for (index_axis = 0; index_axis < i; index_axis++) {
    // coder
    if (index_axis > t_red.size(0) - 1) {
      rtDynamicBoundsError(index_axis, 0, t_red.size(0) - 1, &wb_emlrtBCI);
    }
    t_red[index_axis].f1.size[0] = 0;
    if (index_axis > t_red.size(0) - 1) {
      rtDynamicBoundsError(index_axis, 0, t_red.size(0) - 1, &wb_emlrtBCI);
    }
    t_red[index_axis].f1.size[1] = 4;
    if (index_axis > t_red.size(0) - 1) {
      rtDynamicBoundsError(index_axis, 0, t_red.size(0) - 1, &wb_emlrtBCI);
    }
  }
  i = P_init.size(0);
  J_red.set_size(P_init.size(0));
  for (i1 = 0; i1 < i; i1++) {
    b_i = J_red.size(0) - 1;
    if (i1 > J_red.size(0) - 1) {
      rtDynamicBoundsError(i1, 0, J_red.size(0) - 1, &xb_emlrtBCI);
    }
    J_red[i1].f1.size[0] = 0;
    if (i1 > b_i) {
      rtDynamicBoundsError(i1, 0, b_i, &xb_emlrtBCI);
    }
    J_red[i1].f1.size[1] = 4;
  }
  i = P_init.size(0);
  J_red.set_size(P_init.size(0));
  for (index_axis = 0; index_axis < i; index_axis++) {
    // coder
    if (index_axis > J_red.size(0) - 1) {
      rtDynamicBoundsError(index_axis, 0, J_red.size(0) - 1, &yb_emlrtBCI);
    }
    J_red[index_axis].f1.size[0] = 0;
    if (index_axis > J_red.size(0) - 1) {
      rtDynamicBoundsError(index_axis, 0, J_red.size(0) - 1, &yb_emlrtBCI);
    }
    J_red[index_axis].f1.size[1] = 4;
    if (index_axis > J_red.size(0) - 1) {
      rtDynamicBoundsError(index_axis, 0, J_red.size(0) - 1, &yb_emlrtBCI);
    }
  }
  i = P_init.size(0);
  t_opt.set_size(P_init.size(0));
  for (i1 = 0; i1 < i; i1++) {
    b_i = t_opt.size(0) - 1;
    if (i1 > t_opt.size(0) - 1) {
      rtDynamicBoundsError(i1, 0, t_opt.size(0) - 1, &ac_emlrtBCI);
    }
    t_opt[i1].f1.size[0] = 0;
    if (i1 > b_i) {
      rtDynamicBoundsError(i1, 0, b_i, &ac_emlrtBCI);
    }
    t_opt[i1].f1.size[1] = 7;
  }
  i1 = P_init.size(0);
  t_opt.set_size(P_init.size(0));
  for (index_axis = 0; index_axis < i1; index_axis++) {
    // coder
    if (index_axis > t_opt.size(0) - 1) {
      rtDynamicBoundsError(index_axis, 0, t_opt.size(0) - 1, &bc_emlrtBCI);
    }
    t_opt[index_axis].f1.size[0] = 0;
    if (index_axis > t_opt.size(0) - 1) {
      rtDynamicBoundsError(index_axis, 0, t_opt.size(0) - 1, &bc_emlrtBCI);
    }
    t_opt[index_axis].f1.size[1] = 7;
    if (index_axis > t_opt.size(0) - 1) {
      rtDynamicBoundsError(index_axis, 0, t_opt.size(0) - 1, &bc_emlrtBCI);
    }
  }
  unnamed_idx_0 = P_init.size(0);
  J_opt.set_size(P_init.size(0));
  for (i1 = 0; i1 < unnamed_idx_0; i1++) {
    b_i = J_opt.size(0) - 1;
    if (i1 > J_opt.size(0) - 1) {
      rtDynamicBoundsError(i1, 0, J_opt.size(0) - 1, &cc_emlrtBCI);
    }
    J_opt[i1].f1.size[0] = 0;
    if (i1 > b_i) {
      rtDynamicBoundsError(i1, 0, b_i, &cc_emlrtBCI);
    }
    J_opt[i1].f1.size[1] = 7;
  }
  i1 = P_init.size(0);
  J_opt.set_size(P_init.size(0));
  for (index_axis = 0; index_axis < i1; index_axis++) {
    // coder
    if (index_axis > J_opt.size(0) - 1) {
      rtDynamicBoundsError(index_axis, 0, J_opt.size(0) - 1, &dc_emlrtBCI);
    }
    J_opt[index_axis].f1.size[0] = 0;
    if (index_axis > J_opt.size(0) - 1) {
      rtDynamicBoundsError(index_axis, 0, J_opt.size(0) - 1, &dc_emlrtBCI);
    }
    J_opt[index_axis].f1.size[1] = 7;
    if (index_axis > J_opt.size(0) - 1) {
      rtDynamicBoundsError(index_axis, 0, J_opt.size(0) - 1, &dc_emlrtBCI);
    }
  }
  b_unnamed_idx_0 = P_init.size(0);
  t_tim.set_size(P_init.size(0));
  for (i1 = 0; i1 < b_unnamed_idx_0; i1++) {
    b_i = t_tim.size(0) - 1;
    if (i1 > t_tim.size(0) - 1) {
      rtDynamicBoundsError(i1, 0, t_tim.size(0) - 1, &ec_emlrtBCI);
    }
    t_tim[i1].f1.size[0] = 0;
    if (i1 > b_i) {
      rtDynamicBoundsError(i1, 0, b_i, &ec_emlrtBCI);
    }
    t_tim[i1].f1.size[1] = 11;
  }
  i1 = P_init.size(0);
  t_tim.set_size(P_init.size(0));
  for (index_axis = 0; index_axis < i1; index_axis++) {
    // coder
    if (index_axis > t_tim.size(0) - 1) {
      rtDynamicBoundsError(index_axis, 0, t_tim.size(0) - 1, &fc_emlrtBCI);
    }
    t_tim[index_axis].f1.size[0] = 0;
    if (index_axis > t_tim.size(0) - 1) {
      rtDynamicBoundsError(index_axis, 0, t_tim.size(0) - 1, &fc_emlrtBCI);
    }
    t_tim[index_axis].f1.size[1] = 11;
    if (index_axis > t_tim.size(0) - 1) {
      rtDynamicBoundsError(index_axis, 0, t_tim.size(0) - 1, &fc_emlrtBCI);
    }
  }
  b_unnamed_idx_0 = P_init.size(0);
  J_tim.set_size(P_init.size(0));
  for (i1 = 0; i1 < b_unnamed_idx_0; i1++) {
    b_i = J_tim.size(0) - 1;
    if (i1 > J_tim.size(0) - 1) {
      rtDynamicBoundsError(i1, 0, J_tim.size(0) - 1, &gc_emlrtBCI);
    }
    J_tim[i1].f1.size[0] = 0;
    if (i1 > b_i) {
      rtDynamicBoundsError(i1, 0, b_i, &gc_emlrtBCI);
    }
    J_tim[i1].f1.size[1] = 11;
  }
  i1 = P_init.size(0);
  J_tim.set_size(P_init.size(0));
  for (index_axis = 0; index_axis < i1; index_axis++) {
    // coder
    if (index_axis > J_tim.size(0) - 1) {
      rtDynamicBoundsError(index_axis, 0, J_tim.size(0) - 1, &hc_emlrtBCI);
    }
    J_tim[index_axis].f1.size[0] = 0;
    if (index_axis > J_tim.size(0) - 1) {
      rtDynamicBoundsError(index_axis, 0, J_tim.size(0) - 1, &hc_emlrtBCI);
    }
    J_tim[index_axis].f1.size[1] = 11;
    if (index_axis > J_tim.size(0) - 1) {
      rtDynamicBoundsError(index_axis, 0, J_tim.size(0) - 1, &hc_emlrtBCI);
    }
  }
  b_unnamed_idx_0 = P_init.size(0);
  t_out.set_size(P_init.size(0));
  for (i1 = 0; i1 < b_unnamed_idx_0; i1++) {
    b_i = t_out.size(0) - 1;
    if (i1 > t_out.size(0) - 1) {
      rtDynamicBoundsError(i1, 0, t_out.size(0) - 1, &ic_emlrtBCI);
    }
    t_out[i1].f1.set_size(0, t_out[i1].f1.size(1));
    if (i1 > b_i) {
      rtDynamicBoundsError(i1, 0, b_i, &ic_emlrtBCI);
    }
    t_out[i1].f1.set_size(t_out[i1].f1.size(0), 0);
  }
  i1 = P_init.size(0);
  t_out.set_size(P_init.size(0));
  for (index_axis = 0; index_axis < i1; index_axis++) {
    // coder
    if (index_axis > t_out.size(0) - 1) {
      rtDynamicBoundsError(index_axis, 0, t_out.size(0) - 1, &jc_emlrtBCI);
    }
    t_out[index_axis].f1.set_size(0, t_out[index_axis].f1.size(1));
    if (index_axis > t_out.size(0) - 1) {
      rtDynamicBoundsError(index_axis, 0, t_out.size(0) - 1, &jc_emlrtBCI);
    }
    t_out[index_axis].f1.set_size(t_out[index_axis].f1.size(0), 7);
    if (index_axis > t_out.size(0) - 1) {
      rtDynamicBoundsError(index_axis, 0, t_out.size(0) - 1, &jc_emlrtBCI);
    }
  }
  c_unnamed_idx_0 = P_init.size(0);
  J_out.set_size(P_init.size(0));
  for (i1 = 0; i1 < c_unnamed_idx_0; i1++) {
    b_i = J_out.size(0) - 1;
    if (i1 > J_out.size(0) - 1) {
      rtDynamicBoundsError(i1, 0, J_out.size(0) - 1, &kc_emlrtBCI);
    }
    J_out[i1].f1.set_size(0, J_out[i1].f1.size(1));
    if (i1 > b_i) {
      rtDynamicBoundsError(i1, 0, b_i, &kc_emlrtBCI);
    }
    J_out[i1].f1.set_size(J_out[i1].f1.size(0), 0);
  }
  i1 = P_init.size(0);
  J_out.set_size(P_init.size(0));
  for (index_axis = 0; index_axis < i1; index_axis++) {
    // coder
    if (index_axis > J_out.size(0) - 1) {
      rtDynamicBoundsError(index_axis, 0, J_out.size(0) - 1, &lc_emlrtBCI);
    }
    J_out[index_axis].f1.set_size(0, J_out[index_axis].f1.size(1));
    if (index_axis > J_out.size(0) - 1) {
      rtDynamicBoundsError(index_axis, 0, J_out.size(0) - 1, &lc_emlrtBCI);
    }
    J_out[index_axis].f1.set_size(J_out[index_axis].f1.size(0), 7);
    if (index_axis > J_out.size(0) - 1) {
      rtDynamicBoundsError(index_axis, 0, J_out.size(0) - 1, &lc_emlrtBCI);
    }
  }
  i1 = P_init.size(0);
  d_unnamed_idx_0 = P_init.size(0);
  t.set_size(P_init.size(0));
  for (index_axis = 0; index_axis < i1; index_axis++) {
    // coder
    loop_ub = t.size(0) - 1;
    for (b_i = 0; b_i < 11; b_i++) {
      if (index_axis > loop_ub) {
        rtDynamicBoundsError(index_axis, 0, loop_ub, &mc_emlrtBCI);
      }
      t[index_axis].f1[b_i] = 0.0;
    }
  }
  i1 = P_init.size(0);
  e_unnamed_idx_0 = P_init.size(0);
  J.set_size(P_init.size(0));
  for (index_axis = 0; index_axis < i1; index_axis++) {
    // coder
    loop_ub = J.size(0) - 1;
    for (b_i = 0; b_i < 11; b_i++) {
      if (index_axis > loop_ub) {
        rtDynamicBoundsError(index_axis, 0, loop_ub, &nc_emlrtBCI);
      }
      J[index_axis].f1[b_i] = 0.0;
    }
  }
  b_i = P_init.size(0);
  solution_opt.set_size(P_init.size(0));
  for (i1 = 0; i1 < b_i; i1++) {
    if (i1 > solution_opt.size(0) - 1) {
      rtDynamicBoundsError(i1, 0, solution_opt.size(0) - 1, &oc_emlrtBCI);
    }
    solution_opt[i1].f1.size[0] = 0;
  }
  i1 = P_init.size(0);
  solution_opt.set_size(P_init.size(0));
  for (index_axis = 0; index_axis < i1; index_axis++) {
    // coder
    b_i = solution_opt.size(0) - 1;
    if (index_axis > solution_opt.size(0) - 1) {
      rtDynamicBoundsError(index_axis, 0, solution_opt.size(0) - 1,
                           &pc_emlrtBCI);
    }
    solution_opt[index_axis].f1.size[0] = 1;
    if (index_axis > b_i) {
      rtDynamicBoundsError(index_axis, 0, b_i, &pc_emlrtBCI);
    }
    solution_opt[index_axis].f1.data[0] = -1;
  }
  b_i = P_init.size(0);
  solution_tim.set_size(P_init.size(0));
  for (i1 = 0; i1 < b_i; i1++) {
    if (i1 > solution_tim.size(0) - 1) {
      rtDynamicBoundsError(i1, 0, solution_tim.size(0) - 1, &qc_emlrtBCI);
    }
    solution_tim[i1].f1.size[0] = 0;
  }
  i1 = P_init.size(0);
  solution_tim.set_size(P_init.size(0));
  for (index_axis = 0; index_axis < i1; index_axis++) {
    // coder
    b_i = solution_tim.size(0) - 1;
    if (index_axis > solution_tim.size(0) - 1) {
      rtDynamicBoundsError(index_axis, 0, solution_tim.size(0) - 1,
                           &rc_emlrtBCI);
    }
    solution_tim[index_axis].f1.size[0] = 1;
    if (index_axis > b_i) {
      rtDynamicBoundsError(index_axis, 0, b_i, &rc_emlrtBCI);
    }
    solution_tim[index_axis].f1.data[0] = -1;
  }
  i1 = P_init.size(0);
  for (index_axis = 0; index_axis < i1; index_axis++) {
    if (index_axis + 1 > J_max.size(0)) {
      rtDynamicBoundsError(index_axis + 1, 1, J_max.size(0), &sc_emlrtBCI);
    }
    if (rtIsInf(J_max[index_axis])) {
      if (index_axis + 1 > J_max.size(0)) {
        rtDynamicBoundsError(index_axis + 1, 1, J_max.size(0), &tc_emlrtBCI);
      }
      J_max[index_axis] = 1.0E+6;
    }
    if (index_axis + 1 > J_min.size(0)) {
      rtDynamicBoundsError(index_axis + 1, 1, J_min.size(0), &uc_emlrtBCI);
    }
    if (rtIsInf(J_min[index_axis])) {
      if (index_axis + 1 > J_min.size(0)) {
        rtDynamicBoundsError(index_axis + 1, 1, J_min.size(0), &vc_emlrtBCI);
      }
      J_min[index_axis] = -1.0E+6;
    }
  }
  //     %% Correct Feasibility
  // parfor index_axis = 1:num_axes
  i1 = P_init.size(0);
  for (index_axis = 0; index_axis < i1; index_axis++) {
    printf("Debug: Ax[");
    fflush(stdout);
    printint(static_cast<double>(index_axis) + 1.0);
    printf("] Correcting feasibility!\n");
    fflush(stdout);
    if (index_axis + 1 > P_init.size(0)) {
      rtDynamicBoundsError(index_axis + 1, 1, P_init.size(0), &wc_emlrtBCI);
    }
    if (index_axis + 1 > V_init.size(0)) {
      rtDynamicBoundsError(index_axis + 1, 1, V_init.size(0), &xc_emlrtBCI);
    }
    if (index_axis + 1 > A_init.size(0)) {
      rtDynamicBoundsError(index_axis + 1, 1, A_init.size(0), &yc_emlrtBCI);
    }
    if (index_axis + 1 > V_max.size(0)) {
      rtDynamicBoundsError(index_axis + 1, 1, V_max.size(0), &ad_emlrtBCI);
    }
    if (index_axis + 1 > V_min.size(0)) {
      rtDynamicBoundsError(index_axis + 1, 1, V_min.size(0), &bd_emlrtBCI);
    }
    if (index_axis + 1 > A_max.size(0)) {
      rtDynamicBoundsError(index_axis + 1, 1, A_max.size(0), &cd_emlrtBCI);
    }
    if (index_axis + 1 > A_min.size(0)) {
      rtDynamicBoundsError(index_axis + 1, 1, A_min.size(0), &dd_emlrtBCI);
    }
    if (index_axis + 1 > J_max.size(0)) {
      rtDynamicBoundsError(index_axis + 1, 1, J_max.size(0), &ed_emlrtBCI);
    }
    if (index_axis + 1 > J_min.size(0)) {
      rtDynamicBoundsError(index_axis + 1, 1, J_min.size(0), &fd_emlrtBCI);
    }
    if (index_axis + 1 > b_hard_V_lim.size(0)) {
      rtDynamicBoundsError(index_axis + 1, 1, b_hard_V_lim.size(0),
                           &gd_emlrtBCI);
    }
    correct_feasibility(P_init[index_axis], V_init[index_axis],
                        A_init[index_axis], V_max[index_axis],
                        V_min[index_axis], A_max[index_axis], A_min[index_axis],
                        J_max[index_axis], J_min[index_axis],
                        b_hard_V_lim[index_axis], dv, dv1);
    if (index_axis > t_red.size(0) - 1) {
      rtDynamicBoundsError(index_axis, 0, t_red.size(0) - 1, &hd_emlrtBCI);
    }
    t_red[index_axis].f1.size[0] = 1;
    if (index_axis > t_red.size(0) - 1) {
      rtDynamicBoundsError(index_axis, 0, t_red.size(0) - 1, &hd_emlrtBCI);
    }
    t_red[index_axis].f1.size[1] = 4;
    b_i = t_red.size(0);
    if (index_axis > t_red.size(0) - 1) {
      rtDynamicBoundsError(index_axis, 0, t_red.size(0) - 1, &hd_emlrtBCI);
    }
    if (index_axis > t_red.size(0) - 1) {
      rtDynamicBoundsError(index_axis, 0, t_red.size(0) - 1, &hd_emlrtBCI);
    }
    t_red[index_axis].f1.data[0] = dv[0];
    if (index_axis > b_i - 1) {
      rtDynamicBoundsError(index_axis, 0, b_i - 1, &id_emlrtBCI);
    }
    t_red[index_axis].f1.data[1] = dv[1];
    if (index_axis > b_i - 1) {
      rtDynamicBoundsError(index_axis, 0, b_i - 1, &id_emlrtBCI);
    }
    t_red[index_axis].f1.data[2] = dv[2];
    if (index_axis > b_i - 1) {
      rtDynamicBoundsError(index_axis, 0, b_i - 1, &id_emlrtBCI);
    }
    t_red[index_axis].f1.data[3] = dv[3];
    if (index_axis > J_red.size(0) - 1) {
      rtDynamicBoundsError(index_axis, 0, J_red.size(0) - 1, &jd_emlrtBCI);
    }
    J_red[index_axis].f1.size[0] = 1;
    if (index_axis > J_red.size(0) - 1) {
      rtDynamicBoundsError(index_axis, 0, J_red.size(0) - 1, &jd_emlrtBCI);
    }
    J_red[index_axis].f1.size[1] = 4;
    b_i = J_red.size(0);
    if (index_axis > J_red.size(0) - 1) {
      rtDynamicBoundsError(index_axis, 0, J_red.size(0) - 1, &jd_emlrtBCI);
    }
    if (index_axis > J_red.size(0) - 1) {
      rtDynamicBoundsError(index_axis, 0, J_red.size(0) - 1, &jd_emlrtBCI);
    }
    J_red[index_axis].f1.data[0] = dv1[0];
    if (index_axis > b_i - 1) {
      rtDynamicBoundsError(index_axis, 0, b_i - 1, &kd_emlrtBCI);
    }
    J_red[index_axis].f1.data[1] = dv1[1];
    if (index_axis > b_i - 1) {
      rtDynamicBoundsError(index_axis, 0, b_i - 1, &kd_emlrtBCI);
    }
    J_red[index_axis].f1.data[2] = dv1[2];
    if (index_axis > b_i - 1) {
      rtDynamicBoundsError(index_axis, 0, b_i - 1, &kd_emlrtBCI);
    }
    J_red[index_axis].f1.data[3] = dv1[3];
    if (index_axis > t_red.size(0) - 1) {
      rtDynamicBoundsError(index_axis, 0, t_red.size(0) - 1, &ld_emlrtBCI);
    }
    if (index_axis > J_red.size(0) - 1) {
      rtDynamicBoundsError(index_axis, 0, J_red.size(0) - 1, &md_emlrtBCI);
    }
    construct_setp_struct(t_red[index_axis].f1.data, t_red[index_axis].f1.size,
                          J_red[index_axis].f1.data, J_red[index_axis].f1.size,
                          t18_time, t18_signals_values);
    if (index_axis + 1 > P_init.size(0)) {
      rtDynamicBoundsError(index_axis + 1, 1, P_init.size(0), &nd_emlrtBCI);
    }
    if (index_axis + 1 > V_init.size(0)) {
      rtDynamicBoundsError(index_axis + 1, 1, V_init.size(0), &od_emlrtBCI);
    }
    if (index_axis + 1 > A_init.size(0)) {
      rtDynamicBoundsError(index_axis + 1, 1, A_init.size(0), &pd_emlrtBCI);
    }
    if (index_axis + 1 > P_init.size(0)) {
      rtDynamicBoundsError(index_axis + 1, 1, P_init.size(0), &qd_emlrtBCI);
    }
    if (index_axis + 1 > V_init.size(0)) {
      rtDynamicBoundsError(index_axis + 1, 1, V_init.size(0), &rd_emlrtBCI);
    }
    if (index_axis + 1 > A_init.size(0)) {
      rtDynamicBoundsError(index_axis + 1, 1, A_init.size(0), &sd_emlrtBCI);
    }
    evaluate_to_time(P_init[index_axis], V_init[index_axis], A_init[index_axis],
                     t18_time, t18_signals_values, &P_init[index_axis],
                     &V_init[index_axis], &A_init[index_axis]);
  }
  //     %% Find optimal solution
  // parfor index_axis = 1:num_axes
  J_opt.set_size(unnamed_idx_0);
  t_opt.set_size(i);
  for (index_axis = 0; index_axis <= num_axes; index_axis++) {
    printf("Debug: Ax[");
    fflush(stdout);
    printint(static_cast<double>(index_axis) + 1.0);
    printf("] Generating optimal solutions!\n");
    fflush(stdout);
    if (index_axis + 1 > P_init.size(0)) {
      rtDynamicBoundsError(index_axis + 1, 1, P_init.size(0), &td_emlrtBCI);
    }
    if (index_axis + 1 > V_init.size(0)) {
      rtDynamicBoundsError(index_axis + 1, 1, V_init.size(0), &ud_emlrtBCI);
    }
    if (index_axis + 1 > A_init.size(0)) {
      rtDynamicBoundsError(index_axis + 1, 1, A_init.size(0), &vd_emlrtBCI);
    }
    if (index_axis + 1 > P_wayp.size(0)) {
      rtDynamicBoundsError(index_axis + 1, 1, P_wayp.size(0), &wd_emlrtBCI);
    }
    if (index_axis + 1 > V_wayp.size(0)) {
      rtDynamicBoundsError(index_axis + 1, 1, V_wayp.size(0), &xd_emlrtBCI);
    }
    if (index_axis + 1 > A_wayp.size(0)) {
      rtDynamicBoundsError(index_axis + 1, 1, A_wayp.size(0), &yd_emlrtBCI);
    }
    if (index_axis + 1 > V_max.size(0)) {
      rtDynamicBoundsError(index_axis + 1, 1, V_max.size(0), &ae_emlrtBCI);
    }
    if (index_axis + 1 > V_min.size(0)) {
      rtDynamicBoundsError(index_axis + 1, 1, V_min.size(0), &be_emlrtBCI);
    }
    if (index_axis + 1 > A_max.size(0)) {
      rtDynamicBoundsError(index_axis + 1, 1, A_max.size(0), &ce_emlrtBCI);
    }
    if (index_axis + 1 > A_min.size(0)) {
      rtDynamicBoundsError(index_axis + 1, 1, A_min.size(0), &de_emlrtBCI);
    }
    if (index_axis + 1 > J_max.size(0)) {
      rtDynamicBoundsError(index_axis + 1, 1, J_max.size(0), &ee_emlrtBCI);
    }
    if (index_axis + 1 > J_min.size(0)) {
      rtDynamicBoundsError(index_axis + 1, 1, J_min.size(0), &fe_emlrtBCI);
    }
    if (index_axis > t_opt.size(0) - 1) {
      rtDynamicBoundsError(index_axis, 0, t_opt.size(0) - 1, &ge_emlrtBCI);
    }
    if (index_axis > J_opt.size(0) - 1) {
      rtDynamicBoundsError(index_axis, 0, J_opt.size(0) - 1, &he_emlrtBCI);
    }
    if (index_axis > solution_opt.size(0) - 1) {
      rtDynamicBoundsError(index_axis, 0, solution_opt.size(0) - 1,
                           &ie_emlrtBCI);
    }
    solve_O(P_init[index_axis], V_init[index_axis], A_init[index_axis],
            P_wayp[index_axis], V_wayp[index_axis], A_wayp[index_axis],
            V_max[index_axis], V_min[index_axis], A_max[index_axis],
            A_min[index_axis], J_max[index_axis], J_min[index_axis],
            t_opt[index_axis].f1.data, iv, J_opt[index_axis].f1.data, iv1,
            solution_opt[index_axis].f1.data, &i);
    solution_opt[index_axis].f1.size[0] = i;
    J_opt[index_axis].f1.size[0] = iv1[0];
    J_opt[index_axis].f1.size[1] = iv1[1];
    t_opt[index_axis].f1.size[0] = iv[0];
    t_opt[index_axis].f1.size[1] = iv[1];
    if (index_axis > solution_opt.size(0) - 1) {
      rtDynamicBoundsError(index_axis, 0, solution_opt.size(0) - 1,
                           &je_emlrtBCI);
    }
    if (solution_opt[index_axis].f1.size[0] == 0) {
      // So that coder does not crash
      printf("Error: This usually can NOT happen. Using idle trajectory to "
             "prevent segfault!\n");
      fflush(stdout);
      if (index_axis > t_opt.size(0) - 1) {
        rtDynamicBoundsError(index_axis, 0, t_opt.size(0) - 1, &ke_emlrtBCI);
      }
      t_opt[index_axis].f1.size[0] = 1;
      if (index_axis > t_opt.size(0) - 1) {
        rtDynamicBoundsError(index_axis, 0, t_opt.size(0) - 1, &ke_emlrtBCI);
      }
      t_opt[index_axis].f1.size[1] = 7;
      b_i = t_opt.size(0) - 1;
      if (index_axis > t_opt.size(0) - 1) {
        rtDynamicBoundsError(index_axis, 0, t_opt.size(0) - 1, &ke_emlrtBCI);
      }
      if (index_axis > t_opt.size(0) - 1) {
        rtDynamicBoundsError(index_axis, 0, t_opt.size(0) - 1, &ke_emlrtBCI);
      }
      for (i = 0; i < 7; i++) {
        if (index_axis > b_i) {
          rtDynamicBoundsError(index_axis, 0, b_i, &le_emlrtBCI);
        }
        t_opt[index_axis].f1.data[i] = 0.0;
      }
      if (index_axis > J_opt.size(0) - 1) {
        rtDynamicBoundsError(index_axis, 0, J_opt.size(0) - 1, &me_emlrtBCI);
      }
      J_opt[index_axis].f1.size[0] = 1;
      if (index_axis > J_opt.size(0) - 1) {
        rtDynamicBoundsError(index_axis, 0, J_opt.size(0) - 1, &me_emlrtBCI);
      }
      J_opt[index_axis].f1.size[1] = 7;
      b_i = J_opt.size(0) - 1;
      if (index_axis > J_opt.size(0) - 1) {
        rtDynamicBoundsError(index_axis, 0, J_opt.size(0) - 1, &me_emlrtBCI);
      }
      if (index_axis > J_opt.size(0) - 1) {
        rtDynamicBoundsError(index_axis, 0, J_opt.size(0) - 1, &me_emlrtBCI);
      }
      for (i = 0; i < 7; i++) {
        if (index_axis > b_i) {
          rtDynamicBoundsError(index_axis, 0, b_i, &ne_emlrtBCI);
        }
        J_opt[index_axis].f1.data[i] = 0.0;
      }
      b_i = solution_opt.size(0) - 1;
      if (index_axis > solution_opt.size(0) - 1) {
        rtDynamicBoundsError(index_axis, 0, solution_opt.size(0) - 1,
                             &oe_emlrtBCI);
      }
      solution_opt[index_axis].f1.size[0] = 1;
      if (index_axis > b_i) {
        rtDynamicBoundsError(index_axis, 0, b_i, &oe_emlrtBCI);
      }
      solution_opt[index_axis].f1.data[0] = -1;
    }
  }
  //     %% Find timed solution
  guard1 = false;
  if (num_axes + 1 > 1) {
    b_solution_out.set_size(b_sync_V.size(0));
    loop_ub = b_sync_V.size(0);
    for (i = 0; i < loop_ub; i++) {
      b_solution_out[i] = b_sync_V[i];
    }
    r.set_size(b_sync_A.size(0));
    loop_ub = b_sync_A.size(0);
    for (i = 0; i < loop_ub; i++) {
      r[i] = b_sync_A[i];
    }
    if (b_solution_out.size(0) != r.size(0)) {
      rtSizeEq1DError(b_solution_out.size(0), r.size(0), &c_emlrtECI);
    }
    loop_ub = b_solution_out.size(0);
    for (i = 0; i < loop_ub; i++) {
      b_solution_out[i] = (b_solution_out[i] || r[i]);
    }
    r.set_size(b_sync_J.size(0));
    loop_ub = b_sync_J.size(0);
    for (i = 0; i < loop_ub; i++) {
      r[i] = b_sync_J[i];
    }
    if (b_solution_out.size(0) != r.size(0)) {
      rtSizeEq1DError(b_solution_out.size(0), r.size(0), &c_emlrtECI);
    }
    loop_ub = b_solution_out.size(0);
    for (i = 0; i < loop_ub; i++) {
      b_solution_out[i] = (b_solution_out[i] || r[i]);
    }
    r.set_size(b_sync_W.size(0));
    loop_ub = b_sync_W.size(0);
    for (i = 0; i < loop_ub; i++) {
      r[i] = b_sync_W[i];
    }
    if (b_solution_out.size(0) != r.size(0)) {
      rtSizeEq1DError(b_solution_out.size(0), r.size(0), &c_emlrtECI);
    }
    loop_ub = b_solution_out.size(0);
    for (i = 0; i < loop_ub; i++) {
      b_solution_out[i] = (b_solution_out[i] || r[i]);
    }
    if (coder::any(b_solution_out)) {
      double max_iterations;
      bool exitg1;
      t_opt_test.set_size(t_opt.size(0));
      loop_ub = t_opt.size(0);
      for (i = 0; i < loop_ub; i++) {
        t_opt_test[i] = t_opt[i];
      }
      J_opt_test.set_size(J_opt.size(0));
      loop_ub = J_opt.size(0);
      for (i = 0; i < loop_ub; i++) {
        J_opt_test[i] = J_opt[i];
      }
      solution_opt_test.set_size(solution_opt.size(0));
      loop_ub = solution_opt.size(0);
      for (i = 0; i < loop_ub; i++) {
        solution_opt_test[i] = solution_opt[i];
      }
      max_iterations = 0.0;
      for (index_axis = 0; index_axis <= num_axes; index_axis++) {
        if (index_axis > t_opt.size(0) - 1) {
          rtDynamicBoundsError(index_axis, 0, t_opt.size(0) - 1, &pe_emlrtBCI);
        }
        max_iterations += static_cast<double>(t_opt[index_axis].f1.size[0]);
      }
      printf("Debug: Starting synchronization!\n");
      fflush(stdout);
      t_out.set_size(b_unnamed_idx_0);
      J_out.set_size(c_unnamed_idx_0);
      e_unnamed_idx_0 = 0;
      exitg1 = false;
      while ((!exitg1) &&
             (e_unnamed_idx_0 <= static_cast<int>(max_iterations) - 1)) {
        bool exitg2;
        T.set_size(num_axes + 1);
        index_axis = 1;
        for (d_unnamed_idx_0 = 0; d_unnamed_idx_0 <= num_axes;
             d_unnamed_idx_0++) {
          index_axis = d_unnamed_idx_0 + 1;
          if (d_unnamed_idx_0 > t_opt_test.size(0) - 1) {
            rtDynamicBoundsError(d_unnamed_idx_0, 0, t_opt_test.size(0) - 1,
                                 &qe_emlrtBCI);
          }
          i = t_opt_test[d_unnamed_idx_0].f1.size[0];
          if (1 > i) {
            rtDynamicBoundsError(1, 1, i, &re_emlrtBCI);
          }
          idx = t_opt_test[d_unnamed_idx_0].f1.data[0];
          for (loop_ub = 0; loop_ub < 6; loop_ub++) {
            idx += t_opt_test[d_unnamed_idx_0]
                       .f1.data[t_opt_test[d_unnamed_idx_0].f1.size[0] *
                                (loop_ub + 1)];
          }
          if (d_unnamed_idx_0 + 1 > T.size(0)) {
            rtDynamicBoundsError(d_unnamed_idx_0 + 1, 1, T.size(0),
                                 &se_emlrtBCI);
          }
          T[d_unnamed_idx_0] = idx;
        }
        if (T.size(0) != T_catch_up.size(0)) {
          rtSizeEq1DError(T.size(0), T_catch_up.size(0), &d_emlrtECI);
        }
        t_sync.set_size(T.size(0));
        loop_ub = T.size(0);
        for (i = 0; i < loop_ub; i++) {
          t_sync[i] = T[i] - T_catch_up[i];
        }
        coder::internal::maximum(t_sync, &idx, &b_i);
        if ((b_i < 1) || (b_i > T.size(0))) {
          rtDynamicBoundsError(b_i, 1, T.size(0), &te_emlrtBCI);
        }
        if (b_i > T_catch_up.size(0)) {
          rtDynamicBoundsError(b_i, 1, T_catch_up.size(0), &ue_emlrtBCI);
        }
        t_sync.set_size(T_catch_up.size(0));
        loop_ub = T_catch_up.size(0);
        for (i = 0; i < loop_ub; i++) {
          t_sync[i] = (T[b_i - 1] + T_catch_up[i]) - T_catch_up[b_i - 1];
        }
        printf("Debug: Ax[");
        fflush(stdout);
        printint(static_cast<double>(index_axis));
        printf("] is the slowest axis!\n");
        fflush(stdout);
        index_axis = 0;
        exitg2 = false;
        while ((!exitg2) && (index_axis <= num_axes)) {
          bool b_guard1 = false;
          bool guard2 = false;
          b_guard1 = false;
          guard2 = false;
          if (index_axis + 1 != b_i) {
            if (index_axis + 1 > b_sync_V.size(0)) {
              rtDynamicBoundsError(index_axis + 1, 1, b_sync_V.size(0),
                                   &ve_emlrtBCI);
            }
            if (b_sync_V[index_axis]) {
              guard2 = true;
            } else {
              if (index_axis + 1 > b_sync_A.size(0)) {
                rtDynamicBoundsError(index_axis + 1, 1, b_sync_A.size(0),
                                     &ci_emlrtBCI);
              }
              if (b_sync_A[index_axis]) {
                guard2 = true;
              } else {
                if (index_axis + 1 > b_sync_J.size(0)) {
                  rtDynamicBoundsError(index_axis + 1, 1, b_sync_J.size(0),
                                       &di_emlrtBCI);
                }
                if (b_sync_J[index_axis]) {
                  guard2 = true;
                } else {
                  if (index_axis + 1 > b_sync_W.size(0)) {
                    rtDynamicBoundsError(index_axis + 1, 1, b_sync_W.size(0),
                                         &ei_emlrtBCI);
                  }
                  if (b_sync_W[index_axis]) {
                    guard2 = true;
                  } else {
                    b_guard1 = true;
                  }
                }
              }
            }
          } else {
            b_guard1 = true;
          }
          if (guard2) {
            if (index_axis + 1 > direction.size(0)) {
              rtDynamicBoundsError(index_axis + 1, 1, direction.size(0),
                                   &we_emlrtBCI);
            }
            if (direction[index_axis] == 0) {
              bool b_guard2 = false;
              if (index_axis + 1 > P_wayp.size(0)) {
                rtDynamicBoundsError(index_axis + 1, 1, P_wayp.size(0),
                                     &xe_emlrtBCI);
              }
              b_guard2 = false;
              if (rtIsNaN(P_wayp[index_axis])) {
                b_guard2 = true;
              } else {
                if (index_axis + 1 > V_wayp.size(0)) {
                  rtDynamicBoundsError(index_axis + 1, 1, V_wayp.size(0),
                                       &ai_emlrtBCI);
                }
                if (rtIsNaN(V_wayp[index_axis])) {
                  b_guard2 = true;
                } else {
                  if (index_axis + 1 > A_wayp.size(0)) {
                    rtDynamicBoundsError(index_axis + 1, 1, A_wayp.size(0),
                                         &bi_emlrtBCI);
                  }
                  if (rtIsNaN(A_wayp[index_axis])) {
                    b_guard2 = true;
                  }
                }
              }
              if (b_guard2) {
                printf("Debug: Ax[");
                fflush(stdout);
                printint(static_cast<double>(index_axis) + 1.0);
                printf("] Replacing NaNs with values from optimal solution!\n");
                fflush(stdout);
                if (index_axis > t_opt_test.size(0) - 1) {
                  rtDynamicBoundsError(index_axis, 0, t_opt_test.size(0) - 1,
                                       &ye_emlrtBCI);
                }
                if (index_axis > J_opt_test.size(0) - 1) {
                  rtDynamicBoundsError(index_axis, 0, J_opt_test.size(0) - 1,
                                       &af_emlrtBCI);
                }
                i = t_opt_test[index_axis].f1.size[0];
                if (1 > i) {
                  rtDynamicBoundsError(1, 1, i, &bf_emlrtBCI);
                }
                i = J_opt_test[index_axis].f1.size[0];
                if (1 > i) {
                  rtDynamicBoundsError(1, 1, i, &cf_emlrtBCI);
                }
                for (i = 0; i < 7; i++) {
                  b_t_opt_test[i] =
                      t_opt_test[index_axis]
                          .f1.data[t_opt_test[index_axis].f1.size[0] * i];
                  b_J_opt_test[i] =
                      J_opt_test[index_axis]
                          .f1.data[J_opt_test[index_axis].f1.size[0] * i];
                }
                construct_setp_struct(b_t_opt_test, b_J_opt_test, t18_time,
                                      t18_signals_values);
                if (index_axis + 1 > P_init.size(0)) {
                  rtDynamicBoundsError(index_axis + 1, 1, P_init.size(0),
                                       &df_emlrtBCI);
                }
                if (index_axis + 1 > V_init.size(0)) {
                  rtDynamicBoundsError(index_axis + 1, 1, V_init.size(0),
                                       &ef_emlrtBCI);
                }
                if (index_axis + 1 > A_init.size(0)) {
                  rtDynamicBoundsError(index_axis + 1, 1, A_init.size(0),
                                       &ff_emlrtBCI);
                }
                if (index_axis + 1 > P_wayp.size(0)) {
                  rtDynamicBoundsError(index_axis + 1, 1, P_wayp.size(0),
                                       &gf_emlrtBCI);
                }
                if (index_axis + 1 > V_wayp.size(0)) {
                  rtDynamicBoundsError(index_axis + 1, 1, V_wayp.size(0),
                                       &hf_emlrtBCI);
                }
                if (index_axis + 1 > A_wayp.size(0)) {
                  rtDynamicBoundsError(index_axis + 1, 1, A_wayp.size(0),
                                       &if_emlrtBCI);
                }
                evaluate_to_time(
                    P_init[index_axis], V_init[index_axis], A_init[index_axis],
                    t18_time, t18_signals_values, &P_wayp[index_axis],
                    &V_wayp[index_axis], &A_wayp[index_axis], &idx);
                printf("Debug: Ax[");
                fflush(stdout);
                printint(static_cast<double>(index_axis) + 1.0);
                printf("] Finding possible multiple solutions for updated "
                       "state!\n");
                fflush(stdout);
                if (index_axis + 1 > P_init.size(0)) {
                  rtDynamicBoundsError(index_axis + 1, 1, P_init.size(0),
                                       &jf_emlrtBCI);
                }
                if (index_axis + 1 > V_init.size(0)) {
                  rtDynamicBoundsError(index_axis + 1, 1, V_init.size(0),
                                       &kf_emlrtBCI);
                }
                if (index_axis + 1 > A_init.size(0)) {
                  rtDynamicBoundsError(index_axis + 1, 1, A_init.size(0),
                                       &lf_emlrtBCI);
                }
                if (index_axis + 1 > P_wayp.size(0)) {
                  rtDynamicBoundsError(index_axis + 1, 1, P_wayp.size(0),
                                       &mf_emlrtBCI);
                }
                if (index_axis + 1 > V_wayp.size(0)) {
                  rtDynamicBoundsError(index_axis + 1, 1, V_wayp.size(0),
                                       &nf_emlrtBCI);
                }
                if (index_axis + 1 > A_wayp.size(0)) {
                  rtDynamicBoundsError(index_axis + 1, 1, A_wayp.size(0),
                                       &of_emlrtBCI);
                }
                if (index_axis + 1 > V_max.size(0)) {
                  rtDynamicBoundsError(index_axis + 1, 1, V_max.size(0),
                                       &pf_emlrtBCI);
                }
                if (index_axis + 1 > V_min.size(0)) {
                  rtDynamicBoundsError(index_axis + 1, 1, V_min.size(0),
                                       &qf_emlrtBCI);
                }
                if (index_axis + 1 > A_max.size(0)) {
                  rtDynamicBoundsError(index_axis + 1, 1, A_max.size(0),
                                       &rf_emlrtBCI);
                }
                if (index_axis + 1 > A_min.size(0)) {
                  rtDynamicBoundsError(index_axis + 1, 1, A_min.size(0),
                                       &sf_emlrtBCI);
                }
                if (index_axis + 1 > J_max.size(0)) {
                  rtDynamicBoundsError(index_axis + 1, 1, J_max.size(0),
                                       &tf_emlrtBCI);
                }
                if (index_axis + 1 > J_min.size(0)) {
                  rtDynamicBoundsError(index_axis + 1, 1, J_min.size(0),
                                       &uf_emlrtBCI);
                }
                if (index_axis > t_opt_test.size(0) - 1) {
                  rtDynamicBoundsError(index_axis, 0, t_opt_test.size(0) - 1,
                                       &vf_emlrtBCI);
                }
                if (index_axis > J_opt_test.size(0) - 1) {
                  rtDynamicBoundsError(index_axis, 0, J_opt_test.size(0) - 1,
                                       &wf_emlrtBCI);
                }
                if (index_axis > solution_opt_test.size(0) - 1) {
                  rtDynamicBoundsError(index_axis, 0,
                                       solution_opt_test.size(0) - 1,
                                       &xf_emlrtBCI);
                }
                solve_O(P_init[index_axis], V_init[index_axis],
                        A_init[index_axis], P_wayp[index_axis],
                        V_wayp[index_axis], A_wayp[index_axis],
                        V_max[index_axis], V_min[index_axis], A_max[index_axis],
                        A_min[index_axis], J_max[index_axis], J_min[index_axis],
                        t_opt_test[index_axis].f1.data, iv,
                        J_opt_test[index_axis].f1.data, iv1,
                        solution_opt_test[index_axis].f1.data, &i);
                solution_opt_test[index_axis].f1.size[0] = i;
                J_opt_test[index_axis].f1.size[0] = iv1[0];
                J_opt_test[index_axis].f1.size[1] = iv1[1];
                t_opt_test[index_axis].f1.size[0] = iv[0];
                t_opt_test[index_axis].f1.size[1] = iv[1];
              }
            }
            printf("Debug: Ax[");
            fflush(stdout);
            printint(static_cast<double>(index_axis) + 1.0);
            printf("] Searching for timed solutions!\n");
            fflush(stdout);
            if (index_axis + 1 > P_init.size(0)) {
              rtDynamicBoundsError(index_axis + 1, 1, P_init.size(0),
                                   &yf_emlrtBCI);
            }
            if (index_axis + 1 > V_init.size(0)) {
              rtDynamicBoundsError(index_axis + 1, 1, V_init.size(0),
                                   &ag_emlrtBCI);
            }
            if (index_axis + 1 > A_init.size(0)) {
              rtDynamicBoundsError(index_axis + 1, 1, A_init.size(0),
                                   &bg_emlrtBCI);
            }
            if (index_axis + 1 > P_wayp.size(0)) {
              rtDynamicBoundsError(index_axis + 1, 1, P_wayp.size(0),
                                   &cg_emlrtBCI);
            }
            if (index_axis + 1 > V_wayp.size(0)) {
              rtDynamicBoundsError(index_axis + 1, 1, V_wayp.size(0),
                                   &dg_emlrtBCI);
            }
            if (index_axis + 1 > A_wayp.size(0)) {
              rtDynamicBoundsError(index_axis + 1, 1, A_wayp.size(0),
                                   &eg_emlrtBCI);
            }
            if (index_axis + 1 > V_max.size(0)) {
              rtDynamicBoundsError(index_axis + 1, 1, V_max.size(0),
                                   &fg_emlrtBCI);
            }
            if (index_axis + 1 > V_min.size(0)) {
              rtDynamicBoundsError(index_axis + 1, 1, V_min.size(0),
                                   &gg_emlrtBCI);
            }
            if (index_axis + 1 > A_max.size(0)) {
              rtDynamicBoundsError(index_axis + 1, 1, A_max.size(0),
                                   &hg_emlrtBCI);
            }
            if (index_axis + 1 > A_min.size(0)) {
              rtDynamicBoundsError(index_axis + 1, 1, A_min.size(0),
                                   &ig_emlrtBCI);
            }
            if (index_axis + 1 > J_max.size(0)) {
              rtDynamicBoundsError(index_axis + 1, 1, J_max.size(0),
                                   &jg_emlrtBCI);
            }
            if (index_axis + 1 > J_min.size(0)) {
              rtDynamicBoundsError(index_axis + 1, 1, J_min.size(0),
                                   &kg_emlrtBCI);
            }
            if (index_axis + 1 > t_sync.size(0)) {
              rtDynamicBoundsError(index_axis + 1, 1, t_sync.size(0),
                                   &lg_emlrtBCI);
            }
            if (index_axis + 1 > b_sync_V.size(0)) {
              rtDynamicBoundsError(index_axis + 1, 1, b_sync_V.size(0),
                                   &mg_emlrtBCI);
            }
            if (index_axis + 1 > b_sync_A.size(0)) {
              rtDynamicBoundsError(index_axis + 1, 1, b_sync_A.size(0),
                                   &ng_emlrtBCI);
            }
            if (index_axis + 1 > b_sync_J.size(0)) {
              rtDynamicBoundsError(index_axis + 1, 1, b_sync_J.size(0),
                                   &og_emlrtBCI);
            }
            if (index_axis + 1 > b_sync_W.size(0)) {
              rtDynamicBoundsError(index_axis + 1, 1, b_sync_W.size(0),
                                   &pg_emlrtBCI);
            }
            if (index_axis + 1 > direction.size(0)) {
              rtDynamicBoundsError(index_axis + 1, 1, direction.size(0),
                                   &qg_emlrtBCI);
            }
            if (index_axis > t_tim.size(0) - 1) {
              rtDynamicBoundsError(index_axis, 0, t_tim.size(0) - 1,
                                   &rg_emlrtBCI);
            }
            if (index_axis > J_tim.size(0) - 1) {
              rtDynamicBoundsError(index_axis, 0, J_tim.size(0) - 1,
                                   &sg_emlrtBCI);
            }
            if (index_axis > solution_tim.size(0) - 1) {
              rtDynamicBoundsError(index_axis, 0, solution_tim.size(0) - 1,
                                   &tg_emlrtBCI);
            }
            solve_T(P_init[index_axis], V_init[index_axis], A_init[index_axis],
                    P_wayp[index_axis], V_wayp[index_axis], A_wayp[index_axis],
                    V_max[index_axis], V_min[index_axis], A_max[index_axis],
                    A_min[index_axis], J_max[index_axis], J_min[index_axis],
                    t_sync[index_axis], b_sync_V[index_axis],
                    b_sync_A[index_axis], b_sync_J[index_axis],
                    b_sync_W[index_axis], direction[index_axis],
                    t_tim[index_axis].f1.data, iv, J_tim[index_axis].f1.data,
                    iv1, solution_tim[index_axis].f1.data, &i);
            solution_tim[index_axis].f1.size[0] = i;
            J_tim[index_axis].f1.size[0] = iv1[0];
            J_tim[index_axis].f1.size[1] = iv1[1];
            t_tim[index_axis].f1.size[0] = iv[0];
            t_tim[index_axis].f1.size[1] = iv[1];
            if (index_axis > solution_tim.size(0) - 1) {
              rtDynamicBoundsError(index_axis, 0, solution_tim.size(0) - 1,
                                   &ug_emlrtBCI);
            }
            if (solution_tim[index_axis].f1.size[0] != 0) {
              printf("Debug: Ax[");
              fflush(stdout);
              printint(static_cast<double>(index_axis) + 1.0);
              printf("] Found timed solution!\n");
              fflush(stdout);
              if (index_axis > t_tim.size(0) - 1) {
                rtDynamicBoundsError(index_axis, 0, t_tim.size(0) - 1,
                                     &vg_emlrtBCI);
              }
              i = t_tim[index_axis].f1.size[0];
              if (1 > i) {
                rtDynamicBoundsError(1, 1, i, &wg_emlrtBCI);
              }
              loop_ub = t.size(0) - 1;
              for (i = 0; i < 11; i++) {
                if (index_axis > loop_ub) {
                  rtDynamicBoundsError(index_axis, 0, loop_ub, &xg_emlrtBCI);
                }
                t[index_axis].f1[i] =
                    t_tim[index_axis].f1.data[t_tim[index_axis].f1.size[0] * i];
              }
              if (index_axis > J_tim.size(0) - 1) {
                rtDynamicBoundsError(index_axis, 0, J_tim.size(0) - 1,
                                     &yg_emlrtBCI);
              }
              i = J_tim[index_axis].f1.size[0];
              if (1 > i) {
                rtDynamicBoundsError(1, 1, i, &ah_emlrtBCI);
              }
              loop_ub = J.size(0) - 1;
              for (i = 0; i < 11; i++) {
                if (index_axis > loop_ub) {
                  rtDynamicBoundsError(index_axis, 0, loop_ub, &bh_emlrtBCI);
                }
                J[index_axis].f1[i] =
                    J_tim[index_axis].f1.data[J_tim[index_axis].f1.size[0] * i];
              }
              if (index_axis > solution_tim.size(0) - 1) {
                rtDynamicBoundsError(index_axis, 0, solution_tim.size(0) - 1,
                                     &ch_emlrtBCI);
              }
              i = solution_tim[index_axis].f1.size[0];
              if (1 > i) {
                rtDynamicBoundsError(1, 1, i, &dh_emlrtBCI);
              }
              if (index_axis + 1 > solution_out.size(0)) {
                rtDynamicBoundsError(index_axis + 1, 1, solution_out.size(0),
                                     &eh_emlrtBCI);
              }
              solution_out[index_axis] = solution_tim[index_axis].f1.data[0];
              index_axis++;
            } else {
              printf("Debug: Ax[");
              fflush(stdout);
              printint(static_cast<double>(index_axis) + 1.0);
              printf("] Considering slower solution!\n");
              fflush(stdout);
              if (index_axis > t_opt_test.size(0) - 1) {
                rtDynamicBoundsError(index_axis, 0, t_opt_test.size(0) - 1,
                                     &oh_emlrtBCI);
              }
              coder::circshift(t_opt_test[index_axis].f1.data,
                               t_opt_test[index_axis].f1.size);
              if (index_axis > J_opt_test.size(0) - 1) {
                rtDynamicBoundsError(index_axis, 0, J_opt_test.size(0) - 1,
                                     &ph_emlrtBCI);
              }
              coder::circshift(J_opt_test[index_axis].f1.data,
                               J_opt_test[index_axis].f1.size);
              if (index_axis > solution_opt_test.size(0) - 1) {
                rtDynamicBoundsError(
                    index_axis, 0, solution_opt_test.size(0) - 1, &qh_emlrtBCI);
              }
              coder::circshift(solution_opt_test[index_axis].f1.data,
                               &solution_opt_test[index_axis].f1.size[0]);
              b_i = t_opt_test.size(0) - 1;
              if (index_axis > t_opt_test.size(0) - 1) {
                rtDynamicBoundsError(index_axis, 0, t_opt_test.size(0) - 1,
                                     &rh_emlrtBCI);
              }
              loop_ub = t_opt_test[index_axis].f1.size[0];
              if (index_axis > t_opt_test.size(0) - 1) {
                rtDynamicBoundsError(index_axis, 0, t_opt_test.size(0) - 1,
                                     &rh_emlrtBCI);
              }
              i = t_opt_test[index_axis].f1.size[0];
              i1 = t_opt_test[index_axis].f1.size[0];
              if ((i1 < 1) || (i1 > i)) {
                rtDynamicBoundsError(i1, 1, i, &sh_emlrtBCI);
              }
              for (i = 0; i < 7; i++) {
                if (index_axis > b_i) {
                  rtDynamicBoundsError(index_axis, 0, b_i, &th_emlrtBCI);
                }
                t_opt_test[index_axis]
                    .f1.data[(loop_ub + t_opt_test[index_axis].f1.size[0] * i) -
                             1] = rtMinusInf;
              }
              b_i = J_opt_test.size(0) - 1;
              if (index_axis > J_opt_test.size(0) - 1) {
                rtDynamicBoundsError(index_axis, 0, J_opt_test.size(0) - 1,
                                     &uh_emlrtBCI);
              }
              loop_ub = J_opt_test[index_axis].f1.size[0];
              if (index_axis > J_opt_test.size(0) - 1) {
                rtDynamicBoundsError(index_axis, 0, J_opt_test.size(0) - 1,
                                     &uh_emlrtBCI);
              }
              i = J_opt_test[index_axis].f1.size[0];
              i1 = J_opt_test[index_axis].f1.size[0];
              if ((i1 < 1) || (i1 > i)) {
                rtDynamicBoundsError(i1, 1, i, &vh_emlrtBCI);
              }
              for (i = 0; i < 7; i++) {
                if (index_axis > b_i) {
                  rtDynamicBoundsError(index_axis, 0, b_i, &wh_emlrtBCI);
                }
                J_opt_test[index_axis]
                    .f1.data[(loop_ub + J_opt_test[index_axis].f1.size[0] * i) -
                             1] = rtInf;
              }
              if (index_axis > solution_opt_test.size(0) - 1) {
                rtDynamicBoundsError(
                    index_axis, 0, solution_opt_test.size(0) - 1, &xh_emlrtBCI);
              }
              i = solution_opt_test[index_axis].f1.size[0];
              i1 = solution_opt_test[index_axis].f1.size[0];
              if ((i1 < 1) || (i1 > i)) {
                rtDynamicBoundsError(i1, 1, i, &yh_emlrtBCI);
              }
              solution_opt_test[index_axis].f1.data[i1 - 1] = -1;
              exitg2 = true;
            }
          }
          if (b_guard1) {
            if (index_axis > t_opt_test.size(0) - 1) {
              rtDynamicBoundsError(index_axis, 0, t_opt_test.size(0) - 1,
                                   &fh_emlrtBCI);
            }
            i = t_opt_test[index_axis].f1.size[0];
            if (1 > i) {
              rtDynamicBoundsError(1, 1, i, &gh_emlrtBCI);
            }
            loop_ub = t.size(0) - 1;
            for (i = 0; i < 7; i++) {
              if (index_axis > loop_ub) {
                rtDynamicBoundsError(index_axis, 0, loop_ub, &hh_emlrtBCI);
              }
              t[index_axis].f1[i] =
                  t_opt_test[index_axis]
                      .f1.data[t_opt_test[index_axis].f1.size[0] * i];
            }
            if (index_axis > loop_ub) {
              rtDynamicBoundsError(index_axis, 0, loop_ub, &hh_emlrtBCI);
            }
            t[index_axis].f1[7] = 0.0;
            if (index_axis > loop_ub) {
              rtDynamicBoundsError(index_axis, 0, loop_ub, &hh_emlrtBCI);
            }
            t[index_axis].f1[8] = 0.0;
            if (index_axis > loop_ub) {
              rtDynamicBoundsError(index_axis, 0, loop_ub, &hh_emlrtBCI);
            }
            t[index_axis].f1[9] = 0.0;
            if (index_axis > loop_ub) {
              rtDynamicBoundsError(index_axis, 0, loop_ub, &hh_emlrtBCI);
            }
            t[index_axis].f1[10] = 0.0;
            if (index_axis > J_opt_test.size(0) - 1) {
              rtDynamicBoundsError(index_axis, 0, J_opt_test.size(0) - 1,
                                   &ih_emlrtBCI);
            }
            i = J_opt_test[index_axis].f1.size[0];
            if (1 > i) {
              rtDynamicBoundsError(1, 1, i, &jh_emlrtBCI);
            }
            loop_ub = J.size(0) - 1;
            for (i = 0; i < 7; i++) {
              if (index_axis > loop_ub) {
                rtDynamicBoundsError(index_axis, 0, loop_ub, &kh_emlrtBCI);
              }
              J[index_axis].f1[i] =
                  J_opt_test[index_axis]
                      .f1.data[J_opt_test[index_axis].f1.size[0] * i];
            }
            if (index_axis > loop_ub) {
              rtDynamicBoundsError(index_axis, 0, loop_ub, &kh_emlrtBCI);
            }
            J[index_axis].f1[7] = 0.0;
            if (index_axis > loop_ub) {
              rtDynamicBoundsError(index_axis, 0, loop_ub, &kh_emlrtBCI);
            }
            J[index_axis].f1[8] = 0.0;
            if (index_axis > loop_ub) {
              rtDynamicBoundsError(index_axis, 0, loop_ub, &kh_emlrtBCI);
            }
            J[index_axis].f1[9] = 0.0;
            if (index_axis > loop_ub) {
              rtDynamicBoundsError(index_axis, 0, loop_ub, &kh_emlrtBCI);
            }
            J[index_axis].f1[10] = 0.0;
            if (index_axis > solution_opt_test.size(0) - 1) {
              rtDynamicBoundsError(index_axis, 0, solution_opt_test.size(0) - 1,
                                   &lh_emlrtBCI);
            }
            i = solution_opt_test[index_axis].f1.size[0];
            if (1 > i) {
              rtDynamicBoundsError(1, 1, i, &mh_emlrtBCI);
            }
            if (index_axis + 1 > solution_out.size(0)) {
              rtDynamicBoundsError(index_axis + 1, 1, solution_out.size(0),
                                   &nh_emlrtBCI);
            }
            solution_out[index_axis] = solution_opt_test[index_axis].f1.data[0];
            index_axis++;
          }
        }
        for (b_i = 0; b_i < 100; b_i++) {
          solution_opt_test_all[b_i] = -1;
        }
        idx = 1.0;
        for (index_axis = 0; index_axis <= num_axes; index_axis++) {
          double d;
          if (index_axis > solution_opt_test.size(0) - 1) {
            rtDynamicBoundsError(index_axis, 0, solution_opt_test.size(0) - 1,
                                 &fi_emlrtBCI);
          }
          d = (idx +
               static_cast<double>(solution_opt_test[index_axis].f1.size[0])) -
              1.0;
          if (idx > d) {
            i1 = 0;
            i = 0;
          } else {
            if ((static_cast<int>(idx) < 1) || (static_cast<int>(idx) > 100)) {
              rtDynamicBoundsError(static_cast<int>(idx), 1, 100, &hi_emlrtBCI);
            }
            i1 = static_cast<int>(idx) - 1;
            if ((static_cast<int>(d) < 1) || (static_cast<int>(d) > 100)) {
              rtDynamicBoundsError(static_cast<int>(d), 1, 100, &ii_emlrtBCI);
            }
            i = static_cast<int>(d);
          }
          if (index_axis > solution_opt_test.size(0) - 1) {
            rtDynamicBoundsError(index_axis, 0, solution_opt_test.size(0) - 1,
                                 &gi_emlrtBCI);
          }
          i -= i1;
          rtSubAssignSizeCheck(&i, 1, &solution_opt_test[index_axis].f1.size[0],
                               1, &e_emlrtECI);
          loop_ub = solution_opt_test[index_axis].f1.size[0];
          for (i = 0; i < loop_ub; i++) {
            solution_opt_test_all[i1 + i] =
                solution_opt_test[index_axis].f1.data[i];
          }
          idx += static_cast<double>(solution_opt_test[index_axis].f1.size[0]);
        }
        if (1.0 > idx - 1.0) {
          loop_ub = 0;
        } else {
          if ((static_cast<int>(idx - 1.0) < 1) ||
              (static_cast<int>(idx - 1.0) > 100)) {
            rtDynamicBoundsError(static_cast<int>(idx - 1.0), 1, 100,
                                 &cj_emlrtBCI);
          }
          loop_ub = static_cast<int>(idx - 1.0);
        }
        b_i = solution_out.size(0);
        b_solution_out.set_size(solution_out.size(0));
        for (i = 0; i < b_i; i++) {
          b_solution_out[i] = (solution_out[i] != -1);
        }
        if (coder::b_all(b_solution_out)) {
          printf("Debug: Found timed solutions for all ");
          fflush(stdout);
          printint(static_cast<double>(num_axes + 1));
          printf(" axes after ");
          fflush(stdout);
          printint(static_cast<double>(e_unnamed_idx_0) + 1.0);
          printf(" iterations. Exiting synchronization!\n");
          fflush(stdout);
          exitg1 = true;
        } else {
          for (i = 0; i < loop_ub; i++) {
            solution_opt_test_all_data[i] = (solution_opt_test_all[i] != -1);
          }
          if (coder::intnnz(solution_opt_test_all_data, loop_ub) == 0) {
            printf("Error: Could not find a timed solution after ");
            fflush(stdout);
            printint(static_cast<double>(e_unnamed_idx_0) + 1.0);
            printf(" iterations. Exiting synchronization!\n");
            fflush(stdout);
            for (index_axis = 0; index_axis <= num_axes; index_axis++) {
              if (index_axis > t_opt.size(0) - 1) {
                rtDynamicBoundsError(index_axis, 0, t_opt.size(0) - 1,
                                     &ji_emlrtBCI);
              }
              i = t_opt[index_axis].f1.size[0];
              if (1 > i) {
                rtDynamicBoundsError(1, 1, i, &ki_emlrtBCI);
              }
              loop_ub = t.size(0) - 1;
              for (i = 0; i < 7; i++) {
                if (index_axis > loop_ub) {
                  rtDynamicBoundsError(index_axis, 0, loop_ub, &li_emlrtBCI);
                }
                t[index_axis].f1[i] =
                    t_opt[index_axis].f1.data[t_opt[index_axis].f1.size[0] * i];
              }
              if (index_axis > loop_ub) {
                rtDynamicBoundsError(index_axis, 0, loop_ub, &li_emlrtBCI);
              }
              t[index_axis].f1[7] = 0.0;
              if (index_axis > loop_ub) {
                rtDynamicBoundsError(index_axis, 0, loop_ub, &li_emlrtBCI);
              }
              t[index_axis].f1[8] = 0.0;
              if (index_axis > loop_ub) {
                rtDynamicBoundsError(index_axis, 0, loop_ub, &li_emlrtBCI);
              }
              t[index_axis].f1[9] = 0.0;
              if (index_axis > loop_ub) {
                rtDynamicBoundsError(index_axis, 0, loop_ub, &li_emlrtBCI);
              }
              t[index_axis].f1[10] = 0.0;
              if (index_axis > J_opt.size(0) - 1) {
                rtDynamicBoundsError(index_axis, 0, J_opt.size(0) - 1,
                                     &mi_emlrtBCI);
              }
              i = J_opt[index_axis].f1.size[0];
              if (1 > i) {
                rtDynamicBoundsError(1, 1, i, &ni_emlrtBCI);
              }
              loop_ub = J.size(0) - 1;
              for (i = 0; i < 7; i++) {
                if (index_axis > loop_ub) {
                  rtDynamicBoundsError(index_axis, 0, loop_ub, &oi_emlrtBCI);
                }
                J[index_axis].f1[i] =
                    J_opt[index_axis].f1.data[J_opt[index_axis].f1.size[0] * i];
              }
              if (index_axis > loop_ub) {
                rtDynamicBoundsError(index_axis, 0, loop_ub, &oi_emlrtBCI);
              }
              J[index_axis].f1[7] = 0.0;
              if (index_axis > loop_ub) {
                rtDynamicBoundsError(index_axis, 0, loop_ub, &oi_emlrtBCI);
              }
              J[index_axis].f1[8] = 0.0;
              if (index_axis > loop_ub) {
                rtDynamicBoundsError(index_axis, 0, loop_ub, &oi_emlrtBCI);
              }
              J[index_axis].f1[9] = 0.0;
              if (index_axis > loop_ub) {
                rtDynamicBoundsError(index_axis, 0, loop_ub, &oi_emlrtBCI);
              }
              J[index_axis].f1[10] = 0.0;
              if (index_axis > solution_opt.size(0) - 1) {
                rtDynamicBoundsError(index_axis, 0, solution_opt.size(0) - 1,
                                     &pi_emlrtBCI);
              }
              i = solution_opt[index_axis].f1.size[0];
              if (1 > i) {
                rtDynamicBoundsError(1, 1, i, &qi_emlrtBCI);
              }
              if (index_axis + 1 > solution_out.size(0)) {
                rtDynamicBoundsError(index_axis + 1, 1, solution_out.size(0),
                                     &ri_emlrtBCI);
              }
              solution_out[index_axis] = solution_opt[index_axis].f1.data[0];
            }
            exitg1 = true;
          } else if (static_cast<double>(e_unnamed_idx_0) + 1.0 ==
                     max_iterations) {
            printf("Error: Could not find a timed solution after maximum "
                   "iterations (");
            fflush(stdout);
            printint(max_iterations);
            printf("). Exiting synchronization!\n");
            fflush(stdout);
            for (index_axis = 0; index_axis <= num_axes; index_axis++) {
              if (index_axis > t_opt.size(0) - 1) {
                rtDynamicBoundsError(index_axis, 0, t_opt.size(0) - 1,
                                     &si_emlrtBCI);
              }
              i = t_opt[index_axis].f1.size[0];
              if (1 > i) {
                rtDynamicBoundsError(1, 1, i, &ti_emlrtBCI);
              }
              loop_ub = t.size(0) - 1;
              for (i = 0; i < 7; i++) {
                if (index_axis > loop_ub) {
                  rtDynamicBoundsError(index_axis, 0, loop_ub, &ui_emlrtBCI);
                }
                t[index_axis].f1[i] =
                    t_opt[index_axis].f1.data[t_opt[index_axis].f1.size[0] * i];
              }
              if (index_axis > loop_ub) {
                rtDynamicBoundsError(index_axis, 0, loop_ub, &ui_emlrtBCI);
              }
              t[index_axis].f1[7] = 0.0;
              if (index_axis > loop_ub) {
                rtDynamicBoundsError(index_axis, 0, loop_ub, &ui_emlrtBCI);
              }
              t[index_axis].f1[8] = 0.0;
              if (index_axis > loop_ub) {
                rtDynamicBoundsError(index_axis, 0, loop_ub, &ui_emlrtBCI);
              }
              t[index_axis].f1[9] = 0.0;
              if (index_axis > loop_ub) {
                rtDynamicBoundsError(index_axis, 0, loop_ub, &ui_emlrtBCI);
              }
              t[index_axis].f1[10] = 0.0;
              if (index_axis > J_opt.size(0) - 1) {
                rtDynamicBoundsError(index_axis, 0, J_opt.size(0) - 1,
                                     &vi_emlrtBCI);
              }
              i = J_opt[index_axis].f1.size[0];
              if (1 > i) {
                rtDynamicBoundsError(1, 1, i, &wi_emlrtBCI);
              }
              loop_ub = J.size(0) - 1;
              for (i = 0; i < 7; i++) {
                if (index_axis > loop_ub) {
                  rtDynamicBoundsError(index_axis, 0, loop_ub, &xi_emlrtBCI);
                }
                J[index_axis].f1[i] =
                    J_opt[index_axis].f1.data[J_opt[index_axis].f1.size[0] * i];
              }
              if (index_axis > loop_ub) {
                rtDynamicBoundsError(index_axis, 0, loop_ub, &xi_emlrtBCI);
              }
              J[index_axis].f1[7] = 0.0;
              if (index_axis > loop_ub) {
                rtDynamicBoundsError(index_axis, 0, loop_ub, &xi_emlrtBCI);
              }
              J[index_axis].f1[8] = 0.0;
              if (index_axis > loop_ub) {
                rtDynamicBoundsError(index_axis, 0, loop_ub, &xi_emlrtBCI);
              }
              J[index_axis].f1[9] = 0.0;
              if (index_axis > loop_ub) {
                rtDynamicBoundsError(index_axis, 0, loop_ub, &xi_emlrtBCI);
              }
              J[index_axis].f1[10] = 0.0;
              if (index_axis > solution_opt.size(0) - 1) {
                rtDynamicBoundsError(index_axis, 0, solution_opt.size(0) - 1,
                                     &yi_emlrtBCI);
              }
              i = solution_opt[index_axis].f1.size[0];
              if (1 > i) {
                rtDynamicBoundsError(1, 1, i, &aj_emlrtBCI);
              }
              if (index_axis + 1 > solution_out.size(0)) {
                rtDynamicBoundsError(index_axis + 1, 1, solution_out.size(0),
                                     &bj_emlrtBCI);
              }
              solution_out[index_axis] = solution_opt[index_axis].f1.data[0];
            }
            exitg1 = true;
          } else {
            e_unnamed_idx_0++;
          }
        }
      }
    } else {
      guard1 = true;
    }
  } else {
    guard1 = true;
  }
  if (guard1) {
    printf("Debug: No axis to be synchronized!\n");
    fflush(stdout);
    t.set_size(d_unnamed_idx_0);
    J.set_size(e_unnamed_idx_0);
    t_out.set_size(b_unnamed_idx_0);
    J_out.set_size(c_unnamed_idx_0);
    for (index_axis = 0; index_axis <= num_axes; index_axis++) {
      if (index_axis > t_opt.size(0) - 1) {
        rtDynamicBoundsError(index_axis, 0, t_opt.size(0) - 1, &dj_emlrtBCI);
      }
      i = t_opt[index_axis].f1.size[0];
      if (1 > i) {
        rtDynamicBoundsError(1, 1, i, &ej_emlrtBCI);
      }
      loop_ub = t.size(0) - 1;
      for (i = 0; i < 7; i++) {
        if (index_axis > loop_ub) {
          rtDynamicBoundsError(index_axis, 0, loop_ub, &fj_emlrtBCI);
        }
        t[index_axis].f1[i] =
            t_opt[index_axis].f1.data[t_opt[index_axis].f1.size[0] * i];
      }
      if (index_axis > loop_ub) {
        rtDynamicBoundsError(index_axis, 0, loop_ub, &fj_emlrtBCI);
      }
      t[index_axis].f1[7] = 0.0;
      if (index_axis > loop_ub) {
        rtDynamicBoundsError(index_axis, 0, loop_ub, &fj_emlrtBCI);
      }
      t[index_axis].f1[8] = 0.0;
      if (index_axis > loop_ub) {
        rtDynamicBoundsError(index_axis, 0, loop_ub, &fj_emlrtBCI);
      }
      t[index_axis].f1[9] = 0.0;
      if (index_axis > loop_ub) {
        rtDynamicBoundsError(index_axis, 0, loop_ub, &fj_emlrtBCI);
      }
      t[index_axis].f1[10] = 0.0;
      if (index_axis > J_opt.size(0) - 1) {
        rtDynamicBoundsError(index_axis, 0, J_opt.size(0) - 1, &gj_emlrtBCI);
      }
      i = J_opt[index_axis].f1.size[0];
      if (1 > i) {
        rtDynamicBoundsError(1, 1, i, &hj_emlrtBCI);
      }
      loop_ub = J.size(0) - 1;
      for (i = 0; i < 7; i++) {
        if (index_axis > loop_ub) {
          rtDynamicBoundsError(index_axis, 0, loop_ub, &ij_emlrtBCI);
        }
        J[index_axis].f1[i] =
            J_opt[index_axis].f1.data[J_opt[index_axis].f1.size[0] * i];
      }
      if (index_axis > loop_ub) {
        rtDynamicBoundsError(index_axis, 0, loop_ub, &ij_emlrtBCI);
      }
      J[index_axis].f1[7] = 0.0;
      if (index_axis > loop_ub) {
        rtDynamicBoundsError(index_axis, 0, loop_ub, &ij_emlrtBCI);
      }
      J[index_axis].f1[8] = 0.0;
      if (index_axis > loop_ub) {
        rtDynamicBoundsError(index_axis, 0, loop_ub, &ij_emlrtBCI);
      }
      J[index_axis].f1[9] = 0.0;
      if (index_axis > loop_ub) {
        rtDynamicBoundsError(index_axis, 0, loop_ub, &ij_emlrtBCI);
      }
      J[index_axis].f1[10] = 0.0;
      if (index_axis > solution_opt.size(0) - 1) {
        rtDynamicBoundsError(index_axis, 0, solution_opt.size(0) - 1,
                             &jj_emlrtBCI);
      }
      i = solution_opt[index_axis].f1.size[0];
      if (1 > i) {
        rtDynamicBoundsError(1, 1, i, &kj_emlrtBCI);
      }
      if (index_axis + 1 > solution_out.size(0)) {
        rtDynamicBoundsError(index_axis + 1, 1, solution_out.size(0),
                             &lj_emlrtBCI);
      }
      solution_out[index_axis] = solution_opt[index_axis].f1.data[0];
    }
  }
  for (index_axis = 0; index_axis <= num_axes; index_axis++) {
    signed char i2;
    if (index_axis > t_red.size(0) - 1) {
      rtDynamicBoundsError(index_axis, 0, t_red.size(0) - 1, &mj_emlrtBCI);
    }
    if (index_axis > t.size(0) - 1) {
      rtDynamicBoundsError(index_axis, 0, t.size(0) - 1, &nj_emlrtBCI);
    }
    if ((t_red[index_axis].f1.size[0] != 1) &&
        (t_red[index_axis].f1.size[0] != 0)) {
      h_rtErrorWithMessageID(i_emlrtRTEI.fName, i_emlrtRTEI.lineNo);
    }
    if (t_red[index_axis].f1.size[0] != 0) {
      i2 = 4;
    } else {
      i2 = 0;
    }
    b_i = t_out.size(0) - 1;
    if (index_axis > t_out.size(0) - 1) {
      rtDynamicBoundsError(index_axis, 0, t_out.size(0) - 1, &oj_emlrtBCI);
    }
    t_out[index_axis].f1.set_size(1, t_out[index_axis].f1.size(1));
    if (index_axis > b_i) {
      rtDynamicBoundsError(index_axis, 0, b_i, &oj_emlrtBCI);
    }
    t_out[index_axis].f1.set_size(t_out[index_axis].f1.size(0), i2 + 11);
    loop_ub = i2;
    for (i = 0; i < loop_ub; i++) {
      if (index_axis > b_i) {
        rtDynamicBoundsError(index_axis, 0, b_i, &oj_emlrtBCI);
      }
      t_out[index_axis].f1[t_out[index_axis].f1.size(0) * i] =
          t_red[index_axis].f1.data[i];
    }
    for (i = 0; i < 11; i++) {
      if (index_axis > b_i) {
        rtDynamicBoundsError(index_axis, 0, b_i, &oj_emlrtBCI);
      }
      t_out[index_axis].f1[t_out[index_axis].f1.size(0) * (i + i2)] =
          t[index_axis].f1[i];
    }
    if (index_axis > J_red.size(0) - 1) {
      rtDynamicBoundsError(index_axis, 0, J_red.size(0) - 1, &pj_emlrtBCI);
    }
    if (index_axis > J.size(0) - 1) {
      rtDynamicBoundsError(index_axis, 0, J.size(0) - 1, &qj_emlrtBCI);
    }
    if ((J_red[index_axis].f1.size[0] != 1) &&
        (J_red[index_axis].f1.size[0] != 0)) {
      h_rtErrorWithMessageID(i_emlrtRTEI.fName, i_emlrtRTEI.lineNo);
    }
    if (J_red[index_axis].f1.size[0] != 0) {
      i2 = 4;
    } else {
      i2 = 0;
    }
    b_i = J_out.size(0) - 1;
    if (index_axis > J_out.size(0) - 1) {
      rtDynamicBoundsError(index_axis, 0, J_out.size(0) - 1, &rj_emlrtBCI);
    }
    J_out[index_axis].f1.set_size(1, J_out[index_axis].f1.size(1));
    if (index_axis > b_i) {
      rtDynamicBoundsError(index_axis, 0, b_i, &rj_emlrtBCI);
    }
    J_out[index_axis].f1.set_size(J_out[index_axis].f1.size(0), i2 + 11);
    loop_ub = i2;
    for (i = 0; i < loop_ub; i++) {
      if (index_axis > b_i) {
        rtDynamicBoundsError(index_axis, 0, b_i, &rj_emlrtBCI);
      }
      J_out[index_axis].f1[J_out[index_axis].f1.size(0) * i] =
          J_red[index_axis].f1.data[i];
    }
    for (i = 0; i < 11; i++) {
      if (index_axis > b_i) {
        rtDynamicBoundsError(index_axis, 0, b_i, &rj_emlrtBCI);
      }
      J_out[index_axis].f1[J_out[index_axis].f1.size(0) * (i + i2)] =
          J[index_axis].f1[i];
    }
  }
}

// End of code generation (synchronize_trajectory.cpp)
