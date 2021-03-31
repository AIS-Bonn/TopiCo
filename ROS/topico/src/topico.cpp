//
// Student License - for use by students to meet course requirements and
// perform academic research at degree granting institutions only.  Not
// for government, commercial, or other organizational use.
//
// topico.cpp
//
// Code generation for function 'topico'
//

// Include files
#include "topico.h"
#include "check_inputs.h"
#include "combineVectorElements.h"
#include "construct_setp_struct.h"
#include "evaluate_to_time.h"
#include "evolve_waypoints.h"
#include "indexShapeCheck.h"
#include "minOrMax.h"
#include "printint.h"
#include "repmat.h"
#include "rollout_t.h"
#include "rotate_jerk.h"
#include "rt_nonfinite.h"
#include "sum.h"
#include "synchronize_trajectory.h"
#include "topico_data.h"
#include "topico_initialize.h"
#include "topico_internal_types.h"
#include "topico_rtwutil.h"
#include "topico_types.h"
#include "coder_array.h"
#include "rt_nonfinite.h"
#include <cmath>
#include <sstream>
#include <stdexcept>
#include <stdio.h>
#include <string>

// Function Declarations
static void rtNonNegativeError(const double aPositive,
                               const rtDoubleCheckInfo *aInfo);

// Function Definitions
static void rtNonNegativeError(const double aPositive,
                               const rtDoubleCheckInfo *aInfo)
{
  std::stringstream outStream;
  ((outStream << "Value ") << aPositive)
      << " is not greater than or equal to zero.\nExiting to prevent memory "
         "corruption.";
  outStream << "\n";
  ((((outStream << "Error in ") << aInfo->fName) << " (line ") << aInfo->lineNo)
      << ")";
  throw std::runtime_error(outStream.str());
}

void topico(const coder::array<double, 2U> &State_start,
            const coder::array<double, 3U> &Waypoints,
            const coder::array<double, 2U> &V_max_in,
            const coder::array<double, 2U> &V_min_in,
            const coder::array<double, 2U> &A_max_in,
            const coder::array<double, 2U> &A_min_in,
            const coder::array<double, 2U> &J_max_in,
            const coder::array<double, 2U> &J_min_in,
            const coder::array<double, 1U> &A_global,
            const coder::array<bool, 2U> &b_sync_V_in,
            const coder::array<bool, 2U> &b_sync_A_in,
            const coder::array<bool, 2U> &b_sync_J_in,
            const coder::array<bool, 2U> &b_sync_W_in,
            const coder::array<bool, 2U> &b_rotate_in,
            const coder::array<bool, 2U> &b_hard_V_lim_in,
            const coder::array<bool, 2U> &b_catch_up_in,
            const coder::array<signed char, 2U> &direction_in,
            double ts_rollout, coder::array<struct0_T, 2U> &J_setp_struct,
            coder::array<int, 2U> &solution_out,
            coder::array<double, 2U> &T_waypoints, coder::array<double, 2U> &P,
            coder::array<double, 2U> &V, coder::array<double, 2U> &A,
            coder::array<double, 2U> &J, coder::array<double, 2U> &t)
{
  static rtBoundsCheckInfo ac_emlrtBCI = {
      -1,                                            // iFirst
      -1,                                            // iLast
      86,                                            // lineNo
      28,                                            // colNo
      "A_min_in",                                    // aName
      "topico",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/topico.m", // pName
      0                                              // checkKind
  };
  static rtBoundsCheckInfo ad_emlrtBCI = {
      -1,                                            // iFirst
      -1,                                            // iLast
      114,                                           // lineNo
      29,                                            // colNo
      "A_curr_pred",                                 // aName
      "topico",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/topico.m", // pName
      0                                              // checkKind
  };
  static rtBoundsCheckInfo ae_emlrtBCI = {
      -1,                                            // iFirst
      -1,                                            // iLast
      133,                                           // lineNo
      31,                                            // colNo
      "t_setp",                                      // aName
      "topico",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/topico.m", // pName
      0                                              // checkKind
  };
  static rtBoundsCheckInfo af_emlrtBCI = {
      -1,                                            // iFirst
      -1,                                            // iLast
      143,                                           // lineNo
      195,                                           // colNo
      "J_traj",                                      // aName
      "topico",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/topico.m", // pName
      0                                              // checkKind
  };
  static rtBoundsCheckInfo bc_emlrtBCI = {
      -1,                                            // iFirst
      -1,                                            // iLast
      87,                                            // lineNo
      28,                                            // colNo
      "J_max_in",                                    // aName
      "topico",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/topico.m", // pName
      0                                              // checkKind
  };
  static rtBoundsCheckInfo bd_emlrtBCI = {
      -1,                                            // iFirst
      -1,                                            // iLast
      115,                                           // lineNo
      64,                                            // colNo
      "theta",                                       // aName
      "topico",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/topico.m", // pName
      0                                              // checkKind
  };
  static rtBoundsCheckInfo be_emlrtBCI = {
      -1,                                            // iFirst
      -1,                                            // iLast
      133,                                           // lineNo
      49,                                            // colNo
      "t_traj",                                      // aName
      "topico",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/topico.m", // pName
      0                                              // checkKind
  };
  static rtBoundsCheckInfo bf_emlrtBCI = {
      -1,                                            // iFirst
      -1,                                            // iLast
      143,                                           // lineNo
      97,                                            // colNo
      "P_curr",                                      // aName
      "topico",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/topico.m", // pName
      0                                              // checkKind
  };
  static rtBoundsCheckInfo cc_emlrtBCI = {
      -1,                                            // iFirst
      -1,                                            // iLast
      88,                                            // lineNo
      28,                                            // colNo
      "J_min_in",                                    // aName
      "topico",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/topico.m", // pName
      0                                              // checkKind
  };
  static rtBoundsCheckInfo cd_emlrtBCI = {
      -1,                                            // iFirst
      -1,                                            // iLast
      115,                                           // lineNo
      87,                                            // colNo
      "P_wayp_pred",                                 // aName
      "topico",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/topico.m", // pName
      0                                              // checkKind
  };
  static rtBoundsCheckInfo ce_emlrtBCI = {
      -1,                                            // iFirst
      -1,                                            // iLast
      133,                                           // lineNo
      13,                                            // colNo
      "t_setp",                                      // aName
      "topico",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/topico.m", // pName
      0                                              // checkKind
  };
  static rtBoundsCheckInfo cf_emlrtBCI = {
      -1,                                            // iFirst
      -1,                                            // iLast
      143,                                           // lineNo
      116,                                           // colNo
      "V_curr",                                      // aName
      "topico",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/topico.m", // pName
      0                                              // checkKind
  };
  static rtBoundsCheckInfo dc_emlrtBCI = {
      -1,                                            // iFirst
      -1,                                            // iLast
      90,                                            // lineNo
      38,                                            // colNo
      "b_sync_V_in",                                 // aName
      "topico",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/topico.m", // pName
      0                                              // checkKind
  };
  static rtBoundsCheckInfo dd_emlrtBCI = {
      -1,                                            // iFirst
      -1,                                            // iLast
      115,                                           // lineNo
      29,                                            // colNo
      "P_wayp_pred",                                 // aName
      "topico",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/topico.m", // pName
      0                                              // checkKind
  };
  static rtBoundsCheckInfo de_emlrtBCI = {
      -1,                                            // iFirst
      -1,                                            // iLast
      134,                                           // lineNo
      56,                                            // colNo
      "J_traj",                                      // aName
      "topico",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/topico.m", // pName
      0                                              // checkKind
  };
  static rtBoundsCheckInfo df_emlrtBCI = {
      -1,                                            // iFirst
      -1,                                            // iLast
      143,                                           // lineNo
      135,                                           // colNo
      "A_curr",                                      // aName
      "topico",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/topico.m", // pName
      0                                              // checkKind
  };
  static rtBoundsCheckInfo ec_emlrtBCI = {
      -1,                                            // iFirst
      -1,                                            // iLast
      91,                                            // lineNo
      38,                                            // colNo
      "b_sync_A_in",                                 // aName
      "topico",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/topico.m", // pName
      0                                              // checkKind
  };
  static rtBoundsCheckInfo ed_emlrtBCI = {
      -1,                                            // iFirst
      -1,                                            // iLast
      116,                                           // lineNo
      64,                                            // colNo
      "theta",                                       // aName
      "topico",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/topico.m", // pName
      0                                              // checkKind
  };
  static rtBoundsCheckInfo ee_emlrtBCI = {
      -1,                                            // iFirst
      -1,                                            // iLast
      134,                                           // lineNo
      20,                                            // colNo
      "J_setp",                                      // aName
      "topico",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/topico.m", // pName
      0                                              // checkKind
  };
  static rtBoundsCheckInfo ef_emlrtBCI = {
      -1,                                            // iFirst
      -1,                                            // iLast
      143,                                           // lineNo
      215,                                           // colNo
      "T_waypoints",                                 // aName
      "topico",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/topico.m", // pName
      0                                              // checkKind
  };
  static rtBoundsCheckInfo fc_emlrtBCI = {
      -1,                                            // iFirst
      -1,                                            // iLast
      92,                                            // lineNo
      38,                                            // colNo
      "b_sync_J_in",                                 // aName
      "topico",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/topico.m", // pName
      0                                              // checkKind
  };
  static rtBoundsCheckInfo fd_emlrtBCI = {
      -1,                                            // iFirst
      -1,                                            // iLast
      116,                                           // lineNo
      87,                                            // colNo
      "V_wayp_pred",                                 // aName
      "topico",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/topico.m", // pName
      0                                              // checkKind
  };
  static rtBoundsCheckInfo fe_emlrtBCI = {
      -1,                                            // iFirst
      -1,                                            // iLast
      134,                                           // lineNo
      31,                                            // colNo
      "J_setp",                                      // aName
      "topico",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/topico.m", // pName
      0                                              // checkKind
  };
  static rtBoundsCheckInfo ff_emlrtBCI = {
      -1,                                            // iFirst
      -1,                                            // iLast
      143,                                           // lineNo
      18,                                            // colNo
      "P_curr",                                      // aName
      "topico",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/topico.m", // pName
      0                                              // checkKind
  };
  static rtBoundsCheckInfo gc_emlrtBCI = {
      -1,                                            // iFirst
      -1,                                            // iLast
      93,                                            // lineNo
      38,                                            // colNo
      "b_sync_W_in",                                 // aName
      "topico",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/topico.m", // pName
      0                                              // checkKind
  };
  static rtBoundsCheckInfo gd_emlrtBCI = {
      -1,                                            // iFirst
      -1,                                            // iLast
      116,                                           // lineNo
      29,                                            // colNo
      "V_wayp_pred",                                 // aName
      "topico",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/topico.m", // pName
      0                                              // checkKind
  };
  static rtBoundsCheckInfo ge_emlrtBCI = {
      -1,                                            // iFirst
      -1,                                            // iLast
      134,                                           // lineNo
      49,                                            // colNo
      "J_traj",                                      // aName
      "topico",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/topico.m", // pName
      0                                              // checkKind
  };
  static rtBoundsCheckInfo gf_emlrtBCI = {
      -1,                                            // iFirst
      -1,                                            // iLast
      143,                                           // lineNo
      37,                                            // colNo
      "V_curr",                                      // aName
      "topico",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/topico.m", // pName
      0                                              // checkKind
  };
  static rtBoundsCheckInfo hc_emlrtBCI = {
      -1,                                            // iFirst
      -1,                                            // iLast
      94,                                            // lineNo
      38,                                            // colNo
      "b_rotate_in",                                 // aName
      "topico",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/topico.m", // pName
      0                                              // checkKind
  };
  static rtBoundsCheckInfo hd_emlrtBCI = {
      -1,                                            // iFirst
      -1,                                            // iLast
      117,                                           // lineNo
      64,                                            // colNo
      "theta",                                       // aName
      "topico",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/topico.m", // pName
      0                                              // checkKind
  };
  static rtBoundsCheckInfo he_emlrtBCI = {
      -1,                                            // iFirst
      -1,                                            // iLast
      134,                                           // lineNo
      13,                                            // colNo
      "J_setp",                                      // aName
      "topico",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/topico.m", // pName
      0                                              // checkKind
  };
  static rtBoundsCheckInfo hf_emlrtBCI = {
      -1,                                            // iFirst
      -1,                                            // iLast
      143,                                           // lineNo
      56,                                            // colNo
      "A_curr",                                      // aName
      "topico",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/topico.m", // pName
      0                                              // checkKind
  };
  static rtBoundsCheckInfo ic_emlrtBCI = {
      -1,                                            // iFirst
      -1,                                            // iLast
      95,                                            // lineNo
      42,                                            // colNo
      "b_hard_V_lim_in",                             // aName
      "topico",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/topico.m", // pName
      0                                              // checkKind
  };
  static rtBoundsCheckInfo id_emlrtBCI = {
      -1,                                            // iFirst
      -1,                                            // iLast
      117,                                           // lineNo
      87,                                            // colNo
      "A_wayp_pred",                                 // aName
      "topico",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/topico.m", // pName
      0                                              // checkKind
  };
  static rtBoundsCheckInfo ie_emlrtBCI = {
      -1,                                            // iFirst
      -1,                                            // iLast
      135,                                           // lineNo
      65,                                            // colNo
      "t_traj",                                      // aName
      "topico",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/topico.m", // pName
      0                                              // checkKind
  };
  static rtBoundsCheckInfo if_emlrtBCI = {
      -1,                                            // iFirst
      -1,                                            // iLast
      103,                                           // lineNo
      45,                                            // colNo
      "T_waypoints",                                 // aName
      "topico",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/topico.m", // pName
      0                                              // checkKind
  };
  static rtBoundsCheckInfo jc_emlrtBCI = {
      -1,                                            // iFirst
      -1,                                            // iLast
      96,                                            // lineNo
      40,                                            // colNo
      "b_catch_up_in",                               // aName
      "topico",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/topico.m", // pName
      0                                              // checkKind
  };
  static rtBoundsCheckInfo jd_emlrtBCI = {
      -1,                                            // iFirst
      -1,                                            // iLast
      117,                                           // lineNo
      29,                                            // colNo
      "A_wayp_pred",                                 // aName
      "topico",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/topico.m", // pName
      0                                              // checkKind
  };
  static rtBoundsCheckInfo je_emlrtBCI = {
      -1,                                            // iFirst
      -1,                                            // iLast
      135,                                           // lineNo
      77,                                            // colNo
      "t_traj{index_axis}",                          // aName
      "topico",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/topico.m", // pName
      0                                              // checkKind
  };
  static rtBoundsCheckInfo jf_emlrtBCI = {
      -1,                                            // iFirst
      -1,                                            // iLast
      103,                                           // lineNo
      47,                                            // colNo
      "T_waypoints",                                 // aName
      "topico",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/topico.m", // pName
      0                                              // checkKind
  };
  static rtBoundsCheckInfo kc_emlrtBCI = {
      -1,                                            // iFirst
      -1,                                            // iLast
      97,                                            // lineNo
      39,                                            // colNo
      "direction_in",                                // aName
      "topico",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/topico.m", // pName
      0                                              // checkKind
  };
  static rtBoundsCheckInfo kd_emlrtBCI = {
      -1,                                            // iFirst
      -1,                                            // iLast
      122,                                           // lineNo
      39,                                            // colNo
      "solution_out",                                // aName
      "topico",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/topico.m", // pName
      0                                              // checkKind
  };
  static rtBoundsCheckInfo ke_emlrtBCI = {
      -1,                                            // iFirst
      -1,                                            // iLast
      135,                                           // lineNo
      13,                                            // colNo
      "T_waypoints",                                 // aName
      "topico",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/topico.m", // pName
      0                                              // checkKind
  };
  static rtBoundsCheckInfo kf_emlrtBCI = {
      -1,                                            // iFirst
      -1,                                            // iLast
      103,                                           // lineNo
      87,                                            // colNo
      "T_waypoints",                                 // aName
      "topico",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/topico.m", // pName
      0                                              // checkKind
  };
  static rtBoundsCheckInfo lc_emlrtBCI = {
      -1,                                            // iFirst
      -1,                                            // iLast
      109,                                           // lineNo
      17,                                            // colNo
      "b_rotate",                                    // aName
      "topico",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/topico.m", // pName
      0                                              // checkKind
  };
  static rtBoundsCheckInfo ld_emlrtBCI = {
      -1,                                            // iFirst
      -1,                                            // iLast
      126,                                           // lineNo
      17,                                            // colNo
      "b_rotate",                                    // aName
      "topico",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/topico.m", // pName
      0                                              // checkKind
  };
  static rtBoundsCheckInfo le_emlrtBCI = {
      -1,                                            // iFirst
      -1,                                            // iLast
      138,                                           // lineNo
      17,                                            // colNo
      "VP_wayp",                                     // aName
      "topico",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/topico.m", // pName
      0                                              // checkKind
  };
  static rtBoundsCheckInfo lf_emlrtBCI = {
      -1,                                            // iFirst
      -1,                                            // iLast
      103,                                           // lineNo
      89,                                            // colNo
      "T_waypoints",                                 // aName
      "topico",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/topico.m", // pName
      0                                              // checkKind
  };
  static rtBoundsCheckInfo mc_emlrtBCI = {
      -1,                                            // iFirst
      -1,                                            // iLast
      111,                                           // lineNo
      103,                                           // colNo
      "P_wayp_pred",                                 // aName
      "topico",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/topico.m", // pName
      0                                              // checkKind
  };
  static rtBoundsCheckInfo md_emlrtBCI = {
      -1,                                            // iFirst
      -1,                                            // iLast
      128,                                           // lineNo
      111,                                           // colNo
      "t_traj",                                      // aName
      "topico",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/topico.m", // pName
      0                                              // checkKind
  };
  static rtBoundsCheckInfo me_emlrtBCI = {
      -1,                                            // iFirst
      -1,                                            // iLast
      138,                                           // lineNo
      45,                                            // colNo
      "AP_wayp",                                     // aName
      "topico",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/topico.m", // pName
      0                                              // checkKind
  };
  static rtBoundsCheckInfo mf_emlrtBCI = {
      -1,                                            // iFirst
      -1,                                            // iLast
      74,                                            // lineNo
      94,                                            // colNo
      "T_waypoints",                                 // aName
      "topico",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/topico.m", // pName
      0                                              // checkKind
  };
  static rtBoundsCheckInfo nb_emlrtBCI = {
      -1,                                            // iFirst
      -1,                                            // iLast
      45,                                            // lineNo
      28,                                            // colNo
      "t_setp",                                      // aName
      "topico",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/topico.m", // pName
      0                                              // checkKind
  };
  static rtBoundsCheckInfo nc_emlrtBCI = {
      -1,                                            // iFirst
      -1,                                            // iLast
      111,                                           // lineNo
      120,                                           // colNo
      "P_curr_pred",                                 // aName
      "topico",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/topico.m", // pName
      0                                              // checkKind
  };
  static rtBoundsCheckInfo nd_emlrtBCI = {
      -1,                                            // iFirst
      -1,                                            // iLast
      128,                                           // lineNo
      121,                                           // colNo
      "J_traj",                                      // aName
      "topico",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/topico.m", // pName
      0                                              // checkKind
  };
  static rtBoundsCheckInfo ne_emlrtBCI = {
      -1,                                            // iFirst
      -1,                                            // iLast
      138,                                           // lineNo
      87,                                            // colNo
      "P_wayp",                                      // aName
      "topico",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/topico.m", // pName
      0                                              // checkKind
  };
  static rtBoundsCheckInfo nf_emlrtBCI = {
      -1,                                            // iFirst
      -1,                                            // iLast
      74,                                            // lineNo
      96,                                            // colNo
      "T_waypoints",                                 // aName
      "topico",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/topico.m", // pName
      0                                              // checkKind
  };
  static rtBoundsCheckInfo ob_emlrtBCI = {
      -1,                                            // iFirst
      -1,                                            // iLast
      48,                                            // lineNo
      20,                                            // colNo
      "t_setp",                                      // aName
      "topico",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/topico.m", // pName
      0                                              // checkKind
  };
  static rtBoundsCheckInfo oc_emlrtBCI = {
      -1,                                            // iFirst
      -1,                                            // iLast
      111,                                           // lineNo
      55,                                            // colNo
      "P_wayp_pred",                                 // aName
      "topico",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/topico.m", // pName
      0                                              // checkKind
  };
  static rtBoundsCheckInfo od_emlrtBCI = {
      -1,                                            // iFirst
      -1,                                            // iLast
      128,                                           // lineNo
      87,                                            // colNo
      "theta",                                       // aName
      "topico",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/topico.m", // pName
      0                                              // checkKind
  };
  static rtBoundsCheckInfo oe_emlrtBCI = {
      -1,                                            // iFirst
      -1,                                            // iLast
      138,                                           // lineNo
      117,                                           // colNo
      "V_wayp",                                      // aName
      "topico",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/topico.m", // pName
      0                                              // checkKind
  };
  static rtBoundsCheckInfo pb_emlrtBCI = {
      -1,                                            // iFirst
      -1,                                            // iLast
      48,                                            // lineNo
      31,                                            // colNo
      "t_setp",                                      // aName
      "topico",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/topico.m", // pName
      0                                              // checkKind
  };
  static rtBoundsCheckInfo pc_emlrtBCI = {
      -1,                                            // iFirst
      -1,                                            // iLast
      111,                                           // lineNo
      79,                                            // colNo
      "P_curr_pred",                                 // aName
      "topico",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/topico.m", // pName
      0                                              // checkKind
  };
  static rtBoundsCheckInfo pd_emlrtBCI = {
      -1,                                            // iFirst
      -1,                                            // iLast
      128,                                           // lineNo
      124,                                           // colNo
      "t_traj",                                      // aName
      "topico",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/topico.m", // pName
      0                                              // checkKind
  };
  static rtBoundsCheckInfo pe_emlrtBCI = {
      -1,                                            // iFirst
      -1,                                            // iLast
      138,                                           // lineNo
      147,                                           // colNo
      "A_wayp",                                      // aName
      "topico",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/topico.m", // pName
      0                                              // checkKind
  };
  static rtBoundsCheckInfo qb_emlrtBCI = {
      -1,                                            // iFirst
      -1,                                            // iLast
      48,                                            // lineNo
      13,                                            // colNo
      "t_setp",                                      // aName
      "topico",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/topico.m", // pName
      0                                              // checkKind
  };
  static rtBoundsCheckInfo qc_emlrtBCI = {
      -1,                                            // iFirst
      -1,                                            // iLast
      111,                                           // lineNo
      17,                                            // colNo
      "theta",                                       // aName
      "topico",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/topico.m", // pName
      0                                              // checkKind
  };
  static rtBoundsCheckInfo qd_emlrtBCI = {
      -1,                                            // iFirst
      -1,                                            // iLast
      128,                                           // lineNo
      141,                                           // colNo
      "J_traj",                                      // aName
      "topico",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/topico.m", // pName
      0                                              // checkKind
  };
  static rtBoundsCheckInfo qe_emlrtBCI = {
      -1,                                            // iFirst
      -1,                                            // iLast
      138,                                           // lineNo
      177,                                           // colNo
      "solution_out",                                // aName
      "topico",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/topico.m", // pName
      0                                              // checkKind
  };
  static rtBoundsCheckInfo rb_emlrtBCI = {
      -1,                                            // iFirst
      -1,                                            // iLast
      45,                                            // lineNo
      28,                                            // colNo
      "J_setp",                                      // aName
      "topico",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/topico.m", // pName
      0                                              // checkKind
  };
  static rtBoundsCheckInfo rc_emlrtBCI = {
      -1,                                            // iFirst
      -1,                                            // iLast
      112,                                           // lineNo
      64,                                            // colNo
      "theta",                                       // aName
      "topico",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/topico.m", // pName
      0                                              // checkKind
  };
  static rtBoundsCheckInfo rd_emlrtBCI = {
      -1,                                            // iFirst
      -1,                                            // iLast
      128,                                           // lineNo
      25,                                            // colNo
      "t_traj",                                      // aName
      "topico",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/topico.m", // pName
      0                                              // checkKind
  };
  static rtBoundsCheckInfo re_emlrtBCI = {
      -1,                                            // iFirst
      -1,                                            // iLast
      138,                                           // lineNo
      188,                                           // colNo
      "solution_out",                                // aName
      "topico",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/topico.m", // pName
      0                                              // checkKind
  };
  static rtBoundsCheckInfo sb_emlrtBCI = {
      -1,                                            // iFirst
      -1,                                            // iLast
      55,                                            // lineNo
      20,                                            // colNo
      "J_setp",                                      // aName
      "topico",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/topico.m", // pName
      0                                              // checkKind
  };
  static rtBoundsCheckInfo sc_emlrtBCI = {
      -1,                                            // iFirst
      -1,                                            // iLast
      112,                                           // lineNo
      87,                                            // colNo
      "P_curr_pred",                                 // aName
      "topico",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/topico.m", // pName
      0                                              // checkKind
  };
  static rtBoundsCheckInfo sd_emlrtBCI = {
      -1,                                            // iFirst
      -1,                                            // iLast
      128,                                           // lineNo
      35,                                            // colNo
      "J_traj",                                      // aName
      "topico",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/topico.m", // pName
      0                                              // checkKind
  };
  static rtBoundsCheckInfo se_emlrtBCI = {
      -1,                                            // iFirst
      -1,                                            // iLast
      139,                                           // lineNo
      38,                                            // colNo
      "Waypoints",                                   // aName
      "topico",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/topico.m", // pName
      0                                              // checkKind
  };
  static rtBoundsCheckInfo tb_emlrtBCI = {
      -1,                                            // iFirst
      -1,                                            // iLast
      55,                                            // lineNo
      31,                                            // colNo
      "J_setp",                                      // aName
      "topico",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/topico.m", // pName
      0                                              // checkKind
  };
  static rtBoundsCheckInfo tc_emlrtBCI = {
      -1,                                            // iFirst
      -1,                                            // iLast
      112,                                           // lineNo
      29,                                            // colNo
      "P_curr_pred",                                 // aName
      "topico",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/topico.m", // pName
      0                                              // checkKind
  };
  static rtBoundsCheckInfo td_emlrtBCI = {
      -1,                                            // iFirst
      -1,                                            // iLast
      128,                                           // lineNo
      45,                                            // colNo
      "t_traj",                                      // aName
      "topico",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/topico.m", // pName
      0                                              // checkKind
  };
  static rtBoundsCheckInfo te_emlrtBCI = {
      -1,                                            // iFirst
      -1,                                            // iLast
      139,                                           // lineNo
      17,                                            // colNo
      "P_curr",                                      // aName
      "topico",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/topico.m", // pName
      0                                              // checkKind
  };
  static rtBoundsCheckInfo ub_emlrtBCI = {
      -1,                                            // iFirst
      -1,                                            // iLast
      55,                                            // lineNo
      13,                                            // colNo
      "J_setp",                                      // aName
      "topico",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/topico.m", // pName
      0                                              // checkKind
  };
  static rtBoundsCheckInfo uc_emlrtBCI = {
      -1,                                            // iFirst
      -1,                                            // iLast
      113,                                           // lineNo
      64,                                            // colNo
      "theta",                                       // aName
      "topico",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/topico.m", // pName
      0                                              // checkKind
  };
  static rtBoundsCheckInfo ud_emlrtBCI = {
      -1,                                            // iFirst
      -1,                                            // iLast
      128,                                           // lineNo
      38,                                            // colNo
      "t_traj",                                      // aName
      "topico",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/topico.m", // pName
      0                                              // checkKind
  };
  static rtBoundsCheckInfo ue_emlrtBCI = {
      -1,                                            // iFirst
      -1,                                            // iLast
      140,                                           // lineNo
      38,                                            // colNo
      "Waypoints",                                   // aName
      "topico",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/topico.m", // pName
      0                                              // checkKind
  };
  static rtBoundsCheckInfo vb_emlrtBCI = {
      -1,                                            // iFirst
      -1,                                            // iLast
      74,                                            // lineNo
      60,                                            // colNo
      "Waypoints",                                   // aName
      "topico",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/topico.m", // pName
      0                                              // checkKind
  };
  static rtBoundsCheckInfo vc_emlrtBCI = {
      -1,                                            // iFirst
      -1,                                            // iLast
      113,                                           // lineNo
      87,                                            // colNo
      "V_curr_pred",                                 // aName
      "topico",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/topico.m", // pName
      0                                              // checkKind
  };
  static rtBoundsCheckInfo vd_emlrtBCI = {
      -1,                                            // iFirst
      -1,                                            // iLast
      128,                                           // lineNo
      62,                                            // colNo
      "J_traj",                                      // aName
      "topico",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/topico.m", // pName
      0                                              // checkKind
  };
  static rtBoundsCheckInfo ve_emlrtBCI = {
      -1,                                            // iFirst
      -1,                                            // iLast
      140,                                           // lineNo
      17,                                            // colNo
      "V_curr",                                      // aName
      "topico",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/topico.m", // pName
      0                                              // checkKind
  };
  static rtBoundsCheckInfo wb_emlrtBCI = {
      -1,                                            // iFirst
      -1,                                            // iLast
      83,                                            // lineNo
      28,                                            // colNo
      "V_max_in",                                    // aName
      "topico",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/topico.m", // pName
      0                                              // checkKind
  };
  static rtBoundsCheckInfo wc_emlrtBCI = {
      -1,                                            // iFirst
      -1,                                            // iLast
      113,                                           // lineNo
      29,                                            // colNo
      "V_curr_pred",                                 // aName
      "topico",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/topico.m", // pName
      0                                              // checkKind
  };
  static rtBoundsCheckInfo wd_emlrtBCI = {
      -1,                                            // iFirst
      -1,                                            // iLast
      128,                                           // lineNo
      55,                                            // colNo
      "J_traj",                                      // aName
      "topico",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/topico.m", // pName
      0                                              // checkKind
  };
  static rtBoundsCheckInfo we_emlrtBCI = {
      -1,                                            // iFirst
      -1,                                            // iLast
      141,                                           // lineNo
      38,                                            // colNo
      "Waypoints",                                   // aName
      "topico",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/topico.m", // pName
      0                                              // checkKind
  };
  static rtBoundsCheckInfo xb_emlrtBCI = {
      -1,                                            // iFirst
      -1,                                            // iLast
      84,                                            // lineNo
      28,                                            // colNo
      "V_min_in",                                    // aName
      "topico",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/topico.m", // pName
      0                                              // checkKind
  };
  static rtBoundsCheckInfo xc_emlrtBCI = {
      -1,                                            // iFirst
      -1,                                            // iLast
      114,                                           // lineNo
      64,                                            // colNo
      "theta",                                       // aName
      "topico",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/topico.m", // pName
      0                                              // checkKind
  };
  static rtBoundsCheckInfo xd_emlrtBCI = {
      -1,                                            // iFirst
      -1,                                            // iLast
      133,                                           // lineNo
      56,                                            // colNo
      "t_traj",                                      // aName
      "topico",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/topico.m", // pName
      0                                              // checkKind
  };
  static rtBoundsCheckInfo xe_emlrtBCI = {
      -1,                                            // iFirst
      -1,                                            // iLast
      141,                                           // lineNo
      17,                                            // colNo
      "A_curr",                                      // aName
      "topico",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/topico.m", // pName
      0                                              // checkKind
  };
  static rtBoundsCheckInfo yb_emlrtBCI = {
      -1,                                            // iFirst
      -1,                                            // iLast
      85,                                            // lineNo
      28,                                            // colNo
      "A_max_in",                                    // aName
      "topico",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/topico.m", // pName
      0                                              // checkKind
  };
  static rtBoundsCheckInfo yc_emlrtBCI = {
      -1,                                            // iFirst
      -1,                                            // iLast
      114,                                           // lineNo
      87,                                            // colNo
      "A_curr_pred",                                 // aName
      "topico",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/topico.m", // pName
      0                                              // checkKind
  };
  static rtBoundsCheckInfo yd_emlrtBCI = {
      -1,                                            // iFirst
      -1,                                            // iLast
      133,                                           // lineNo
      20,                                            // colNo
      "t_setp",                                      // aName
      "topico",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/topico.m", // pName
      0                                              // checkKind
  };
  static rtBoundsCheckInfo ye_emlrtBCI = {
      -1,                                            // iFirst
      -1,                                            // iLast
      143,                                           // lineNo
      176,                                           // colNo
      "t_traj",                                      // aName
      "topico",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/topico.m", // pName
      0                                              // checkKind
  };
  static rtDoubleCheckInfo emlrtDCI = {
      107,                                           // lineNo
      9,                                             // colNo
      "topico",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/topico.m", // pName
      4                                              // checkKind
  };
  static rtEqualityCheckInfo c_emlrtECI = {
      -1,                                                      // nDims
      45,                                                      // lineNo
      22,                                                      // colNo
      "compensate_global",                                     // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/compensate_global.m" // pName
  };
  static rtEqualityCheckInfo d_emlrtECI = {
      -1,                                                      // nDims
      45,                                                      // lineNo
      5,                                                       // colNo
      "compensate_global",                                     // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/compensate_global.m" // pName
  };
  static rtEqualityCheckInfo e_emlrtECI = {
      2,                                                       // nDims
      46,                                                      // lineNo
      17,                                                      // colNo
      "compensate_global",                                     // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/compensate_global.m" // pName
  };
  static rtEqualityCheckInfo f_emlrtECI = {
      2,                                                       // nDims
      47,                                                      // lineNo
      17,                                                      // colNo
      "compensate_global",                                     // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/compensate_global.m" // pName
  };
  static rtEqualityCheckInfo g_emlrtECI = {
      -1,                                                  // nDims
      42,                                                  // lineNo
      11,                                                  // colNo
      "predict_state",                                     // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/predict_state.m" // pName
  };
  static rtEqualityCheckInfo h_emlrtECI = {
      -1,                                                  // nDims
      43,                                                  // lineNo
      11,                                                  // colNo
      "predict_state",                                     // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/predict_state.m" // pName
  };
  static rtEqualityCheckInfo i_emlrtECI = {
      -1,                                                  // nDims
      48,                                                  // lineNo
      12,                                                  // colNo
      "predict_state",                                     // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/predict_state.m" // pName
  };
  static rtEqualityCheckInfo j_emlrtECI = {
      -1,                                                  // nDims
      49,                                                  // lineNo
      12,                                                  // colNo
      "predict_state",                                     // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/predict_state.m" // pName
  };
  static rtEqualityCheckInfo k_emlrtECI = {
      -1,                                                  // nDims
      50,                                                  // lineNo
      12,                                                  // colNo
      "predict_state",                                     // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/predict_state.m" // pName
  };
  static rtEqualityCheckInfo l_emlrtECI = {
      -1,                                                  // nDims
      51,                                                  // lineNo
      12,                                                  // colNo
      "predict_state",                                     // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/predict_state.m" // pName
  };
  static rtEqualityCheckInfo m_emlrtECI = {
      -1,                                           // nDims
      103,                                          // lineNo
      22,                                           // colNo
      "topico",                                     // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/topico.m" // pName
  };
  static rtEqualityCheckInfo n_emlrtECI = {
      -1,                                           // nDims
      122,                                          // lineNo
      24,                                           // colNo
      "topico",                                     // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/topico.m" // pName
  };
  coder::array<cell_wrap_0, 2U> J_setp;
  coder::array<cell_wrap_0, 2U> t_setp;
  coder::array<cell_wrap_0, 1U> J_traj;
  coder::array<cell_wrap_0, 1U> t_traj;
  coder::array<double, 2U> Waypoint_evolved;
  coder::array<double, 2U> b_A_max_in;
  coder::array<double, 2U> b_A_min_in;
  coder::array<double, 2U> b_State_start;
  coder::array<double, 2U> b_T_waypoints;
  coder::array<double, 2U> b_Waypoints;
  coder::array<double, 2U> b_t_traj;
  coder::array<double, 2U> r1;
  coder::array<double, 2U> r2;
  coder::array<double, 2U> r3;
  coder::array<double, 2U> t19_time;
  coder::array<double, 1U> A_curr;
  coder::array<double, 1U> A_curr_pred;
  coder::array<double, 1U> A_wayp_pred;
  coder::array<double, 1U> P_curr;
  coder::array<double, 1U> P_curr_pred;
  coder::array<double, 1U> P_wayp_pred;
  coder::array<double, 1U> V_curr;
  coder::array<double, 1U> V_curr_pred;
  coder::array<double, 1U> V_wayp_pred;
  coder::array<double, 1U> b_J_max_in;
  coder::array<double, 1U> b_J_min_in;
  coder::array<double, 1U> b_V_max_in;
  coder::array<double, 1U> b_V_min_in;
  coder::array<double, 1U> c_A_max_in;
  coder::array<double, 1U> c_A_min_in;
  coder::array<double, 1U> maxval;
  coder::array<double, 1U> theta;
  coder::array<int, 1U> r;
  coder::array<signed char, 1U> b_direction_in;
  coder::array<bool, 1U> b_b_hard_V_lim_in;
  coder::array<bool, 1U> b_b_sync_A_in;
  coder::array<bool, 1U> b_b_sync_J_in;
  coder::array<bool, 1U> b_b_sync_V_in;
  coder::array<bool, 1U> b_b_sync_W_in;
  double theta_tmp;
  int iv[2];
  int iv1[2];
  int b_solution_out;
  int i;
  int index_waypoint;
  int num_axes;
  int unnamed_idx_0;
  int unnamed_idx_1;
  if (!isInitialized_topico_mex) {
    topico_initialize();
  }
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
  num_axes = Waypoints.size(0) - 1;
  unnamed_idx_0 = Waypoints.size(0);
  unnamed_idx_1 = Waypoints.size(2);
  t_setp.set_size(Waypoints.size(0), Waypoints.size(2));
  for (i = 0; i < unnamed_idx_0 * unnamed_idx_1; i++) {
    b_solution_out = t_setp.size(0) * t_setp.size(1);
    if (i > b_solution_out - 1) {
      rtDynamicBoundsError(i, 0, b_solution_out - 1, &nb_emlrtBCI);
    }
    t_setp[i].f1.set_size(0, t_setp[i].f1.size(1));
    if (i > b_solution_out - 1) {
      rtDynamicBoundsError(i, 0, b_solution_out - 1, &nb_emlrtBCI);
    }
    t_setp[i].f1.set_size(t_setp[i].f1.size(0), 0);
  }
  i = Waypoints.size(2);
  t_setp.set_size(Waypoints.size(0), Waypoints.size(2));
  for (index_waypoint = 0; index_waypoint < i; index_waypoint++) {
    // coder
    for (unnamed_idx_0 = 0; unnamed_idx_0 <= num_axes; unnamed_idx_0++) {
      if (unnamed_idx_0 > t_setp.size(0) - 1) {
        rtDynamicBoundsError(unnamed_idx_0, 0, t_setp.size(0) - 1,
                             &ob_emlrtBCI);
      }
      if (index_waypoint > t_setp.size(1) - 1) {
        rtDynamicBoundsError(index_waypoint, 0, t_setp.size(1) - 1,
                             &pb_emlrtBCI);
      }
      t_setp[unnamed_idx_0 + t_setp.size(0) * index_waypoint].f1.set_size(
          1,
          t_setp[unnamed_idx_0 + t_setp.size(0) * index_waypoint].f1.size(1));
      if (unnamed_idx_0 > t_setp.size(0) - 1) {
        rtDynamicBoundsError(unnamed_idx_0, 0, t_setp.size(0) - 1,
                             &ob_emlrtBCI);
      }
      if (index_waypoint > t_setp.size(1) - 1) {
        rtDynamicBoundsError(index_waypoint, 0, t_setp.size(1) - 1,
                             &pb_emlrtBCI);
      }
      t_setp[unnamed_idx_0 + t_setp.size(0) * index_waypoint].f1.set_size(
          t_setp[unnamed_idx_0 + t_setp.size(0) * index_waypoint].f1.size(0),
          1);
      if (unnamed_idx_0 > t_setp.size(0) - 1) {
        rtDynamicBoundsError(unnamed_idx_0, 0, t_setp.size(0) - 1,
                             &ob_emlrtBCI);
      }
      if (index_waypoint > t_setp.size(1) - 1) {
        rtDynamicBoundsError(index_waypoint, 0, t_setp.size(1) - 1,
                             &pb_emlrtBCI);
      }
      if (unnamed_idx_0 > t_setp.size(0) - 1) {
        rtDynamicBoundsError(unnamed_idx_0, 0, t_setp.size(0) - 1,
                             &ob_emlrtBCI);
      }
      if (index_waypoint > t_setp.size(1) - 1) {
        rtDynamicBoundsError(index_waypoint, 0, t_setp.size(1) - 1,
                             &pb_emlrtBCI);
      }
      if (unnamed_idx_0 > t_setp.size(0) - 1) {
        rtDynamicBoundsError(unnamed_idx_0, 0, t_setp.size(0) - 1,
                             &qb_emlrtBCI);
      }
      if (index_waypoint > t_setp.size(1) - 1) {
        rtDynamicBoundsError(index_waypoint, 0, t_setp.size(1) - 1,
                             &qb_emlrtBCI);
      }
      t_setp[unnamed_idx_0 + t_setp.size(0) * index_waypoint].f1[0] = 0.0;
    }
  }
  unnamed_idx_0 = Waypoints.size(0);
  unnamed_idx_1 = Waypoints.size(2);
  J_setp.set_size(Waypoints.size(0), Waypoints.size(2));
  for (i = 0; i < unnamed_idx_0 * unnamed_idx_1; i++) {
    b_solution_out = J_setp.size(0) * J_setp.size(1);
    if (i > b_solution_out - 1) {
      rtDynamicBoundsError(i, 0, b_solution_out - 1, &rb_emlrtBCI);
    }
    J_setp[i].f1.set_size(0, J_setp[i].f1.size(1));
    if (i > b_solution_out - 1) {
      rtDynamicBoundsError(i, 0, b_solution_out - 1, &rb_emlrtBCI);
    }
    J_setp[i].f1.set_size(J_setp[i].f1.size(0), 0);
  }
  i = Waypoints.size(2);
  J_setp.set_size(Waypoints.size(0), Waypoints.size(2));
  for (index_waypoint = 0; index_waypoint < i; index_waypoint++) {
    // coder
    for (unnamed_idx_0 = 0; unnamed_idx_0 <= num_axes; unnamed_idx_0++) {
      if (unnamed_idx_0 > J_setp.size(0) - 1) {
        rtDynamicBoundsError(unnamed_idx_0, 0, J_setp.size(0) - 1,
                             &sb_emlrtBCI);
      }
      if (index_waypoint > J_setp.size(1) - 1) {
        rtDynamicBoundsError(index_waypoint, 0, J_setp.size(1) - 1,
                             &tb_emlrtBCI);
      }
      J_setp[unnamed_idx_0 + J_setp.size(0) * index_waypoint].f1.set_size(
          1,
          J_setp[unnamed_idx_0 + J_setp.size(0) * index_waypoint].f1.size(1));
      if (unnamed_idx_0 > J_setp.size(0) - 1) {
        rtDynamicBoundsError(unnamed_idx_0, 0, J_setp.size(0) - 1,
                             &sb_emlrtBCI);
      }
      if (index_waypoint > J_setp.size(1) - 1) {
        rtDynamicBoundsError(index_waypoint, 0, J_setp.size(1) - 1,
                             &tb_emlrtBCI);
      }
      J_setp[unnamed_idx_0 + J_setp.size(0) * index_waypoint].f1.set_size(
          J_setp[unnamed_idx_0 + J_setp.size(0) * index_waypoint].f1.size(0),
          1);
      if (unnamed_idx_0 > J_setp.size(0) - 1) {
        rtDynamicBoundsError(unnamed_idx_0, 0, J_setp.size(0) - 1,
                             &sb_emlrtBCI);
      }
      if (index_waypoint > J_setp.size(1) - 1) {
        rtDynamicBoundsError(index_waypoint, 0, J_setp.size(1) - 1,
                             &tb_emlrtBCI);
      }
      if (unnamed_idx_0 > J_setp.size(0) - 1) {
        rtDynamicBoundsError(unnamed_idx_0, 0, J_setp.size(0) - 1,
                             &sb_emlrtBCI);
      }
      if (index_waypoint > J_setp.size(1) - 1) {
        rtDynamicBoundsError(index_waypoint, 0, J_setp.size(1) - 1,
                             &tb_emlrtBCI);
      }
      if (unnamed_idx_0 > J_setp.size(0) - 1) {
        rtDynamicBoundsError(unnamed_idx_0, 0, J_setp.size(0) - 1,
                             &ub_emlrtBCI);
      }
      if (index_waypoint > J_setp.size(1) - 1) {
        rtDynamicBoundsError(index_waypoint, 0, J_setp.size(1) - 1,
                             &ub_emlrtBCI);
      }
      J_setp[unnamed_idx_0 + J_setp.size(0) * index_waypoint].f1[0] = 0.0;
    }
  }
  T_waypoints.set_size(Waypoints.size(0), Waypoints.size(2));
  b_solution_out = Waypoints.size(0) * Waypoints.size(2);
  for (i = 0; i < b_solution_out; i++) {
    T_waypoints[i] = 0.0;
  }
  solution_out.set_size(Waypoints.size(0), Waypoints.size(2));
  b_solution_out = Waypoints.size(0) * Waypoints.size(2);
  for (i = 0; i < b_solution_out; i++) {
    solution_out[i] = -1;
  }
  check_inputs(State_start, Waypoints, V_max_in, V_min_in, A_max_in, A_min_in,
               J_max_in, J_min_in, A_global, b_sync_V_in, b_sync_A_in,
               b_sync_J_in, b_sync_W_in, b_rotate_in, b_hard_V_lim_in,
               b_catch_up_in, direction_in, ts_rollout);
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
  b_State_start.set_size(State_start.size(0), 3);
  b_solution_out = State_start.size(0) * 3;
  for (i = 0; i < b_solution_out; i++) {
    b_State_start[i] = State_start[i];
  }
  if (State_start.size(0) != A_global.size(0)) {
    rtSizeEq1DError(State_start.size(0), A_global.size(0), &c_emlrtECI);
  }
  rtSubAssignSizeCheck(((coder::array<double, 2U> *)&State_start)->size(), 1,
                       ((coder::array<double, 2U> *)&State_start)->size(), 1,
                       &d_emlrtECI);
  b_solution_out = State_start.size(0);
  for (i = 0; i < b_solution_out; i++) {
    b_State_start[i + b_State_start.size(0) * 2] =
        State_start[i + State_start.size(0) * 2] + A_global[i];
  }
  coder::repmat(A_global, static_cast<double>(A_max_in.size(1)), b_A_max_in);
  iv[0] = (*(int(*)[2])((coder::array<double, 2U> *)&A_max_in)->size())[0];
  iv[1] = (*(int(*)[2])((coder::array<double, 2U> *)&A_max_in)->size())[1];
  iv1[0] = (*(int(*)[2])b_A_max_in.size())[0];
  iv1[1] = (*(int(*)[2])b_A_max_in.size())[1];
  rtSizeEqNDCheck(&iv[0], &iv1[0], &e_emlrtECI);
  b_solution_out = A_max_in.size(0) * A_max_in.size(1);
  b_A_max_in.set_size(A_max_in.size(0), A_max_in.size(1));
  for (i = 0; i < b_solution_out; i++) {
    b_A_max_in[i] = A_max_in[i] + b_A_max_in[i];
  }
  coder::repmat(A_global, static_cast<double>(A_max_in.size(1)), b_A_min_in);
  iv[0] = (*(int(*)[2])((coder::array<double, 2U> *)&A_min_in)->size())[0];
  iv[1] = (*(int(*)[2])((coder::array<double, 2U> *)&A_min_in)->size())[1];
  iv1[0] = (*(int(*)[2])b_A_min_in.size())[0];
  iv1[1] = (*(int(*)[2])b_A_min_in.size())[1];
  rtSizeEqNDCheck(&iv[0], &iv1[0], &f_emlrtECI);
  b_solution_out = A_min_in.size(0) * A_min_in.size(1);
  b_A_min_in.set_size(A_min_in.size(0), A_min_in.size(1));
  for (i = 0; i < b_solution_out; i++) {
    b_A_min_in[i] = A_min_in[i] + b_A_min_in[i];
  }
  b_solution_out = b_State_start.size(0);
  P_curr.set_size(b_State_start.size(0));
  for (i = 0; i < b_solution_out; i++) {
    P_curr[i] = b_State_start[i];
  }
  b_solution_out = b_State_start.size(0);
  V_curr.set_size(b_State_start.size(0));
  for (i = 0; i < b_solution_out; i++) {
    V_curr[i] = b_State_start[i + b_State_start.size(0)];
  }
  b_solution_out = b_State_start.size(0);
  A_curr.set_size(b_State_start.size(0));
  for (i = 0; i < b_solution_out; i++) {
    A_curr[i] = b_State_start[i + b_State_start.size(0) * 2];
  }
  i = Waypoints.size(2);
  J_setp.set_size(Waypoints.size(0), Waypoints.size(2));
  t_setp.set_size(Waypoints.size(0), Waypoints.size(2));
  for (index_waypoint = 0; index_waypoint < i; index_waypoint++) {
    double b_maxval;
    int i1;
    int i2;
    int i3;
    printf("Debug: Generating trajectory for waypoint ");
    fflush(stdout);
    printint(static_cast<double>(index_waypoint) + 1.0);
    printf("!\n");
    fflush(stdout);
    //  Evolve
    if (1 > index_waypoint) {
      b_solution_out = 0;
    } else {
      if (1 > T_waypoints.size(1)) {
        rtDynamicBoundsError(1, 1, T_waypoints.size(1), &mf_emlrtBCI);
      }
      if (index_waypoint > T_waypoints.size(1)) {
        rtDynamicBoundsError(index_waypoint, 1, T_waypoints.size(1),
                             &nf_emlrtBCI);
      }
      b_solution_out = index_waypoint;
    }
    unnamed_idx_0 = T_waypoints.size(0);
    b_T_waypoints.set_size(T_waypoints.size(0), b_solution_out);
    for (i1 = 0; i1 < b_solution_out; i1++) {
      for (i2 = 0; i2 < unnamed_idx_0; i2++) {
        b_T_waypoints[i2 + b_T_waypoints.size(0) * i1] =
            T_waypoints[i2 + T_waypoints.size(0) * i1];
      }
    }
    coder::sum(b_T_waypoints, maxval);
    if (index_waypoint + 1 > Waypoints.size(2)) {
      rtDynamicBoundsError(index_waypoint + 1, 1, Waypoints.size(2),
                           &vb_emlrtBCI);
    }
    b_solution_out = Waypoints.size(0);
    b_Waypoints.set_size(Waypoints.size(0), 5);
    for (i1 = 0; i1 < 5; i1++) {
      for (i2 = 0; i2 < b_solution_out; i2++) {
        b_Waypoints[i2 + b_Waypoints.size(0) * i1] =
            Waypoints[(i2 + Waypoints.size(0) * i1) +
                      Waypoints.size(0) * 5 * index_waypoint];
      }
    }
    evolve_waypoints(b_Waypoints, maxval, Waypoint_evolved);
    //  Assign Variables
    if (index_waypoint + 1 > V_max_in.size(1)) {
      rtDynamicBoundsError(index_waypoint + 1, 1, V_max_in.size(1),
                           &wb_emlrtBCI);
    }
    if (index_waypoint + 1 > V_min_in.size(1)) {
      rtDynamicBoundsError(index_waypoint + 1, 1, V_min_in.size(1),
                           &xb_emlrtBCI);
    }
    if (index_waypoint + 1 > b_A_max_in.size(1)) {
      rtDynamicBoundsError(index_waypoint + 1, 1, b_A_max_in.size(1),
                           &yb_emlrtBCI);
    }
    if (index_waypoint + 1 > b_A_min_in.size(1)) {
      rtDynamicBoundsError(index_waypoint + 1, 1, b_A_min_in.size(1),
                           &ac_emlrtBCI);
    }
    if (index_waypoint + 1 > J_max_in.size(1)) {
      rtDynamicBoundsError(index_waypoint + 1, 1, J_max_in.size(1),
                           &bc_emlrtBCI);
    }
    if (index_waypoint + 1 > J_min_in.size(1)) {
      rtDynamicBoundsError(index_waypoint + 1, 1, J_min_in.size(1),
                           &cc_emlrtBCI);
    }
    if (index_waypoint + 1 > b_sync_V_in.size(1)) {
      rtDynamicBoundsError(index_waypoint + 1, 1, b_sync_V_in.size(1),
                           &dc_emlrtBCI);
    }
    if (index_waypoint + 1 > b_sync_A_in.size(1)) {
      rtDynamicBoundsError(index_waypoint + 1, 1, b_sync_A_in.size(1),
                           &ec_emlrtBCI);
    }
    if (index_waypoint + 1 > b_sync_J_in.size(1)) {
      rtDynamicBoundsError(index_waypoint + 1, 1, b_sync_J_in.size(1),
                           &fc_emlrtBCI);
    }
    if (index_waypoint + 1 > b_sync_W_in.size(1)) {
      rtDynamicBoundsError(index_waypoint + 1, 1, b_sync_W_in.size(1),
                           &gc_emlrtBCI);
    }
    if (index_waypoint + 1 > b_rotate_in.size(1)) {
      rtDynamicBoundsError(index_waypoint + 1, 1, b_rotate_in.size(1),
                           &hc_emlrtBCI);
    }
    if (index_waypoint + 1 > b_hard_V_lim_in.size(1)) {
      rtDynamicBoundsError(index_waypoint + 1, 1, b_hard_V_lim_in.size(1),
                           &ic_emlrtBCI);
    }
    if (index_waypoint + 1 > b_catch_up_in.size(1)) {
      rtDynamicBoundsError(index_waypoint + 1, 1, b_catch_up_in.size(1),
                           &jc_emlrtBCI);
    }
    if (index_waypoint + 1 > direction_in.size(1)) {
      rtDynamicBoundsError(index_waypoint + 1, 1, direction_in.size(1),
                           &kc_emlrtBCI);
    }
    //  Prediction
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
    b_solution_out = V_curr.size(0);
    if (V_curr.size(0) != Waypoint_evolved.size(0)) {
      rtSizeEq1DError(V_curr.size(0), Waypoint_evolved.size(0), &g_emlrtECI);
    }
    V_curr_pred.set_size(V_curr.size(0));
    for (i1 = 0; i1 < b_solution_out; i1++) {
      V_curr_pred[i1] =
          V_curr[i1] - Waypoint_evolved[i1 + Waypoint_evolved.size(0) * 3];
    }
    b_solution_out = A_curr.size(0);
    if (A_curr.size(0) != Waypoint_evolved.size(0)) {
      rtSizeEq1DError(A_curr.size(0), Waypoint_evolved.size(0), &h_emlrtECI);
    }
    A_curr_pred.set_size(A_curr.size(0));
    for (i1 = 0; i1 < b_solution_out; i1++) {
      A_curr_pred[i1] =
          A_curr[i1] - Waypoint_evolved[i1 + Waypoint_evolved.size(0) * 4];
    }
    b_solution_out = Waypoint_evolved.size(0);
    V_wayp_pred.set_size(Waypoint_evolved.size(0));
    for (i1 = 0; i1 < b_solution_out; i1++) {
      V_wayp_pred[i1] = Waypoint_evolved[i1 + Waypoint_evolved.size(0)] -
                        Waypoint_evolved[i1 + Waypoint_evolved.size(0) * 3];
    }
    b_solution_out = Waypoint_evolved.size(0);
    A_wayp_pred.set_size(Waypoint_evolved.size(0));
    for (i1 = 0; i1 < b_solution_out; i1++) {
      A_wayp_pred[i1] = Waypoint_evolved[i1 + Waypoint_evolved.size(0) * 2] -
                        Waypoint_evolved[i1 + Waypoint_evolved.size(0) * 4];
    }
    if (V_max_in.size(0) != Waypoint_evolved.size(0)) {
      rtSizeEq1DError(V_max_in.size(0), Waypoint_evolved.size(0), &i_emlrtECI);
    }
    if (V_min_in.size(0) != Waypoint_evolved.size(0)) {
      rtSizeEq1DError(V_min_in.size(0), Waypoint_evolved.size(0), &j_emlrtECI);
    }
    if (b_A_max_in.size(0) != Waypoint_evolved.size(0)) {
      rtSizeEq1DError(b_A_max_in.size(0), Waypoint_evolved.size(0),
                      &k_emlrtECI);
    }
    if (b_A_min_in.size(0) != Waypoint_evolved.size(0)) {
      rtSizeEq1DError(b_A_min_in.size(0), Waypoint_evolved.size(0),
                      &l_emlrtECI);
    }
    b_solution_out = P_curr.size(0);
    P_curr_pred.set_size(P_curr.size(0));
    for (i1 = 0; i1 < b_solution_out; i1++) {
      P_curr_pred[i1] = P_curr[i1];
    }
    b_solution_out = Waypoint_evolved.size(0);
    P_wayp_pred.set_size(Waypoint_evolved.size(0));
    for (i1 = 0; i1 < b_solution_out; i1++) {
      P_wayp_pred[i1] = Waypoint_evolved[i1];
    }
    //  Catch up
    if (1 > index_waypoint) {
      b_solution_out = 0;
      unnamed_idx_0 = 0;
    } else {
      if (1 > T_waypoints.size(1)) {
        rtDynamicBoundsError(1, 1, T_waypoints.size(1), &if_emlrtBCI);
      }
      if (index_waypoint > T_waypoints.size(1)) {
        rtDynamicBoundsError(index_waypoint, 1, T_waypoints.size(1),
                             &jf_emlrtBCI);
      }
      b_solution_out = index_waypoint;
      if (1 > T_waypoints.size(1)) {
        rtDynamicBoundsError(1, 1, T_waypoints.size(1), &kf_emlrtBCI);
      }
      if (index_waypoint > T_waypoints.size(1)) {
        rtDynamicBoundsError(index_waypoint, 1, T_waypoints.size(1),
                             &lf_emlrtBCI);
      }
      unnamed_idx_0 = index_waypoint;
    }
    unnamed_idx_1 = T_waypoints.size(0);
    b_T_waypoints.set_size(T_waypoints.size(0), unnamed_idx_0);
    for (i1 = 0; i1 < unnamed_idx_0; i1++) {
      for (i2 = 0; i2 < unnamed_idx_1; i2++) {
        b_T_waypoints[i2 + b_T_waypoints.size(0) * i1] =
            T_waypoints[i2 + T_waypoints.size(0) * i1];
      }
    }
    coder::sum(b_T_waypoints, maxval);
    if (maxval.size(0) != b_catch_up_in.size(0)) {
      rtSizeEq1DError(maxval.size(0), b_catch_up_in.size(0), &m_emlrtECI);
    }
    unnamed_idx_0 = T_waypoints.size(0);
    b_T_waypoints.set_size(T_waypoints.size(0), b_solution_out);
    for (i1 = 0; i1 < b_solution_out; i1++) {
      for (i2 = 0; i2 < unnamed_idx_0; i2++) {
        b_T_waypoints[i2 + b_T_waypoints.size(0) * i1] =
            T_waypoints[i2 + T_waypoints.size(0) * i1];
      }
    }
    coder::sum(b_T_waypoints, theta);
    b_maxval = coder::internal::maximum(theta);
    //  Rotation
    if (num_axes < 0) {
      rtNonNegativeError(-1.0, &emlrtDCI);
    }
    theta.set_size(num_axes);
    for (i1 = 0; i1 < num_axes; i1++) {
      theta[i1] = 0.0;
    }
    for (unnamed_idx_1 = 0; unnamed_idx_1 < num_axes; unnamed_idx_1++) {
      if (unnamed_idx_1 + 1 > b_rotate_in.size(0)) {
        rtDynamicBoundsError(unnamed_idx_1 + 1, 1, b_rotate_in.size(0),
                             &lc_emlrtBCI);
      }
      if (b_rotate_in[unnamed_idx_1 + b_rotate_in.size(0) * index_waypoint]) {
        double b_theta_tmp;
        double d;
        double d1;
        double output_idx_1;
        if (1 > P_wayp_pred.size(0)) {
          rtDynamicBoundsError(1, 1, P_wayp_pred.size(0), &mc_emlrtBCI);
        }
        if (1 > P_curr_pred.size(0)) {
          rtDynamicBoundsError(1, 1, P_curr_pred.size(0), &nc_emlrtBCI);
        }
        if (unnamed_idx_1 + 2 > P_wayp_pred.size(0)) {
          rtDynamicBoundsError(unnamed_idx_1 + 2, 1, P_wayp_pred.size(0),
                               &oc_emlrtBCI);
        }
        if (unnamed_idx_1 + 2 > P_curr_pred.size(0)) {
          rtDynamicBoundsError(unnamed_idx_1 + 2, 1, P_curr_pred.size(0),
                               &pc_emlrtBCI);
        }
        if (unnamed_idx_1 + 1 > theta.size(0)) {
          rtDynamicBoundsError(unnamed_idx_1 + 1, 1, theta.size(0),
                               &qc_emlrtBCI);
        }
        theta_tmp = P_curr_pred[unnamed_idx_1 + 1];
        b_theta_tmp = P_wayp_pred[unnamed_idx_1 + 1];
        theta[unnamed_idx_1] = rt_atan2d_snf(b_theta_tmp - theta_tmp,
                                             P_wayp_pred[0] - P_curr_pred[0]);
        coder::internal::indexShapeCheck(P_curr_pred.size(0));
        if (unnamed_idx_1 + 1 > theta.size(0)) {
          rtDynamicBoundsError(unnamed_idx_1 + 1, 1, theta.size(0),
                               &rc_emlrtBCI);
        }
        if (1 > P_curr_pred.size(0)) {
          rtDynamicBoundsError(1, 1, P_curr_pred.size(0), &sc_emlrtBCI);
        }
        if (unnamed_idx_1 + 2 > P_curr_pred.size(0)) {
          rtDynamicBoundsError(unnamed_idx_1 + 2, 1, P_curr_pred.size(0),
                               &sc_emlrtBCI);
        }
        //  ---------------------------------------------------------------------
        //  Package:    TopiCo (https://github.com/AIS-Bonn/TopiCo)
        //  Version:    2021-03-18 12:09:55
        //  Maintainer: Marius Beul (mbeul@ais.uni-bonn.de)
        //  License:    BSD
        //  ---------------------------------------------------------------------
        //  Software License Agreement (BSD License)
        //  Copyright (c) 2021, Computer Science Institute VI, University of
        //  Bonn All rights reserved. Redistribution and use in source and
        //  binary forms, with or without modification, are permitted provided
        //  that the following conditions are met:
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
        d = std::cos(theta[unnamed_idx_1]);
        d1 = std::sin(theta[unnamed_idx_1]);
        output_idx_1 = d * theta_tmp - d1 * P_curr_pred[0];
        if (1 > P_curr_pred.size(0)) {
          rtDynamicBoundsError(1, 1, P_curr_pred.size(0), &tc_emlrtBCI);
        }
        if (unnamed_idx_1 + 2 > P_curr_pred.size(0)) {
          rtDynamicBoundsError(unnamed_idx_1 + 2, 1, P_curr_pred.size(0),
                               &tc_emlrtBCI);
        }
        P_curr_pred[0] = d * P_curr_pred[0] + d1 * theta_tmp;
        P_curr_pred[unnamed_idx_1 + 1] = output_idx_1;
        coder::internal::indexShapeCheck(V_curr_pred.size(0));
        if (unnamed_idx_1 + 1 > theta.size(0)) {
          rtDynamicBoundsError(unnamed_idx_1 + 1, 1, theta.size(0),
                               &uc_emlrtBCI);
        }
        if (1 > V_curr_pred.size(0)) {
          rtDynamicBoundsError(1, 1, V_curr_pred.size(0), &vc_emlrtBCI);
        }
        if (unnamed_idx_1 + 2 > V_curr_pred.size(0)) {
          rtDynamicBoundsError(unnamed_idx_1 + 2, 1, V_curr_pred.size(0),
                               &vc_emlrtBCI);
        }
        //  ---------------------------------------------------------------------
        //  Package:    TopiCo (https://github.com/AIS-Bonn/TopiCo)
        //  Version:    2021-03-18 12:09:55
        //  Maintainer: Marius Beul (mbeul@ais.uni-bonn.de)
        //  License:    BSD
        //  ---------------------------------------------------------------------
        //  Software License Agreement (BSD License)
        //  Copyright (c) 2021, Computer Science Institute VI, University of
        //  Bonn All rights reserved. Redistribution and use in source and
        //  binary forms, with or without modification, are permitted provided
        //  that the following conditions are met:
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
        theta_tmp = V_curr_pred[unnamed_idx_1 + 1];
        output_idx_1 = d * theta_tmp - d1 * V_curr_pred[0];
        if (1 > V_curr_pred.size(0)) {
          rtDynamicBoundsError(1, 1, V_curr_pred.size(0), &wc_emlrtBCI);
        }
        if (unnamed_idx_1 + 2 > V_curr_pred.size(0)) {
          rtDynamicBoundsError(unnamed_idx_1 + 2, 1, V_curr_pred.size(0),
                               &wc_emlrtBCI);
        }
        V_curr_pred[0] = d * V_curr_pred[0] + d1 * theta_tmp;
        V_curr_pred[unnamed_idx_1 + 1] = output_idx_1;
        coder::internal::indexShapeCheck(A_curr_pred.size(0));
        if (unnamed_idx_1 + 1 > theta.size(0)) {
          rtDynamicBoundsError(unnamed_idx_1 + 1, 1, theta.size(0),
                               &xc_emlrtBCI);
        }
        if (1 > A_curr_pred.size(0)) {
          rtDynamicBoundsError(1, 1, A_curr_pred.size(0), &yc_emlrtBCI);
        }
        if (unnamed_idx_1 + 2 > A_curr_pred.size(0)) {
          rtDynamicBoundsError(unnamed_idx_1 + 2, 1, A_curr_pred.size(0),
                               &yc_emlrtBCI);
        }
        //  ---------------------------------------------------------------------
        //  Package:    TopiCo (https://github.com/AIS-Bonn/TopiCo)
        //  Version:    2021-03-18 12:09:55
        //  Maintainer: Marius Beul (mbeul@ais.uni-bonn.de)
        //  License:    BSD
        //  ---------------------------------------------------------------------
        //  Software License Agreement (BSD License)
        //  Copyright (c) 2021, Computer Science Institute VI, University of
        //  Bonn All rights reserved. Redistribution and use in source and
        //  binary forms, with or without modification, are permitted provided
        //  that the following conditions are met:
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
        theta_tmp = A_curr_pred[unnamed_idx_1 + 1];
        output_idx_1 = d * theta_tmp - d1 * A_curr_pred[0];
        if (1 > A_curr_pred.size(0)) {
          rtDynamicBoundsError(1, 1, A_curr_pred.size(0), &ad_emlrtBCI);
        }
        if (unnamed_idx_1 + 2 > A_curr_pred.size(0)) {
          rtDynamicBoundsError(unnamed_idx_1 + 2, 1, A_curr_pred.size(0),
                               &ad_emlrtBCI);
        }
        A_curr_pred[0] = d * A_curr_pred[0] + d1 * theta_tmp;
        A_curr_pred[unnamed_idx_1 + 1] = output_idx_1;
        coder::internal::indexShapeCheck(P_wayp_pred.size(0));
        if (unnamed_idx_1 + 1 > theta.size(0)) {
          rtDynamicBoundsError(unnamed_idx_1 + 1, 1, theta.size(0),
                               &bd_emlrtBCI);
        }
        if (1 > P_wayp_pred.size(0)) {
          rtDynamicBoundsError(1, 1, P_wayp_pred.size(0), &cd_emlrtBCI);
        }
        if (unnamed_idx_1 + 2 > P_wayp_pred.size(0)) {
          rtDynamicBoundsError(unnamed_idx_1 + 2, 1, P_wayp_pred.size(0),
                               &cd_emlrtBCI);
        }
        //  ---------------------------------------------------------------------
        //  Package:    TopiCo (https://github.com/AIS-Bonn/TopiCo)
        //  Version:    2021-03-18 12:09:55
        //  Maintainer: Marius Beul (mbeul@ais.uni-bonn.de)
        //  License:    BSD
        //  ---------------------------------------------------------------------
        //  Software License Agreement (BSD License)
        //  Copyright (c) 2021, Computer Science Institute VI, University of
        //  Bonn All rights reserved. Redistribution and use in source and
        //  binary forms, with or without modification, are permitted provided
        //  that the following conditions are met:
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
        output_idx_1 = d * b_theta_tmp - d1 * P_wayp_pred[0];
        if (1 > P_wayp_pred.size(0)) {
          rtDynamicBoundsError(1, 1, P_wayp_pred.size(0), &dd_emlrtBCI);
        }
        if (unnamed_idx_1 + 2 > P_wayp_pred.size(0)) {
          rtDynamicBoundsError(unnamed_idx_1 + 2, 1, P_wayp_pred.size(0),
                               &dd_emlrtBCI);
        }
        P_wayp_pred[0] = d * P_wayp_pred[0] + d1 * b_theta_tmp;
        P_wayp_pred[unnamed_idx_1 + 1] = output_idx_1;
        coder::internal::indexShapeCheck(V_wayp_pred.size(0));
        if (unnamed_idx_1 + 1 > theta.size(0)) {
          rtDynamicBoundsError(unnamed_idx_1 + 1, 1, theta.size(0),
                               &ed_emlrtBCI);
        }
        if (1 > V_wayp_pred.size(0)) {
          rtDynamicBoundsError(1, 1, V_wayp_pred.size(0), &fd_emlrtBCI);
        }
        if (unnamed_idx_1 + 2 > V_wayp_pred.size(0)) {
          rtDynamicBoundsError(unnamed_idx_1 + 2, 1, V_wayp_pred.size(0),
                               &fd_emlrtBCI);
        }
        //  ---------------------------------------------------------------------
        //  Package:    TopiCo (https://github.com/AIS-Bonn/TopiCo)
        //  Version:    2021-03-18 12:09:55
        //  Maintainer: Marius Beul (mbeul@ais.uni-bonn.de)
        //  License:    BSD
        //  ---------------------------------------------------------------------
        //  Software License Agreement (BSD License)
        //  Copyright (c) 2021, Computer Science Institute VI, University of
        //  Bonn All rights reserved. Redistribution and use in source and
        //  binary forms, with or without modification, are permitted provided
        //  that the following conditions are met:
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
        theta_tmp = V_wayp_pred[unnamed_idx_1 + 1];
        output_idx_1 = d * theta_tmp - d1 * V_wayp_pred[0];
        if (1 > V_wayp_pred.size(0)) {
          rtDynamicBoundsError(1, 1, V_wayp_pred.size(0), &gd_emlrtBCI);
        }
        if (unnamed_idx_1 + 2 > V_wayp_pred.size(0)) {
          rtDynamicBoundsError(unnamed_idx_1 + 2, 1, V_wayp_pred.size(0),
                               &gd_emlrtBCI);
        }
        V_wayp_pred[0] = d * V_wayp_pred[0] + d1 * theta_tmp;
        V_wayp_pred[unnamed_idx_1 + 1] = output_idx_1;
        coder::internal::indexShapeCheck(A_wayp_pred.size(0));
        if (unnamed_idx_1 + 1 > theta.size(0)) {
          rtDynamicBoundsError(unnamed_idx_1 + 1, 1, theta.size(0),
                               &hd_emlrtBCI);
        }
        if (1 > A_wayp_pred.size(0)) {
          rtDynamicBoundsError(1, 1, A_wayp_pred.size(0), &id_emlrtBCI);
        }
        if (unnamed_idx_1 + 2 > A_wayp_pred.size(0)) {
          rtDynamicBoundsError(unnamed_idx_1 + 2, 1, A_wayp_pred.size(0),
                               &id_emlrtBCI);
        }
        //  ---------------------------------------------------------------------
        //  Package:    TopiCo (https://github.com/AIS-Bonn/TopiCo)
        //  Version:    2021-03-18 12:09:55
        //  Maintainer: Marius Beul (mbeul@ais.uni-bonn.de)
        //  License:    BSD
        //  ---------------------------------------------------------------------
        //  Software License Agreement (BSD License)
        //  Copyright (c) 2021, Computer Science Institute VI, University of
        //  Bonn All rights reserved. Redistribution and use in source and
        //  binary forms, with or without modification, are permitted provided
        //  that the following conditions are met:
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
        theta_tmp = A_wayp_pred[unnamed_idx_1 + 1];
        output_idx_1 = d * theta_tmp - d1 * A_wayp_pred[0];
        if (1 > A_wayp_pred.size(0)) {
          rtDynamicBoundsError(1, 1, A_wayp_pred.size(0), &jd_emlrtBCI);
        }
        if (unnamed_idx_1 + 2 > A_wayp_pred.size(0)) {
          rtDynamicBoundsError(unnamed_idx_1 + 2, 1, A_wayp_pred.size(0),
                               &jd_emlrtBCI);
        }
        A_wayp_pred[0] = d * A_wayp_pred[0] + d1 * theta_tmp;
        A_wayp_pred[unnamed_idx_1 + 1] = output_idx_1;
      }
    }
    //  Generate Trajectory
    b_solution_out = V_max_in.size(0);
    b_V_max_in.set_size(V_max_in.size(0));
    for (i1 = 0; i1 < b_solution_out; i1++) {
      b_V_max_in[i1] = V_max_in[i1 + V_max_in.size(0) * index_waypoint] -
                       Waypoint_evolved[i1 + Waypoint_evolved.size(0) * 3];
    }
    b_solution_out = V_min_in.size(0);
    b_V_min_in.set_size(V_min_in.size(0));
    for (i1 = 0; i1 < b_solution_out; i1++) {
      b_V_min_in[i1] = V_min_in[i1 + V_min_in.size(0) * index_waypoint] -
                       Waypoint_evolved[i1 + Waypoint_evolved.size(0) * 3];
    }
    b_solution_out = b_A_max_in.size(0);
    c_A_max_in.set_size(b_A_max_in.size(0));
    for (i1 = 0; i1 < b_solution_out; i1++) {
      c_A_max_in[i1] = b_A_max_in[i1 + b_A_max_in.size(0) * index_waypoint] -
                       Waypoint_evolved[i1 + Waypoint_evolved.size(0) * 4];
    }
    b_solution_out = b_A_min_in.size(0);
    c_A_min_in.set_size(b_A_min_in.size(0));
    for (i1 = 0; i1 < b_solution_out; i1++) {
      c_A_min_in[i1] = b_A_min_in[i1 + b_A_min_in.size(0) * index_waypoint] -
                       Waypoint_evolved[i1 + Waypoint_evolved.size(0) * 4];
    }
    b_solution_out = J_max_in.size(0);
    b_J_max_in.set_size(J_max_in.size(0));
    for (i1 = 0; i1 < b_solution_out; i1++) {
      b_J_max_in[i1] = J_max_in[i1 + J_max_in.size(0) * index_waypoint];
    }
    b_solution_out = J_min_in.size(0);
    b_J_min_in.set_size(J_min_in.size(0));
    for (i1 = 0; i1 < b_solution_out; i1++) {
      b_J_min_in[i1] = J_min_in[i1 + J_min_in.size(0) * index_waypoint];
    }
    b_solution_out = b_sync_V_in.size(0);
    b_b_sync_V_in.set_size(b_sync_V_in.size(0));
    for (i1 = 0; i1 < b_solution_out; i1++) {
      b_b_sync_V_in[i1] =
          b_sync_V_in[i1 + b_sync_V_in.size(0) * index_waypoint];
    }
    b_solution_out = b_sync_A_in.size(0);
    b_b_sync_A_in.set_size(b_sync_A_in.size(0));
    for (i1 = 0; i1 < b_solution_out; i1++) {
      b_b_sync_A_in[i1] =
          b_sync_A_in[i1 + b_sync_A_in.size(0) * index_waypoint];
    }
    b_solution_out = b_sync_J_in.size(0);
    b_b_sync_J_in.set_size(b_sync_J_in.size(0));
    for (i1 = 0; i1 < b_solution_out; i1++) {
      b_b_sync_J_in[i1] =
          b_sync_J_in[i1 + b_sync_J_in.size(0) * index_waypoint];
    }
    b_solution_out = b_sync_W_in.size(0);
    b_b_sync_W_in.set_size(b_sync_W_in.size(0));
    for (i1 = 0; i1 < b_solution_out; i1++) {
      b_b_sync_W_in[i1] =
          b_sync_W_in[i1 + b_sync_W_in.size(0) * index_waypoint];
    }
    b_solution_out = b_hard_V_lim_in.size(0);
    b_b_hard_V_lim_in.set_size(b_hard_V_lim_in.size(0));
    for (i1 = 0; i1 < b_solution_out; i1++) {
      b_b_hard_V_lim_in[i1] =
          b_hard_V_lim_in[i1 + b_hard_V_lim_in.size(0) * index_waypoint];
    }
    b_solution_out = maxval.size(0);
    for (i1 = 0; i1 < b_solution_out; i1++) {
      maxval[i1] =
          (b_maxval - maxval[i1]) *
          static_cast<double>(
              b_catch_up_in[i1 + b_catch_up_in.size(0) * index_waypoint]);
    }
    b_solution_out = direction_in.size(0);
    b_direction_in.set_size(direction_in.size(0));
    for (i1 = 0; i1 < b_solution_out; i1++) {
      b_direction_in[i1] =
          direction_in[i1 + direction_in.size(0) * index_waypoint];
    }
    synchronize_trajectory(
        P_curr_pred, V_curr_pred, A_curr_pred, P_wayp_pred, V_wayp_pred,
        A_wayp_pred, b_V_max_in, b_V_min_in, c_A_max_in, c_A_min_in, b_J_max_in,
        b_J_min_in, b_b_sync_V_in, b_b_sync_A_in, b_b_sync_J_in, b_b_sync_W_in,
        b_b_hard_V_lim_in, maxval, b_direction_in, t_traj, J_traj, r);
    if (index_waypoint + 1 > solution_out.size(1)) {
      rtDynamicBoundsError(index_waypoint + 1, 1, solution_out.size(1),
                           &kd_emlrtBCI);
    }
    rtSubAssignSizeCheck(solution_out.size(), 1, r.size(), 1, &n_emlrtECI);
    b_solution_out = r.size(0);
    for (i1 = 0; i1 < b_solution_out; i1++) {
      solution_out[i1 + solution_out.size(0) * index_waypoint] = r[i1];
    }
    //  Rotation
    i1 = static_cast<int>(
        ((-1.0 - (static_cast<double>(num_axes + 1) - 1.0)) + 1.0) / -1.0);
    for (unnamed_idx_1 = 0; unnamed_idx_1 < i1; unnamed_idx_1++) {
      unnamed_idx_0 = num_axes - unnamed_idx_1;
      if ((unnamed_idx_0 < 1) || (unnamed_idx_0 > b_rotate_in.size(0))) {
        rtDynamicBoundsError(unnamed_idx_0, 1, b_rotate_in.size(0),
                             &ld_emlrtBCI);
      }
      if (b_rotate_in[(unnamed_idx_0 + b_rotate_in.size(0) * index_waypoint) -
                      1]) {
        if (0 > t_traj.size(0) - 1) {
          rtDynamicBoundsError(0, 0, t_traj.size(0) - 1, &md_emlrtBCI);
        }
        if (0 > J_traj.size(0) - 1) {
          rtDynamicBoundsError(0, 0, J_traj.size(0) - 1, &nd_emlrtBCI);
        }
        if (unnamed_idx_0 > theta.size(0)) {
          rtDynamicBoundsError(unnamed_idx_0, 1, theta.size(0), &od_emlrtBCI);
        }
        if (unnamed_idx_0 > t_traj.size(0) - 1) {
          rtDynamicBoundsError(unnamed_idx_0, 0, t_traj.size(0) - 1,
                               &pd_emlrtBCI);
        }
        if (unnamed_idx_0 > J_traj.size(0) - 1) {
          rtDynamicBoundsError(unnamed_idx_0, 0, J_traj.size(0) - 1,
                               &qd_emlrtBCI);
        }
        rotate_jerk(theta[unnamed_idx_0 - 1], t_traj[0].f1, J_traj[0].f1,
                    t_traj[unnamed_idx_0].f1, J_traj[unnamed_idx_0].f1,
                    b_t_traj, r1, r2, r3);
        if (0 > t_traj.size(0) - 1) {
          rtDynamicBoundsError(0, 0, t_traj.size(0) - 1, &rd_emlrtBCI);
        }
        t_traj[0].f1.set_size(1, b_t_traj.size(1));
        b_solution_out = b_t_traj.size(1);
        for (i2 = 0; i2 < b_solution_out; i2++) {
          t_traj[0].f1[i2] = b_t_traj[i2];
        }
        if (0 > J_traj.size(0) - 1) {
          rtDynamicBoundsError(0, 0, J_traj.size(0) - 1, &sd_emlrtBCI);
        }
        J_traj[0].f1.set_size(1, r1.size(1));
        b_solution_out = r1.size(1);
        for (i2 = 0; i2 < b_solution_out; i2++) {
          J_traj[0].f1[i2] = r1[i2];
        }
        b_solution_out = r2.size(1) - 1;
        if (unnamed_idx_0 > t_traj.size(0) - 1) {
          rtDynamicBoundsError(unnamed_idx_0, 0, t_traj.size(0) - 1,
                               &td_emlrtBCI);
        }
        t_traj[unnamed_idx_0].f1.set_size(1, t_traj[unnamed_idx_0].f1.size(1));
        if (unnamed_idx_0 > t_traj.size(0) - 1) {
          rtDynamicBoundsError(unnamed_idx_0, 0, t_traj.size(0) - 1,
                               &td_emlrtBCI);
        }
        t_traj[unnamed_idx_0].f1.set_size(t_traj[unnamed_idx_0].f1.size(0),
                                          r2.size(1));
        i2 = t_traj.size(0);
        if (unnamed_idx_0 > t_traj.size(0) - 1) {
          rtDynamicBoundsError(unnamed_idx_0, 0, t_traj.size(0) - 1,
                               &td_emlrtBCI);
        }
        if (unnamed_idx_0 > t_traj.size(0) - 1) {
          rtDynamicBoundsError(unnamed_idx_0, 0, t_traj.size(0) - 1,
                               &td_emlrtBCI);
        }
        for (i3 = 0; i3 <= b_solution_out; i3++) {
          if (unnamed_idx_0 > i2 - 1) {
            rtDynamicBoundsError(unnamed_idx_0, 0, i2 - 1, &ud_emlrtBCI);
          }
          t_traj[unnamed_idx_0].f1[i3] = r2[i3];
        }
        b_solution_out = r3.size(1) - 1;
        if (unnamed_idx_0 > J_traj.size(0) - 1) {
          rtDynamicBoundsError(unnamed_idx_0, 0, J_traj.size(0) - 1,
                               &vd_emlrtBCI);
        }
        J_traj[unnamed_idx_0].f1.set_size(1, J_traj[unnamed_idx_0].f1.size(1));
        if (unnamed_idx_0 > J_traj.size(0) - 1) {
          rtDynamicBoundsError(unnamed_idx_0, 0, J_traj.size(0) - 1,
                               &vd_emlrtBCI);
        }
        J_traj[unnamed_idx_0].f1.set_size(J_traj[unnamed_idx_0].f1.size(0),
                                          r3.size(1));
        i2 = J_traj.size(0);
        if (unnamed_idx_0 > J_traj.size(0) - 1) {
          rtDynamicBoundsError(unnamed_idx_0, 0, J_traj.size(0) - 1,
                               &vd_emlrtBCI);
        }
        if (unnamed_idx_0 > J_traj.size(0) - 1) {
          rtDynamicBoundsError(unnamed_idx_0, 0, J_traj.size(0) - 1,
                               &vd_emlrtBCI);
        }
        for (i3 = 0; i3 <= b_solution_out; i3++) {
          if (unnamed_idx_0 > i2 - 1) {
            rtDynamicBoundsError(unnamed_idx_0, 0, i2 - 1, &wd_emlrtBCI);
          }
          J_traj[unnamed_idx_0].f1[i3] = r3[i3];
        }
      }
    }
    for (unnamed_idx_0 = 0; unnamed_idx_0 <= num_axes; unnamed_idx_0++) {
      bool guard1 = false;
      if (unnamed_idx_0 > t_traj.size(0) - 1) {
        rtDynamicBoundsError(unnamed_idx_0, 0, t_traj.size(0) - 1,
                             &xd_emlrtBCI);
      }
      b_solution_out =
          t_traj[unnamed_idx_0].f1.size(0) * t_traj[unnamed_idx_0].f1.size(1);
      if (unnamed_idx_0 > t_traj.size(0) - 1) {
        rtDynamicBoundsError(unnamed_idx_0, 0, t_traj.size(0) - 1,
                             &xd_emlrtBCI);
      }
      if (unnamed_idx_0 > t_setp.size(0) - 1) {
        rtDynamicBoundsError(unnamed_idx_0, 0, t_setp.size(0) - 1,
                             &yd_emlrtBCI);
      }
      if (index_waypoint > t_setp.size(1) - 1) {
        rtDynamicBoundsError(index_waypoint, 0, t_setp.size(1) - 1,
                             &ae_emlrtBCI);
      }
      t_setp[unnamed_idx_0 + t_setp.size(0) * index_waypoint].f1.set_size(
          t_traj[unnamed_idx_0].f1.size(0),
          t_setp[unnamed_idx_0 + t_setp.size(0) * index_waypoint].f1.size(1));
      if (unnamed_idx_0 > t_traj.size(0) - 1) {
        rtDynamicBoundsError(unnamed_idx_0, 0, t_traj.size(0) - 1,
                             &xd_emlrtBCI);
      }
      if (unnamed_idx_0 > t_setp.size(0) - 1) {
        rtDynamicBoundsError(unnamed_idx_0, 0, t_setp.size(0) - 1,
                             &yd_emlrtBCI);
      }
      if (index_waypoint > t_setp.size(1) - 1) {
        rtDynamicBoundsError(index_waypoint, 0, t_setp.size(1) - 1,
                             &ae_emlrtBCI);
      }
      t_setp[unnamed_idx_0 + t_setp.size(0) * index_waypoint].f1.set_size(
          t_setp[unnamed_idx_0 + t_setp.size(0) * index_waypoint].f1.size(0),
          t_traj[unnamed_idx_0].f1.size(1));
      i1 = t_setp.size(0);
      i2 = t_setp.size(1);
      if (unnamed_idx_0 > t_setp.size(0) - 1) {
        rtDynamicBoundsError(unnamed_idx_0, 0, t_setp.size(0) - 1,
                             &yd_emlrtBCI);
      }
      if (index_waypoint > t_setp.size(1) - 1) {
        rtDynamicBoundsError(index_waypoint, 0, t_setp.size(1) - 1,
                             &ae_emlrtBCI);
      }
      if (unnamed_idx_0 > t_setp.size(0) - 1) {
        rtDynamicBoundsError(unnamed_idx_0, 0, t_setp.size(0) - 1,
                             &yd_emlrtBCI);
      }
      if (index_waypoint > t_setp.size(1) - 1) {
        rtDynamicBoundsError(index_waypoint, 0, t_setp.size(1) - 1,
                             &ae_emlrtBCI);
      }
      for (i3 = 0; i3 < b_solution_out; i3++) {
        if (unnamed_idx_0 > t_traj.size(0) - 1) {
          rtDynamicBoundsError(unnamed_idx_0, 0, t_traj.size(0) - 1,
                               &be_emlrtBCI);
        }
        if (unnamed_idx_0 > i1 - 1) {
          rtDynamicBoundsError(unnamed_idx_0, 0, i1 - 1, &ce_emlrtBCI);
        }
        if (index_waypoint > i2 - 1) {
          rtDynamicBoundsError(index_waypoint, 0, i2 - 1, &ce_emlrtBCI);
        }
        t_setp[unnamed_idx_0 + t_setp.size(0) * index_waypoint].f1[i3] =
            t_traj[unnamed_idx_0].f1[i3];
      }
      if (unnamed_idx_0 > J_traj.size(0) - 1) {
        rtDynamicBoundsError(unnamed_idx_0, 0, J_traj.size(0) - 1,
                             &de_emlrtBCI);
      }
      b_solution_out =
          J_traj[unnamed_idx_0].f1.size(0) * J_traj[unnamed_idx_0].f1.size(1);
      if (unnamed_idx_0 > J_traj.size(0) - 1) {
        rtDynamicBoundsError(unnamed_idx_0, 0, J_traj.size(0) - 1,
                             &de_emlrtBCI);
      }
      if (unnamed_idx_0 > J_setp.size(0) - 1) {
        rtDynamicBoundsError(unnamed_idx_0, 0, J_setp.size(0) - 1,
                             &ee_emlrtBCI);
      }
      if (index_waypoint > J_setp.size(1) - 1) {
        rtDynamicBoundsError(index_waypoint, 0, J_setp.size(1) - 1,
                             &fe_emlrtBCI);
      }
      J_setp[unnamed_idx_0 + J_setp.size(0) * index_waypoint].f1.set_size(
          J_traj[unnamed_idx_0].f1.size(0),
          J_setp[unnamed_idx_0 + J_setp.size(0) * index_waypoint].f1.size(1));
      if (unnamed_idx_0 > J_traj.size(0) - 1) {
        rtDynamicBoundsError(unnamed_idx_0, 0, J_traj.size(0) - 1,
                             &de_emlrtBCI);
      }
      if (unnamed_idx_0 > J_setp.size(0) - 1) {
        rtDynamicBoundsError(unnamed_idx_0, 0, J_setp.size(0) - 1,
                             &ee_emlrtBCI);
      }
      if (index_waypoint > J_setp.size(1) - 1) {
        rtDynamicBoundsError(index_waypoint, 0, J_setp.size(1) - 1,
                             &fe_emlrtBCI);
      }
      J_setp[unnamed_idx_0 + J_setp.size(0) * index_waypoint].f1.set_size(
          J_setp[unnamed_idx_0 + J_setp.size(0) * index_waypoint].f1.size(0),
          J_traj[unnamed_idx_0].f1.size(1));
      i1 = J_setp.size(0);
      i2 = J_setp.size(1);
      if (unnamed_idx_0 > J_setp.size(0) - 1) {
        rtDynamicBoundsError(unnamed_idx_0, 0, J_setp.size(0) - 1,
                             &ee_emlrtBCI);
      }
      if (index_waypoint > J_setp.size(1) - 1) {
        rtDynamicBoundsError(index_waypoint, 0, J_setp.size(1) - 1,
                             &fe_emlrtBCI);
      }
      if (unnamed_idx_0 > J_setp.size(0) - 1) {
        rtDynamicBoundsError(unnamed_idx_0, 0, J_setp.size(0) - 1,
                             &ee_emlrtBCI);
      }
      if (index_waypoint > J_setp.size(1) - 1) {
        rtDynamicBoundsError(index_waypoint, 0, J_setp.size(1) - 1,
                             &fe_emlrtBCI);
      }
      for (i3 = 0; i3 < b_solution_out; i3++) {
        if (unnamed_idx_0 > J_traj.size(0) - 1) {
          rtDynamicBoundsError(unnamed_idx_0, 0, J_traj.size(0) - 1,
                               &ge_emlrtBCI);
        }
        if (unnamed_idx_0 > i1 - 1) {
          rtDynamicBoundsError(unnamed_idx_0, 0, i1 - 1, &he_emlrtBCI);
        }
        if (index_waypoint > i2 - 1) {
          rtDynamicBoundsError(index_waypoint, 0, i2 - 1, &he_emlrtBCI);
        }
        J_setp[unnamed_idx_0 + J_setp.size(0) * index_waypoint].f1[i3] =
            J_traj[unnamed_idx_0].f1[i3];
      }
      if (unnamed_idx_0 > t_traj.size(0) - 1) {
        rtDynamicBoundsError(unnamed_idx_0, 0, t_traj.size(0) - 1,
                             &ie_emlrtBCI);
      }
      i1 = t_traj[unnamed_idx_0].f1.size(0);
      if (1 > i1) {
        rtDynamicBoundsError(1, 1, i1, &je_emlrtBCI);
      }
      b_solution_out = t_traj[unnamed_idx_0].f1.size(1);
      b_t_traj.set_size(1, t_traj[unnamed_idx_0].f1.size(1));
      for (i1 = 0; i1 < b_solution_out; i1++) {
        b_t_traj[i1] =
            t_traj[unnamed_idx_0].f1[t_traj[unnamed_idx_0].f1.size(0) * i1];
      }
      if (unnamed_idx_0 + 1 > T_waypoints.size(0)) {
        rtDynamicBoundsError(unnamed_idx_0 + 1, 1, T_waypoints.size(0),
                             &ke_emlrtBCI);
      }
      if (index_waypoint + 1 > T_waypoints.size(1)) {
        rtDynamicBoundsError(index_waypoint + 1, 1, T_waypoints.size(1),
                             &ke_emlrtBCI);
      }
      T_waypoints[unnamed_idx_0 + T_waypoints.size(0) * index_waypoint] =
          coder::b_combineVectorElements(b_t_traj);
      // For numerical reasons and computational optimization
      if (unnamed_idx_0 + 1 > Waypoint_evolved.size(0)) {
        rtDynamicBoundsError(unnamed_idx_0 + 1, 1, Waypoint_evolved.size(0),
                             &le_emlrtBCI);
      }
      guard1 = false;
      if (Waypoint_evolved[unnamed_idx_0 + Waypoint_evolved.size(0) * 3] ==
          0.0) {
        if (unnamed_idx_0 + 1 > Waypoint_evolved.size(0)) {
          rtDynamicBoundsError(unnamed_idx_0 + 1, 1, Waypoint_evolved.size(0),
                               &me_emlrtBCI);
        }
        if (Waypoint_evolved[unnamed_idx_0 + Waypoint_evolved.size(0) * 4] ==
            0.0) {
          if (unnamed_idx_0 + 1 > Waypoint_evolved.size(0)) {
            rtDynamicBoundsError(unnamed_idx_0 + 1, 1, Waypoint_evolved.size(0),
                                 &ne_emlrtBCI);
          }
          if (!rtIsNaN(Waypoint_evolved[unnamed_idx_0])) {
            if (unnamed_idx_0 + 1 > Waypoint_evolved.size(0)) {
              rtDynamicBoundsError(unnamed_idx_0 + 1, 1,
                                   Waypoint_evolved.size(0), &oe_emlrtBCI);
            }
            if (!rtIsNaN(Waypoint_evolved[unnamed_idx_0 +
                                          Waypoint_evolved.size(0)])) {
              if (unnamed_idx_0 + 1 > Waypoint_evolved.size(0)) {
                rtDynamicBoundsError(unnamed_idx_0 + 1, 1,
                                     Waypoint_evolved.size(0), &pe_emlrtBCI);
              }
              if (!rtIsNaN(Waypoint_evolved[unnamed_idx_0 +
                                            Waypoint_evolved.size(0) * 2])) {
                if (unnamed_idx_0 + 1 > solution_out.size(0)) {
                  rtDynamicBoundsError(unnamed_idx_0 + 1, 1,
                                       solution_out.size(0), &qe_emlrtBCI);
                }
                if (index_waypoint + 1 > solution_out.size(1)) {
                  rtDynamicBoundsError(index_waypoint + 1, 1,
                                       solution_out.size(1), &re_emlrtBCI);
                }
                if (unnamed_idx_0 + 1 > Waypoints.size(0)) {
                  rtDynamicBoundsError(unnamed_idx_0 + 1, 1, Waypoints.size(0),
                                       &se_emlrtBCI);
                }
                if (index_waypoint + 1 > Waypoints.size(2)) {
                  rtDynamicBoundsError(index_waypoint + 1, 1, Waypoints.size(2),
                                       &se_emlrtBCI);
                }
                if (unnamed_idx_0 + 1 > P_curr.size(0)) {
                  rtDynamicBoundsError(unnamed_idx_0 + 1, 1, P_curr.size(0),
                                       &te_emlrtBCI);
                }
                P_curr[unnamed_idx_0] =
                    Waypoints[unnamed_idx_0 +
                              Waypoints.size(0) * 5 * index_waypoint];
                if (unnamed_idx_0 + 1 > Waypoints.size(0)) {
                  rtDynamicBoundsError(unnamed_idx_0 + 1, 1, Waypoints.size(0),
                                       &ue_emlrtBCI);
                }
                if (index_waypoint + 1 > Waypoints.size(2)) {
                  rtDynamicBoundsError(index_waypoint + 1, 1, Waypoints.size(2),
                                       &ue_emlrtBCI);
                }
                if (unnamed_idx_0 + 1 > V_curr.size(0)) {
                  rtDynamicBoundsError(unnamed_idx_0 + 1, 1, V_curr.size(0),
                                       &ve_emlrtBCI);
                }
                V_curr[unnamed_idx_0] =
                    Waypoints[(unnamed_idx_0 + Waypoints.size(0)) +
                              Waypoints.size(0) * 5 * index_waypoint];
                if (unnamed_idx_0 + 1 > Waypoints.size(0)) {
                  rtDynamicBoundsError(unnamed_idx_0 + 1, 1, Waypoints.size(0),
                                       &we_emlrtBCI);
                }
                if (index_waypoint + 1 > Waypoints.size(2)) {
                  rtDynamicBoundsError(index_waypoint + 1, 1, Waypoints.size(2),
                                       &we_emlrtBCI);
                }
                if (unnamed_idx_0 + 1 > A_curr.size(0)) {
                  rtDynamicBoundsError(unnamed_idx_0 + 1, 1, A_curr.size(0),
                                       &xe_emlrtBCI);
                }
                A_curr[unnamed_idx_0] =
                    Waypoints[(unnamed_idx_0 + Waypoints.size(0) * 2) +
                              Waypoints.size(0) * 5 * index_waypoint];
              } else {
                guard1 = true;
              }
            } else {
              guard1 = true;
            }
          } else {
            guard1 = true;
          }
        } else {
          guard1 = true;
        }
      } else {
        guard1 = true;
      }
      if (guard1) {
        if (unnamed_idx_0 > t_traj.size(0) - 1) {
          rtDynamicBoundsError(unnamed_idx_0, 0, t_traj.size(0) - 1,
                               &ye_emlrtBCI);
        }
        if (unnamed_idx_0 > J_traj.size(0) - 1) {
          rtDynamicBoundsError(unnamed_idx_0, 0, J_traj.size(0) - 1,
                               &af_emlrtBCI);
        }
        construct_setp_struct(t_traj[unnamed_idx_0].f1,
                              J_traj[unnamed_idx_0].f1, t19_time, b_t_traj);
        if (unnamed_idx_0 + 1 > P_curr.size(0)) {
          rtDynamicBoundsError(unnamed_idx_0 + 1, 1, P_curr.size(0),
                               &bf_emlrtBCI);
        }
        if (unnamed_idx_0 + 1 > V_curr.size(0)) {
          rtDynamicBoundsError(unnamed_idx_0 + 1, 1, V_curr.size(0),
                               &cf_emlrtBCI);
        }
        if (unnamed_idx_0 + 1 > A_curr.size(0)) {
          rtDynamicBoundsError(unnamed_idx_0 + 1, 1, A_curr.size(0),
                               &df_emlrtBCI);
        }
        if (unnamed_idx_0 + 1 > T_waypoints.size(0)) {
          rtDynamicBoundsError(unnamed_idx_0 + 1, 1, T_waypoints.size(0),
                               &ef_emlrtBCI);
        }
        if (index_waypoint + 1 > T_waypoints.size(1)) {
          rtDynamicBoundsError(index_waypoint + 1, 1, T_waypoints.size(1),
                               &ef_emlrtBCI);
        }
        if (unnamed_idx_0 + 1 > P_curr.size(0)) {
          rtDynamicBoundsError(unnamed_idx_0 + 1, 1, P_curr.size(0),
                               &ff_emlrtBCI);
        }
        if (unnamed_idx_0 + 1 > V_curr.size(0)) {
          rtDynamicBoundsError(unnamed_idx_0 + 1, 1, V_curr.size(0),
                               &gf_emlrtBCI);
        }
        if (unnamed_idx_0 + 1 > A_curr.size(0)) {
          rtDynamicBoundsError(unnamed_idx_0 + 1, 1, A_curr.size(0),
                               &hf_emlrtBCI);
        }
        evaluate_to_time(
            P_curr[unnamed_idx_0], V_curr[unnamed_idx_0], A_curr[unnamed_idx_0],
            t19_time, b_t_traj,
            T_waypoints[unnamed_idx_0 + T_waypoints.size(0) * index_waypoint],
            &P_curr[unnamed_idx_0], &V_curr[unnamed_idx_0],
            &A_curr[unnamed_idx_0], &theta_tmp);
      }
    }
  }
  construct_setp_struct(t_setp, J_setp, J_setp_struct);
  b_solution_out = b_State_start.size(0);
  P_curr.set_size(b_State_start.size(0));
  for (i = 0; i < b_solution_out; i++) {
    P_curr[i] = b_State_start[i];
  }
  b_solution_out = b_State_start.size(0);
  V_curr.set_size(b_State_start.size(0));
  for (i = 0; i < b_solution_out; i++) {
    V_curr[i] = b_State_start[i + b_State_start.size(0)];
  }
  b_solution_out = b_State_start.size(0);
  A_curr.set_size(b_State_start.size(0));
  for (i = 0; i < b_solution_out; i++) {
    A_curr[i] = b_State_start[i + b_State_start.size(0) * 2];
  }
  rollout_t(P_curr, V_curr, A_curr, J_setp_struct, ts_rollout, P, V, A, J, t);
}

// End of code generation (topico.cpp)
