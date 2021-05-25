//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// check_inputs.cpp
//
// Code generation for function 'check_inputs'
//

// Include files
#include "check_inputs.h"
#include "any1.h"
#include "check_feasibility.h"
#include "combineVectorElements.h"
#include "rt_nonfinite.h"
#include "topico_wrapper_rtwutil.h"
#include "topico_wrapper_types.h"
#include "validateattributes.h"
#include "coder_array.h"
#include "rt_nonfinite.h"
#include <sstream>
#include <stdexcept>
#include <stdio.h>
#include <string>

// Function Declarations
static void b_rtErrorWithMessageID(const char *aFcnName, int aLineNum);

static void rtErrorWithMessageID(const char *aFcnName, int aLineNum);

// Function Definitions
static void b_rtErrorWithMessageID(const char *aFcnName, int aLineNum)
{
  std::stringstream outStream;
  outStream
      << "To RESHAPE the number of elements must not change, and if the input "
         "is empty, the maximum dimension length cannot be increased u"
         "nless the output size is fixed.";
  outStream << "\n";
  ((((outStream << "Error in ") << aFcnName) << " (line ") << aLineNum) << ")";
  throw std::runtime_error(outStream.str());
}

static void rtErrorWithMessageID(const char *aFcnName, int aLineNum)
{
  std::stringstream outStream;
  outStream << "To RESHAPE the number of elements must not change.";
  outStream << "\n";
  ((((outStream << "Error in ") << aFcnName) << " (line ") << aLineNum) << ")";
  throw std::runtime_error(outStream.str());
}

void check_inputs(const coder::array<double, 2U> &State_start,
                  const coder::array<double, 3U> &Waypoints,
                  const coder::array<double, 2U> &V_max,
                  const coder::array<double, 2U> &V_min,
                  const coder::array<double, 2U> &A_max,
                  const coder::array<double, 2U> &A_min,
                  const coder::array<double, 2U> &J_max,
                  const coder::array<double, 2U> &J_min,
                  const coder::array<double, 1U> &A_global,
                  const coder::array<bool, 2U> &b_sync_V,
                  const coder::array<bool, 2U> &b_sync_A,
                  const coder::array<bool, 2U> &b_sync_J,
                  const coder::array<bool, 2U> &b_sync_W,
                  const coder::array<bool, 2U> &b_rotate,
                  const coder::array<bool, 2U> &b_hard_V_lim,
                  const coder::array<bool, 2U> &b_catch_up,
                  const coder::array<signed char, 2U> &direction,
                  double ts_rollout)
{
  static rtBoundsCheckInfo ac_emlrtBCI = {
      -1,                                                  // iFirst
      -1,                                                  // iLast
      148,                                                 // lineNo
      133,                                                 // colNo
      "Waypoints",                                         // aName
      "check_inputs",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/check_inputs.m", // pName
      0                                                    // checkKind
  };
  static rtBoundsCheckInfo ad_emlrtBCI = {
      -1,                                                  // iFirst
      -1,                                                  // iLast
      176,                                                 // lineNo
      226,                                                 // colNo
      "V_max",                                             // aName
      "check_inputs",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/check_inputs.m", // pName
      0                                                    // checkKind
  };
  static rtBoundsCheckInfo bc_emlrtBCI = {
      -1,                                                  // iFirst
      -1,                                                  // iLast
      148,                                                 // lineNo
      146,                                                 // colNo
      "Waypoints",                                         // aName
      "check_inputs",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/check_inputs.m", // pName
      0                                                    // checkKind
  };
  static rtBoundsCheckInfo bd_emlrtBCI = {
      -1,                                                  // iFirst
      -1,                                                  // iLast
      176,                                                 // lineNo
      248,                                                 // colNo
      "V_min",                                             // aName
      "check_inputs",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/check_inputs.m", // pName
      0                                                    // checkKind
  };
  static rtBoundsCheckInfo cc_emlrtBCI = {
      -1,                                                  // iFirst
      -1,                                                  // iLast
      148,                                                 // lineNo
      172,                                                 // colNo
      "Waypoints",                                         // aName
      "check_inputs",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/check_inputs.m", // pName
      0                                                    // checkKind
  };
  static rtBoundsCheckInfo cd_emlrtBCI = {
      -1,                                                  // iFirst
      -1,                                                  // iLast
      176,                                                 // lineNo
      259,                                                 // colNo
      "V_min",                                             // aName
      "check_inputs",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/check_inputs.m", // pName
      0                                                    // checkKind
  };
  static rtBoundsCheckInfo dc_emlrtBCI = {
      -1,                                                  // iFirst
      -1,                                                  // iLast
      148,                                                 // lineNo
      185,                                                 // colNo
      "Waypoints",                                         // aName
      "check_inputs",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/check_inputs.m", // pName
      0                                                    // checkKind
  };
  static rtBoundsCheckInfo dd_emlrtBCI = {
      -1,                                                  // iFirst
      -1,                                                  // iLast
      176,                                                 // lineNo
      281,                                                 // colNo
      "A_max",                                             // aName
      "check_inputs",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/check_inputs.m", // pName
      0                                                    // checkKind
  };
  static rtBoundsCheckInfo ec_emlrtBCI = {
      -1,                                                  // iFirst
      -1,                                                  // iLast
      148,                                                 // lineNo
      207,                                                 // colNo
      "V_max",                                             // aName
      "check_inputs",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/check_inputs.m", // pName
      0                                                    // checkKind
  };
  static rtBoundsCheckInfo ed_emlrtBCI = {
      -1,                                                  // iFirst
      -1,                                                  // iLast
      176,                                                 // lineNo
      292,                                                 // colNo
      "A_max",                                             // aName
      "check_inputs",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/check_inputs.m", // pName
      0                                                    // checkKind
  };
  static rtBoundsCheckInfo fc_emlrtBCI = {
      -1,                                                  // iFirst
      -1,                                                  // iLast
      148,                                                 // lineNo
      218,                                                 // colNo
      "V_max",                                             // aName
      "check_inputs",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/check_inputs.m", // pName
      0                                                    // checkKind
  };
  static rtBoundsCheckInfo fd_emlrtBCI = {
      -1,                                                  // iFirst
      -1,                                                  // iLast
      176,                                                 // lineNo
      314,                                                 // colNo
      "A_min",                                             // aName
      "check_inputs",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/check_inputs.m", // pName
      0                                                    // checkKind
  };
  static rtBoundsCheckInfo gc_emlrtBCI = {
      -1,                                                  // iFirst
      -1,                                                  // iLast
      148,                                                 // lineNo
      240,                                                 // colNo
      "V_min",                                             // aName
      "check_inputs",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/check_inputs.m", // pName
      0                                                    // checkKind
  };
  static rtBoundsCheckInfo gd_emlrtBCI = {
      -1,                                                  // iFirst
      -1,                                                  // iLast
      176,                                                 // lineNo
      325,                                                 // colNo
      "A_min",                                             // aName
      "check_inputs",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/check_inputs.m", // pName
      0                                                    // checkKind
  };
  static rtBoundsCheckInfo hc_emlrtBCI = {
      -1,                                                  // iFirst
      -1,                                                  // iLast
      148,                                                 // lineNo
      251,                                                 // colNo
      "V_min",                                             // aName
      "check_inputs",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/check_inputs.m", // pName
      0                                                    // checkKind
  };
  static rtBoundsCheckInfo hd_emlrtBCI = {
      -1,                                                  // iFirst
      -1,                                                  // iLast
      176,                                                 // lineNo
      347,                                                 // colNo
      "J_max",                                             // aName
      "check_inputs",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/check_inputs.m", // pName
      0                                                    // checkKind
  };
  static rtBoundsCheckInfo ic_emlrtBCI = {
      -1,                                                  // iFirst
      -1,                                                  // iLast
      148,                                                 // lineNo
      273,                                                 // colNo
      "A_max",                                             // aName
      "check_inputs",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/check_inputs.m", // pName
      0                                                    // checkKind
  };
  static rtBoundsCheckInfo id_emlrtBCI = {
      -1,                                                  // iFirst
      -1,                                                  // iLast
      176,                                                 // lineNo
      358,                                                 // colNo
      "J_max",                                             // aName
      "check_inputs",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/check_inputs.m", // pName
      0                                                    // checkKind
  };
  static rtBoundsCheckInfo jc_emlrtBCI = {
      -1,                                                  // iFirst
      -1,                                                  // iLast
      148,                                                 // lineNo
      284,                                                 // colNo
      "A_max",                                             // aName
      "check_inputs",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/check_inputs.m", // pName
      0                                                    // checkKind
  };
  static rtBoundsCheckInfo jd_emlrtBCI = {
      -1,                                                  // iFirst
      -1,                                                  // iLast
      176,                                                 // lineNo
      380,                                                 // colNo
      "J_min",                                             // aName
      "check_inputs",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/check_inputs.m", // pName
      0                                                    // checkKind
  };
  static rtBoundsCheckInfo kc_emlrtBCI = {
      -1,                                                  // iFirst
      -1,                                                  // iLast
      148,                                                 // lineNo
      306,                                                 // colNo
      "A_min",                                             // aName
      "check_inputs",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/check_inputs.m", // pName
      0                                                    // checkKind
  };
  static rtBoundsCheckInfo kd_emlrtBCI = {
      -1,                                                  // iFirst
      -1,                                                  // iLast
      176,                                                 // lineNo
      391,                                                 // colNo
      "J_min",                                             // aName
      "check_inputs",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/check_inputs.m", // pName
      0                                                    // checkKind
  };
  static rtBoundsCheckInfo lc_emlrtBCI = {
      -1,                                                  // iFirst
      -1,                                                  // iLast
      148,                                                 // lineNo
      317,                                                 // colNo
      "A_min",                                             // aName
      "check_inputs",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/check_inputs.m", // pName
      0                                                    // checkKind
  };
  static rtBoundsCheckInfo mc_emlrtBCI = {
      -1,                                                  // iFirst
      -1,                                                  // iLast
      148,                                                 // lineNo
      339,                                                 // colNo
      "J_max",                                             // aName
      "check_inputs",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/check_inputs.m", // pName
      0                                                    // checkKind
  };
  static rtBoundsCheckInfo nb_emlrtBCI = {
      -1,                                                  // iFirst
      -1,                                                  // iLast
      83,                                                  // lineNo
      43,                                                  // colNo
      "Waypoints",                                         // aName
      "check_inputs",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/check_inputs.m", // pName
      0                                                    // checkKind
  };
  static rtBoundsCheckInfo nc_emlrtBCI = {
      -1,                                                  // iFirst
      -1,                                                  // iLast
      148,                                                 // lineNo
      350,                                                 // colNo
      "J_max",                                             // aName
      "check_inputs",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/check_inputs.m", // pName
      0                                                    // checkKind
  };
  static rtBoundsCheckInfo ob_emlrtBCI = {
      -1,                                                  // iFirst
      -1,                                                  // iLast
      125,                                                 // lineNo
      53,                                                  // colNo
      "State_start",                                       // aName
      "check_inputs",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/check_inputs.m", // pName
      0                                                    // checkKind
  };
  static rtBoundsCheckInfo oc_emlrtBCI = {
      -1,                                                  // iFirst
      -1,                                                  // iLast
      148,                                                 // lineNo
      372,                                                 // colNo
      "J_min",                                             // aName
      "check_inputs",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/check_inputs.m", // pName
      0                                                    // checkKind
  };
  static rtBoundsCheckInfo pb_emlrtBCI = {
      -1,                                                  // iFirst
      -1,                                                  // iLast
      125,                                                 // lineNo
      99,                                                  // colNo
      "V_max",                                             // aName
      "check_inputs",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/check_inputs.m", // pName
      0                                                    // checkKind
  };
  static rtBoundsCheckInfo pc_emlrtBCI = {
      -1,                                                  // iFirst
      -1,                                                  // iLast
      148,                                                 // lineNo
      383,                                                 // colNo
      "J_min",                                             // aName
      "check_inputs",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/check_inputs.m", // pName
      0                                                    // checkKind
  };
  static rtBoundsCheckInfo qb_emlrtBCI = {
      -1,                                                  // iFirst
      -1,                                                  // iLast
      125,                                                 // lineNo
      117,                                                 // colNo
      "V_min",                                             // aName
      "check_inputs",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/check_inputs.m", // pName
      0                                                    // checkKind
  };
  static rtBoundsCheckInfo qc_emlrtBCI = {
      -1,                                                  // iFirst
      -1,                                                  // iLast
      176,                                                 // lineNo
      55,                                                  // colNo
      "Waypoints",                                         // aName
      "check_inputs",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/check_inputs.m", // pName
      0                                                    // checkKind
  };
  static rtBoundsCheckInfo rb_emlrtBCI = {
      -1,                                                  // iFirst
      -1,                                                  // iLast
      125,                                                 // lineNo
      135,                                                 // colNo
      "A_max",                                             // aName
      "check_inputs",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/check_inputs.m", // pName
      0                                                    // checkKind
  };
  static rtBoundsCheckInfo rc_emlrtBCI = {
      -1,                                                  // iFirst
      -1,                                                  // iLast
      176,                                                 // lineNo
      68,                                                  // colNo
      "Waypoints",                                         // aName
      "check_inputs",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/check_inputs.m", // pName
      0                                                    // checkKind
  };
  static rtBoundsCheckInfo sb_emlrtBCI = {
      -1,                                                  // iFirst
      -1,                                                  // iLast
      125,                                                 // lineNo
      153,                                                 // colNo
      "A_min",                                             // aName
      "check_inputs",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/check_inputs.m", // pName
      0                                                    // checkKind
  };
  static rtBoundsCheckInfo sc_emlrtBCI = {
      -1,                                                  // iFirst
      -1,                                                  // iLast
      176,                                                 // lineNo
      96,                                                  // colNo
      "Waypoints",                                         // aName
      "check_inputs",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/check_inputs.m", // pName
      0                                                    // checkKind
  };
  static rtBoundsCheckInfo tb_emlrtBCI = {
      -1,                                                  // iFirst
      -1,                                                  // iLast
      125,                                                 // lineNo
      171,                                                 // colNo
      "J_max",                                             // aName
      "check_inputs",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/check_inputs.m", // pName
      0                                                    // checkKind
  };
  static rtBoundsCheckInfo tc_emlrtBCI = {
      -1,                                                  // iFirst
      -1,                                                  // iLast
      176,                                                 // lineNo
      109,                                                 // colNo
      "Waypoints",                                         // aName
      "check_inputs",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/check_inputs.m", // pName
      0                                                    // checkKind
  };
  static rtBoundsCheckInfo ub_emlrtBCI = {
      -1,                                                  // iFirst
      -1,                                                  // iLast
      125,                                                 // lineNo
      189,                                                 // colNo
      "J_min",                                             // aName
      "check_inputs",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/check_inputs.m", // pName
      0                                                    // checkKind
  };
  static rtBoundsCheckInfo uc_emlrtBCI = {
      -1,                                                  // iFirst
      -1,                                                  // iLast
      176,                                                 // lineNo
      137,                                                 // colNo
      "Waypoints",                                         // aName
      "check_inputs",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/check_inputs.m", // pName
      0                                                    // checkKind
  };
  static rtBoundsCheckInfo vb_emlrtBCI = {
      -1,                                                  // iFirst
      -1,                                                  // iLast
      148,                                                 // lineNo
      55,                                                  // colNo
      "Waypoints",                                         // aName
      "check_inputs",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/check_inputs.m", // pName
      0                                                    // checkKind
  };
  static rtBoundsCheckInfo vc_emlrtBCI = {
      -1,                                                  // iFirst
      -1,                                                  // iLast
      176,                                                 // lineNo
      150,                                                 // colNo
      "Waypoints",                                         // aName
      "check_inputs",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/check_inputs.m", // pName
      0                                                    // checkKind
  };
  static rtBoundsCheckInfo wb_emlrtBCI = {
      -1,                                                  // iFirst
      -1,                                                  // iLast
      148,                                                 // lineNo
      68,                                                  // colNo
      "Waypoints",                                         // aName
      "check_inputs",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/check_inputs.m", // pName
      0                                                    // checkKind
  };
  static rtBoundsCheckInfo wc_emlrtBCI = {
      -1,                                                  // iFirst
      -1,                                                  // iLast
      176,                                                 // lineNo
      178,                                                 // colNo
      "Waypoints",                                         // aName
      "check_inputs",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/check_inputs.m", // pName
      0                                                    // checkKind
  };
  static rtBoundsCheckInfo xb_emlrtBCI = {
      -1,                                                  // iFirst
      -1,                                                  // iLast
      148,                                                 // lineNo
      94,                                                  // colNo
      "Waypoints",                                         // aName
      "check_inputs",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/check_inputs.m", // pName
      0                                                    // checkKind
  };
  static rtBoundsCheckInfo xc_emlrtBCI = {
      -1,                                                  // iFirst
      -1,                                                  // iLast
      176,                                                 // lineNo
      191,                                                 // colNo
      "Waypoints",                                         // aName
      "check_inputs",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/check_inputs.m", // pName
      0                                                    // checkKind
  };
  static rtBoundsCheckInfo yb_emlrtBCI = {
      -1,                                                  // iFirst
      -1,                                                  // iLast
      148,                                                 // lineNo
      107,                                                 // colNo
      "Waypoints",                                         // aName
      "check_inputs",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/check_inputs.m", // pName
      0                                                    // checkKind
  };
  static rtBoundsCheckInfo yc_emlrtBCI = {
      -1,                                                  // iFirst
      -1,                                                  // iLast
      176,                                                 // lineNo
      215,                                                 // colNo
      "V_max",                                             // aName
      "check_inputs",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/check_inputs.m", // pName
      0                                                    // checkKind
  };
  static rtEqualityCheckInfo c_emlrtECI = {
      2,                                                  // nDims
      66,                                                 // lineNo
      17,                                                 // colNo
      "check_inputs",                                     // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/check_inputs.m" // pName
  };
  static rtEqualityCheckInfo d_emlrtECI = {
      2,                                                  // nDims
      87,                                                 // lineNo
      13,                                                 // colNo
      "check_inputs",                                     // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/check_inputs.m" // pName
  };
  static rtEqualityCheckInfo e_emlrtECI = {
      2,                                                  // nDims
      91,                                                 // lineNo
      13,                                                 // colNo
      "check_inputs",                                     // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/check_inputs.m" // pName
  };
  static rtEqualityCheckInfo f_emlrtECI = {
      2,                                                  // nDims
      95,                                                 // lineNo
      13,                                                 // colNo
      "check_inputs",                                     // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/check_inputs.m" // pName
  };
  static rtEqualityCheckInfo g_emlrtECI = {
      2,                                                  // nDims
      99,                                                 // lineNo
      13,                                                 // colNo
      "check_inputs",                                     // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/check_inputs.m" // pName
  };
  static rtRunTimeErrorInfo s_emlrtRTEI = {
      52,                  // lineNo
      13,                  // colNo
      "reshapeSizeChecks", // fName
      "/usr/local/MATLAB/R2021a/toolbox/eml/eml/+coder/+internal/"
      "reshapeSizeChecks.m" // pName
  };
  static rtRunTimeErrorInfo t_emlrtRTEI = {
      59,                  // lineNo
      23,                  // colNo
      "reshapeSizeChecks", // fName
      "/usr/local/MATLAB/R2021a/toolbox/eml/eml/+coder/+internal/"
      "reshapeSizeChecks.m" // pName
  };
  coder::array<double, 3U> c_Waypoints;
  coder::array<double, 2U> b_nz;
  coder::array<int, 2U> nz;
  coder::array<int, 1U> c_nz;
  coder::array<bool, 3U> c_s;
  coder::array<bool, 3U> d_s;
  coder::array<bool, 2U> b_s;
  coder::array<bool, 2U> r;
  coder::array<bool, 2U> r1;
  coder::array<bool, 2U> s;
  coder::array<bool, 1U> d_nz;
  double t1_f2[3];
  double t0_f2[2];
  double y;
  int b_Waypoints[2];
  int iv[2];
  int i;
  int i1;
  int index_waypoint;
  int maxdimlen;
  int num_waypoints;
  int nx;
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
  num_waypoints = Waypoints.size(2);
  t0_f2[0] = Waypoints.size(0);
  t0_f2[1] = 3.0;
  coder::validateattributes(State_start, t0_f2);
  t1_f2[0] = static_cast<unsigned int>(Waypoints.size(0));
  t1_f2[1] = 5.0;
  t1_f2[2] = static_cast<unsigned int>(Waypoints.size(2));
  coder::validateattributes(Waypoints, t1_f2);
  t0_f2[0] = static_cast<unsigned int>(Waypoints.size(0));
  t0_f2[1] = static_cast<unsigned int>(Waypoints.size(2));
  coder::b_validateattributes(V_max, t0_f2);
  t0_f2[0] = static_cast<unsigned int>(Waypoints.size(0));
  t0_f2[1] = static_cast<unsigned int>(Waypoints.size(2));
  coder::c_validateattributes(V_min, t0_f2);
  t0_f2[0] = static_cast<unsigned int>(Waypoints.size(0));
  t0_f2[1] = static_cast<unsigned int>(Waypoints.size(2));
  coder::d_validateattributes(A_max, t0_f2);
  t0_f2[0] = static_cast<unsigned int>(Waypoints.size(0));
  t0_f2[1] = static_cast<unsigned int>(Waypoints.size(2));
  coder::e_validateattributes(A_min, t0_f2);
  t0_f2[0] = static_cast<unsigned int>(Waypoints.size(0));
  t0_f2[1] = static_cast<unsigned int>(Waypoints.size(2));
  coder::f_validateattributes(J_max, t0_f2);
  t0_f2[0] = static_cast<unsigned int>(Waypoints.size(0));
  t0_f2[1] = static_cast<unsigned int>(Waypoints.size(2));
  coder::g_validateattributes(J_min, t0_f2);
  t0_f2[0] = Waypoints.size(0);
  t0_f2[1] = 1.0;
  coder::validateattributes(A_global, t0_f2);
  t0_f2[0] = static_cast<unsigned int>(Waypoints.size(0));
  t0_f2[1] = static_cast<unsigned int>(Waypoints.size(2));
  coder::h_validateattributes(b_sync_V, t0_f2);
  t0_f2[0] = static_cast<unsigned int>(Waypoints.size(0));
  t0_f2[1] = static_cast<unsigned int>(Waypoints.size(2));
  coder::i_validateattributes(b_sync_A, t0_f2);
  t0_f2[0] = static_cast<unsigned int>(Waypoints.size(0));
  t0_f2[1] = static_cast<unsigned int>(Waypoints.size(2));
  coder::j_validateattributes(b_sync_J, t0_f2);
  t0_f2[0] = static_cast<unsigned int>(Waypoints.size(0));
  t0_f2[1] = static_cast<unsigned int>(Waypoints.size(2));
  coder::k_validateattributes(b_sync_W, t0_f2);
  t0_f2[0] = Waypoints.size(0) - 1;
  t0_f2[1] = Waypoints.size(2);
  coder::l_validateattributes(b_rotate, t0_f2);
  t0_f2[0] = static_cast<unsigned int>(Waypoints.size(0));
  t0_f2[1] = static_cast<unsigned int>(Waypoints.size(2));
  coder::m_validateattributes(b_hard_V_lim, t0_f2);
  t0_f2[0] = static_cast<unsigned int>(Waypoints.size(0));
  t0_f2[1] = static_cast<unsigned int>(Waypoints.size(2));
  coder::n_validateattributes(b_catch_up, t0_f2);
  t0_f2[0] = static_cast<unsigned int>(Waypoints.size(0));
  t0_f2[1] = static_cast<unsigned int>(Waypoints.size(2));
  coder::validateattributes(direction, t0_f2);
  coder::validateattributes(ts_rollout);
  s.set_size(direction.size(0), direction.size(1));
  maxdimlen = direction.size(0) * direction.size(1);
  for (i = 0; i < maxdimlen; i++) {
    s[i] = (direction[i] == -1);
  }
  r.set_size(direction.size(0), direction.size(1));
  maxdimlen = direction.size(0) * direction.size(1);
  for (i = 0; i < maxdimlen; i++) {
    r[i] = (direction[i] == 0);
  }
  iv[0] = (*(int(*)[2])s.size())[0];
  iv[1] = (*(int(*)[2])s.size())[1];
  b_Waypoints[0] = (*(int(*)[2])r.size())[0];
  b_Waypoints[1] = (*(int(*)[2])r.size())[1];
  rtSizeEqNDCheck(&iv[0], &b_Waypoints[0], &c_emlrtECI);
  maxdimlen = s.size(0) * s.size(1);
  for (i = 0; i < maxdimlen; i++) {
    s[i] = (s[i] || r[i]);
  }
  r.set_size(direction.size(0), direction.size(1));
  maxdimlen = direction.size(0) * direction.size(1);
  for (i = 0; i < maxdimlen; i++) {
    r[i] = (direction[i] == 1);
  }
  iv[0] = (*(int(*)[2])s.size())[0];
  iv[1] = (*(int(*)[2])s.size())[1];
  b_Waypoints[0] = (*(int(*)[2])r.size())[0];
  b_Waypoints[1] = (*(int(*)[2])r.size())[1];
  rtSizeEqNDCheck(&iv[0], &b_Waypoints[0], &c_emlrtECI);
  b_s.set_size(s.size(0), s.size(1));
  maxdimlen = s.size(0) * s.size(1);
  for (i = 0; i < maxdimlen; i++) {
    b_s[i] = (s[i] || r[i]);
  }
  coder::combineVectorElements(b_s, nz);
  b_nz.set_size(1, nz.size(1));
  maxdimlen = nz.size(1);
  for (i = 0; i < maxdimlen; i++) {
    b_nz[i] = nz[i];
  }
  y = coder::b_combineVectorElements(b_nz);
  if (y != static_cast<double>(Waypoints.size(0)) *
               static_cast<double>(Waypoints.size(2))) {
    printf("Error: direction is not -1, 0 or +1!\n");
    fflush(stdout);
  }
  c_s.set_size(Waypoints.size(0), 5, Waypoints.size(2));
  maxdimlen = Waypoints.size(0) * 5 * Waypoints.size(2);
  for (i = 0; i < maxdimlen; i++) {
    c_s[i] = rtIsInf(Waypoints[i]);
  }
  nx = 0;
  i = c_s.size(0) * 5 * c_s.size(2);
  for (maxdimlen = 0; maxdimlen < i; maxdimlen++) {
    if (c_s[maxdimlen]) {
      nx++;
    }
  }
  if (nx > 0) {
    printf("Error: Waypoints not allowed to be Inf!\n");
    fflush(stdout);
  }
  maxdimlen = Waypoints.size(0);
  nx = Waypoints.size(2);
  d_s.set_size(Waypoints.size(0), 1, Waypoints.size(2));
  for (i = 0; i < nx; i++) {
    for (i1 = 0; i1 < maxdimlen; i1++) {
      d_s[i1 + d_s.size(0) * i] = (Waypoints[(i1 + Waypoints.size(0) * 4) +
                                             Waypoints.size(0) * 5 * i] != 0.0);
    }
  }
  nx = 0;
  i = d_s.size(0) * d_s.size(2);
  for (maxdimlen = 0; maxdimlen < i; maxdimlen++) {
    if (d_s[maxdimlen]) {
      nx++;
    }
  }
  if (nx > 0) {
    printf("Error: Waypoint acceleration prediction not allowed to be ~= 0!\n");
    fflush(stdout);
  }
  maxdimlen = Waypoints.size(0);
  nx = Waypoints.size(2);
  d_s.set_size(Waypoints.size(0), 1, Waypoints.size(2));
  for (i = 0; i < nx; i++) {
    for (i1 = 0; i1 < maxdimlen; i1++) {
      d_s[i1 + d_s.size(0) * i] = rtIsNaN(
          Waypoints[(i1 + Waypoints.size(0) * 3) + Waypoints.size(0) * 5 * i]);
    }
  }
  nx = 0;
  i = d_s.size(0) * d_s.size(2);
  for (maxdimlen = 0; maxdimlen < i; maxdimlen++) {
    if (d_s[maxdimlen]) {
      nx++;
    }
  }
  if (nx > 0) {
    printf("Error: Waypoint velocity prediction not allowed to be NaN!\n");
    fflush(stdout);
  }
  maxdimlen = Waypoints.size(0);
  nx = Waypoints.size(2);
  d_s.set_size(Waypoints.size(0), 1, Waypoints.size(2));
  for (i = 0; i < nx; i++) {
    for (i1 = 0; i1 < maxdimlen; i1++) {
      d_s[i1 + d_s.size(0) * i] = rtIsNaN(
          Waypoints[(i1 + Waypoints.size(0) * 4) + Waypoints.size(0) * 5 * i]);
    }
  }
  nx = 0;
  i = d_s.size(0) * d_s.size(2);
  for (maxdimlen = 0; maxdimlen < i; maxdimlen++) {
    if (d_s[maxdimlen]) {
      nx++;
    }
  }
  if (nx > 0) {
    printf("Error: Waypoint acceleration prediction not allowed to be NaN!\n");
    fflush(stdout);
  }
  i = Waypoints.size(2);
  for (index_waypoint = 0; index_waypoint < i; index_waypoint++) {
    if (index_waypoint + 1 > Waypoints.size(2)) {
      rtDynamicBoundsError(index_waypoint + 1, 1, Waypoints.size(2),
                           &nb_emlrtBCI);
    }
    maxdimlen = Waypoints.size(0);
    r1.set_size(Waypoints.size(0), 3);
    for (i1 = 0; i1 < 3; i1++) {
      for (nx = 0; nx < maxdimlen; nx++) {
        r1[nx + r1.size(0) * i1] =
            rtIsNaN(Waypoints[(nx + Waypoints.size(0) * i1) +
                              Waypoints.size(0) * 5 * index_waypoint]);
      }
    }
    coder::c_combineVectorElements(r1, c_nz);
    d_nz.set_size(c_nz.size(0));
    maxdimlen = c_nz.size(0);
    for (i1 = 0; i1 < maxdimlen; i1++) {
      d_nz[i1] = (c_nz[i1] == 3);
    }
    if (coder::any(d_nz)) {
      printf("Error: Waypoint needs to be defined by at least one nonNaN in "
             "each axis!\n");
      fflush(stdout);
    }
  }
  nx = Waypoints.size(0) * Waypoints.size(2);
  maxdimlen = Waypoints.size(0);
  if (1 > Waypoints.size(0)) {
    maxdimlen = 1;
  }
  if (Waypoints.size(2) > maxdimlen) {
    maxdimlen = Waypoints.size(2);
  }
  if (nx > maxdimlen) {
    maxdimlen = nx;
  }
  if (Waypoints.size(0) > maxdimlen) {
    b_rtErrorWithMessageID(s_emlrtRTEI.fName, s_emlrtRTEI.lineNo);
  }
  if (Waypoints.size(2) > maxdimlen) {
    b_rtErrorWithMessageID(s_emlrtRTEI.fName, s_emlrtRTEI.lineNo);
  }
  if (Waypoints.size(0) * Waypoints.size(2) != nx) {
    rtErrorWithMessageID(t_emlrtRTEI.fName, t_emlrtRTEI.lineNo);
  }
  b_Waypoints[0] = Waypoints.size(0);
  b_Waypoints[1] = Waypoints.size(2);
  iv[0] = (*(int(*)[2])((coder::array<double, 2U> *)&V_max)->size())[0];
  iv[1] = (*(int(*)[2])((coder::array<double, 2U> *)&V_max)->size())[1];
  rtSizeEqNDCheck(&b_Waypoints[0], &iv[0], &d_emlrtECI);
  maxdimlen = Waypoints.size(0);
  nx = Waypoints.size(2);
  c_Waypoints.set_size(Waypoints.size(0), 1, Waypoints.size(2));
  for (i = 0; i < nx; i++) {
    for (i1 = 0; i1 < maxdimlen; i1++) {
      c_Waypoints[i1 + c_Waypoints.size(0) * i] =
          Waypoints[(i1 + Waypoints.size(0) * 3) + Waypoints.size(0) * 5 * i];
    }
  }
  s.set_size(Waypoints.size(0), Waypoints.size(2));
  maxdimlen = Waypoints.size(0) * Waypoints.size(2);
  for (i = 0; i < maxdimlen; i++) {
    s[i] = (c_Waypoints[i] > V_max[i]);
  }
  nx = 0;
  i = s.size(0) * s.size(1);
  for (maxdimlen = 0; maxdimlen < i; maxdimlen++) {
    if (s[maxdimlen]) {
      nx++;
    }
  }
  if (nx > 0) {
    printf("Error: Waypoint velocity prediction exceeds V_max!\n");
    fflush(stdout);
  }
  nx = Waypoints.size(0) * Waypoints.size(2);
  maxdimlen = Waypoints.size(0);
  if (1 > Waypoints.size(0)) {
    maxdimlen = 1;
  }
  if (Waypoints.size(2) > maxdimlen) {
    maxdimlen = Waypoints.size(2);
  }
  if (nx > maxdimlen) {
    maxdimlen = nx;
  }
  if (Waypoints.size(0) > maxdimlen) {
    b_rtErrorWithMessageID(s_emlrtRTEI.fName, s_emlrtRTEI.lineNo);
  }
  if (Waypoints.size(2) > maxdimlen) {
    b_rtErrorWithMessageID(s_emlrtRTEI.fName, s_emlrtRTEI.lineNo);
  }
  if (Waypoints.size(0) * Waypoints.size(2) != nx) {
    rtErrorWithMessageID(t_emlrtRTEI.fName, t_emlrtRTEI.lineNo);
  }
  b_Waypoints[0] = Waypoints.size(0);
  b_Waypoints[1] = Waypoints.size(2);
  iv[0] = (*(int(*)[2])((coder::array<double, 2U> *)&V_min)->size())[0];
  iv[1] = (*(int(*)[2])((coder::array<double, 2U> *)&V_min)->size())[1];
  rtSizeEqNDCheck(&b_Waypoints[0], &iv[0], &e_emlrtECI);
  maxdimlen = Waypoints.size(0);
  nx = Waypoints.size(2);
  c_Waypoints.set_size(Waypoints.size(0), 1, Waypoints.size(2));
  for (i = 0; i < nx; i++) {
    for (i1 = 0; i1 < maxdimlen; i1++) {
      c_Waypoints[i1 + c_Waypoints.size(0) * i] =
          Waypoints[(i1 + Waypoints.size(0) * 3) + Waypoints.size(0) * 5 * i];
    }
  }
  s.set_size(Waypoints.size(0), Waypoints.size(2));
  maxdimlen = Waypoints.size(0) * Waypoints.size(2);
  for (i = 0; i < maxdimlen; i++) {
    s[i] = (c_Waypoints[i] < V_min[i]);
  }
  nx = 0;
  i = s.size(0) * s.size(1);
  for (maxdimlen = 0; maxdimlen < i; maxdimlen++) {
    if (s[maxdimlen]) {
      nx++;
    }
  }
  if (nx > 0) {
    printf("Error: Waypoint velocity prediction exceeds V_min!\n");
    fflush(stdout);
  }
  nx = Waypoints.size(0) * Waypoints.size(2);
  maxdimlen = Waypoints.size(0);
  if (1 > Waypoints.size(0)) {
    maxdimlen = 1;
  }
  if (Waypoints.size(2) > maxdimlen) {
    maxdimlen = Waypoints.size(2);
  }
  if (nx > maxdimlen) {
    maxdimlen = nx;
  }
  if (Waypoints.size(0) > maxdimlen) {
    b_rtErrorWithMessageID(s_emlrtRTEI.fName, s_emlrtRTEI.lineNo);
  }
  if (Waypoints.size(2) > maxdimlen) {
    b_rtErrorWithMessageID(s_emlrtRTEI.fName, s_emlrtRTEI.lineNo);
  }
  if (Waypoints.size(0) * Waypoints.size(2) != nx) {
    rtErrorWithMessageID(t_emlrtRTEI.fName, t_emlrtRTEI.lineNo);
  }
  b_Waypoints[0] = Waypoints.size(0);
  b_Waypoints[1] = Waypoints.size(2);
  iv[0] = (*(int(*)[2])((coder::array<double, 2U> *)&A_max)->size())[0];
  iv[1] = (*(int(*)[2])((coder::array<double, 2U> *)&A_max)->size())[1];
  rtSizeEqNDCheck(&b_Waypoints[0], &iv[0], &f_emlrtECI);
  maxdimlen = Waypoints.size(0);
  nx = Waypoints.size(2);
  c_Waypoints.set_size(Waypoints.size(0), 1, Waypoints.size(2));
  for (i = 0; i < nx; i++) {
    for (i1 = 0; i1 < maxdimlen; i1++) {
      c_Waypoints[i1 + c_Waypoints.size(0) * i] =
          Waypoints[(i1 + Waypoints.size(0) * 4) + Waypoints.size(0) * 5 * i];
    }
  }
  s.set_size(Waypoints.size(0), Waypoints.size(2));
  maxdimlen = Waypoints.size(0) * Waypoints.size(2);
  for (i = 0; i < maxdimlen; i++) {
    s[i] = (c_Waypoints[i] > A_max[i]);
  }
  nx = 0;
  i = s.size(0) * s.size(1);
  for (maxdimlen = 0; maxdimlen < i; maxdimlen++) {
    if (s[maxdimlen]) {
      nx++;
    }
  }
  if (nx > 0) {
    printf("Error: Waypoint acceleration prediction exceeds A_max!\n");
    fflush(stdout);
  }
  nx = Waypoints.size(0) * Waypoints.size(2);
  maxdimlen = Waypoints.size(0);
  if (1 > Waypoints.size(0)) {
    maxdimlen = 1;
  }
  if (Waypoints.size(2) > maxdimlen) {
    maxdimlen = Waypoints.size(2);
  }
  if (nx > maxdimlen) {
    maxdimlen = nx;
  }
  if (Waypoints.size(0) > maxdimlen) {
    b_rtErrorWithMessageID(s_emlrtRTEI.fName, s_emlrtRTEI.lineNo);
  }
  if (Waypoints.size(2) > maxdimlen) {
    b_rtErrorWithMessageID(s_emlrtRTEI.fName, s_emlrtRTEI.lineNo);
  }
  if (Waypoints.size(0) * Waypoints.size(2) != nx) {
    rtErrorWithMessageID(t_emlrtRTEI.fName, t_emlrtRTEI.lineNo);
  }
  b_Waypoints[0] = Waypoints.size(0);
  b_Waypoints[1] = Waypoints.size(2);
  iv[0] = (*(int(*)[2])((coder::array<double, 2U> *)&A_min)->size())[0];
  iv[1] = (*(int(*)[2])((coder::array<double, 2U> *)&A_min)->size())[1];
  rtSizeEqNDCheck(&b_Waypoints[0], &iv[0], &g_emlrtECI);
  maxdimlen = Waypoints.size(0);
  nx = Waypoints.size(2);
  c_Waypoints.set_size(Waypoints.size(0), 1, Waypoints.size(2));
  for (i = 0; i < nx; i++) {
    for (i1 = 0; i1 < maxdimlen; i1++) {
      c_Waypoints[i1 + c_Waypoints.size(0) * i] =
          Waypoints[(i1 + Waypoints.size(0) * 4) + Waypoints.size(0) * 5 * i];
    }
  }
  s.set_size(Waypoints.size(0), Waypoints.size(2));
  maxdimlen = Waypoints.size(0) * Waypoints.size(2);
  for (i = 0; i < maxdimlen; i++) {
    s[i] = (c_Waypoints[i] < A_min[i]);
  }
  nx = 0;
  i = s.size(0) * s.size(1);
  for (maxdimlen = 0; maxdimlen < i; maxdimlen++) {
    if (s[maxdimlen]) {
      nx++;
    }
  }
  if (nx > 0) {
    printf("Error: Waypoint acceleration prediction exceeds A_min!\n");
    fflush(stdout);
  }
  s.set_size(V_max.size(0), V_max.size(1));
  maxdimlen = V_max.size(0) * V_max.size(1);
  for (i = 0; i < maxdimlen; i++) {
    s[i] = (V_max[i] < 0.0);
  }
  nx = 0;
  i = s.size(0) * s.size(1);
  for (maxdimlen = 0; maxdimlen < i; maxdimlen++) {
    if (s[maxdimlen]) {
      nx++;
    }
  }
  if (nx > 0) {
    printf("Error: V_max < 0!\n");
    fflush(stdout);
  }
  s.set_size(A_max.size(0), A_max.size(1));
  maxdimlen = A_max.size(0) * A_max.size(1);
  for (i = 0; i < maxdimlen; i++) {
    s[i] = (A_max[i] < 0.0);
  }
  nx = 0;
  i = s.size(0) * s.size(1);
  for (maxdimlen = 0; maxdimlen < i; maxdimlen++) {
    if (s[maxdimlen]) {
      nx++;
    }
  }
  if (nx > 0) {
    printf("Error: A_max < 0!\n");
    fflush(stdout);
  }
  s.set_size(J_max.size(0), J_max.size(1));
  maxdimlen = J_max.size(0) * J_max.size(1);
  for (i = 0; i < maxdimlen; i++) {
    s[i] = (J_max[i] < 0.0);
  }
  nx = 0;
  i = s.size(0) * s.size(1);
  for (maxdimlen = 0; maxdimlen < i; maxdimlen++) {
    if (s[maxdimlen]) {
      nx++;
    }
  }
  if (nx > 0) {
    printf("Error: J_max < 0!\n");
    fflush(stdout);
  }
  s.set_size(V_min.size(0), V_min.size(1));
  maxdimlen = V_min.size(0) * V_min.size(1);
  for (i = 0; i < maxdimlen; i++) {
    s[i] = (V_min[i] > 0.0);
  }
  nx = 0;
  i = s.size(0) * s.size(1);
  for (maxdimlen = 0; maxdimlen < i; maxdimlen++) {
    if (s[maxdimlen]) {
      nx++;
    }
  }
  if (nx > 0) {
    printf("Error: V_min > 0!\n");
    fflush(stdout);
  }
  s.set_size(A_min.size(0), A_min.size(1));
  maxdimlen = A_min.size(0) * A_min.size(1);
  for (i = 0; i < maxdimlen; i++) {
    s[i] = (A_min[i] > 0.0);
  }
  nx = 0;
  i = s.size(0) * s.size(1);
  for (maxdimlen = 0; maxdimlen < i; maxdimlen++) {
    if (s[maxdimlen]) {
      nx++;
    }
  }
  if (nx > 0) {
    printf("Error: A_min > 0!\n");
    fflush(stdout);
  }
  s.set_size(J_min.size(0), J_min.size(1));
  maxdimlen = J_min.size(0) * J_min.size(1);
  for (i = 0; i < maxdimlen; i++) {
    s[i] = (J_min[i] > 0.0);
  }
  nx = 0;
  i = s.size(0) * s.size(1);
  for (maxdimlen = 0; maxdimlen < i; maxdimlen++) {
    if (s[maxdimlen]) {
      nx++;
    }
  }
  if (nx > 0) {
    printf("Error: J_min > 0!\n");
    fflush(stdout);
  }
  i = Waypoints.size(0);
  for (nx = 0; nx < i; nx++) {
    if (nx + 1 > State_start.size(0)) {
      rtDynamicBoundsError(nx + 1, 1, State_start.size(0), &ob_emlrtBCI);
    }
    i1 = V_max.size(0) * V_max.size(1);
    if (nx + 1 > i1) {
      rtDynamicBoundsError(nx + 1, 1, i1, &pb_emlrtBCI);
    }
    i1 = V_min.size(0) * V_min.size(1);
    if (nx + 1 > i1) {
      rtDynamicBoundsError(nx + 1, 1, i1, &qb_emlrtBCI);
    }
    i1 = A_max.size(0) * A_max.size(1);
    if (nx + 1 > i1) {
      rtDynamicBoundsError(nx + 1, 1, i1, &rb_emlrtBCI);
    }
    i1 = A_min.size(0) * A_min.size(1);
    if (nx + 1 > i1) {
      rtDynamicBoundsError(nx + 1, 1, i1, &sb_emlrtBCI);
    }
    i1 = J_max.size(0) * J_max.size(1);
    if (nx + 1 > i1) {
      rtDynamicBoundsError(nx + 1, 1, i1, &tb_emlrtBCI);
    }
    i1 = J_min.size(0) * J_min.size(1);
    if (nx + 1 > i1) {
      rtDynamicBoundsError(nx + 1, 1, i1, &ub_emlrtBCI);
    }
    switch (check_feasibility(State_start[nx + State_start.size(0)],
                              State_start[nx + State_start.size(0) * 2],
                              V_max[nx], V_min[nx], A_max[nx], A_min[nx],
                              J_max[nx], J_min[nx])) {
    case 1U:
      printf("Warning: A_init > A_max!\n");
      fflush(stdout);
      break;
    case 2U:
      printf("Warning: A_init < A_min!\n");
      fflush(stdout);
      break;
    case 3U:
      printf("Warning: V_init > V_max!\n");
      fflush(stdout);
      break;
    case 4U:
      printf("Warning: V_init < V_min!\n");
      fflush(stdout);
      break;
    case 5U:
      printf("Warning: V_init to close to V_max to prevent future violation of "
             "constraints with A_wayp and J_min!\n");
      fflush(stdout);
      break;
    case 6U:
      printf("Warning: V_init to close to V_min to prevent future violation of "
             "constraints with A_wayp and J_max!\n");
      fflush(stdout);
      break;
    case 7U:
      printf("Warning: V_init to close to V_min and A_init too large "
             "(positive) to be reached with J_max!\n");
      fflush(stdout);
      break;
    case 8U:
      printf("Warning: V_init to close to V_max and A_init too large "
             "(negative) to be reached with J_min!\n");
      fflush(stdout);
      break;
    }
  }
  i = Waypoints.size(0);
  for (nx = 0; nx < i; nx++) {
    for (index_waypoint = 0; index_waypoint < num_waypoints; index_waypoint++) {
      if (nx + 1 > Waypoints.size(0)) {
        rtDynamicBoundsError(nx + 1, 1, Waypoints.size(0), &vb_emlrtBCI);
      }
      if (index_waypoint + 1 > Waypoints.size(2)) {
        rtDynamicBoundsError(index_waypoint + 1, 1, Waypoints.size(2),
                             &wb_emlrtBCI);
      }
      if (nx + 1 > Waypoints.size(0)) {
        rtDynamicBoundsError(nx + 1, 1, Waypoints.size(0), &xb_emlrtBCI);
      }
      if (index_waypoint + 1 > Waypoints.size(2)) {
        rtDynamicBoundsError(index_waypoint + 1, 1, Waypoints.size(2),
                             &yb_emlrtBCI);
      }
      if (nx + 1 > Waypoints.size(0)) {
        rtDynamicBoundsError(nx + 1, 1, Waypoints.size(0), &ac_emlrtBCI);
      }
      if (index_waypoint + 1 > Waypoints.size(2)) {
        rtDynamicBoundsError(index_waypoint + 1, 1, Waypoints.size(2),
                             &bc_emlrtBCI);
      }
      if (nx + 1 > Waypoints.size(0)) {
        rtDynamicBoundsError(nx + 1, 1, Waypoints.size(0), &cc_emlrtBCI);
      }
      if (index_waypoint + 1 > Waypoints.size(2)) {
        rtDynamicBoundsError(index_waypoint + 1, 1, Waypoints.size(2),
                             &dc_emlrtBCI);
      }
      if (nx + 1 > V_max.size(0)) {
        rtDynamicBoundsError(nx + 1, 1, V_max.size(0), &ec_emlrtBCI);
      }
      if (index_waypoint + 1 > V_max.size(1)) {
        rtDynamicBoundsError(index_waypoint + 1, 1, V_max.size(1),
                             &fc_emlrtBCI);
      }
      if (nx + 1 > V_min.size(0)) {
        rtDynamicBoundsError(nx + 1, 1, V_min.size(0), &gc_emlrtBCI);
      }
      if (index_waypoint + 1 > V_min.size(1)) {
        rtDynamicBoundsError(index_waypoint + 1, 1, V_min.size(1),
                             &hc_emlrtBCI);
      }
      if (nx + 1 > A_max.size(0)) {
        rtDynamicBoundsError(nx + 1, 1, A_max.size(0), &ic_emlrtBCI);
      }
      if (index_waypoint + 1 > A_max.size(1)) {
        rtDynamicBoundsError(index_waypoint + 1, 1, A_max.size(1),
                             &jc_emlrtBCI);
      }
      if (nx + 1 > A_min.size(0)) {
        rtDynamicBoundsError(nx + 1, 1, A_min.size(0), &kc_emlrtBCI);
      }
      if (index_waypoint + 1 > A_min.size(1)) {
        rtDynamicBoundsError(index_waypoint + 1, 1, A_min.size(1),
                             &lc_emlrtBCI);
      }
      if (nx + 1 > J_max.size(0)) {
        rtDynamicBoundsError(nx + 1, 1, J_max.size(0), &mc_emlrtBCI);
      }
      if (index_waypoint + 1 > J_max.size(1)) {
        rtDynamicBoundsError(index_waypoint + 1, 1, J_max.size(1),
                             &nc_emlrtBCI);
      }
      if (nx + 1 > J_min.size(0)) {
        rtDynamicBoundsError(nx + 1, 1, J_min.size(0), &oc_emlrtBCI);
      }
      if (index_waypoint + 1 > J_min.size(1)) {
        rtDynamicBoundsError(index_waypoint + 1, 1, J_min.size(1),
                             &pc_emlrtBCI);
      }
      switch (check_feasibility(
          Waypoints[(nx + Waypoints.size(0)) +
                    Waypoints.size(0) * 5 * index_waypoint] +
              Waypoints[(nx + Waypoints.size(0) * 3) +
                        Waypoints.size(0) * 5 * index_waypoint],
          Waypoints[(nx + Waypoints.size(0) * 2) +
                    Waypoints.size(0) * 5 * index_waypoint] +
              Waypoints[(nx + Waypoints.size(0) * 4) +
                        Waypoints.size(0) * 5 * index_waypoint],
          V_max[nx + V_max.size(0) * index_waypoint],
          V_min[nx + V_min.size(0) * index_waypoint],
          A_max[nx + A_max.size(0) * index_waypoint],
          A_min[nx + A_min.size(0) * index_waypoint],
          J_max[nx + J_max.size(0) * index_waypoint],
          J_min[nx + J_min.size(0) * index_waypoint])) {
      case 1U:
        printf(
            "Error: Waypoint is infeasible; A_wayp + A_wayp_pred > A_max!\n");
        fflush(stdout);
        break;
      case 2U:
        printf(
            "Error: Waypoint is infeasible; A_wayp + A_wayp_pred < A_min!\n");
        fflush(stdout);
        break;
      case 3U:
        printf(
            "Error: Waypoint is infeasible; V_wayp + V_wayp_pred > V_max!\n");
        fflush(stdout);
        break;
      case 4U:
        printf(
            "Error: Waypoint is infeasible; V_wayp + V_wayp_pred < V_min!\n");
        fflush(stdout);
        break;
      case 5U:
        if (index_waypoint + 1 == num_waypoints) {
          printf("Warning: V_wayp to close to V_max to prevent future "
                 "violation of constraints with A_wayp and J_min (when "
                 "constraints stay the sa"
                 "me)!\n");
          fflush(stdout);
        }
        break;
      case 6U:
        if (index_waypoint + 1 == num_waypoints) {
          printf("Warning: V_wayp to close to V_min to prevent future "
                 "violation of constraints with A_wayp and J_max (when "
                 "constraints stay the sa"
                 "me)!\n");
          fflush(stdout);
        }
        break;
      case 7U:
        printf("Error: Waypoint is infeasible; V_wayp to close to V_min and "
               "A_wayp too large (positive) to be reached with J_max!\n");
        fflush(stdout);
        break;
      case 8U:
        printf("Error: Waypoint is infeasible; V_wayp to close to V_max and "
               "A_wayp too large (negative) to be reached with J_min!\n");
        fflush(stdout);
        break;
      }
    }
  }
  i = Waypoints.size(0);
  for (nx = 0; nx < i; nx++) {
    for (index_waypoint = 0; index_waypoint <= num_waypoints - 2;
         index_waypoint++) {
      if (nx + 1 > Waypoints.size(0)) {
        rtDynamicBoundsError(nx + 1, 1, Waypoints.size(0), &qc_emlrtBCI);
      }
      if (index_waypoint + 1 > Waypoints.size(2)) {
        rtDynamicBoundsError(index_waypoint + 1, 1, Waypoints.size(2),
                             &rc_emlrtBCI);
      }
      if (nx + 1 > Waypoints.size(0)) {
        rtDynamicBoundsError(nx + 1, 1, Waypoints.size(0), &sc_emlrtBCI);
      }
      if (index_waypoint + 1 > Waypoints.size(2)) {
        rtDynamicBoundsError(index_waypoint + 1, 1, Waypoints.size(2),
                             &tc_emlrtBCI);
      }
      if (nx + 1 > Waypoints.size(0)) {
        rtDynamicBoundsError(nx + 1, 1, Waypoints.size(0), &uc_emlrtBCI);
      }
      if (index_waypoint + 1 > Waypoints.size(2)) {
        rtDynamicBoundsError(index_waypoint + 1, 1, Waypoints.size(2),
                             &vc_emlrtBCI);
      }
      if (nx + 1 > Waypoints.size(0)) {
        rtDynamicBoundsError(nx + 1, 1, Waypoints.size(0), &wc_emlrtBCI);
      }
      if (index_waypoint + 1 > Waypoints.size(2)) {
        rtDynamicBoundsError(index_waypoint + 1, 1, Waypoints.size(2),
                             &xc_emlrtBCI);
      }
      if (nx + 1 > V_max.size(0)) {
        rtDynamicBoundsError(nx + 1, 1, V_max.size(0), &yc_emlrtBCI);
      }
      if (index_waypoint + 2 > V_max.size(1)) {
        rtDynamicBoundsError(index_waypoint + 2, 1, V_max.size(1),
                             &ad_emlrtBCI);
      }
      if (nx + 1 > V_min.size(0)) {
        rtDynamicBoundsError(nx + 1, 1, V_min.size(0), &bd_emlrtBCI);
      }
      if (index_waypoint + 2 > V_min.size(1)) {
        rtDynamicBoundsError(index_waypoint + 2, 1, V_min.size(1),
                             &cd_emlrtBCI);
      }
      if (nx + 1 > A_max.size(0)) {
        rtDynamicBoundsError(nx + 1, 1, A_max.size(0), &dd_emlrtBCI);
      }
      if (index_waypoint + 2 > A_max.size(1)) {
        rtDynamicBoundsError(index_waypoint + 2, 1, A_max.size(1),
                             &ed_emlrtBCI);
      }
      if (nx + 1 > A_min.size(0)) {
        rtDynamicBoundsError(nx + 1, 1, A_min.size(0), &fd_emlrtBCI);
      }
      if (index_waypoint + 2 > A_min.size(1)) {
        rtDynamicBoundsError(index_waypoint + 2, 1, A_min.size(1),
                             &gd_emlrtBCI);
      }
      if (nx + 1 > J_max.size(0)) {
        rtDynamicBoundsError(nx + 1, 1, J_max.size(0), &hd_emlrtBCI);
      }
      if (index_waypoint + 2 > J_max.size(1)) {
        rtDynamicBoundsError(index_waypoint + 2, 1, J_max.size(1),
                             &id_emlrtBCI);
      }
      if (nx + 1 > J_min.size(0)) {
        rtDynamicBoundsError(nx + 1, 1, J_min.size(0), &jd_emlrtBCI);
      }
      if (index_waypoint + 2 > J_min.size(1)) {
        rtDynamicBoundsError(index_waypoint + 2, 1, J_min.size(1),
                             &kd_emlrtBCI);
      }
      switch (check_feasibility(
          Waypoints[(nx + Waypoints.size(0)) +
                    Waypoints.size(0) * 5 * index_waypoint] +
              Waypoints[(nx + Waypoints.size(0) * 3) +
                        Waypoints.size(0) * 5 * index_waypoint],
          Waypoints[(nx + Waypoints.size(0) * 2) +
                    Waypoints.size(0) * 5 * index_waypoint] +
              Waypoints[(nx + Waypoints.size(0) * 4) +
                        Waypoints.size(0) * 5 * index_waypoint],
          V_max[nx + V_max.size(0) * (index_waypoint + 1)],
          V_min[nx + V_min.size(0) * (index_waypoint + 1)],
          A_max[nx + A_max.size(0) * (index_waypoint + 1)],
          A_min[nx + A_min.size(0) * (index_waypoint + 1)],
          J_max[nx + J_max.size(0) * (index_waypoint + 1)],
          J_min[nx + J_min.size(0) * (index_waypoint + 1)])) {
      case 1U:
        printf("Warning: Waypoint configuration leads to violation of "
               "constraints; A_wayp + A_wayp_pred > A_max!\n");
        fflush(stdout);
        break;
      case 2U:
        printf("Warning: Waypoint configuration leads to violation of "
               "constraints; A_wayp + A_wayp_pred < A_min!\n");
        fflush(stdout);
        break;
      case 3U:
        printf("Warning: Waypoint configuration leads to violation of "
               "constraints; V_wayp + V_wayp_pred > V_max!\n");
        fflush(stdout);
        break;
      case 4U:
        printf("Warning: Waypoint configuration leads to violation of "
               "constraints; V_wayp + V_wayp_pred < V_min!\n");
        fflush(stdout);
        break;
      case 5U:
        printf("Warning: Waypoint configuration leads to violation of "
               "constraints; V_wayp to close to V_max to prevent future "
               "violation of const"
               "raints with J_min!\n");
        fflush(stdout);
        break;
      case 6U:
        printf("Warning: Waypoint configuration leads to violation of "
               "constraints; V_wayp to close to V_min to prevent future "
               "violation of const"
               "raints with J_max!\n");
        fflush(stdout);
        break;
      case 7U:
        printf("Warning: Waypoint configuration leads to violation of "
               "constraints; V_wayp to close to V_min and A_wayp too large "
               "(positive) to b"
               "e reached with J_max!\n");
        fflush(stdout);
        break;
      case 8U:
        printf("Warning: Waypoint configuration leads to violation of "
               "constraints; V_wayp to close to V_max and A_wayp too large "
               "(negative) to b"
               "e reached with J_min!\n");
        fflush(stdout);
        break;
      }
    }
  }
  if (ts_rollout <= 0.0) {
    printf("Error: ts_rollout <= 0!\n");
    fflush(stdout);
  }
  if (Waypoints.size(0) < 2) {
    s.set_size(b_rotate.size(0), b_rotate.size(1));
    maxdimlen = b_rotate.size(0) * b_rotate.size(1);
    for (i = 0; i < maxdimlen; i++) {
      s[i] = b_rotate[i];
    }
    nx = 0;
    i = s.size(0) * s.size(1);
    for (maxdimlen = 0; maxdimlen < i; maxdimlen++) {
      if (s[maxdimlen]) {
        nx++;
      }
    }
    if (nx > 0) {
      printf("Error: num_axes has to be at least 2 when b_rotate == true!\n");
      fflush(stdout);
    }
  }
}

// End of code generation (check_inputs.cpp)
