//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// solve_T.cpp
//
// Code generation for function 'solve_T'
//

// Include files
#include "solve_T.h"
#include "abc_T_A.h"
#include "abc_T_P.h"
#include "abc_T_V.h"
#include "abcde_T_A.h"
#include "abcde_T_P.h"
#include "abcde_T_V.h"
#include "abcdef_T_P.h"
#include "abcdef_T_V.h"
#include "abcdefg_NTV_AVP.h"
#include "abcdefg_TA_AVP.h"
#include "abcdefg_TV_AVP.h"
#include "abcdefg_T_A.h"
#include "abcdefg_T_AP.h"
#include "abcdefg_T_AV.h"
#include "abcdefg_T_P.h"
#include "abcdefg_T_V.h"
#include "abcdefg_T_VP.h"
#include "abcdeg_NTV_AVP.h"
#include "abcdeg_TV_AVP.h"
#include "abcdeg_T_A.h"
#include "abcdeg_T_AP.h"
#include "abcdeg_T_AV.h"
#include "abcdeg_T_P.h"
#include "abcdeg_T_V.h"
#include "abcef_T_P.h"
#include "abcef_T_V.h"
#include "abcefg_TA_AVP.h"
#include "abcefg_T_A.h"
#include "abcefg_T_AP.h"
#include "abcefg_T_AV.h"
#include "abcefg_T_P.h"
#include "abcefg_T_V.h"
#include "abceg_TA_AVP.h"
#include "abceg_T_A.h"
#include "abceg_T_AP.h"
#include "abceg_T_AV.h"
#include "abceg_T_P.h"
#include "abceg_T_V.h"
#include "abs.h"
#include "ac_T_P.h"
#include "ac_T_V.h"
#include "acde_T_A.h"
#include "acde_T_P.h"
#include "acde_T_V.h"
#include "acdef_T_P.h"
#include "acdef_T_V.h"
#include "acdefg_NTA_AVP.h"
#include "acdefg_NTV_AVP.h"
#include "acdefg_TV_AVP.h"
#include "acdefg_T_A.h"
#include "acdefg_T_AP.h"
#include "acdefg_T_AV.h"
#include "acdefg_T_P.h"
#include "acdefg_T_V.h"
#include "acdeg_TV_AVP.h"
#include "acdeg_T_A.h"
#include "acdeg_T_AP.h"
#include "acdeg_T_AV.h"
#include "acdeg_T_P.h"
#include "acdeg_T_V.h"
#include "acef_T_P.h"
#include "acef_T_V.h"
#include "acefg_TA_AVP.h"
#include "acefg_T_A.h"
#include "acefg_T_AP.h"
#include "acefg_T_AV.h"
#include "acefg_T_P.h"
#include "acefg_T_V.h"
#include "aceg_T_A.h"
#include "aceg_T_AP.h"
#include "aceg_T_AV.h"
#include "aceg_T_P.h"
#include "aceg_T_V.h"
#include "all.h"
#include "check.h"
#include "check_feasibility.h"
#include "construct_setp_struct.h"
#include "evaluate_to_time.h"
#include "indexShapeCheck.h"
#include "mod.h"
#include "printint.h"
#include "rt_nonfinite.h"
#include "select_cases_TA.h"
#include "select_cases_TV.h"
#include "simplify_setp.h"
#include "solve_O.h"
#include "sort.h"
#include "sub2ind.h"
#include "sum.h"
#include "topico_wrapper_data.h"
#include "topico_wrapper_rtwutil.h"
#include "topico_wrapper_types.h"
#include "coder_array.h"
#include "rt_nonfinite.h"
#include <algorithm>
#include <cmath>
#include <cstring>
#include <stdio.h>

// Function Definitions
void solve_T(double P_init, double V_init, double A_init, double P_wayp,
             double V_wayp, double A_wayp, double V_max, double V_min,
             double A_max, double A_min, double J_max, double J_min,
             double t_sync, bool b_sync_V, bool b_sync_A, bool b_sync_J,
             bool b_sync_W, signed char direction, double t_out_data[],
             int t_out_size[2], double J_out_data[], int J_out_size[2],
             int solution_out_data[], int *solution_out_size)
{
  static rtBoundsCheckInfo ac_emlrtBCI = {
      -1,                                             // iFirst
      -1,                                             // iLast
      61,                                             // lineNo
      40,                                             // colNo
      "t_1",                                          // aName
      "solve_T",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/solve_T.m", // pName
      0                                               // checkKind
  };
  static rtBoundsCheckInfo bc_emlrtBCI = {
      -1,                                             // iFirst
      -1,                                             // iLast
      59,                                             // lineNo
      37,                                             // colNo
      "t_2",                                          // aName
      "solve_T",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/solve_T.m", // pName
      0                                               // checkKind
  };
  static rtBoundsCheckInfo cc_emlrtBCI = {
      -1,                                             // iFirst
      -1,                                             // iLast
      59,                                             // lineNo
      21,                                             // colNo
      "t_1",                                          // aName
      "solve_T",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/solve_T.m", // pName
      0                                               // checkKind
  };
  static rtBoundsCheckInfo dc_emlrtBCI = {
      -1,                                             // iFirst
      -1,                                             // iLast
      601,                                            // lineNo
      23,                                             // colNo
      "J_out",                                        // aName
      "solve_T",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/solve_T.m", // pName
      0                                               // checkKind
  };
  static rtBoundsCheckInfo ec_emlrtBCI = {
      -1,                                             // iFirst
      -1,                                             // iLast
      600,                                            // lineNo
      23,                                             // colNo
      "t_out",                                        // aName
      "solve_T",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/solve_T.m", // pName
      0                                               // checkKind
  };
  static rtBoundsCheckInfo nb_emlrtBCI = {
      -1,                                             // iFirst
      -1,                                             // iLast
      602,                                            // lineNo
      37,                                             // colNo
      "solution_out",                                 // aName
      "solve_T",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/solve_T.m", // pName
      0                                               // checkKind
  };
  static rtBoundsCheckInfo ob_emlrtBCI = {
      -1,                                             // iFirst
      -1,                                             // iLast
      138,                                            // lineNo
      26,                                             // colNo
      "cases",                                        // aName
      "solve_T",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/solve_T.m", // pName
      0                                               // checkKind
  };
  static rtBoundsCheckInfo pb_emlrtBCI = {
      -1,                                             // iFirst
      -1,                                             // iLast
      577,                                            // lineNo
      27,                                             // colNo
      "t_out",                                        // aName
      "solve_T",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/solve_T.m", // pName
      0                                               // checkKind
  };
  static rtBoundsCheckInfo qb_emlrtBCI = {
      -1,                                             // iFirst
      -1,                                             // iLast
      563,                                            // lineNo
      48,                                             // colNo
      "t_test",                                       // aName
      "solve_T",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/solve_T.m", // pName
      0                                               // checkKind
  };
  static rtBoundsCheckInfo rb_emlrtBCI = {
      -1,                                             // iFirst
      -1,                                             // iLast
      562,                                            // lineNo
      17,                                             // colNo
      "valid",                                        // aName
      "solve_T",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/solve_T.m", // pName
      0                                               // checkKind
  };
  static rtBoundsCheckInfo sb_emlrtBCI = {
      0,                                                      // iFirst
      31,                                                     // iLast
      272,                                                    // lineNo
      27,                                                     // colNo
      "caselut",                                              // aName
      "select_cases_TJ",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/select_cases_TJ.m", // pName
      0                                                       // checkKind
  };
  static rtBoundsCheckInfo tb_emlrtBCI = {
      0,                                                      // iFirst
      6,                                                      // iLast
      272,                                                    // lineNo
      25,                                                     // colNo
      "caselut",                                              // aName
      "select_cases_TJ",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/select_cases_TJ.m", // pName
      0                                                       // checkKind
  };
  static rtBoundsCheckInfo ub_emlrtBCI = {
      -1,                                             // iFirst
      -1,                                             // iLast
      65,                                             // lineNo
      67,                                             // colNo
      "t_2",                                          // aName
      "solve_T",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/solve_T.m", // pName
      0                                               // checkKind
  };
  static rtBoundsCheckInfo vb_emlrtBCI = {
      -1,                                             // iFirst
      -1,                                             // iLast
      65,                                             // lineNo
      51,                                             // colNo
      "t_1",                                          // aName
      "solve_T",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/solve_T.m", // pName
      0                                               // checkKind
  };
  static rtBoundsCheckInfo wb_emlrtBCI = {
      -1,                                             // iFirst
      -1,                                             // iLast
      64,                                             // lineNo
      41,                                             // colNo
      "J_2",                                          // aName
      "solve_T",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/solve_T.m", // pName
      0                                               // checkKind
  };
  static rtBoundsCheckInfo xb_emlrtBCI = {
      -1,                                             // iFirst
      -1,                                             // iLast
      63,                                             // lineNo
      41,                                             // colNo
      "t_2",                                          // aName
      "solve_T",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/solve_T.m", // pName
      0                                               // checkKind
  };
  static rtBoundsCheckInfo yb_emlrtBCI = {
      -1,                                             // iFirst
      -1,                                             // iLast
      62,                                             // lineNo
      40,                                             // colNo
      "J_1",                                          // aName
      "solve_T",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/solve_T.m", // pName
      0                                               // checkKind
  };
  static rtEqualityCheckInfo c_emlrtECI = {
      -1,                                            // nDims
      592,                                           // lineNo
      22,                                            // colNo
      "solve_T",                                     // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/solve_T.m" // pName
  };
  static rtEqualityCheckInfo d_emlrtECI = {
      2,                                             // nDims
      570,                                           // lineNo
      186,                                           // colNo
      "solve_T",                                     // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/solve_T.m" // pName
  };
  static rtEqualityCheckInfo e_emlrtECI = {
      2,                                             // nDims
      570,                                           // lineNo
      138,                                           // colNo
      "solve_T",                                     // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/solve_T.m" // pName
  };
  static rtEqualityCheckInfo f_emlrtECI = {
      -1,                                            // nDims
      592,                                           // lineNo
      9,                                             // colNo
      "solve_T",                                     // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/solve_T.m" // pName
  };
  coder::array<double, 2U> t17_signals_values;
  coder::array<double, 2U> t17_time;
  creal_T t_test_data[42];
  creal_T dcv2[35];
  creal_T dcv1[28];
  creal_T dcv3[21];
  creal_T dcv[14];
  creal_T b_t1[7];
  double b_t_out_data[1100];
  double J_1_data[700];
  double J_2_data[700];
  double t_1_data[700];
  double t_2_data[700];
  double a__3_data[100];
  double dv[14];
  double J_out[7];
  double J_out_simpl_data[7];
  double J_test[7];
  double J_test_simpl_data[7];
  double t_out_simpl_data[7];
  double t_test_simpl_data[7];
  double t_test_valid[7];
  double varargin_1[7];
  double A_int;
  double V_int;
  double t1;
  int a__1_data[100];
  int b_cases_data[43];
  int cases_data[40];
  int cases_TA_data[24];
  int cases_TV_data[16];
  int J_1_size[2];
  int J_2_size[2];
  int J_out_simpl_size[2];
  int b_t_test_simpl_size[2];
  int t_1_size[2];
  int t_2_size[2];
  int t_test_simpl_size[2];
  int t_test_size[2];
  int a__1_size;
  int i;
  int k;
  int loop_ub;
  int num_valid;
  int t_1_tmp;
  bool b_t_test_simpl_data[7];
  bool valid_data[6];
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
  num_valid = -1;
  t_out_size[0] = 100;
  t_out_size[1] = 11;
  J_out_size[0] = 100;
  J_out_size[1] = 11;
  std::memset(&t_out_data[0], 0, 1100U * sizeof(double));
  std::memset(&J_out_data[0], 0, 1100U * sizeof(double));
  std::memset(&solution_out_data[0], 0, 100U * sizeof(int));
  guard1 = false;
  if (b_sync_W && (!rtIsNaN(P_wayp)) && (!rtIsNaN(V_wayp)) &&
      (!rtIsNaN(A_wayp))) {
    c_solve_O(P_init, V_init, A_init, V_max, V_min, A_max, A_min, J_max, J_min,
              t_1_data, t_1_size, J_1_data, J_1_size, a__1_data, &a__1_size);
    b_construct_setp_struct(t_1_data, t_1_size, J_1_data, J_1_size, t17_time,
                            t17_signals_values);
    evaluate_to_time(P_init, V_init, A_init, t17_time, t17_signals_values, &t1,
                     &V_int, &A_int);
    solve_O(t1, V_int, A_int, P_wayp, V_wayp, A_wayp, V_max, V_min, A_max,
            A_min, J_max, J_min, t_2_data, t_2_size, J_2_data, J_2_size,
            a__1_data, &a__1_size);
    coder::internal::indexShapeCheck(t_1_size);
    coder::internal::indexShapeCheck(t_2_size);
    t_1_tmp = t_1_size[0] * 7;
    if (1 > t_1_tmp) {
      rtDynamicBoundsError(1, 1, t_1_tmp, &cc_emlrtBCI);
    }
    if (2 > t_1_tmp) {
      rtDynamicBoundsError(2, 1, 1, &cc_emlrtBCI);
    }
    if (3 > t_1_tmp) {
      rtDynamicBoundsError(3, 1, 2, &cc_emlrtBCI);
    }
    a__1_size = t_2_size[0] * 7;
    for (i = 0; i < 7; i++) {
      if (i + 1 > a__1_size) {
        rtDynamicBoundsError(i + 1, 1, a__1_size, &bc_emlrtBCI);
      }
    }
    t1 = t_2_data[0];
    for (k = 0; k < 6; k++) {
      t1 += t_2_data[k + 1];
    }
    V_int = (t_1_data[0] + t_1_data[1]) + t_1_data[2];
    if (V_int + t1 < t_sync) {
      coder::internal::indexShapeCheck(t_1_size);
      if (1 > t_1_tmp) {
        rtDynamicBoundsError(1, 1, t_1_tmp, &ac_emlrtBCI);
      }
      t_out_data[0] = t_1_data[0];
      if (2 > t_1_tmp) {
        rtDynamicBoundsError(2, 1, 1, &ac_emlrtBCI);
      }
      t_out_data[100] = t_1_data[1];
      if (3 > t_1_tmp) {
        rtDynamicBoundsError(3, 1, 2, &ac_emlrtBCI);
      }
      t_out_data[200] = t_1_data[2];
      coder::internal::indexShapeCheck(J_1_size);
      k = J_1_size[0] * 7;
      if (1 > k) {
        rtDynamicBoundsError(1, 1, k, &yb_emlrtBCI);
      }
      J_out_data[0] = J_1_data[0];
      if (2 > k) {
        rtDynamicBoundsError(2, 1, 1, &yb_emlrtBCI);
      }
      J_out_data[100] = J_1_data[1];
      if (3 > k) {
        rtDynamicBoundsError(3, 1, 2, &yb_emlrtBCI);
      }
      J_out_data[200] = J_1_data[2];
      coder::internal::indexShapeCheck(t_2_size);
      for (i = 0; i < 7; i++) {
        if (i + 1 > a__1_size) {
          rtDynamicBoundsError(i + 1, 1, a__1_size, &xb_emlrtBCI);
        }
        t_out_data[100 * (i + 4)] = t_2_data[i];
      }
      coder::internal::indexShapeCheck(J_2_size);
      k = J_2_size[0] * 7;
      for (i = 0; i < 7; i++) {
        if (i + 1 > k) {
          rtDynamicBoundsError(i + 1, 1, k, &wb_emlrtBCI);
        }
        J_out_data[100 * (i + 4)] = J_2_data[i];
      }
      coder::internal::indexShapeCheck(t_1_size);
      coder::internal::indexShapeCheck(t_2_size);
      if (1 > t_1_tmp) {
        rtDynamicBoundsError(1, 1, t_1_tmp, &vb_emlrtBCI);
      }
      if (2 > t_1_tmp) {
        rtDynamicBoundsError(2, 1, 1, &vb_emlrtBCI);
      }
      if (3 > t_1_tmp) {
        rtDynamicBoundsError(3, 1, 2, &vb_emlrtBCI);
      }
      for (i = 0; i < 7; i++) {
        if (i + 1 > a__1_size) {
          rtDynamicBoundsError(i + 1, 1, a__1_size, &ub_emlrtBCI);
        }
      }
      t1 = t_2_data[0];
      for (k = 0; k < 6; k++) {
        t1 += t_2_data[k + 1];
      }
      t_out_data[300] = (t_sync - V_int) - t1;
      J_out_data[300] = 0.0;
      *solution_out_size = 1;
      solution_out_data[0] = 11111;
    } else {
      guard1 = true;
    }
  } else {
    guard1 = true;
  }
  if (guard1) {
    int b_loop_ub;
    int cases_size_idx_1;
    int i1;
    signed char input_sizes_idx_1;
    bool empty_non_axis_sizes;
    k = 0;
    cases_size_idx_1 = 0;
    empty_non_axis_sizes = rtIsNaN(P_wayp);
    if ((!empty_non_axis_sizes) && (!rtIsNaN(V_wayp)) && (!rtIsNaN(A_wayp))) {
      signed char b_input_sizes_idx_1;
      signed char sizes_idx_0;
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
      t_1_tmp =
          coder::eml_sub2ind(static_cast<double>(!rtIsNaN(P_wayp)) + 1.0,
                             static_cast<double>(!rtIsNaN(V_wayp)) + 1.0,
                             static_cast<double>(!rtIsNaN(A_wayp)) + 1.0) -
          1;
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
      switch (check_feasibility(V_init, A_init, V_max, V_min, A_max, A_min,
                                J_max, J_min)) {
      case 3U:
        k = 2;
        break;
      case 5U:
        k = 2;
        break;
      case 4U:
        k = 3;
        break;
      case 6U:
        k = 3;
        break;
      default:
        k = 1;
        break;
      }
      k = coder::eml_sub2ind(static_cast<double>(k),
                             static_cast<double>(!rtIsInf(V_max)) + 1.0,
                             static_cast<double>(!rtIsInf(V_min)) + 1.0,
                             static_cast<double>(!rtIsInf(A_max)) + 1.0,
                             static_cast<double>(!rtIsInf(A_min)) + 1.0) -
          1;
      select_cases_TV(nlut[t_1_tmp], condlut[k], b_sync_V, cases_TV_data,
                      t_1_size);
      select_cases_TA(nlut[t_1_tmp], condlut[k], b_sync_A, cases_TA_data,
                      J_1_size);
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
      if (b_sync_J) {
        i = nlut[t_1_tmp] - 1;
        if ((i < 0) || (i > 6)) {
          rtDynamicBoundsError(i, 0, 6, &tb_emlrtBCI);
        }
        i = condlut[k] - 1;
        if ((i < 0) || (i > 31)) {
          rtDynamicBoundsError(i, 0, 31, &sb_emlrtBCI);
        }
      }
      if ((t_1_size[0] != 0) && (t_1_size[1] != 0)) {
        sizes_idx_0 = 1;
      } else if ((J_1_size[0] != 0) && (J_1_size[1] != 0)) {
        sizes_idx_0 = 1;
      } else {
        sizes_idx_0 = 0;
      }
      if ((t_1_size[0] != sizes_idx_0) &&
          ((t_1_size[0] != 0) && (t_1_size[1] != 0))) {
        h_rtErrorWithMessageID(i_emlrtRTEI.fName, i_emlrtRTEI.lineNo);
      }
      if ((J_1_size[0] != sizes_idx_0) &&
          ((J_1_size[0] != 0) && (J_1_size[1] != 0))) {
        h_rtErrorWithMessageID(i_emlrtRTEI.fName, i_emlrtRTEI.lineNo);
      }
      empty_non_axis_sizes = (sizes_idx_0 == 0);
      if (empty_non_axis_sizes || ((t_1_size[0] != 0) && (t_1_size[1] != 0))) {
        input_sizes_idx_1 = static_cast<signed char>(t_1_size[1]);
      } else {
        input_sizes_idx_1 = 0;
      }
      if (empty_non_axis_sizes || ((J_1_size[0] != 0) && (J_1_size[1] != 0))) {
        b_input_sizes_idx_1 = static_cast<signed char>(J_1_size[1]);
      } else {
        b_input_sizes_idx_1 = 0;
      }
      k = sizes_idx_0;
      cases_size_idx_1 = input_sizes_idx_1 + b_input_sizes_idx_1;
      loop_ub = input_sizes_idx_1;
      for (i = 0; i < loop_ub; i++) {
        b_loop_ub = sizes_idx_0;
        if (0 <= b_loop_ub - 1) {
          cases_data[sizes_idx_0 * i] = cases_TV_data[t_1_size[0] * i];
        }
      }
      loop_ub = b_input_sizes_idx_1;
      for (i = 0; i < loop_ub; i++) {
        b_loop_ub = sizes_idx_0;
        if (0 <= b_loop_ub - 1) {
          cases_data[sizes_idx_0 * (i + input_sizes_idx_1)] =
              cases_TA_data[J_1_size[0] * i];
        }
      }
    } else if ((!empty_non_axis_sizes) && (!rtIsNaN(V_wayp)) &&
               rtIsNaN(A_wayp)) {
      if (direction == 1) {
        k = 1;
        cases_size_idx_1 = 1;
        cases_data[0] = 40202;
      } else if (direction == -1) {
        k = 1;
        cases_size_idx_1 = 1;
        cases_data[0] = 40201;
      } else {
        printf("Error: Direction not -1 or 1 and free DoF!\n");
        fflush(stdout);
      }
    } else if ((!empty_non_axis_sizes) && rtIsNaN(V_wayp) &&
               (!rtIsNaN(A_wayp))) {
      if (direction == 1) {
        k = 1;
        cases_size_idx_1 = 8;
        for (i = 0; i < 8; i++) {
          cases_data[i] = (i << 1) + 40302;
        }
      } else if (direction == -1) {
        k = 1;
        cases_size_idx_1 = 8;
        for (i = 0; i < 8; i++) {
          cases_data[i] = (i << 1) + 40301;
        }
      } else {
        printf("Error: Direction not -1 or 1 and free DoF!\n");
        fflush(stdout);
      }
    } else if ((!empty_non_axis_sizes) && rtIsNaN(V_wayp) && rtIsNaN(A_wayp)) {
      if (direction == 1) {
        k = 1;
        cases_size_idx_1 = 16;
        for (i = 0; i < 16; i++) {
          cases_data[i] = (i << 1) + 40702;
        }
      } else if (direction == -1) {
        k = 1;
        cases_size_idx_1 = 16;
        for (i = 0; i < 16; i++) {
          cases_data[i] = (i << 1) + 40701;
        }
      } else {
        printf("Error: Direction not -1 or 1 and free DoF!\n");
        fflush(stdout);
      }
    } else if (rtIsNaN(P_wayp) && (!rtIsNaN(V_wayp)) && (!rtIsNaN(A_wayp))) {
      if (direction == 1) {
        k = 1;
        cases_size_idx_1 = 8;
        for (i = 0; i < 8; i++) {
          cases_data[i] = (i << 1) + 40402;
        }
      } else if (direction == -1) {
        k = 1;
        cases_size_idx_1 = 8;
        for (i = 0; i < 8; i++) {
          cases_data[i] = (i << 1) + 40401;
        }
      } else {
        printf("Error: Direction not -1 or 1 and free DoF!\n");
        fflush(stdout);
      }
    } else if (rtIsNaN(P_wayp) && (!rtIsNaN(V_wayp)) && rtIsNaN(A_wayp)) {
      if (direction == 1) {
        k = 1;
        cases_size_idx_1 = 16;
        for (i = 0; i < 16; i++) {
          cases_data[i] = (i << 1) + 40602;
        }
      } else if (direction == -1) {
        k = 1;
        cases_size_idx_1 = 16;
        for (i = 0; i < 16; i++) {
          cases_data[i] = (i << 1) + 40601;
        }
      } else {
        printf("Error: Direction not -1 or 1 and free DoF!\n");
        fflush(stdout);
      }
    } else if (rtIsNaN(P_wayp) && rtIsNaN(V_wayp) && (!rtIsNaN(A_wayp))) {
      if (direction == 1) {
        k = 1;
        cases_size_idx_1 = 12;
        for (i = 0; i < 12; i++) {
          cases_data[i] = (i << 1) + 40502;
        }
      } else if (direction == -1) {
        k = 1;
        cases_size_idx_1 = 12;
        for (i = 0; i < 12; i++) {
          cases_data[i] = (i << 1) + 40501;
        }
      } else {
        printf("Error: Direction not -1 or 1 and free DoF!\n");
        fflush(stdout);
      }
    } else {
      printf("Error: P_wayp and V_wayp and A_wayp = NaN!\n");
      fflush(stdout);
    }
    if ((k != 0) && (cases_size_idx_1 != 0)) {
      input_sizes_idx_1 = static_cast<signed char>(cases_size_idx_1);
    } else {
      input_sizes_idx_1 = 0;
    }
    cases_size_idx_1 = input_sizes_idx_1 + 3;
    loop_ub = input_sizes_idx_1;
    if (0 <= loop_ub - 1) {
      std::copy(&cases_data[0], &cases_data[loop_ub], &b_cases_data[0]);
    }
    b_cases_data[input_sizes_idx_1] = 0;
    b_cases_data[input_sizes_idx_1 + 1] = 21109;
    b_cases_data[input_sizes_idx_1 + 2] = 21110;
    // TODO Put these into appropriate places. They are sometimes necessary to
    // find a solution!
    for (a__1_size = 0; a__1_size < cases_size_idx_1; a__1_size++) {
      if (a__1_size + 1 > input_sizes_idx_1 + 3) {
        rtDynamicBoundsError(a__1_size + 1, 1, input_sizes_idx_1 + 3,
                             &ob_emlrtBCI);
      }
      i = b_cases_data[a__1_size];
      if (coder::b_mod(std::floor(static_cast<double>(i) / 1000.0)) == 0.0) {
        if (i - ((i >> 1) << 1) == 1) {
          J_test[0] = J_max;
          J_test[1] = 0.0;
          J_test[2] = J_min;
          J_test[3] = 0.0;
          J_test[4] = J_min;
          J_test[5] = 0.0;
          J_test[6] = J_max;
        } else {
          J_test[0] = J_min;
          J_test[1] = 0.0;
          J_test[2] = J_max;
          J_test[3] = 0.0;
          J_test[4] = J_max;
          J_test[5] = 0.0;
          J_test[6] = J_min;
        }
      } else if (i - ((i >> 1) << 1) == 1) {
        J_test[0] = J_min;
        J_test[1] = 0.0;
        J_test[2] = J_max;
        J_test[3] = 0.0;
        J_test[4] = J_min;
        J_test[5] = 0.0;
        J_test[6] = J_max;
      } else {
        J_test[0] = J_max;
        J_test[1] = 0.0;
        J_test[2] = J_min;
        J_test[3] = 0.0;
        J_test[4] = J_max;
        J_test[5] = 0.0;
        J_test[6] = J_min;
      }
      switch (i) {
      case 0:
        t_test_size[0] = 1;
        t_test_size[1] = 7;
        std::memset(&t_test_data[0], 0, 7U * sizeof(creal_T));
        t_test_data[3].re = t_sync;
        t_test_data[3].im = 0.0;
        break;
      case 10101:
        abcdefg_TV_AVP(P_init, V_init, A_init, P_wayp, V_wayp, A_wayp, A_max,
                       A_min, J_max, J_min, t_sync, dcv);
        t_test_size[0] = 2;
        t_test_size[1] = 7;
        std::copy(&dcv[0], &dcv[14], &t_test_data[0]);
        break;
      case 10102:
        abcdefg_TV_AVP(-P_init, -V_init, -A_init, -P_wayp, -V_wayp, -A_wayp,
                       -A_min, -A_max, -J_min, -J_max, t_sync, dcv);
        t_test_size[0] = 2;
        t_test_size[1] = 7;
        std::copy(&dcv[0], &dcv[14], &t_test_data[0]);
        break;
      case 10103:
        abcdeg_TV_AVP(P_init, V_init, A_init, P_wayp, V_wayp, A_wayp, A_max,
                      J_max, J_min, t_sync, dcv1);
        t_test_size[0] = 4;
        t_test_size[1] = 7;
        std::copy(&dcv1[0], &dcv1[28], &t_test_data[0]);
        break;
      case 10104:
        abcdeg_TV_AVP(-P_init, -V_init, -A_init, -P_wayp, -V_wayp, -A_wayp,
                      -A_min, -J_min, -J_max, t_sync, dcv1);
        t_test_size[0] = 4;
        t_test_size[1] = 7;
        std::copy(&dcv1[0], &dcv1[28], &t_test_data[0]);
        break;
      case 10105:
        acdefg_TV_AVP(P_init, V_init, A_init, P_wayp, V_wayp, A_wayp, A_min,
                      J_max, J_min, t_sync, dcv1);
        t_test_size[0] = 4;
        t_test_size[1] = 7;
        std::copy(&dcv1[0], &dcv1[28], &t_test_data[0]);
        break;
      case 10106:
        acdefg_TV_AVP(-P_init, -V_init, -A_init, -P_wayp, -V_wayp, -A_wayp,
                      -A_max, -J_min, -J_max, t_sync, dcv1);
        t_test_size[0] = 4;
        t_test_size[1] = 7;
        std::copy(&dcv1[0], &dcv1[28], &t_test_data[0]);
        break;
      case 10107:
        acdeg_TV_AVP(P_init, V_init, A_init, P_wayp, V_wayp, A_wayp, J_max,
                     J_min, t_sync, dcv2);
        t_test_size[0] = 5;
        t_test_size[1] = 7;
        std::copy(&dcv2[0], &dcv2[35], &t_test_data[0]);
        break;
      case 10108:
        acdeg_TV_AVP(-P_init, -V_init, -A_init, -P_wayp, -V_wayp, -A_wayp,
                     -J_min, -J_max, t_sync, dcv2);
        t_test_size[0] = 5;
        t_test_size[1] = 7;
        std::copy(&dcv2[0], &dcv2[35], &t_test_data[0]);
        // -----------------------
        break;
      case 20101:
        abcdefg_TA_AVP(P_init, V_init, A_init, P_wayp, V_wayp, A_wayp, V_max,
                       A_max, A_min, J_max, J_min, t_sync, dcv1);
        t_test_size[0] = 4;
        t_test_size[1] = 7;
        std::copy(&dcv1[0], &dcv1[28], &t_test_data[0]);
        break;
      case 20102:
        abcdefg_TA_AVP(-P_init, -V_init, -A_init, -P_wayp, -V_wayp, -A_wayp,
                       -V_min, -A_min, -A_max, -J_min, -J_max, t_sync, dcv1);
        t_test_size[0] = 4;
        t_test_size[1] = 7;
        std::copy(&dcv1[0], &dcv1[28], &t_test_data[0]);
        break;
      case 20103:
        t_test_size[0] = 1;
        t_test_size[1] = 7;
        std::memset(&t_test_data[0], 0, 7U * sizeof(creal_T));
        break;
      case 20104:
        t_test_size[0] = 1;
        t_test_size[1] = 7;
        std::memset(&t_test_data[0], 0, 7U * sizeof(creal_T));
        break;
      case 20105:
        abcefg_TA_AVP(P_init, V_init, A_init, P_wayp, V_wayp, A_wayp, A_max,
                      A_min, J_max, J_min, t_sync, dcv1);
        t_test_size[0] = 4;
        t_test_size[1] = 7;
        std::copy(&dcv1[0], &dcv1[28], &t_test_data[0]);
        break;
      case 20106:
        abcefg_TA_AVP(-P_init, -V_init, -A_init, -P_wayp, -V_wayp, -A_wayp,
                      -A_min, -A_max, -J_min, -J_max, t_sync, dcv1);
        t_test_size[0] = 4;
        t_test_size[1] = 7;
        std::copy(&dcv1[0], &dcv1[28], &t_test_data[0]);
        break;
      case 20107:
        abceg_TA_AVP(P_init, V_init, A_init, P_wayp, V_wayp, A_wayp, J_max,
                     J_min, t_sync, dcv1);
        t_test_size[0] = 4;
        t_test_size[1] = 7;
        std::copy(&dcv1[0], &dcv1[28], &t_test_data[0]);
        break;
      case 20108:
        abceg_TA_AVP(-P_init, -V_init, -A_init, -P_wayp, -V_wayp, -A_wayp,
                     -J_min, -J_max, t_sync, dcv1);
        t_test_size[0] = 4;
        t_test_size[1] = 7;
        std::copy(&dcv1[0], &dcv1[28], &t_test_data[0]);
        break;
      case 20109:
        t_test_size[0] = 1;
        t_test_size[1] = 7;
        std::memset(&t_test_data[0], 0, 7U * sizeof(creal_T));
        break;
      case 20110:
        t_test_size[0] = 1;
        t_test_size[1] = 7;
        std::memset(&t_test_data[0], 0, 7U * sizeof(creal_T));
        break;
      case 20111:
        acefg_TA_AVP(P_init, V_init, A_init, P_wayp, V_wayp, A_wayp, J_max,
                     J_min, t_sync, dcv1);
        t_test_size[0] = 4;
        t_test_size[1] = 7;
        std::copy(&dcv1[0], &dcv1[28], &t_test_data[0]);
        break;
      case 20112:
        acefg_TA_AVP(-P_init, -V_init, -A_init, -P_wayp, -V_wayp, -A_wayp,
                     -J_min, -J_max, t_sync, dcv1);
        t_test_size[0] = 4;
        t_test_size[1] = 7;
        std::copy(&dcv1[0], &dcv1[28], &t_test_data[0]);
        // -----------------------
        break;
      case 30101:
        t_test_size[0] = 1;
        t_test_size[1] = 7;
        std::memset(&t_test_data[0], 0, 7U * sizeof(creal_T));
        break;
      case 30102:
        t_test_size[0] = 1;
        t_test_size[1] = 7;
        std::memset(&t_test_data[0], 0, 7U * sizeof(creal_T));
        break;
      case 30103:
        t_test_size[0] = 1;
        t_test_size[1] = 7;
        std::memset(&t_test_data[0], 0, 7U * sizeof(creal_T));
        break;
      case 30104:
        t_test_size[0] = 1;
        t_test_size[1] = 7;
        std::memset(&t_test_data[0], 0, 7U * sizeof(creal_T));
        break;
      case 30105:
        t_test_size[0] = 1;
        t_test_size[1] = 7;
        std::memset(&t_test_data[0], 0, 7U * sizeof(creal_T));
        break;
      case 30106:
        t_test_size[0] = 1;
        t_test_size[1] = 7;
        std::memset(&t_test_data[0], 0, 7U * sizeof(creal_T));
        break;
      case 30107:
        t_test_size[0] = 1;
        t_test_size[1] = 7;
        std::memset(&t_test_data[0], 0, 7U * sizeof(creal_T));
        break;
      case 30108:
        t_test_size[0] = 1;
        t_test_size[1] = 7;
        std::memset(&t_test_data[0], 0, 7U * sizeof(creal_T));
        break;
      case 30109:
        t_test_size[0] = 1;
        t_test_size[1] = 7;
        std::memset(&t_test_data[0], 0, 7U * sizeof(creal_T));
        break;
      case 30110:
        t_test_size[0] = 1;
        t_test_size[1] = 7;
        std::memset(&t_test_data[0], 0, 7U * sizeof(creal_T));
        break;
      case 30111:
        t_test_size[0] = 1;
        t_test_size[1] = 7;
        std::memset(&t_test_data[0], 0, 7U * sizeof(creal_T));
        break;
      case 30112:
        t_test_size[0] = 1;
        t_test_size[1] = 7;
        std::memset(&t_test_data[0], 0, 7U * sizeof(creal_T));
        break;
      case 30113:
        t_test_size[0] = 1;
        t_test_size[1] = 7;
        std::memset(&t_test_data[0], 0, 7U * sizeof(creal_T));
        break;
      case 30114:
        t_test_size[0] = 1;
        t_test_size[1] = 7;
        std::memset(&t_test_data[0], 0, 7U * sizeof(creal_T));
        break;
      case 30115:
        t_test_size[0] = 1;
        t_test_size[1] = 7;
        std::memset(&t_test_data[0], 0, 7U * sizeof(creal_T));
        break;
      case 30116:
        t_test_size[0] = 1;
        t_test_size[1] = 7;
        std::memset(&t_test_data[0], 0, 7U * sizeof(creal_T));
        // -----------------------
        break;
      case 11101:
        abcdefg_NTV_AVP(P_init, V_init, A_init, P_wayp, V_wayp, A_wayp, A_min,
                        J_max, J_min, t_sync, varargin_1);
        t_test_size[0] = 1;
        t_test_size[1] = 7;
        for (i = 0; i < 7; i++) {
          t_test_data[i].re = varargin_1[i];
          t_test_data[i].im = 0.0;
        }
        break;
      case 11102:
        abcdefg_NTV_AVP(-P_init, -V_init, -A_init, -P_wayp, -V_wayp, -A_wayp,
                        -A_max, -J_min, -J_max, t_sync, varargin_1);
        t_test_size[0] = 1;
        t_test_size[1] = 7;
        for (i = 0; i < 7; i++) {
          t_test_data[i].re = varargin_1[i];
          t_test_data[i].im = 0.0;
        }
        break;
      case 11103:
        abcdeg_NTV_AVP(P_init, V_init, A_init, P_wayp, V_wayp, A_wayp, A_min,
                       J_max, J_min, t_sync, dcv1);
        t_test_size[0] = 4;
        t_test_size[1] = 7;
        std::copy(&dcv1[0], &dcv1[28], &t_test_data[0]);
        break;
      case 11104:
        abcdeg_NTV_AVP(-P_init, -V_init, -A_init, -P_wayp, -V_wayp, -A_wayp,
                       -A_max, -J_min, -J_max, t_sync, dcv1);
        t_test_size[0] = 4;
        t_test_size[1] = 7;
        std::copy(&dcv1[0], &dcv1[28], &t_test_data[0]);
        break;
      case 11105:
        acdefg_NTV_AVP(P_init, V_init, A_init, P_wayp, V_wayp, A_wayp, A_min,
                       J_max, J_min, t_sync, dcv1);
        t_test_size[0] = 4;
        t_test_size[1] = 7;
        std::copy(&dcv1[0], &dcv1[28], &t_test_data[0]);
        break;
      case 11106:
        acdefg_NTV_AVP(-P_init, -V_init, -A_init, -P_wayp, -V_wayp, -A_wayp,
                       -A_max, -J_min, -J_max, t_sync, dcv1);
        t_test_size[0] = 4;
        t_test_size[1] = 7;
        std::copy(&dcv1[0], &dcv1[28], &t_test_data[0]);
        break;
      case 11107:
        t_test_size[0] = 1;
        t_test_size[1] = 7;
        std::memset(&t_test_data[0], 0, 7U * sizeof(creal_T));
        break;
      case 11108:
        t_test_size[0] = 1;
        t_test_size[1] = 7;
        std::memset(&t_test_data[0], 0, 7U * sizeof(creal_T));
        // -----------------------
        break;
      case 21101:
        t_test_size[0] = 1;
        t_test_size[1] = 7;
        std::memset(&t_test_data[0], 0, 7U * sizeof(creal_T));
        break;
      case 21102:
        t_test_size[0] = 1;
        t_test_size[1] = 7;
        std::memset(&t_test_data[0], 0, 7U * sizeof(creal_T));
        break;
      case 21103:
        t_test_size[0] = 1;
        t_test_size[1] = 7;
        std::memset(&t_test_data[0], 0, 7U * sizeof(creal_T));
        break;
      case 21104:
        t_test_size[0] = 1;
        t_test_size[1] = 7;
        std::memset(&t_test_data[0], 0, 7U * sizeof(creal_T));
        break;
      case 21105:
        t_test_size[0] = 1;
        t_test_size[1] = 7;
        std::memset(&t_test_data[0], 0, 7U * sizeof(creal_T));
        break;
      case 21106:
        t_test_size[0] = 1;
        t_test_size[1] = 7;
        std::memset(&t_test_data[0], 0, 7U * sizeof(creal_T));
        break;
      case 21107:
        t_test_size[0] = 1;
        t_test_size[1] = 7;
        std::memset(&t_test_data[0], 0, 7U * sizeof(creal_T));
        break;
      case 21108:
        t_test_size[0] = 1;
        t_test_size[1] = 7;
        std::memset(&t_test_data[0], 0, 7U * sizeof(creal_T));
        break;
      case 21109:
        acdefg_NTA_AVP(P_init, V_init, A_init, P_wayp, V_wayp, A_wayp, V_max,
                       J_max, J_min, t_sync, dcv1);
        t_test_size[0] = 4;
        t_test_size[1] = 7;
        std::copy(&dcv1[0], &dcv1[28], &t_test_data[0]);
        break;
      case 21110:
        acdefg_NTA_AVP(-P_init, -V_init, -A_init, -P_wayp, -V_wayp, -A_wayp,
                       -V_min, -J_min, -J_max, t_sync, dcv1);
        t_test_size[0] = 4;
        t_test_size[1] = 7;
        std::copy(&dcv1[0], &dcv1[28], &t_test_data[0]);
        break;
      case 21111:
        t_test_size[0] = 1;
        t_test_size[1] = 7;
        std::memset(&t_test_data[0], 0, 7U * sizeof(creal_T));
        break;
      case 21112:
        t_test_size[0] = 1;
        t_test_size[1] = 7;
        std::memset(&t_test_data[0], 0, 7U * sizeof(creal_T));
        // -----------------------
        break;
      case 31101:
        t_test_size[0] = 1;
        t_test_size[1] = 7;
        std::memset(&t_test_data[0], 0, 7U * sizeof(creal_T));
        break;
      case 31102:
        t_test_size[0] = 1;
        t_test_size[1] = 7;
        std::memset(&t_test_data[0], 0, 7U * sizeof(creal_T));
        break;
      case 31103:
        t_test_size[0] = 1;
        t_test_size[1] = 7;
        std::memset(&t_test_data[0], 0, 7U * sizeof(creal_T));
        break;
      case 31104:
        t_test_size[0] = 1;
        t_test_size[1] = 7;
        std::memset(&t_test_data[0], 0, 7U * sizeof(creal_T));
        break;
      case 31105:
        t_test_size[0] = 1;
        t_test_size[1] = 7;
        std::memset(&t_test_data[0], 0, 7U * sizeof(creal_T));
        break;
      case 31106:
        t_test_size[0] = 1;
        t_test_size[1] = 7;
        std::memset(&t_test_data[0], 0, 7U * sizeof(creal_T));
        break;
      case 31107:
        t_test_size[0] = 1;
        t_test_size[1] = 7;
        std::memset(&t_test_data[0], 0, 7U * sizeof(creal_T));
        break;
      case 31108:
        t_test_size[0] = 1;
        t_test_size[1] = 7;
        std::memset(&t_test_data[0], 0, 7U * sizeof(creal_T));
        break;
      case 31109:
        t_test_size[0] = 1;
        t_test_size[1] = 7;
        std::memset(&t_test_data[0], 0, 7U * sizeof(creal_T));
        break;
      case 31110:
        t_test_size[0] = 1;
        t_test_size[1] = 7;
        std::memset(&t_test_data[0], 0, 7U * sizeof(creal_T));
        break;
      case 31111:
        t_test_size[0] = 1;
        t_test_size[1] = 7;
        std::memset(&t_test_data[0], 0, 7U * sizeof(creal_T));
        break;
      case 31112:
        t_test_size[0] = 1;
        t_test_size[1] = 7;
        std::memset(&t_test_data[0], 0, 7U * sizeof(creal_T));
        break;
      case 31113:
        t_test_size[0] = 1;
        t_test_size[1] = 7;
        std::memset(&t_test_data[0], 0, 7U * sizeof(creal_T));
        break;
      case 31114:
        t_test_size[0] = 1;
        t_test_size[1] = 7;
        std::memset(&t_test_data[0], 0, 7U * sizeof(creal_T));
        break;
      case 31115:
        t_test_size[0] = 1;
        t_test_size[1] = 7;
        std::memset(&t_test_data[0], 0, 7U * sizeof(creal_T));
        break;
      case 31116:
        t_test_size[0] = 1;
        t_test_size[1] = 7;
        std::memset(&t_test_data[0], 0, 7U * sizeof(creal_T));
        // -----------------------
        break;
      case 40201:
        abcdefg_T_VP(P_init, V_init, A_init, P_wayp, V_wayp, V_max, A_max,
                     A_min, J_max, J_min, t_sync, dcv1);
        t_test_size[0] = 4;
        t_test_size[1] = 7;
        std::copy(&dcv1[0], &dcv1[28], &t_test_data[0]);
        break;
      case 40202:
        abcdefg_T_VP(-P_init, -V_init, -A_init, -P_wayp, -V_wayp, -V_min,
                     -A_min, -A_max, -J_min, -J_max, t_sync, dcv1);
        t_test_size[0] = 4;
        t_test_size[1] = 7;
        std::copy(&dcv1[0], &dcv1[28], &t_test_data[0]);
        // -----------------------
        break;
      case 40301:
        abcdefg_T_AP(P_init, V_init, A_init, P_wayp, A_wayp, V_max, A_max,
                     A_min, J_max, J_min, t_sync, dcv);
        t_test_size[0] = 2;
        t_test_size[1] = 7;
        std::copy(&dcv[0], &dcv[14], &t_test_data[0]);
        break;
      case 40302:
        abcdefg_T_AP(-P_init, -V_init, -A_init, -P_wayp, -A_wayp, -V_min,
                     -A_min, -A_max, -J_min, -J_max, t_sync, dcv);
        t_test_size[0] = 2;
        t_test_size[1] = 7;
        std::copy(&dcv[0], &dcv[14], &t_test_data[0]);
        break;
      case 40303:
        abcefg_T_AP(P_init, V_init, A_init, P_wayp, A_wayp, A_max, A_min, J_max,
                    J_min, t_sync, dcv);
        t_test_size[0] = 2;
        t_test_size[1] = 7;
        std::copy(&dcv[0], &dcv[14], &t_test_data[0]);
        break;
      case 40304:
        abcefg_T_AP(-P_init, -V_init, -A_init, -P_wayp, -A_wayp, -A_min, -A_max,
                    -J_min, -J_max, t_sync, dcv);
        t_test_size[0] = 2;
        t_test_size[1] = 7;
        std::copy(&dcv[0], &dcv[14], &t_test_data[0]);
        break;
      case 40305:
        acefg_T_AP(P_init, V_init, A_init, P_wayp, A_wayp, A_min, J_max, J_min,
                   t_sync, dcv3);
        t_test_size[0] = 3;
        t_test_size[1] = 7;
        std::copy(&dcv3[0], &dcv3[21], &t_test_data[0]);
        break;
      case 40306:
        acefg_T_AP(-P_init, -V_init, -A_init, -P_wayp, -A_wayp, -A_max, -J_min,
                   -J_max, t_sync, dcv3);
        t_test_size[0] = 3;
        t_test_size[1] = 7;
        std::copy(&dcv3[0], &dcv3[21], &t_test_data[0]);
        break;
      case 40307:
        abceg_T_AP(P_init, V_init, A_init, P_wayp, A_wayp, A_max, J_max, J_min,
                   t_sync, dcv3);
        t_test_size[0] = 3;
        t_test_size[1] = 7;
        std::copy(&dcv3[0], &dcv3[21], &t_test_data[0]);
        break;
      case 40308:
        abceg_T_AP(-P_init, -V_init, -A_init, -P_wayp, -A_wayp, -A_min, -J_min,
                   -J_max, t_sync, dcv3);
        t_test_size[0] = 3;
        t_test_size[1] = 7;
        std::copy(&dcv3[0], &dcv3[21], &t_test_data[0]);
        break;
      case 40309:
        aceg_T_AP(P_init, V_init, A_init, P_wayp, A_wayp, J_max, J_min, t_sync,
                  dcv);
        t_test_size[0] = 2;
        t_test_size[1] = 7;
        std::copy(&dcv[0], &dcv[14], &t_test_data[0]);
        break;
      case 40310:
        aceg_T_AP(-P_init, -V_init, -A_init, -P_wayp, -A_wayp, -J_min, -J_max,
                  t_sync, dcv);
        t_test_size[0] = 2;
        t_test_size[1] = 7;
        std::copy(&dcv[0], &dcv[14], &t_test_data[0]);
        break;
      case 40311:
        acdefg_T_AP(P_init, V_init, A_init, P_wayp, A_wayp, V_max, A_min, J_max,
                    J_min, t_sync, dcv);
        t_test_size[0] = 2;
        t_test_size[1] = 7;
        std::copy(&dcv[0], &dcv[14], &t_test_data[0]);
        break;
      case 40312:
        acdefg_T_AP(-P_init, -V_init, -A_init, -P_wayp, -A_wayp, -V_min, -A_max,
                    -J_min, -J_max, t_sync, dcv);
        t_test_size[0] = 2;
        t_test_size[1] = 7;
        std::copy(&dcv[0], &dcv[14], &t_test_data[0]);
        break;
      case 40313:
        abcdeg_T_AP(P_init, V_init, A_init, P_wayp, A_wayp, V_max, A_max, J_max,
                    J_min, t_sync, dcv3);
        t_test_size[0] = 3;
        t_test_size[1] = 7;
        std::copy(&dcv3[0], &dcv3[21], &t_test_data[0]);
        break;
      case 40314:
        abcdeg_T_AP(-P_init, -V_init, -A_init, -P_wayp, -A_wayp, -V_min, -A_min,
                    -J_min, -J_max, t_sync, dcv3);
        t_test_size[0] = 3;
        t_test_size[1] = 7;
        std::copy(&dcv3[0], &dcv3[21], &t_test_data[0]);
        break;
      case 40315:
        acdeg_T_AP(P_init, V_init, A_init, P_wayp, A_wayp, V_max, J_max, J_min,
                   t_sync, dcv3);
        t_test_size[0] = 3;
        t_test_size[1] = 7;
        std::copy(&dcv3[0], &dcv3[21], &t_test_data[0]);
        break;
      case 40316:
        acdeg_T_AP(-P_init, -V_init, -A_init, -P_wayp, -A_wayp, -V_min, -J_min,
                   -J_max, t_sync, dcv3);
        t_test_size[0] = 3;
        t_test_size[1] = 7;
        std::copy(&dcv3[0], &dcv3[21], &t_test_data[0]);
        // -----------------------
        break;
      case 40401:
        aceg_T_AV(V_init, A_init, V_wayp, A_wayp, J_max, J_min, t_sync,
                  varargin_1);
        t_test_size[0] = 1;
        t_test_size[1] = 7;
        for (i = 0; i < 7; i++) {
          t_test_data[i].re = varargin_1[i];
          t_test_data[i].im = 0.0;
        }
        break;
      case 40402:
        aceg_T_AV(-V_init, -A_init, -V_wayp, -A_wayp, -J_min, -J_max, t_sync,
                  varargin_1);
        t_test_size[0] = 1;
        t_test_size[1] = 7;
        for (i = 0; i < 7; i++) {
          t_test_data[i].re = varargin_1[i];
          t_test_data[i].im = 0.0;
        }
        break;
      case 40403:
        abceg_T_AV(V_init, A_init, V_wayp, A_wayp, A_max, J_max, J_min, t_sync,
                   dcv);
        t_test_size[0] = 2;
        t_test_size[1] = 7;
        std::copy(&dcv[0], &dcv[14], &t_test_data[0]);
        break;
      case 40404:
        abceg_T_AV(-V_init, -A_init, -V_wayp, -A_wayp, -A_min, -J_min, -J_max,
                   t_sync, dcv);
        t_test_size[0] = 2;
        t_test_size[1] = 7;
        std::copy(&dcv[0], &dcv[14], &t_test_data[0]);
        break;
      case 40405:
        acefg_T_AV(V_init, A_init, V_wayp, A_wayp, A_min, J_max, J_min, t_sync,
                   dcv);
        t_test_size[0] = 2;
        t_test_size[1] = 7;
        std::copy(&dcv[0], &dcv[14], &t_test_data[0]);
        break;
      case 40406:
        acefg_T_AV(-V_init, -A_init, -V_wayp, -A_wayp, -A_max, -J_min, -J_max,
                   t_sync, dcv);
        t_test_size[0] = 2;
        t_test_size[1] = 7;
        std::copy(&dcv[0], &dcv[14], &t_test_data[0]);
        break;
      case 40407:
        abcefg_T_AV(V_init, A_init, V_wayp, A_wayp, A_max, A_min, J_max, J_min,
                    t_sync, varargin_1);
        t_test_size[0] = 1;
        t_test_size[1] = 7;
        for (i = 0; i < 7; i++) {
          t_test_data[i].re = varargin_1[i];
          t_test_data[i].im = 0.0;
        }
        break;
      case 40408:
        abcefg_T_AV(-V_init, -A_init, -V_wayp, -A_wayp, -A_min, -A_max, -J_min,
                    -J_max, t_sync, varargin_1);
        t_test_size[0] = 1;
        t_test_size[1] = 7;
        for (i = 0; i < 7; i++) {
          t_test_data[i].re = varargin_1[i];
          t_test_data[i].im = 0.0;
        }
        break;
      case 40409:
        abcdefg_T_AV(V_init, A_init, V_wayp, A_wayp, V_max, A_max, A_min, J_max,
                     J_min, t_sync, varargin_1);
        t_test_size[0] = 1;
        t_test_size[1] = 7;
        for (i = 0; i < 7; i++) {
          t_test_data[i].re = varargin_1[i];
          t_test_data[i].im = 0.0;
        }
        break;
      case 40410:
        abcdefg_T_AV(-V_init, -A_init, -V_wayp, -A_wayp, -V_min, -A_min, -A_max,
                     -J_min, -J_max, t_sync, varargin_1);
        t_test_size[0] = 1;
        t_test_size[1] = 7;
        for (i = 0; i < 7; i++) {
          t_test_data[i].re = varargin_1[i];
          t_test_data[i].im = 0.0;
        }
        break;
      case 40411:
        acdefg_T_AV(V_init, A_init, V_wayp, A_wayp, V_max, A_min, J_max, J_min,
                    t_sync, dcv);
        t_test_size[0] = 2;
        t_test_size[1] = 7;
        std::copy(&dcv[0], &dcv[14], &t_test_data[0]);
        break;
      case 40412:
        acdefg_T_AV(-V_init, -A_init, -V_wayp, -A_wayp, -V_min, -A_max, -J_min,
                    -J_max, t_sync, dcv);
        t_test_size[0] = 2;
        t_test_size[1] = 7;
        std::copy(&dcv[0], &dcv[14], &t_test_data[0]);
        break;
      case 40413:
        acdeg_T_AV(V_init, A_init, V_wayp, A_wayp, V_max, J_max, J_min, t_sync,
                   dcv1);
        t_test_size[0] = 4;
        t_test_size[1] = 7;
        std::copy(&dcv1[0], &dcv1[28], &t_test_data[0]);
        break;
      case 40414:
        acdeg_T_AV(-V_init, -A_init, -V_wayp, -A_wayp, -V_min, -J_min, -J_max,
                   t_sync, dcv1);
        t_test_size[0] = 4;
        t_test_size[1] = 7;
        std::copy(&dcv1[0], &dcv1[28], &t_test_data[0]);
        break;
      case 40415:
        abcdeg_T_AV(V_init, A_init, V_wayp, A_wayp, V_max, A_max, J_max, J_min,
                    t_sync, dcv);
        t_test_size[0] = 2;
        t_test_size[1] = 7;
        std::copy(&dcv[0], &dcv[14], &t_test_data[0]);
        break;
      case 40416:
        abcdeg_T_AV(-V_init, -A_init, -V_wayp, -A_wayp, -V_min, -A_min, -J_min,
                    -J_max, t_sync, dcv);
        t_test_size[0] = 2;
        t_test_size[1] = 7;
        std::copy(&dcv[0], &dcv[14], &t_test_data[0]);
        // -----------------------
        break;
      case 40501:
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
        //  Generated on 29-Aug-2019 15:08:31
        t1 = ((A_init - A_wayp) + J_min * t_sync) / (J_min - J_max);
        b_t1[0].re = t1;
        b_t1[0].im = 0.0;
        b_t1[1].re = 0.0;
        b_t1[1].im = 0.0;
        b_t1[2].re = t_sync - t1;
        b_t1[2].im = 0.0;
        b_t1[3].re = 0.0;
        b_t1[3].im = 0.0;
        b_t1[4].re = 0.0;
        b_t1[4].im = 0.0;
        b_t1[5].re = 0.0;
        b_t1[5].im = 0.0;
        b_t1[6].re = 0.0;
        b_t1[6].im = 0.0;
        t_test_size[0] = 1;
        t_test_size[1] = 7;
        std::copy(&b_t1[0], &b_t1[7], &t_test_data[0]);
        break;
      case 40502:
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
        //  Generated on 29-Aug-2019 15:08:31
        t1 = ((-A_init - (-A_wayp)) + -J_max * t_sync) / (-J_max - (-J_min));
        b_t1[0].re = t1;
        b_t1[0].im = 0.0;
        b_t1[1].re = 0.0;
        b_t1[1].im = 0.0;
        b_t1[2].re = t_sync - t1;
        b_t1[2].im = 0.0;
        b_t1[3].re = 0.0;
        b_t1[3].im = 0.0;
        b_t1[4].re = 0.0;
        b_t1[4].im = 0.0;
        b_t1[5].re = 0.0;
        b_t1[5].im = 0.0;
        b_t1[6].re = 0.0;
        b_t1[6].im = 0.0;
        t_test_size[0] = 1;
        t_test_size[1] = 7;
        std::copy(&b_t1[0], &b_t1[7], &t_test_data[0]);
        break;
      case 40503:
        abc_T_A(A_init, A_wayp, A_max, J_max, J_min, t_sync, varargin_1);
        t_test_size[0] = 1;
        t_test_size[1] = 7;
        for (i = 0; i < 7; i++) {
          t_test_data[i].re = varargin_1[i];
          t_test_data[i].im = 0.0;
        }
        break;
      case 40504:
        abc_T_A(-A_init, -A_wayp, -A_min, -J_min, -J_max, t_sync, varargin_1);
        t_test_size[0] = 1;
        t_test_size[1] = 7;
        for (i = 0; i < 7; i++) {
          t_test_data[i].re = varargin_1[i];
          t_test_data[i].im = 0.0;
        }
        break;
      case 40505:
        abcde_T_A(V_init, A_init, A_wayp, V_max, A_max, J_max, J_min, t_sync,
                  varargin_1);
        t_test_size[0] = 1;
        t_test_size[1] = 7;
        for (i = 0; i < 7; i++) {
          t_test_data[i].re = varargin_1[i];
          t_test_data[i].im = 0.0;
        }
        break;
      case 40506:
        abcde_T_A(-V_init, -A_init, -A_wayp, -V_min, -A_min, -J_min, -J_max,
                  t_sync, varargin_1);
        t_test_size[0] = 1;
        t_test_size[1] = 7;
        for (i = 0; i < 7; i++) {
          t_test_data[i].re = varargin_1[i];
          t_test_data[i].im = 0.0;
        }
        break;
      case 40507:
        acde_T_A(V_init, A_init, A_wayp, V_max, J_max, J_min, t_sync, dcv);
        t_test_size[0] = 2;
        t_test_size[1] = 7;
        std::copy(&dcv[0], &dcv[14], &t_test_data[0]);
        break;
      case 40508:
        acde_T_A(-V_init, -A_init, -A_wayp, -V_min, -J_min, -J_max, t_sync,
                 dcv);
        t_test_size[0] = 2;
        t_test_size[1] = 7;
        std::copy(&dcv[0], &dcv[14], &t_test_data[0]);
        break;
      case 40509:
        abcdeg_T_A(V_init, A_init, A_wayp, V_max, A_max, J_max, J_min, t_sync,
                   dv);
        t_test_size[0] = 2;
        t_test_size[1] = 7;
        for (i = 0; i < 14; i++) {
          t_test_data[i].re = dv[i];
          t_test_data[i].im = 0.0;
        }
        break;
      case 40510:
        abcdeg_T_A(-V_init, -A_init, -A_wayp, -V_min, -A_min, -J_min, -J_max,
                   t_sync, dv);
        t_test_size[0] = 2;
        t_test_size[1] = 7;
        for (i = 0; i < 14; i++) {
          t_test_data[i].re = dv[i];
          t_test_data[i].im = 0.0;
        }
        break;
      case 40511:
        abcdefg_T_A(V_init, A_init, A_wayp, V_max, A_max, A_min, J_max, J_min,
                    t_sync, varargin_1);
        t_test_size[0] = 1;
        t_test_size[1] = 7;
        for (i = 0; i < 7; i++) {
          t_test_data[i].re = varargin_1[i];
          t_test_data[i].im = 0.0;
        }
        break;
      case 40512:
        abcdefg_T_A(-V_init, -A_init, -A_wayp, -V_min, -A_min, -A_max, -J_min,
                    -J_max, t_sync, varargin_1);
        t_test_size[0] = 1;
        t_test_size[1] = 7;
        for (i = 0; i < 7; i++) {
          t_test_data[i].re = varargin_1[i];
          t_test_data[i].im = 0.0;
        }
        break;
      case 40513:
        acdefg_T_A(V_init, A_init, A_wayp, V_max, A_min, J_max, J_min, t_sync,
                   dcv);
        t_test_size[0] = 2;
        t_test_size[1] = 7;
        std::copy(&dcv[0], &dcv[14], &t_test_data[0]);
        break;
      case 40514:
        acdefg_T_A(-V_init, -A_init, -A_wayp, -V_min, -A_max, -J_min, -J_max,
                   t_sync, dcv);
        t_test_size[0] = 2;
        t_test_size[1] = 7;
        std::copy(&dcv[0], &dcv[14], &t_test_data[0]);
        break;
      case 40515:
        acdeg_T_A(V_init, A_init, A_wayp, V_max, J_max, J_min, t_sync, dcv);
        t_test_size[0] = 2;
        t_test_size[1] = 7;
        std::copy(&dcv[0], &dcv[14], &t_test_data[0]);
        break;
      case 40516:
        acdeg_T_A(-V_init, -A_init, -A_wayp, -V_min, -J_min, -J_max, t_sync,
                  dcv);
        t_test_size[0] = 2;
        t_test_size[1] = 7;
        std::copy(&dcv[0], &dcv[14], &t_test_data[0]);
        break;
      case 40517:
        aceg_T_A(V_init, A_init, A_wayp, V_max, J_max, J_min, t_sync,
                 varargin_1);
        t_test_size[0] = 1;
        t_test_size[1] = 7;
        for (i = 0; i < 7; i++) {
          t_test_data[i].re = varargin_1[i];
          t_test_data[i].im = 0.0;
        }
        break;
      case 40518:
        aceg_T_A(-V_init, -A_init, -A_wayp, -V_min, -J_min, -J_max, t_sync,
                 varargin_1);
        t_test_size[0] = 1;
        t_test_size[1] = 7;
        for (i = 0; i < 7; i++) {
          t_test_data[i].re = varargin_1[i];
          t_test_data[i].im = 0.0;
        }
        break;
      case 40519:
        acefg_T_A(V_init, A_init, A_wayp, V_max, A_min, J_max, J_min, t_sync,
                  dcv);
        t_test_size[0] = 2;
        t_test_size[1] = 7;
        std::copy(&dcv[0], &dcv[14], &t_test_data[0]);
        break;
      case 40520:
        acefg_T_A(-V_init, -A_init, -A_wayp, -V_min, -A_max, -J_min, -J_max,
                  t_sync, dcv);
        t_test_size[0] = 2;
        t_test_size[1] = 7;
        std::copy(&dcv[0], &dcv[14], &t_test_data[0]);
        break;
      case 40521:
        abceg_T_A(V_init, A_init, A_wayp, V_max, A_max, J_max, J_min, t_sync,
                  varargin_1);
        t_test_size[0] = 1;
        t_test_size[1] = 7;
        for (i = 0; i < 7; i++) {
          t_test_data[i].re = varargin_1[i];
          t_test_data[i].im = 0.0;
        }
        break;
      case 40522:
        abceg_T_A(-V_init, -A_init, -A_wayp, -V_min, -A_min, -J_min, -J_max,
                  t_sync, varargin_1);
        t_test_size[0] = 1;
        t_test_size[1] = 7;
        for (i = 0; i < 7; i++) {
          t_test_data[i].re = varargin_1[i];
          t_test_data[i].im = 0.0;
        }
        break;
      case 40523:
        abcefg_T_A(V_init, A_init, A_wayp, V_max, A_max, A_min, J_max, J_min,
                   t_sync, varargin_1);
        t_test_size[0] = 1;
        t_test_size[1] = 7;
        for (i = 0; i < 7; i++) {
          t_test_data[i].re = varargin_1[i];
          t_test_data[i].im = 0.0;
        }
        break;
      case 40524:
        abcefg_T_A(-V_init, -A_init, -A_wayp, -V_min, -A_min, -A_max, -J_min,
                   -J_max, t_sync, varargin_1);
        t_test_size[0] = 1;
        t_test_size[1] = 7;
        for (i = 0; i < 7; i++) {
          t_test_data[i].re = varargin_1[i];
          t_test_data[i].im = 0.0;
        }
        // -----------------------
        break;
      case 40601:
        ac_T_V(V_init, A_init, V_wayp, J_max, J_min, t_sync, dcv);
        t_test_size[0] = 2;
        t_test_size[1] = 7;
        std::copy(&dcv[0], &dcv[14], &t_test_data[0]);
        break;
      case 40602:
        ac_T_V(-V_init, -A_init, -V_wayp, -J_min, -J_max, t_sync, dcv);
        t_test_size[0] = 2;
        t_test_size[1] = 7;
        std::copy(&dcv[0], &dcv[14], &t_test_data[0]);
        break;
      case 40603:
        abc_T_V(V_init, A_init, V_wayp, A_max, J_max, J_min, t_sync, dcv);
        t_test_size[0] = 2;
        t_test_size[1] = 7;
        std::copy(&dcv[0], &dcv[14], &t_test_data[0]);
        break;
      case 40604:
        abc_T_V(-V_init, -A_init, -V_wayp, -A_min, -J_min, -J_max, t_sync, dcv);
        t_test_size[0] = 2;
        t_test_size[1] = 7;
        std::copy(&dcv[0], &dcv[14], &t_test_data[0]);
        break;
      case 40605:
        abcef_T_V(V_init, A_init, V_wayp, A_max, A_min, J_max, J_min, t_sync,
                  varargin_1);
        t_test_size[0] = 1;
        t_test_size[1] = 7;
        for (i = 0; i < 7; i++) {
          t_test_data[i].re = varargin_1[i];
          t_test_data[i].im = 0.0;
        }
        break;
      case 40606:
        abcef_T_V(-V_init, -A_init, -V_wayp, -A_min, -A_max, -J_min, -J_max,
                  t_sync, varargin_1);
        t_test_size[0] = 1;
        t_test_size[1] = 7;
        for (i = 0; i < 7; i++) {
          t_test_data[i].re = varargin_1[i];
          t_test_data[i].im = 0.0;
        }
        break;
      case 40607:
        acef_T_V(V_init, A_init, V_wayp, A_min, J_max, J_min, t_sync, dcv);
        t_test_size[0] = 2;
        t_test_size[1] = 7;
        std::copy(&dcv[0], &dcv[14], &t_test_data[0]);
        break;
      case 40608:
        acef_T_V(-V_init, -A_init, -V_wayp, -A_max, -J_min, -J_max, t_sync,
                 dcv);
        t_test_size[0] = 2;
        t_test_size[1] = 7;
        std::copy(&dcv[0], &dcv[14], &t_test_data[0]);
        break;
      case 40609:
        aceg_T_V(V_init, A_init, V_wayp, V_min, J_max, J_min, t_sync, dcv);
        t_test_size[0] = 2;
        t_test_size[1] = 7;
        std::copy(&dcv[0], &dcv[14], &t_test_data[0]);
        break;
      case 40610:
        aceg_T_V(-V_init, -A_init, -V_wayp, -V_max, -J_min, -J_max, t_sync,
                 dcv);
        t_test_size[0] = 2;
        t_test_size[1] = 7;
        std::copy(&dcv[0], &dcv[14], &t_test_data[0]);
        break;
      case 40611:
        abceg_T_V(V_init, A_init, V_wayp, V_min, A_max, J_max, J_min, t_sync,
                  dcv1);
        t_test_size[0] = 4;
        t_test_size[1] = 7;
        std::copy(&dcv1[0], &dcv1[28], &t_test_data[0]);
        break;
      case 40612:
        abceg_T_V(-V_init, -A_init, -V_wayp, -V_max, -A_min, -J_min, -J_max,
                  t_sync, dcv1);
        t_test_size[0] = 4;
        t_test_size[1] = 7;
        std::copy(&dcv1[0], &dcv1[28], &t_test_data[0]);
        break;
      case 40613:
        abcefg_T_V(V_init, A_init, V_wayp, V_min, A_max, A_min, J_max, J_min,
                   t_sync, dcv);
        t_test_size[0] = 2;
        t_test_size[1] = 7;
        std::copy(&dcv[0], &dcv[14], &t_test_data[0]);
        break;
      case 40614:
        abcefg_T_V(-V_init, -A_init, -V_wayp, -V_max, -A_min, -A_max, -J_min,
                   -J_max, t_sync, dcv);
        t_test_size[0] = 2;
        t_test_size[1] = 7;
        std::copy(&dcv[0], &dcv[14], &t_test_data[0]);
        break;
      case 40615:
        abcdefg_T_V(V_init, A_init, V_wayp, V_max, V_min, A_max, A_min, J_max,
                    J_min, t_sync, dv);
        t_test_size[0] = 2;
        t_test_size[1] = 7;
        for (i = 0; i < 14; i++) {
          t_test_data[i].re = dv[i];
          t_test_data[i].im = 0.0;
        }
        break;
      case 40616:
        abcdefg_T_V(-V_init, -A_init, -V_wayp, -V_min, -V_max, -A_min, -A_max,
                    -J_min, -J_max, t_sync, dv);
        t_test_size[0] = 2;
        t_test_size[1] = 7;
        for (i = 0; i < 14; i++) {
          t_test_data[i].re = dv[i];
          t_test_data[i].im = 0.0;
        }
        break;
      case 40617:
        abcdef_T_V(V_init, A_init, V_wayp, V_max, A_max, A_min, J_max, J_min,
                   t_sync, varargin_1);
        t_test_size[0] = 1;
        t_test_size[1] = 7;
        for (i = 0; i < 7; i++) {
          t_test_data[i].re = varargin_1[i];
          t_test_data[i].im = 0.0;
        }
        break;
      case 40618:
        abcdef_T_V(-V_init, -A_init, -V_wayp, -V_min, -A_min, -A_max, -J_min,
                   -J_max, t_sync, varargin_1);
        t_test_size[0] = 1;
        t_test_size[1] = 7;
        for (i = 0; i < 7; i++) {
          t_test_data[i].re = varargin_1[i];
          t_test_data[i].im = 0.0;
        }
        break;
      case 40619:
        acdef_T_V(V_init, A_init, V_wayp, V_max, A_min, J_max, J_min, t_sync,
                  dcv);
        t_test_size[0] = 2;
        t_test_size[1] = 7;
        std::copy(&dcv[0], &dcv[14], &t_test_data[0]);
        break;
      case 40620:
        acdef_T_V(-V_init, -A_init, -V_wayp, -V_min, -A_max, -J_min, -J_max,
                  t_sync, dcv);
        t_test_size[0] = 2;
        t_test_size[1] = 7;
        std::copy(&dcv[0], &dcv[14], &t_test_data[0]);
        break;
      case 40621:
        acdefg_T_V(V_init, A_init, V_wayp, V_max, V_min, A_min, J_max, J_min,
                   t_sync, dcv);
        t_test_size[0] = 2;
        t_test_size[1] = 7;
        std::copy(&dcv[0], &dcv[14], &t_test_data[0]);
        break;
      case 40622:
        acdefg_T_V(-V_init, -A_init, -V_wayp, -V_min, -V_max, -A_max, -J_min,
                   -J_max, t_sync, dcv);
        t_test_size[0] = 2;
        t_test_size[1] = 7;
        std::copy(&dcv[0], &dcv[14], &t_test_data[0]);
        break;
      case 40623:
        acdeg_T_V(V_init, A_init, V_wayp, V_max, V_min, J_max, J_min, t_sync,
                  dcv);
        t_test_size[0] = 2;
        t_test_size[1] = 7;
        std::copy(&dcv[0], &dcv[14], &t_test_data[0]);
        break;
      case 40624:
        acdeg_T_V(-V_init, -A_init, -V_wayp, -V_min, -V_max, -J_min, -J_max,
                  t_sync, dcv);
        t_test_size[0] = 2;
        t_test_size[1] = 7;
        std::copy(&dcv[0], &dcv[14], &t_test_data[0]);
        break;
      case 40625:
        abcdeg_T_V(V_init, A_init, V_wayp, V_max, V_min, A_max, J_max, J_min,
                   t_sync, dcv);
        t_test_size[0] = 2;
        t_test_size[1] = 7;
        std::copy(&dcv[0], &dcv[14], &t_test_data[0]);
        break;
      case 40626:
        abcdeg_T_V(-V_init, -A_init, -V_wayp, -V_min, -V_max, -A_min, -J_min,
                   -J_max, t_sync, dcv);
        t_test_size[0] = 2;
        t_test_size[1] = 7;
        std::copy(&dcv[0], &dcv[14], &t_test_data[0]);
        break;
      case 40627:
        acefg_T_V(V_init, A_init, V_wayp, V_min, A_min, J_max, J_min, t_sync,
                  dcv);
        t_test_size[0] = 2;
        t_test_size[1] = 7;
        std::copy(&dcv[0], &dcv[14], &t_test_data[0]);
        break;
      case 40628:
        acefg_T_V(-V_init, -A_init, -V_wayp, -V_max, -A_max, -J_min, -J_max,
                  t_sync, dcv);
        t_test_size[0] = 2;
        t_test_size[1] = 7;
        std::copy(&dcv[0], &dcv[14], &t_test_data[0]);
        break;
      case 40629:
        abcde_T_V(V_init, A_init, V_wayp, V_max, A_max, J_max, J_min, t_sync,
                  dcv);
        t_test_size[0] = 2;
        t_test_size[1] = 7;
        std::copy(&dcv[0], &dcv[14], &t_test_data[0]);
        break;
      case 40630:
        abcde_T_V(-V_init, -A_init, -V_wayp, -V_min, -A_min, -J_min, -J_max,
                  t_sync, dcv);
        t_test_size[0] = 2;
        t_test_size[1] = 7;
        std::copy(&dcv[0], &dcv[14], &t_test_data[0]);
        break;
      case 40631:
        acde_T_V(V_init, A_init, V_wayp, V_max, J_max, J_min, t_sync, dcv);
        t_test_size[0] = 2;
        t_test_size[1] = 7;
        std::copy(&dcv[0], &dcv[14], &t_test_data[0]);
        break;
      case 40632:
        acde_T_V(-V_init, -A_init, -V_wayp, -V_min, -J_min, -J_max, t_sync,
                 dcv);
        t_test_size[0] = 2;
        t_test_size[1] = 7;
        std::copy(&dcv[0], &dcv[14], &t_test_data[0]);
        // -----------------------
        break;
      case 40701:
        abcef_T_P(P_init, V_init, A_init, P_wayp, A_max, A_min, J_max, J_min,
                  t_sync, dcv);
        t_test_size[0] = 2;
        t_test_size[1] = 7;
        std::copy(&dcv[0], &dcv[14], &t_test_data[0]);
        break;
      case 40702:
        abcef_T_P(-P_init, -V_init, -A_init, -P_wayp, -A_min, -A_max, -J_min,
                  -J_max, t_sync, dcv);
        t_test_size[0] = 2;
        t_test_size[1] = 7;
        std::copy(&dcv[0], &dcv[14], &t_test_data[0]);
        break;
      case 40703:
        acef_T_P(P_init, V_init, A_init, P_wayp, A_min, J_max, J_min, t_sync,
                 dcv3);
        t_test_size[0] = 3;
        t_test_size[1] = 7;
        std::copy(&dcv3[0], &dcv3[21], &t_test_data[0]);
        break;
      case 40704:
        acef_T_P(-P_init, -V_init, -A_init, -P_wayp, -A_max, -J_min, -J_max,
                 t_sync, dcv3);
        t_test_size[0] = 3;
        t_test_size[1] = 7;
        std::copy(&dcv3[0], &dcv3[21], &t_test_data[0]);
        break;
      case 40705:
        ac_T_P(P_init, V_init, A_init, P_wayp, J_max, J_min, t_sync, dcv3);
        t_test_size[0] = 3;
        t_test_size[1] = 7;
        std::copy(&dcv3[0], &dcv3[21], &t_test_data[0]);
        break;
      case 40706:
        ac_T_P(-P_init, -V_init, -A_init, -P_wayp, -J_min, -J_max, t_sync,
               dcv3);
        t_test_size[0] = 3;
        t_test_size[1] = 7;
        std::copy(&dcv3[0], &dcv3[21], &t_test_data[0]);
        break;
      case 40707:
        abcdef_T_P(P_init, V_init, A_init, P_wayp, V_max, A_max, A_min, J_max,
                   J_min, t_sync, dcv);
        t_test_size[0] = 2;
        t_test_size[1] = 7;
        std::copy(&dcv[0], &dcv[14], &t_test_data[0]);
        break;
      case 40708:
        abcdef_T_P(-P_init, -V_init, -A_init, -P_wayp, -V_min, -A_min, -A_max,
                   -J_min, -J_max, t_sync, dcv);
        t_test_size[0] = 2;
        t_test_size[1] = 7;
        std::copy(&dcv[0], &dcv[14], &t_test_data[0]);
        break;
      case 40709:
        acdef_T_P(P_init, V_init, A_init, P_wayp, V_max, A_min, J_max, J_min,
                  t_sync, dcv);
        t_test_size[0] = 2;
        t_test_size[1] = 7;
        std::copy(&dcv[0], &dcv[14], &t_test_data[0]);
        break;
      case 40710:
        acdef_T_P(-P_init, -V_init, -A_init, -P_wayp, -V_min, -A_max, -J_min,
                  -J_max, t_sync, dcv);
        t_test_size[0] = 2;
        t_test_size[1] = 7;
        std::copy(&dcv[0], &dcv[14], &t_test_data[0]);
        break;
      case 40711:
        abc_T_P(P_init, V_init, A_init, P_wayp, A_max, J_max, J_min, t_sync,
                dcv3);
        t_test_size[0] = 3;
        t_test_size[1] = 7;
        std::copy(&dcv3[0], &dcv3[21], &t_test_data[0]);
        break;
      case 40712:
        abc_T_P(-P_init, -V_init, -A_init, -P_wayp, -A_min, -J_min, -J_max,
                t_sync, dcv3);
        t_test_size[0] = 3;
        t_test_size[1] = 7;
        std::copy(&dcv3[0], &dcv3[21], &t_test_data[0]);
        break;
      case 40713:
        abcdefg_T_P(P_init, V_init, A_init, P_wayp, V_max, V_min, A_max, A_min,
                    J_max, J_min, t_sync, dcv3);
        t_test_size[0] = 3;
        t_test_size[1] = 7;
        std::copy(&dcv3[0], &dcv3[21], &t_test_data[0]);
        break;
      case 40714:
        abcdefg_T_P(-P_init, -V_init, -A_init, -P_wayp, -V_min, -V_max, -A_min,
                    -A_max, -J_min, -J_max, t_sync, dcv3);
        t_test_size[0] = 3;
        t_test_size[1] = 7;
        std::copy(&dcv3[0], &dcv3[21], &t_test_data[0]);
        break;
      case 40715:
        abcdeg_T_P(P_init, V_init, A_init, P_wayp, V_max, V_min, A_max, J_max,
                   J_min, t_sync, dcv3);
        t_test_size[0] = 3;
        t_test_size[1] = 7;
        std::copy(&dcv3[0], &dcv3[21], &t_test_data[0]);
        break;
      case 40716:
        abcdeg_T_P(-P_init, -V_init, -A_init, -P_wayp, -V_min, -V_max, -A_min,
                   -J_min, -J_max, t_sync, dcv3);
        t_test_size[0] = 3;
        t_test_size[1] = 7;
        std::copy(&dcv3[0], &dcv3[21], &t_test_data[0]);
        break;
      case 40717:
        aceg_T_P(P_init, V_init, A_init, P_wayp, V_min, J_max, J_min, t_sync,
                 dcv1);
        t_test_size[0] = 4;
        t_test_size[1] = 7;
        std::copy(&dcv1[0], &dcv1[28], &t_test_data[0]);
        break;
      case 40718:
        aceg_T_P(-P_init, -V_init, -A_init, -P_wayp, -V_max, -J_min, -J_max,
                 t_sync, dcv1);
        t_test_size[0] = 4;
        t_test_size[1] = 7;
        std::copy(&dcv1[0], &dcv1[28], &t_test_data[0]);
        break;
      case 40719:
        abceg_T_P(P_init, V_init, A_init, P_wayp, V_min, A_max, J_max, J_min,
                  t_sync, t_test_data);
        t_test_size[0] = 6;
        t_test_size[1] = 7;
        break;
      case 40720:
        abceg_T_P(-P_init, -V_init, -A_init, -P_wayp, -V_max, -A_min, -J_min,
                  -J_max, t_sync, t_test_data);
        t_test_size[0] = 6;
        t_test_size[1] = 7;
        break;
      case 40721:
        abcefg_T_P(P_init, V_init, A_init, P_wayp, V_min, A_max, A_min, J_max,
                   J_min, t_sync, dcv3);
        t_test_size[0] = 3;
        t_test_size[1] = 7;
        std::copy(&dcv3[0], &dcv3[21], &t_test_data[0]);
        break;
      case 40722:
        abcefg_T_P(-P_init, -V_init, -A_init, -P_wayp, -V_max, -A_min, -A_max,
                   -J_min, -J_max, t_sync, dcv3);
        t_test_size[0] = 3;
        t_test_size[1] = 7;
        std::copy(&dcv3[0], &dcv3[21], &t_test_data[0]);
        break;
      case 40723:
        acefg_T_P(P_init, V_init, A_init, P_wayp, V_min, A_min, J_max, J_min,
                  t_sync, t_test_data);
        t_test_size[0] = 6;
        t_test_size[1] = 7;
        break;
      case 40724:
        acefg_T_P(-P_init, -V_init, -A_init, -P_wayp, -V_max, -A_max, -J_min,
                  -J_max, t_sync, t_test_data);
        t_test_size[0] = 6;
        t_test_size[1] = 7;
        break;
      case 40725:
        abcde_T_P(P_init, V_init, A_init, P_wayp, V_max, A_max, J_max, J_min,
                  t_sync, dcv3);
        t_test_size[0] = 3;
        t_test_size[1] = 7;
        std::copy(&dcv3[0], &dcv3[21], &t_test_data[0]);
        break;
      case 40726:
        abcde_T_P(-P_init, -V_init, -A_init, -P_wayp, -V_min, -A_min, -J_min,
                  -J_max, t_sync, dcv3);
        t_test_size[0] = 3;
        t_test_size[1] = 7;
        std::copy(&dcv3[0], &dcv3[21], &t_test_data[0]);
        break;
      case 40727:
        acde_T_P(P_init, V_init, A_init, P_wayp, V_max, J_max, J_min, t_sync,
                 dcv3);
        t_test_size[0] = 3;
        t_test_size[1] = 7;
        std::copy(&dcv3[0], &dcv3[21], &t_test_data[0]);
        break;
      case 40728:
        acde_T_P(-P_init, -V_init, -A_init, -P_wayp, -V_min, -J_min, -J_max,
                 t_sync, dcv3);
        t_test_size[0] = 3;
        t_test_size[1] = 7;
        std::copy(&dcv3[0], &dcv3[21], &t_test_data[0]);
        break;
      case 40729:
        acdeg_T_P(P_init, V_init, A_init, P_wayp, V_max, V_min, J_max, J_min,
                  t_sync, dcv3);
        t_test_size[0] = 3;
        t_test_size[1] = 7;
        std::copy(&dcv3[0], &dcv3[21], &t_test_data[0]);
        break;
      case 40730:
        acdeg_T_P(-P_init, -V_init, -A_init, -P_wayp, -V_min, -V_max, -J_min,
                  -J_max, t_sync, dcv3);
        t_test_size[0] = 3;
        t_test_size[1] = 7;
        std::copy(&dcv3[0], &dcv3[21], &t_test_data[0]);
        break;
      case 40731:
        acdefg_T_P(P_init, V_init, A_init, P_wayp, V_max, V_min, A_min, J_max,
                   J_min, t_sync, dcv3);
        t_test_size[0] = 3;
        t_test_size[1] = 7;
        std::copy(&dcv3[0], &dcv3[21], &t_test_data[0]);
        break;
      case 40732:
        acdefg_T_P(-P_init, -V_init, -A_init, -P_wayp, -V_min, -V_max, -A_max,
                   -J_min, -J_max, t_sync, dcv3);
        t_test_size[0] = 3;
        t_test_size[1] = 7;
        std::copy(&dcv3[0], &dcv3[21], &t_test_data[0]);
        break;
      default:
        t_test_size[0] = 1;
        t_test_size[1] = 7;
        std::memset(&t_test_data[0], 0, 7U * sizeof(creal_T));
        printf("Error: Solution prior ");
        fflush(stdout);
        printint(i);
        printf(" is not valid!\n");
        fflush(stdout);
        break;
      }
      e_check(t_test_data, t_test_size, J_test, P_init, V_init, A_init, P_wayp,
              V_wayp, A_wayp, V_max, V_min, A_max, A_min, J_max, J_min, t_sync,
              valid_data, t_1_size);
      i = t_1_size[1];
      for (b_loop_ub = 0; b_loop_ub < i; b_loop_ub++) {
        if (b_loop_ub + 1 > t_1_size[1]) {
          rtDynamicBoundsError(b_loop_ub + 1, 1, t_1_size[1], &rb_emlrtBCI);
        }
        if (valid_data[b_loop_ub]) {
          bool exitg1;
          if (b_loop_ub + 1 > t_test_size[0]) {
            rtDynamicBoundsError(b_loop_ub + 1, 1, t_test_size[0],
                                 &qb_emlrtBCI);
          }
          for (k = 0; k < 7; k++) {
            V_int = t_test_data[b_loop_ub + t_test_size[0] * k].re;
            if (V_int > 0.0) {
              t_test_valid[k] = V_int;
            } else {
              t_test_valid[k] = 0.0;
            }
          }
          empty_non_axis_sizes = false;
          k = 0;
          exitg1 = false;
          while ((!exitg1) && (k <= num_valid)) {
            simplify_setp(t_test_valid, J_test, t_test_simpl_data, J_1_size,
                          J_test_simpl_data, t_2_size);
            for (i1 = 0; i1 < 7; i1++) {
              t_1_tmp = k + 100 * i1;
              varargin_1[i1] = t_out_data[t_1_tmp];
              J_out[i1] = J_out_data[t_1_tmp];
            }
            simplify_setp(varargin_1, J_out, t_out_simpl_data, J_2_size,
                          J_out_simpl_data, J_out_simpl_size);
            // numerical
            // numerical
            if ((J_1_size[1] == J_2_size[1]) &&
                (t_2_size[1] == J_out_simpl_size[1])) {
              rtSizeEqNDCheck(&J_1_size[0], &J_2_size[0], &e_emlrtECI);
              t_test_simpl_size[0] = 1;
              t_test_simpl_size[1] = J_1_size[1];
              loop_ub = J_1_size[1];
              for (i1 = 0; i1 < loop_ub; i1++) {
                varargin_1[i1] = t_test_simpl_data[i1] - t_out_simpl_data[i1];
              }
              coder::b_abs(varargin_1, t_test_simpl_size, t_test_simpl_data,
                           J_1_size);
              b_t_test_simpl_size[0] = 1;
              b_t_test_simpl_size[1] = J_1_size[1];
              loop_ub = J_1_size[1];
              for (i1 = 0; i1 < loop_ub; i1++) {
                b_t_test_simpl_data[i1] = (t_test_simpl_data[i1] < 1.0E-7);
              }
              if (coder::all(b_t_test_simpl_data, b_t_test_simpl_size)) {
                rtSizeEqNDCheck(&t_2_size[0], &J_out_simpl_size[0],
                                &d_emlrtECI);
                J_2_size[0] = 1;
                J_2_size[1] = t_2_size[1];
                loop_ub = t_2_size[1];
                for (i1 = 0; i1 < loop_ub; i1++) {
                  t_out_simpl_data[i1] =
                      J_test_simpl_data[i1] - J_out_simpl_data[i1];
                }
                coder::b_abs(t_out_simpl_data, J_2_size, t_test_simpl_data,
                             J_1_size);
                b_t_test_simpl_size[0] = 1;
                b_t_test_simpl_size[1] = J_1_size[1];
                loop_ub = J_1_size[1];
                for (i1 = 0; i1 < loop_ub; i1++) {
                  b_t_test_simpl_data[i1] = (t_test_simpl_data[i1] < 1.0E-7);
                }
                if (coder::all(b_t_test_simpl_data, b_t_test_simpl_size)) {
                  empty_non_axis_sizes = true;
                } else {
                  empty_non_axis_sizes = false;
                }
              } else {
                empty_non_axis_sizes = false;
              }
            } else {
              empty_non_axis_sizes = false;
            }
            if (empty_non_axis_sizes) {
              exitg1 = true;
            } else {
              k++;
            }
          }
          if (!empty_non_axis_sizes) {
            num_valid++;
            if ((num_valid + 1 < 1) || (num_valid + 1 > 100)) {
              rtDynamicBoundsError(num_valid + 1, 1, 100, &pb_emlrtBCI);
            }
            for (i1 = 0; i1 < 7; i1++) {
              k = num_valid + 100 * i1;
              t_out_data[k] = t_test_valid[i1];
              J_out_data[k] = J_test[i1];
            }
            t_out_data[num_valid + 700] = 0.0;
            J_out_data[num_valid + 700] = 0.0;
            t_out_data[num_valid + 800] = 0.0;
            J_out_data[num_valid + 800] = 0.0;
            t_out_data[num_valid + 900] = 0.0;
            J_out_data[num_valid + 900] = 0.0;
            t_out_data[num_valid + 1000] = 0.0;
            J_out_data[num_valid + 1000] = 0.0;
            solution_out_data[num_valid] = b_cases_data[a__1_size];
          }
        }
      }
    }
    if (1 > num_valid + 1) {
      loop_ub = 0;
      b_loop_ub = 0;
    } else {
      loop_ub = num_valid + 1;
      b_loop_ub = num_valid + 1;
    }
    for (i = 0; i < 11; i++) {
      for (i1 = 0; i1 < loop_ub; i1++) {
        t_out_data[i1 + loop_ub * i] = t_out_data[i1 + 100 * i];
      }
      for (i1 = 0; i1 < b_loop_ub; i1++) {
        J_out_data[i1 + b_loop_ub * i] = J_out_data[i1 + 100 * i];
      }
    }
    t_out_size[0] = loop_ub;
    t_out_size[1] = 11;
    J_out_size[0] = b_loop_ub;
    J_out_size[1] = 11;
    if (1 > num_valid + 1) {
      i = -1;
    } else {
      i = num_valid;
    }
    *solution_out_size = i + 1;
    if (num_valid + 1 > 0) {
      // So that sync time is really the same. adjust t4 so that J == 0 and
      // induced error is minimal
      coder::b_sum(t_out_data, t_out_size, a__3_data, &k);
      if (loop_ub != k) {
        rtSizeEq1DError(loop_ub, k, &c_emlrtECI);
      }
      rtSubAssignSizeCheck(&loop_ub, 1, &loop_ub, 1, &f_emlrtECI);
      t_1_tmp = loop_ub - 1;
      for (i = 0; i <= t_1_tmp; i++) {
        a__3_data[i] = (t_out_data[i + loop_ub * 3] - a__3_data[i]) + t_sync;
      }
      for (i = 0; i < loop_ub; i++) {
        t_out_data[i + loop_ub * 3] = a__3_data[i];
      }
    }
    if (num_valid + 1 == 0) {
      printf("Error: Could not find valid timed solution!\n");
      fflush(stdout);
    } else if (num_valid + 1 > 1) {
      printf("Debug: Multiple (");
      fflush(stdout);
      printint(static_cast<double>(num_valid + 1));
      printf(") timed solutions!\n");
      fflush(stdout);
      coder::sum(solution_out_data, *solution_out_size, a__3_data, &k);
      coder::internal::b_sort(a__3_data, &k, a__1_data, &a__1_size);
      for (i = 0; i < a__1_size; i++) {
        a__3_data[i] = a__1_data[i];
      }
      // TODO Here should be a better sorting instead of sorting via the Case ID
      for (i = 0; i < 11; i++) {
        for (i1 = 0; i1 < a__1_size; i1++) {
          t_1_tmp = static_cast<int>(a__3_data[i1]);
          if ((t_1_tmp < 1) || (t_1_tmp > loop_ub)) {
            rtDynamicBoundsError(t_1_tmp, 1, loop_ub, &ec_emlrtBCI);
          }
          b_t_out_data[i1 + a__1_size * i] =
              t_out_data[(t_1_tmp + loop_ub * i) - 1];
        }
      }
      t_out_size[0] = a__1_size;
      t_out_size[1] = 11;
      loop_ub = a__1_size * 11;
      if (0 <= loop_ub - 1) {
        std::copy(&b_t_out_data[0], &b_t_out_data[loop_ub], &t_out_data[0]);
      }
      for (i = 0; i < 11; i++) {
        for (i1 = 0; i1 < a__1_size; i1++) {
          t_1_tmp = static_cast<int>(a__3_data[i1]);
          if ((t_1_tmp < 1) || (t_1_tmp > b_loop_ub)) {
            rtDynamicBoundsError(t_1_tmp, 1, b_loop_ub, &dc_emlrtBCI);
          }
          b_t_out_data[i1 + a__1_size * i] =
              J_out_data[(t_1_tmp + b_loop_ub * i) - 1];
        }
      }
      J_out_size[0] = a__1_size;
      J_out_size[1] = 11;
      loop_ub = a__1_size * 11;
      if (0 <= loop_ub - 1) {
        std::copy(&b_t_out_data[0], &b_t_out_data[loop_ub], &J_out_data[0]);
      }
      for (i = 0; i < a__1_size; i++) {
        i1 = static_cast<int>(a__3_data[i]);
        if ((i1 < 1) || (i1 > *solution_out_size)) {
          rtDynamicBoundsError(i1, 1, *solution_out_size, &nb_emlrtBCI);
        }
        a__1_data[i] = solution_out_data[i1 - 1];
      }
      *solution_out_size = a__1_size;
      if (0 <= a__1_size - 1) {
        std::copy(&a__1_data[0], &a__1_data[a__1_size], &solution_out_data[0]);
      }
    }
  }
}

// End of code generation (solve_T.cpp)
