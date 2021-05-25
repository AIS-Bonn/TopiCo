//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// solve_O.cpp
//
// Code generation for function 'solve_O'
//

// Include files
#include "solve_O.h"
#include "a_O_P.h"
#include "a_O_V.h"
#include "ab_O_P.h"
#include "abc_O_A.h"
#include "abc_O_AP.h"
#include "abc_O_P.h"
#include "abc_O_V.h"
#include "abc_O_VP.h"
#include "abcd_NO_P.h"
#include "abcd_O_P.h"
#include "abcde_NO_AP.h"
#include "abcde_NO_VP.h"
#include "abcde_O_AP.h"
#include "abcde_O_VP.h"
#include "abcdef_NO_VP.h"
#include "abcdef_O_VP.h"
#include "abcdefg_NO_AP.h"
#include "abcdefg_NO_AVP.h"
#include "abcdefg_NO_VP.h"
#include "abcdefg_O_AP.h"
#include "abcdefg_O_AVP.h"
#include "abcdefg_O_VP.h"
#include "abcdeg_NO_AP.h"
#include "abcdeg_NO_AVP.h"
#include "abcdeg_NO_VP.h"
#include "abcdeg_O_AP.h"
#include "abcdeg_O_AVP.h"
#include "abcdeg_O_VP.h"
#include "abcef_O_VP.h"
#include "abcefg_O_AP.h"
#include "abcefg_O_AVP.h"
#include "abceg_O_AP.h"
#include "abceg_O_AVP.h"
#include "abs.h"
#include "ac_O_A.h"
#include "ac_O_AP.h"
#include "ac_O_AV.h"
#include "ac_O_P.h"
#include "ac_O_V.h"
#include "acd_NO_P.h"
#include "acd_O_P.h"
#include "acde_NO_AP.h"
#include "acde_NO_VP.h"
#include "acde_O_AP.h"
#include "acde_O_VP.h"
#include "acdef_NO_VP.h"
#include "acdef_O_VP.h"
#include "acdefg_NO_AP.h"
#include "acdefg_NO_AVP.h"
#include "acdefg_NO_VP.h"
#include "acdefg_O_AP.h"
#include "acdefg_O_AVP.h"
#include "acdefg_O_VP.h"
#include "acdeg_NO_AP.h"
#include "acdeg_NO_AVP.h"
#include "acdeg_O_AP.h"
#include "acdeg_O_AVP.h"
#include "acef_O_VP.h"
#include "acefg_O_AP.h"
#include "acefg_O_AVP.h"
#include "aceg_O_AP.h"
#include "aceg_O_AVP.h"
#include "all.h"
#include "check.h"
#include "check_feasibility.h"
#include "mod.h"
#include "printint.h"
#include "rt_nonfinite.h"
#include "select_cases_O.h"
#include "simplify_setp.h"
#include "sort.h"
#include "sub2ind.h"
#include "sum.h"
#include "topico_wrapper_data.h"
#include "topico_wrapper_rtwutil.h"
#include "topico_wrapper_types.h"
#include "rt_nonfinite.h"
#include <algorithm>
#include <cmath>
#include <cstring>
#include <stdio.h>

// Variable Definitions
static rtBoundsCheckInfo bb_emlrtBCI = { -1,// iFirst
  -1,                                  // iLast
  57,                                  // lineNo
  26,                                  // colNo
  "cases",                             // aName
  "solve_O",                           // fName
  "/home/lmbeul/Desktop/TopiCo/MATLAB/solve_O.m",// pName
  0                                    // checkKind
};

static rtBoundsCheckInfo cb_emlrtBCI = { -1,// iFirst
  -1,                                  // iLast
  374,                                 // lineNo
  48,                                  // colNo
  "t_test",                            // aName
  "solve_O",                           // fName
  "/home/lmbeul/Desktop/TopiCo/MATLAB/solve_O.m",// pName
  0                                    // checkKind
};

static rtEqualityCheckInfo emlrtECI = { 2,// nDims
  381,                                 // lineNo
  138,                                 // colNo
  "solve_O",                           // fName
  "/home/lmbeul/Desktop/TopiCo/MATLAB/solve_O.m"// pName
};

static rtEqualityCheckInfo b_emlrtECI = { 2,// nDims
  381,                                 // lineNo
  186,                                 // colNo
  "solve_O",                           // fName
  "/home/lmbeul/Desktop/TopiCo/MATLAB/solve_O.m"// pName
};

static rtBoundsCheckInfo db_emlrtBCI = { 1,// iFirst
  100,                                 // iLast
  388,                                 // lineNo
  27,                                  // colNo
  "t_out",                             // aName
  "solve_O",                           // fName
  "/home/lmbeul/Desktop/TopiCo/MATLAB/solve_O.m",// pName
  3                                    // checkKind
};

static rtBoundsCheckInfo eb_emlrtBCI = { -1,// iFirst
  -1,                                  // iLast
  373,                                 // lineNo
  17,                                  // colNo
  "valid",                             // aName
  "solve_O",                           // fName
  "/home/lmbeul/Desktop/TopiCo/MATLAB/solve_O.m",// pName
  0                                    // checkKind
};

static rtBoundsCheckInfo fb_emlrtBCI = { -1,// iFirst
  -1,                                  // iLast
  404,                                 // lineNo
  23,                                  // colNo
  "t_out",                             // aName
  "solve_O",                           // fName
  "/home/lmbeul/Desktop/TopiCo/MATLAB/solve_O.m",// pName
  0                                    // checkKind
};

static rtBoundsCheckInfo gb_emlrtBCI = { -1,// iFirst
  -1,                                  // iLast
  405,                                 // lineNo
  23,                                  // colNo
  "J_out",                             // aName
  "solve_O",                           // fName
  "/home/lmbeul/Desktop/TopiCo/MATLAB/solve_O.m",// pName
  0                                    // checkKind
};

static rtBoundsCheckInfo hb_emlrtBCI = { -1,// iFirst
  -1,                                  // iLast
  406,                                 // lineNo
  37,                                  // colNo
  "solution_out",                      // aName
  "solve_O",                           // fName
  "/home/lmbeul/Desktop/TopiCo/MATLAB/solve_O.m",// pName
  0                                    // checkKind
};

// Function Definitions
void b_solve_O(double P_init, double V_init, double A_init, double V_wayp,
               double V_max, double A_max, double A_min, double J_max, double
               J_min, double t_out_data[], int t_out_size[2], double J_out_data[],
               int J_out_size[2], int solution_out_data[], int
               *solution_out_size)
{
  creal_T t_test_data[35];
  creal_T dcv1[28];
  creal_T dcv2[21];
  creal_T dcv[14];
  creal_T b_A_init[7];
  double J_out[700];
  double t_out[700];
  double a__1_data[100];
  double dv[14];
  double J_out_simpl_data[7];
  double J_test[7];
  double J_test_simpl_data[7];
  double b_J_out[7];
  double b_varargin_1[7];
  double t_out_simpl_data[7];
  double t_test_simpl_data[7];
  double t_test_valid[7];
  int iidx_data[100];
  int solution_out[100];
  int cases_data[45];
  int J_out_simpl_size[2];
  int J_test_simpl_size[2];
  int b_t_test_simpl_size[2];
  int c_t_test_simpl_size[2];
  int cases_size[2];
  int t_out_simpl_size[2];
  int t_test_simpl_size[2];
  int t_test_size[2];
  int valid_size[2];
  int b_index;
  int i;
  int i1;
  int i2;
  int idx;
  int loop_ub;
  int num_valid;
  int varargin_1;
  bool b_t_test_simpl_data[7];
  bool valid_data[5];

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
  num_valid = 0;
  std::memset(&t_out[0], 0, 700U * sizeof(double));
  std::memset(&J_out[0], 0, 700U * sizeof(double));
  std::memset(&solution_out[0], 0, 100U * sizeof(int));

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
  switch (check_feasibility(V_init, A_init, V_max, rtMinusInf, A_max, A_min,
           J_max, J_min)) {
   case 3U:
    varargin_1 = 2;
    break;

   case 5U:
    varargin_1 = 2;
    break;

   case 4U:
    varargin_1 = 3;
    break;

   case 6U:
    varargin_1 = 3;
    break;

   default:
    varargin_1 = 1;
    break;
  }

  varargin_1 = coder::b_eml_sub2ind(static_cast<double>(varargin_1),
    static_cast<double>(!rtIsInf(V_max)) + 1.0, static_cast<double>(!rtIsInf
    (A_max)) + 1.0, static_cast<double>(!rtIsInf(A_min)) + 1.0);
  select_cases_O(nlut[!rtIsNaN(V_wayp) << 1], condlut[varargin_1 - 1],
                 cases_data, cases_size);
  i = cases_size[1];
  for (b_index = 0; b_index < i; b_index++) {
    if (b_index + 1 > cases_size[1]) {
      rtDynamicBoundsError(b_index + 1, 1, cases_size[1], &bb_emlrtBCI);
    }

    i1 = cases_data[b_index];
    if (coder::b_mod(std::floor(static_cast<double>(i1) / 1000.0)) == 0.0) {
      if (i1 - ((i1 >> 1) << 1) == 1) {
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
    } else if (i1 - ((i1 >> 1) << 1) == 1) {
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

    switch (i1) {
     case 0:
      t_test_size[0] = 1;
      t_test_size[1] = 7;
      std::memset(&t_test_data[0], 0, 7U * sizeof(creal_T));
      break;

     case 101:
      abcdefg_O_AVP(P_init, V_init, A_init, rtNaN, V_wayp, rtNaN, V_max, A_max,
                    A_min, J_max, J_min, b_varargin_1);
      t_test_size[0] = 1;
      t_test_size[1] = 7;
      for (i1 = 0; i1 < 7; i1++) {
        t_test_data[i1].re = b_varargin_1[i1];
        t_test_data[i1].im = 0.0;
      }
      break;

     case 102:
      abcdefg_O_AVP(-P_init, -V_init, -A_init, rtNaN, -V_wayp, rtNaN, rtInf,
                    -A_min, -A_max, -J_min, -J_max, b_varargin_1);
      t_test_size[0] = 1;
      t_test_size[1] = 7;
      for (i1 = 0; i1 < 7; i1++) {
        t_test_data[i1].re = b_varargin_1[i1];
        t_test_data[i1].im = 0.0;
      }
      break;

     case 103:
      abcdeg_O_AVP(P_init, V_init, A_init, rtNaN, V_wayp, rtNaN, V_max, A_max,
                   J_max, J_min, dcv);
      t_test_size[0] = 2;
      t_test_size[1] = 7;
      std::copy(&dcv[0], &dcv[14], &t_test_data[0]);
      break;

     case 104:
      abcdeg_O_AVP(-P_init, -V_init, -A_init, rtNaN, -V_wayp, rtNaN, rtInf,
                   -A_min, -J_min, -J_max, dcv);
      t_test_size[0] = 2;
      t_test_size[1] = 7;
      std::copy(&dcv[0], &dcv[14], &t_test_data[0]);
      break;

     case 105:
      abcefg_O_AVP(P_init, V_init, A_init, rtNaN, V_wayp, rtNaN, A_max, A_min,
                   J_max, J_min, dcv);
      t_test_size[0] = 2;
      t_test_size[1] = 7;
      std::copy(&dcv[0], &dcv[14], &t_test_data[0]);
      break;

     case 106:
      abcefg_O_AVP(-P_init, -V_init, -A_init, rtNaN, -V_wayp, rtNaN, -A_min,
                   -A_max, -J_min, -J_max, dcv);
      t_test_size[0] = 2;
      t_test_size[1] = 7;
      std::copy(&dcv[0], &dcv[14], &t_test_data[0]);
      break;

     case 107:
      abceg_O_AVP(P_init, V_init, A_init, rtNaN, V_wayp, rtNaN, A_max, J_max,
                  J_min, dcv1);
      t_test_size[0] = 4;
      t_test_size[1] = 7;
      std::copy(&dcv1[0], &dcv1[28], &t_test_data[0]);
      break;

     case 108:
      abceg_O_AVP(-P_init, -V_init, -A_init, rtNaN, -V_wayp, rtNaN, -A_min,
                  -J_min, -J_max, dcv1);
      t_test_size[0] = 4;
      t_test_size[1] = 7;
      std::copy(&dcv1[0], &dcv1[28], &t_test_data[0]);
      break;

     case 109:
      acdefg_O_AVP(P_init, V_init, A_init, rtNaN, V_wayp, rtNaN, V_max, A_min,
                   J_max, J_min, dcv);
      t_test_size[0] = 2;
      t_test_size[1] = 7;
      std::copy(&dcv[0], &dcv[14], &t_test_data[0]);
      break;

     case 110:
      acdefg_O_AVP(-P_init, -V_init, -A_init, rtNaN, -V_wayp, rtNaN, rtInf,
                   -A_max, -J_min, -J_max, dcv);
      t_test_size[0] = 2;
      t_test_size[1] = 7;
      std::copy(&dcv[0], &dcv[14], &t_test_data[0]);
      break;

     case 111:
      acdeg_O_AVP(P_init, V_init, A_init, rtNaN, V_wayp, rtNaN, V_max, J_max,
                  J_min, dcv1);
      t_test_size[0] = 4;
      t_test_size[1] = 7;
      std::copy(&dcv1[0], &dcv1[28], &t_test_data[0]);
      break;

     case 112:
      acdeg_O_AVP(-P_init, -V_init, -A_init, rtNaN, -V_wayp, rtNaN, rtInf,
                  -J_min, -J_max, dcv1);
      t_test_size[0] = 4;
      t_test_size[1] = 7;
      std::copy(&dcv1[0], &dcv1[28], &t_test_data[0]);
      break;

     case 113:
      acefg_O_AVP(P_init, V_init, A_init, rtNaN, V_wayp, rtNaN, A_min, J_max,
                  J_min, dcv1);
      t_test_size[0] = 4;
      t_test_size[1] = 7;
      std::copy(&dcv1[0], &dcv1[28], &t_test_data[0]);
      break;

     case 114:
      acefg_O_AVP(-P_init, -V_init, -A_init, rtNaN, -V_wayp, rtNaN, -A_max,
                  -J_min, -J_max, dcv1);
      t_test_size[0] = 4;
      t_test_size[1] = 7;
      std::copy(&dcv1[0], &dcv1[28], &t_test_data[0]);
      break;

     case 115:
      aceg_O_AVP(P_init, V_init, A_init, rtNaN, V_wayp, rtNaN, J_max, J_min,
                 t_test_data);
      t_test_size[0] = 5;
      t_test_size[1] = 7;
      break;

     case 116:
      aceg_O_AVP(-P_init, -V_init, -A_init, rtNaN, -V_wayp, rtNaN, -J_min,
                 -J_max, t_test_data);
      t_test_size[0] = 5;
      t_test_size[1] = 7;

      // -----------------------
      break;

     case 201:
      abc_O_VP(P_init, V_init, A_init, rtNaN, V_wayp, A_max, J_max, J_min, dcv1);
      t_test_size[0] = 4;
      t_test_size[1] = 7;
      std::copy(&dcv1[0], &dcv1[28], &t_test_data[0]);
      break;

     case 202:
      abc_O_VP(-P_init, -V_init, -A_init, rtNaN, -V_wayp, -A_min, -J_min, -J_max,
               dcv1);
      t_test_size[0] = 4;
      t_test_size[1] = 7;
      std::copy(&dcv1[0], &dcv1[28], &t_test_data[0]);
      break;

     case 203:
      abcde_O_VP(P_init, V_init, A_init, rtNaN, V_wayp, V_max, A_max, J_max,
                 J_min, dcv);
      t_test_size[0] = 2;
      t_test_size[1] = 7;
      std::copy(&dcv[0], &dcv[14], &t_test_data[0]);
      break;

     case 204:
      abcde_O_VP(-P_init, -V_init, -A_init, rtNaN, -V_wayp, rtInf, -A_min,
                 -J_min, -J_max, dcv);
      t_test_size[0] = 2;
      t_test_size[1] = 7;
      std::copy(&dcv[0], &dcv[14], &t_test_data[0]);
      break;

     case 205:
      abcdef_O_VP(P_init, V_init, A_init, rtNaN, V_wayp, V_max, A_max, A_min,
                  J_max, J_min, b_varargin_1);
      t_test_size[0] = 1;
      t_test_size[1] = 7;
      for (i1 = 0; i1 < 7; i1++) {
        t_test_data[i1].re = b_varargin_1[i1];
        t_test_data[i1].im = 0.0;
      }
      break;

     case 206:
      abcdef_O_VP(-P_init, -V_init, -A_init, rtNaN, -V_wayp, rtInf, -A_min,
                  -A_max, -J_min, -J_max, b_varargin_1);
      t_test_size[0] = 1;
      t_test_size[1] = 7;
      for (i1 = 0; i1 < 7; i1++) {
        t_test_data[i1].re = b_varargin_1[i1];
        t_test_data[i1].im = 0.0;
      }
      break;

     case 207:
      abcdefg_O_VP(P_init, V_init, A_init, rtNaN, V_wayp, V_max, rtMinusInf,
                   A_max, A_min, J_max, J_min, dv);
      t_test_size[0] = 2;
      t_test_size[1] = 7;
      for (i1 = 0; i1 < 14; i1++) {
        t_test_data[i1].re = dv[i1];
        t_test_data[i1].im = 0.0;
      }
      break;

     case 208:
      abcdefg_O_VP(-P_init, -V_init, -A_init, rtNaN, -V_wayp, rtInf, -V_max,
                   -A_min, -A_max, -J_min, -J_max, dv);
      t_test_size[0] = 2;
      t_test_size[1] = 7;
      for (i1 = 0; i1 < 14; i1++) {
        t_test_data[i1].re = dv[i1];
        t_test_data[i1].im = 0.0;
      }
      break;

     case 209:
      abcdeg_O_VP(P_init, V_init, A_init, rtNaN, V_wayp, V_max, rtMinusInf,
                  A_max, J_max, J_min, dcv);
      t_test_size[0] = 2;
      t_test_size[1] = 7;
      std::copy(&dcv[0], &dcv[14], &t_test_data[0]);
      break;

     case 210:
      abcdeg_O_VP(-P_init, -V_init, -A_init, rtNaN, -V_wayp, rtInf, -V_max,
                  -A_min, -J_min, -J_max, dcv);
      t_test_size[0] = 2;
      t_test_size[1] = 7;
      std::copy(&dcv[0], &dcv[14], &t_test_data[0]);
      break;

     case 211:
      abcef_O_VP(P_init, V_init, A_init, rtNaN, V_wayp, A_max, A_min, J_max,
                 J_min, dcv);
      t_test_size[0] = 2;
      t_test_size[1] = 7;
      std::copy(&dcv[0], &dcv[14], &t_test_data[0]);
      break;

     case 212:
      abcef_O_VP(-P_init, -V_init, -A_init, rtNaN, -V_wayp, -A_min, -A_max,
                 -J_min, -J_max, dcv);
      t_test_size[0] = 2;
      t_test_size[1] = 7;
      std::copy(&dcv[0], &dcv[14], &t_test_data[0]);
      break;

     case 213:
      t_test_size[0] = 1;
      t_test_size[1] = 7;
      std::memset(&t_test_data[0], 0, 7U * sizeof(creal_T));
      break;

     case 214:
      t_test_size[0] = 1;
      t_test_size[1] = 7;
      std::memset(&t_test_data[0], 0, 7U * sizeof(creal_T));
      break;

     case 215:
      t_test_size[0] = 1;
      t_test_size[1] = 7;
      std::memset(&t_test_data[0], 0, 7U * sizeof(creal_T));
      break;

     case 216:
      t_test_size[0] = 1;
      t_test_size[1] = 7;
      std::memset(&t_test_data[0], 0, 7U * sizeof(creal_T));
      break;

     case 217:
      t_test_size[0] = 1;
      t_test_size[1] = 7;
      std::memset(&t_test_data[0], 0, 7U * sizeof(creal_T));
      break;

     case 218:
      t_test_size[0] = 1;
      t_test_size[1] = 7;
      std::memset(&t_test_data[0], 0, 7U * sizeof(creal_T));
      break;

     case 219:
      acde_O_VP(P_init, V_init, A_init, rtNaN, V_wayp, V_max, J_max, J_min, dcv);
      t_test_size[0] = 2;
      t_test_size[1] = 7;
      std::copy(&dcv[0], &dcv[14], &t_test_data[0]);
      break;

     case 220:
      acde_O_VP(-P_init, -V_init, -A_init, rtNaN, -V_wayp, rtInf, -J_min, -J_max,
                dcv);
      t_test_size[0] = 2;
      t_test_size[1] = 7;
      std::copy(&dcv[0], &dcv[14], &t_test_data[0]);
      break;

     case 221:
      acdef_O_VP(P_init, V_init, A_init, rtNaN, V_wayp, V_max, A_min, J_max,
                 J_min, dcv);
      t_test_size[0] = 2;
      t_test_size[1] = 7;
      std::copy(&dcv[0], &dcv[14], &t_test_data[0]);
      break;

     case 222:
      acdef_O_VP(-P_init, -V_init, -A_init, rtNaN, -V_wayp, rtInf, -A_max,
                 -J_min, -J_max, dcv);
      t_test_size[0] = 2;
      t_test_size[1] = 7;
      std::copy(&dcv[0], &dcv[14], &t_test_data[0]);
      break;

     case 223:
      acdefg_O_VP(P_init, V_init, A_init, rtNaN, V_wayp, V_max, rtMinusInf,
                  A_min, J_max, J_min, dcv1);
      t_test_size[0] = 4;
      t_test_size[1] = 7;
      std::copy(&dcv1[0], &dcv1[28], &t_test_data[0]);
      break;

     case 224:
      acdefg_O_VP(-P_init, -V_init, -A_init, rtNaN, -V_wayp, rtInf, -V_max,
                  -A_max, -J_min, -J_max, dcv1);
      t_test_size[0] = 4;
      t_test_size[1] = 7;
      std::copy(&dcv1[0], &dcv1[28], &t_test_data[0]);
      break;

     case 225:
      t_test_size[0] = 1;
      t_test_size[1] = 7;
      std::memset(&t_test_data[0], 0, 7U * sizeof(creal_T));
      break;

     case 226:
      t_test_size[0] = 1;
      t_test_size[1] = 7;
      std::memset(&t_test_data[0], 0, 7U * sizeof(creal_T));
      break;

     case 227:
      acef_O_VP(P_init, V_init, A_init, rtNaN, V_wayp, A_min, J_max, J_min, dcv1);
      t_test_size[0] = 4;
      t_test_size[1] = 7;
      std::copy(&dcv1[0], &dcv1[28], &t_test_data[0]);
      break;

     case 228:
      acef_O_VP(-P_init, -V_init, -A_init, rtNaN, -V_wayp, -A_max, -J_min,
                -J_max, dcv1);
      t_test_size[0] = 4;
      t_test_size[1] = 7;
      std::copy(&dcv1[0], &dcv1[28], &t_test_data[0]);
      break;

     case 229:
      t_test_size[0] = 1;
      t_test_size[1] = 7;
      std::memset(&t_test_data[0], 0, 7U * sizeof(creal_T));
      break;

     case 230:
      t_test_size[0] = 1;
      t_test_size[1] = 7;
      std::memset(&t_test_data[0], 0, 7U * sizeof(creal_T));
      break;

     case 231:
      t_test_size[0] = 1;
      t_test_size[1] = 7;
      std::memset(&t_test_data[0], 0, 7U * sizeof(creal_T));
      break;

     case 232:
      t_test_size[0] = 1;
      t_test_size[1] = 7;
      std::memset(&t_test_data[0], 0, 7U * sizeof(creal_T));

      // -----------------------
      break;

     case 301:
      abc_O_AP(P_init, V_init, A_init, rtNaN, rtNaN, A_max, J_max, J_min, dcv);
      t_test_size[0] = 2;
      t_test_size[1] = 7;
      std::copy(&dcv[0], &dcv[14], &t_test_data[0]);
      break;

     case 302:
      abc_O_AP(-P_init, -V_init, -A_init, rtNaN, rtNaN, -A_min, -J_min, -J_max,
               dcv);
      t_test_size[0] = 2;
      t_test_size[1] = 7;
      std::copy(&dcv[0], &dcv[14], &t_test_data[0]);
      break;

     case 303:
      abcde_O_AP(P_init, V_init, A_init, rtNaN, rtNaN, V_max, A_max, J_max,
                 J_min, b_varargin_1);
      t_test_size[0] = 1;
      t_test_size[1] = 7;
      for (i1 = 0; i1 < 7; i1++) {
        t_test_data[i1].re = b_varargin_1[i1];
        t_test_data[i1].im = 0.0;
      }
      break;

     case 304:
      abcde_O_AP(-P_init, -V_init, -A_init, rtNaN, rtNaN, rtInf, -A_min, -J_min,
                 -J_max, b_varargin_1);
      t_test_size[0] = 1;
      t_test_size[1] = 7;
      for (i1 = 0; i1 < 7; i1++) {
        t_test_data[i1].re = b_varargin_1[i1];
        t_test_data[i1].im = 0.0;
      }
      break;

     case 305:
      abcdefg_O_AP(P_init, V_init, A_init, rtNaN, rtNaN, V_max, A_max, A_min,
                   J_max, J_min, b_varargin_1);
      t_test_size[0] = 1;
      t_test_size[1] = 7;
      for (i1 = 0; i1 < 7; i1++) {
        t_test_data[i1].re = b_varargin_1[i1];
        t_test_data[i1].im = 0.0;
      }
      break;

     case 306:
      abcdefg_O_AP(-P_init, -V_init, -A_init, rtNaN, rtNaN, rtInf, -A_min,
                   -A_max, -J_min, -J_max, b_varargin_1);
      t_test_size[0] = 1;
      t_test_size[1] = 7;
      for (i1 = 0; i1 < 7; i1++) {
        t_test_data[i1].re = b_varargin_1[i1];
        t_test_data[i1].im = 0.0;
      }
      break;

     case 307:
      abcefg_O_AP(P_init, V_init, A_init, rtNaN, rtNaN, V_max, A_max, A_min,
                  J_max, J_min, dcv);
      t_test_size[0] = 2;
      t_test_size[1] = 7;
      std::copy(&dcv[0], &dcv[14], &t_test_data[0]);
      break;

     case 308:
      abcefg_O_AP(-P_init, -V_init, -A_init, rtNaN, rtNaN, rtInf, -A_min, -A_max,
                  -J_min, -J_max, dcv);
      t_test_size[0] = 2;
      t_test_size[1] = 7;
      std::copy(&dcv[0], &dcv[14], &t_test_data[0]);
      break;

     case 309:
      abceg_O_AP(P_init, V_init, A_init, rtNaN, rtNaN, V_max, A_max, J_max,
                 J_min, dcv1);
      t_test_size[0] = 4;
      t_test_size[1] = 7;
      std::copy(&dcv1[0], &dcv1[28], &t_test_data[0]);
      break;

     case 310:
      abceg_O_AP(-P_init, -V_init, -A_init, rtNaN, rtNaN, rtInf, -A_min, -J_min,
                 -J_max, dcv1);
      t_test_size[0] = 4;
      t_test_size[1] = 7;
      std::copy(&dcv1[0], &dcv1[28], &t_test_data[0]);
      break;

     case 311:
      ac_O_AP(P_init, V_init, A_init, rtNaN, rtNaN, J_max, J_min, dcv2);
      t_test_size[0] = 3;
      t_test_size[1] = 7;
      std::copy(&dcv2[0], &dcv2[21], &t_test_data[0]);
      break;

     case 312:
      ac_O_AP(-P_init, -V_init, -A_init, rtNaN, rtNaN, -J_min, -J_max, dcv2);
      t_test_size[0] = 3;
      t_test_size[1] = 7;
      std::copy(&dcv2[0], &dcv2[21], &t_test_data[0]);
      break;

     case 313:
      acde_O_AP(P_init, V_init, A_init, rtNaN, rtNaN, V_max, J_max, J_min, dcv);
      t_test_size[0] = 2;
      t_test_size[1] = 7;
      std::copy(&dcv[0], &dcv[14], &t_test_data[0]);
      break;

     case 314:
      acde_O_AP(-P_init, -V_init, -A_init, rtNaN, rtNaN, rtInf, -J_min, -J_max,
                dcv);
      t_test_size[0] = 2;
      t_test_size[1] = 7;
      std::copy(&dcv[0], &dcv[14], &t_test_data[0]);
      break;

     case 315:
      acdefg_O_AP(P_init, V_init, A_init, rtNaN, rtNaN, V_max, A_min, J_max,
                  J_min, dcv);
      t_test_size[0] = 2;
      t_test_size[1] = 7;
      std::copy(&dcv[0], &dcv[14], &t_test_data[0]);
      break;

     case 316:
      acdefg_O_AP(-P_init, -V_init, -A_init, rtNaN, rtNaN, rtInf, -A_max, -J_min,
                  -J_max, dcv);
      t_test_size[0] = 2;
      t_test_size[1] = 7;
      std::copy(&dcv[0], &dcv[14], &t_test_data[0]);
      break;

     case 317:
      acefg_O_AP(P_init, V_init, A_init, rtNaN, rtNaN, V_max, A_min, J_max,
                 J_min, dcv1);
      t_test_size[0] = 4;
      t_test_size[1] = 7;
      std::copy(&dcv1[0], &dcv1[28], &t_test_data[0]);
      break;

     case 318:
      acefg_O_AP(-P_init, -V_init, -A_init, rtNaN, rtNaN, rtInf, -A_max, -J_min,
                 -J_max, dcv1);
      t_test_size[0] = 4;
      t_test_size[1] = 7;
      std::copy(&dcv1[0], &dcv1[28], &t_test_data[0]);
      break;

     case 319:
      aceg_O_AP(P_init, V_init, A_init, rtNaN, rtNaN, V_max, J_max, J_min, dcv1);
      t_test_size[0] = 4;
      t_test_size[1] = 7;
      std::copy(&dcv1[0], &dcv1[28], &t_test_data[0]);
      break;

     case 320:
      aceg_O_AP(-P_init, -V_init, -A_init, rtNaN, rtNaN, rtInf, -J_min, -J_max,
                dcv1);
      t_test_size[0] = 4;
      t_test_size[1] = 7;
      std::copy(&dcv1[0], &dcv1[28], &t_test_data[0]);
      break;

     case 321:
      abcdeg_O_AP(P_init, V_init, A_init, rtNaN, rtNaN, V_max, A_max, J_max,
                  J_min, dv);
      t_test_size[0] = 2;
      t_test_size[1] = 7;
      for (i1 = 0; i1 < 14; i1++) {
        t_test_data[i1].re = dv[i1];
        t_test_data[i1].im = 0.0;
      }
      break;

     case 322:
      abcdeg_O_AP(-P_init, -V_init, -A_init, rtNaN, rtNaN, rtInf, -A_min, -J_min,
                  -J_max, dv);
      t_test_size[0] = 2;
      t_test_size[1] = 7;
      for (i1 = 0; i1 < 14; i1++) {
        t_test_data[i1].re = dv[i1];
        t_test_data[i1].im = 0.0;
      }
      break;

     case 323:
      acdeg_O_AP(P_init, V_init, A_init, rtNaN, rtNaN, V_max, J_max, J_min, dcv);
      t_test_size[0] = 2;
      t_test_size[1] = 7;
      std::copy(&dcv[0], &dcv[14], &t_test_data[0]);
      break;

     case 324:
      acdeg_O_AP(-P_init, -V_init, -A_init, rtNaN, rtNaN, rtInf, -J_min, -J_max,
                 dcv);
      t_test_size[0] = 2;
      t_test_size[1] = 7;
      std::copy(&dcv[0], &dcv[14], &t_test_data[0]);

      // -----------------------
      break;

     case 401:
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
      //  Generated on 28-Aug-2019 17:25:45
      b_A_init[0].re = -(A_init - A_max) / J_max;
      b_A_init[0].im = 0.0;
      b_A_init[1].re = rtNaN;
      b_A_init[1].im = 0.0;
      b_A_init[2].re = rtNaN;
      b_A_init[2].im = 0.0;
      b_A_init[3].re = 0.0;
      b_A_init[3].im = 0.0;
      b_A_init[4].re = 0.0;
      b_A_init[4].im = 0.0;
      b_A_init[5].re = 0.0;
      b_A_init[5].im = 0.0;
      b_A_init[6].re = 0.0;
      b_A_init[6].im = 0.0;
      t_test_size[0] = 1;
      t_test_size[1] = 7;
      std::copy(&b_A_init[0], &b_A_init[7], &t_test_data[0]);
      break;

     case 402:
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
      //  Generated on 28-Aug-2019 17:25:45
      b_A_init[0].re = -(-A_init - (-A_min)) / -J_min;
      b_A_init[0].im = 0.0;
      b_A_init[1].re = rtNaN;
      b_A_init[1].im = 0.0;
      b_A_init[2].re = rtNaN;
      b_A_init[2].im = 0.0;
      b_A_init[3].re = 0.0;
      b_A_init[3].im = 0.0;
      b_A_init[4].re = 0.0;
      b_A_init[4].im = 0.0;
      b_A_init[5].re = 0.0;
      b_A_init[5].im = 0.0;
      b_A_init[6].re = 0.0;
      b_A_init[6].im = 0.0;
      t_test_size[0] = 1;
      t_test_size[1] = 7;
      std::copy(&b_A_init[0], &b_A_init[7], &t_test_data[0]);
      break;

     case 403:
      ac_O_AV(V_init, A_init, V_wayp, rtNaN, J_max, J_min, dcv);
      t_test_size[0] = 2;
      t_test_size[1] = 7;
      std::copy(&dcv[0], &dcv[14], &t_test_data[0]);
      break;

     case 404:
      ac_O_AV(-V_init, -A_init, -V_wayp, rtNaN, -J_min, -J_max, dcv);
      t_test_size[0] = 2;
      t_test_size[1] = 7;
      std::copy(&dcv[0], &dcv[14], &t_test_data[0]);
      break;

     case 501:
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
      //  Generated on 28-Aug-2019 17:25:45
      b_A_init[0].re = rtNaN;
      b_A_init[0].im = 0.0;
      b_A_init[1].re = 0.0;
      b_A_init[1].im = 0.0;
      b_A_init[2].re = 0.0;
      b_A_init[2].im = 0.0;
      b_A_init[3].re = 0.0;
      b_A_init[3].im = 0.0;
      b_A_init[4].re = 0.0;
      b_A_init[4].im = 0.0;
      b_A_init[5].re = 0.0;
      b_A_init[5].im = 0.0;
      b_A_init[6].re = 0.0;
      b_A_init[6].im = 0.0;
      t_test_size[0] = 1;
      t_test_size[1] = 7;
      std::copy(&b_A_init[0], &b_A_init[7], &t_test_data[0]);
      break;

     case 502:
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
      //  Generated on 28-Aug-2019 17:25:45
      b_A_init[0].re = rtNaN;
      b_A_init[0].im = 0.0;
      b_A_init[1].re = 0.0;
      b_A_init[1].im = 0.0;
      b_A_init[2].re = 0.0;
      b_A_init[2].im = 0.0;
      b_A_init[3].re = 0.0;
      b_A_init[3].im = 0.0;
      b_A_init[4].re = 0.0;
      b_A_init[4].im = 0.0;
      b_A_init[5].re = 0.0;
      b_A_init[5].im = 0.0;
      b_A_init[6].re = 0.0;
      b_A_init[6].im = 0.0;
      t_test_size[0] = 1;
      t_test_size[1] = 7;
      std::copy(&b_A_init[0], &b_A_init[7], &t_test_data[0]);
      break;

     case 503:
      abc_O_A(V_init, A_init, rtNaN, rtMinusInf, A_max, J_max, J_min,
              b_varargin_1);
      t_test_size[0] = 1;
      t_test_size[1] = 7;
      for (i1 = 0; i1 < 7; i1++) {
        t_test_data[i1].re = b_varargin_1[i1];
        t_test_data[i1].im = 0.0;
      }
      break;

     case 504:
      abc_O_A(-V_init, -A_init, rtNaN, -V_max, -A_min, -J_min, -J_max,
              b_varargin_1);
      t_test_size[0] = 1;
      t_test_size[1] = 7;
      for (i1 = 0; i1 < 7; i1++) {
        t_test_data[i1].re = b_varargin_1[i1];
        t_test_data[i1].im = 0.0;
      }
      break;

     case 505:
      ac_O_A(V_init, A_init, rtNaN, rtMinusInf, J_max, J_min, dcv);
      t_test_size[0] = 2;
      t_test_size[1] = 7;
      std::copy(&dcv[0], &dcv[14], &t_test_data[0]);
      break;

     case 506:
      ac_O_A(-V_init, -A_init, rtNaN, -V_max, -J_min, -J_max, dcv);
      t_test_size[0] = 2;
      t_test_size[1] = 7;
      std::copy(&dcv[0], &dcv[14], &t_test_data[0]);

      // -----------------------
      break;

     case 601:
      a_O_V(V_init, A_init, V_wayp, J_max, dcv);
      t_test_size[0] = 2;
      t_test_size[1] = 7;
      std::copy(&dcv[0], &dcv[14], &t_test_data[0]);
      break;

     case 602:
      a_O_V(-V_init, -A_init, -V_wayp, -J_min, dcv);
      t_test_size[0] = 2;
      t_test_size[1] = 7;
      std::copy(&dcv[0], &dcv[14], &t_test_data[0]);
      break;

     case 603:
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
      //  Generated on 28-Aug-2019 17:25:45
      b_A_init[0].re = -(A_init - A_max) / J_max;
      b_A_init[0].im = 0.0;
      b_A_init[1].re = (((J_max * V_init * 2.0 - J_max * V_wayp * 2.0) - A_init *
                         A_init) + A_max * A_max) * -0.5 / (A_max * J_max);
      b_A_init[1].im = 0.0;
      b_A_init[2].re = 0.0;
      b_A_init[2].im = 0.0;
      b_A_init[3].re = 0.0;
      b_A_init[3].im = 0.0;
      b_A_init[4].re = 0.0;
      b_A_init[4].im = 0.0;
      b_A_init[5].re = 0.0;
      b_A_init[5].im = 0.0;
      b_A_init[6].re = 0.0;
      b_A_init[6].im = 0.0;
      t_test_size[0] = 1;
      t_test_size[1] = 7;
      std::copy(&b_A_init[0], &b_A_init[7], &t_test_data[0]);
      break;

     case 604:
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
      //  Generated on 28-Aug-2019 17:25:45
      b_A_init[0].re = -(-A_init - (-A_min)) / -J_min;
      b_A_init[0].im = 0.0;
      b_A_init[1].re = (((-J_min * -V_init * 2.0 - -J_min * -V_wayp * 2.0) -
                         -A_init * -A_init) + -A_min * -A_min) * -0.5 / (-A_min *
        -J_min);
      b_A_init[1].im = 0.0;
      b_A_init[2].re = 0.0;
      b_A_init[2].im = 0.0;
      b_A_init[3].re = 0.0;
      b_A_init[3].im = 0.0;
      b_A_init[4].re = 0.0;
      b_A_init[4].im = 0.0;
      b_A_init[5].re = 0.0;
      b_A_init[5].im = 0.0;
      b_A_init[6].re = 0.0;
      b_A_init[6].im = 0.0;
      t_test_size[0] = 1;
      t_test_size[1] = 7;
      std::copy(&b_A_init[0], &b_A_init[7], &t_test_data[0]);
      break;

     case 605:
      abc_O_V(V_init, A_init, V_wayp, V_max, A_max, J_max, J_min, dcv);
      t_test_size[0] = 2;
      t_test_size[1] = 7;
      std::copy(&dcv[0], &dcv[14], &t_test_data[0]);
      break;

     case 606:
      abc_O_V(-V_init, -A_init, -V_wayp, rtInf, -A_min, -J_min, -J_max, dcv);
      t_test_size[0] = 2;
      t_test_size[1] = 7;
      std::copy(&dcv[0], &dcv[14], &t_test_data[0]);
      break;

     case 607:
      ac_O_V(V_init, A_init, V_wayp, V_max, J_max, J_min, dcv);
      t_test_size[0] = 2;
      t_test_size[1] = 7;
      std::copy(&dcv[0], &dcv[14], &t_test_data[0]);
      break;

     case 608:
      ac_O_V(-V_init, -A_init, -V_wayp, rtInf, -J_min, -J_max, dcv);
      t_test_size[0] = 2;
      t_test_size[1] = 7;
      std::copy(&dcv[0], &dcv[14], &t_test_data[0]);

      // -----------------------
      break;

     case 701:
      a_O_P(P_init, V_init, A_init, rtNaN, J_max, dcv2);
      t_test_size[0] = 3;
      t_test_size[1] = 7;
      std::copy(&dcv2[0], &dcv2[21], &t_test_data[0]);
      break;

     case 702:
      a_O_P(-P_init, -V_init, -A_init, rtNaN, -J_min, dcv2);
      t_test_size[0] = 3;
      t_test_size[1] = 7;
      std::copy(&dcv2[0], &dcv2[21], &t_test_data[0]);
      break;

     case 703:
      ab_O_P(P_init, V_init, A_init, rtNaN, A_max, J_max, dcv);
      t_test_size[0] = 2;
      t_test_size[1] = 7;
      std::copy(&dcv[0], &dcv[14], &t_test_data[0]);
      break;

     case 704:
      ab_O_P(-P_init, -V_init, -A_init, rtNaN, -A_min, -J_min, dcv);
      t_test_size[0] = 2;
      t_test_size[1] = 7;
      std::copy(&dcv[0], &dcv[14], &t_test_data[0]);
      break;

     case 705:
      abc_O_P(P_init, V_init, A_init, rtNaN, V_max, A_max, J_max, J_min, dcv2);
      t_test_size[0] = 3;
      t_test_size[1] = 7;
      std::copy(&dcv2[0], &dcv2[21], &t_test_data[0]);
      break;

     case 706:
      abc_O_P(-P_init, -V_init, -A_init, rtNaN, rtInf, -A_min, -J_min, -J_max,
              dcv2);
      t_test_size[0] = 3;
      t_test_size[1] = 7;
      std::copy(&dcv2[0], &dcv2[21], &t_test_data[0]);
      break;

     case 707:
      abcd_O_P(P_init, V_init, A_init, rtNaN, V_max, A_max, J_max, J_min,
               b_varargin_1);
      t_test_size[0] = 1;
      t_test_size[1] = 7;
      for (i1 = 0; i1 < 7; i1++) {
        t_test_data[i1].re = b_varargin_1[i1];
        t_test_data[i1].im = 0.0;
      }
      break;

     case 708:
      abcd_O_P(-P_init, -V_init, -A_init, rtNaN, rtInf, -A_min, -J_min, -J_max,
               b_varargin_1);
      t_test_size[0] = 1;
      t_test_size[1] = 7;
      for (i1 = 0; i1 < 7; i1++) {
        t_test_data[i1].re = b_varargin_1[i1];
        t_test_data[i1].im = 0.0;
      }
      break;

     case 709:
      ac_O_P(P_init, V_init, A_init, rtNaN, V_max, J_max, J_min, dcv2);
      t_test_size[0] = 3;
      t_test_size[1] = 7;
      std::copy(&dcv2[0], &dcv2[21], &t_test_data[0]);
      break;

     case 710:
      ac_O_P(-P_init, -V_init, -A_init, rtNaN, rtInf, -J_min, -J_max, dcv2);
      t_test_size[0] = 3;
      t_test_size[1] = 7;
      std::copy(&dcv2[0], &dcv2[21], &t_test_data[0]);
      break;

     case 711:
      acd_O_P(P_init, V_init, A_init, rtNaN, V_max, J_max, J_min, dcv);
      t_test_size[0] = 2;
      t_test_size[1] = 7;
      std::copy(&dcv[0], &dcv[14], &t_test_data[0]);
      break;

     case 712:
      acd_O_P(-P_init, -V_init, -A_init, rtNaN, rtInf, -J_min, -J_max, dcv);
      t_test_size[0] = 2;
      t_test_size[1] = 7;
      std::copy(&dcv[0], &dcv[14], &t_test_data[0]);

      // -----------------------
      break;

     case 1101:
      abcdefg_NO_AVP(P_init, V_init, A_init, rtNaN, V_wayp, rtNaN, V_max, A_min,
                     J_max, J_min, b_varargin_1);
      t_test_size[0] = 1;
      t_test_size[1] = 7;
      for (i1 = 0; i1 < 7; i1++) {
        t_test_data[i1].re = b_varargin_1[i1];
        t_test_data[i1].im = 0.0;
      }
      break;

     case 1102:
      abcdefg_NO_AVP(-P_init, -V_init, -A_init, rtNaN, -V_wayp, rtNaN, rtInf,
                     -A_max, -J_min, -J_max, b_varargin_1);
      t_test_size[0] = 1;
      t_test_size[1] = 7;
      for (i1 = 0; i1 < 7; i1++) {
        t_test_data[i1].re = b_varargin_1[i1];
        t_test_data[i1].im = 0.0;
      }
      break;

     case 1103:
      abcdeg_NO_AVP(P_init, V_init, A_init, rtNaN, V_wayp, rtNaN, V_max, A_min,
                    J_max, J_min, dcv);
      t_test_size[0] = 2;
      t_test_size[1] = 7;
      std::copy(&dcv[0], &dcv[14], &t_test_data[0]);
      break;

     case 1104:
      abcdeg_NO_AVP(-P_init, -V_init, -A_init, rtNaN, -V_wayp, rtNaN, rtInf,
                    -A_max, -J_min, -J_max, dcv);
      t_test_size[0] = 2;
      t_test_size[1] = 7;
      std::copy(&dcv[0], &dcv[14], &t_test_data[0]);
      break;

     case 1105:
      acdefg_NO_AVP(P_init, V_init, A_init, rtNaN, V_wayp, rtNaN, V_max, A_min,
                    J_max, J_min, dcv);
      t_test_size[0] = 2;
      t_test_size[1] = 7;
      std::copy(&dcv[0], &dcv[14], &t_test_data[0]);
      break;

     case 1106:
      acdefg_NO_AVP(-P_init, -V_init, -A_init, rtNaN, -V_wayp, rtNaN, rtInf,
                    -A_max, -J_min, -J_max, dcv);
      t_test_size[0] = 2;
      t_test_size[1] = 7;
      std::copy(&dcv[0], &dcv[14], &t_test_data[0]);
      break;

     case 1107:
      acdeg_NO_AVP(P_init, V_init, A_init, rtNaN, V_wayp, rtNaN, V_max, J_max,
                   J_min, dcv1);
      t_test_size[0] = 4;
      t_test_size[1] = 7;
      std::copy(&dcv1[0], &dcv1[28], &t_test_data[0]);
      break;

     case 1108:
      acdeg_NO_AVP(-P_init, -V_init, -A_init, rtNaN, -V_wayp, rtNaN, rtInf,
                   -J_min, -J_max, dcv1);
      t_test_size[0] = 4;
      t_test_size[1] = 7;
      std::copy(&dcv1[0], &dcv1[28], &t_test_data[0]);

      // -----------------------
      break;

     case 1201:
      abcde_NO_VP(P_init, V_init, A_init, rtNaN, V_wayp, V_max, A_min, J_max,
                  J_min, dcv);
      t_test_size[0] = 2;
      t_test_size[1] = 7;
      std::copy(&dcv[0], &dcv[14], &t_test_data[0]);
      break;

     case 1202:
      abcde_NO_VP(-P_init, -V_init, -A_init, rtNaN, -V_wayp, rtInf, -A_max,
                  -J_min, -J_max, dcv);
      t_test_size[0] = 2;
      t_test_size[1] = 7;
      std::copy(&dcv[0], &dcv[14], &t_test_data[0]);
      break;

     case 1203:
      abcdef_NO_VP(P_init, V_init, A_init, rtNaN, V_wayp, V_max, A_min, J_max,
                   J_min, b_varargin_1);
      t_test_size[0] = 1;
      t_test_size[1] = 7;
      for (i1 = 0; i1 < 7; i1++) {
        t_test_data[i1].re = b_varargin_1[i1];
        t_test_data[i1].im = 0.0;
      }
      break;

     case 1204:
      abcdef_NO_VP(-P_init, -V_init, -A_init, rtNaN, -V_wayp, rtInf, -A_max,
                   -J_min, -J_max, b_varargin_1);
      t_test_size[0] = 1;
      t_test_size[1] = 7;
      for (i1 = 0; i1 < 7; i1++) {
        t_test_data[i1].re = b_varargin_1[i1];
        t_test_data[i1].im = 0.0;
      }
      break;

     case 1205:
      abcdefg_NO_VP(P_init, V_init, A_init, rtNaN, V_wayp, V_max, rtMinusInf,
                    A_min, J_max, J_min, dv);
      t_test_size[0] = 2;
      t_test_size[1] = 7;
      for (i1 = 0; i1 < 14; i1++) {
        t_test_data[i1].re = dv[i1];
        t_test_data[i1].im = 0.0;
      }
      break;

     case 1206:
      abcdefg_NO_VP(-P_init, -V_init, -A_init, rtNaN, -V_wayp, rtInf, -V_max,
                    -A_max, -J_min, -J_max, dv);
      t_test_size[0] = 2;
      t_test_size[1] = 7;
      for (i1 = 0; i1 < 14; i1++) {
        t_test_data[i1].re = dv[i1];
        t_test_data[i1].im = 0.0;
      }
      break;

     case 1207:
      abcdeg_NO_VP(P_init, V_init, A_init, rtNaN, V_wayp, V_max, rtMinusInf,
                   A_min, J_max, J_min, dcv);
      t_test_size[0] = 2;
      t_test_size[1] = 7;
      std::copy(&dcv[0], &dcv[14], &t_test_data[0]);
      break;

     case 1208:
      abcdeg_NO_VP(-P_init, -V_init, -A_init, rtNaN, -V_wayp, rtInf, -V_max,
                   -A_max, -J_min, -J_max, dcv);
      t_test_size[0] = 2;
      t_test_size[1] = 7;
      std::copy(&dcv[0], &dcv[14], &t_test_data[0]);
      break;

     case 1209:
      acde_NO_VP(P_init, V_init, A_init, rtNaN, V_wayp, V_max, J_max, J_min, dcv);
      t_test_size[0] = 2;
      t_test_size[1] = 7;
      std::copy(&dcv[0], &dcv[14], &t_test_data[0]);
      break;

     case 1210:
      acde_NO_VP(-P_init, -V_init, -A_init, rtNaN, -V_wayp, rtInf, -J_min,
                 -J_max, dcv);
      t_test_size[0] = 2;
      t_test_size[1] = 7;
      std::copy(&dcv[0], &dcv[14], &t_test_data[0]);
      break;

     case 1211:
      acdef_NO_VP(P_init, V_init, A_init, rtNaN, V_wayp, V_max, A_min, J_max,
                  J_min, dcv);
      t_test_size[0] = 2;
      t_test_size[1] = 7;
      std::copy(&dcv[0], &dcv[14], &t_test_data[0]);
      break;

     case 1212:
      acdef_NO_VP(-P_init, -V_init, -A_init, rtNaN, -V_wayp, rtInf, -A_max,
                  -J_min, -J_max, dcv);
      t_test_size[0] = 2;
      t_test_size[1] = 7;
      std::copy(&dcv[0], &dcv[14], &t_test_data[0]);
      break;

     case 1213:
      acdefg_NO_VP(P_init, V_init, A_init, rtNaN, V_wayp, V_max, rtMinusInf,
                   A_min, J_max, J_min, dcv1);
      t_test_size[0] = 4;
      t_test_size[1] = 7;
      std::copy(&dcv1[0], &dcv1[28], &t_test_data[0]);
      break;

     case 1214:
      acdefg_NO_VP(-P_init, -V_init, -A_init, rtNaN, -V_wayp, rtInf, -V_max,
                   -A_max, -J_min, -J_max, dcv1);
      t_test_size[0] = 4;
      t_test_size[1] = 7;
      std::copy(&dcv1[0], &dcv1[28], &t_test_data[0]);
      break;

     case 1215:
      t_test_size[0] = 1;
      t_test_size[1] = 7;
      std::memset(&t_test_data[0], 0, 7U * sizeof(creal_T));
      break;

     case 1216:
      t_test_size[0] = 1;
      t_test_size[1] = 7;
      std::memset(&t_test_data[0], 0, 7U * sizeof(creal_T));

      // -----------------------
      break;

     case 1301:
      abcde_NO_AP(P_init, V_init, A_init, rtNaN, rtNaN, V_max, A_min, J_max,
                  J_min, b_varargin_1);
      t_test_size[0] = 1;
      t_test_size[1] = 7;
      for (i1 = 0; i1 < 7; i1++) {
        t_test_data[i1].re = b_varargin_1[i1];
        t_test_data[i1].im = 0.0;
      }
      break;

     case 1302:
      abcde_NO_AP(-P_init, -V_init, -A_init, rtNaN, rtNaN, rtInf, -A_max, -J_min,
                  -J_max, b_varargin_1);
      t_test_size[0] = 1;
      t_test_size[1] = 7;
      for (i1 = 0; i1 < 7; i1++) {
        t_test_data[i1].re = b_varargin_1[i1];
        t_test_data[i1].im = 0.0;
      }
      break;

     case 1303:
      acde_NO_AP(P_init, V_init, A_init, rtNaN, rtNaN, V_max, J_max, J_min, dcv);
      t_test_size[0] = 2;
      t_test_size[1] = 7;
      std::copy(&dcv[0], &dcv[14], &t_test_data[0]);
      break;

     case 1304:
      acde_NO_AP(-P_init, -V_init, -A_init, rtNaN, rtNaN, rtInf, -J_min, -J_max,
                 dcv);
      t_test_size[0] = 2;
      t_test_size[1] = 7;
      std::copy(&dcv[0], &dcv[14], &t_test_data[0]);
      break;

     case 1305:
      abcdefg_NO_AP(P_init, V_init, A_init, rtNaN, rtNaN, V_max, A_min, J_max,
                    J_min, b_varargin_1);
      t_test_size[0] = 1;
      t_test_size[1] = 7;
      for (i1 = 0; i1 < 7; i1++) {
        t_test_data[i1].re = b_varargin_1[i1];
        t_test_data[i1].im = 0.0;
      }
      break;

     case 1306:
      abcdefg_NO_AP(-P_init, -V_init, -A_init, rtNaN, rtNaN, rtInf, -A_max,
                    -J_min, -J_max, b_varargin_1);
      t_test_size[0] = 1;
      t_test_size[1] = 7;
      for (i1 = 0; i1 < 7; i1++) {
        t_test_data[i1].re = b_varargin_1[i1];
        t_test_data[i1].im = 0.0;
      }
      break;

     case 1307:
      acdefg_NO_AP(P_init, V_init, A_init, rtNaN, rtNaN, V_max, A_min, J_max,
                   J_min, dcv);
      t_test_size[0] = 2;
      t_test_size[1] = 7;
      std::copy(&dcv[0], &dcv[14], &t_test_data[0]);
      break;

     case 1308:
      acdefg_NO_AP(-P_init, -V_init, -A_init, rtNaN, rtNaN, rtInf, -A_max,
                   -J_min, -J_max, dcv);
      t_test_size[0] = 2;
      t_test_size[1] = 7;
      std::copy(&dcv[0], &dcv[14], &t_test_data[0]);
      break;

     case 1309:
      abcdeg_NO_AP(P_init, V_init, A_init, rtNaN, rtNaN, V_max, A_min, J_max,
                   J_min, dv);
      t_test_size[0] = 2;
      t_test_size[1] = 7;
      for (i1 = 0; i1 < 14; i1++) {
        t_test_data[i1].re = dv[i1];
        t_test_data[i1].im = 0.0;
      }
      break;

     case 1310:
      abcdeg_NO_AP(-P_init, -V_init, -A_init, rtNaN, rtNaN, rtInf, -A_max,
                   -J_min, -J_max, dv);
      t_test_size[0] = 2;
      t_test_size[1] = 7;
      for (i1 = 0; i1 < 14; i1++) {
        t_test_data[i1].re = dv[i1];
        t_test_data[i1].im = 0.0;
      }
      break;

     case 1311:
      acdeg_NO_AP(P_init, V_init, A_init, rtNaN, rtNaN, V_max, J_max, J_min, dcv);
      t_test_size[0] = 2;
      t_test_size[1] = 7;
      std::copy(&dcv[0], &dcv[14], &t_test_data[0]);
      break;

     case 1312:
      acdeg_NO_AP(-P_init, -V_init, -A_init, rtNaN, rtNaN, rtInf, -J_min, -J_max,
                  dcv);
      t_test_size[0] = 2;
      t_test_size[1] = 7;
      std::copy(&dcv[0], &dcv[14], &t_test_data[0]);

      // -----------------------
      break;

     case 1701:
      abcd_NO_P(P_init, V_init, A_init, rtNaN, V_max, A_min, J_max, J_min,
                b_varargin_1);
      t_test_size[0] = 1;
      t_test_size[1] = 7;
      for (i1 = 0; i1 < 7; i1++) {
        t_test_data[i1].re = b_varargin_1[i1];
        t_test_data[i1].im = 0.0;
      }
      break;

     case 1702:
      abcd_NO_P(-P_init, -V_init, -A_init, rtNaN, rtInf, -A_max, -J_min, -J_max,
                b_varargin_1);
      t_test_size[0] = 1;
      t_test_size[1] = 7;
      for (i1 = 0; i1 < 7; i1++) {
        t_test_data[i1].re = b_varargin_1[i1];
        t_test_data[i1].im = 0.0;
      }
      break;

     case 1703:
      acd_NO_P(P_init, V_init, A_init, rtNaN, V_max, J_max, J_min, dcv);
      t_test_size[0] = 2;
      t_test_size[1] = 7;
      std::copy(&dcv[0], &dcv[14], &t_test_data[0]);
      break;

     case 1704:
      acd_NO_P(-P_init, -V_init, -A_init, rtNaN, rtInf, -J_min, -J_max, dcv);
      t_test_size[0] = 2;
      t_test_size[1] = 7;
      std::copy(&dcv[0], &dcv[14], &t_test_data[0]);
      break;

     default:
      t_test_size[0] = 1;
      t_test_size[1] = 7;
      std::memset(&t_test_data[0], 0, 7U * sizeof(creal_T));
      printf("Error: Solution prior ");
      fflush(stdout);
      printint(i1);
      printf(" is not valid!\n");
      fflush(stdout);
      break;
    }

    b_check(t_test_data, t_test_size, J_test, V_init, A_init, V_wayp, V_max,
            A_max, A_min, J_max, J_min, valid_data, valid_size);
    i1 = valid_size[1];
    for (int index_t = 0; index_t < i1; index_t++) {
      if (index_t + 1 > valid_size[1]) {
        rtDynamicBoundsError(index_t + 1, 1, valid_size[1], &eb_emlrtBCI);
      }

      if (valid_data[index_t]) {
        bool b_existing;
        bool exitg1;
        if (index_t + 1 > t_test_size[0]) {
          rtDynamicBoundsError(index_t + 1, 1, t_test_size[0], &cb_emlrtBCI);
        }

        for (varargin_1 = 0; varargin_1 < 7; varargin_1++) {
          double d;
          d = t_test_data[index_t + t_test_size[0] * varargin_1].re;
          if (d > 0.0) {
            t_test_valid[varargin_1] = d;
          } else {
            t_test_valid[varargin_1] = 0.0;
          }
        }

        b_existing = false;
        idx = 0;
        exitg1 = false;
        while ((!exitg1) && (idx <= num_valid - 1)) {
          simplify_setp(t_test_valid, J_test, t_test_simpl_data,
                        t_test_simpl_size, J_test_simpl_data, J_test_simpl_size);
          for (i2 = 0; i2 < 7; i2++) {
            varargin_1 = idx + 100 * i2;
            b_varargin_1[i2] = t_out[varargin_1];
            b_J_out[i2] = J_out[varargin_1];
          }

          simplify_setp(b_varargin_1, b_J_out, t_out_simpl_data,
                        t_out_simpl_size, J_out_simpl_data, J_out_simpl_size);

          // numerical
          // numerical
          if ((t_test_simpl_size[1] == t_out_simpl_size[1]) &&
              (J_test_simpl_size[1] == J_out_simpl_size[1])) {
            rtSizeEqNDCheck(&t_test_simpl_size[0], &t_out_simpl_size[0],
                            &emlrtECI);
            b_t_test_simpl_size[0] = 1;
            b_t_test_simpl_size[1] = t_test_simpl_size[1];
            loop_ub = t_test_simpl_size[1];
            for (i2 = 0; i2 < loop_ub; i2++) {
              b_varargin_1[i2] = t_test_simpl_data[i2] - t_out_simpl_data[i2];
            }

            coder::b_abs(b_varargin_1, b_t_test_simpl_size, t_test_simpl_data,
                         t_test_simpl_size);
            c_t_test_simpl_size[0] = 1;
            c_t_test_simpl_size[1] = t_test_simpl_size[1];
            loop_ub = t_test_simpl_size[1];
            for (i2 = 0; i2 < loop_ub; i2++) {
              b_t_test_simpl_data[i2] = (t_test_simpl_data[i2] < 1.0E-7);
            }

            if (coder::all(b_t_test_simpl_data, c_t_test_simpl_size)) {
              rtSizeEqNDCheck(&J_test_simpl_size[0], &J_out_simpl_size[0],
                              &b_emlrtECI);
              t_out_simpl_size[0] = 1;
              t_out_simpl_size[1] = J_test_simpl_size[1];
              loop_ub = J_test_simpl_size[1];
              for (i2 = 0; i2 < loop_ub; i2++) {
                t_out_simpl_data[i2] = J_test_simpl_data[i2] -
                  J_out_simpl_data[i2];
              }

              coder::b_abs(t_out_simpl_data, t_out_simpl_size, t_test_simpl_data,
                           t_test_simpl_size);
              c_t_test_simpl_size[0] = 1;
              c_t_test_simpl_size[1] = t_test_simpl_size[1];
              loop_ub = t_test_simpl_size[1];
              for (i2 = 0; i2 < loop_ub; i2++) {
                b_t_test_simpl_data[i2] = (t_test_simpl_data[i2] < 1.0E-7);
              }

              if (coder::all(b_t_test_simpl_data, c_t_test_simpl_size)) {
                b_existing = true;
              } else {
                b_existing = false;
              }
            } else {
              b_existing = false;
            }
          } else {
            b_existing = false;
          }

          if (b_existing) {
            exitg1 = true;
          } else {
            idx++;
          }
        }

        if (!b_existing) {
          num_valid++;
          if (num_valid > 100) {
            rtDynamicBoundsError(101, 1, 100, &db_emlrtBCI);
          }

          for (i2 = 0; i2 < 7; i2++) {
            varargin_1 = (num_valid + 100 * i2) - 1;
            t_out[varargin_1] = t_test_valid[i2];
            J_out[varargin_1] = J_test[i2];
          }

          solution_out[num_valid - 1] = cases_data[b_index];
        }
      }
    }
  }

  if (1 > num_valid) {
    loop_ub = 0;
  } else {
    loop_ub = num_valid;
  }

  t_out_size[0] = loop_ub;
  t_out_size[1] = 7;
  if (1 > num_valid) {
    idx = 0;
  } else {
    idx = num_valid;
  }

  J_out_size[0] = idx;
  J_out_size[1] = 7;
  for (i = 0; i < 7; i++) {
    for (i1 = 0; i1 < loop_ub; i1++) {
      t_out_data[i1 + loop_ub * i] = t_out[i1 + 100 * i];
    }

    for (i1 = 0; i1 < idx; i1++) {
      J_out_data[i1 + idx * i] = J_out[i1 + 100 * i];
    }
  }

  if (1 > num_valid) {
    b_index = 0;
  } else {
    b_index = num_valid;
  }

  *solution_out_size = b_index;
  if (0 <= b_index - 1) {
    std::copy(&solution_out[0], &solution_out[b_index], &solution_out_data[0]);
  }

  if (num_valid == 0) {
    printf("Error: Could not find valid optimal solution!\n");
    fflush(stdout);
  } else if (num_valid > 1) {
    printf("Debug: Multiple (");
    fflush(stdout);
    printint(static_cast<double>(num_valid));
    printf(") optimal solutions!\n");
    fflush(stdout);
    t_out_size[0] = loop_ub;
    t_out_size[1] = 7;
    for (i = 0; i < 7; i++) {
      for (i1 = 0; i1 < loop_ub; i1++) {
        t_out_data[i1 + loop_ub * i] = t_out[i1 + 100 * i];
      }
    }

    coder::sum(t_out_data, t_out_size, a__1_data, &varargin_1);
    coder::internal::sort(a__1_data, &varargin_1, iidx_data, solution_out_size);
    for (i = 0; i < *solution_out_size; i++) {
      a__1_data[i] = iidx_data[i];
    }

    t_out_size[0] = *solution_out_size;
    t_out_size[1] = 7;
    for (i = 0; i < 7; i++) {
      for (i1 = 0; i1 < *solution_out_size; i1++) {
        i2 = static_cast<int>(a__1_data[i1]);
        if ((i2 < 1) || (i2 > loop_ub)) {
          rtDynamicBoundsError(i2, 1, loop_ub, &fb_emlrtBCI);
        }

        t_out_data[i1 + *solution_out_size * i] = t_out[(i2 + 100 * i) - 1];
      }
    }

    J_out_size[0] = *solution_out_size;
    J_out_size[1] = 7;
    for (i = 0; i < 7; i++) {
      for (i1 = 0; i1 < *solution_out_size; i1++) {
        i2 = static_cast<int>(a__1_data[i1]);
        if ((i2 < 1) || (i2 > idx)) {
          rtDynamicBoundsError(i2, 1, idx, &gb_emlrtBCI);
        }

        J_out_data[i1 + *solution_out_size * i] = J_out[(i2 + 100 * i) - 1];
      }
    }

    for (i = 0; i < *solution_out_size; i++) {
      i1 = static_cast<int>(a__1_data[i]);
      if ((i1 < 1) || (i1 > b_index)) {
        rtDynamicBoundsError(i1, 1, b_index, &hb_emlrtBCI);
      }

      solution_out_data[i] = solution_out[i1 - 1];
    }
  }
}

void c_solve_O(double P_init, double V_init, double A_init, double V_max, double
               V_min, double A_max, double A_min, double J_max, double J_min,
               double t_out_data[], int t_out_size[2], double J_out_data[], int
               J_out_size[2], int solution_out_data[], int *solution_out_size)
{
  creal_T t_test_data[35];
  creal_T dcv1[28];
  creal_T dcv2[21];
  creal_T dcv[14];
  creal_T b_A_init[7];
  double J_out[700];
  double t_out[700];
  double a__1_data[100];
  double dv[14];
  double J_out_simpl_data[7];
  double J_test[7];
  double J_test_simpl_data[7];
  double b_J_out[7];
  double b_varargin_1[7];
  double t_out_simpl_data[7];
  double t_test_simpl_data[7];
  double t_test_valid[7];
  int iidx_data[100];
  int solution_out[100];
  int cases_data[45];
  int J_out_simpl_size[2];
  int J_test_simpl_size[2];
  int b_t_test_simpl_size[2];
  int c_t_test_simpl_size[2];
  int cases_size[2];
  int t_out_simpl_size[2];
  int t_test_simpl_size[2];
  int t_test_size[2];
  int valid_size[2];
  int b_index;
  int i;
  int i1;
  int i2;
  int idx;
  int loop_ub;
  int num_valid;
  int varargin_1;
  bool b_t_test_simpl_data[7];
  bool valid_data[5];

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
  num_valid = 0;
  std::memset(&t_out[0], 0, 700U * sizeof(double));
  std::memset(&J_out[0], 0, 700U * sizeof(double));
  std::memset(&solution_out[0], 0, 100U * sizeof(int));

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
  coder::eml_sub2ind(1.0, 2.0, 2.0);

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
  switch (check_feasibility(V_init, A_init, V_max, V_min, A_max, A_min, J_max,
           J_min)) {
   case 3U:
    varargin_1 = 2;
    break;

   case 5U:
    varargin_1 = 2;
    break;

   case 4U:
    varargin_1 = 3;
    break;

   case 6U:
    varargin_1 = 3;
    break;

   default:
    varargin_1 = 1;
    break;
  }

  varargin_1 = coder::eml_sub2ind(static_cast<double>(varargin_1), static_cast<
    double>(!rtIsInf(V_max)) + 1.0, static_cast<double>(!rtIsInf(V_min)) + 1.0,
    static_cast<double>(!rtIsInf(A_max)) + 1.0, static_cast<double>(!rtIsInf
    (A_min)) + 1.0);
  select_cases_O(4U, condlut[varargin_1 - 1], cases_data, cases_size);
  i = cases_size[1];
  for (b_index = 0; b_index < i; b_index++) {
    double d;
    if (b_index + 1 > cases_size[1]) {
      rtDynamicBoundsError(b_index + 1, 1, cases_size[1], &bb_emlrtBCI);
    }

    i1 = cases_data[b_index];
    if (coder::b_mod(std::floor(static_cast<double>(i1) / 1000.0)) == 0.0) {
      if (i1 - ((i1 >> 1) << 1) == 1) {
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
    } else if (i1 - ((i1 >> 1) << 1) == 1) {
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

    switch (i1) {
     case 0:
      t_test_size[0] = 1;
      t_test_size[1] = 7;
      std::memset(&t_test_data[0], 0, 7U * sizeof(creal_T));
      break;

     case 101:
      abcdefg_O_AVP(P_init, V_init, A_init, rtNaN, 0.0, 0.0, V_max, A_max, A_min,
                    J_max, J_min, b_varargin_1);
      t_test_size[0] = 1;
      t_test_size[1] = 7;
      for (i1 = 0; i1 < 7; i1++) {
        t_test_data[i1].re = b_varargin_1[i1];
        t_test_data[i1].im = 0.0;
      }
      break;

     case 102:
      abcdefg_O_AVP(-P_init, -V_init, -A_init, rtNaN, -0.0, -0.0, -V_min, -A_min,
                    -A_max, -J_min, -J_max, b_varargin_1);
      t_test_size[0] = 1;
      t_test_size[1] = 7;
      for (i1 = 0; i1 < 7; i1++) {
        t_test_data[i1].re = b_varargin_1[i1];
        t_test_data[i1].im = 0.0;
      }
      break;

     case 103:
      abcdeg_O_AVP(P_init, V_init, A_init, rtNaN, 0.0, 0.0, V_max, A_max, J_max,
                   J_min, dcv);
      t_test_size[0] = 2;
      t_test_size[1] = 7;
      std::copy(&dcv[0], &dcv[14], &t_test_data[0]);
      break;

     case 104:
      abcdeg_O_AVP(-P_init, -V_init, -A_init, rtNaN, -0.0, -0.0, -V_min, -A_min,
                   -J_min, -J_max, dcv);
      t_test_size[0] = 2;
      t_test_size[1] = 7;
      std::copy(&dcv[0], &dcv[14], &t_test_data[0]);
      break;

     case 105:
      abcefg_O_AVP(P_init, V_init, A_init, rtNaN, 0.0, 0.0, A_max, A_min, J_max,
                   J_min, dcv);
      t_test_size[0] = 2;
      t_test_size[1] = 7;
      std::copy(&dcv[0], &dcv[14], &t_test_data[0]);
      break;

     case 106:
      abcefg_O_AVP(-P_init, -V_init, -A_init, rtNaN, -0.0, -0.0, -A_min, -A_max,
                   -J_min, -J_max, dcv);
      t_test_size[0] = 2;
      t_test_size[1] = 7;
      std::copy(&dcv[0], &dcv[14], &t_test_data[0]);
      break;

     case 107:
      abceg_O_AVP(P_init, V_init, A_init, rtNaN, 0.0, 0.0, A_max, J_max, J_min,
                  dcv1);
      t_test_size[0] = 4;
      t_test_size[1] = 7;
      std::copy(&dcv1[0], &dcv1[28], &t_test_data[0]);
      break;

     case 108:
      abceg_O_AVP(-P_init, -V_init, -A_init, rtNaN, -0.0, -0.0, -A_min, -J_min,
                  -J_max, dcv1);
      t_test_size[0] = 4;
      t_test_size[1] = 7;
      std::copy(&dcv1[0], &dcv1[28], &t_test_data[0]);
      break;

     case 109:
      acdefg_O_AVP(P_init, V_init, A_init, rtNaN, 0.0, 0.0, V_max, A_min, J_max,
                   J_min, dcv);
      t_test_size[0] = 2;
      t_test_size[1] = 7;
      std::copy(&dcv[0], &dcv[14], &t_test_data[0]);
      break;

     case 110:
      acdefg_O_AVP(-P_init, -V_init, -A_init, rtNaN, -0.0, -0.0, -V_min, -A_max,
                   -J_min, -J_max, dcv);
      t_test_size[0] = 2;
      t_test_size[1] = 7;
      std::copy(&dcv[0], &dcv[14], &t_test_data[0]);
      break;

     case 111:
      acdeg_O_AVP(P_init, V_init, A_init, rtNaN, 0.0, 0.0, V_max, J_max, J_min,
                  dcv1);
      t_test_size[0] = 4;
      t_test_size[1] = 7;
      std::copy(&dcv1[0], &dcv1[28], &t_test_data[0]);
      break;

     case 112:
      acdeg_O_AVP(-P_init, -V_init, -A_init, rtNaN, -0.0, -0.0, -V_min, -J_min,
                  -J_max, dcv1);
      t_test_size[0] = 4;
      t_test_size[1] = 7;
      std::copy(&dcv1[0], &dcv1[28], &t_test_data[0]);
      break;

     case 113:
      acefg_O_AVP(P_init, V_init, A_init, rtNaN, 0.0, 0.0, A_min, J_max, J_min,
                  dcv1);
      t_test_size[0] = 4;
      t_test_size[1] = 7;
      std::copy(&dcv1[0], &dcv1[28], &t_test_data[0]);
      break;

     case 114:
      acefg_O_AVP(-P_init, -V_init, -A_init, rtNaN, -0.0, -0.0, -A_max, -J_min,
                  -J_max, dcv1);
      t_test_size[0] = 4;
      t_test_size[1] = 7;
      std::copy(&dcv1[0], &dcv1[28], &t_test_data[0]);
      break;

     case 115:
      aceg_O_AVP(P_init, V_init, A_init, rtNaN, 0.0, 0.0, J_max, J_min,
                 t_test_data);
      t_test_size[0] = 5;
      t_test_size[1] = 7;
      break;

     case 116:
      aceg_O_AVP(-P_init, -V_init, -A_init, rtNaN, -0.0, -0.0, -J_min, -J_max,
                 t_test_data);
      t_test_size[0] = 5;
      t_test_size[1] = 7;

      // -----------------------
      break;

     case 201:
      abc_O_VP(P_init, V_init, A_init, rtNaN, 0.0, A_max, J_max, J_min, dcv1);
      t_test_size[0] = 4;
      t_test_size[1] = 7;
      std::copy(&dcv1[0], &dcv1[28], &t_test_data[0]);
      break;

     case 202:
      abc_O_VP(-P_init, -V_init, -A_init, rtNaN, -0.0, -A_min, -J_min, -J_max,
               dcv1);
      t_test_size[0] = 4;
      t_test_size[1] = 7;
      std::copy(&dcv1[0], &dcv1[28], &t_test_data[0]);
      break;

     case 203:
      abcde_O_VP(P_init, V_init, A_init, rtNaN, 0.0, V_max, A_max, J_max, J_min,
                 dcv);
      t_test_size[0] = 2;
      t_test_size[1] = 7;
      std::copy(&dcv[0], &dcv[14], &t_test_data[0]);
      break;

     case 204:
      abcde_O_VP(-P_init, -V_init, -A_init, rtNaN, -0.0, -V_min, -A_min, -J_min,
                 -J_max, dcv);
      t_test_size[0] = 2;
      t_test_size[1] = 7;
      std::copy(&dcv[0], &dcv[14], &t_test_data[0]);
      break;

     case 205:
      abcdef_O_VP(P_init, V_init, A_init, rtNaN, 0.0, V_max, A_max, A_min, J_max,
                  J_min, b_varargin_1);
      t_test_size[0] = 1;
      t_test_size[1] = 7;
      for (i1 = 0; i1 < 7; i1++) {
        t_test_data[i1].re = b_varargin_1[i1];
        t_test_data[i1].im = 0.0;
      }
      break;

     case 206:
      abcdef_O_VP(-P_init, -V_init, -A_init, rtNaN, -0.0, -V_min, -A_min, -A_max,
                  -J_min, -J_max, b_varargin_1);
      t_test_size[0] = 1;
      t_test_size[1] = 7;
      for (i1 = 0; i1 < 7; i1++) {
        t_test_data[i1].re = b_varargin_1[i1];
        t_test_data[i1].im = 0.0;
      }
      break;

     case 207:
      abcdefg_O_VP(P_init, V_init, A_init, rtNaN, 0.0, V_max, V_min, A_max,
                   A_min, J_max, J_min, dv);
      t_test_size[0] = 2;
      t_test_size[1] = 7;
      for (i1 = 0; i1 < 14; i1++) {
        t_test_data[i1].re = dv[i1];
        t_test_data[i1].im = 0.0;
      }
      break;

     case 208:
      abcdefg_O_VP(-P_init, -V_init, -A_init, rtNaN, -0.0, -V_min, -V_max,
                   -A_min, -A_max, -J_min, -J_max, dv);
      t_test_size[0] = 2;
      t_test_size[1] = 7;
      for (i1 = 0; i1 < 14; i1++) {
        t_test_data[i1].re = dv[i1];
        t_test_data[i1].im = 0.0;
      }
      break;

     case 209:
      abcdeg_O_VP(P_init, V_init, A_init, rtNaN, 0.0, V_max, V_min, A_max, J_max,
                  J_min, dcv);
      t_test_size[0] = 2;
      t_test_size[1] = 7;
      std::copy(&dcv[0], &dcv[14], &t_test_data[0]);
      break;

     case 210:
      abcdeg_O_VP(-P_init, -V_init, -A_init, rtNaN, -0.0, -V_min, -V_max, -A_min,
                  -J_min, -J_max, dcv);
      t_test_size[0] = 2;
      t_test_size[1] = 7;
      std::copy(&dcv[0], &dcv[14], &t_test_data[0]);
      break;

     case 211:
      abcef_O_VP(P_init, V_init, A_init, rtNaN, 0.0, A_max, A_min, J_max, J_min,
                 dcv);
      t_test_size[0] = 2;
      t_test_size[1] = 7;
      std::copy(&dcv[0], &dcv[14], &t_test_data[0]);
      break;

     case 212:
      abcef_O_VP(-P_init, -V_init, -A_init, rtNaN, -0.0, -A_min, -A_max, -J_min,
                 -J_max, dcv);
      t_test_size[0] = 2;
      t_test_size[1] = 7;
      std::copy(&dcv[0], &dcv[14], &t_test_data[0]);
      break;

     case 213:
      t_test_size[0] = 1;
      t_test_size[1] = 7;
      std::memset(&t_test_data[0], 0, 7U * sizeof(creal_T));
      break;

     case 214:
      t_test_size[0] = 1;
      t_test_size[1] = 7;
      std::memset(&t_test_data[0], 0, 7U * sizeof(creal_T));
      break;

     case 215:
      t_test_size[0] = 1;
      t_test_size[1] = 7;
      std::memset(&t_test_data[0], 0, 7U * sizeof(creal_T));
      break;

     case 216:
      t_test_size[0] = 1;
      t_test_size[1] = 7;
      std::memset(&t_test_data[0], 0, 7U * sizeof(creal_T));
      break;

     case 217:
      t_test_size[0] = 1;
      t_test_size[1] = 7;
      std::memset(&t_test_data[0], 0, 7U * sizeof(creal_T));
      break;

     case 218:
      t_test_size[0] = 1;
      t_test_size[1] = 7;
      std::memset(&t_test_data[0], 0, 7U * sizeof(creal_T));
      break;

     case 219:
      acde_O_VP(P_init, V_init, A_init, rtNaN, 0.0, V_max, J_max, J_min, dcv);
      t_test_size[0] = 2;
      t_test_size[1] = 7;
      std::copy(&dcv[0], &dcv[14], &t_test_data[0]);
      break;

     case 220:
      acde_O_VP(-P_init, -V_init, -A_init, rtNaN, -0.0, -V_min, -J_min, -J_max,
                dcv);
      t_test_size[0] = 2;
      t_test_size[1] = 7;
      std::copy(&dcv[0], &dcv[14], &t_test_data[0]);
      break;

     case 221:
      acdef_O_VP(P_init, V_init, A_init, rtNaN, 0.0, V_max, A_min, J_max, J_min,
                 dcv);
      t_test_size[0] = 2;
      t_test_size[1] = 7;
      std::copy(&dcv[0], &dcv[14], &t_test_data[0]);
      break;

     case 222:
      acdef_O_VP(-P_init, -V_init, -A_init, rtNaN, -0.0, -V_min, -A_max, -J_min,
                 -J_max, dcv);
      t_test_size[0] = 2;
      t_test_size[1] = 7;
      std::copy(&dcv[0], &dcv[14], &t_test_data[0]);
      break;

     case 223:
      acdefg_O_VP(P_init, V_init, A_init, rtNaN, 0.0, V_max, V_min, A_min, J_max,
                  J_min, dcv1);
      t_test_size[0] = 4;
      t_test_size[1] = 7;
      std::copy(&dcv1[0], &dcv1[28], &t_test_data[0]);
      break;

     case 224:
      acdefg_O_VP(-P_init, -V_init, -A_init, rtNaN, -0.0, -V_min, -V_max, -A_max,
                  -J_min, -J_max, dcv1);
      t_test_size[0] = 4;
      t_test_size[1] = 7;
      std::copy(&dcv1[0], &dcv1[28], &t_test_data[0]);
      break;

     case 225:
      t_test_size[0] = 1;
      t_test_size[1] = 7;
      std::memset(&t_test_data[0], 0, 7U * sizeof(creal_T));
      break;

     case 226:
      t_test_size[0] = 1;
      t_test_size[1] = 7;
      std::memset(&t_test_data[0], 0, 7U * sizeof(creal_T));
      break;

     case 227:
      acef_O_VP(P_init, V_init, A_init, rtNaN, 0.0, A_min, J_max, J_min, dcv1);
      t_test_size[0] = 4;
      t_test_size[1] = 7;
      std::copy(&dcv1[0], &dcv1[28], &t_test_data[0]);
      break;

     case 228:
      acef_O_VP(-P_init, -V_init, -A_init, rtNaN, -0.0, -A_max, -J_min, -J_max,
                dcv1);
      t_test_size[0] = 4;
      t_test_size[1] = 7;
      std::copy(&dcv1[0], &dcv1[28], &t_test_data[0]);
      break;

     case 229:
      t_test_size[0] = 1;
      t_test_size[1] = 7;
      std::memset(&t_test_data[0], 0, 7U * sizeof(creal_T));
      break;

     case 230:
      t_test_size[0] = 1;
      t_test_size[1] = 7;
      std::memset(&t_test_data[0], 0, 7U * sizeof(creal_T));
      break;

     case 231:
      t_test_size[0] = 1;
      t_test_size[1] = 7;
      std::memset(&t_test_data[0], 0, 7U * sizeof(creal_T));
      break;

     case 232:
      t_test_size[0] = 1;
      t_test_size[1] = 7;
      std::memset(&t_test_data[0], 0, 7U * sizeof(creal_T));

      // -----------------------
      break;

     case 301:
      abc_O_AP(P_init, V_init, A_init, rtNaN, 0.0, A_max, J_max, J_min, dcv);
      t_test_size[0] = 2;
      t_test_size[1] = 7;
      std::copy(&dcv[0], &dcv[14], &t_test_data[0]);
      break;

     case 302:
      abc_O_AP(-P_init, -V_init, -A_init, rtNaN, -0.0, -A_min, -J_min, -J_max,
               dcv);
      t_test_size[0] = 2;
      t_test_size[1] = 7;
      std::copy(&dcv[0], &dcv[14], &t_test_data[0]);
      break;

     case 303:
      abcde_O_AP(P_init, V_init, A_init, rtNaN, 0.0, V_max, A_max, J_max, J_min,
                 b_varargin_1);
      t_test_size[0] = 1;
      t_test_size[1] = 7;
      for (i1 = 0; i1 < 7; i1++) {
        t_test_data[i1].re = b_varargin_1[i1];
        t_test_data[i1].im = 0.0;
      }
      break;

     case 304:
      abcde_O_AP(-P_init, -V_init, -A_init, rtNaN, -0.0, -V_min, -A_min, -J_min,
                 -J_max, b_varargin_1);
      t_test_size[0] = 1;
      t_test_size[1] = 7;
      for (i1 = 0; i1 < 7; i1++) {
        t_test_data[i1].re = b_varargin_1[i1];
        t_test_data[i1].im = 0.0;
      }
      break;

     case 305:
      abcdefg_O_AP(P_init, V_init, A_init, rtNaN, 0.0, V_max, A_max, A_min,
                   J_max, J_min, b_varargin_1);
      t_test_size[0] = 1;
      t_test_size[1] = 7;
      for (i1 = 0; i1 < 7; i1++) {
        t_test_data[i1].re = b_varargin_1[i1];
        t_test_data[i1].im = 0.0;
      }
      break;

     case 306:
      abcdefg_O_AP(-P_init, -V_init, -A_init, rtNaN, -0.0, -V_min, -A_min,
                   -A_max, -J_min, -J_max, b_varargin_1);
      t_test_size[0] = 1;
      t_test_size[1] = 7;
      for (i1 = 0; i1 < 7; i1++) {
        t_test_data[i1].re = b_varargin_1[i1];
        t_test_data[i1].im = 0.0;
      }
      break;

     case 307:
      abcefg_O_AP(P_init, V_init, A_init, rtNaN, 0.0, V_max, A_max, A_min, J_max,
                  J_min, dcv);
      t_test_size[0] = 2;
      t_test_size[1] = 7;
      std::copy(&dcv[0], &dcv[14], &t_test_data[0]);
      break;

     case 308:
      abcefg_O_AP(-P_init, -V_init, -A_init, rtNaN, -0.0, -V_min, -A_min, -A_max,
                  -J_min, -J_max, dcv);
      t_test_size[0] = 2;
      t_test_size[1] = 7;
      std::copy(&dcv[0], &dcv[14], &t_test_data[0]);
      break;

     case 309:
      abceg_O_AP(P_init, V_init, A_init, rtNaN, 0.0, V_max, A_max, J_max, J_min,
                 dcv1);
      t_test_size[0] = 4;
      t_test_size[1] = 7;
      std::copy(&dcv1[0], &dcv1[28], &t_test_data[0]);
      break;

     case 310:
      abceg_O_AP(-P_init, -V_init, -A_init, rtNaN, -0.0, -V_min, -A_min, -J_min,
                 -J_max, dcv1);
      t_test_size[0] = 4;
      t_test_size[1] = 7;
      std::copy(&dcv1[0], &dcv1[28], &t_test_data[0]);
      break;

     case 311:
      ac_O_AP(P_init, V_init, A_init, rtNaN, 0.0, J_max, J_min, dcv2);
      t_test_size[0] = 3;
      t_test_size[1] = 7;
      std::copy(&dcv2[0], &dcv2[21], &t_test_data[0]);
      break;

     case 312:
      ac_O_AP(-P_init, -V_init, -A_init, rtNaN, -0.0, -J_min, -J_max, dcv2);
      t_test_size[0] = 3;
      t_test_size[1] = 7;
      std::copy(&dcv2[0], &dcv2[21], &t_test_data[0]);
      break;

     case 313:
      acde_O_AP(P_init, V_init, A_init, rtNaN, 0.0, V_max, J_max, J_min, dcv);
      t_test_size[0] = 2;
      t_test_size[1] = 7;
      std::copy(&dcv[0], &dcv[14], &t_test_data[0]);
      break;

     case 314:
      acde_O_AP(-P_init, -V_init, -A_init, rtNaN, -0.0, -V_min, -J_min, -J_max,
                dcv);
      t_test_size[0] = 2;
      t_test_size[1] = 7;
      std::copy(&dcv[0], &dcv[14], &t_test_data[0]);
      break;

     case 315:
      acdefg_O_AP(P_init, V_init, A_init, rtNaN, 0.0, V_max, A_min, J_max, J_min,
                  dcv);
      t_test_size[0] = 2;
      t_test_size[1] = 7;
      std::copy(&dcv[0], &dcv[14], &t_test_data[0]);
      break;

     case 316:
      acdefg_O_AP(-P_init, -V_init, -A_init, rtNaN, -0.0, -V_min, -A_max, -J_min,
                  -J_max, dcv);
      t_test_size[0] = 2;
      t_test_size[1] = 7;
      std::copy(&dcv[0], &dcv[14], &t_test_data[0]);
      break;

     case 317:
      acefg_O_AP(P_init, V_init, A_init, rtNaN, 0.0, V_max, A_min, J_max, J_min,
                 dcv1);
      t_test_size[0] = 4;
      t_test_size[1] = 7;
      std::copy(&dcv1[0], &dcv1[28], &t_test_data[0]);
      break;

     case 318:
      acefg_O_AP(-P_init, -V_init, -A_init, rtNaN, -0.0, -V_min, -A_max, -J_min,
                 -J_max, dcv1);
      t_test_size[0] = 4;
      t_test_size[1] = 7;
      std::copy(&dcv1[0], &dcv1[28], &t_test_data[0]);
      break;

     case 319:
      aceg_O_AP(P_init, V_init, A_init, rtNaN, 0.0, V_max, J_max, J_min, dcv1);
      t_test_size[0] = 4;
      t_test_size[1] = 7;
      std::copy(&dcv1[0], &dcv1[28], &t_test_data[0]);
      break;

     case 320:
      aceg_O_AP(-P_init, -V_init, -A_init, rtNaN, -0.0, -V_min, -J_min, -J_max,
                dcv1);
      t_test_size[0] = 4;
      t_test_size[1] = 7;
      std::copy(&dcv1[0], &dcv1[28], &t_test_data[0]);
      break;

     case 321:
      abcdeg_O_AP(P_init, V_init, A_init, rtNaN, 0.0, V_max, A_max, J_max, J_min,
                  dv);
      t_test_size[0] = 2;
      t_test_size[1] = 7;
      for (i1 = 0; i1 < 14; i1++) {
        t_test_data[i1].re = dv[i1];
        t_test_data[i1].im = 0.0;
      }
      break;

     case 322:
      abcdeg_O_AP(-P_init, -V_init, -A_init, rtNaN, -0.0, -V_min, -A_min, -J_min,
                  -J_max, dv);
      t_test_size[0] = 2;
      t_test_size[1] = 7;
      for (i1 = 0; i1 < 14; i1++) {
        t_test_data[i1].re = dv[i1];
        t_test_data[i1].im = 0.0;
      }
      break;

     case 323:
      acdeg_O_AP(P_init, V_init, A_init, rtNaN, 0.0, V_max, J_max, J_min, dcv);
      t_test_size[0] = 2;
      t_test_size[1] = 7;
      std::copy(&dcv[0], &dcv[14], &t_test_data[0]);
      break;

     case 324:
      acdeg_O_AP(-P_init, -V_init, -A_init, rtNaN, -0.0, -V_min, -J_min, -J_max,
                 dcv);
      t_test_size[0] = 2;
      t_test_size[1] = 7;
      std::copy(&dcv[0], &dcv[14], &t_test_data[0]);

      // -----------------------
      break;

     case 401:
      {
        double l2;

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
        //  Generated on 28-Aug-2019 17:25:45
        l2 = A_max * A_max;
        b_A_init[0].re = -(A_init - A_max) / J_max;
        b_A_init[0].im = 0.0;
        d = J_min * J_max;
        b_A_init[1].re = (((((J_min * l2 - J_max * l2) - A_init * A_init * J_min)
                            + 0.0 * J_max) + d * V_init * 2.0) - d * 0.0 * 2.0) *
          -0.5 / (A_max * J_min * J_max);
        b_A_init[1].im = 0.0;
        b_A_init[2].re = -A_max / J_min;
        b_A_init[2].im = 0.0;
        b_A_init[3].re = 0.0;
        b_A_init[3].im = 0.0;
        b_A_init[4].re = 0.0;
        b_A_init[4].im = 0.0;
        b_A_init[5].re = 0.0;
        b_A_init[5].im = 0.0;
        b_A_init[6].re = 0.0;
        b_A_init[6].im = 0.0;
        t_test_size[0] = 1;
        t_test_size[1] = 7;
        std::copy(&b_A_init[0], &b_A_init[7], &t_test_data[0]);
      }
      break;

     case 402:
      {
        double l2;

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
        //  Generated on 28-Aug-2019 17:25:45
        l2 = -A_min * -A_min;
        b_A_init[0].re = -(-A_init - (-A_min)) / -J_min;
        b_A_init[0].im = 0.0;
        d = -J_max * -J_min;
        b_A_init[1].re = (((((-J_max * l2 - -J_min * l2) - -A_init * -A_init *
                             -J_max) + 0.0 * -J_min) + d * -V_init * 2.0) - d *
                          -0.0 * 2.0) * -0.5 / (-A_min * -J_max * -J_min);
        b_A_init[1].im = 0.0;
        b_A_init[2].re = A_min / -J_max;
        b_A_init[2].im = 0.0;
        b_A_init[3].re = 0.0;
        b_A_init[3].im = 0.0;
        b_A_init[4].re = 0.0;
        b_A_init[4].im = 0.0;
        b_A_init[5].re = 0.0;
        b_A_init[5].im = 0.0;
        b_A_init[6].re = 0.0;
        b_A_init[6].im = 0.0;
        t_test_size[0] = 1;
        t_test_size[1] = 7;
        std::copy(&b_A_init[0], &b_A_init[7], &t_test_data[0]);
      }
      break;

     case 403:
      ac_O_AV(V_init, A_init, 0.0, 0.0, J_max, J_min, dcv);
      t_test_size[0] = 2;
      t_test_size[1] = 7;
      std::copy(&dcv[0], &dcv[14], &t_test_data[0]);
      break;

     case 404:
      ac_O_AV(-V_init, -A_init, -0.0, -0.0, -J_min, -J_max, dcv);
      t_test_size[0] = 2;
      t_test_size[1] = 7;
      std::copy(&dcv[0], &dcv[14], &t_test_data[0]);
      break;

     case 501:
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
      //  Generated on 28-Aug-2019 17:25:45
      b_A_init[0].re = -A_init / J_max;
      b_A_init[0].im = 0.0;
      b_A_init[1].re = 0.0;
      b_A_init[1].im = 0.0;
      b_A_init[2].re = 0.0;
      b_A_init[2].im = 0.0;
      b_A_init[3].re = 0.0;
      b_A_init[3].im = 0.0;
      b_A_init[4].re = 0.0;
      b_A_init[4].im = 0.0;
      b_A_init[5].re = 0.0;
      b_A_init[5].im = 0.0;
      b_A_init[6].re = 0.0;
      b_A_init[6].im = 0.0;
      t_test_size[0] = 1;
      t_test_size[1] = 7;
      std::copy(&b_A_init[0], &b_A_init[7], &t_test_data[0]);
      break;

     case 502:
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
      //  Generated on 28-Aug-2019 17:25:45
      b_A_init[0].re = A_init / -J_min;
      b_A_init[0].im = 0.0;
      b_A_init[1].re = 0.0;
      b_A_init[1].im = 0.0;
      b_A_init[2].re = 0.0;
      b_A_init[2].im = 0.0;
      b_A_init[3].re = 0.0;
      b_A_init[3].im = 0.0;
      b_A_init[4].re = 0.0;
      b_A_init[4].im = 0.0;
      b_A_init[5].re = 0.0;
      b_A_init[5].im = 0.0;
      b_A_init[6].re = 0.0;
      b_A_init[6].im = 0.0;
      t_test_size[0] = 1;
      t_test_size[1] = 7;
      std::copy(&b_A_init[0], &b_A_init[7], &t_test_data[0]);
      break;

     case 503:
      abc_O_A(V_init, A_init, 0.0, V_min, A_max, J_max, J_min, b_varargin_1);
      t_test_size[0] = 1;
      t_test_size[1] = 7;
      for (i1 = 0; i1 < 7; i1++) {
        t_test_data[i1].re = b_varargin_1[i1];
        t_test_data[i1].im = 0.0;
      }
      break;

     case 504:
      abc_O_A(-V_init, -A_init, -0.0, -V_max, -A_min, -J_min, -J_max,
              b_varargin_1);
      t_test_size[0] = 1;
      t_test_size[1] = 7;
      for (i1 = 0; i1 < 7; i1++) {
        t_test_data[i1].re = b_varargin_1[i1];
        t_test_data[i1].im = 0.0;
      }
      break;

     case 505:
      ac_O_A(V_init, A_init, 0.0, V_min, J_max, J_min, dcv);
      t_test_size[0] = 2;
      t_test_size[1] = 7;
      std::copy(&dcv[0], &dcv[14], &t_test_data[0]);
      break;

     case 506:
      ac_O_A(-V_init, -A_init, -0.0, -V_max, -J_min, -J_max, dcv);
      t_test_size[0] = 2;
      t_test_size[1] = 7;
      std::copy(&dcv[0], &dcv[14], &t_test_data[0]);

      // -----------------------
      break;

     case 601:
      a_O_V(V_init, A_init, 0.0, J_max, dcv);
      t_test_size[0] = 2;
      t_test_size[1] = 7;
      std::copy(&dcv[0], &dcv[14], &t_test_data[0]);
      break;

     case 602:
      a_O_V(-V_init, -A_init, -0.0, -J_min, dcv);
      t_test_size[0] = 2;
      t_test_size[1] = 7;
      std::copy(&dcv[0], &dcv[14], &t_test_data[0]);
      break;

     case 603:
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
      //  Generated on 28-Aug-2019 17:25:45
      b_A_init[0].re = -(A_init - A_max) / J_max;
      b_A_init[0].im = 0.0;
      b_A_init[1].re = (((J_max * V_init * 2.0 - J_max * 0.0 * 2.0) - A_init *
                         A_init) + A_max * A_max) * -0.5 / (A_max * J_max);
      b_A_init[1].im = 0.0;
      b_A_init[2].re = 0.0;
      b_A_init[2].im = 0.0;
      b_A_init[3].re = 0.0;
      b_A_init[3].im = 0.0;
      b_A_init[4].re = 0.0;
      b_A_init[4].im = 0.0;
      b_A_init[5].re = 0.0;
      b_A_init[5].im = 0.0;
      b_A_init[6].re = 0.0;
      b_A_init[6].im = 0.0;
      t_test_size[0] = 1;
      t_test_size[1] = 7;
      std::copy(&b_A_init[0], &b_A_init[7], &t_test_data[0]);
      break;

     case 604:
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
      //  Generated on 28-Aug-2019 17:25:45
      b_A_init[0].re = -(-A_init - (-A_min)) / -J_min;
      b_A_init[0].im = 0.0;
      b_A_init[1].re = (((-J_min * -V_init * 2.0 - -J_min * -0.0 * 2.0) -
                         -A_init * -A_init) + -A_min * -A_min) * -0.5 / (-A_min *
        -J_min);
      b_A_init[1].im = 0.0;
      b_A_init[2].re = 0.0;
      b_A_init[2].im = 0.0;
      b_A_init[3].re = 0.0;
      b_A_init[3].im = 0.0;
      b_A_init[4].re = 0.0;
      b_A_init[4].im = 0.0;
      b_A_init[5].re = 0.0;
      b_A_init[5].im = 0.0;
      b_A_init[6].re = 0.0;
      b_A_init[6].im = 0.0;
      t_test_size[0] = 1;
      t_test_size[1] = 7;
      std::copy(&b_A_init[0], &b_A_init[7], &t_test_data[0]);
      break;

     case 605:
      abc_O_V(V_init, A_init, 0.0, V_max, A_max, J_max, J_min, dcv);
      t_test_size[0] = 2;
      t_test_size[1] = 7;
      std::copy(&dcv[0], &dcv[14], &t_test_data[0]);
      break;

     case 606:
      abc_O_V(-V_init, -A_init, -0.0, -V_min, -A_min, -J_min, -J_max, dcv);
      t_test_size[0] = 2;
      t_test_size[1] = 7;
      std::copy(&dcv[0], &dcv[14], &t_test_data[0]);
      break;

     case 607:
      ac_O_V(V_init, A_init, 0.0, V_max, J_max, J_min, dcv);
      t_test_size[0] = 2;
      t_test_size[1] = 7;
      std::copy(&dcv[0], &dcv[14], &t_test_data[0]);
      break;

     case 608:
      ac_O_V(-V_init, -A_init, -0.0, -V_min, -J_min, -J_max, dcv);
      t_test_size[0] = 2;
      t_test_size[1] = 7;
      std::copy(&dcv[0], &dcv[14], &t_test_data[0]);

      // -----------------------
      break;

     case 701:
      a_O_P(P_init, V_init, A_init, rtNaN, J_max, dcv2);
      t_test_size[0] = 3;
      t_test_size[1] = 7;
      std::copy(&dcv2[0], &dcv2[21], &t_test_data[0]);
      break;

     case 702:
      a_O_P(-P_init, -V_init, -A_init, rtNaN, -J_min, dcv2);
      t_test_size[0] = 3;
      t_test_size[1] = 7;
      std::copy(&dcv2[0], &dcv2[21], &t_test_data[0]);
      break;

     case 703:
      ab_O_P(P_init, V_init, A_init, rtNaN, A_max, J_max, dcv);
      t_test_size[0] = 2;
      t_test_size[1] = 7;
      std::copy(&dcv[0], &dcv[14], &t_test_data[0]);
      break;

     case 704:
      ab_O_P(-P_init, -V_init, -A_init, rtNaN, -A_min, -J_min, dcv);
      t_test_size[0] = 2;
      t_test_size[1] = 7;
      std::copy(&dcv[0], &dcv[14], &t_test_data[0]);
      break;

     case 705:
      abc_O_P(P_init, V_init, A_init, rtNaN, V_max, A_max, J_max, J_min, dcv2);
      t_test_size[0] = 3;
      t_test_size[1] = 7;
      std::copy(&dcv2[0], &dcv2[21], &t_test_data[0]);
      break;

     case 706:
      abc_O_P(-P_init, -V_init, -A_init, rtNaN, -V_min, -A_min, -J_min, -J_max,
              dcv2);
      t_test_size[0] = 3;
      t_test_size[1] = 7;
      std::copy(&dcv2[0], &dcv2[21], &t_test_data[0]);
      break;

     case 707:
      abcd_O_P(P_init, V_init, A_init, rtNaN, V_max, A_max, J_max, J_min,
               b_varargin_1);
      t_test_size[0] = 1;
      t_test_size[1] = 7;
      for (i1 = 0; i1 < 7; i1++) {
        t_test_data[i1].re = b_varargin_1[i1];
        t_test_data[i1].im = 0.0;
      }
      break;

     case 708:
      abcd_O_P(-P_init, -V_init, -A_init, rtNaN, -V_min, -A_min, -J_min, -J_max,
               b_varargin_1);
      t_test_size[0] = 1;
      t_test_size[1] = 7;
      for (i1 = 0; i1 < 7; i1++) {
        t_test_data[i1].re = b_varargin_1[i1];
        t_test_data[i1].im = 0.0;
      }
      break;

     case 709:
      ac_O_P(P_init, V_init, A_init, rtNaN, V_max, J_max, J_min, dcv2);
      t_test_size[0] = 3;
      t_test_size[1] = 7;
      std::copy(&dcv2[0], &dcv2[21], &t_test_data[0]);
      break;

     case 710:
      ac_O_P(-P_init, -V_init, -A_init, rtNaN, -V_min, -J_min, -J_max, dcv2);
      t_test_size[0] = 3;
      t_test_size[1] = 7;
      std::copy(&dcv2[0], &dcv2[21], &t_test_data[0]);
      break;

     case 711:
      acd_O_P(P_init, V_init, A_init, rtNaN, V_max, J_max, J_min, dcv);
      t_test_size[0] = 2;
      t_test_size[1] = 7;
      std::copy(&dcv[0], &dcv[14], &t_test_data[0]);
      break;

     case 712:
      acd_O_P(-P_init, -V_init, -A_init, rtNaN, -V_min, -J_min, -J_max, dcv);
      t_test_size[0] = 2;
      t_test_size[1] = 7;
      std::copy(&dcv[0], &dcv[14], &t_test_data[0]);

      // -----------------------
      break;

     case 1101:
      abcdefg_NO_AVP(P_init, V_init, A_init, rtNaN, 0.0, 0.0, V_max, A_min,
                     J_max, J_min, b_varargin_1);
      t_test_size[0] = 1;
      t_test_size[1] = 7;
      for (i1 = 0; i1 < 7; i1++) {
        t_test_data[i1].re = b_varargin_1[i1];
        t_test_data[i1].im = 0.0;
      }
      break;

     case 1102:
      abcdefg_NO_AVP(-P_init, -V_init, -A_init, rtNaN, -0.0, -0.0, -V_min,
                     -A_max, -J_min, -J_max, b_varargin_1);
      t_test_size[0] = 1;
      t_test_size[1] = 7;
      for (i1 = 0; i1 < 7; i1++) {
        t_test_data[i1].re = b_varargin_1[i1];
        t_test_data[i1].im = 0.0;
      }
      break;

     case 1103:
      abcdeg_NO_AVP(P_init, V_init, A_init, rtNaN, 0.0, 0.0, V_max, A_min, J_max,
                    J_min, dcv);
      t_test_size[0] = 2;
      t_test_size[1] = 7;
      std::copy(&dcv[0], &dcv[14], &t_test_data[0]);
      break;

     case 1104:
      abcdeg_NO_AVP(-P_init, -V_init, -A_init, rtNaN, -0.0, -0.0, -V_min, -A_max,
                    -J_min, -J_max, dcv);
      t_test_size[0] = 2;
      t_test_size[1] = 7;
      std::copy(&dcv[0], &dcv[14], &t_test_data[0]);
      break;

     case 1105:
      acdefg_NO_AVP(P_init, V_init, A_init, rtNaN, 0.0, 0.0, V_max, A_min, J_max,
                    J_min, dcv);
      t_test_size[0] = 2;
      t_test_size[1] = 7;
      std::copy(&dcv[0], &dcv[14], &t_test_data[0]);
      break;

     case 1106:
      acdefg_NO_AVP(-P_init, -V_init, -A_init, rtNaN, -0.0, -0.0, -V_min, -A_max,
                    -J_min, -J_max, dcv);
      t_test_size[0] = 2;
      t_test_size[1] = 7;
      std::copy(&dcv[0], &dcv[14], &t_test_data[0]);
      break;

     case 1107:
      acdeg_NO_AVP(P_init, V_init, A_init, rtNaN, 0.0, 0.0, V_max, J_max, J_min,
                   dcv1);
      t_test_size[0] = 4;
      t_test_size[1] = 7;
      std::copy(&dcv1[0], &dcv1[28], &t_test_data[0]);
      break;

     case 1108:
      acdeg_NO_AVP(-P_init, -V_init, -A_init, rtNaN, -0.0, -0.0, -V_min, -J_min,
                   -J_max, dcv1);
      t_test_size[0] = 4;
      t_test_size[1] = 7;
      std::copy(&dcv1[0], &dcv1[28], &t_test_data[0]);

      // -----------------------
      break;

     case 1201:
      abcde_NO_VP(P_init, V_init, A_init, rtNaN, 0.0, V_max, A_min, J_max, J_min,
                  dcv);
      t_test_size[0] = 2;
      t_test_size[1] = 7;
      std::copy(&dcv[0], &dcv[14], &t_test_data[0]);
      break;

     case 1202:
      abcde_NO_VP(-P_init, -V_init, -A_init, rtNaN, -0.0, -V_min, -A_max, -J_min,
                  -J_max, dcv);
      t_test_size[0] = 2;
      t_test_size[1] = 7;
      std::copy(&dcv[0], &dcv[14], &t_test_data[0]);
      break;

     case 1203:
      abcdef_NO_VP(P_init, V_init, A_init, rtNaN, 0.0, V_max, A_min, J_max,
                   J_min, b_varargin_1);
      t_test_size[0] = 1;
      t_test_size[1] = 7;
      for (i1 = 0; i1 < 7; i1++) {
        t_test_data[i1].re = b_varargin_1[i1];
        t_test_data[i1].im = 0.0;
      }
      break;

     case 1204:
      abcdef_NO_VP(-P_init, -V_init, -A_init, rtNaN, -0.0, -V_min, -A_max,
                   -J_min, -J_max, b_varargin_1);
      t_test_size[0] = 1;
      t_test_size[1] = 7;
      for (i1 = 0; i1 < 7; i1++) {
        t_test_data[i1].re = b_varargin_1[i1];
        t_test_data[i1].im = 0.0;
      }
      break;

     case 1205:
      abcdefg_NO_VP(P_init, V_init, A_init, rtNaN, 0.0, V_max, V_min, A_min,
                    J_max, J_min, dv);
      t_test_size[0] = 2;
      t_test_size[1] = 7;
      for (i1 = 0; i1 < 14; i1++) {
        t_test_data[i1].re = dv[i1];
        t_test_data[i1].im = 0.0;
      }
      break;

     case 1206:
      abcdefg_NO_VP(-P_init, -V_init, -A_init, rtNaN, -0.0, -V_min, -V_max,
                    -A_max, -J_min, -J_max, dv);
      t_test_size[0] = 2;
      t_test_size[1] = 7;
      for (i1 = 0; i1 < 14; i1++) {
        t_test_data[i1].re = dv[i1];
        t_test_data[i1].im = 0.0;
      }
      break;

     case 1207:
      abcdeg_NO_VP(P_init, V_init, A_init, rtNaN, 0.0, V_max, V_min, A_min,
                   J_max, J_min, dcv);
      t_test_size[0] = 2;
      t_test_size[1] = 7;
      std::copy(&dcv[0], &dcv[14], &t_test_data[0]);
      break;

     case 1208:
      abcdeg_NO_VP(-P_init, -V_init, -A_init, rtNaN, -0.0, -V_min, -V_max,
                   -A_max, -J_min, -J_max, dcv);
      t_test_size[0] = 2;
      t_test_size[1] = 7;
      std::copy(&dcv[0], &dcv[14], &t_test_data[0]);
      break;

     case 1209:
      acde_NO_VP(P_init, V_init, A_init, rtNaN, 0.0, V_max, J_max, J_min, dcv);
      t_test_size[0] = 2;
      t_test_size[1] = 7;
      std::copy(&dcv[0], &dcv[14], &t_test_data[0]);
      break;

     case 1210:
      acde_NO_VP(-P_init, -V_init, -A_init, rtNaN, -0.0, -V_min, -J_min, -J_max,
                 dcv);
      t_test_size[0] = 2;
      t_test_size[1] = 7;
      std::copy(&dcv[0], &dcv[14], &t_test_data[0]);
      break;

     case 1211:
      acdef_NO_VP(P_init, V_init, A_init, rtNaN, 0.0, V_max, A_min, J_max, J_min,
                  dcv);
      t_test_size[0] = 2;
      t_test_size[1] = 7;
      std::copy(&dcv[0], &dcv[14], &t_test_data[0]);
      break;

     case 1212:
      acdef_NO_VP(-P_init, -V_init, -A_init, rtNaN, -0.0, -V_min, -A_max, -J_min,
                  -J_max, dcv);
      t_test_size[0] = 2;
      t_test_size[1] = 7;
      std::copy(&dcv[0], &dcv[14], &t_test_data[0]);
      break;

     case 1213:
      acdefg_NO_VP(P_init, V_init, A_init, rtNaN, 0.0, V_max, V_min, A_min,
                   J_max, J_min, dcv1);
      t_test_size[0] = 4;
      t_test_size[1] = 7;
      std::copy(&dcv1[0], &dcv1[28], &t_test_data[0]);
      break;

     case 1214:
      acdefg_NO_VP(-P_init, -V_init, -A_init, rtNaN, -0.0, -V_min, -V_max,
                   -A_max, -J_min, -J_max, dcv1);
      t_test_size[0] = 4;
      t_test_size[1] = 7;
      std::copy(&dcv1[0], &dcv1[28], &t_test_data[0]);
      break;

     case 1215:
      t_test_size[0] = 1;
      t_test_size[1] = 7;
      std::memset(&t_test_data[0], 0, 7U * sizeof(creal_T));
      break;

     case 1216:
      t_test_size[0] = 1;
      t_test_size[1] = 7;
      std::memset(&t_test_data[0], 0, 7U * sizeof(creal_T));

      // -----------------------
      break;

     case 1301:
      abcde_NO_AP(P_init, V_init, A_init, rtNaN, 0.0, V_max, A_min, J_max, J_min,
                  b_varargin_1);
      t_test_size[0] = 1;
      t_test_size[1] = 7;
      for (i1 = 0; i1 < 7; i1++) {
        t_test_data[i1].re = b_varargin_1[i1];
        t_test_data[i1].im = 0.0;
      }
      break;

     case 1302:
      abcde_NO_AP(-P_init, -V_init, -A_init, rtNaN, -0.0, -V_min, -A_max, -J_min,
                  -J_max, b_varargin_1);
      t_test_size[0] = 1;
      t_test_size[1] = 7;
      for (i1 = 0; i1 < 7; i1++) {
        t_test_data[i1].re = b_varargin_1[i1];
        t_test_data[i1].im = 0.0;
      }
      break;

     case 1303:
      acde_NO_AP(P_init, V_init, A_init, rtNaN, 0.0, V_max, J_max, J_min, dcv);
      t_test_size[0] = 2;
      t_test_size[1] = 7;
      std::copy(&dcv[0], &dcv[14], &t_test_data[0]);
      break;

     case 1304:
      acde_NO_AP(-P_init, -V_init, -A_init, rtNaN, -0.0, -V_min, -J_min, -J_max,
                 dcv);
      t_test_size[0] = 2;
      t_test_size[1] = 7;
      std::copy(&dcv[0], &dcv[14], &t_test_data[0]);
      break;

     case 1305:
      abcdefg_NO_AP(P_init, V_init, A_init, rtNaN, 0.0, V_max, A_min, J_max,
                    J_min, b_varargin_1);
      t_test_size[0] = 1;
      t_test_size[1] = 7;
      for (i1 = 0; i1 < 7; i1++) {
        t_test_data[i1].re = b_varargin_1[i1];
        t_test_data[i1].im = 0.0;
      }
      break;

     case 1306:
      abcdefg_NO_AP(-P_init, -V_init, -A_init, rtNaN, -0.0, -V_min, -A_max,
                    -J_min, -J_max, b_varargin_1);
      t_test_size[0] = 1;
      t_test_size[1] = 7;
      for (i1 = 0; i1 < 7; i1++) {
        t_test_data[i1].re = b_varargin_1[i1];
        t_test_data[i1].im = 0.0;
      }
      break;

     case 1307:
      acdefg_NO_AP(P_init, V_init, A_init, rtNaN, 0.0, V_max, A_min, J_max,
                   J_min, dcv);
      t_test_size[0] = 2;
      t_test_size[1] = 7;
      std::copy(&dcv[0], &dcv[14], &t_test_data[0]);
      break;

     case 1308:
      acdefg_NO_AP(-P_init, -V_init, -A_init, rtNaN, -0.0, -V_min, -A_max,
                   -J_min, -J_max, dcv);
      t_test_size[0] = 2;
      t_test_size[1] = 7;
      std::copy(&dcv[0], &dcv[14], &t_test_data[0]);
      break;

     case 1309:
      abcdeg_NO_AP(P_init, V_init, A_init, rtNaN, 0.0, V_max, A_min, J_max,
                   J_min, dv);
      t_test_size[0] = 2;
      t_test_size[1] = 7;
      for (i1 = 0; i1 < 14; i1++) {
        t_test_data[i1].re = dv[i1];
        t_test_data[i1].im = 0.0;
      }
      break;

     case 1310:
      abcdeg_NO_AP(-P_init, -V_init, -A_init, rtNaN, -0.0, -V_min, -A_max,
                   -J_min, -J_max, dv);
      t_test_size[0] = 2;
      t_test_size[1] = 7;
      for (i1 = 0; i1 < 14; i1++) {
        t_test_data[i1].re = dv[i1];
        t_test_data[i1].im = 0.0;
      }
      break;

     case 1311:
      acdeg_NO_AP(P_init, V_init, A_init, rtNaN, 0.0, V_max, J_max, J_min, dcv);
      t_test_size[0] = 2;
      t_test_size[1] = 7;
      std::copy(&dcv[0], &dcv[14], &t_test_data[0]);
      break;

     case 1312:
      acdeg_NO_AP(-P_init, -V_init, -A_init, rtNaN, -0.0, -V_min, -J_min, -J_max,
                  dcv);
      t_test_size[0] = 2;
      t_test_size[1] = 7;
      std::copy(&dcv[0], &dcv[14], &t_test_data[0]);

      // -----------------------
      break;

     case 1701:
      abcd_NO_P(P_init, V_init, A_init, rtNaN, V_max, A_min, J_max, J_min,
                b_varargin_1);
      t_test_size[0] = 1;
      t_test_size[1] = 7;
      for (i1 = 0; i1 < 7; i1++) {
        t_test_data[i1].re = b_varargin_1[i1];
        t_test_data[i1].im = 0.0;
      }
      break;

     case 1702:
      abcd_NO_P(-P_init, -V_init, -A_init, rtNaN, -V_min, -A_max, -J_min, -J_max,
                b_varargin_1);
      t_test_size[0] = 1;
      t_test_size[1] = 7;
      for (i1 = 0; i1 < 7; i1++) {
        t_test_data[i1].re = b_varargin_1[i1];
        t_test_data[i1].im = 0.0;
      }
      break;

     case 1703:
      acd_NO_P(P_init, V_init, A_init, rtNaN, V_max, J_max, J_min, dcv);
      t_test_size[0] = 2;
      t_test_size[1] = 7;
      std::copy(&dcv[0], &dcv[14], &t_test_data[0]);
      break;

     case 1704:
      acd_NO_P(-P_init, -V_init, -A_init, rtNaN, -V_min, -J_min, -J_max, dcv);
      t_test_size[0] = 2;
      t_test_size[1] = 7;
      std::copy(&dcv[0], &dcv[14], &t_test_data[0]);
      break;

     default:
      t_test_size[0] = 1;
      t_test_size[1] = 7;
      std::memset(&t_test_data[0], 0, 7U * sizeof(creal_T));
      printf("Error: Solution prior ");
      fflush(stdout);
      printint(i1);
      printf(" is not valid!\n");
      fflush(stdout);
      break;
    }

    d_check(t_test_data, t_test_size, J_test, V_init, A_init, V_max, V_min,
            A_max, A_min, J_max, J_min, valid_data, valid_size);
    i1 = valid_size[1];
    for (int index_t = 0; index_t < i1; index_t++) {
      if (index_t + 1 > valid_size[1]) {
        rtDynamicBoundsError(index_t + 1, 1, valid_size[1], &eb_emlrtBCI);
      }

      if (valid_data[index_t]) {
        bool b_existing;
        bool exitg1;
        if (index_t + 1 > t_test_size[0]) {
          rtDynamicBoundsError(index_t + 1, 1, t_test_size[0], &cb_emlrtBCI);
        }

        for (varargin_1 = 0; varargin_1 < 7; varargin_1++) {
          d = t_test_data[index_t + t_test_size[0] * varargin_1].re;
          if (d > 0.0) {
            t_test_valid[varargin_1] = d;
          } else {
            t_test_valid[varargin_1] = 0.0;
          }
        }

        b_existing = false;
        idx = 0;
        exitg1 = false;
        while ((!exitg1) && (idx <= num_valid - 1)) {
          simplify_setp(t_test_valid, J_test, t_test_simpl_data,
                        t_test_simpl_size, J_test_simpl_data, J_test_simpl_size);
          for (i2 = 0; i2 < 7; i2++) {
            varargin_1 = idx + 100 * i2;
            b_varargin_1[i2] = t_out[varargin_1];
            b_J_out[i2] = J_out[varargin_1];
          }

          simplify_setp(b_varargin_1, b_J_out, t_out_simpl_data,
                        t_out_simpl_size, J_out_simpl_data, J_out_simpl_size);

          // numerical
          // numerical
          if ((t_test_simpl_size[1] == t_out_simpl_size[1]) &&
              (J_test_simpl_size[1] == J_out_simpl_size[1])) {
            rtSizeEqNDCheck(&t_test_simpl_size[0], &t_out_simpl_size[0],
                            &emlrtECI);
            b_t_test_simpl_size[0] = 1;
            b_t_test_simpl_size[1] = t_test_simpl_size[1];
            loop_ub = t_test_simpl_size[1];
            for (i2 = 0; i2 < loop_ub; i2++) {
              b_varargin_1[i2] = t_test_simpl_data[i2] - t_out_simpl_data[i2];
            }

            coder::b_abs(b_varargin_1, b_t_test_simpl_size, t_test_simpl_data,
                         t_test_simpl_size);
            c_t_test_simpl_size[0] = 1;
            c_t_test_simpl_size[1] = t_test_simpl_size[1];
            loop_ub = t_test_simpl_size[1];
            for (i2 = 0; i2 < loop_ub; i2++) {
              b_t_test_simpl_data[i2] = (t_test_simpl_data[i2] < 1.0E-7);
            }

            if (coder::all(b_t_test_simpl_data, c_t_test_simpl_size)) {
              rtSizeEqNDCheck(&J_test_simpl_size[0], &J_out_simpl_size[0],
                              &b_emlrtECI);
              t_out_simpl_size[0] = 1;
              t_out_simpl_size[1] = J_test_simpl_size[1];
              loop_ub = J_test_simpl_size[1];
              for (i2 = 0; i2 < loop_ub; i2++) {
                t_out_simpl_data[i2] = J_test_simpl_data[i2] -
                  J_out_simpl_data[i2];
              }

              coder::b_abs(t_out_simpl_data, t_out_simpl_size, t_test_simpl_data,
                           t_test_simpl_size);
              c_t_test_simpl_size[0] = 1;
              c_t_test_simpl_size[1] = t_test_simpl_size[1];
              loop_ub = t_test_simpl_size[1];
              for (i2 = 0; i2 < loop_ub; i2++) {
                b_t_test_simpl_data[i2] = (t_test_simpl_data[i2] < 1.0E-7);
              }

              if (coder::all(b_t_test_simpl_data, c_t_test_simpl_size)) {
                b_existing = true;
              } else {
                b_existing = false;
              }
            } else {
              b_existing = false;
            }
          } else {
            b_existing = false;
          }

          if (b_existing) {
            exitg1 = true;
          } else {
            idx++;
          }
        }

        if (!b_existing) {
          num_valid++;
          if (num_valid > 100) {
            rtDynamicBoundsError(101, 1, 100, &db_emlrtBCI);
          }

          for (i2 = 0; i2 < 7; i2++) {
            varargin_1 = (num_valid + 100 * i2) - 1;
            t_out[varargin_1] = t_test_valid[i2];
            J_out[varargin_1] = J_test[i2];
          }

          solution_out[num_valid - 1] = cases_data[b_index];
        }
      }
    }
  }

  if (1 > num_valid) {
    loop_ub = 0;
  } else {
    loop_ub = num_valid;
  }

  t_out_size[0] = loop_ub;
  t_out_size[1] = 7;
  if (1 > num_valid) {
    idx = 0;
  } else {
    idx = num_valid;
  }

  J_out_size[0] = idx;
  J_out_size[1] = 7;
  for (i = 0; i < 7; i++) {
    for (i1 = 0; i1 < loop_ub; i1++) {
      t_out_data[i1 + loop_ub * i] = t_out[i1 + 100 * i];
    }

    for (i1 = 0; i1 < idx; i1++) {
      J_out_data[i1 + idx * i] = J_out[i1 + 100 * i];
    }
  }

  if (1 > num_valid) {
    b_index = 0;
  } else {
    b_index = num_valid;
  }

  *solution_out_size = b_index;
  if (0 <= b_index - 1) {
    std::copy(&solution_out[0], &solution_out[b_index], &solution_out_data[0]);
  }

  if (num_valid == 0) {
    printf("Error: Could not find valid optimal solution!\n");
    fflush(stdout);
  } else if (num_valid > 1) {
    printf("Debug: Multiple (");
    fflush(stdout);
    printint(static_cast<double>(num_valid));
    printf(") optimal solutions!\n");
    fflush(stdout);
    t_out_size[0] = loop_ub;
    t_out_size[1] = 7;
    for (i = 0; i < 7; i++) {
      for (i1 = 0; i1 < loop_ub; i1++) {
        t_out_data[i1 + loop_ub * i] = t_out[i1 + 100 * i];
      }
    }

    coder::sum(t_out_data, t_out_size, a__1_data, &varargin_1);
    coder::internal::sort(a__1_data, &varargin_1, iidx_data, solution_out_size);
    for (i = 0; i < *solution_out_size; i++) {
      a__1_data[i] = iidx_data[i];
    }

    t_out_size[0] = *solution_out_size;
    t_out_size[1] = 7;
    for (i = 0; i < 7; i++) {
      for (i1 = 0; i1 < *solution_out_size; i1++) {
        i2 = static_cast<int>(a__1_data[i1]);
        if ((i2 < 1) || (i2 > loop_ub)) {
          rtDynamicBoundsError(i2, 1, loop_ub, &fb_emlrtBCI);
        }

        t_out_data[i1 + *solution_out_size * i] = t_out[(i2 + 100 * i) - 1];
      }
    }

    J_out_size[0] = *solution_out_size;
    J_out_size[1] = 7;
    for (i = 0; i < 7; i++) {
      for (i1 = 0; i1 < *solution_out_size; i1++) {
        i2 = static_cast<int>(a__1_data[i1]);
        if ((i2 < 1) || (i2 > idx)) {
          rtDynamicBoundsError(i2, 1, idx, &gb_emlrtBCI);
        }

        J_out_data[i1 + *solution_out_size * i] = J_out[(i2 + 100 * i) - 1];
      }
    }

    for (i = 0; i < *solution_out_size; i++) {
      i1 = static_cast<int>(a__1_data[i]);
      if ((i1 < 1) || (i1 > b_index)) {
        rtDynamicBoundsError(i1, 1, b_index, &hb_emlrtBCI);
      }

      solution_out_data[i] = solution_out[i1 - 1];
    }
  }
}

void solve_O(double P_init, double V_init, double A_init, double V_wayp, double
             V_min, double A_max, double A_min, double J_max, double J_min,
             double t_out_data[], int t_out_size[2], double J_out_data[], int
             J_out_size[2], int solution_out_data[], int *solution_out_size)
{
  creal_T t_test_data[35];
  creal_T dcv1[28];
  creal_T dcv2[21];
  creal_T dcv[14];
  creal_T b_A_init[7];
  double J_out[700];
  double t_out[700];
  double a__1_data[100];
  double dv[14];
  double J_out_simpl_data[7];
  double J_test[7];
  double J_test_simpl_data[7];
  double b_J_out[7];
  double b_varargin_1[7];
  double t_out_simpl_data[7];
  double t_test_simpl_data[7];
  double t_test_valid[7];
  int iidx_data[100];
  int solution_out[100];
  int cases_data[45];
  int J_out_simpl_size[2];
  int J_test_simpl_size[2];
  int b_t_test_simpl_size[2];
  int c_t_test_simpl_size[2];
  int cases_size[2];
  int t_out_simpl_size[2];
  int t_test_simpl_size[2];
  int t_test_size[2];
  int valid_size[2];
  int b_index;
  int i;
  int i1;
  int i2;
  int idx;
  int loop_ub;
  int num_valid;
  int varargin_1;
  bool b_t_test_simpl_data[7];
  bool valid_data[5];

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
  num_valid = 0;
  std::memset(&t_out[0], 0, 700U * sizeof(double));
  std::memset(&J_out[0], 0, 700U * sizeof(double));
  std::memset(&solution_out[0], 0, 100U * sizeof(int));

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
  switch (check_feasibility(V_init, A_init, rtInf, V_min, A_max, A_min, J_max,
           J_min)) {
   case 3U:
    varargin_1 = 2;
    break;

   case 5U:
    varargin_1 = 2;
    break;

   case 4U:
    varargin_1 = 3;
    break;

   case 6U:
    varargin_1 = 3;
    break;

   default:
    varargin_1 = 1;
    break;
  }

  varargin_1 = coder::eml_sub2ind(static_cast<double>(varargin_1), static_cast<
    double>(!rtIsInf(V_min)) + 1.0, static_cast<double>(!rtIsInf(A_max)) + 1.0,
    static_cast<double>(!rtIsInf(A_min)) + 1.0);
  select_cases_O(nlut[!rtIsNaN(V_wayp) << 1], condlut[varargin_1 - 1],
                 cases_data, cases_size);
  i = cases_size[1];
  for (b_index = 0; b_index < i; b_index++) {
    if (b_index + 1 > cases_size[1]) {
      rtDynamicBoundsError(b_index + 1, 1, cases_size[1], &bb_emlrtBCI);
    }

    i1 = cases_data[b_index];
    if (coder::b_mod(std::floor(static_cast<double>(i1) / 1000.0)) == 0.0) {
      if (i1 - ((i1 >> 1) << 1) == 1) {
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
    } else if (i1 - ((i1 >> 1) << 1) == 1) {
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

    switch (i1) {
     case 0:
      t_test_size[0] = 1;
      t_test_size[1] = 7;
      std::memset(&t_test_data[0], 0, 7U * sizeof(creal_T));
      break;

     case 101:
      abcdefg_O_AVP(P_init, V_init, A_init, rtNaN, V_wayp, rtNaN, rtInf, A_max,
                    A_min, J_max, J_min, b_varargin_1);
      t_test_size[0] = 1;
      t_test_size[1] = 7;
      for (i1 = 0; i1 < 7; i1++) {
        t_test_data[i1].re = b_varargin_1[i1];
        t_test_data[i1].im = 0.0;
      }
      break;

     case 102:
      abcdefg_O_AVP(-P_init, -V_init, -A_init, rtNaN, -V_wayp, rtNaN, -V_min,
                    -A_min, -A_max, -J_min, -J_max, b_varargin_1);
      t_test_size[0] = 1;
      t_test_size[1] = 7;
      for (i1 = 0; i1 < 7; i1++) {
        t_test_data[i1].re = b_varargin_1[i1];
        t_test_data[i1].im = 0.0;
      }
      break;

     case 103:
      abcdeg_O_AVP(P_init, V_init, A_init, rtNaN, V_wayp, rtNaN, rtInf, A_max,
                   J_max, J_min, dcv);
      t_test_size[0] = 2;
      t_test_size[1] = 7;
      std::copy(&dcv[0], &dcv[14], &t_test_data[0]);
      break;

     case 104:
      abcdeg_O_AVP(-P_init, -V_init, -A_init, rtNaN, -V_wayp, rtNaN, -V_min,
                   -A_min, -J_min, -J_max, dcv);
      t_test_size[0] = 2;
      t_test_size[1] = 7;
      std::copy(&dcv[0], &dcv[14], &t_test_data[0]);
      break;

     case 105:
      abcefg_O_AVP(P_init, V_init, A_init, rtNaN, V_wayp, rtNaN, A_max, A_min,
                   J_max, J_min, dcv);
      t_test_size[0] = 2;
      t_test_size[1] = 7;
      std::copy(&dcv[0], &dcv[14], &t_test_data[0]);
      break;

     case 106:
      abcefg_O_AVP(-P_init, -V_init, -A_init, rtNaN, -V_wayp, rtNaN, -A_min,
                   -A_max, -J_min, -J_max, dcv);
      t_test_size[0] = 2;
      t_test_size[1] = 7;
      std::copy(&dcv[0], &dcv[14], &t_test_data[0]);
      break;

     case 107:
      abceg_O_AVP(P_init, V_init, A_init, rtNaN, V_wayp, rtNaN, A_max, J_max,
                  J_min, dcv1);
      t_test_size[0] = 4;
      t_test_size[1] = 7;
      std::copy(&dcv1[0], &dcv1[28], &t_test_data[0]);
      break;

     case 108:
      abceg_O_AVP(-P_init, -V_init, -A_init, rtNaN, -V_wayp, rtNaN, -A_min,
                  -J_min, -J_max, dcv1);
      t_test_size[0] = 4;
      t_test_size[1] = 7;
      std::copy(&dcv1[0], &dcv1[28], &t_test_data[0]);
      break;

     case 109:
      acdefg_O_AVP(P_init, V_init, A_init, rtNaN, V_wayp, rtNaN, rtInf, A_min,
                   J_max, J_min, dcv);
      t_test_size[0] = 2;
      t_test_size[1] = 7;
      std::copy(&dcv[0], &dcv[14], &t_test_data[0]);
      break;

     case 110:
      acdefg_O_AVP(-P_init, -V_init, -A_init, rtNaN, -V_wayp, rtNaN, -V_min,
                   -A_max, -J_min, -J_max, dcv);
      t_test_size[0] = 2;
      t_test_size[1] = 7;
      std::copy(&dcv[0], &dcv[14], &t_test_data[0]);
      break;

     case 111:
      acdeg_O_AVP(P_init, V_init, A_init, rtNaN, V_wayp, rtNaN, rtInf, J_max,
                  J_min, dcv1);
      t_test_size[0] = 4;
      t_test_size[1] = 7;
      std::copy(&dcv1[0], &dcv1[28], &t_test_data[0]);
      break;

     case 112:
      acdeg_O_AVP(-P_init, -V_init, -A_init, rtNaN, -V_wayp, rtNaN, -V_min,
                  -J_min, -J_max, dcv1);
      t_test_size[0] = 4;
      t_test_size[1] = 7;
      std::copy(&dcv1[0], &dcv1[28], &t_test_data[0]);
      break;

     case 113:
      acefg_O_AVP(P_init, V_init, A_init, rtNaN, V_wayp, rtNaN, A_min, J_max,
                  J_min, dcv1);
      t_test_size[0] = 4;
      t_test_size[1] = 7;
      std::copy(&dcv1[0], &dcv1[28], &t_test_data[0]);
      break;

     case 114:
      acefg_O_AVP(-P_init, -V_init, -A_init, rtNaN, -V_wayp, rtNaN, -A_max,
                  -J_min, -J_max, dcv1);
      t_test_size[0] = 4;
      t_test_size[1] = 7;
      std::copy(&dcv1[0], &dcv1[28], &t_test_data[0]);
      break;

     case 115:
      aceg_O_AVP(P_init, V_init, A_init, rtNaN, V_wayp, rtNaN, J_max, J_min,
                 t_test_data);
      t_test_size[0] = 5;
      t_test_size[1] = 7;
      break;

     case 116:
      aceg_O_AVP(-P_init, -V_init, -A_init, rtNaN, -V_wayp, rtNaN, -J_min,
                 -J_max, t_test_data);
      t_test_size[0] = 5;
      t_test_size[1] = 7;

      // -----------------------
      break;

     case 201:
      abc_O_VP(P_init, V_init, A_init, rtNaN, V_wayp, A_max, J_max, J_min, dcv1);
      t_test_size[0] = 4;
      t_test_size[1] = 7;
      std::copy(&dcv1[0], &dcv1[28], &t_test_data[0]);
      break;

     case 202:
      abc_O_VP(-P_init, -V_init, -A_init, rtNaN, -V_wayp, -A_min, -J_min, -J_max,
               dcv1);
      t_test_size[0] = 4;
      t_test_size[1] = 7;
      std::copy(&dcv1[0], &dcv1[28], &t_test_data[0]);
      break;

     case 203:
      abcde_O_VP(P_init, V_init, A_init, rtNaN, V_wayp, rtInf, A_max, J_max,
                 J_min, dcv);
      t_test_size[0] = 2;
      t_test_size[1] = 7;
      std::copy(&dcv[0], &dcv[14], &t_test_data[0]);
      break;

     case 204:
      abcde_O_VP(-P_init, -V_init, -A_init, rtNaN, -V_wayp, -V_min, -A_min,
                 -J_min, -J_max, dcv);
      t_test_size[0] = 2;
      t_test_size[1] = 7;
      std::copy(&dcv[0], &dcv[14], &t_test_data[0]);
      break;

     case 205:
      abcdef_O_VP(P_init, V_init, A_init, rtNaN, V_wayp, rtInf, A_max, A_min,
                  J_max, J_min, b_varargin_1);
      t_test_size[0] = 1;
      t_test_size[1] = 7;
      for (i1 = 0; i1 < 7; i1++) {
        t_test_data[i1].re = b_varargin_1[i1];
        t_test_data[i1].im = 0.0;
      }
      break;

     case 206:
      abcdef_O_VP(-P_init, -V_init, -A_init, rtNaN, -V_wayp, -V_min, -A_min,
                  -A_max, -J_min, -J_max, b_varargin_1);
      t_test_size[0] = 1;
      t_test_size[1] = 7;
      for (i1 = 0; i1 < 7; i1++) {
        t_test_data[i1].re = b_varargin_1[i1];
        t_test_data[i1].im = 0.0;
      }
      break;

     case 207:
      abcdefg_O_VP(P_init, V_init, A_init, rtNaN, V_wayp, rtInf, V_min, A_max,
                   A_min, J_max, J_min, dv);
      t_test_size[0] = 2;
      t_test_size[1] = 7;
      for (i1 = 0; i1 < 14; i1++) {
        t_test_data[i1].re = dv[i1];
        t_test_data[i1].im = 0.0;
      }
      break;

     case 208:
      abcdefg_O_VP(-P_init, -V_init, -A_init, rtNaN, -V_wayp, -V_min, rtMinusInf,
                   -A_min, -A_max, -J_min, -J_max, dv);
      t_test_size[0] = 2;
      t_test_size[1] = 7;
      for (i1 = 0; i1 < 14; i1++) {
        t_test_data[i1].re = dv[i1];
        t_test_data[i1].im = 0.0;
      }
      break;

     case 209:
      abcdeg_O_VP(P_init, V_init, A_init, rtNaN, V_wayp, rtInf, V_min, A_max,
                  J_max, J_min, dcv);
      t_test_size[0] = 2;
      t_test_size[1] = 7;
      std::copy(&dcv[0], &dcv[14], &t_test_data[0]);
      break;

     case 210:
      abcdeg_O_VP(-P_init, -V_init, -A_init, rtNaN, -V_wayp, -V_min, rtMinusInf,
                  -A_min, -J_min, -J_max, dcv);
      t_test_size[0] = 2;
      t_test_size[1] = 7;
      std::copy(&dcv[0], &dcv[14], &t_test_data[0]);
      break;

     case 211:
      abcef_O_VP(P_init, V_init, A_init, rtNaN, V_wayp, A_max, A_min, J_max,
                 J_min, dcv);
      t_test_size[0] = 2;
      t_test_size[1] = 7;
      std::copy(&dcv[0], &dcv[14], &t_test_data[0]);
      break;

     case 212:
      abcef_O_VP(-P_init, -V_init, -A_init, rtNaN, -V_wayp, -A_min, -A_max,
                 -J_min, -J_max, dcv);
      t_test_size[0] = 2;
      t_test_size[1] = 7;
      std::copy(&dcv[0], &dcv[14], &t_test_data[0]);
      break;

     case 213:
      t_test_size[0] = 1;
      t_test_size[1] = 7;
      std::memset(&t_test_data[0], 0, 7U * sizeof(creal_T));
      break;

     case 214:
      t_test_size[0] = 1;
      t_test_size[1] = 7;
      std::memset(&t_test_data[0], 0, 7U * sizeof(creal_T));
      break;

     case 215:
      t_test_size[0] = 1;
      t_test_size[1] = 7;
      std::memset(&t_test_data[0], 0, 7U * sizeof(creal_T));
      break;

     case 216:
      t_test_size[0] = 1;
      t_test_size[1] = 7;
      std::memset(&t_test_data[0], 0, 7U * sizeof(creal_T));
      break;

     case 217:
      t_test_size[0] = 1;
      t_test_size[1] = 7;
      std::memset(&t_test_data[0], 0, 7U * sizeof(creal_T));
      break;

     case 218:
      t_test_size[0] = 1;
      t_test_size[1] = 7;
      std::memset(&t_test_data[0], 0, 7U * sizeof(creal_T));
      break;

     case 219:
      acde_O_VP(P_init, V_init, A_init, rtNaN, V_wayp, rtInf, J_max, J_min, dcv);
      t_test_size[0] = 2;
      t_test_size[1] = 7;
      std::copy(&dcv[0], &dcv[14], &t_test_data[0]);
      break;

     case 220:
      acde_O_VP(-P_init, -V_init, -A_init, rtNaN, -V_wayp, -V_min, -J_min,
                -J_max, dcv);
      t_test_size[0] = 2;
      t_test_size[1] = 7;
      std::copy(&dcv[0], &dcv[14], &t_test_data[0]);
      break;

     case 221:
      acdef_O_VP(P_init, V_init, A_init, rtNaN, V_wayp, rtInf, A_min, J_max,
                 J_min, dcv);
      t_test_size[0] = 2;
      t_test_size[1] = 7;
      std::copy(&dcv[0], &dcv[14], &t_test_data[0]);
      break;

     case 222:
      acdef_O_VP(-P_init, -V_init, -A_init, rtNaN, -V_wayp, -V_min, -A_max,
                 -J_min, -J_max, dcv);
      t_test_size[0] = 2;
      t_test_size[1] = 7;
      std::copy(&dcv[0], &dcv[14], &t_test_data[0]);
      break;

     case 223:
      acdefg_O_VP(P_init, V_init, A_init, rtNaN, V_wayp, rtInf, V_min, A_min,
                  J_max, J_min, dcv1);
      t_test_size[0] = 4;
      t_test_size[1] = 7;
      std::copy(&dcv1[0], &dcv1[28], &t_test_data[0]);
      break;

     case 224:
      acdefg_O_VP(-P_init, -V_init, -A_init, rtNaN, -V_wayp, -V_min, rtMinusInf,
                  -A_max, -J_min, -J_max, dcv1);
      t_test_size[0] = 4;
      t_test_size[1] = 7;
      std::copy(&dcv1[0], &dcv1[28], &t_test_data[0]);
      break;

     case 225:
      t_test_size[0] = 1;
      t_test_size[1] = 7;
      std::memset(&t_test_data[0], 0, 7U * sizeof(creal_T));
      break;

     case 226:
      t_test_size[0] = 1;
      t_test_size[1] = 7;
      std::memset(&t_test_data[0], 0, 7U * sizeof(creal_T));
      break;

     case 227:
      acef_O_VP(P_init, V_init, A_init, rtNaN, V_wayp, A_min, J_max, J_min, dcv1);
      t_test_size[0] = 4;
      t_test_size[1] = 7;
      std::copy(&dcv1[0], &dcv1[28], &t_test_data[0]);
      break;

     case 228:
      acef_O_VP(-P_init, -V_init, -A_init, rtNaN, -V_wayp, -A_max, -J_min,
                -J_max, dcv1);
      t_test_size[0] = 4;
      t_test_size[1] = 7;
      std::copy(&dcv1[0], &dcv1[28], &t_test_data[0]);
      break;

     case 229:
      t_test_size[0] = 1;
      t_test_size[1] = 7;
      std::memset(&t_test_data[0], 0, 7U * sizeof(creal_T));
      break;

     case 230:
      t_test_size[0] = 1;
      t_test_size[1] = 7;
      std::memset(&t_test_data[0], 0, 7U * sizeof(creal_T));
      break;

     case 231:
      t_test_size[0] = 1;
      t_test_size[1] = 7;
      std::memset(&t_test_data[0], 0, 7U * sizeof(creal_T));
      break;

     case 232:
      t_test_size[0] = 1;
      t_test_size[1] = 7;
      std::memset(&t_test_data[0], 0, 7U * sizeof(creal_T));

      // -----------------------
      break;

     case 301:
      abc_O_AP(P_init, V_init, A_init, rtNaN, rtNaN, A_max, J_max, J_min, dcv);
      t_test_size[0] = 2;
      t_test_size[1] = 7;
      std::copy(&dcv[0], &dcv[14], &t_test_data[0]);
      break;

     case 302:
      abc_O_AP(-P_init, -V_init, -A_init, rtNaN, rtNaN, -A_min, -J_min, -J_max,
               dcv);
      t_test_size[0] = 2;
      t_test_size[1] = 7;
      std::copy(&dcv[0], &dcv[14], &t_test_data[0]);
      break;

     case 303:
      abcde_O_AP(P_init, V_init, A_init, rtNaN, rtNaN, rtInf, A_max, J_max,
                 J_min, b_varargin_1);
      t_test_size[0] = 1;
      t_test_size[1] = 7;
      for (i1 = 0; i1 < 7; i1++) {
        t_test_data[i1].re = b_varargin_1[i1];
        t_test_data[i1].im = 0.0;
      }
      break;

     case 304:
      abcde_O_AP(-P_init, -V_init, -A_init, rtNaN, rtNaN, -V_min, -A_min, -J_min,
                 -J_max, b_varargin_1);
      t_test_size[0] = 1;
      t_test_size[1] = 7;
      for (i1 = 0; i1 < 7; i1++) {
        t_test_data[i1].re = b_varargin_1[i1];
        t_test_data[i1].im = 0.0;
      }
      break;

     case 305:
      abcdefg_O_AP(P_init, V_init, A_init, rtNaN, rtNaN, rtInf, A_max, A_min,
                   J_max, J_min, b_varargin_1);
      t_test_size[0] = 1;
      t_test_size[1] = 7;
      for (i1 = 0; i1 < 7; i1++) {
        t_test_data[i1].re = b_varargin_1[i1];
        t_test_data[i1].im = 0.0;
      }
      break;

     case 306:
      abcdefg_O_AP(-P_init, -V_init, -A_init, rtNaN, rtNaN, -V_min, -A_min,
                   -A_max, -J_min, -J_max, b_varargin_1);
      t_test_size[0] = 1;
      t_test_size[1] = 7;
      for (i1 = 0; i1 < 7; i1++) {
        t_test_data[i1].re = b_varargin_1[i1];
        t_test_data[i1].im = 0.0;
      }
      break;

     case 307:
      abcefg_O_AP(P_init, V_init, A_init, rtNaN, rtNaN, rtInf, A_max, A_min,
                  J_max, J_min, dcv);
      t_test_size[0] = 2;
      t_test_size[1] = 7;
      std::copy(&dcv[0], &dcv[14], &t_test_data[0]);
      break;

     case 308:
      abcefg_O_AP(-P_init, -V_init, -A_init, rtNaN, rtNaN, -V_min, -A_min,
                  -A_max, -J_min, -J_max, dcv);
      t_test_size[0] = 2;
      t_test_size[1] = 7;
      std::copy(&dcv[0], &dcv[14], &t_test_data[0]);
      break;

     case 309:
      abceg_O_AP(P_init, V_init, A_init, rtNaN, rtNaN, rtInf, A_max, J_max,
                 J_min, dcv1);
      t_test_size[0] = 4;
      t_test_size[1] = 7;
      std::copy(&dcv1[0], &dcv1[28], &t_test_data[0]);
      break;

     case 310:
      abceg_O_AP(-P_init, -V_init, -A_init, rtNaN, rtNaN, -V_min, -A_min, -J_min,
                 -J_max, dcv1);
      t_test_size[0] = 4;
      t_test_size[1] = 7;
      std::copy(&dcv1[0], &dcv1[28], &t_test_data[0]);
      break;

     case 311:
      ac_O_AP(P_init, V_init, A_init, rtNaN, rtNaN, J_max, J_min, dcv2);
      t_test_size[0] = 3;
      t_test_size[1] = 7;
      std::copy(&dcv2[0], &dcv2[21], &t_test_data[0]);
      break;

     case 312:
      ac_O_AP(-P_init, -V_init, -A_init, rtNaN, rtNaN, -J_min, -J_max, dcv2);
      t_test_size[0] = 3;
      t_test_size[1] = 7;
      std::copy(&dcv2[0], &dcv2[21], &t_test_data[0]);
      break;

     case 313:
      acde_O_AP(P_init, V_init, A_init, rtNaN, rtNaN, rtInf, J_max, J_min, dcv);
      t_test_size[0] = 2;
      t_test_size[1] = 7;
      std::copy(&dcv[0], &dcv[14], &t_test_data[0]);
      break;

     case 314:
      acde_O_AP(-P_init, -V_init, -A_init, rtNaN, rtNaN, -V_min, -J_min, -J_max,
                dcv);
      t_test_size[0] = 2;
      t_test_size[1] = 7;
      std::copy(&dcv[0], &dcv[14], &t_test_data[0]);
      break;

     case 315:
      acdefg_O_AP(P_init, V_init, A_init, rtNaN, rtNaN, rtInf, A_min, J_max,
                  J_min, dcv);
      t_test_size[0] = 2;
      t_test_size[1] = 7;
      std::copy(&dcv[0], &dcv[14], &t_test_data[0]);
      break;

     case 316:
      acdefg_O_AP(-P_init, -V_init, -A_init, rtNaN, rtNaN, -V_min, -A_max,
                  -J_min, -J_max, dcv);
      t_test_size[0] = 2;
      t_test_size[1] = 7;
      std::copy(&dcv[0], &dcv[14], &t_test_data[0]);
      break;

     case 317:
      acefg_O_AP(P_init, V_init, A_init, rtNaN, rtNaN, rtInf, A_min, J_max,
                 J_min, dcv1);
      t_test_size[0] = 4;
      t_test_size[1] = 7;
      std::copy(&dcv1[0], &dcv1[28], &t_test_data[0]);
      break;

     case 318:
      acefg_O_AP(-P_init, -V_init, -A_init, rtNaN, rtNaN, -V_min, -A_max, -J_min,
                 -J_max, dcv1);
      t_test_size[0] = 4;
      t_test_size[1] = 7;
      std::copy(&dcv1[0], &dcv1[28], &t_test_data[0]);
      break;

     case 319:
      aceg_O_AP(P_init, V_init, A_init, rtNaN, rtNaN, rtInf, J_max, J_min, dcv1);
      t_test_size[0] = 4;
      t_test_size[1] = 7;
      std::copy(&dcv1[0], &dcv1[28], &t_test_data[0]);
      break;

     case 320:
      aceg_O_AP(-P_init, -V_init, -A_init, rtNaN, rtNaN, -V_min, -J_min, -J_max,
                dcv1);
      t_test_size[0] = 4;
      t_test_size[1] = 7;
      std::copy(&dcv1[0], &dcv1[28], &t_test_data[0]);
      break;

     case 321:
      abcdeg_O_AP(P_init, V_init, A_init, rtNaN, rtNaN, rtInf, A_max, J_max,
                  J_min, dv);
      t_test_size[0] = 2;
      t_test_size[1] = 7;
      for (i1 = 0; i1 < 14; i1++) {
        t_test_data[i1].re = dv[i1];
        t_test_data[i1].im = 0.0;
      }
      break;

     case 322:
      abcdeg_O_AP(-P_init, -V_init, -A_init, rtNaN, rtNaN, -V_min, -A_min,
                  -J_min, -J_max, dv);
      t_test_size[0] = 2;
      t_test_size[1] = 7;
      for (i1 = 0; i1 < 14; i1++) {
        t_test_data[i1].re = dv[i1];
        t_test_data[i1].im = 0.0;
      }
      break;

     case 323:
      acdeg_O_AP(P_init, V_init, A_init, rtNaN, rtNaN, rtInf, J_max, J_min, dcv);
      t_test_size[0] = 2;
      t_test_size[1] = 7;
      std::copy(&dcv[0], &dcv[14], &t_test_data[0]);
      break;

     case 324:
      acdeg_O_AP(-P_init, -V_init, -A_init, rtNaN, rtNaN, -V_min, -J_min, -J_max,
                 dcv);
      t_test_size[0] = 2;
      t_test_size[1] = 7;
      std::copy(&dcv[0], &dcv[14], &t_test_data[0]);

      // -----------------------
      break;

     case 401:
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
      //  Generated on 28-Aug-2019 17:25:45
      b_A_init[0].re = -(A_init - A_max) / J_max;
      b_A_init[0].im = 0.0;
      b_A_init[1].re = rtNaN;
      b_A_init[1].im = 0.0;
      b_A_init[2].re = rtNaN;
      b_A_init[2].im = 0.0;
      b_A_init[3].re = 0.0;
      b_A_init[3].im = 0.0;
      b_A_init[4].re = 0.0;
      b_A_init[4].im = 0.0;
      b_A_init[5].re = 0.0;
      b_A_init[5].im = 0.0;
      b_A_init[6].re = 0.0;
      b_A_init[6].im = 0.0;
      t_test_size[0] = 1;
      t_test_size[1] = 7;
      std::copy(&b_A_init[0], &b_A_init[7], &t_test_data[0]);
      break;

     case 402:
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
      //  Generated on 28-Aug-2019 17:25:45
      b_A_init[0].re = -(-A_init - (-A_min)) / -J_min;
      b_A_init[0].im = 0.0;
      b_A_init[1].re = rtNaN;
      b_A_init[1].im = 0.0;
      b_A_init[2].re = rtNaN;
      b_A_init[2].im = 0.0;
      b_A_init[3].re = 0.0;
      b_A_init[3].im = 0.0;
      b_A_init[4].re = 0.0;
      b_A_init[4].im = 0.0;
      b_A_init[5].re = 0.0;
      b_A_init[5].im = 0.0;
      b_A_init[6].re = 0.0;
      b_A_init[6].im = 0.0;
      t_test_size[0] = 1;
      t_test_size[1] = 7;
      std::copy(&b_A_init[0], &b_A_init[7], &t_test_data[0]);
      break;

     case 403:
      ac_O_AV(V_init, A_init, V_wayp, rtNaN, J_max, J_min, dcv);
      t_test_size[0] = 2;
      t_test_size[1] = 7;
      std::copy(&dcv[0], &dcv[14], &t_test_data[0]);
      break;

     case 404:
      ac_O_AV(-V_init, -A_init, -V_wayp, rtNaN, -J_min, -J_max, dcv);
      t_test_size[0] = 2;
      t_test_size[1] = 7;
      std::copy(&dcv[0], &dcv[14], &t_test_data[0]);
      break;

     case 501:
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
      //  Generated on 28-Aug-2019 17:25:45
      b_A_init[0].re = rtNaN;
      b_A_init[0].im = 0.0;
      b_A_init[1].re = 0.0;
      b_A_init[1].im = 0.0;
      b_A_init[2].re = 0.0;
      b_A_init[2].im = 0.0;
      b_A_init[3].re = 0.0;
      b_A_init[3].im = 0.0;
      b_A_init[4].re = 0.0;
      b_A_init[4].im = 0.0;
      b_A_init[5].re = 0.0;
      b_A_init[5].im = 0.0;
      b_A_init[6].re = 0.0;
      b_A_init[6].im = 0.0;
      t_test_size[0] = 1;
      t_test_size[1] = 7;
      std::copy(&b_A_init[0], &b_A_init[7], &t_test_data[0]);
      break;

     case 502:
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
      //  Generated on 28-Aug-2019 17:25:45
      b_A_init[0].re = rtNaN;
      b_A_init[0].im = 0.0;
      b_A_init[1].re = 0.0;
      b_A_init[1].im = 0.0;
      b_A_init[2].re = 0.0;
      b_A_init[2].im = 0.0;
      b_A_init[3].re = 0.0;
      b_A_init[3].im = 0.0;
      b_A_init[4].re = 0.0;
      b_A_init[4].im = 0.0;
      b_A_init[5].re = 0.0;
      b_A_init[5].im = 0.0;
      b_A_init[6].re = 0.0;
      b_A_init[6].im = 0.0;
      t_test_size[0] = 1;
      t_test_size[1] = 7;
      std::copy(&b_A_init[0], &b_A_init[7], &t_test_data[0]);
      break;

     case 503:
      abc_O_A(V_init, A_init, rtNaN, V_min, A_max, J_max, J_min, b_varargin_1);
      t_test_size[0] = 1;
      t_test_size[1] = 7;
      for (i1 = 0; i1 < 7; i1++) {
        t_test_data[i1].re = b_varargin_1[i1];
        t_test_data[i1].im = 0.0;
      }
      break;

     case 504:
      abc_O_A(-V_init, -A_init, rtNaN, rtMinusInf, -A_min, -J_min, -J_max,
              b_varargin_1);
      t_test_size[0] = 1;
      t_test_size[1] = 7;
      for (i1 = 0; i1 < 7; i1++) {
        t_test_data[i1].re = b_varargin_1[i1];
        t_test_data[i1].im = 0.0;
      }
      break;

     case 505:
      ac_O_A(V_init, A_init, rtNaN, V_min, J_max, J_min, dcv);
      t_test_size[0] = 2;
      t_test_size[1] = 7;
      std::copy(&dcv[0], &dcv[14], &t_test_data[0]);
      break;

     case 506:
      ac_O_A(-V_init, -A_init, rtNaN, rtMinusInf, -J_min, -J_max, dcv);
      t_test_size[0] = 2;
      t_test_size[1] = 7;
      std::copy(&dcv[0], &dcv[14], &t_test_data[0]);

      // -----------------------
      break;

     case 601:
      a_O_V(V_init, A_init, V_wayp, J_max, dcv);
      t_test_size[0] = 2;
      t_test_size[1] = 7;
      std::copy(&dcv[0], &dcv[14], &t_test_data[0]);
      break;

     case 602:
      a_O_V(-V_init, -A_init, -V_wayp, -J_min, dcv);
      t_test_size[0] = 2;
      t_test_size[1] = 7;
      std::copy(&dcv[0], &dcv[14], &t_test_data[0]);
      break;

     case 603:
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
      //  Generated on 28-Aug-2019 17:25:45
      b_A_init[0].re = -(A_init - A_max) / J_max;
      b_A_init[0].im = 0.0;
      b_A_init[1].re = (((J_max * V_init * 2.0 - J_max * V_wayp * 2.0) - A_init *
                         A_init) + A_max * A_max) * -0.5 / (A_max * J_max);
      b_A_init[1].im = 0.0;
      b_A_init[2].re = 0.0;
      b_A_init[2].im = 0.0;
      b_A_init[3].re = 0.0;
      b_A_init[3].im = 0.0;
      b_A_init[4].re = 0.0;
      b_A_init[4].im = 0.0;
      b_A_init[5].re = 0.0;
      b_A_init[5].im = 0.0;
      b_A_init[6].re = 0.0;
      b_A_init[6].im = 0.0;
      t_test_size[0] = 1;
      t_test_size[1] = 7;
      std::copy(&b_A_init[0], &b_A_init[7], &t_test_data[0]);
      break;

     case 604:
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
      //  Generated on 28-Aug-2019 17:25:45
      b_A_init[0].re = -(-A_init - (-A_min)) / -J_min;
      b_A_init[0].im = 0.0;
      b_A_init[1].re = (((-J_min * -V_init * 2.0 - -J_min * -V_wayp * 2.0) -
                         -A_init * -A_init) + -A_min * -A_min) * -0.5 / (-A_min *
        -J_min);
      b_A_init[1].im = 0.0;
      b_A_init[2].re = 0.0;
      b_A_init[2].im = 0.0;
      b_A_init[3].re = 0.0;
      b_A_init[3].im = 0.0;
      b_A_init[4].re = 0.0;
      b_A_init[4].im = 0.0;
      b_A_init[5].re = 0.0;
      b_A_init[5].im = 0.0;
      b_A_init[6].re = 0.0;
      b_A_init[6].im = 0.0;
      t_test_size[0] = 1;
      t_test_size[1] = 7;
      std::copy(&b_A_init[0], &b_A_init[7], &t_test_data[0]);
      break;

     case 605:
      abc_O_V(V_init, A_init, V_wayp, rtInf, A_max, J_max, J_min, dcv);
      t_test_size[0] = 2;
      t_test_size[1] = 7;
      std::copy(&dcv[0], &dcv[14], &t_test_data[0]);
      break;

     case 606:
      abc_O_V(-V_init, -A_init, -V_wayp, -V_min, -A_min, -J_min, -J_max, dcv);
      t_test_size[0] = 2;
      t_test_size[1] = 7;
      std::copy(&dcv[0], &dcv[14], &t_test_data[0]);
      break;

     case 607:
      ac_O_V(V_init, A_init, V_wayp, rtInf, J_max, J_min, dcv);
      t_test_size[0] = 2;
      t_test_size[1] = 7;
      std::copy(&dcv[0], &dcv[14], &t_test_data[0]);
      break;

     case 608:
      ac_O_V(-V_init, -A_init, -V_wayp, -V_min, -J_min, -J_max, dcv);
      t_test_size[0] = 2;
      t_test_size[1] = 7;
      std::copy(&dcv[0], &dcv[14], &t_test_data[0]);

      // -----------------------
      break;

     case 701:
      a_O_P(P_init, V_init, A_init, rtNaN, J_max, dcv2);
      t_test_size[0] = 3;
      t_test_size[1] = 7;
      std::copy(&dcv2[0], &dcv2[21], &t_test_data[0]);
      break;

     case 702:
      a_O_P(-P_init, -V_init, -A_init, rtNaN, -J_min, dcv2);
      t_test_size[0] = 3;
      t_test_size[1] = 7;
      std::copy(&dcv2[0], &dcv2[21], &t_test_data[0]);
      break;

     case 703:
      ab_O_P(P_init, V_init, A_init, rtNaN, A_max, J_max, dcv);
      t_test_size[0] = 2;
      t_test_size[1] = 7;
      std::copy(&dcv[0], &dcv[14], &t_test_data[0]);
      break;

     case 704:
      ab_O_P(-P_init, -V_init, -A_init, rtNaN, -A_min, -J_min, dcv);
      t_test_size[0] = 2;
      t_test_size[1] = 7;
      std::copy(&dcv[0], &dcv[14], &t_test_data[0]);
      break;

     case 705:
      abc_O_P(P_init, V_init, A_init, rtNaN, rtInf, A_max, J_max, J_min, dcv2);
      t_test_size[0] = 3;
      t_test_size[1] = 7;
      std::copy(&dcv2[0], &dcv2[21], &t_test_data[0]);
      break;

     case 706:
      abc_O_P(-P_init, -V_init, -A_init, rtNaN, -V_min, -A_min, -J_min, -J_max,
              dcv2);
      t_test_size[0] = 3;
      t_test_size[1] = 7;
      std::copy(&dcv2[0], &dcv2[21], &t_test_data[0]);
      break;

     case 707:
      abcd_O_P(P_init, V_init, A_init, rtNaN, rtInf, A_max, J_max, J_min,
               b_varargin_1);
      t_test_size[0] = 1;
      t_test_size[1] = 7;
      for (i1 = 0; i1 < 7; i1++) {
        t_test_data[i1].re = b_varargin_1[i1];
        t_test_data[i1].im = 0.0;
      }
      break;

     case 708:
      abcd_O_P(-P_init, -V_init, -A_init, rtNaN, -V_min, -A_min, -J_min, -J_max,
               b_varargin_1);
      t_test_size[0] = 1;
      t_test_size[1] = 7;
      for (i1 = 0; i1 < 7; i1++) {
        t_test_data[i1].re = b_varargin_1[i1];
        t_test_data[i1].im = 0.0;
      }
      break;

     case 709:
      ac_O_P(P_init, V_init, A_init, rtNaN, rtInf, J_max, J_min, dcv2);
      t_test_size[0] = 3;
      t_test_size[1] = 7;
      std::copy(&dcv2[0], &dcv2[21], &t_test_data[0]);
      break;

     case 710:
      ac_O_P(-P_init, -V_init, -A_init, rtNaN, -V_min, -J_min, -J_max, dcv2);
      t_test_size[0] = 3;
      t_test_size[1] = 7;
      std::copy(&dcv2[0], &dcv2[21], &t_test_data[0]);
      break;

     case 711:
      acd_O_P(P_init, V_init, A_init, rtNaN, rtInf, J_max, J_min, dcv);
      t_test_size[0] = 2;
      t_test_size[1] = 7;
      std::copy(&dcv[0], &dcv[14], &t_test_data[0]);
      break;

     case 712:
      acd_O_P(-P_init, -V_init, -A_init, rtNaN, -V_min, -J_min, -J_max, dcv);
      t_test_size[0] = 2;
      t_test_size[1] = 7;
      std::copy(&dcv[0], &dcv[14], &t_test_data[0]);

      // -----------------------
      break;

     case 1101:
      abcdefg_NO_AVP(P_init, V_init, A_init, rtNaN, V_wayp, rtNaN, rtInf, A_min,
                     J_max, J_min, b_varargin_1);
      t_test_size[0] = 1;
      t_test_size[1] = 7;
      for (i1 = 0; i1 < 7; i1++) {
        t_test_data[i1].re = b_varargin_1[i1];
        t_test_data[i1].im = 0.0;
      }
      break;

     case 1102:
      abcdefg_NO_AVP(-P_init, -V_init, -A_init, rtNaN, -V_wayp, rtNaN, -V_min,
                     -A_max, -J_min, -J_max, b_varargin_1);
      t_test_size[0] = 1;
      t_test_size[1] = 7;
      for (i1 = 0; i1 < 7; i1++) {
        t_test_data[i1].re = b_varargin_1[i1];
        t_test_data[i1].im = 0.0;
      }
      break;

     case 1103:
      abcdeg_NO_AVP(P_init, V_init, A_init, rtNaN, V_wayp, rtNaN, rtInf, A_min,
                    J_max, J_min, dcv);
      t_test_size[0] = 2;
      t_test_size[1] = 7;
      std::copy(&dcv[0], &dcv[14], &t_test_data[0]);
      break;

     case 1104:
      abcdeg_NO_AVP(-P_init, -V_init, -A_init, rtNaN, -V_wayp, rtNaN, -V_min,
                    -A_max, -J_min, -J_max, dcv);
      t_test_size[0] = 2;
      t_test_size[1] = 7;
      std::copy(&dcv[0], &dcv[14], &t_test_data[0]);
      break;

     case 1105:
      acdefg_NO_AVP(P_init, V_init, A_init, rtNaN, V_wayp, rtNaN, rtInf, A_min,
                    J_max, J_min, dcv);
      t_test_size[0] = 2;
      t_test_size[1] = 7;
      std::copy(&dcv[0], &dcv[14], &t_test_data[0]);
      break;

     case 1106:
      acdefg_NO_AVP(-P_init, -V_init, -A_init, rtNaN, -V_wayp, rtNaN, -V_min,
                    -A_max, -J_min, -J_max, dcv);
      t_test_size[0] = 2;
      t_test_size[1] = 7;
      std::copy(&dcv[0], &dcv[14], &t_test_data[0]);
      break;

     case 1107:
      acdeg_NO_AVP(P_init, V_init, A_init, rtNaN, V_wayp, rtNaN, rtInf, J_max,
                   J_min, dcv1);
      t_test_size[0] = 4;
      t_test_size[1] = 7;
      std::copy(&dcv1[0], &dcv1[28], &t_test_data[0]);
      break;

     case 1108:
      acdeg_NO_AVP(-P_init, -V_init, -A_init, rtNaN, -V_wayp, rtNaN, -V_min,
                   -J_min, -J_max, dcv1);
      t_test_size[0] = 4;
      t_test_size[1] = 7;
      std::copy(&dcv1[0], &dcv1[28], &t_test_data[0]);

      // -----------------------
      break;

     case 1201:
      abcde_NO_VP(P_init, V_init, A_init, rtNaN, V_wayp, rtInf, A_min, J_max,
                  J_min, dcv);
      t_test_size[0] = 2;
      t_test_size[1] = 7;
      std::copy(&dcv[0], &dcv[14], &t_test_data[0]);
      break;

     case 1202:
      abcde_NO_VP(-P_init, -V_init, -A_init, rtNaN, -V_wayp, -V_min, -A_max,
                  -J_min, -J_max, dcv);
      t_test_size[0] = 2;
      t_test_size[1] = 7;
      std::copy(&dcv[0], &dcv[14], &t_test_data[0]);
      break;

     case 1203:
      abcdef_NO_VP(P_init, V_init, A_init, rtNaN, V_wayp, rtInf, A_min, J_max,
                   J_min, b_varargin_1);
      t_test_size[0] = 1;
      t_test_size[1] = 7;
      for (i1 = 0; i1 < 7; i1++) {
        t_test_data[i1].re = b_varargin_1[i1];
        t_test_data[i1].im = 0.0;
      }
      break;

     case 1204:
      abcdef_NO_VP(-P_init, -V_init, -A_init, rtNaN, -V_wayp, -V_min, -A_max,
                   -J_min, -J_max, b_varargin_1);
      t_test_size[0] = 1;
      t_test_size[1] = 7;
      for (i1 = 0; i1 < 7; i1++) {
        t_test_data[i1].re = b_varargin_1[i1];
        t_test_data[i1].im = 0.0;
      }
      break;

     case 1205:
      abcdefg_NO_VP(P_init, V_init, A_init, rtNaN, V_wayp, rtInf, V_min, A_min,
                    J_max, J_min, dv);
      t_test_size[0] = 2;
      t_test_size[1] = 7;
      for (i1 = 0; i1 < 14; i1++) {
        t_test_data[i1].re = dv[i1];
        t_test_data[i1].im = 0.0;
      }
      break;

     case 1206:
      abcdefg_NO_VP(-P_init, -V_init, -A_init, rtNaN, -V_wayp, -V_min,
                    rtMinusInf, -A_max, -J_min, -J_max, dv);
      t_test_size[0] = 2;
      t_test_size[1] = 7;
      for (i1 = 0; i1 < 14; i1++) {
        t_test_data[i1].re = dv[i1];
        t_test_data[i1].im = 0.0;
      }
      break;

     case 1207:
      abcdeg_NO_VP(P_init, V_init, A_init, rtNaN, V_wayp, rtInf, V_min, A_min,
                   J_max, J_min, dcv);
      t_test_size[0] = 2;
      t_test_size[1] = 7;
      std::copy(&dcv[0], &dcv[14], &t_test_data[0]);
      break;

     case 1208:
      abcdeg_NO_VP(-P_init, -V_init, -A_init, rtNaN, -V_wayp, -V_min, rtMinusInf,
                   -A_max, -J_min, -J_max, dcv);
      t_test_size[0] = 2;
      t_test_size[1] = 7;
      std::copy(&dcv[0], &dcv[14], &t_test_data[0]);
      break;

     case 1209:
      acde_NO_VP(P_init, V_init, A_init, rtNaN, V_wayp, rtInf, J_max, J_min, dcv);
      t_test_size[0] = 2;
      t_test_size[1] = 7;
      std::copy(&dcv[0], &dcv[14], &t_test_data[0]);
      break;

     case 1210:
      acde_NO_VP(-P_init, -V_init, -A_init, rtNaN, -V_wayp, -V_min, -J_min,
                 -J_max, dcv);
      t_test_size[0] = 2;
      t_test_size[1] = 7;
      std::copy(&dcv[0], &dcv[14], &t_test_data[0]);
      break;

     case 1211:
      acdef_NO_VP(P_init, V_init, A_init, rtNaN, V_wayp, rtInf, A_min, J_max,
                  J_min, dcv);
      t_test_size[0] = 2;
      t_test_size[1] = 7;
      std::copy(&dcv[0], &dcv[14], &t_test_data[0]);
      break;

     case 1212:
      acdef_NO_VP(-P_init, -V_init, -A_init, rtNaN, -V_wayp, -V_min, -A_max,
                  -J_min, -J_max, dcv);
      t_test_size[0] = 2;
      t_test_size[1] = 7;
      std::copy(&dcv[0], &dcv[14], &t_test_data[0]);
      break;

     case 1213:
      acdefg_NO_VP(P_init, V_init, A_init, rtNaN, V_wayp, rtInf, V_min, A_min,
                   J_max, J_min, dcv1);
      t_test_size[0] = 4;
      t_test_size[1] = 7;
      std::copy(&dcv1[0], &dcv1[28], &t_test_data[0]);
      break;

     case 1214:
      acdefg_NO_VP(-P_init, -V_init, -A_init, rtNaN, -V_wayp, -V_min, rtMinusInf,
                   -A_max, -J_min, -J_max, dcv1);
      t_test_size[0] = 4;
      t_test_size[1] = 7;
      std::copy(&dcv1[0], &dcv1[28], &t_test_data[0]);
      break;

     case 1215:
      t_test_size[0] = 1;
      t_test_size[1] = 7;
      std::memset(&t_test_data[0], 0, 7U * sizeof(creal_T));
      break;

     case 1216:
      t_test_size[0] = 1;
      t_test_size[1] = 7;
      std::memset(&t_test_data[0], 0, 7U * sizeof(creal_T));

      // -----------------------
      break;

     case 1301:
      abcde_NO_AP(P_init, V_init, A_init, rtNaN, rtNaN, rtInf, A_min, J_max,
                  J_min, b_varargin_1);
      t_test_size[0] = 1;
      t_test_size[1] = 7;
      for (i1 = 0; i1 < 7; i1++) {
        t_test_data[i1].re = b_varargin_1[i1];
        t_test_data[i1].im = 0.0;
      }
      break;

     case 1302:
      abcde_NO_AP(-P_init, -V_init, -A_init, rtNaN, rtNaN, -V_min, -A_max,
                  -J_min, -J_max, b_varargin_1);
      t_test_size[0] = 1;
      t_test_size[1] = 7;
      for (i1 = 0; i1 < 7; i1++) {
        t_test_data[i1].re = b_varargin_1[i1];
        t_test_data[i1].im = 0.0;
      }
      break;

     case 1303:
      acde_NO_AP(P_init, V_init, A_init, rtNaN, rtNaN, rtInf, J_max, J_min, dcv);
      t_test_size[0] = 2;
      t_test_size[1] = 7;
      std::copy(&dcv[0], &dcv[14], &t_test_data[0]);
      break;

     case 1304:
      acde_NO_AP(-P_init, -V_init, -A_init, rtNaN, rtNaN, -V_min, -J_min, -J_max,
                 dcv);
      t_test_size[0] = 2;
      t_test_size[1] = 7;
      std::copy(&dcv[0], &dcv[14], &t_test_data[0]);
      break;

     case 1305:
      abcdefg_NO_AP(P_init, V_init, A_init, rtNaN, rtNaN, rtInf, A_min, J_max,
                    J_min, b_varargin_1);
      t_test_size[0] = 1;
      t_test_size[1] = 7;
      for (i1 = 0; i1 < 7; i1++) {
        t_test_data[i1].re = b_varargin_1[i1];
        t_test_data[i1].im = 0.0;
      }
      break;

     case 1306:
      abcdefg_NO_AP(-P_init, -V_init, -A_init, rtNaN, rtNaN, -V_min, -A_max,
                    -J_min, -J_max, b_varargin_1);
      t_test_size[0] = 1;
      t_test_size[1] = 7;
      for (i1 = 0; i1 < 7; i1++) {
        t_test_data[i1].re = b_varargin_1[i1];
        t_test_data[i1].im = 0.0;
      }
      break;

     case 1307:
      acdefg_NO_AP(P_init, V_init, A_init, rtNaN, rtNaN, rtInf, A_min, J_max,
                   J_min, dcv);
      t_test_size[0] = 2;
      t_test_size[1] = 7;
      std::copy(&dcv[0], &dcv[14], &t_test_data[0]);
      break;

     case 1308:
      acdefg_NO_AP(-P_init, -V_init, -A_init, rtNaN, rtNaN, -V_min, -A_max,
                   -J_min, -J_max, dcv);
      t_test_size[0] = 2;
      t_test_size[1] = 7;
      std::copy(&dcv[0], &dcv[14], &t_test_data[0]);
      break;

     case 1309:
      abcdeg_NO_AP(P_init, V_init, A_init, rtNaN, rtNaN, rtInf, A_min, J_max,
                   J_min, dv);
      t_test_size[0] = 2;
      t_test_size[1] = 7;
      for (i1 = 0; i1 < 14; i1++) {
        t_test_data[i1].re = dv[i1];
        t_test_data[i1].im = 0.0;
      }
      break;

     case 1310:
      abcdeg_NO_AP(-P_init, -V_init, -A_init, rtNaN, rtNaN, -V_min, -A_max,
                   -J_min, -J_max, dv);
      t_test_size[0] = 2;
      t_test_size[1] = 7;
      for (i1 = 0; i1 < 14; i1++) {
        t_test_data[i1].re = dv[i1];
        t_test_data[i1].im = 0.0;
      }
      break;

     case 1311:
      acdeg_NO_AP(P_init, V_init, A_init, rtNaN, rtNaN, rtInf, J_max, J_min, dcv);
      t_test_size[0] = 2;
      t_test_size[1] = 7;
      std::copy(&dcv[0], &dcv[14], &t_test_data[0]);
      break;

     case 1312:
      acdeg_NO_AP(-P_init, -V_init, -A_init, rtNaN, rtNaN, -V_min, -J_min,
                  -J_max, dcv);
      t_test_size[0] = 2;
      t_test_size[1] = 7;
      std::copy(&dcv[0], &dcv[14], &t_test_data[0]);

      // -----------------------
      break;

     case 1701:
      abcd_NO_P(P_init, V_init, A_init, rtNaN, rtInf, A_min, J_max, J_min,
                b_varargin_1);
      t_test_size[0] = 1;
      t_test_size[1] = 7;
      for (i1 = 0; i1 < 7; i1++) {
        t_test_data[i1].re = b_varargin_1[i1];
        t_test_data[i1].im = 0.0;
      }
      break;

     case 1702:
      abcd_NO_P(-P_init, -V_init, -A_init, rtNaN, -V_min, -A_max, -J_min, -J_max,
                b_varargin_1);
      t_test_size[0] = 1;
      t_test_size[1] = 7;
      for (i1 = 0; i1 < 7; i1++) {
        t_test_data[i1].re = b_varargin_1[i1];
        t_test_data[i1].im = 0.0;
      }
      break;

     case 1703:
      acd_NO_P(P_init, V_init, A_init, rtNaN, rtInf, J_max, J_min, dcv);
      t_test_size[0] = 2;
      t_test_size[1] = 7;
      std::copy(&dcv[0], &dcv[14], &t_test_data[0]);
      break;

     case 1704:
      acd_NO_P(-P_init, -V_init, -A_init, rtNaN, -V_min, -J_min, -J_max, dcv);
      t_test_size[0] = 2;
      t_test_size[1] = 7;
      std::copy(&dcv[0], &dcv[14], &t_test_data[0]);
      break;

     default:
      t_test_size[0] = 1;
      t_test_size[1] = 7;
      std::memset(&t_test_data[0], 0, 7U * sizeof(creal_T));
      printf("Error: Solution prior ");
      fflush(stdout);
      printint(i1);
      printf(" is not valid!\n");
      fflush(stdout);
      break;
    }

    check(t_test_data, t_test_size, J_test, V_init, A_init, V_wayp, V_min, A_max,
          A_min, J_max, J_min, valid_data, valid_size);
    i1 = valid_size[1];
    for (int index_t = 0; index_t < i1; index_t++) {
      if (index_t + 1 > valid_size[1]) {
        rtDynamicBoundsError(index_t + 1, 1, valid_size[1], &eb_emlrtBCI);
      }

      if (valid_data[index_t]) {
        bool b_existing;
        bool exitg1;
        if (index_t + 1 > t_test_size[0]) {
          rtDynamicBoundsError(index_t + 1, 1, t_test_size[0], &cb_emlrtBCI);
        }

        for (varargin_1 = 0; varargin_1 < 7; varargin_1++) {
          double d;
          d = t_test_data[index_t + t_test_size[0] * varargin_1].re;
          if (d > 0.0) {
            t_test_valid[varargin_1] = d;
          } else {
            t_test_valid[varargin_1] = 0.0;
          }
        }

        b_existing = false;
        idx = 0;
        exitg1 = false;
        while ((!exitg1) && (idx <= num_valid - 1)) {
          simplify_setp(t_test_valid, J_test, t_test_simpl_data,
                        t_test_simpl_size, J_test_simpl_data, J_test_simpl_size);
          for (i2 = 0; i2 < 7; i2++) {
            varargin_1 = idx + 100 * i2;
            b_varargin_1[i2] = t_out[varargin_1];
            b_J_out[i2] = J_out[varargin_1];
          }

          simplify_setp(b_varargin_1, b_J_out, t_out_simpl_data,
                        t_out_simpl_size, J_out_simpl_data, J_out_simpl_size);

          // numerical
          // numerical
          if ((t_test_simpl_size[1] == t_out_simpl_size[1]) &&
              (J_test_simpl_size[1] == J_out_simpl_size[1])) {
            rtSizeEqNDCheck(&t_test_simpl_size[0], &t_out_simpl_size[0],
                            &emlrtECI);
            b_t_test_simpl_size[0] = 1;
            b_t_test_simpl_size[1] = t_test_simpl_size[1];
            loop_ub = t_test_simpl_size[1];
            for (i2 = 0; i2 < loop_ub; i2++) {
              b_varargin_1[i2] = t_test_simpl_data[i2] - t_out_simpl_data[i2];
            }

            coder::b_abs(b_varargin_1, b_t_test_simpl_size, t_test_simpl_data,
                         t_test_simpl_size);
            c_t_test_simpl_size[0] = 1;
            c_t_test_simpl_size[1] = t_test_simpl_size[1];
            loop_ub = t_test_simpl_size[1];
            for (i2 = 0; i2 < loop_ub; i2++) {
              b_t_test_simpl_data[i2] = (t_test_simpl_data[i2] < 1.0E-7);
            }

            if (coder::all(b_t_test_simpl_data, c_t_test_simpl_size)) {
              rtSizeEqNDCheck(&J_test_simpl_size[0], &J_out_simpl_size[0],
                              &b_emlrtECI);
              t_out_simpl_size[0] = 1;
              t_out_simpl_size[1] = J_test_simpl_size[1];
              loop_ub = J_test_simpl_size[1];
              for (i2 = 0; i2 < loop_ub; i2++) {
                t_out_simpl_data[i2] = J_test_simpl_data[i2] -
                  J_out_simpl_data[i2];
              }

              coder::b_abs(t_out_simpl_data, t_out_simpl_size, t_test_simpl_data,
                           t_test_simpl_size);
              c_t_test_simpl_size[0] = 1;
              c_t_test_simpl_size[1] = t_test_simpl_size[1];
              loop_ub = t_test_simpl_size[1];
              for (i2 = 0; i2 < loop_ub; i2++) {
                b_t_test_simpl_data[i2] = (t_test_simpl_data[i2] < 1.0E-7);
              }

              if (coder::all(b_t_test_simpl_data, c_t_test_simpl_size)) {
                b_existing = true;
              } else {
                b_existing = false;
              }
            } else {
              b_existing = false;
            }
          } else {
            b_existing = false;
          }

          if (b_existing) {
            exitg1 = true;
          } else {
            idx++;
          }
        }

        if (!b_existing) {
          num_valid++;
          if (num_valid > 100) {
            rtDynamicBoundsError(101, 1, 100, &db_emlrtBCI);
          }

          for (i2 = 0; i2 < 7; i2++) {
            varargin_1 = (num_valid + 100 * i2) - 1;
            t_out[varargin_1] = t_test_valid[i2];
            J_out[varargin_1] = J_test[i2];
          }

          solution_out[num_valid - 1] = cases_data[b_index];
        }
      }
    }
  }

  if (1 > num_valid) {
    loop_ub = 0;
  } else {
    loop_ub = num_valid;
  }

  t_out_size[0] = loop_ub;
  t_out_size[1] = 7;
  if (1 > num_valid) {
    idx = 0;
  } else {
    idx = num_valid;
  }

  J_out_size[0] = idx;
  J_out_size[1] = 7;
  for (i = 0; i < 7; i++) {
    for (i1 = 0; i1 < loop_ub; i1++) {
      t_out_data[i1 + loop_ub * i] = t_out[i1 + 100 * i];
    }

    for (i1 = 0; i1 < idx; i1++) {
      J_out_data[i1 + idx * i] = J_out[i1 + 100 * i];
    }
  }

  if (1 > num_valid) {
    b_index = 0;
  } else {
    b_index = num_valid;
  }

  *solution_out_size = b_index;
  if (0 <= b_index - 1) {
    std::copy(&solution_out[0], &solution_out[b_index], &solution_out_data[0]);
  }

  if (num_valid == 0) {
    printf("Error: Could not find valid optimal solution!\n");
    fflush(stdout);
  } else if (num_valid > 1) {
    printf("Debug: Multiple (");
    fflush(stdout);
    printint(static_cast<double>(num_valid));
    printf(") optimal solutions!\n");
    fflush(stdout);
    t_out_size[0] = loop_ub;
    t_out_size[1] = 7;
    for (i = 0; i < 7; i++) {
      for (i1 = 0; i1 < loop_ub; i1++) {
        t_out_data[i1 + loop_ub * i] = t_out[i1 + 100 * i];
      }
    }

    coder::sum(t_out_data, t_out_size, a__1_data, &varargin_1);
    coder::internal::sort(a__1_data, &varargin_1, iidx_data, solution_out_size);
    for (i = 0; i < *solution_out_size; i++) {
      a__1_data[i] = iidx_data[i];
    }

    t_out_size[0] = *solution_out_size;
    t_out_size[1] = 7;
    for (i = 0; i < 7; i++) {
      for (i1 = 0; i1 < *solution_out_size; i1++) {
        i2 = static_cast<int>(a__1_data[i1]);
        if ((i2 < 1) || (i2 > loop_ub)) {
          rtDynamicBoundsError(i2, 1, loop_ub, &fb_emlrtBCI);
        }

        t_out_data[i1 + *solution_out_size * i] = t_out[(i2 + 100 * i) - 1];
      }
    }

    J_out_size[0] = *solution_out_size;
    J_out_size[1] = 7;
    for (i = 0; i < 7; i++) {
      for (i1 = 0; i1 < *solution_out_size; i1++) {
        i2 = static_cast<int>(a__1_data[i1]);
        if ((i2 < 1) || (i2 > idx)) {
          rtDynamicBoundsError(i2, 1, idx, &gb_emlrtBCI);
        }

        J_out_data[i1 + *solution_out_size * i] = J_out[(i2 + 100 * i) - 1];
      }
    }

    for (i = 0; i < *solution_out_size; i++) {
      i1 = static_cast<int>(a__1_data[i]);
      if ((i1 < 1) || (i1 > b_index)) {
        rtDynamicBoundsError(i1, 1, b_index, &hb_emlrtBCI);
      }

      solution_out_data[i] = solution_out[i1 - 1];
    }
  }
}

void solve_O(double P_init, double V_init, double A_init, double P_wayp, double
             V_wayp, double A_wayp, double V_max, double V_min, double A_max,
             double A_min, double J_max, double J_min, double t_out_data[], int
             t_out_size[2], double J_out_data[], int J_out_size[2], int
             solution_out_data[], int *solution_out_size)
{
  creal_T t_test_data[35];
  creal_T dcv1[28];
  creal_T dcv2[21];
  creal_T dcv[14];
  creal_T b_A_init[7];
  double J_out[700];
  double t_out[700];
  double a__1_data[100];
  double dv[14];
  double J_out_simpl_data[7];
  double J_test[7];
  double J_test_simpl_data[7];
  double b_J_out[7];
  double b_varargin_1[7];
  double t_out_simpl_data[7];
  double t_test_simpl_data[7];
  double t_test_valid[7];
  int iidx_data[100];
  int solution_out[100];
  int cases_data[45];
  int J_out_simpl_size[2];
  int J_test_simpl_size[2];
  int b_t_test_simpl_size[2];
  int c_t_test_simpl_size[2];
  int cases_size[2];
  int t_out_simpl_size[2];
  int t_test_simpl_size[2];
  int t_test_size[2];
  int valid_size[2];
  int b_index;
  int i;
  int i1;
  int i2;
  int loop_ub;
  int ndx;
  int num_valid;
  int varargin_1;
  bool b_t_test_simpl_data[7];
  bool valid_data[5];

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
  num_valid = 0;
  std::memset(&t_out[0], 0, 700U * sizeof(double));
  std::memset(&J_out[0], 0, 700U * sizeof(double));
  std::memset(&solution_out[0], 0, 100U * sizeof(int));

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
  ndx = coder::eml_sub2ind(static_cast<double>(!rtIsNaN(P_wayp)) + 1.0,
    static_cast<double>(!rtIsNaN(V_wayp)) + 1.0, static_cast<double>(!rtIsNaN
    (A_wayp)) + 1.0);

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
  switch (check_feasibility(V_init, A_init, V_max, V_min, A_max, A_min, J_max,
           J_min)) {
   case 3U:
    varargin_1 = 2;
    break;

   case 5U:
    varargin_1 = 2;
    break;

   case 4U:
    varargin_1 = 3;
    break;

   case 6U:
    varargin_1 = 3;
    break;

   default:
    varargin_1 = 1;
    break;
  }

  varargin_1 = coder::eml_sub2ind(static_cast<double>(varargin_1), static_cast<
    double>(!rtIsInf(V_max)) + 1.0, static_cast<double>(!rtIsInf(V_min)) + 1.0,
    static_cast<double>(!rtIsInf(A_max)) + 1.0, static_cast<double>(!rtIsInf
    (A_min)) + 1.0);
  select_cases_O(nlut[ndx - 1], condlut[varargin_1 - 1], cases_data, cases_size);
  i = cases_size[1];
  for (b_index = 0; b_index < i; b_index++) {
    double d;
    if (b_index + 1 > cases_size[1]) {
      rtDynamicBoundsError(b_index + 1, 1, cases_size[1], &bb_emlrtBCI);
    }

    i1 = cases_data[b_index];
    if (coder::b_mod(std::floor(static_cast<double>(i1) / 1000.0)) == 0.0) {
      if (i1 - ((i1 >> 1) << 1) == 1) {
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
    } else if (i1 - ((i1 >> 1) << 1) == 1) {
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

    switch (i1) {
     case 0:
      t_test_size[0] = 1;
      t_test_size[1] = 7;
      std::memset(&t_test_data[0], 0, 7U * sizeof(creal_T));
      break;

     case 101:
      abcdefg_O_AVP(P_init, V_init, A_init, P_wayp, V_wayp, A_wayp, V_max, A_max,
                    A_min, J_max, J_min, b_varargin_1);
      t_test_size[0] = 1;
      t_test_size[1] = 7;
      for (i1 = 0; i1 < 7; i1++) {
        t_test_data[i1].re = b_varargin_1[i1];
        t_test_data[i1].im = 0.0;
      }
      break;

     case 102:
      abcdefg_O_AVP(-P_init, -V_init, -A_init, -P_wayp, -V_wayp, -A_wayp, -V_min,
                    -A_min, -A_max, -J_min, -J_max, b_varargin_1);
      t_test_size[0] = 1;
      t_test_size[1] = 7;
      for (i1 = 0; i1 < 7; i1++) {
        t_test_data[i1].re = b_varargin_1[i1];
        t_test_data[i1].im = 0.0;
      }
      break;

     case 103:
      abcdeg_O_AVP(P_init, V_init, A_init, P_wayp, V_wayp, A_wayp, V_max, A_max,
                   J_max, J_min, dcv);
      t_test_size[0] = 2;
      t_test_size[1] = 7;
      std::copy(&dcv[0], &dcv[14], &t_test_data[0]);
      break;

     case 104:
      abcdeg_O_AVP(-P_init, -V_init, -A_init, -P_wayp, -V_wayp, -A_wayp, -V_min,
                   -A_min, -J_min, -J_max, dcv);
      t_test_size[0] = 2;
      t_test_size[1] = 7;
      std::copy(&dcv[0], &dcv[14], &t_test_data[0]);
      break;

     case 105:
      abcefg_O_AVP(P_init, V_init, A_init, P_wayp, V_wayp, A_wayp, A_max, A_min,
                   J_max, J_min, dcv);
      t_test_size[0] = 2;
      t_test_size[1] = 7;
      std::copy(&dcv[0], &dcv[14], &t_test_data[0]);
      break;

     case 106:
      abcefg_O_AVP(-P_init, -V_init, -A_init, -P_wayp, -V_wayp, -A_wayp, -A_min,
                   -A_max, -J_min, -J_max, dcv);
      t_test_size[0] = 2;
      t_test_size[1] = 7;
      std::copy(&dcv[0], &dcv[14], &t_test_data[0]);
      break;

     case 107:
      abceg_O_AVP(P_init, V_init, A_init, P_wayp, V_wayp, A_wayp, A_max, J_max,
                  J_min, dcv1);
      t_test_size[0] = 4;
      t_test_size[1] = 7;
      std::copy(&dcv1[0], &dcv1[28], &t_test_data[0]);
      break;

     case 108:
      abceg_O_AVP(-P_init, -V_init, -A_init, -P_wayp, -V_wayp, -A_wayp, -A_min,
                  -J_min, -J_max, dcv1);
      t_test_size[0] = 4;
      t_test_size[1] = 7;
      std::copy(&dcv1[0], &dcv1[28], &t_test_data[0]);
      break;

     case 109:
      acdefg_O_AVP(P_init, V_init, A_init, P_wayp, V_wayp, A_wayp, V_max, A_min,
                   J_max, J_min, dcv);
      t_test_size[0] = 2;
      t_test_size[1] = 7;
      std::copy(&dcv[0], &dcv[14], &t_test_data[0]);
      break;

     case 110:
      acdefg_O_AVP(-P_init, -V_init, -A_init, -P_wayp, -V_wayp, -A_wayp, -V_min,
                   -A_max, -J_min, -J_max, dcv);
      t_test_size[0] = 2;
      t_test_size[1] = 7;
      std::copy(&dcv[0], &dcv[14], &t_test_data[0]);
      break;

     case 111:
      acdeg_O_AVP(P_init, V_init, A_init, P_wayp, V_wayp, A_wayp, V_max, J_max,
                  J_min, dcv1);
      t_test_size[0] = 4;
      t_test_size[1] = 7;
      std::copy(&dcv1[0], &dcv1[28], &t_test_data[0]);
      break;

     case 112:
      acdeg_O_AVP(-P_init, -V_init, -A_init, -P_wayp, -V_wayp, -A_wayp, -V_min,
                  -J_min, -J_max, dcv1);
      t_test_size[0] = 4;
      t_test_size[1] = 7;
      std::copy(&dcv1[0], &dcv1[28], &t_test_data[0]);
      break;

     case 113:
      acefg_O_AVP(P_init, V_init, A_init, P_wayp, V_wayp, A_wayp, A_min, J_max,
                  J_min, dcv1);
      t_test_size[0] = 4;
      t_test_size[1] = 7;
      std::copy(&dcv1[0], &dcv1[28], &t_test_data[0]);
      break;

     case 114:
      acefg_O_AVP(-P_init, -V_init, -A_init, -P_wayp, -V_wayp, -A_wayp, -A_max,
                  -J_min, -J_max, dcv1);
      t_test_size[0] = 4;
      t_test_size[1] = 7;
      std::copy(&dcv1[0], &dcv1[28], &t_test_data[0]);
      break;

     case 115:
      aceg_O_AVP(P_init, V_init, A_init, P_wayp, V_wayp, A_wayp, J_max, J_min,
                 t_test_data);
      t_test_size[0] = 5;
      t_test_size[1] = 7;
      break;

     case 116:
      aceg_O_AVP(-P_init, -V_init, -A_init, -P_wayp, -V_wayp, -A_wayp, -J_min,
                 -J_max, t_test_data);
      t_test_size[0] = 5;
      t_test_size[1] = 7;

      // -----------------------
      break;

     case 201:
      abc_O_VP(P_init, V_init, A_init, P_wayp, V_wayp, A_max, J_max, J_min, dcv1);
      t_test_size[0] = 4;
      t_test_size[1] = 7;
      std::copy(&dcv1[0], &dcv1[28], &t_test_data[0]);
      break;

     case 202:
      abc_O_VP(-P_init, -V_init, -A_init, -P_wayp, -V_wayp, -A_min, -J_min,
               -J_max, dcv1);
      t_test_size[0] = 4;
      t_test_size[1] = 7;
      std::copy(&dcv1[0], &dcv1[28], &t_test_data[0]);
      break;

     case 203:
      abcde_O_VP(P_init, V_init, A_init, P_wayp, V_wayp, V_max, A_max, J_max,
                 J_min, dcv);
      t_test_size[0] = 2;
      t_test_size[1] = 7;
      std::copy(&dcv[0], &dcv[14], &t_test_data[0]);
      break;

     case 204:
      abcde_O_VP(-P_init, -V_init, -A_init, -P_wayp, -V_wayp, -V_min, -A_min,
                 -J_min, -J_max, dcv);
      t_test_size[0] = 2;
      t_test_size[1] = 7;
      std::copy(&dcv[0], &dcv[14], &t_test_data[0]);
      break;

     case 205:
      abcdef_O_VP(P_init, V_init, A_init, P_wayp, V_wayp, V_max, A_max, A_min,
                  J_max, J_min, b_varargin_1);
      t_test_size[0] = 1;
      t_test_size[1] = 7;
      for (i1 = 0; i1 < 7; i1++) {
        t_test_data[i1].re = b_varargin_1[i1];
        t_test_data[i1].im = 0.0;
      }
      break;

     case 206:
      abcdef_O_VP(-P_init, -V_init, -A_init, -P_wayp, -V_wayp, -V_min, -A_min,
                  -A_max, -J_min, -J_max, b_varargin_1);
      t_test_size[0] = 1;
      t_test_size[1] = 7;
      for (i1 = 0; i1 < 7; i1++) {
        t_test_data[i1].re = b_varargin_1[i1];
        t_test_data[i1].im = 0.0;
      }
      break;

     case 207:
      abcdefg_O_VP(P_init, V_init, A_init, P_wayp, V_wayp, V_max, V_min, A_max,
                   A_min, J_max, J_min, dv);
      t_test_size[0] = 2;
      t_test_size[1] = 7;
      for (i1 = 0; i1 < 14; i1++) {
        t_test_data[i1].re = dv[i1];
        t_test_data[i1].im = 0.0;
      }
      break;

     case 208:
      abcdefg_O_VP(-P_init, -V_init, -A_init, -P_wayp, -V_wayp, -V_min, -V_max,
                   -A_min, -A_max, -J_min, -J_max, dv);
      t_test_size[0] = 2;
      t_test_size[1] = 7;
      for (i1 = 0; i1 < 14; i1++) {
        t_test_data[i1].re = dv[i1];
        t_test_data[i1].im = 0.0;
      }
      break;

     case 209:
      abcdeg_O_VP(P_init, V_init, A_init, P_wayp, V_wayp, V_max, V_min, A_max,
                  J_max, J_min, dcv);
      t_test_size[0] = 2;
      t_test_size[1] = 7;
      std::copy(&dcv[0], &dcv[14], &t_test_data[0]);
      break;

     case 210:
      abcdeg_O_VP(-P_init, -V_init, -A_init, -P_wayp, -V_wayp, -V_min, -V_max,
                  -A_min, -J_min, -J_max, dcv);
      t_test_size[0] = 2;
      t_test_size[1] = 7;
      std::copy(&dcv[0], &dcv[14], &t_test_data[0]);
      break;

     case 211:
      abcef_O_VP(P_init, V_init, A_init, P_wayp, V_wayp, A_max, A_min, J_max,
                 J_min, dcv);
      t_test_size[0] = 2;
      t_test_size[1] = 7;
      std::copy(&dcv[0], &dcv[14], &t_test_data[0]);
      break;

     case 212:
      abcef_O_VP(-P_init, -V_init, -A_init, -P_wayp, -V_wayp, -A_min, -A_max,
                 -J_min, -J_max, dcv);
      t_test_size[0] = 2;
      t_test_size[1] = 7;
      std::copy(&dcv[0], &dcv[14], &t_test_data[0]);
      break;

     case 213:
      t_test_size[0] = 1;
      t_test_size[1] = 7;
      std::memset(&t_test_data[0], 0, 7U * sizeof(creal_T));
      break;

     case 214:
      t_test_size[0] = 1;
      t_test_size[1] = 7;
      std::memset(&t_test_data[0], 0, 7U * sizeof(creal_T));
      break;

     case 215:
      t_test_size[0] = 1;
      t_test_size[1] = 7;
      std::memset(&t_test_data[0], 0, 7U * sizeof(creal_T));
      break;

     case 216:
      t_test_size[0] = 1;
      t_test_size[1] = 7;
      std::memset(&t_test_data[0], 0, 7U * sizeof(creal_T));
      break;

     case 217:
      t_test_size[0] = 1;
      t_test_size[1] = 7;
      std::memset(&t_test_data[0], 0, 7U * sizeof(creal_T));
      break;

     case 218:
      t_test_size[0] = 1;
      t_test_size[1] = 7;
      std::memset(&t_test_data[0], 0, 7U * sizeof(creal_T));
      break;

     case 219:
      acde_O_VP(P_init, V_init, A_init, P_wayp, V_wayp, V_max, J_max, J_min, dcv);
      t_test_size[0] = 2;
      t_test_size[1] = 7;
      std::copy(&dcv[0], &dcv[14], &t_test_data[0]);
      break;

     case 220:
      acde_O_VP(-P_init, -V_init, -A_init, -P_wayp, -V_wayp, -V_min, -J_min,
                -J_max, dcv);
      t_test_size[0] = 2;
      t_test_size[1] = 7;
      std::copy(&dcv[0], &dcv[14], &t_test_data[0]);
      break;

     case 221:
      acdef_O_VP(P_init, V_init, A_init, P_wayp, V_wayp, V_max, A_min, J_max,
                 J_min, dcv);
      t_test_size[0] = 2;
      t_test_size[1] = 7;
      std::copy(&dcv[0], &dcv[14], &t_test_data[0]);
      break;

     case 222:
      acdef_O_VP(-P_init, -V_init, -A_init, -P_wayp, -V_wayp, -V_min, -A_max,
                 -J_min, -J_max, dcv);
      t_test_size[0] = 2;
      t_test_size[1] = 7;
      std::copy(&dcv[0], &dcv[14], &t_test_data[0]);
      break;

     case 223:
      acdefg_O_VP(P_init, V_init, A_init, P_wayp, V_wayp, V_max, V_min, A_min,
                  J_max, J_min, dcv1);
      t_test_size[0] = 4;
      t_test_size[1] = 7;
      std::copy(&dcv1[0], &dcv1[28], &t_test_data[0]);
      break;

     case 224:
      acdefg_O_VP(-P_init, -V_init, -A_init, -P_wayp, -V_wayp, -V_min, -V_max,
                  -A_max, -J_min, -J_max, dcv1);
      t_test_size[0] = 4;
      t_test_size[1] = 7;
      std::copy(&dcv1[0], &dcv1[28], &t_test_data[0]);
      break;

     case 225:
      t_test_size[0] = 1;
      t_test_size[1] = 7;
      std::memset(&t_test_data[0], 0, 7U * sizeof(creal_T));
      break;

     case 226:
      t_test_size[0] = 1;
      t_test_size[1] = 7;
      std::memset(&t_test_data[0], 0, 7U * sizeof(creal_T));
      break;

     case 227:
      acef_O_VP(P_init, V_init, A_init, P_wayp, V_wayp, A_min, J_max, J_min,
                dcv1);
      t_test_size[0] = 4;
      t_test_size[1] = 7;
      std::copy(&dcv1[0], &dcv1[28], &t_test_data[0]);
      break;

     case 228:
      acef_O_VP(-P_init, -V_init, -A_init, -P_wayp, -V_wayp, -A_max, -J_min,
                -J_max, dcv1);
      t_test_size[0] = 4;
      t_test_size[1] = 7;
      std::copy(&dcv1[0], &dcv1[28], &t_test_data[0]);
      break;

     case 229:
      t_test_size[0] = 1;
      t_test_size[1] = 7;
      std::memset(&t_test_data[0], 0, 7U * sizeof(creal_T));
      break;

     case 230:
      t_test_size[0] = 1;
      t_test_size[1] = 7;
      std::memset(&t_test_data[0], 0, 7U * sizeof(creal_T));
      break;

     case 231:
      t_test_size[0] = 1;
      t_test_size[1] = 7;
      std::memset(&t_test_data[0], 0, 7U * sizeof(creal_T));
      break;

     case 232:
      t_test_size[0] = 1;
      t_test_size[1] = 7;
      std::memset(&t_test_data[0], 0, 7U * sizeof(creal_T));

      // -----------------------
      break;

     case 301:
      abc_O_AP(P_init, V_init, A_init, P_wayp, A_wayp, A_max, J_max, J_min, dcv);
      t_test_size[0] = 2;
      t_test_size[1] = 7;
      std::copy(&dcv[0], &dcv[14], &t_test_data[0]);
      break;

     case 302:
      abc_O_AP(-P_init, -V_init, -A_init, -P_wayp, -A_wayp, -A_min, -J_min,
               -J_max, dcv);
      t_test_size[0] = 2;
      t_test_size[1] = 7;
      std::copy(&dcv[0], &dcv[14], &t_test_data[0]);
      break;

     case 303:
      abcde_O_AP(P_init, V_init, A_init, P_wayp, A_wayp, V_max, A_max, J_max,
                 J_min, b_varargin_1);
      t_test_size[0] = 1;
      t_test_size[1] = 7;
      for (i1 = 0; i1 < 7; i1++) {
        t_test_data[i1].re = b_varargin_1[i1];
        t_test_data[i1].im = 0.0;
      }
      break;

     case 304:
      abcde_O_AP(-P_init, -V_init, -A_init, -P_wayp, -A_wayp, -V_min, -A_min,
                 -J_min, -J_max, b_varargin_1);
      t_test_size[0] = 1;
      t_test_size[1] = 7;
      for (i1 = 0; i1 < 7; i1++) {
        t_test_data[i1].re = b_varargin_1[i1];
        t_test_data[i1].im = 0.0;
      }
      break;

     case 305:
      abcdefg_O_AP(P_init, V_init, A_init, P_wayp, A_wayp, V_max, A_max, A_min,
                   J_max, J_min, b_varargin_1);
      t_test_size[0] = 1;
      t_test_size[1] = 7;
      for (i1 = 0; i1 < 7; i1++) {
        t_test_data[i1].re = b_varargin_1[i1];
        t_test_data[i1].im = 0.0;
      }
      break;

     case 306:
      abcdefg_O_AP(-P_init, -V_init, -A_init, -P_wayp, -A_wayp, -V_min, -A_min,
                   -A_max, -J_min, -J_max, b_varargin_1);
      t_test_size[0] = 1;
      t_test_size[1] = 7;
      for (i1 = 0; i1 < 7; i1++) {
        t_test_data[i1].re = b_varargin_1[i1];
        t_test_data[i1].im = 0.0;
      }
      break;

     case 307:
      abcefg_O_AP(P_init, V_init, A_init, P_wayp, A_wayp, V_max, A_max, A_min,
                  J_max, J_min, dcv);
      t_test_size[0] = 2;
      t_test_size[1] = 7;
      std::copy(&dcv[0], &dcv[14], &t_test_data[0]);
      break;

     case 308:
      abcefg_O_AP(-P_init, -V_init, -A_init, -P_wayp, -A_wayp, -V_min, -A_min,
                  -A_max, -J_min, -J_max, dcv);
      t_test_size[0] = 2;
      t_test_size[1] = 7;
      std::copy(&dcv[0], &dcv[14], &t_test_data[0]);
      break;

     case 309:
      abceg_O_AP(P_init, V_init, A_init, P_wayp, A_wayp, V_max, A_max, J_max,
                 J_min, dcv1);
      t_test_size[0] = 4;
      t_test_size[1] = 7;
      std::copy(&dcv1[0], &dcv1[28], &t_test_data[0]);
      break;

     case 310:
      abceg_O_AP(-P_init, -V_init, -A_init, -P_wayp, -A_wayp, -V_min, -A_min,
                 -J_min, -J_max, dcv1);
      t_test_size[0] = 4;
      t_test_size[1] = 7;
      std::copy(&dcv1[0], &dcv1[28], &t_test_data[0]);
      break;

     case 311:
      ac_O_AP(P_init, V_init, A_init, P_wayp, A_wayp, J_max, J_min, dcv2);
      t_test_size[0] = 3;
      t_test_size[1] = 7;
      std::copy(&dcv2[0], &dcv2[21], &t_test_data[0]);
      break;

     case 312:
      ac_O_AP(-P_init, -V_init, -A_init, -P_wayp, -A_wayp, -J_min, -J_max, dcv2);
      t_test_size[0] = 3;
      t_test_size[1] = 7;
      std::copy(&dcv2[0], &dcv2[21], &t_test_data[0]);
      break;

     case 313:
      acde_O_AP(P_init, V_init, A_init, P_wayp, A_wayp, V_max, J_max, J_min, dcv);
      t_test_size[0] = 2;
      t_test_size[1] = 7;
      std::copy(&dcv[0], &dcv[14], &t_test_data[0]);
      break;

     case 314:
      acde_O_AP(-P_init, -V_init, -A_init, -P_wayp, -A_wayp, -V_min, -J_min,
                -J_max, dcv);
      t_test_size[0] = 2;
      t_test_size[1] = 7;
      std::copy(&dcv[0], &dcv[14], &t_test_data[0]);
      break;

     case 315:
      acdefg_O_AP(P_init, V_init, A_init, P_wayp, A_wayp, V_max, A_min, J_max,
                  J_min, dcv);
      t_test_size[0] = 2;
      t_test_size[1] = 7;
      std::copy(&dcv[0], &dcv[14], &t_test_data[0]);
      break;

     case 316:
      acdefg_O_AP(-P_init, -V_init, -A_init, -P_wayp, -A_wayp, -V_min, -A_max,
                  -J_min, -J_max, dcv);
      t_test_size[0] = 2;
      t_test_size[1] = 7;
      std::copy(&dcv[0], &dcv[14], &t_test_data[0]);
      break;

     case 317:
      acefg_O_AP(P_init, V_init, A_init, P_wayp, A_wayp, V_max, A_min, J_max,
                 J_min, dcv1);
      t_test_size[0] = 4;
      t_test_size[1] = 7;
      std::copy(&dcv1[0], &dcv1[28], &t_test_data[0]);
      break;

     case 318:
      acefg_O_AP(-P_init, -V_init, -A_init, -P_wayp, -A_wayp, -V_min, -A_max,
                 -J_min, -J_max, dcv1);
      t_test_size[0] = 4;
      t_test_size[1] = 7;
      std::copy(&dcv1[0], &dcv1[28], &t_test_data[0]);
      break;

     case 319:
      aceg_O_AP(P_init, V_init, A_init, P_wayp, A_wayp, V_max, J_max, J_min,
                dcv1);
      t_test_size[0] = 4;
      t_test_size[1] = 7;
      std::copy(&dcv1[0], &dcv1[28], &t_test_data[0]);
      break;

     case 320:
      aceg_O_AP(-P_init, -V_init, -A_init, -P_wayp, -A_wayp, -V_min, -J_min,
                -J_max, dcv1);
      t_test_size[0] = 4;
      t_test_size[1] = 7;
      std::copy(&dcv1[0], &dcv1[28], &t_test_data[0]);
      break;

     case 321:
      abcdeg_O_AP(P_init, V_init, A_init, P_wayp, A_wayp, V_max, A_max, J_max,
                  J_min, dv);
      t_test_size[0] = 2;
      t_test_size[1] = 7;
      for (i1 = 0; i1 < 14; i1++) {
        t_test_data[i1].re = dv[i1];
        t_test_data[i1].im = 0.0;
      }
      break;

     case 322:
      abcdeg_O_AP(-P_init, -V_init, -A_init, -P_wayp, -A_wayp, -V_min, -A_min,
                  -J_min, -J_max, dv);
      t_test_size[0] = 2;
      t_test_size[1] = 7;
      for (i1 = 0; i1 < 14; i1++) {
        t_test_data[i1].re = dv[i1];
        t_test_data[i1].im = 0.0;
      }
      break;

     case 323:
      acdeg_O_AP(P_init, V_init, A_init, P_wayp, A_wayp, V_max, J_max, J_min,
                 dcv);
      t_test_size[0] = 2;
      t_test_size[1] = 7;
      std::copy(&dcv[0], &dcv[14], &t_test_data[0]);
      break;

     case 324:
      acdeg_O_AP(-P_init, -V_init, -A_init, -P_wayp, -A_wayp, -V_min, -J_min,
                 -J_max, dcv);
      t_test_size[0] = 2;
      t_test_size[1] = 7;
      std::copy(&dcv[0], &dcv[14], &t_test_data[0]);

      // -----------------------
      break;

     case 401:
      {
        double l2;

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
        //  Generated on 28-Aug-2019 17:25:45
        l2 = A_max * A_max;
        b_A_init[0].re = -(A_init - A_max) / J_max;
        b_A_init[0].im = 0.0;
        d = J_min * J_max;
        b_A_init[1].re = (((((J_min * l2 - J_max * l2) - A_init * A_init * J_min)
                            + A_wayp * A_wayp * J_max) + d * V_init * 2.0) - d *
                          V_wayp * 2.0) * -0.5 / (A_max * J_min * J_max);
        b_A_init[1].im = 0.0;
        b_A_init[2].re = -(A_max - A_wayp) / J_min;
        b_A_init[2].im = 0.0;
        b_A_init[3].re = 0.0;
        b_A_init[3].im = 0.0;
        b_A_init[4].re = 0.0;
        b_A_init[4].im = 0.0;
        b_A_init[5].re = 0.0;
        b_A_init[5].im = 0.0;
        b_A_init[6].re = 0.0;
        b_A_init[6].im = 0.0;
        t_test_size[0] = 1;
        t_test_size[1] = 7;
        std::copy(&b_A_init[0], &b_A_init[7], &t_test_data[0]);
      }
      break;

     case 402:
      {
        double l2;

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
        //  Generated on 28-Aug-2019 17:25:45
        l2 = -A_min * -A_min;
        b_A_init[0].re = -(-A_init - (-A_min)) / -J_min;
        b_A_init[0].im = 0.0;
        d = -J_max * -J_min;
        b_A_init[1].re = (((((-J_max * l2 - -J_min * l2) - -A_init * -A_init *
                             -J_max) + -A_wayp * -A_wayp * -J_min) + d * -V_init
                           * 2.0) - d * -V_wayp * 2.0) * -0.5 / (-A_min * -J_max
          * -J_min);
        b_A_init[1].im = 0.0;
        b_A_init[2].re = -(-A_min - (-A_wayp)) / -J_max;
        b_A_init[2].im = 0.0;
        b_A_init[3].re = 0.0;
        b_A_init[3].im = 0.0;
        b_A_init[4].re = 0.0;
        b_A_init[4].im = 0.0;
        b_A_init[5].re = 0.0;
        b_A_init[5].im = 0.0;
        b_A_init[6].re = 0.0;
        b_A_init[6].im = 0.0;
        t_test_size[0] = 1;
        t_test_size[1] = 7;
        std::copy(&b_A_init[0], &b_A_init[7], &t_test_data[0]);
      }
      break;

     case 403:
      ac_O_AV(V_init, A_init, V_wayp, A_wayp, J_max, J_min, dcv);
      t_test_size[0] = 2;
      t_test_size[1] = 7;
      std::copy(&dcv[0], &dcv[14], &t_test_data[0]);
      break;

     case 404:
      ac_O_AV(-V_init, -A_init, -V_wayp, -A_wayp, -J_min, -J_max, dcv);
      t_test_size[0] = 2;
      t_test_size[1] = 7;
      std::copy(&dcv[0], &dcv[14], &t_test_data[0]);
      break;

     case 501:
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
      //  Generated on 28-Aug-2019 17:25:45
      b_A_init[0].re = -(A_init - A_wayp) / J_max;
      b_A_init[0].im = 0.0;
      b_A_init[1].re = 0.0;
      b_A_init[1].im = 0.0;
      b_A_init[2].re = 0.0;
      b_A_init[2].im = 0.0;
      b_A_init[3].re = 0.0;
      b_A_init[3].im = 0.0;
      b_A_init[4].re = 0.0;
      b_A_init[4].im = 0.0;
      b_A_init[5].re = 0.0;
      b_A_init[5].im = 0.0;
      b_A_init[6].re = 0.0;
      b_A_init[6].im = 0.0;
      t_test_size[0] = 1;
      t_test_size[1] = 7;
      std::copy(&b_A_init[0], &b_A_init[7], &t_test_data[0]);
      break;

     case 502:
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
      //  Generated on 28-Aug-2019 17:25:45
      b_A_init[0].re = -(-A_init - (-A_wayp)) / -J_min;
      b_A_init[0].im = 0.0;
      b_A_init[1].re = 0.0;
      b_A_init[1].im = 0.0;
      b_A_init[2].re = 0.0;
      b_A_init[2].im = 0.0;
      b_A_init[3].re = 0.0;
      b_A_init[3].im = 0.0;
      b_A_init[4].re = 0.0;
      b_A_init[4].im = 0.0;
      b_A_init[5].re = 0.0;
      b_A_init[5].im = 0.0;
      b_A_init[6].re = 0.0;
      b_A_init[6].im = 0.0;
      t_test_size[0] = 1;
      t_test_size[1] = 7;
      std::copy(&b_A_init[0], &b_A_init[7], &t_test_data[0]);
      break;

     case 503:
      abc_O_A(V_init, A_init, A_wayp, V_min, A_max, J_max, J_min, b_varargin_1);
      t_test_size[0] = 1;
      t_test_size[1] = 7;
      for (i1 = 0; i1 < 7; i1++) {
        t_test_data[i1].re = b_varargin_1[i1];
        t_test_data[i1].im = 0.0;
      }
      break;

     case 504:
      abc_O_A(-V_init, -A_init, -A_wayp, -V_max, -A_min, -J_min, -J_max,
              b_varargin_1);
      t_test_size[0] = 1;
      t_test_size[1] = 7;
      for (i1 = 0; i1 < 7; i1++) {
        t_test_data[i1].re = b_varargin_1[i1];
        t_test_data[i1].im = 0.0;
      }
      break;

     case 505:
      ac_O_A(V_init, A_init, A_wayp, V_min, J_max, J_min, dcv);
      t_test_size[0] = 2;
      t_test_size[1] = 7;
      std::copy(&dcv[0], &dcv[14], &t_test_data[0]);
      break;

     case 506:
      ac_O_A(-V_init, -A_init, -A_wayp, -V_max, -J_min, -J_max, dcv);
      t_test_size[0] = 2;
      t_test_size[1] = 7;
      std::copy(&dcv[0], &dcv[14], &t_test_data[0]);

      // -----------------------
      break;

     case 601:
      a_O_V(V_init, A_init, V_wayp, J_max, dcv);
      t_test_size[0] = 2;
      t_test_size[1] = 7;
      std::copy(&dcv[0], &dcv[14], &t_test_data[0]);
      break;

     case 602:
      a_O_V(-V_init, -A_init, -V_wayp, -J_min, dcv);
      t_test_size[0] = 2;
      t_test_size[1] = 7;
      std::copy(&dcv[0], &dcv[14], &t_test_data[0]);
      break;

     case 603:
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
      //  Generated on 28-Aug-2019 17:25:45
      b_A_init[0].re = -(A_init - A_max) / J_max;
      b_A_init[0].im = 0.0;
      b_A_init[1].re = (((J_max * V_init * 2.0 - J_max * V_wayp * 2.0) - A_init *
                         A_init) + A_max * A_max) * -0.5 / (A_max * J_max);
      b_A_init[1].im = 0.0;
      b_A_init[2].re = 0.0;
      b_A_init[2].im = 0.0;
      b_A_init[3].re = 0.0;
      b_A_init[3].im = 0.0;
      b_A_init[4].re = 0.0;
      b_A_init[4].im = 0.0;
      b_A_init[5].re = 0.0;
      b_A_init[5].im = 0.0;
      b_A_init[6].re = 0.0;
      b_A_init[6].im = 0.0;
      t_test_size[0] = 1;
      t_test_size[1] = 7;
      std::copy(&b_A_init[0], &b_A_init[7], &t_test_data[0]);
      break;

     case 604:
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
      //  Generated on 28-Aug-2019 17:25:45
      b_A_init[0].re = -(-A_init - (-A_min)) / -J_min;
      b_A_init[0].im = 0.0;
      b_A_init[1].re = (((-J_min * -V_init * 2.0 - -J_min * -V_wayp * 2.0) -
                         -A_init * -A_init) + -A_min * -A_min) * -0.5 / (-A_min *
        -J_min);
      b_A_init[1].im = 0.0;
      b_A_init[2].re = 0.0;
      b_A_init[2].im = 0.0;
      b_A_init[3].re = 0.0;
      b_A_init[3].im = 0.0;
      b_A_init[4].re = 0.0;
      b_A_init[4].im = 0.0;
      b_A_init[5].re = 0.0;
      b_A_init[5].im = 0.0;
      b_A_init[6].re = 0.0;
      b_A_init[6].im = 0.0;
      t_test_size[0] = 1;
      t_test_size[1] = 7;
      std::copy(&b_A_init[0], &b_A_init[7], &t_test_data[0]);
      break;

     case 605:
      abc_O_V(V_init, A_init, V_wayp, V_max, A_max, J_max, J_min, dcv);
      t_test_size[0] = 2;
      t_test_size[1] = 7;
      std::copy(&dcv[0], &dcv[14], &t_test_data[0]);
      break;

     case 606:
      abc_O_V(-V_init, -A_init, -V_wayp, -V_min, -A_min, -J_min, -J_max, dcv);
      t_test_size[0] = 2;
      t_test_size[1] = 7;
      std::copy(&dcv[0], &dcv[14], &t_test_data[0]);
      break;

     case 607:
      ac_O_V(V_init, A_init, V_wayp, V_max, J_max, J_min, dcv);
      t_test_size[0] = 2;
      t_test_size[1] = 7;
      std::copy(&dcv[0], &dcv[14], &t_test_data[0]);
      break;

     case 608:
      ac_O_V(-V_init, -A_init, -V_wayp, -V_min, -J_min, -J_max, dcv);
      t_test_size[0] = 2;
      t_test_size[1] = 7;
      std::copy(&dcv[0], &dcv[14], &t_test_data[0]);

      // -----------------------
      break;

     case 701:
      a_O_P(P_init, V_init, A_init, P_wayp, J_max, dcv2);
      t_test_size[0] = 3;
      t_test_size[1] = 7;
      std::copy(&dcv2[0], &dcv2[21], &t_test_data[0]);
      break;

     case 702:
      a_O_P(-P_init, -V_init, -A_init, -P_wayp, -J_min, dcv2);
      t_test_size[0] = 3;
      t_test_size[1] = 7;
      std::copy(&dcv2[0], &dcv2[21], &t_test_data[0]);
      break;

     case 703:
      ab_O_P(P_init, V_init, A_init, P_wayp, A_max, J_max, dcv);
      t_test_size[0] = 2;
      t_test_size[1] = 7;
      std::copy(&dcv[0], &dcv[14], &t_test_data[0]);
      break;

     case 704:
      ab_O_P(-P_init, -V_init, -A_init, -P_wayp, -A_min, -J_min, dcv);
      t_test_size[0] = 2;
      t_test_size[1] = 7;
      std::copy(&dcv[0], &dcv[14], &t_test_data[0]);
      break;

     case 705:
      abc_O_P(P_init, V_init, A_init, P_wayp, V_max, A_max, J_max, J_min, dcv2);
      t_test_size[0] = 3;
      t_test_size[1] = 7;
      std::copy(&dcv2[0], &dcv2[21], &t_test_data[0]);
      break;

     case 706:
      abc_O_P(-P_init, -V_init, -A_init, -P_wayp, -V_min, -A_min, -J_min, -J_max,
              dcv2);
      t_test_size[0] = 3;
      t_test_size[1] = 7;
      std::copy(&dcv2[0], &dcv2[21], &t_test_data[0]);
      break;

     case 707:
      abcd_O_P(P_init, V_init, A_init, P_wayp, V_max, A_max, J_max, J_min,
               b_varargin_1);
      t_test_size[0] = 1;
      t_test_size[1] = 7;
      for (i1 = 0; i1 < 7; i1++) {
        t_test_data[i1].re = b_varargin_1[i1];
        t_test_data[i1].im = 0.0;
      }
      break;

     case 708:
      abcd_O_P(-P_init, -V_init, -A_init, -P_wayp, -V_min, -A_min, -J_min,
               -J_max, b_varargin_1);
      t_test_size[0] = 1;
      t_test_size[1] = 7;
      for (i1 = 0; i1 < 7; i1++) {
        t_test_data[i1].re = b_varargin_1[i1];
        t_test_data[i1].im = 0.0;
      }
      break;

     case 709:
      ac_O_P(P_init, V_init, A_init, P_wayp, V_max, J_max, J_min, dcv2);
      t_test_size[0] = 3;
      t_test_size[1] = 7;
      std::copy(&dcv2[0], &dcv2[21], &t_test_data[0]);
      break;

     case 710:
      ac_O_P(-P_init, -V_init, -A_init, -P_wayp, -V_min, -J_min, -J_max, dcv2);
      t_test_size[0] = 3;
      t_test_size[1] = 7;
      std::copy(&dcv2[0], &dcv2[21], &t_test_data[0]);
      break;

     case 711:
      acd_O_P(P_init, V_init, A_init, P_wayp, V_max, J_max, J_min, dcv);
      t_test_size[0] = 2;
      t_test_size[1] = 7;
      std::copy(&dcv[0], &dcv[14], &t_test_data[0]);
      break;

     case 712:
      acd_O_P(-P_init, -V_init, -A_init, -P_wayp, -V_min, -J_min, -J_max, dcv);
      t_test_size[0] = 2;
      t_test_size[1] = 7;
      std::copy(&dcv[0], &dcv[14], &t_test_data[0]);

      // -----------------------
      break;

     case 1101:
      abcdefg_NO_AVP(P_init, V_init, A_init, P_wayp, V_wayp, A_wayp, V_max,
                     A_min, J_max, J_min, b_varargin_1);
      t_test_size[0] = 1;
      t_test_size[1] = 7;
      for (i1 = 0; i1 < 7; i1++) {
        t_test_data[i1].re = b_varargin_1[i1];
        t_test_data[i1].im = 0.0;
      }
      break;

     case 1102:
      abcdefg_NO_AVP(-P_init, -V_init, -A_init, -P_wayp, -V_wayp, -A_wayp,
                     -V_min, -A_max, -J_min, -J_max, b_varargin_1);
      t_test_size[0] = 1;
      t_test_size[1] = 7;
      for (i1 = 0; i1 < 7; i1++) {
        t_test_data[i1].re = b_varargin_1[i1];
        t_test_data[i1].im = 0.0;
      }
      break;

     case 1103:
      abcdeg_NO_AVP(P_init, V_init, A_init, P_wayp, V_wayp, A_wayp, V_max, A_min,
                    J_max, J_min, dcv);
      t_test_size[0] = 2;
      t_test_size[1] = 7;
      std::copy(&dcv[0], &dcv[14], &t_test_data[0]);
      break;

     case 1104:
      abcdeg_NO_AVP(-P_init, -V_init, -A_init, -P_wayp, -V_wayp, -A_wayp, -V_min,
                    -A_max, -J_min, -J_max, dcv);
      t_test_size[0] = 2;
      t_test_size[1] = 7;
      std::copy(&dcv[0], &dcv[14], &t_test_data[0]);
      break;

     case 1105:
      acdefg_NO_AVP(P_init, V_init, A_init, P_wayp, V_wayp, A_wayp, V_max, A_min,
                    J_max, J_min, dcv);
      t_test_size[0] = 2;
      t_test_size[1] = 7;
      std::copy(&dcv[0], &dcv[14], &t_test_data[0]);
      break;

     case 1106:
      acdefg_NO_AVP(-P_init, -V_init, -A_init, -P_wayp, -V_wayp, -A_wayp, -V_min,
                    -A_max, -J_min, -J_max, dcv);
      t_test_size[0] = 2;
      t_test_size[1] = 7;
      std::copy(&dcv[0], &dcv[14], &t_test_data[0]);
      break;

     case 1107:
      acdeg_NO_AVP(P_init, V_init, A_init, P_wayp, V_wayp, A_wayp, V_max, J_max,
                   J_min, dcv1);
      t_test_size[0] = 4;
      t_test_size[1] = 7;
      std::copy(&dcv1[0], &dcv1[28], &t_test_data[0]);
      break;

     case 1108:
      acdeg_NO_AVP(-P_init, -V_init, -A_init, -P_wayp, -V_wayp, -A_wayp, -V_min,
                   -J_min, -J_max, dcv1);
      t_test_size[0] = 4;
      t_test_size[1] = 7;
      std::copy(&dcv1[0], &dcv1[28], &t_test_data[0]);

      // -----------------------
      break;

     case 1201:
      abcde_NO_VP(P_init, V_init, A_init, P_wayp, V_wayp, V_max, A_min, J_max,
                  J_min, dcv);
      t_test_size[0] = 2;
      t_test_size[1] = 7;
      std::copy(&dcv[0], &dcv[14], &t_test_data[0]);
      break;

     case 1202:
      abcde_NO_VP(-P_init, -V_init, -A_init, -P_wayp, -V_wayp, -V_min, -A_max,
                  -J_min, -J_max, dcv);
      t_test_size[0] = 2;
      t_test_size[1] = 7;
      std::copy(&dcv[0], &dcv[14], &t_test_data[0]);
      break;

     case 1203:
      abcdef_NO_VP(P_init, V_init, A_init, P_wayp, V_wayp, V_max, A_min, J_max,
                   J_min, b_varargin_1);
      t_test_size[0] = 1;
      t_test_size[1] = 7;
      for (i1 = 0; i1 < 7; i1++) {
        t_test_data[i1].re = b_varargin_1[i1];
        t_test_data[i1].im = 0.0;
      }
      break;

     case 1204:
      abcdef_NO_VP(-P_init, -V_init, -A_init, -P_wayp, -V_wayp, -V_min, -A_max,
                   -J_min, -J_max, b_varargin_1);
      t_test_size[0] = 1;
      t_test_size[1] = 7;
      for (i1 = 0; i1 < 7; i1++) {
        t_test_data[i1].re = b_varargin_1[i1];
        t_test_data[i1].im = 0.0;
      }
      break;

     case 1205:
      abcdefg_NO_VP(P_init, V_init, A_init, P_wayp, V_wayp, V_max, V_min, A_min,
                    J_max, J_min, dv);
      t_test_size[0] = 2;
      t_test_size[1] = 7;
      for (i1 = 0; i1 < 14; i1++) {
        t_test_data[i1].re = dv[i1];
        t_test_data[i1].im = 0.0;
      }
      break;

     case 1206:
      abcdefg_NO_VP(-P_init, -V_init, -A_init, -P_wayp, -V_wayp, -V_min, -V_max,
                    -A_max, -J_min, -J_max, dv);
      t_test_size[0] = 2;
      t_test_size[1] = 7;
      for (i1 = 0; i1 < 14; i1++) {
        t_test_data[i1].re = dv[i1];
        t_test_data[i1].im = 0.0;
      }
      break;

     case 1207:
      abcdeg_NO_VP(P_init, V_init, A_init, P_wayp, V_wayp, V_max, V_min, A_min,
                   J_max, J_min, dcv);
      t_test_size[0] = 2;
      t_test_size[1] = 7;
      std::copy(&dcv[0], &dcv[14], &t_test_data[0]);
      break;

     case 1208:
      abcdeg_NO_VP(-P_init, -V_init, -A_init, -P_wayp, -V_wayp, -V_min, -V_max,
                   -A_max, -J_min, -J_max, dcv);
      t_test_size[0] = 2;
      t_test_size[1] = 7;
      std::copy(&dcv[0], &dcv[14], &t_test_data[0]);
      break;

     case 1209:
      acde_NO_VP(P_init, V_init, A_init, P_wayp, V_wayp, V_max, J_max, J_min,
                 dcv);
      t_test_size[0] = 2;
      t_test_size[1] = 7;
      std::copy(&dcv[0], &dcv[14], &t_test_data[0]);
      break;

     case 1210:
      acde_NO_VP(-P_init, -V_init, -A_init, -P_wayp, -V_wayp, -V_min, -J_min,
                 -J_max, dcv);
      t_test_size[0] = 2;
      t_test_size[1] = 7;
      std::copy(&dcv[0], &dcv[14], &t_test_data[0]);
      break;

     case 1211:
      acdef_NO_VP(P_init, V_init, A_init, P_wayp, V_wayp, V_max, A_min, J_max,
                  J_min, dcv);
      t_test_size[0] = 2;
      t_test_size[1] = 7;
      std::copy(&dcv[0], &dcv[14], &t_test_data[0]);
      break;

     case 1212:
      acdef_NO_VP(-P_init, -V_init, -A_init, -P_wayp, -V_wayp, -V_min, -A_max,
                  -J_min, -J_max, dcv);
      t_test_size[0] = 2;
      t_test_size[1] = 7;
      std::copy(&dcv[0], &dcv[14], &t_test_data[0]);
      break;

     case 1213:
      acdefg_NO_VP(P_init, V_init, A_init, P_wayp, V_wayp, V_max, V_min, A_min,
                   J_max, J_min, dcv1);
      t_test_size[0] = 4;
      t_test_size[1] = 7;
      std::copy(&dcv1[0], &dcv1[28], &t_test_data[0]);
      break;

     case 1214:
      acdefg_NO_VP(-P_init, -V_init, -A_init, -P_wayp, -V_wayp, -V_min, -V_max,
                   -A_max, -J_min, -J_max, dcv1);
      t_test_size[0] = 4;
      t_test_size[1] = 7;
      std::copy(&dcv1[0], &dcv1[28], &t_test_data[0]);
      break;

     case 1215:
      t_test_size[0] = 1;
      t_test_size[1] = 7;
      std::memset(&t_test_data[0], 0, 7U * sizeof(creal_T));
      break;

     case 1216:
      t_test_size[0] = 1;
      t_test_size[1] = 7;
      std::memset(&t_test_data[0], 0, 7U * sizeof(creal_T));

      // -----------------------
      break;

     case 1301:
      abcde_NO_AP(P_init, V_init, A_init, P_wayp, A_wayp, V_max, A_min, J_max,
                  J_min, b_varargin_1);
      t_test_size[0] = 1;
      t_test_size[1] = 7;
      for (i1 = 0; i1 < 7; i1++) {
        t_test_data[i1].re = b_varargin_1[i1];
        t_test_data[i1].im = 0.0;
      }
      break;

     case 1302:
      abcde_NO_AP(-P_init, -V_init, -A_init, -P_wayp, -A_wayp, -V_min, -A_max,
                  -J_min, -J_max, b_varargin_1);
      t_test_size[0] = 1;
      t_test_size[1] = 7;
      for (i1 = 0; i1 < 7; i1++) {
        t_test_data[i1].re = b_varargin_1[i1];
        t_test_data[i1].im = 0.0;
      }
      break;

     case 1303:
      acde_NO_AP(P_init, V_init, A_init, P_wayp, A_wayp, V_max, J_max, J_min,
                 dcv);
      t_test_size[0] = 2;
      t_test_size[1] = 7;
      std::copy(&dcv[0], &dcv[14], &t_test_data[0]);
      break;

     case 1304:
      acde_NO_AP(-P_init, -V_init, -A_init, -P_wayp, -A_wayp, -V_min, -J_min,
                 -J_max, dcv);
      t_test_size[0] = 2;
      t_test_size[1] = 7;
      std::copy(&dcv[0], &dcv[14], &t_test_data[0]);
      break;

     case 1305:
      abcdefg_NO_AP(P_init, V_init, A_init, P_wayp, A_wayp, V_max, A_min, J_max,
                    J_min, b_varargin_1);
      t_test_size[0] = 1;
      t_test_size[1] = 7;
      for (i1 = 0; i1 < 7; i1++) {
        t_test_data[i1].re = b_varargin_1[i1];
        t_test_data[i1].im = 0.0;
      }
      break;

     case 1306:
      abcdefg_NO_AP(-P_init, -V_init, -A_init, -P_wayp, -A_wayp, -V_min, -A_max,
                    -J_min, -J_max, b_varargin_1);
      t_test_size[0] = 1;
      t_test_size[1] = 7;
      for (i1 = 0; i1 < 7; i1++) {
        t_test_data[i1].re = b_varargin_1[i1];
        t_test_data[i1].im = 0.0;
      }
      break;

     case 1307:
      acdefg_NO_AP(P_init, V_init, A_init, P_wayp, A_wayp, V_max, A_min, J_max,
                   J_min, dcv);
      t_test_size[0] = 2;
      t_test_size[1] = 7;
      std::copy(&dcv[0], &dcv[14], &t_test_data[0]);
      break;

     case 1308:
      acdefg_NO_AP(-P_init, -V_init, -A_init, -P_wayp, -A_wayp, -V_min, -A_max,
                   -J_min, -J_max, dcv);
      t_test_size[0] = 2;
      t_test_size[1] = 7;
      std::copy(&dcv[0], &dcv[14], &t_test_data[0]);
      break;

     case 1309:
      abcdeg_NO_AP(P_init, V_init, A_init, P_wayp, A_wayp, V_max, A_min, J_max,
                   J_min, dv);
      t_test_size[0] = 2;
      t_test_size[1] = 7;
      for (i1 = 0; i1 < 14; i1++) {
        t_test_data[i1].re = dv[i1];
        t_test_data[i1].im = 0.0;
      }
      break;

     case 1310:
      abcdeg_NO_AP(-P_init, -V_init, -A_init, -P_wayp, -A_wayp, -V_min, -A_max,
                   -J_min, -J_max, dv);
      t_test_size[0] = 2;
      t_test_size[1] = 7;
      for (i1 = 0; i1 < 14; i1++) {
        t_test_data[i1].re = dv[i1];
        t_test_data[i1].im = 0.0;
      }
      break;

     case 1311:
      acdeg_NO_AP(P_init, V_init, A_init, P_wayp, A_wayp, V_max, J_max, J_min,
                  dcv);
      t_test_size[0] = 2;
      t_test_size[1] = 7;
      std::copy(&dcv[0], &dcv[14], &t_test_data[0]);
      break;

     case 1312:
      acdeg_NO_AP(-P_init, -V_init, -A_init, -P_wayp, -A_wayp, -V_min, -J_min,
                  -J_max, dcv);
      t_test_size[0] = 2;
      t_test_size[1] = 7;
      std::copy(&dcv[0], &dcv[14], &t_test_data[0]);

      // -----------------------
      break;

     case 1701:
      abcd_NO_P(P_init, V_init, A_init, P_wayp, V_max, A_min, J_max, J_min,
                b_varargin_1);
      t_test_size[0] = 1;
      t_test_size[1] = 7;
      for (i1 = 0; i1 < 7; i1++) {
        t_test_data[i1].re = b_varargin_1[i1];
        t_test_data[i1].im = 0.0;
      }
      break;

     case 1702:
      abcd_NO_P(-P_init, -V_init, -A_init, -P_wayp, -V_min, -A_max, -J_min,
                -J_max, b_varargin_1);
      t_test_size[0] = 1;
      t_test_size[1] = 7;
      for (i1 = 0; i1 < 7; i1++) {
        t_test_data[i1].re = b_varargin_1[i1];
        t_test_data[i1].im = 0.0;
      }
      break;

     case 1703:
      acd_NO_P(P_init, V_init, A_init, P_wayp, V_max, J_max, J_min, dcv);
      t_test_size[0] = 2;
      t_test_size[1] = 7;
      std::copy(&dcv[0], &dcv[14], &t_test_data[0]);
      break;

     case 1704:
      acd_NO_P(-P_init, -V_init, -A_init, -P_wayp, -V_min, -J_min, -J_max, dcv);
      t_test_size[0] = 2;
      t_test_size[1] = 7;
      std::copy(&dcv[0], &dcv[14], &t_test_data[0]);
      break;

     default:
      t_test_size[0] = 1;
      t_test_size[1] = 7;
      std::memset(&t_test_data[0], 0, 7U * sizeof(creal_T));
      printf("Error: Solution prior ");
      fflush(stdout);
      printint(i1);
      printf(" is not valid!\n");
      fflush(stdout);
      break;
    }

    c_check(t_test_data, t_test_size, J_test, P_init, V_init, A_init, P_wayp,
            V_wayp, A_wayp, V_max, V_min, A_max, A_min, J_max, J_min, valid_data,
            valid_size);
    i1 = valid_size[1];
    for (int index_t = 0; index_t < i1; index_t++) {
      if (index_t + 1 > valid_size[1]) {
        rtDynamicBoundsError(index_t + 1, 1, valid_size[1], &eb_emlrtBCI);
      }

      if (valid_data[index_t]) {
        bool b_existing;
        bool exitg1;
        if (index_t + 1 > t_test_size[0]) {
          rtDynamicBoundsError(index_t + 1, 1, t_test_size[0], &cb_emlrtBCI);
        }

        for (varargin_1 = 0; varargin_1 < 7; varargin_1++) {
          d = t_test_data[index_t + t_test_size[0] * varargin_1].re;
          if (d > 0.0) {
            t_test_valid[varargin_1] = d;
          } else {
            t_test_valid[varargin_1] = 0.0;
          }
        }

        b_existing = false;
        ndx = 0;
        exitg1 = false;
        while ((!exitg1) && (ndx <= num_valid - 1)) {
          simplify_setp(t_test_valid, J_test, t_test_simpl_data,
                        t_test_simpl_size, J_test_simpl_data, J_test_simpl_size);
          for (i2 = 0; i2 < 7; i2++) {
            varargin_1 = ndx + 100 * i2;
            b_varargin_1[i2] = t_out[varargin_1];
            b_J_out[i2] = J_out[varargin_1];
          }

          simplify_setp(b_varargin_1, b_J_out, t_out_simpl_data,
                        t_out_simpl_size, J_out_simpl_data, J_out_simpl_size);

          // numerical
          // numerical
          if ((t_test_simpl_size[1] == t_out_simpl_size[1]) &&
              (J_test_simpl_size[1] == J_out_simpl_size[1])) {
            rtSizeEqNDCheck(&t_test_simpl_size[0], &t_out_simpl_size[0],
                            &emlrtECI);
            b_t_test_simpl_size[0] = 1;
            b_t_test_simpl_size[1] = t_test_simpl_size[1];
            loop_ub = t_test_simpl_size[1];
            for (i2 = 0; i2 < loop_ub; i2++) {
              b_varargin_1[i2] = t_test_simpl_data[i2] - t_out_simpl_data[i2];
            }

            coder::b_abs(b_varargin_1, b_t_test_simpl_size, t_test_simpl_data,
                         t_test_simpl_size);
            c_t_test_simpl_size[0] = 1;
            c_t_test_simpl_size[1] = t_test_simpl_size[1];
            loop_ub = t_test_simpl_size[1];
            for (i2 = 0; i2 < loop_ub; i2++) {
              b_t_test_simpl_data[i2] = (t_test_simpl_data[i2] < 1.0E-7);
            }

            if (coder::all(b_t_test_simpl_data, c_t_test_simpl_size)) {
              rtSizeEqNDCheck(&J_test_simpl_size[0], &J_out_simpl_size[0],
                              &b_emlrtECI);
              t_out_simpl_size[0] = 1;
              t_out_simpl_size[1] = J_test_simpl_size[1];
              loop_ub = J_test_simpl_size[1];
              for (i2 = 0; i2 < loop_ub; i2++) {
                t_out_simpl_data[i2] = J_test_simpl_data[i2] -
                  J_out_simpl_data[i2];
              }

              coder::b_abs(t_out_simpl_data, t_out_simpl_size, t_test_simpl_data,
                           t_test_simpl_size);
              c_t_test_simpl_size[0] = 1;
              c_t_test_simpl_size[1] = t_test_simpl_size[1];
              loop_ub = t_test_simpl_size[1];
              for (i2 = 0; i2 < loop_ub; i2++) {
                b_t_test_simpl_data[i2] = (t_test_simpl_data[i2] < 1.0E-7);
              }

              if (coder::all(b_t_test_simpl_data, c_t_test_simpl_size)) {
                b_existing = true;
              } else {
                b_existing = false;
              }
            } else {
              b_existing = false;
            }
          } else {
            b_existing = false;
          }

          if (b_existing) {
            exitg1 = true;
          } else {
            ndx++;
          }
        }

        if (!b_existing) {
          num_valid++;
          if (num_valid > 100) {
            rtDynamicBoundsError(101, 1, 100, &db_emlrtBCI);
          }

          for (i2 = 0; i2 < 7; i2++) {
            varargin_1 = (num_valid + 100 * i2) - 1;
            t_out[varargin_1] = t_test_valid[i2];
            J_out[varargin_1] = J_test[i2];
          }

          solution_out[num_valid - 1] = cases_data[b_index];
        }
      }
    }
  }

  if (1 > num_valid) {
    loop_ub = 0;
  } else {
    loop_ub = num_valid;
  }

  t_out_size[0] = loop_ub;
  t_out_size[1] = 7;
  if (1 > num_valid) {
    ndx = 0;
  } else {
    ndx = num_valid;
  }

  J_out_size[0] = ndx;
  J_out_size[1] = 7;
  for (i = 0; i < 7; i++) {
    for (i1 = 0; i1 < loop_ub; i1++) {
      t_out_data[i1 + loop_ub * i] = t_out[i1 + 100 * i];
    }

    for (i1 = 0; i1 < ndx; i1++) {
      J_out_data[i1 + ndx * i] = J_out[i1 + 100 * i];
    }
  }

  if (1 > num_valid) {
    b_index = 0;
  } else {
    b_index = num_valid;
  }

  *solution_out_size = b_index;
  if (0 <= b_index - 1) {
    std::copy(&solution_out[0], &solution_out[b_index], &solution_out_data[0]);
  }

  if (num_valid == 0) {
    printf("Error: Could not find valid optimal solution!\n");
    fflush(stdout);
  } else if (num_valid > 1) {
    printf("Debug: Multiple (");
    fflush(stdout);
    printint(static_cast<double>(num_valid));
    printf(") optimal solutions!\n");
    fflush(stdout);
    t_out_size[0] = loop_ub;
    t_out_size[1] = 7;
    for (i = 0; i < 7; i++) {
      for (i1 = 0; i1 < loop_ub; i1++) {
        t_out_data[i1 + loop_ub * i] = t_out[i1 + 100 * i];
      }
    }

    coder::sum(t_out_data, t_out_size, a__1_data, &varargin_1);
    coder::internal::sort(a__1_data, &varargin_1, iidx_data, solution_out_size);
    for (i = 0; i < *solution_out_size; i++) {
      a__1_data[i] = iidx_data[i];
    }

    t_out_size[0] = *solution_out_size;
    t_out_size[1] = 7;
    for (i = 0; i < 7; i++) {
      for (i1 = 0; i1 < *solution_out_size; i1++) {
        i2 = static_cast<int>(a__1_data[i1]);
        if ((i2 < 1) || (i2 > loop_ub)) {
          rtDynamicBoundsError(i2, 1, loop_ub, &fb_emlrtBCI);
        }

        t_out_data[i1 + *solution_out_size * i] = t_out[(i2 + 100 * i) - 1];
      }
    }

    J_out_size[0] = *solution_out_size;
    J_out_size[1] = 7;
    for (i = 0; i < 7; i++) {
      for (i1 = 0; i1 < *solution_out_size; i1++) {
        i2 = static_cast<int>(a__1_data[i1]);
        if ((i2 < 1) || (i2 > ndx)) {
          rtDynamicBoundsError(i2, 1, ndx, &gb_emlrtBCI);
        }

        J_out_data[i1 + *solution_out_size * i] = J_out[(i2 + 100 * i) - 1];
      }
    }

    for (i = 0; i < *solution_out_size; i++) {
      i1 = static_cast<int>(a__1_data[i]);
      if ((i1 < 1) || (i1 > b_index)) {
        rtDynamicBoundsError(i1, 1, b_index, &hb_emlrtBCI);
      }

      solution_out_data[i] = solution_out[i1 - 1];
    }
  }
}

// End of code generation (solve_O.cpp)
