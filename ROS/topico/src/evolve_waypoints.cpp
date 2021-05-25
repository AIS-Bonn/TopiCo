//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// evolve_waypoints.cpp
//
// Code generation for function 'evolve_waypoints'
//

// Include files
#include "evolve_waypoints.h"
#include "eml_int_forloop_overflow_check.h"
#include "rt_nonfinite.h"
#include "topico_wrapper_rtwutil.h"
#include "topico_wrapper_types.h"
#include "coder_array.h"

// Function Definitions
void evolve_waypoints(const coder::array<double, 2U> &Waypoints,
                      const coder::array<double, 1U> &T,
                      coder::array<double, 2U> &Waypoints_evolved)
{
  static rtEqualityCheckInfo c_emlrtECI = {
      -1,                                                     // nDims
      44,                                                     // lineNo
      81,                                                     // colNo
      "evolve_waypoints",                                     // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/evolve_waypoints.m" // pName
  };
  static rtEqualityCheckInfo d_emlrtECI = {
      -1,                                                     // nDims
      44,                                                     // lineNo
      49,                                                     // colNo
      "evolve_waypoints",                                     // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/evolve_waypoints.m" // pName
  };
  static rtEqualityCheckInfo e_emlrtECI = {
      -1,                                                     // nDims
      44,                                                     // lineNo
      118,                                                    // colNo
      "evolve_waypoints",                                     // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/evolve_waypoints.m" // pName
  };
  static rtEqualityCheckInfo f_emlrtECI = {
      -1,                                                     // nDims
      44,                                                     // lineNo
      9,                                                      // colNo
      "evolve_waypoints",                                     // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/evolve_waypoints.m" // pName
  };
  static rtEqualityCheckInfo g_emlrtECI = {
      -1,                                                     // nDims
      45,                                                     // lineNo
      81,                                                     // colNo
      "evolve_waypoints",                                     // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/evolve_waypoints.m" // pName
  };
  static rtEqualityCheckInfo h_emlrtECI = {
      -1,                                                     // nDims
      45,                                                     // lineNo
      49,                                                     // colNo
      "evolve_waypoints",                                     // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/evolve_waypoints.m" // pName
  };
  static rtEqualityCheckInfo i_emlrtECI = {
      -1,                                                     // nDims
      45,                                                     // lineNo
      9,                                                      // colNo
      "evolve_waypoints",                                     // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/evolve_waypoints.m" // pName
  };
  coder::array<double, 1U> r;
  coder::array<double, 1U> r1;
  coder::array<double, 1U> r2;
  int k;
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
  Waypoints_evolved.set_size(Waypoints.size(0), 5);
  nx = Waypoints.size(0) * 5;
  for (k = 0; k < nx; k++) {
    Waypoints_evolved[k] = Waypoints[k];
  }
  if (Waypoints.size(0) != T.size(0)) {
    rtSizeEq1DError(Waypoints.size(0), T.size(0), &c_emlrtECI);
  }
  nx = Waypoints.size(0);
  r.set_size(Waypoints.size(0));
  for (k = 0; k < nx; k++) {
    r[k] = Waypoints[k + Waypoints.size(0) * 3] * T[k];
  }
  if (Waypoints.size(0) != r.size(0)) {
    rtSizeEq1DError(Waypoints.size(0), r.size(0), &d_emlrtECI);
  }
  nx = Waypoints.size(0);
  r1.set_size(Waypoints.size(0));
  for (k = 0; k < nx; k++) {
    r1[k] = 0.5 * Waypoints[k + Waypoints.size(0) * 4];
  }
  r2.set_size(T.size(0));
  nx = T.size(0);
  if ((1 <= T.size(0)) && (T.size(0) > 2147483646)) {
    coder::check_forloop_overflow_error();
  }
  for (k = 0; k < nx; k++) {
    r2[k] = T[k] * T[k];
  }
  if (r1.size(0) != r2.size(0)) {
    rtSizeEq1DError(r1.size(0), r2.size(0), &e_emlrtECI);
  }
  nx = r1.size(0);
  for (k = 0; k < nx; k++) {
    r1[k] = r1[k] * r2[k];
  }
  if (Waypoints.size(0) != r1.size(0)) {
    rtSizeEq1DError(Waypoints.size(0), r1.size(0), &d_emlrtECI);
  }
  rtSubAssignSizeCheck(((coder::array<double, 2U> *)&Waypoints)->size(), 1,
                       ((coder::array<double, 2U> *)&Waypoints)->size(), 1,
                       &f_emlrtECI);
  nx = Waypoints.size(0);
  for (k = 0; k < nx; k++) {
    Waypoints_evolved[k] = (Waypoints[k] + r[k]) + r1[k];
  }
  if (Waypoints.size(0) != T.size(0)) {
    rtSizeEq1DError(Waypoints.size(0), T.size(0), &g_emlrtECI);
  }
  nx = Waypoints.size(0);
  r.set_size(Waypoints.size(0));
  for (k = 0; k < nx; k++) {
    r[k] = Waypoints[k + Waypoints.size(0) * 4] * T[k];
  }
  if (Waypoints.size(0) != r.size(0)) {
    rtSizeEq1DError(Waypoints.size(0), r.size(0), &h_emlrtECI);
  }
  rtSubAssignSizeCheck(Waypoints_evolved.size(), 1,
                       ((coder::array<double, 2U> *)&Waypoints)->size(), 1,
                       &i_emlrtECI);
  nx = Waypoints.size(0);
  for (k = 0; k < nx; k++) {
    Waypoints_evolved[k + Waypoints_evolved.size(0) * 3] =
        Waypoints[k + Waypoints.size(0) * 3] + r[k];
  }
}

// End of code generation (evolve_waypoints.cpp)
