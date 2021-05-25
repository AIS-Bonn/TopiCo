//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// cut_to_time.cpp
//
// Code generation for function 'cut_to_time'
//

// Include files
#include "cut_to_time.h"
#include "combineVectorElements.h"
#include "eml_int_forloop_overflow_check.h"
#include "rt_nonfinite.h"
#include "topico_wrapper_internal_types.h"
#include "topico_wrapper_rtwutil.h"
#include "topico_wrapper_types.h"
#include "coder_array.h"

// Function Definitions
void cut_to_time(const cell_wrap_17 *t_in, const cell_wrap_17 *J_in, double T,
                 cell_wrap_17 *t_out, cell_wrap_17 *J_out)
{
  static rtBoundsCheckInfo nb_emlrtBCI = {
      -1,                                                 // iFirst
      -1,                                                 // iLast
      51,                                                 // lineNo
      30,                                                 // colNo
      "t_cumsum",                                         // aName
      "cut_to_time",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/cut_to_time.m", // pName
      0                                                   // checkKind
  };
  static rtBoundsCheckInfo ob_emlrtBCI = {
      -1,                                                 // iFirst
      -1,                                                 // iLast
      52,                                                 // lineNo
      81,                                                 // colNo
      "t_cumsum",                                         // aName
      "cut_to_time",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/cut_to_time.m", // pName
      0                                                   // checkKind
  };
  static rtBoundsCheckInfo pb_emlrtBCI = {
      -1,                                                 // iFirst
      -1,                                                 // iLast
      56,                                                 // lineNo
      38,                                                 // colNo
      "t_in{index_axis}",                                 // aName
      "cut_to_time",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/cut_to_time.m", // pName
      0                                                   // checkKind
  };
  static rtBoundsCheckInfo qb_emlrtBCI = {
      -1,                                                 // iFirst
      -1,                                                 // iLast
      58,                                                 // lineNo
      50,                                                 // colNo
      "J_in{index_axis}",                                 // aName
      "cut_to_time",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/cut_to_time.m", // pName
      0                                                   // checkKind
  };
  static rtBoundsCheckInfo rb_emlrtBCI = {
      -1,                                                 // iFirst
      -1,                                                 // iLast
      58,                                                 // lineNo
      52,                                                 // colNo
      "J_in{index_axis}",                                 // aName
      "cut_to_time",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/cut_to_time.m", // pName
      0                                                   // checkKind
  };
  coder::array<double, 2U> b_t_in;
  coder::array<double, 2U> t_cumsum;
  coder::array<int, 2U> r;
  int k;
  int loop_ub;
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
  t_cumsum.set_size(1, t_in->f1.size(1));
  loop_ub = t_in->f1.size(1);
  for (k = 0; k < loop_ub; k++) {
    t_cumsum[k] = t_in->f1[k];
  }
  if ((t_cumsum.size(1) != 0) && (t_cumsum.size(1) != 1)) {
    loop_ub = t_cumsum.size(1);
    if ((1 <= t_cumsum.size(1) - 1) && (t_cumsum.size(1) - 1 > 2147483646)) {
      coder::check_forloop_overflow_error();
    }
    for (k = 0; k <= loop_ub - 2; k++) {
      t_cumsum[k + 1] = t_cumsum[k] + t_cumsum[k + 1];
    }
  }
  if (t_cumsum.size(1) < 1) {
    rtDynamicBoundsError(t_cumsum.size(1), 1, t_cumsum.size(1), &nb_emlrtBCI);
  }
  if (T >= t_cumsum[t_cumsum.size(1) - 1]) {
    if (t_cumsum.size(1) < 1) {
      rtDynamicBoundsError(t_cumsum.size(1), 1, t_cumsum.size(1), &ob_emlrtBCI);
    }
    t_out->f1.set_size(1, t_in->f1.size(1));
    loop_ub = t_in->f1.size(1);
    for (k = 0; k < loop_ub; k++) {
      t_out->f1[k] = t_in->f1[k];
    }
    t_out->f1.set_size(1, t_in->f1.size(1) + 1);
    t_out->f1[t_in->f1.size(1)] = T - t_cumsum[t_cumsum.size(1) - 1];
    J_out->f1.set_size(1, J_in->f1.size(1));
    loop_ub = J_in->f1.size(1);
    for (k = 0; k < loop_ub; k++) {
      J_out->f1[k] = J_in->f1[k];
    }
    J_out->f1.set_size(1, J_in->f1.size(1) + 1);
    J_out->f1[J_in->f1.size(1)] = 0.0;
  } else {
    double y;
    int i;
    k = t_cumsum.size(1);
    for (i = 0; i < k; i++) {
      if ((t_cumsum[i] <= T) && ((i + 1 < 1) || (i + 1 > t_in->f1.size(1)))) {
        rtDynamicBoundsError(i + 1, 1, t_in->f1.size(1), &pb_emlrtBCI);
      }
    }
    k = t_cumsum.size(1) - 1;
    loop_ub = 0;
    for (i = 0; i <= k; i++) {
      if (t_cumsum[i] <= T) {
        loop_ub++;
      }
    }
    r.set_size(1, loop_ub);
    loop_ub = 0;
    for (i = 0; i <= k; i++) {
      if (t_cumsum[i] <= T) {
        r[loop_ub] = i + 1;
        loop_ub++;
      }
    }
    b_t_in.set_size(1, r.size(1));
    loop_ub = r.size(1);
    for (k = 0; k < loop_ub; k++) {
      b_t_in[k] = t_in->f1[r[k] - 1];
    }
    y = coder::b_combineVectorElements(b_t_in);
    t_out->f1.set_size(1, r.size(1));
    loop_ub = r.size(1);
    for (k = 0; k < loop_ub; k++) {
      t_out->f1[k] = t_in->f1[r[k] - 1];
    }
    t_out->f1.set_size(1, r.size(1) + 1);
    t_out->f1[r.size(1)] = T - y;
    k = t_cumsum.size(1);
    loop_ub = 0;
    for (i = 0; i < k; i++) {
      if (t_cumsum[i] <= T) {
        loop_ub++;
      }
    }
    if (1 > J_in->f1.size(1)) {
      rtDynamicBoundsError(1, 1, J_in->f1.size(1), &qb_emlrtBCI);
    }
    if ((loop_ub + 1 < 1) || (loop_ub + 1 > J_in->f1.size(1))) {
      rtDynamicBoundsError(loop_ub + 1, 1, J_in->f1.size(1), &rb_emlrtBCI);
    }
    J_out->f1.set_size(1, loop_ub + 1);
    for (k = 0; k <= loop_ub; k++) {
      J_out->f1[k] = J_in->f1[k];
    }
  }
}

// End of code generation (cut_to_time.cpp)
