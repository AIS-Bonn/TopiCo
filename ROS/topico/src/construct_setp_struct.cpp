//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// construct_setp_struct.cpp
//
// Code generation for function 'construct_setp_struct'
//

// Include files
#include "construct_setp_struct.h"
#include "eml_int_forloop_overflow_check.h"
#include "rt_nonfinite.h"
#include "simplify_setp.h"
#include "topico_wrapper_data.h"
#include "topico_wrapper_internal_types.h"
#include "topico_wrapper_rtwutil.h"
#include "topico_wrapper_types.h"
#include "coder_array.h"
#include <algorithm>

// Function Definitions
void b_construct_setp_struct(
    const double t_in2_data[], const int t_in2_size[2],
    const double J_in2_data[], const int J_in2_size[2],
    coder::array<double, 2U> &J_setp_struct_time,
    coder::array<double, 2U> &J_setp_struct_signals_values)
{
  coder::array<double, 2U> J_2;
  coder::array<double, 2U> b_result;
  coder::array<double, 2U> b_result_data;
  coder::array<double, 2U> d_result_data;
  coder::array<double, 2U> r;
  coder::array<double, 2U> result;
  coder::array<double, 2U> t_2;
  double b_tmp_data[8];
  double tmp_data[8];
  double c_result_data[7];
  double result_data[7];
  int b_loop_ub;
  int i;
  int loop_ub;
  int result_size_idx_1;
  signed char sizes_idx_0;
  signed char sizes_idx_1;
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
  if (t_in2_size[0] != 0) {
    sizes_idx_0 = static_cast<signed char>(t_in2_size[0]);
  } else {
    sizes_idx_0 = 1;
  }
  if ((t_in2_size[0] != sizes_idx_0) && (t_in2_size[0] != 0)) {
    h_rtErrorWithMessageID(i_emlrtRTEI.fName, i_emlrtRTEI.lineNo);
  }
  if (t_in2_size[0] != 0) {
    sizes_idx_1 = 7;
  } else {
    sizes_idx_1 = 0;
  }
  result.set_size(static_cast<int>(sizes_idx_0), static_cast<int>(sizes_idx_1));
  loop_ub = sizes_idx_1;
  for (i = 0; i < loop_ub; i++) {
    b_loop_ub = sizes_idx_0;
    for (result_size_idx_1 = 0; result_size_idx_1 < b_loop_ub;
         result_size_idx_1++) {
      result[result_size_idx_1 + result.size(0) * i] =
          t_in2_data[result_size_idx_1 + sizes_idx_0 * i];
    }
  }
  if (J_in2_size[0] != 0) {
    sizes_idx_0 = static_cast<signed char>(J_in2_size[0]);
  } else {
    sizes_idx_0 = 1;
  }
  if ((J_in2_size[0] != sizes_idx_0) && (J_in2_size[0] != 0)) {
    h_rtErrorWithMessageID(i_emlrtRTEI.fName, i_emlrtRTEI.lineNo);
  }
  if (J_in2_size[0] != 0) {
    sizes_idx_1 = 7;
  } else {
    sizes_idx_1 = 0;
  }
  b_result.set_size(static_cast<int>(sizes_idx_0),
                    static_cast<int>(sizes_idx_1));
  loop_ub = sizes_idx_1;
  for (i = 0; i < loop_ub; i++) {
    b_loop_ub = sizes_idx_0;
    for (result_size_idx_1 = 0; result_size_idx_1 < b_loop_ub;
         result_size_idx_1++) {
      b_result[result_size_idx_1 + b_result.size(0) * i] =
          J_in2_data[result_size_idx_1 + sizes_idx_0 * i];
    }
  }
  loop_ub = result.size(1);
  b_loop_ub = result.size(1);
  for (i = 0; i < loop_ub; i++) {
    result_data[i] = result[result.size(0) * i];
  }
  loop_ub = b_result.size(1);
  result_size_idx_1 = b_result.size(1);
  for (i = 0; i < loop_ub; i++) {
    c_result_data[i] = b_result[b_result.size(0) * i];
  }
  b_result_data.set(&result_data[0], 1, b_loop_ub);
  d_result_data.set(&c_result_data[0], 1, result_size_idx_1);
  simplify_setp(b_result_data, d_result_data, t_2, J_2);
  if ((t_2.size(1) != 0) && (t_2.size(1) != 1)) {
    b_loop_ub = t_2.size(1);
    if ((1 <= t_2.size(1) - 1) && (t_2.size(1) - 1 > 2147483646)) {
      coder::check_forloop_overflow_error();
    }
    for (result_size_idx_1 = 0; result_size_idx_1 <= b_loop_ub - 2;
         result_size_idx_1++) {
      t_2[result_size_idx_1 + 1] =
          t_2[result_size_idx_1] + t_2[result_size_idx_1 + 1];
    }
  }
  r.set_size(1, t_2.size(1) + 1);
  r[0] = 0.0;
  b_loop_ub = t_2.size(1);
  if ((1 <= t_2.size(1)) && (t_2.size(1) > 2147483646)) {
    coder::check_forloop_overflow_error();
  }
  for (result_size_idx_1 = 0; result_size_idx_1 < b_loop_ub;
       result_size_idx_1++) {
    r[result_size_idx_1 + 1] = t_2[result_size_idx_1];
  }
  b_loop_ub = r.size(1);
  loop_ub = r.size(1);
  for (i = 0; i < loop_ub; i++) {
    tmp_data[i] = r[i];
  }
  if (0 <= b_loop_ub - 1) {
    std::copy(&tmp_data[0], &tmp_data[b_loop_ub], &b_tmp_data[0]);
  }
  J_setp_struct_time.set_size(b_loop_ub, 1);
  for (i = 0; i < b_loop_ub; i++) {
    J_setp_struct_time[i] = b_tmp_data[i];
  }
  J_setp_struct_signals_values.set_size(1, J_2.size(1));
  loop_ub = J_2.size(1);
  for (i = 0; i < loop_ub; i++) {
    J_setp_struct_signals_values[i] = J_2[i];
  }
  J_setp_struct_signals_values.set_size(1, J_2.size(1) + 1);
  J_setp_struct_signals_values[J_2.size(1)] = 0.0;
}

void construct_setp_struct(const coder::array<cell_wrap_0, 2U> &t_in2,
                           const coder::array<cell_wrap_0, 2U> &J_in2,
                           coder::array<struct0_T, 2U> &J_setp_struct)
{
  static rtBoundsCheckInfo nb_emlrtBCI = {
      -1,                                                           // iFirst
      -1,                                                           // iLast
      69,                                                           // lineNo
      36,                                                           // colNo
      "t_in",                                                       // aName
      "construct_setp_struct",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/construct_setp_struct.m", // pName
      0                                                             // checkKind
  };
  static rtBoundsCheckInfo ob_emlrtBCI = {
      -1,                                                           // iFirst
      -1,                                                           // iLast
      69,                                                           // lineNo
      25,                                                           // colNo
      "t_in",                                                       // aName
      "construct_setp_struct",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/construct_setp_struct.m", // pName
      0                                                             // checkKind
  };
  static rtBoundsCheckInfo pb_emlrtBCI = {
      -1,                                                           // iFirst
      -1,                                                           // iLast
      70,                                                           // lineNo
      25,                                                           // colNo
      "J_in",                                                       // aName
      "construct_setp_struct",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/construct_setp_struct.m", // pName
      0                                                             // checkKind
  };
  static rtBoundsCheckInfo qb_emlrtBCI = {
      -1,                                                           // iFirst
      -1,                                                           // iLast
      70,                                                           // lineNo
      36,                                                           // colNo
      "J_in",                                                       // aName
      "construct_setp_struct",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/construct_setp_struct.m", // pName
      0                                                             // checkKind
  };
  static rtBoundsCheckInfo rb_emlrtBCI = {
      -1,                                                           // iFirst
      -1,                                                           // iLast
      79,                                                           // lineNo
      9,                                                            // colNo
      "J_setp_struct",                                              // aName
      "construct_setp_struct",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/construct_setp_struct.m", // pName
      0                                                             // checkKind
  };
  static rtBoundsCheckInfo sb_emlrtBCI = {
      -1,                                                           // iFirst
      -1,                                                           // iLast
      80,                                                           // lineNo
      23,                                                           // colNo
      "J_setp_struct",                                              // aName
      "construct_setp_struct",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/construct_setp_struct.m", // pName
      0                                                             // checkKind
  };
  static rtBoundsCheckInfo tb_emlrtBCI = {
      -1,                                                           // iFirst
      -1,                                                           // iLast
      80,                                                           // lineNo
      9,                                                            // colNo
      "J_setp_struct",                                              // aName
      "construct_setp_struct",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/construct_setp_struct.m", // pName
      0                                                             // checkKind
  };
  static rtBoundsCheckInfo ub_emlrtBCI = {
      -1,                                                            // iFirst
      -1,                                                            // iLast
      67,                                                            // lineNo
      9,                                                             // colNo
      "J_setp_struct",                                               // aName
      "cat",                                                         // fName
      "/usr/local/MATLAB/R2021a/toolbox/eml/lib/matlab/elmat/cat.m", // pName
      0 // checkKind
  };
  coder::array<double, 2U> J;
  coder::array<double, 2U> J_2;
  coder::array<double, 2U> t;
  coder::array<double, 2U> t_2;
  int i;
  int i1;
  int i2;
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
  J_setp_struct.set_size(1, t_in2.size(0));
  i = t_in2.size(0) - 1;
  for (i1 = 0; i1 <= i; i1++) {
    J_setp_struct[J_setp_struct.size(0) * i1].time.set_size(1, 1);
    J_setp_struct[i1].time[0] = 0.0;
    J_setp_struct[J_setp_struct.size(0) * i1].signals.values.set_size(1, 1);
    J_setp_struct[i1].signals.values[0] = 0.0;
  }
  i = t_in2.size(0);
  if (0 <= t_in2.size(0) - 1) {
    i2 = t_in2.size(1);
  }
  for (int index_axis = 0; index_axis < i; index_axis++) {
    int b_loop_ub;
    int loop_ub;
    int t_idx_1;
    t.set_size(1, 0);
    J.set_size(1, 0);
    for (int index_waypoint = 0; index_waypoint < i2; index_waypoint++) {
      int i3;
      if (index_axis > t_in2.size(0) - 1) {
        rtDynamicBoundsError(index_axis, 0, t_in2.size(0) - 1, &ob_emlrtBCI);
      }
      if (index_waypoint > t_in2.size(1) - 1) {
        rtDynamicBoundsError(index_waypoint, 0, t_in2.size(1) - 1,
                             &nb_emlrtBCI);
      }
      if (t.size(1) != 0) {
        b_loop_ub = t.size(1);
      } else {
        b_loop_ub = 0;
      }
      if (t_in2[index_axis + t_in2.size(0) * index_waypoint].f1.size(1) != 0) {
        loop_ub = t_in2[index_axis + t_in2.size(0) * index_waypoint].f1.size(1);
      } else {
        loop_ub = 0;
      }
      if (t.size(1) != 0) {
        t_idx_1 = t.size(1);
      } else {
        t_idx_1 = 0;
      }
      for (i1 = 0; i1 < b_loop_ub; i1++) {
        for (i3 = 0; i3 < 1; i3++) {
          t[i1] = t[i1];
        }
      }
      t.set_size(1, b_loop_ub + loop_ub);
      for (i1 = 0; i1 < loop_ub; i1++) {
        for (i3 = 0; i3 < 1; i3++) {
          t[i1 + t_idx_1] =
              t_in2[index_axis + t_in2.size(0) * index_waypoint].f1[i1];
        }
      }
      if (index_axis > J_in2.size(0) - 1) {
        rtDynamicBoundsError(index_axis, 0, J_in2.size(0) - 1, &pb_emlrtBCI);
      }
      if (index_waypoint > J_in2.size(1) - 1) {
        rtDynamicBoundsError(index_waypoint, 0, J_in2.size(1) - 1,
                             &qb_emlrtBCI);
      }
      if (J.size(1) != 0) {
        b_loop_ub = J.size(1);
      } else {
        b_loop_ub = 0;
      }
      if (J_in2[index_axis + J_in2.size(0) * index_waypoint].f1.size(1) != 0) {
        loop_ub = J_in2[index_axis + J_in2.size(0) * index_waypoint].f1.size(1);
      } else {
        loop_ub = 0;
      }
      if (J.size(1) != 0) {
        t_idx_1 = J.size(1);
      } else {
        t_idx_1 = 0;
      }
      for (i1 = 0; i1 < b_loop_ub; i1++) {
        for (i3 = 0; i3 < 1; i3++) {
          J[i1] = J[i1];
        }
      }
      J.set_size(1, b_loop_ub + loop_ub);
      for (i1 = 0; i1 < loop_ub; i1++) {
        for (i3 = 0; i3 < 1; i3++) {
          J[i1 + t_idx_1] =
              J_in2[index_axis + J_in2.size(0) * index_waypoint].f1[i1];
        }
      }
    }
    t.set_size(1, t.size(1));
    J.set_size(1, J.size(1));
    simplify_setp(t, J, t_2, J_2);
    if ((t_2.size(1) != 0) && (t_2.size(1) != 1)) {
      loop_ub = t_2.size(1);
      if ((1 <= t_2.size(1) - 1) && (t_2.size(1) - 1 > 2147483646)) {
        coder::check_forloop_overflow_error();
      }
      for (t_idx_1 = 0; t_idx_1 <= loop_ub - 2; t_idx_1++) {
        t_2[t_idx_1 + 1] = t_2[t_idx_1] + t_2[t_idx_1 + 1];
      }
    }
    t.set_size(1, t_2.size(1) + 1);
    t[0] = 0.0;
    loop_ub = t_2.size(1);
    if ((1 <= t_2.size(1)) && (t_2.size(1) > 2147483646)) {
      coder::check_forloop_overflow_error();
    }
    for (t_idx_1 = 0; t_idx_1 < loop_ub; t_idx_1++) {
      t[t_idx_1 + 1] = t_2[t_idx_1];
    }
    loop_ub = J_setp_struct.size(1);
    if (index_axis + 1 > J_setp_struct.size(1)) {
      rtDynamicBoundsError(index_axis + 1, 1, J_setp_struct.size(1),
                           &rb_emlrtBCI);
    }
    J_setp_struct[J_setp_struct.size(0) * index_axis].time.set_size(
        t.size(1),
        J_setp_struct[J_setp_struct.size(0) * index_axis].time.size(1));
    if (index_axis + 1 > loop_ub) {
      rtDynamicBoundsError(index_axis + 1, 1, loop_ub, &rb_emlrtBCI);
    }
    J_setp_struct[J_setp_struct.size(0) * index_axis].time.set_size(
        J_setp_struct[J_setp_struct.size(0) * index_axis].time.size(0), 1);
    b_loop_ub = t.size(1);
    for (i1 = 0; i1 < b_loop_ub; i1++) {
      if (index_axis + 1 > loop_ub) {
        rtDynamicBoundsError(index_axis + 1, 1, loop_ub, &rb_emlrtBCI);
      }
      J_setp_struct[index_axis].time[i1] = t[i1];
    }
    b_loop_ub = J_2.size(1) - 1;
    if (index_axis + 1 > J_setp_struct.size(1)) {
      rtDynamicBoundsError(index_axis + 1, 1, J_setp_struct.size(1),
                           &sb_emlrtBCI);
    }
    J_setp_struct[J_setp_struct.size(0) * index_axis].signals.values.set_size(
        1,
        J_setp_struct[J_setp_struct.size(0) * index_axis].signals.values.size(
            1));
    if (index_axis + 1 > J_setp_struct.size(1)) {
      rtDynamicBoundsError(index_axis + 1, 1, J_setp_struct.size(1),
                           &sb_emlrtBCI);
    }
    J_setp_struct[J_setp_struct.size(0) * index_axis].signals.values.set_size(
        J_setp_struct[J_setp_struct.size(0) * index_axis].signals.values.size(
            0),
        J_2.size(1));
    loop_ub = J_setp_struct.size(1);
    if (index_axis + 1 > J_setp_struct.size(1)) {
      rtDynamicBoundsError(index_axis + 1, 1, J_setp_struct.size(1),
                           &sb_emlrtBCI);
    }
    if (index_axis + 1 > J_setp_struct.size(1)) {
      rtDynamicBoundsError(index_axis + 1, 1, J_setp_struct.size(1),
                           &sb_emlrtBCI);
    }
    for (i1 = 0; i1 <= b_loop_ub; i1++) {
      if (index_axis + 1 > loop_ub) {
        rtDynamicBoundsError(index_axis + 1, 1, loop_ub, &tb_emlrtBCI);
      }
      J_setp_struct[index_axis].signals.values[i1] = J_2[i1];
    }
    loop_ub = J_setp_struct.size(1);
    if (index_axis + 1 > J_setp_struct.size(1)) {
      rtDynamicBoundsError(index_axis + 1, 1, J_setp_struct.size(1),
                           &ub_emlrtBCI);
    }
    J_setp_struct[J_setp_struct.size(0) * index_axis].signals.values.set_size(
        1,
        J_setp_struct[J_setp_struct.size(0) * index_axis].signals.values.size(
            1));
    if (index_axis + 1 > loop_ub) {
      rtDynamicBoundsError(index_axis + 1, 1, loop_ub, &tb_emlrtBCI);
    }
    J_setp_struct[J_setp_struct.size(0) * index_axis].signals.values.set_size(
        J_setp_struct[J_setp_struct.size(0) * index_axis].signals.values.size(
            0),
        J_2.size(1) + 1);
    if (index_axis + 1 > J_setp_struct.size(1)) {
      rtDynamicBoundsError(index_axis + 1, 1, J_setp_struct.size(1),
                           &sb_emlrtBCI);
    }
    J_setp_struct[index_axis].signals.values[J_2.size(1)] = 0.0;
  }
}

void construct_setp_struct(
    const double t_in2_data[], const int t_in2_size[2],
    const double J_in2_data[], const int J_in2_size[2],
    coder::array<double, 2U> &J_setp_struct_time,
    coder::array<double, 2U> &J_setp_struct_signals_values)
{
  coder::array<double, 2U> J_2;
  coder::array<double, 2U> b_result;
  coder::array<double, 2U> b_result_data;
  coder::array<double, 2U> d_result_data;
  coder::array<double, 2U> r;
  coder::array<double, 2U> result;
  coder::array<double, 2U> t_2;
  double b_tmp_data[5];
  double tmp_data[5];
  double c_result_data[4];
  double result_data[4];
  int b_result_size_idx_1;
  int i;
  int loop_ub;
  int result_size_idx_1;
  signed char sizes_idx_1;
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
  if (t_in2_size[0] != 0) {
    sizes_idx_1 = 4;
  } else {
    sizes_idx_1 = 0;
  }
  result.set_size(1, static_cast<int>(sizes_idx_1));
  loop_ub = sizes_idx_1;
  for (i = 0; i < loop_ub; i++) {
    for (result_size_idx_1 = 0; result_size_idx_1 < 1; result_size_idx_1++) {
      result[result.size(0) * i] = t_in2_data[i];
    }
  }
  if (J_in2_size[0] != 0) {
    sizes_idx_1 = 4;
  } else {
    sizes_idx_1 = 0;
  }
  b_result.set_size(1, static_cast<int>(sizes_idx_1));
  loop_ub = sizes_idx_1;
  for (i = 0; i < loop_ub; i++) {
    for (result_size_idx_1 = 0; result_size_idx_1 < 1; result_size_idx_1++) {
      b_result[b_result.size(0) * i] = J_in2_data[i];
    }
  }
  loop_ub = result.size(1);
  result_size_idx_1 = result.size(1);
  for (i = 0; i < loop_ub; i++) {
    result_data[i] = result[result.size(0) * i];
  }
  loop_ub = b_result.size(1);
  b_result_size_idx_1 = b_result.size(1);
  for (i = 0; i < loop_ub; i++) {
    c_result_data[i] = b_result[b_result.size(0) * i];
  }
  b_result_data.set(&result_data[0], 1, result_size_idx_1);
  d_result_data.set(&c_result_data[0], 1, b_result_size_idx_1);
  simplify_setp(b_result_data, d_result_data, t_2, J_2);
  if ((t_2.size(1) != 0) && (t_2.size(1) != 1)) {
    result_size_idx_1 = t_2.size(1);
    if ((1 <= t_2.size(1) - 1) && (t_2.size(1) - 1 > 2147483646)) {
      coder::check_forloop_overflow_error();
    }
    for (b_result_size_idx_1 = 0; b_result_size_idx_1 <= result_size_idx_1 - 2;
         b_result_size_idx_1++) {
      t_2[b_result_size_idx_1 + 1] =
          t_2[b_result_size_idx_1] + t_2[b_result_size_idx_1 + 1];
    }
  }
  r.set_size(1, t_2.size(1) + 1);
  r[0] = 0.0;
  result_size_idx_1 = t_2.size(1);
  if ((1 <= t_2.size(1)) && (t_2.size(1) > 2147483646)) {
    coder::check_forloop_overflow_error();
  }
  for (b_result_size_idx_1 = 0; b_result_size_idx_1 < result_size_idx_1;
       b_result_size_idx_1++) {
    r[b_result_size_idx_1 + 1] = t_2[b_result_size_idx_1];
  }
  result_size_idx_1 = r.size(1);
  loop_ub = r.size(1);
  for (i = 0; i < loop_ub; i++) {
    tmp_data[i] = r[i];
  }
  if (0 <= result_size_idx_1 - 1) {
    std::copy(&tmp_data[0], &tmp_data[result_size_idx_1], &b_tmp_data[0]);
  }
  J_setp_struct_time.set_size(result_size_idx_1, 1);
  for (i = 0; i < result_size_idx_1; i++) {
    J_setp_struct_time[i] = b_tmp_data[i];
  }
  J_setp_struct_signals_values.set_size(1, J_2.size(1));
  loop_ub = J_2.size(1);
  for (i = 0; i < loop_ub; i++) {
    J_setp_struct_signals_values[i] = J_2[i];
  }
  J_setp_struct_signals_values.set_size(1, J_2.size(1) + 1);
  J_setp_struct_signals_values[J_2.size(1)] = 0.0;
}

void construct_setp_struct(
    const double t_in2[7], const double J_in2[7],
    coder::array<double, 2U> &J_setp_struct_time,
    coder::array<double, 2U> &J_setp_struct_signals_values)
{
  coder::array<double, 2U> J_2;
  coder::array<double, 2U> c_tmp_data;
  coder::array<double, 2U> r;
  coder::array<double, 2U> t_2;
  coder::array<double, 2U> tmp_data;
  double e_tmp_data[8];
  double f_tmp_data[8];
  double b_tmp_data[7];
  double d_tmp_data[7];
  int b;
  int i;
  int k;
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
  for (i = 0; i < 7; i++) {
    b_tmp_data[i] = t_in2[i];
    d_tmp_data[i] = J_in2[i];
  }
  tmp_data.set(&b_tmp_data[0], 1, 7);
  c_tmp_data.set(&d_tmp_data[0], 1, 7);
  simplify_setp(tmp_data, c_tmp_data, t_2, J_2);
  if ((t_2.size(1) != 0) && (t_2.size(1) != 1)) {
    b = t_2.size(1);
    if ((1 <= t_2.size(1) - 1) && (t_2.size(1) - 1 > 2147483646)) {
      coder::check_forloop_overflow_error();
    }
    for (k = 0; k <= b - 2; k++) {
      t_2[k + 1] = t_2[k] + t_2[k + 1];
    }
  }
  r.set_size(1, t_2.size(1) + 1);
  r[0] = 0.0;
  b = t_2.size(1);
  if ((1 <= t_2.size(1)) && (t_2.size(1) > 2147483646)) {
    coder::check_forloop_overflow_error();
  }
  for (k = 0; k < b; k++) {
    r[k + 1] = t_2[k];
  }
  b = r.size(1);
  k = r.size(1);
  for (i = 0; i < k; i++) {
    e_tmp_data[i] = r[i];
  }
  if (0 <= b - 1) {
    std::copy(&e_tmp_data[0], &e_tmp_data[b], &f_tmp_data[0]);
  }
  J_setp_struct_time.set_size(b, 1);
  for (i = 0; i < b; i++) {
    J_setp_struct_time[i] = f_tmp_data[i];
  }
  J_setp_struct_signals_values.set_size(1, J_2.size(1));
  k = J_2.size(1);
  for (i = 0; i < k; i++) {
    J_setp_struct_signals_values[i] = J_2[i];
  }
  J_setp_struct_signals_values.set_size(1, J_2.size(1) + 1);
  J_setp_struct_signals_values[J_2.size(1)] = 0.0;
}

// End of code generation (construct_setp_struct.cpp)
