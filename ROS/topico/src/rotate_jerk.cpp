//
// Student License - for use by students to meet course requirements and
// perform academic research at degree granting institutions only.  Not
// for government, commercial, or other organizational use.
//
// rotate_jerk.cpp
//
// Code generation for function 'rotate_jerk'
//

// Include files
#include "rotate_jerk.h"
#include "diff.h"
#include "eml_int_forloop_overflow_check.h"
#include "find.h"
#include "rt_nonfinite.h"
#include "topico_data.h"
#include "topico_rtwutil.h"
#include "topico_types.h"
#include "unique.h"
#include "coder_array.h"
#include <cmath>

// Function Definitions
void rotate_jerk(double alpha, const coder::array<double, 2U> &t_1,
                 const coder::array<double, 2U> &J_1,
                 const coder::array<double, 2U> &t_2,
                 const coder::array<double, 2U> &J_2,
                 coder::array<double, 2U> &t_out_1,
                 coder::array<double, 2U> &J_out_1,
                 coder::array<double, 2U> &t_out_2,
                 coder::array<double, 2U> &J_out_2)
{
  static rtBoundsCheckInfo nb_emlrtBCI = {
      -1,                                                 // iFirst
      -1,                                                 // iLast
      64,                                                 // lineNo
      19,                                                 // colNo
      "J_rot",                                            // aName
      "rotate_jerk",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/rotate_jerk.m", // pName
      0                                                   // checkKind
  };
  static rtBoundsCheckInfo ob_emlrtBCI = {
      -1,                                                 // iFirst
      -1,                                                 // iLast
      64,                                                 // lineNo
      66,                                                 // colNo
      "J_y",                                              // aName
      "rotate_jerk",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/rotate_jerk.m", // pName
      0                                                   // checkKind
  };
  static rtBoundsCheckInfo pb_emlrtBCI = {
      -1,                                                 // iFirst
      -1,                                                 // iLast
      64,                                                 // lineNo
      53,                                                 // colNo
      "J_x",                                              // aName
      "rotate_jerk",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/rotate_jerk.m", // pName
      0                                                   // checkKind
  };
  static rtBoundsCheckInfo qb_emlrtBCI = {
      -1,                                                 // iFirst
      -1,                                                 // iLast
      54,                                                 // lineNo
      45,                                                 // colNo
      "t_cumsum",                                         // aName
      "rotate_jerk",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/rotate_jerk.m", // pName
      0                                                   // checkKind
  };
  static rtBoundsCheckInfo rb_emlrtBCI = {
      -1,                                                 // iFirst
      -1,                                                 // iLast
      53,                                                 // lineNo
      45,                                                 // colNo
      "t_cumsum",                                         // aName
      "rotate_jerk",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/rotate_jerk.m", // pName
      0                                                   // checkKind
  };
  static rtBoundsCheckInfo sb_emlrtBCI = {
      -1,                                                 // iFirst
      -1,                                                 // iLast
      43,                                                 // lineNo
      29,                                                 // colNo
      "t_2",                                              // aName
      "rotate_jerk",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/rotate_jerk.m", // pName
      0                                                   // checkKind
  };
  static rtBoundsCheckInfo tb_emlrtBCI = {
      -1,                                                 // iFirst
      -1,                                                 // iLast
      42,                                                 // lineNo
      29,                                                 // colNo
      "t_1",                                              // aName
      "rotate_jerk",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/rotate_jerk.m", // pName
      0                                                   // checkKind
  };
  coder::array<double, 2U> J_rot;
  coder::array<double, 2U> J_x;
  coder::array<double, 2U> J_y;
  coder::array<double, 2U> t_cumsum;
  coder::array<double, 2U> t_cumsum_x;
  coder::array<double, 2U> t_cumsum_y;
  coder::array<bool, 2U> b_t_cumsum_x;
  double index_x_data;
  int ii_size[2];
  int i;
  int ii_data;
  int input_sizes_idx_1;
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
  if (1 > t_1.size(0)) {
    rtDynamicBoundsError(1, 1, t_1.size(0), &tb_emlrtBCI);
  }
  input_sizes_idx_1 = t_1.size(1);
  t_cumsum_x.set_size(1, t_1.size(1));
  for (i = 0; i < input_sizes_idx_1; i++) {
    t_cumsum_x[i] = t_1[t_1.size(0) * i];
  }
  if ((t_cumsum_x.size(1) != 0) && (t_cumsum_x.size(1) != 1)) {
    input_sizes_idx_1 = t_cumsum_x.size(1);
    if ((1 <= t_cumsum_x.size(1) - 1) &&
        (t_cumsum_x.size(1) - 1 > 2147483646)) {
      coder::check_forloop_overflow_error();
    }
    for (ii_data = 0; ii_data <= input_sizes_idx_1 - 2; ii_data++) {
      t_cumsum_x[ii_data + 1] = t_cumsum_x[ii_data] + t_cumsum_x[ii_data + 1];
    }
  }
  if (1 > t_2.size(0)) {
    rtDynamicBoundsError(1, 1, t_2.size(0), &sb_emlrtBCI);
  }
  input_sizes_idx_1 = t_2.size(1);
  t_cumsum_y.set_size(1, t_2.size(1));
  for (i = 0; i < input_sizes_idx_1; i++) {
    t_cumsum_y[i] = t_2[t_2.size(0) * i];
  }
  if ((t_cumsum_y.size(1) != 0) && (t_cumsum_y.size(1) != 1)) {
    input_sizes_idx_1 = t_cumsum_y.size(1);
    if ((1 <= t_cumsum_y.size(1) - 1) &&
        (t_cumsum_y.size(1) - 1 > 2147483646)) {
      coder::check_forloop_overflow_error();
    }
    for (ii_data = 0; ii_data <= input_sizes_idx_1 - 2; ii_data++) {
      t_cumsum_y[ii_data + 1] = t_cumsum_y[ii_data] + t_cumsum_y[ii_data + 1];
    }
  }
  J_x.set_size(1, t_cumsum_x.size(1) + t_cumsum_y.size(1));
  input_sizes_idx_1 = t_cumsum_x.size(1);
  for (i = 0; i < input_sizes_idx_1; i++) {
    J_x[i] = t_cumsum_x[i];
  }
  input_sizes_idx_1 = t_cumsum_y.size(1);
  for (i = 0; i < input_sizes_idx_1; i++) {
    J_x[i + t_cumsum_x.size(1)] = t_cumsum_y[i];
  }
  coder::unique_vector(J_x, t_cumsum);
  J_rot.set_size(2, t_cumsum.size(1));
  input_sizes_idx_1 = t_cumsum.size(1) << 1;
  for (i = 0; i < input_sizes_idx_1; i++) {
    J_rot[i] = 0.0;
  }
  if ((J_1.size(0) != 1) && ((J_1.size(0) != 0) && (J_1.size(1) != 0))) {
    h_rtErrorWithMessageID(k_emlrtRTEI.fName, k_emlrtRTEI.lineNo);
  }
  if ((J_1.size(0) != 0) && (J_1.size(1) != 0)) {
    input_sizes_idx_1 = J_1.size(1);
  } else {
    input_sizes_idx_1 = 0;
  }
  J_x.set_size(1, input_sizes_idx_1 + 1);
  for (i = 0; i < input_sizes_idx_1; i++) {
    J_x[i] = J_1[i];
  }
  J_x[input_sizes_idx_1] = 0.0;
  //  Assume zero Jerk input after trajectory
  if ((J_2.size(0) != 1) && ((J_2.size(0) != 0) && (J_2.size(1) != 0))) {
    h_rtErrorWithMessageID(k_emlrtRTEI.fName, k_emlrtRTEI.lineNo);
  }
  if ((J_2.size(0) != 0) && (J_2.size(1) != 0)) {
    input_sizes_idx_1 = J_2.size(1);
  } else {
    input_sizes_idx_1 = 0;
  }
  J_y.set_size(1, input_sizes_idx_1 + 1);
  for (i = 0; i < input_sizes_idx_1; i++) {
    J_y[i] = J_2[i];
  }
  J_y[input_sizes_idx_1] = 0.0;
  //  Assume zero Jerk input after trajectory
  i = t_cumsum.size(1);
  for (int b_index = 0; b_index < i; b_index++) {
    double d;
    double index_y_data;
    double result_data_idx_0;
    int i1;
    int loop_ub;
    if (b_index + 1 > t_cumsum.size(1)) {
      rtDynamicBoundsError(b_index + 1, 1, t_cumsum.size(1), &rb_emlrtBCI);
    }
    index_y_data = t_cumsum[b_index];
    b_t_cumsum_x.set_size(1, t_cumsum_x.size(1));
    input_sizes_idx_1 = t_cumsum_x.size(1);
    for (i1 = 0; i1 < input_sizes_idx_1; i1++) {
      b_t_cumsum_x[i1] = (t_cumsum_x[i1] >= index_y_data);
    }
    coder::b_eml_find(b_t_cumsum_x, (int *)&ii_data, ii_size);
    input_sizes_idx_1 = ii_size[1];
    for (i1 = 0; i1 < input_sizes_idx_1; i1++) {
      index_x_data = ii_data;
    }
    if (b_index + 1 > t_cumsum.size(1)) {
      rtDynamicBoundsError(b_index + 1, 1, t_cumsum.size(1), &qb_emlrtBCI);
    }
    index_y_data = t_cumsum[b_index];
    b_t_cumsum_x.set_size(1, t_cumsum_y.size(1));
    loop_ub = t_cumsum_y.size(1);
    for (i1 = 0; i1 < loop_ub; i1++) {
      b_t_cumsum_x[i1] = (t_cumsum_y[i1] >= index_y_data);
    }
    coder::b_eml_find(b_t_cumsum_x, (int *)&ii_data, ii_size);
    loop_ub = ii_size[1];
    for (i1 = 0; i1 < loop_ub; i1++) {
      index_y_data = ii_data;
    }
    if (input_sizes_idx_1 == 0) {
      index_x_data = static_cast<double>(t_1.size(1)) + 1.0;
      //  Assume zero Jerk input after trajectory
    }
    if (ii_size[1] == 0) {
      index_y_data = static_cast<double>(t_2.size(1)) + 1.0;
      //  Assume zero Jerk input after trajectory
    }
    for (i1 = 0; i1 < 1; i1++) {
      if ((static_cast<int>(index_x_data) < 1) ||
          (static_cast<int>(index_x_data) > J_x.size(1))) {
        rtDynamicBoundsError(static_cast<int>(index_x_data), 1, J_x.size(1),
                             &pb_emlrtBCI);
      }
    }
    for (i1 = 0; i1 < 1; i1++) {
      if ((static_cast<int>(index_y_data) < 1) ||
          (static_cast<int>(index_y_data) > J_y.size(1))) {
        rtDynamicBoundsError(static_cast<int>(index_y_data), 1, J_y.size(1),
                             &ob_emlrtBCI);
      }
    }
    result_data_idx_0 = J_x[static_cast<int>(index_x_data) - 1];
    index_x_data = J_y[static_cast<int>(index_y_data) - 1];
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
    index_y_data = std::cos(-alpha);
    d = std::sin(-alpha);
    if (b_index + 1 > J_rot.size(1)) {
      rtDynamicBoundsError(b_index + 1, 1, J_rot.size(1), &nb_emlrtBCI);
    }
    J_rot[2 * b_index] = index_y_data * result_data_idx_0 + d * index_x_data;
    if (b_index + 1 > J_rot.size(1)) {
      rtDynamicBoundsError(b_index + 1, 1, J_rot.size(1), &nb_emlrtBCI);
    }
    J_rot[2 * b_index + 1] =
        index_y_data * index_x_data - d * result_data_idx_0;
  }
  t_cumsum_x.set_size(1, t_cumsum.size(1) + 1);
  t_cumsum_x[0] = 0.0;
  input_sizes_idx_1 = t_cumsum.size(1);
  if ((1 <= t_cumsum.size(1)) && (t_cumsum.size(1) > 2147483646)) {
    coder::check_forloop_overflow_error();
  }
  for (ii_data = 0; ii_data < input_sizes_idx_1; ii_data++) {
    t_cumsum_x[ii_data + 1] = t_cumsum[ii_data];
  }
  coder::diff(t_cumsum_x, t_out_1);
  t_cumsum_x.set_size(1, t_cumsum.size(1) + 1);
  t_cumsum_x[0] = 0.0;
  input_sizes_idx_1 = t_cumsum.size(1);
  if ((1 <= t_cumsum.size(1)) && (t_cumsum.size(1) > 2147483646)) {
    coder::check_forloop_overflow_error();
  }
  for (ii_data = 0; ii_data < input_sizes_idx_1; ii_data++) {
    t_cumsum_x[ii_data + 1] = t_cumsum[ii_data];
  }
  coder::diff(t_cumsum_x, t_out_2);
  input_sizes_idx_1 = J_rot.size(1);
  J_out_1.set_size(1, J_rot.size(1));
  for (i = 0; i < input_sizes_idx_1; i++) {
    J_out_1[i] = J_rot[2 * i];
  }
  input_sizes_idx_1 = J_rot.size(1);
  J_out_2.set_size(1, J_rot.size(1));
  for (i = 0; i < input_sizes_idx_1; i++) {
    J_out_2[i] = J_rot[2 * i + 1];
  }
}

// End of code generation (rotate_jerk.cpp)
