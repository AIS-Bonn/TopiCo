//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// simplify_setp.cpp
//
// Code generation for function 'simplify_setp'
//

// Include files
#include "simplify_setp.h"
#include "diff.h"
#include "find.h"
#include "rt_nonfinite.h"
#include "topico_wrapper_rtwutil.h"
#include "topico_wrapper_types.h"
#include "coder_array.h"
#include "rt_nonfinite.h"

// Variable Definitions
static rtBoundsCheckInfo emlrtBCI = {
    -1,                                                   // iFirst
    -1,                                                   // iLast
    57,                                                   // lineNo
    21,                                                   // colNo
    "J_1",                                                // aName
    "simplify_setp",                                      // fName
    "/home/lmbeul/Desktop/TopiCo/MATLAB/simplify_setp.m", // pName
    0                                                     // checkKind
};

static rtBoundsCheckInfo b_emlrtBCI = {
    -1,                                                   // iFirst
    -1,                                                   // iLast
    53,                                                   // lineNo
    34,                                                   // colNo
    "t_1",                                                // aName
    "simplify_setp",                                      // fName
    "/home/lmbeul/Desktop/TopiCo/MATLAB/simplify_setp.m", // pName
    0                                                     // checkKind
};

static rtBoundsCheckInfo c_emlrtBCI = {
    -1,                                                   // iFirst
    -1,                                                   // iLast
    53,                                                   // lineNo
    34,                                                   // colNo
    "index_jerk",                                         // aName
    "simplify_setp",                                      // fName
    "/home/lmbeul/Desktop/TopiCo/MATLAB/simplify_setp.m", // pName
    0                                                     // checkKind
};

static rtBoundsCheckInfo d_emlrtBCI = {
    -1,                                                   // iFirst
    -1,                                                   // iLast
    53,                                                   // lineNo
    55,                                                   // colNo
    "t_1",                                                // aName
    "simplify_setp",                                      // fName
    "/home/lmbeul/Desktop/TopiCo/MATLAB/simplify_setp.m", // pName
    0                                                     // checkKind
};

static rtBoundsCheckInfo e_emlrtBCI = {
    -1,                                                   // iFirst
    -1,                                                   // iLast
    53,                                                   // lineNo
    55,                                                   // colNo
    "index_jerk",                                         // aName
    "simplify_setp",                                      // fName
    "/home/lmbeul/Desktop/TopiCo/MATLAB/simplify_setp.m", // pName
    0                                                     // checkKind
};

static rtBoundsCheckInfo f_emlrtBCI = {
    -1,                                                   // iFirst
    -1,                                                   // iLast
    53,                                                   // lineNo
    13,                                                   // colNo
    "t_1",                                                // aName
    "simplify_setp",                                      // fName
    "/home/lmbeul/Desktop/TopiCo/MATLAB/simplify_setp.m", // pName
    0                                                     // checkKind
};

static rtBoundsCheckInfo g_emlrtBCI = {
    -1,                                                   // iFirst
    -1,                                                   // iLast
    53,                                                   // lineNo
    13,                                                   // colNo
    "index_jerk",                                         // aName
    "simplify_setp",                                      // fName
    "/home/lmbeul/Desktop/TopiCo/MATLAB/simplify_setp.m", // pName
    0                                                     // checkKind
};

static rtBoundsCheckInfo h_emlrtBCI = {
    -1,                                                   // iFirst
    -1,                                                   // iLast
    54,                                                   // lineNo
    13,                                                   // colNo
    "t_1",                                                // aName
    "simplify_setp",                                      // fName
    "/home/lmbeul/Desktop/TopiCo/MATLAB/simplify_setp.m", // pName
    0                                                     // checkKind
};

static rtBoundsCheckInfo i_emlrtBCI = {
    -1,                                                   // iFirst
    -1,                                                   // iLast
    54,                                                   // lineNo
    13,                                                   // colNo
    "index_jerk",                                         // aName
    "simplify_setp",                                      // fName
    "/home/lmbeul/Desktop/TopiCo/MATLAB/simplify_setp.m", // pName
    0                                                     // checkKind
};

static rtBoundsCheckInfo j_emlrtBCI = {
    -1,                                                   // iFirst
    -1,                                                   // iLast
    56,                                                   // lineNo
    17,                                                   // colNo
    "t_1",                                                // aName
    "simplify_setp",                                      // fName
    "/home/lmbeul/Desktop/TopiCo/MATLAB/simplify_setp.m", // pName
    0                                                     // checkKind
};

// Function Definitions
void simplify_setp(const coder::array<double, 2U> &t_in,
                   const coder::array<double, 2U> &J_in,
                   coder::array<double, 2U> &t_out,
                   coder::array<double, 2U> &J_out)
{
  static rtBoundsCheckInfo nb_emlrtBCI = {
      -1,                                                   // iFirst
      -1,                                                   // iLast
      46,                                                   // lineNo
      18,                                                   // colNo
      "t_in",                                               // aName
      "simplify_setp",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/simplify_setp.m", // pName
      0                                                     // checkKind
  };
  static rtBoundsCheckInfo ob_emlrtBCI = {
      -1,                                                   // iFirst
      -1,                                                   // iLast
      47,                                                   // lineNo
      18,                                                   // colNo
      "J_in",                                               // aName
      "simplify_setp",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/simplify_setp.m", // pName
      0                                                     // checkKind
  };
  coder::array<double, 2U> J_1;
  coder::array<double, 2U> r1;
  coder::array<double, 2U> t_1;
  coder::array<int, 2U> ii;
  coder::array<int, 2U> r;
  coder::array<bool, 2U> x;
  int b_i;
  int end;
  int i;
  int trueCount;
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
  // numerical
  //  Find zero times and group
  end = t_in.size(1) - 1;
  trueCount = 0;
  for (i = 0; i <= end; i++) {
    if (t_in[i] > 1.0E-7) {
      trueCount++;
    }
  }
  ii.set_size(1, trueCount);
  trueCount = 0;
  for (i = 0; i <= end; i++) {
    if (t_in[i] > 1.0E-7) {
      ii[trueCount] = i + 1;
      trueCount++;
    }
  }
  t_1.set_size(1, ii.size(1));
  trueCount = ii.size(1);
  for (b_i = 0; b_i < trueCount; b_i++) {
    if (ii[b_i] > t_in.size(1)) {
      rtDynamicBoundsError(ii[b_i], 1, t_in.size(1), &nb_emlrtBCI);
    }
    t_1[b_i] = t_in[ii[b_i] - 1];
  }
  end = t_in.size(1) - 1;
  trueCount = 0;
  for (i = 0; i <= end; i++) {
    if (t_in[i] > 1.0E-7) {
      trueCount++;
    }
  }
  r.set_size(1, trueCount);
  trueCount = 0;
  for (i = 0; i <= end; i++) {
    if (t_in[i] > 1.0E-7) {
      r[trueCount] = i + 1;
      trueCount++;
    }
  }
  J_1.set_size(1, r.size(1));
  trueCount = r.size(1);
  for (b_i = 0; b_i < trueCount; b_i++) {
    if (r[b_i] > J_in.size(1)) {
      rtDynamicBoundsError(r[b_i], 1, J_in.size(1), &ob_emlrtBCI);
    }
    J_1[b_i] = J_in[r[b_i] - 1];
  }
  if (ii.size(1) != 0) {
    //  Find repeating jerks and group
    coder::diff(J_1, r1);
    x.set_size(1, r1.size(1));
    trueCount = r1.size(1);
    for (b_i = 0; b_i < trueCount; b_i++) {
      x[b_i] = (r1[b_i] == 0.0);
    }
    coder::eml_find(x, ii);
    J_1.set_size(1, ii.size(1));
    trueCount = ii.size(1);
    for (b_i = 0; b_i < trueCount; b_i++) {
      J_1[b_i] = ii[b_i];
    }
    b_i = static_cast<int>(((-1.0 - static_cast<double>(J_1.size(1))) + 1.0) /
                           -1.0);
    for (i = 0; i < b_i; i++) {
      trueCount = J_1.size(1) - i;
      if ((trueCount < 1) || (trueCount > J_1.size(1))) {
        rtDynamicBoundsError(trueCount, 1, J_1.size(1), &c_emlrtBCI);
      }
      end = static_cast<int>(J_1[trueCount - 1]);
      if ((end < 1) || (end > t_1.size(1))) {
        rtDynamicBoundsError(end, 1, t_1.size(1), &b_emlrtBCI);
      }
      if (trueCount > J_1.size(1)) {
        rtDynamicBoundsError(trueCount, 1, J_1.size(1), &e_emlrtBCI);
      }
      if ((end + 1 < 1) || (end + 1 > t_1.size(1))) {
        rtDynamicBoundsError(end + 1, 1, t_1.size(1), &d_emlrtBCI);
      }
      if (trueCount > J_1.size(1)) {
        rtDynamicBoundsError(trueCount, 1, J_1.size(1), &g_emlrtBCI);
      }
      if (end > t_1.size(1)) {
        rtDynamicBoundsError(end, 1, t_1.size(1), &f_emlrtBCI);
      }
      t_1[end - 1] = t_1[end - 1] + t_1[end];
      if (trueCount > J_1.size(1)) {
        rtDynamicBoundsError(trueCount, 1, J_1.size(1), &i_emlrtBCI);
      }
      if ((end + 1 < 1) || (end + 1 > t_1.size(1))) {
        rtDynamicBoundsError(end + 1, 1, t_1.size(1), &h_emlrtBCI);
      }
      t_1[end] = rtNaN;
    }
    x.set_size(1, t_1.size(1));
    trueCount = t_1.size(1);
    for (b_i = 0; b_i < trueCount; b_i++) {
      x[b_i] = rtIsNaN(t_1[b_i]);
    }
    x.set_size(1, x.size(1));
    trueCount = x.size(1) - 1;
    for (b_i = 0; b_i <= trueCount; b_i++) {
      x[b_i] = !x[b_i];
    }
    end = x.size(1) - 1;
    trueCount = 0;
    for (i = 0; i <= end; i++) {
      if (x[i]) {
        trueCount++;
      }
    }
    t_out.set_size(1, trueCount);
    trueCount = 0;
    for (i = 0; i <= end; i++) {
      if (x[i]) {
        if (i + 1 > t_1.size(1)) {
          rtDynamicBoundsError(i + 1, 1, t_1.size(1), &j_emlrtBCI);
        }
        t_out[trueCount] = t_1[i];
        trueCount++;
      }
    }
    end = x.size(1) - 1;
    trueCount = 0;
    for (i = 0; i <= end; i++) {
      if (x[i]) {
        trueCount++;
      }
    }
    J_out.set_size(1, trueCount);
    trueCount = 0;
    for (i = 0; i <= end; i++) {
      if (x[i]) {
        if (i + 1 > r.size(1)) {
          rtDynamicBoundsError(i + 1, 1, r.size(1), &emlrtBCI);
        }
        J_out[trueCount] = J_in[r[i] - 1];
        trueCount++;
      }
    }
  } else {
    t_out.set_size(1, 1);
    t_out[0] = 0.0;
    J_out.set_size(1, 1);
    J_out[0] = 0.0;
  }
}

void simplify_setp(const double t_in[7], const double J_in[7],
                   double t_out_data[], int t_out_size[2], double J_out_data[],
                   int J_out_size[2])
{
  coder::array<double, 2U> J_in_data;
  coder::array<double, 2U> c_i;
  coder::array<int, 2U> ii;
  coder::array<bool, 2U> x;
  double b_J_in_data[7];
  double t_1_data[7];
  int b_i;
  int b_trueCount;
  int i;
  int partialTrueCount;
  int trueCount;
  signed char b_tmp_data[7];
  signed char tmp_data[7];
  bool c_tmp_data[7];
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
  // numerical
  //  Find zero times and group
  trueCount = 0;
  partialTrueCount = 0;
  for (i = 0; i < 7; i++) {
    if (t_in[i] > 1.0E-7) {
      trueCount++;
      tmp_data[partialTrueCount] = static_cast<signed char>(i + 1);
      partialTrueCount++;
    }
  }
  for (b_i = 0; b_i < trueCount; b_i++) {
    t_1_data[b_i] = t_in[tmp_data[b_i] - 1];
  }
  b_trueCount = 0;
  partialTrueCount = 0;
  for (i = 0; i < 7; i++) {
    if (t_in[i] > 1.0E-7) {
      b_trueCount++;
      b_tmp_data[partialTrueCount] = static_cast<signed char>(i + 1);
      partialTrueCount++;
    }
  }
  if (trueCount != 0) {
    int tmp_size_idx_1;
    //  Find repeating jerks and group
    for (b_i = 0; b_i < b_trueCount; b_i++) {
      b_J_in_data[b_i] = J_in[b_tmp_data[b_i] - 1];
    }
    J_in_data.set(&b_J_in_data[0], 1, b_trueCount);
    coder::diff(J_in_data, c_i);
    x.set_size(1, c_i.size(1));
    partialTrueCount = c_i.size(1);
    for (b_i = 0; b_i < partialTrueCount; b_i++) {
      x[b_i] = (c_i[b_i] == 0.0);
    }
    coder::eml_find(x, ii);
    c_i.set_size(1, ii.size(1));
    partialTrueCount = ii.size(1);
    for (b_i = 0; b_i < partialTrueCount; b_i++) {
      c_i[b_i] = ii[b_i];
    }
    b_i = static_cast<int>(
        ((-1.0 - static_cast<double>(static_cast<signed char>(ii.size(1)))) +
         1.0) /
        -1.0);
    for (i = 0; i < b_i; i++) {
      int i1;
      int i2;
      partialTrueCount = c_i.size(1) - i;
      if ((partialTrueCount < 1) || (partialTrueCount > c_i.size(1))) {
        rtDynamicBoundsError(partialTrueCount, 1, c_i.size(1), &c_emlrtBCI);
      }
      tmp_size_idx_1 = static_cast<int>(c_i[partialTrueCount - 1]);
      if ((tmp_size_idx_1 < 1) || (tmp_size_idx_1 > trueCount)) {
        rtDynamicBoundsError(tmp_size_idx_1, 1, trueCount, &b_emlrtBCI);
      }
      if (partialTrueCount > c_i.size(1)) {
        rtDynamicBoundsError(partialTrueCount, 1, c_i.size(1), &e_emlrtBCI);
      }
      i1 = static_cast<int>(c_i[partialTrueCount - 1]) + 1;
      if ((i1 < 1) || (i1 > trueCount)) {
        rtDynamicBoundsError(i1, 1, trueCount, &d_emlrtBCI);
      }
      if (partialTrueCount > c_i.size(1)) {
        rtDynamicBoundsError(partialTrueCount, 1, c_i.size(1), &g_emlrtBCI);
      }
      i2 = static_cast<int>(c_i[partialTrueCount - 1]);
      if ((i2 < 1) || (i2 > trueCount)) {
        rtDynamicBoundsError(i2, 1, trueCount, &f_emlrtBCI);
      }
      t_1_data[i2 - 1] = t_1_data[tmp_size_idx_1 - 1] + t_1_data[i1 - 1];
      if (partialTrueCount > c_i.size(1)) {
        rtDynamicBoundsError(partialTrueCount, 1, c_i.size(1), &i_emlrtBCI);
      }
      tmp_size_idx_1 = static_cast<int>(c_i[partialTrueCount - 1]) + 1;
      if ((tmp_size_idx_1 < 1) || (tmp_size_idx_1 > trueCount)) {
        rtDynamicBoundsError(tmp_size_idx_1, 1, trueCount, &h_emlrtBCI);
      }
      t_1_data[tmp_size_idx_1 - 1] = rtNaN;
    }
    x.set_size(1, trueCount);
    for (b_i = 0; b_i < trueCount; b_i++) {
      x[b_i] = rtIsNaN(t_1_data[b_i]);
    }
    tmp_size_idx_1 = x.size(1);
    partialTrueCount = x.size(1);
    for (b_i = 0; b_i < partialTrueCount; b_i++) {
      c_tmp_data[b_i] = !x[b_i];
    }
    tmp_size_idx_1--;
    partialTrueCount = 0;
    for (i = 0; i <= tmp_size_idx_1; i++) {
      if (c_tmp_data[i]) {
        partialTrueCount++;
      }
    }
    t_out_size[0] = 1;
    t_out_size[1] = partialTrueCount;
    partialTrueCount = 0;
    for (i = 0; i <= tmp_size_idx_1; i++) {
      if (c_tmp_data[i]) {
        if (i + 1 > trueCount) {
          rtDynamicBoundsError(i + 1, 1, trueCount, &j_emlrtBCI);
        }
        t_out_data[partialTrueCount] = t_1_data[i];
        partialTrueCount++;
      }
    }
    trueCount = 0;
    for (i = 0; i <= tmp_size_idx_1; i++) {
      if (c_tmp_data[i]) {
        trueCount++;
      }
    }
    J_out_size[0] = 1;
    J_out_size[1] = trueCount;
    partialTrueCount = 0;
    for (i = 0; i <= tmp_size_idx_1; i++) {
      if (c_tmp_data[i]) {
        if (i + 1 > b_trueCount) {
          rtDynamicBoundsError(i + 1, 1, b_trueCount, &emlrtBCI);
        }
        J_out_data[partialTrueCount] = J_in[b_tmp_data[i] - 1];
        partialTrueCount++;
      }
    }
  } else {
    t_out_size[0] = 1;
    t_out_size[1] = 1;
    t_out_data[0] = 0.0;
    J_out_size[0] = 1;
    J_out_size[1] = 1;
    J_out_data[0] = 0.0;
  }
}

// End of code generation (simplify_setp.cpp)
