//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
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
#include "topico_wrapper_rtwutil.h"
#include "topico_wrapper_types.h"
#include "useConstantDim.h"
#include "coder_array.h"
#include "rt_nonfinite.h"
#include <cmath>
#include <math.h>

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
  static rtRunTimeErrorInfo s_emlrtRTEI = {
      236,                                                           // lineNo
      1,                                                             // colNo
      "unique_vector",                                               // fName
      "/usr/local/MATLAB/R2021a/toolbox/eml/lib/matlab/ops/unique.m" // pName
  };
  coder::array<double, 2U> J_rot;
  coder::array<double, 2U> J_x;
  coder::array<double, 2U> J_y;
  coder::array<double, 2U> t_cumsum;
  coder::array<double, 2U> t_cumsum_x;
  coder::array<double, 2U> t_cumsum_y;
  coder::array<int, 2U> idx;
  coder::array<int, 1U> iwork;
  coder::array<bool, 2U> b_t_cumsum_x;
  double index_x_data;
  double index_y_data;
  int ii_size[2];
  int b_i;
  int exponent;
  int i;
  int i2;
  int j;
  int k;
  int n;
  int na;
  int p;
  int pEnd;
  int q;
  int qEnd;
  bool exitg1;
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
  q = t_1.size(1);
  t_cumsum_x.set_size(1, t_1.size(1));
  for (i = 0; i < q; i++) {
    t_cumsum_x[i] = t_1[i];
  }
  coder::internal::useConstantDim(t_cumsum_x);
  q = t_2.size(1);
  t_cumsum_y.set_size(1, t_2.size(1));
  for (i = 0; i < q; i++) {
    t_cumsum_y[i] = t_2[i];
  }
  coder::internal::useConstantDim(t_cumsum_y);
  J_x.set_size(1, t_cumsum_x.size(1) + t_cumsum_y.size(1));
  q = t_cumsum_x.size(1);
  for (i = 0; i < q; i++) {
    J_x[i] = t_cumsum_x[i];
  }
  q = t_cumsum_y.size(1);
  for (i = 0; i < q; i++) {
    J_x[i + t_cumsum_x.size(1)] = t_cumsum_y[i];
  }
  na = J_x.size(1);
  n = J_x.size(1) + 1;
  idx.set_size(1, J_x.size(1));
  q = J_x.size(1);
  for (i = 0; i < q; i++) {
    idx[i] = 0;
  }
  if (J_x.size(1) != 0) {
    iwork.set_size(J_x.size(1));
    b_i = J_x.size(1) - 1;
    if ((1 <= J_x.size(1) - 1) && (J_x.size(1) - 1 > 2147483645)) {
      coder::check_forloop_overflow_error();
    }
    for (k = 1; k <= b_i; k += 2) {
      index_x_data = J_x[k];
      if ((J_x[k - 1] <= index_x_data) || rtIsNaN(index_x_data)) {
        idx[k - 1] = k;
        idx[k] = k + 1;
      } else {
        idx[k - 1] = k + 1;
        idx[k] = k;
      }
    }
    if ((J_x.size(1) & 1) != 0) {
      idx[J_x.size(1) - 1] = J_x.size(1);
    }
    b_i = 2;
    while (b_i < n - 1) {
      i2 = b_i << 1;
      j = 1;
      for (pEnd = b_i + 1; pEnd < n; pEnd = qEnd + b_i) {
        int kEnd;
        p = j;
        q = pEnd - 1;
        qEnd = j + i2;
        if (qEnd > n) {
          qEnd = n;
        }
        k = 0;
        kEnd = qEnd - j;
        while (k + 1 <= kEnd) {
          index_x_data = J_x[idx[q] - 1];
          i = idx[p - 1];
          if ((J_x[i - 1] <= index_x_data) || rtIsNaN(index_x_data)) {
            iwork[k] = i;
            p++;
            if (p == pEnd) {
              while (q + 1 < qEnd) {
                k++;
                iwork[k] = idx[q];
                q++;
              }
            }
          } else {
            iwork[k] = idx[q];
            q++;
            if (q + 1 == qEnd) {
              while (p < pEnd) {
                k++;
                iwork[k] = idx[p - 1];
                p++;
              }
            }
          }
          k++;
        }
        for (k = 0; k < kEnd; k++) {
          idx[(j + k) - 1] = iwork[k];
        }
        j = qEnd;
      }
      b_i = i2;
    }
  }
  t_cumsum.set_size(1, J_x.size(1));
  if ((1 <= J_x.size(1)) && (J_x.size(1) > 2147483646)) {
    coder::check_forloop_overflow_error();
  }
  for (k = 0; k < na; k++) {
    t_cumsum[k] = J_x[idx[k] - 1];
  }
  k = 0;
  while ((k + 1 <= na) && rtIsInf(t_cumsum[k]) && (t_cumsum[k] < 0.0)) {
    k++;
  }
  q = k;
  k = J_x.size(1);
  while ((k >= 1) && rtIsNaN(t_cumsum[k - 1])) {
    k--;
  }
  pEnd = J_x.size(1) - k;
  exitg1 = false;
  while ((!exitg1) && (k >= 1)) {
    index_x_data = t_cumsum[k - 1];
    if (rtIsInf(index_x_data) && (index_x_data > 0.0)) {
      k--;
    } else {
      exitg1 = true;
    }
  }
  b_i = (J_x.size(1) - k) - pEnd;
  p = 0;
  if (q > 0) {
    p = 1;
    if (q > 2147483646) {
      coder::check_forloop_overflow_error();
    }
  }
  while (q + 1 <= k) {
    index_x_data = t_cumsum[q];
    i2 = q;
    int exitg2;
    do {
      exitg2 = 0;
      q++;
      if (q + 1 > k) {
        exitg2 = 1;
      } else {
        index_y_data = std::abs(index_x_data / 2.0);
        if ((!rtIsInf(index_y_data)) && (!rtIsNaN(index_y_data))) {
          if (index_y_data <= 2.2250738585072014E-308) {
            index_y_data = 4.94065645841247E-324;
          } else {
            frexp(index_y_data, &exponent);
            index_y_data = std::ldexp(1.0, exponent - 53);
          }
        } else {
          index_y_data = rtNaN;
        }
        if ((!(std::abs(index_x_data - t_cumsum[q]) < index_y_data)) &&
            ((!rtIsInf(t_cumsum[q])) || (!rtIsInf(index_x_data)) ||
             ((t_cumsum[q] > 0.0) != (index_x_data > 0.0)))) {
          exitg2 = 1;
        }
      }
    } while (exitg2 == 0);
    p++;
    t_cumsum[p - 1] = index_x_data;
    if ((i2 + 1 <= q) && (q > 2147483646)) {
      coder::check_forloop_overflow_error();
    }
  }
  if (b_i > 0) {
    p++;
    t_cumsum[p - 1] = t_cumsum[k];
    if (b_i > 2147483646) {
      coder::check_forloop_overflow_error();
    }
  }
  q = k + b_i;
  for (j = 0; j < pEnd; j++) {
    t_cumsum[p + j] = t_cumsum[q + j];
  }
  p += pEnd;
  if (p > J_x.size(1)) {
    i_rtErrorWithMessageID(s_emlrtRTEI.fName, s_emlrtRTEI.lineNo);
  }
  if (1 > p) {
    p = 0;
  }
  t_cumsum.set_size(t_cumsum.size(0), p);
  J_rot.set_size(2, t_cumsum.size(1));
  q = t_cumsum.size(1) << 1;
  for (i = 0; i < q; i++) {
    J_rot[i] = 0.0;
  }
  if (J_1.size(1) != 0) {
    b_i = J_1.size(1);
  } else {
    b_i = 0;
  }
  J_x.set_size(1, b_i + 1);
  for (i = 0; i < b_i; i++) {
    J_x[i] = J_1[i];
  }
  J_x[b_i] = 0.0;
  //  Assume zero Jerk input after trajectory
  if (J_2.size(1) != 0) {
    b_i = J_2.size(1);
  } else {
    b_i = 0;
  }
  J_y.set_size(1, b_i + 1);
  for (i = 0; i < b_i; i++) {
    J_y[i] = J_2[i];
  }
  J_y[b_i] = 0.0;
  //  Assume zero Jerk input after trajectory
  i = t_cumsum.size(1);
  for (pEnd = 0; pEnd < i; pEnd++) {
    double d;
    double result_data_idx_0;
    if (pEnd + 1 > t_cumsum.size(1)) {
      rtDynamicBoundsError(pEnd + 1, 1, t_cumsum.size(1), &rb_emlrtBCI);
    }
    index_y_data = t_cumsum[pEnd];
    b_t_cumsum_x.set_size(1, t_cumsum_x.size(1));
    q = t_cumsum_x.size(1);
    for (p = 0; p < q; p++) {
      b_t_cumsum_x[p] = (t_cumsum_x[p] >= index_y_data);
    }
    coder::b_eml_find(b_t_cumsum_x, (int *)&b_i, ii_size);
    q = ii_size[1];
    for (p = 0; p < q; p++) {
      index_x_data = b_i;
    }
    if (pEnd + 1 > t_cumsum.size(1)) {
      rtDynamicBoundsError(pEnd + 1, 1, t_cumsum.size(1), &qb_emlrtBCI);
    }
    index_y_data = t_cumsum[pEnd];
    b_t_cumsum_x.set_size(1, t_cumsum_y.size(1));
    i2 = t_cumsum_y.size(1);
    for (p = 0; p < i2; p++) {
      b_t_cumsum_x[p] = (t_cumsum_y[p] >= index_y_data);
    }
    coder::b_eml_find(b_t_cumsum_x, (int *)&b_i, ii_size);
    i2 = ii_size[1];
    for (p = 0; p < i2; p++) {
      index_y_data = b_i;
    }
    if (q == 0) {
      index_x_data = static_cast<double>(t_1.size(1)) + 1.0;
      //  Assume zero Jerk input after trajectory
    }
    if (ii_size[1] == 0) {
      index_y_data = static_cast<double>(t_2.size(1)) + 1.0;
      //  Assume zero Jerk input after trajectory
    }
    for (p = 0; p < 1; p++) {
      if ((static_cast<int>(index_x_data) < 1) ||
          (static_cast<int>(index_x_data) > J_x.size(1))) {
        rtDynamicBoundsError(static_cast<int>(index_x_data), 1, J_x.size(1),
                             &pb_emlrtBCI);
      }
    }
    for (p = 0; p < 1; p++) {
      if ((static_cast<int>(index_y_data) < 1) ||
          (static_cast<int>(index_y_data) > J_y.size(1))) {
        rtDynamicBoundsError(static_cast<int>(index_y_data), 1, J_y.size(1),
                             &ob_emlrtBCI);
      }
    }
    result_data_idx_0 = J_x[static_cast<int>(index_x_data) - 1];
    index_y_data = J_y[static_cast<int>(index_y_data) - 1];
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
    index_x_data = std::cos(-alpha);
    d = std::sin(-alpha);
    if (pEnd + 1 > J_rot.size(1)) {
      rtDynamicBoundsError(pEnd + 1, 1, J_rot.size(1), &nb_emlrtBCI);
    }
    J_rot[2 * pEnd] = index_x_data * result_data_idx_0 + d * index_y_data;
    if (pEnd + 1 > J_rot.size(1)) {
      rtDynamicBoundsError(pEnd + 1, 1, J_rot.size(1), &nb_emlrtBCI);
    }
    J_rot[2 * pEnd + 1] = index_x_data * index_y_data - d * result_data_idx_0;
  }
  J_x.set_size(1, t_cumsum.size(1) + 1);
  J_x[0] = 0.0;
  b_i = t_cumsum.size(1);
  if ((1 <= t_cumsum.size(1)) && (t_cumsum.size(1) > 2147483646)) {
    coder::check_forloop_overflow_error();
  }
  for (j = 0; j < b_i; j++) {
    J_x[j + 1] = t_cumsum[j];
  }
  coder::diff(J_x, t_out_1);
  J_x.set_size(1, t_cumsum.size(1) + 1);
  J_x[0] = 0.0;
  b_i = t_cumsum.size(1);
  if ((1 <= t_cumsum.size(1)) && (t_cumsum.size(1) > 2147483646)) {
    coder::check_forloop_overflow_error();
  }
  for (j = 0; j < b_i; j++) {
    J_x[j + 1] = t_cumsum[j];
  }
  coder::diff(J_x, t_out_2);
  q = J_rot.size(1);
  J_out_1.set_size(1, J_rot.size(1));
  for (i = 0; i < q; i++) {
    J_out_1[i] = J_rot[2 * i];
  }
  q = J_rot.size(1);
  J_out_2.set_size(1, J_rot.size(1));
  for (i = 0; i < q; i++) {
    J_out_2[i] = J_rot[2 * i + 1];
  }
}

// End of code generation (rotate_jerk.cpp)
