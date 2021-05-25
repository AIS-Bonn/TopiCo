//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// acdeg_T_V.cpp
//
// Code generation for function 'acdeg_T_V'
//

// Include files
#include "acdeg_T_V.h"
#include "rt_nonfinite.h"
#include "sqrt.h"
#include "topico_wrapper_data.h"
#include "topico_wrapper_rtwutil.h"
#include "topico_wrapper_types.h"
#include <cmath>

// Function Definitions
void acdeg_T_V(double V_init, double A_init, double V_wayp, double V_max,
               double V_min, double J_max, double J_min, double T,
               creal_T t[14])
{
  creal_T b_l3[2];
  creal_T t7[2];
  creal_T l21;
  creal_T l25;
  double a;
  double l23_re_tmp;
  double l26_re_tmp;
  double l28;
  double l29_im;
  double l29_re;
  double l2_tmp;
  double l3;
  double l4;
  double l9;
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
  //  Generated on 02-Sep-2019 16:42:37
  l2_tmp = J_min * J_min;
  l3 = rt_powd_snf(J_min, 3.0);
  if (J_max < 0.0) {
    f_rtErrorWithMessageID("sqrt", n_emlrtRTEI.fName, n_emlrtRTEI.lineNo);
  }
  l9 = std::sqrt(J_max);
  l4 = l2_tmp * l2_tmp;
  a = J_max + -J_min;
  if (-J_min < 0.0) {
    k_rtErrorWithMessageID(o_emlrtRTEI.fName, o_emlrtRTEI.lineNo);
  }
  if (a < 0.0) {
    k_rtErrorWithMessageID(o_emlrtRTEI.fName, o_emlrtRTEI.lineNo);
  }
  l21.re = V_wayp + -V_min;
  l21.im = 0.0;
  coder::internal::scalar::b_sqrt(&l21);
  l26_re_tmp = l2_tmp * (J_max * J_max) * 1.4142135623730951;
  l25.re = V_max + -V_min;
  l25.im = 0.0;
  coder::internal::scalar::b_sqrt(&l25);
  a = 1.4142135623730951 * rt_powd_snf(-J_min, 2.5) * rt_powd_snf(a, 1.5);
  l29_re = a * l25.re;
  l29_im = a * l25.im;
  a = J_max * l3 * 1.4142135623730951;
  l28 = 1.0 / ((l4 * l9 + l2_tmp * rt_powd_snf(l9, 5.0)) +
               -(l3 * rt_powd_snf(l9, 3.0) * 2.0));
  l23_re_tmp = (l4 * 1.4142135623730951 * l21.re + -(a * l21.re * 2.0)) +
               l26_re_tmp * l21.re;
  a = (l4 * 1.4142135623730951 * l21.im + -(a * l21.im * 2.0)) +
      l26_re_tmp * l21.im;
  t7[0].re = l28 * (l23_re_tmp + l29_re);
  t7[0].im = l28 * (a + l29_im);
  t7[1].re = -l28 * (l23_re_tmp - l29_re);
  t7[1].im = -l28 * (a - l29_im);
  l23_re_tmp = A_init * A_init;
  l28 = J_max * V_init;
  l25.re = J_min * (J_min + -J_max) *
           ((l23_re_tmp + J_max * V_max * 2.0) + -(l28 * 2.0));
  l25.im = 0.0;
  coder::internal::scalar::b_sqrt(&l25);
  l3 = J_min * J_max;
  a = 1.0 / (l2_tmp + -l3);
  l25.re *= a;
  l25.im *= a;
  l29_im = -J_min * (J_min - J_max);
  l21.re = l28 * -2.0 + J_max * V_min * 2.0;
  l28 = l25.re * l25.re - l25.im * l25.im;
  l9 = l25.re * l25.im;
  l4 = l9 + l9;
  a = (0.0 - l2_tmp * l4) + l3 * l4;
  b_l3[0].re = l29_im * (((l21.re - l2_tmp * l28) + l23_re_tmp) + l3 * l28);
  b_l3[0].im = l29_im * a;
  l4 = l9 + l9;
  a = (0.0 - l2_tmp * l4) + l3 * l4;
  b_l3[1].re = l29_im * (((l21.re - l2_tmp * l28) + l23_re_tmp) + l3 * l28);
  b_l3[1].im = l29_im * a;
  coder::internal::scalar::b_sqrt(&b_l3[0]);
  coder::internal::scalar::b_sqrt(&b_l3[1]);
  a = l2_tmp - l3;
  l21.re = J_max * T;
  if (b_l3[0].im == 0.0) {
    l28 = b_l3[0].re / a;
    l4 = 0.0;
  } else if (b_l3[0].re == 0.0) {
    l28 = 0.0;
    l4 = b_l3[0].im / a;
  } else {
    l28 = b_l3[0].re / a;
    l4 = b_l3[0].im / a;
  }
  l9 = J_min * l25.im;
  l23_re_tmp = A_init + J_min * l25.re;
  if (-l9 == 0.0) {
    t[0].re = -l23_re_tmp / J_max;
    t[0].im = 0.0;
  } else if (-l23_re_tmp == 0.0) {
    t[0].re = 0.0;
    t[0].im = -l9 / J_max;
  } else {
    t[0].re = -l23_re_tmp / J_max;
    t[0].im = -l9 / J_max;
  }
  t[2].re = 0.0;
  t[2].im = 0.0;
  t[4] = l25;
  l29_im = (l23_re_tmp - J_max * ((l25.re + l28) + t7[0].re)) + l21.re;
  l3 = l9 - J_max * ((l25.im + l4) + t7[0].im);
  if (l3 == 0.0) {
    t[6].re = l29_im / J_max;
    t[6].im = 0.0;
  } else if (l29_im == 0.0) {
    t[6].re = 0.0;
    t[6].im = l3 / J_max;
  } else {
    t[6].re = l29_im / J_max;
    t[6].im = l3 / J_max;
  }
  t[8].re = l28;
  t[8].im = l4;
  t[10].re = 0.0;
  t[10].im = 0.0;
  t[12] = t7[0];
  if (b_l3[1].im == 0.0) {
    l28 = b_l3[1].re / a;
    l4 = 0.0;
  } else if (b_l3[1].re == 0.0) {
    l28 = 0.0;
    l4 = b_l3[1].im / a;
  } else {
    l28 = b_l3[1].re / a;
    l4 = b_l3[1].im / a;
  }
  if (-l9 == 0.0) {
    t[1].re = -l23_re_tmp / J_max;
    t[1].im = 0.0;
  } else if (-l23_re_tmp == 0.0) {
    t[1].re = 0.0;
    t[1].im = -l9 / J_max;
  } else {
    t[1].re = -l23_re_tmp / J_max;
    t[1].im = -l9 / J_max;
  }
  t[3].re = 0.0;
  t[3].im = 0.0;
  t[5] = l25;
  l29_im = (l23_re_tmp - J_max * ((l25.re + l28) + t7[1].re)) + l21.re;
  l3 = l9 - J_max * ((l25.im + l4) + t7[1].im);
  if (l3 == 0.0) {
    t[7].re = l29_im / J_max;
    t[7].im = 0.0;
  } else if (l29_im == 0.0) {
    t[7].re = 0.0;
    t[7].im = l3 / J_max;
  } else {
    t[7].re = l29_im / J_max;
    t[7].im = l3 / J_max;
  }
  t[9].re = l28;
  t[9].im = l4;
  t[11].re = 0.0;
  t[11].im = 0.0;
  t[13] = t7[1];
}

// End of code generation (acdeg_T_V.cpp)
