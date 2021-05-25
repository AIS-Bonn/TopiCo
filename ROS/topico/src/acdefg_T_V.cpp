//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// acdefg_T_V.cpp
//
// Code generation for function 'acdefg_T_V'
//

// Include files
#include "acdefg_T_V.h"
#include "rt_nonfinite.h"
#include "sqrt.h"
#include "topico_wrapper_data.h"
#include "topico_wrapper_rtwutil.h"
#include "topico_wrapper_types.h"
#include <cmath>

// Function Definitions
void acdefg_T_V(double V_init, double A_init, double V_wayp, double V_max,
                double V_min, double A_min, double J_max, double J_min,
                double T, creal_T t[14])
{
  creal_T b_l3[2];
  creal_T t4_tmp[2];
  creal_T t7[2];
  creal_T l8;
  double J_max_re;
  double ai;
  double ar;
  double b_l8_tmp;
  double b_l8_tmp_tmp;
  double im;
  double l2;
  double l3;
  double l8_tmp;
  double l8_tmp_tmp;
  double re;
  double t3_idx_0_im_tmp;
  double t3_idx_0_re_tmp;
  double y;
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
  //  Generated on 02-Sep-2019 16:18:40
  l2 = 1.0 / J_max;
  if (J_max < 0.0) {
    f_rtErrorWithMessageID("sqrt", n_emlrtRTEI.fName, n_emlrtRTEI.lineNo);
  }
  l8.re = V_wayp + -V_min;
  l8.im = 0.0;
  coder::internal::scalar::b_sqrt(&l8);
  l8_tmp = 1.4142135623730951 * std::sqrt(J_max);
  l8.re *= l8_tmp;
  l8.im *= l8_tmp;
  t7[0].re = -l2 * (A_min + l8.re);
  t7[0].im = -l2 * l8.im;
  t7[1].re = -l2 * (A_min - l8.re);
  t7[1].im = -l2 * (0.0 - l8.im);
  l8_tmp = A_init * A_init;
  b_l8_tmp = J_max * V_init * 2.0;
  l8.re =
      J_min * (J_min + -J_max) * ((l8_tmp + J_max * V_max * 2.0) + -b_l8_tmp);
  l8.im = 0.0;
  coder::internal::scalar::b_sqrt(&l8);
  l8_tmp_tmp = J_min * J_min;
  b_l8_tmp_tmp = J_min * J_max;
  l2 = 1.0 / (l8_tmp_tmp + -b_l8_tmp_tmp);
  l8.re *= l2;
  l8.im *= l2;
  t3_idx_0_re_tmp = l8.re;
  t3_idx_0_im_tmp = l8.im;
  l2 = A_min * A_min;
  y = A_min * J_max * 2.0;
  l8.re = (l2 - b_l8_tmp) + J_max * V_min * 2.0;
  J_max_re = J_max * l2 / J_min;
  l3 = A_min * (1.0 / J_min);
  re = t3_idx_0_re_tmp * t3_idx_0_re_tmp - t3_idx_0_im_tmp * t3_idx_0_im_tmp;
  l2 = t3_idx_0_re_tmp * t3_idx_0_im_tmp;
  b_l8_tmp = l2 + l2;
  ar = (((l8.re - l8_tmp_tmp * re) + l8_tmp) + b_l8_tmp_tmp * re) - J_max_re;
  ai = (0.0 - l8_tmp_tmp * b_l8_tmp) + b_l8_tmp_tmp * b_l8_tmp;
  if (ai == 0.0) {
    re = ar / y;
    b_l8_tmp = 0.0;
  } else if (ar == 0.0) {
    re = 0.0;
    b_l8_tmp = ai / y;
  } else {
    re = ar / y;
    b_l8_tmp = ai / y;
  }
  b_l3[0].re = re;
  b_l3[0].im = b_l8_tmp;
  re = J_min * t3_idx_0_re_tmp;
  im = J_min * t3_idx_0_im_tmp;
  t4_tmp[0].re = re;
  t4_tmp[0].im = im;
  ar = -(A_init + re);
  if (-im == 0.0) {
    t[0].re = ar / J_max;
    t[0].im = 0.0;
  } else if (ar == 0.0) {
    t[0].re = 0.0;
    t[0].im = -im / J_max;
  } else {
    t[0].re = ar / J_max;
    t[0].im = -im / J_max;
  }
  t[2].re = 0.0;
  t[2].im = 0.0;
  t[4].re = t3_idx_0_re_tmp;
  t[4].im = t3_idx_0_im_tmp;
  re = t3_idx_0_re_tmp * t3_idx_0_re_tmp - t3_idx_0_im_tmp * t3_idx_0_im_tmp;
  l2 = t3_idx_0_re_tmp * t3_idx_0_im_tmp;
  b_l8_tmp = l2 + l2;
  ar = (((l8.re - l8_tmp_tmp * re) + l8_tmp) + b_l8_tmp_tmp * re) - J_max_re;
  ai = (0.0 - l8_tmp_tmp * b_l8_tmp) + b_l8_tmp_tmp * b_l8_tmp;
  if (ai == 0.0) {
    re = ar / y;
    b_l8_tmp = 0.0;
  } else if (ar == 0.0) {
    re = 0.0;
    b_l8_tmp = ai / y;
  } else {
    re = ar / y;
    b_l8_tmp = ai / y;
  }
  b_l3[1].re = re;
  b_l3[1].im = b_l8_tmp;
  im = J_min * t3_idx_0_im_tmp;
  l2 = A_init + J_min * t3_idx_0_re_tmp;
  if (-im == 0.0) {
    t[1].re = -l2 / J_max;
    t[1].im = 0.0;
  } else if (-l2 == 0.0) {
    t[1].re = 0.0;
    t[1].im = -im / J_max;
  } else {
    t[1].re = -l2 / J_max;
    t[1].im = -im / J_max;
  }
  t[3].re = 0.0;
  t[3].im = 0.0;
  t[5].re = t3_idx_0_re_tmp;
  t[5].im = t3_idx_0_im_tmp;
  l8.re = A_min / J_min;
  J_max_re = J_max * T;
  t[8].re = l3;
  t[8].im = 0.0;
  t[9].re = l3;
  t[9].im = 0.0;
  ar = ((A_init + t4_tmp[0].re) -
        J_max * (((t3_idx_0_re_tmp + b_l3[0].re) + t7[0].re) + l8.re)) +
       J_max_re;
  ai = t4_tmp[0].im - J_max * ((t3_idx_0_im_tmp + b_l3[0].im) + t7[0].im);
  if (ai == 0.0) {
    t[6].re = ar / J_max;
    t[6].im = 0.0;
  } else if (ar == 0.0) {
    t[6].re = 0.0;
    t[6].im = ai / J_max;
  } else {
    t[6].re = ar / J_max;
    t[6].im = ai / J_max;
  }
  t[10] = b_l3[0];
  t[12] = t7[0];
  ar = (l2 - J_max * (((t3_idx_0_re_tmp + re) + t7[1].re) + l8.re)) + J_max_re;
  ai = im - J_max * ((t3_idx_0_im_tmp + b_l8_tmp) + t7[1].im);
  if (ai == 0.0) {
    t[7].re = ar / J_max;
    t[7].im = 0.0;
  } else if (ar == 0.0) {
    t[7].re = 0.0;
    t[7].im = ai / J_max;
  } else {
    t[7].re = ar / J_max;
    t[7].im = ai / J_max;
  }
  t[11] = b_l3[1];
  t[13] = t7[1];
}

// End of code generation (acdefg_T_V.cpp)
