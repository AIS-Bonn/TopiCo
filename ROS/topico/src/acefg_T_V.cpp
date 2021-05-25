//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// acefg_T_V.cpp
//
// Code generation for function 'acefg_T_V'
//

// Include files
#include "acefg_T_V.h"
#include "rt_nonfinite.h"
#include "sqrt.h"
#include "topico_wrapper_data.h"
#include "topico_wrapper_rtwutil.h"
#include "topico_wrapper_types.h"
#include <cmath>

// Function Definitions
void acefg_T_V(double V_init, double A_init, double V_wayp, double V_min,
               double A_min, double J_max, double J_min, double T,
               creal_T t[14])
{
  creal_T l2[2];
  creal_T t3[2];
  creal_T t7[2];
  creal_T x_tmp[2];
  creal_T b_A_init;
  creal_T l18;
  double A_min_im_tmp;
  double A_min_re_tmp;
  double b_im;
  double b_l3_tmp;
  double b_re;
  double b_y_tmp;
  double im;
  double im_tmp;
  double l10_tmp;
  double l10_tmp_tmp;
  double l16;
  double l2_tmp;
  double l3_tmp;
  double l4;
  double l6;
  double l8_tmp;
  double re;
  double y_tmp;
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
  //  Generated on 02-Sep-2019 16:58:44
  l2_tmp = A_init * A_init;
  l3_tmp = J_max * V_init;
  b_l3_tmp = l3_tmp * 2.0;
  l4 = J_max * V_min * 2.0;
  l6 = 1.0 / J_min;
  l8_tmp = A_init * A_min * 2.0;
  l10_tmp_tmp = A_min * J_max;
  l10_tmp = l10_tmp_tmp * T * 2.0;
  if (J_max < 0.0) {
    f_rtErrorWithMessageID("sqrt", n_emlrtRTEI.fName, n_emlrtRTEI.lineNo);
  }
  l16 = 1.0 / (J_min + -J_max);
  b_A_init.re = V_wayp + -V_min;
  b_A_init.im = 0.0;
  coder::internal::scalar::b_sqrt(&b_A_init);
  A_min_re_tmp = A_min * 1.4142135623730951 * std::sqrt(J_max);
  A_min_im_tmp = A_min_re_tmp * b_A_init.im;
  l18.re = 2.0 * (A_min_re_tmp * b_A_init.re);
  A_min_re_tmp = J_min * l16;
  b_A_init.re = A_min_re_tmp *
                (((((l2_tmp - b_l3_tmp) + l4) - l8_tmp) - l10_tmp) + l18.re);
  b_A_init.im = A_min_re_tmp * (2.0 * A_min_im_tmp);
  coder::internal::scalar::b_sqrt(&b_A_init);
  l4 = ((((b_l3_tmp + l8_tmp) + -l4) + l10_tmp) + -l2_tmp) + l18.re;
  A_min_re_tmp = -J_min * l16;
  l18.re = A_min_re_tmp * l4;
  l18.im = A_min_re_tmp * (2.0 * A_min_im_tmp);
  coder::internal::scalar::b_sqrt(&l18);
  t3[0].re = -l6 * b_A_init.re;
  t3[0].im = -l6 * b_A_init.im;
  t3[1].re = -l6 * l18.re;
  t3[1].im = -l6 * l18.im;
  y_tmp = J_min * J_min;
  b_y_tmp = A_min * A_min;
  re = t3[0].re * t3[0].re - t3[0].im * t3[0].im;
  im_tmp = t3[0].re * t3[0].im;
  im = im_tmp + im_tmp;
  l2[0].re = re;
  l2[0].im = im;
  A_min_re_tmp = J_min * J_max;
  b_re = A_min_re_tmp * re;
  b_im = A_min_re_tmp * im;
  x_tmp[0].re = b_re;
  x_tmp[0].im = b_im;
  l4 = J_max * V_wayp * 2.0;
  l3_tmp = l3_tmp * -2.0 + l4;
  t7[0].re =
      (((((l3_tmp - y_tmp * re) + l2_tmp) + b_y_tmp) - l8_tmp) - l10_tmp) +
      b_re;
  t7[0].im = (0.0 - y_tmp * im) + b_im;
  re = t3[1].re * t3[1].re - t3[1].im * t3[1].im;
  im_tmp = t3[1].re * t3[1].im;
  im = im_tmp + im_tmp;
  b_re = A_min_re_tmp * re;
  b_im = A_min_re_tmp * im;
  A_min_im_tmp = y_tmp * re;
  t7[1].re =
      (((((l3_tmp - A_min_im_tmp) + l2_tmp) + b_y_tmp) - l8_tmp) - l10_tmp) +
      b_re;
  l3_tmp = y_tmp * im;
  t7[1].im = (0.0 - l3_tmp) + b_im;
  coder::internal::scalar::b_sqrt(&t7[0]);
  coder::internal::scalar::b_sqrt(&t7[1]);
  l6 = J_max * J_max;
  b_A_init.re = A_init - A_min;
  l18.re = b_l3_tmp - l4;
  l4 = A_min * J_min;
  if (t7[0].im == 0.0) {
    re = t7[0].re / J_max;
    im = 0.0;
  } else if (t7[0].re == 0.0) {
    re = 0.0;
    im = t7[0].im / J_max;
  } else {
    re = t7[0].re / J_max;
    im = t7[0].im / J_max;
  }
  l16 = -(b_A_init.re + J_min * t3[0].re);
  A_min_re_tmp = -(J_min * t3[0].im);
  if (A_min_re_tmp == 0.0) {
    t[0].re = l16 / J_max;
    t[0].im = 0.0;
  } else if (l16 == 0.0) {
    t[0].re = 0.0;
    t[0].im = A_min_re_tmp / J_max;
  } else {
    t[0].re = l16 / J_max;
    t[0].im = A_min_re_tmp / J_max;
  }
  t[2].re = 0.0;
  t[2].im = 0.0;
  t[4] = t3[0];
  t[6].re = 0.0;
  t[6].im = 0.0;
  t[8].re = 0.0;
  t[8].im = 0.0;
  im_tmp = re * im;
  l16 = ((((((((l18.re + y_tmp * l2[0].re) - l2_tmp) + b_y_tmp) +
             l6 * (re * re - im * im)) -
            l4 * t3[0].re * 2.0) +
           l10_tmp_tmp * t3[0].re * 2.0) +
          l10_tmp_tmp * re * 2.0) -
         x_tmp[0].re) *
        -0.5;
  A_min_re_tmp =
      (((((y_tmp * l2[0].im + l6 * (im_tmp + im_tmp)) - l4 * t3[0].im * 2.0) +
         l10_tmp_tmp * t3[0].im * 2.0) +
        l10_tmp_tmp * im * 2.0) -
       x_tmp[0].im) *
      -0.5;
  if (A_min_re_tmp == 0.0) {
    t[10].re = l16 / l10_tmp_tmp;
    t[10].im = 0.0;
  } else if (l16 == 0.0) {
    t[10].re = 0.0;
    t[10].im = A_min_re_tmp / l10_tmp_tmp;
  } else {
    t[10].re = l16 / l10_tmp_tmp;
    t[10].im = A_min_re_tmp / l10_tmp_tmp;
  }
  t[12].re = re;
  t[12].im = im;
  if (t7[1].im == 0.0) {
    re = t7[1].re / J_max;
    im = 0.0;
  } else if (t7[1].re == 0.0) {
    re = 0.0;
    im = t7[1].im / J_max;
  } else {
    re = t7[1].re / J_max;
    im = t7[1].im / J_max;
  }
  l16 = -(b_A_init.re + J_min * t3[1].re);
  A_min_re_tmp = -(J_min * t3[1].im);
  if (A_min_re_tmp == 0.0) {
    t[1].re = l16 / J_max;
    t[1].im = 0.0;
  } else if (l16 == 0.0) {
    t[1].re = 0.0;
    t[1].im = A_min_re_tmp / J_max;
  } else {
    t[1].re = l16 / J_max;
    t[1].im = A_min_re_tmp / J_max;
  }
  t[3].re = 0.0;
  t[3].im = 0.0;
  t[5] = t3[1];
  t[7].re = 0.0;
  t[7].im = 0.0;
  t[9].re = 0.0;
  t[9].im = 0.0;
  im_tmp = re * im;
  l16 = ((((((((l18.re + A_min_im_tmp) - l2_tmp) + b_y_tmp) +
             l6 * (re * re - im * im)) -
            l4 * t3[1].re * 2.0) +
           l10_tmp_tmp * t3[1].re * 2.0) +
          l10_tmp_tmp * re * 2.0) -
         b_re) *
        -0.5;
  A_min_re_tmp = (((((l3_tmp + l6 * (im_tmp + im_tmp)) - l4 * t3[1].im * 2.0) +
                    l10_tmp_tmp * t3[1].im * 2.0) +
                   l10_tmp_tmp * im * 2.0) -
                  b_im) *
                 -0.5;
  if (A_min_re_tmp == 0.0) {
    t[11].re = l16 / l10_tmp_tmp;
    t[11].im = 0.0;
  } else if (l16 == 0.0) {
    t[11].re = 0.0;
    t[11].im = A_min_re_tmp / l10_tmp_tmp;
  } else {
    t[11].re = l16 / l10_tmp_tmp;
    t[11].im = A_min_re_tmp / l10_tmp_tmp;
  }
  t[13].re = re;
  t[13].im = im;
}

// End of code generation (acefg_T_V.cpp)
