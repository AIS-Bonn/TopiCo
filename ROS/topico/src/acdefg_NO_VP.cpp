//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// acdefg_NO_VP.cpp
//
// Code generation for function 'acdefg_NO_VP'
//

// Include files
#include "acdefg_NO_VP.h"
#include "power.h"
#include "rt_nonfinite.h"
#include "sqrt.h"
#include "topico_wrapper_data.h"
#include "topico_wrapper_rtwutil.h"
#include "topico_wrapper_types.h"
#include <cmath>

// Function Definitions
void acdefg_NO_VP(double P_init, double V_init, double A_init, double P_wayp,
                  double V_wayp, double V_max, double V_min, double A_min,
                  double J_max, double J_min, creal_T t[28])
{
  creal_T b_l2[4];
  creal_T b_l4[4];
  creal_T c_y[4];
  creal_T d_y[4];
  creal_T dcv[4];
  creal_T l7[4];
  creal_T t3[4];
  creal_T t4_tmp[4];
  creal_T t6[4];
  creal_T t6_tmp[4];
  creal_T t7[4];
  creal_T l8;
  double A_min_im;
  double A_min_re;
  double J_min_tmp;
  double b_J_max;
  double b_J_min_tmp;
  double b_im;
  double b_im_tmp;
  double b_l8_tmp;
  double b_re;
  double b_y;
  double c_im;
  double c_re;
  double d;
  double d1;
  double d2;
  double d3;
  double d_im;
  double im;
  double im_tmp;
  double l14_im;
  double l14_re;
  double l2;
  double l4;
  double l8_tmp;
  double l8_tmp_tmp;
  double re;
  double re_tmp;
  double y;
  double y_re;
  double y_re_tmp;
  double y_tmp;
  double y_tmp_tmp;
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
  //  Generated on 28-Aug-2019 13:51:13
  l2 = 1.0 / J_max;
  if (J_max < 0.0) {
    f_rtErrorWithMessageID("sqrt", n_emlrtRTEI.fName, n_emlrtRTEI.lineNo);
  }
  l8.re = V_wayp + -V_min;
  l8.im = 0.0;
  coder::internal::scalar::b_sqrt(&l8);
  l8_tmp_tmp = std::sqrt(J_max);
  l8_tmp = 1.4142135623730951 * l8_tmp_tmp;
  l8.re *= l8_tmp;
  l8.im *= l8_tmp;
  l14_re = -(l2 * (A_min + l8.re));
  l14_im = -(l2 * l8.im);
  A_min_re = A_min + -l8.re;
  A_min_im = -l8.im;
  l8.re = -(l2 * A_min_re);
  l8.im = -(l2 * A_min_im);
  t7[0].re = l14_re;
  t7[0].im = l14_im;
  t7[1] = l8;
  t7[2].re = l14_re;
  t7[2].im = l14_im;
  t7[3] = l8;
  l8_tmp = A_init * A_init;
  l8.re = -((J_min + -J_max) *
            ((l8_tmp + J_min * V_max * 2.0) + -(J_min * V_init * 2.0)));
  l8.im = 0.0;
  coder::internal::scalar::b_sqrt(&l8);
  l8_tmp_tmp = rt_powd_snf(l8_tmp_tmp, 3.0) - J_min * l8_tmp_tmp;
  l2 = -1.0 / l8_tmp_tmp;
  l14_re = l2 * l8.re;
  l14_im = l2 * l8.im;
  if (l8.im == 0.0) {
    re = l8.re / l8_tmp_tmp;
    im = 0.0;
  } else if (l8.re == 0.0) {
    re = 0.0;
    im = l8.im / l8_tmp_tmp;
  } else {
    re = l8.re / l8_tmp_tmp;
    im = l8.im / l8_tmp_tmp;
  }
  l8.re = re;
  l8.im = im;
  t3[0].re = l14_re;
  t3[0].im = l14_im;
  t3[1].re = l14_re;
  t3[1].im = l14_im;
  t3[2] = l8;
  t3[3] = l8;
  y_tmp_tmp = A_min * J_min;
  y_tmp = y_tmp_tmp * J_max;
  y_re_tmp = A_min * A_min;
  y_re = y_re_tmp * J_max;
  l2 = J_min * J_max;
  b_l8_tmp = l2 * V_init;
  l8.re = b_l8_tmp * 2.0;
  l14_re = l2 * V_min * 2.0;
  A_min_im = A_init * J_max;
  J_min_tmp = J_max * J_max;
  b_J_min_tmp = J_min * J_min_tmp;
  l4 = J_min * J_min;
  y = rt_powd_snf(J_max, 3.0);
  b_y = rt_powd_snf(A_min, 3.0);
  re = J_max * t3[0].re;
  im = J_max * t3[0].im;
  b_l2[0].re = re;
  b_l2[0].im = im;
  b_re = A_init + re;
  c_re = t7[0].re * t7[0].re - t7[0].im * t7[0].im;
  l14_im = t7[0].re * t7[0].im;
  b_im = l14_im + l14_im;
  t6_tmp[0].re = c_re;
  t6_tmp[0].im = b_im;
  l2 = A_min + J_max * t7[0].re;
  c_im = J_max * t7[0].im;
  l14_im = re * im;
  im_tmp = b_re * im;
  b_im_tmp = l2 * c_im;
  l8_tmp_tmp = im * im;
  l8_tmp_tmp = ((((((((y_re - J_min * (re * re - l8_tmp_tmp)) +
                      J_max * (b_re * b_re - l8_tmp_tmp)) -
                     J_min * (l2 * l2 - c_im * c_im)) +
                    l8.re) -
                   l14_re) -
                  A_min_im * b_re * 2.0) +
                 b_J_min_tmp * c_re) +
                y_tmp * t7[0].re * 2.0) *
               -0.5;
  l2 = ((((((0.0 - J_min * (l14_im + l14_im)) + J_max * (im_tmp + im_tmp)) -
           J_min * (b_im_tmp + b_im_tmp)) -
          A_min_im * im * 2.0) +
         b_J_min_tmp * b_im) +
        y_tmp * t7[0].im * 2.0) *
       -0.5;
  if (l2 == 0.0) {
    re = l8_tmp_tmp / y_tmp;
    d_im = 0.0;
  } else if (l8_tmp_tmp == 0.0) {
    re = 0.0;
    d_im = l2 / y_tmp;
  } else {
    re = l8_tmp_tmp / y_tmp;
    d_im = l2 / y_tmp;
  }
  t6[0].re = re;
  t6[0].im = d_im;
  b_re = t3[0].re * t3[0].re - t3[0].im * t3[0].im;
  l14_im = t3[0].re * t3[0].im;
  c_im = l14_im + l14_im;
  b_l4[0].re = b_re;
  b_l4[0].im = c_im;
  c_y[0].re = b_J_min_tmp * b_re;
  c_y[0].im = b_J_min_tmp * c_im;
  b_J_max = J_max * l4;
  t4_tmp[0].re = b_J_max * b_re;
  t4_tmp[0].im = b_J_max * c_im;
  d_y[0].re = re * re - d_im * d_im;
  d = re * d_im;
  re = J_max * t3[1].re;
  im = J_max * t3[1].im;
  b_l2[1].re = re;
  b_l2[1].im = im;
  b_re = A_init + re;
  c_re = t7[1].re * t7[1].re - t7[1].im * t7[1].im;
  l14_im = t7[1].re * t7[1].im;
  b_im = l14_im + l14_im;
  t6_tmp[1].re = c_re;
  t6_tmp[1].im = b_im;
  l2 = A_min + J_max * t7[1].re;
  c_im = J_max * t7[1].im;
  l14_im = re * im;
  im_tmp = b_re * im;
  b_im_tmp = l2 * c_im;
  l8_tmp_tmp = im * im;
  l8_tmp_tmp = ((((((((y_re - J_min * (re * re - l8_tmp_tmp)) +
                      J_max * (b_re * b_re - l8_tmp_tmp)) -
                     J_min * (l2 * l2 - c_im * c_im)) +
                    l8.re) -
                   l14_re) -
                  A_min_im * b_re * 2.0) +
                 b_J_min_tmp * c_re) +
                y_tmp * t7[1].re * 2.0) *
               -0.5;
  l2 = ((((((0.0 - J_min * (l14_im + l14_im)) + J_max * (im_tmp + im_tmp)) -
           J_min * (b_im_tmp + b_im_tmp)) -
          A_min_im * im * 2.0) +
         b_J_min_tmp * b_im) +
        y_tmp * t7[1].im * 2.0) *
       -0.5;
  if (l2 == 0.0) {
    re = l8_tmp_tmp / y_tmp;
    d_im = 0.0;
  } else if (l8_tmp_tmp == 0.0) {
    re = 0.0;
    d_im = l2 / y_tmp;
  } else {
    re = l8_tmp_tmp / y_tmp;
    d_im = l2 / y_tmp;
  }
  t6[1].re = re;
  t6[1].im = d_im;
  b_re = t3[1].re * t3[1].re - t3[1].im * t3[1].im;
  l14_im = t3[1].re * t3[1].im;
  c_im = l14_im + l14_im;
  b_l4[1].re = b_re;
  b_l4[1].im = c_im;
  c_y[1].re = b_J_min_tmp * b_re;
  c_y[1].im = b_J_min_tmp * c_im;
  t4_tmp[1].re = b_J_max * b_re;
  t4_tmp[1].im = b_J_max * c_im;
  d_y[1].re = re * re - d_im * d_im;
  d1 = re * d_im;
  re = J_max * t3[2].re;
  im = J_max * t3[2].im;
  b_l2[2].re = re;
  b_l2[2].im = im;
  b_re = A_init + re;
  c_re = t7[2].re * t7[2].re - t7[2].im * t7[2].im;
  l14_im = t7[2].re * t7[2].im;
  b_im = l14_im + l14_im;
  t6_tmp[2].re = c_re;
  t6_tmp[2].im = b_im;
  l2 = A_min + J_max * t7[2].re;
  c_im = J_max * t7[2].im;
  l14_im = re * im;
  im_tmp = b_re * im;
  b_im_tmp = l2 * c_im;
  l8_tmp_tmp = im * im;
  l8_tmp_tmp = ((((((((y_re - J_min * (re * re - l8_tmp_tmp)) +
                      J_max * (b_re * b_re - l8_tmp_tmp)) -
                     J_min * (l2 * l2 - c_im * c_im)) +
                    l8.re) -
                   l14_re) -
                  A_min_im * b_re * 2.0) +
                 b_J_min_tmp * c_re) +
                y_tmp * t7[2].re * 2.0) *
               -0.5;
  l2 = ((((((0.0 - J_min * (l14_im + l14_im)) + J_max * (im_tmp + im_tmp)) -
           J_min * (b_im_tmp + b_im_tmp)) -
          A_min_im * im * 2.0) +
         b_J_min_tmp * b_im) +
        y_tmp * t7[2].im * 2.0) *
       -0.5;
  if (l2 == 0.0) {
    re = l8_tmp_tmp / y_tmp;
    d_im = 0.0;
  } else if (l8_tmp_tmp == 0.0) {
    re = 0.0;
    d_im = l2 / y_tmp;
  } else {
    re = l8_tmp_tmp / y_tmp;
    d_im = l2 / y_tmp;
  }
  t6[2].re = re;
  t6[2].im = d_im;
  b_re = t3[2].re * t3[2].re - t3[2].im * t3[2].im;
  l14_im = t3[2].re * t3[2].im;
  c_im = l14_im + l14_im;
  b_l4[2].re = b_re;
  b_l4[2].im = c_im;
  c_y[2].re = b_J_min_tmp * b_re;
  c_y[2].im = b_J_min_tmp * c_im;
  t4_tmp[2].re = b_J_max * b_re;
  t4_tmp[2].im = b_J_max * c_im;
  d_y[2].re = re * re - d_im * d_im;
  d2 = re * d_im;
  re = J_max * t3[3].re;
  im = J_max * t3[3].im;
  re_tmp = A_init + re;
  c_re = t7[3].re * t7[3].re - t7[3].im * t7[3].im;
  l14_im = t7[3].re * t7[3].im;
  b_im = l14_im + l14_im;
  l2 = A_min + J_max * t7[3].re;
  c_im = J_max * t7[3].im;
  l14_im = re * im;
  im_tmp = re_tmp * im;
  b_im_tmp = l2 * c_im;
  l8_tmp_tmp = im * im;
  l8_tmp_tmp = ((((((((y_re - J_min * (re * re - l8_tmp_tmp)) +
                      J_max * (re_tmp * re_tmp - l8_tmp_tmp)) -
                     J_min * (l2 * l2 - c_im * c_im)) +
                    l8.re) -
                   l14_re) -
                  A_min_im * re_tmp * 2.0) +
                 b_J_min_tmp * c_re) +
                y_tmp * t7[3].re * 2.0) *
               -0.5;
  l2 = ((((((0.0 - J_min * (l14_im + l14_im)) + J_max * (im_tmp + im_tmp)) -
           J_min * (b_im_tmp + b_im_tmp)) -
          A_min_im * im * 2.0) +
         b_J_min_tmp * b_im) +
        y_tmp * t7[3].im * 2.0) *
       -0.5;
  if (l2 == 0.0) {
    re = l8_tmp_tmp / y_tmp;
    d_im = 0.0;
  } else if (l8_tmp_tmp == 0.0) {
    re = 0.0;
    d_im = l2 / y_tmp;
  } else {
    re = l8_tmp_tmp / y_tmp;
    d_im = l2 / y_tmp;
  }
  t6[3].re = re;
  t6[3].im = d_im;
  b_re = t3[3].re * t3[3].re - t3[3].im * t3[3].im;
  l14_im = t3[3].re * t3[3].im;
  c_im = l14_im + l14_im;
  c_y[3].re = b_J_min_tmp * b_re;
  c_y[3].im = b_J_min_tmp * c_im;
  t4_tmp[3].re = b_J_max * b_re;
  t4_tmp[3].im = b_J_max * c_im;
  d3 = re * d_im;
  coder::b_power(t3, l7);
  coder::b_power(t7, dcv);
  l8.re = (A_min * l8_tmp * -3.0 + P_init * l4 * 6.0) - P_wayp * l4 * 6.0;
  y_re = rt_powd_snf(A_init, 3.0) * 2.0;
  l14_re = A_init * J_min * V_init * 6.0;
  A_min_re = y_tmp_tmp * V_init * 6.0;
  y_tmp_tmp = A_min * J_min_tmp;
  J_min_tmp = J_min * l8_tmp;
  im_tmp = J_max * l8_tmp;
  l14_im = J_min * y_re_tmp;
  b_im_tmp = V_init * l4;
  l2 = A_min * l4;
  l8_tmp_tmp = l2 * t6[0].re;
  A_min_im = l2 * t6[0].im;
  l7[0].re =
      ((((((((((((((((((((((((((l8.re - y * l7[0].re) + y_re) + b_y) - l14_re) +
                            A_min_re) +
                           y_tmp_tmp * b_l4[0].re * 3.0) +
                          b_J_min_tmp * l7[0].re * 3.0) -
                         b_J_max * l7[0].re * 2.0) -
                        J_min_tmp * t3[0].re * 3.0) +
                       im_tmp * t3[0].re * 3.0) -
                      J_min_tmp * t6[0].re * 3.0) -
                     J_min_tmp * t7[0].re * 3.0) +
                    l14_im * t6[0].re * 3.0) +
                   l14_im * t7[0].re * 3.0) +
                  b_im_tmp * t3[0].re * 6.0) +
                 b_im_tmp * t6[0].re * 6.0) +
                b_im_tmp * t7[0].re * 6.0) +
               l2 * d_y[0].re * 3.0) +
              l2 * t6_tmp[0].re * 3.0) +
             b_J_max * dcv[0].re) +
            (l8_tmp_tmp * t7[0].re - A_min_im * t7[0].im) * 6.0) +
           (c_y[0].re * t6[0].re - c_y[0].im * t6[0].im) * 3.0) +
          (c_y[0].re * t7[0].re - c_y[0].im * t7[0].im) * 3.0) -
         (t4_tmp[0].re * t6[0].re - t4_tmp[0].im * t6[0].im) * 3.0) -
        (t4_tmp[0].re * t7[0].re - t4_tmp[0].im * t7[0].im) * 3.0) -
       y_tmp * b_l4[0].re * 3.0) -
      b_l8_tmp * t3[0].re * 6.0;
  l7[0].im = ((((((((((((((((((((((0.0 - y * l7[0].im) +
                                  y_tmp_tmp * b_l4[0].im * 3.0) +
                                 b_J_min_tmp * l7[0].im * 3.0) -
                                b_J_max * l7[0].im * 2.0) -
                               J_min_tmp * t3[0].im * 3.0) +
                              im_tmp * t3[0].im * 3.0) -
                             J_min_tmp * t6[0].im * 3.0) -
                            J_min_tmp * t7[0].im * 3.0) +
                           l14_im * t6[0].im * 3.0) +
                          l14_im * t7[0].im * 3.0) +
                         b_im_tmp * t3[0].im * 6.0) +
                        b_im_tmp * t6[0].im * 6.0) +
                       b_im_tmp * t7[0].im * 6.0) +
                      l2 * (d + d) * 3.0) +
                     l2 * t6_tmp[0].im * 3.0) +
                    b_J_max * dcv[0].im) +
                   (l8_tmp_tmp * t7[0].im + A_min_im * t7[0].re) * 6.0) +
                  (c_y[0].re * t6[0].im + c_y[0].im * t6[0].re) * 3.0) +
                 (c_y[0].re * t7[0].im + c_y[0].im * t7[0].re) * 3.0) -
                (t4_tmp[0].re * t6[0].im + t4_tmp[0].im * t6[0].re) * 3.0) -
               (t4_tmp[0].re * t7[0].im + t4_tmp[0].im * t7[0].re) * 3.0) -
              y_tmp * b_l4[0].im * 3.0) -
             b_l8_tmp * t3[0].im * 6.0;
  l8_tmp_tmp = l2 * t6[1].re;
  A_min_im = l2 * t6[1].im;
  l7[1].re =
      ((((((((((((((((((((((((((l8.re - y * l7[1].re) + y_re) + b_y) - l14_re) +
                            A_min_re) +
                           y_tmp_tmp * b_l4[1].re * 3.0) +
                          b_J_min_tmp * l7[1].re * 3.0) -
                         b_J_max * l7[1].re * 2.0) -
                        J_min_tmp * t3[1].re * 3.0) +
                       im_tmp * t3[1].re * 3.0) -
                      J_min_tmp * t6[1].re * 3.0) -
                     J_min_tmp * t7[1].re * 3.0) +
                    l14_im * t6[1].re * 3.0) +
                   l14_im * t7[1].re * 3.0) +
                  b_im_tmp * t3[1].re * 6.0) +
                 b_im_tmp * t6[1].re * 6.0) +
                b_im_tmp * t7[1].re * 6.0) +
               l2 * d_y[1].re * 3.0) +
              l2 * t6_tmp[1].re * 3.0) +
             b_J_max * dcv[1].re) +
            (l8_tmp_tmp * t7[1].re - A_min_im * t7[1].im) * 6.0) +
           (c_y[1].re * t6[1].re - c_y[1].im * t6[1].im) * 3.0) +
          (c_y[1].re * t7[1].re - c_y[1].im * t7[1].im) * 3.0) -
         (t4_tmp[1].re * t6[1].re - t4_tmp[1].im * t6[1].im) * 3.0) -
        (t4_tmp[1].re * t7[1].re - t4_tmp[1].im * t7[1].im) * 3.0) -
       y_tmp * b_l4[1].re * 3.0) -
      b_l8_tmp * t3[1].re * 6.0;
  l7[1].im = ((((((((((((((((((((((0.0 - y * l7[1].im) +
                                  y_tmp_tmp * b_l4[1].im * 3.0) +
                                 b_J_min_tmp * l7[1].im * 3.0) -
                                b_J_max * l7[1].im * 2.0) -
                               J_min_tmp * t3[1].im * 3.0) +
                              im_tmp * t3[1].im * 3.0) -
                             J_min_tmp * t6[1].im * 3.0) -
                            J_min_tmp * t7[1].im * 3.0) +
                           l14_im * t6[1].im * 3.0) +
                          l14_im * t7[1].im * 3.0) +
                         b_im_tmp * t3[1].im * 6.0) +
                        b_im_tmp * t6[1].im * 6.0) +
                       b_im_tmp * t7[1].im * 6.0) +
                      l2 * (d1 + d1) * 3.0) +
                     l2 * t6_tmp[1].im * 3.0) +
                    b_J_max * dcv[1].im) +
                   (l8_tmp_tmp * t7[1].im + A_min_im * t7[1].re) * 6.0) +
                  (c_y[1].re * t6[1].im + c_y[1].im * t6[1].re) * 3.0) +
                 (c_y[1].re * t7[1].im + c_y[1].im * t7[1].re) * 3.0) -
                (t4_tmp[1].re * t6[1].im + t4_tmp[1].im * t6[1].re) * 3.0) -
               (t4_tmp[1].re * t7[1].im + t4_tmp[1].im * t7[1].re) * 3.0) -
              y_tmp * b_l4[1].im * 3.0) -
             b_l8_tmp * t3[1].im * 6.0;
  l8_tmp_tmp = l2 * t6[2].re;
  A_min_im = l2 * t6[2].im;
  l7[2].re =
      ((((((((((((((((((((((((((l8.re - y * l7[2].re) + y_re) + b_y) - l14_re) +
                            A_min_re) +
                           y_tmp_tmp * b_l4[2].re * 3.0) +
                          b_J_min_tmp * l7[2].re * 3.0) -
                         b_J_max * l7[2].re * 2.0) -
                        J_min_tmp * t3[2].re * 3.0) +
                       im_tmp * t3[2].re * 3.0) -
                      J_min_tmp * t6[2].re * 3.0) -
                     J_min_tmp * t7[2].re * 3.0) +
                    l14_im * t6[2].re * 3.0) +
                   l14_im * t7[2].re * 3.0) +
                  b_im_tmp * t3[2].re * 6.0) +
                 b_im_tmp * t6[2].re * 6.0) +
                b_im_tmp * t7[2].re * 6.0) +
               l2 * d_y[2].re * 3.0) +
              l2 * t6_tmp[2].re * 3.0) +
             b_J_max * dcv[2].re) +
            (l8_tmp_tmp * t7[2].re - A_min_im * t7[2].im) * 6.0) +
           (c_y[2].re * t6[2].re - c_y[2].im * t6[2].im) * 3.0) +
          (c_y[2].re * t7[2].re - c_y[2].im * t7[2].im) * 3.0) -
         (t4_tmp[2].re * t6[2].re - t4_tmp[2].im * t6[2].im) * 3.0) -
        (t4_tmp[2].re * t7[2].re - t4_tmp[2].im * t7[2].im) * 3.0) -
       y_tmp * b_l4[2].re * 3.0) -
      b_l8_tmp * t3[2].re * 6.0;
  l7[2].im = ((((((((((((((((((((((0.0 - y * l7[2].im) +
                                  y_tmp_tmp * b_l4[2].im * 3.0) +
                                 b_J_min_tmp * l7[2].im * 3.0) -
                                b_J_max * l7[2].im * 2.0) -
                               J_min_tmp * t3[2].im * 3.0) +
                              im_tmp * t3[2].im * 3.0) -
                             J_min_tmp * t6[2].im * 3.0) -
                            J_min_tmp * t7[2].im * 3.0) +
                           l14_im * t6[2].im * 3.0) +
                          l14_im * t7[2].im * 3.0) +
                         b_im_tmp * t3[2].im * 6.0) +
                        b_im_tmp * t6[2].im * 6.0) +
                       b_im_tmp * t7[2].im * 6.0) +
                      l2 * (d2 + d2) * 3.0) +
                     l2 * t6_tmp[2].im * 3.0) +
                    b_J_max * dcv[2].im) +
                   (l8_tmp_tmp * t7[2].im + A_min_im * t7[2].re) * 6.0) +
                  (c_y[2].re * t6[2].im + c_y[2].im * t6[2].re) * 3.0) +
                 (c_y[2].re * t7[2].im + c_y[2].im * t7[2].re) * 3.0) -
                (t4_tmp[2].re * t6[2].im + t4_tmp[2].im * t6[2].re) * 3.0) -
               (t4_tmp[2].re * t7[2].im + t4_tmp[2].im * t7[2].re) * 3.0) -
              y_tmp * b_l4[2].im * 3.0) -
             b_l8_tmp * t3[2].im * 6.0;
  l8_tmp_tmp = l2 * re;
  A_min_im = l2 * d_im;
  l7[3].re =
      ((((((((((((((((((((((((((l8.re - y * l7[3].re) + y_re) + b_y) - l14_re) +
                            A_min_re) +
                           y_tmp_tmp * b_re * 3.0) +
                          b_J_min_tmp * l7[3].re * 3.0) -
                         b_J_max * l7[3].re * 2.0) -
                        J_min_tmp * t3[3].re * 3.0) +
                       im_tmp * t3[3].re * 3.0) -
                      J_min_tmp * re * 3.0) -
                     J_min_tmp * t7[3].re * 3.0) +
                    l14_im * re * 3.0) +
                   l14_im * t7[3].re * 3.0) +
                  b_im_tmp * t3[3].re * 6.0) +
                 b_im_tmp * re * 6.0) +
                b_im_tmp * t7[3].re * 6.0) +
               l2 * (re * re - d_im * d_im) * 3.0) +
              l2 * c_re * 3.0) +
             b_J_max * dcv[3].re) +
            (l8_tmp_tmp * t7[3].re - A_min_im * t7[3].im) * 6.0) +
           (c_y[3].re * re - c_y[3].im * d_im) * 3.0) +
          (c_y[3].re * t7[3].re - c_y[3].im * t7[3].im) * 3.0) -
         (t4_tmp[3].re * re - t4_tmp[3].im * d_im) * 3.0) -
        (t4_tmp[3].re * t7[3].re - t4_tmp[3].im * t7[3].im) * 3.0) -
       y_tmp * b_re * 3.0) -
      b_l8_tmp * t3[3].re * 6.0;
  l7[3].im =
      ((((((((((((((((((((((0.0 - y * l7[3].im) + y_tmp_tmp * c_im * 3.0) +
                          b_J_min_tmp * l7[3].im * 3.0) -
                         b_J_max * l7[3].im * 2.0) -
                        J_min_tmp * t3[3].im * 3.0) +
                       im_tmp * t3[3].im * 3.0) -
                      J_min_tmp * d_im * 3.0) -
                     J_min_tmp * t7[3].im * 3.0) +
                    l14_im * d_im * 3.0) +
                   l14_im * t7[3].im * 3.0) +
                  b_im_tmp * t3[3].im * 6.0) +
                 b_im_tmp * d_im * 6.0) +
                b_im_tmp * t7[3].im * 6.0) +
               l2 * (d3 + d3) * 3.0) +
              l2 * b_im * 3.0) +
             b_J_max * dcv[3].im) +
            (l8_tmp_tmp * t7[3].im + A_min_im * t7[3].re) * 6.0) +
           (c_y[3].re * d_im + c_y[3].im * re) * 3.0) +
          (c_y[3].re * t7[3].im + c_y[3].im * t7[3].re) * 3.0) -
         (t4_tmp[3].re * d_im + t4_tmp[3].im * re) * 3.0) -
        (t4_tmp[3].re * t7[3].im + t4_tmp[3].im * t7[3].re) * 3.0) -
       y_tmp * c_im * 3.0) -
      b_l8_tmp * t3[3].im * 6.0;
  l2 = A_min * (1.0 / J_min);
  l8.re = J_min_tmp * 3.0 - b_im_tmp * 6.0;
  t[16].re = l2;
  t[16].im = 0.0;
  t[17].re = l2;
  t[17].im = 0.0;
  t[18].re = l2;
  t[18].im = 0.0;
  t[19].re = l2;
  t[19].im = 0.0;
  l8_tmp_tmp = -(A_init + b_l2[0].re);
  if (-b_l2[0].im == 0.0) {
    t[0].re = l8_tmp_tmp / J_min;
    t[0].im = 0.0;
  } else if (l8_tmp_tmp == 0.0) {
    t[0].re = 0.0;
    t[0].im = -b_l2[0].im / J_min;
  } else {
    t[0].re = l8_tmp_tmp / J_min;
    t[0].im = -b_l2[0].im / J_min;
  }
  t[4].re = 0.0;
  t[4].im = 0.0;
  t[8] = t3[0];
  l14_im = (l8.re - c_y[0].re * 3.0) + t4_tmp[0].re * 3.0;
  im_tmp = (0.0 - c_y[0].im * 3.0) + t4_tmp[0].im * 3.0;
  if (im_tmp == 0.0) {
    if (l7[0].im == 0.0) {
      t[12].re = l7[0].re / l14_im;
      t[12].im = 0.0;
    } else if (l7[0].re == 0.0) {
      t[12].re = 0.0;
      t[12].im = l7[0].im / l14_im;
    } else {
      t[12].re = l7[0].re / l14_im;
      t[12].im = l7[0].im / l14_im;
    }
  } else if (l14_im == 0.0) {
    if (l7[0].re == 0.0) {
      t[12].re = l7[0].im / im_tmp;
      t[12].im = 0.0;
    } else if (l7[0].im == 0.0) {
      t[12].re = 0.0;
      t[12].im = -(l7[0].re / im_tmp);
    } else {
      t[12].re = l7[0].im / im_tmp;
      t[12].im = -(l7[0].re / im_tmp);
    }
  } else {
    b_im_tmp = std::abs(l14_im);
    l2 = std::abs(im_tmp);
    if (b_im_tmp > l2) {
      l8_tmp_tmp = im_tmp / l14_im;
      l2 = l14_im + l8_tmp_tmp * im_tmp;
      t[12].re = (l7[0].re + l8_tmp_tmp * l7[0].im) / l2;
      t[12].im = (l7[0].im - l8_tmp_tmp * l7[0].re) / l2;
    } else if (l2 == b_im_tmp) {
      if (l14_im > 0.0) {
        l8_tmp_tmp = 0.5;
      } else {
        l8_tmp_tmp = -0.5;
      }
      if (im_tmp > 0.0) {
        l2 = 0.5;
      } else {
        l2 = -0.5;
      }
      t[12].re = (l7[0].re * l8_tmp_tmp + l7[0].im * l2) / b_im_tmp;
      t[12].im = (l7[0].im * l8_tmp_tmp - l7[0].re * l2) / b_im_tmp;
    } else {
      l8_tmp_tmp = l14_im / im_tmp;
      l2 = im_tmp + l8_tmp_tmp * l14_im;
      t[12].re = (l8_tmp_tmp * l7[0].re + l7[0].im) / l2;
      t[12].im = (l8_tmp_tmp * l7[0].im - l7[0].re) / l2;
    }
  }
  t[20] = t6[0];
  t[24] = t7[0];
  l8_tmp_tmp = -(A_init + b_l2[1].re);
  if (-b_l2[1].im == 0.0) {
    t[1].re = l8_tmp_tmp / J_min;
    t[1].im = 0.0;
  } else if (l8_tmp_tmp == 0.0) {
    t[1].re = 0.0;
    t[1].im = -b_l2[1].im / J_min;
  } else {
    t[1].re = l8_tmp_tmp / J_min;
    t[1].im = -b_l2[1].im / J_min;
  }
  t[5].re = 0.0;
  t[5].im = 0.0;
  t[9] = t3[1];
  l14_im = (l8.re - c_y[1].re * 3.0) + t4_tmp[1].re * 3.0;
  im_tmp = (0.0 - c_y[1].im * 3.0) + t4_tmp[1].im * 3.0;
  if (im_tmp == 0.0) {
    if (l7[1].im == 0.0) {
      t[13].re = l7[1].re / l14_im;
      t[13].im = 0.0;
    } else if (l7[1].re == 0.0) {
      t[13].re = 0.0;
      t[13].im = l7[1].im / l14_im;
    } else {
      t[13].re = l7[1].re / l14_im;
      t[13].im = l7[1].im / l14_im;
    }
  } else if (l14_im == 0.0) {
    if (l7[1].re == 0.0) {
      t[13].re = l7[1].im / im_tmp;
      t[13].im = 0.0;
    } else if (l7[1].im == 0.0) {
      t[13].re = 0.0;
      t[13].im = -(l7[1].re / im_tmp);
    } else {
      t[13].re = l7[1].im / im_tmp;
      t[13].im = -(l7[1].re / im_tmp);
    }
  } else {
    b_im_tmp = std::abs(l14_im);
    l2 = std::abs(im_tmp);
    if (b_im_tmp > l2) {
      l8_tmp_tmp = im_tmp / l14_im;
      l2 = l14_im + l8_tmp_tmp * im_tmp;
      t[13].re = (l7[1].re + l8_tmp_tmp * l7[1].im) / l2;
      t[13].im = (l7[1].im - l8_tmp_tmp * l7[1].re) / l2;
    } else if (l2 == b_im_tmp) {
      if (l14_im > 0.0) {
        l8_tmp_tmp = 0.5;
      } else {
        l8_tmp_tmp = -0.5;
      }
      if (im_tmp > 0.0) {
        l2 = 0.5;
      } else {
        l2 = -0.5;
      }
      t[13].re = (l7[1].re * l8_tmp_tmp + l7[1].im * l2) / b_im_tmp;
      t[13].im = (l7[1].im * l8_tmp_tmp - l7[1].re * l2) / b_im_tmp;
    } else {
      l8_tmp_tmp = l14_im / im_tmp;
      l2 = im_tmp + l8_tmp_tmp * l14_im;
      t[13].re = (l8_tmp_tmp * l7[1].re + l7[1].im) / l2;
      t[13].im = (l8_tmp_tmp * l7[1].im - l7[1].re) / l2;
    }
  }
  t[21] = t6[1];
  t[25] = t7[1];
  l8_tmp_tmp = -(A_init + b_l2[2].re);
  if (-b_l2[2].im == 0.0) {
    t[2].re = l8_tmp_tmp / J_min;
    t[2].im = 0.0;
  } else if (l8_tmp_tmp == 0.0) {
    t[2].re = 0.0;
    t[2].im = -b_l2[2].im / J_min;
  } else {
    t[2].re = l8_tmp_tmp / J_min;
    t[2].im = -b_l2[2].im / J_min;
  }
  t[6].re = 0.0;
  t[6].im = 0.0;
  t[10] = t3[2];
  l14_im = (l8.re - c_y[2].re * 3.0) + t4_tmp[2].re * 3.0;
  im_tmp = (0.0 - c_y[2].im * 3.0) + t4_tmp[2].im * 3.0;
  if (im_tmp == 0.0) {
    if (l7[2].im == 0.0) {
      t[14].re = l7[2].re / l14_im;
      t[14].im = 0.0;
    } else if (l7[2].re == 0.0) {
      t[14].re = 0.0;
      t[14].im = l7[2].im / l14_im;
    } else {
      t[14].re = l7[2].re / l14_im;
      t[14].im = l7[2].im / l14_im;
    }
  } else if (l14_im == 0.0) {
    if (l7[2].re == 0.0) {
      t[14].re = l7[2].im / im_tmp;
      t[14].im = 0.0;
    } else if (l7[2].im == 0.0) {
      t[14].re = 0.0;
      t[14].im = -(l7[2].re / im_tmp);
    } else {
      t[14].re = l7[2].im / im_tmp;
      t[14].im = -(l7[2].re / im_tmp);
    }
  } else {
    b_im_tmp = std::abs(l14_im);
    l2 = std::abs(im_tmp);
    if (b_im_tmp > l2) {
      l8_tmp_tmp = im_tmp / l14_im;
      l2 = l14_im + l8_tmp_tmp * im_tmp;
      t[14].re = (l7[2].re + l8_tmp_tmp * l7[2].im) / l2;
      t[14].im = (l7[2].im - l8_tmp_tmp * l7[2].re) / l2;
    } else if (l2 == b_im_tmp) {
      if (l14_im > 0.0) {
        l8_tmp_tmp = 0.5;
      } else {
        l8_tmp_tmp = -0.5;
      }
      if (im_tmp > 0.0) {
        l2 = 0.5;
      } else {
        l2 = -0.5;
      }
      t[14].re = (l7[2].re * l8_tmp_tmp + l7[2].im * l2) / b_im_tmp;
      t[14].im = (l7[2].im * l8_tmp_tmp - l7[2].re * l2) / b_im_tmp;
    } else {
      l8_tmp_tmp = l14_im / im_tmp;
      l2 = im_tmp + l8_tmp_tmp * l14_im;
      t[14].re = (l8_tmp_tmp * l7[2].re + l7[2].im) / l2;
      t[14].im = (l8_tmp_tmp * l7[2].im - l7[2].re) / l2;
    }
  }
  t[22] = t6[2];
  t[26] = t7[2];
  if (-im == 0.0) {
    t[3].re = -re_tmp / J_min;
    t[3].im = 0.0;
  } else if (-re_tmp == 0.0) {
    t[3].re = 0.0;
    t[3].im = -im / J_min;
  } else {
    t[3].re = -re_tmp / J_min;
    t[3].im = -im / J_min;
  }
  t[7].re = 0.0;
  t[7].im = 0.0;
  t[11] = t3[3];
  l14_im = (l8.re - c_y[3].re * 3.0) + t4_tmp[3].re * 3.0;
  im_tmp = (0.0 - c_y[3].im * 3.0) + t4_tmp[3].im * 3.0;
  if (im_tmp == 0.0) {
    if (l7[3].im == 0.0) {
      t[15].re = l7[3].re / l14_im;
      t[15].im = 0.0;
    } else if (l7[3].re == 0.0) {
      t[15].re = 0.0;
      t[15].im = l7[3].im / l14_im;
    } else {
      t[15].re = l7[3].re / l14_im;
      t[15].im = l7[3].im / l14_im;
    }
  } else if (l14_im == 0.0) {
    if (l7[3].re == 0.0) {
      t[15].re = l7[3].im / im_tmp;
      t[15].im = 0.0;
    } else if (l7[3].im == 0.0) {
      t[15].re = 0.0;
      t[15].im = -(l7[3].re / im_tmp);
    } else {
      t[15].re = l7[3].im / im_tmp;
      t[15].im = -(l7[3].re / im_tmp);
    }
  } else {
    b_im_tmp = std::abs(l14_im);
    l2 = std::abs(im_tmp);
    if (b_im_tmp > l2) {
      l8_tmp_tmp = im_tmp / l14_im;
      l2 = l14_im + l8_tmp_tmp * im_tmp;
      t[15].re = (l7[3].re + l8_tmp_tmp * l7[3].im) / l2;
      t[15].im = (l7[3].im - l8_tmp_tmp * l7[3].re) / l2;
    } else if (l2 == b_im_tmp) {
      if (l14_im > 0.0) {
        l8_tmp_tmp = 0.5;
      } else {
        l8_tmp_tmp = -0.5;
      }
      if (im_tmp > 0.0) {
        l2 = 0.5;
      } else {
        l2 = -0.5;
      }
      t[15].re = (l7[3].re * l8_tmp_tmp + l7[3].im * l2) / b_im_tmp;
      t[15].im = (l7[3].im * l8_tmp_tmp - l7[3].re * l2) / b_im_tmp;
    } else {
      l8_tmp_tmp = l14_im / im_tmp;
      l2 = im_tmp + l8_tmp_tmp * l14_im;
      t[15].re = (l8_tmp_tmp * l7[3].re + l7[3].im) / l2;
      t[15].im = (l8_tmp_tmp * l7[3].im - l7[3].re) / l2;
    }
  }
  t[23] = t6[3];
  t[27] = t7[3];
}

// End of code generation (acdefg_NO_VP.cpp)
