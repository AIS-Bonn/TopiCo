//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// acdefg_T_AP.cpp
//
// Code generation for function 'acdefg_T_AP'
//

// Include files
#include "acdefg_T_AP.h"
#include "power.h"
#include "rt_nonfinite.h"
#include "sqrt.h"
#include "topico_wrapper_data.h"
#include "topico_wrapper_rtwutil.h"
#include "topico_wrapper_types.h"
#include <cmath>

// Function Definitions
void acdefg_T_AP(double P_init, double V_init, double A_init, double P_wayp,
                 double A_wayp, double V_max, double A_min, double J_max,
                 double J_min, double T, creal_T t[14])
{
  creal_T b_t4_tmp[2];
  creal_T l10[2];
  creal_T l9[2];
  creal_T t1[2];
  creal_T t3[2];
  creal_T t4_tmp[2];
  creal_T t6[2];
  creal_T l29;
  creal_T l30;
  double A_min_re;
  double A_wayp_re;
  double J_max_im;
  double J_max_re;
  double J_max_re_tmp_tmp;
  double J_max_tmp;
  double P_init_re;
  double P_wayp_re;
  double V_init_im;
  double V_init_re;
  double V_max_re;
  double a;
  double b_A_min;
  double b_A_wayp;
  double b_A_wayp_re;
  double b_J_max_im;
  double b_J_max_re;
  double b_l2;
  double b_l29_tmp;
  double b_re_tmp;
  double c_J_max_im;
  double c_J_max_re;
  double c_l29_tmp;
  double c_re_tmp;
  double d_l29_tmp;
  double l10_re_tmp;
  double l10_tmp;
  double l2;
  double l20_tmp;
  double l21_tmp;
  double l24;
  double l25;
  double l26;
  double l29_tmp;
  double l2_im;
  double l2_re;
  double l4_tmp;
  double l5_tmp;
  double l6;
  double l6_tmp;
  double l8_tmp;
  double l9_tmp;
  double re_tmp;
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
  //  Generated on 29-Aug-2019 12:12:49
  l2 = A_init * A_init;
  l4_tmp = A_min * A_min;
  l5_tmp = rt_powd_snf(A_min, 3.0);
  l8_tmp = J_min * J_min;
  l10_tmp = J_max * J_max;
  l9_tmp = l8_tmp * l8_tmp;
  a = J_max + -J_min;
  l20_tmp = A_min * A_wayp;
  l21_tmp = J_min * J_max;
  if (-J_min < 0.0) {
    k_rtErrorWithMessageID(o_emlrtRTEI.fName, o_emlrtRTEI.lineNo);
  }
  l24 = rt_powd_snf(-J_min, 4.5);
  if (-J_min < 0.0) {
    k_rtErrorWithMessageID(o_emlrtRTEI.fName, o_emlrtRTEI.lineNo);
  }
  l25 = rt_powd_snf(-J_min, 5.5);
  if (-J_min < 0.0) {
    k_rtErrorWithMessageID(o_emlrtRTEI.fName, o_emlrtRTEI.lineNo);
  }
  l26 = rt_powd_snf(a, 1.5);
  if (a < 0.0) {
    k_rtErrorWithMessageID(o_emlrtRTEI.fName, o_emlrtRTEI.lineNo);
  }
  l29_tmp = J_max * V_init;
  b_l29_tmp = J_max * V_max;
  l29.re = (l2 + b_l29_tmp * 2.0) + -(l29_tmp * 2.0);
  l29.im = 0.0;
  coder::internal::scalar::b_sqrt(&l29);
  l30 = coder::d_power(l29);
  a = l29_tmp * l25;
  J_max_re = a * l29.re;
  J_max_im = a * l29.im;
  l10_re_tmp = l10_tmp * rt_powd_snf(-J_min, 3.5);
  a = b_l29_tmp * l25;
  b_J_max_re = a * l29.re;
  b_J_max_im = a * l29.im;
  a = l2 * l25;
  l2_re = a * l29.re;
  l2_im = a * l29.im;
  a = V_init * l10_tmp * l24;
  V_init_re = a * l29.re;
  V_init_im = a * l29.im;
  J_max_re_tmp_tmp = J_max * l2;
  a = J_max_re_tmp_tmp * l24;
  c_J_max_re = a * l29.re;
  c_J_max_im = a * l29.im;
  a = V_max * l10_tmp * l24;
  V_max_re = a * l29.re;
  a *= l29.im;
  re_tmp = A_init * J_max;
  l24 = rt_powd_snf(A_init, 3.0);
  b_re_tmp = rt_powd_snf(A_wayp, 3.0);
  c_re_tmp = re_tmp * V_init;
  b_l29_tmp = A_min * (1.0 / l26);
  c_l29_tmp = l5_tmp * l8_tmp;
  d_l29_tmp = A_wayp * l4_tmp;
  l29.re = -(b_l29_tmp *
             ((((((((((((((((((b_re_tmp * l9_tmp * l26 * 4.0 +
                               l24 * l9_tmp * l26 * 8.0) +
                              -(l5_tmp * l9_tmp * l26 * 4.0)) +
                             re_tmp * V_max * l9_tmp * l26 * 24.0) +
                            c_l29_tmp * l10_tmp * l26) +
                           -(c_re_tmp * l9_tmp * l26 * 24.0)) +
                          d_l29_tmp * l9_tmp * l26 * 12.0) +
                         P_init * l9_tmp * l10_tmp * l26 * 24.0) +
                        T * V_max * l9_tmp * l10_tmp * l26 * 24.0) +
                       -(A_min * (A_wayp * A_wayp) * l9_tmp * l26 * 12.0)) +
                      -(P_wayp * l9_tmp * l10_tmp * l26 * 24.0)) +
                     l25 * l30.re * 4.0) +
                    J_max_re * 24.0) +
                   -(l10_re_tmp * l30.re * 4.0)) +
                  -(b_J_max_re * 24.0)) +
                 -(l2_re * 12.0)) +
                V_init_re * 24.0) +
               -(c_J_max_re * 12.0)) +
              -(V_max_re * 24.0)));
  l29.im = -(b_l29_tmp * (((((((l25 * l30.im * 4.0 + J_max_im * 24.0) +
                               -(l10_re_tmp * l30.im * 4.0)) +
                              -(b_J_max_im * 24.0)) +
                             -(l2_im * 12.0)) +
                            V_init_im * 24.0) +
                           -(c_J_max_im * 12.0)) +
                          -(a * 24.0)));
  coder::internal::scalar::b_sqrt(&l29);
  l29.re *= 1.7320508075688772;
  l29.im *= 1.7320508075688772;
  a = (l20_tmp * l8_tmp * 6.0 + l21_tmp * l4_tmp * 3.0) - l4_tmp * l8_tmp * 6.0;
  V_max_re = 1.0 / A_min * (1.0 / l8_tmp) * (1.0 / J_max);
  t6[0].re = -0.16666666666666666 * (V_max_re * (a + l29.re));
  t6[0].im = -0.16666666666666666 * (V_max_re * l29.im);
  t6[1].re = -0.16666666666666666 * (V_max_re * (a - l29.re));
  t6[1].im = -0.16666666666666666 * (V_max_re * (0.0 - l29.im));
  l29.re = J_min * (J_min + -J_max) *
           ((A_init * A_init + J_max * V_max * 2.0) + -(J_max * V_init * 2.0));
  l29.im = 0.0;
  coder::internal::scalar::b_sqrt(&l29);
  b_l29_tmp = 1.0 / (l8_tmp + -l21_tmp);
  l29.re *= b_l29_tmp;
  l29.im *= b_l29_tmp;
  t3[0] = l29;
  t3[1] = l29;
  l6 = rt_powd_snf(J_min, 3.0);
  coder::power(t3, l10);
  y = rt_powd_snf(J_min, 5.0);
  l29.re = ((c_l29_tmp * 2.0 + l5_tmp * l10_tmp) + l24 * l8_tmp * 2.0) +
           b_re_tmp * l8_tmp;
  l30.re = l21_tmp * l5_tmp * 3.0;
  A_min_re = A_min * l2 * l8_tmp * 3.0;
  b_A_min = A_min * l9_tmp;
  A_wayp_re = A_wayp * l2 * l8_tmp * 3.0;
  b_A_wayp_re = d_l29_tmp * l8_tmp * 3.0;
  b_A_wayp = A_wayp * l9_tmp;
  J_max_tmp = J_max * l9_tmp;
  P_init_re = P_init * l8_tmp * l10_tmp * 6.0;
  P_wayp_re = P_wayp * l8_tmp * l10_tmp * 6.0;
  l6_tmp = l6 * l10_tmp;
  b_l2 = l2 * l6;
  l26 = J_max_re_tmp_tmp * l8_tmp;
  d_l29_tmp = J_max * l4_tmp * l8_tmp;
  l5_tmp = J_min * l4_tmp * l10_tmp;
  l21_tmp = V_init * l8_tmp * l10_tmp;
  l9_tmp = A_min * l8_tmp * l10_tmp;
  a = A_min * J_min;
  l2_re = a * J_max * l2 * 3.0;
  c_l29_tmp = A_wayp * J_min * J_max * l4_tmp * 3.0;
  J_max_re_tmp_tmp = c_re_tmp * l8_tmp * 6.0;
  b_l29_tmp = A_min * J_max;
  l2_im = b_l29_tmp * V_init * l8_tmp * 6.0;
  V_init_im = a * V_init * l10_tmp * 6.0;
  a = A_wayp * J_max;
  c_J_max_re = a * V_init * l8_tmp * 6.0;
  c_J_max_im = b_l29_tmp * l6;
  b_l29_tmp = a * l6;
  V_max_re = l29_tmp * l6;
  l24 = l20_tmp * J_max * l8_tmp;
  l25 = t3[0].re * t3[0].re - t3[0].im * t3[0].im;
  a = t3[0].re * t3[0].im;
  J_max_re = a + a;
  J_max_im = J_max_tmp * l25;
  l10_re_tmp = J_max_tmp * J_max_re;
  t4_tmp[0].re = J_max_im;
  t4_tmp[0].im = l10_re_tmp;
  b_J_max_re = l6_tmp * l25;
  b_J_max_im = l6_tmp * J_max_re;
  b_t4_tmp[0].re = b_J_max_re;
  b_t4_tmp[0].im = b_J_max_im;
  a = t6[0].re * t6[0].im;
  l10[0].re =
      (((((((((((((((((((((((((((((((l29.re - y * l10[0].re) - l30.re) +
                                   A_min_re) -
                                  b_A_min * l25 * 3.0) -
                                 A_wayp_re) -
                                b_A_wayp_re) +
                               b_A_wayp * l25 * 3.0) +
                              J_max_tmp * l10[0].re * 3.0) +
                             P_init_re) -
                            P_wayp_re) -
                           l6_tmp * l10[0].re * 2.0) +
                          b_l2 * t3[0].re * 3.0) -
                         l26 * t3[0].re * 3.0) -
                        l26 * t6[0].re * 3.0) -
                       d_l29_tmp * t6[0].re * 6.0) +
                      l5_tmp * t6[0].re * 3.0) +
                     (J_max_im * t6[0].re - l10_re_tmp * t6[0].im) * 3.0) +
                    l21_tmp * t3[0].re * 6.0) +
                   l21_tmp * t6[0].re * 6.0) -
                  (b_J_max_re * t6[0].re - b_J_max_im * t6[0].im) * 3.0) +
                 l9_tmp * (t6[0].re * t6[0].re - t6[0].im * t6[0].im) * 3.0) -
                l2_re) +
               c_l29_tmp) -
              J_max_re_tmp_tmp) -
             l2_im) +
            V_init_im) +
           c_J_max_re) +
          c_J_max_im * l25 * 6.0) -
         b_l29_tmp * l25 * 3.0) -
        V_max_re * t3[0].re * 6.0) -
       l9_tmp * l25 * 3.0) +
      l24 * t6[0].re * 6.0;
  l10[0].im =
      (((((((((((((((((((0.0 - y * l10[0].im) - b_A_min * J_max_re * 3.0) +
                       b_A_wayp * J_max_re * 3.0) +
                      J_max_tmp * l10[0].im * 3.0) -
                     l6_tmp * l10[0].im * 2.0) +
                    b_l2 * t3[0].im * 3.0) -
                   l26 * t3[0].im * 3.0) -
                  l26 * t6[0].im * 3.0) -
                 d_l29_tmp * t6[0].im * 6.0) +
                l5_tmp * t6[0].im * 3.0) +
               (J_max_im * t6[0].im + l10_re_tmp * t6[0].re) * 3.0) +
              l21_tmp * t3[0].im * 6.0) +
             l21_tmp * t6[0].im * 6.0) -
            (b_J_max_re * t6[0].im + b_J_max_im * t6[0].re) * 3.0) +
           l9_tmp * (a + a) * 3.0) +
          c_J_max_im * J_max_re * 6.0) -
         b_l29_tmp * J_max_re * 3.0) -
        V_max_re * t3[0].im * 6.0) -
       l9_tmp * J_max_re * 3.0) +
      l24 * t6[0].im * 6.0;
  l25 = J_min * t3[0].re;
  J_max_re = J_min * t3[0].im;
  l9[0].re = l25;
  l9[0].im = J_max_re;
  re_tmp = -(A_init + l25);
  if (-J_max_re == 0.0) {
    V_init_re = re_tmp / J_max;
    b_re_tmp = 0.0;
  } else if (re_tmp == 0.0) {
    V_init_re = 0.0;
    b_re_tmp = -J_max_re / J_max;
  } else {
    V_init_re = re_tmp / J_max;
    b_re_tmp = -J_max_re / J_max;
  }
  t1[0].re = V_init_re;
  t1[0].im = b_re_tmp;
  t[0].re = V_init_re;
  t[0].im = b_re_tmp;
  t[2].re = 0.0;
  t[2].im = 0.0;
  t[4] = t3[0];
  l25 = t3[1].re * t3[1].re - t3[1].im * t3[1].im;
  a = t3[1].re * t3[1].im;
  J_max_re = a + a;
  J_max_im = J_max_tmp * l25;
  l10_re_tmp = J_max_tmp * J_max_re;
  b_J_max_re = l6_tmp * l25;
  b_J_max_im = l6_tmp * J_max_re;
  a = t6[1].re * t6[1].im;
  l10[1].re =
      (((((((((((((((((((((((((((((((l29.re - y * l10[1].re) - l30.re) +
                                   A_min_re) -
                                  b_A_min * l25 * 3.0) -
                                 A_wayp_re) -
                                b_A_wayp_re) +
                               b_A_wayp * l25 * 3.0) +
                              J_max_tmp * l10[1].re * 3.0) +
                             P_init_re) -
                            P_wayp_re) -
                           l6_tmp * l10[1].re * 2.0) +
                          b_l2 * t3[1].re * 3.0) -
                         l26 * t3[1].re * 3.0) -
                        l26 * t6[1].re * 3.0) -
                       d_l29_tmp * t6[1].re * 6.0) +
                      l5_tmp * t6[1].re * 3.0) +
                     (J_max_im * t6[1].re - l10_re_tmp * t6[1].im) * 3.0) +
                    l21_tmp * t3[1].re * 6.0) +
                   l21_tmp * t6[1].re * 6.0) -
                  (b_J_max_re * t6[1].re - b_J_max_im * t6[1].im) * 3.0) +
                 l9_tmp * (t6[1].re * t6[1].re - t6[1].im * t6[1].im) * 3.0) -
                l2_re) +
               c_l29_tmp) -
              J_max_re_tmp_tmp) -
             l2_im) +
            V_init_im) +
           c_J_max_re) +
          c_J_max_im * l25 * 6.0) -
         b_l29_tmp * l25 * 3.0) -
        V_max_re * t3[1].re * 6.0) -
       l9_tmp * l25 * 3.0) +
      l24 * t6[1].re * 6.0;
  l10[1].im =
      (((((((((((((((((((0.0 - y * l10[1].im) - b_A_min * J_max_re * 3.0) +
                       b_A_wayp * J_max_re * 3.0) +
                      J_max_tmp * l10[1].im * 3.0) -
                     l6_tmp * l10[1].im * 2.0) +
                    b_l2 * t3[1].im * 3.0) -
                   l26 * t3[1].im * 3.0) -
                  l26 * t6[1].im * 3.0) -
                 d_l29_tmp * t6[1].im * 6.0) +
                l5_tmp * t6[1].im * 3.0) +
               (J_max_im * t6[1].im + l10_re_tmp * t6[1].re) * 3.0) +
              l21_tmp * t3[1].im * 6.0) +
             l21_tmp * t6[1].im * 6.0) -
            (b_J_max_re * t6[1].im + b_J_max_im * t6[1].re) * 3.0) +
           l9_tmp * (a + a) * 3.0) +
          c_J_max_im * J_max_re * 6.0) -
         b_l29_tmp * J_max_re * 3.0) -
        V_max_re * t3[1].im * 6.0) -
       l9_tmp * J_max_re * 3.0) +
      l24 * t6[1].im * 6.0;
  l25 = J_min * t3[1].re;
  J_max_re = J_min * t3[1].im;
  re_tmp = -(A_init + l25);
  if (-J_max_re == 0.0) {
    V_init_re = re_tmp / J_max;
    b_re_tmp = 0.0;
  } else if (re_tmp == 0.0) {
    V_init_re = 0.0;
    b_re_tmp = -J_max_re / J_max;
  } else {
    V_init_re = re_tmp / J_max;
    b_re_tmp = -J_max_re / J_max;
  }
  t[1].re = V_init_re;
  t[1].im = b_re_tmp;
  t[3].re = 0.0;
  t[3].im = 0.0;
  t[5] = t3[1];
  l6 = -(1.0 / J_max * (A_min + -A_wayp));
  l29.re = l26 * 3.0;
  l30.re = l21_tmp * 6.0;
  J_max_re_tmp_tmp = A_init - A_min;
  l24 = ((l29.re - t4_tmp[0].re * 3.0) - l30.re) + b_t4_tmp[0].re * 3.0;
  b_l29_tmp = (0.0 - t4_tmp[0].im * 3.0) + b_t4_tmp[0].im * 3.0;
  if (b_l29_tmp == 0.0) {
    if (l10[0].im == 0.0) {
      t[6].re = l10[0].re / l24;
      t[6].im = 0.0;
    } else if (l10[0].re == 0.0) {
      t[6].re = 0.0;
      t[6].im = l10[0].im / l24;
    } else {
      t[6].re = l10[0].re / l24;
      t[6].im = l10[0].im / l24;
    }
  } else if (l24 == 0.0) {
    if (l10[0].re == 0.0) {
      t[6].re = l10[0].im / b_l29_tmp;
      t[6].im = 0.0;
    } else if (l10[0].im == 0.0) {
      t[6].re = 0.0;
      t[6].im = -(l10[0].re / b_l29_tmp);
    } else {
      t[6].re = l10[0].im / b_l29_tmp;
      t[6].im = -(l10[0].re / b_l29_tmp);
    }
  } else {
    re_tmp = std::abs(l24);
    a = std::abs(b_l29_tmp);
    if (re_tmp > a) {
      V_max_re = b_l29_tmp / l24;
      a = l24 + V_max_re * b_l29_tmp;
      t[6].re = (l10[0].re + V_max_re * l10[0].im) / a;
      t[6].im = (l10[0].im - V_max_re * l10[0].re) / a;
    } else if (a == re_tmp) {
      if (l24 > 0.0) {
        V_max_re = 0.5;
      } else {
        V_max_re = -0.5;
      }
      if (b_l29_tmp > 0.0) {
        a = 0.5;
      } else {
        a = -0.5;
      }
      t[6].re = (l10[0].re * V_max_re + l10[0].im * a) / re_tmp;
      t[6].im = (l10[0].im * V_max_re - l10[0].re * a) / re_tmp;
    } else {
      V_max_re = l24 / b_l29_tmp;
      a = b_l29_tmp + V_max_re * l24;
      t[6].re = (V_max_re * l10[0].re + l10[0].im) / a;
      t[6].im = (V_max_re * l10[0].im - l10[0].re) / a;
    }
  }
  re_tmp = -((J_max_re_tmp_tmp + l9[0].re) + J_max * t1[0].re);
  a = -(l9[0].im + J_max * t1[0].im);
  if (a == 0.0) {
    t[8].re = re_tmp / J_min;
    t[8].im = 0.0;
  } else if (re_tmp == 0.0) {
    t[8].re = 0.0;
    t[8].im = a / J_min;
  } else {
    t[8].re = re_tmp / J_min;
    t[8].im = a / J_min;
  }
  t[10] = t6[0];
  l24 = ((l29.re - J_max_im * 3.0) - l30.re) + b_J_max_re * 3.0;
  b_l29_tmp = (0.0 - l10_re_tmp * 3.0) + b_J_max_im * 3.0;
  if (b_l29_tmp == 0.0) {
    if (l10[1].im == 0.0) {
      t[7].re = l10[1].re / l24;
      t[7].im = 0.0;
    } else if (l10[1].re == 0.0) {
      t[7].re = 0.0;
      t[7].im = l10[1].im / l24;
    } else {
      t[7].re = l10[1].re / l24;
      t[7].im = l10[1].im / l24;
    }
  } else if (l24 == 0.0) {
    if (l10[1].re == 0.0) {
      t[7].re = l10[1].im / b_l29_tmp;
      t[7].im = 0.0;
    } else if (l10[1].im == 0.0) {
      t[7].re = 0.0;
      t[7].im = -(l10[1].re / b_l29_tmp);
    } else {
      t[7].re = l10[1].im / b_l29_tmp;
      t[7].im = -(l10[1].re / b_l29_tmp);
    }
  } else {
    re_tmp = std::abs(l24);
    a = std::abs(b_l29_tmp);
    if (re_tmp > a) {
      V_max_re = b_l29_tmp / l24;
      a = l24 + V_max_re * b_l29_tmp;
      t[7].re = (l10[1].re + V_max_re * l10[1].im) / a;
      t[7].im = (l10[1].im - V_max_re * l10[1].re) / a;
    } else if (a == re_tmp) {
      if (l24 > 0.0) {
        V_max_re = 0.5;
      } else {
        V_max_re = -0.5;
      }
      if (b_l29_tmp > 0.0) {
        a = 0.5;
      } else {
        a = -0.5;
      }
      t[7].re = (l10[1].re * V_max_re + l10[1].im * a) / re_tmp;
      t[7].im = (l10[1].im * V_max_re - l10[1].re * a) / re_tmp;
    } else {
      V_max_re = l24 / b_l29_tmp;
      a = b_l29_tmp + V_max_re * l24;
      t[7].re = (V_max_re * l10[1].re + l10[1].im) / a;
      t[7].im = (V_max_re * l10[1].im - l10[1].re) / a;
    }
  }
  re_tmp = -((J_max_re_tmp_tmp + l25) + J_max * V_init_re);
  a = -(J_max_re + J_max * b_re_tmp);
  if (a == 0.0) {
    t[9].re = re_tmp / J_min;
    t[9].im = 0.0;
  } else if (re_tmp == 0.0) {
    t[9].re = 0.0;
    t[9].im = a / J_min;
  } else {
    t[9].re = re_tmp / J_min;
    t[9].im = a / J_min;
  }
  t[11] = t6[1];
  t[12].re = l6;
  t[12].im = 0.0;
  t[13].re = l6;
  t[13].im = 0.0;
}

// End of code generation (acdefg_T_AP.cpp)
