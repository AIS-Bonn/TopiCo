//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// acdeg_T_AP.cpp
//
// Code generation for function 'acdeg_T_AP'
//

// Include files
#include "acdeg_T_AP.h"
#include "power.h"
#include "rt_nonfinite.h"
#include "sqrt.h"
#include "topico_wrapper_data.h"
#include "topico_wrapper_rtwutil.h"
#include "topico_wrapper_types.h"
#include <cmath>

// Function Definitions
void acdeg_T_AP(double P_init, double V_init, double A_init, double P_wayp,
                double A_wayp, double V_max, double J_max, double J_min,
                double T, creal_T t[21])
{
  creal_T l11[3];
  creal_T l13[3];
  creal_T t3[3];
  creal_T t7[3];
  creal_T l28;
  creal_T l29;
  double A_init_re_tmp;
  double J_max_re_tmp;
  double J_max_re_tmp_tmp;
  double J_max_tmp;
  double V_init_re_tmp;
  double V_max_re_tmp;
  double a;
  double ai;
  double ar;
  double b_A_init_re_tmp;
  double b_A_wayp;
  double b_J_max_re_tmp;
  double b_V_init_re_tmp;
  double b_V_max_re_tmp;
  double b_l2;
  double b_l2_re_tmp;
  double b_l4_tmp;
  double b_l88_tmp;
  double c_A_init_re_tmp;
  double c_J_max_re_tmp;
  double d_A_init_re_tmp;
  double l10_tmp;
  double l112_re;
  double l115_im;
  double l115_re;
  double l11_tmp;
  double l12_tmp;
  double l2;
  double l21;
  double l22;
  double l23;
  double l24;
  double l28_tmp;
  double l2_re_tmp;
  double l3_tmp;
  double l4_tmp;
  double l5_tmp;
  double l6_tmp;
  double l7_tmp;
  double l83;
  double l83_tmp;
  double l85;
  double l86;
  double l88;
  double l88_tmp;
  double l93;
  double l9_tmp;
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
  //  Generated on 29-Aug-2019 11:55:45
  l2 = A_init * A_init;
  l3_tmp = rt_powd_snf(A_init, 3.0);
  l4_tmp = A_wayp * A_wayp;
  l5_tmp = rt_powd_snf(A_wayp, 3.0);
  l6_tmp = J_min * J_min;
  l7_tmp = rt_powd_snf(J_min, 3.0);
  l9_tmp = J_max * J_max;
  l10_tmp = rt_powd_snf(J_max, 3.0);
  l12_tmp = rt_powd_snf(J_max, 5.0);
  l112_re = l6_tmp * l6_tmp;
  l11_tmp = l9_tmp * l9_tmp;
  a = J_max + -J_min;
  if (-J_min < 0.0) {
    k_rtErrorWithMessageID(o_emlrtRTEI.fName, o_emlrtRTEI.lineNo);
  }
  l21 = rt_powd_snf(-J_min, 3.5);
  if (-J_min < 0.0) {
    k_rtErrorWithMessageID(o_emlrtRTEI.fName, o_emlrtRTEI.lineNo);
  }
  l22 = rt_powd_snf(-J_min, 4.5);
  if (-J_min < 0.0) {
    k_rtErrorWithMessageID(o_emlrtRTEI.fName, o_emlrtRTEI.lineNo);
  }
  l23 = rt_powd_snf(-J_min, 5.5);
  if (-J_min < 0.0) {
    k_rtErrorWithMessageID(o_emlrtRTEI.fName, o_emlrtRTEI.lineNo);
  }
  l24 = rt_powd_snf(a, 1.5);
  if (a < 0.0) {
    k_rtErrorWithMessageID(o_emlrtRTEI.fName, o_emlrtRTEI.lineNo);
  }
  a = J_max * V_max;
  l28_tmp = J_max * V_init;
  l28.re = (l2 + a * 2.0) + -(l28_tmp * 2.0);
  l28.im = 0.0;
  coder::internal::scalar::b_sqrt(&l28);
  l29 = coder::d_power(l28);
  l83_tmp = J_min * l4_tmp;
  l83 = (l83_tmp * l11_tmp * l24 * 3.0 + l4_tmp * l7_tmp * l9_tmp * l24 * 3.0) +
        -(l4_tmp * l6_tmp * l10_tmp * l24 * 6.0);
  l88_tmp = A_wayp * J_min;
  b_l88_tmp = A_wayp * l6_tmp;
  l88 = ((l88_tmp * l12_tmp * l24 * 3.0 +
          -(A_wayp * l112_re * l9_tmp * l24 * 3.0)) +
         A_wayp * l7_tmp * l10_tmp * l24 * 9.0) +
        -(b_l88_tmp * l11_tmp * l24 * 9.0);
  l85 = 1.0 / (((J_min * rt_powd_snf(l9_tmp, 3.0) * l24 +
                 l7_tmp * l11_tmp * l24 * 5.0) +
                -(l112_re * l10_tmp * l24 * 2.0)) +
               -(l6_tmp * l12_tmp * l24 * 4.0));
  l86 = l85 * l85;
  l93 = l85 * l88 / 3.0;
  J_max_re_tmp = a * l23;
  b_J_max_re_tmp = l28_tmp * l23;
  l2_re_tmp = l2 * l23;
  V_max_re_tmp = V_max * l10_tmp * l21;
  J_max_re_tmp_tmp = J_max * l2;
  c_J_max_re_tmp = J_max_re_tmp_tmp * l22;
  V_init_re_tmp = V_init * l10_tmp * l21;
  b_l2_re_tmp = l2 * l9_tmp * l21;
  b_V_max_re_tmp = V_max * l9_tmp * l22;
  b_V_init_re_tmp = V_init * l9_tmp * l22;
  A_init_re_tmp = A_init * J_max;
  l115_im = T * V_max;
  b_A_init_re_tmp = l10_tmp * rt_powd_snf(-J_min, 2.5);
  c_A_init_re_tmp = l9_tmp * l21;
  a = J_max * l22;
  d_A_init_re_tmp = A_init_re_tmp * V_init;
  ar = l85 *
       ((((((((((((((((((((((((((d_A_init_re_tmp * l112_re * l24 * 6.0 +
                                 J_min * l5_tmp * l10_tmp * l24) +
                                -(l3_tmp * l112_re * l24 * 2.0)) +
                               -(A_init_re_tmp * V_max * l112_re * l24 * 6.0)) +
                              J_max * l3_tmp * l7_tmp * l24 * 2.0) +
                             P_init * l7_tmp * l10_tmp * l24 * 6.0) +
                            P_wayp * l112_re * l9_tmp * l24 * 6.0) +
                           l115_im * l7_tmp * l10_tmp * l24 * 6.0) +
                          -(P_init * l112_re * l9_tmp * l24 * 6.0)) +
                         -(P_wayp * l7_tmp * l10_tmp * l24 * 6.0)) +
                        A_init * V_max * l7_tmp * l9_tmp * l24 * 6.0) +
                       -(l115_im * l112_re * l9_tmp * l24 * 6.0)) +
                      -(A_init * V_init * l7_tmp * l9_tmp * l24 * 6.0)) +
                     -(l5_tmp * l6_tmp * l9_tmp * l24)) +
                    b_A_init_re_tmp * l29.re) +
                   c_A_init_re_tmp * l29.re) +
                  -(l23 * l29.re)) +
                 -(a * l29.re)) +
                J_max_re_tmp * l28.re * 6.0) +
               -(b_J_max_re_tmp * l28.re * 6.0)) +
              l2_re_tmp * l28.re * 3.0) +
             V_max_re_tmp * l28.re * 6.0) +
            c_J_max_re_tmp * l28.re * 6.0) +
           -(V_init_re_tmp * l28.re * 6.0)) +
          b_l2_re_tmp * l28.re * 3.0) +
         b_V_max_re_tmp * l28.re * 12.0) +
        -(b_V_init_re_tmp * l28.re * 12.0));
  ai = l85 * ((((((((((((b_A_init_re_tmp * l29.im + c_A_init_re_tmp * l29.im) +
                        -(l23 * l29.im)) +
                       -(a * l29.im)) +
                      J_max_re_tmp * l28.im * 6.0) +
                     -(b_J_max_re_tmp * l28.im * 6.0)) +
                    l2_re_tmp * l28.im * 3.0) +
                   V_max_re_tmp * l28.im * 6.0) +
                  c_J_max_re_tmp * l28.im * 6.0) +
                 -(V_init_re_tmp * l28.im * 6.0)) +
                b_l2_re_tmp * l28.im * 3.0) +
               b_V_max_re_tmp * l28.im * 12.0) +
              -(b_V_init_re_tmp * l28.im * 12.0));
  if (ai == 0.0) {
    a = ar / 2.0;
    l21 = 0.0;
  } else if (ar == 0.0) {
    a = 0.0;
    l21 = ai / 2.0;
  } else {
    a = ar / 2.0;
    l21 = ai / 2.0;
  }
  l28.re = (rt_powd_snf(l85, 3.0) * rt_powd_snf(l88, 3.0) / 27.0 +
            -(l83 * l86 * l88 / 6.0)) +
           a;
  a = l28.re * l21;
  c_A_init_re_tmp = l83 * l85 / 3.0 - l86 * (l88 * l88) / 9.0;
  l29.re = rt_powd_snf(c_A_init_re_tmp, 3.0) + (l28.re * l28.re - l21 * l21);
  l29.im = a + a;
  coder::internal::scalar::b_sqrt(&l29);
  l28.re += l29.re;
  l28.im = l21 + l29.im;
  l28 = coder::power(l28);
  if (l28.im == 0.0) {
    l29.re = 1.0 / l28.re;
    l29.im = 0.0;
    l22 = l28.re / 2.0;
    A_init_re_tmp = 0.0;
  } else if (l28.re == 0.0) {
    l29.re = 0.0;
    l29.im = -(1.0 / l28.im);
    l22 = 0.0;
    A_init_re_tmp = l28.im / 2.0;
  } else {
    l22 = std::abs(l28.re);
    l115_im = std::abs(l28.im);
    if (l22 > l115_im) {
      l115_im = l28.im / l28.re;
      a = l28.re + l115_im * l28.im;
      l29.re = (l115_im * 0.0 + 1.0) / a;
      l29.im = (0.0 - l115_im) / a;
    } else if (l115_im == l22) {
      if (l28.re > 0.0) {
        l115_im = 0.5;
      } else {
        l115_im = -0.5;
      }
      if (l28.im > 0.0) {
        a = 0.5;
      } else {
        a = -0.5;
      }
      l29.re = (l115_im + 0.0 * a) / l22;
      l29.im = (0.0 * l115_im - a) / l22;
    } else {
      l115_im = l28.re / l28.im;
      a = l28.im + l115_im * l28.re;
      l29.re = l115_im / a;
      l29.im = (l115_im * 0.0 - 1.0) / a;
    }
    l22 = l28.re / 2.0;
    A_init_re_tmp = l28.im / 2.0;
  }
  a = l29.re * c_A_init_re_tmp;
  l21 = l29.im * c_A_init_re_tmp;
  if (l21 == 0.0) {
    l115_re = a / 2.0;
    l115_im = 0.0;
  } else if (a == 0.0) {
    l115_re = 0.0;
    l115_im = l21 / 2.0;
  } else {
    l115_re = a / 2.0;
    l115_im = l21 / 2.0;
  }
  t7[0].re = (l93 + l28.re) + -l29.re * c_A_init_re_tmp;
  t7[0].im = l28.im + -l29.im * c_A_init_re_tmp;
  c_A_init_re_tmp = 1.7320508075688772 * (l28.re + a);
  l2_re_tmp = 1.7320508075688772 * (l28.im + l21);
  l21 = (l93 + -l22) + l115_re;
  t7[1].re = l21 + (c_A_init_re_tmp * 0.0 - l2_re_tmp * 0.5);
  a = -A_init_re_tmp + l115_im;
  t7[1].im = a + (c_A_init_re_tmp * 0.5 + l2_re_tmp * 0.0);
  t7[2].re = l21 + (c_A_init_re_tmp * -0.0 - l2_re_tmp * -0.5);
  t7[2].im = a + (c_A_init_re_tmp * -0.5 + l2_re_tmp * -0.0);
  l29.re = J_min * (J_min + -J_max) *
           ((A_init * A_init + J_max * V_max * 2.0) + -(J_max * V_init * 2.0));
  l29.im = 0.0;
  coder::internal::scalar::b_sqrt(&l29);
  a = 1.0 / (l6_tmp + -(J_min * J_max));
  l28.re = a * l29.re;
  l28.im = a * l29.im;
  t3[0] = l28;
  t3[1] = l28;
  t3[2] = l28;
  coder::c_power(t3, l11);
  coder::c_power(t7, l13);
  y = rt_powd_snf(J_min, 5.0);
  l28.re = l3_tmp * l6_tmp * 2.0 + l5_tmp * l9_tmp;
  b_A_wayp = A_wayp * l11_tmp;
  J_max_tmp = J_max * l112_re;
  l11_tmp *= J_min;
  l29.re = P_init * l6_tmp * l9_tmp * 6.0;
  l112_re = P_wayp * l6_tmp * l9_tmp * 6.0;
  l5_tmp = l7_tmp * l9_tmp;
  b_l4_tmp = l6_tmp * l10_tmp;
  b_l2 = l2 * l7_tmp;
  l3_tmp = l4_tmp * l10_tmp;
  l86 = J_max_re_tmp_tmp * l6_tmp;
  l93 = J_min * l2 * l9_tmp;
  l88 = l83_tmp * l9_tmp;
  l85 = V_init * l6_tmp * l9_tmp;
  l115_re = l88_tmp * J_max * l2 * 3.0;
  l24 = d_A_init_re_tmp * l6_tmp * 6.0;
  l23 = l88_tmp * V_init * l9_tmp * 6.0;
  l83 = A_wayp * J_max * l7_tmp;
  b_A_init_re_tmp = l88_tmp * l10_tmp;
  b_V_init_re_tmp = l28_tmp * l7_tmp;
  b_V_max_re_tmp = J_min * V_init * l10_tmp;
  c_J_max_re_tmp = b_l88_tmp * l9_tmp;
  V_init_re_tmp = l86 * 3.0;
  b_l2_re_tmp = l85 * 6.0;
  c_A_init_re_tmp = t3[0].re * t3[0].re - t3[0].im * t3[0].im;
  l2_re_tmp = t3[0].re * t3[0].im;
  J_max_re_tmp = l2_re_tmp + l2_re_tmp;
  b_J_max_re_tmp = t7[0].re * t7[0].re - t7[0].im * t7[0].im;
  l2_re_tmp = t7[0].re * t7[0].im;
  l22 = l2_re_tmp + l2_re_tmp;
  A_init_re_tmp = J_max_tmp * c_A_init_re_tmp;
  l2_re_tmp = J_max_tmp * J_max_re_tmp;
  l115_im = l5_tmp * c_A_init_re_tmp;
  V_max_re_tmp = l5_tmp * J_max_re_tmp;
  ar = -(A_init + J_min * t3[0].re);
  ai = -(J_min * t3[0].im);
  if (ai == 0.0) {
    t[0].re = ar / J_max;
    t[0].im = 0.0;
  } else if (ar == 0.0) {
    t[0].re = 0.0;
    t[0].im = ai / J_max;
  } else {
    t[0].re = ar / J_max;
    t[0].im = ai / J_max;
  }
  t[3].re = 0.0;
  t[3].im = 0.0;
  t[6] = t3[0];
  a = b_l4_tmp * c_A_init_re_tmp;
  l21 = b_l4_tmp * J_max_re_tmp;
  ar = ((((((((((((((((((((((((((((l28.re - y * l11[0].re) -
                                  l12_tmp * l13[0].re) +
                                 b_A_wayp * b_J_max_re_tmp * 3.0) +
                                J_max_tmp * l11[0].re * 3.0) +
                               l11_tmp * l13[0].re * 3.0) +
                              l29.re) -
                             l112_re) -
                            l5_tmp * l11[0].re * 2.0) -
                           b_l4_tmp * l13[0].re * 2.0) +
                          b_l2 * t3[0].re * 3.0) -
                         l3_tmp * t7[0].re * 3.0) -
                        l86 * t3[0].re * 3.0) -
                       l86 * t7[0].re * 3.0) +
                      l93 * t7[0].re * 3.0) +
                     l88 * t7[0].re * 3.0) +
                    (A_init_re_tmp * t7[0].re - l2_re_tmp * t7[0].im) * 3.0) +
                   l85 * t3[0].re * 6.0) +
                  l85 * t7[0].re * 6.0) +
                 (a * t7[0].re - l21 * t7[0].im) * 3.0) -
                (l115_im * t7[0].re - V_max_re_tmp * t7[0].im) * 6.0) -
               l115_re) -
              l24) +
             l23) +
            l83 * c_A_init_re_tmp * 3.0) -
           b_A_init_re_tmp * b_J_max_re_tmp * 6.0) -
          b_V_init_re_tmp * t3[0].re * 6.0) -
         b_V_max_re_tmp * t7[0].re * 6.0) -
        c_J_max_re_tmp * c_A_init_re_tmp * 3.0) +
       c_J_max_re_tmp * b_J_max_re_tmp * 3.0;
  ai = (((((((((((((((((((((((0.0 - y * l11[0].im) - l12_tmp * l13[0].im) +
                            b_A_wayp * l22 * 3.0) +
                           J_max_tmp * l11[0].im * 3.0) +
                          l11_tmp * l13[0].im * 3.0) -
                         l5_tmp * l11[0].im * 2.0) -
                        b_l4_tmp * l13[0].im * 2.0) +
                       b_l2 * t3[0].im * 3.0) -
                      l3_tmp * t7[0].im * 3.0) -
                     l86 * t3[0].im * 3.0) -
                    l86 * t7[0].im * 3.0) +
                   l93 * t7[0].im * 3.0) +
                  l88 * t7[0].im * 3.0) +
                 (A_init_re_tmp * t7[0].im + l2_re_tmp * t7[0].re) * 3.0) +
                l85 * t3[0].im * 6.0) +
               l85 * t7[0].im * 6.0) +
              (a * t7[0].im + l21 * t7[0].re) * 3.0) -
             (l115_im * t7[0].im + V_max_re_tmp * t7[0].re) * 6.0) +
            l83 * J_max_re_tmp * 3.0) -
           b_A_init_re_tmp * l22 * 6.0) -
          b_V_init_re_tmp * t3[0].im * 6.0) -
         b_V_max_re_tmp * t7[0].im * 6.0) -
        c_J_max_re_tmp * J_max_re_tmp * 3.0) +
       c_J_max_re_tmp * l22 * 3.0;
  l21 = ((V_init_re_tmp - A_init_re_tmp * 3.0) - b_l2_re_tmp) + l115_im * 3.0;
  a = (0.0 - l2_re_tmp * 3.0) + V_max_re_tmp * 3.0;
  if (a == 0.0) {
    if (ai == 0.0) {
      t[9].re = ar / l21;
      t[9].im = 0.0;
    } else if (ar == 0.0) {
      t[9].re = 0.0;
      t[9].im = ai / l21;
    } else {
      t[9].re = ar / l21;
      t[9].im = ai / l21;
    }
  } else if (l21 == 0.0) {
    if (ar == 0.0) {
      t[9].re = ai / a;
      t[9].im = 0.0;
    } else if (ai == 0.0) {
      t[9].re = 0.0;
      t[9].im = -(ar / a);
    } else {
      t[9].re = ai / a;
      t[9].im = -(ar / a);
    }
  } else {
    l22 = std::abs(l21);
    l115_im = std::abs(a);
    if (l22 > l115_im) {
      l115_im = a / l21;
      a = l21 + l115_im * a;
      t[9].re = (ar + l115_im * ai) / a;
      t[9].im = (ai - l115_im * ar) / a;
    } else if (l115_im == l22) {
      if (l21 > 0.0) {
        l115_im = 0.5;
      } else {
        l115_im = -0.5;
      }
      if (a > 0.0) {
        a = 0.5;
      } else {
        a = -0.5;
      }
      t[9].re = (ar * l115_im + ai * a) / l22;
      t[9].im = (ai * l115_im - ar * a) / l22;
    } else {
      l115_im = l21 / a;
      a += l115_im * l21;
      t[9].re = (l115_im * ar + ai) / a;
      t[9].im = (l115_im * ai - ar) / a;
    }
  }
  ar = A_wayp - J_max * t7[0].re;
  ai = 0.0 - J_max * t7[0].im;
  if (ai == 0.0) {
    t[12].re = ar / J_min;
    t[12].im = 0.0;
  } else if (ar == 0.0) {
    t[12].re = 0.0;
    t[12].im = ai / J_min;
  } else {
    t[12].re = ar / J_min;
    t[12].im = ai / J_min;
  }
  t[15].re = 0.0;
  t[15].im = 0.0;
  t[18] = t7[0];
  c_A_init_re_tmp = t3[1].re * t3[1].re - t3[1].im * t3[1].im;
  l2_re_tmp = t3[1].re * t3[1].im;
  J_max_re_tmp = l2_re_tmp + l2_re_tmp;
  b_J_max_re_tmp = t7[1].re * t7[1].re - t7[1].im * t7[1].im;
  l2_re_tmp = t7[1].re * t7[1].im;
  l22 = l2_re_tmp + l2_re_tmp;
  A_init_re_tmp = J_max_tmp * c_A_init_re_tmp;
  l2_re_tmp = J_max_tmp * J_max_re_tmp;
  l115_im = l5_tmp * c_A_init_re_tmp;
  V_max_re_tmp = l5_tmp * J_max_re_tmp;
  ar = -(A_init + J_min * t3[1].re);
  ai = -(J_min * t3[1].im);
  if (ai == 0.0) {
    t[1].re = ar / J_max;
    t[1].im = 0.0;
  } else if (ar == 0.0) {
    t[1].re = 0.0;
    t[1].im = ai / J_max;
  } else {
    t[1].re = ar / J_max;
    t[1].im = ai / J_max;
  }
  t[4].re = 0.0;
  t[4].im = 0.0;
  t[7] = t3[1];
  a = b_l4_tmp * c_A_init_re_tmp;
  l21 = b_l4_tmp * J_max_re_tmp;
  ar = ((((((((((((((((((((((((((((l28.re - y * l11[1].re) -
                                  l12_tmp * l13[1].re) +
                                 b_A_wayp * b_J_max_re_tmp * 3.0) +
                                J_max_tmp * l11[1].re * 3.0) +
                               l11_tmp * l13[1].re * 3.0) +
                              l29.re) -
                             l112_re) -
                            l5_tmp * l11[1].re * 2.0) -
                           b_l4_tmp * l13[1].re * 2.0) +
                          b_l2 * t3[1].re * 3.0) -
                         l3_tmp * t7[1].re * 3.0) -
                        l86 * t3[1].re * 3.0) -
                       l86 * t7[1].re * 3.0) +
                      l93 * t7[1].re * 3.0) +
                     l88 * t7[1].re * 3.0) +
                    (A_init_re_tmp * t7[1].re - l2_re_tmp * t7[1].im) * 3.0) +
                   l85 * t3[1].re * 6.0) +
                  l85 * t7[1].re * 6.0) +
                 (a * t7[1].re - l21 * t7[1].im) * 3.0) -
                (l115_im * t7[1].re - V_max_re_tmp * t7[1].im) * 6.0) -
               l115_re) -
              l24) +
             l23) +
            l83 * c_A_init_re_tmp * 3.0) -
           b_A_init_re_tmp * b_J_max_re_tmp * 6.0) -
          b_V_init_re_tmp * t3[1].re * 6.0) -
         b_V_max_re_tmp * t7[1].re * 6.0) -
        c_J_max_re_tmp * c_A_init_re_tmp * 3.0) +
       c_J_max_re_tmp * b_J_max_re_tmp * 3.0;
  ai = (((((((((((((((((((((((0.0 - y * l11[1].im) - l12_tmp * l13[1].im) +
                            b_A_wayp * l22 * 3.0) +
                           J_max_tmp * l11[1].im * 3.0) +
                          l11_tmp * l13[1].im * 3.0) -
                         l5_tmp * l11[1].im * 2.0) -
                        b_l4_tmp * l13[1].im * 2.0) +
                       b_l2 * t3[1].im * 3.0) -
                      l3_tmp * t7[1].im * 3.0) -
                     l86 * t3[1].im * 3.0) -
                    l86 * t7[1].im * 3.0) +
                   l93 * t7[1].im * 3.0) +
                  l88 * t7[1].im * 3.0) +
                 (A_init_re_tmp * t7[1].im + l2_re_tmp * t7[1].re) * 3.0) +
                l85 * t3[1].im * 6.0) +
               l85 * t7[1].im * 6.0) +
              (a * t7[1].im + l21 * t7[1].re) * 3.0) -
             (l115_im * t7[1].im + V_max_re_tmp * t7[1].re) * 6.0) +
            l83 * J_max_re_tmp * 3.0) -
           b_A_init_re_tmp * l22 * 6.0) -
          b_V_init_re_tmp * t3[1].im * 6.0) -
         b_V_max_re_tmp * t7[1].im * 6.0) -
        c_J_max_re_tmp * J_max_re_tmp * 3.0) +
       c_J_max_re_tmp * l22 * 3.0;
  l21 = ((V_init_re_tmp - A_init_re_tmp * 3.0) - b_l2_re_tmp) + l115_im * 3.0;
  a = (0.0 - l2_re_tmp * 3.0) + V_max_re_tmp * 3.0;
  if (a == 0.0) {
    if (ai == 0.0) {
      t[10].re = ar / l21;
      t[10].im = 0.0;
    } else if (ar == 0.0) {
      t[10].re = 0.0;
      t[10].im = ai / l21;
    } else {
      t[10].re = ar / l21;
      t[10].im = ai / l21;
    }
  } else if (l21 == 0.0) {
    if (ar == 0.0) {
      t[10].re = ai / a;
      t[10].im = 0.0;
    } else if (ai == 0.0) {
      t[10].re = 0.0;
      t[10].im = -(ar / a);
    } else {
      t[10].re = ai / a;
      t[10].im = -(ar / a);
    }
  } else {
    l22 = std::abs(l21);
    l115_im = std::abs(a);
    if (l22 > l115_im) {
      l115_im = a / l21;
      a = l21 + l115_im * a;
      t[10].re = (ar + l115_im * ai) / a;
      t[10].im = (ai - l115_im * ar) / a;
    } else if (l115_im == l22) {
      if (l21 > 0.0) {
        l115_im = 0.5;
      } else {
        l115_im = -0.5;
      }
      if (a > 0.0) {
        a = 0.5;
      } else {
        a = -0.5;
      }
      t[10].re = (ar * l115_im + ai * a) / l22;
      t[10].im = (ai * l115_im - ar * a) / l22;
    } else {
      l115_im = l21 / a;
      a += l115_im * l21;
      t[10].re = (l115_im * ar + ai) / a;
      t[10].im = (l115_im * ai - ar) / a;
    }
  }
  ar = A_wayp - J_max * t7[1].re;
  ai = 0.0 - J_max * t7[1].im;
  if (ai == 0.0) {
    t[13].re = ar / J_min;
    t[13].im = 0.0;
  } else if (ar == 0.0) {
    t[13].re = 0.0;
    t[13].im = ai / J_min;
  } else {
    t[13].re = ar / J_min;
    t[13].im = ai / J_min;
  }
  t[16].re = 0.0;
  t[16].im = 0.0;
  t[19] = t7[1];
  c_A_init_re_tmp = t3[2].re * t3[2].re - t3[2].im * t3[2].im;
  l2_re_tmp = t3[2].re * t3[2].im;
  J_max_re_tmp = l2_re_tmp + l2_re_tmp;
  b_J_max_re_tmp = t7[2].re * t7[2].re - t7[2].im * t7[2].im;
  l2_re_tmp = t7[2].re * t7[2].im;
  l22 = l2_re_tmp + l2_re_tmp;
  A_init_re_tmp = J_max_tmp * c_A_init_re_tmp;
  l2_re_tmp = J_max_tmp * J_max_re_tmp;
  l115_im = l5_tmp * c_A_init_re_tmp;
  V_max_re_tmp = l5_tmp * J_max_re_tmp;
  ar = -(A_init + J_min * t3[2].re);
  ai = -(J_min * t3[2].im);
  if (ai == 0.0) {
    t[2].re = ar / J_max;
    t[2].im = 0.0;
  } else if (ar == 0.0) {
    t[2].re = 0.0;
    t[2].im = ai / J_max;
  } else {
    t[2].re = ar / J_max;
    t[2].im = ai / J_max;
  }
  t[5].re = 0.0;
  t[5].im = 0.0;
  t[8] = t3[2];
  a = b_l4_tmp * c_A_init_re_tmp;
  l21 = b_l4_tmp * J_max_re_tmp;
  ar = ((((((((((((((((((((((((((((l28.re - y * l11[2].re) -
                                  l12_tmp * l13[2].re) +
                                 b_A_wayp * b_J_max_re_tmp * 3.0) +
                                J_max_tmp * l11[2].re * 3.0) +
                               l11_tmp * l13[2].re * 3.0) +
                              l29.re) -
                             l112_re) -
                            l5_tmp * l11[2].re * 2.0) -
                           b_l4_tmp * l13[2].re * 2.0) +
                          b_l2 * t3[2].re * 3.0) -
                         l3_tmp * t7[2].re * 3.0) -
                        l86 * t3[2].re * 3.0) -
                       l86 * t7[2].re * 3.0) +
                      l93 * t7[2].re * 3.0) +
                     l88 * t7[2].re * 3.0) +
                    (A_init_re_tmp * t7[2].re - l2_re_tmp * t7[2].im) * 3.0) +
                   l85 * t3[2].re * 6.0) +
                  l85 * t7[2].re * 6.0) +
                 (a * t7[2].re - l21 * t7[2].im) * 3.0) -
                (l115_im * t7[2].re - V_max_re_tmp * t7[2].im) * 6.0) -
               l115_re) -
              l24) +
             l23) +
            l83 * c_A_init_re_tmp * 3.0) -
           b_A_init_re_tmp * b_J_max_re_tmp * 6.0) -
          b_V_init_re_tmp * t3[2].re * 6.0) -
         b_V_max_re_tmp * t7[2].re * 6.0) -
        c_J_max_re_tmp * c_A_init_re_tmp * 3.0) +
       c_J_max_re_tmp * b_J_max_re_tmp * 3.0;
  ai = (((((((((((((((((((((((0.0 - y * l11[2].im) - l12_tmp * l13[2].im) +
                            b_A_wayp * l22 * 3.0) +
                           J_max_tmp * l11[2].im * 3.0) +
                          l11_tmp * l13[2].im * 3.0) -
                         l5_tmp * l11[2].im * 2.0) -
                        b_l4_tmp * l13[2].im * 2.0) +
                       b_l2 * t3[2].im * 3.0) -
                      l3_tmp * t7[2].im * 3.0) -
                     l86 * t3[2].im * 3.0) -
                    l86 * t7[2].im * 3.0) +
                   l93 * t7[2].im * 3.0) +
                  l88 * t7[2].im * 3.0) +
                 (A_init_re_tmp * t7[2].im + l2_re_tmp * t7[2].re) * 3.0) +
                l85 * t3[2].im * 6.0) +
               l85 * t7[2].im * 6.0) +
              (a * t7[2].im + l21 * t7[2].re) * 3.0) -
             (l115_im * t7[2].im + V_max_re_tmp * t7[2].re) * 6.0) +
            l83 * J_max_re_tmp * 3.0) -
           b_A_init_re_tmp * l22 * 6.0) -
          b_V_init_re_tmp * t3[2].im * 6.0) -
         b_V_max_re_tmp * t7[2].im * 6.0) -
        c_J_max_re_tmp * J_max_re_tmp * 3.0) +
       c_J_max_re_tmp * l22 * 3.0;
  l21 = ((V_init_re_tmp - A_init_re_tmp * 3.0) - b_l2_re_tmp) + l115_im * 3.0;
  a = (0.0 - l2_re_tmp * 3.0) + V_max_re_tmp * 3.0;
  if (a == 0.0) {
    if (ai == 0.0) {
      t[11].re = ar / l21;
      t[11].im = 0.0;
    } else if (ar == 0.0) {
      t[11].re = 0.0;
      t[11].im = ai / l21;
    } else {
      t[11].re = ar / l21;
      t[11].im = ai / l21;
    }
  } else if (l21 == 0.0) {
    if (ar == 0.0) {
      t[11].re = ai / a;
      t[11].im = 0.0;
    } else if (ai == 0.0) {
      t[11].re = 0.0;
      t[11].im = -(ar / a);
    } else {
      t[11].re = ai / a;
      t[11].im = -(ar / a);
    }
  } else {
    l22 = std::abs(l21);
    l115_im = std::abs(a);
    if (l22 > l115_im) {
      l115_im = a / l21;
      a = l21 + l115_im * a;
      t[11].re = (ar + l115_im * ai) / a;
      t[11].im = (ai - l115_im * ar) / a;
    } else if (l115_im == l22) {
      if (l21 > 0.0) {
        l115_im = 0.5;
      } else {
        l115_im = -0.5;
      }
      if (a > 0.0) {
        a = 0.5;
      } else {
        a = -0.5;
      }
      t[11].re = (ar * l115_im + ai * a) / l22;
      t[11].im = (ai * l115_im - ar * a) / l22;
    } else {
      l115_im = l21 / a;
      a += l115_im * l21;
      t[11].re = (l115_im * ar + ai) / a;
      t[11].im = (l115_im * ai - ar) / a;
    }
  }
  ar = A_wayp - J_max * t7[2].re;
  ai = 0.0 - J_max * t7[2].im;
  if (ai == 0.0) {
    t[14].re = ar / J_min;
    t[14].im = 0.0;
  } else if (ar == 0.0) {
    t[14].re = 0.0;
    t[14].im = ai / J_min;
  } else {
    t[14].re = ar / J_min;
    t[14].im = ai / J_min;
  }
  t[17].re = 0.0;
  t[17].im = 0.0;
  t[20] = t7[2];
}

// End of code generation (acdeg_T_AP.cpp)
