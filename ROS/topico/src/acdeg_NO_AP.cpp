//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// acdeg_NO_AP.cpp
//
// Code generation for function 'acdeg_NO_AP'
//

// Include files
#include "acdeg_NO_AP.h"
#include "power.h"
#include "rt_nonfinite.h"
#include "sqrt.h"
#include "topico_wrapper_data.h"
#include "topico_wrapper_rtwutil.h"
#include "topico_wrapper_types.h"
#include <cmath>

// Function Definitions
void acdeg_NO_AP(double P_init, double V_init, double A_init, double P_wayp,
                 double A_wayp, double V_max, double J_max, double J_min,
                 creal_T t[14])
{
  creal_T b_l15[2];
  creal_T l14[2];
  creal_T l24[2];
  creal_T l28[2];
  creal_T l29[2];
  creal_T l33[2];
  creal_T l35[2];
  creal_T t1[2];
  creal_T l20;
  double A_init_re;
  double A_init_re_tmp;
  double A_init_re_tmp_tmp;
  double A_init_tmp;
  double A_wayp_re;
  double J_max_re;
  double J_max_tmp;
  double J_min_re;
  double P_init_re;
  double P_wayp_re;
  double V_init_tmp;
  double b_A_init;
  double b_A_init_re;
  double b_A_init_tmp;
  double b_A_wayp_re;
  double b_J_max;
  double b_J_max_re;
  double b_J_max_tmp;
  double b_J_min;
  double b_P_init_re;
  double b_P_wayp_re;
  double b_V_init_tmp;
  double b_l15_tmp;
  double b_l17_tmp;
  double b_l2_tmp;
  double c_A_init;
  double c_A_init_re;
  double c_A_wayp_re;
  double c_J_max;
  double c_P_init_re;
  double c_P_wayp_re;
  double c_V_init_tmp;
  double d;
  double d1;
  double d2;
  double d3;
  double d_J_max;
  double im;
  double l10;
  double l11;
  double l12;
  double l13;
  double l15;
  double l15_tmp;
  double l17_tmp;
  double l18_tmp;
  double l19;
  double l2;
  double l20_tmp;
  double l23;
  double l2_tmp;
  double l3;
  double l31;
  double l32;
  double l3_re;
  double l4_tmp;
  double l5;
  double l5_re;
  double l5_tmp;
  double l7;
  double l7_tmp;
  double l8;
  double l8_tmp;
  double l9;
  double l9_tmp;
  double re;
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
  //  Generated on 05-Sep-2019 11:36:45
  l31 = A_init * J_min;
  l5_tmp = A_init * J_max;
  if (J_max < 0.0) {
    f_rtErrorWithMessageID("sqrt", n_emlrtRTEI.fName, n_emlrtRTEI.lineNo);
  }
  l15_tmp = J_min * J_min;
  b_l15_tmp = J_min * J_max;
  l15 = 1.0 / (l15_tmp + -b_l15_tmp);
  l20_tmp = A_init * A_init;
  l20.re = -((J_min + -J_max) *
             ((l20_tmp + J_min * V_max * 2.0) + -(J_min * V_init * 2.0)));
  l20.im = 0.0;
  coder::internal::scalar::b_sqrt(&l20);
  l23 = std::sqrt(J_max);
  l20.re *= l23;
  l20.im *= l23;
  t1[0].re = l15 * ((-l31 + l5_tmp) + l20.re);
  t1[0].im = l15 * l20.im;
  t1[1].re = -l15 * ((l31 - l5_tmp) + l20.re);
  t1[1].im = -l15 * l20.im;
  l3 = rt_powd_snf(A_init, 3.0);
  l4_tmp = A_wayp * A_wayp;
  l5 = rt_powd_snf(A_wayp, 3.0);
  l7_tmp = rt_powd_snf(J_min, 3.0);
  l9 = J_max * J_max;
  l10 = rt_powd_snf(J_min, 5.0);
  l11 = rt_powd_snf(J_max, 3.0);
  coder::power(t1, b_l15);
  l17_tmp = b_l15_tmp * V_init;
  b_l17_tmp = l17_tmp * 2.0;
  l18_tmp = b_l15_tmp * V_max * 2.0;
  l8 = l15_tmp * l15_tmp;
  l12 = rt_powd_snf(l15_tmp, 3.0);
  l13 = l9 * l9;
  l19 = J_min * l20_tmp;
  l23 = J_max + -J_min;
  re = t1[0].re * t1[0].re - t1[0].im * t1[0].im;
  l15 = t1[0].re * t1[0].im;
  im = l15 + l15;
  l14[0].re = re;
  l14[0].im = im;
  A_init_re_tmp = l31 * J_max;
  l24[0].re = 2.0 * (A_init_re_tmp * t1[0].re);
  l24[0].im = 2.0 * (A_init_re_tmp * t1[0].im);
  l28[0].re = l7_tmp * re;
  l28[0].im = l7_tmp * im;
  b_J_max = J_max * l15_tmp;
  l29[0].re = b_J_max * re;
  l29[0].im = b_J_max * im;
  re = t1[1].re * t1[1].re - t1[1].im * t1[1].im;
  l15 = t1[1].re * t1[1].im;
  im = l15 + l15;
  l24[1].re = 2.0 * (A_init_re_tmp * t1[1].re);
  l24[1].im = 2.0 * (A_init_re_tmp * t1[1].im);
  d = l7_tmp * re;
  d1 = l7_tmp * im;
  l29[1].re = b_J_max * re;
  l29[1].im = b_J_max * im;
  l31 = rt_powd_snf(l23, 1.5);
  if (l23 < 0.0) {
    k_rtErrorWithMessageID(o_emlrtRTEI.fName, o_emlrtRTEI.lineNo);
  }
  l32 = rt_powd_snf(l23, 2.5);
  A_init_re_tmp_tmp = A_init * l15_tmp;
  d2 = l4_tmp * -J_min;
  d3 = J_max * l4_tmp;
  l15 = (((l18_tmp + l19) + d3) + -b_l17_tmp) + d2;
  l33[0].re =
      (((l15 + -l24[0].re) + A_init_re_tmp_tmp * t1[0].re * 2.0) + l28[0].re) +
      -l29[0].re;
  l33[0].im = ((-l24[0].im + A_init_re_tmp_tmp * t1[0].im * 2.0) + l28[0].im) +
              -l29[0].im;
  l33[1].re = (((l15 + -l24[1].re) + A_init_re_tmp_tmp * t1[1].re * 2.0) + d) +
              -l29[1].re;
  l33[1].im =
      ((-l24[1].im + A_init_re_tmp_tmp * t1[1].im * 2.0) + d1) + -l29[1].im;
  coder::internal::scalar::b_sqrt(&l33[0]);
  coder::internal::scalar::b_sqrt(&l33[1]);
  coder::power(l33, l35);
  y = rt_powd_snf(J_min, 7.0);
  l20.re = l3 * l8 * 2.0 - l5 * l8 * 2.0;
  b_A_init = A_init * l12;
  c_A_init = A_init * l11;
  J_max_re = J_max * l3 * l7_tmp * 4.0;
  b_J_max_re = J_max * l5 * l7_tmp * 7.0;
  J_min_re = J_min * l5 * l11 * 3.0;
  J_max_tmp = J_max * l12;
  b_J_min = J_min * l31;
  c_J_max = J_max * l31;
  P_init_re = P_init * l8 * l9 * 6.0;
  b_P_init_re = P_init * l7_tmp * l11 * 12.0;
  c_P_init_re = P_init * l15_tmp * l13 * 6.0;
  P_wayp_re = P_wayp * l8 * l9 * 6.0;
  b_P_wayp_re = P_wayp * l7_tmp * l11 * 12.0;
  c_P_wayp_re = P_wayp * l15_tmp * l13 * 6.0;
  l3_re = l3 * l15_tmp * l9 * 2.0;
  l5_re = l5 * l15_tmp * l9 * 8.0;
  l8_tmp = l8 * l11;
  l9_tmp = l9 * l10;
  l7 = l7_tmp * l13;
  l19 *= l32;
  l2 = l20_tmp * l10;
  b_J_max_tmp = J_max * l20_tmp * l8;
  V_init_tmp = V_init * l8 * l9;
  b_V_init_tmp = V_init * l7_tmp * l11;
  c_V_init_tmp = V_init * l15_tmp * l13;
  l2_tmp = l20_tmp * l7_tmp * l9;
  b_l2_tmp = l20_tmp * l15_tmp * l11;
  A_init_re = l5_tmp * V_init * l8 * 6.0;
  A_wayp_re = A_wayp * J_max * V_max * l8 * 6.0;
  A_init_tmp = l5_tmp * l10;
  l15 = A_init * V_init;
  b_A_init_re = l15 * l7_tmp * l9 * 12.0;
  c_A_init_re = l15 * l15_tmp * l11 * 6.0;
  l15 = A_wayp * V_max;
  b_A_wayp_re = l15 * l7_tmp * l9 * 12.0;
  c_A_wayp_re = l15 * l15_tmp * l11 * 6.0;
  d_J_max = J_max * V_init * l10;
  l8 = A_init * l8 * l9;
  b_A_init_tmp = A_init_re_tmp_tmp * l13;
  l10 = l17_tmp * l32;
  l9 = A_init_re_tmp_tmp * l32;
  l15 = A_init_re_tmp * l32;
  l23 = l32 * l28[0].re;
  l4_tmp = l32 * l28[0].im;
  l12 = l32 * l29[0].re;
  l31 = l32 * l29[0].im;
  l3 = l9 * l33[0].re;
  l5 = l9 * l33[0].im;
  l20_tmp = l15 * l33[0].re;
  l5_tmp = l15 * l33[0].im;
  b_l15[0].re =
      -(((((((((((((((((((((((((((((((((((((((((((l20.re +
                                                  y * b_l15[0].re * 2.0) +
                                                 b_A_init * l14[0].re * 6.0) -
                                                c_A_init * l28[0].re * 15.0) -
                                               J_max_re) +
                                              b_J_max_re) +
                                             J_min_re) -
                                            J_max_tmp * b_l15[0].re * 7.0) +
                                           b_J_min * l35[0].re * 2.0) -
                                          c_J_max * l35[0].re) +
                                         P_init_re) -
                                        b_P_init_re) +
                                       c_P_init_re) -
                                      P_wayp_re) +
                                     b_P_wayp_re) -
                                    c_P_wayp_re) +
                                   l3_re) -
                                  l5_re) -
                                 l8_tmp * b_l15[0].re * 5.0) +
                                l9_tmp * b_l15[0].re * 9.0) +
                               l7 * b_l15[0].re) +
                              l19 * l33[0].re * 3.0) +
                             (l23 * l33[0].re - l4_tmp * l33[0].im) * 3.0) -
                            (l12 * l33[0].re - l31 * l33[0].im) * 3.0) +
                           l2 * t1[0].re * 6.0) -
                          b_J_max_tmp * t1[0].re * 18.0) +
                         V_init_tmp * t1[0].re * 18.0) -
                        b_V_init_tmp * t1[0].re * 18.0) +
                       c_V_init_tmp * t1[0].re * 6.0) +
                      l2_tmp * t1[0].re * 18.0) -
                     b_l2_tmp * t1[0].re * 6.0) -
                    A_init_re) +
                   A_wayp_re) -
                  A_init_tmp * l14[0].re * 21.0) +
                 b_A_init_re) -
                c_A_init_re) -
               b_A_wayp_re) +
              c_A_wayp_re) -
             d_J_max * t1[0].re * 6.0) +
            l8 * l14[0].re * 27.0) +
           b_A_init_tmp * l14[0].re * 3.0) -
          l10 * l33[0].re * 6.0) +
         (l3 * t1[0].re - l5 * t1[0].im) * 6.0) -
        (l20_tmp * t1[0].re - l5_tmp * t1[0].im) * 6.0);
  b_l15[0].im = -(((((((((((((((((((((((((y * b_l15[0].im * 2.0 +
                                          b_A_init * l14[0].im * 6.0) -
                                         c_A_init * l28[0].im * 15.0) -
                                        J_max_tmp * b_l15[0].im * 7.0) +
                                       b_J_min * l35[0].im * 2.0) -
                                      c_J_max * l35[0].im) -
                                     l8_tmp * b_l15[0].im * 5.0) +
                                    l9_tmp * b_l15[0].im * 9.0) +
                                   l7 * b_l15[0].im) +
                                  l19 * l33[0].im * 3.0) +
                                 (l23 * l33[0].im + l4_tmp * l33[0].re) * 3.0) -
                                (l12 * l33[0].im + l31 * l33[0].re) * 3.0) +
                               l2 * t1[0].im * 6.0) -
                              b_J_max_tmp * t1[0].im * 18.0) +
                             V_init_tmp * t1[0].im * 18.0) -
                            b_V_init_tmp * t1[0].im * 18.0) +
                           c_V_init_tmp * t1[0].im * 6.0) +
                          l2_tmp * t1[0].im * 18.0) -
                         b_l2_tmp * t1[0].im * 6.0) -
                        A_init_tmp * l14[0].im * 21.0) -
                       d_J_max * t1[0].im * 6.0) +
                      l8 * l14[0].im * 27.0) +
                     b_A_init_tmp * l14[0].im * 3.0) -
                    l10 * l33[0].im * 6.0) +
                   (l3 * t1[0].im + l5 * t1[0].re) * 6.0) -
                  (l20_tmp * t1[0].im + l5_tmp * t1[0].re) * 6.0);
  l23 = l32 * d;
  l4_tmp = l32 * d1;
  l12 = l32 * l29[1].re;
  l31 = l32 * l29[1].im;
  l3 = l9 * l33[1].re;
  l5 = l9 * l33[1].im;
  l20_tmp = l15 * l33[1].re;
  l5_tmp = l15 * l33[1].im;
  b_l15[1].re =
      -(((((((((((((((((((((((((((((((((((((((((((l20.re +
                                                  y * b_l15[1].re * 2.0) +
                                                 b_A_init * re * 6.0) -
                                                c_A_init * d * 15.0) -
                                               J_max_re) +
                                              b_J_max_re) +
                                             J_min_re) -
                                            J_max_tmp * b_l15[1].re * 7.0) +
                                           b_J_min * l35[1].re * 2.0) -
                                          c_J_max * l35[1].re) +
                                         P_init_re) -
                                        b_P_init_re) +
                                       c_P_init_re) -
                                      P_wayp_re) +
                                     b_P_wayp_re) -
                                    c_P_wayp_re) +
                                   l3_re) -
                                  l5_re) -
                                 l8_tmp * b_l15[1].re * 5.0) +
                                l9_tmp * b_l15[1].re * 9.0) +
                               l7 * b_l15[1].re) +
                              l19 * l33[1].re * 3.0) +
                             (l23 * l33[1].re - l4_tmp * l33[1].im) * 3.0) -
                            (l12 * l33[1].re - l31 * l33[1].im) * 3.0) +
                           l2 * t1[1].re * 6.0) -
                          b_J_max_tmp * t1[1].re * 18.0) +
                         V_init_tmp * t1[1].re * 18.0) -
                        b_V_init_tmp * t1[1].re * 18.0) +
                       c_V_init_tmp * t1[1].re * 6.0) +
                      l2_tmp * t1[1].re * 18.0) -
                     b_l2_tmp * t1[1].re * 6.0) -
                    A_init_re) +
                   A_wayp_re) -
                  A_init_tmp * re * 21.0) +
                 b_A_init_re) -
                c_A_init_re) -
               b_A_wayp_re) +
              c_A_wayp_re) -
             d_J_max * t1[1].re * 6.0) +
            l8 * re * 27.0) +
           b_A_init_tmp * re * 3.0) -
          l10 * l33[1].re * 6.0) +
         (l3 * t1[1].re - l5 * t1[1].im) * 6.0) -
        (l20_tmp * t1[1].re - l5_tmp * t1[1].im) * 6.0);
  b_l15[1].im =
      -(((((((((((((((((((((((((y * b_l15[1].im * 2.0 + b_A_init * im * 6.0) -
                               c_A_init * d1 * 15.0) -
                              J_max_tmp * b_l15[1].im * 7.0) +
                             b_J_min * l35[1].im * 2.0) -
                            c_J_max * l35[1].im) -
                           l8_tmp * b_l15[1].im * 5.0) +
                          l9_tmp * b_l15[1].im * 9.0) +
                         l7 * b_l15[1].im) +
                        l19 * l33[1].im * 3.0) +
                       (l23 * l33[1].im + l4_tmp * l33[1].re) * 3.0) -
                      (l12 * l33[1].im + l31 * l33[1].re) * 3.0) +
                     l2 * t1[1].im * 6.0) -
                    b_J_max_tmp * t1[1].im * 18.0) +
                   V_init_tmp * t1[1].im * 18.0) -
                  b_V_init_tmp * t1[1].im * 18.0) +
                 c_V_init_tmp * t1[1].im * 6.0) +
                l2_tmp * t1[1].im * 18.0) -
               b_l2_tmp * t1[1].im * 6.0) -
              A_init_tmp * im * 21.0) -
             d_J_max * t1[1].im * 6.0) +
            l8 * im * 27.0) +
           b_A_init_tmp * im * 3.0) -
          l10 * l33[1].im * 6.0) +
         (l3 * t1[1].im + l5 * t1[1].re) * 6.0) -
        (l20_tmp * t1[1].im + l5_tmp * t1[1].re) * 6.0);
  J_max_re = b_J_max_tmp * 3.0;
  l20.re = V_init_tmp * 6.0;
  b_J_max_re = b_V_init_tmp * 12.0;
  P_init_re = c_V_init_tmp * 6.0;
  b_P_init_re = l2_tmp * 6.0;
  c_P_init_re = b_l2_tmp * 3.0;
  b_A_init = A_init * l7_tmp * l11;
  l28[0].re = ((((((((((((l13 * l28[0].re * 3.0 - J_max_re) -
                         J_max_tmp * l14[0].re * 3.0) +
                        l20.re) -
                       b_J_max_re) +
                      P_init_re) +
                     b_P_init_re) -
                    c_P_init_re) -
                   l8_tmp * l14[0].re * 9.0) +
                  l9_tmp * l14[0].re * 9.0) -
                 A_init_tmp * t1[0].re * 6.0) +
                l8 * t1[0].re * 18.0) -
               b_A_init * t1[0].re * 18.0) +
              b_A_init_tmp * t1[0].re * 6.0;
  l28[0].im = ((((((l13 * l28[0].im * 3.0 - J_max_tmp * l14[0].im * 3.0) -
                   l8_tmp * l14[0].im * 9.0) +
                  l9_tmp * l14[0].im * 9.0) -
                 A_init_tmp * t1[0].im * 6.0) +
                l8 * t1[0].im * 18.0) -
               b_A_init * t1[0].im * 18.0) +
              b_A_init_tmp * t1[0].im * 6.0;
  l28[1].re =
      ((((((((((((l13 * d * 3.0 - J_max_re) - J_max_tmp * re * 3.0) + l20.re) -
               b_J_max_re) +
              P_init_re) +
             b_P_init_re) -
            c_P_init_re) -
           l8_tmp * re * 9.0) +
          l9_tmp * re * 9.0) -
         A_init_tmp * t1[1].re * 6.0) +
        l8 * t1[1].re * 18.0) -
       b_A_init * t1[1].re * 18.0) +
      b_A_init_tmp * t1[1].re * 6.0;
  l28[1].im =
      ((((((l13 * d1 * 3.0 - J_max_tmp * im * 3.0) - l8_tmp * im * 9.0) +
          l9_tmp * im * 9.0) -
         A_init_tmp * t1[1].im * 6.0) +
        l8 * t1[1].im * 18.0) -
       b_A_init * t1[1].im * 18.0) +
      b_A_init_tmp * t1[1].im * 6.0;
  b_J_min = -(J_min - J_max);
  J_min_re = (d2 + d3) + A_init * A_init * J_min;
  l14[0].re =
      b_J_min * ((((((J_min_re + l7_tmp * l14[0].re) - b_l17_tmp) + l18_tmp) +
                   A_init_re_tmp_tmp * t1[0].re * 2.0) -
                  b_J_max * l14[0].re) -
                 l24[0].re);
  l14[0].im =
      b_J_min * (((l7_tmp * l14[0].im + A_init_re_tmp_tmp * t1[0].im * 2.0) -
                  b_J_max * l14[0].im) -
                 l24[0].im);
  l14[1].re = b_J_min * ((((((J_min_re + d) - b_l17_tmp) + l18_tmp) +
                           A_init_re_tmp_tmp * t1[1].re * 2.0) -
                          b_J_max * re) -
                         l24[1].re);
  l14[1].im =
      b_J_min *
      (((d1 + A_init_re_tmp_tmp * t1[1].im * 2.0) - b_J_max * im) - l24[1].im);
  coder::internal::scalar::b_sqrt(&l14[0]);
  coder::internal::scalar::b_sqrt(&l14[1]);
  y = l15_tmp - b_l15_tmp;
  if (l14[0].im == 0.0) {
    re = l14[0].re / y;
    im = 0.0;
  } else if (l14[0].re == 0.0) {
    re = 0.0;
    im = l14[0].im / y;
  } else {
    re = l14[0].re / y;
    im = l14[0].im / y;
  }
  t[0] = t1[0];
  t[2].re = 0.0;
  t[2].im = 0.0;
  l15 = -(A_init + J_min * t1[0].re);
  l23 = -(J_min * t1[0].im);
  if (l23 == 0.0) {
    t[4].re = l15 / J_max;
    t[4].im = 0.0;
  } else if (l15 == 0.0) {
    t[4].re = 0.0;
    t[4].im = l23 / J_max;
  } else {
    t[4].re = l15 / J_max;
    t[4].im = l23 / J_max;
  }
  if (l28[0].im == 0.0) {
    if (b_l15[0].im == 0.0) {
      t[6].re = b_l15[0].re / l28[0].re;
      t[6].im = 0.0;
    } else if (b_l15[0].re == 0.0) {
      t[6].re = 0.0;
      t[6].im = b_l15[0].im / l28[0].re;
    } else {
      t[6].re = b_l15[0].re / l28[0].re;
      t[6].im = b_l15[0].im / l28[0].re;
    }
  } else if (l28[0].re == 0.0) {
    if (b_l15[0].re == 0.0) {
      t[6].re = b_l15[0].im / l28[0].im;
      t[6].im = 0.0;
    } else if (b_l15[0].im == 0.0) {
      t[6].re = 0.0;
      t[6].im = -(b_l15[0].re / l28[0].im);
    } else {
      t[6].re = b_l15[0].im / l28[0].im;
      t[6].im = -(b_l15[0].re / l28[0].im);
    }
  } else {
    l31 = std::abs(l28[0].re);
    l15 = std::abs(l28[0].im);
    if (l31 > l15) {
      l15 = l28[0].im / l28[0].re;
      l23 = l28[0].re + l15 * l28[0].im;
      t[6].re = (b_l15[0].re + l15 * b_l15[0].im) / l23;
      t[6].im = (b_l15[0].im - l15 * b_l15[0].re) / l23;
    } else if (l15 == l31) {
      if (l28[0].re > 0.0) {
        l15 = 0.5;
      } else {
        l15 = -0.5;
      }
      if (l28[0].im > 0.0) {
        l23 = 0.5;
      } else {
        l23 = -0.5;
      }
      t[6].re = (b_l15[0].re * l15 + b_l15[0].im * l23) / l31;
      t[6].im = (b_l15[0].im * l15 - b_l15[0].re * l23) / l31;
    } else {
      l15 = l28[0].re / l28[0].im;
      l23 = l28[0].im + l15 * l28[0].re;
      t[6].re = (l15 * b_l15[0].re + b_l15[0].im) / l23;
      t[6].im = (l15 * b_l15[0].im - b_l15[0].re) / l23;
    }
  }
  t[8].re = re;
  t[8].im = im;
  t[10].re = 0.0;
  t[10].im = 0.0;
  l15 = A_wayp - J_min * re;
  l23 = 0.0 - J_min * im;
  if (l23 == 0.0) {
    t[12].re = l15 / J_max;
    t[12].im = 0.0;
  } else if (l15 == 0.0) {
    t[12].re = 0.0;
    t[12].im = l23 / J_max;
  } else {
    t[12].re = l15 / J_max;
    t[12].im = l23 / J_max;
  }
  if (l14[1].im == 0.0) {
    re = l14[1].re / y;
    im = 0.0;
  } else if (l14[1].re == 0.0) {
    re = 0.0;
    im = l14[1].im / y;
  } else {
    re = l14[1].re / y;
    im = l14[1].im / y;
  }
  t[1] = t1[1];
  t[3].re = 0.0;
  t[3].im = 0.0;
  l15 = -(A_init + J_min * t1[1].re);
  l23 = -(J_min * t1[1].im);
  if (l23 == 0.0) {
    t[5].re = l15 / J_max;
    t[5].im = 0.0;
  } else if (l15 == 0.0) {
    t[5].re = 0.0;
    t[5].im = l23 / J_max;
  } else {
    t[5].re = l15 / J_max;
    t[5].im = l23 / J_max;
  }
  if (l28[1].im == 0.0) {
    if (b_l15[1].im == 0.0) {
      t[7].re = b_l15[1].re / l28[1].re;
      t[7].im = 0.0;
    } else if (b_l15[1].re == 0.0) {
      t[7].re = 0.0;
      t[7].im = b_l15[1].im / l28[1].re;
    } else {
      t[7].re = b_l15[1].re / l28[1].re;
      t[7].im = b_l15[1].im / l28[1].re;
    }
  } else if (l28[1].re == 0.0) {
    if (b_l15[1].re == 0.0) {
      t[7].re = b_l15[1].im / l28[1].im;
      t[7].im = 0.0;
    } else if (b_l15[1].im == 0.0) {
      t[7].re = 0.0;
      t[7].im = -(b_l15[1].re / l28[1].im);
    } else {
      t[7].re = b_l15[1].im / l28[1].im;
      t[7].im = -(b_l15[1].re / l28[1].im);
    }
  } else {
    l31 = std::abs(l28[1].re);
    l15 = std::abs(l28[1].im);
    if (l31 > l15) {
      l15 = l28[1].im / l28[1].re;
      l23 = l28[1].re + l15 * l28[1].im;
      t[7].re = (b_l15[1].re + l15 * b_l15[1].im) / l23;
      t[7].im = (b_l15[1].im - l15 * b_l15[1].re) / l23;
    } else if (l15 == l31) {
      if (l28[1].re > 0.0) {
        l15 = 0.5;
      } else {
        l15 = -0.5;
      }
      if (l28[1].im > 0.0) {
        l23 = 0.5;
      } else {
        l23 = -0.5;
      }
      t[7].re = (b_l15[1].re * l15 + b_l15[1].im * l23) / l31;
      t[7].im = (b_l15[1].im * l15 - b_l15[1].re * l23) / l31;
    } else {
      l15 = l28[1].re / l28[1].im;
      l23 = l28[1].im + l15 * l28[1].re;
      t[7].re = (l15 * b_l15[1].re + b_l15[1].im) / l23;
      t[7].im = (l15 * b_l15[1].im - b_l15[1].re) / l23;
    }
  }
  t[9].re = re;
  t[9].im = im;
  t[11].re = 0.0;
  t[11].im = 0.0;
  l15 = A_wayp - J_min * re;
  l23 = 0.0 - J_min * im;
  if (l23 == 0.0) {
    t[13].re = l15 / J_max;
    t[13].im = 0.0;
  } else if (l15 == 0.0) {
    t[13].re = 0.0;
    t[13].im = l23 / J_max;
  } else {
    t[13].re = l15 / J_max;
    t[13].im = l23 / J_max;
  }
}

// End of code generation (acdeg_NO_AP.cpp)
