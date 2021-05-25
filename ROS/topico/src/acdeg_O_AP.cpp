//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// acdeg_O_AP.cpp
//
// Code generation for function 'acdeg_O_AP'
//

// Include files
#include "acdeg_O_AP.h"
#include "power.h"
#include "rt_nonfinite.h"
#include "sqrt.h"
#include "topico_wrapper_data.h"
#include "topico_wrapper_rtwutil.h"
#include "topico_wrapper_types.h"
#include <cmath>

// Function Definitions
void acdeg_O_AP(double P_init, double V_init, double A_init, double P_wayp,
                double A_wayp, double V_max, double J_max, double J_min,
                creal_T t[14])
{
  creal_T b_l14[2];
  creal_T l15[2];
  creal_T l24[2];
  creal_T l28[2];
  creal_T l33[2];
  creal_T l35[2];
  creal_T t1[2];
  creal_T l17;
  double A_init_re;
  double A_init_re_tmp;
  double A_init_re_tmp_tmp;
  double A_init_tmp;
  double A_wayp_re;
  double J_max_re;
  double J_min_re;
  double J_min_tmp;
  double P_init_re;
  double P_wayp_re;
  double V_init_tmp;
  double b_A_init;
  double b_A_init_re;
  double b_A_init_tmp;
  double b_A_wayp_re;
  double b_J_max;
  double b_J_min;
  double b_J_min_re;
  double b_J_min_tmp;
  double b_P_init_re;
  double b_P_wayp_re;
  double b_V_init_tmp;
  double b_l14_tmp;
  double b_l17_tmp;
  double b_l2_tmp;
  double b_l4_tmp;
  double b_l6_tmp;
  double b_l8;
  double c_A_init_re;
  double c_A_init_tmp;
  double c_A_wayp_re;
  double c_J_min;
  double c_J_min_tmp;
  double c_P_init_re;
  double c_P_wayp_re;
  double c_V_init_tmp;
  double c_l17_tmp;
  double d;
  double d1;
  double d2;
  double d3;
  double im;
  double l10_tmp;
  double l11;
  double l12;
  double l13;
  double l14;
  double l14_tmp;
  double l17_tmp;
  double l18_tmp;
  double l19;
  double l2;
  double l2_tmp;
  double l3;
  double l31;
  double l32;
  double l3_re;
  double l4_tmp;
  double l5;
  double l5_re;
  double l6_tmp;
  double l7;
  double l7_tmp;
  double l8;
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
  //  Generated on 05-Sep-2019 11:33:59
  l4_tmp = A_init * J_min;
  l5 = A_init * J_max;
  l14_tmp = J_max * J_max;
  b_l14_tmp = J_min * J_max;
  l14 = 1.0 / (l14_tmp + -b_l14_tmp);
  l17_tmp = A_init * A_init;
  l17.re = J_min * (J_min + -J_max) *
           ((l17_tmp + J_max * V_max * 2.0) + -(J_max * V_init * 2.0));
  l17.im = 0.0;
  coder::internal::scalar::b_sqrt(&l17);
  t1[0].re = l14 * ((l4_tmp - l5) + l17.re);
  t1[0].im = l14 * l17.im;
  t1[1].re = -l14 * ((-l4_tmp + l5) + l17.re);
  t1[1].im = -l14 * l17.im;
  l3 = rt_powd_snf(A_init, 3.0);
  b_l4_tmp = A_wayp * A_wayp;
  l5 = rt_powd_snf(A_wayp, 3.0);
  l6_tmp = J_min * J_min;
  l7 = rt_powd_snf(J_min, 3.0);
  l10_tmp = rt_powd_snf(J_max, 3.0);
  l12 = rt_powd_snf(J_max, 5.0);
  coder::power(t1, l15);
  b_l17_tmp = b_l14_tmp * V_init;
  c_l17_tmp = b_l17_tmp * 2.0;
  l18_tmp = b_l14_tmp * V_max * 2.0;
  l8 = l6_tmp * l6_tmp;
  l11 = l14_tmp * l14_tmp;
  l13 = rt_powd_snf(l14_tmp, 3.0);
  l19 = J_max * l17_tmp;
  l14 = J_max + -J_min;
  re = t1[0].re * t1[0].re - t1[0].im * t1[0].im;
  l31 = t1[0].re * t1[0].im;
  im = l31 + l31;
  b_l14[0].re = re;
  b_l14[0].im = im;
  A_init_re_tmp = l4_tmp * J_max;
  l24[0].re = 2.0 * (A_init_re_tmp * t1[0].re);
  l24[0].im = 2.0 * (A_init_re_tmp * t1[0].im);
  l28[0].re = l10_tmp * re;
  l28[0].im = l10_tmp * im;
  re = t1[1].re * t1[1].re - t1[1].im * t1[1].im;
  l31 = t1[1].re * t1[1].im;
  im = l31 + l31;
  l24[1].re = 2.0 * (A_init_re_tmp * t1[1].re);
  l24[1].im = 2.0 * (A_init_re_tmp * t1[1].im);
  d = l10_tmp * re;
  d1 = l10_tmp * im;
  l31 = rt_powd_snf(l14, 1.5);
  if (l14 < 0.0) {
    k_rtErrorWithMessageID(o_emlrtRTEI.fName, o_emlrtRTEI.lineNo);
  }
  l32 = rt_powd_snf(l14, 2.5);
  A_init_re_tmp_tmp = A_init * l14_tmp;
  d2 = b_l4_tmp * -J_min;
  d3 = J_max * b_l4_tmp;
  l14 = (((l18_tmp + l19) + d3) + -c_l17_tmp) + d2;
  l33[0].re =
      (((l14 + -l24[0].re) + A_init_re_tmp_tmp * t1[0].re * 2.0) + l28[0].re) +
      l14_tmp * b_l14[0].re * -J_min;
  l33[0].im = ((-l24[0].im + A_init_re_tmp_tmp * t1[0].im * 2.0) + l28[0].im) +
              l14_tmp * b_l14[0].im * -J_min;
  l33[1].re = (((l14 + -l24[1].re) + A_init_re_tmp_tmp * t1[1].re * 2.0) + d) +
              l14_tmp * re * -J_min;
  l33[1].im = ((-l24[1].im + A_init_re_tmp_tmp * t1[1].im * 2.0) + d1) +
              l14_tmp * im * -J_min;
  coder::internal::scalar::b_sqrt(&l33[0]);
  coder::internal::scalar::b_sqrt(&l33[1]);
  coder::power(l33, l35);
  y = rt_powd_snf(J_max, 7.0);
  l17.re = l5 * l8 * -2.0 + l3 * l11 * 2.0;
  b_A_init = A_init * l13;
  A_init_tmp = A_init * l7;
  J_min_re = J_min * l3 * l10_tmp * 4.0;
  J_max_re = J_max * l5 * l7 * 7.0;
  b_J_min_re = J_min * l5 * l10_tmp * 3.0;
  J_min_tmp = J_min * l13;
  b_J_min = J_min * l31;
  b_J_max = J_max * l31;
  P_init_re = P_init * l6_tmp * l11 * 6.0;
  b_P_init_re = P_init * l7 * l10_tmp * 12.0;
  c_P_init_re = P_init * l8 * l14_tmp * 6.0;
  P_wayp_re = P_wayp * l6_tmp * l11 * 6.0;
  b_P_wayp_re = P_wayp * l7 * l10_tmp * 12.0;
  c_P_wayp_re = P_wayp * l8 * l14_tmp * 6.0;
  l3_re = l3 * l6_tmp * l14_tmp * 2.0;
  l5_re = l5 * l6_tmp * l14_tmp * 8.0;
  b_l6_tmp = l6_tmp * l12;
  l7_tmp = l7 * l11;
  b_l8 = l8 * l10_tmp;
  l19 *= l32;
  b_J_min_tmp = J_min * l14_tmp;
  l2 = l17_tmp * l12;
  c_J_min_tmp = J_min * l17_tmp * l11;
  V_init_tmp = V_init * l6_tmp * l11;
  b_V_init_tmp = V_init * l7 * l10_tmp;
  c_V_init_tmp = V_init * l8 * l14_tmp;
  l2_tmp = l17_tmp * l6_tmp * l10_tmp;
  b_l2_tmp = l17_tmp * l7 * l14_tmp;
  A_init_re = l4_tmp * V_init * l11 * 6.0;
  A_wayp_re = A_wayp * J_max * V_max * l8 * 6.0;
  b_A_init_tmp = l4_tmp * l12;
  l14 = A_init * V_init;
  b_A_init_re = l14 * l6_tmp * l10_tmp * 12.0;
  c_A_init_re = l14 * l7 * l14_tmp * 6.0;
  l14 = A_wayp * V_max;
  b_A_wayp_re = l14 * l6_tmp * l10_tmp * 6.0;
  c_A_wayp_re = l14 * l7 * l14_tmp * 12.0;
  c_J_min = J_min * V_init * l12;
  c_A_init_tmp = A_init * l6_tmp * l11;
  l14_tmp *= A_init * l8;
  l12 = b_l17_tmp * l32;
  l11 = A_init_re_tmp_tmp * l32;
  l14 = A_init_re_tmp * l32;
  b_l4_tmp = l32 * l28[0].re;
  l13 = l32 * l28[0].im;
  l31 = l32 * (b_J_min_tmp * b_l14[0].re);
  l3 = l32 * (b_J_min_tmp * b_l14[0].im);
  l5 = l11 * l33[0].re;
  l17_tmp = l11 * l33[0].im;
  l4_tmp = l14 * l33[0].re;
  l7 = l14 * l33[0].im;
  l15[0].re = -(
      ((((((((((((((((((((((((((((((((((((((((((l17.re + y * l15[0].re * 2.0) +
                                               b_A_init * b_l14[0].re * 6.0) -
                                              A_init_tmp * l28[0].re * 15.0) -
                                             J_min_re) +
                                            J_max_re) +
                                           b_J_min_re) -
                                          J_min_tmp * l15[0].re * 7.0) +
                                         b_J_min * l35[0].re * 2.0) -
                                        b_J_max * l35[0].re) +
                                       P_init_re) -
                                      b_P_init_re) +
                                     c_P_init_re) -
                                    P_wayp_re) +
                                   b_P_wayp_re) -
                                  c_P_wayp_re) +
                                 l3_re) -
                                l5_re) +
                               b_l6_tmp * l15[0].re * 9.0) -
                              l7_tmp * l15[0].re * 5.0) +
                             b_l8 * l15[0].re) +
                            l19 * l33[0].re * 3.0) +
                           (b_l4_tmp * l33[0].re - l13 * l33[0].im) * 3.0) -
                          (l31 * l33[0].re - l3 * l33[0].im) * 3.0) +
                         l2 * t1[0].re * 6.0) -
                        c_J_min_tmp * t1[0].re * 18.0) +
                       V_init_tmp * t1[0].re * 18.0) -
                      b_V_init_tmp * t1[0].re * 18.0) +
                     c_V_init_tmp * t1[0].re * 6.0) +
                    l2_tmp * t1[0].re * 18.0) -
                   b_l2_tmp * t1[0].re * 6.0) -
                  A_init_re) +
                 A_wayp_re) -
                b_A_init_tmp * b_l14[0].re * 21.0) +
               b_A_init_re) -
              c_A_init_re) +
             b_A_wayp_re) -
            c_A_wayp_re) -
           c_J_min * t1[0].re * 6.0) +
          c_A_init_tmp * b_l14[0].re * 27.0) +
         l14_tmp * b_l14[0].re * 3.0) -
        l12 * l33[0].re * 6.0) +
       (l5 * t1[0].re - l17_tmp * t1[0].im) * 6.0) -
      (l4_tmp * t1[0].re - l7 * t1[0].im) * 6.0);
  l15[0].im = -(((((((((((((((((((((((((y * l15[0].im * 2.0 +
                                        b_A_init * b_l14[0].im * 6.0) -
                                       A_init_tmp * l28[0].im * 15.0) -
                                      J_min_tmp * l15[0].im * 7.0) +
                                     b_J_min * l35[0].im * 2.0) -
                                    b_J_max * l35[0].im) +
                                   b_l6_tmp * l15[0].im * 9.0) -
                                  l7_tmp * l15[0].im * 5.0) +
                                 b_l8 * l15[0].im) +
                                l19 * l33[0].im * 3.0) +
                               (b_l4_tmp * l33[0].im + l13 * l33[0].re) * 3.0) -
                              (l31 * l33[0].im + l3 * l33[0].re) * 3.0) +
                             l2 * t1[0].im * 6.0) -
                            c_J_min_tmp * t1[0].im * 18.0) +
                           V_init_tmp * t1[0].im * 18.0) -
                          b_V_init_tmp * t1[0].im * 18.0) +
                         c_V_init_tmp * t1[0].im * 6.0) +
                        l2_tmp * t1[0].im * 18.0) -
                       b_l2_tmp * t1[0].im * 6.0) -
                      b_A_init_tmp * b_l14[0].im * 21.0) -
                     c_J_min * t1[0].im * 6.0) +
                    c_A_init_tmp * b_l14[0].im * 27.0) +
                   l14_tmp * b_l14[0].im * 3.0) -
                  l12 * l33[0].im * 6.0) +
                 (l5 * t1[0].im + l17_tmp * t1[0].re) * 6.0) -
                (l4_tmp * t1[0].im + l7 * t1[0].re) * 6.0);
  b_l4_tmp = l32 * d;
  l13 = l32 * d1;
  l31 = l32 * (b_J_min_tmp * re);
  l3 = l32 * (b_J_min_tmp * im);
  l5 = l11 * l33[1].re;
  l17_tmp = l11 * l33[1].im;
  l4_tmp = l14 * l33[1].re;
  l7 = l14 * l33[1].im;
  l15[1].re = -(
      ((((((((((((((((((((((((((((((((((((((((((l17.re + y * l15[1].re * 2.0) +
                                               b_A_init * re * 6.0) -
                                              A_init_tmp * d * 15.0) -
                                             J_min_re) +
                                            J_max_re) +
                                           b_J_min_re) -
                                          J_min_tmp * l15[1].re * 7.0) +
                                         b_J_min * l35[1].re * 2.0) -
                                        b_J_max * l35[1].re) +
                                       P_init_re) -
                                      b_P_init_re) +
                                     c_P_init_re) -
                                    P_wayp_re) +
                                   b_P_wayp_re) -
                                  c_P_wayp_re) +
                                 l3_re) -
                                l5_re) +
                               b_l6_tmp * l15[1].re * 9.0) -
                              l7_tmp * l15[1].re * 5.0) +
                             b_l8 * l15[1].re) +
                            l19 * l33[1].re * 3.0) +
                           (b_l4_tmp * l33[1].re - l13 * l33[1].im) * 3.0) -
                          (l31 * l33[1].re - l3 * l33[1].im) * 3.0) +
                         l2 * t1[1].re * 6.0) -
                        c_J_min_tmp * t1[1].re * 18.0) +
                       V_init_tmp * t1[1].re * 18.0) -
                      b_V_init_tmp * t1[1].re * 18.0) +
                     c_V_init_tmp * t1[1].re * 6.0) +
                    l2_tmp * t1[1].re * 18.0) -
                   b_l2_tmp * t1[1].re * 6.0) -
                  A_init_re) +
                 A_wayp_re) -
                b_A_init_tmp * re * 21.0) +
               b_A_init_re) -
              c_A_init_re) +
             b_A_wayp_re) -
            c_A_wayp_re) -
           c_J_min * t1[1].re * 6.0) +
          c_A_init_tmp * re * 27.0) +
         l14_tmp * re * 3.0) -
        l12 * l33[1].re * 6.0) +
       (l5 * t1[1].re - l17_tmp * t1[1].im) * 6.0) -
      (l4_tmp * t1[1].re - l7 * t1[1].im) * 6.0);
  l15[1].im =
      -(((((((((((((((((((((((((y * l15[1].im * 2.0 + b_A_init * im * 6.0) -
                               A_init_tmp * d1 * 15.0) -
                              J_min_tmp * l15[1].im * 7.0) +
                             b_J_min * l35[1].im * 2.0) -
                            b_J_max * l35[1].im) +
                           b_l6_tmp * l15[1].im * 9.0) -
                          l7_tmp * l15[1].im * 5.0) +
                         b_l8 * l15[1].im) +
                        l19 * l33[1].im * 3.0) +
                       (b_l4_tmp * l33[1].im + l13 * l33[1].re) * 3.0) -
                      (l31 * l33[1].im + l3 * l33[1].re) * 3.0) +
                     l2 * t1[1].im * 6.0) -
                    c_J_min_tmp * t1[1].im * 18.0) +
                   V_init_tmp * t1[1].im * 18.0) -
                  b_V_init_tmp * t1[1].im * 18.0) +
                 c_V_init_tmp * t1[1].im * 6.0) +
                l2_tmp * t1[1].im * 18.0) -
               b_l2_tmp * t1[1].im * 6.0) -
              b_A_init_tmp * im * 21.0) -
             c_J_min * t1[1].im * 6.0) +
            c_A_init_tmp * im * 27.0) +
           l14_tmp * im * 3.0) -
          l12 * l33[1].im * 6.0) +
         (l5 * t1[1].im + l17_tmp * t1[1].re) * 6.0) -
        (l4_tmp * t1[1].im + l7 * t1[1].re) * 6.0);
  J_min_re = c_J_min_tmp * 3.0;
  l17.re = V_init_tmp * 6.0;
  J_max_re = b_V_init_tmp * 12.0;
  P_init_re = c_V_init_tmp * 6.0;
  b_P_init_re = l2_tmp * 6.0;
  c_P_init_re = b_l2_tmp * 3.0;
  b_A_init = A_init_tmp * l10_tmp;
  l28[0].re = ((((((((((((l8 * l28[0].re * 3.0 - J_min_re) -
                         J_min_tmp * b_l14[0].re * 3.0) +
                        l17.re) -
                       J_max_re) +
                      P_init_re) +
                     b_P_init_re) -
                    c_P_init_re) +
                   b_l6_tmp * b_l14[0].re * 9.0) -
                  l7_tmp * b_l14[0].re * 9.0) -
                 b_A_init_tmp * t1[0].re * 6.0) +
                c_A_init_tmp * t1[0].re * 18.0) -
               b_A_init * t1[0].re * 18.0) +
              l14_tmp * t1[0].re * 6.0;
  l28[0].im = ((((((l8 * l28[0].im * 3.0 - J_min_tmp * b_l14[0].im * 3.0) +
                   b_l6_tmp * b_l14[0].im * 9.0) -
                  l7_tmp * b_l14[0].im * 9.0) -
                 b_A_init_tmp * t1[0].im * 6.0) +
                c_A_init_tmp * t1[0].im * 18.0) -
               b_A_init * t1[0].im * 18.0) +
              l14_tmp * t1[0].im * 6.0;
  l28[1].re =
      ((((((((((((l8 * d * 3.0 - J_min_re) - J_min_tmp * re * 3.0) + l17.re) -
               J_max_re) +
              P_init_re) +
             b_P_init_re) -
            c_P_init_re) +
           b_l6_tmp * re * 9.0) -
          l7_tmp * re * 9.0) -
         b_A_init_tmp * t1[1].re * 6.0) +
        c_A_init_tmp * t1[1].re * 18.0) -
       b_A_init * t1[1].re * 18.0) +
      l14_tmp * t1[1].re * 6.0;
  l28[1].im =
      ((((((l8 * d1 * 3.0 - J_min_tmp * im * 3.0) + b_l6_tmp * im * 9.0) -
          l7_tmp * im * 9.0) -
         b_A_init_tmp * t1[1].im * 6.0) +
        c_A_init_tmp * t1[1].im * 18.0) -
       b_A_init * t1[1].im * 18.0) +
      l14_tmp * t1[1].im * 6.0;
  b_J_min = -(J_min - J_max);
  J_min_re = (d2 + d3) + A_init * A_init * J_max;
  b_l14[0].re =
      b_J_min *
      ((((((J_min_re + l10_tmp * b_l14[0].re) - c_l17_tmp) + l18_tmp) +
         A_init_re_tmp_tmp * t1[0].re * 2.0) -
        b_J_min_tmp * b_l14[0].re) -
       l24[0].re);
  b_l14[0].im =
      b_J_min * (((l10_tmp * b_l14[0].im + A_init_re_tmp_tmp * t1[0].im * 2.0) -
                  b_J_min_tmp * b_l14[0].im) -
                 l24[0].im);
  b_l14[1].re = b_J_min * ((((((J_min_re + d) - c_l17_tmp) + l18_tmp) +
                             A_init_re_tmp_tmp * t1[1].re * 2.0) -
                            b_J_min_tmp * re) -
                           l24[1].re);
  b_l14[1].im =
      b_J_min *
      (((d1 + A_init_re_tmp_tmp * t1[1].im * 2.0) - b_J_min_tmp * im) -
       l24[1].im);
  coder::internal::scalar::b_sqrt(&b_l14[0]);
  coder::internal::scalar::b_sqrt(&b_l14[1]);
  y = l6_tmp - b_l14_tmp;
  if (b_l14[0].im == 0.0) {
    re = b_l14[0].re / y;
    im = 0.0;
  } else if (b_l14[0].re == 0.0) {
    re = 0.0;
    im = b_l14[0].im / y;
  } else {
    re = b_l14[0].re / y;
    im = b_l14[0].im / y;
  }
  t[0] = t1[0];
  t[2].re = 0.0;
  t[2].im = 0.0;
  l14 = -(A_init + J_max * t1[0].re);
  l31 = -(J_max * t1[0].im);
  if (l31 == 0.0) {
    t[4].re = l14 / J_min;
    t[4].im = 0.0;
  } else if (l14 == 0.0) {
    t[4].re = 0.0;
    t[4].im = l31 / J_min;
  } else {
    t[4].re = l14 / J_min;
    t[4].im = l31 / J_min;
  }
  if (l28[0].im == 0.0) {
    if (l15[0].im == 0.0) {
      t[6].re = l15[0].re / l28[0].re;
      t[6].im = 0.0;
    } else if (l15[0].re == 0.0) {
      t[6].re = 0.0;
      t[6].im = l15[0].im / l28[0].re;
    } else {
      t[6].re = l15[0].re / l28[0].re;
      t[6].im = l15[0].im / l28[0].re;
    }
  } else if (l28[0].re == 0.0) {
    if (l15[0].re == 0.0) {
      t[6].re = l15[0].im / l28[0].im;
      t[6].im = 0.0;
    } else if (l15[0].im == 0.0) {
      t[6].re = 0.0;
      t[6].im = -(l15[0].re / l28[0].im);
    } else {
      t[6].re = l15[0].im / l28[0].im;
      t[6].im = -(l15[0].re / l28[0].im);
    }
  } else {
    b_l4_tmp = std::abs(l28[0].re);
    l14 = std::abs(l28[0].im);
    if (b_l4_tmp > l14) {
      l14 = l28[0].im / l28[0].re;
      l31 = l28[0].re + l14 * l28[0].im;
      t[6].re = (l15[0].re + l14 * l15[0].im) / l31;
      t[6].im = (l15[0].im - l14 * l15[0].re) / l31;
    } else if (l14 == b_l4_tmp) {
      if (l28[0].re > 0.0) {
        l31 = 0.5;
      } else {
        l31 = -0.5;
      }
      if (l28[0].im > 0.0) {
        l14 = 0.5;
      } else {
        l14 = -0.5;
      }
      t[6].re = (l15[0].re * l31 + l15[0].im * l14) / b_l4_tmp;
      t[6].im = (l15[0].im * l31 - l15[0].re * l14) / b_l4_tmp;
    } else {
      l14 = l28[0].re / l28[0].im;
      l31 = l28[0].im + l14 * l28[0].re;
      t[6].re = (l14 * l15[0].re + l15[0].im) / l31;
      t[6].im = (l14 * l15[0].im - l15[0].re) / l31;
    }
  }
  t[8].re = re;
  t[8].im = im;
  t[10].re = 0.0;
  t[10].im = 0.0;
  l14 = A_wayp - J_min * re;
  l31 = 0.0 - J_min * im;
  if (l31 == 0.0) {
    t[12].re = l14 / J_max;
    t[12].im = 0.0;
  } else if (l14 == 0.0) {
    t[12].re = 0.0;
    t[12].im = l31 / J_max;
  } else {
    t[12].re = l14 / J_max;
    t[12].im = l31 / J_max;
  }
  if (b_l14[1].im == 0.0) {
    re = b_l14[1].re / y;
    im = 0.0;
  } else if (b_l14[1].re == 0.0) {
    re = 0.0;
    im = b_l14[1].im / y;
  } else {
    re = b_l14[1].re / y;
    im = b_l14[1].im / y;
  }
  t[1] = t1[1];
  t[3].re = 0.0;
  t[3].im = 0.0;
  l14 = -(A_init + J_max * t1[1].re);
  l31 = -(J_max * t1[1].im);
  if (l31 == 0.0) {
    t[5].re = l14 / J_min;
    t[5].im = 0.0;
  } else if (l14 == 0.0) {
    t[5].re = 0.0;
    t[5].im = l31 / J_min;
  } else {
    t[5].re = l14 / J_min;
    t[5].im = l31 / J_min;
  }
  if (l28[1].im == 0.0) {
    if (l15[1].im == 0.0) {
      t[7].re = l15[1].re / l28[1].re;
      t[7].im = 0.0;
    } else if (l15[1].re == 0.0) {
      t[7].re = 0.0;
      t[7].im = l15[1].im / l28[1].re;
    } else {
      t[7].re = l15[1].re / l28[1].re;
      t[7].im = l15[1].im / l28[1].re;
    }
  } else if (l28[1].re == 0.0) {
    if (l15[1].re == 0.0) {
      t[7].re = l15[1].im / l28[1].im;
      t[7].im = 0.0;
    } else if (l15[1].im == 0.0) {
      t[7].re = 0.0;
      t[7].im = -(l15[1].re / l28[1].im);
    } else {
      t[7].re = l15[1].im / l28[1].im;
      t[7].im = -(l15[1].re / l28[1].im);
    }
  } else {
    b_l4_tmp = std::abs(l28[1].re);
    l14 = std::abs(l28[1].im);
    if (b_l4_tmp > l14) {
      l14 = l28[1].im / l28[1].re;
      l31 = l28[1].re + l14 * l28[1].im;
      t[7].re = (l15[1].re + l14 * l15[1].im) / l31;
      t[7].im = (l15[1].im - l14 * l15[1].re) / l31;
    } else if (l14 == b_l4_tmp) {
      if (l28[1].re > 0.0) {
        l31 = 0.5;
      } else {
        l31 = -0.5;
      }
      if (l28[1].im > 0.0) {
        l14 = 0.5;
      } else {
        l14 = -0.5;
      }
      t[7].re = (l15[1].re * l31 + l15[1].im * l14) / b_l4_tmp;
      t[7].im = (l15[1].im * l31 - l15[1].re * l14) / b_l4_tmp;
    } else {
      l14 = l28[1].re / l28[1].im;
      l31 = l28[1].im + l14 * l28[1].re;
      t[7].re = (l14 * l15[1].re + l15[1].im) / l31;
      t[7].im = (l14 * l15[1].im - l15[1].re) / l31;
    }
  }
  t[9].re = re;
  t[9].im = im;
  t[11].re = 0.0;
  t[11].im = 0.0;
  l14 = A_wayp - J_min * re;
  l31 = 0.0 - J_min * im;
  if (l31 == 0.0) {
    t[13].re = l14 / J_max;
    t[13].im = 0.0;
  } else if (l14 == 0.0) {
    t[13].re = 0.0;
    t[13].im = l31 / J_max;
  } else {
    t[13].re = l14 / J_max;
    t[13].im = l31 / J_max;
  }
}

// End of code generation (acdeg_O_AP.cpp)
