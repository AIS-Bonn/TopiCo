//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// acdefg_NO_AP.cpp
//
// Code generation for function 'acdefg_NO_AP'
//

// Include files
#include "acdefg_NO_AP.h"
#include "power.h"
#include "rt_nonfinite.h"
#include "sqrt.h"
#include "topico_wrapper_data.h"
#include "topico_wrapper_rtwutil.h"
#include "topico_wrapper_types.h"
#include <cmath>

// Function Definitions
void acdefg_NO_AP(double P_init, double V_init, double A_init, double P_wayp,
                  double A_wayp, double V_max, double A_min, double J_max,
                  double J_min, creal_T t[14])
{
  creal_T b_l15[2];
  creal_T b_y[2];
  creal_T l14[2];
  creal_T l16[2];
  creal_T t1[2];
  creal_T l20;
  double A_init_tmp;
  double A_min_re;
  double A_min_tmp;
  double J_max_re;
  double J_max_re_tmp;
  double J_min_re;
  double b_A_init;
  double b_A_init_tmp;
  double b_A_min;
  double b_A_min_re;
  double b_A_min_tmp;
  double b_J_max;
  double b_J_max_re;
  double b_J_min_re;
  double b_V_init;
  double b_im;
  double b_l11;
  double b_l15_tmp;
  double b_l2;
  double b_l20_tmp;
  double b_l3;
  double b_l4;
  double b_y_re;
  double c_A_init;
  double c_A_min;
  double c_A_min_re;
  double c_A_min_tmp;
  double c_J_max;
  double c_J_max_re;
  double c_J_min_re;
  double d_A_init;
  double d_A_min;
  double d_A_min_re;
  double d_J_max;
  double d_J_min_re;
  double e_A_init;
  double e_A_min_re;
  double e_J_max;
  double e_J_min_re;
  double f_A_init;
  double f_J_max;
  double g_A_init;
  double h_A_init;
  double im;
  double l10;
  double l11;
  double l12;
  double l13;
  double l15;
  double l15_tmp;
  double l2;
  double l20_tmp;
  double l3;
  double l4;
  double l4_re;
  double l4_tmp;
  double l5_tmp;
  double l6;
  double l6_tmp;
  double l7;
  double l8;
  double re;
  double y;
  double y_re;
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
  //  Generated on 28-Aug-2019 17:25:45
  l4 = A_init * J_min;
  l5_tmp = A_init * J_max;
  if (J_max < 0.0) {
    f_rtErrorWithMessageID("sqrt", n_emlrtRTEI.fName, n_emlrtRTEI.lineNo);
  }
  l15_tmp = J_min * J_min;
  b_l15_tmp = J_min * J_max;
  l15 = 1.0 / (l15_tmp + -b_l15_tmp);
  l20_tmp = A_init * A_init;
  l6 = J_min * V_init;
  b_l20_tmp = J_min * V_max;
  l20.re = -((J_min + -J_max) * ((l20_tmp + b_l20_tmp * 2.0) + -(l6 * 2.0)));
  l20.im = 0.0;
  coder::internal::scalar::b_sqrt(&l20);
  l8 = std::sqrt(J_max);
  l20.re *= l8;
  l20.im *= l8;
  t1[0].re = l15 * ((-l4 + l5_tmp) + l20.re);
  t1[0].im = l15 * l20.im;
  t1[1].re = -l15 * ((l4 - l5_tmp) + l20.re);
  t1[1].im = -l15 * l20.im;
  l3 = rt_powd_snf(A_init, 3.0);
  l4_tmp = A_min * A_min;
  l6_tmp = A_wayp * A_wayp;
  l7 = rt_powd_snf(A_wayp, 3.0);
  l10 = rt_powd_snf(J_min, 3.0);
  l12 = J_max * J_max;
  l13 = rt_powd_snf(J_min, 5.0);
  coder::power(t1, b_l15);
  l15 = l4_tmp * l4_tmp;
  l8 = l6_tmp * l6_tmp;
  l11 = l15_tmp * l15_tmp;
  y = rt_powd_snf(l15_tmp, 3.0);
  l20.re =
      (((l15 * l15_tmp - l15 * l12) - l8 * l15_tmp * 3.0) - l8 * l12 * 3.0) +
      l20_tmp * l20_tmp * l15_tmp * 3.0;
  J_min_re = b_l15_tmp * l8 * 6.0;
  b_A_init = A_init * l13;
  A_min_re = A_min * l3 * l15_tmp * 8.0;
  b_A_min_re = A_min * l7 * l15_tmp * 8.0;
  b_A_min = A_min * l13;
  b_J_max = J_max * l13;
  l4_re = l4_tmp * l6_tmp * l15_tmp * 6.0;
  l2 = l20_tmp * l11;
  b_l11 = l11 * l12;
  b_l3 = l3 * l10;
  y_re = V_init * V_init * l15_tmp * l12 * 12.0;
  b_y_re = V_max * V_max * l15_tmp * l12 * 12.0;
  c_J_max = J_max * l20_tmp * l10;
  d_J_max = J_max * l4_tmp * l10;
  e_J_max = J_max * l3 * l15_tmp;
  b_V_init = V_init * l10 * l12;
  b_l2 = l20_tmp * l15_tmp * l12;
  b_l4 = l4_tmp * l15_tmp * l12;
  c_A_min_re = A_min * J_min * J_max * l7 * 12.0;
  A_init_tmp = A_init * A_min;
  c_A_init = A_init_tmp * l11;
  d_A_init = l5_tmp * l11;
  A_min_tmp = A_min * J_max;
  b_A_min_tmp = A_min_tmp * l11;
  d_A_min_re = A_min * P_init * l15_tmp * l12 * 24.0;
  e_A_min_re = A_min * P_wayp * l15_tmp * l12 * 24.0;
  b_J_min_re = b_l15_tmp * l20_tmp * l4_tmp * 6.0;
  c_J_min_re = b_l15_tmp * l4_tmp * l6_tmp * 6.0;
  J_max_re_tmp = J_max * V_init;
  J_max_re = J_max_re_tmp * l20_tmp * l15_tmp * 12.0;
  d_J_min_re = l6 * l4_tmp * l12 * 12.0;
  f_J_max = J_max_re_tmp * l11;
  J_max_re_tmp = J_max * V_max;
  b_J_max_re = J_max_re_tmp * l4_tmp * l15_tmp * 12.0;
  c_J_max_re = J_max_re_tmp * l6_tmp * l15_tmp * 12.0;
  e_J_min_re = b_l20_tmp * l6_tmp * l12 * 12.0;
  e_A_init = A_init * l10 * l12;
  c_A_min_tmp = A_min * l10 * l12;
  c_A_min = A_min * l20_tmp * l10;
  l15 = A_init_tmp * J_max;
  b_A_init_tmp = l15 * l10;
  f_A_init = l5_tmp * V_init * l10;
  d_A_min = A_min_tmp * V_init * l10;
  A_init_tmp = A_init_tmp * l15_tmp * l12;
  g_A_init = l5_tmp * l4_tmp * l15_tmp;
  h_A_init = l4 * l4_tmp * l12;
  A_min_tmp = A_min_tmp * l20_tmp * l15_tmp;
  l20_tmp = A_init * V_init * l15_tmp * l12;
  l12 *= A_min * V_init * l15_tmp;
  b_l20_tmp = l15 * V_init * l15_tmp * 24.0;
  l11 = A_min * A_wayp * J_max * V_max * l15_tmp * 24.0;
  l3 = 1.0 / J_min;
  l4 = 1.0 / J_max;
  l5_tmp = A_min + -A_wayp;
  l15_tmp = t1[0].re * t1[0].re - t1[0].im * t1[0].im;
  l15 = t1[0].re * t1[0].im;
  im = l15 + l15;
  l14[0].re = l15_tmp;
  l14[0].im = im;
  re = l15_tmp * l15_tmp - im * im;
  l15 = l15_tmp * im;
  b_im = l15 + l15;
  l8 = ((((((((((((((((((((((((((((((((((((((((((((((l20.re + y * re * 3.0) +
                                                    J_min_re) +
                                                   b_A_init * b_l15[0].re *
                                                       12.0) -
                                                  A_min_re) +
                                                 b_A_min_re) -
                                                b_A_min * b_l15[0].re * 8.0) -
                                               b_J_max * re * 6.0) -
                                              l4_re) +
                                             l2 * l15_tmp * 18.0) +
                                            b_l11 * re * 3.0) +
                                           b_l3 * t1[0].re * 12.0) +
                                          y_re) -
                                         b_y_re) -
                                        c_J_max * l15_tmp * 30.0) +
                                       d_J_max * l15_tmp * 6.0) -
                                      e_J_max * t1[0].re * 12.0) +
                                     b_V_init * l15_tmp * 12.0) +
                                    b_l2 * l15_tmp * 12.0) -
                                   b_l4 * l15_tmp * 6.0) -
                                  c_A_min_re) -
                                 c_A_init * l15_tmp * 24.0) -
                                d_A_init * b_l15[0].re * 24.0) +
                               b_A_min_tmp * b_l15[0].re * 12.0) -
                              d_A_min_re) +
                             e_A_min_re) +
                            b_J_min_re) +
                           c_J_min_re) -
                          J_max_re) -
                         d_J_min_re) -
                        f_J_max * l15_tmp * 12.0) +
                       b_J_max_re) +
                      c_J_max_re) -
                     e_J_min_re) +
                    e_A_init * b_l15[0].re * 12.0) -
                   c_A_min_tmp * b_l15[0].re * 4.0) -
                  c_A_min * t1[0].re * 24.0) +
                 b_A_init_tmp * l15_tmp * 36.0) -
                f_A_init * t1[0].re * 24.0) +
               d_A_min * t1[0].re * 24.0) -
              A_init_tmp * l15_tmp * 12.0) +
             g_A_init * t1[0].re * 12.0) -
            h_A_init * t1[0].re * 12.0) +
           A_min_tmp * t1[0].re * 24.0) +
          l20_tmp * t1[0].re * 24.0) -
         l12 * t1[0].re * 24.0) +
        b_l20_tmp) -
       l11;
  l15 = (((((((((((((((((((((((((((y * b_im * 3.0 +
                                   b_A_init * b_l15[0].im * 12.0) -
                                  b_A_min * b_l15[0].im * 8.0) -
                                 b_J_max * b_im * 6.0) +
                                l2 * im * 18.0) +
                               b_l11 * b_im * 3.0) +
                              b_l3 * t1[0].im * 12.0) -
                             c_J_max * im * 30.0) +
                            d_J_max * im * 6.0) -
                           e_J_max * t1[0].im * 12.0) +
                          b_V_init * im * 12.0) +
                         b_l2 * im * 12.0) -
                        b_l4 * im * 6.0) -
                       c_A_init * im * 24.0) -
                      d_A_init * b_l15[0].im * 24.0) +
                     b_A_min_tmp * b_l15[0].im * 12.0) -
                    f_J_max * im * 12.0) +
                   e_A_init * b_l15[0].im * 12.0) -
                  c_A_min_tmp * b_l15[0].im * 4.0) -
                 c_A_min * t1[0].im * 24.0) +
                b_A_init_tmp * im * 36.0) -
               f_A_init * t1[0].im * 24.0) +
              d_A_min * t1[0].im * 24.0) -
             A_init_tmp * im * 12.0) +
            g_A_init * t1[0].im * 12.0) -
           h_A_init * t1[0].im * 12.0) +
          A_min_tmp * t1[0].im * 24.0) +
         l20_tmp * t1[0].im * 24.0) -
        l12 * t1[0].im * 24.0;
  l16[0].re = -0.083333333333333329 * l8;
  l16[0].im = -0.083333333333333329 * l15;
  l15_tmp = J_min * t1[0].re;
  l13 = J_min * t1[0].im;
  b_l15[0].re = l15_tmp;
  b_l15[0].im = l13;
  l15_tmp += A_init;
  b_y[0].re = l15_tmp * l15_tmp - l13 * l13;
  l10 = l15_tmp * l13;
  l15_tmp = t1[1].re * t1[1].re - t1[1].im * t1[1].im;
  l15 = t1[1].re * t1[1].im;
  im = l15 + l15;
  re = l15_tmp * l15_tmp - im * im;
  l15 = l15_tmp * im;
  b_im = l15 + l15;
  l7 = y * re;
  l6 = y * b_im;
  J_max_re_tmp = b_J_max * re;
  l8 = b_J_max * b_im;
  l13 = b_l11 * re;
  l15 = b_l11 * b_im;
  re = -0.083333333333333329 *
       (((((((((((((((((((((((((((((((((((((((((((((((l20.re + l7 * 3.0) +
                                                     J_min_re) +
                                                    b_A_init * b_l15[1].re *
                                                        12.0) -
                                                   A_min_re) +
                                                  b_A_min_re) -
                                                 b_A_min * b_l15[1].re * 8.0) -
                                                J_max_re_tmp * 6.0) -
                                               l4_re) +
                                              l2 * l15_tmp * 18.0) +
                                             l13 * 3.0) +
                                            b_l3 * t1[1].re * 12.0) +
                                           y_re) -
                                          b_y_re) -
                                         c_J_max * l15_tmp * 30.0) +
                                        d_J_max * l15_tmp * 6.0) -
                                       e_J_max * t1[1].re * 12.0) +
                                      b_V_init * l15_tmp * 12.0) +
                                     b_l2 * l15_tmp * 12.0) -
                                    b_l4 * l15_tmp * 6.0) -
                                   c_A_min_re) -
                                  c_A_init * l15_tmp * 24.0) -
                                 d_A_init * b_l15[1].re * 24.0) +
                                b_A_min_tmp * b_l15[1].re * 12.0) -
                               d_A_min_re) +
                              e_A_min_re) +
                             b_J_min_re) +
                            c_J_min_re) -
                           J_max_re) -
                          d_J_min_re) -
                         f_J_max * l15_tmp * 12.0) +
                        b_J_max_re) +
                       c_J_max_re) -
                      e_J_min_re) +
                     e_A_init * b_l15[1].re * 12.0) -
                    c_A_min_tmp * b_l15[1].re * 4.0) -
                   c_A_min * t1[1].re * 24.0) +
                  b_A_init_tmp * l15_tmp * 36.0) -
                 f_A_init * t1[1].re * 24.0) +
                d_A_min * t1[1].re * 24.0) -
               A_init_tmp * l15_tmp * 12.0) +
              g_A_init * t1[1].re * 12.0) -
             h_A_init * t1[1].re * 12.0) +
            A_min_tmp * t1[1].re * 24.0) +
           l20_tmp * t1[1].re * 24.0) -
          l12 * t1[1].re * 24.0) +
         b_l20_tmp) -
        l11);
  b_im = -0.083333333333333329 *
         ((((((((((((((((((((((((((((l6 * 3.0 + b_A_init * b_l15[1].im * 12.0) -
                                    b_A_min * b_l15[1].im * 8.0) -
                                   l8 * 6.0) +
                                  l2 * im * 18.0) +
                                 l15 * 3.0) +
                                b_l3 * t1[1].im * 12.0) -
                               c_J_max * im * 30.0) +
                              d_J_max * im * 6.0) -
                             e_J_max * t1[1].im * 12.0) +
                            b_V_init * im * 12.0) +
                           b_l2 * im * 12.0) -
                          b_l4 * im * 6.0) -
                         c_A_init * im * 24.0) -
                        d_A_init * b_l15[1].im * 24.0) +
                       b_A_min_tmp * b_l15[1].im * 12.0) -
                      f_J_max * im * 12.0) +
                     e_A_init * b_l15[1].im * 12.0) -
                    c_A_min_tmp * b_l15[1].im * 4.0) -
                   c_A_min * t1[1].im * 24.0) +
                  b_A_init_tmp * im * 36.0) -
                 f_A_init * t1[1].im * 24.0) +
                d_A_min * t1[1].im * 24.0) -
               A_init_tmp * im * 12.0) +
              g_A_init * t1[1].im * 12.0) -
             h_A_init * t1[1].im * 12.0) +
            A_min_tmp * t1[1].im * 24.0) +
           l20_tmp * t1[1].im * 24.0) -
          l12 * t1[1].im * 24.0);
  l16[1].re =
      -0.083333333333333329 *
      (((((((((((((((((((((((((((((((((((((((((((((((l20.re + l7 * 3.0) +
                                                    J_min_re) +
                                                   b_A_init * b_l15[1].re *
                                                       12.0) -
                                                  A_min_re) +
                                                 b_A_min_re) -
                                                b_A_min * b_l15[1].re * 8.0) -
                                               J_max_re_tmp * 6.0) -
                                              l4_re) +
                                             l2 * l15_tmp * 18.0) +
                                            l13 * 3.0) +
                                           b_l3 * t1[1].re * 12.0) +
                                          y_re) -
                                         b_y_re) -
                                        c_J_max * l15_tmp * 30.0) +
                                       d_J_max * l15_tmp * 6.0) -
                                      e_J_max * t1[1].re * 12.0) +
                                     b_V_init * l15_tmp * 12.0) +
                                    b_l2 * l15_tmp * 12.0) -
                                   b_l4 * l15_tmp * 6.0) -
                                  c_A_min_re) -
                                 c_A_init * l15_tmp * 24.0) -
                                d_A_init * b_l15[1].re * 24.0) +
                               b_A_min_tmp * b_l15[1].re * 12.0) -
                              d_A_min_re) +
                             e_A_min_re) +
                            b_J_min_re) +
                           c_J_min_re) -
                          J_max_re) -
                         d_J_min_re) -
                        f_J_max * l15_tmp * 12.0) +
                       b_J_max_re) +
                      c_J_max_re) -
                     e_J_min_re) +
                    e_A_init * b_l15[1].re * 12.0) -
                   c_A_min_tmp * b_l15[1].re * 4.0) -
                  c_A_min * t1[1].re * 24.0) +
                 b_A_init_tmp * l15_tmp * 36.0) -
                f_A_init * t1[1].re * 24.0) +
               d_A_min * t1[1].re * 24.0) -
              A_init_tmp * l15_tmp * 12.0) +
             g_A_init * t1[1].re * 12.0) -
            h_A_init * t1[1].re * 12.0) +
           A_min_tmp * t1[1].re * 24.0) +
          l20_tmp * t1[1].re * 24.0) -
         l12 * t1[1].re * 24.0) +
        b_l20_tmp) -
       l11);
  l16[1].im =
      -0.083333333333333329 *
      ((((((((((((((((((((((((((((l6 * 3.0 + b_A_init * b_l15[1].im * 12.0) -
                                 b_A_min * b_l15[1].im * 8.0) -
                                l8 * 6.0) +
                               l2 * im * 18.0) +
                              l15 * 3.0) +
                             b_l3 * t1[1].im * 12.0) -
                            c_J_max * im * 30.0) +
                           d_J_max * im * 6.0) -
                          e_J_max * t1[1].im * 12.0) +
                         b_V_init * im * 12.0) +
                        b_l2 * im * 12.0) -
                       b_l4 * im * 6.0) -
                      c_A_init * im * 24.0) -
                     d_A_init * b_l15[1].im * 24.0) +
                    b_A_min_tmp * b_l15[1].im * 12.0) -
                   f_J_max * im * 12.0) +
                  e_A_init * b_l15[1].im * 12.0) -
                 c_A_min_tmp * b_l15[1].im * 4.0) -
                c_A_min * t1[1].im * 24.0) +
               b_A_init_tmp * im * 36.0) -
              f_A_init * t1[1].im * 24.0) +
             d_A_min * t1[1].im * 24.0) -
            A_init_tmp * im * 12.0) +
           g_A_init * t1[1].im * 12.0) -
          h_A_init * t1[1].im * 12.0) +
         A_min_tmp * t1[1].im * 24.0) +
        l20_tmp * t1[1].im * 24.0) -
       l12 * t1[1].im * 24.0);
  l13 = J_min * t1[1].im;
  l8 = A_init + J_min * t1[1].re;
  l7 = l8 * l13;
  b_l3 = l3 * l4;
  y_re = l6_tmp * J_max;
  l20.re = V_init * 2.0;
  b_y_re = l4_tmp * l3;
  l4_re = l4 * (l5_tmp * l5_tmp);
  A_min_re = A_min * l4 * l5_tmp * 2.0;
  J_min_re = b_l15_tmp * V_max * 2.0;
  y = A_min * 2.0;
  l6 = -(1.0 / J_max * l5_tmp);
  l3 = A_min * (1.0 / J_min);
  b_A_min_re = l12 * 2.0;
  b_y[0].re =
      b_l3 * (((y_re + J_min * b_y[0].re) -
               b_l15_tmp * (((((l20.re + A_init * t1[0].re * 2.0) + b_y_re) +
                              J_min * l14[0].re) +
                             l4_re) -
                            A_min_re)) +
              J_min_re);
  b_y[0].im =
      b_l3 * (J_min * (l10 + l10) -
              b_l15_tmp * (A_init * t1[0].im * 2.0 + J_min * l14[0].im));
  t[0] = t1[0];
  t[2].re = 0.0;
  t[2].im = 0.0;
  l15 = -(A_init + b_l15[0].re);
  if (-b_l15[0].im == 0.0) {
    t[4].re = l15 / J_max;
    t[4].im = 0.0;
  } else if (l15 == 0.0) {
    t[4].re = 0.0;
    t[4].im = -b_l15[0].im / J_max;
  } else {
    t[4].re = l15 / J_max;
    t[4].im = -b_l15[0].im / J_max;
  }
  b_y[1].re =
      b_l3 * (((y_re + J_min * (l8 * l8 - l13 * l13)) -
               b_l15_tmp * (((((l20.re + A_init * t1[1].re * 2.0) + b_y_re) +
                              J_min * l15_tmp) +
                             l4_re) -
                            A_min_re)) +
              J_min_re);
  b_y[1].im = b_l3 * (J_min * (l7 + l7) -
                      b_l15_tmp * (A_init * t1[1].im * 2.0 + J_min * im));
  t[1] = t1[1];
  t[3].re = 0.0;
  t[3].im = 0.0;
  if (-l13 == 0.0) {
    t[5].re = -l8 / J_max;
    t[5].im = 0.0;
  } else if (-l8 == 0.0) {
    t[5].re = 0.0;
    t[5].im = -l13 / J_max;
  } else {
    t[5].re = -l8 / J_max;
    t[5].im = -l13 / J_max;
  }
  t[8].re = l3;
  t[8].im = 0.0;
  t[9].re = l3;
  t[9].im = 0.0;
  l13 = ((((A_min_tmp + b_A_min_tmp * l14[0].re) - b_A_min_re) -
          c_A_min_tmp * l14[0].re) +
         b_A_init_tmp * t1[0].re * 2.0) -
        A_init_tmp * t1[0].re * 2.0;
  l7 = ((b_A_min_tmp * l14[0].im - c_A_min_tmp * l14[0].im) +
        b_A_init_tmp * t1[0].im * 2.0) -
       A_init_tmp * t1[0].im * 2.0;
  if (l7 == 0.0) {
    if (l16[0].im == 0.0) {
      t[6].re = l16[0].re / l13;
      t[6].im = 0.0;
    } else if (l16[0].re == 0.0) {
      t[6].re = 0.0;
      t[6].im = l16[0].im / l13;
    } else {
      t[6].re = l16[0].re / l13;
      t[6].im = l16[0].im / l13;
    }
  } else if (l13 == 0.0) {
    if (l16[0].re == 0.0) {
      t[6].re = l16[0].im / l7;
      t[6].im = 0.0;
    } else if (l16[0].im == 0.0) {
      t[6].re = 0.0;
      t[6].im = -(l16[0].re / l7);
    } else {
      t[6].re = l16[0].im / l7;
      t[6].im = -(l16[0].re / l7);
    }
  } else {
    l11 = std::abs(l13);
    l15 = std::abs(l7);
    if (l11 > l15) {
      l8 = l7 / l13;
      l15 = l13 + l8 * l7;
      t[6].re = (l16[0].re + l8 * l16[0].im) / l15;
      t[6].im = (l16[0].im - l8 * l16[0].re) / l15;
    } else if (l15 == l11) {
      if (l13 > 0.0) {
        l8 = 0.5;
      } else {
        l8 = -0.5;
      }
      if (l7 > 0.0) {
        l15 = 0.5;
      } else {
        l15 = -0.5;
      }
      t[6].re = (l16[0].re * l8 + l16[0].im * l15) / l11;
      t[6].im = (l16[0].im * l8 - l16[0].re * l15) / l11;
    } else {
      l8 = l13 / l7;
      l15 = l7 + l8 * l13;
      t[6].re = (l8 * l16[0].re + l16[0].im) / l15;
      t[6].im = (l8 * l16[0].im - l16[0].re) / l15;
    }
  }
  if (b_y[0].im == 0.0) {
    t[10].re = b_y[0].re / y;
    t[10].im = 0.0;
  } else if (b_y[0].re == 0.0) {
    t[10].re = 0.0;
    t[10].im = b_y[0].im / y;
  } else {
    t[10].re = b_y[0].re / y;
    t[10].im = b_y[0].im / y;
  }
  l13 = ((((A_min_tmp + b_A_min_tmp * l15_tmp) - b_A_min_re) -
          c_A_min_tmp * l15_tmp) +
         b_A_init_tmp * t1[1].re * 2.0) -
        A_init_tmp * t1[1].re * 2.0;
  l7 = ((b_A_min_tmp * im - c_A_min_tmp * im) + b_A_init_tmp * t1[1].im * 2.0) -
       A_init_tmp * t1[1].im * 2.0;
  if (l7 == 0.0) {
    if (l16[1].im == 0.0) {
      t[7].re = l16[1].re / l13;
      t[7].im = 0.0;
    } else if (l16[1].re == 0.0) {
      t[7].re = 0.0;
      t[7].im = l16[1].im / l13;
    } else {
      t[7].re = l16[1].re / l13;
      t[7].im = l16[1].im / l13;
    }
  } else if (l13 == 0.0) {
    if (l16[1].re == 0.0) {
      t[7].re = l16[1].im / l7;
      t[7].im = 0.0;
    } else if (l16[1].im == 0.0) {
      t[7].re = 0.0;
      t[7].im = -(l16[1].re / l7);
    } else {
      t[7].re = l16[1].im / l7;
      t[7].im = -(l16[1].re / l7);
    }
  } else {
    l11 = std::abs(l13);
    l15 = std::abs(l7);
    if (l11 > l15) {
      l8 = l7 / l13;
      l15 = l13 + l8 * l7;
      t[7].re = (re + l8 * b_im) / l15;
      t[7].im = (b_im - l8 * re) / l15;
    } else if (l15 == l11) {
      if (l13 > 0.0) {
        l8 = 0.5;
      } else {
        l8 = -0.5;
      }
      if (l7 > 0.0) {
        l15 = 0.5;
      } else {
        l15 = -0.5;
      }
      t[7].re = (re * l8 + b_im * l15) / l11;
      t[7].im = (b_im * l8 - re * l15) / l11;
    } else {
      l8 = l13 / l7;
      l15 = l7 + l8 * l13;
      t[7].re = (l8 * re + b_im) / l15;
      t[7].im = (l8 * b_im - re) / l15;
    }
  }
  if (b_y[1].im == 0.0) {
    t[11].re = b_y[1].re / y;
    t[11].im = 0.0;
  } else if (b_y[1].re == 0.0) {
    t[11].re = 0.0;
    t[11].im = b_y[1].im / y;
  } else {
    t[11].re = b_y[1].re / y;
    t[11].im = b_y[1].im / y;
  }
  t[12].re = l6;
  t[12].im = 0.0;
  t[13].re = l6;
  t[13].im = 0.0;
}

// End of code generation (acdefg_NO_AP.cpp)
