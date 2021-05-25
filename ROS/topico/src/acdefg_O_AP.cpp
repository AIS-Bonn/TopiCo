//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// acdefg_O_AP.cpp
//
// Code generation for function 'acdefg_O_AP'
//

// Include files
#include "acdefg_O_AP.h"
#include "power.h"
#include "rt_nonfinite.h"
#include "sqrt.h"
#include "topico_wrapper_data.h"
#include "topico_wrapper_rtwutil.h"
#include <cmath>

// Function Definitions
void acdefg_O_AP(double P_init, double V_init, double A_init, double P_wayp,
                 double A_wayp, double V_max, double A_min, double J_max,
                 double J_min, creal_T t[14])
{
  creal_T b_l14[2];
  creal_T b_y[2];
  creal_T l15[2];
  creal_T l16[2];
  creal_T t1[2];
  creal_T l17;
  double A_init_tmp;
  double A_min_re;
  double A_min_tmp;
  double J_max_re;
  double J_min_re;
  double J_min_re_tmp;
  double b_A_init;
  double b_A_init_tmp;
  double b_A_min;
  double b_A_min_re;
  double b_A_min_tmp;
  double b_J_max_re;
  double b_J_min;
  double b_J_min_re;
  double b_V_init;
  double b_im;
  double b_l14_tmp;
  double b_l2;
  double b_l3;
  double b_l4;
  double b_l4_tmp;
  double b_l9;
  double b_y_re;
  double c_A_init;
  double c_A_min;
  double c_A_min_re;
  double c_A_min_tmp;
  double c_J_min;
  double c_J_min_re;
  double d_A_init;
  double d_A_min;
  double d_A_min_re;
  double d_A_min_tmp;
  double d_J_min;
  double d_J_min_re;
  double e_A_init;
  double e_A_min_re;
  double e_J_min;
  double e_J_min_re;
  double f_A_init;
  double f_J_min;
  double g_A_init;
  double h_A_init;
  double i_A_init;
  double im;
  double l11;
  double l12;
  double l13;
  double l14;
  double l14_tmp;
  double l17_tmp;
  double l2;
  double l2_re;
  double l3;
  double l4;
  double l4_re;
  double l4_tmp;
  double l5;
  double l6_tmp;
  double l7;
  double l9;
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
  l4_tmp = A_init * J_min;
  l5 = A_init * J_max;
  l14_tmp = J_max * J_max;
  b_l14_tmp = J_min * J_max;
  l14 = 1.0 / (l14_tmp + -b_l14_tmp);
  l4 = A_init * A_init;
  l17_tmp = J_max * V_max;
  l17.re = J_min * (J_min + -J_max) *
           ((l4 + l17_tmp * 2.0) + -(J_max * V_init * 2.0));
  l17.im = 0.0;
  coder::internal::scalar::b_sqrt(&l17);
  t1[0].re = l14 * ((l4_tmp - l5) + l17.re);
  t1[0].im = l14 * l17.im;
  t1[1].re = -l14 * ((-l4_tmp + l5) + l17.re);
  t1[1].im = -l14 * l17.im;
  l3 = rt_powd_snf(A_init, 3.0);
  b_l4_tmp = A_min * A_min;
  l6_tmp = A_wayp * A_wayp;
  l7 = rt_powd_snf(A_wayp, 3.0);
  l9 = J_min * J_min;
  l11 = rt_powd_snf(J_max, 3.0);
  l13 = rt_powd_snf(J_max, 5.0);
  coder::power(t1, l15);
  l5 = b_l4_tmp * b_l4_tmp;
  l14 = l6_tmp * l6_tmp;
  l12 = l14_tmp * l14_tmp;
  y = rt_powd_snf(l14_tmp, 3.0);
  l17.re = (((l5 * l9 - l5 * l14_tmp) - l14 * l9 * 3.0) - l14 * l14_tmp * 3.0) +
           l4 * l4 * l14_tmp * 3.0;
  J_min_re = b_l14_tmp * l14 * 6.0;
  b_A_init = A_init * l13;
  A_min_re = A_min * l3 * l14_tmp * 8.0;
  b_A_min_re = A_min * l7 * l9 * 8.0;
  b_A_min = A_min * l13;
  b_J_min = J_min * l13;
  l2_re = l4 * b_l4_tmp * l14_tmp * 6.0;
  l4_re = b_l4_tmp * l6_tmp * l9 * 6.0;
  l2 = l4 * l12;
  b_l4 = b_l4_tmp * l12;
  b_l9 = l9 * l12;
  b_l3 = l3 * l11;
  y_re = V_init * V_init * l9 * l14_tmp * 12.0;
  b_y_re = V_max * V_max * l9 * l14_tmp * 12.0;
  c_J_min = J_min * l4 * l11;
  d_J_min = J_min * b_l4_tmp * l11;
  e_J_min = J_min * l3 * l14_tmp;
  b_V_init = V_init * l9 * l11;
  b_l2 = l4 * l9 * l14_tmp;
  l5 = A_min * J_min;
  c_A_min_re = l5 * J_max * l7 * 12.0;
  A_init_tmp = A_init * A_min;
  c_A_init = A_init_tmp * l12;
  d_A_init = l4_tmp * l12;
  A_min_tmp = l5 * l12;
  d_A_min_re = A_min * P_init * l9 * l14_tmp * 24.0;
  e_A_min_re = A_min * P_wayp * l9 * l14_tmp * 24.0;
  b_J_min_re = b_l14_tmp * b_l4_tmp * l6_tmp * 6.0;
  J_min_re_tmp = J_min * V_init;
  c_J_min_re = J_min_re_tmp * l4 * l14_tmp * 12.0;
  d_J_min_re = J_min_re_tmp * b_l4_tmp * l14_tmp * 12.0;
  f_J_min = J_min_re_tmp * l12;
  J_max_re = l17_tmp * b_l4_tmp * l9 * 12.0;
  J_min_re_tmp = J_min * V_max;
  e_J_min_re = J_min_re_tmp * l6_tmp * l14_tmp * 12.0;
  b_J_max_re = l17_tmp * l6_tmp * l9 * 12.0;
  e_A_init = A_init * l9 * l11;
  b_A_min_tmp = A_min * l9 * l11;
  f_A_init = A_init * b_l4_tmp * l11;
  c_A_min = A_min * l4 * l11;
  l14 = A_init_tmp * J_min;
  b_A_init_tmp = l14 * l11;
  g_A_init = l4_tmp * V_init * l11;
  d_A_min = l5 * V_init * l11;
  A_init_tmp = A_init_tmp * l9 * l14_tmp;
  h_A_init = l4_tmp * b_l4_tmp * l14_tmp;
  c_A_min_tmp = l5 * l4 * l14_tmp;
  i_A_init = A_init * V_init * l9 * l14_tmp;
  d_A_min_tmp = A_min * V_init * l9 * l14_tmp;
  l11 = l14 * V_init * l14_tmp * 24.0;
  l17_tmp = A_min * A_wayp * J_max * V_max * l9 * 24.0;
  l3 = 1.0 / J_min;
  l4 = 1.0 / J_max;
  l14_tmp = A_min + -A_wayp;
  l9 = t1[0].re * t1[0].re - t1[0].im * t1[0].im;
  l14 = t1[0].re * t1[0].im;
  im = l14 + l14;
  b_l14[0].re = l9;
  b_l14[0].im = im;
  re = l9 * l9 - im * im;
  l14 = l9 * im;
  b_im = l14 + l14;
  l5 =
      ((((((((((((((((((((((((((((((((((((((((((((((l17.re + y * re * 3.0) +
                                                   J_min_re) +
                                                  b_A_init * l15[0].re * 12.0) -
                                                 A_min_re) +
                                                b_A_min_re) -
                                               b_A_min * l15[0].re * 8.0) -
                                              b_J_min * re * 6.0) +
                                             l2_re) -
                                            l4_re) +
                                           l2 * l9 * 18.0) +
                                          b_l4 * l9 * 6.0) +
                                         b_l9 * re * 3.0) +
                                        b_l3 * t1[0].re * 12.0) +
                                       y_re) -
                                      b_y_re) -
                                     c_J_min * l9 * 30.0) -
                                    d_J_min * l9 * 6.0) -
                                   e_J_min * t1[0].re * 12.0) +
                                  b_V_init * l9 * 12.0) +
                                 b_l2 * l9 * 12.0) -
                                c_A_min_re) -
                               c_A_init * l9 * 24.0) -
                              d_A_init * l15[0].re * 24.0) +
                             A_min_tmp * l15[0].re * 12.0) -
                            d_A_min_re) +
                           e_A_min_re) +
                          b_J_min_re) -
                         c_J_min_re) -
                        d_J_min_re) -
                       f_J_min * l9 * 12.0) +
                      J_max_re) -
                     e_J_min_re) +
                    b_J_max_re) +
                   e_A_init * l15[0].re * 12.0) -
                  b_A_min_tmp * l15[0].re * 4.0) +
                 f_A_init * t1[0].re * 12.0) -
                c_A_min * t1[0].re * 24.0) +
               b_A_init_tmp * l9 * 36.0) -
              g_A_init * t1[0].re * 24.0) +
             d_A_min * t1[0].re * 24.0) -
            A_init_tmp * l9 * 12.0) -
           h_A_init * t1[0].re * 12.0) +
          c_A_min_tmp * t1[0].re * 24.0) +
         i_A_init * t1[0].re * 24.0) -
        d_A_min_tmp * t1[0].re * 24.0) +
       l11) -
      l17_tmp;
  l14 =
      (((((((((((((((((((((((((((y * b_im * 3.0 + b_A_init * l15[0].im * 12.0) -
                                b_A_min * l15[0].im * 8.0) -
                               b_J_min * b_im * 6.0) +
                              l2 * im * 18.0) +
                             b_l4 * im * 6.0) +
                            b_l9 * b_im * 3.0) +
                           b_l3 * t1[0].im * 12.0) -
                          c_J_min * im * 30.0) -
                         d_J_min * im * 6.0) -
                        e_J_min * t1[0].im * 12.0) +
                       b_V_init * im * 12.0) +
                      b_l2 * im * 12.0) -
                     c_A_init * im * 24.0) -
                    d_A_init * l15[0].im * 24.0) +
                   A_min_tmp * l15[0].im * 12.0) -
                  f_J_min * im * 12.0) +
                 e_A_init * l15[0].im * 12.0) -
                b_A_min_tmp * l15[0].im * 4.0) +
               f_A_init * t1[0].im * 12.0) -
              c_A_min * t1[0].im * 24.0) +
             b_A_init_tmp * im * 36.0) -
            g_A_init * t1[0].im * 24.0) +
           d_A_min * t1[0].im * 24.0) -
          A_init_tmp * im * 12.0) -
         h_A_init * t1[0].im * 12.0) +
        c_A_min_tmp * t1[0].im * 24.0) +
       i_A_init * t1[0].im * 24.0) -
      d_A_min_tmp * t1[0].im * 24.0;
  l16[0].re = -0.083333333333333329 * l5;
  l16[0].im = -0.083333333333333329 * l14;
  l9 = J_max * t1[0].re;
  l13 = J_max * t1[0].im;
  l15[0].re = l9;
  l15[0].im = l13;
  l9 += A_init;
  b_y[0].re = l9 * l9 - l13 * l13;
  l4_tmp = l9 * l13;
  l9 = t1[1].re * t1[1].re - t1[1].im * t1[1].im;
  l14 = t1[1].re * t1[1].im;
  im = l14 + l14;
  re = l9 * l9 - im * im;
  l14 = l9 * im;
  b_im = l14 + l14;
  l12 = y * re;
  l7 = y * b_im;
  b_l14_tmp = b_J_min * re;
  l5 = b_J_min * b_im;
  l13 = b_l9 * re;
  l14 = b_l9 * b_im;
  re = -0.083333333333333329 *
       (((((((((((((((((((((((((((((((((((((((((((((((l17.re + l12 * 3.0) +
                                                     J_min_re) +
                                                    b_A_init * l15[1].re *
                                                        12.0) -
                                                   A_min_re) +
                                                  b_A_min_re) -
                                                 b_A_min * l15[1].re * 8.0) -
                                                b_l14_tmp * 6.0) +
                                               l2_re) -
                                              l4_re) +
                                             l2 * l9 * 18.0) +
                                            b_l4 * l9 * 6.0) +
                                           l13 * 3.0) +
                                          b_l3 * t1[1].re * 12.0) +
                                         y_re) -
                                        b_y_re) -
                                       c_J_min * l9 * 30.0) -
                                      d_J_min * l9 * 6.0) -
                                     e_J_min * t1[1].re * 12.0) +
                                    b_V_init * l9 * 12.0) +
                                   b_l2 * l9 * 12.0) -
                                  c_A_min_re) -
                                 c_A_init * l9 * 24.0) -
                                d_A_init * l15[1].re * 24.0) +
                               A_min_tmp * l15[1].re * 12.0) -
                              d_A_min_re) +
                             e_A_min_re) +
                            b_J_min_re) -
                           c_J_min_re) -
                          d_J_min_re) -
                         f_J_min * l9 * 12.0) +
                        J_max_re) -
                       e_J_min_re) +
                      b_J_max_re) +
                     e_A_init * l15[1].re * 12.0) -
                    b_A_min_tmp * l15[1].re * 4.0) +
                   f_A_init * t1[1].re * 12.0) -
                  c_A_min * t1[1].re * 24.0) +
                 b_A_init_tmp * l9 * 36.0) -
                g_A_init * t1[1].re * 24.0) +
               d_A_min * t1[1].re * 24.0) -
              A_init_tmp * l9 * 12.0) -
             h_A_init * t1[1].re * 12.0) +
            c_A_min_tmp * t1[1].re * 24.0) +
           i_A_init * t1[1].re * 24.0) -
          d_A_min_tmp * t1[1].re * 24.0) +
         l11) -
        l17_tmp);
  b_im = -0.083333333333333329 *
         ((((((((((((((((((((((((((((l7 * 3.0 + b_A_init * l15[1].im * 12.0) -
                                    b_A_min * l15[1].im * 8.0) -
                                   l5 * 6.0) +
                                  l2 * im * 18.0) +
                                 b_l4 * im * 6.0) +
                                l14 * 3.0) +
                               b_l3 * t1[1].im * 12.0) -
                              c_J_min * im * 30.0) -
                             d_J_min * im * 6.0) -
                            e_J_min * t1[1].im * 12.0) +
                           b_V_init * im * 12.0) +
                          b_l2 * im * 12.0) -
                         c_A_init * im * 24.0) -
                        d_A_init * l15[1].im * 24.0) +
                       A_min_tmp * l15[1].im * 12.0) -
                      f_J_min * im * 12.0) +
                     e_A_init * l15[1].im * 12.0) -
                    b_A_min_tmp * l15[1].im * 4.0) +
                   f_A_init * t1[1].im * 12.0) -
                  c_A_min * t1[1].im * 24.0) +
                 b_A_init_tmp * im * 36.0) -
                g_A_init * t1[1].im * 24.0) +
               d_A_min * t1[1].im * 24.0) -
              A_init_tmp * im * 12.0) -
             h_A_init * t1[1].im * 12.0) +
            c_A_min_tmp * t1[1].im * 24.0) +
           i_A_init * t1[1].im * 24.0) -
          d_A_min_tmp * t1[1].im * 24.0);
  l16[1].re =
      -0.083333333333333329 *
      (((((((((((((((((((((((((((((((((((((((((((((((l17.re + l12 * 3.0) +
                                                    J_min_re) +
                                                   b_A_init * l15[1].re *
                                                       12.0) -
                                                  A_min_re) +
                                                 b_A_min_re) -
                                                b_A_min * l15[1].re * 8.0) -
                                               b_l14_tmp * 6.0) +
                                              l2_re) -
                                             l4_re) +
                                            l2 * l9 * 18.0) +
                                           b_l4 * l9 * 6.0) +
                                          l13 * 3.0) +
                                         b_l3 * t1[1].re * 12.0) +
                                        y_re) -
                                       b_y_re) -
                                      c_J_min * l9 * 30.0) -
                                     d_J_min * l9 * 6.0) -
                                    e_J_min * t1[1].re * 12.0) +
                                   b_V_init * l9 * 12.0) +
                                  b_l2 * l9 * 12.0) -
                                 c_A_min_re) -
                                c_A_init * l9 * 24.0) -
                               d_A_init * l15[1].re * 24.0) +
                              A_min_tmp * l15[1].re * 12.0) -
                             d_A_min_re) +
                            e_A_min_re) +
                           b_J_min_re) -
                          c_J_min_re) -
                         d_J_min_re) -
                        f_J_min * l9 * 12.0) +
                       J_max_re) -
                      e_J_min_re) +
                     b_J_max_re) +
                    e_A_init * l15[1].re * 12.0) -
                   b_A_min_tmp * l15[1].re * 4.0) +
                  f_A_init * t1[1].re * 12.0) -
                 c_A_min * t1[1].re * 24.0) +
                b_A_init_tmp * l9 * 36.0) -
               g_A_init * t1[1].re * 24.0) +
              d_A_min * t1[1].re * 24.0) -
             A_init_tmp * l9 * 12.0) -
            h_A_init * t1[1].re * 12.0) +
           c_A_min_tmp * t1[1].re * 24.0) +
          i_A_init * t1[1].re * 24.0) -
         d_A_min_tmp * t1[1].re * 24.0) +
        l11) -
       l17_tmp);
  l16[1].im =
      -0.083333333333333329 *
      ((((((((((((((((((((((((((((l7 * 3.0 + b_A_init * l15[1].im * 12.0) -
                                 b_A_min * l15[1].im * 8.0) -
                                l5 * 6.0) +
                               l2 * im * 18.0) +
                              b_l4 * im * 6.0) +
                             l14 * 3.0) +
                            b_l3 * t1[1].im * 12.0) -
                           c_J_min * im * 30.0) -
                          d_J_min * im * 6.0) -
                         e_J_min * t1[1].im * 12.0) +
                        b_V_init * im * 12.0) +
                       b_l2 * im * 12.0) -
                      c_A_init * im * 24.0) -
                     d_A_init * l15[1].im * 24.0) +
                    A_min_tmp * l15[1].im * 12.0) -
                   f_J_min * im * 12.0) +
                  e_A_init * l15[1].im * 12.0) -
                 b_A_min_tmp * l15[1].im * 4.0) +
                f_A_init * t1[1].im * 12.0) -
               c_A_min * t1[1].im * 24.0) +
              b_A_init_tmp * im * 36.0) -
             g_A_init * t1[1].im * 24.0) +
            d_A_min * t1[1].im * 24.0) -
           A_init_tmp * im * 12.0) -
          h_A_init * t1[1].im * 12.0) +
         c_A_min_tmp * t1[1].im * 24.0) +
        i_A_init * t1[1].im * 24.0) -
       d_A_min_tmp * t1[1].im * 24.0);
  l13 = J_max * t1[1].im;
  l5 = A_init + J_max * t1[1].re;
  l7 = l5 * l13;
  J_min_re = J_min_re_tmp * 2.0;
  l17.re = V_init * 2.0;
  y_re = b_l4_tmp * l3;
  l4_re = l4 * (l14_tmp * l14_tmp);
  A_min_re = A_min * l4 * l14_tmp * 2.0;
  y = A_min * 2.0;
  l12 = -(1.0 / J_max * l14_tmp);
  b_A_min_re = d_A_min_tmp * 2.0;
  b_y[0].re = l3 * (((J_min_re + b_y[0].re) + l6_tmp) -
                    J_min * (((((l17.re + A_init * t1[0].re * 2.0) + y_re) +
                               J_max * b_l14[0].re) +
                              l4_re) -
                             A_min_re));
  b_y[0].im = l3 * ((l4_tmp + l4_tmp) -
                    J_min * (A_init * t1[0].im * 2.0 + J_max * b_l14[0].im));
  t[0] = t1[0];
  t[2].re = 0.0;
  t[2].im = 0.0;
  l14 = -(A_init + l15[0].re);
  if (-l15[0].im == 0.0) {
    t[4].re = l14 / J_min;
    t[4].im = 0.0;
  } else if (l14 == 0.0) {
    t[4].re = 0.0;
    t[4].im = -l15[0].im / J_min;
  } else {
    t[4].re = l14 / J_min;
    t[4].im = -l15[0].im / J_min;
  }
  b_y[1].re =
      l3 *
      (((J_min_re + (l5 * l5 - l13 * l13)) + l6_tmp) -
       J_min * (((((l17.re + A_init * t1[1].re * 2.0) + y_re) + J_max * l9) +
                 l4_re) -
                A_min_re));
  b_y[1].im = l3 * ((l7 + l7) - J_min * (A_init * t1[1].im * 2.0 + J_max * im));
  t[1] = t1[1];
  t[3].re = 0.0;
  t[3].im = 0.0;
  if (-l13 == 0.0) {
    t[5].re = -l5 / J_min;
    t[5].im = 0.0;
  } else if (-l5 == 0.0) {
    t[5].re = 0.0;
    t[5].im = -l13 / J_min;
  } else {
    t[5].re = -l5 / J_min;
    t[5].im = -l13 / J_min;
  }
  l3 = A_min * (1.0 / J_min);
  t[8].re = l3;
  t[8].im = 0.0;
  t[9].re = l3;
  t[9].im = 0.0;
  l13 = ((((c_A_min_tmp + A_min_tmp * b_l14[0].re) - b_A_min_re) -
          b_A_min_tmp * b_l14[0].re) +
         b_A_init_tmp * t1[0].re * 2.0) -
        A_init_tmp * t1[0].re * 2.0;
  l7 = ((A_min_tmp * b_l14[0].im - b_A_min_tmp * b_l14[0].im) +
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
    b_l14_tmp = std::abs(l13);
    l14 = std::abs(l7);
    if (b_l14_tmp > l14) {
      l5 = l7 / l13;
      l14 = l13 + l5 * l7;
      t[6].re = (l16[0].re + l5 * l16[0].im) / l14;
      t[6].im = (l16[0].im - l5 * l16[0].re) / l14;
    } else if (l14 == b_l14_tmp) {
      if (l13 > 0.0) {
        l5 = 0.5;
      } else {
        l5 = -0.5;
      }
      if (l7 > 0.0) {
        l14 = 0.5;
      } else {
        l14 = -0.5;
      }
      t[6].re = (l16[0].re * l5 + l16[0].im * l14) / b_l14_tmp;
      t[6].im = (l16[0].im * l5 - l16[0].re * l14) / b_l14_tmp;
    } else {
      l5 = l13 / l7;
      l14 = l7 + l5 * l13;
      t[6].re = (l5 * l16[0].re + l16[0].im) / l14;
      t[6].im = (l5 * l16[0].im - l16[0].re) / l14;
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
  l13 = ((((c_A_min_tmp + A_min_tmp * l9) - b_A_min_re) - b_A_min_tmp * l9) +
         b_A_init_tmp * t1[1].re * 2.0) -
        A_init_tmp * t1[1].re * 2.0;
  l7 = ((A_min_tmp * im - b_A_min_tmp * im) + b_A_init_tmp * t1[1].im * 2.0) -
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
    b_l14_tmp = std::abs(l13);
    l14 = std::abs(l7);
    if (b_l14_tmp > l14) {
      l5 = l7 / l13;
      l14 = l13 + l5 * l7;
      t[7].re = (re + l5 * b_im) / l14;
      t[7].im = (b_im - l5 * re) / l14;
    } else if (l14 == b_l14_tmp) {
      if (l13 > 0.0) {
        l5 = 0.5;
      } else {
        l5 = -0.5;
      }
      if (l7 > 0.0) {
        l14 = 0.5;
      } else {
        l14 = -0.5;
      }
      t[7].re = (re * l5 + b_im * l14) / b_l14_tmp;
      t[7].im = (b_im * l5 - re * l14) / b_l14_tmp;
    } else {
      l5 = l13 / l7;
      l14 = l7 + l5 * l13;
      t[7].re = (l5 * re + b_im) / l14;
      t[7].im = (l5 * b_im - re) / l14;
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
  t[12].re = l12;
  t[12].im = 0.0;
  t[13].re = l12;
  t[13].im = 0.0;
}

// End of code generation (acdefg_O_AP.cpp)
