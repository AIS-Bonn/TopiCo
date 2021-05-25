//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// acdef_T_P.cpp
//
// Code generation for function 'acdef_T_P'
//

// Include files
#include "acdef_T_P.h"
#include "power.h"
#include "rt_nonfinite.h"
#include "sqrt.h"
#include "topico_wrapper_data.h"
#include "topico_wrapper_rtwutil.h"

// Function Definitions
void acdef_T_P(double P_init, double V_init, double A_init, double P_wayp,
               double V_max, double A_min, double J_max, double J_min, double T,
               creal_T t[14])
{
  creal_T l6[2];
  creal_T t1[2];
  creal_T l17;
  double A_init_tmp;
  double J_min_re;
  double J_min_re_tmp;
  double T_re;
  double b_A_init;
  double b_J_max;
  double b_J_min;
  double b_y;
  double c_A_init;
  double c_J_max;
  double c_J_min;
  double d_A_init;
  double im;
  double l14;
  double l14_tmp;
  double l17_tmp;
  double l4_tmp;
  double l5;
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
  //  Generated on 29-Aug-2019 14:48:21
  l4_tmp = A_init * J_min;
  l5 = A_init * J_max;
  l14_tmp = J_max * J_max;
  l14 = 1.0 / (l14_tmp + -(J_min * J_max));
  l17_tmp = A_init * A_init;
  l17.re = J_min * (J_min + -J_max) *
           ((l17_tmp + J_max * V_max * 2.0) + -(J_max * V_init * 2.0));
  l17.im = 0.0;
  coder::internal::scalar::b_sqrt(&l17);
  t1[0].re = l14 * ((l4_tmp - l5) + l17.re);
  t1[0].im = l14 * l17.im;
  t1[1].re = -l14 * ((-l4_tmp + l5) + l17.re);
  t1[1].im = -l14 * l17.im;
  l5 = J_min * J_min;
  coder::power(t1, l6);
  y = rt_powd_snf(J_max, 3.0);
  b_y = rt_powd_snf(A_min, 3.0);
  l17.re = P_init * l5 * -24.0 + P_wayp * l5 * 24.0;
  y_re = rt_powd_snf(A_init, 3.0) * 4.0;
  J_min_re_tmp = J_min * T;
  J_min_re = J_min_re_tmp * l17_tmp * 12.0;
  T_re = T * V_init * l5 * 24.0;
  b_A_init = A_init * l5;
  c_A_init = A_init * l14_tmp;
  b_J_min = J_min * l14_tmp;
  b_J_max = J_max * l5;
  c_J_min = J_min * l17_tmp;
  c_J_max = J_max * l17_tmp;
  A_init_tmp = l4_tmp * J_max;
  d_A_init = A_init * T * l5;
  l14_tmp *= J_min_re_tmp;
  l14 = J_max * T * l5;
  l17_tmp = A_init_tmp * T;
  l4_tmp = A_min * A_min * 3.0;
  re = t1[0].re * t1[0].re - t1[0].im * t1[0].im;
  l5 = t1[0].re * t1[0].im;
  im = l5 + l5;
  l6[0].re =
      A_min * ((((((((((((((((l17.re + y * l6[0].re * 4.0) + y_re) - b_y) +
                           J_min_re) -
                          T_re) +
                         b_A_init * re * 12.0) +
                        c_A_init * re * 12.0) -
                       b_J_min * l6[0].re * 12.0) +
                      b_J_max * l6[0].re * 8.0) -
                     c_J_min * t1[0].re * 12.0) +
                    c_J_max * t1[0].re * 12.0) -
                   A_init_tmp * re * 24.0) -
                  d_A_init * t1[0].re * 24.0) +
                 l14_tmp * re * 12.0) -
                l14 * re * 12.0) +
               l17_tmp * t1[0].re * 24.0) -
      l4_tmp;
  l6[0].im = A_min * (((((((((((y * l6[0].im * 4.0 + b_A_init * im * 12.0) +
                               c_A_init * im * 12.0) -
                              b_J_min * l6[0].im * 12.0) +
                             b_J_max * l6[0].im * 8.0) -
                            c_J_min * t1[0].im * 12.0) +
                           c_J_max * t1[0].im * 12.0) -
                          A_init_tmp * im * 24.0) -
                         d_A_init * t1[0].im * 24.0) +
                        l14_tmp * im * 12.0) -
                       l14 * im * 12.0) +
                      l17_tmp * t1[0].im * 24.0);
  re = t1[1].re * t1[1].re - t1[1].im * t1[1].im;
  l5 = t1[1].re * t1[1].im;
  im = l5 + l5;
  l6[1].re =
      A_min * ((((((((((((((((l17.re + y * l6[1].re * 4.0) + y_re) - b_y) +
                           J_min_re) -
                          T_re) +
                         b_A_init * re * 12.0) +
                        c_A_init * re * 12.0) -
                       b_J_min * l6[1].re * 12.0) +
                      b_J_max * l6[1].re * 8.0) -
                     c_J_min * t1[1].re * 12.0) +
                    c_J_max * t1[1].re * 12.0) -
                   A_init_tmp * re * 24.0) -
                  d_A_init * t1[1].re * 24.0) +
                 l14_tmp * re * 12.0) -
                l14 * re * 12.0) +
               l17_tmp * t1[1].re * 24.0) -
      l4_tmp;
  l6[1].im = A_min * (((((((((((y * l6[1].im * 4.0 + b_A_init * im * 12.0) +
                               c_A_init * im * 12.0) -
                              b_J_min * l6[1].im * 12.0) +
                             b_J_max * l6[1].im * 8.0) -
                            c_J_min * t1[1].im * 12.0) +
                           c_J_max * t1[1].im * 12.0) -
                          A_init_tmp * im * 24.0) -
                         d_A_init * t1[1].im * 24.0) +
                        l14_tmp * im * 12.0) -
                       l14 * im * 12.0) +
                      l17_tmp * t1[1].im * 24.0);
  coder::internal::scalar::b_sqrt(&l6[0]);
  coder::internal::scalar::b_sqrt(&l6[1]);
  y = A_min * J_min * 6.0;
  l5 = A_min * (1.0 / J_min);
  l17.re = A_init - A_min;
  t[8].re = l5;
  t[8].im = 0.0;
  t[9].re = l5;
  t[9].im = 0.0;
  l17_tmp = 1.7320508075688772 * l6[0].re;
  l5 = 1.7320508075688772 * l6[0].im;
  if (l5 == 0.0) {
    re = l17_tmp / y;
    im = 0.0;
  } else if (l17_tmp == 0.0) {
    re = 0.0;
    im = l5 / y;
  } else {
    re = l17_tmp / y;
    im = l5 / y;
  }
  l5 = J_max * t1[0].re;
  l14 = J_max * t1[0].im;
  t[0] = t1[0];
  t[2].re = 0.0;
  t[2].im = 0.0;
  l17_tmp = -(A_init + l5);
  if (-l14 == 0.0) {
    t[4].re = l17_tmp / J_min;
    t[4].im = 0.0;
  } else if (l17_tmp == 0.0) {
    t[4].re = 0.0;
    t[4].im = -l14 / J_min;
  } else {
    t[4].re = l17_tmp / J_min;
    t[4].im = -l14 / J_min;
  }
  l17_tmp = (((l17.re - J_min * t1[0].re) + l5) - J_min * re) + J_min_re_tmp;
  l5 = ((0.0 - J_min * t1[0].im) + l14) - J_min * im;
  if (l5 == 0.0) {
    t[6].re = l17_tmp / J_min;
    t[6].im = 0.0;
  } else if (l17_tmp == 0.0) {
    t[6].re = 0.0;
    t[6].im = l5 / J_min;
  } else {
    t[6].re = l17_tmp / J_min;
    t[6].im = l5 / J_min;
  }
  t[10].re = re;
  t[10].im = im;
  t[12].re = 0.0;
  t[12].im = 0.0;
  l17_tmp = 1.7320508075688772 * l6[1].re;
  l5 = 1.7320508075688772 * l6[1].im;
  if (l5 == 0.0) {
    re = l17_tmp / y;
    im = 0.0;
  } else if (l17_tmp == 0.0) {
    re = 0.0;
    im = l5 / y;
  } else {
    re = l17_tmp / y;
    im = l5 / y;
  }
  l5 = J_max * t1[1].re;
  l14 = J_max * t1[1].im;
  t[1] = t1[1];
  t[3].re = 0.0;
  t[3].im = 0.0;
  l17_tmp = -(A_init + l5);
  if (-l14 == 0.0) {
    t[5].re = l17_tmp / J_min;
    t[5].im = 0.0;
  } else if (l17_tmp == 0.0) {
    t[5].re = 0.0;
    t[5].im = -l14 / J_min;
  } else {
    t[5].re = l17_tmp / J_min;
    t[5].im = -l14 / J_min;
  }
  l17_tmp = (((l17.re - J_min * t1[1].re) + l5) - J_min * re) + J_min_re_tmp;
  l5 = ((0.0 - J_min * t1[1].im) + l14) - J_min * im;
  if (l5 == 0.0) {
    t[7].re = l17_tmp / J_min;
    t[7].im = 0.0;
  } else if (l17_tmp == 0.0) {
    t[7].re = 0.0;
    t[7].im = l5 / J_min;
  } else {
    t[7].re = l17_tmp / J_min;
    t[7].im = l5 / J_min;
  }
  t[11].re = re;
  t[11].im = im;
  t[13].re = 0.0;
  t[13].im = 0.0;
}

// End of code generation (acdef_T_P.cpp)
