//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// acdefg_O_VP.cpp
//
// Code generation for function 'acdefg_O_VP'
//

// Include files
#include "acdefg_O_VP.h"
#include "power.h"
#include "rt_nonfinite.h"
#include "sqrt.h"
#include "topico_wrapper_data.h"
#include "topico_wrapper_rtwutil.h"
#include "topico_wrapper_types.h"
#include <cmath>

// Function Definitions
void acdefg_O_VP(double P_init, double V_init, double A_init, double P_wayp,
                 double V_wayp, double V_max, double V_min, double A_min,
                 double J_max, double J_min, creal_T t[28])
{
  creal_T b_l3[4];
  creal_T b_t4_tmp[4];
  creal_T dcv[4];
  creal_T l9[4];
  creal_T t3[4];
  creal_T t4_tmp[4];
  creal_T t6[4];
  creal_T t7[4];
  creal_T l8;
  double A_init_re;
  double A_min_im;
  double A_min_re;
  double A_min_tmp;
  double J_max_tmp;
  double J_max_tmp_tmp;
  double J_min_tmp;
  double P_wayp_re;
  double V_init_tmp;
  double ai;
  double ar_tmp;
  double b_A_min_re;
  double b_J_max;
  double b_l5;
  double b_l8_tmp;
  double b_l8_tmp_tmp;
  double b_re_tmp;
  double b_y;
  double c_l8_tmp_tmp;
  double im;
  double l14_im;
  double l14_re;
  double l2;
  double l3;
  double l5;
  double l7;
  double l8_tmp;
  double l8_tmp_tmp;
  double re_tmp;
  double y;
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
  //  Generated on 28-Aug-2019 13:49:07
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
  l8_tmp_tmp = J_max * V_init;
  b_l8_tmp = l8_tmp_tmp * 2.0;
  l8.re =
      J_min * (J_min + -J_max) * ((l8_tmp + J_max * V_max * 2.0) + -b_l8_tmp);
  l8.im = 0.0;
  coder::internal::scalar::b_sqrt(&l8);
  b_l8_tmp_tmp = J_min * J_min;
  c_l8_tmp_tmp = J_min * J_max;
  l14_im = 1.0 / (b_l8_tmp_tmp + -c_l8_tmp_tmp);
  l8.re *= l14_im;
  l8.im *= l14_im;
  t3[0] = l8;
  t3[1] = l8;
  t3[2].re = -l8.re;
  t3[2].im = -l8.im;
  t3[3].re = -l8.re;
  t3[3].im = -l8.im;
  l3 = A_min * A_min;
  y_tmp = A_min * J_max;
  y = y_tmp * 2.0;
  l5 = rt_powd_snf(J_min, 3.0);
  l7 = J_max * J_max;
  coder::b_power(t3, l9);
  re_tmp = l8.re * l8.re - l8.im * l8.im;
  A_min_im = l8.re * l8.im;
  im = A_min_im + A_min_im;
  b_l3[0].im = im;
  l14_im = (l3 - b_l8_tmp) + J_max * V_min * 2.0;
  ar_tmp = J_max * l3 / J_min;
  b_l8_tmp =
      (((l14_im - b_l8_tmp_tmp * re_tmp) + l8_tmp) + c_l8_tmp_tmp * re_tmp) -
      ar_tmp;
  ai = (0.0 - b_l8_tmp_tmp * im) + c_l8_tmp_tmp * im;
  if (ai == 0.0) {
    t6[0].re = b_l8_tmp / y;
    t6[0].im = 0.0;
  } else if (b_l8_tmp == 0.0) {
    t6[0].re = 0.0;
    t6[0].im = ai / y;
  } else {
    t6[0].re = b_l8_tmp / y;
    t6[0].im = ai / y;
  }
  b_J_max = J_max * (b_l8_tmp_tmp * b_l8_tmp_tmp);
  t4_tmp[0].re = b_J_max * re_tmp;
  t4_tmp[0].im = b_J_max * im;
  b_l5 = l5 * l7;
  b_t4_tmp[0].re = b_l5 * re_tmp;
  b_t4_tmp[0].im = b_l5 * im;
  im = A_min_im + A_min_im;
  b_l3[1].im = im;
  b_l8_tmp =
      (((l14_im - b_l8_tmp_tmp * re_tmp) + l8_tmp) + c_l8_tmp_tmp * re_tmp) -
      ar_tmp;
  ai = (0.0 - b_l8_tmp_tmp * im) + c_l8_tmp_tmp * im;
  if (ai == 0.0) {
    t6[1].re = b_l8_tmp / y;
    t6[1].im = 0.0;
  } else if (b_l8_tmp == 0.0) {
    t6[1].re = 0.0;
    t6[1].im = ai / y;
  } else {
    t6[1].re = b_l8_tmp / y;
    t6[1].im = ai / y;
  }
  t4_tmp[1].re = b_J_max * re_tmp;
  t4_tmp[1].im = b_J_max * im;
  b_t4_tmp[1].re = b_l5 * re_tmp;
  b_t4_tmp[1].im = b_l5 * im;
  b_re_tmp = -l8.re * -l8.re - -l8.im * -l8.im;
  A_min_im = -l8.re * -l8.im;
  im = A_min_im + A_min_im;
  b_l3[2].im = im;
  b_l8_tmp = (((l14_im - b_l8_tmp_tmp * b_re_tmp) + l8_tmp) +
              c_l8_tmp_tmp * b_re_tmp) -
             ar_tmp;
  ai = (0.0 - b_l8_tmp_tmp * im) + c_l8_tmp_tmp * im;
  if (ai == 0.0) {
    t6[2].re = b_l8_tmp / y;
    t6[2].im = 0.0;
  } else if (b_l8_tmp == 0.0) {
    t6[2].re = 0.0;
    t6[2].im = ai / y;
  } else {
    t6[2].re = b_l8_tmp / y;
    t6[2].im = ai / y;
  }
  t4_tmp[2].re = b_J_max * b_re_tmp;
  t4_tmp[2].im = b_J_max * im;
  b_t4_tmp[2].re = b_l5 * b_re_tmp;
  b_t4_tmp[2].im = b_l5 * im;
  im = A_min_im + A_min_im;
  b_l8_tmp = (((l14_im - b_l8_tmp_tmp * b_re_tmp) + l8_tmp) +
              c_l8_tmp_tmp * b_re_tmp) -
             ar_tmp;
  ai = (0.0 - b_l8_tmp_tmp * im) + c_l8_tmp_tmp * im;
  if (ai == 0.0) {
    t6[3].re = b_l8_tmp / y;
    t6[3].im = 0.0;
  } else if (b_l8_tmp == 0.0) {
    t6[3].re = 0.0;
    t6[3].im = ai / y;
  } else {
    t6[3].re = b_l8_tmp / y;
    t6[3].im = ai / y;
  }
  t4_tmp[3].re = b_J_max * b_re_tmp;
  t4_tmp[3].im = b_J_max * im;
  b_t4_tmp[3].re = b_l5 * b_re_tmp;
  b_t4_tmp[3].im = b_l5 * im;
  y = rt_powd_snf(J_min, 5.0);
  coder::b_power(t7, dcv);
  l8.re = rt_powd_snf(A_init, 3.0) * b_l8_tmp_tmp * 2.0 +
          rt_powd_snf(A_min, 3.0) * l7;
  b_y = rt_powd_snf(J_max, 3.0) * b_l8_tmp_tmp;
  l14_re = P_init * b_l8_tmp_tmp * l7 * 6.0;
  P_wayp_re = P_wayp * b_l8_tmp_tmp * l7 * 6.0;
  l2 = l8_tmp * l5;
  J_max_tmp_tmp = J_max * l8_tmp;
  J_max_tmp = J_max_tmp_tmp * b_l8_tmp_tmp;
  J_min_tmp = J_min * l3 * l7;
  V_init_tmp = V_init * b_l8_tmp_tmp * l7;
  A_min_tmp = A_min * b_l8_tmp_tmp * l7;
  l14_im = A_min * J_min;
  A_min_re = l14_im * J_max * l8_tmp * 3.0;
  A_init_re = A_init * J_max * V_init * b_l8_tmp_tmp * 6.0;
  b_A_min_re = l14_im * V_init * l7 * 6.0;
  l8_tmp = y_tmp * l5;
  c_l8_tmp_tmp = l8_tmp_tmp * l5;
  l3 = A_min * (1.0 / J_min);
  y_tmp = J_max * b_l8_tmp_tmp;
  l5 = J_min * l7;
  l14_im = t6[0].re * t6[0].im;
  b_l8_tmp = t7[0].re * t7[0].im;
  ar_tmp = A_min_tmp * t6[0].re;
  A_min_im = A_min_tmp * t6[0].im;
  l9[0].re =
      (((((((((((((((((((((((((((l8.re - y * l9[0].re) + b_y * dcv[0].re) +
                               b_J_max * l9[0].re * 3.0) +
                              l14_re) -
                             P_wayp_re) -
                            b_l5 * l9[0].re * 2.0) +
                           l2 * t3[0].re * 3.0) -
                          J_max_tmp * t3[0].re * 3.0) -
                         J_max_tmp * t6[0].re * 3.0) -
                        J_max_tmp * t7[0].re * 3.0) +
                       J_min_tmp * t6[0].re * 3.0) +
                      J_min_tmp * t7[0].re * 3.0) +
                     (t4_tmp[0].re * t6[0].re - t4_tmp[0].im * t6[0].im) *
                         3.0) +
                    (t4_tmp[0].re * t7[0].re - t4_tmp[0].im * t7[0].im) * 3.0) +
                   V_init_tmp * t3[0].re * 6.0) +
                  V_init_tmp * t6[0].re * 6.0) +
                 V_init_tmp * t7[0].re * 6.0) -
                (b_t4_tmp[0].re * t6[0].re - b_t4_tmp[0].im * t6[0].im) * 3.0) -
               (b_t4_tmp[0].re * t7[0].re - b_t4_tmp[0].im * t7[0].im) * 3.0) +
              A_min_tmp * (t6[0].re * t6[0].re - t6[0].im * t6[0].im) * 3.0) +
             A_min_tmp * (t7[0].re * t7[0].re - t7[0].im * t7[0].im) * 3.0) -
            A_min_re) -
           A_init_re) +
          b_A_min_re) +
         l8_tmp * re_tmp * 3.0) -
        c_l8_tmp_tmp * t3[0].re * 6.0) -
       A_min_tmp * re_tmp * 3.0) +
      (ar_tmp * t7[0].re - A_min_im * t7[0].im) * 6.0;
  l9[0].im =
      ((((((((((((((((((((((0.0 - y * l9[0].im) + b_y * dcv[0].im) +
                          b_J_max * l9[0].im * 3.0) -
                         b_l5 * l9[0].im * 2.0) +
                        l2 * t3[0].im * 3.0) -
                       J_max_tmp * t3[0].im * 3.0) -
                      J_max_tmp * t6[0].im * 3.0) -
                     J_max_tmp * t7[0].im * 3.0) +
                    J_min_tmp * t6[0].im * 3.0) +
                   J_min_tmp * t7[0].im * 3.0) +
                  (t4_tmp[0].re * t6[0].im + t4_tmp[0].im * t6[0].re) * 3.0) +
                 (t4_tmp[0].re * t7[0].im + t4_tmp[0].im * t7[0].re) * 3.0) +
                V_init_tmp * t3[0].im * 6.0) +
               V_init_tmp * t6[0].im * 6.0) +
              V_init_tmp * t7[0].im * 6.0) -
             (b_t4_tmp[0].re * t6[0].im + b_t4_tmp[0].im * t6[0].re) * 3.0) -
            (b_t4_tmp[0].re * t7[0].im + b_t4_tmp[0].im * t7[0].re) * 3.0) +
           A_min_tmp * (l14_im + l14_im) * 3.0) +
          A_min_tmp * (b_l8_tmp + b_l8_tmp) * 3.0) +
         l8_tmp * b_l3[0].im * 3.0) -
        c_l8_tmp_tmp * t3[0].im * 6.0) -
       A_min_tmp * b_l3[0].im * 3.0) +
      (ar_tmp * t7[0].im + A_min_im * t7[0].re) * 6.0;
  b_l8_tmp = -(A_init + J_min * t3[0].re);
  ai = -(J_min * t3[0].im);
  if (ai == 0.0) {
    t[0].re = b_l8_tmp / J_max;
    t[0].im = 0.0;
  } else if (b_l8_tmp == 0.0) {
    t[0].re = 0.0;
    t[0].im = ai / J_max;
  } else {
    t[0].re = b_l8_tmp / J_max;
    t[0].im = ai / J_max;
  }
  t[4].re = 0.0;
  t[4].im = 0.0;
  t[8] = t3[0];
  l14_im = t6[1].re * t6[1].im;
  b_l8_tmp = t7[1].re * t7[1].im;
  ar_tmp = A_min_tmp * t6[1].re;
  A_min_im = A_min_tmp * t6[1].im;
  l9[1].re =
      (((((((((((((((((((((((((((l8.re - y * l9[1].re) + b_y * dcv[1].re) +
                               b_J_max * l9[1].re * 3.0) +
                              l14_re) -
                             P_wayp_re) -
                            b_l5 * l9[1].re * 2.0) +
                           l2 * t3[1].re * 3.0) -
                          J_max_tmp * t3[1].re * 3.0) -
                         J_max_tmp * t6[1].re * 3.0) -
                        J_max_tmp * t7[1].re * 3.0) +
                       J_min_tmp * t6[1].re * 3.0) +
                      J_min_tmp * t7[1].re * 3.0) +
                     (t4_tmp[1].re * t6[1].re - t4_tmp[1].im * t6[1].im) *
                         3.0) +
                    (t4_tmp[1].re * t7[1].re - t4_tmp[1].im * t7[1].im) * 3.0) +
                   V_init_tmp * t3[1].re * 6.0) +
                  V_init_tmp * t6[1].re * 6.0) +
                 V_init_tmp * t7[1].re * 6.0) -
                (b_t4_tmp[1].re * t6[1].re - b_t4_tmp[1].im * t6[1].im) * 3.0) -
               (b_t4_tmp[1].re * t7[1].re - b_t4_tmp[1].im * t7[1].im) * 3.0) +
              A_min_tmp * (t6[1].re * t6[1].re - t6[1].im * t6[1].im) * 3.0) +
             A_min_tmp * (t7[1].re * t7[1].re - t7[1].im * t7[1].im) * 3.0) -
            A_min_re) -
           A_init_re) +
          b_A_min_re) +
         l8_tmp * re_tmp * 3.0) -
        c_l8_tmp_tmp * t3[1].re * 6.0) -
       A_min_tmp * re_tmp * 3.0) +
      (ar_tmp * t7[1].re - A_min_im * t7[1].im) * 6.0;
  l9[1].im =
      ((((((((((((((((((((((0.0 - y * l9[1].im) + b_y * dcv[1].im) +
                          b_J_max * l9[1].im * 3.0) -
                         b_l5 * l9[1].im * 2.0) +
                        l2 * t3[1].im * 3.0) -
                       J_max_tmp * t3[1].im * 3.0) -
                      J_max_tmp * t6[1].im * 3.0) -
                     J_max_tmp * t7[1].im * 3.0) +
                    J_min_tmp * t6[1].im * 3.0) +
                   J_min_tmp * t7[1].im * 3.0) +
                  (t4_tmp[1].re * t6[1].im + t4_tmp[1].im * t6[1].re) * 3.0) +
                 (t4_tmp[1].re * t7[1].im + t4_tmp[1].im * t7[1].re) * 3.0) +
                V_init_tmp * t3[1].im * 6.0) +
               V_init_tmp * t6[1].im * 6.0) +
              V_init_tmp * t7[1].im * 6.0) -
             (b_t4_tmp[1].re * t6[1].im + b_t4_tmp[1].im * t6[1].re) * 3.0) -
            (b_t4_tmp[1].re * t7[1].im + b_t4_tmp[1].im * t7[1].re) * 3.0) +
           A_min_tmp * (l14_im + l14_im) * 3.0) +
          A_min_tmp * (b_l8_tmp + b_l8_tmp) * 3.0) +
         l8_tmp * b_l3[1].im * 3.0) -
        c_l8_tmp_tmp * t3[1].im * 6.0) -
       A_min_tmp * b_l3[1].im * 3.0) +
      (ar_tmp * t7[1].im + A_min_im * t7[1].re) * 6.0;
  b_l8_tmp = -(A_init + J_min * t3[1].re);
  ai = -(J_min * t3[1].im);
  if (ai == 0.0) {
    t[1].re = b_l8_tmp / J_max;
    t[1].im = 0.0;
  } else if (b_l8_tmp == 0.0) {
    t[1].re = 0.0;
    t[1].im = ai / J_max;
  } else {
    t[1].re = b_l8_tmp / J_max;
    t[1].im = ai / J_max;
  }
  t[5].re = 0.0;
  t[5].im = 0.0;
  t[9] = t3[1];
  l14_im = t6[2].re * t6[2].im;
  b_l8_tmp = t7[2].re * t7[2].im;
  ar_tmp = A_min_tmp * t6[2].re;
  A_min_im = A_min_tmp * t6[2].im;
  l9[2].re =
      (((((((((((((((((((((((((((l8.re - y * l9[2].re) + b_y * dcv[2].re) +
                               b_J_max * l9[2].re * 3.0) +
                              l14_re) -
                             P_wayp_re) -
                            b_l5 * l9[2].re * 2.0) +
                           l2 * t3[2].re * 3.0) -
                          J_max_tmp * t3[2].re * 3.0) -
                         J_max_tmp * t6[2].re * 3.0) -
                        J_max_tmp * t7[2].re * 3.0) +
                       J_min_tmp * t6[2].re * 3.0) +
                      J_min_tmp * t7[2].re * 3.0) +
                     (t4_tmp[2].re * t6[2].re - t4_tmp[2].im * t6[2].im) *
                         3.0) +
                    (t4_tmp[2].re * t7[2].re - t4_tmp[2].im * t7[2].im) * 3.0) +
                   V_init_tmp * t3[2].re * 6.0) +
                  V_init_tmp * t6[2].re * 6.0) +
                 V_init_tmp * t7[2].re * 6.0) -
                (b_t4_tmp[2].re * t6[2].re - b_t4_tmp[2].im * t6[2].im) * 3.0) -
               (b_t4_tmp[2].re * t7[2].re - b_t4_tmp[2].im * t7[2].im) * 3.0) +
              A_min_tmp * (t6[2].re * t6[2].re - t6[2].im * t6[2].im) * 3.0) +
             A_min_tmp * (t7[2].re * t7[2].re - t7[2].im * t7[2].im) * 3.0) -
            A_min_re) -
           A_init_re) +
          b_A_min_re) +
         l8_tmp * b_re_tmp * 3.0) -
        c_l8_tmp_tmp * t3[2].re * 6.0) -
       A_min_tmp * b_re_tmp * 3.0) +
      (ar_tmp * t7[2].re - A_min_im * t7[2].im) * 6.0;
  l9[2].im =
      ((((((((((((((((((((((0.0 - y * l9[2].im) + b_y * dcv[2].im) +
                          b_J_max * l9[2].im * 3.0) -
                         b_l5 * l9[2].im * 2.0) +
                        l2 * t3[2].im * 3.0) -
                       J_max_tmp * t3[2].im * 3.0) -
                      J_max_tmp * t6[2].im * 3.0) -
                     J_max_tmp * t7[2].im * 3.0) +
                    J_min_tmp * t6[2].im * 3.0) +
                   J_min_tmp * t7[2].im * 3.0) +
                  (t4_tmp[2].re * t6[2].im + t4_tmp[2].im * t6[2].re) * 3.0) +
                 (t4_tmp[2].re * t7[2].im + t4_tmp[2].im * t7[2].re) * 3.0) +
                V_init_tmp * t3[2].im * 6.0) +
               V_init_tmp * t6[2].im * 6.0) +
              V_init_tmp * t7[2].im * 6.0) -
             (b_t4_tmp[2].re * t6[2].im + b_t4_tmp[2].im * t6[2].re) * 3.0) -
            (b_t4_tmp[2].re * t7[2].im + b_t4_tmp[2].im * t7[2].re) * 3.0) +
           A_min_tmp * (l14_im + l14_im) * 3.0) +
          A_min_tmp * (b_l8_tmp + b_l8_tmp) * 3.0) +
         l8_tmp * b_l3[2].im * 3.0) -
        c_l8_tmp_tmp * t3[2].im * 6.0) -
       A_min_tmp * b_l3[2].im * 3.0) +
      (ar_tmp * t7[2].im + A_min_im * t7[2].re) * 6.0;
  b_l8_tmp = -(A_init + J_min * t3[2].re);
  ai = -(J_min * t3[2].im);
  if (ai == 0.0) {
    t[2].re = b_l8_tmp / J_max;
    t[2].im = 0.0;
  } else if (b_l8_tmp == 0.0) {
    t[2].re = 0.0;
    t[2].im = ai / J_max;
  } else {
    t[2].re = b_l8_tmp / J_max;
    t[2].im = ai / J_max;
  }
  t[6].re = 0.0;
  t[6].im = 0.0;
  t[10] = t3[2];
  l14_im = t6[3].re * t6[3].im;
  b_l8_tmp = t7[3].re * t7[3].im;
  ar_tmp = A_min_tmp * t6[3].re;
  A_min_im = A_min_tmp * t6[3].im;
  l9[3].re =
      (((((((((((((((((((((((((((l8.re - y * l9[3].re) + b_y * dcv[3].re) +
                               b_J_max * l9[3].re * 3.0) +
                              l14_re) -
                             P_wayp_re) -
                            b_l5 * l9[3].re * 2.0) +
                           l2 * t3[3].re * 3.0) -
                          J_max_tmp * t3[3].re * 3.0) -
                         J_max_tmp * t6[3].re * 3.0) -
                        J_max_tmp * t7[3].re * 3.0) +
                       J_min_tmp * t6[3].re * 3.0) +
                      J_min_tmp * t7[3].re * 3.0) +
                     (t4_tmp[3].re * t6[3].re - t4_tmp[3].im * t6[3].im) *
                         3.0) +
                    (t4_tmp[3].re * t7[3].re - t4_tmp[3].im * t7[3].im) * 3.0) +
                   V_init_tmp * t3[3].re * 6.0) +
                  V_init_tmp * t6[3].re * 6.0) +
                 V_init_tmp * t7[3].re * 6.0) -
                (b_t4_tmp[3].re * t6[3].re - b_t4_tmp[3].im * t6[3].im) * 3.0) -
               (b_t4_tmp[3].re * t7[3].re - b_t4_tmp[3].im * t7[3].im) * 3.0) +
              A_min_tmp * (t6[3].re * t6[3].re - t6[3].im * t6[3].im) * 3.0) +
             A_min_tmp * (t7[3].re * t7[3].re - t7[3].im * t7[3].im) * 3.0) -
            A_min_re) -
           A_init_re) +
          b_A_min_re) +
         l8_tmp * b_re_tmp * 3.0) -
        c_l8_tmp_tmp * t3[3].re * 6.0) -
       A_min_tmp * b_re_tmp * 3.0) +
      (ar_tmp * t7[3].re - A_min_im * t7[3].im) * 6.0;
  l9[3].im =
      ((((((((((((((((((((((0.0 - y * l9[3].im) + b_y * dcv[3].im) +
                          b_J_max * l9[3].im * 3.0) -
                         b_l5 * l9[3].im * 2.0) +
                        l2 * t3[3].im * 3.0) -
                       J_max_tmp * t3[3].im * 3.0) -
                      J_max_tmp * t6[3].im * 3.0) -
                     J_max_tmp * t7[3].im * 3.0) +
                    J_min_tmp * t6[3].im * 3.0) +
                   J_min_tmp * t7[3].im * 3.0) +
                  (t4_tmp[3].re * t6[3].im + t4_tmp[3].im * t6[3].re) * 3.0) +
                 (t4_tmp[3].re * t7[3].im + t4_tmp[3].im * t7[3].re) * 3.0) +
                V_init_tmp * t3[3].im * 6.0) +
               V_init_tmp * t6[3].im * 6.0) +
              V_init_tmp * t7[3].im * 6.0) -
             (b_t4_tmp[3].re * t6[3].im + b_t4_tmp[3].im * t6[3].re) * 3.0) -
            (b_t4_tmp[3].re * t7[3].im + b_t4_tmp[3].im * t7[3].re) * 3.0) +
           A_min_tmp * (l14_im + l14_im) * 3.0) +
          A_min_tmp * (b_l8_tmp + b_l8_tmp) * 3.0) +
         l8_tmp * im * 3.0) -
        c_l8_tmp_tmp * t3[3].im * 6.0) -
       A_min_tmp * im * 3.0) +
      (ar_tmp * t7[3].im + A_min_im * t7[3].re) * 6.0;
  b_l8_tmp = -(A_init + J_min * t3[3].re);
  ai = -(J_min * t3[3].im);
  if (ai == 0.0) {
    t[3].re = b_l8_tmp / J_max;
    t[3].im = 0.0;
  } else if (b_l8_tmp == 0.0) {
    t[3].re = 0.0;
    t[3].im = ai / J_max;
  } else {
    t[3].re = b_l8_tmp / J_max;
    t[3].im = ai / J_max;
  }
  t[7].re = 0.0;
  t[7].im = 0.0;
  t[11] = t3[3];
  l8.re = J_max_tmp_tmp - V_init * l7 * 2.0;
  t[16].re = l3;
  t[16].im = 0.0;
  t[17].re = l3;
  t[17].im = 0.0;
  t[18].re = l3;
  t[18].im = 0.0;
  t[19].re = l3;
  t[19].im = 0.0;
  ar_tmp = b_l8_tmp_tmp * ((l8.re - y_tmp * re_tmp) + l5 * re_tmp) * 3.0;
  A_min_im =
      b_l8_tmp_tmp * ((0.0 - y_tmp * b_l3[0].im) + l5 * b_l3[0].im) * 3.0;
  if (A_min_im == 0.0) {
    if (l9[0].im == 0.0) {
      t[12].re = l9[0].re / ar_tmp;
      t[12].im = 0.0;
    } else if (l9[0].re == 0.0) {
      t[12].re = 0.0;
      t[12].im = l9[0].im / ar_tmp;
    } else {
      t[12].re = l9[0].re / ar_tmp;
      t[12].im = l9[0].im / ar_tmp;
    }
  } else if (ar_tmp == 0.0) {
    if (l9[0].re == 0.0) {
      t[12].re = l9[0].im / A_min_im;
      t[12].im = 0.0;
    } else if (l9[0].im == 0.0) {
      t[12].re = 0.0;
      t[12].im = -(l9[0].re / A_min_im);
    } else {
      t[12].re = l9[0].im / A_min_im;
      t[12].im = -(l9[0].re / A_min_im);
    }
  } else {
    c_l8_tmp_tmp = std::abs(ar_tmp);
    l14_im = std::abs(A_min_im);
    if (c_l8_tmp_tmp > l14_im) {
      b_l8_tmp = A_min_im / ar_tmp;
      l14_im = ar_tmp + b_l8_tmp * A_min_im;
      t[12].re = (l9[0].re + b_l8_tmp * l9[0].im) / l14_im;
      t[12].im = (l9[0].im - b_l8_tmp * l9[0].re) / l14_im;
    } else if (l14_im == c_l8_tmp_tmp) {
      if (ar_tmp > 0.0) {
        b_l8_tmp = 0.5;
      } else {
        b_l8_tmp = -0.5;
      }
      if (A_min_im > 0.0) {
        l14_im = 0.5;
      } else {
        l14_im = -0.5;
      }
      t[12].re = (l9[0].re * b_l8_tmp + l9[0].im * l14_im) / c_l8_tmp_tmp;
      t[12].im = (l9[0].im * b_l8_tmp - l9[0].re * l14_im) / c_l8_tmp_tmp;
    } else {
      b_l8_tmp = ar_tmp / A_min_im;
      l14_im = A_min_im + b_l8_tmp * ar_tmp;
      t[12].re = (b_l8_tmp * l9[0].re + l9[0].im) / l14_im;
      t[12].im = (b_l8_tmp * l9[0].im - l9[0].re) / l14_im;
    }
  }
  t[20] = t6[0];
  t[24] = t7[0];
  ar_tmp = b_l8_tmp_tmp * ((l8.re - y_tmp * re_tmp) + l5 * re_tmp) * 3.0;
  A_min_im =
      b_l8_tmp_tmp * ((0.0 - y_tmp * b_l3[1].im) + l5 * b_l3[1].im) * 3.0;
  if (A_min_im == 0.0) {
    if (l9[1].im == 0.0) {
      t[13].re = l9[1].re / ar_tmp;
      t[13].im = 0.0;
    } else if (l9[1].re == 0.0) {
      t[13].re = 0.0;
      t[13].im = l9[1].im / ar_tmp;
    } else {
      t[13].re = l9[1].re / ar_tmp;
      t[13].im = l9[1].im / ar_tmp;
    }
  } else if (ar_tmp == 0.0) {
    if (l9[1].re == 0.0) {
      t[13].re = l9[1].im / A_min_im;
      t[13].im = 0.0;
    } else if (l9[1].im == 0.0) {
      t[13].re = 0.0;
      t[13].im = -(l9[1].re / A_min_im);
    } else {
      t[13].re = l9[1].im / A_min_im;
      t[13].im = -(l9[1].re / A_min_im);
    }
  } else {
    c_l8_tmp_tmp = std::abs(ar_tmp);
    l14_im = std::abs(A_min_im);
    if (c_l8_tmp_tmp > l14_im) {
      b_l8_tmp = A_min_im / ar_tmp;
      l14_im = ar_tmp + b_l8_tmp * A_min_im;
      t[13].re = (l9[1].re + b_l8_tmp * l9[1].im) / l14_im;
      t[13].im = (l9[1].im - b_l8_tmp * l9[1].re) / l14_im;
    } else if (l14_im == c_l8_tmp_tmp) {
      if (ar_tmp > 0.0) {
        b_l8_tmp = 0.5;
      } else {
        b_l8_tmp = -0.5;
      }
      if (A_min_im > 0.0) {
        l14_im = 0.5;
      } else {
        l14_im = -0.5;
      }
      t[13].re = (l9[1].re * b_l8_tmp + l9[1].im * l14_im) / c_l8_tmp_tmp;
      t[13].im = (l9[1].im * b_l8_tmp - l9[1].re * l14_im) / c_l8_tmp_tmp;
    } else {
      b_l8_tmp = ar_tmp / A_min_im;
      l14_im = A_min_im + b_l8_tmp * ar_tmp;
      t[13].re = (b_l8_tmp * l9[1].re + l9[1].im) / l14_im;
      t[13].im = (b_l8_tmp * l9[1].im - l9[1].re) / l14_im;
    }
  }
  t[21] = t6[1];
  t[25] = t7[1];
  ar_tmp = b_l8_tmp_tmp * ((l8.re - y_tmp * b_re_tmp) + l5 * b_re_tmp) * 3.0;
  A_min_im =
      b_l8_tmp_tmp * ((0.0 - y_tmp * b_l3[2].im) + l5 * b_l3[2].im) * 3.0;
  if (A_min_im == 0.0) {
    if (l9[2].im == 0.0) {
      t[14].re = l9[2].re / ar_tmp;
      t[14].im = 0.0;
    } else if (l9[2].re == 0.0) {
      t[14].re = 0.0;
      t[14].im = l9[2].im / ar_tmp;
    } else {
      t[14].re = l9[2].re / ar_tmp;
      t[14].im = l9[2].im / ar_tmp;
    }
  } else if (ar_tmp == 0.0) {
    if (l9[2].re == 0.0) {
      t[14].re = l9[2].im / A_min_im;
      t[14].im = 0.0;
    } else if (l9[2].im == 0.0) {
      t[14].re = 0.0;
      t[14].im = -(l9[2].re / A_min_im);
    } else {
      t[14].re = l9[2].im / A_min_im;
      t[14].im = -(l9[2].re / A_min_im);
    }
  } else {
    c_l8_tmp_tmp = std::abs(ar_tmp);
    l14_im = std::abs(A_min_im);
    if (c_l8_tmp_tmp > l14_im) {
      b_l8_tmp = A_min_im / ar_tmp;
      l14_im = ar_tmp + b_l8_tmp * A_min_im;
      t[14].re = (l9[2].re + b_l8_tmp * l9[2].im) / l14_im;
      t[14].im = (l9[2].im - b_l8_tmp * l9[2].re) / l14_im;
    } else if (l14_im == c_l8_tmp_tmp) {
      if (ar_tmp > 0.0) {
        b_l8_tmp = 0.5;
      } else {
        b_l8_tmp = -0.5;
      }
      if (A_min_im > 0.0) {
        l14_im = 0.5;
      } else {
        l14_im = -0.5;
      }
      t[14].re = (l9[2].re * b_l8_tmp + l9[2].im * l14_im) / c_l8_tmp_tmp;
      t[14].im = (l9[2].im * b_l8_tmp - l9[2].re * l14_im) / c_l8_tmp_tmp;
    } else {
      b_l8_tmp = ar_tmp / A_min_im;
      l14_im = A_min_im + b_l8_tmp * ar_tmp;
      t[14].re = (b_l8_tmp * l9[2].re + l9[2].im) / l14_im;
      t[14].im = (b_l8_tmp * l9[2].im - l9[2].re) / l14_im;
    }
  }
  t[22] = t6[2];
  t[26] = t7[2];
  ar_tmp = b_l8_tmp_tmp * ((l8.re - y_tmp * b_re_tmp) + l5 * b_re_tmp) * 3.0;
  A_min_im = b_l8_tmp_tmp * ((0.0 - y_tmp * im) + l5 * im) * 3.0;
  if (A_min_im == 0.0) {
    if (l9[3].im == 0.0) {
      t[15].re = l9[3].re / ar_tmp;
      t[15].im = 0.0;
    } else if (l9[3].re == 0.0) {
      t[15].re = 0.0;
      t[15].im = l9[3].im / ar_tmp;
    } else {
      t[15].re = l9[3].re / ar_tmp;
      t[15].im = l9[3].im / ar_tmp;
    }
  } else if (ar_tmp == 0.0) {
    if (l9[3].re == 0.0) {
      t[15].re = l9[3].im / A_min_im;
      t[15].im = 0.0;
    } else if (l9[3].im == 0.0) {
      t[15].re = 0.0;
      t[15].im = -(l9[3].re / A_min_im);
    } else {
      t[15].re = l9[3].im / A_min_im;
      t[15].im = -(l9[3].re / A_min_im);
    }
  } else {
    c_l8_tmp_tmp = std::abs(ar_tmp);
    l14_im = std::abs(A_min_im);
    if (c_l8_tmp_tmp > l14_im) {
      b_l8_tmp = A_min_im / ar_tmp;
      l14_im = ar_tmp + b_l8_tmp * A_min_im;
      t[15].re = (l9[3].re + b_l8_tmp * l9[3].im) / l14_im;
      t[15].im = (l9[3].im - b_l8_tmp * l9[3].re) / l14_im;
    } else if (l14_im == c_l8_tmp_tmp) {
      if (ar_tmp > 0.0) {
        b_l8_tmp = 0.5;
      } else {
        b_l8_tmp = -0.5;
      }
      if (A_min_im > 0.0) {
        l14_im = 0.5;
      } else {
        l14_im = -0.5;
      }
      t[15].re = (l9[3].re * b_l8_tmp + l9[3].im * l14_im) / c_l8_tmp_tmp;
      t[15].im = (l9[3].im * b_l8_tmp - l9[3].re * l14_im) / c_l8_tmp_tmp;
    } else {
      b_l8_tmp = ar_tmp / A_min_im;
      l14_im = A_min_im + b_l8_tmp * ar_tmp;
      t[15].re = (b_l8_tmp * l9[3].re + l9[3].im) / l14_im;
      t[15].im = (b_l8_tmp * l9[3].im - l9[3].re) / l14_im;
    }
  }
  t[23] = t6[3];
  t[27] = t7[3];
}

// End of code generation (acdefg_O_VP.cpp)
