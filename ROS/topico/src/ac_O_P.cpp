//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// ac_O_P.cpp
//
// Code generation for function 'ac_O_P'
//

// Include files
#include "ac_O_P.h"
#include "power.h"
#include "rt_nonfinite.h"
#include "sqrt.h"
#include "topico_wrapper_data.h"
#include "topico_wrapper_rtwutil.h"
#include <cmath>

// Function Definitions
void ac_O_P(double P_init, double V_init, double A_init, double P_wayp,
            double V_max, double J_max, double J_min, creal_T t[21])
{
  creal_T dc;
  creal_T l26;
  creal_T l27;
  creal_T l28;
  creal_T l56;
  creal_T l66;
  creal_T l87;
  double J_max_im;
  double J_max_re;
  double P_init_re_tmp;
  double V_init_im;
  double V_init_re;
  double b_J_max_re;
  double b_l9_re;
  double l10;
  double l2;
  double l27_im;
  double l27_re;
  double l2_im;
  double l2_re;
  double l3;
  double l4;
  double l5;
  double l66_tmp;
  double l69;
  double l69_im;
  double l69_re;
  double l70;
  double l7_im;
  double l7_re;
  double l7_tmp;
  double l8;
  double l85;
  double l8_im;
  double l8_re;
  double l9;
  double l93_im;
  double l93_re;
  double l95_im;
  double l95_re;
  double l9_im;
  double l9_re;
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
  //  Generated on 17-Sep-2019 12:00:55
  l2 = A_init * A_init;
  l3 = rt_powd_snf(A_init, 3.0);
  l4 = J_min * J_min;
  l5 = rt_powd_snf(J_min, 3.0);
  l7_tmp = J_max * J_max;
  l8 = rt_powd_snf(J_max, 3.0);
  l10 = rt_powd_snf(J_max, 5.0);
  l9 = l7_tmp * l7_tmp;
  l26.re = -J_min;
  l26.im = 0.0;
  coder::internal::scalar::b_sqrt(&l26);
  l27 = coder::d_power(l26);
  l28 = coder::e_power(l26);
  l56.re = J_max + -J_min;
  l56.im = 0.0;
  coder::internal::scalar::b_sqrt(&l56);
  l66_tmp = J_max * V_init;
  l66.re = (l2 + J_max * V_max * 2.0) + -(l66_tmp * 2.0);
  l66.im = 0.0;
  coder::internal::scalar::b_sqrt(&l66);
  l85 = (((((((-(V_init * l10 * 6.0) + J_min * V_max * l9 * 6.0) +
              J_min * V_init * l9 * 12.0) +
             l2 * l9 * 3.0) +
            V_max * l5 * l7_tmp * 6.0) +
           -(V_init * l4 * l8 * 6.0)) +
          -(J_min * l2 * l8 * 6.0)) +
         -(V_max * l4 * l8 * 12.0)) +
        l2 * l4 * l7_tmp * 3.0;
  l69 = 1.0 / (((J_min * l10 + l5 * l8 * 3.0) + -(l4 * l4 * l7_tmp)) +
               -(l4 * l9 * 3.0));
  l9_re = l9 * l26.re;
  l9_im = l9 * l26.im;
  b_l9_re = l9_re * l56.re - l9_im * l56.im;
  l9_im = l9_re * l56.im + l9_im * l56.re;
  l9_re = l7_tmp * l28.re;
  l7_im = l7_tmp * l28.im;
  l7_re = l9_re * l56.re - l7_im * l56.im;
  l7_im = l9_re * l56.im + l7_im * l56.re;
  l9_re = l8 * l27.re;
  l8_im = l8 * l27.im;
  l8_re = l9_re * l56.re - l8_im * l56.im;
  l8_im = l9_re * l56.im + l8_im * l56.re;
  l87.re = ((b_l9_re * l66.re - l9_im * l66.im) * 3.0 +
            (l7_re * l66.re - l7_im * l66.im) * 3.0) +
           (l8_re * l66.re - l8_im * l66.im) * 6.0;
  l87.im = ((b_l9_re * l66.im + l9_im * l66.re) * 3.0 +
            (l7_re * l66.im + l7_im * l66.re) * 3.0) +
           (l8_re * l66.im + l8_im * l66.re) * 6.0;
  l70 = l69 * l69;
  l7_re = l69 * l87.re;
  l9_re = l69 * l87.im;
  if (l9_re == 0.0) {
    l69_re = l7_re / 3.0;
    l69_im = 0.0;
  } else if (l7_re == 0.0) {
    l69_re = 0.0;
    l69_im = l9_re / 3.0;
  } else {
    l69_re = l7_re / 3.0;
    l69_im = l9_re / 3.0;
  }
  dc = coder::d_power(l87);
  l9_re = rt_powd_snf(l69, 3.0);
  l7_re = l9_re * dc.re;
  l9_re *= dc.im;
  if (l9_re == 0.0) {
    l93_re = l7_re / 27.0;
    l93_im = 0.0;
  } else if (l7_re == 0.0) {
    l93_re = 0.0;
    l93_im = l9_re / 27.0;
  } else {
    l93_re = l7_re / 27.0;
    l93_im = l9_re / 27.0;
  }
  l9_re = l70 * l85;
  l7_re = l9_re * l87.re;
  l9_re *= l87.im;
  if (l9_re == 0.0) {
    l95_re = l7_re / 6.0;
    l95_im = 0.0;
  } else if (l7_re == 0.0) {
    l95_re = 0.0;
    l95_im = l9_re / 6.0;
  } else {
    l95_re = l7_re / 6.0;
    l95_im = l9_re / 6.0;
  }
  dc = coder::d_power(l56);
  l27_re = l27.re * dc.re - l27.im * dc.im;
  l27_im = l27.re * dc.im + l27.im * dc.re;
  dc = coder::d_power(l66);
  l8_im = l66_tmp * l28.re;
  J_max_im = l66_tmp * l28.im;
  J_max_re = l8_im * l56.re - J_max_im * l56.im;
  J_max_im = l8_im * l56.im + J_max_im * l56.re;
  l8_im = l2 * l28.re;
  l2_im = l2 * l28.im;
  l2_re = l8_im * l56.re - l2_im * l56.im;
  l2_im = l8_im * l56.im + l2_im * l56.re;
  l9_re = V_init * l8;
  l8_im = l9_re * l26.re;
  V_init_im = l9_re * l26.im;
  V_init_re = l8_im * l56.re - V_init_im * l56.im;
  V_init_im = l8_im * l56.im + V_init_im * l56.re;
  l9_re = J_max * l2;
  l8_im = l9_re * l27.re;
  l66_tmp = l9_re * l27.im;
  b_J_max_re = l8_im * l56.re - l66_tmp * l56.im;
  l66_tmp = l8_im * l56.im + l66_tmp * l56.re;
  l9_re = V_init * l7_tmp;
  l8_im = l9_re * l27.re;
  l9_im = l9_re * l27.im;
  b_l9_re = l8_im * l56.re - l9_im * l56.im;
  l9_im = l8_im * l56.im + l9_im * l56.re;
  l9_re = l2 * l7_tmp;
  l8_im = l9_re * l26.re;
  l8_re = l9_re * l26.im;
  l7_im = l8_im * l56.re - l8_re * l56.im;
  l8_re = l8_im * l56.im + l8_re * l56.re;
  l9_re = A_init * V_init;
  l2 = A_init * J_max;
  P_init_re_tmp = A_init * J_min;
  l7_re =
      l69 * ((((((((((((((((((((((P_init * l10 * 6.0 + -(P_wayp * l10 * 6.0)) +
                                 -(l9_re * l9 * 6.0)) +
                                J_min * P_wayp * l9 * 18.0) +
                               l3 * l8 * 2.0) +
                              l2 * V_init * l5 * 6.0) +
                             P_wayp * l5 * l7_tmp * 6.0) +
                            -(J_min * P_init * l9 * 18.0)) +
                           -(l3 * l5 * 2.0)) +
                          P_init_re_tmp * V_init * l8 * 18.0) +
                         J_max * l3 * l4 * 6.0) +
                        -(P_init * l5 * l7_tmp * 6.0)) +
                       P_init * l4 * l8 * 18.0) +
                      -(J_min * l3 * l7_tmp * 6.0)) +
                     -(P_wayp * l4 * l8 * 18.0)) +
                    -(l9_re * l4 * l7_tmp * 18.0)) +
                   (l27_re * dc.re - l27_im * dc.im)) +
                  (J_max_re * l66.re - J_max_im * l66.im) * 6.0) +
                 -((l2_re * l66.re - l2_im * l66.im) * 3.0)) +
                (V_init_re * l66.re - V_init_im * l66.im) * 6.0) +
               -((b_J_max_re * l66.re - l66_tmp * l66.im) * 6.0)) +
              (b_l9_re * l66.re - l9_im * l66.im) * 12.0) +
             -((l7_im * l66.re - l8_re * l66.im) * 3.0));
  l9_re = l69 * (((((((l27_re * dc.im + l27_im * dc.re) +
                      (J_max_re * l66.im + J_max_im * l66.re) * 6.0) +
                     -((l2_re * l66.im + l2_im * l66.re) * 3.0)) +
                    (V_init_re * l66.im + V_init_im * l66.re) * 6.0) +
                   -((b_J_max_re * l66.im + l66_tmp * l66.re) * 6.0)) +
                  (b_l9_re * l66.im + l9_im * l66.re) * 12.0) +
                 -((l7_im * l66.im + l8_re * l66.re) * 3.0));
  if (l9_re == 0.0) {
    l27.re = l7_re / 2.0;
    l27.im = 0.0;
  } else if (l7_re == 0.0) {
    l27.re = 0.0;
    l27.im = l9_re / 2.0;
  } else {
    l27.re = l7_re / 2.0;
    l27.im = l9_re / 2.0;
  }
  l8_im = l87.re * l87.im;
  l7_re = l70 * (l87.re * l87.re - l87.im * l87.im);
  l9_re = l70 * (l8_im + l8_im);
  if (l9_re == 0.0) {
    l8_re = l7_re / 9.0;
    b_l9_re = 0.0;
  } else if (l7_re == 0.0) {
    l8_re = 0.0;
    b_l9_re = l9_re / 9.0;
  } else {
    l8_re = l7_re / 9.0;
    b_l9_re = l9_re / 9.0;
  }
  l28.re = l69 * l85 / 3.0 + l8_re;
  l28.im = b_l9_re;
  l26.re = (l93_re + l95_re) + l27.re;
  l26.im = (l93_im + l95_im) + l27.im;
  dc = coder::d_power(l28);
  l8_im = l26.re * l26.re - l26.im * l26.im;
  l8_re = l26.re * l26.im;
  l26.re = -dc.re + l8_im;
  l26.im = -dc.im + (l8_re + l8_re);
  coder::internal::scalar::b_sqrt(&l26);
  l56.re = ((-l93_re + -l95_re) + -l27.re) + l26.re;
  l56.im = ((-l93_im + -l95_im) + -l27.im) + l26.im;
  l66 = coder::power(l56);
  if (l66.im == 0.0) {
    l7_re = l66.re / 2.0;
    l9_im = 0.0;
    l7_im = 1.0 / l66.re;
    l9_re = 0.0;
  } else if (l66.re == 0.0) {
    l7_re = 0.0;
    l9_im = l66.im / 2.0;
    l7_im = 0.0;
    l9_re = -(1.0 / l66.im);
  } else {
    l7_re = l66.re / 2.0;
    l9_im = l66.im / 2.0;
    l8_re = std::abs(l66.re);
    l8_im = std::abs(l66.im);
    if (l8_re > l8_im) {
      l9_re = l66.im / l66.re;
      l8_im = l66.re + l9_re * l66.im;
      l7_im = (l9_re * 0.0 + 1.0) / l8_im;
      l9_re = (0.0 - l9_re) / l8_im;
    } else if (l8_im == l8_re) {
      if (l66.re > 0.0) {
        l9_re = 0.5;
      } else {
        l9_re = -0.5;
      }
      if (l66.im > 0.0) {
        l8_im = 0.5;
      } else {
        l8_im = -0.5;
      }
      l7_im = (l9_re + 0.0 * l8_im) / l8_re;
      l9_re = (0.0 * l9_re - l8_im) / l8_re;
    } else {
      l9_re = l66.re / l66.im;
      l8_im = l66.im + l9_re * l66.re;
      l7_im = l9_re / l8_im;
      l9_re = (l9_re * 0.0 - 1.0) / l8_im;
    }
  }
  l27.re = l28.re * l7_im - b_l9_re * l9_re;
  l27.im = l28.re * l9_re + b_l9_re * l7_im;
  if (l27.im == 0.0) {
    l27_re = l27.re / 2.0;
    l27_im = 0.0;
  } else if (l27.re == 0.0) {
    l27_re = 0.0;
    l27_im = l27.im / 2.0;
  } else {
    l27_re = l27.re / 2.0;
    l27_im = l27.im / 2.0;
  }
  l7_im = 1.7320508075688772 * (l66.re + -l27.re);
  l9_re = 1.7320508075688772 * (l66.im + -l27.im);
  l56.re = l7_im * 0.0 - l9_re * 0.5;
  l56.im = l7_im * 0.5 + l9_re * 0.0;
  l26.re = J_min * (J_min + -J_max) *
           ((A_init * A_init + J_max * V_max * 2.0) + -(J_max * V_init * 2.0));
  l26.im = 0.0;
  coder::internal::scalar::b_sqrt(&l26);
  l8_im = (P_init_re_tmp + -l2) + l26.re;
  l8_re = l26.im;
  l9_re = 1.0 / (l7_tmp + -(J_min * J_max));
  l26.re = l9_re * l8_im;
  l26.im = l9_re * l8_re;
  t[0] = l26;
  t[1] = l26;
  t[2] = l26;
  t[6].re = (-l69_re + l66.re) + l27.re;
  t[6].im = (-l69_im + l66.im) + l27.im;
  l8_im = (-l69_re + -l7_re) + -l27_re;
  t[7].re = l8_im - l56.re;
  l9_re = (-l69_im + -l9_im) + -l27_im;
  t[7].im = l9_re - l56.im;
  t[8].re = l8_im + l56.re;
  t[8].im = l9_re + l56.im;
  t[3].re = 0.0;
  t[3].im = 0.0;
  t[9].re = 0.0;
  t[9].im = 0.0;
  t[12].re = 0.0;
  t[12].im = 0.0;
  t[15].re = 0.0;
  t[15].im = 0.0;
  t[18].re = 0.0;
  t[18].im = 0.0;
  t[4].re = 0.0;
  t[4].im = 0.0;
  t[10].re = 0.0;
  t[10].im = 0.0;
  t[13].re = 0.0;
  t[13].im = 0.0;
  t[16].re = 0.0;
  t[16].im = 0.0;
  t[19].re = 0.0;
  t[19].im = 0.0;
  t[5].re = 0.0;
  t[5].im = 0.0;
  t[11].re = 0.0;
  t[11].im = 0.0;
  t[14].re = 0.0;
  t[14].im = 0.0;
  t[17].re = 0.0;
  t[17].im = 0.0;
  t[20].re = 0.0;
  t[20].im = 0.0;
}

// End of code generation (ac_O_P.cpp)
