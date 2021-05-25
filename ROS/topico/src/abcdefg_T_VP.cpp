//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// abcdefg_T_VP.cpp
//
// Code generation for function 'abcdefg_T_VP'
//

// Include files
#include "abcdefg_T_VP.h"
#include "power.h"
#include "rt_nonfinite.h"
#include "sqrt.h"
#include "topico_wrapper_data.h"
#include "topico_wrapper_rtwutil.h"
#include "rt_nonfinite.h"
#include <cmath>

// Function Definitions
void abcdefg_T_VP(double P_init, double V_init, double A_init, double P_wayp,
                  double V_wayp, double V_max, double A_max, double A_min,
                  double J_max, double J_min, double T, creal_T t[28])
{
  creal_T dcv[4];
  creal_T t4_tmp[4];
  creal_T t7[4];
  creal_T y[4];
  creal_T l114;
  creal_T l119;
  creal_T l120;
  creal_T l3;
  double b_l102_tmp;
  double b_l99_tmp;
  double b_l99_tmp_tmp;
  double c_l99_tmp;
  double c_l99_tmp_tmp;
  double d;
  double d_l99_tmp;
  double e_l99_tmp;
  double f_l99_tmp;
  double g_l99_tmp;
  double h_l99_tmp;
  double i_l99_tmp;
  double j_l99_tmp;
  double k_l99_tmp;
  double l102;
  double l102_tmp;
  double l102_tmp_tmp;
  double l10_idx_0_tmp;
  double l11_tmp;
  double l121_re;
  double l122_im;
  double l122_re;
  double l125_re;
  double l129_re;
  double l12_tmp;
  double l13;
  double l135_im;
  double l135_re;
  double l136_im;
  double l136_re;
  double l13_tmp;
  double l14;
  double l143_re;
  double l17_tmp;
  double l18;
  double l19;
  double l20;
  double l21;
  double l23;
  double l28_tmp;
  double l29;
  double l2_tmp;
  double l36;
  double l5_tmp;
  double l73;
  double l7_tmp;
  double l80;
  double l81;
  double l82;
  double l86;
  double l86_tmp;
  double l88;
  double l8_tmp;
  double l99_tmp;
  double l99_tmp_tmp;
  double l_l99_tmp;
  double m_l99_tmp;
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
  //  Generated on 04-Sep-2019 15:47:08
  l2_tmp = A_init * A_init;
  l5_tmp = A_min * A_min;
  l8_tmp = A_max * A_max;
  l11_tmp = J_min * J_min;
  l12_tmp = J_max * J_max;
  l14 = V_max * V_max;
  l17_tmp = 1.0 / J_min;
  l18 = 1.0 / J_max;
  l23 = 1.0 / V_max;
  l7_tmp = l5_tmp * l5_tmp;
  l19 = 1.0 / l12_tmp;
  l20 = rt_powd_snf(l18, 3.0);
  l28_tmp = J_max * l8_tmp;
  l29 = J_min * l12_tmp;
  l36 = A_min * l18 / 3.0;
  l21 = l19 * l19;
  l73 = l29 + -(V_wayp * l23 * l29);
  l102 = V_max * l17_tmp * l20;
  l80 = l5_tmp * l19 * 0.66666666666666663 + -(l102 * l73 * 4.0);
  l86_tmp = A_min * V_max;
  l86 = rt_powd_snf(A_min, 3.0) * l20 * 0.29629629629629628 +
        -(l86_tmp * l17_tmp * l21 * l73 * 2.6666666666666665);
  l99_tmp_tmp = l8_tmp * l8_tmp;
  l19 = A_min * l99_tmp_tmp;
  l99_tmp = A_init * A_min;
  b_l99_tmp = A_min * A_max;
  c_l99_tmp = A_min * J_max;
  l20 = A_min * V_init;
  b_l99_tmp_tmp = A_min * l11_tmp;
  l136_im = b_l99_tmp_tmp * l12_tmp;
  l135_im = A_max * l11_tmp * l12_tmp;
  d_l99_tmp = rt_powd_snf(A_max, 3.0);
  e_l99_tmp = l2_tmp * l2_tmp;
  f_l99_tmp = J_min * J_max;
  g_l99_tmp = rt_powd_snf(A_init, 3.0);
  h_l99_tmp = V_init * V_init;
  i_l99_tmp = V_wayp * V_wayp;
  c_l99_tmp_tmp = A_min * J_min;
  j_l99_tmp = c_l99_tmp_tmp * J_max;
  k_l99_tmp = A_max * J_max;
  l_l99_tmp = b_l99_tmp * J_max;
  m_l99_tmp = J_min * l2_tmp;
  l81 = l80 * l80;
  l82 = rt_powd_snf(l80, 3.0);
  l88 = l86 * l86;
  l102_tmp = l7_tmp * l21;
  b_l102_tmp = V_max * l5_tmp * l17_tmp * rt_powd_snf(l18, 5.0) * l73;
  l102_tmp_tmp = l20 * l11_tmp;
  l18 = l102 *
        ((((((l99_tmp * J_min * 2.0 + l_l99_tmp * 2.0) +
             f_l99_tmp * V_max * 2.0) +
            m_l99_tmp) +
           l28_tmp) +
          j_l99_tmp * T * 2.0) +
         -(1.0 / A_max * l17_tmp * l18 * l23 *
           (((((((((((((((((((((l19 * l12_tmp +
                                A_min * e_l99_tmp * l11_tmp * 3.0) +
                               -(l19 * l11_tmp)) +
                              -(A_max * l7_tmp * l12_tmp)) +
                             V_max * d_l99_tmp * l29 * 12.0) +
                            l99_tmp * A_max * J_max * V_init * l11_tmp * 24.0) +
                           -(b_l99_tmp * g_l99_tmp * l11_tmp * 8.0)) +
                          b_l99_tmp * P_wayp * l11_tmp * l12_tmp * 24.0) +
                         c_l99_tmp * V_max * l2_tmp * l11_tmp * 12.0) +
                        k_l99_tmp * V_max * l2_tmp * l11_tmp * 12.0) +
                       l86_tmp * l11_tmp * l28_tmp * 12.0) +
                      l86_tmp * l8_tmp * l29 * 24.0) +
                     A_min * l2_tmp * l8_tmp * l11_tmp * 6.0) +
                    A_max * V_max * V_wayp * l11_tmp * l12_tmp * 24.0) +
                   -(b_l99_tmp * P_init * l11_tmp * l12_tmp * 24.0)) +
                  -(c_l99_tmp * V_init * l2_tmp * l11_tmp * 12.0)) +
                 -(l102_tmp_tmp * l28_tmp * 12.0)) +
                -(l20 * V_max * l11_tmp * l12_tmp * 24.0)) +
               l136_im * h_l99_tmp * 12.0) +
              l136_im * l14 * 12.0) +
             l135_im * l14 * 12.0) +
            -(l135_im * i_l99_tmp * 12.0)) /
           12.0));
  l102 = (-(l102_tmp / 27.0) + b_l102_tmp * 0.44444444444444442) + l18 * 4.0;
  l114.re = ((((l88 * l88 * 27.0 + -(l82 * l88 * 4.0)) +
               -(rt_powd_snf(l102, 3.0) * 256.0)) +
              -(l81 * l81 * l102 * 16.0)) +
             l81 * (l102 * l102) * 128.0) +
            l80 * l88 * l102 * 144.0;
  l114.im = 0.0;
  coder::internal::scalar::b_sqrt(&l114);
  l21 = 1.7320508075688772 * l114.re;
  l73 = 1.7320508075688772 * l114.im;
  if (l73 == 0.0) {
    l86_tmp = l21 / 18.0;
    l14 = 0.0;
  } else if (l21 == 0.0) {
    l86_tmp = 0.0;
    l14 = l73 / 18.0;
  } else {
    l86_tmp = l21 / 18.0;
    l14 = l73 / 18.0;
  }
  l23 = l80 * l102;
  l119.re = ((-(l82 / 27.0) + l88 / 2.0) + l23 * 1.3333333333333333) + l86_tmp;
  l119.im = l14;
  l120 = coder::power(l119);
  l119 = coder::b_power(l119);
  if (l119.im == 0.0) {
    l122_re = 1.0 / l119.re;
    l122_im = 0.0;
  } else if (l119.re == 0.0) {
    l122_re = 0.0;
    l122_im = -(1.0 / l119.im);
  } else {
    l86_tmp = std::abs(l119.re);
    l19 = std::abs(l119.im);
    if (l86_tmp > l19) {
      l19 = l119.im / l119.re;
      l20 = l119.re + l19 * l119.im;
      l122_re = (l19 * 0.0 + 1.0) / l20;
      l122_im = (0.0 - l19) / l20;
    } else if (l19 == l86_tmp) {
      if (l119.re > 0.0) {
        l19 = 0.5;
      } else {
        l19 = -0.5;
      }
      if (l119.im > 0.0) {
        l20 = 0.5;
      } else {
        l20 = -0.5;
      }
      l122_re = (l19 + 0.0 * l20) / l86_tmp;
      l122_im = (0.0 * l19 - l20) / l86_tmp;
    } else {
      l19 = l119.re / l119.im;
      l20 = l119.im + l19 * l119.re;
      l122_re = l19 / l20;
      l122_im = (l19 * 0.0 - 1.0) / l20;
    }
  }
  l121_re = l120.re * l120.re - l120.im * l120.im;
  l19 = l120.re * l120.im;
  l29 = l19 + l19;
  l3.re = ((-(l82 * 2.0) + l88 * 27.0) + l23 * 72.0) + l21 * 3.0;
  l3.im = l73 * 3.0;
  coder::internal::scalar::b_sqrt(&l3);
  l125_re = 3.0 * (2.4494897427831779 * l86 * l3.re);
  l23 = 3.0 * (2.4494897427831779 * l86 * l3.im);
  l14 = l80 * l120.re;
  l73 = l80 * l120.im;
  l119.re =
      ((((-(l102_tmp * 0.44444444444444442) + b_l102_tmp * 5.333333333333333) +
         l81) +
        l18 * 48.0) +
       l121_re * 9.0) +
      l14 * 6.0;
  l119.im = l29 * 9.0 + l73 * 6.0;
  l114 = l119;
  coder::internal::scalar::b_sqrt(&l114);
  l119 = coder::c_power(l119);
  if (l119.im == 0.0) {
    l129_re = 1.0 / l119.re;
    l21 = 0.0;
  } else if (l119.re == 0.0) {
    l129_re = 0.0;
    l21 = -(1.0 / l119.im);
  } else {
    l86_tmp = std::abs(l119.re);
    l19 = std::abs(l119.im);
    if (l86_tmp > l19) {
      l19 = l119.im / l119.re;
      l20 = l119.re + l19 * l119.im;
      l129_re = (l19 * 0.0 + 1.0) / l20;
      l21 = (0.0 - l19) / l20;
    } else if (l19 == l86_tmp) {
      if (l119.re > 0.0) {
        l19 = 0.5;
      } else {
        l19 = -0.5;
      }
      if (l119.im > 0.0) {
        l20 = 0.5;
      } else {
        l20 = -0.5;
      }
      l129_re = (l19 + 0.0 * l20) / l86_tmp;
      l21 = (0.0 * l19 - l20) / l86_tmp;
    } else {
      l19 = l119.re / l119.im;
      l20 = l119.im + l19 * l119.re;
      l129_re = l19 / l20;
      l21 = (l19 * 0.0 - 1.0) / l20;
    }
  }
  l135_re = -9.0 * (l121_re * l114.re - l29 * l114.im);
  l135_im = -9.0 * (l121_re * l114.im + l29 * l114.re);
  l19 = l122_re * l114.re - l122_im * l114.im;
  l20 = l122_re * l114.im + l122_im * l114.re;
  if (l20 == 0.0) {
    l136_re = l19 / 6.0;
    l136_im = 0.0;
  } else if (l19 == 0.0) {
    l136_re = 0.0;
    l136_im = l20 / 6.0;
  } else {
    l136_re = l19 / 6.0;
    l136_im = l20 / 6.0;
  }
  l121_re = 12.0 * (l14 * l114.re - l73 * l114.im);
  l29 = 12.0 * (l14 * l114.im + l73 * l114.re);
  l120.re = -(l81 * l114.re);
  l120.im = -(l81 * l114.im);
  l119.re = -(l102 * l114.re * 12.0);
  l119.im = -(l102 * l114.im * 12.0);
  l3.re = (((l125_re + l120.re) + l119.re) + l135_re) + l121_re;
  l3.im = (((l23 + l120.im) + l119.im) + l135_im) + l29;
  coder::internal::scalar::b_sqrt(&l3);
  l73 = l122_re * l129_re - l122_im * l21;
  l19 = l122_re * l21 + l122_im * l129_re;
  l122_re = l73 * l3.re - l19 * l3.im;
  l122_im = l73 * l3.im + l19 * l3.re;
  if (l122_im == 0.0) {
    l143_re = l122_re / 6.0;
    l20 = 0.0;
  } else if (l122_re == 0.0) {
    l143_re = 0.0;
    l20 = l122_im / 6.0;
  } else {
    l143_re = l122_re / 6.0;
    l20 = l122_im / 6.0;
  }
  l3.re = (((-l125_re + l120.re) + l119.re) + l135_re) + l121_re;
  l3.im = (((-l23 + l120.im) + l119.im) + l135_im) + l29;
  coder::internal::scalar::b_sqrt(&l3);
  l122_re = l73 * l3.re - l19 * l3.im;
  l122_im = l73 * l3.im + l19 * l3.re;
  if (l122_im == 0.0) {
    l119.re = l122_re / 6.0;
    l119.im = 0.0;
  } else if (l122_re == 0.0) {
    l119.re = 0.0;
    l119.im = l122_im / 6.0;
  } else {
    l119.re = l122_re / 6.0;
    l119.im = l122_im / 6.0;
  }
  d = -l36 + -l136_re;
  t7[0].re = d - l143_re;
  t7[0].im = -l136_im - l20;
  t7[1].re = d + l143_re;
  t7[1].im = -l136_im + l20;
  d = -l36 + l136_re;
  t7[2].re = d - l119.re;
  t7[2].im = l136_im - l119.im;
  t7[3].re = d + l119.re;
  t7[3].im = l136_im + l119.im;
  l20 = J_max * V_init;
  l13_tmp = l20 * 2.0;
  l13 = 1.0 / A_max * (1.0 / J_max) *
        ((((l2_tmp + J_max * V_max * 2.0) + -l13_tmp) + -l8_tmp) +
         l28_tmp * (1.0 / J_min)) /
        2.0;
  l19 = rt_powd_snf(J_max, 3.0);
  l10_idx_0_tmp = l13 * l13;
  t4_tmp[0].re = t7[0].re * t7[0].re - t7[0].im * t7[0].im;
  d = t7[0].re * t7[0].im;
  t4_tmp[0].im = d + d;
  if ((t7[0].im == 0.0) && (t7[0].re >= 0.0)) {
    y[0].re = rt_powd_snf(t7[0].re, 4.0);
    y[0].im = 0.0;
  } else if (t7[0].re == 0.0) {
    y[0].re = rt_powd_snf(t7[0].im, 4.0);
    y[0].im = 0.0;
  } else {
    if (t7[0].im == 0.0) {
      if (t7[0].re < 0.0) {
        l119.re = std::log(std::abs(t7[0].re));
        l119.im = 3.1415926535897931;
      } else {
        l119.re = std::log(std::abs(t7[0].re));
        l119.im = 0.0;
      }
    } else if ((std::abs(t7[0].re) > 8.9884656743115785E+307) ||
               (std::abs(t7[0].im) > 8.9884656743115785E+307)) {
      l119.re = std::log(rt_hypotd_snf(t7[0].re / 2.0, t7[0].im / 2.0)) +
                0.69314718055994529;
      l119.im = rt_atan2d_snf(t7[0].im, t7[0].re);
    } else {
      l119.re = std::log(rt_hypotd_snf(t7[0].re, t7[0].im));
      l119.im = rt_atan2d_snf(t7[0].im, t7[0].re);
    }
    l119.re *= 4.0;
    l119.im *= 4.0;
    if (l119.im == 0.0) {
      y[0].re = std::exp(l119.re);
      y[0].im = 0.0;
    } else if (rtIsInf(l119.im) && rtIsInf(l119.re) && (l119.re < 0.0)) {
      y[0].re = 0.0;
      y[0].im = 0.0;
    } else {
      l21 = std::exp(l119.re / 2.0);
      y[0].re = l21 * (l21 * std::cos(l119.im));
      y[0].im = l21 * (l21 * std::sin(l119.im));
    }
  }
  t4_tmp[1].re = t7[1].re * t7[1].re - t7[1].im * t7[1].im;
  d = t7[1].re * t7[1].im;
  t4_tmp[1].im = d + d;
  if ((t7[1].im == 0.0) && (t7[1].re >= 0.0)) {
    y[1].re = rt_powd_snf(t7[1].re, 4.0);
    y[1].im = 0.0;
  } else if (t7[1].re == 0.0) {
    y[1].re = rt_powd_snf(t7[1].im, 4.0);
    y[1].im = 0.0;
  } else {
    if (t7[1].im == 0.0) {
      if (t7[1].re < 0.0) {
        l119.re = std::log(std::abs(t7[1].re));
        l119.im = 3.1415926535897931;
      } else {
        l119.re = std::log(std::abs(t7[1].re));
        l119.im = 0.0;
      }
    } else if ((std::abs(t7[1].re) > 8.9884656743115785E+307) ||
               (std::abs(t7[1].im) > 8.9884656743115785E+307)) {
      l119.re = std::log(rt_hypotd_snf(t7[1].re / 2.0, t7[1].im / 2.0)) +
                0.69314718055994529;
      l119.im = rt_atan2d_snf(t7[1].im, t7[1].re);
    } else {
      l119.re = std::log(rt_hypotd_snf(t7[1].re, t7[1].im));
      l119.im = rt_atan2d_snf(t7[1].im, t7[1].re);
    }
    l119.re *= 4.0;
    l119.im *= 4.0;
    if (l119.im == 0.0) {
      y[1].re = std::exp(l119.re);
      y[1].im = 0.0;
    } else if (rtIsInf(l119.im) && rtIsInf(l119.re) && (l119.re < 0.0)) {
      y[1].re = 0.0;
      y[1].im = 0.0;
    } else {
      l21 = std::exp(l119.re / 2.0);
      y[1].re = l21 * (l21 * std::cos(l119.im));
      y[1].im = l21 * (l21 * std::sin(l119.im));
    }
  }
  t4_tmp[2].re = t7[2].re * t7[2].re - t7[2].im * t7[2].im;
  d = t7[2].re * t7[2].im;
  t4_tmp[2].im = d + d;
  if ((t7[2].im == 0.0) && (t7[2].re >= 0.0)) {
    y[2].re = rt_powd_snf(t7[2].re, 4.0);
    y[2].im = 0.0;
  } else if (t7[2].re == 0.0) {
    y[2].re = rt_powd_snf(t7[2].im, 4.0);
    y[2].im = 0.0;
  } else {
    if (t7[2].im == 0.0) {
      if (t7[2].re < 0.0) {
        l119.re = std::log(std::abs(t7[2].re));
        l119.im = 3.1415926535897931;
      } else {
        l119.re = std::log(std::abs(t7[2].re));
        l119.im = 0.0;
      }
    } else if ((std::abs(t7[2].re) > 8.9884656743115785E+307) ||
               (std::abs(t7[2].im) > 8.9884656743115785E+307)) {
      l119.re = std::log(rt_hypotd_snf(t7[2].re / 2.0, t7[2].im / 2.0)) +
                0.69314718055994529;
      l119.im = rt_atan2d_snf(t7[2].im, t7[2].re);
    } else {
      l119.re = std::log(rt_hypotd_snf(t7[2].re, t7[2].im));
      l119.im = rt_atan2d_snf(t7[2].im, t7[2].re);
    }
    l119.re *= 4.0;
    l119.im *= 4.0;
    if (l119.im == 0.0) {
      y[2].re = std::exp(l119.re);
      y[2].im = 0.0;
    } else if (rtIsInf(l119.im) && rtIsInf(l119.re) && (l119.re < 0.0)) {
      y[2].re = 0.0;
      y[2].im = 0.0;
    } else {
      l21 = std::exp(l119.re / 2.0);
      y[2].re = l21 * (l21 * std::cos(l119.im));
      y[2].im = l21 * (l21 * std::sin(l119.im));
    }
  }
  t4_tmp[3].re = t7[3].re * t7[3].re - t7[3].im * t7[3].im;
  d = t7[3].re * t7[3].im;
  t4_tmp[3].im = d + d;
  if ((t7[3].im == 0.0) && (t7[3].re >= 0.0)) {
    y[3].re = rt_powd_snf(t7[3].re, 4.0);
    y[3].im = 0.0;
  } else if (t7[3].re == 0.0) {
    y[3].re = rt_powd_snf(t7[3].im, 4.0);
    y[3].im = 0.0;
  } else {
    if (t7[3].im == 0.0) {
      if (t7[3].re < 0.0) {
        l119.re = std::log(std::abs(t7[3].re));
        l119.im = 3.1415926535897931;
      } else {
        l119.re = std::log(std::abs(t7[3].re));
        l119.im = 0.0;
      }
    } else if ((std::abs(t7[3].re) > 8.9884656743115785E+307) ||
               (std::abs(t7[3].im) > 8.9884656743115785E+307)) {
      l119.re = std::log(rt_hypotd_snf(t7[3].re / 2.0, t7[3].im / 2.0)) +
                0.69314718055994529;
      l119.im = rt_atan2d_snf(t7[3].im, t7[3].re);
    } else {
      l119.re = std::log(rt_hypotd_snf(t7[3].re, t7[3].im));
      l119.im = rt_atan2d_snf(t7[3].im, t7[3].re);
    }
    l119.re *= 4.0;
    l119.im *= 4.0;
    if (l119.im == 0.0) {
      y[3].re = std::exp(l119.re);
      y[3].im = 0.0;
    } else if (rtIsInf(l119.im) && rtIsInf(l119.re) && (l119.re < 0.0)) {
      y[3].re = 0.0;
      y[3].im = 0.0;
    } else {
      l21 = std::exp(l119.re / 2.0);
      y[3].re = l21 * (l21 * std::cos(l119.im));
      y[3].im = l21 * (l21 * std::sin(l119.im));
    }
  }
  coder::b_power(t7, dcv);
  l114.re = (((l99_tmp_tmp * l11_tmp * -3.0 - l99_tmp_tmp * l12_tmp * 3.0) -
              e_l99_tmp * l11_tmp * 3.0) +
             l7_tmp * l12_tmp) +
            f_l99_tmp * l99_tmp_tmp * 6.0;
  l7_tmp = l11_tmp * (l12_tmp * l12_tmp);
  l23 = A_min * d_l99_tmp;
  l119.re = l23 * l11_tmp * 4.0;
  l120.re = l23 * l12_tmp * 8.0;
  l122_re = g_l99_tmp * A_min * l11_tmp * 8.0;
  l121_re = l2_tmp * l8_tmp * l11_tmp * 6.0;
  l3.re = l5_tmp * l8_tmp * l12_tmp * 6.0;
  l125_re = h_l99_tmp * l11_tmp * l12_tmp * 12.0;
  l129_re = i_l99_tmp * l11_tmp * l12_tmp * 12.0;
  i_l99_tmp = l8_tmp * l11_tmp * l12_tmp;
  h_l99_tmp = b_l99_tmp_tmp * l19;
  g_l99_tmp = V_wayp * l11_tmp * l19;
  l135_re = j_l99_tmp * d_l99_tmp * 12.0;
  l136_re = b_l99_tmp * l2_tmp * l11_tmp * 12.0;
  l143_re = A_min * P_init * l11_tmp * l12_tmp * 24.0;
  l81 = A_min * P_wayp * l11_tmp * l12_tmp * 24.0;
  l19 = f_l99_tmp * l2_tmp;
  b_l99_tmp_tmp = l19 * l5_tmp * 6.0;
  j_l99_tmp = l19 * l8_tmp * 6.0;
  l36 = f_l99_tmp * l5_tmp * l8_tmp * 6.0;
  l19 = J_min * V_init;
  l99_tmp_tmp = l19 * l5_tmp * l12_tmp * 12.0;
  e_l99_tmp = l20 * l2_tmp * l11_tmp * 12.0;
  l80 = l19 * l8_tmp * l12_tmp * 12.0;
  l122_im = l20 * l8_tmp * l11_tmp * 12.0;
  l102_tmp = b_l99_tmp * l11_tmp * l12_tmp;
  b_l102_tmp = c_l99_tmp * l2_tmp * l11_tmp;
  l88 = c_l99_tmp_tmp * l8_tmp * l12_tmp;
  l86 = c_l99_tmp * l8_tmp * l11_tmp;
  l135_im = l102_tmp_tmp * l12_tmp;
  l19 = b_l99_tmp * J_min;
  l82 = l19 * J_max * l2_tmp * 12.0;
  l136_im = l99_tmp * J_max * V_init * l11_tmp * 24.0;
  l102 = l19 * V_init * l12_tmp * 24.0;
  l29 = l_l99_tmp * V_init * l11_tmp * 24.0;
  d = J_min * d_l99_tmp * l12_tmp * l13 * 12.0;
  l20 = J_max * d_l99_tmp * l11_tmp * l13 * 12.0;
  l21 = b_l102_tmp * l13 * 12.0;
  l73 = l88 * l13 * 24.0;
  l86_tmp = l86 * l13 * 12.0;
  l14 = A_max * J_min * l5_tmp * l12_tmp * l13 * 12.0;
  l18 = k_l99_tmp * l2_tmp * l11_tmp * l13 * 12.0;
  l23 = l135_im * l13 * 24.0;
  l19 = A_max * V_init * l11_tmp * l12_tmp * l13 * 24.0;
  y[0].re =
      -((((((((((((((((((((((((((((((((((((l114.re + l7_tmp * y[0].re * 3.0) +
                                          l119.re) +
                                         l120.re) +
                                        l122_re) +
                                       l121_re) -
                                      l3.re) -
                                     l125_re) +
                                    l129_re) +
                                   d) -
                                  l20) -
                                 i_l99_tmp * l10_idx_0_tmp * 12.0) +
                                h_l99_tmp * dcv[0].re * 4.0) -
                               g_l99_tmp * t4_tmp[0].re * 12.0) -
                              l135_re) -
                             l136_re) +
                            l143_re) -
                           l81) -
                          b_l99_tmp_tmp) -
                         j_l99_tmp) +
                        l36) +
                       l99_tmp_tmp) +
                      e_l99_tmp) +
                     l80) -
                    l122_im) +
                   l102_tmp * l10_idx_0_tmp * 12.0) -
                  l21) -
                 l73) +
                l86_tmp) +
               l14) +
              l18) +
             l23) -
            l19) +
           l82) -
          l136_im) -
         l102) +
        l29);
  y[0].im = -((l7_tmp * y[0].im * 3.0 + h_l99_tmp * dcv[0].im * 4.0) -
              g_l99_tmp * t4_tmp[0].im * 12.0);
  l135_im =
      (((b_l102_tmp * -12.0 - l88 * 12.0) + l86 * 12.0) + l135_im * 24.0) +
      l102_tmp * l13 * 24.0;
  y[1].re =
      -((((((((((((((((((((((((((((((((((((l114.re + l7_tmp * y[1].re * 3.0) +
                                          l119.re) +
                                         l120.re) +
                                        l122_re) +
                                       l121_re) -
                                      l3.re) -
                                     l125_re) +
                                    l129_re) +
                                   d) -
                                  l20) -
                                 i_l99_tmp * l10_idx_0_tmp * 12.0) +
                                h_l99_tmp * dcv[1].re * 4.0) -
                               g_l99_tmp * t4_tmp[1].re * 12.0) -
                              l135_re) -
                             l136_re) +
                            l143_re) -
                           l81) -
                          b_l99_tmp_tmp) -
                         j_l99_tmp) +
                        l36) +
                       l99_tmp_tmp) +
                      e_l99_tmp) +
                     l80) -
                    l122_im) +
                   l102_tmp * l10_idx_0_tmp * 12.0) -
                  l21) -
                 l73) +
                l86_tmp) +
               l14) +
              l18) +
             l23) -
            l19) +
           l82) -
          l136_im) -
         l102) +
        l29);
  y[1].im = -((l7_tmp * y[1].im * 3.0 + h_l99_tmp * dcv[1].im * 4.0) -
              g_l99_tmp * t4_tmp[1].im * 12.0);
  y[2].re =
      -((((((((((((((((((((((((((((((((((((l114.re + l7_tmp * y[2].re * 3.0) +
                                          l119.re) +
                                         l120.re) +
                                        l122_re) +
                                       l121_re) -
                                      l3.re) -
                                     l125_re) +
                                    l129_re) +
                                   d) -
                                  l20) -
                                 i_l99_tmp * l10_idx_0_tmp * 12.0) +
                                h_l99_tmp * dcv[2].re * 4.0) -
                               g_l99_tmp * t4_tmp[2].re * 12.0) -
                              l135_re) -
                             l136_re) +
                            l143_re) -
                           l81) -
                          b_l99_tmp_tmp) -
                         j_l99_tmp) +
                        l36) +
                       l99_tmp_tmp) +
                      e_l99_tmp) +
                     l80) -
                    l122_im) +
                   l102_tmp * l10_idx_0_tmp * 12.0) -
                  l21) -
                 l73) +
                l86_tmp) +
               l14) +
              l18) +
             l23) -
            l19) +
           l82) -
          l136_im) -
         l102) +
        l29);
  y[2].im = -((l7_tmp * y[2].im * 3.0 + h_l99_tmp * dcv[2].im * 4.0) -
              g_l99_tmp * t4_tmp[2].im * 12.0);
  y[3].re =
      -((((((((((((((((((((((((((((((((((((l114.re + l7_tmp * y[3].re * 3.0) +
                                          l119.re) +
                                         l120.re) +
                                        l122_re) +
                                       l121_re) -
                                      l3.re) -
                                     l125_re) +
                                    l129_re) +
                                   d) -
                                  l20) -
                                 i_l99_tmp * l10_idx_0_tmp * 12.0) +
                                h_l99_tmp * dcv[3].re * 4.0) -
                               g_l99_tmp * t4_tmp[3].re * 12.0) -
                              l135_re) -
                             l136_re) +
                            l143_re) -
                           l81) -
                          b_l99_tmp_tmp) -
                         j_l99_tmp) +
                        l36) +
                       l99_tmp_tmp) +
                      e_l99_tmp) +
                     l80) -
                    l122_im) +
                   l102_tmp * l10_idx_0_tmp * 12.0) -
                  l21) -
                 l73) +
                l86_tmp) +
               l14) +
              l18) +
             l23) -
            l19) +
           l82) -
          l136_im) -
         l102) +
        l29);
  y[3].im = -((l7_tmp * y[3].im * 3.0 + h_l99_tmp * dcv[3].im * 4.0) -
              g_l99_tmp * t4_tmp[3].im * 12.0);
  b_l99_tmp_tmp = m_l99_tmp * 2.0 + l28_tmp;
  l121_re = (l2_tmp + l8_tmp) + l13_tmp;
  l122_re = l5_tmp * J_max * l17_tmp;
  j_l99_tmp = f_l99_tmp * V_wayp * 2.0;
  l73 = c_l99_tmp * 2.0;
  l19 = A_min * (1.0 / J_min);
  l20 = l19;
  l19 = A_max * (1.0 / J_min);
  l21 = -(1.0 / J_max * (A_init + -A_max));
  t[0].re = l21;
  t[0].im = 0.0;
  t[1].re = l21;
  t[1].im = 0.0;
  t[2].re = l21;
  t[2].im = 0.0;
  t[3].re = l21;
  t[3].im = 0.0;
  t[8].re = -l19;
  t[8].im = 0.0;
  t[9].re = -l19;
  t[9].im = 0.0;
  t[10].re = -l19;
  t[10].im = 0.0;
  t[11].re = -l19;
  t[11].im = 0.0;
  l19 = k_l99_tmp * l13 * 2.0;
  l86_tmp =
      l17_tmp *
      ((b_l99_tmp_tmp - J_min * ((((l121_re + l12_tmp * t4_tmp[0].re) + l19) +
                                  c_l99_tmp * t7[0].re * 2.0) +
                                 l122_re)) +
       j_l99_tmp);
  l14 = l17_tmp *
        (0.0 - J_min * (l12_tmp * t4_tmp[0].im + c_l99_tmp * t7[0].im * 2.0));
  t[4].re = l13;
  t[4].im = 0.0;
  if (y[0].im == 0.0) {
    t[12].re = y[0].re / l135_im;
    t[12].im = 0.0;
  } else if (y[0].re == 0.0) {
    t[12].re = 0.0;
    t[12].im = y[0].im / l135_im;
  } else {
    t[12].re = y[0].re / l135_im;
    t[12].im = y[0].im / l135_im;
  }
  t[16].re = l20;
  t[16].im = 0.0;
  if (l14 == 0.0) {
    t[20].re = l86_tmp / l73;
    t[20].im = 0.0;
  } else if (l86_tmp == 0.0) {
    t[20].re = 0.0;
    t[20].im = l14 / l73;
  } else {
    t[20].re = l86_tmp / l73;
    t[20].im = l14 / l73;
  }
  t[24] = t7[0];
  l86_tmp =
      l17_tmp *
      ((b_l99_tmp_tmp - J_min * ((((l121_re + l12_tmp * t4_tmp[1].re) + l19) +
                                  c_l99_tmp * t7[1].re * 2.0) +
                                 l122_re)) +
       j_l99_tmp);
  l14 = l17_tmp *
        (0.0 - J_min * (l12_tmp * t4_tmp[1].im + c_l99_tmp * t7[1].im * 2.0));
  t[5].re = l13;
  t[5].im = 0.0;
  if (y[1].im == 0.0) {
    t[13].re = y[1].re / l135_im;
    t[13].im = 0.0;
  } else if (y[1].re == 0.0) {
    t[13].re = 0.0;
    t[13].im = y[1].im / l135_im;
  } else {
    t[13].re = y[1].re / l135_im;
    t[13].im = y[1].im / l135_im;
  }
  t[17].re = l20;
  t[17].im = 0.0;
  if (l14 == 0.0) {
    t[21].re = l86_tmp / l73;
    t[21].im = 0.0;
  } else if (l86_tmp == 0.0) {
    t[21].re = 0.0;
    t[21].im = l14 / l73;
  } else {
    t[21].re = l86_tmp / l73;
    t[21].im = l14 / l73;
  }
  t[25] = t7[1];
  l86_tmp =
      l17_tmp *
      ((b_l99_tmp_tmp - J_min * ((((l121_re + l12_tmp * t4_tmp[2].re) + l19) +
                                  c_l99_tmp * t7[2].re * 2.0) +
                                 l122_re)) +
       j_l99_tmp);
  l14 = l17_tmp *
        (0.0 - J_min * (l12_tmp * t4_tmp[2].im + c_l99_tmp * t7[2].im * 2.0));
  t[6].re = l13;
  t[6].im = 0.0;
  if (y[2].im == 0.0) {
    t[14].re = y[2].re / l135_im;
    t[14].im = 0.0;
  } else if (y[2].re == 0.0) {
    t[14].re = 0.0;
    t[14].im = y[2].im / l135_im;
  } else {
    t[14].re = y[2].re / l135_im;
    t[14].im = y[2].im / l135_im;
  }
  t[18].re = l20;
  t[18].im = 0.0;
  if (l14 == 0.0) {
    t[22].re = l86_tmp / l73;
    t[22].im = 0.0;
  } else if (l86_tmp == 0.0) {
    t[22].re = 0.0;
    t[22].im = l14 / l73;
  } else {
    t[22].re = l86_tmp / l73;
    t[22].im = l14 / l73;
  }
  t[26] = t7[2];
  l86_tmp =
      l17_tmp *
      ((b_l99_tmp_tmp - J_min * ((((l121_re + l12_tmp * t4_tmp[3].re) + l19) +
                                  c_l99_tmp * t7[3].re * 2.0) +
                                 l122_re)) +
       j_l99_tmp);
  l14 = l17_tmp *
        (0.0 - J_min * (l12_tmp * t4_tmp[3].im + c_l99_tmp * t7[3].im * 2.0));
  t[7].re = l13;
  t[7].im = 0.0;
  if (y[3].im == 0.0) {
    t[15].re = y[3].re / l135_im;
    t[15].im = 0.0;
  } else if (y[3].re == 0.0) {
    t[15].re = 0.0;
    t[15].im = y[3].im / l135_im;
  } else {
    t[15].re = y[3].re / l135_im;
    t[15].im = y[3].im / l135_im;
  }
  t[19].re = l20;
  t[19].im = 0.0;
  if (l14 == 0.0) {
    t[23].re = l86_tmp / l73;
    t[23].im = 0.0;
  } else if (l86_tmp == 0.0) {
    t[23].re = 0.0;
    t[23].im = l14 / l73;
  } else {
    t[23].re = l86_tmp / l73;
    t[23].im = l14 / l73;
  }
  t[27] = t7[3];
}

// End of code generation (abcdefg_T_VP.cpp)
