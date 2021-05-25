//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// abcdefg_T_P.cpp
//
// Code generation for function 'abcdefg_T_P'
//

// Include files
#include "abcdefg_T_P.h"
#include "power.h"
#include "rt_nonfinite.h"
#include "sqrt.h"
#include "topico_wrapper_data.h"
#include "topico_wrapper_rtwutil.h"
#include <cmath>

// Function Definitions
void abcdefg_T_P(double P_init, double V_init, double A_init, double P_wayp,
                 double V_max, double V_min, double A_max, double A_min,
                 double J_max, double J_min, double T, creal_T t[21])
{
  creal_T t7[3];
  creal_T x[3];
  creal_T b_l36;
  creal_T l96;
  double J_max_re;
  double J_min_re;
  double b_A_max;
  double b_J_max_re;
  double b_l77_tmp;
  double b_l89_tmp;
  double b_l89_tmp_tmp;
  double c_l89_tmp;
  double c_l89_tmp_tmp;
  double d;
  double d_l89_tmp;
  double e_l89_tmp;
  double f_l89_tmp;
  double g_l89_tmp;
  double h_l89_tmp;
  double i_l89_tmp;
  double j_l89_tmp;
  double k_l89_tmp;
  double l100_re;
  double l103_re;
  double l105_re;
  double l10_tmp;
  double l11_tmp;
  double l14;
  double l15;
  double l16;
  double l17;
  double l18;
  double l19;
  double l20;
  double l21;
  double l29_tmp;
  double l2_tmp;
  double l3;
  double l32;
  double l36;
  double l5_tmp;
  double l6_tmp;
  double l77;
  double l77_tmp;
  double l78;
  double l82;
  double l89_tmp;
  double l89_tmp_tmp;
  double l8_tmp;
  double l99_re;
  double l_l89_tmp;
  double m_l89_tmp;
  double n_l89_tmp;
  double o_l89_tmp;
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
  //  Generated on 03-Sep-2019 14:12:58
  l2_tmp = A_init * A_init;
  l5_tmp = A_min * A_min;
  l6_tmp = rt_powd_snf(A_min, 3.0);
  l8_tmp = A_max * A_max;
  l10_tmp = J_min * J_min;
  l11_tmp = J_max * J_max;
  l14 = V_max * V_max;
  l15 = 1.0 / A_min;
  l16 = 1.0 / A_max;
  l17 = 1.0 / J_min;
  l19 = 1.0 / J_max;
  l18 = 1.0 / l10_tmp;
  l20 = 1.0 / l11_tmp;
  l21 = rt_powd_snf(l19, 3.0);
  l29_tmp = J_max * l8_tmp;
  l32 = A_min * l19;
  l36 = l6_tmp * l21;
  l77_tmp = A_min * A_max;
  b_l77_tmp = A_max * J_max;
  l77 = (b_l77_tmp * l6_tmp * l10_tmp * 12.0 +
         l77_tmp * V_min * l10_tmp * l11_tmp * 24.0) +
        -(l77_tmp * V_max * l10_tmp * l11_tmp * 24.0);
  l78 = l16 * l18 * (l20 * l20) * l77 / 8.0;
  l82 = l5_tmp * l20 + -(l15 * l16 * l18 * l21 * l77 / 12.0);
  l89_tmp = J_min * J_max;
  l89_tmp_tmp = l8_tmp * l8_tmp;
  l18 = A_min * l89_tmp_tmp;
  b_l89_tmp_tmp = l5_tmp * l5_tmp;
  l21 = A_max * b_l89_tmp_tmp;
  l77 = A_min * J_max;
  b_l89_tmp = A_min * l10_tmp * l11_tmp;
  c_l89_tmp = A_max * l10_tmp * l11_tmp;
  d_l89_tmp = rt_powd_snf(A_init, 3.0);
  e_l89_tmp = V_init * V_init;
  f_l89_tmp = V_min * V_min;
  c_l89_tmp_tmp = A_max * J_min;
  g_l89_tmp = c_l89_tmp_tmp * J_max;
  h_l89_tmp = A_min * J_min;
  i_l89_tmp = A_min * V_min;
  j_l89_tmp = A_max * V_min;
  k_l89_tmp = A_init * A_max;
  l_l89_tmp = l89_tmp * V_init * 2.0;
  m_l89_tmp = l89_tmp * V_min * 2.0;
  n_l89_tmp = l2_tmp * l2_tmp;
  o_l89_tmp = J_min * l5_tmp;
  l18 = V_max * l16 * l17 * l20 *
        (((((((k_l89_tmp * J_min * 2.0 + l_l89_tmp) + m_l89_tmp) + o_l89_tmp) +
            l29_tmp) +
           -(l89_tmp * V_max * 2.0)) +
          g_l89_tmp * T * 2.0) +
         -(l15 * l17 * l19 * (1.0 / V_max) *
           (((((((((((((((((((((((l18 * l11_tmp +
                                  A_min * n_l89_tmp * l10_tmp * 3.0) +
                                 -(l18 * l10_tmp)) +
                                -(l21 * l10_tmp * 3.0)) +
                               -(l21 * l11_tmp)) +
                              J_max * V_max * l6_tmp * l10_tmp * 12.0) +
                             A_init * A_min * A_max * J_max * V_init * l10_tmp *
                                 24.0) +
                            -(l77_tmp * d_l89_tmp * l10_tmp * 8.0)) +
                           l77_tmp * P_wayp * l10_tmp * l11_tmp * 24.0) +
                          l77 * V_max * l2_tmp * l10_tmp * 12.0) +
                         h_l89_tmp * V_max * l8_tmp * l11_tmp * 12.0) +
                        A_min * V_max * l10_tmp * l29_tmp * 12.0) +
                       b_l77_tmp * V_max * l5_tmp * l10_tmp * 12.0) +
                      A_min * l2_tmp * l8_tmp * l10_tmp * 6.0) +
                     i_l89_tmp * V_max * l10_tmp * l11_tmp * 24.0) +
                    j_l89_tmp * V_max * l10_tmp * l11_tmp * 24.0) +
                   -(l77_tmp * P_init * l10_tmp * l11_tmp * 24.0)) +
                  -(l77 * V_init * l2_tmp * l10_tmp * 12.0)) +
                 -(A_min * V_init * l10_tmp * l29_tmp * 12.0)) +
                -(b_l77_tmp * V_min * l5_tmp * l10_tmp * 12.0)) +
               b_l89_tmp * e_l89_tmp * 12.0) +
              -(b_l89_tmp * l14 * 12.0)) +
             -(c_l89_tmp * f_l89_tmp * 12.0)) +
            -(c_l89_tmp * l14 * 12.0)) /
           12.0)) *
        1.5;
  l21 = (l36 + -l78) + l18;
  l96.re = -rt_powd_snf(l82, 3.0) + l21 * l21;
  l96.im = 0.0;
  coder::internal::scalar::b_sqrt(&l96);
  b_l36.re = ((-l36 + l78) + -l18) + l96.re;
  b_l36.im = l96.im;
  l96 = coder::power(b_l36);
  if (l96.im == 0.0) {
    c_l89_tmp = l96.re / 2.0;
    l16 = 0.0;
    l15 = 1.0 / l96.re;
    l18 = 0.0;
  } else if (l96.re == 0.0) {
    c_l89_tmp = 0.0;
    l16 = l96.im / 2.0;
    l15 = 0.0;
    l18 = -(1.0 / l96.im);
  } else {
    c_l89_tmp = l96.re / 2.0;
    l16 = l96.im / 2.0;
    l77 = std::abs(l96.re);
    l18 = std::abs(l96.im);
    if (l77 > l18) {
      l18 = l96.im / l96.re;
      l21 = l96.re + l18 * l96.im;
      l15 = (l18 * 0.0 + 1.0) / l21;
      l18 = (0.0 - l18) / l21;
    } else if (l18 == l77) {
      if (l96.re > 0.0) {
        l18 = 0.5;
      } else {
        l18 = -0.5;
      }
      if (l96.im > 0.0) {
        l21 = 0.5;
      } else {
        l21 = -0.5;
      }
      l15 = (l18 + 0.0 * l21) / l77;
      l18 = (0.0 * l18 - l21) / l77;
    } else {
      l18 = l96.re / l96.im;
      l21 = l96.im + l18 * l96.re;
      l15 = l18 / l21;
      l18 = (l18 * 0.0 - 1.0) / l21;
    }
  }
  l100_re = l82 * l15;
  l21 = l82 * l18;
  if (l21 == 0.0) {
    l77 = l100_re / 2.0;
    b_l89_tmp = 0.0;
  } else if (l100_re == 0.0) {
    l77 = 0.0;
    b_l89_tmp = l21 / 2.0;
  } else {
    l77 = l100_re / 2.0;
    b_l89_tmp = l21 / 2.0;
  }
  l15 = 1.7320508075688772 * (l96.re + -l100_re);
  l18 = 1.7320508075688772 * (l96.im + -l21);
  l105_re = l15 * 0.0 - l18 * 0.5;
  l18 = l15 * 0.5 + l18 * 0.0;
  t7[0].re = (-l32 + l96.re) + l100_re;
  t7[0].im = l96.im + l21;
  d = (-l32 + -c_l89_tmp) + -l77;
  t7[1].re = d - l105_re;
  l32 = -l16 + -b_l89_tmp;
  t7[1].im = l32 - l18;
  t7[2].re = d + l105_re;
  t7[2].im = l32 + l18;
  l21 = J_max * V_min;
  l14 = 1.0 / A_min * (1.0 / J_min) * (1.0 / J_max) *
        (J_min * (l5_tmp + l21 * 2.0) +
         -(J_max * (l5_tmp + J_min * V_max * 2.0))) /
        2.0;
  coder::c_power(t7, x);
  l3 = l5_tmp * l10_tmp * l11_tmp;
  b_A_max = A_max * rt_powd_snf(J_max, 3.0) * l10_tmp;
  l96.re = g_l89_tmp * l6_tmp * 12.0;
  l99_re = A_max * P_init * l10_tmp * l11_tmp * 24.0;
  l100_re = A_max * P_wayp * l10_tmp * l11_tmp * 24.0;
  l103_re = l89_tmp * l5_tmp * l8_tmp * 6.0;
  l18 = J_max * V_init;
  l105_re = l18 * l2_tmp * l10_tmp * 12.0;
  J_max_re = l18 * l8_tmp * l10_tmp * 12.0;
  l18 = J_min * V_min;
  J_min_re = l18 * l5_tmp * l11_tmp * 12.0;
  b_J_max_re = l21 * l5_tmp * l10_tmp * 12.0;
  l78 = l18 * l8_tmp * l11_tmp * 12.0;
  l82 = l77_tmp * l10_tmp * l11_tmp;
  l77 = c_l89_tmp_tmp * l5_tmp * l11_tmp;
  l36 = b_l77_tmp * l5_tmp * l10_tmp;
  l20 = j_l89_tmp * l10_tmp * l11_tmp;
  l19 = k_l89_tmp * J_max * V_init * l10_tmp * 24.0;
  l16 = l77_tmp * J_min * V_min * l11_tmp * 24.0;
  d = l14 * l14;
  l17 = t7[0].re * t7[0].im;
  l32 = ((((((((((((b_l89_tmp_tmp * l10_tmp * 3.0 +
                    b_l89_tmp_tmp * l11_tmp * 3.0) +
                   l89_tmp_tmp * l10_tmp) -
                  l89_tmp_tmp * l11_tmp) -
                 n_l89_tmp * l10_tmp * 3.0) -
                l89_tmp * b_l89_tmp_tmp * 6.0) -
               A_max * l6_tmp * l11_tmp * 8.0) +
              d_l89_tmp * A_max * l10_tmp * 8.0) -
             l2_tmp * l8_tmp * l10_tmp * 6.0) +
            l5_tmp * l8_tmp * l11_tmp * 6.0) -
           e_l89_tmp * l10_tmp * l11_tmp * 12.0) +
          f_l89_tmp * l10_tmp * l11_tmp * 12.0) +
         J_min * l6_tmp * l11_tmp * l14 * 12.0) -
        J_max * l6_tmp * l10_tmp * l14 * 12.0;
  b_l89_tmp = l77 * l14 * 24.0;
  c_l89_tmp = h_l89_tmp * l8_tmp * l11_tmp * l14 * 12.0;
  l15 = l36 * l14 * 12.0;
  l18 = i_l89_tmp * l10_tmp * l11_tmp * l14 * 24.0;
  l21 = l20 * l14 * 24.0;
  x[0].re =
      (((((((((((((((((((((l32 + l3 * d * 12.0) + b_A_max * x[0].re * 4.0) +
                         l96.re) +
                        l99_re) -
                       l100_re) -
                      l103_re) +
                     l105_re) +
                    J_max_re) -
                   J_min_re) +
                  b_J_max_re) -
                 l78) -
                l82 * d * 12.0) -
               b_l89_tmp) +
              c_l89_tmp) +
             l15) +
            l36 * t7[0].re * 12.0) -
           l18) +
          l21) +
         l20 * t7[0].re * 24.0) +
        l82 * (t7[0].re * t7[0].re - t7[0].im * t7[0].im) * 12.0) -
       l19) +
      l16;
  x[0].im = ((b_A_max * x[0].im * 4.0 + l36 * t7[0].im * 12.0) +
             l20 * t7[0].im * 24.0) +
            l82 * (l17 + l17) * 12.0;
  l77 = ((l77 * 12.0 - l36 * 12.0) - l20 * 24.0) + l82 * l14 * 24.0;
  l17 = t7[1].re * t7[1].im;
  x[1].re =
      (((((((((((((((((((((l32 + l3 * d * 12.0) + b_A_max * x[1].re * 4.0) +
                         l96.re) +
                        l99_re) -
                       l100_re) -
                      l103_re) +
                     l105_re) +
                    J_max_re) -
                   J_min_re) +
                  b_J_max_re) -
                 l78) -
                l82 * d * 12.0) -
               b_l89_tmp) +
              c_l89_tmp) +
             l15) +
            l36 * t7[1].re * 12.0) -
           l18) +
          l21) +
         l20 * t7[1].re * 24.0) +
        l82 * (t7[1].re * t7[1].re - t7[1].im * t7[1].im) * 12.0) -
       l19) +
      l16;
  x[1].im = ((b_A_max * x[1].im * 4.0 + l36 * t7[1].im * 12.0) +
             l20 * t7[1].im * 24.0) +
            l82 * (l17 + l17) * 12.0;
  l17 = t7[2].re * t7[2].im;
  x[2].re =
      (((((((((((((((((((((l32 + l3 * d * 12.0) + b_A_max * x[2].re * 4.0) +
                         l96.re) +
                        l99_re) -
                       l100_re) -
                      l103_re) +
                     l105_re) +
                    J_max_re) -
                   J_min_re) +
                  b_J_max_re) -
                 l78) -
                l82 * d * 12.0) -
               b_l89_tmp) +
              c_l89_tmp) +
             l15) +
            l36 * t7[2].re * 12.0) -
           l18) +
          l21) +
         l20 * t7[2].re * 24.0) +
        l82 * (t7[2].re * t7[2].re - t7[2].im * t7[2].im) * 12.0) -
       l19) +
      l16;
  x[2].im = ((b_A_max * x[2].im * 4.0 + l36 * t7[2].im * 12.0) +
             l20 * t7[2].im * 24.0) +
            l82 * (l17 + l17) * 12.0;
  l18 = -(1.0 / J_max * (A_init + -A_max));
  t[0].re = l18;
  t[0].im = 0.0;
  t[1].re = l18;
  t[1].im = 0.0;
  t[2].re = l18;
  t[2].im = 0.0;
  d = (((((((o_l89_tmp - J_min * l8_tmp) - J_max * l5_tmp) + l29_tmp) +
          l2_tmp * J_min) -
         l_l89_tmp) +
        m_l89_tmp) -
       h_l89_tmp * J_max * l14 * 2.0) /
      (g_l89_tmp * 2.0);
  t[3].re = d;
  t[3].im = 0.0;
  if (x[0].im == 0.0) {
    t[9].re = x[0].re / l77;
    t[9].im = 0.0;
  } else if (x[0].re == 0.0) {
    t[9].re = 0.0;
    t[9].im = x[0].im / l77;
  } else {
    t[9].re = x[0].re / l77;
    t[9].im = x[0].im / l77;
  }
  t[4].re = d;
  t[4].im = 0.0;
  if (x[1].im == 0.0) {
    t[10].re = x[1].re / l77;
    t[10].im = 0.0;
  } else if (x[1].re == 0.0) {
    t[10].re = 0.0;
    t[10].im = x[1].im / l77;
  } else {
    t[10].re = x[1].re / l77;
    t[10].im = x[1].im / l77;
  }
  t[5].re = d;
  t[5].im = 0.0;
  if (x[2].im == 0.0) {
    t[11].re = x[2].re / l77;
    t[11].im = 0.0;
  } else if (x[2].re == 0.0) {
    t[11].re = 0.0;
    t[11].im = x[2].im / l77;
  } else {
    t[11].re = x[2].re / l77;
    t[11].im = x[2].im / l77;
  }
  l3 = A_min * (1.0 / J_min);
  l18 = l3;
  l3 = A_max * (1.0 / J_min);
  t[6].re = -l3;
  t[6].im = 0.0;
  t[7].re = -l3;
  t[7].im = 0.0;
  t[8].re = -l3;
  t[8].im = 0.0;
  t[12].re = l18;
  t[12].im = 0.0;
  t[15].re = l14;
  t[15].im = 0.0;
  t[18] = t7[0];
  t[13].re = l18;
  t[13].im = 0.0;
  t[16].re = l14;
  t[16].im = 0.0;
  t[19] = t7[1];
  t[14].re = l18;
  t[14].im = 0.0;
  t[17].re = l14;
  t[17].im = 0.0;
  t[20] = t7[2];
}

// End of code generation (abcdefg_T_P.cpp)
