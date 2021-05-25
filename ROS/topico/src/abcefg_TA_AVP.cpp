//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// abcefg_TA_AVP.cpp
//
// Code generation for function 'abcefg_TA_AVP'
//

// Include files
#include "abcefg_TA_AVP.h"
#include "power.h"
#include "rt_nonfinite.h"
#include "sqrt.h"
#include "topico_wrapper_data.h"
#include "topico_wrapper_rtwutil.h"
#include <cmath>

// Function Definitions
void abcefg_TA_AVP(double P_init, double V_init, double A_init, double P_wayp,
                   double V_wayp, double A_wayp, double A_max, double A_min,
                   double J_max, double J_min, double T, creal_T t[28])
{
  creal_T l2[4];
  creal_T t1[4];
  creal_T t2[4];
  creal_T t3[4];
  creal_T l309;
  creal_T l312;
  creal_T l314;
  creal_T l315;
  creal_T l323;
  double a_tmp;
  double b_l236_tmp;
  double b_l243_tmp;
  double b_l243_tmp_tmp;
  double b_l247_tmp;
  double b_l254_tmp;
  double b_l78_tmp;
  double c_l236_tmp;
  double c_l243_tmp;
  double c_l247_tmp;
  double c_l254_tmp;
  double d_l254_tmp;
  double e_l254_tmp;
  double f_l254_tmp;
  double g_l254_tmp;
  double h_l254_tmp;
  double i_l254_tmp;
  double j_l254_tmp;
  double k_l254_tmp;
  double l10_tmp;
  double l11;
  double l12;
  double l13;
  double l15;
  double l156;
  double l16;
  double l17_tmp;
  double l18;
  double l236_tmp;
  double l236_tmp_tmp;
  double l238;
  double l239;
  double l243_tmp;
  double l243_tmp_tmp;
  double l247_tmp;
  double l250;
  double l254_tmp;
  double l259;
  double l268;
  double l280;
  double l282;
  double l289;
  double l2_tmp;
  double l3;
  double l324_im;
  double l327_im;
  double l330_im;
  double l330_re;
  double l4;
  double l5;
  double l6;
  double l62;
  double l63;
  double l64;
  double l78;
  double l78_tmp;
  double l78_tmp_tmp;
  double l7_tmp;
  double l8;
  double l9;
  double l_l254_tmp;
  double m_l254_tmp;
  double n_l254_tmp;
  double o_l254_tmp;
  double p_l254_tmp;
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
  //  Generated on 28-Sep-2020 17:24:53
  l2_tmp = A_init * A_init;
  l3 = rt_powd_snf(A_init, 3.0);
  l5 = A_min * A_min;
  l6 = A_max * A_max;
  l7_tmp = A_wayp * A_wayp;
  l8 = rt_powd_snf(A_wayp, 3.0);
  l10_tmp = J_min * J_min;
  l11 = rt_powd_snf(J_min, 3.0);
  l13 = J_max * J_max;
  l15 = V_init * V_init;
  l16 = V_wayp * V_wayp;
  l17_tmp = A_init * A_min;
  l18 = A_init * A_max;
  l4 = l2_tmp * l2_tmp;
  l9 = l7_tmp * l7_tmp;
  l12 = l10_tmp * l10_tmp;
  l78_tmp = A_min * A_max;
  l78_tmp_tmp = l78_tmp * J_min;
  b_l78_tmp = l78_tmp_tmp * J_max;
  l78 = b_l78_tmp * l7_tmp * 12.0;
  l62 = l17_tmp + -l18;
  l63 = 1.0 / (A_min + -A_max);
  l64 = l63 * l63;
  l156 = l62 * l62;
  l330_im = l78_tmp * A_wayp;
  l243_tmp = A_wayp * J_max;
  b_l243_tmp = A_init * J_max;
  l324_im = A_init * l6;
  l243_tmp_tmp = J_max * T;
  l289 = l243_tmp_tmp * l6;
  b_l243_tmp_tmp = A_max * J_max;
  l282 = b_l243_tmp_tmp * l10_tmp;
  c_l243_tmp = l78_tmp * J_max;
  l254_tmp = J_min * J_max;
  b_l254_tmp = l254_tmp * l2_tmp;
  c_l254_tmp = J_min * V_init;
  d_l254_tmp = J_max * V_init;
  e_l254_tmp = J_max * V_wayp;
  f_l254_tmp = J_min * V_wayp;
  g_l254_tmp = l78_tmp * l10_tmp * l13;
  h_l254_tmp = A_init * A_wayp;
  i_l254_tmp = l243_tmp * T;
  j_l254_tmp = d_l254_tmp * l5;
  k_l254_tmp = d_l254_tmp * l6;
  l_l254_tmp = e_l254_tmp * l5;
  m_l254_tmp = e_l254_tmp * l6;
  n_l254_tmp = l2_tmp * l5;
  o_l254_tmp = l2_tmp * l6;
  p_l254_tmp = b_l243_tmp * T;
  l236_tmp = A_wayp * J_min;
  b_l236_tmp = A_max * J_min;
  c_l236_tmp = A_max * A_wayp * J_min * J_max;
  l236_tmp_tmp = A_init * J_min;
  l280 = l236_tmp_tmp * J_max;
  l268 = l236_tmp * J_max;
  l238 = 1.0 /
         (J_min *
              ((J_min * l13 * 3.0 + J_max * l10_tmp * 3.0) + l282 * l63 * 6.0) *
              2.0 +
          -l64 * (((((((l78_tmp * l12 * 2.0 - l5 * l12) - l6 * l12) +
                      J_max * l5 * l11 * 6.0) -
                     J_max * l6 * l11 * 6.0) -
                    g_l254_tmp * 14.0) +
                   l5 * l10_tmp * l13 * 7.0) +
                  l6 * l10_tmp * l13 * 7.0));
  l247_tmp = h_l254_tmp * J_max;
  b_l247_tmp = b_l236_tmp * J_max;
  c_l247_tmp = b_l236_tmp * l7_tmp;
  a_tmp =
      J_min *
          (((((((l280 * 6.0 + l268 * 6.0) + l254_tmp * l18 * l63 * 6.0) +
               c_l236_tmp * l63 * -6.0) +
              b_l236_tmp * T * l13 * l63 * 6.0) +
             l324_im * l10_tmp * l64 * 6.0) +
            l289 * l10_tmp * l64 * 6.0) +
           -(l254_tmp * l62 * l63 * 6.0)) *
          2.0 -
      l64 * (((((((((((A_max * l11 * l17_tmp * 12.0 + l282 * l17_tmp * 12.0) +
                      c_l243_tmp * T * l11 * 12.0) +
                     -(l330_im * l11 * 12.0)) +
                    l324_im * l11 * 12.0) +
                   -(l330_im * J_max * l10_tmp * 36.0)) +
                  l243_tmp * l5 * l10_tmp * 12.0) +
                 l243_tmp * l6 * l10_tmp * 24.0) +
                l289 * l11 * 12.0) +
               -(b_l243_tmp * l6 * l10_tmp * 12.0)) +
              l78_tmp * T * l10_tmp * l13 * 12.0) +
             -(T * l6 * l10_tmp * l13 * 12.0));
  l250 = a_tmp * a_tmp;
  l324_im = l330_im * J_min;
  l289 = l324_im * J_max;
  l282 = J_min * P_init;
  l330_im = l78_tmp_tmp * T;
  l327_im = J_min * T;
  l330_re = l254_tmp * T;
  l268 = J_min *
             (((((((((((((l3 * 2.0 + l8 * 6.0) + l243_tmp * V_init * 12.0) +
                        A_wayp * l2_tmp * 6.0) +
                       -(l243_tmp * V_wayp * 12.0)) +
                      -(A_init * l7_tmp * 6.0)) +
                     P_wayp * l13 * 12.0) +
                    l247_tmp * T * 12.0) +
                   A_init * l64 * l156 * 6.0) +
                  l7_tmp * l62 * l63 * 6.0) +
                 l243_tmp_tmp * l64 * l156 * 6.0) +
                -(h_l254_tmp * l62 * l63 * 12.0)) +
               -(l2_tmp * l62 * l63 * 6.0)) +
              -(i_l254_tmp * l62 * l63 * 12.0)) *
             2.0 +
         -(l64 *
           (((((((((((((((((((((((((((b_l247_tmp * V_init * l17_tmp * 24.0 +
                                      l289 * V_wayp * 24.0) +
                                     J_min * l5 * l8 * 4.0) +
                                    -(b_l247_tmp * V_wayp * l17_tmp * 24.0)) +
                                   -(l289 * V_init * 24.0)) +
                                  c_l247_tmp * l17_tmp * 12.0) +
                                 l324_im * l2_tmp * 12.0) +
                                l280 * V_wayp * l6 * 24.0) +
                               l268 * V_init * l6 * 24.0) +
                              -(l78_tmp_tmp * l3 * 12.0)) +
                             -(l78_tmp_tmp * l8 * 20.0)) +
                            J_min * l3 * l6 * 12.0) +
                           J_min * l6 * l8 * 16.0) +
                          -(l78_tmp_tmp * P_init * l13 * 48.0)) +
                         -(l280 * V_init * l6 * 24.0)) +
                        -(l268 * V_wayp * l6 * 24.0)) +
                       l282 * l5 * l13 * 24.0) +
                      l282 * l6 * l13 * 24.0) +
                     T * l78) +
                    -(l236_tmp_tmp * l6 * l7_tmp * 12.0)) +
                   -(l236_tmp * l2_tmp * l6 * 12.0)) +
                  b_l78_tmp * T * l2_tmp * -12.0) +
                 l330_im * V_init * l13 * -24.0) +
                l330_im * V_wayp * l13 * -24.0) +
               l330_re * l2_tmp * l6 * 12.0) +
              l327_im * V_init * l5 * l13 * 24.0) +
             l327_im * V_wayp * l6 * l13 * 24.0) +
            -(l330_re * l6 * l7_tmp * 12.0)));
  l239 = l238 * l238;
  l12 = rt_powd_snf(l238, 3.0);
  l259 = l238 * a_tmp * -0.25;
  l8 = c_l243_tmp * V_init;
  l324_im = c_l243_tmp * V_wayp;
  l289 = e_l254_tmp * l2_tmp;
  l282 = d_l254_tmp * l2_tmp;
  l330_im = l78_tmp * l13;
  l327_im = V_init * V_wayp;
  l330_re = l5 * l13;
  l3 = l6 * l13;
  l16 = l64 * l238 *
        (((((((((((((((((((((((((((((-(l78_tmp * l4 * 6.0) +
                                     -(l78_tmp * l9 * 6.0)) +
                                    l4 * l5 * 3.0) +
                                   l4 * l6 * 3.0) +
                                  l5 * l9 * 3.0) +
                                 l6 * l9 * 3.0) +
                                l8 * l2_tmp * 24.0) +
                               l324_im * l7_tmp * 24.0) +
                              l78_tmp * V_init * V_wayp * l13 * 48.0) +
                             -(l8 * l7_tmp * 24.0)) +
                            -(l324_im * l2_tmp * 24.0)) +
                           l78_tmp * l2_tmp * l7_tmp * 12.0) +
                          l289 * l5 * 12.0) +
                         j_l254_tmp * l7_tmp * 12.0) +
                        l289 * l6 * 12.0) +
                       k_l254_tmp * l7_tmp * 12.0) +
                      -(l282 * l5 * 12.0)) +
                     -(l282 * l6 * 12.0)) +
                    -(l330_im * l15 * 24.0)) +
                   -(l330_im * l16 * 24.0)) +
                  -(l_l254_tmp * l7_tmp * 12.0)) +
                 -(m_l254_tmp * l7_tmp * 12.0)) +
                -(n_l254_tmp * l7_tmp * 6.0)) +
               -(o_l254_tmp * l7_tmp * 6.0)) +
              -(l327_im * l5 * l13 * 24.0)) +
             -(l327_im * l6 * l13 * 24.0)) +
            l330_re * l15 * 12.0) +
           l3 * l15 * 12.0) +
          l330_re * l16 * 12.0) +
         l3 * l16 * 12.0);
  l11 = J_min *
            ((((((((((((((-(l247_tmp * 6.0) + J_max * l2_tmp * 3.0) +
                         J_max * l7_tmp * 3.0) +
                        V_init * l13 * 6.0) +
                       -(V_wayp * l13 * 6.0)) +
                      A_init * T * l13 * 6.0) +
                     l236_tmp * l18 * l63 * 12.0) +
                    b_l236_tmp * l2_tmp * l63 * 6.0) +
                   -(c_l247_tmp * l63 * 6.0)) +
                  c_l236_tmp * T * l63 * 12.0) +
                 l243_tmp * l62 * l63 * 6.0) +
                -(b_l243_tmp * l62 * l63 * 6.0)) +
               -(T * l13 * l62 * l63 * 6.0)) +
              -(J_min * l18 * l62 * l64 * 12.0)) +
             -(b_l247_tmp * T * l62 * l64 * 12.0)) *
            2.0 -
        l64 * (((((((((((((((((((((((b_l78_tmp * l2_tmp * 12.0 +
                                     l78_tmp_tmp * V_wayp * l13 * 24.0) +
                                    l254_tmp * l5 * l7_tmp * 6.0) +
                                   l254_tmp * l6 * l7_tmp * 6.0) +
                                  -l78) +
                                 -(l78_tmp_tmp * V_init * l13 * 24.0)) +
                                -(b_l254_tmp * l5 * 6.0)) +
                               -(b_l254_tmp * l6 * 6.0)) +
                              c_l254_tmp * l5 * l13 * 12.0) +
                             j_l254_tmp * l10_tmp * 12.0) +
                            c_l254_tmp * l6 * l13 * 12.0) +
                           m_l254_tmp * l10_tmp * 12.0) +
                          l5 * l7_tmp * l10_tmp * 6.0) +
                         l6 * l7_tmp * l10_tmp * 6.0) +
                        -(h_l254_tmp * l6 * l10_tmp * 24.0)) +
                       -(k_l254_tmp * l10_tmp * 12.0)) +
                      -(f_l254_tmp * l5 * l13 * 12.0)) +
                     -(l_l254_tmp * l10_tmp * 12.0)) +
                    -(f_l254_tmp * l6 * l13 * 12.0)) +
                   -(n_l254_tmp * l10_tmp * 6.0)) +
                  o_l254_tmp * l10_tmp * 18.0) +
                 p_l254_tmp * l6 * l10_tmp * 24.0) +
                i_l254_tmp * l6 * l10_tmp * -24.0) +
               g_l254_tmp * (T * T) * 12.0);
  l280 = l239 * l250 * 0.375 + -l238 * l11;
  c_l243_tmp = (l238 * l268 - l239 * a_tmp * l11 / 2.0) +
               l12 * rt_powd_snf(a_tmp, 3.0) / 8.0;
  l289 = c_l243_tmp * c_l243_tmp;
  l15 = l280 * l280;
  l282 = rt_powd_snf(l280, 3.0);
  l3 = l239 * l239 * (l250 * l250);
  l327_im = l239 * l268 * a_tmp;
  l330_im = l12 * l250 * l11;
  l156 = ((l3 * 0.01171875 + l16) + l327_im / 4.0) + l330_im * -0.0625;
  l309.re = ((((l289 * l289 * 27.0 + rt_powd_snf(l156, 3.0) * 256.0) +
               -(l282 * l289 * 4.0)) +
              l15 * l15 * l156 * 16.0) +
             l15 * (l156 * l156) * 128.0) +
            -(l280 * l289 * l156 * 144.0);
  l309.im = 0.0;
  coder::internal::scalar::b_sqrt(&l309);
  l12 = 1.7320508075688772 * l309.re;
  l324_im = 1.7320508075688772 * l309.im;
  if (l324_im == 0.0) {
    l268 = l12 / 18.0;
    l239 = 0.0;
  } else if (l12 == 0.0) {
    l268 = 0.0;
    l239 = l324_im / 18.0;
  } else {
    l268 = l12 / 18.0;
    l239 = l324_im / 18.0;
  }
  l236_tmp_tmp = l280 * l156;
  l314.re =
      ((-(l282 / 27.0) + l289 / 2.0) + -(l236_tmp_tmp * 1.3333333333333333)) +
      l268;
  l314.im = l239;
  l312.re =
      ((-(l282 * 2.0) + l289 * 27.0) + -(l236_tmp_tmp * 72.0)) + l12 * 3.0;
  l312.im = l324_im * 3.0;
  coder::internal::scalar::b_sqrt(&l312);
  l315 = coder::power(l314);
  l309 = coder::b_power(l314);
  if (l309.im == 0.0) {
    l268 = 1.0 / l309.re;
    l239 = 0.0;
  } else if (l309.re == 0.0) {
    l268 = 0.0;
    l239 = -(1.0 / l309.im);
  } else {
    l330_re = std::abs(l309.re);
    l8 = std::abs(l309.im);
    if (l330_re > l8) {
      l330_re = l309.im / l309.re;
      l11 = l309.re + l330_re * l309.im;
      l268 = (l330_re * 0.0 + 1.0) / l11;
      l239 = (0.0 - l330_re) / l11;
    } else if (l8 == l330_re) {
      if (l309.re > 0.0) {
        l289 = 0.5;
      } else {
        l289 = -0.5;
      }
      if (l309.im > 0.0) {
        l11 = 0.5;
      } else {
        l11 = -0.5;
      }
      l268 = (l289 + 0.0 * l11) / l330_re;
      l239 = (0.0 * l289 - l11) / l330_re;
    } else {
      l330_re = l309.re / l309.im;
      l11 = l309.im + l330_re * l309.re;
      l268 = l330_re / l11;
      l239 = (l330_re * 0.0 - 1.0) / l11;
    }
  }
  l314.re = l315.re * l315.re - l315.im * l315.im;
  l236_tmp_tmp = l315.re * l315.im;
  l314.im = l236_tmp_tmp + l236_tmp_tmp;
  l236_tmp_tmp = l280 * l315.re;
  l12 = l280 * l315.im;
  l309.re = (((((-(l3 * 0.140625) + -(l16 * 12.0)) + l327_im * -3.0) +
               l330_im * 0.75) +
              l15) +
             l314.re * 9.0) +
            l236_tmp_tmp * 6.0;
  l309.im = l314.im * 9.0 + l12 * 6.0;
  l323 = l309;
  coder::internal::scalar::b_sqrt(&l323);
  l309 = coder::c_power(l309);
  if (l309.im == 0.0) {
    l8 = 1.0 / l309.re;
    l324_im = 0.0;
  } else if (l309.re == 0.0) {
    l8 = 0.0;
    l324_im = -(1.0 / l309.im);
  } else {
    l330_re = std::abs(l309.re);
    l8 = std::abs(l309.im);
    if (l330_re > l8) {
      l330_re = l309.im / l309.re;
      l11 = l309.re + l330_re * l309.im;
      l8 = (l330_re * 0.0 + 1.0) / l11;
      l324_im = (0.0 - l330_re) / l11;
    } else if (l8 == l330_re) {
      if (l309.re > 0.0) {
        l289 = 0.5;
      } else {
        l289 = -0.5;
      }
      if (l309.im > 0.0) {
        l11 = 0.5;
      } else {
        l11 = -0.5;
      }
      l8 = (l289 + 0.0 * l11) / l330_re;
      l324_im = (0.0 * l289 - l11) / l330_re;
    } else {
      l330_re = l309.re / l309.im;
      l11 = l309.im + l330_re * l309.re;
      l8 = l330_re / l11;
      l324_im = (l330_re * 0.0 - 1.0) / l11;
    }
  }
  l3 = 12.0 * (l156 * l323.re);
  l327_im = 12.0 * (l156 * l323.im);
  l156 = -9.0 * (l314.re * l323.re - l314.im * l323.im);
  l280 = -9.0 * (l314.re * l323.im + l314.im * l323.re);
  l289 = l268 * l323.re - l239 * l323.im;
  l282 = l268 * l323.im + l239 * l323.re;
  if (l282 == 0.0) {
    l330_re = l289 / 6.0;
    l330_im = 0.0;
  } else if (l289 == 0.0) {
    l330_re = 0.0;
    l330_im = l282 / 6.0;
  } else {
    l330_re = l289 / 6.0;
    l330_im = l282 / 6.0;
  }
  l315.re = 12.0 * (l236_tmp_tmp * l323.re - l12 * l323.im);
  l315.im = 12.0 * (l236_tmp_tmp * l323.im + l12 * l323.re);
  l309.re = -(l15 * l323.re);
  l309.im = -(l15 * l323.im);
  l236_tmp_tmp = c_l243_tmp * (2.4494897427831779 * l312.re);
  l314.re = (((l236_tmp_tmp * -3.0 + l309.re) + l3) + l156) + l315.re;
  l289 = c_l243_tmp * (2.4494897427831779 * l312.im);
  l314.im = (((l289 * -3.0 + l309.im) + l327_im) + l280) + l315.im;
  coder::internal::scalar::b_sqrt(&l314);
  l12 = l268 * l8 - l239 * l324_im;
  l282 = l268 * l324_im + l239 * l8;
  l268 = l12 * l314.re - l282 * l314.im;
  l239 = l12 * l314.im + l282 * l314.re;
  if (l239 == 0.0) {
    l8 = l268 / 6.0;
    l11 = 0.0;
  } else if (l268 == 0.0) {
    l8 = 0.0;
    l11 = l239 / 6.0;
  } else {
    l8 = l268 / 6.0;
    l11 = l239 / 6.0;
  }
  l314.re = (((l236_tmp_tmp * 3.0 + l309.re) + l3) + l156) + l315.re;
  l314.im = (((l289 * 3.0 + l309.im) + l327_im) + l280) + l315.im;
  coder::internal::scalar::b_sqrt(&l314);
  l268 = l12 * l314.re - l282 * l314.im;
  l239 = l12 * l314.im + l282 * l314.re;
  if (l239 == 0.0) {
    l309.re = l268 / 6.0;
    l309.im = 0.0;
  } else if (l268 == 0.0) {
    l309.re = 0.0;
    l309.im = l239 / 6.0;
  } else {
    l309.re = l268 / 6.0;
    l309.im = l239 / 6.0;
  }
  l289 = l259 + -l330_re;
  t3[0].re = l289 - l309.re;
  t3[0].im = -l330_im - l309.im;
  t3[1].re = l289 + l309.re;
  t3[1].im = -l330_im + l309.im;
  l289 = l259 + l330_re;
  t3[2].re = l289 - l8;
  t3[2].im = l330_im - l11;
  t3[3].re = l289 + l8;
  t3[3].im = l330_im + l11;
  l280 = A_min * J_max - b_l243_tmp_tmp;
  l309.re = d_l254_tmp * 2.0 - e_l254_tmp * 2.0;
  l314.re = h_l254_tmp * 2.0;
  l312.re = p_l254_tmp * 2.0;
  l330_im = J_max * J_max * T;
  l268 = J_min * t3[0].re;
  l239 = J_min * t3[0].im;
  t2[0].re = l268;
  t2[0].im = l239;
  l12 = A_max * (A_init + l268) - l17_tmp;
  l324_im = A_max * l239;
  if (l324_im == 0.0) {
    l327_im = l12 / l280;
    l156 = 0.0;
  } else if (l12 == 0.0) {
    l327_im = 0.0;
    l156 = l324_im / l280;
  } else {
    l327_im = l12 / l280;
    l156 = l324_im / l280;
  }
  t1[0].re = l327_im;
  t1[0].im = l156;
  l8 = t3[0].re * t3[0].re - t3[0].im * t3[0].im;
  l289 = t3[0].re * t3[0].im;
  l11 = l289 + l289;
  l282 = ((((((((l309.re - l10_tmp * l8) + l2_tmp) + l7_tmp) - l314.re) +
             l312.re) +
            b_l243_tmp * l327_im * 2.0) -
           l243_tmp * l327_im * 2.0) +
          l254_tmp * l8) +
         l330_im * l327_im * 2.0;
  l289 = ((((0.0 - l10_tmp * l11) + b_l243_tmp * l156 * 2.0) -
           l243_tmp * l156 * 2.0) +
          l254_tmp * l11) +
         l330_im * l156 * 2.0;
  l12 = l282 * -0.5;
  l324_im = l289 * -0.5;
  l289 = l254_tmp * t3[0].re;
  l282 = l254_tmp * t3[0].im;
  if (l282 == 0.0) {
    if (l324_im == 0.0) {
      l8 = l12 / l289;
      l11 = 0.0;
    } else if (l12 == 0.0) {
      l8 = 0.0;
      l11 = l324_im / l289;
    } else {
      l8 = l12 / l289;
      l11 = l324_im / l289;
    }
  } else if (l289 == 0.0) {
    if (l12 == 0.0) {
      l8 = l324_im / l282;
      l11 = 0.0;
    } else if (l324_im == 0.0) {
      l8 = 0.0;
      l11 = -(l12 / l282);
    } else {
      l8 = l324_im / l282;
      l11 = -(l12 / l282);
    }
  } else {
    l330_re = std::abs(l289);
    l8 = std::abs(l282);
    if (l330_re > l8) {
      l330_re = l282 / l289;
      l11 = l289 + l330_re * l282;
      l8 = (l12 + l330_re * l324_im) / l11;
      l11 = (l324_im - l330_re * l12) / l11;
    } else if (l8 == l330_re) {
      if (l289 > 0.0) {
        l289 = 0.5;
      } else {
        l289 = -0.5;
      }
      if (l282 > 0.0) {
        l11 = 0.5;
      } else {
        l11 = -0.5;
      }
      l8 = (l12 * l289 + l324_im * l11) / l330_re;
      l11 = (l324_im * l289 - l12 * l11) / l330_re;
    } else {
      l330_re = l289 / l282;
      l11 = l282 + l330_re * l289;
      l8 = (l330_re * l12 + l324_im) / l11;
      l11 = (l330_re * l324_im - l12) / l11;
    }
  }
  l2[0].re = l8;
  l2[0].im = l11;
  l268 = J_min * t3[1].re;
  l239 = J_min * t3[1].im;
  t2[1].re = l268;
  t2[1].im = l239;
  l12 = A_max * (A_init + l268) - l17_tmp;
  l324_im = A_max * l239;
  if (l324_im == 0.0) {
    l327_im = l12 / l280;
    l156 = 0.0;
  } else if (l12 == 0.0) {
    l327_im = 0.0;
    l156 = l324_im / l280;
  } else {
    l327_im = l12 / l280;
    l156 = l324_im / l280;
  }
  t1[1].re = l327_im;
  t1[1].im = l156;
  l8 = t3[1].re * t3[1].re - t3[1].im * t3[1].im;
  l289 = t3[1].re * t3[1].im;
  l11 = l289 + l289;
  l282 = ((((((((l309.re - l10_tmp * l8) + l2_tmp) + l7_tmp) - l314.re) +
             l312.re) +
            b_l243_tmp * l327_im * 2.0) -
           l243_tmp * l327_im * 2.0) +
          l254_tmp * l8) +
         l330_im * l327_im * 2.0;
  l289 = ((((0.0 - l10_tmp * l11) + b_l243_tmp * l156 * 2.0) -
           l243_tmp * l156 * 2.0) +
          l254_tmp * l11) +
         l330_im * l156 * 2.0;
  l12 = l282 * -0.5;
  l324_im = l289 * -0.5;
  l289 = l254_tmp * t3[1].re;
  l282 = l254_tmp * t3[1].im;
  if (l282 == 0.0) {
    if (l324_im == 0.0) {
      l8 = l12 / l289;
      l11 = 0.0;
    } else if (l12 == 0.0) {
      l8 = 0.0;
      l11 = l324_im / l289;
    } else {
      l8 = l12 / l289;
      l11 = l324_im / l289;
    }
  } else if (l289 == 0.0) {
    if (l12 == 0.0) {
      l8 = l324_im / l282;
      l11 = 0.0;
    } else if (l324_im == 0.0) {
      l8 = 0.0;
      l11 = -(l12 / l282);
    } else {
      l8 = l324_im / l282;
      l11 = -(l12 / l282);
    }
  } else {
    l330_re = std::abs(l289);
    l8 = std::abs(l282);
    if (l330_re > l8) {
      l330_re = l282 / l289;
      l11 = l289 + l330_re * l282;
      l8 = (l12 + l330_re * l324_im) / l11;
      l11 = (l324_im - l330_re * l12) / l11;
    } else if (l8 == l330_re) {
      if (l289 > 0.0) {
        l289 = 0.5;
      } else {
        l289 = -0.5;
      }
      if (l282 > 0.0) {
        l11 = 0.5;
      } else {
        l11 = -0.5;
      }
      l8 = (l12 * l289 + l324_im * l11) / l330_re;
      l11 = (l324_im * l289 - l12 * l11) / l330_re;
    } else {
      l330_re = l289 / l282;
      l11 = l282 + l330_re * l289;
      l8 = (l330_re * l12 + l324_im) / l11;
      l11 = (l330_re * l324_im - l12) / l11;
    }
  }
  l2[1].re = l8;
  l2[1].im = l11;
  l268 = J_min * t3[2].re;
  l239 = J_min * t3[2].im;
  t2[2].re = l268;
  t2[2].im = l239;
  l12 = A_max * (A_init + l268) - l17_tmp;
  l324_im = A_max * l239;
  if (l324_im == 0.0) {
    l327_im = l12 / l280;
    l156 = 0.0;
  } else if (l12 == 0.0) {
    l327_im = 0.0;
    l156 = l324_im / l280;
  } else {
    l327_im = l12 / l280;
    l156 = l324_im / l280;
  }
  t1[2].re = l327_im;
  t1[2].im = l156;
  l8 = t3[2].re * t3[2].re - t3[2].im * t3[2].im;
  l289 = t3[2].re * t3[2].im;
  l11 = l289 + l289;
  l282 = ((((((((l309.re - l10_tmp * l8) + l2_tmp) + l7_tmp) - l314.re) +
             l312.re) +
            b_l243_tmp * l327_im * 2.0) -
           l243_tmp * l327_im * 2.0) +
          l254_tmp * l8) +
         l330_im * l327_im * 2.0;
  l289 = ((((0.0 - l10_tmp * l11) + b_l243_tmp * l156 * 2.0) -
           l243_tmp * l156 * 2.0) +
          l254_tmp * l11) +
         l330_im * l156 * 2.0;
  l12 = l282 * -0.5;
  l324_im = l289 * -0.5;
  l289 = l254_tmp * t3[2].re;
  l282 = l254_tmp * t3[2].im;
  if (l282 == 0.0) {
    if (l324_im == 0.0) {
      l8 = l12 / l289;
      l11 = 0.0;
    } else if (l12 == 0.0) {
      l8 = 0.0;
      l11 = l324_im / l289;
    } else {
      l8 = l12 / l289;
      l11 = l324_im / l289;
    }
  } else if (l289 == 0.0) {
    if (l12 == 0.0) {
      l8 = l324_im / l282;
      l11 = 0.0;
    } else if (l324_im == 0.0) {
      l8 = 0.0;
      l11 = -(l12 / l282);
    } else {
      l8 = l324_im / l282;
      l11 = -(l12 / l282);
    }
  } else {
    l330_re = std::abs(l289);
    l8 = std::abs(l282);
    if (l330_re > l8) {
      l330_re = l282 / l289;
      l11 = l289 + l330_re * l282;
      l8 = (l12 + l330_re * l324_im) / l11;
      l11 = (l324_im - l330_re * l12) / l11;
    } else if (l8 == l330_re) {
      if (l289 > 0.0) {
        l289 = 0.5;
      } else {
        l289 = -0.5;
      }
      if (l282 > 0.0) {
        l11 = 0.5;
      } else {
        l11 = -0.5;
      }
      l8 = (l12 * l289 + l324_im * l11) / l330_re;
      l11 = (l324_im * l289 - l12 * l11) / l330_re;
    } else {
      l330_re = l289 / l282;
      l11 = l282 + l330_re * l289;
      l8 = (l330_re * l12 + l324_im) / l11;
      l11 = (l330_re * l324_im - l12) / l11;
    }
  }
  l2[2].re = l8;
  l2[2].im = l11;
  l268 = J_min * t3[3].re;
  l239 = J_min * t3[3].im;
  l12 = A_max * (A_init + l268) - l17_tmp;
  l324_im = A_max * l239;
  if (l324_im == 0.0) {
    l327_im = l12 / l280;
    l156 = 0.0;
  } else if (l12 == 0.0) {
    l327_im = 0.0;
    l156 = l324_im / l280;
  } else {
    l327_im = l12 / l280;
    l156 = l324_im / l280;
  }
  t1[3].re = l327_im;
  t1[3].im = l156;
  l8 = t3[3].re * t3[3].re - t3[3].im * t3[3].im;
  l289 = t3[3].re * t3[3].im;
  l11 = l289 + l289;
  l282 = ((((((((l309.re - l10_tmp * l8) + l2_tmp) + l7_tmp) - l314.re) +
             l312.re) +
            b_l243_tmp * l327_im * 2.0) -
           l243_tmp * l327_im * 2.0) +
          l254_tmp * l8) +
         l330_im * l327_im * 2.0;
  l289 = ((((0.0 - l10_tmp * l11) + b_l243_tmp * l156 * 2.0) -
           l243_tmp * l156 * 2.0) +
          l254_tmp * l11) +
         l330_im * l156 * 2.0;
  l12 = l282 * -0.5;
  l324_im = l289 * -0.5;
  l289 = l254_tmp * t3[3].re;
  l282 = l254_tmp * t3[3].im;
  if (l282 == 0.0) {
    if (l324_im == 0.0) {
      l8 = l12 / l289;
      l11 = 0.0;
    } else if (l12 == 0.0) {
      l8 = 0.0;
      l11 = l324_im / l289;
    } else {
      l8 = l12 / l289;
      l11 = l324_im / l289;
    }
  } else if (l289 == 0.0) {
    if (l12 == 0.0) {
      l8 = l324_im / l282;
      l11 = 0.0;
    } else if (l324_im == 0.0) {
      l8 = 0.0;
      l11 = -(l12 / l282);
    } else {
      l8 = l324_im / l282;
      l11 = -(l12 / l282);
    }
  } else {
    l330_re = std::abs(l289);
    l8 = std::abs(l282);
    if (l330_re > l8) {
      l330_re = l282 / l289;
      l11 = l289 + l330_re * l282;
      l8 = (l12 + l330_re * l324_im) / l11;
      l11 = (l324_im - l330_re * l12) / l11;
    } else if (l8 == l330_re) {
      if (l289 > 0.0) {
        l289 = 0.5;
      } else {
        l289 = -0.5;
      }
      if (l282 > 0.0) {
        l11 = 0.5;
      } else {
        l11 = -0.5;
      }
      l8 = (l12 * l289 + l324_im * l11) / l330_re;
      l11 = (l324_im * l289 - l12 * l11) / l330_re;
    } else {
      l330_re = l289 / l282;
      l11 = l282 + l330_re * l289;
      l8 = (l330_re * l12 + l324_im) / l11;
      l11 = (l330_re * l324_im - l12) / l11;
    }
  }
  l2[3].re = l8;
  l2[3].im = l11;
  l314.re = A_init - A_wayp;
  l12 = (((l314.re + t2[0].re) - J_max * t3[0].re) - J_max * l2[0].re) +
        l243_tmp_tmp;
  l324_im = (t2[0].im - J_max * t3[0].im) - J_max * l2[0].im;
  if (l324_im == 0.0) {
    l282 = l12 / J_max;
    l289 = 0.0;
  } else if (l12 == 0.0) {
    l282 = 0.0;
    l289 = l324_im / J_max;
  } else {
    l282 = l12 / J_max;
    l289 = l324_im / J_max;
  }
  t[0] = t1[0];
  t[4].re = l282;
  t[4].im = l289;
  t[8] = t3[0];
  t[12].re = 0.0;
  t[12].im = 0.0;
  t[16].re = 0.0;
  t[16].im = 0.0;
  t[20] = l2[0];
  t[24].re = (((T - t1[0].re) - l282) - t3[0].re) - l2[0].re;
  t[24].im = (((0.0 - t1[0].im) - l289) - t3[0].im) - l2[0].im;
  l12 = (((l314.re + t2[1].re) - J_max * t3[1].re) - J_max * l2[1].re) +
        l243_tmp_tmp;
  l324_im = (t2[1].im - J_max * t3[1].im) - J_max * l2[1].im;
  if (l324_im == 0.0) {
    l282 = l12 / J_max;
    l289 = 0.0;
  } else if (l12 == 0.0) {
    l282 = 0.0;
    l289 = l324_im / J_max;
  } else {
    l282 = l12 / J_max;
    l289 = l324_im / J_max;
  }
  t[1] = t1[1];
  t[5].re = l282;
  t[5].im = l289;
  t[9] = t3[1];
  t[13].re = 0.0;
  t[13].im = 0.0;
  t[17].re = 0.0;
  t[17].im = 0.0;
  t[21] = l2[1];
  t[25].re = (((T - t1[1].re) - l282) - t3[1].re) - l2[1].re;
  t[25].im = (((0.0 - t1[1].im) - l289) - t3[1].im) - l2[1].im;
  l12 = (((l314.re + t2[2].re) - J_max * t3[2].re) - J_max * l2[2].re) +
        l243_tmp_tmp;
  l324_im = (t2[2].im - J_max * t3[2].im) - J_max * l2[2].im;
  if (l324_im == 0.0) {
    l282 = l12 / J_max;
    l289 = 0.0;
  } else if (l12 == 0.0) {
    l282 = 0.0;
    l289 = l324_im / J_max;
  } else {
    l282 = l12 / J_max;
    l289 = l324_im / J_max;
  }
  t[2] = t1[2];
  t[6].re = l282;
  t[6].im = l289;
  t[10] = t3[2];
  t[14].re = 0.0;
  t[14].im = 0.0;
  t[18].re = 0.0;
  t[18].im = 0.0;
  t[22] = l2[2];
  t[26].re = (((T - t1[2].re) - l282) - t3[2].re) - l2[2].re;
  t[26].im = (((0.0 - t1[2].im) - l289) - t3[2].im) - l2[2].im;
  l12 = (((l314.re + l268) - J_max * t3[3].re) - J_max * l8) + l243_tmp_tmp;
  l324_im = (l239 - J_max * t3[3].im) - J_max * l11;
  if (l324_im == 0.0) {
    l282 = l12 / J_max;
    l289 = 0.0;
  } else if (l12 == 0.0) {
    l282 = 0.0;
    l289 = l324_im / J_max;
  } else {
    l282 = l12 / J_max;
    l289 = l324_im / J_max;
  }
  t[3] = t1[3];
  t[7].re = l282;
  t[7].im = l289;
  t[11] = t3[3];
  t[15].re = 0.0;
  t[15].im = 0.0;
  t[19].re = 0.0;
  t[19].im = 0.0;
  t[23] = l2[3];
  t[27].re = (((T - l327_im) - l282) - t3[3].re) - l8;
  t[27].im = (((0.0 - l156) - l289) - t3[3].im) - l11;
}

// End of code generation (abcefg_TA_AVP.cpp)
