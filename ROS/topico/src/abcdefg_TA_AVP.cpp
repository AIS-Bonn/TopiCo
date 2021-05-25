//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// abcdefg_TA_AVP.cpp
//
// Code generation for function 'abcdefg_TA_AVP'
//

// Include files
#include "abcdefg_TA_AVP.h"
#include "power.h"
#include "rt_nonfinite.h"
#include "sqrt.h"
#include "topico_wrapper_data.h"
#include "topico_wrapper_rtwutil.h"
#include <cmath>

// Function Definitions
void abcdefg_TA_AVP(double P_init, double V_init, double A_init, double P_wayp,
                    double V_wayp, double A_wayp, double V_max, double A_max,
                    double A_min, double J_max, double J_min, double T,
                    creal_T t[28])
{
  creal_T b_l7[4];
  creal_T l19[4];
  creal_T l2[4];
  creal_T l5[4];
  creal_T t7[4];
  creal_T b_A_min;
  creal_T l151;
  creal_T l156;
  creal_T l157;
  double ar;
  double b_J_min;
  double b_br_tmp;
  double b_im;
  double b_re;
  double bim;
  double br_tmp;
  double brm;
  double c_br_tmp;
  double c_im;
  double c_re;
  double d_im;
  double d_re;
  double im;
  double l101;
  double l102;
  double l105;
  double l10_tmp;
  double l113;
  double l113_tmp;
  double l115;
  double l115_tmp;
  double l11_tmp;
  double l12;
  double l13;
  double l134;
  double l135;
  double l14_tmp;
  double l151_tmp;
  double l158_re;
  double l159_im;
  double l159_re;
  double l15_tmp;
  double l16;
  double l163_im;
  double l167_re;
  double l17;
  double l172_im;
  double l173_im;
  double l173_re;
  double l18;
  double l18_tmp;
  double l18_tmp_tmp;
  double l20;
  double l20_tmp;
  double l25;
  double l26_tmp_tmp;
  double l2_tmp;
  double l3;
  double l5_tmp;
  double l6;
  double l7;
  double l8_tmp;
  double l9;
  double l99;
  double re;
  double s;
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
  //  Generated on 28-Aug-2019 13:55:05
  l2_tmp = A_init * A_init;
  l3 = rt_powd_snf(A_init, 3.0);
  l5_tmp = A_min * A_min;
  l6 = rt_powd_snf(A_min, 3.0);
  l8_tmp = A_max * A_max;
  l10_tmp = A_wayp * A_wayp;
  l11_tmp = rt_powd_snf(A_wayp, 3.0);
  l13 = J_min * J_min;
  l14_tmp = J_max * J_max;
  l15_tmp = rt_powd_snf(J_max, 3.0);
  l17 = rt_powd_snf(J_max, 5.0);
  l20 = V_max * V_max;
  l7 = l5_tmp * l5_tmp;
  l9 = l8_tmp * l8_tmp;
  l12 = l10_tmp * l10_tmp;
  l16 = l14_tmp * l14_tmp;
  l18 = rt_powd_snf(l14_tmp, 3.0);
  l135 = A_wayp * l9;
  l25 = A_max * A_wayp;
  l173_im = l25 * l6;
  l101 = ((l135 * l17 * 4.0 + -(l173_im * l17 * 4.0)) +
          -(l135 * l13 * l15_tmp * 4.0)) +
         l173_im * l13 * l15_tmp * 4.0;
  l17 = l9 * l10_tmp;
  l134 = A_max * l6;
  l163_im = A_max * V_max;
  l113_tmp = l2_tmp * l5_tmp * l8_tmp;
  l113 = ((((((l17 * l16 * 6.0 + -(l134 * l10_tmp * l16 * 6.0)) +
              A_max * V_wayp * l6 * l13 * l15_tmp * 12.0) +
             -(l163_im * l6 * l13 * l15_tmp * 12.0)) +
            -(l17 * l13 * l14_tmp * 6.0)) +
           V_max * l5_tmp * l8_tmp * l13 * l15_tmp * 12.0) +
          -(V_init * l5_tmp * l8_tmp * l13 * l15_tmp * 12.0)) +
         l113_tmp * l13 * l14_tmp * 6.0;
  l17 = A_init * A_max;
  l115_tmp = A_wayp * J_max;
  l115 = ((((((((((-(l9 * l11_tmp * l15_tmp * 4.0) +
                   l134 * l11_tmp * l15_tmp * 4.0) +
                  J_max * l9 * l11_tmp * l13 * 4.0) +
                 A_max * J_max * l3 * l6 * l13 * 8.0) +
                A_max * P_init * l6 * l13 * l15_tmp * 24.0) +
               l17 * V_max * l6 * l13 * l14_tmp * 24.0) +
              A_max * T * V_max * l6 * l13 * l15_tmp * 24.0) +
             -(A_max * P_wayp * l6 * l13 * l15_tmp * 24.0)) +
            -(l17 * V_init * l6 * l13 * l14_tmp * 24.0)) +
           A_wayp * V_init * l5_tmp * l8_tmp * l13 * l14_tmp * 24.0) +
          -(l115_tmp * l2_tmp * l5_tmp * l8_tmp * l13 * 12.0)) +
         -(A_wayp * V_max * l5_tmp * l8_tmp * l13 * l14_tmp * 24.0);
  l135 = l134 * l13;
  l16 = 1.0 / (((l9 * l18 + -(l134 * l18)) + l135 * l16) + -(l9 * l13 * l16));
  l102 = l101 * l101;
  l18 = l16 * l16;
  l99 = rt_powd_snf(l16, 3.0);
  l105 = l16 * l101 / 4.0;
  l173_re = l9 * l12;
  l9 = l17 * A_wayp * J_max;
  l172_im = l7 * l13 * l14_tmp;
  l17 = l135 * l14_tmp;
  l135 = J_max * V_init;
  l173_im = J_max * V_max;
  l13 = l16 * ((((((((((((((((((((l173_re * l13 + l134 * l12 * l14_tmp) +
                                 -(l2_tmp * l2_tmp * l7 * l13 * 3.0)) +
                                -(l173_re * l14_tmp)) +
                               V_init * V_max * l7 * l13 * l14_tmp * 24.0) +
                              l9 * V_max * l6 * l13 * 24.0) +
                             l25 * l3 * l6 * l13 * 8.0) +
                            l135 * l2_tmp * l7 * l13 * 12.0) +
                           -(l9 * V_init * l6 * l13 * 24.0)) +
                          l25 * P_init * l6 * l13 * l14_tmp * 24.0) +
                         -(l173_im * l2_tmp * l7 * l13 * 12.0)) +
                        -(l172_im * (V_init * V_init) * 12.0)) +
                       -(l172_im * l20 * 12.0)) +
                      -(l25 * P_wayp * l6 * l13 * l14_tmp * 24.0)) +
                     -(l163_im * V_wayp * l6 * l13 * l14_tmp * 24.0)) +
                    l17 * l20 * 12.0) +
                   l17 * (V_wayp * V_wayp) * 12.0) +
                  l25 * T * V_max * l6 * l13 * l14_tmp * 24.0) +
                 l135 * l5_tmp * l8_tmp * l10_tmp * l13 * 12.0) +
                -(l173_im * l5_tmp * l8_tmp * l10_tmp * l13 * 12.0)) +
               -(l113_tmp * l10_tmp * l13 * 6.0));
  l20 = l18 * l102 * 0.375 - l16 * l113;
  l7 = l20 * l20;
  l17 = l18 * l101;
  l134 =
      (-(l99 * rt_powd_snf(l101, 3.0) / 8.0) + l17 * l113 / 2.0) + l16 * l115;
  l113_tmp = l18 * l18 * (l102 * l102);
  l18 = l99 * l102 * l113;
  l16 = l17 * l115;
  l9 = ((l113_tmp * 0.01171875 + -(l18 / 16.0)) + -(l16 / 4.0)) + l13;
  l135 = l134 * l134;
  l151_tmp = rt_powd_snf(l20, 3.0);
  l151.re = ((((l135 * l135 * 27.0 + l135 * l151_tmp * -4.0) +
               rt_powd_snf(l9, 3.0) * 256.0) +
              l7 * l7 * l9 * 16.0) +
             l7 * (l9 * l9) * 128.0) +
            l135 * l9 * l20 * -144.0;
  l151.im = 0.0;
  coder::internal::scalar::b_sqrt(&l151);
  l173_im = 1.7320508075688772 * l151.re;
  l172_im = 1.7320508075688772 * l151.im;
  if (l172_im == 0.0) {
    re = l173_im / 18.0;
    im = 0.0;
  } else if (l173_im == 0.0) {
    re = 0.0;
    im = l172_im / 18.0;
  } else {
    re = l173_im / 18.0;
    im = l172_im / 18.0;
  }
  l173_re = l9 * l20;
  l156.re = ((l151_tmp * -0.037037037037037035 + l135 / 2.0) +
             l173_re * -1.3333333333333333) +
            re;
  l156.im = im;
  l157 = coder::power(l156);
  l156 = coder::b_power(l156);
  if (l156.im == 0.0) {
    l159_re = 1.0 / l156.re;
    l159_im = 0.0;
  } else if (l156.re == 0.0) {
    l159_re = 0.0;
    l159_im = -(1.0 / l156.im);
  } else {
    brm = std::abs(l156.re);
    bim = std::abs(l156.im);
    if (brm > bim) {
      s = l156.im / l156.re;
      bim = l156.re + s * l156.im;
      l159_re = (s * 0.0 + 1.0) / bim;
      l159_im = (0.0 - s) / bim;
    } else if (bim == brm) {
      if (l156.re > 0.0) {
        s = 0.5;
      } else {
        s = -0.5;
      }
      if (l156.im > 0.0) {
        bim = 0.5;
      } else {
        bim = -0.5;
      }
      l159_re = (s + 0.0 * bim) / brm;
      l159_im = (0.0 * s - bim) / brm;
    } else {
      s = l156.re / l156.im;
      bim = l156.im + s * l156.re;
      l159_re = s / bim;
      l159_im = (s * 0.0 - 1.0) / bim;
    }
  }
  l158_re = l157.re * l157.re - l157.im * l157.im;
  l17 = l157.re * l157.im;
  l6 = l17 + l17;
  b_A_min.re =
      ((l151_tmp * -2.0 + l135 * 27.0) + l173_re * -72.0) + l173_im * 3.0;
  b_A_min.im = l172_im * 3.0;
  coder::internal::scalar::b_sqrt(&b_A_min);
  l12 = 3.0 * (2.4494897427831779 * l134 * b_A_min.re);
  l163_im = 3.0 * (2.4494897427831779 * l134 * b_A_min.im);
  l156.re = (((((-(l113_tmp * 0.140625) + l18 * 0.75) + l7) + l16 * 3.0) +
              -(l13 * 12.0)) +
             l158_re * 9.0) +
            l20 * l157.re * 6.0;
  l156.im = l6 * 9.0 + l20 * l157.im * 6.0;
  l151 = l156;
  coder::internal::scalar::b_sqrt(&l151);
  l156 = coder::c_power(l156);
  if (l156.im == 0.0) {
    l167_re = 1.0 / l156.re;
    l16 = 0.0;
  } else if (l156.re == 0.0) {
    l167_re = 0.0;
    l16 = -(1.0 / l156.im);
  } else {
    brm = std::abs(l156.re);
    bim = std::abs(l156.im);
    if (brm > bim) {
      s = l156.im / l156.re;
      bim = l156.re + s * l156.im;
      l167_re = (s * 0.0 + 1.0) / bim;
      l16 = (0.0 - s) / bim;
    } else if (bim == brm) {
      if (l156.re > 0.0) {
        s = 0.5;
      } else {
        s = -0.5;
      }
      if (l156.im > 0.0) {
        bim = 0.5;
      } else {
        bim = -0.5;
      }
      l167_re = (s + 0.0 * bim) / brm;
      l16 = (0.0 * s - bim) / brm;
    } else {
      s = l156.re / l156.im;
      bim = l156.im + s * l156.re;
      l167_re = s / bim;
      l16 = (s * 0.0 - 1.0) / bim;
    }
  }
  l134 = 12.0 * (l9 * l151.re);
  l18 = 12.0 * (l9 * l151.im);
  l9 = -9.0 * (l158_re * l151.re - l6 * l151.im);
  l172_im = -9.0 * (l158_re * l151.im + l6 * l151.re);
  l17 = l159_re * l151.re - l159_im * l151.im;
  l135 = l159_re * l151.im + l159_im * l151.re;
  if (l135 == 0.0) {
    l173_re = l17 / 6.0;
    l173_im = 0.0;
  } else if (l17 == 0.0) {
    l173_re = 0.0;
    l173_im = l135 / 6.0;
  } else {
    l173_re = l17 / 6.0;
    l173_im = l135 / 6.0;
  }
  l158_re = 12.0 * (l20 * (l157.re * l151.re - l157.im * l151.im));
  l6 = 12.0 * (l20 * (l157.re * l151.im + l157.im * l151.re));
  l156.re = -(l7 * l151.re);
  l156.im = -(l7 * l151.im);
  b_A_min.re = (((l12 + l156.re) + l134) + l9) + l158_re;
  b_A_min.im = (((l163_im + l156.im) + l18) + l172_im) + l6;
  coder::internal::scalar::b_sqrt(&b_A_min);
  l135 = l159_re * l167_re - l159_im * l16;
  l17 = l159_re * l16 + l159_im * l167_re;
  l159_re = l135 * b_A_min.re - l17 * b_A_min.im;
  l159_im = l135 * b_A_min.im + l17 * b_A_min.re;
  if (l159_im == 0.0) {
    l157.re = l159_re / 6.0;
    l157.im = 0.0;
  } else if (l159_re == 0.0) {
    l157.re = 0.0;
    l157.im = l159_im / 6.0;
  } else {
    l157.re = l159_re / 6.0;
    l157.im = l159_im / 6.0;
  }
  b_A_min.re = (((-l12 + l156.re) + l134) + l9) + l158_re;
  b_A_min.im = (((-l163_im + l156.im) + l18) + l172_im) + l6;
  coder::internal::scalar::b_sqrt(&b_A_min);
  l159_re = l135 * b_A_min.re - l17 * b_A_min.im;
  l159_im = l135 * b_A_min.im + l17 * b_A_min.re;
  if (l159_im == 0.0) {
    l156.re = l159_re / 6.0;
    l156.im = 0.0;
  } else if (l159_re == 0.0) {
    l156.re = 0.0;
    l156.im = l159_im / 6.0;
  } else {
    l156.re = l159_re / 6.0;
    l156.im = l159_im / 6.0;
  }
  l17 = l105 + -l173_re;
  t7[0].re = l17 - l157.re;
  t7[0].im = -l173_im - l157.im;
  t7[1].re = l17 + l157.re;
  t7[1].im = -l173_im + l157.im;
  l17 = l105 + l173_re;
  t7[2].re = l17 - l156.re;
  t7[2].im = l173_im - l156.im;
  t7[3].re = l17 + l156.re;
  t7[3].im = l173_im + l156.im;
  re = J_max * t7[0].re;
  im = J_max * t7[0].im;
  l2[0].re = re;
  l2[0].im = im;
  l5[0].re = -re;
  l5[0].im = -im;
  re = J_max * t7[1].re;
  im = J_max * t7[1].im;
  l2[1].re = re;
  l2[1].im = im;
  l5[1].re = -re;
  l5[1].im = -im;
  re = J_max * t7[2].re;
  im = J_max * t7[2].im;
  l2[2].re = re;
  l2[2].im = im;
  l5[2].re = -re;
  l5[2].im = -im;
  re = J_max * t7[3].re;
  im = J_max * t7[3].im;
  l2[3].re = re;
  l2[3].im = im;
  coder::b_power(l2, b_l7);
  l18_tmp_tmp = A_init * A_min * A_max;
  l18_tmp = l18_tmp_tmp * J_min;
  l7 = A_wayp * J_min;
  l20_tmp = l7 * J_max;
  l12 = J_max * l8_tmp;
  l25 = -(l12 * l11_tmp);
  l3 = A_min * A_max;
  l26_tmp_tmp = l3 * J_min;
  l102 = l26_tmp_tmp * J_max * T;
  l99 = -(l102 * l10_tmp * 2.0);
  l151_tmp = -(l18_tmp * l10_tmp * 2.0) + -(l20_tmp * V_init * l5_tmp * 2.0);
  l115 = J_min * J_max;
  b_J_min = l115 * V_wayp;
  l156.re = l20_tmp * V_max * l5_tmp * 2.0;
  l157.re = l20_tmp * V_wayp * l5_tmp * 2.0;
  l158_re = J_max * l5_tmp * l11_tmp;
  l101 = l115 * V_max;
  l167_re = l115 * l5_tmp * l10_tmp * 2.0;
  l134 = l3 * J_max;
  b_re = J_min * l2[0].re;
  b_im = J_min * l2[0].im;
  l11_tmp = 2.0 * (l5_tmp * (l20_tmp * l2[0].re));
  c_im = 2.0 * (l5_tmp * (l20_tmp * l2[0].im));
  c_re = l2[0].re * l2[0].re - l2[0].im * l2[0].im;
  l17 = l2[0].re * l2[0].im;
  l105 = l17 + l17;
  l163_im = l115_tmp * c_re;
  l6 = l115_tmp * l105;
  l17 = J_min * b_l7[0].re;
  l135 = J_min * b_l7[0].im;
  l113 = J_min * l8_tmp;
  l20 = l113 * l10_tmp;
  br_tmp = A_wayp + l5[0].re;
  if (l5[0].im == 0.0) {
    d_re = 1.0 / br_tmp;
    d_im = 0.0;
  } else if (br_tmp == 0.0) {
    d_re = 0.0;
    d_im = -(1.0 / l5[0].im);
  } else {
    brm = std::abs(br_tmp);
    bim = std::abs(l5[0].im);
    if (brm > bim) {
      s = l5[0].im / br_tmp;
      bim = br_tmp + s * l5[0].im;
      d_re = (s * 0.0 + 1.0) / bim;
      d_im = (0.0 - s) / bim;
    } else if (bim == brm) {
      if (br_tmp > 0.0) {
        s = 0.5;
      } else {
        s = -0.5;
      }
      if (l5[0].im > 0.0) {
        bim = 0.5;
      } else {
        bim = -0.5;
      }
      d_re = (s + 0.0 * bim) / brm;
      d_im = (0.0 * s - bim) / brm;
    } else {
      s = br_tmp / l5[0].im;
      bim = l5[0].im + s * br_tmp;
      d_re = s / bim;
      d_im = (s * 0.0 - 1.0) / bim;
    }
  }
  l18 = l10_tmp * (l5_tmp * (J_max * l2[0].re));
  l16 = l10_tmp * (l5_tmp * (J_max * l2[0].im));
  l9 = l5_tmp * (l7 * l2[0].re);
  l113_tmp = l5_tmp * (l7 * l2[0].im);
  l172_im = (((((((((l151_tmp + 2.0 * (l134 * b_l7[0].re)) +
                    l5_tmp * (l2_tmp * b_re)) +
                   l25) +
                  l99) +
                 l8_tmp * l163_im) +
                l8_tmp * l17) +
               2.0 * (l102 * c_re)) +
              -(l10_tmp * (l134 * l2[0].re) * 2.0)) +
             l20 * l5[0].re) +
            2.0 * (l18_tmp * c_re);
  l173_im =
      ((((((l172_im + l17 * l5_tmp) - l18 * 2.0) + l5_tmp * l163_im * 2.0) +
         l5_tmp * (b_J_min * l2[0].re) * 2.0) +
        (l9 * l5[0].re - l113_tmp * l5[0].im)) +
       l156.re) -
      l157.re;
  l173_re = ((((((2.0 * (l134 * b_l7[0].im) + l5_tmp * (l2_tmp * b_im)) +
                 l8_tmp * l6) +
                l8_tmp * l135) +
               2.0 * (l102 * l105)) +
              -(l10_tmp * (l134 * l2[0].im) * 2.0)) +
             l20 * l5[0].im) +
            2.0 * (l18_tmp * l105);
  l17 = ((((l173_re + l135 * l5_tmp) - l16 * 2.0) + l5_tmp * l6 * 2.0) +
         l5_tmp * (b_J_min * l2[0].im) * 2.0) +
        (l9 * l5[0].im + l113_tmp * l5[0].re);
  l163_im = d_re * l173_im - d_im * l17;
  l6 = d_re * l17 + d_im * l173_im;
  l9 = l5_tmp * (l115_tmp * l2[0].re);
  l113_tmp = l5_tmp * (l115_tmp * l2[0].im);
  l159_re = ((((((l172_im - l158_re) + J_max * b_l7[0].re * l5_tmp) -
                l10_tmp * (l5_tmp * b_re) * 2.0) +
               l18) +
              l5_tmp * (l7 * c_re) * 2.0) +
             l5_tmp * (l101 * l2[0].re) * 2.0) +
            (l9 * l5[0].re - l113_tmp * l5[0].im);
  l159_im = (((((l173_re + J_max * b_l7[0].im * l5_tmp) -
                l10_tmp * (l5_tmp * b_im) * 2.0) +
               l16) +
              l5_tmp * (l7 * l105) * 2.0) +
             l5_tmp * (l101 * l2[0].im) * 2.0) +
            (l9 * l5[0].im + l113_tmp * l5[0].re);
  b_re = d_re * l159_re - d_im * l159_im;
  b_im = d_re * l159_im + d_im * l159_re;
  l17 = l11_tmp - l167_re;
  l18 = d_re * l17 - d_im * c_im;
  l135 = d_re * c_im + d_im * l17;
  l11_tmp -= l5_tmp * (l115 * c_re) * 2.0;
  c_im -= l5_tmp * (l115 * l105) * 2.0;
  c_re = d_re * l11_tmp - d_im * c_im;
  c_im = d_re * c_im + d_im * l11_tmp;
  ar = l163_im - b_re;
  l159_im = l6 - b_im;
  l13 = l18 + c_re;
  l17 = l135 + c_im;
  if (l17 == 0.0) {
    if (l159_im == 0.0) {
      d_re = ar / l13;
      d_im = 0.0;
    } else if (ar == 0.0) {
      d_re = 0.0;
      d_im = l159_im / l13;
    } else {
      d_re = ar / l13;
      d_im = l159_im / l13;
    }
  } else if (l13 == 0.0) {
    if (ar == 0.0) {
      d_re = l159_im / l17;
      d_im = 0.0;
    } else if (l159_im == 0.0) {
      d_re = 0.0;
      d_im = -(ar / l17);
    } else {
      d_re = l159_im / l17;
      d_im = -(ar / l17);
    }
  } else {
    brm = std::abs(l13);
    bim = std::abs(l17);
    if (brm > bim) {
      s = l17 / l13;
      bim = l13 + s * l17;
      d_re = (ar + s * l159_im) / bim;
      d_im = (l159_im - s * ar) / bim;
    } else if (bim == brm) {
      if (l13 > 0.0) {
        s = 0.5;
      } else {
        s = -0.5;
      }
      if (l17 > 0.0) {
        bim = 0.5;
      } else {
        bim = -0.5;
      }
      d_re = (ar * s + l159_im * bim) / brm;
      d_im = (l159_im * s - ar * bim) / brm;
    } else {
      s = l13 / l17;
      bim = l17 + s * l13;
      d_re = (s * ar + l159_im) / bim;
      d_im = (s * l159_im - ar) / bim;
    }
  }
  l19[0].re = d_re;
  l19[0].im = d_im;
  b_l7[0].re = t7[0].re * t7[0].re - t7[0].im * t7[0].im;
  l17 = t7[0].re * t7[0].im;
  b_l7[0].im = l17 + l17;
  b_re = J_min * l2[1].re;
  b_im = J_min * l2[1].im;
  l11_tmp = 2.0 * (l5_tmp * (l20_tmp * l2[1].re));
  c_im = 2.0 * (l5_tmp * (l20_tmp * l2[1].im));
  c_re = l2[1].re * l2[1].re - l2[1].im * l2[1].im;
  l17 = l2[1].re * l2[1].im;
  l105 = l17 + l17;
  l163_im = l115_tmp * c_re;
  l6 = l115_tmp * l105;
  l17 = J_min * b_l7[1].re;
  l135 = J_min * b_l7[1].im;
  b_br_tmp = A_wayp + l5[1].re;
  if (l5[1].im == 0.0) {
    d_re = 1.0 / b_br_tmp;
    d_im = 0.0;
  } else if (b_br_tmp == 0.0) {
    d_re = 0.0;
    d_im = -(1.0 / l5[1].im);
  } else {
    brm = std::abs(b_br_tmp);
    bim = std::abs(l5[1].im);
    if (brm > bim) {
      s = l5[1].im / b_br_tmp;
      bim = b_br_tmp + s * l5[1].im;
      d_re = (s * 0.0 + 1.0) / bim;
      d_im = (0.0 - s) / bim;
    } else if (bim == brm) {
      if (b_br_tmp > 0.0) {
        s = 0.5;
      } else {
        s = -0.5;
      }
      if (l5[1].im > 0.0) {
        bim = 0.5;
      } else {
        bim = -0.5;
      }
      d_re = (s + 0.0 * bim) / brm;
      d_im = (0.0 * s - bim) / brm;
    } else {
      s = b_br_tmp / l5[1].im;
      bim = l5[1].im + s * b_br_tmp;
      d_re = s / bim;
      d_im = (s * 0.0 - 1.0) / bim;
    }
  }
  l18 = l10_tmp * (l5_tmp * (J_max * l2[1].re));
  l16 = l10_tmp * (l5_tmp * (J_max * l2[1].im));
  l9 = l5_tmp * (l7 * l2[1].re);
  l113_tmp = l5_tmp * (l7 * l2[1].im);
  l172_im = (((((((((l151_tmp + 2.0 * (l134 * b_l7[1].re)) +
                    l5_tmp * (l2_tmp * b_re)) +
                   l25) +
                  l99) +
                 l8_tmp * l163_im) +
                l8_tmp * l17) +
               2.0 * (l102 * c_re)) +
              -(l10_tmp * (l134 * l2[1].re) * 2.0)) +
             l20 * l5[1].re) +
            2.0 * (l18_tmp * c_re);
  l173_im =
      ((((((l172_im + l17 * l5_tmp) - l18 * 2.0) + l5_tmp * l163_im * 2.0) +
         l5_tmp * (b_J_min * l2[1].re) * 2.0) +
        (l9 * l5[1].re - l113_tmp * l5[1].im)) +
       l156.re) -
      l157.re;
  l173_re = ((((((2.0 * (l134 * b_l7[1].im) + l5_tmp * (l2_tmp * b_im)) +
                 l8_tmp * l6) +
                l8_tmp * l135) +
               2.0 * (l102 * l105)) +
              -(l10_tmp * (l134 * l2[1].im) * 2.0)) +
             l20 * l5[1].im) +
            2.0 * (l18_tmp * l105);
  l17 = ((((l173_re + l135 * l5_tmp) - l16 * 2.0) + l5_tmp * l6 * 2.0) +
         l5_tmp * (b_J_min * l2[1].im) * 2.0) +
        (l9 * l5[1].im + l113_tmp * l5[1].re);
  l163_im = d_re * l173_im - d_im * l17;
  l6 = d_re * l17 + d_im * l173_im;
  l9 = l5_tmp * (l115_tmp * l2[1].re);
  l113_tmp = l5_tmp * (l115_tmp * l2[1].im);
  l159_re = ((((((l172_im - l158_re) + J_max * b_l7[1].re * l5_tmp) -
                l10_tmp * (l5_tmp * b_re) * 2.0) +
               l18) +
              l5_tmp * (l7 * c_re) * 2.0) +
             l5_tmp * (l101 * l2[1].re) * 2.0) +
            (l9 * l5[1].re - l113_tmp * l5[1].im);
  l159_im = (((((l173_re + J_max * b_l7[1].im * l5_tmp) -
                l10_tmp * (l5_tmp * b_im) * 2.0) +
               l16) +
              l5_tmp * (l7 * l105) * 2.0) +
             l5_tmp * (l101 * l2[1].im) * 2.0) +
            (l9 * l5[1].im + l113_tmp * l5[1].re);
  b_re = d_re * l159_re - d_im * l159_im;
  b_im = d_re * l159_im + d_im * l159_re;
  l17 = l11_tmp - l167_re;
  l18 = d_re * l17 - d_im * c_im;
  l135 = d_re * c_im + d_im * l17;
  l11_tmp -= l5_tmp * (l115 * c_re) * 2.0;
  c_im -= l5_tmp * (l115 * l105) * 2.0;
  c_re = d_re * l11_tmp - d_im * c_im;
  c_im = d_re * c_im + d_im * l11_tmp;
  ar = l163_im - b_re;
  l159_im = l6 - b_im;
  l13 = l18 + c_re;
  l17 = l135 + c_im;
  if (l17 == 0.0) {
    if (l159_im == 0.0) {
      d_re = ar / l13;
      d_im = 0.0;
    } else if (ar == 0.0) {
      d_re = 0.0;
      d_im = l159_im / l13;
    } else {
      d_re = ar / l13;
      d_im = l159_im / l13;
    }
  } else if (l13 == 0.0) {
    if (ar == 0.0) {
      d_re = l159_im / l17;
      d_im = 0.0;
    } else if (l159_im == 0.0) {
      d_re = 0.0;
      d_im = -(ar / l17);
    } else {
      d_re = l159_im / l17;
      d_im = -(ar / l17);
    }
  } else {
    brm = std::abs(l13);
    bim = std::abs(l17);
    if (brm > bim) {
      s = l17 / l13;
      bim = l13 + s * l17;
      d_re = (ar + s * l159_im) / bim;
      d_im = (l159_im - s * ar) / bim;
    } else if (bim == brm) {
      if (l13 > 0.0) {
        s = 0.5;
      } else {
        s = -0.5;
      }
      if (l17 > 0.0) {
        bim = 0.5;
      } else {
        bim = -0.5;
      }
      d_re = (ar * s + l159_im * bim) / brm;
      d_im = (l159_im * s - ar * bim) / brm;
    } else {
      s = l13 / l17;
      bim = l17 + s * l13;
      d_re = (s * ar + l159_im) / bim;
      d_im = (s * l159_im - ar) / bim;
    }
  }
  l19[1].re = d_re;
  l19[1].im = d_im;
  b_l7[1].re = t7[1].re * t7[1].re - t7[1].im * t7[1].im;
  l17 = t7[1].re * t7[1].im;
  b_l7[1].im = l17 + l17;
  b_re = J_min * l2[2].re;
  b_im = J_min * l2[2].im;
  l11_tmp = 2.0 * (l5_tmp * (l20_tmp * l2[2].re));
  c_im = 2.0 * (l5_tmp * (l20_tmp * l2[2].im));
  c_re = l2[2].re * l2[2].re - l2[2].im * l2[2].im;
  l17 = l2[2].re * l2[2].im;
  l105 = l17 + l17;
  l163_im = l115_tmp * c_re;
  l6 = l115_tmp * l105;
  l17 = J_min * b_l7[2].re;
  l135 = J_min * b_l7[2].im;
  l13 = A_wayp + l5[2].re;
  if (l5[2].im == 0.0) {
    d_re = 1.0 / l13;
    d_im = 0.0;
  } else if (l13 == 0.0) {
    d_re = 0.0;
    d_im = -(1.0 / l5[2].im);
  } else {
    brm = std::abs(l13);
    bim = std::abs(l5[2].im);
    if (brm > bim) {
      s = l5[2].im / l13;
      bim = l13 + s * l5[2].im;
      d_re = (s * 0.0 + 1.0) / bim;
      d_im = (0.0 - s) / bim;
    } else if (bim == brm) {
      if (l13 > 0.0) {
        s = 0.5;
      } else {
        s = -0.5;
      }
      if (l5[2].im > 0.0) {
        bim = 0.5;
      } else {
        bim = -0.5;
      }
      d_re = (s + 0.0 * bim) / brm;
      d_im = (0.0 * s - bim) / brm;
    } else {
      s = l13 / l5[2].im;
      bim = l5[2].im + s * l13;
      d_re = s / bim;
      d_im = (s * 0.0 - 1.0) / bim;
    }
  }
  l18 = l10_tmp * (l5_tmp * (J_max * l2[2].re));
  l16 = l10_tmp * (l5_tmp * (J_max * l2[2].im));
  l9 = l5_tmp * (l7 * l2[2].re);
  l113_tmp = l5_tmp * (l7 * l2[2].im);
  l172_im = (((((((((l151_tmp + 2.0 * (l134 * b_l7[2].re)) +
                    l5_tmp * (l2_tmp * b_re)) +
                   l25) +
                  l99) +
                 l8_tmp * l163_im) +
                l8_tmp * l17) +
               2.0 * (l102 * c_re)) +
              -(l10_tmp * (l134 * l2[2].re) * 2.0)) +
             l20 * l5[2].re) +
            2.0 * (l18_tmp * c_re);
  l173_im =
      ((((((l172_im + l17 * l5_tmp) - l18 * 2.0) + l5_tmp * l163_im * 2.0) +
         l5_tmp * (b_J_min * l2[2].re) * 2.0) +
        (l9 * l5[2].re - l113_tmp * l5[2].im)) +
       l156.re) -
      l157.re;
  l173_re = ((((((2.0 * (l134 * b_l7[2].im) + l5_tmp * (l2_tmp * b_im)) +
                 l8_tmp * l6) +
                l8_tmp * l135) +
               2.0 * (l102 * l105)) +
              -(l10_tmp * (l134 * l2[2].im) * 2.0)) +
             l20 * l5[2].im) +
            2.0 * (l18_tmp * l105);
  l17 = ((((l173_re + l135 * l5_tmp) - l16 * 2.0) + l5_tmp * l6 * 2.0) +
         l5_tmp * (b_J_min * l2[2].im) * 2.0) +
        (l9 * l5[2].im + l113_tmp * l5[2].re);
  l163_im = d_re * l173_im - d_im * l17;
  l6 = d_re * l17 + d_im * l173_im;
  l9 = l5_tmp * (l115_tmp * l2[2].re);
  l113_tmp = l5_tmp * (l115_tmp * l2[2].im);
  l159_re = ((((((l172_im - l158_re) + J_max * b_l7[2].re * l5_tmp) -
                l10_tmp * (l5_tmp * b_re) * 2.0) +
               l18) +
              l5_tmp * (l7 * c_re) * 2.0) +
             l5_tmp * (l101 * l2[2].re) * 2.0) +
            (l9 * l5[2].re - l113_tmp * l5[2].im);
  l159_im = (((((l173_re + J_max * b_l7[2].im * l5_tmp) -
                l10_tmp * (l5_tmp * b_im) * 2.0) +
               l16) +
              l5_tmp * (l7 * l105) * 2.0) +
             l5_tmp * (l101 * l2[2].im) * 2.0) +
            (l9 * l5[2].im + l113_tmp * l5[2].re);
  b_re = d_re * l159_re - d_im * l159_im;
  b_im = d_re * l159_im + d_im * l159_re;
  l17 = l11_tmp - l167_re;
  l18 = d_re * l17 - d_im * c_im;
  l135 = d_re * c_im + d_im * l17;
  l11_tmp -= l5_tmp * (l115 * c_re) * 2.0;
  c_im -= l5_tmp * (l115 * l105) * 2.0;
  c_re = d_re * l11_tmp - d_im * c_im;
  c_im = d_re * c_im + d_im * l11_tmp;
  ar = l163_im - b_re;
  l159_im = l6 - b_im;
  l13 = l18 + c_re;
  l17 = l135 + c_im;
  if (l17 == 0.0) {
    if (l159_im == 0.0) {
      d_re = ar / l13;
      d_im = 0.0;
    } else if (ar == 0.0) {
      d_re = 0.0;
      d_im = l159_im / l13;
    } else {
      d_re = ar / l13;
      d_im = l159_im / l13;
    }
  } else if (l13 == 0.0) {
    if (ar == 0.0) {
      d_re = l159_im / l17;
      d_im = 0.0;
    } else if (l159_im == 0.0) {
      d_re = 0.0;
      d_im = -(ar / l17);
    } else {
      d_re = l159_im / l17;
      d_im = -(ar / l17);
    }
  } else {
    brm = std::abs(l13);
    bim = std::abs(l17);
    if (brm > bim) {
      s = l17 / l13;
      bim = l13 + s * l17;
      d_re = (ar + s * l159_im) / bim;
      d_im = (l159_im - s * ar) / bim;
    } else if (bim == brm) {
      if (l13 > 0.0) {
        s = 0.5;
      } else {
        s = -0.5;
      }
      if (l17 > 0.0) {
        bim = 0.5;
      } else {
        bim = -0.5;
      }
      d_re = (ar * s + l159_im * bim) / brm;
      d_im = (l159_im * s - ar * bim) / brm;
    } else {
      s = l13 / l17;
      bim = l17 + s * l13;
      d_re = (s * ar + l159_im) / bim;
      d_im = (s * l159_im - ar) / bim;
    }
  }
  l19[2].re = d_re;
  l19[2].im = d_im;
  b_l7[2].re = t7[2].re * t7[2].re - t7[2].im * t7[2].im;
  l17 = t7[2].re * t7[2].im;
  b_l7[2].im = l17 + l17;
  b_re = J_min * re;
  b_im = J_min * im;
  l11_tmp = 2.0 * (l5_tmp * (l20_tmp * re));
  c_im = 2.0 * (l5_tmp * (l20_tmp * im));
  c_re = re * re - im * im;
  l17 = re * im;
  l105 = l17 + l17;
  l163_im = l115_tmp * c_re;
  l6 = l115_tmp * l105;
  l17 = J_min * b_l7[3].re;
  l135 = J_min * b_l7[3].im;
  c_br_tmp = A_wayp + -re;
  if (-im == 0.0) {
    d_re = 1.0 / c_br_tmp;
    d_im = 0.0;
  } else if (c_br_tmp == 0.0) {
    d_re = 0.0;
    d_im = -(1.0 / -im);
  } else {
    brm = std::abs(c_br_tmp);
    bim = std::abs(-im);
    if (brm > bim) {
      s = -im / c_br_tmp;
      bim = c_br_tmp + s * -im;
      d_re = (s * 0.0 + 1.0) / bim;
      d_im = (0.0 - s) / bim;
    } else if (bim == brm) {
      if (c_br_tmp > 0.0) {
        s = 0.5;
      } else {
        s = -0.5;
      }
      if (-im > 0.0) {
        bim = 0.5;
      } else {
        bim = -0.5;
      }
      d_re = (s + 0.0 * bim) / brm;
      d_im = (0.0 * s - bim) / brm;
    } else {
      s = c_br_tmp / -im;
      bim = -im + s * c_br_tmp;
      d_re = s / bim;
      d_im = (s * 0.0 - 1.0) / bim;
    }
  }
  l18 = l10_tmp * (l5_tmp * (J_max * re));
  l16 = l10_tmp * (l5_tmp * (J_max * im));
  l9 = l5_tmp * (l7 * re);
  l113_tmp = l5_tmp * (l7 * im);
  l172_im = (((((((((l151_tmp + 2.0 * (l134 * b_l7[3].re)) +
                    l5_tmp * (l2_tmp * b_re)) +
                   l25) +
                  l99) +
                 l8_tmp * l163_im) +
                l8_tmp * l17) +
               2.0 * (l102 * c_re)) +
              -(l10_tmp * (l134 * re) * 2.0)) +
             l20 * -re) +
            2.0 * (l18_tmp * c_re);
  l173_im =
      ((((((l172_im + l17 * l5_tmp) - l18 * 2.0) + l5_tmp * l163_im * 2.0) +
         l5_tmp * (b_J_min * re) * 2.0) +
        (l9 * -re - l113_tmp * -im)) +
       l156.re) -
      l157.re;
  l173_re = ((((((2.0 * (l134 * b_l7[3].im) + l5_tmp * (l2_tmp * b_im)) +
                 l8_tmp * l6) +
                l8_tmp * l135) +
               2.0 * (l102 * l105)) +
              -(l10_tmp * (l134 * im) * 2.0)) +
             l20 * -im) +
            2.0 * (l18_tmp * l105);
  l17 = ((((l173_re + l135 * l5_tmp) - l16 * 2.0) + l5_tmp * l6 * 2.0) +
         l5_tmp * (b_J_min * im) * 2.0) +
        (l9 * -im + l113_tmp * -re);
  l163_im = d_re * l173_im - d_im * l17;
  l6 = d_re * l17 + d_im * l173_im;
  l9 = l5_tmp * (l115_tmp * re);
  l113_tmp = l5_tmp * (l115_tmp * im);
  l159_re = ((((((l172_im - l158_re) + J_max * b_l7[3].re * l5_tmp) -
                l10_tmp * (l5_tmp * b_re) * 2.0) +
               l18) +
              l5_tmp * (l7 * c_re) * 2.0) +
             l5_tmp * (l101 * re) * 2.0) +
            (l9 * -re - l113_tmp * -im);
  l159_im = (((((l173_re + J_max * b_l7[3].im * l5_tmp) -
                l10_tmp * (l5_tmp * b_im) * 2.0) +
               l16) +
              l5_tmp * (l7 * l105) * 2.0) +
             l5_tmp * (l101 * im) * 2.0) +
            (l9 * -im + l113_tmp * -re);
  b_re = d_re * l159_re - d_im * l159_im;
  b_im = d_re * l159_im + d_im * l159_re;
  l17 = l11_tmp - l167_re;
  l18 = d_re * l17 - d_im * c_im;
  l135 = d_re * c_im + d_im * l17;
  l11_tmp -= l5_tmp * (l115 * c_re) * 2.0;
  c_im -= l5_tmp * (l115 * l105) * 2.0;
  c_re = d_re * l11_tmp - d_im * c_im;
  c_im = d_re * c_im + d_im * l11_tmp;
  ar = l163_im - b_re;
  l159_im = l6 - b_im;
  l13 = l18 + c_re;
  l17 = l135 + c_im;
  if (l17 == 0.0) {
    if (l159_im == 0.0) {
      d_re = ar / l13;
      d_im = 0.0;
    } else if (ar == 0.0) {
      d_re = 0.0;
      d_im = l159_im / l13;
    } else {
      d_re = ar / l13;
      d_im = l159_im / l13;
    }
  } else if (l13 == 0.0) {
    if (ar == 0.0) {
      d_re = l159_im / l17;
      d_im = 0.0;
    } else if (l159_im == 0.0) {
      d_re = 0.0;
      d_im = -(ar / l17);
    } else {
      d_re = l159_im / l17;
      d_im = -(ar / l17);
    }
  } else {
    brm = std::abs(l13);
    bim = std::abs(l17);
    if (brm > bim) {
      s = l17 / l13;
      bim = l13 + s * l17;
      d_re = (ar + s * l159_im) / bim;
      d_im = (l159_im - s * ar) / bim;
    } else if (bim == brm) {
      if (l13 > 0.0) {
        s = 0.5;
      } else {
        s = -0.5;
      }
      if (l17 > 0.0) {
        bim = 0.5;
      } else {
        bim = -0.5;
      }
      d_re = (ar * s + l159_im * bim) / brm;
      d_im = (l159_im * s - ar * bim) / brm;
    } else {
      s = l13 / l17;
      bim = l17 + s * l13;
      d_re = (s * ar + l159_im) / bim;
      d_im = (s * l159_im - ar) / bim;
    }
  }
  l19[3].re = d_re;
  l19[3].im = d_im;
  b_l7[3].re = t7[3].re * t7[3].re - t7[3].im * t7[3].im;
  l17 = t7[3].re * t7[3].im;
  b_l7[3].im = l17 + l17;
  l167_re = l113 * l10_tmp - l12 * l10_tmp;
  l163_im = l8_tmp * l15_tmp;
  l151.re = l2_tmp * J_min * l5_tmp;
  b_J_min = l113 * l14_tmp;
  l156.re = l134 * l10_tmp * 2.0;
  l157.re = l115 * V_init * l5_tmp * 2.0;
  l159_re = l101 * l5_tmp * 2.0;
  l6 = l3 * l15_tmp;
  l113_tmp = A_wayp * l8_tmp * l14_tmp;
  l16 = l3 * A_wayp;
  l18 = l16 * l14_tmp;
  l9 = l26_tmp_tmp * l14_tmp;
  l173_re = l20_tmp * l8_tmp;
  l158_re = l18_tmp_tmp * A_wayp * J_min * 2.0;
  l16 = l16 * J_min * J_max;
  b_A_min.re = l16 * T * 2.0;
  l172_im = l18_tmp * J_max;
  l173_im = l26_tmp_tmp * T * l14_tmp;
  l12 = l16 * 2.0;
  l3 = 1.0 / J_min;
  l17 = l9 * l19[0].re;
  l135 = l9 * l19[0].im;
  ar = (((((((((((((((((l167_re - l163_im * b_l7[0].re) + l151.re) +
                      b_J_min * b_l7[0].re) +
                     l156.re) -
                    l157.re) +
                   l159_re) +
                  l6 * b_l7[0].re * 2.0) +
                 l113_tmp * t7[0].re * 2.0) -
                l18 * t7[0].re * 4.0) -
               l9 * b_l7[0].re * 2.0) -
              l173_re * t7[0].re * 2.0) -
             l158_re) -
            b_A_min.re) +
           l172_im * t7[0].re * 2.0) +
          l16 * l19[0].re * 2.0) +
         l16 * t7[0].re * 2.0) +
        l173_im * t7[0].re * 2.0) -
       (l17 * t7[0].re - l135 * t7[0].im) * 2.0;
  l159_im = (((((((((((0.0 - l163_im * b_l7[0].im) + b_J_min * b_l7[0].im) +
                     l6 * b_l7[0].im * 2.0) +
                    l113_tmp * t7[0].im * 2.0) -
                   l18 * t7[0].im * 4.0) -
                  l9 * b_l7[0].im * 2.0) -
                 l173_re * t7[0].im * 2.0) +
                l172_im * t7[0].im * 2.0) +
               l16 * l19[0].im * 2.0) +
              l16 * t7[0].im * 2.0) +
             l173_im * t7[0].im * 2.0) -
            (l17 * t7[0].im + l135 * t7[0].re) * 2.0;
  l13 = l9 * t7[0].re * 2.0 - l12;
  l17 = l9 * t7[0].im * 2.0;
  if (l17 == 0.0) {
    if (l159_im == 0.0) {
      b_re = ar / l13;
      b_im = 0.0;
    } else if (ar == 0.0) {
      b_re = 0.0;
      b_im = l159_im / l13;
    } else {
      b_re = ar / l13;
      b_im = l159_im / l13;
    }
  } else if (l13 == 0.0) {
    if (ar == 0.0) {
      b_re = l159_im / l17;
      b_im = 0.0;
    } else if (l159_im == 0.0) {
      b_re = 0.0;
      b_im = -(ar / l17);
    } else {
      b_re = l159_im / l17;
      b_im = -(ar / l17);
    }
  } else {
    brm = std::abs(l13);
    bim = std::abs(l17);
    if (brm > bim) {
      s = l17 / l13;
      bim = l13 + s * l17;
      b_re = (ar + s * l159_im) / bim;
      b_im = (l159_im - s * ar) / bim;
    } else if (bim == brm) {
      if (l13 > 0.0) {
        s = 0.5;
      } else {
        s = -0.5;
      }
      if (l17 > 0.0) {
        bim = 0.5;
      } else {
        bim = -0.5;
      }
      b_re = (ar * s + l159_im * bim) / brm;
      b_im = (l159_im * s - ar * bim) / brm;
    } else {
      s = l13 / l17;
      bim = l17 + s * l13;
      b_re = (s * ar + l159_im) / bim;
      b_im = (s * l159_im - ar) / bim;
    }
  }
  b_l7[0].re = b_re;
  b_l7[0].im = b_im;
  l17 = l9 * l19[1].re;
  l135 = l9 * l19[1].im;
  ar = (((((((((((((((((l167_re - l163_im * b_l7[1].re) + l151.re) +
                      b_J_min * b_l7[1].re) +
                     l156.re) -
                    l157.re) +
                   l159_re) +
                  l6 * b_l7[1].re * 2.0) +
                 l113_tmp * t7[1].re * 2.0) -
                l18 * t7[1].re * 4.0) -
               l9 * b_l7[1].re * 2.0) -
              l173_re * t7[1].re * 2.0) -
             l158_re) -
            b_A_min.re) +
           l172_im * t7[1].re * 2.0) +
          l16 * l19[1].re * 2.0) +
         l16 * t7[1].re * 2.0) +
        l173_im * t7[1].re * 2.0) -
       (l17 * t7[1].re - l135 * t7[1].im) * 2.0;
  l159_im = (((((((((((0.0 - l163_im * b_l7[1].im) + b_J_min * b_l7[1].im) +
                     l6 * b_l7[1].im * 2.0) +
                    l113_tmp * t7[1].im * 2.0) -
                   l18 * t7[1].im * 4.0) -
                  l9 * b_l7[1].im * 2.0) -
                 l173_re * t7[1].im * 2.0) +
                l172_im * t7[1].im * 2.0) +
               l16 * l19[1].im * 2.0) +
              l16 * t7[1].im * 2.0) +
             l173_im * t7[1].im * 2.0) -
            (l17 * t7[1].im + l135 * t7[1].re) * 2.0;
  l13 = l9 * t7[1].re * 2.0 - l12;
  l17 = l9 * t7[1].im * 2.0;
  if (l17 == 0.0) {
    if (l159_im == 0.0) {
      b_re = ar / l13;
      b_im = 0.0;
    } else if (ar == 0.0) {
      b_re = 0.0;
      b_im = l159_im / l13;
    } else {
      b_re = ar / l13;
      b_im = l159_im / l13;
    }
  } else if (l13 == 0.0) {
    if (ar == 0.0) {
      b_re = l159_im / l17;
      b_im = 0.0;
    } else if (l159_im == 0.0) {
      b_re = 0.0;
      b_im = -(ar / l17);
    } else {
      b_re = l159_im / l17;
      b_im = -(ar / l17);
    }
  } else {
    brm = std::abs(l13);
    bim = std::abs(l17);
    if (brm > bim) {
      s = l17 / l13;
      bim = l13 + s * l17;
      b_re = (ar + s * l159_im) / bim;
      b_im = (l159_im - s * ar) / bim;
    } else if (bim == brm) {
      if (l13 > 0.0) {
        s = 0.5;
      } else {
        s = -0.5;
      }
      if (l17 > 0.0) {
        bim = 0.5;
      } else {
        bim = -0.5;
      }
      b_re = (ar * s + l159_im * bim) / brm;
      b_im = (l159_im * s - ar * bim) / brm;
    } else {
      s = l13 / l17;
      bim = l17 + s * l13;
      b_re = (s * ar + l159_im) / bim;
      b_im = (s * l159_im - ar) / bim;
    }
  }
  b_l7[1].re = b_re;
  b_l7[1].im = b_im;
  l17 = l9 * l19[2].re;
  l135 = l9 * l19[2].im;
  ar = (((((((((((((((((l167_re - l163_im * b_l7[2].re) + l151.re) +
                      b_J_min * b_l7[2].re) +
                     l156.re) -
                    l157.re) +
                   l159_re) +
                  l6 * b_l7[2].re * 2.0) +
                 l113_tmp * t7[2].re * 2.0) -
                l18 * t7[2].re * 4.0) -
               l9 * b_l7[2].re * 2.0) -
              l173_re * t7[2].re * 2.0) -
             l158_re) -
            b_A_min.re) +
           l172_im * t7[2].re * 2.0) +
          l16 * l19[2].re * 2.0) +
         l16 * t7[2].re * 2.0) +
        l173_im * t7[2].re * 2.0) -
       (l17 * t7[2].re - l135 * t7[2].im) * 2.0;
  l159_im = (((((((((((0.0 - l163_im * b_l7[2].im) + b_J_min * b_l7[2].im) +
                     l6 * b_l7[2].im * 2.0) +
                    l113_tmp * t7[2].im * 2.0) -
                   l18 * t7[2].im * 4.0) -
                  l9 * b_l7[2].im * 2.0) -
                 l173_re * t7[2].im * 2.0) +
                l172_im * t7[2].im * 2.0) +
               l16 * l19[2].im * 2.0) +
              l16 * t7[2].im * 2.0) +
             l173_im * t7[2].im * 2.0) -
            (l17 * t7[2].im + l135 * t7[2].re) * 2.0;
  l13 = l9 * t7[2].re * 2.0 - l12;
  l17 = l9 * t7[2].im * 2.0;
  if (l17 == 0.0) {
    if (l159_im == 0.0) {
      b_re = ar / l13;
      b_im = 0.0;
    } else if (ar == 0.0) {
      b_re = 0.0;
      b_im = l159_im / l13;
    } else {
      b_re = ar / l13;
      b_im = l159_im / l13;
    }
  } else if (l13 == 0.0) {
    if (ar == 0.0) {
      b_re = l159_im / l17;
      b_im = 0.0;
    } else if (l159_im == 0.0) {
      b_re = 0.0;
      b_im = -(ar / l17);
    } else {
      b_re = l159_im / l17;
      b_im = -(ar / l17);
    }
  } else {
    brm = std::abs(l13);
    bim = std::abs(l17);
    if (brm > bim) {
      s = l17 / l13;
      bim = l13 + s * l17;
      b_re = (ar + s * l159_im) / bim;
      b_im = (l159_im - s * ar) / bim;
    } else if (bim == brm) {
      if (l13 > 0.0) {
        s = 0.5;
      } else {
        s = -0.5;
      }
      if (l17 > 0.0) {
        bim = 0.5;
      } else {
        bim = -0.5;
      }
      b_re = (ar * s + l159_im * bim) / brm;
      b_im = (l159_im * s - ar * bim) / brm;
    } else {
      s = l13 / l17;
      bim = l17 + s * l13;
      b_re = (s * ar + l159_im) / bim;
      b_im = (s * l159_im - ar) / bim;
    }
  }
  b_l7[2].re = b_re;
  b_l7[2].im = b_im;
  l5[2].re += A_wayp;
  l17 = l9 * d_re;
  l135 = l9 * d_im;
  ar = (((((((((((((((((l167_re - l163_im * b_l7[3].re) + l151.re) +
                      b_J_min * b_l7[3].re) +
                     l156.re) -
                    l157.re) +
                   l159_re) +
                  l6 * b_l7[3].re * 2.0) +
                 l113_tmp * t7[3].re * 2.0) -
                l18 * t7[3].re * 4.0) -
               l9 * b_l7[3].re * 2.0) -
              l173_re * t7[3].re * 2.0) -
             l158_re) -
            b_A_min.re) +
           l172_im * t7[3].re * 2.0) +
          l16 * d_re * 2.0) +
         l16 * t7[3].re * 2.0) +
        l173_im * t7[3].re * 2.0) -
       (l17 * t7[3].re - l135 * t7[3].im) * 2.0;
  l159_im = (((((((((((0.0 - l163_im * b_l7[3].im) + b_J_min * b_l7[3].im) +
                     l6 * b_l7[3].im * 2.0) +
                    l113_tmp * t7[3].im * 2.0) -
                   l18 * t7[3].im * 4.0) -
                  l9 * b_l7[3].im * 2.0) -
                 l173_re * t7[3].im * 2.0) +
                l172_im * t7[3].im * 2.0) +
               l16 * d_im * 2.0) +
              l16 * t7[3].im * 2.0) +
             l173_im * t7[3].im * 2.0) -
            (l17 * t7[3].im + l135 * t7[3].re) * 2.0;
  l13 = l9 * t7[3].re * 2.0 - l12;
  l17 = l9 * t7[3].im * 2.0;
  if (l17 == 0.0) {
    if (l159_im == 0.0) {
      b_re = ar / l13;
      b_im = 0.0;
    } else if (ar == 0.0) {
      b_re = 0.0;
      b_im = l159_im / l13;
    } else {
      b_re = ar / l13;
      b_im = l159_im / l13;
    }
  } else if (l13 == 0.0) {
    if (ar == 0.0) {
      b_re = l159_im / l17;
      b_im = 0.0;
    } else if (l159_im == 0.0) {
      b_re = 0.0;
      b_im = -(ar / l17);
    } else {
      b_re = l159_im / l17;
      b_im = -(ar / l17);
    }
  } else {
    brm = std::abs(l13);
    bim = std::abs(l17);
    if (brm > bim) {
      s = l17 / l13;
      bim = l13 + s * l17;
      b_re = (ar + s * l159_im) / bim;
      b_im = (l159_im - s * ar) / bim;
    } else if (bim == brm) {
      if (l13 > 0.0) {
        s = 0.5;
      } else {
        s = -0.5;
      }
      if (l17 > 0.0) {
        bim = 0.5;
      } else {
        bim = -0.5;
      }
      b_re = (ar * s + l159_im * bim) / brm;
      b_im = (l159_im * s - ar * bim) / brm;
    } else {
      s = l13 / l17;
      bim = l17 + s * l13;
      b_re = (s * ar + l159_im) / bim;
      b_im = (s * l159_im - ar) / bim;
    }
  }
  b_l7[3].re = b_re;
  b_l7[3].im = b_im;
  l17 = A_min * J_max;
  l6 = A_min * J_min;
  l135 = J_max * T;
  b_J_min = J_min - J_max;
  l173_im = J_min + -J_max;
  ar = l3 * (l6 * ((A_init - J_max * (((b_l7[0].re + l19[0].re) + t7[0].re) +
                                      l3 * br_tmp)) +
                   l135) -
             A_max * br_tmp * b_J_min);
  l159_im = l3 * (l6 * (0.0 - J_max * (((b_l7[0].im + l19[0].im) + t7[0].im) +
                                       l3 * l5[0].im)) -
                  A_max * l5[0].im * b_J_min);
  if (l159_im == 0.0) {
    l11_tmp = ar / l17;
    c_im = 0.0;
  } else if (ar == 0.0) {
    l11_tmp = 0.0;
    c_im = l159_im / l17;
  } else {
    l11_tmp = ar / l17;
    c_im = l159_im / l17;
  }
  ar = A_wayp + -J_max * t7[0].re;
  l159_im = -J_max * t7[0].im;
  if (l159_im == 0.0) {
    l9 = ar / J_min;
    l113_tmp = 0.0;
  } else if (ar == 0.0) {
    l9 = 0.0;
    l113_tmp = l159_im / J_min;
  } else {
    l9 = ar / J_min;
    l113_tmp = l159_im / J_min;
  }
  ar = -((A_init +
          -J_max * ((((l11_tmp + b_l7[0].re) + l19[0].re) + t7[0].re) + l9)) +
         l135);
  l159_im =
      -(-J_max * ((((c_im + b_l7[0].im) + l19[0].im) + t7[0].im) + l113_tmp));
  if (l159_im == 0.0) {
    c_re = ar / l173_im;
    l105 = 0.0;
  } else if (ar == 0.0) {
    c_re = 0.0;
    l105 = l159_im / l173_im;
  } else {
    c_re = ar / l173_im;
    l105 = l159_im / l173_im;
  }
  ar = -(A_init + J_min * c_re);
  l159_im = -(J_min * l105);
  if (l159_im == 0.0) {
    t[0].re = ar / J_max;
    t[0].im = 0.0;
  } else if (ar == 0.0) {
    t[0].re = 0.0;
    t[0].im = l159_im / J_max;
  } else {
    t[0].re = ar / J_max;
    t[0].im = l159_im / J_max;
  }
  t[4].re = l11_tmp;
  t[4].im = c_im;
  t[8].re = c_re;
  t[8].im = l105;
  t[12] = b_l7[0];
  ar = A_wayp - l2[0].re;
  if (0.0 - l2[0].im == 0.0) {
    t[16].re = ar / J_min;
    t[16].im = 0.0;
  } else if (ar == 0.0) {
    t[16].re = 0.0;
    t[16].im = (0.0 - l2[0].im) / J_min;
  } else {
    t[16].re = ar / J_min;
    t[16].im = (0.0 - l2[0].im) / J_min;
  }
  t[20] = l19[0];
  t[24] = t7[0];
  ar = l3 * (l6 * ((A_init - J_max * (((b_l7[1].re + l19[1].re) + t7[1].re) +
                                      l3 * b_br_tmp)) +
                   l135) -
             A_max * b_br_tmp * b_J_min);
  l159_im = l3 * (l6 * (0.0 - J_max * (((b_l7[1].im + l19[1].im) + t7[1].im) +
                                       l3 * l5[1].im)) -
                  A_max * l5[1].im * b_J_min);
  if (l159_im == 0.0) {
    l11_tmp = ar / l17;
    c_im = 0.0;
  } else if (ar == 0.0) {
    l11_tmp = 0.0;
    c_im = l159_im / l17;
  } else {
    l11_tmp = ar / l17;
    c_im = l159_im / l17;
  }
  ar = A_wayp + -J_max * t7[1].re;
  l159_im = -J_max * t7[1].im;
  if (l159_im == 0.0) {
    l9 = ar / J_min;
    l113_tmp = 0.0;
  } else if (ar == 0.0) {
    l9 = 0.0;
    l113_tmp = l159_im / J_min;
  } else {
    l9 = ar / J_min;
    l113_tmp = l159_im / J_min;
  }
  ar = -((A_init +
          -J_max * ((((l11_tmp + b_l7[1].re) + l19[1].re) + t7[1].re) + l9)) +
         l135);
  l159_im =
      -(-J_max * ((((c_im + b_l7[1].im) + l19[1].im) + t7[1].im) + l113_tmp));
  if (l159_im == 0.0) {
    c_re = ar / l173_im;
    l105 = 0.0;
  } else if (ar == 0.0) {
    c_re = 0.0;
    l105 = l159_im / l173_im;
  } else {
    c_re = ar / l173_im;
    l105 = l159_im / l173_im;
  }
  ar = -(A_init + J_min * c_re);
  l159_im = -(J_min * l105);
  if (l159_im == 0.0) {
    t[1].re = ar / J_max;
    t[1].im = 0.0;
  } else if (ar == 0.0) {
    t[1].re = 0.0;
    t[1].im = l159_im / J_max;
  } else {
    t[1].re = ar / J_max;
    t[1].im = l159_im / J_max;
  }
  t[5].re = l11_tmp;
  t[5].im = c_im;
  t[9].re = c_re;
  t[9].im = l105;
  t[13] = b_l7[1];
  ar = A_wayp - l2[1].re;
  if (0.0 - l2[1].im == 0.0) {
    t[17].re = ar / J_min;
    t[17].im = 0.0;
  } else if (ar == 0.0) {
    t[17].re = 0.0;
    t[17].im = (0.0 - l2[1].im) / J_min;
  } else {
    t[17].re = ar / J_min;
    t[17].im = (0.0 - l2[1].im) / J_min;
  }
  t[21] = l19[1];
  t[25] = t7[1];
  ar = l3 * (l6 * ((A_init - J_max * (((b_l7[2].re + l19[2].re) + t7[2].re) +
                                      l3 * l5[2].re)) +
                   l135) -
             A_max * l5[2].re * b_J_min);
  l159_im = l3 * (l6 * (0.0 - J_max * (((b_l7[2].im + l19[2].im) + t7[2].im) +
                                       l3 * l5[2].im)) -
                  A_max * l5[2].im * b_J_min);
  if (l159_im == 0.0) {
    l11_tmp = ar / l17;
    c_im = 0.0;
  } else if (ar == 0.0) {
    l11_tmp = 0.0;
    c_im = l159_im / l17;
  } else {
    l11_tmp = ar / l17;
    c_im = l159_im / l17;
  }
  ar = A_wayp + -J_max * t7[2].re;
  l159_im = -J_max * t7[2].im;
  if (l159_im == 0.0) {
    l9 = ar / J_min;
    l113_tmp = 0.0;
  } else if (ar == 0.0) {
    l9 = 0.0;
    l113_tmp = l159_im / J_min;
  } else {
    l9 = ar / J_min;
    l113_tmp = l159_im / J_min;
  }
  ar = -((A_init +
          -J_max * ((((l11_tmp + b_l7[2].re) + l19[2].re) + t7[2].re) + l9)) +
         l135);
  l159_im =
      -(-J_max * ((((c_im + b_l7[2].im) + l19[2].im) + t7[2].im) + l113_tmp));
  if (l159_im == 0.0) {
    c_re = ar / l173_im;
    l105 = 0.0;
  } else if (ar == 0.0) {
    c_re = 0.0;
    l105 = l159_im / l173_im;
  } else {
    c_re = ar / l173_im;
    l105 = l159_im / l173_im;
  }
  ar = -(A_init + J_min * c_re);
  l159_im = -(J_min * l105);
  if (l159_im == 0.0) {
    t[2].re = ar / J_max;
    t[2].im = 0.0;
  } else if (ar == 0.0) {
    t[2].re = 0.0;
    t[2].im = l159_im / J_max;
  } else {
    t[2].re = ar / J_max;
    t[2].im = l159_im / J_max;
  }
  t[6].re = l11_tmp;
  t[6].im = c_im;
  t[10].re = c_re;
  t[10].im = l105;
  t[14] = b_l7[2];
  ar = A_wayp - l2[2].re;
  if (0.0 - l2[2].im == 0.0) {
    t[18].re = ar / J_min;
    t[18].im = 0.0;
  } else if (ar == 0.0) {
    t[18].re = 0.0;
    t[18].im = (0.0 - l2[2].im) / J_min;
  } else {
    t[18].re = ar / J_min;
    t[18].im = (0.0 - l2[2].im) / J_min;
  }
  t[22] = l19[2];
  t[26] = t7[2];
  ar = l3 *
       (l6 * ((A_init - J_max * (((b_re + d_re) + t7[3].re) + l3 * c_br_tmp)) +
              l135) -
        A_max * c_br_tmp * b_J_min);
  l159_im = l3 * (l6 * (0.0 - J_max * (((b_im + d_im) + t7[3].im) + l3 * -im)) -
                  A_max * -im * b_J_min);
  if (l159_im == 0.0) {
    l11_tmp = ar / l17;
    c_im = 0.0;
  } else if (ar == 0.0) {
    l11_tmp = 0.0;
    c_im = l159_im / l17;
  } else {
    l11_tmp = ar / l17;
    c_im = l159_im / l17;
  }
  ar = A_wayp + -J_max * t7[3].re;
  l159_im = -J_max * t7[3].im;
  if (l159_im == 0.0) {
    l9 = ar / J_min;
    l113_tmp = 0.0;
  } else if (ar == 0.0) {
    l9 = 0.0;
    l113_tmp = l159_im / J_min;
  } else {
    l9 = ar / J_min;
    l113_tmp = l159_im / J_min;
  }
  ar = -((A_init + -J_max * ((((l11_tmp + b_re) + d_re) + t7[3].re) + l9)) +
         l135);
  l159_im = -(-J_max * ((((c_im + b_im) + d_im) + t7[3].im) + l113_tmp));
  if (l159_im == 0.0) {
    c_re = ar / l173_im;
    l105 = 0.0;
  } else if (ar == 0.0) {
    c_re = 0.0;
    l105 = l159_im / l173_im;
  } else {
    c_re = ar / l173_im;
    l105 = l159_im / l173_im;
  }
  ar = -(A_init + J_min * c_re);
  l159_im = -(J_min * l105);
  if (l159_im == 0.0) {
    t[3].re = ar / J_max;
    t[3].im = 0.0;
  } else if (ar == 0.0) {
    t[3].re = 0.0;
    t[3].im = l159_im / J_max;
  } else {
    t[3].re = ar / J_max;
    t[3].im = l159_im / J_max;
  }
  t[7].re = l11_tmp;
  t[7].im = c_im;
  t[11].re = c_re;
  t[11].im = l105;
  t[15] = b_l7[3];
  ar = A_wayp - re;
  if (0.0 - im == 0.0) {
    t[19].re = ar / J_min;
    t[19].im = 0.0;
  } else if (ar == 0.0) {
    t[19].re = 0.0;
    t[19].im = (0.0 - im) / J_min;
  } else {
    t[19].re = ar / J_min;
    t[19].im = (0.0 - im) / J_min;
  }
  t[23] = l19[3];
  t[27] = t7[3];
}

// End of code generation (abcdefg_TA_AVP.cpp)
