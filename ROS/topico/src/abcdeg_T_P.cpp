//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// abcdeg_T_P.cpp
//
// Code generation for function 'abcdeg_T_P'
//

// Include files
#include "abcdeg_T_P.h"
#include "power.h"
#include "rt_nonfinite.h"
#include "sqrt.h"
#include "topico_wrapper_data.h"
#include "topico_wrapper_rtwutil.h"
#include "topico_wrapper_types.h"
#include <cmath>

// Function Definitions
void abcdeg_T_P(double P_init, double V_init, double A_init, double P_wayp,
                double V_max, double V_min, double A_max, double J_max,
                double J_min, double T, creal_T t[21])
{
  creal_T t7[3];
  creal_T x[3];
  creal_T b_l4;
  creal_T dc;
  creal_T l42;
  creal_T l43;
  creal_T l44;
  creal_T l58;
  creal_T l59;
  double A_max_im;
  double A_max_im_tmp;
  double A_max_re;
  double A_max_re_tmp;
  double A_max_re_tmp_tmp;
  double ar_tmp;
  double b_A_max_im;
  double b_A_max_re;
  double b_A_max_re_tmp_tmp;
  double b_ar_tmp;
  double b_d;
  double b_im;
  double b_l157_tmp;
  double b_l7_re_tmp;
  double b_re;
  double b_re_tmp;
  double bim;
  double brm;
  double c_A_max_im;
  double c_A_max_re;
  double c_A_max_re_tmp_tmp;
  double c_l7_re_tmp;
  double d;
  double d_l7_re_tmp;
  double e_l7_re_tmp;
  double f_l7_re_tmp;
  double g_l7_re_tmp;
  double im;
  double l10;
  double l11_tmp;
  double l12;
  double l13_tmp;
  double l14;
  double l15;
  double l153;
  double l153_tmp;
  double l154;
  double l157;
  double l157_tmp;
  double l158_im;
  double l158_re;
  double l163;
  double l17;
  double l18;
  double l28_tmp;
  double l2_tmp;
  double l3;
  double l31_tmp;
  double l32;
  double l32_tmp;
  double l34_tmp;
  double l4;
  double l46_tmp;
  double l56_tmp;
  double l57_tmp;
  double l58_tmp;
  double l5_tmp;
  double l6;
  double l7_re_tmp;
  double l7_tmp;
  double l8_tmp;
  double l9;
  double re;
  double re_tmp;
  double re_tmp_tmp;
  double s;
  double sgnbi;
  double sgnbr;
  bool p;
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
  //  Generated on 03-Sep-2019 14:20:07
  l2_tmp = A_init * A_init;
  l3 = rt_powd_snf(A_init, 3.0);
  l5_tmp = A_max * A_max;
  l8_tmp = J_min * J_min;
  l9 = rt_powd_snf(J_min, 3.0);
  l11_tmp = J_max * J_max;
  l12 = rt_powd_snf(J_min, 5.0);
  l13_tmp = rt_powd_snf(J_max, 3.0);
  l15 = rt_powd_snf(J_max, 5.0);
  l17 = V_init * V_init;
  l18 = V_max * V_max;
  l31_tmp = rt_powd_snf(J_max, 1.5);
  if (J_max < 0.0) {
    k_rtErrorWithMessageID(o_emlrtRTEI.fName, o_emlrtRTEI.lineNo);
  }
  l32_tmp = rt_powd_snf(J_max, 2.5);
  l4 = l2_tmp * l2_tmp;
  l7_tmp = l5_tmp * l5_tmp;
  l10 = l8_tmp * l8_tmp;
  l14 = l11_tmp * l11_tmp;
  l28_tmp = J_max * l5_tmp;
  l34_tmp = J_min + -J_max;
  l6 = J_max + -J_min;
  l42.re = -J_min;
  l42.im = 0.0;
  coder::internal::scalar::b_sqrt(&l42);
  l43 = coder::d_power(l42);
  l44 = coder::e_power(l42);
  l46_tmp = rt_powd_snf(l34_tmp, 3.0);
  l56_tmp = rt_powd_snf(l6, 1.5);
  if (l6 < 0.0) {
    k_rtErrorWithMessageID(o_emlrtRTEI.fName, o_emlrtRTEI.lineNo);
  }
  l57_tmp = rt_powd_snf(l6, 2.5);
  l58_tmp = V_max + -V_min;
  l58.re = l58_tmp;
  l58.im = 0.0;
  coder::internal::scalar::b_sqrt(&l58);
  l59 = coder::d_power(l58);
  l153_tmp = A_max * l8_tmp;
  l153 = 1.0 / (((l153_tmp * rt_powd_snf(l11_tmp, 3.0) * 4.0 +
                  -(A_max * l12 * l13_tmp * 4.0)) +
                 A_max * l10 * l14 * 12.0) +
                -(A_max * l9 * l15 * 12.0));
  l157_tmp = A_max * V_min;
  b_l157_tmp = A_max * V_max;
  l157 = ((((l157_tmp * l8_tmp * l15 * 24.0 + l157_tmp * l10 * l13_tmp * 24.0) +
            b_l157_tmp * l9 * l14 * 48.0) +
           -(b_l157_tmp * l8_tmp * l15 * 24.0)) +
          -(b_l157_tmp * l10 * l13_tmp * 24.0)) +
         -(l157_tmp * l9 * l14 * 48.0);
  l154 = l153 * l153;
  A_max_re_tmp_tmp = A_max * 1.4142135623730951 * l32_tmp;
  A_max_re_tmp = l57_tmp * (A_max_re_tmp_tmp * l44.re);
  A_max_im_tmp = l57_tmp * (A_max_re_tmp_tmp * l44.im);
  l158_re = 4.0 * (l153 * (A_max_re_tmp * l58.re - A_max_im_tmp * l58.im));
  l158_im = 4.0 * (l153 * (A_max_re_tmp * l58.im + A_max_im_tmp * l58.re));
  l163 = l5_tmp * l12 * l15 * (V_min + -V_max) * rt_powd_snf(l34_tmp, 5.0) *
             l154 * 32.0 +
         l153 * l157 / 3.0;
  b_l4.re = l6;
  b_l4.im = 0.0;
  coder::internal::scalar::b_sqrt(&b_l4);
  dc = coder::e_power(l43);
  re_tmp_tmp = rt_powd_snf(A_max, 3.0);
  re_tmp = re_tmp_tmp * 1.4142135623730951 * rt_powd_snf(l31_tmp, 5.0);
  re = re_tmp * dc.re;
  im = re_tmp * dc.im;
  re_tmp = rt_powd_snf(l56_tmp, 5.0);
  b_re_tmp = rt_powd_snf(l153, 3.0);
  b_A_max_re_tmp_tmp = A_max * 1.4142135623730951 * l31_tmp;
  A_max_re = b_A_max_re_tmp_tmp * l42.re;
  A_max_im = b_A_max_re_tmp_tmp * l42.im;
  b_A_max_re = A_max_re * l58.re - A_max_im * l58.im;
  A_max_im = A_max_re * l58.im + A_max_im * l58.re;
  if (b_l4.im == 0.0) {
    b_re = 1.0 / b_l4.re;
    b_im = 0.0;
  } else if (b_l4.re == 0.0) {
    b_re = 0.0;
    b_im = -(1.0 / b_l4.im);
  } else {
    brm = std::abs(b_l4.re);
    bim = std::abs(b_l4.im);
    if (brm > bim) {
      s = b_l4.im / b_l4.re;
      d = b_l4.re + s * b_l4.im;
      b_re = (s * 0.0 + 1.0) / d;
      b_im = (0.0 - s) / d;
    } else if (bim == brm) {
      if (b_l4.re > 0.0) {
        sgnbr = 0.5;
      } else {
        sgnbr = -0.5;
      }
      if (b_l4.im > 0.0) {
        sgnbi = 0.5;
      } else {
        sgnbi = -0.5;
      }
      b_re = (sgnbr + 0.0 * sgnbi) / brm;
      b_im = (0.0 * sgnbr - sgnbi) / brm;
    } else {
      s = b_l4.re / b_l4.im;
      d = b_l4.im + s * b_l4.re;
      b_re = s / d;
      b_im = (s * 0.0 - 1.0) / d;
    }
  }
  c_A_max_re_tmp_tmp = rt_powd_snf(J_max, 3.5);
  l6 = A_max * 1.4142135623730951 * c_A_max_re_tmp_tmp;
  A_max_re = l56_tmp * (l6 * l43.re);
  b_A_max_im = l56_tmp * (l6 * l43.im);
  l6 = b_l157_tmp * 1.4142135623730951 * l32_tmp;
  c_A_max_re = l57_tmp * (l6 * l43.re);
  c_A_max_im = l57_tmp * (l6 * l43.im);
  sgnbi = A_max * l3;
  brm = A_init * A_max;
  l6 = A_max * P_init;
  l157_tmp = A_max * P_wayp;
  b_d = J_max * V_init;
  l7_re_tmp = l8_tmp * l15;
  l32 = l10 * l13_tmp;
  b_l7_re_tmp = l11_tmp * l12;
  c_l7_re_tmp = l9 * l14;
  b_l157_tmp = brm * V_init;
  bim = V_max * l2_tmp;
  d_l7_re_tmp = V_init * l2_tmp;
  e_l7_re_tmp = V_init * l5_tmp;
  s = V_max * l5_tmp;
  d = l2_tmp * l5_tmp;
  f_l7_re_tmp = A_max * J_max;
  g_l7_re_tmp = J_max * V_max;
  sgnbr = 1.0 / J_min * (1.0 / J_max) * (1.0 / V_max) * (1.0 / l46_tmp);
  ar_tmp = l6 * l8_tmp;
  b_ar_tmp = l157_tmp * l8_tmp;
    l157_tmp = sgnbr * (((((((((((((((((((((((((((((((((((((((((((((((((((((((l7_tmp * l12 + l7_tmp * l15) + -(l4 * l12 * 3.0)) + sgnbi * l12 * 8.0) + J_max * l4 * l10 * 9.0) + -(J_min * l7_tmp * l14 * 3.0)) + -(J_max * l7_tmp * l10 * 3.0)) + -(brm * J_max * V_init * l12 * 24.0)) + l6 * l11_tmp * l12 * 24.0) + b_ar_tmp * l15 * 24.0) + b_d * l2_tmp * l12 * 12.0) + V_init * l12 * l28_tmp * 12.0) + J_min * V_max * l5_tmp * l15 * 12.0) + l4 * l8_tmp * l13_tmp * 3.0) + l7_tmp * l8_tmp * l13_tmp * 2.0) + l7_tmp * l9 * l11_tmp * 2.0) + -(f_l7_re_tmp * l3 * l10 * 24.0)) + -(ar_tmp * l15 * 24.0)) + l6 * l9 * l14 * 72.0) + -(l157_tmp * l11_tmp * l12 * 24.0)) + l157_tmp * l10 * l13_tmp * 72.0) + -(g_l7_re_tmp * l2_tmp * l12 * 12.0)) + -(V_max * l12 * l28_tmp * 12.0)) + -(d * l12 * 6.0)) + l7_re_tmp * l17 * 12.0) + l32 * l17 * 36.0) + l7_re_tmp * l18 * 12.0) + l32 * l18 * 36.0) + b_l157_tmp * l8_tmp * l14 * 24.0) + -(l6 * l10 * l13_tmp * 72.0)) + -(l157_tmp * l9 * l14 * 72.0)) + -(l4 * l9 * l11_tmp * 9.0)) + -(b_l7_re_tmp * l17 * 12.0)) + -(c_l7_re_tmp * l17 * 36.0)) + -(b_l7_re_tmp * l18 * 12.0)) + -(c_l7_re_tmp * l18 * 36.0)) + b_l157_tmp * l10 * l11_tmp * 72.0) + l2_tmp * l10 * l28_tmp * 18.0) + sgnbi * l9 * l11_tmp * 24.0) + d_l7_re_tmp * l9 * l13_tmp * 36.0) + bim * l8_tmp * l14 * 12.0) + bim * l10 * l11_tmp * 36.0) + e_l7_re_tmp * l9 * l13_tmp * 36.0) + s * l10 * l11_tmp * 24.0) + -(b_l157_tmp * l9 * l13_tmp * 72.0)) + -(sgnbi * l8_tmp * l13_tmp * 8.0)) + -(d_l7_re_tmp * l8_tmp * l14 * 12.0)) + -(d_l7_re_tmp * l10 * l11_tmp * 36.0)) + -(e_l7_re_tmp * l8_tmp * l14 * 12.0)) + -(bim * l9 * l13_tmp * 36.0)) + -(e_l7_re_tmp * l10 * l11_tmp * 36.0)) + -(s * l8_tmp * l14 * 24.0)) + d * l8_tmp * l13_tmp * 6.0) + -(d * l9 * l11_tmp * 18.0)) + (A_max_re * l59.re - b_A_max_im * l59.im) * 8.0) + -((c_A_max_re * l58.re - c_A_max_im * l58.im) * 24.0));
    l6 = sgnbr * ((A_max_re * l59.im + b_A_max_im * l59.re) * 8.0 +
                  -((c_A_max_re * l58.im + c_A_max_im * l58.re) * 24.0));
    if (l6 == 0.0) {
      sgnbr = l157_tmp / 12.0;
      l157_tmp = 0.0;
    } else if (l157_tmp == 0.0) {
      sgnbr = 0.0;
      l157_tmp = l6 / 12.0;
    } else {
      sgnbr = l157_tmp / 12.0;
      l157_tmp = l6 / 12.0;
    }
    l10 = J_min * J_max;
    l15 = l10 * V_init;
    l18 = l15 * 2.0;
    l4 = A_max * J_min * J_max;
    l9 = l10 * V_max;
    l6 = l9 * l46_tmp * l153;
    l42.re =
        (b_re_tmp * (re_tmp * (re * l59.re - im * l59.im)) * 128.0 +
         -(l157 * (l154 * (A_max_re_tmp * l58.re - A_max_im_tmp * l58.im)) *
           2.0)) +
        l6 *
            (((((brm * J_min * 2.0 + l18) + l28_tmp) + l4 * T * 2.0) +
              (b_A_max_re * b_re - A_max_im * b_im) * 2.0) +
             sgnbr) *
            6.0;
    l42.im =
        (b_re_tmp * (re_tmp * (re * l59.im + im * l59.re)) * 128.0 +
         -(l157 * (l154 * (A_max_re_tmp * l58.im + A_max_im_tmp * l58.re)) *
           2.0)) +
        l6 * ((b_A_max_re * b_im + A_max_im * b_re) * 2.0 + l157_tmp) * 6.0;
    l6 = l42.re * l42.im;
    b_l4.re = rt_powd_snf(l163, 3.0) + (l42.re * l42.re - l42.im * l42.im);
    b_l4.im = l6 + l6;
    coder::internal::scalar::b_sqrt(&b_l4);
    l43.re = l42.re + b_l4.re;
    l43.im = l42.im + b_l4.im;
    l42 = coder::power(l43);
    if (l42.im == 0.0) {
      l6 = l42.re / 2.0;
      l157_tmp = 0.0;
      re = 1.0 / l42.re;
      im = 0.0;
    } else if (l42.re == 0.0) {
      l6 = 0.0;
      l157_tmp = l42.im / 2.0;
      re = 0.0;
      im = -(1.0 / l42.im);
    } else {
      l6 = l42.re / 2.0;
      l157_tmp = l42.im / 2.0;
      brm = std::abs(l42.re);
      bim = std::abs(l42.im);
      if (brm > bim) {
        s = l42.im / l42.re;
        d = l42.re + s * l42.im;
        re = (s * 0.0 + 1.0) / d;
        im = (0.0 - s) / d;
      } else if (bim == brm) {
        if (l42.re > 0.0) {
          sgnbr = 0.5;
        } else {
          sgnbr = -0.5;
        }
        if (l42.im > 0.0) {
          sgnbi = 0.5;
        } else {
          sgnbi = -0.5;
        }
        re = (sgnbr + 0.0 * sgnbi) / brm;
        im = (0.0 * sgnbr - sgnbi) / brm;
      } else {
        s = l42.re / l42.im;
        d = l42.im + s * l42.re;
        re = s / d;
        im = (s * 0.0 - 1.0) / d;
      }
    }
    l44.re = l163 * re;
    l44.im = l163 * im;
    if (l44.im == 0.0) {
      l58.re = l44.re / 2.0;
      l58.im = 0.0;
    } else if (l44.re == 0.0) {
      l58.re = 0.0;
      l58.im = l44.im / 2.0;
    } else {
      l58.re = l44.re / 2.0;
      l58.im = l44.im / 2.0;
    }
    re = 1.7320508075688772 * (l42.re + l44.re);
    im = 1.7320508075688772 * (l42.im + l44.im);
    l59.re = re * 0.0 - im * 0.5;
    l59.im = re * 0.5 + im * 0.0;
    t7[0].re = (l158_re + l42.re) - l44.re;
    t7[0].im = (l158_im + l42.im) - l44.im;
    sgnbi = (l158_re + -l6) + l58.re;
    t7[1].re = sgnbi - l59.re;
    brm = (l158_im + -l157_tmp) + l58.im;
    t7[1].im = brm - l59.im;
    t7[2].re = sgnbi + l59.re;
    t7[2].im = brm + l59.im;
    sgnbr = J_min * l2_tmp;
    l12 = J_min * l5_tmp;
    l17 = A_init + -A_max;
    l42.re = -J_min;
    l42.im = 0.0;
    coder::internal::scalar::b_sqrt(&l42);
    l43 = coder::d_power(l42);
    l44.re = l58_tmp;
    l44.im = 0.0;
    coder::internal::scalar::b_sqrt(&l44);
    l32 = (((l9 * 2.0 + sgnbr) + l28_tmp) + -l18) + l5_tmp * -J_min;
    coder::c_power(t7, x);
    l158_re =
        (((((((((((((l46_tmp * (l32 * l32) * 3.0 +
                     l7_tmp * l11_tmp * l46_tmp * 8.0) +
                    sgnbr * l28_tmp * l46_tmp * 12.0) -
                   sgnbr * l46_tmp * l32 * 6.0) +
                  l12 * l46_tmp * l32 * 6.0) -
                 l28_tmp * l46_tmp * l32 * 12.0) -
                V_init * l11_tmp * l12 * l46_tmp * 24.0) +
               rt_powd_snf(A_init, 3.0) * A_max * l8_tmp * l46_tmp * 12.0) -
              l2_tmp * l5_tmp * l8_tmp * l46_tmp * 24.0) -
             l153_tmp * rt_powd_snf(l17, 3.0) * l46_tmp * 4.0) -
            l10 * l7_tmp * l46_tmp * 12.0) +
           A_init * re_tmp_tmp * l8_tmp * l46_tmp * 12.0) +
          l15 * l46_tmp * l32 * 12.0) +
         ar_tmp * l11_tmp * l46_tmp * 24.0) -
        b_ar_tmp * l11_tmp * l46_tmp * 24.0;
    d_l7_re_tmp = J_min * re_tmp_tmp * l11_tmp * l46_tmp;
    e_l7_re_tmp = J_max * re_tmp_tmp * l8_tmp * l46_tmp;
    b_l7_re_tmp = l153_tmp * l13_tmp;
    c_l7_re_tmp = b_l7_re_tmp * l46_tmp;
    l6 = re_tmp_tmp * 1.4142135623730951 * l31_tmp;
    l157_tmp = l57_tmp * (l6 * l43.re);
    l7_re_tmp = l57_tmp * (l6 * l43.im);
    b_l4.re = 12.0 * (l157_tmp * l44.re - l7_re_tmp * l44.im);
    b_l4.im = 12.0 * (l157_tmp * l44.im + l7_re_tmp * l44.re);
    l6 = re_tmp_tmp * 1.4142135623730951 * l32_tmp;
    l157_tmp = l57_tmp * (l6 * l42.re);
    l7_re_tmp = l57_tmp * (l6 * l42.im);
    l15 = 12.0 * (l157_tmp * l44.re - l7_re_tmp * l44.im);
    l7_re_tmp = 12.0 * (l157_tmp * l44.im + l7_re_tmp * l44.re);
    brm = b_l7_re_tmp * (l34_tmp * l34_tmp);
    l9 = V_min - V_max;
    b_l7_re_tmp = l4 * l46_tmp * l32;
    A_max_re = f_l7_re_tmp * V_init * l8_tmp * l17 * l46_tmp * 24.0;
    sgnbi = f_l7_re_tmp * l2_tmp * l8_tmp * l46_tmp;
    l6 = A_max * V_init;
    d = l6 * l8_tmp * l11_tmp * l46_tmp;
    dc = coder::d_power(l44);
    A_max_re_tmp = A_max * c_A_max_re_tmp_tmp * 1.4142135623730951 * l56_tmp;
    b_A_max_re = A_max_re_tmp * l43.re;
    A_max_im = A_max_re_tmp * l43.im;
    c_A_max_re = 8.0 * (b_A_max_re * dc.re - A_max_im * dc.im);
    A_max_im = 8.0 * (b_A_max_re * dc.im + A_max_im * dc.re);
    dc = coder::e_power(l42);
    b_A_max_re = l57_tmp * (A_max_re_tmp_tmp * dc.re);
    b_A_max_im = l57_tmp * (A_max_re_tmp_tmp * dc.im);
    s = b_A_max_re * l44.re - b_A_max_im * l44.im;
    b_A_max_im = b_A_max_re * l44.im + b_A_max_im * l44.re;
    A_max_re_tmp = l6 * 1.4142135623730951 * l32_tmp;
    b_A_max_re = l57_tmp * (A_max_re_tmp * l43.re);
    c_A_max_im = l57_tmp * (A_max_re_tmp * l43.im);
    bim = 24.0 * (b_A_max_re * l44.re - c_A_max_im * l44.im);
    c_A_max_im = 24.0 * (b_A_max_re * l44.im + c_A_max_im * l44.re);
    A_max_re_tmp = A_max * l2_tmp * 1.4142135623730951 * l31_tmp;
    b_A_max_re = l57_tmp * (A_max_re_tmp * l43.re);
    l6 = l57_tmp * (A_max_re_tmp * l43.im);
    l59.re = 12.0 * (b_A_max_re * l44.re - l6 * l44.im);
    l59.im = 12.0 * (b_A_max_re * l44.im + l6 * l44.re);
    b_A_max_re = l57_tmp * (b_A_max_re_tmp_tmp * l42.re);
    l6 = l57_tmp * (b_A_max_re_tmp_tmp * l42.im);
    l58.re = 12.0 * (l32 * (b_A_max_re * l44.re - l6 * l44.im));
    l58.im = 12.0 * (l32 * (b_A_max_re * l44.im + l6 * l44.re));
    b_l157_tmp =
        (((d_l7_re_tmp * -12.0 + e_l7_re_tmp * 12.0) + b_l7_re_tmp * 12.0) -
         sgnbi * 12.0) +
        d * 24.0;
    l32 = 1.0 / A_max * (1.0 / J_max) *
          ((((l2_tmp + g_l7_re_tmp * 2.0) + -(b_d * 2.0)) + -l5_tmp) +
           l28_tmp * (1.0 / J_min)) /
          2.0;
    l157_tmp = t7[0].re * t7[0].re - t7[0].im * t7[0].im;
    l6 = t7[0].re * t7[0].im;
    l6 += l6;
    x[0].re = -(((((((((((((((l158_re - d_l7_re_tmp * t7[0].re * 12.0) +
                             e_l7_re_tmp * t7[0].re * 12.0) +
                            c_l7_re_tmp * x[0].re * 4.0) -
                           b_l4.re) -
                          l15) -
                         l9 * (brm * t7[0].re) * 24.0) +
                        b_l7_re_tmp * t7[0].re * 12.0) -
                       A_max_re) -
                      sgnbi * t7[0].re * 12.0) +
                     d * t7[0].re * 24.0) +
                    c_A_max_re) +
                   (s * l157_tmp - b_A_max_im * l6) * 12.0) -
                  bim) +
                 l59.re) +
                l58.re);
    x[0].im = -((((((((((((((0.0 - d_l7_re_tmp * t7[0].im * 12.0) +
                            e_l7_re_tmp * t7[0].im * 12.0) +
                           c_l7_re_tmp * x[0].im * 4.0) -
                          b_l4.im) -
                         l7_re_tmp) -
                        l9 * (brm * t7[0].im) * 24.0) +
                       b_l7_re_tmp * t7[0].im * 12.0) -
                      sgnbi * t7[0].im * 12.0) +
                     d * t7[0].im * 24.0) +
                    A_max_im) +
                   (s * l6 + b_A_max_im * l157_tmp) * 12.0) -
                  c_A_max_im) +
                 l59.im) +
                l58.im);
    sgnbr = (J_min - J_max) *
            (((((l12 - l28_tmp) - sgnbr) + l18) - l10 * V_min * 2.0) +
             l4 * l32 * 2.0);
    p = (sgnbr < 0.0);
    l157_tmp = t7[1].re * t7[1].re - t7[1].im * t7[1].im;
    l6 = t7[1].re * t7[1].im;
    l6 += l6;
    x[1].re = -(((((((((((((((l158_re - d_l7_re_tmp * t7[1].re * 12.0) +
                             e_l7_re_tmp * t7[1].re * 12.0) +
                            c_l7_re_tmp * x[1].re * 4.0) -
                           b_l4.re) -
                          l15) -
                         l9 * (brm * t7[1].re) * 24.0) +
                        b_l7_re_tmp * t7[1].re * 12.0) -
                       A_max_re) -
                      sgnbi * t7[1].re * 12.0) +
                     d * t7[1].re * 24.0) +
                    c_A_max_re) +
                   (s * l157_tmp - b_A_max_im * l6) * 12.0) -
                  bim) +
                 l59.re) +
                l58.re);
    x[1].im = -((((((((((((((0.0 - d_l7_re_tmp * t7[1].im * 12.0) +
                            e_l7_re_tmp * t7[1].im * 12.0) +
                           c_l7_re_tmp * x[1].im * 4.0) -
                          b_l4.im) -
                         l7_re_tmp) -
                        l9 * (brm * t7[1].im) * 24.0) +
                       b_l7_re_tmp * t7[1].im * 12.0) -
                      sgnbi * t7[1].im * 12.0) +
                     d * t7[1].im * 24.0) +
                    A_max_im) +
                   (s * l6 + b_A_max_im * l157_tmp) * 12.0) -
                  c_A_max_im) +
                 l59.im) +
                l58.im);
    if (p || (sgnbr < 0.0)) {
      p = true;
    }
    l157_tmp = t7[2].re * t7[2].re - t7[2].im * t7[2].im;
    l6 = t7[2].re * t7[2].im;
    l6 += l6;
    x[2].re = -(((((((((((((((l158_re - d_l7_re_tmp * t7[2].re * 12.0) +
                             e_l7_re_tmp * t7[2].re * 12.0) +
                            c_l7_re_tmp * x[2].re * 4.0) -
                           b_l4.re) -
                          l15) -
                         l9 * (brm * t7[2].re) * 24.0) +
                        b_l7_re_tmp * t7[2].re * 12.0) -
                       A_max_re) -
                      sgnbi * t7[2].re * 12.0) +
                     d * t7[2].re * 24.0) +
                    c_A_max_re) +
                   (s * l157_tmp - b_A_max_im * l6) * 12.0) -
                  bim) +
                 l59.re) +
                l58.re);
    x[2].im = -((((((((((((((0.0 - d_l7_re_tmp * t7[2].im * 12.0) +
                            e_l7_re_tmp * t7[2].im * 12.0) +
                           c_l7_re_tmp * x[2].im * 4.0) -
                          b_l4.im) -
                         l7_re_tmp) -
                        l9 * (brm * t7[2].im) * 24.0) +
                       b_l7_re_tmp * t7[2].im * 12.0) -
                      sgnbi * t7[2].im * 12.0) +
                     d * t7[2].im * 24.0) +
                    A_max_im) +
                   (s * l6 + b_A_max_im * l157_tmp) * 12.0) -
                  c_A_max_im) +
                 l59.im) +
                l58.im);
    if (p || (sgnbr < 0.0)) {
      p = true;
    }
    if (p) {
      f_rtErrorWithMessageID("sqrt", n_emlrtRTEI.fName, n_emlrtRTEI.lineNo);
    }
    l3 = A_max * (1.0 / J_min);
    l6 = -(1.0 / J_max * l17);
    l157_tmp = l8_tmp - l10;
    t[0].re = l6;
    t[0].im = 0.0;
    t[1].re = l6;
    t[1].im = 0.0;
    t[2].re = l6;
    t[2].im = 0.0;
    t[6].re = -l3;
    t[6].im = 0.0;
    t[7].re = -l3;
    t[7].im = 0.0;
    t[8].re = -l3;
    t[8].im = 0.0;
    t[3].re = l32;
    t[3].im = 0.0;
    if (x[0].im == 0.0) {
      t[9].re = x[0].re / b_l157_tmp;
      t[9].im = 0.0;
    } else if (x[0].re == 0.0) {
      t[9].re = 0.0;
      t[9].im = x[0].im / b_l157_tmp;
    } else {
      t[9].re = x[0].re / b_l157_tmp;
      t[9].im = x[0].im / b_l157_tmp;
    }
    t[12].re = std::sqrt(sgnbr) / l157_tmp;
    t[12].im = 0.0;
    t[15].re = 0.0;
    t[15].im = 0.0;
    t[18] = t7[0];
    t[4].re = l32;
    t[4].im = 0.0;
    if (x[1].im == 0.0) {
      t[10].re = x[1].re / b_l157_tmp;
      t[10].im = 0.0;
    } else if (x[1].re == 0.0) {
      t[10].re = 0.0;
      t[10].im = x[1].im / b_l157_tmp;
    } else {
      t[10].re = x[1].re / b_l157_tmp;
      t[10].im = x[1].im / b_l157_tmp;
    }
    t[13].re = std::sqrt(sgnbr) / l157_tmp;
    t[13].im = 0.0;
    t[16].re = 0.0;
    t[16].im = 0.0;
    t[19] = t7[1];
    t[5].re = l32;
    t[5].im = 0.0;
    if (x[2].im == 0.0) {
      t[11].re = x[2].re / b_l157_tmp;
      t[11].im = 0.0;
    } else if (x[2].re == 0.0) {
      t[11].re = 0.0;
      t[11].im = x[2].im / b_l157_tmp;
    } else {
      t[11].re = x[2].re / b_l157_tmp;
      t[11].im = x[2].im / b_l157_tmp;
    }
    t[14].re = std::sqrt(sgnbr) / l157_tmp;
    t[14].im = 0.0;
    t[17].re = 0.0;
    t[17].im = 0.0;
    t[20] = t7[2];
}

// End of code generation (abcdeg_T_P.cpp)
