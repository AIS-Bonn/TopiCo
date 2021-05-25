//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// acdefg_T_P.cpp
//
// Code generation for function 'acdefg_T_P'
//

// Include files
#include "acdefg_T_P.h"
#include "power.h"
#include "rt_nonfinite.h"
#include "sqrt.h"
#include "topico_wrapper_data.h"
#include "topico_wrapper_rtwutil.h"
#include "topico_wrapper_types.h"
#include <cmath>

// Function Definitions
void acdefg_T_P(double P_init, double V_init, double A_init, double P_wayp,
                double V_max, double V_min, double A_min, double J_max,
                double J_min, double T, creal_T t[21])
{
  creal_T t3[3];
  creal_T t4[3];
  creal_T x[3];
  creal_T dc;
  creal_T l134;
  creal_T l213;
  creal_T l281;
  creal_T l292;
  creal_T l34;
  creal_T l35;
  creal_T l36;
  creal_T l37;
  creal_T l41;
  creal_T l42;
  creal_T l43;
  creal_T l44;
  creal_T l45;
  creal_T l46;
  creal_T l53;
  double A_init_im;
  double A_init_im_tmp;
  double A_init_re;
  double A_init_re_tmp;
  double A_init_re_tmp_tmp;
  double A_min_im;
  double A_min_re;
  double A_min_re_tmp;
  double A_min_re_tmp_tmp;
  double J_max_im;
  double J_max_re;
  double J_max_re_tmp;
  double J_max_re_tmp_tmp;
  double J_min_im;
  double J_min_re;
  double V_init_im;
  double V_init_re;
  double V_init_re_tmp;
  double V_min_im;
  double V_min_re;
  double V_min_re_tmp;
  double V_min_re_tmp_tmp;
  double a;
  double ar;
  double b_A_init_im;
  double b_A_init_im_tmp;
  double b_A_init_re;
  double b_A_init_re_tmp;
  double b_A_init_re_tmp_tmp;
  double b_A_min_im;
  double b_A_min_re;
  double b_A_min_re_tmp;
  double b_A_min_re_tmp_tmp;
  double b_J_max_im;
  double b_J_max_re;
  double b_J_max_re_tmp;
  double b_V_init_im;
  double b_V_init_re;
  double b_V_init_re_tmp;
  double b_V_min_im;
  double b_V_min_re;
  double b_V_min_re_tmp;
  double b_l25_tmp;
  double b_l2_im;
  double b_l2_re;
  double b_l2_re_tmp;
  double b_l47_tmp;
  double b_l4_im;
  double b_l4_re;
  double b_l4_re_tmp;
  double b_l8_re_tmp;
  double c_A_init_im;
  double c_A_init_re;
  double c_A_init_re_tmp;
  double c_A_min_im;
  double c_A_min_re;
  double c_A_min_re_tmp;
  double c_J_max_im;
  double c_J_max_re;
  double c_J_max_re_tmp;
  double c_V_init_im;
  double c_V_init_re;
  double c_V_init_re_tmp;
  double c_V_min_im;
  double c_V_min_re;
  double c_V_min_re_tmp;
  double c_l2_im;
  double c_l2_re;
  double c_l2_re_tmp;
  double c_l4_im;
  double c_l4_re;
  double c_l4_re_tmp;
  double c_l8_re_tmp;
  double d;
  double d1;
  double d2;
  double d_A_init_im;
  double d_A_init_re;
  double d_A_init_re_tmp;
  double d_A_min_im;
  double d_A_min_re;
  double d_A_min_re_tmp;
  double d_J_max_im;
  double d_J_max_re;
  double d_J_max_re_tmp;
  double d_l4_im;
  double d_l4_re;
  double e_A_init_im;
  double e_A_init_re;
  double e_A_min_im;
  double e_A_min_re;
  double e_A_min_re_tmp;
  double f_A_init_re;
  double f_A_min_re_tmp;
  double im;
  double l10_tmp;
  double l11;
  double l11_im;
  double l11_re;
  double l11_re_tmp;
  double l12;
  double l129;
  double l129_tmp;
  double l129_tmp_tmp;
  double l13;
  double l135;
  double l14;
  double l15;
  double l151_im;
  double l151_re;
  double l16;
  double l20;
  double l205;
  double l205_tmp;
  double l206;
  double l206_im;
  double l206_re;
  double l21;
  double l213_tmp;
  double l213_tmp_tmp;
  double l24;
  double l25_tmp;
  double l281_tmp;
  double l281_tmp_tmp;
  double l284_im;
  double l284_re;
  double l287_im;
  double l287_re;
  double l28_tmp;
  double l2_im;
  double l2_re;
  double l2_re_tmp;
  double l2_tmp;
  double l3;
  double l31;
  double l38;
  double l39;
  double l42_tmp;
  double l47_tmp;
  double l4_im;
  double l4_re;
  double l4_re_tmp;
  double l4_re_tmp_tmp;
  double l4_tmp;
  double l5;
  double l6;
  double l7_im;
  double l7_re;
  double l7_re_tmp;
  double l7_re_tmp_tmp;
  double l7_tmp;
  double l8;
  double l8_im;
  double l8_re;
  double l8_re_tmp;
  double l8_re_tmp_tmp;
  double l9;
  double l98;
  double l9_im;
  double l9_re;
  double re;
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
  //  Generated on 03-Sep-2019 16:16:41
  l2_tmp = A_init * A_init;
  l3 = rt_powd_snf(A_init, 3.0);
  l4_tmp = A_min * A_min;
  l5 = rt_powd_snf(A_min, 3.0);
  l7_tmp = J_min * J_min;
  l8 = rt_powd_snf(J_min, 3.0);
  l10_tmp = J_max * J_max;
  l11 = rt_powd_snf(J_min, 5.0);
  l12 = rt_powd_snf(J_max, 3.0);
  l15 = rt_powd_snf(J_min, 7.0);
  l16 = rt_powd_snf(J_max, 5.0);
  l206 = J_max * V_max * 2.0;
  l20 = 1.0 / A_min;
  l25_tmp = J_min * J_max;
  b_l25_tmp = l25_tmp * V_min * 2.0;
  l6 = l4_tmp * l4_tmp;
  l9 = l7_tmp * l7_tmp;
  l13 = rt_powd_snf(l7_tmp, 3.0);
  l14 = l10_tmp * l10_tmp;
  l21 = 1.0 / l4_tmp;
  l28_tmp = J_max * l4_tmp;
  l31 = J_min + -J_max;
  l34.re = -J_min;
  l34.im = 0.0;
  coder::internal::scalar::b_sqrt(&l34);
  l35 = coder::d_power(l34);
  l36 = coder::e_power(l34);
  l37 = coder::f_power(l34);
  l38 = l31 * l31;
  l42_tmp = J_max + -J_min;
  l42.re = l42_tmp;
  l42.im = 0.0;
  coder::internal::scalar::b_sqrt(&l42);
  l47_tmp = J_max * V_init;
  b_l47_tmp = (l2_tmp + l206) + -(l47_tmp * 2.0);
  l129_tmp_tmp = J_min * l4_tmp;
  l24 = b_l25_tmp + l129_tmp_tmp;
  l129_tmp = l25_tmp * V_max;
  l129 = (l24 + l129_tmp * -2.0) + l4_tmp * -J_max;
  l39 = l38 * l38;
  l41 = coder::d_power(l35);
  l43 = coder::d_power(l42);
  l44 = coder::f_power(l42);
  l46.re = A_init * l42.re;
  l46.im = A_init * l42.im;
  l53.re = b_l47_tmp;
  l53.im = 0.0;
  coder::internal::scalar::b_sqrt(&l53);
  a = -J_min * b_l47_tmp;
  l135 = l129 * l129;
  l45 = coder::d_power(l43);
  l98 = rt_powd_snf(a, 1.5);
  if (a < 0.0) {
    k_rtErrorWithMessageID(o_emlrtRTEI.fName, o_emlrtRTEI.lineNo);
  }
  l134.re = l34.re * l53.re - l34.im * l53.im;
  l134.im = l34.re * l53.im + l34.im * l53.re;
  l151_re = l46.re + -l134.re;
  l151_im = l46.im + -l134.im;
  l205_tmp = A_min * l9;
  l205 = 1.0 / (((l205_tmp * rt_powd_snf(l10_tmp, 3.0) * l38 * 4.0 +
                  -(A_min * l12 * l15 * l38 * 4.0)) +
                 A_min * l13 * l14 * l38 * 12.0) +
                -(A_min * l11 * l16 * l38 * 12.0));
  A_min_re_tmp = A_min * J_min;
  A_min_re_tmp_tmp = A_min_re_tmp * J_max;
  b_A_min_re_tmp = A_min_re_tmp_tmp * T;
  A_min_re = A_min * l35.re;
  A_min_im = A_min * l35.im;
  c_A_min_re_tmp = A_min * J_max;
  l213_tmp_tmp = J_min * l206;
  l213_tmp = l24 + l28_tmp;
  l213.re = ((((A_min_re_tmp * l46.re * 2.0 + l213_tmp_tmp * l42.re) +
               b_A_min_re_tmp * l42.re * 2.0) +
              (A_min_re * l53.re - A_min_im * l53.im) * 2.0) +
             -(l42.re * l213_tmp)) +
            c_A_min_re_tmp * l134.re * 2.0;
  l213.im = ((((A_min_re_tmp * l46.im * 2.0 + l213_tmp_tmp * l42.im) +
               b_A_min_re_tmp * l42.im * 2.0) +
              (A_min_re * l53.im + A_min_im * l53.re) * 2.0) +
             -(l42.im * l213_tmp)) +
            c_A_min_re_tmp * l134.im * 2.0;
  l34.re = l151_re * l151_re - l151_im * l151_im;
  l287_im = l151_re * l151_im;
  l34.im = l287_im + l287_im;
  l206 = l205 * l205;
  l134.re = l213.re * l213.re - l213.im * l213.im;
  l287_re = l213.re * l213.im;
  l134.im = l287_re + l287_re;
  l35 = coder::d_power(l213);
  l11_re_tmp = l11 * l31;
  l46.re = 3.0 * (l11_re_tmp * l134.re);
  l46.im = 3.0 * (l11_re_tmp * l134.im);
  l8_re_tmp = l8 * l16;
  l8_re = l8_re_tmp * l43.re;
  l8_im = l8_re_tmp * l43.im;
  a = l10_tmp * l13;
  l24 = a * l43.re;
  a *= l43.im;
  l11_re_tmp = l11 * l12;
  l11_re = l11_re_tmp * l43.re;
  l11_im = l11_re_tmp * l43.im;
  l213_tmp = l9 * l14;
  l9_re = l213_tmp * l43.re;
  l9_im = l213_tmp * l43.im;
  l281_tmp_tmp = l4_tmp * l10_tmp;
  l281_tmp = l4_tmp * l9;
  l281.re =
      ((((((l281_tmp * l16 * l38 * 12.0 + l4_tmp * l12 * l13 * l38 * 36.0) +
           -(l281_tmp_tmp * l15 * l38 * 12.0)) +
          -(l4_tmp * l11 * l14 * l38 * 36.0)) +
         (l8_re * l213.re - l8_im * l213.im) * 6.0) +
        -((l24 * l213.re - a * l213.im) * 6.0)) +
       (l11_re * l213.re - l11_im * l213.im) * 18.0) +
      -((l9_re * l213.re - l9_im * l213.im) * 18.0);
  l281.im = (((l8_re * l213.im + l8_im * l213.re) * 6.0 +
              -((l24 * l213.im + a * l213.re) * 6.0)) +
             (l11_re * l213.im + l11_im * l213.re) * 18.0) +
            -((l9_re * l213.im + l9_im * l213.re) * 18.0);
  ar = l205 * l281.re;
  l213_tmp = l205 * l281.im;
  if (l213_tmp == 0.0) {
    l284_re = ar / 3.0;
    l284_im = 0.0;
  } else if (ar == 0.0) {
    l284_re = 0.0;
    l284_im = l213_tmp / 3.0;
  } else {
    l284_re = ar / 3.0;
    l284_im = l213_tmp / 3.0;
  }
  d_A_min_re_tmp = c_A_min_re_tmp * l13;
  A_min_re = d_A_min_re_tmp * l43.re;
  A_min_im = d_A_min_re_tmp * l43.im;
  b_A_min_re_tmp_tmp = A_min * l8;
  e_A_min_re_tmp = b_A_min_re_tmp_tmp * l14;
  b_A_min_re = e_A_min_re_tmp * l43.re;
  b_A_min_im = e_A_min_re_tmp * l43.im;
  f_A_min_re_tmp = A_min * l10_tmp * l11;
  c_A_min_re = f_A_min_re_tmp * l43.re;
  c_A_min_im = f_A_min_re_tmp * l43.im;
  l7_re_tmp = l7_tmp * l14 * l20 * l31;
  l8_re_tmp = l8 * l12 * l20 * l31;
  a = l205_tmp * l12;
  d_A_min_re = a * l43.re;
  d_A_min_im = a * l43.im;
  l213_tmp = l9 * l10_tmp * l20 * l31;
  l24 = J_max * l20;
  l287_re =
      ((((((((((((((l5 * l8 * l16 * l38 * 12.0 + l5 * l11 * l12 * l38 * 36.0) +
                   -(l5 * l10_tmp * l13 * l38 * 12.0)) +
                  -(l5 * l9 * l14 * l38 * 36.0)) +
                 -(d_A_min_re_tmp * l38 * l129 * 12.0)) +
                e_A_min_re_tmp * l38 * l129 * 12.0) +
               f_A_min_re_tmp * l38 * l129 * 36.0) +
              -(a * l38 * l129 * 36.0)) +
             l24 * l46.re) +
            -((A_min_re * l213.re - A_min_im * l213.im) * 12.0)) +
           (b_A_min_re * l213.re - b_A_min_im * l213.im) * 12.0) +
          (c_A_min_re * l213.re - c_A_min_im * l213.im) * 36.0) +
         -(l7_re_tmp * l134.re * 3.0)) +
        l8_re_tmp * l134.re * 9.0) +
       -((d_A_min_re * l213.re - d_A_min_im * l213.im) * 36.0)) +
      -(l213_tmp * l134.re * 9.0);
  l287_im = ((((((l24 * l46.im +
                  -((A_min_re * l213.im + A_min_im * l213.re) * 12.0)) +
                 (b_A_min_re * l213.im + b_A_min_im * l213.re) * 12.0) +
                (c_A_min_re * l213.im + c_A_min_im * l213.re) * 36.0) +
               -(l7_re_tmp * l134.im * 3.0)) +
              l8_re_tmp * l134.im * 9.0) +
             -((d_A_min_re * l213.im + d_A_min_im * l213.re) * 36.0)) +
            -(l213_tmp * l134.im * 9.0);
  a = l281.re * l281.im;
  ar = l206 * (l281.re * l281.re - l281.im * l281.im);
  l213_tmp = l206 * (a + a);
  if (l213_tmp == 0.0) {
    l206_re = ar / 9.0;
    l206_im = 0.0;
  } else if (ar == 0.0) {
    l206_re = 0.0;
    l206_im = l213_tmp / 9.0;
  } else {
    l206_re = ar / 9.0;
    l206_im = l213_tmp / 9.0;
  }
  ar = l205 * l287_re;
  l213_tmp = l205 * l287_im;
  if (l213_tmp == 0.0) {
    l24 = ar / 3.0;
    a = 0.0;
  } else if (ar == 0.0) {
    l24 = 0.0;
    a = l213_tmp / 3.0;
  } else {
    l24 = ar / 3.0;
    a = l213_tmp / 3.0;
  }
  l292.re = l206_re + -l24;
  l292.im = l206_im + -a;
  dc = coder::d_power(l281);
  a = rt_powd_snf(l205, 3.0);
  ar = a * dc.re;
  l213_tmp = a * dc.im;
  if (l213_tmp == 0.0) {
    re = ar / 27.0;
    im = 0.0;
  } else if (ar == 0.0) {
    re = 0.0;
    im = l213_tmp / 27.0;
  } else {
    re = ar / 27.0;
    im = l213_tmp / 27.0;
  }
  l206_re = l206 * l281.re;
  l206_im = l206 * l281.im;
  a = l206_re * l287_re - l206_im * l287_im;
  l206_im = l206_re * l287_im + l206_im * l287_re;
  if (l206_im == 0.0) {
    l206_re = a / 6.0;
    l206_im = 0.0;
  } else if (a == 0.0) {
    l206_re = 0.0;
    l206_im /= 6.0;
  } else {
    l206_re = a / 6.0;
    l206_im /= 6.0;
  }
  d_A_min_re_tmp = c_A_min_re_tmp * l8;
  e_A_min_re_tmp = A_min * l7_tmp * l10_tmp;
  f_A_min_re_tmp = c_A_min_re_tmp * V_init;
  A_min_re = f_A_min_re_tmp * l41.re;
  A_min_im = f_A_min_re_tmp * l41.im;
  b_A_min_re = A_min_re * l45.re - A_min_im * l45.im;
  A_min_im = A_min_re * l45.im + A_min_im * l45.re;
  f_A_min_re_tmp = A_min * l2_tmp;
  A_min_re = f_A_min_re_tmp * l41.re;
  b_A_min_im = f_A_min_re_tmp * l41.im;
  c_A_min_re = A_min_re * l45.re - b_A_min_im * l45.im;
  b_A_min_im = A_min_re * l45.im + b_A_min_im * l45.re;
  f_A_min_re_tmp = c_A_min_re_tmp * l2_tmp;
  A_min_re = f_A_min_re_tmp * l37.re;
  c_A_min_im = f_A_min_re_tmp * l37.im;
  d_A_min_re = A_min_re * l45.re - c_A_min_im * l45.im;
  c_A_min_im = A_min_re * l45.im + c_A_min_im * l45.re;
  f_A_min_re_tmp = A_min * V_init * l10_tmp;
  A_min_re = f_A_min_re_tmp * l37.re;
  d_A_min_im = f_A_min_re_tmp * l37.im;
  e_A_min_re = A_min_re * l45.re - d_A_min_im * l45.im;
  d_A_min_im = A_min_re * l45.im + d_A_min_im * l45.re;
  A_init_re_tmp = A_init * l28_tmp;
  A_init_re = A_init_re_tmp * l37.re;
  A_init_im = A_init_re_tmp * l37.im;
  b_A_init_re = A_init_re * l45.re - A_init_im * l45.im;
  A_init_im = A_init_re * l45.im + A_init_im * l45.re;
  A_init_re_tmp = A_init * l4_tmp * l10_tmp;
  A_init_re = A_init_re_tmp * l36.re;
  b_A_init_im = A_init_re_tmp * l36.im;
  c_A_init_re = A_init_re * l45.re - b_A_init_im * l45.im;
  b_A_init_im = A_init_re * l45.im + b_A_init_im * l45.re;
  A_init_re_tmp_tmp = A_init * A_min;
  b_A_init_re_tmp_tmp = A_init_re_tmp_tmp * l9;
  A_init_re_tmp = b_A_init_re_tmp_tmp * l39;
  l8_re_tmp = l8 * l28_tmp * l39;
  b_A_init_re_tmp = A_init * l37.re;
  A_init_im_tmp = A_init * l37.im;
  A_init_re = b_A_init_re_tmp * l45.re - A_init_im_tmp * l45.im;
  c_A_init_im = b_A_init_re_tmp * l45.im + A_init_im_tmp * l45.re;
  c_A_init_re_tmp = A_init * J_max;
  l24 = c_A_init_re_tmp * l36.re;
  b_A_init_im_tmp = c_A_init_re_tmp * l36.im;
  d_A_init_re = l24 * l45.re - b_A_init_im_tmp * l45.im;
  d_A_init_im = l24 * l45.im + b_A_init_im_tmp * l45.re;
  f_A_min_re_tmp = b_A_min_re_tmp_tmp * l10_tmp;
  A_min_re = b_l47_tmp * (f_A_min_re_tmp * l44.re);
  e_A_min_im = b_l47_tmp * (f_A_min_re_tmp * l44.im);
  b_l8_re_tmp = l8 * l39 * l129;
  d_A_init_re_tmp = A_init_re_tmp_tmp * J_max;
  a = l39 * (d_A_init_re_tmp * l37.re);
  e_A_init_im = l39 * (d_A_init_re_tmp * l37.im);
  e_A_init_re = a * l53.re - e_A_init_im * l53.im;
  e_A_init_im = a * l53.im + e_A_init_im * l53.re;
  l4_re = l39 * (l281_tmp_tmp * l36.re);
  l4_im = l39 * (l281_tmp_tmp * l36.im);
  b_l4_re = l4_re * l53.re - l4_im * l53.im;
  l4_im = l4_re * l53.im + l4_im * l53.re;
  J_max_re_tmp = J_max * l36.re;
  ar = J_max * l36.im;
  J_max_re = l39 * J_max_re_tmp;
  J_max_im = l39 * ar;
  b_J_max_re = l129 * (J_max_re * l53.re - J_max_im * l53.im);
  J_max_im = l129 * (J_max_re * l53.im + J_max_im * l53.re);
  J_max_re_tmp_tmp = J_max * l9;
  b_J_max_re_tmp = J_max_re_tmp_tmp * l31;
  l7_re_tmp_tmp = l7_tmp * l12;
  l7_re_tmp = l7_re_tmp_tmp * l31;
  l8_re_tmp_tmp = l8 * l10_tmp;
  c_l8_re_tmp = l8_re_tmp_tmp * l31;
  c_J_max_re_tmp = l47_tmp * l13;
  J_max_re = c_J_max_re_tmp * l43.re;
  b_J_max_im = c_J_max_re_tmp * l43.im;
  l2_re_tmp = l2_tmp * l13;
  l2_re = l2_re_tmp * l43.re;
  l2_im = l2_re_tmp * l43.im;
  V_init_re_tmp = V_init * l8 * l14;
  V_init_re = V_init_re_tmp * l43.re;
  V_init_im = V_init_re_tmp * l43.im;
  b_V_init_re_tmp = V_init * l10_tmp * l11;
  b_V_init_re = b_V_init_re_tmp * l43.re;
  b_V_init_im = b_V_init_re_tmp * l43.im;
  V_min_re_tmp = V_min * l9 * l12;
  V_min_re = V_min_re_tmp * l43.re;
  V_min_im = V_min_re_tmp * l43.im;
  d_J_max_re_tmp = J_max * l2_tmp * l11;
  c_J_max_re = d_J_max_re_tmp * l43.re;
  c_J_max_im = d_J_max_re_tmp * l43.im;
  l11_re_tmp = l11 * l28_tmp;
  l11_re = l11_re_tmp * l43.re;
  l11_im = l11_re_tmp * l43.im;
  c_V_init_re_tmp = V_init * l9 * l12;
  c_V_init_re = c_V_init_re_tmp * l43.re;
  c_V_init_im = c_V_init_re_tmp * l43.im;
  b_V_min_re_tmp = V_min * l8 * l14;
  b_V_min_re = b_V_min_re_tmp * l43.re;
  b_V_min_im = b_V_min_re_tmp * l43.im;
  V_min_re_tmp_tmp = V_min * l10_tmp;
  c_V_min_re_tmp = V_min_re_tmp_tmp * l11;
  c_V_min_re = c_V_min_re_tmp * l43.re;
  c_V_min_im = c_V_min_re_tmp * l43.im;
  b_l2_re_tmp = l2_tmp * l9 * l10_tmp;
  b_l2_re = b_l2_re_tmp * l43.re;
  b_l2_im = b_l2_re_tmp * l43.im;
  l4_re_tmp_tmp = l4_tmp * l7_tmp;
  l4_re_tmp = l4_re_tmp_tmp * l14;
  l4_re = l4_re_tmp * l43.re;
  b_l4_im = l4_re_tmp * l43.im;
  b_l4_re_tmp = l281_tmp * l10_tmp;
  c_l4_re = b_l4_re_tmp * l43.re;
  c_l4_im = b_l4_re_tmp * l43.im;
  c_l2_re_tmp = l2_tmp * l8 * l12;
  c_l2_re = c_l2_re_tmp * l43.re;
  c_l2_im = c_l2_re_tmp * l43.im;
  c_l4_re_tmp = l4_tmp * l8 * l12;
  d_l4_re = c_l4_re_tmp * l43.re;
  d_l4_im = c_l4_re_tmp * l43.im;
  a = J_min * l12 * l21;
  J_min_re = a * l42.re;
  J_min_im = a * l42.im;
  a = J_min_re * l35.re - J_min_im * l35.im;
  J_min_im = J_min_re * l35.im + J_min_im * l35.re;
  if (J_min_im == 0.0) {
    J_min_re = a / 2.0;
    J_min_im = 0.0;
  } else if (a == 0.0) {
    J_min_re = 0.0;
    J_min_im /= 2.0;
  } else {
    J_min_re = a / 2.0;
    J_min_im /= 2.0;
  }
  a = J_max * l8 * l21;
  d_J_max_re = a * l42.re;
  d_J_max_im = a * l42.im;
  l213_tmp = l9 * l21;
  l9_re = l213_tmp * l42.re;
  l9_im = l213_tmp * l42.im;
  a = l9_re * l35.re - l9_im * l35.im;
  l9_im = l9_re * l35.im + l9_im * l35.re;
  if (l9_im == 0.0) {
    l9_re = a / 2.0;
    l9_im = 0.0;
  } else if (a == 0.0) {
    l9_re = 0.0;
    l9_im /= 2.0;
  } else {
    l9_re = a / 2.0;
    l9_im /= 2.0;
  }
  a = l7_tmp * l10_tmp * l21;
  l7_re = a * l42.re;
  l7_im = a * l42.im;
  a = l39 * b_A_init_re_tmp;
  A_init_im_tmp *= l39;
  f_A_init_re = a * l53.re - A_init_im_tmp * l53.im;
  A_init_im_tmp = a * l53.im + A_init_im_tmp * l53.re;
  a = l39 * l24;
  l21 = l39 * b_A_init_im_tmp;
  b_A_init_re_tmp = a * l53.re - l21 * l53.im;
  l21 = a * l53.im + l21 * l53.re;
  l287_re = l129 * (l11 * l43.re);
  l20 = l129 * (l11 * l43.im);
  l206 = l129 * (J_max_re_tmp_tmp * l43.re);
  b_A_min_re_tmp_tmp = l129 * (J_max_re_tmp_tmp * l43.im);
  f_A_min_re_tmp = l129 * (l7_re_tmp_tmp * l43.re);
  l47_tmp = l129 * (l7_re_tmp_tmp * l43.im);
  l8_re = l129 * (l8_re_tmp_tmp * l43.re);
  l8_im = l129 * (l8_re_tmp_tmp * l43.im);
  a = l8 * l44.re;
  l281_tmp = l8 * l44.im;
  b_A_init_im_tmp = a * l34.re - l281_tmp * l34.im;
  l281_tmp = a * l34.im + l281_tmp * l34.re;
  l287_im = J_max_re_tmp * l44.re - ar * l44.im;
  J_max_re_tmp = J_max_re_tmp * l44.im + ar * l44.re;
  a = l287_im * l53.re - J_max_re_tmp * l53.im;
  J_max_re_tmp = l287_im * l53.im + J_max_re_tmp * l53.re;
  l287_im = a * l151_re - J_max_re_tmp * l151_im;
  J_max_re_tmp = a * l151_im + J_max_re_tmp * l151_re;
  d = A_init_re_tmp_tmp * V_init;
  l39 = A_min * P_init;
  d1 = A_init_re_tmp_tmp * V_min;
  d2 = A_min * l3;
  ar = A_init * l5;
  l24 = V_init * l4_tmp;
  l213_tmp = V_min * l4_tmp;
  a = l2_tmp * l4_tmp;
    ar = l205 * ((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((ar * l15 * l38 * 12.0 + d2 * l15 * l38 * 16.0) + d_A_init_re_tmp * V_min * l15 * l38 * 24.0) + l39 * l9 * l16 * l38 * 24.0) + A_min * P_wayp * l9 * l10_tmp * rt_powd_snf(l31, 5.0) * 24.0) + l6 * l7_tmp * l16 * l38 * 16.0) + l6 * l9 * l12 * l38 * 48.0) + d * l8 * l16 * l38 * 24.0) + d * l10_tmp * l13 * l38 * 24.0) + -(c_A_init_re_tmp * l5 * l13 * l38 * 60.0)) + -(c_A_min_re_tmp * l3 * l13 * l38 * 60.0)) + -(l39 * l10_tmp * l15 * l38 * 24.0)) + l39 * l12 * l13 * l38 * 72.0) + -(l6 * l10_tmp * l11 * l38 * 16.0)) + -(l6 * l8 * l14 * l38 * 48.0)) + -(d * l9 * l14 * l38 * 24.0)) + -(d * l11 * l12 * l38 * 24.0)) + -(d1 * l8 * l16 * l38 * 24.0)) + d1 * l11 * l12 * l38 * 96.0) + -(d2 * l9 * l12 * l38 * 4.0)) + ar * l7_tmp * l16 * l38 * 12.0) + l2_re_tmp * l28_tmp * l38 * 24.0) + d2 * l10_tmp * l11 * l38 * 60.0) + l24 * l8 * l16 * l38 * 48.0) + l213_tmp * l9 * l14 * l38 * 48.0) + -(l39 * l11 * l14 * l38 * 72.0)) + -(d1 * l10_tmp * l13 * l38 * 96.0)) + -(ar * l8 * l14 * l38 * 12.0)) + -(d2 * l8 * l14 * l38 * 12.0)) + -(ar * l9 * l12 * l38 * 48.0)) + ar * l10_tmp * l11 * l38 * 96.0) + -(l24 * l10_tmp * l13 * l38 * 24.0)) + l24 * l11 * l12 * l38 * 96.0) + -(l213_tmp * l8 * l16 * l38 * 24.0)) + -(l213_tmp * l11 * l12 * l38 * 24.0)) + -(l24 * l9 * l14 * l38 * 120.0)) + -(a * l8 * l14 * l38 * 36.0)) + a * l9 * l12 * l38 * 96.0) + -(a * l10_tmp * l11 * l38 * 84.0)) + l98 * (l205_tmp * l44.re) * 4.0) + -(l98 * (d_A_min_re_tmp * l44.re) * 12.0)) + -(l11 * l38 * l135 * 3.0)) + d_A_init_re_tmp * l11 * l38 * l129 * 48.0) + -(l98 * (e_A_min_re_tmp * l44.re) * 4.0)) + -(A_init_re_tmp_tmp * l13 * l38 * l129 * 12.0)) + -(c_J_max_re_tmp * l38 * l129 * 12.0)) + l2_re_tmp * l38 * l129 * 12.0) + J_max_re_tmp_tmp * l38 * l135 * 15.0) + V_init_re_tmp * l38 * l129 * 24.0) + b_V_init_re_tmp * l38 * l129 * 48.0) + V_min_re_tmp * l38 * l129 * 24.0) + -(d_J_max_re_tmp * l38 * l129 * 42.0)) + -(l11_re_tmp * l38 * l129 * 12.0)) + -(c_V_init_re_tmp * l38 * l129 * 60.0)) + -(b_V_min_re_tmp * l38 * l129 * 12.0)) + -(c_V_min_re_tmp * l38 * l129 * 12.0)) + l7_re_tmp_tmp * l38 * l135 * 9.0) + (b_A_min_re * l53.re - A_min_im * l53.im) * 24.0) + A_init_re_tmp_tmp * l7_tmp * l14 * l38 * l129 * 12.0) + -(l8_re_tmp_tmp * l38 * l135 * 21.0)) + (c_A_min_re * l53.re - b_A_min_im * l53.im) * 12.0) + -(b_A_init_re_tmp_tmp * l10_tmp * l38 * l129 * 48.0)) + b_l2_re_tmp * l38 * l129 * 48.0) + l4_re_tmp * l38 * l129 * 24.0) + b_l4_re_tmp * l38 * l129 * 48.0) + (d_A_min_re * l53.re - c_A_min_im * l53.im) * 12.0) + (e_A_min_re * l53.re - d_A_min_im * l53.im) * 24.0) + -(c_l2_re_tmp * l38 * l129 * 18.0)) + -(c_l4_re_tmp * l38 * l129 * 60.0)) + -((b_A_init_re * l53.re - A_init_im * l53.im) * 24.0)) + -((c_A_init_re * l53.re - b_A_init_im * l53.im) * 24.0)) + A_init_re_tmp * l34.re * 12.0) + l8_re_tmp * l34.re * 12.0) + -(l129 * (A_init_re * l53.re - c_A_init_im * l53.im) * 12.0)) + -(l129 * (d_A_init_re * l53.re - d_A_init_im * l53.im) * 12.0)) + (A_min_re * l151_re - e_A_min_im * l151_im) * 12.0) + b_l8_re_tmp * l34.re * 6.0) + -((e_A_init_re * l151_re - e_A_init_im * l151_im) * 24.0)) + (b_l4_re * l151_re - l4_im * l151_im) * 24.0) + (b_J_max_re * l151_re - J_max_im * l151_im) * 12.0) + l46.re) + -(b_J_max_re_tmp * l134.re * 9.0)) + -(l7_re_tmp * l134.re * 3.0)) + c_l8_re_tmp * l134.re * 9.0) + -((J_max_re * l213.re - b_J_max_im * l213.im) * 12.0)) + (l2_re * l213.re - l2_im * l213.im) * 12.0) + (V_init_re * l213.re - V_init_im * l213.im) * 24.0) + (b_V_init_re * l213.re - b_V_init_im * l213.im) * 48.0) + (V_min_re * l213.re - V_min_im * l213.im) * 24.0) + -((c_J_max_re * l213.re - c_J_max_im * l213.im) * 42.0)) + -((l11_re * l213.re - l11_im * l213.im) * 12.0)) + -((c_V_init_re * l213.re - c_V_init_im * l213.im) * 60.0)) + -((b_V_min_re * l213.re - b_V_min_im * l213.im) * 12.0)) + -((c_V_min_re * l213.re - c_V_min_im * l213.im) * 12.0)) + (b_l2_re * l213.re - b_l2_im * l213.im) * 48.0) + (l4_re * l213.re - b_l4_im * l213.im) * 12.0) + (c_l4_re * l213.re - c_l4_im * l213.im) * 36.0) + -((c_l2_re * l213.re - c_l2_im * l213.im) * 18.0)) + -((d_l4_re * l213.re - d_l4_im * l213.im) * 36.0)) + J_min_re) + (d_J_max_re * l35.re - d_J_max_im * l35.im) * 1.5) + -l9_re) + -((l7_re * l35.re - l7_im * l35.im) * 1.5)) + -((f_A_init_re * l213.re - A_init_im_tmp * l213.im) * 12.0)) + -((b_A_init_re_tmp * l213.re - l21 * l213.im) * 12.0)) + -((l287_re * l213.re - l20 * l213.im) * 6.0)) + (l206 * l213.re - b_A_min_re_tmp_tmp * l213.im) * 24.0) + (f_A_min_re_tmp * l213.re - l47_tmp * l213.im) * 12.0) + -((l8_re * l213.re - l8_im * l213.im) * 30.0)) + (b_A_init_im_tmp * l213.re - l281_tmp * l213.im) * 6.0) + (l287_im * l213.re - J_max_re_tmp * l213.im) * 12.0);
    l213_tmp =
        l205 *
        ((((((((((((((((((((((((((((((((((((((((((((((((l98 *
                                                            (l205_tmp *
                                                             l44.im) *
                                                            4.0 +
                                                        -(l98 *
                                                          (d_A_min_re_tmp *
                                                           l44.im) *
                                                          12.0)) +
                                                       -(l98 *
                                                         (e_A_min_re_tmp *
                                                          l44.im) *
                                                         4.0)) +
                                                      (b_A_min_re * l53.im +
                                                       A_min_im * l53.re) *
                                                          24.0) +
                                                     (c_A_min_re * l53.im +
                                                      b_A_min_im * l53.re) *
                                                         12.0) +
                                                    (d_A_min_re * l53.im +
                                                     c_A_min_im * l53.re) *
                                                        12.0) +
                                                   (e_A_min_re * l53.im +
                                                    d_A_min_im * l53.re) *
                                                       24.0) +
                                                  -((b_A_init_re * l53.im +
                                                     A_init_im * l53.re) *
                                                    24.0)) +
                                                 -((c_A_init_re * l53.im +
                                                    b_A_init_im * l53.re) *
                                                   24.0)) +
                                                A_init_re_tmp * l34.im * 12.0) +
                                               l8_re_tmp * l34.im * 12.0) +
                                              -(l129 *
                                                (A_init_re * l53.im +
                                                 c_A_init_im * l53.re) *
                                                12.0)) +
                                             -(l129 *
                                               (d_A_init_re * l53.im +
                                                d_A_init_im * l53.re) *
                                               12.0)) +
                                            (A_min_re * l151_im +
                                             e_A_min_im * l151_re) *
                                                12.0) +
                                           b_l8_re_tmp * l34.im * 6.0) +
                                          -((e_A_init_re * l151_im +
                                             e_A_init_im * l151_re) *
                                            24.0)) +
                                         (b_l4_re * l151_im + l4_im * l151_re) *
                                             24.0) +
                                        (b_J_max_re * l151_im +
                                         J_max_im * l151_re) *
                                            12.0) +
                                       l46.im) +
                                      -(b_J_max_re_tmp * l134.im * 9.0)) +
                                     -(l7_re_tmp * l134.im * 3.0)) +
                                    c_l8_re_tmp * l134.im * 9.0) +
                                   -((J_max_re * l213.im +
                                      b_J_max_im * l213.re) *
                                     12.0)) +
                                  (l2_re * l213.im + l2_im * l213.re) * 12.0) +
                                 (V_init_re * l213.im + V_init_im * l213.re) *
                                     24.0) +
                                (b_V_init_re * l213.im +
                                 b_V_init_im * l213.re) *
                                    48.0) +
                               (V_min_re * l213.im + V_min_im * l213.re) *
                                   24.0) +
                              -((c_J_max_re * l213.im + c_J_max_im * l213.re) *
                                42.0)) +
                             -((l11_re * l213.im + l11_im * l213.re) * 12.0)) +
                            -((c_V_init_re * l213.im + c_V_init_im * l213.re) *
                              60.0)) +
                           -((b_V_min_re * l213.im + b_V_min_im * l213.re) *
                             12.0)) +
                          -((c_V_min_re * l213.im + c_V_min_im * l213.re) *
                            12.0)) +
                         (b_l2_re * l213.im + b_l2_im * l213.re) * 48.0) +
                        (l4_re * l213.im + b_l4_im * l213.re) * 12.0) +
                       (c_l4_re * l213.im + c_l4_im * l213.re) * 36.0) +
                      -((c_l2_re * l213.im + c_l2_im * l213.re) * 18.0)) +
                     -((d_l4_re * l213.im + d_l4_im * l213.re) * 36.0)) +
                    J_min_im) +
                   (d_J_max_re * l35.im + d_J_max_im * l35.re) * 1.5) +
                  -l9_im) +
                 -((l7_re * l35.im + l7_im * l35.re) * 1.5)) +
                -((f_A_init_re * l213.im + A_init_im_tmp * l213.re) * 12.0)) +
               -((b_A_init_re_tmp * l213.im + l21 * l213.re) * 12.0)) +
              -((l287_re * l213.im + l20 * l213.re) * 6.0)) +
             (l206 * l213.im + b_A_min_re_tmp_tmp * l213.re) * 24.0) +
            (f_A_min_re_tmp * l213.im + l47_tmp * l213.re) * 12.0) +
           -((l8_re * l213.im + l8_im * l213.re) * 30.0)) +
          (b_A_init_im_tmp * l213.im + l281_tmp * l213.re) * 6.0) +
         (l287_im * l213.im + J_max_re_tmp * l213.re) * 12.0);
    if (l213_tmp == 0.0) {
      l24 = ar / 2.0;
      a = 0.0;
    } else if (ar == 0.0) {
      l24 = 0.0;
      a = l213_tmp / 2.0;
    } else {
      l24 = ar / 2.0;
      a = l213_tmp / 2.0;
    }
    l34.re = (re + -l206_re) + l24;
    l34.im = (im + -l206_im) + a;
    dc = coder::d_power(l292);
    l281_tmp = l34.re * l34.im;
    l134.re = -dc.re + (l34.re * l34.re - l34.im * l34.im);
    l134.im = -dc.im + (l281_tmp + l281_tmp);
    coder::internal::scalar::b_sqrt(&l134);
    l35.re = l34.re + l134.re;
    l35.im = l34.im + l134.im;
    l46 = coder::power(l35);
    if (l46.im == 0.0) {
      l213_tmp = l46.re / 2.0;
      l206 = 0.0;
      re = 1.0 / l46.re;
      im = 0.0;
    } else if (l46.re == 0.0) {
      l213_tmp = 0.0;
      l206 = l46.im / 2.0;
      re = 0.0;
      im = -(1.0 / l46.im);
    } else {
      l213_tmp = l46.re / 2.0;
      l206 = l46.im / 2.0;
      l20 = std::abs(l46.re);
      a = std::abs(l46.im);
      if (l20 > a) {
        a = l46.im / l46.re;
        l287_im = l46.re + a * l46.im;
        re = (a * 0.0 + 1.0) / l287_im;
        im = (0.0 - a) / l287_im;
      } else if (a == l20) {
        if (l46.re > 0.0) {
          a = 0.5;
        } else {
          a = -0.5;
        }
        if (l46.im > 0.0) {
          l287_im = 0.5;
        } else {
          l287_im = -0.5;
        }
        re = (a + 0.0 * l287_im) / l20;
        im = (0.0 * a - l287_im) / l20;
      } else {
        a = l46.re / l46.im;
        l287_im = l46.im + a * l46.re;
        re = a / l287_im;
        im = (a * 0.0 - 1.0) / l287_im;
      }
    }
    l134.re = l292.re * re - l292.im * im;
    l134.im = l292.re * im + l292.im * re;
    if (l134.im == 0.0) {
      a = l134.re / 2.0;
      l287_im = 0.0;
    } else if (l134.re == 0.0) {
      a = 0.0;
      l287_im = l134.im / 2.0;
    } else {
      a = l134.re / 2.0;
      l287_im = l134.im / 2.0;
    }
    re = 1.7320508075688772 * (l46.re + -l134.re);
    im = 1.7320508075688772 * (l46.im + -l134.im);
    l34.re = re * 0.0 - im * 0.5;
    l34.im = re * 0.5 + im * 0.0;
    t4[0].re = (l284_re + l46.re) + l134.re;
    t4[0].im = (l284_im + l46.im) + l134.im;
    d = (l284_re + -l213_tmp) + -a;
    t4[1].re = d - l34.re;
    l39 = (l284_im + -l206) + -l287_im;
    t4[1].im = l39 - l34.im;
    t4[2].re = d + l34.re;
    t4[2].im = l39 + l34.im;
    l34.re = l42_tmp;
    l34.im = 0.0;
    coder::internal::scalar::b_sqrt(&l34);
    l134.re = b_l47_tmp;
    l134.im = 0.0;
    coder::internal::scalar::b_sqrt(&l134);
    if (-J_min < 0.0) {
      k_rtErrorWithMessageID(o_emlrtRTEI.fName, o_emlrtRTEI.lineNo);
    }
    l35.re = -J_min;
    l35.im = 0.0;
    coder::internal::scalar::b_sqrt(&l35);
    a = (l129_tmp_tmp + l28_tmp) + b_l25_tmp;
    J_min_re = a + A_min_re_tmp_tmp * t4[0].re * 2.0;
    J_min_im = A_min_re_tmp_tmp * t4[0].im * 2.0;
    d_A_min_re_tmp = A_min * rt_powd_snf(-J_min, 1.5);
    e_A_min_re_tmp = d_A_min_re_tmp * l134.re;
    b_A_init_im_tmp = d_A_min_re_tmp * l134.im;
    d_A_min_re_tmp = c_A_min_re_tmp * l35.re;
    J_max_re_tmp = c_A_min_re_tmp * l35.im;
    A_init_re_tmp = A_init_re_tmp_tmp * J_min;
    b_A_init_re_tmp = A_init_re_tmp * l34.re;
    A_init_im_tmp = A_init_re_tmp * l34.im;
    f_A_min_re_tmp = b_A_min_re_tmp * l34.re;
    l47_tmp = b_A_min_re_tmp * l34.im;
    d = l213_tmp_tmp * l34.re;
    l39 = (d_A_min_re_tmp * l134.re - J_max_re_tmp * l134.im) * 2.0;
    x[0].re = (((((-l34.re * J_min_re - -l34.im * J_min_im) + d) +
                 e_A_min_re_tmp * 2.0) +
                l39) +
               b_A_init_re_tmp * 2.0) +
              f_A_min_re_tmp * 2.0;
    d1 = l213_tmp_tmp * l34.im;
    d2 = (d_A_min_re_tmp * l134.im + J_max_re_tmp * l134.re) * 2.0;
    x[0].im = (((((-l34.re * J_min_im + -l34.im * J_min_re) + d1) +
                 b_A_init_im_tmp * 2.0) +
                d2) +
               A_init_im_tmp * 2.0) +
              l47_tmp * 2.0;
    J_min_re = a + A_min_re_tmp_tmp * t4[1].re * 2.0;
    J_min_im = A_min_re_tmp_tmp * t4[1].im * 2.0;
    x[1].re = (((((-l34.re * J_min_re - -l34.im * J_min_im) + d) +
                 e_A_min_re_tmp * 2.0) +
                l39) +
               b_A_init_re_tmp * 2.0) +
              f_A_min_re_tmp * 2.0;
    x[1].im = (((((-l34.re * J_min_im + -l34.im * J_min_re) + d1) +
                 b_A_init_im_tmp * 2.0) +
                d2) +
               A_init_im_tmp * 2.0) +
              l47_tmp * 2.0;
    J_min_re = a + A_min_re_tmp_tmp * t4[2].re * 2.0;
    J_min_im = A_min_re_tmp_tmp * t4[2].im * 2.0;
    b_A_min_re_tmp_tmp = -l34.re * J_min_re - -l34.im * J_min_im;
    l281_tmp = -l34.re * J_min_im + -l34.im * J_min_re;
    l39 = ((((b_A_min_re_tmp_tmp + d) + e_A_min_re_tmp * 2.0) + l39) +
           b_A_init_re_tmp * 2.0) +
          f_A_min_re_tmp * 2.0;
    d2 = ((((l281_tmp + d1) + b_A_init_im_tmp * 2.0) + d2) +
          A_init_im_tmp * 2.0) +
         l47_tmp * 2.0;
    A_min_re = A_min_re_tmp_tmp * l34.re;
    A_min_im = A_min_re_tmp_tmp * l34.im;
    l34.re = 2.0 * A_min_re;
    l34.im = 2.0 * A_min_im;
    l206 = J_min * l2_tmp;
    l24 = 1.0 /
          (A_min_re_tmp * l10_tmp * 2.0 + -(c_A_min_re_tmp * l7_tmp * 2.0)) *
          (((((J_max * V_min * l7_tmp * 2.0 + J_min * V_max * l10_tmp * 2.0) +
              l2_tmp * l7_tmp) +
             l4_re_tmp_tmp) +
            l281_tmp_tmp) +
           -(J_min * (((l129_tmp * 2.0 + l206) + l28_tmp * 2.0) +
                      V_min_re_tmp_tmp * 2.0)));
    ar = (J_min - J_max) *
         (((((l129_tmp_tmp - l28_tmp) + l206) - l25_tmp * V_init * 2.0) +
           b_l25_tmp) -
          A_min_re_tmp_tmp * -l24 * 2.0);
    t3[0].re = ar;
    t3[0].im = 0.0;
    t3[1].re = ar;
    t3[1].im = 0.0;
    t3[2].re = ar;
    t3[2].im = 0.0;
    coder::internal::scalar::b_sqrt(&t3[0]);
    coder::internal::scalar::b_sqrt(&t3[1]);
    coder::internal::scalar::b_sqrt(&t3[2]);
    l287_re = l7_tmp - l25_tmp;
    l35.re = A_init - A_min;
    if (t3[0].im == 0.0) {
      re = t3[0].re / l287_re;
      im = 0.0;
    } else if (t3[0].re == 0.0) {
      re = 0.0;
      im = t3[0].im / l287_re;
    } else {
      re = t3[0].re / l287_re;
      im = t3[0].im / l287_re;
    }
    l287_im = J_min * re;
    l206 = J_min * im;
    ar = -(A_init + l287_im);
    if (-l206 == 0.0) {
      a = ar / J_max;
      l213_tmp = 0.0;
    } else if (ar == 0.0) {
      a = 0.0;
      l213_tmp = -l206 / J_max;
    } else {
      a = ar / J_max;
      l213_tmp = -l206 / J_max;
    }
    t[0].re = a;
    t[0].im = l213_tmp;
    t[3].re = 0.0;
    t[3].im = 0.0;
    t[6].re = re;
    t[6].im = im;
    t[9] = t4[0];
    ar = -((l35.re + l287_im) + J_max * a);
    l213_tmp = -(l206 + J_max * l213_tmp);
    if (l213_tmp == 0.0) {
      t[12].re = ar / J_min;
      t[12].im = 0.0;
    } else if (ar == 0.0) {
      t[12].re = 0.0;
      t[12].im = l213_tmp / J_min;
    } else {
      t[12].re = ar / J_min;
      t[12].im = l213_tmp / J_min;
    }
    t[15].re = -l24;
    t[15].im = 0.0;
    if (l34.im == 0.0) {
      if (x[0].im == 0.0) {
        t[18].re = x[0].re / l34.re;
        t[18].im = 0.0;
      } else if (x[0].re == 0.0) {
        t[18].re = 0.0;
        t[18].im = x[0].im / l34.re;
      } else {
        t[18].re = x[0].re / l34.re;
        t[18].im = x[0].im / l34.re;
      }
    } else if (l34.re == 0.0) {
      if (x[0].re == 0.0) {
        t[18].re = x[0].im / l34.im;
        t[18].im = 0.0;
      } else if (x[0].im == 0.0) {
        t[18].re = 0.0;
        t[18].im = -(x[0].re / l34.im);
      } else {
        t[18].re = x[0].im / l34.im;
        t[18].im = -(x[0].re / l34.im);
      }
    } else {
      l20 = std::abs(l34.re);
      a = std::abs(l34.im);
      if (l20 > a) {
        a = l34.im / l34.re;
        l287_im = l34.re + a * l34.im;
        t[18].re = (x[0].re + a * x[0].im) / l287_im;
        t[18].im = (x[0].im - a * x[0].re) / l287_im;
      } else if (a == l20) {
        if (l34.re > 0.0) {
          a = 0.5;
        } else {
          a = -0.5;
        }
        if (l34.im > 0.0) {
          l287_im = 0.5;
        } else {
          l287_im = -0.5;
        }
        t[18].re = (x[0].re * a + x[0].im * l287_im) / l20;
        t[18].im = (x[0].im * a - x[0].re * l287_im) / l20;
      } else {
        a = l34.re / l34.im;
        l287_im = l34.im + a * l34.re;
        t[18].re = (a * x[0].re + x[0].im) / l287_im;
        t[18].im = (a * x[0].im - x[0].re) / l287_im;
      }
    }
    if (t3[1].im == 0.0) {
      re = t3[1].re / l287_re;
      im = 0.0;
    } else if (t3[1].re == 0.0) {
      re = 0.0;
      im = t3[1].im / l287_re;
    } else {
      re = t3[1].re / l287_re;
      im = t3[1].im / l287_re;
    }
    l287_im = J_min * re;
    l206 = J_min * im;
    ar = -(A_init + l287_im);
    if (-l206 == 0.0) {
      a = ar / J_max;
      l213_tmp = 0.0;
    } else if (ar == 0.0) {
      a = 0.0;
      l213_tmp = -l206 / J_max;
    } else {
      a = ar / J_max;
      l213_tmp = -l206 / J_max;
    }
    t[1].re = a;
    t[1].im = l213_tmp;
    t[4].re = 0.0;
    t[4].im = 0.0;
    t[7].re = re;
    t[7].im = im;
    t[10] = t4[1];
    ar = -((l35.re + l287_im) + J_max * a);
    l213_tmp = -(l206 + J_max * l213_tmp);
    if (l213_tmp == 0.0) {
      t[13].re = ar / J_min;
      t[13].im = 0.0;
    } else if (ar == 0.0) {
      t[13].re = 0.0;
      t[13].im = l213_tmp / J_min;
    } else {
      t[13].re = ar / J_min;
      t[13].im = l213_tmp / J_min;
    }
    t[16].re = -l24;
    t[16].im = 0.0;
    if (l34.im == 0.0) {
      if (x[1].im == 0.0) {
        t[19].re = x[1].re / l34.re;
        t[19].im = 0.0;
      } else if (x[1].re == 0.0) {
        t[19].re = 0.0;
        t[19].im = x[1].im / l34.re;
      } else {
        t[19].re = x[1].re / l34.re;
        t[19].im = x[1].im / l34.re;
      }
    } else if (l34.re == 0.0) {
      if (x[1].re == 0.0) {
        t[19].re = x[1].im / l34.im;
        t[19].im = 0.0;
      } else if (x[1].im == 0.0) {
        t[19].re = 0.0;
        t[19].im = -(x[1].re / l34.im);
      } else {
        t[19].re = x[1].im / l34.im;
        t[19].im = -(x[1].re / l34.im);
      }
    } else {
      l20 = std::abs(l34.re);
      a = std::abs(l34.im);
      if (l20 > a) {
        a = l34.im / l34.re;
        l287_im = l34.re + a * l34.im;
        t[19].re = (x[1].re + a * x[1].im) / l287_im;
        t[19].im = (x[1].im - a * x[1].re) / l287_im;
      } else if (a == l20) {
        if (l34.re > 0.0) {
          a = 0.5;
        } else {
          a = -0.5;
        }
        if (l34.im > 0.0) {
          l287_im = 0.5;
        } else {
          l287_im = -0.5;
        }
        t[19].re = (x[1].re * a + x[1].im * l287_im) / l20;
        t[19].im = (x[1].im * a - x[1].re * l287_im) / l20;
      } else {
        a = l34.re / l34.im;
        l287_im = l34.im + a * l34.re;
        t[19].re = (a * x[1].re + x[1].im) / l287_im;
        t[19].im = (a * x[1].im - x[1].re) / l287_im;
      }
    }
    if (t3[2].im == 0.0) {
      re = t3[2].re / l287_re;
      im = 0.0;
    } else if (t3[2].re == 0.0) {
      re = 0.0;
      im = t3[2].im / l287_re;
    } else {
      re = t3[2].re / l287_re;
      im = t3[2].im / l287_re;
    }
    l287_im = J_min * re;
    l206 = J_min * im;
    ar = -(A_init + l287_im);
    if (-l206 == 0.0) {
      a = ar / J_max;
      l213_tmp = 0.0;
    } else if (ar == 0.0) {
      a = 0.0;
      l213_tmp = -l206 / J_max;
    } else {
      a = ar / J_max;
      l213_tmp = -l206 / J_max;
    }
    t[2].re = a;
    t[2].im = l213_tmp;
    t[5].re = 0.0;
    t[5].im = 0.0;
    t[8].re = re;
    t[8].im = im;
    t[11] = t4[2];
    ar = -((l35.re + l287_im) + J_max * a);
    l213_tmp = -(l206 + J_max * l213_tmp);
    if (l213_tmp == 0.0) {
      t[14].re = ar / J_min;
      t[14].im = 0.0;
    } else if (ar == 0.0) {
      t[14].re = 0.0;
      t[14].im = l213_tmp / J_min;
    } else {
      t[14].re = ar / J_min;
      t[14].im = l213_tmp / J_min;
    }
    t[17].re = -l24;
    t[17].im = 0.0;
    ar = ((((b_A_min_re_tmp_tmp + d) + e_A_min_re_tmp * 2.0) +
           (d_A_min_re_tmp * l134.re - J_max_re_tmp * l134.im) * 2.0) +
          b_A_init_re_tmp * 2.0) +
         f_A_min_re_tmp * 2.0;
    l213_tmp = ((((l281_tmp + d1) + b_A_init_im_tmp * 2.0) +
                 (d_A_min_re_tmp * l134.im + J_max_re_tmp * l134.re) * 2.0) +
                A_init_im_tmp * 2.0) +
               l47_tmp * 2.0;
    if (l34.im == 0.0) {
      if (d2 == 0.0) {
        t[20].re = l39 / l34.re;
        t[20].im = 0.0;
      } else if (l39 == 0.0) {
        t[20].re = 0.0;
        t[20].im = d2 / l34.re;
      } else {
        t[20].re = l39 / l34.re;
        t[20].im = d2 / l34.re;
      }
    } else if (l34.re == 0.0) {
      if (l39 == 0.0) {
        t[20].re = d2 / l34.im;
        t[20].im = 0.0;
      } else if (d2 == 0.0) {
        t[20].re = 0.0;
        t[20].im = -(l39 / l34.im);
      } else {
        t[20].re = d2 / l34.im;
        t[20].im = -(l39 / l34.im);
      }
    } else {
      l20 = std::abs(l34.re);
      a = std::abs(l34.im);
      if (l20 > a) {
        a = l34.im / l34.re;
        l287_im = l34.re + a * l34.im;
        t[20].re = (ar + a * l213_tmp) / l287_im;
        t[20].im = (l213_tmp - a * ar) / l287_im;
      } else if (a == l20) {
        if (l34.re > 0.0) {
          a = 0.5;
        } else {
          a = -0.5;
        }
        if (l34.im > 0.0) {
          l287_im = 0.5;
        } else {
          l287_im = -0.5;
        }
        t[20].re = (ar * a + l213_tmp * l287_im) / l20;
        t[20].im = (l213_tmp * a - ar * l287_im) / l20;
      } else {
        a = l34.re / l34.im;
        l287_im = l34.im + a * l34.re;
        t[20].re = (a * ar + l213_tmp) / l287_im;
        t[20].im = (a * l213_tmp - ar) / l287_im;
      }
    }
}

// End of code generation (acdefg_T_P.cpp)
