//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// acdeg_T_P.cpp
//
// Code generation for function 'acdeg_T_P'
//

// Include files
#include "acdeg_T_P.h"
#include "power.h"
#include "rt_nonfinite.h"
#include "sqrt.h"
#include "topico_wrapper_data.h"
#include "topico_wrapper_rtwutil.h"
#include "topico_wrapper_types.h"
#include "rt_nonfinite.h"
#include <cmath>

// Function Definitions
void acdeg_T_P(double P_init, double V_init, double A_init, double P_wayp,
               double V_max, double V_min, double J_max, double J_min, double T,
               creal_T t[21])
{
  creal_T b_l3[3];
  creal_T t4[3];
  creal_T dc;
  creal_T l124;
  creal_T l125;
  creal_T l294;
  creal_T l295;
  creal_T l312;
  creal_T l349;
  creal_T l409;
  creal_T l578;
  creal_T l67;
  creal_T l68;
  creal_T l69;
  creal_T l75;
  creal_T l76;
  double A_init_im;
  double A_init_re;
  double A_init_re_tmp;
  double A_init_re_tmp_tmp;
  double J_max_im;
  double J_max_re;
  double T_im;
  double T_re;
  double V_init_im;
  double V_init_re;
  double V_max_im;
  double V_max_re;
  double V_min_im;
  double V_min_re;
  double ab_V_max_im;
  double ab_V_max_re;
  double b_A_init_im;
  double b_A_init_re;
  double b_A_init_re_tmp_tmp;
  double b_J_max_im;
  double b_J_max_re;
  double b_T_im;
  double b_T_re;
  double b_V_init_im;
  double b_V_init_re;
  double b_V_max_im;
  double b_V_max_re;
  double b_V_min_im;
  double b_V_min_re;
  double b_im;
  double b_l10_im;
  double b_l10_re;
  double b_l11_im;
  double b_l11_re;
  double b_l13_im;
  double b_l13_re;
  double b_l2_im;
  double b_l2_re;
  double b_l4_im;
  double b_l4_re;
  double b_l578_tmp;
  double b_l586_re_tmp;
  double b_l6_im;
  double b_l6_re;
  double b_l8_im;
  double b_l8_re;
  double b_re;
  double c_A_init_im;
  double c_A_init_re;
  double c_J_max_im;
  double c_J_max_re;
  double c_T_im;
  double c_T_re;
  double c_V_init_im;
  double c_V_init_re;
  double c_V_max_im;
  double c_V_max_re;
  double c_V_min_im;
  double c_V_min_re;
  double c_im;
  double c_l13_im;
  double c_l13_re;
  double c_l2_im;
  double c_l2_re;
  double c_l4_im;
  double c_l4_re;
  double c_l578_tmp;
  double c_l586_re_tmp;
  double c_re;
  double d;
  double d1;
  double d2;
  double d3;
  double d4;
  double d5;
  double d6;
  double d_A_init_im;
  double d_A_init_re;
  double d_J_max_im;
  double d_J_max_re;
  double d_T_im;
  double d_T_re;
  double d_V_init_im;
  double d_V_init_re;
  double d_V_max_im;
  double d_V_max_re;
  double d_V_min_im;
  double d_V_min_re;
  double d_im;
  double d_l2_im;
  double d_l2_re;
  double d_l578_tmp;
  double d_l586_re_tmp;
  double d_re;
  double e_A_init_im;
  double e_A_init_re;
  double e_J_max_im;
  double e_J_max_re;
  double e_T_im;
  double e_T_re;
  double e_V_init_im;
  double e_V_init_re;
  double e_V_max_im;
  double e_V_max_re;
  double e_V_min_im;
  double e_V_min_re;
  double e_im;
  double e_l2_im;
  double e_l2_re;
  double e_l586_re_tmp;
  double e_re;
  double f_A_init_im;
  double f_A_init_re;
  double f_J_max_im;
  double f_J_max_re;
  double f_T_im;
  double f_T_re;
  double f_V_init_im;
  double f_V_init_re;
  double f_V_max_im;
  double f_V_max_re;
  double f_V_min_im;
  double f_V_min_re;
  double f_l2_im;
  double f_l2_re;
  double f_l586_re_tmp;
  double g_A_init_im;
  double g_A_init_re;
  double g_J_max_im;
  double g_J_max_re;
  double g_T_im;
  double g_T_re;
  double g_V_init_im;
  double g_V_init_re;
  double g_V_max_im;
  double g_V_max_re;
  double g_V_min_im;
  double g_V_min_re;
  double g_l2_im;
  double g_l2_re;
  double g_l586_re_tmp;
  double h_A_init_im;
  double h_A_init_re;
  double h_J_max_im;
  double h_J_max_re;
  double h_T_im;
  double h_T_re;
  double h_V_init_im;
  double h_V_init_re;
  double h_V_max_im;
  double h_V_max_re;
  double h_V_min_im;
  double h_V_min_re;
  double h_l2_im;
  double h_l2_re;
  double h_l586_re_tmp;
  double i_A_init_im;
  double i_A_init_re;
  double i_J_max_im;
  double i_J_max_re;
  double i_T_im;
  double i_T_re;
  double i_V_init_im;
  double i_V_init_re;
  double i_V_max_im;
  double i_V_max_re;
  double i_V_min_im;
  double i_V_min_re;
  double i_l2_im;
  double i_l2_re;
  double i_l586_re_tmp;
  double im;
  double j_A_init_im;
  double j_A_init_re;
  double j_T_im;
  double j_T_re;
  double j_V_init_im;
  double j_V_init_re;
  double j_V_max_im;
  double j_V_max_re;
  double j_V_min_im;
  double j_V_min_re;
  double j_l2_im;
  double j_l2_re;
  double j_l586_re_tmp;
  double k_A_init_im;
  double k_A_init_re;
  double k_T_im;
  double k_T_re;
  double k_V_init_im;
  double k_V_init_re;
  double k_V_max_im;
  double k_V_max_re;
  double k_V_min_im;
  double k_V_min_re;
  double k_l2_im;
  double k_l2_re;
  double k_l586_re_tmp;
  double l10;
  double l102;
  double l103;
  double l104;
  double l104_tmp;
  double l10_im;
  double l10_re;
  double l11;
  double l11_im;
  double l11_re;
  double l12;
  double l12_im;
  double l12_re;
  double l12_re_tmp;
  double l13;
  double l131;
  double l131_tmp;
  double l135;
  double l135_tmp;
  double l13_im;
  double l13_re;
  double l14;
  double l147;
  double l147_tmp;
  double l148;
  double l148_tmp;
  double l153;
  double l153_tmp;
  double l170;
  double l18;
  double l19;
  double l20;
  double l21;
  double l218;
  double l218_tmp;
  double l219;
  double l21_im;
  double l21_re;
  double l21_re_tmp;
  double l22;
  double l23;
  double l24;
  double l24_im;
  double l24_re;
  double l263;
  double l263_tmp;
  double l294_tmp;
  double l29_tmp;
  double l2_im;
  double l2_re;
  double l2_tmp;
  double l3;
  double l30;
  double l31;
  double l310_im;
  double l310_re;
  double l315_im;
  double l315_re;
  double l319_im;
  double l319_re;
  double l32;
  double l322_im;
  double l322_re;
  double l324_im;
  double l324_re;
  double l326_im;
  double l326_re;
  double l328_im;
  double l328_re;
  double l35;
  double l36;
  double l361_im;
  double l361_re;
  double l37;
  double l38;
  double l39;
  double l4;
  double l40;
  double l41;
  double l410_im;
  double l410_re;
  double l411_im;
  double l411_re;
  double l412_im;
  double l412_re;
  double l419_im;
  double l419_re;
  double l42;
  double l43;
  double l44;
  double l45;
  double l45_im;
  double l45_re;
  double l46;
  double l47;
  double l48;
  double l484_im;
  double l484_re;
  double l48_im;
  double l48_re;
  double l49;
  double l49_im;
  double l49_re;
  double l4_im;
  double l4_re;
  double l5;
  double l51;
  double l52;
  double l527_im;
  double l527_re;
  double l575;
  double l575_im;
  double l575_re;
  double l576;
  double l578_tmp;
  double l584_im;
  double l584_re;
  double l586_im;
  double l586_re;
  double l586_re_tmp;
  double l588_im;
  double l588_re;
  double l6;
  double l62;
  double l62_tmp;
  double l67_tmp;
  double l6_im;
  double l6_re;
  double l7;
  double l74;
  double l75_tmp;
  double l8;
  double l82;
  double l82_tmp;
  double l8_im;
  double l8_re;
  double l9;
  double l96;
  double l_A_init_im;
  double l_A_init_re;
  double l_T_im;
  double l_T_re;
  double l_V_init_im;
  double l_V_init_re;
  double l_V_max_im;
  double l_V_max_re;
  double l_V_min_im;
  double l_V_min_re;
  double l_l2_im;
  double l_l2_re;
  double m_A_init_im;
  double m_A_init_re;
  double m_V_init_im;
  double m_V_init_re;
  double m_V_max_im;
  double m_V_max_re;
  double m_V_min_im;
  double m_V_min_re;
  double m_l2_im;
  double m_l2_re;
  double n_A_init_im;
  double n_A_init_re;
  double n_V_init_im;
  double n_V_init_re;
  double n_V_max_im;
  double n_V_max_re;
  double n_V_min_im;
  double n_V_min_re;
  double n_l2_im;
  double n_l2_re;
  double o_A_init_im;
  double o_A_init_re;
  double o_V_init_im;
  double o_V_init_re;
  double o_V_max_im;
  double o_V_max_re;
  double o_V_min_im;
  double o_V_min_re;
  double o_l2_im;
  double o_l2_re;
  double p_A_init_im;
  double p_A_init_re;
  double p_V_init_im;
  double p_V_init_re;
  double p_V_max_im;
  double p_V_max_re;
  double p_V_min_im;
  double p_V_min_re;
  double p_l2_im;
  double p_l2_re;
  double q_A_init_im;
  double q_A_init_re;
  double q_V_max_im;
  double q_V_max_re;
  double q_V_min_im;
  double q_V_min_re;
  double q_l2_im;
  double q_l2_re;
  double r;
  double r_A_init_im;
  double r_A_init_re;
  double r_V_max_im;
  double r_V_max_re;
  double r_V_min_im;
  double r_V_min_re;
  double r_l2_im;
  double r_l2_re;
  double re;
  double re_tmp;
  double s_A_init_im;
  double s_A_init_re;
  double s_V_max_im;
  double s_V_max_re;
  double t_A_init_im;
  double t_A_init_re;
  double t_V_max_im;
  double t_V_max_re;
  double u_A_init_im;
  double u_A_init_re;
  double u_V_max_im;
  double u_V_max_re;
  double v_V_max_im;
  double v_V_max_re;
  double w_V_max_im;
  double w_V_max_re;
  double x_V_max_im;
  double x_V_max_re;
  double y_V_max_im;
  double y_V_max_re;
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
  //  Generated on 03-Sep-2019 16:11:46
  l2_tmp = A_init * A_init;
  l3 = rt_powd_snf(A_init, 3.0);
  l4 = J_max * J_max;
  l5 = rt_powd_snf(J_min, 5.0);
  l6 = rt_powd_snf(J_max, 3.0);
  l7 = rt_powd_snf(J_min, 6.0);
  l9 = rt_powd_snf(J_min, 7.0);
  l10 = rt_powd_snf(J_max, 5.0);
  l12 = rt_powd_snf(J_max, 7.0);
  l13 = T * T;
  l14 = rt_powd_snf(T, 3.0);
  l18 = rt_powd_snf(J_min, 8.0);
  l19 = rt_powd_snf(J_min, 9.0);
  l22 = rt_powd_snf(J_min, 11.0);
  if (J_max < 0.0) {
    f_rtErrorWithMessageID("sqrt", n_emlrtRTEI.fName, n_emlrtRTEI.lineNo);
  }
  l29_tmp = std::sqrt(J_max);
  l8 = l4 * l4;
  l11 = rt_powd_snf(l4, 3.0);
  l20 = l5 * l5;
  l23 = rt_powd_snf(l6, 3.0);
  l24 = l7 * l7;
  l30 = rt_powd_snf(l29_tmp, 3.0);
  l31 = rt_powd_snf(l29_tmp, 5.0);
  l32 = rt_powd_snf(l29_tmp, 7.0);
  l36 = rt_powd_snf(l29_tmp, 11.0);
  l37 = rt_powd_snf(l29_tmp, 13.0);
  l39 = rt_powd_snf(l29_tmp, 17.0);
  l42 = rt_powd_snf(-J_min, 4.5);
  if (-J_min < 0.0) {
    k_rtErrorWithMessageID(o_emlrtRTEI.fName, o_emlrtRTEI.lineNo);
  }
  l43 = rt_powd_snf(-J_min, 5.5);
  if (-J_min < 0.0) {
    k_rtErrorWithMessageID(o_emlrtRTEI.fName, o_emlrtRTEI.lineNo);
  }
  l44 = rt_powd_snf(-J_min, 6.5);
  if (-J_min < 0.0) {
    k_rtErrorWithMessageID(o_emlrtRTEI.fName, o_emlrtRTEI.lineNo);
  }
  l45 = rt_powd_snf(-J_min, 7.5);
  if (-J_min < 0.0) {
    k_rtErrorWithMessageID(o_emlrtRTEI.fName, o_emlrtRTEI.lineNo);
  }
  l46 = rt_powd_snf(-J_min, 8.5);
  if (-J_min < 0.0) {
    k_rtErrorWithMessageID(o_emlrtRTEI.fName, o_emlrtRTEI.lineNo);
  }
  l47 = rt_powd_snf(-J_min, 9.5);
  if (-J_min < 0.0) {
    k_rtErrorWithMessageID(o_emlrtRTEI.fName, o_emlrtRTEI.lineNo);
  }
  l48 = rt_powd_snf(-J_min, 10.5);
  if (-J_min < 0.0) {
    k_rtErrorWithMessageID(o_emlrtRTEI.fName, o_emlrtRTEI.lineNo);
  }
  l49 = rt_powd_snf(-J_min, 11.5);
  if (-J_min < 0.0) {
    k_rtErrorWithMessageID(o_emlrtRTEI.fName, o_emlrtRTEI.lineNo);
  }
  l82_tmp = A_init * l9;
  l82 = l82_tmp * l12 * 18.0;
  l131_tmp = A_init * l6;
  l131 = l131_tmp * l22 * 18.0;
  l135_tmp = A_init * l10;
  l135 = l135_tmp * l19 * 60.0;
  l218_tmp = l2_tmp * l4;
  l218 = l218_tmp * l22 * 39.0;
  l21 = l8 * l8;
  l35 = rt_powd_snf(l30, 3.0);
  l38 = rt_powd_snf(l30, 5.0);
  l40 = l7 * l23;
  l41 = l6 * l24;
  l52 = l8 * l22 * 6.0;
  l62_tmp = V_init * l4;
  l62 = l62_tmp * l24 * 6.0;
  l67_tmp = J_max + -J_min;
  l67.re = l67_tmp;
  l67.im = 0.0;
  coder::internal::scalar::b_sqrt(&l67);
  l74 = l11 * l19 * 20.0;
  l75_tmp = V_max + -V_min;
  l75.re = l75_tmp;
  l75.im = 0.0;
  coder::internal::scalar::b_sqrt(&l75);
  l102 = V_max * l5 * l23 * 12.0;
  l104_tmp = V_min * l8;
  l104 = l104_tmp * l20 * 36.0;
  l147_tmp = V_init * l8;
  l147 = l147_tmp * l20 * 126.0;
  l148_tmp = V_init * l11;
  l148 = l148_tmp * l18 * 210.0;
  l153_tmp = V_min * l11;
  l153 = l153_tmp * l18 * 120.0;
  l219 = l2_tmp * l9 * l11 * 81.0;
  l263_tmp = l2_tmp * l8;
  l263 = l263_tmp * l19 * 165.0;
  l51 = l9 * l21 * 6.0;
  l68 = coder::d_power(l67);
  l69 = coder::f_power(l67);
  l76 = coder::d_power(l75);
  l96 = V_init * l7 * l21 * 42.0;
  l103 = V_min * l7 * l21 * 36.0;
  if ((l67.im == 0.0) && (l67.re >= 0.0)) {
    l125.re = rt_powd_snf(l67.re, 11.0);
    l125.im = 0.0;
  } else if (l67.re == 0.0) {
    l125.re = 0.0;
    l125.im = -rt_powd_snf(l67.im, 11.0);
  } else {
    if (l67.im == 0.0) {
      if (l67.re < 0.0) {
        l125.re = std::log(std::abs(l67.re));
        l125.im = 3.1415926535897931;
      } else {
        l125.re = std::log(l67.re);
        l125.im = 0.0;
      }
    } else if ((std::abs(l67.re) > 8.9884656743115785E+307) ||
               (std::abs(l67.im) > 8.9884656743115785E+307)) {
      l125.re = std::log(rt_hypotd_snf(l67.re / 2.0, l67.im / 2.0)) +
                0.69314718055994529;
      l125.im = rt_atan2d_snf(l67.im, l67.re);
    } else {
      l125.re = std::log(rt_hypotd_snf(l67.re, l67.im));
      l125.im = rt_atan2d_snf(l67.im, l67.re);
    }
    l125.re *= 11.0;
    l125.im *= 11.0;
    if (l125.im == 0.0) {
      d = l125.re;
      l125.re = std::exp(d);
      l125.im = 0.0;
    } else if (rtIsInf(l125.im) && rtIsInf(l125.re) && (l125.re < 0.0)) {
      l125.re = 0.0;
      l125.im = 0.0;
    } else {
      r = std::exp(l125.re / 2.0);
      d = l125.im;
      d1 = l125.im;
      l125.re = r * (r * std::cos(d));
      l125.im = r * (r * std::sin(d1));
    }
  }
  l170 = l2_tmp * l5 * l21 * 3.0;
  l294_tmp = J_max * V_init;
  l294.re = (l2_tmp + J_max * V_max * 2.0) + -(l294_tmp * 2.0);
  l294.im = 0.0;
  coder::internal::scalar::b_sqrt(&l294);
  re_tmp = 1.4142135623730951 * l32 * l48;
  re = re_tmp * l67.re;
  im = re_tmp * l67.im;
  l312.re = 3.0 * (re * l75.re - im * l75.im);
  l312.im = 3.0 * (re * l75.im + im * l75.re);
  re_tmp = 1.4142135623730951 * l39 * l43;
  re = re_tmp * l67.re;
  im = re_tmp * l67.im;
  l349.re = 3.0 * (re * l75.re - im * l75.im);
  l349.im = 3.0 * (re * l75.im + im * l75.re);
  re_tmp = 1.4142135623730951 * l38 * l44;
  re = re_tmp * l67.re;
  im = re_tmp * l67.im;
  l409.re = 15.0 * (re * l75.re - im * l75.im);
  l409.im = 15.0 * (re * l75.im + im * l75.re);
  re_tmp = 1.4142135623730951 * l35 * l47;
  re = re_tmp * l67.re;
  im = re_tmp * l67.im;
  l410_re = 15.0 * (re * l75.re - im * l75.im);
  l410_im = 15.0 * (re * l75.im + im * l75.re);
  re_tmp = 1.4142135623730951 * l37 * l45;
  re = re_tmp * l67.re;
  im = re_tmp * l67.im;
  l411_re = 30.0 * (re * l75.re - im * l75.im);
  l411_im = 30.0 * (re * l75.im + im * l75.re);
  re_tmp = 1.4142135623730951 * l36 * l46;
  re = re_tmp * l67.re;
  im = re_tmp * l67.im;
  l412_re = 30.0 * (re * l75.re - im * l75.im);
  l412_im = 30.0 * (re * l75.im + im * l75.re);
  l124 = coder::d_power(l68);
  l295 = coder::d_power(l294);
  r = l4 * l49;
  l4_re = r * l67.re;
  l4_im = r * l67.im;
  l310_re = 3.0 * (l4_re * l294.re - l4_im * l294.im);
  l310_im = 3.0 * (l4_re * l294.im + l4_im * l294.re);
  l21_re_tmp = l21 * l43;
  l21_re = l21_re_tmp * l67.re;
  l21_im = l21_re_tmp * l67.im;
  l315_re = 3.0 * (l21_re * l294.re - l21_im * l294.im);
  l315_im = 3.0 * (l21_re * l294.im + l21_im * l294.re);
  l12_re_tmp = l12 * l44;
  l12_re = l12_re_tmp * l67.re;
  l12_im = l12_re_tmp * l67.im;
  l319_re = 18.0 * (l12_re * l294.re - l12_im * l294.im);
  l319_im = 18.0 * (l12_re * l294.im + l12_im * l294.re);
  r = l6 * l48;
  l6_re = r * l67.re;
  l6_im = r * l67.im;
  l322_re = 18.0 * (l6_re * l294.re - l6_im * l294.im);
  l322_im = 18.0 * (l6_re * l294.im + l6_im * l294.re);
  r = l11 * l45;
  l11_re = r * l67.re;
  l11_im = r * l67.im;
  l324_re = 45.0 * (l11_re * l294.re - l11_im * l294.im);
  l324_im = 45.0 * (l11_re * l294.im + l11_im * l294.re);
  r = l8 * l47;
  l8_re = r * l67.re;
  l8_im = r * l67.im;
  l326_re = 45.0 * (l8_re * l294.re - l8_im * l294.im);
  l326_im = 45.0 * (l8_re * l294.im + l8_im * l294.re);
  r = l10 * l46;
  l10_re = r * l67.re;
  l10_im = r * l67.im;
  l328_re = 60.0 * (l10_re * l294.re - l10_im * l294.im);
  l328_im = 60.0 * (l10_re * l294.im + l10_im * l294.re);
  r = l5 * 1.4142135623730951 * l39;
  l21_re_tmp = r * l75.re;
  r *= l75.im;
  l361_re = 6.0 * (l21_re_tmp * l294.re - r * l294.im);
  l361_im = 6.0 * (l21_re_tmp * l294.im + r * l294.re);
  r = l22 * 1.4142135623730951 * l31;
  l21_re_tmp = r * l75.re;
  r *= l75.im;
  l419_re = 42.0 * (l21_re_tmp * l294.re - r * l294.im);
  l419_im = 42.0 * (l21_re_tmp * l294.im + r * l294.re);
  r = l9 * 1.4142135623730951 * l37;
  l21_re_tmp = r * l75.re;
  r *= l75.im;
  l484_re = 126.0 * (l21_re_tmp * l294.re - r * l294.im);
  l484_im = 126.0 * (l21_re_tmp * l294.im + r * l294.re);
  r = l19 * 1.4142135623730951 * l35;
  l21_re_tmp = r * l75.re;
  r *= l75.im;
  l527_re = 210.0 * (l21_re_tmp * l294.re - r * l294.im);
  l527_im = 210.0 * (l21_re_tmp * l294.im + r * l294.re);
  re_tmp = 1.4142135623730951 * l31 * l44;
  re = re_tmp * l75.re;
  im = re_tmp * l75.im;
  l578_tmp = A_init * l11;
  b_l578_tmp = A_init * l4;
  l21_re_tmp = T * l12;
  c_l578_tmp = A_init * l8;
  l21_im = T * l11;
  l21_re = T * l8;
  l12_re_tmp = T * l10;
  d_l578_tmp = A_init * l7;
  l578.re =
      ((((((((((((((((((((((((((l82 + -(T * l40 * 3.0)) + -(T * l41 * 3.0)) +
                              T * l9 * l21 * 18.0) +
                             l21_re * l22 * 18.0) +
                            l21_im * l19 * 60.0) +
                           -(d_l578_tmp * l21 * 3.0)) +
                          -(b_l578_tmp * l24 * 3.0)) +
                         l131) +
                        l135) +
                       -(l21_re_tmp * l18 * 45.0)) +
                      -(l12_re_tmp * l20 * 45.0)) +
                     -(l578_tmp * l18 * 45.0)) +
                    -(c_l578_tmp * l20 * 45.0)) +
                   l310_re) +
                  l312.re) +
                 l315_re) +
                l319_re) +
               l322_re) +
              l324_re) +
             l326_re) +
            l328_re) +
           l349.re) +
          (re * l125.re - im * l125.im) * 3.0) +
         l409.re) +
        l410_re) +
       l411_re) +
      l412_re;
  l578.im = ((((((((((((l310_im + l312.im) + l315_im) + l319_im) + l322_im) +
                    l324_im) +
                   l326_im) +
                  l328_im) +
                 l349.im) +
                (re * l125.im + im * l125.re) * 3.0) +
               l409.im) +
              l410_im) +
             l411_im) +
            l412_im;
  l575 =
      1.0 /
      ((((((l40 + l41) + -l51) + -l52) + l12 * l18 * 15.0) + l10 * l20 * 15.0) +
       -l74);
  A_init_re_tmp_tmp = A_init * J_max;
  A_init_re_tmp = A_init_re_tmp_tmp * l49;
  A_init_re = A_init_re_tmp * l67.re;
  A_init_im = A_init_re_tmp * l67.im;
  b_A_init_re_tmp_tmp = A_init * 1.4142135623730951 * l31;
  A_init_re_tmp = b_A_init_re_tmp_tmp * l48;
  b_A_init_re = A_init_re_tmp * l67.re;
  b_A_init_im = A_init_re_tmp * l67.im;
  r = T * 1.4142135623730951 * l32;
  A_init_re_tmp = r * l48;
  T_re = A_init_re_tmp * l67.re;
  T_im = A_init_re_tmp * l67.im;
  A_init_re_tmp = A_init * l12 * l43;
  c_A_init_re = A_init_re_tmp * l67.re;
  c_A_init_im = A_init_re_tmp * l67.im;
  A_init_re_tmp = T * l4 * l49;
  b_T_re = A_init_re_tmp * l67.re;
  b_T_im = A_init_re_tmp * l67.im;
  A_init_re_tmp = A_init * 1.4142135623730951 * l38 * l43;
  d_A_init_re = A_init_re_tmp * l67.re;
  d_A_init_im = A_init_re_tmp * l67.im;
  A_init_re_tmp = b_A_init_re_tmp_tmp * l44;
  e_A_init_re = A_init_re_tmp * l75.re;
  e_A_init_im = A_init_re_tmp * l75.im;
  A_init_re_tmp = A_init * 1.4142135623730951 * l30 * l45;
  f_A_init_re = A_init_re_tmp * l75.re;
  f_A_init_im = A_init_re_tmp * l75.im;
  A_init_re_tmp = A_init * 1.4142135623730951 * l32 * l47;
  g_A_init_re = A_init_re_tmp * l67.re;
  g_A_init_im = A_init_re_tmp * l67.im;
  A_init_re_tmp = T * 1.4142135623730951 * l39 * l43;
  c_T_re = A_init_re_tmp * l67.re;
  c_T_im = A_init_re_tmp * l67.im;
  A_init_re_tmp = r * l44;
  d_T_re = A_init_re_tmp * l75.re;
  d_T_im = A_init_re_tmp * l75.im;
  A_init_re_tmp = T * 1.4142135623730951 * l31 * l45;
  e_T_re = A_init_re_tmp * l75.re;
  e_T_im = A_init_re_tmp * l75.im;
  A_init_re_tmp = l578_tmp * l44;
  h_A_init_re = A_init_re_tmp * l67.re;
  h_A_init_im = A_init_re_tmp * l67.im;
  A_init_re_tmp = b_l578_tmp * l48;
  i_A_init_re = A_init_re_tmp * l67.re;
  i_A_init_im = A_init_re_tmp * l67.im;
  A_init_re_tmp = T * l21 * l43;
  f_T_re = A_init_re_tmp * l67.re;
  f_T_im = A_init_re_tmp * l67.im;
  A_init_re_tmp = l21_re_tmp * l44;
  g_T_re = A_init_re_tmp * l67.re;
  g_T_im = A_init_re_tmp * l67.im;
  A_init_re_tmp = T * l6 * l48;
  h_T_re = A_init_re_tmp * l67.re;
  h_T_im = A_init_re_tmp * l67.im;
  A_init_re_tmp = A_init * 1.4142135623730951 * l37 * l44;
  j_A_init_re = A_init_re_tmp * l67.re;
  j_A_init_im = A_init_re_tmp * l67.im;
  A_init_re_tmp = A_init * 1.4142135623730951 * l36 * l45;
  k_A_init_re = A_init_re_tmp * l67.re;
  k_A_init_im = A_init_re_tmp * l67.im;
  A_init_re_tmp = A_init * 1.4142135623730951 * l35 * l46;
  l_A_init_re = A_init_re_tmp * l67.re;
  l_A_init_im = A_init_re_tmp * l67.im;
  r = l24 * 1.4142135623730951 * l30;
  l24_re = r * l75.re;
  l24_im = r * l75.im;
  A_init_re_tmp = T * 1.4142135623730951 * l38 * l44;
  i_T_re = A_init_re_tmp * l67.re;
  i_T_im = A_init_re_tmp * l67.im;
  A_init_re_tmp = T * 1.4142135623730951 * l35 * l47;
  j_T_re = A_init_re_tmp * l67.re;
  j_T_im = A_init_re_tmp * l67.im;
  A_init_re_tmp = T * 1.4142135623730951 * l37 * l45;
  k_T_re = A_init_re_tmp * l67.re;
  k_T_im = A_init_re_tmp * l67.im;
  A_init_re_tmp = T * 1.4142135623730951 * l36 * l46;
  l_T_re = A_init_re_tmp * l67.re;
  l_T_im = A_init_re_tmp * l67.im;
  A_init_re_tmp = l135_tmp * l45;
  m_A_init_re = A_init_re_tmp * l67.re;
  m_A_init_im = A_init_re_tmp * l67.im;
  A_init_re_tmp = l131_tmp * l47;
  n_A_init_re = A_init_re_tmp * l67.re;
  n_A_init_im = A_init_re_tmp * l67.im;
  A_init_re_tmp = c_l578_tmp * l46;
  o_A_init_re = A_init_re_tmp * l67.re;
  o_A_init_im = A_init_re_tmp * l67.im;
  A_init_re_tmp = l21_im * l45;
  re_tmp = A_init_re_tmp * l67.re;
  b_A_init_re_tmp_tmp = A_init_re_tmp * l67.im;
  A_init_re_tmp = l21_re * l47;
  l135_tmp = A_init_re_tmp * l67.re;
  l131_tmp = A_init_re_tmp * l67.im;
  A_init_re_tmp = l12_re_tmp * l46;
  l39 = A_init_re_tmp * l67.re;
  l12_im = A_init_re_tmp * l67.im;
  r = l7 * 1.4142135623730951 * l38;
  l12_re = r * l75.re;
  l21_re = r * l75.im;
  r = l20 * 1.4142135623730951 * l32;
  l12_re_tmp = r * l75.re;
  l21_re_tmp = r * l75.im;
  r = l18 * 1.4142135623730951 * l36;
  l21_im = r * l75.re;
  r *= l75.im;
  d = A_init * T;
  d1 = V_max * l6;
  d2 = J_max * l2_tmp;
  d3 = V_min * l6;
  d4 = V_max * l4;
  d5 = V_init * l6;
  d6 = V_max * l10;
  l586_re_tmp = V_max * l11;
  b_l586_re_tmp = V_max * l8;
  c_l586_re_tmp = V_min * l10;
  d_l586_re_tmp = V_init * l10;
  e_l586_re_tmp = l2_tmp * l10;
  f_l586_re_tmp = d * l11;
  g_l586_re_tmp = d * l10;
  h_l586_re_tmp = d * l8;
  i_l586_re_tmp = l2_tmp * l6;
  j_l586_re_tmp = d * l4;
  k_l586_re_tmp = d * l6;
    l586_re = (((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((l62 + -(V_init * l5 * l23 * 6.0)) + l96) + -(V_min * l5 * l23 * 6.0)) + -(d3 * l22 * 6.0)) + -(d4 * l24 * 6.0)) + l102) + l103) + l104) + d1 * l22 * 48.0) + V_max * l9 * l12 * 216.0) + d * l9 * l12 * 36.0) + -(d2 * l24 * 6.0)) + -(d5 * l22 * 42.0)) + -(V_init * l9 * l12 * 126.0)) + l147) + l148) + -(V_min * l9 * l12 * 90.0)) + l153) + d6 * l19 * 300.0) + -(d * l7 * l21 * 6.0)) + -(j_l586_re_tmp * l24 * 6.0)) + k_l586_re_tmp * l22 * 36.0) + l170) + -(d_l586_re_tmp * l19 * 210.0)) + -(V_max * l7 * l21 * 78.0)) + -(c_l586_re_tmp * l19 * 90.0)) + -(b_l586_re_tmp * l20 * 162.0)) + -(l586_re_tmp * l18 * 330.0)) + g_l586_re_tmp * l19 * 120.0) + -(l2_tmp * l7 * l12 * 24.0)) + l218) + l219) + -(l13 * l40 * 3.0)) + -(l13 * l41 * 3.0)) + l9 * l13 * l21 * 18.0) + l8 * l13 * l22 * 18.0) + l11 * l13 * l19 * 60.0) + -(f_l586_re_tmp * l18 * 90.0)) + -(h_l586_re_tmp * l20 * 90.0)) + l263) + -(l12 * l13 * l18 * 45.0)) + -(l10 * l13 * l20 * 45.0)) + -(i_l586_re_tmp * l20 * 108.0)) + -(e_l586_re_tmp * l18 * 150.0)) + (A_init_re * l294.re - A_init_im * l294.im) * 6.0) + (b_A_init_re * l75.re - b_A_init_im * l75.im) * 6.0) + (T_re * l75.re - T_im * l75.im) * 6.0) + (c_A_init_re * l294.re - c_A_init_im * l294.im) * 6.0) + (b_T_re * l294.re - b_T_im * l294.im) * 6.0) + (d_A_init_re * l75.re - d_A_init_im * l75.im) * 6.0) + (e_A_init_re * l124.re - e_A_init_im * l124.im) * 6.0) + (f_A_init_re * l124.re - f_A_init_im * l124.im) * 6.0) + (g_A_init_re * l75.re - g_A_init_im * l75.im) * 30.0) + l361_re) + (c_T_re * l75.re - c_T_im * l75.im) * 6.0) + (d_T_re * l124.re - d_T_im * l124.im) * 6.0) + (e_T_re * l124.re - e_T_im * l124.im) * 6.0) + (h_A_init_re * l294.re - h_A_init_im * l294.im) * 36.0) + (i_A_init_re * l294.re - i_A_init_im * l294.im) * 36.0) + (f_T_re * l294.re - f_T_im * l294.im) * 6.0) + (g_T_re * l294.re - g_T_im * l294.im) * 36.0) + (h_T_re * l294.re - h_T_im * l294.im) * 36.0) + (j_A_init_re * l75.re - j_A_init_im * l75.im) * 30.0) + (k_A_init_re * l75.re - k_A_init_im * l75.im) * 60.0) + (l_A_init_re * l75.re - l_A_init_im * l75.im) * 60.0) + -((l24_re * l294.re - l24_im * l294.im) * 6.0)) + l419_re) + (i_T_re * l75.re - i_T_im * l75.im) * 30.0) + (j_T_re * l75.re - j_T_im * l75.im) * 30.0) + (k_T_re * l75.re - k_T_im * l75.im) * 60.0) + (l_T_re * l75.re - l_T_im * l75.im) * 60.0) + (m_A_init_re * l294.re - m_A_init_im * l294.im) * 90.0) + (n_A_init_re * l294.re - n_A_init_im * l294.im) * 90.0) + (o_A_init_re * l294.re - o_A_init_im * l294.im) * 120.0) + (re_tmp * l294.re - b_A_init_re_tmp_tmp * l294.im) * 90.0) + (l135_tmp * l294.re - l131_tmp * l294.im) * 90.0) + (l39 * l294.re - l12_im * l294.im) * 120.0) + -((l12_re * l294.re - l21_re * l294.im) * 42.0)) + l484_re) + -((l12_re_tmp * l294.re - l21_re_tmp * l294.im) * 126.0)) + l527_re) + -((l21_im * l294.re - r * l294.im) * 210.0);
    l586_im =
        (((((((((((((((((((((((((((((((((((((A_init_re * l294.im +
                                             A_init_im * l294.re) *
                                                6.0 +
                                            (b_A_init_re * l75.im +
                                             b_A_init_im * l75.re) *
                                                6.0) +
                                           (T_re * l75.im + T_im * l75.re) *
                                               6.0) +
                                          (c_A_init_re * l294.im +
                                           c_A_init_im * l294.re) *
                                              6.0) +
                                         (b_T_re * l294.im + b_T_im * l294.re) *
                                             6.0) +
                                        (d_A_init_re * l75.im +
                                         d_A_init_im * l75.re) *
                                            6.0) +
                                       (e_A_init_re * l124.im +
                                        e_A_init_im * l124.re) *
                                           6.0) +
                                      (f_A_init_re * l124.im +
                                       f_A_init_im * l124.re) *
                                          6.0) +
                                     (g_A_init_re * l75.im +
                                      g_A_init_im * l75.re) *
                                         30.0) +
                                    l361_im) +
                                   (c_T_re * l75.im + c_T_im * l75.re) * 6.0) +
                                  (d_T_re * l124.im + d_T_im * l124.re) * 6.0) +
                                 (e_T_re * l124.im + e_T_im * l124.re) * 6.0) +
                                (h_A_init_re * l294.im +
                                 h_A_init_im * l294.re) *
                                    36.0) +
                               (i_A_init_re * l294.im + i_A_init_im * l294.re) *
                                   36.0) +
                              (f_T_re * l294.im + f_T_im * l294.re) * 6.0) +
                             (g_T_re * l294.im + g_T_im * l294.re) * 36.0) +
                            (h_T_re * l294.im + h_T_im * l294.re) * 36.0) +
                           (j_A_init_re * l75.im + j_A_init_im * l75.re) *
                               30.0) +
                          (k_A_init_re * l75.im + k_A_init_im * l75.re) *
                              60.0) +
                         (l_A_init_re * l75.im + l_A_init_im * l75.re) * 60.0) +
                        -((l24_re * l294.im + l24_im * l294.re) * 6.0)) +
                       l419_im) +
                      (i_T_re * l75.im + i_T_im * l75.re) * 30.0) +
                     (j_T_re * l75.im + j_T_im * l75.re) * 30.0) +
                    (k_T_re * l75.im + k_T_im * l75.re) * 60.0) +
                   (l_T_re * l75.im + l_T_im * l75.re) * 60.0) +
                  (m_A_init_re * l294.im + m_A_init_im * l294.re) * 90.0) +
                 (n_A_init_re * l294.im + n_A_init_im * l294.re) * 90.0) +
                (o_A_init_re * l294.im + o_A_init_im * l294.re) * 120.0) +
               (re_tmp * l294.im + b_A_init_re_tmp_tmp * l294.re) * 90.0) +
              (l135_tmp * l294.im + l131_tmp * l294.re) * 90.0) +
             (l39 * l294.im + l12_im * l294.re) * 120.0) +
            -((l12_re * l294.im + l21_re * l294.re) * 42.0)) +
           l484_im) +
          -((l12_re_tmp * l294.im + l21_re_tmp * l294.re) * 126.0)) +
         l527_im) +
        -((l21_im * l294.im + r * l294.re) * 210.0);
    l576 = l575 * l575;
    l12_re_tmp = l575 * l578.re;
    r = l575 * l578.im;
    if (r == 0.0) {
      l575_re = l12_re_tmp / 3.0;
      l575_im = 0.0;
    } else if (l12_re_tmp == 0.0) {
      l575_re = 0.0;
      l575_im = r / 3.0;
    } else {
      l575_re = l12_re_tmp / 3.0;
      l575_im = r / 3.0;
    }
    dc = coder::d_power(l578);
    r = rt_powd_snf(l575, 3.0);
    l12_re_tmp = r * dc.re;
    r *= dc.im;
    if (r == 0.0) {
      l584_re = l12_re_tmp / 27.0;
      l584_im = 0.0;
    } else if (l12_re_tmp == 0.0) {
      l584_re = 0.0;
      l584_im = r / 27.0;
    } else {
      l584_re = l12_re_tmp / 27.0;
      l584_im = r / 27.0;
    }
    l21_im = l576 * l578.re;
    l21_re = l576 * l578.im;
    r = l21_im * l586_re - l21_re * l586_im;
    l21_re = l21_im * l586_im + l21_re * l586_re;
    if (l21_re == 0.0) {
      l588_re = r / 6.0;
      l588_im = 0.0;
    } else if (r == 0.0) {
      l588_re = 0.0;
      l588_im = l21_re / 6.0;
    } else {
      l588_re = r / 6.0;
      l588_im = l21_re / 6.0;
    }
    l48_re = l48 * l68.re;
    l48_im = l48 * l68.im;
    l49_re = l49 * l67.re;
    l49_im = l49 * l67.im;
    r = J_max * l47;
    J_max_re = r * l68.re;
    J_max_im = r * l68.im;
    r = l294_tmp * l49;
    b_J_max_re = r * l67.re;
    b_J_max_im = r * l67.im;
    r = l11 * l42;
    l11_re = r * l68.re;
    l11_im = r * l68.im;
    l45_re = l45 * l124.re;
    l45_im = l45 * l124.im;
    r = J_max * l48;
    c_J_max_re = r * l67.re;
    c_J_max_im = r * l67.im;
    A_init_re_tmp = l2_tmp * l48;
    l2_re = A_init_re_tmp * l68.re;
    l2_im = A_init_re_tmp * l68.im;
    r = l11 * l43;
    b_l11_re = r * l67.re;
    b_l11_im = r * l67.im;
    r = l10 * l43;
    l10_re = r * l68.re;
    l10_im = r * l68.im;
    r = J_max * l44;
    d_J_max_re = r * l124.re;
    d_J_max_im = r * l124.im;
    r = l294_tmp * l44;
    e_J_max_re = r * l125.re;
    e_J_max_im = r * l125.im;
    A_init_re_tmp = l2_tmp * l49;
    b_l2_re = A_init_re_tmp * l67.re;
    b_l2_im = A_init_re_tmp * l67.im;
    r = l8 * l44;
    l8_re = r * l68.re;
    l8_im = r * l68.im;
    r = l10 * l44;
    b_l10_re = r * l67.re;
    b_l10_im = r * l67.im;
    r = l4 * l46;
    l4_re = r * l68.re;
    l4_im = r * l68.im;
    r = l6 * l45;
    l6_re = r * l68.re;
    l6_im = r * l68.im;
    r = l8 * l45;
    b_l8_re = r * l67.re;
    b_l8_im = r * l67.im;
    r = l4 * l47;
    b_l4_re = r * l67.re;
    b_l4_im = r * l67.im;
    r = l6 * l46;
    b_l6_re = r * l67.re;
    b_l6_im = r * l67.im;
    r = V_min * 1.4142135623730951 * l32 * l47;
    V_min_re = r * l67.re;
    V_min_im = r * l67.im;
    l12_re_tmp = V_max * 1.4142135623730951 * l31;
    l12_im = l12_re_tmp * l47;
    V_max_re = l12_im * l68.re;
    V_max_im = l12_im * l68.im;
    l12_im = l12_re_tmp * l48;
    b_V_max_re = l12_im * l67.re;
    b_V_max_im = l12_im * l67.im;
    b_A_init_re_tmp_tmp = A_init_re_tmp_tmp * T;
    A_init_re_tmp = b_A_init_re_tmp_tmp * l49;
    A_init_re = A_init_re_tmp * l67.re;
    A_init_im = A_init_re_tmp * l67.im;
    r = V_init * l12 * l43;
    V_init_re = r * l67.re;
    V_init_im = r * l67.im;
    l21_re_tmp = V_min * l12;
    r = l21_re_tmp * l43;
    b_V_min_re = r * l67.re;
    b_V_min_im = r * l67.im;
    l21_re = V_max * l12;
    l12_im = l21_re * l42;
    c_V_max_re = l12_im * l68.re;
    c_V_max_im = l12_im * l68.im;
    l12_re = V_min * l4;
    r = l12_re * l48;
    c_V_min_re = r * l67.re;
    c_V_min_im = r * l67.im;
    l12_im = d1 * l46;
    d_V_max_re = l12_im * l68.re;
    d_V_max_im = l12_im * l68.im;
    re_tmp = 1.4142135623730951 * l38 * l42;
    re = re_tmp * l68.re;
    im = re_tmp * l68.im;
    re_tmp = 1.4142135623730951 * l35 * l45;
    b_re = re_tmp * l68.re;
    b_im = re_tmp * l68.im;
    re_tmp = 1.4142135623730951 * l37 * l43;
    c_re = re_tmp * l68.re;
    c_im = re_tmp * l68.im;
    re_tmp = 1.4142135623730951 * l36 * l44;
    d_re = re_tmp * l68.re;
    d_im = re_tmp * l68.im;
    r = l4 * l43;
    c_l4_re = r * l124.re;
    c_l4_im = r * l124.im;
    l21_im = V_init * 1.4142135623730951 * l31;
    r = l21_im * l47;
    b_V_init_re = r * l68.re;
    b_V_init_im = r * l68.im;
    r = V_min * 1.4142135623730951 * l38 * l43;
    d_V_min_re = r * l67.re;
    d_V_min_im = r * l67.im;
    l12_im = V_max * 1.4142135623730951 * l38 * l42;
    e_V_max_re = l12_im * l68.re;
    e_V_max_im = l12_im * l68.im;
    l12_im = l12_re_tmp * l44;
    f_V_max_re = l12_im * l75.re;
    f_V_max_im = l12_im * l75.im;
    l12_re_tmp = V_max * 1.4142135623730951 * l37;
    l12_im = l12_re_tmp * l44;
    g_V_max_re = l12_im * l67.re;
    g_V_max_im = l12_im * l67.im;
    l12_im = V_max * 1.4142135623730951 * l30 * l45;
    h_V_max_re = l12_im * l75.re;
    h_V_max_im = l12_im * l75.im;
    r = V_max * 1.4142135623730951 * l32;
    l12_im = r * l47;
    i_V_max_re = l12_im * l67.re;
    i_V_max_im = l12_im * l67.im;
    l12_im = r * l46;
    j_V_max_re = l12_im * l68.re;
    j_V_max_im = l12_im * l68.im;
    r = d2 * l47;
    f_J_max_re = r * l68.re;
    f_J_max_im = r * l68.im;
    r = l148_tmp * l44;
    c_V_init_re = r * l67.re;
    c_V_init_im = r * l67.im;
    r = l62_tmp * l48;
    d_V_init_re = r * l67.re;
    d_V_init_im = r * l67.im;
    r = l21_re_tmp * l42;
    e_V_min_re = r * l68.re;
    e_V_min_im = r * l68.im;
    l12_im = l21_re * l43;
    k_V_max_re = l12_im * l67.re;
    k_V_max_im = l12_im * l67.im;
    r = d3 * l46;
    f_V_min_re = r * l68.re;
    f_V_min_im = r * l68.im;
    l12_im = d4 * l48;
    l_V_max_re = l12_im * l67.re;
    l_V_max_im = l12_im * l67.im;
    l12_im = l586_re_tmp * l43;
    m_V_max_re = l12_im * l68.re;
    m_V_max_im = l12_im * l68.im;
    l12_im = b_l586_re_tmp * l45;
    n_V_max_re = l12_im * l68.re;
    n_V_max_im = l12_im * l68.im;
    r = l153_tmp * l44;
    g_V_min_re = r * l67.re;
    g_V_min_im = r * l67.im;
    r = d3 * l47;
    h_V_min_re = r * l67.re;
    h_V_min_im = r * l67.im;
    l12_im = d6 * l44;
    o_V_max_re = l12_im * l68.re;
    o_V_max_im = l12_im * l68.im;
    r = c_l586_re_tmp * l45;
    i_V_min_re = r * l67.re;
    i_V_min_im = r * l67.im;
    r = l104_tmp * l46;
    j_V_min_re = r * l67.re;
    j_V_min_im = r * l67.im;
    l39 = d * 1.4142135623730951;
    l21_re_tmp = l39 * l32;
    A_init_re_tmp = l21_re_tmp * l44;
    b_A_init_re = A_init_re_tmp * l69.re;
    b_A_init_im = A_init_re_tmp * l69.im;
    A_init_re_tmp = l39 * l30 * l46;
    c_A_init_re = A_init_re_tmp * l69.re;
    c_A_init_im = A_init_re_tmp * l69.im;
    re_tmp = 1.4142135623730951 * l32 * l43;
    e_re = re_tmp * l76.re;
    e_im = re_tmp * l76.im;
    A_init_re_tmp = l2_tmp * l44;
    c_l2_re = A_init_re_tmp * l125.re;
    c_l2_im = A_init_re_tmp * l125.im;
    A_init_re_tmp = l2_tmp * l45;
    d_l2_re = A_init_re_tmp * l124.re;
    d_l2_im = A_init_re_tmp * l124.im;
    r = V_init * 1.4142135623730951 * l38 * l42;
    e_V_init_re = r * l68.re;
    e_V_init_im = r * l68.im;
    r = l21_im * l43;
    f_V_init_re = r * l75.re;
    f_V_init_im = r * l75.im;
    r = l21_im * l44;
    g_V_init_re = r * l75.re;
    g_V_init_im = r * l75.im;
    r = V_init * 1.4142135623730951 * l30 * l45;
    h_V_init_re = r * l75.re;
    h_V_init_im = r * l75.im;
    r = V_init * 1.4142135623730951 * l32 * l46;
    i_V_init_re = r * l68.re;
    i_V_init_im = r * l68.im;
    r = V_min * 1.4142135623730951 * l37 * l44;
    k_V_min_re = r * l67.re;
    k_V_min_im = r * l67.im;
    r = V_min * 1.4142135623730951 * l35 * l46;
    l_V_min_re = r * l67.re;
    l_V_min_im = r * l67.im;
    l21_re = V_max * 1.4142135623730951 * l36;
    l12_im = l21_re * l45;
    p_V_max_re = l12_im * l67.re;
    p_V_max_im = l12_im * l67.im;
    l12_im = l12_re_tmp * l43;
    q_V_max_re = l12_im * l68.re;
    q_V_max_im = l12_im * l68.im;
    r = V_min * 1.4142135623730951 * l36 * l45;
    m_V_min_re = r * l67.re;
    m_V_min_im = r * l67.im;
    l12_re_tmp = V_max * 1.4142135623730951 * l35;
    l12_im = l12_re_tmp * l46;
    r_V_max_re = l12_im * l67.re;
    r_V_max_im = l12_im * l67.im;
    l12_im = l21_re * l44;
    s_V_max_re = l12_im * l68.re;
    s_V_max_im = l12_im * l68.im;
    l12_im = l12_re_tmp * l45;
    t_V_max_re = l12_im * l68.re;
    t_V_max_im = l12_im * l68.im;
    A_init_re_tmp = b_A_init_re_tmp_tmp * l45;
    d_A_init_re = A_init_re_tmp * l124.re;
    d_A_init_im = A_init_re_tmp * l124.im;
    r = d2 * l48;
    g_J_max_re = r * l67.re;
    g_J_max_im = r * l67.im;
    r = l62_tmp * l43;
    j_V_init_re = r * l125.re;
    j_V_init_im = r * l125.im;
    r = d_l586_re_tmp * l45;
    k_V_init_re = r * l67.re;
    k_V_init_im = r * l67.im;
    r = d5 * l47;
    l_V_init_re = r * l67.re;
    l_V_init_im = r * l67.im;
    r = l147_tmp * l46;
    m_V_init_re = r * l67.re;
    m_V_init_im = r * l67.im;
    l12_im = d1 * l43;
    u_V_max_re = l12_im * l124.re;
    u_V_max_im = l12_im * l124.im;
    l12_im = d4 * l44;
    v_V_max_re = l12_im * l124.re;
    v_V_max_im = l12_im * l124.im;
    r = l153_tmp * l43;
    n_V_min_re = r * l68.re;
    n_V_min_im = r * l68.im;
    r = l104_tmp * l45;
    o_V_min_re = r * l68.re;
    o_V_min_im = r * l68.im;
    l12_im = l586_re_tmp * l44;
    w_V_max_re = l12_im * l67.re;
    w_V_max_im = l12_im * l67.im;
    l12_im = d1 * l47;
    x_V_max_re = l12_im * l67.re;
    x_V_max_im = l12_im * l67.im;
    r = c_l586_re_tmp * l44;
    p_V_min_re = r * l68.re;
    p_V_min_im = r * l68.im;
    l12_im = d6 * l45;
    y_V_max_re = l12_im * l67.re;
    y_V_max_im = l12_im * l67.im;
    l12_im = b_l586_re_tmp * l46;
    ab_V_max_re = l12_im * l67.re;
    ab_V_max_im = l12_im * l67.im;
    A_init_re_tmp = A_init * l5 * 1.4142135623730951 * l38;
    e_A_init_re = A_init_re_tmp * l75.re;
    e_A_init_im = A_init_re_tmp * l75.im;
    b_A_init_re_tmp_tmp = l39 * l31;
    A_init_re_tmp = b_A_init_re_tmp_tmp * l45;
    f_A_init_re = A_init_re_tmp * l69.re;
    f_A_init_im = A_init_re_tmp * l69.im;
    A_init_re_tmp = b_A_init_re_tmp_tmp * l48;
    g_A_init_re = A_init_re_tmp * l67.re;
    g_A_init_im = A_init_re_tmp * l67.im;
    l12_im = l2_tmp * 1.4142135623730951 * l31;
    A_init_re_tmp = l12_im * l44;
    e_l2_re = A_init_re_tmp * l69.re;
    e_l2_im = A_init_re_tmp * l69.im;
    l21_im = l2_tmp * 1.4142135623730951 * l29_tmp;
    A_init_re_tmp = l21_im * l46;
    f_l2_re = A_init_re_tmp * l69.re;
    f_l2_im = A_init_re_tmp * l69.im;
    l12_re_tmp = l2_tmp * 1.4142135623730951 * l30;
    A_init_re_tmp = l12_re_tmp * l45;
    g_l2_re = A_init_re_tmp * l69.re;
    g_l2_im = A_init_re_tmp * l69.im;
    A_init_re_tmp = l12_re_tmp * l47;
    h_l2_re = A_init_re_tmp * l68.re;
    h_l2_im = A_init_re_tmp * l68.im;
    r = l13 * 1.4142135623730951 * l31 * l46;
    l13_re = r * l69.re;
    l13_im = r * l69.im;
    r = l13 * 1.4142135623730951 * l32 * l45;
    b_l13_re = r * l69.re;
    b_l13_im = r * l69.im;
    A_init_re_tmp = d * l12 * l43;
    h_A_init_re = A_init_re_tmp * l67.re;
    h_A_init_im = A_init_re_tmp * l67.im;
    A_init_re_tmp = e_l586_re_tmp * l43;
    i_l2_re = A_init_re_tmp * l68.re;
    i_l2_im = A_init_re_tmp * l68.im;
    r = V_init * 1.4142135623730951 * l37 * l43;
    n_V_init_re = r * l68.re;
    n_V_init_im = r * l68.im;
    r = V_init * 1.4142135623730951 * l36 * l44;
    o_V_init_re = r * l68.re;
    o_V_init_im = r * l68.im;
    r = V_init * 1.4142135623730951 * l35 * l45;
    p_V_init_re = r * l68.re;
    p_V_init_im = r * l68.im;
    r = d2 * l43;
    h_J_max_re = r * l125.re;
    h_J_max_im = r * l125.im;
    r = d2 * l44;
    i_J_max_re = r * l124.re;
    i_J_max_im = r * l124.im;
    r = d3 * l43;
    q_V_min_re = r * l124.re;
    q_V_min_im = r * l124.im;
    r = l12_re * l44;
    r_V_min_re = r * l124.re;
    r_V_min_im = r * l124.im;
    A_init_re_tmp = A_init * l24 * 1.4142135623730951 * l29_tmp;
    i_A_init_re = A_init_re_tmp * l75.re;
    i_A_init_im = A_init_re_tmp * l75.im;
    A_init_re_tmp = A_init * l22 * 1.4142135623730951 * l30;
    j_A_init_re = A_init_re_tmp * l75.re;
    j_A_init_im = A_init_re_tmp * l75.im;
    A_init_re_tmp = l39 * l38 * l43;
    k_A_init_re = A_init_re_tmp * l67.re;
    k_A_init_im = A_init_re_tmp * l67.im;
    A_init_re_tmp = l21_re_tmp * l47;
    l_A_init_re = A_init_re_tmp * l67.re;
    l_A_init_im = A_init_re_tmp * l67.im;
    l21_re = l2_tmp * 1.4142135623730951 * l37;
    A_init_re_tmp = l21_re * l42;
    j_l2_re = A_init_re_tmp * l68.re;
    j_l2_im = A_init_re_tmp * l68.im;
    A_init_re_tmp = l21_im * l45;
    k_l2_re = A_init_re_tmp * l75.re;
    k_l2_im = A_init_re_tmp * l75.im;
    A_init_re_tmp = l12_re_tmp * l48;
    l_l2_re = A_init_re_tmp * l67.re;
    l_l2_im = A_init_re_tmp * l67.im;
    A_init_re_tmp = l12_im * l46;
    m_l2_re = A_init_re_tmp * l68.re;
    m_l2_im = A_init_re_tmp * l68.im;
    l21_im = l2_tmp * 1.4142135623730951 * l32;
    A_init_re_tmp = l21_im * l45;
    n_l2_re = A_init_re_tmp * l68.re;
    n_l2_im = A_init_re_tmp * l68.im;
    A_init_re_tmp = T * l24 * 1.4142135623730951 * l30;
    T_re = A_init_re_tmp * l75.re;
    T_im = A_init_re_tmp * l75.im;
    r = l13 * 1.4142135623730951 * l35 * l44;
    c_l13_re = r * l69.re;
    c_l13_im = r * l69.im;
    A_init_re_tmp = f_l586_re_tmp * l44;
    m_A_init_re = A_init_re_tmp * l67.re;
    m_A_init_im = A_init_re_tmp * l67.im;
    A_init_re_tmp = j_l586_re_tmp * l48;
    n_A_init_re = A_init_re_tmp * l67.re;
    n_A_init_im = A_init_re_tmp * l67.im;
    A_init_re_tmp = g_l586_re_tmp * l45;
    o_A_init_re = A_init_re_tmp * l67.re;
    o_A_init_im = A_init_re_tmp * l67.im;
    A_init_re_tmp = k_l586_re_tmp * l47;
    p_A_init_re = A_init_re_tmp * l67.re;
    p_A_init_im = A_init_re_tmp * l67.im;
    A_init_re_tmp = h_l586_re_tmp * l46;
    q_A_init_re = A_init_re_tmp * l67.re;
    q_A_init_im = A_init_re_tmp * l67.im;
    A_init_re_tmp = l263_tmp * l44;
    o_l2_re = A_init_re_tmp * l68.re;
    o_l2_im = A_init_re_tmp * l68.im;
    A_init_re_tmp = l2_tmp * l11 * l43;
    p_l2_re = A_init_re_tmp * l67.re;
    p_l2_im = A_init_re_tmp * l67.im;
    A_init_re_tmp = i_l586_re_tmp * l45;
    q_l2_re = A_init_re_tmp * l68.re;
    q_l2_im = A_init_re_tmp * l68.im;
    A_init_re_tmp = l218_tmp * l46;
    r_l2_re = A_init_re_tmp * l68.re;
    r_l2_im = A_init_re_tmp * l68.im;
    A_init_re_tmp = d_l578_tmp * 1.4142135623730951 * l37;
    r_A_init_re = A_init_re_tmp * l75.re;
    r_A_init_im = A_init_re_tmp * l75.im;
    A_init_re_tmp = l82_tmp * 1.4142135623730951 * l36;
    s_A_init_re = A_init_re_tmp * l75.re;
    s_A_init_im = A_init_re_tmp * l75.im;
    A_init_re_tmp = A_init * l19 * 1.4142135623730951 * l32;
    t_A_init_re = A_init_re_tmp * l75.re;
    t_A_init_im = A_init_re_tmp * l75.im;
    A_init_re_tmp = l21_re_tmp * l43;
    u_A_init_re = A_init_re_tmp * l75.re;
    u_A_init_im = A_init_re_tmp * l75.im;
    A_init_re_tmp = b_A_init_re_tmp_tmp * l44;
    l48 = A_init_re_tmp * l75.re;
    l82_tmp = A_init_re_tmp * l75.im;
    A_init_re_tmp = l39 * l37 * l44;
    g_l586_re_tmp = A_init_re_tmp * l67.re;
    h_l586_re_tmp = A_init_re_tmp * l67.im;
    A_init_re_tmp = l39 * l36 * l45;
    l42 = A_init_re_tmp * l67.re;
    l30 = A_init_re_tmp * l67.im;
    A_init_re_tmp = l39 * l35 * l46;
    l148_tmp = A_init_re_tmp * l67.re;
    d_l586_re_tmp = A_init_re_tmp * l67.im;
    r = l2_tmp * 1.4142135623730951 * l36;
    A_init_re_tmp = r * l43;
    l586_re_tmp = A_init_re_tmp * l68.re;
    c_l586_re_tmp = A_init_re_tmp * l68.im;
    A_init_re_tmp = l21_re * l43;
    b_l586_re_tmp = A_init_re_tmp * l67.re;
    l62_tmp = A_init_re_tmp * l67.im;
    l21_re = l2_tmp * 1.4142135623730951 * l35;
    A_init_re_tmp = l21_re * l44;
    l147_tmp = A_init_re_tmp * l68.re;
    l153_tmp = A_init_re_tmp * l68.im;
    A_init_re_tmp = l12_im * l47;
    l104_tmp = A_init_re_tmp * l67.re;
    f_l586_re_tmp = A_init_re_tmp * l67.im;
    A_init_re_tmp = l21_im * l46;
    re_tmp = A_init_re_tmp * l67.re;
    l49 = A_init_re_tmp * l67.im;
    A_init_re_tmp = T * l7 * 1.4142135623730951 * l38;
    b_T_re = A_init_re_tmp * l75.re;
    b_T_im = A_init_re_tmp * l75.im;
    A_init_re_tmp = k_l586_re_tmp * l43;
    l135_tmp = A_init_re_tmp * l124.re;
    l131_tmp = A_init_re_tmp * l124.im;
    A_init_re_tmp = j_l586_re_tmp * l44;
    f_T_re = A_init_re_tmp * l124.re;
    f_T_im = A_init_re_tmp * l124.im;
    A_init_re_tmp = e_l586_re_tmp * l44;
    e_T_re = A_init_re_tmp * l67.re;
    e_T_im = A_init_re_tmp * l67.im;
    A_init_re_tmp = l263_tmp * l45;
    h_T_im = A_init_re_tmp * l67.re;
    l24_re = A_init_re_tmp * l67.im;
    A_init_re_tmp = l218_tmp * l47;
    g_T_im = A_init_re_tmp * l67.re;
    h_T_re = A_init_re_tmp * l67.im;
    A_init_re_tmp = i_l586_re_tmp * l46;
    l_T_im = A_init_re_tmp * l67.re;
    g_T_re = A_init_re_tmp * l67.im;
    A_init_re_tmp = A_init * l20 * 1.4142135623730951 * l31;
    j_T_im = A_init_re_tmp * l75.re;
    k_T_re = A_init_re_tmp * l75.im;
    A_init_re_tmp = l12_re_tmp * l43;
    k_T_im = A_init_re_tmp * l75.re;
    l_T_re = A_init_re_tmp * l75.im;
    A_init_re_tmp = l12_re_tmp * l44;
    i_T_im = A_init_re_tmp * l75.re;
    j_T_re = A_init_re_tmp * l75.im;
    A_init_re_tmp = l12_im * l43;
    l24_im = A_init_re_tmp * l75.re;
    i_T_re = A_init_re_tmp * l75.im;
    A_init_re_tmp = r * l44;
    l39 = A_init_re_tmp * l67.re;
    b_A_init_re_tmp_tmp = A_init_re_tmp * l67.im;
    A_init_re_tmp = l21_re * l45;
    l12_im = A_init_re_tmp * l67.re;
    l12_re = A_init_re_tmp * l67.im;
    A_init_re_tmp = T * l20 * 1.4142135623730951 * l32;
    c_T_re = A_init_re_tmp * l75.re;
    c_T_im = A_init_re_tmp * l75.im;
    A_init_re_tmp = l218_tmp * l43;
    l12_re_tmp = A_init_re_tmp * l124.re;
    l21_re = A_init_re_tmp * l124.im;
    A_init_re_tmp = A_init * l18 * 1.4142135623730951 * l35;
    l21_im = A_init_re_tmp * l75.re;
    l21_re_tmp = A_init_re_tmp * l75.im;
    A_init_re_tmp = T * l18 * 1.4142135623730951 * l36;
    d_T_re = A_init_re_tmp * l75.re;
    d_T_im = A_init_re_tmp * l75.im;
    d = A_init * V_init;
    d1 = A_init * V_min;
    d2 = A_init * V_max;
    d3 = T * V_min;
    d4 = T * V_init;
    d5 = J_max * T;
    d6 = T * V_max;
    r = T * l2_tmp;
    r = ((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((P_init * l9 * l12 * 36.0 + P_wayp * l7 * l21 * 6.0) + P_wayp * l4 * l24 * 6.0) + -(l3 * l24 * 6.0)) + A_init_re_tmp_tmp * V_init * l24 * 12.0) + -(P_init * l7 * l21 * 6.0)) + -(P_init * l4 * l24 * 6.0)) + P_init * l6 * l22 * 36.0) + -(P_wayp * l9 * l12 * 36.0)) + d * l7 * l12 * 48.0) + d1 * l7 * l12 * 36.0) + l3 * l5 * l12 * 3.0) + T * l62) + -(A_init_re_tmp_tmp * V_max * l24 * 12.0)) + J_max * l3 * l22 * 39.0) + P_init * l10 * l19 * 120.0) + -(P_wayp * l6 * l22 * 36.0)) + P_wayp * l11 * l18 * 90.0) + P_wayp * l8 * l20 * 90.0) + d5 * l2_tmp * l24 * -6.0) + -(d * l5 * l21 * 6.0)) + -(d1 * l5 * l21 * 6.0)) + -(d1 * l4 * l22 * 6.0)) + d2 * l5 * l21 * 12.0) + d1 * l6 * l20 * 36.0) + d2 * l9 * l11 * 252.0) + d4 * l5 * l23 * -6.0) + T * l96) + d3 * l5 * l23 * -6.0) + d3 * l6 * l22 * -6.0) + T * l102) + T * l103) + T * l104) + d6 * l9 * l12 * 252.0) + l14 * l51) + l14 * l52) + -(P_init * l11 * l18 * 90.0)) + -(P_init * l8 * l20 * 90.0)) + -(P_wayp * l10 * l19 * 120.0)) + -(d * l9 * l11 * 162.0)) + d * l6 * l20 * 216.0) + d * l10 * l18 * 300.0) + -(d2 * l7 * l12 * 84.0)) + d2 * l4 * l22 * 84.0) + -(d1 * l9 * l11 * 90.0)) + d1 * l10 * l18 * 120.0) + d2 * l8 * l19 * 420.0) + -(l3 * l7 * l11 * 24.0)) + l3 * l9 * l10 * 81.0) + d4 * l6 * l22 * -42.0) + d4 * l9 * l12 * -126.0) + T * l147) + T * l148) + -(d6 * l4 * l24 * 12.0)) + d6 * l6 * l22 * 84.0) + d3 * l9 * l12 * -90.0) + T * l153) + d6 * l10 * l19 * 420.0) + -(l14 * l40)) + -(l14 * l41)) + l14 * l74) + T * l170) + l13 * l82) + -(d * l4 * l22 * 78.0)) + -(d * l8 * l19 * 330.0)) + -(d1 * l8 * l19 * 90.0)) + -(d2 * l6 * l20 * 252.0)) + -(d2 * l10 * l18 * 420.0)) + l3 * l6 * l19 * 165.0) + d4 * l10 * l19 * -210.0) + -(d6 * l7 * l21 * 84.0)) + d3 * l10 * l19 * -90.0) + -(d6 * l8 * l20 * 252.0)) + -(d6 * l11 * l18 * 420.0)) + l12 * l14 * l18 * -15.0) + l10 * l14 * l20 * -15.0) + d_l578_tmp * l13 * l21 * -3.0) + b_l578_tmp * l13 * l24 * -3.0) + l13 * l131) + r * l7 * l12 * -24.0) + T * l218) + l13 * l135) + T * l219) + -(l3 * l4 * l20 * 108.0)) + -(l3 * l8 * l18 * 150.0)) + l578_tmp * l13 * l18 * -45.0) + c_l578_tmp * l13 * l20 * -45.0) + T * l263) + r * l6 * l20 * -108.0) + r * l10 * l18 * -150.0) + (l48_re * l295.re - l48_im * l295.im)) + (l49_re * l295.re - l49_im * l295.im) * 3.0) + (J_max_re * l295.re - J_max_im * l295.im) * 6.0) + (b_J_max_re * l294.re - b_J_max_im * l294.im) * 6.0) + (l11_re * l295.re - l11_im * l295.im)) + -(l45_re * l295.re - l45_im * l295.im)) + (c_J_max_re * l295.re - c_J_max_im * l295.im) * 18.0) + (l2_re * l294.re - l2_im * l294.im) * 3.0) + (b_l11_re * l295.re - b_l11_im * l295.im) * 3.0) + (l10_re * l295.re - l10_im * l295.im) * 6.0) + -((d_J_max_re * l295.re - d_J_max_im * l295.im) * 3.0)) + -((e_J_max_re * l294.re - e_J_max_im * l294.im) * 6.0)) + (b_l2_re * l294.re - b_l2_im * l294.im) * 21.0) + (l8_re * l295.re - l8_im * l295.im) * 15.0) + (b_l10_re * l295.re - b_l10_im * l295.im) * 18.0) + (l4_re * l295.re - l4_im * l295.im) * 15.0) + (l6_re * l295.re - l6_im * l295.im) * 20.0) + (b_l8_re * l295.re - b_l8_im * l295.im) * 45.0) + (b_l4_re * l295.re - b_l4_im * l295.im) * 45.0) + (b_l6_re * l295.re - b_l6_im * l295.im) * 60.0) + (V_min_re * l75.re - V_min_im * l75.im) * 6.0) + (V_max_re * l75.re - V_max_im * l75.im) * 6.0) + (b_V_max_re * l75.re - b_V_max_im * l75.im) * 6.0) + (A_init_re * l294.re - A_init_im * l294.im) * 18.0) + (V_init_re * l294.re - V_init_im * l294.im) * 6.0) + (b_V_min_re * l294.re - b_V_min_im * l294.im) * 6.0) + (c_V_max_re * l294.re - c_V_max_im * l294.im) * 6.0) + (c_V_min_re * l294.re - c_V_min_im * l294.im) * 6.0) + (d_V_max_re * l294.re - d_V_max_im * l294.im) * 6.0) + (re * l76.re - im * l76.im) * 2.0) + (b_re * l76.re - b_im * l76.im) * 2.0) + (c_re * l76.re - c_im * l76.im) * 6.0) + (d_re * l76.re - d_im * l76.im) * 6.0) + -((c_l4_re * l295.re - c_l4_im * l295.im) * 2.0)) + -((b_V_init_re * l75.re - b_V_init_im * l75.im) * 6.0)) + (d_V_min_re * l75.re - d_V_min_im * l75.im) * 6.0) + (e_V_max_re * l75.re - e_V_max_im * l75.im) * 6.0) + (f_V_max_re * l124.re - f_V_max_im * l124.im) * 6.0) + (g_V_max_re * l75.re - g_V_max_im * l75.im) * 6.0) + (h_V_max_re * l124.re - h_V_max_im * l124.im) * 6.0) + (i_V_max_re * l75.re - i_V_max_im * l75.im) * 24.0) + (j_V_max_re * l75.re - j_V_max_im * l75.im) * 30.0) + (f_J_max_re * l294.re - f_J_max_im * l294.im) * 15.0) + (c_V_init_re * l294.re - c_V_init_im * l294.im) * 36.0) + (d_V_init_re * l294.re - d_V_init_im * l294.im) * 36.0) + -((e_V_min_re * l294.re - e_V_min_im * l294.im) * 6.0)) + -((k_V_max_re * l294.re - k_V_max_im * l294.im) * 6.0)) + -((f_V_min_re * l294.re - f_V_min_im * l294.im) * 6.0)) + -((l_V_max_re * l294.re - l_V_max_im * l294.im) * 6.0)) + (m_V_max_re * l294.re - m_V_max_im * l294.im) * 24.0) + (n_V_max_re * l294.re - n_V_max_im * l294.im) * 24.0) + (g_V_min_re * l294.re - g_V_min_im * l294.im) * 30.0) + (h_V_min_re * l294.re - h_V_min_im * l294.im) * 30.0) + (o_V_max_re * l294.re - o_V_max_im * l294.im) * 36.0) + (i_V_min_re * l294.re - i_V_min_im * l294.im) * 60.0) + (j_V_min_re * l294.re - j_V_min_im * l294.im) * 60.0) + (b_A_init_re * l75.re - b_A_init_im * l75.im) * 6.0) + (c_A_init_re * l75.re - c_A_init_im * l75.im) * 6.0) + (e_re * l124.re - e_im * l124.im) * 8.0) + -((c_l2_re * l294.re - c_l2_im * l294.im) * 9.0)) + -((d_l2_re * l294.re - d_l2_im * l294.im) * 12.0)) + -((e_V_init_re * l75.re - e_V_init_im * l75.im) * 6.0)) + -((f_V_init_re * l125.re - f_V_init_im * l125.im) * 6.0)) + -((g_V_init_re * l124.re - g_V_init_im * l124.im) * 6.0)) + -((h_V_init_re * l124.re - h_V_init_im * l124.im) * 6.0)) + -((i_V_init_re * l75.re - i_V_init_im * l75.im) * 30.0)) + (k_V_min_re * l75.re - k_V_min_im * l75.im) * 24.0) + (l_V_min_re * l75.re - l_V_min_im * l75.im) * 24.0) + (p_V_max_re * l75.re - p_V_max_im * l75.im) * 24.0) + (q_V_max_re * l75.re - q_V_max_im * l75.im) * 30.0) + (m_V_min_re * l75.re - m_V_min_im * l75.im) * 36.0) + (r_V_max_re * l75.re - r_V_max_im * l75.im) * 36.0) + (s_V_max_re * l75.re - s_V_max_im * l75.im) * 60.0) + (t_V_max_re * l75.re - t_V_max_im * l75.im) * 60.0) + -((d_A_init_re * l294.re - d_A_init_im * l294.im) * 12.0)) + (g_J_max_re * l294.re - g_J_max_im * l294.im) * 126.0) + -((j_V_init_re * l294.re - j_V_init_im * l294.im) * 6.0)) + (k_V_init_re * l294.re - k_V_init_im * l294.im) * 90.0) + (l_V_init_re * l294.re - l_V_init_im * l294.im) * 90.0) + (m_V_init_re * l294.re - m_V_init_im * l294.im) * 120.0) + (u_V_max_re * l294.re - u_V_max_im * l294.im) * 12.0) + (v_V_max_re * l294.re - v_V_max_im * l294.im) * 12.0) + -((n_V_min_re * l294.re - n_V_min_im * l294.im) * 24.0)) + -((o_V_min_re * l294.re - o_V_min_im * l294.im) * 24.0)) + -((w_V_max_re * l294.re - w_V_max_im * l294.im) * 30.0)) + -((x_V_max_re * l294.re - x_V_max_im * l294.im) * 30.0)) + -((p_V_min_re * l294.re - p_V_min_im * l294.im) * 36.0)) + -((y_V_max_re * l294.re - y_V_max_im * l294.im) * 60.0)) + -((ab_V_max_re * l294.re - ab_V_max_im * l294.im) * 60.0)) + (e_A_init_re * l294.re - e_A_init_im * l294.im) * 6.0) + (f_A_init_re * l75.re - f_A_init_im * l75.im) * 12.0) + (g_A_init_re * l75.re - g_A_init_im * l75.im) * 18.0) + (e_l2_re * l75.re - e_l2_im * l75.im) * 3.0) + (f_l2_re * l75.re - f_l2_im * l75.im) * 3.0) + (g_l2_re * l75.re - g_l2_im * l75.im) * 6.0) + (h_l2_re * l75.re - h_l2_im * l75.im) * 6.0) + T * l361_re) + (l13_re * l75.re - l13_im * l75.im) * 3.0) + l13 * l312.re) + (b_l13_re * l75.re - b_l13_im * l75.im) * 6.0) + (h_A_init_re * l294.re - h_A_init_im * l294.im) * 18.0) + (i_l2_re * l294.re - i_l2_im * l294.im) * 3.0) + l13 * l310_re) + -((n_V_init_re * l75.re - n_V_init_im * l75.im) * 30.0)) + -((o_V_init_re * l75.re - o_V_init_im * l75.im) * 60.0)) + -((p_V_init_re * l75.re - p_V_init_im * l75.im) * 60.0)) + -((h_J_max_re * l294.re - h_J_max_im * l294.im) * 9.0)) + -((i_J_max_re * l294.re - i_J_max_im * l294.im) * 24.0)) + -((q_V_min_re * l294.re - q_V_min_im * l294.im) * 12.0)) + -((r_V_min_re * l294.re - r_V_min_im * l294.im) * 12.0)) + -((i_A_init_re * l294.re - i_A_init_im * l294.im) * 6.0)) + (j_A_init_re * l294.re - j_A_init_im * l294.im) * 42.0) + (k_A_init_re * l75.re - k_A_init_im * l75.im) * 18.0) + (l_A_init_re * l75.re - l_A_init_im * l75.im) * 90.0) + (j_l2_re * l75.re - j_l2_im * l75.im) * 3.0) + (k_l2_re * l124.re - k_l2_im * l124.im) * 3.0) + (l_l2_re * l75.re - l_l2_im * l75.im) * 24.0) + (m_l2_re * l75.re - m_l2_im * l75.im) * 27.0) + (n_l2_re * l75.re - n_l2_im * l75.im) * 48.0) + (T_re * l294.re - T_im * l294.im) * -6.0) + T * l419_re) + l13 * l349.re) + (c_l13_re * l75.re - c_l13_im * l75.im) * 3.0) + (m_A_init_re * l294.re - m_A_init_im * l294.im) * 108.0) + (n_A_init_re * l294.re - n_A_init_im * l294.im) * 108.0) + (o_A_init_re * l294.re - o_A_init_im * l294.im) * 270.0) + (p_A_init_re * l294.re - p_A_init_im * l294.im) * 270.0) + (q_A_init_re * l294.re - q_A_init_im * l294.im) * 360.0) + (o_l2_re * l294.re - o_l2_im * l294.im) * 15.0) + (p_l2_re * l294.re - p_l2_im * l294.im) * 21.0) + (q_l2_re * l294.re - q_l2_im * l294.im) * 30.0) + (r_l2_re * l294.re - r_l2_im * l294.im) * 30.0) + l13 * l315_re) + l13 * l319_re) + l13 * l322_re) + l13 * l324_re) + l13 * l326_re) + l13 * l328_re) + -((r_A_init_re * l294.re - r_A_init_im * l294.im) * 42.0)) + (s_A_init_re * l294.re - s_A_init_im * l294.im) * 126.0) + (t_A_init_re * l294.re - t_A_init_im * l294.im) * 210.0) + -((u_A_init_re * l124.re - u_A_init_im * l124.im) * 12.0)) + -((l48 * l124.re - l82_tmp * l124.im) * 12.0)) + (g_l586_re_tmp * l75.re - h_l586_re_tmp * l75.im) * 90.0) + (l42 * l75.re - l30 * l75.im) * 180.0) + (l148_tmp * l75.re - d_l586_re_tmp * l75.im) * 180.0) + (l586_re_tmp * l75.re - c_l586_re_tmp * l75.im) * 18.0) + (b_l586_re_tmp * l75.re - l62_tmp * l75.im) * 24.0) + (l147_tmp * l75.re - l153_tmp * l75.im) * 42.0) + (l104_tmp * l75.re - f_l586_re_tmp * l75.im) * 120.0) + (re_tmp * l75.re - l49 * l75.im) * 240.0) + (b_T_re * l294.re - b_T_im * l294.im) * -42.0) + T * l484_re) + l13 * l409.re) + l13 * l410_re) + l13 * l411_re) + l13 * l412_re) + -((l135_tmp * l294.re - l131_tmp * l294.im) * 12.0)) + -((f_T_re * l294.re - f_T_im * l294.im) * 24.0)) + (e_T_re * l294.re - e_T_im * l294.im) * 126.0) + (h_T_im * l294.re - l24_re * l294.im) * 315.0) + (g_T_im * l294.re - h_T_re * l294.im) * 315.0) + (l_T_im * l294.re - g_T_re * l294.im) * 420.0) + -((j_T_im * l294.re - k_T_re * l294.im) * 126.0)) + -((k_T_im * l125.re - l_T_re * l125.im) * 9.0)) + -((i_T_im * l124.re - j_T_re * l124.im) * 9.0)) + -((l24_im * l124.re - i_T_re * l124.im) * 12.0)) + (l39 * l75.re - b_A_init_re_tmp_tmp * l75.im) * 120.0) + (l12_im * l75.re - l12_re * l75.im) * 240.0) + (c_T_re * l294.re - c_T_im * l294.im) * -126.0) + T * l527_re) + -((l12_re_tmp * l294.re - l21_re * l294.im) * 12.0)) + -((l21_im * l294.re - l21_re_tmp * l294.im) * 210.0)) + (d_T_re * l294.re - d_T_im * l294.im) * -210.0;
    l21_re = (((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((l48_re * l295.im + l48_im * l295.re) + (l49_re * l295.im + l49_im * l295.re) * 3.0) + (J_max_re * l295.im + J_max_im * l295.re) * 6.0) + (b_J_max_re * l294.im + b_J_max_im * l294.re) * 6.0) + (l11_re * l295.im + l11_im * l295.re)) + -(l45_re * l295.im + l45_im * l295.re)) + (c_J_max_re * l295.im + c_J_max_im * l295.re) * 18.0) + (l2_re * l294.im + l2_im * l294.re) * 3.0) + (b_l11_re * l295.im + b_l11_im * l295.re) * 3.0) + (l10_re * l295.im + l10_im * l295.re) * 6.0) + -((d_J_max_re * l295.im + d_J_max_im * l295.re) * 3.0)) + -((e_J_max_re * l294.im + e_J_max_im * l294.re) * 6.0)) + (b_l2_re * l294.im + b_l2_im * l294.re) * 21.0) + (l8_re * l295.im + l8_im * l295.re) * 15.0) + (b_l10_re * l295.im + b_l10_im * l295.re) * 18.0) + (l4_re * l295.im + l4_im * l295.re) * 15.0) + (l6_re * l295.im + l6_im * l295.re) * 20.0) + (b_l8_re * l295.im + b_l8_im * l295.re) * 45.0) + (b_l4_re * l295.im + b_l4_im * l295.re) * 45.0) + (b_l6_re * l295.im + b_l6_im * l295.re) * 60.0) + (V_min_re * l75.im + V_min_im * l75.re) * 6.0) + (V_max_re * l75.im + V_max_im * l75.re) * 6.0) + (b_V_max_re * l75.im + b_V_max_im * l75.re) * 6.0) + (A_init_re * l294.im + A_init_im * l294.re) * 18.0) + (V_init_re * l294.im + V_init_im * l294.re) * 6.0) + (b_V_min_re * l294.im + b_V_min_im * l294.re) * 6.0) + (c_V_max_re * l294.im + c_V_max_im * l294.re) * 6.0) + (c_V_min_re * l294.im + c_V_min_im * l294.re) * 6.0) + (d_V_max_re * l294.im + d_V_max_im * l294.re) * 6.0) + (re * l76.im + im * l76.re) * 2.0) + (b_re * l76.im + b_im * l76.re) * 2.0) + (c_re * l76.im + c_im * l76.re) * 6.0) + (d_re * l76.im + d_im * l76.re) * 6.0) + -((c_l4_re * l295.im + c_l4_im * l295.re) * 2.0)) + -((b_V_init_re * l75.im + b_V_init_im * l75.re) * 6.0)) + (d_V_min_re * l75.im + d_V_min_im * l75.re) * 6.0) + (e_V_max_re * l75.im + e_V_max_im * l75.re) * 6.0) + (f_V_max_re * l124.im + f_V_max_im * l124.re) * 6.0) + (g_V_max_re * l75.im + g_V_max_im * l75.re) * 6.0) + (h_V_max_re * l124.im + h_V_max_im * l124.re) * 6.0) + (i_V_max_re * l75.im + i_V_max_im * l75.re) * 24.0) + (j_V_max_re * l75.im + j_V_max_im * l75.re) * 30.0) + (f_J_max_re * l294.im + f_J_max_im * l294.re) * 15.0) + (c_V_init_re * l294.im + c_V_init_im * l294.re) * 36.0) + (d_V_init_re * l294.im + d_V_init_im * l294.re) * 36.0) + -((e_V_min_re * l294.im + e_V_min_im * l294.re) * 6.0)) + -((k_V_max_re * l294.im + k_V_max_im * l294.re) * 6.0)) + -((f_V_min_re * l294.im + f_V_min_im * l294.re) * 6.0)) + -((l_V_max_re * l294.im + l_V_max_im * l294.re) * 6.0)) + (m_V_max_re * l294.im + m_V_max_im * l294.re) * 24.0) + (n_V_max_re * l294.im + n_V_max_im * l294.re) * 24.0) + (g_V_min_re * l294.im + g_V_min_im * l294.re) * 30.0) + (h_V_min_re * l294.im + h_V_min_im * l294.re) * 30.0) + (o_V_max_re * l294.im + o_V_max_im * l294.re) * 36.0) + (i_V_min_re * l294.im + i_V_min_im * l294.re) * 60.0) + (j_V_min_re * l294.im + j_V_min_im * l294.re) * 60.0) + (b_A_init_re * l75.im + b_A_init_im * l75.re) * 6.0) + (c_A_init_re * l75.im + c_A_init_im * l75.re) * 6.0) + (e_re * l124.im + e_im * l124.re) * 8.0) + -((c_l2_re * l294.im + c_l2_im * l294.re) * 9.0)) + -((d_l2_re * l294.im + d_l2_im * l294.re) * 12.0)) + -((e_V_init_re * l75.im + e_V_init_im * l75.re) * 6.0)) + -((f_V_init_re * l125.im + f_V_init_im * l125.re) * 6.0)) + -((g_V_init_re * l124.im + g_V_init_im * l124.re) * 6.0)) + -((h_V_init_re * l124.im + h_V_init_im * l124.re) * 6.0)) + -((i_V_init_re * l75.im + i_V_init_im * l75.re) * 30.0)) + (k_V_min_re * l75.im + k_V_min_im * l75.re) * 24.0) + (l_V_min_re * l75.im + l_V_min_im * l75.re) * 24.0) + (p_V_max_re * l75.im + p_V_max_im * l75.re) * 24.0) + (q_V_max_re * l75.im + q_V_max_im * l75.re) * 30.0) + (m_V_min_re * l75.im + m_V_min_im * l75.re) * 36.0) + (r_V_max_re * l75.im + r_V_max_im * l75.re) * 36.0) + (s_V_max_re * l75.im + s_V_max_im * l75.re) * 60.0) + (t_V_max_re * l75.im + t_V_max_im * l75.re) * 60.0) + -((d_A_init_re * l294.im + d_A_init_im * l294.re) * 12.0)) + (g_J_max_re * l294.im + g_J_max_im * l294.re) * 126.0) + -((j_V_init_re * l294.im + j_V_init_im * l294.re) * 6.0)) + (k_V_init_re * l294.im + k_V_init_im * l294.re) * 90.0) + (l_V_init_re * l294.im + l_V_init_im * l294.re) * 90.0) + (m_V_init_re * l294.im + m_V_init_im * l294.re) * 120.0) + (u_V_max_re * l294.im + u_V_max_im * l294.re) * 12.0) + (v_V_max_re * l294.im + v_V_max_im * l294.re) * 12.0) + -((n_V_min_re * l294.im + n_V_min_im * l294.re) * 24.0)) + -((o_V_min_re * l294.im + o_V_min_im * l294.re) * 24.0)) + -((w_V_max_re * l294.im + w_V_max_im * l294.re) * 30.0)) + -((x_V_max_re * l294.im + x_V_max_im * l294.re) * 30.0)) + -((p_V_min_re * l294.im + p_V_min_im * l294.re) * 36.0)) + -((y_V_max_re * l294.im + y_V_max_im * l294.re) * 60.0)) + -((ab_V_max_re * l294.im + ab_V_max_im * l294.re) * 60.0)) + (e_A_init_re * l294.im + e_A_init_im * l294.re) * 6.0) + (f_A_init_re * l75.im + f_A_init_im * l75.re) * 12.0) + (g_A_init_re * l75.im + g_A_init_im * l75.re) * 18.0) + (e_l2_re * l75.im + e_l2_im * l75.re) * 3.0) + (f_l2_re * l75.im + f_l2_im * l75.re) * 3.0) + (g_l2_re * l75.im + g_l2_im * l75.re) * 6.0) + (h_l2_re * l75.im + h_l2_im * l75.re) * 6.0) + T * l361_im) + (l13_re * l75.im + l13_im * l75.re) * 3.0) + l13 * l312.im) + (b_l13_re * l75.im + b_l13_im * l75.re) * 6.0) + (h_A_init_re * l294.im + h_A_init_im * l294.re) * 18.0) + (i_l2_re * l294.im + i_l2_im * l294.re) * 3.0) + l13 * l310_im) + -((n_V_init_re * l75.im + n_V_init_im * l75.re) * 30.0)) + -((o_V_init_re * l75.im + o_V_init_im * l75.re) * 60.0)) + -((p_V_init_re * l75.im + p_V_init_im * l75.re) * 60.0)) + -((h_J_max_re * l294.im + h_J_max_im * l294.re) * 9.0)) + -((i_J_max_re * l294.im + i_J_max_im * l294.re) * 24.0)) + -((q_V_min_re * l294.im + q_V_min_im * l294.re) * 12.0)) + -((r_V_min_re * l294.im + r_V_min_im * l294.re) * 12.0)) + -((i_A_init_re * l294.im + i_A_init_im * l294.re) * 6.0)) + (j_A_init_re * l294.im + j_A_init_im * l294.re) * 42.0) + (k_A_init_re * l75.im + k_A_init_im * l75.re) * 18.0) + (l_A_init_re * l75.im + l_A_init_im * l75.re) * 90.0) + (j_l2_re * l75.im + j_l2_im * l75.re) * 3.0) + (k_l2_re * l124.im + k_l2_im * l124.re) * 3.0) + (l_l2_re * l75.im + l_l2_im * l75.re) * 24.0) + (m_l2_re * l75.im + m_l2_im * l75.re) * 27.0) + (n_l2_re * l75.im + n_l2_im * l75.re) * 48.0) + (T_re * l294.im + T_im * l294.re) * -6.0) + T * l419_im) + l13 * l349.im) + (c_l13_re * l75.im + c_l13_im * l75.re) * 3.0) + (m_A_init_re * l294.im + m_A_init_im * l294.re) * 108.0) + (n_A_init_re * l294.im + n_A_init_im * l294.re) * 108.0) + (o_A_init_re * l294.im + o_A_init_im * l294.re) * 270.0) + (p_A_init_re * l294.im + p_A_init_im * l294.re) * 270.0) + (q_A_init_re * l294.im + q_A_init_im * l294.re) * 360.0) + (o_l2_re * l294.im + o_l2_im * l294.re) * 15.0) + (p_l2_re * l294.im + p_l2_im * l294.re) * 21.0) + (q_l2_re * l294.im + q_l2_im * l294.re) * 30.0) + (r_l2_re * l294.im + r_l2_im * l294.re) * 30.0) + l13 * l315_im) + l13 * l319_im) + l13 * l322_im) + l13 * l324_im) + l13 * l326_im) + l13 * l328_im) + -((r_A_init_re * l294.im + r_A_init_im * l294.re) * 42.0)) + (s_A_init_re * l294.im + s_A_init_im * l294.re) * 126.0) + (t_A_init_re * l294.im + t_A_init_im * l294.re) * 210.0) + -((u_A_init_re * l124.im + u_A_init_im * l124.re) * 12.0)) + -((l48 * l124.im + l82_tmp * l124.re) * 12.0)) + (g_l586_re_tmp * l75.im + h_l586_re_tmp * l75.re) * 90.0) + (l42 * l75.im + l30 * l75.re) * 180.0) + (l148_tmp * l75.im + d_l586_re_tmp * l75.re) * 180.0) + (l586_re_tmp * l75.im + c_l586_re_tmp * l75.re) * 18.0) + (b_l586_re_tmp * l75.im + l62_tmp * l75.re) * 24.0) + (l147_tmp * l75.im + l153_tmp * l75.re) * 42.0) + (l104_tmp * l75.im + f_l586_re_tmp * l75.re) * 120.0) + (re_tmp * l75.im + l49 * l75.re) * 240.0) + (b_T_re * l294.im + b_T_im * l294.re) * -42.0) + T * l484_im) + l13 * l409.im) + l13 * l410_im) + l13 * l411_im) + l13 * l412_im) + -((l135_tmp * l294.im + l131_tmp * l294.re) * 12.0)) + -((f_T_re * l294.im + f_T_im * l294.re) * 24.0)) + (e_T_re * l294.im + e_T_im * l294.re) * 126.0) + (h_T_im * l294.im + l24_re * l294.re) * 315.0) + (g_T_im * l294.im + h_T_re * l294.re) * 315.0) + (l_T_im * l294.im + g_T_re * l294.re) * 420.0) + -((j_T_im * l294.im + k_T_re * l294.re) * 126.0)) + -((k_T_im * l125.im + l_T_re * l125.re) * 9.0)) + -((i_T_im * l124.im + j_T_re * l124.re) * 9.0)) + -((l24_im * l124.im + i_T_re * l124.re) * 12.0)) + (l39 * l75.im + b_A_init_re_tmp_tmp * l75.re) * 120.0) + (l12_im * l75.im + l12_re * l75.re) * 240.0) + (c_T_re * l294.im + c_T_im * l294.re) * -126.0) + T * l527_im) + -((l12_re_tmp * l294.im + l21_re * l294.re) * 12.0)) + -((l21_im * l294.im + l21_re_tmp * l294.re) * 210.0)) + (d_T_re * l294.im + d_T_im * l294.re) * -210.0;
    l12_re_tmp = l575 * r;
    r = l575 * l21_re;
    if (r == 0.0) {
      l349.re = l12_re_tmp / 2.0;
      l349.im = 0.0;
    } else if (l12_re_tmp == 0.0) {
      l349.re = 0.0;
      l349.im = r / 2.0;
    } else {
      l349.re = l12_re_tmp / 2.0;
      l349.im = r / 2.0;
    }
    l21_im = l578.re * l578.im;
    l12_re_tmp = l576 * (l578.re * l578.re - l578.im * l578.im);
    r = l576 * (l21_im + l21_im);
    if (r == 0.0) {
      l21_im = l12_re_tmp / 9.0;
      l21_re = 0.0;
    } else if (l12_re_tmp == 0.0) {
      l21_im = 0.0;
      l21_re = r / 9.0;
    } else {
      l21_im = l12_re_tmp / 9.0;
      l21_re = r / 9.0;
    }
    l12_re_tmp = l575 * l586_re;
    r = l575 * l586_im;
    if (r == 0.0) {
      l21_re_tmp = l12_re_tmp / 3.0;
      r = 0.0;
    } else if (l12_re_tmp == 0.0) {
      l21_re_tmp = 0.0;
      r /= 3.0;
    } else {
      l21_re_tmp = l12_re_tmp / 3.0;
      r /= 3.0;
    }
    l409.re = l21_im + l21_re_tmp;
    l409.im = l21_re + r;
    l125.re = (l584_re + l588_re) + l349.re;
    l125.im = (l584_im + l588_im) + l349.im;
    dc = coder::d_power(l409);
    l12_re_tmp = l125.re * l125.re - l125.im * l125.im;
    l21_re_tmp = l125.re * l125.im;
    l125.re = -dc.re + l12_re_tmp;
    l125.im = -dc.im + (l21_re_tmp + l21_re_tmp);
    coder::internal::scalar::b_sqrt(&l125);
    l67.re = ((-l584_re + -l588_re) + -l349.re) + l125.re;
    l67.im = ((-l584_im + -l588_im) + -l349.im) + l125.im;
    l294 = coder::power(l67);
    if (l294.im == 0.0) {
      l21_re = l294.re / 2.0;
      l12_re = 0.0;
      re = 1.0 / l294.re;
      im = 0.0;
    } else if (l294.re == 0.0) {
      l21_re = 0.0;
      l12_re = l294.im / 2.0;
      re = 0.0;
      im = -(1.0 / l294.im);
    } else {
      l21_re = l294.re / 2.0;
      l12_re = l294.im / 2.0;
      l12_re_tmp = std::abs(l294.re);
      l21_re_tmp = std::abs(l294.im);
      if (l12_re_tmp > l21_re_tmp) {
        r = l294.im / l294.re;
        l21_re_tmp = l294.re + r * l294.im;
        re = (r * 0.0 + 1.0) / l21_re_tmp;
        im = (0.0 - r) / l21_re_tmp;
      } else if (l21_re_tmp == l12_re_tmp) {
        if (l294.re > 0.0) {
          l21_im = 0.5;
        } else {
          l21_im = -0.5;
        }
        if (l294.im > 0.0) {
          r = 0.5;
        } else {
          r = -0.5;
        }
        re = (l21_im + 0.0 * r) / l12_re_tmp;
        im = (0.0 * l21_im - r) / l12_re_tmp;
      } else {
        r = l294.re / l294.im;
        l21_re_tmp = l294.im + r * l294.re;
        re = r / l21_re_tmp;
        im = (r * 0.0 - 1.0) / l21_re_tmp;
      }
    }
    l125.re = l409.re * re - l409.im * im;
    l125.im = l409.re * im + l409.im * re;
    if (l125.im == 0.0) {
      l12_re_tmp = l125.re / 2.0;
      r = 0.0;
    } else if (l125.re == 0.0) {
      l12_re_tmp = 0.0;
      r = l125.im / 2.0;
    } else {
      l12_re_tmp = l125.re / 2.0;
      r = l125.im / 2.0;
    }
    t4[0].re = (-l575_re + l294.re) + l125.re;
    t4[0].im = (-l575_im + l294.im) + l125.im;
    re_tmp = 1.7320508075688772 * (l294.re - l125.re);
    l21_re_tmp = 1.7320508075688772 * (l294.im - l125.im);
    d = (-l575_re + -l21_re) + -l12_re_tmp;
    t4[1].re = d + (re_tmp * 0.0 - l21_re_tmp * 0.5);
    d1 = (-l575_im + -l12_re) + -r;
    t4[1].im = d1 + (re_tmp * 0.5 + l21_re_tmp * 0.0);
    t4[2].re = d + (re_tmp * -0.0 - l21_re_tmp * -0.5);
    t4[2].im = d1 + (re_tmp * -0.5 + l21_re_tmp * -0.0);
    l12_re = J_min + -J_max;
    l349.re = -J_min;
    l349.im = 0.0;
    coder::internal::scalar::b_sqrt(&l349);
    l294.re = l67_tmp;
    l294.im = 0.0;
    coder::internal::scalar::b_sqrt(&l294);
    l312.re = (A_init * A_init + J_max * V_max * 2.0) + -(J_max * V_init * 2.0);
    l312.im = 0.0;
    coder::internal::scalar::b_sqrt(&l312);
    l409.re = l75_tmp;
    l409.im = 0.0;
    coder::internal::scalar::b_sqrt(&l409);
    l12_re_tmp = J_min * J_max;
    l12_im = l12_re_tmp * l12_re;
    l125.re = -(J_min * l12_re * (V_min + -V_max));
    l125.im = 0.0;
    coder::internal::scalar::b_sqrt(&l125);
    l21_re = J_min * J_min;
    l21_re_tmp = 1.4142135623730951 * l29_tmp * (1.0 / (l21_re + -l12_re_tmp));
    l125.re *= l21_re_tmp;
    l125.im *= l21_re_tmp;
    l21_im = J_min * (J_min - J_max);
    l67.re = l294_tmp * -2.0 + J_max * V_min * 2.0;
    re_tmp = l125.re * l125.re - l125.im * l125.im;
    r = l125.re * l125.im;
    im = r + r;
    l21_re_tmp = l21_re * im - l12_re_tmp * im;
    b_l3[0].re =
        l21_im * (((l67.re + l21_re * re_tmp) + l2_tmp) - l12_re_tmp * re_tmp);
    b_l3[0].im = l21_im * l21_re_tmp;
    im = r + r;
    l21_re_tmp = l21_re * im - l12_re_tmp * im;
    b_l3[1].re =
        l21_im * (((l67.re + l21_re * re_tmp) + l2_tmp) - l12_re_tmp * re_tmp);
    b_l3[1].im = l21_im * l21_re_tmp;
    im = r + r;
    l21_re_tmp = l21_re * im - l12_re_tmp * im;
    b_l3[2].re =
        l21_im * (((l67.re + l21_re * re_tmp) + l2_tmp) - l12_re_tmp * re_tmp);
    b_l3[2].im = l21_im * l21_re_tmp;
    coder::internal::scalar::b_sqrt(&b_l3[0]);
    coder::internal::scalar::b_sqrt(&b_l3[1]);
    coder::internal::scalar::b_sqrt(&b_l3[2]);
    l21_re_tmp = l21_re - l12_re_tmp;
    dc = coder::d_power(l349);
    re = dc.re * l294.re - dc.im * l294.im;
    im = dc.re * l294.im + dc.im * l294.re;
    J_max_re = J_max * l349.re;
    J_max_im = J_max * l349.im;
    b_J_max_re = J_max_re * l294.re - J_max_im * l294.im;
    J_max_im = J_max_re * l294.im + J_max_im * l294.re;
    l75.re = ((re * l312.re - im * l312.im) + -J_min * l12_re * (A_init + d5)) +
             (b_J_max_re * l312.re - J_max_im * l312.im);
    l75.im = (re * l312.im + im * l312.re) +
             (b_J_max_re * l312.im + J_max_im * l312.re);
    re_tmp = 1.4142135623730951 * rt_powd_snf(J_max, 1.5);
    re = re_tmp * l349.re;
    im = re_tmp * l349.im;
    b_re = re * l294.re - im * l294.im;
    im = re * l294.im + im * l294.re;
    l67.re = b_re * l409.re - im * l409.im;
    l67.im = b_re * l409.im + im * l409.re;
    if (b_l3[0].im == 0.0) {
      re = b_l3[0].re / l21_re_tmp;
      im = 0.0;
    } else if (b_l3[0].re == 0.0) {
      re = 0.0;
      im = b_l3[0].im / l21_re_tmp;
    } else {
      re = b_l3[0].re / l21_re_tmp;
      im = b_l3[0].im / l21_re_tmp;
    }
    l12_re_tmp = -(A_init + J_min * re);
    r = -(J_min * im);
    if (r == 0.0) {
      t[0].re = l12_re_tmp / J_max;
      t[0].im = 0.0;
    } else if (l12_re_tmp == 0.0) {
      t[0].re = 0.0;
      t[0].im = r / J_max;
    } else {
      t[0].re = l12_re_tmp / J_max;
      t[0].im = r / J_max;
    }
    t[3].re = 0.0;
    t[3].im = 0.0;
    t[6].re = re;
    t[6].im = im;
    t[9] = t4[0];
    t[12] = l125;
    t[15].re = 0.0;
    t[15].im = 0.0;
    l12_re_tmp = -((l75.re + l12_im * t4[0].re) + l67.re);
    r = -((l75.im + l12_im * t4[0].im) + l67.im);
    if (r == 0.0) {
      t[18].re = l12_re_tmp / l12_im;
      t[18].im = 0.0;
    } else if (l12_re_tmp == 0.0) {
      t[18].re = 0.0;
      t[18].im = r / l12_im;
    } else {
      t[18].re = l12_re_tmp / l12_im;
      t[18].im = r / l12_im;
    }
    if (b_l3[1].im == 0.0) {
      re = b_l3[1].re / l21_re_tmp;
      im = 0.0;
    } else if (b_l3[1].re == 0.0) {
      re = 0.0;
      im = b_l3[1].im / l21_re_tmp;
    } else {
      re = b_l3[1].re / l21_re_tmp;
      im = b_l3[1].im / l21_re_tmp;
    }
    l12_re_tmp = -(A_init + J_min * re);
    r = -(J_min * im);
    if (r == 0.0) {
      t[1].re = l12_re_tmp / J_max;
      t[1].im = 0.0;
    } else if (l12_re_tmp == 0.0) {
      t[1].re = 0.0;
      t[1].im = r / J_max;
    } else {
      t[1].re = l12_re_tmp / J_max;
      t[1].im = r / J_max;
    }
    t[4].re = 0.0;
    t[4].im = 0.0;
    t[7].re = re;
    t[7].im = im;
    t[10] = t4[1];
    t[13] = l125;
    t[16].re = 0.0;
    t[16].im = 0.0;
    l12_re_tmp = -((l75.re + l12_im * t4[1].re) + l67.re);
    r = -((l75.im + l12_im * t4[1].im) + l67.im);
    if (r == 0.0) {
      t[19].re = l12_re_tmp / l12_im;
      t[19].im = 0.0;
    } else if (l12_re_tmp == 0.0) {
      t[19].re = 0.0;
      t[19].im = r / l12_im;
    } else {
      t[19].re = l12_re_tmp / l12_im;
      t[19].im = r / l12_im;
    }
    if (b_l3[2].im == 0.0) {
      re = b_l3[2].re / l21_re_tmp;
      im = 0.0;
    } else if (b_l3[2].re == 0.0) {
      re = 0.0;
      im = b_l3[2].im / l21_re_tmp;
    } else {
      re = b_l3[2].re / l21_re_tmp;
      im = b_l3[2].im / l21_re_tmp;
    }
    l12_re_tmp = -(A_init + J_min * re);
    r = -(J_min * im);
    if (r == 0.0) {
      t[2].re = l12_re_tmp / J_max;
      t[2].im = 0.0;
    } else if (l12_re_tmp == 0.0) {
      t[2].re = 0.0;
      t[2].im = r / J_max;
    } else {
      t[2].re = l12_re_tmp / J_max;
      t[2].im = r / J_max;
    }
    t[5].re = 0.0;
    t[5].im = 0.0;
    t[8].re = re;
    t[8].im = im;
    t[11] = t4[2];
    t[14] = l125;
    t[17].re = 0.0;
    t[17].im = 0.0;
    l12_re_tmp = -((l75.re + l12_im * t4[2].re) + l67.re);
    r = -((l75.im + l12_im * t4[2].im) + l67.im);
    if (r == 0.0) {
      t[20].re = l12_re_tmp / l12_im;
      t[20].im = 0.0;
    } else if (l12_re_tmp == 0.0) {
      t[20].re = 0.0;
      t[20].im = r / l12_im;
    } else {
      t[20].re = l12_re_tmp / l12_im;
      t[20].im = r / l12_im;
    }
}

// End of code generation (acdeg_T_P.cpp)
