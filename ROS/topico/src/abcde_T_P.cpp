//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// abcde_T_P.cpp
//
// Code generation for function 'abcde_T_P'
//

// Include files
#include "abcde_T_P.h"
#include "rt_nonfinite.h"
#include "topico_wrapper_data.h"
#include "topico_wrapper_rtwutil.h"
#include "topico_wrapper_types.h"

// Function Definitions
void abcde_T_P(double P_init, double V_init, double A_init, double P_wayp,
               double V_max, double A_max, double J_max, double J_min, double T,
               creal_T t[21])
{
  creal_T t4[3];
  double a_tmp;
  double b_a_tmp;
  double b_l199_tmp;
  double c_a_tmp;
  double c_l199_tmp;
  double d;
  double d1;
  double d2;
  double d3;
  double d4;
  double d5;
  double d6;
  double d7;
  double d_a_tmp;
  double e_a_tmp;
  double f_a_tmp;
  double g_a_tmp;
  double h_a_tmp;
  double i_a_tmp;
  double j_a_tmp;
  double k_a_tmp;
  double l10;
  double l103;
  double l104;
  double l105;
  double l105_tmp;
  double l108;
  double l11;
  double l110;
  double l111;
  double l12;
  double l13;
  double l137;
  double l14;
  double l140;
  double l15;
  double l16;
  double l18;
  double l199;
  double l199_tmp;
  double l20;
  double l201;
  double l201_tmp;
  double l212_im;
  double l212_re;
  double l22;
  double l24;
  double l25;
  double l2_tmp;
  double l3;
  double l4;
  double l42;
  double l43;
  double l45;
  double l45_tmp;
  double l66;
  double l7_tmp;
  double l8;
  double l9;
  double l91;
  double l92;
  double l94;
  double l94_tmp;
  double l_a_tmp;
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
  //  Generated on 03-Sep-2019 15:43:38
  l2_tmp = A_init * A_init;
  l3 = rt_powd_snf(A_init, 3.0);
  l7_tmp = A_max * A_max;
  l8 = rt_powd_snf(A_max, 3.0);
  l10 = rt_powd_snf(A_max, 5.0);
  l12 = J_min * J_min;
  l13 = rt_powd_snf(J_min, 3.0);
  l14 = J_max * J_max;
  l15 = rt_powd_snf(J_max, 3.0);
  l16 = T * T;
  l18 = V_init * V_init;
  l20 = V_max * V_max;
  l4 = l2_tmp * l2_tmp;
  l9 = l7_tmp * l7_tmp;
  l11 = rt_powd_snf(l7_tmp, 3.0);
  l22 = 1.0 / l8;
  l24 = 1.0 / l13;
  l25 = 1.0 / l15;
  l42 = J_min * l10 * l15 * 6.0;
  l43 = J_max * l10 * l13 * 6.0;
  l91 = A_init * l8 * l13 * l14 * 24.0;
  l94_tmp = J_max * l2_tmp;
  l94 = l94_tmp * l8 * l13 * 36.0;
  l111 = A_max * l13 * l15;
  l103 = l111 * l18 * 24.0;
  l104 = V_init * l7_tmp * l13 * l15 * 24.0;
  l105_tmp = V_init * l8;
  l105 = l105_tmp * l12 * l15 * 24.0;
  l108 = l111 * l20 * 24.0;
  l111 = V_max * l8;
  l110 = l111 * l12 * l15 * 24.0;
  l111 = l111 * l13 * l14 * 24.0;
  l212_im = A_init * V_init;
  l137 = l212_im * l7_tmp * l13 * l14 * 48.0;
  l140 = A_max * V_max * l2_tmp * l13 * l14 * 24.0;
  l45_tmp = A_max * J_max;
  l45 = l45_tmp * l4 * l13 * 6.0;
  l66 = l9 * l12 * l15 * 12.0;
  l92 = A_init * l9 * l12 * l14 * 24.0;
  l199_tmp = l9 * l13 * l14;
  b_l199_tmp = V_max * l7_tmp * l13 * l15;
  c_l199_tmp = l2_tmp * l7_tmp * l13 * l14;
  l199 = (((((l66 + -(l199_tmp * 12.0)) + l91) + T * l8 * l13 * l15 * 24.0) +
           l104) +
          -(b_l199_tmp * 24.0)) +
         -(c_l199_tmp * 12.0);
  l201_tmp = l22 * l24 * l25;
  l201 = l201_tmp * l199 / 24.0;
  d = J_max * V_init;
  d1 = J_max * V_max;
  l212_re = A_init * J_max;
  d2 = l13 * l15;
  d3 = J_max * l3;
  d4 = A_init * J_min;
  d5 = V_init * V_max;
  d6 = d5 * l7_tmp;
  d7 = A_init * A_max;
  a_tmp = T * l9;
  b_a_tmp = A_max * V_init;
  c_a_tmp = d7 * l13 * l14;
  d_a_tmp = A_init * V_max;
  e_a_tmp = l8 * l13 * l15;
  f_a_tmp = T * V_init;
  g_a_tmp = l2_tmp * l13 * l14;
  h_a_tmp = l7_tmp * l12 * l15;
  i_a_tmp = l7_tmp * l13 * l14;
  j_a_tmp = A_max * T * V_init;
  k_a_tmp = A_init * T;
  l_a_tmp = T * l2_tmp;
    l212_im = (rt_powd_snf(l22, 3.0) * rt_powd_snf(l24, 3.0) * rt_powd_snf(l25, 3.0) * rt_powd_snf(l199, 3.0) / 6912.0 + -(1.0 / l11 * (1.0 / rt_powd_snf(l12, 3.0)) * (1.0 / rt_powd_snf(l14, 3.0)) * l199 * ((((((((((((((((((((((((((l42 + l43) + l45) + -(l212_re * l9 * l13 * 24.0)) + -(l10 * l12 * l14 * 12.0)) + l92) + l94) + -(b_a_tmp * V_max * l13 * l15 * 48.0)) + a_tmp * l12 * l15 * 24.0) + l103) + l105) + l108) + l111) + -(d3 * l7_tmp * l13 * 24.0)) + -(a_tmp * l13 * l14 * 24.0)) + -(l105_tmp * l13 * l14 * 24.0)) + -l110) + k_a_tmp * l8 * l13 * l14 * 48.0) + l137) + l140) + f_a_tmp * l7_tmp * l13 * l15 * 48.0) + e_a_tmp * l16 * 24.0) + -(b_a_tmp * l2_tmp * l13 * l14 * 24.0)) + -(d_a_tmp * l7_tmp * l13 * l14 * 48.0)) + -(T * V_max * l7_tmp * l13 * l15 * 48.0)) + -(l2_tmp * l8 * l12 * l14 * 12.0)) + -(l_a_tmp * l7_tmp * l13 * l14 * 24.0)) / 192.0)) + l201_tmp * ((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((-(rt_powd_snf(l2_tmp, 3.0) * l13) + -(l11 * l13)) + -(l11 * l15)) + A_init * l10 * l13 * 6.0) + A_max * rt_powd_snf(A_init, 5.0) * l13 * 6.0) + J_max * l11 * l12 * 5.0) + d4 * l10 * l14 * 6.0) + T * l42) + T * l43) + d * l4 * l13 * 6.0) + J_min * V_init * l9 * l15 * 6.0) + d * l9 * l13 * 6.0) + -(J_min * l11 * l14 * 3.0)) + -(d1 * l4 * l13 * 6.0)) + -(J_min * V_max * l9 * l15 * 6.0)) + -(d1 * l9 * l13 * 6.0)) + T * l45) + -(l212_re * l10 * l12 * 12.0)) + l3 * l8 * l13 * 20.0) + d2 * rt_powd_snf(V_init, 3.0) * 8.0) + l212_re * V_max * l8 * l13 * 24.0) + l45_tmp * V_max * l3 * l13 * 24.0) + d3 * l8 * l12 * 4.0) + l94_tmp * l9 * l12 * 6.0) + -(l2_tmp * l9 * l13 * 15.0)) + -(l4 * l7_tmp * l13 * 15.0)) + -(d2 * rt_powd_snf(V_max, 3.0) * 8.0)) + l212_re * T * l9 * l13 * -24.0) + -(l212_re * V_init * l8 * l13 * 24.0)) + -(l45_tmp * V_init * l3 * l13 * 24.0)) + -(J_min * l2_tmp * l9 * l14 * 3.0)) + -(J_max * l4 * l7_tmp * l12 * 3.0)) + P_init * l8 * l12 * l15 * 48.0) + V_init * l9 * l12 * l14 * 12.0) + V_init * l13 * l15 * l20 * 24.0) + d5 * l2_tmp * l13 * l14 * 24.0) + d6 * l12 * l15 * 24.0) + d6 * l13 * l14 * 24.0) + -(P_wayp * l8 * l12 * l15 * 48.0)) + T * l10 * l12 * l14 * -12.0) + -(V_max * l9 * l12 * l14 * 12.0)) + -(V_max * l13 * l15 * l18 * 24.0)) + -(d7 * V_init * V_max * l13 * l14 * 48.0)) + T * l92) + T * l94) + c_a_tmp * l18 * 24.0) + d * l2_tmp * l7_tmp * l13 * 36.0) + c_a_tmp * l20 * 24.0) + d_a_tmp * l8 * l12 * l14 * 24.0) + j_a_tmp * V_max * l13 * l15 * -48.0) + T * l103) + T * l105) + T * l108) + T * l110) + T * l111) + e_a_tmp * rt_powd_snf(T, 3.0) * 8.0) + l16 * l66) + T * l137) + T * l140) + J_max * T * l3 * l7_tmp * l13 * -24.0) + -(l212_im * l8 * l12 * l14 * 24.0)) + -(d1 * l2_tmp * l7_tmp * l13 * 36.0)) + f_a_tmp * l8 * l13 * l14 * -24.0) + l199_tmp * l16 * -12.0) + -(g_a_tmp * l18 * 12.0)) + -(g_a_tmp * l20 * 12.0)) + -(h_a_tmp * l18 * 12.0)) + -(i_a_tmp * l18 * 12.0)) + -(h_a_tmp * l20 * 12.0)) + -(i_a_tmp * l20 * 12.0)) + j_a_tmp * l2_tmp * l13 * l14 * -24.0) + k_a_tmp * V_max * l7_tmp * l13 * l14 * -48.0) + l16 * l91) + V_init * l2_tmp * l7_tmp * l12 * l14 * 12.0) + l16 * l104) + l_a_tmp * l8 * l12 * l14 * -12.0) + -(V_max * l2_tmp * l7_tmp * l12 * l14 * 12.0)) + b_l199_tmp * l16 * -24.0) + c_l199_tmp * l16 * -12.0) / 8.0;
    l111 = rt_powd_snf(l212_im, 0.33333333333333331);
    if (l212_im < 0.0) {
      k_rtErrorWithMessageID(o_emlrtRTEI.fName, o_emlrtRTEI.lineNo);
    }
    l212_re = 1.7320508075688772 * l111 * 0.0;
    l212_im = 1.7320508075688772 * l111 * 0.5;
    t4[0].re = l201 + l111;
    t4[0].im = 0.0;
    d = l201 + -(l111 / 2.0);
    t4[1].re = d - l212_re;
    t4[1].im = 0.0 - l212_im;
    t4[2].re = d + l212_re;
    t4[2].im = l212_im;
    l111 = A_max * J_min;
    b_l199_tmp = l111 * J_max;
    l199 = b_l199_tmp * 2.0;
    l201_tmp = J_min * J_max;
    l3 = A_max * (1.0 / J_min);
    l212_im = -(1.0 / J_max * (A_init + -A_max));
    l212_re = (d4 - l111) + l45_tmp;
    c_l199_tmp = l201_tmp * T;
    t[0].re = l212_im;
    t[0].im = 0.0;
    t[1].re = l212_im;
    t[1].im = 0.0;
    t[2].re = l212_im;
    t[2].im = 0.0;
    t[6].re = -l3;
    t[6].im = 0.0;
    t[7].re = -l3;
    t[7].im = 0.0;
    t[8].re = -l3;
    t[8].im = 0.0;
    l199_tmp = (l7_tmp + d1 * 2.0) + l2_tmp;
    l92 = J_max * l7_tmp;
    l45 = d7 * J_min * 2.0;
    l66 = l201_tmp * V_init * 2.0;
    b_l199_tmp = b_l199_tmp * T * 2.0;
    l137 =
        (((l92 - J_min * (l199_tmp + l45_tmp * t4[0].re * 2.0)) + l45) + l66) +
        b_l199_tmp;
    if (0.0 - J_min * (l45_tmp * 0.0 * 2.0) == 0.0) {
      l111 = l137 / l199;
      l212_im = 0.0;
    } else if (l137 == 0.0) {
      l111 = 0.0;
      l212_im = rtNaN;
    } else {
      l111 = l137 / l199;
      l212_im = rtNaN;
    }
    l137 = (l212_re - l201_tmp * (t4[0].re + l111)) + c_l199_tmp;
    if (0.0 - l201_tmp * l212_im == 0.0) {
      t[3].re = l137 / l201_tmp;
      t[3].im = 0.0;
    } else if (l137 == 0.0) {
      t[3].re = 0.0;
      t[3].im = rtNaN;
    } else {
      t[3].re = l137 / l201_tmp;
      t[3].im = rtNaN;
    }
    t[9] = t4[0];
    t[12].re = l111;
    t[12].im = l212_im;
    t[15].re = 0.0;
    t[15].im = 0.0;
    t[18].re = 0.0;
    t[18].im = 0.0;
    l137 =
        (((l92 - J_min * (l199_tmp + l45_tmp * t4[1].re * 2.0)) + l45) + l66) +
        b_l199_tmp;
    l140 = 0.0 - J_min * (l45_tmp * t4[1].im * 2.0);
    if (l140 == 0.0) {
      l111 = l137 / l199;
      l212_im = 0.0;
    } else if (l137 == 0.0) {
      l111 = 0.0;
      l212_im = l140 / l199;
    } else {
      l111 = l137 / l199;
      l212_im = l140 / l199;
    }
    l137 = (l212_re - l201_tmp * (t4[1].re + l111)) + c_l199_tmp;
    l140 = 0.0 - l201_tmp * (t4[1].im + l212_im);
    if (l140 == 0.0) {
      t[4].re = l137 / l201_tmp;
      t[4].im = 0.0;
    } else if (l137 == 0.0) {
      t[4].re = 0.0;
      t[4].im = l140 / l201_tmp;
    } else {
      t[4].re = l137 / l201_tmp;
      t[4].im = l140 / l201_tmp;
    }
    t[10] = t4[1];
    t[13].re = l111;
    t[13].im = l212_im;
    t[16].re = 0.0;
    t[16].im = 0.0;
    t[19].re = 0.0;
    t[19].im = 0.0;
    l137 =
        (((l92 - J_min * (l199_tmp + l45_tmp * t4[2].re * 2.0)) + l45) + l66) +
        b_l199_tmp;
    l140 = 0.0 - J_min * (l45_tmp * t4[2].im * 2.0);
    if (l140 == 0.0) {
      l111 = l137 / l199;
      l212_im = 0.0;
    } else if (l137 == 0.0) {
      l111 = 0.0;
      l212_im = l140 / l199;
    } else {
      l111 = l137 / l199;
      l212_im = l140 / l199;
    }
    l137 = (l212_re - l201_tmp * (t4[2].re + l111)) + c_l199_tmp;
    l140 = 0.0 - l201_tmp * (t4[2].im + l212_im);
    if (l140 == 0.0) {
      t[5].re = l137 / l201_tmp;
      t[5].im = 0.0;
    } else if (l137 == 0.0) {
      t[5].re = 0.0;
      t[5].im = l140 / l201_tmp;
    } else {
      t[5].re = l137 / l201_tmp;
      t[5].im = l140 / l201_tmp;
    }
    t[11] = t4[2];
    t[14].re = l111;
    t[14].im = l212_im;
    t[17].re = 0.0;
    t[17].im = 0.0;
    t[20].re = 0.0;
    t[20].im = 0.0;
}

// End of code generation (abcde_T_P.cpp)
