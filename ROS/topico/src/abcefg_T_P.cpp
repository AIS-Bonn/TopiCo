//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// abcefg_T_P.cpp
//
// Code generation for function 'abcefg_T_P'
//

// Include files
#include "abcefg_T_P.h"
#include "power.h"
#include "rt_nonfinite.h"
#include "sqrt.h"
#include "topico_wrapper_data.h"
#include "topico_wrapper_rtwutil.h"
#include <cmath>

// Function Definitions
void abcefg_T_P(double P_init, double V_init, double A_init, double P_wayp,
                double V_min, double A_max, double A_min, double J_max,
                double J_min, double T, creal_T t[21])
{
  creal_T t6[3];
  creal_T b_l140;
  creal_T l158;
  double a_tmp;
  double b_a_tmp;
  double b_l134_tmp;
  double b_l142_tmp;
  double b_l15_tmp;
  double b_l16_tmp;
  double b_l18_tmp;
  double c_l134_tmp;
  double c_l16_tmp;
  double d_l134_tmp;
  double e_l134_tmp;
  double f_l134_tmp;
  double l10;
  double l103;
  double l134;
  double l134_tmp;
  double l134_tmp_tmp;
  double l137;
  double l137_tmp;
  double l140;
  double l142;
  double l142_tmp;
  double l142_tmp_tmp;
  double l145;
  double l148;
  double l150;
  double l15_tmp;
  double l16_tmp;
  double l18_tmp;
  double l21_tmp;
  double l22_tmp;
  double l23_tmp;
  double l24_tmp;
  double l25_tmp;
  double l26_tmp;
  double l27_tmp;
  double l2_tmp;
  double l35;
  double l50;
  double l51;
  double l58;
  double l58_tmp;
  double l5_tmp;
  double l6;
  double l65;
  double l66;
  double l7;
  double l9;
  double l92;
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
  //  Generated on 03-Sep-2019 15:12:31
  l2_tmp = A_init * A_init;
  l148 = A_min * A_min;
  l5_tmp = A_max * A_max;
  l6 = rt_powd_snf(A_max, 3.0);
  l7 = J_min * J_min;
  l9 = J_max * J_max;
  l10 = T * T;
  l140 = A_init * A_max;
  l15_tmp = l140 * J_min;
  b_l15_tmp = l15_tmp * 2.0;
  l16_tmp = A_min * A_max;
  b_l16_tmp = l16_tmp * J_max;
  c_l16_tmp = b_l16_tmp * 2.0;
  l145 = A_max * J_min;
  l18_tmp = l145 * J_max;
  b_l18_tmp = l18_tmp * 2.0;
  l142 = J_min * J_max;
  l21_tmp = l142 * V_init * 2.0;
  l22_tmp = l142 * V_min * 2.0;
  l23_tmp = J_min * l2_tmp;
  l24_tmp = J_min * l148;
  l25_tmp = J_max * l148;
  l26_tmp = J_min * l5_tmp;
  l27_tmp = J_max * l5_tmp;
  a_tmp = A_init + -A_max;
  l35 = A_min + -A_max;
  l50 = a_tmp * a_tmp;
  l51 = l35 * l35;
  l58_tmp = A_min * J_min * J_max;
  l58 = l58_tmp * 2.0 + -b_l18_tmp;
  l65 = l58 * l58;
  l66 = 1.0 / rt_powd_snf(l58, 3.0);
  l92 = (((((l22_tmp + l23_tmp) + l24_tmp) + l27_tmp) + -l21_tmp) + -l25_tmp) +
        -l26_tmp;
  b_a_tmp = ((((((((b_l15_tmp - c_l16_tmp) + l21_tmp) - l22_tmp) - l23_tmp) -
                l24_tmp) +
               l25_tmp) +
              l27_tmp) +
             T * b_l18_tmp) +
            -l26_tmp;
  l103 = b_a_tmp * b_a_tmp;
  l134_tmp = A_init * A_min;
  b_l134_tmp = l134_tmp * A_max;
  l142 = l15_tmp * J_max;
  l134_tmp = l134_tmp * J_min * J_max;
  c_l134_tmp = l18_tmp * l35;
  d_l134_tmp = 1.0 / A_max * (1.0 / J_min);
  l134_tmp_tmp = A_init * l5_tmp;
  e_l134_tmp = l134_tmp_tmp * l7;
  f_l134_tmp = l5_tmp * l7;
  l134 =
      ((((((((-(b_l134_tmp * l7 * l9 * 24.0) + A_init * l148 * l7 * l9 * 12.0) +
             e_l134_tmp * l9 * 12.0) +
            A_init * l65 * 3.0) +
           f_l134_tmp * l9 * l35 * 12.0) +
          l142 * l58 * 12.0) +
         -(l134_tmp * l58 * 12.0)) +
        l35 * l65 * 3.0) +
       c_l134_tmp * l58 * 12.0) +
      d_l134_tmp * l65 * b_a_tmp * 1.5;
  l137_tmp = l145 * l66;
  l137 = l137_tmp * l134 * 0.66666666666666663;
  l150 = A_init * l58;
  l142_tmp = l18_tmp * V_init;
  l142_tmp_tmp = l140 * J_max;
  l15_tmp = l142_tmp_tmp * l35;
  l148 = l145 * l50;
  l140 = A_max * J_max;
  l145 = l140 * l51;
  b_l142_tmp = A_init * l9;
  l142 = ((((((((((((((((((-(l16_tmp * V_init * l7 * l9 * 24.0) +
                           V_init * l5_tmp * l7 * l9 * 24.0) +
                          -(b_l134_tmp * J_min * l9 * l35 * 24.0)) +
                         b_l142_tmp * l26_tmp * l35 * 24.0) +
                        -(b_l16_tmp * l7 * l50 * 12.0)) +
                       l7 * l27_tmp * l50 * 12.0) +
                      l9 * l26_tmp * l51 * 12.0) +
                     l142_tmp * l58 * 12.0) +
                    l15_tmp * l58 * 12.0) +
                   l148 * l58 * 6.0) +
                  l145 * l58 * 6.0) +
                 l142 * l92 * 12.0) +
                -(l134_tmp * l92 * 12.0)) +
               l150 * l92 * 6.0) +
              l134_tmp * b_a_tmp * -12.0) +
             l142 * b_a_tmp * 12.0) +
            c_l134_tmp * b_a_tmp * 12.0) +
           l150 * b_a_tmp * 6.0) +
          l35 * l58 * b_a_tmp * 6.0) +
         d_l134_tmp * l58 * l103 * 1.5;
  l145 = (((((((((((((((((P_init * l5_tmp * l7 * l9 * 24.0 +
                          l5_tmp * l9 * rt_powd_snf(l35, 3.0) * 4.0) +
                         V_init * l9 * l26_tmp * l35 * 24.0) +
                        e_l134_tmp * l50 * 12.0) +
                       l134_tmp_tmp * l9 * l51 * 12.0) +
                      J_max * l26_tmp * l35 * l50 * 12.0) +
                     A_init * (l92 * l92) * 3.0) +
                    l142_tmp * l92 * 12.0) +
                   l15_tmp * l92 * 12.0) +
                  l148 * l92 * 6.0) +
                 A_init * l103 * 3.0) +
                l142_tmp * b_a_tmp * 12.0) +
               l35 * l103 * 3.0) +
              l15_tmp * b_a_tmp * 12.0) +
             l148 * b_a_tmp * 6.0) +
            l145 * b_a_tmp * 6.0) +
           d_l134_tmp * rt_powd_snf(b_a_tmp, 3.0) / 2.0) +
          -(f_l134_tmp *
            (((((((((((A_init * J_max * V_init * 6.0 +
                       rt_powd_snf(A_init, 3.0) * 10.0) +
                      -(l6 * 4.0)) +
                     -(l140 * V_init * 6.0)) +
                    P_wayp * l9 * 6.0) +
                   l134_tmp_tmp * 18.0) +
                  T * l27_tmp * 6.0) +
                 -(A_max * l2_tmp * 24.0)) +
                -(l142_tmp_tmp * T * 18.0)) +
               J_max * T * l2_tmp * 12.0) +
              b_l142_tmp * l10 * 3.0) +
             -(A_max * l9 * l10 * 3.0)) *
            4.0)) +
         A_init * l92 * b_a_tmp * 6.0;
  l140 = l6 * rt_powd_snf(J_min, 3.0) * rt_powd_snf(l66, 3.0) *
         rt_powd_snf(l134, 3.0) * 0.29629629629629628;
  l15_tmp = f_l134_tmp * (1.0 / rt_powd_snf(l65, 3.0));
  l148 = l15_tmp * l134 * l142 * 0.66666666666666663;
  l150 = l15_tmp * (l134 * l134) * 0.44444444444444442 +
         -(l137_tmp * l142 * 0.66666666666666663);
  l142 = (l140 + l137_tmp * l145) + -l148;
  l158.re = -rt_powd_snf(l150, 3.0) + l142 * l142;
  l158.im = 0.0;
  coder::internal::scalar::b_sqrt(&l158);
  b_l140.re = ((-l140 + J_min * -A_max * l66 * l145) + l148) + l158.re;
  b_l140.im = l158.im;
  l158 = coder::power(b_l140);
  if (l158.im == 0.0) {
    b_l142_tmp = l158.re / 2.0;
    l142_tmp_tmp = 0.0;
    l145 = 1.0 / l158.re;
    l142 = 0.0;
  } else if (l158.re == 0.0) {
    b_l142_tmp = 0.0;
    l142_tmp_tmp = l158.im / 2.0;
    l145 = 0.0;
    l142 = -(1.0 / l158.im);
  } else {
    b_l142_tmp = l158.re / 2.0;
    l142_tmp_tmp = l158.im / 2.0;
    l148 = std::abs(l158.re);
    l15_tmp = std::abs(l158.im);
    if (l148 > l15_tmp) {
      l142 = l158.im / l158.re;
      l15_tmp = l158.re + l142 * l158.im;
      l145 = (l142 * 0.0 + 1.0) / l15_tmp;
      l142 = (0.0 - l142) / l15_tmp;
    } else if (l15_tmp == l148) {
      if (l158.re > 0.0) {
        l142 = 0.5;
      } else {
        l142 = -0.5;
      }
      if (l158.im > 0.0) {
        l15_tmp = 0.5;
      } else {
        l15_tmp = -0.5;
      }
      l145 = (l142 + 0.0 * l15_tmp) / l148;
      l142 = (0.0 * l142 - l15_tmp) / l148;
    } else {
      l142 = l158.re / l158.im;
      l15_tmp = l158.im + l142 * l158.re;
      l145 = l142 / l15_tmp;
      l142 = (l142 * 0.0 - 1.0) / l15_tmp;
    }
  }
  l148 = l150 * l145;
  l142 *= l150;
  if (l142 == 0.0) {
    l140 = l148 / 2.0;
    l145 = 0.0;
  } else if (l148 == 0.0) {
    l140 = 0.0;
    l145 = l142 / 2.0;
  } else {
    l140 = l148 / 2.0;
    l145 = l142 / 2.0;
  }
  t6[0].re = (-l137 + l158.re) + l148;
  t6[0].im = l158.im + l142;
  l148 = 1.7320508075688772 * (l158.re - l148);
  l15_tmp = 1.7320508075688772 * (l158.im - l142);
  b_l142_tmp = (-l137 + -b_l142_tmp) + -l140;
  t6[1].re = b_l142_tmp + (l148 * 0.0 - l15_tmp * 0.5);
  l142 = -l142_tmp_tmp + -l145;
  t6[1].im = l142 + (l148 * 0.5 + l15_tmp * 0.0);
  t6[2].re = b_l142_tmp + (l148 * -0.0 - l15_tmp * -0.5);
  t6[2].im = l142 + (l148 * -0.5 + l15_tmp * -0.0);
  l148 = 1.0 / J_max * a_tmp;
  l158.re =
      (((((l24_tmp - l26_tmp) - l25_tmp) + l27_tmp) + A_init * A_init * J_min) -
       l21_tmp) +
      l22_tmp;
  l145 = 2.0 * (l58_tmp * t6[0].re);
  l142 = 2.0 * (l58_tmp * t6[0].im);
  t[0].re = -l148;
  t[0].im = 0.0;
  l140 = l158.re - l145;
  if (0.0 - l142 == 0.0) {
    t[3].re = l140 / b_l18_tmp;
    t[3].im = 0.0;
  } else if (l140 == 0.0) {
    t[3].re = 0.0;
    t[3].im = (0.0 - l142) / b_l18_tmp;
  } else {
    t[3].re = l140 / b_l18_tmp;
    t[3].im = (0.0 - l142) / b_l18_tmp;
  }
  b_l142_tmp = -((A_init - A_min) + J_max * -l148) / J_min;
  t[6].re = b_l142_tmp;
  t[6].im = 0.0;
  t[9].re = 0.0;
  t[9].im = 0.0;
  t[12].re = 0.0;
  t[12].im = 0.0;
  t[15] = t6[0];
  l15_tmp =
      ((((((((l24_tmp + l26_tmp) - l25_tmp) - l27_tmp) + l23_tmp) - b_l15_tmp) +
         c_l16_tmp) -
        l21_tmp) +
       l22_tmp) -
      l18_tmp * T * 2.0;
  l140 = ((l15_tmp - l145) + l18_tmp * t6[0].re * 2.0) * -0.5;
  l142 = ((0.0 - l142) + l18_tmp * t6[0].im * 2.0) * -0.5;
  if (l142 == 0.0) {
    t[18].re = l140 / l18_tmp;
    t[18].im = 0.0;
  } else if (l140 == 0.0) {
    t[18].re = 0.0;
    t[18].im = l142 / l18_tmp;
  } else {
    t[18].re = l140 / l18_tmp;
    t[18].im = l142 / l18_tmp;
  }
  l145 = 2.0 * (l58_tmp * t6[1].re);
  l142 = 2.0 * (l58_tmp * t6[1].im);
  t[1].re = -l148;
  t[1].im = 0.0;
  l140 = l158.re - l145;
  if (0.0 - l142 == 0.0) {
    t[4].re = l140 / b_l18_tmp;
    t[4].im = 0.0;
  } else if (l140 == 0.0) {
    t[4].re = 0.0;
    t[4].im = (0.0 - l142) / b_l18_tmp;
  } else {
    t[4].re = l140 / b_l18_tmp;
    t[4].im = (0.0 - l142) / b_l18_tmp;
  }
  t[7].re = b_l142_tmp;
  t[7].im = 0.0;
  t[10].re = 0.0;
  t[10].im = 0.0;
  t[13].re = 0.0;
  t[13].im = 0.0;
  t[16] = t6[1];
  l140 = ((l15_tmp - l145) + l18_tmp * t6[1].re * 2.0) * -0.5;
  l142 = ((0.0 - l142) + l18_tmp * t6[1].im * 2.0) * -0.5;
  if (l142 == 0.0) {
    t[19].re = l140 / l18_tmp;
    t[19].im = 0.0;
  } else if (l140 == 0.0) {
    t[19].re = 0.0;
    t[19].im = l142 / l18_tmp;
  } else {
    t[19].re = l140 / l18_tmp;
    t[19].im = l142 / l18_tmp;
  }
  l145 = 2.0 * (l58_tmp * t6[2].re);
  l142 = 2.0 * (l58_tmp * t6[2].im);
  t[2].re = -l148;
  t[2].im = 0.0;
  l140 = l158.re - l145;
  if (0.0 - l142 == 0.0) {
    t[5].re = l140 / b_l18_tmp;
    t[5].im = 0.0;
  } else if (l140 == 0.0) {
    t[5].re = 0.0;
    t[5].im = (0.0 - l142) / b_l18_tmp;
  } else {
    t[5].re = l140 / b_l18_tmp;
    t[5].im = (0.0 - l142) / b_l18_tmp;
  }
  t[8].re = b_l142_tmp;
  t[8].im = 0.0;
  t[11].re = 0.0;
  t[11].im = 0.0;
  t[14].re = 0.0;
  t[14].im = 0.0;
  t[17] = t6[2];
  l140 = ((l15_tmp - l145) + l18_tmp * t6[2].re * 2.0) * -0.5;
  l142 = ((0.0 - l142) + l18_tmp * t6[2].im * 2.0) * -0.5;
  if (l142 == 0.0) {
    t[20].re = l140 / l18_tmp;
    t[20].im = 0.0;
  } else if (l140 == 0.0) {
    t[20].re = 0.0;
    t[20].im = l142 / l18_tmp;
  } else {
    t[20].re = l140 / l18_tmp;
    t[20].im = l142 / l18_tmp;
  }
}

// End of code generation (abcefg_T_P.cpp)
