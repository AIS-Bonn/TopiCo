//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// abcdeg_T_AP.cpp
//
// Code generation for function 'abcdeg_T_AP'
//

// Include files
#include "abcdeg_T_AP.h"
#include "power.h"
#include "rt_nonfinite.h"
#include "sqrt.h"
#include "topico_wrapper_data.h"
#include "topico_wrapper_rtwutil.h"
#include <cmath>

// Function Definitions
void abcdeg_T_AP(double P_init, double V_init, double A_init, double P_wayp,
                 double A_wayp, double V_max, double A_max, double J_max,
                 double J_min, double T, creal_T t[21])
{
  creal_T t4[3];
  creal_T l140;
  creal_T l157;
  double b_l22_tmp;
  double l10;
  double l108_tmp;
  double l114_tmp;
  double l117_tmp;
  double l12;
  double l13;
  double l130_tmp;
  double l137;
  double l16;
  double l164_im;
  double l17;
  double l19;
  double l22;
  double l22_tmp;
  double l23;
  double l24;
  double l25;
  double l26_tmp;
  double l29_tmp;
  double l2_tmp;
  double l31;
  double l32;
  double l32_tmp;
  double l32_tmp_tmp;
  double l34;
  double l37;
  double l38;
  double l4_tmp;
  double l5;
  double l55_tmp;
  double l55_tmp_tmp;
  double l6;
  double l60_tmp;
  double l60_tmp_tmp;
  double l7;
  double l8;
  double l80;
  double l9;
  double l94;
  double l95;
  double l96;
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
  //  Generated on 29-Aug-2019 11:03:57
  l2_tmp = A_init * A_init;
  l4_tmp = A_max * A_max;
  l5 = rt_powd_snf(A_max, 3.0);
  l7 = J_min * J_min;
  l8 = rt_powd_snf(J_min, 3.0);
  l10 = J_max * J_max;
  l12 = rt_powd_snf(J_max, 3.0);
  l16 = 1.0 / A_max;
  l19 = A_init * A_max * J_min * 2.0;
  l22_tmp = J_min * J_max;
  b_l22_tmp = l22_tmp * V_init;
  l22 = b_l22_tmp * 2.0;
  l23 = l22_tmp * V_max * 2.0;
  l32_tmp_tmp = A_max * J_min;
  l32_tmp = l32_tmp_tmp * J_max;
  l32 = l32_tmp * T * 2.0;
  l6 = l4_tmp * l4_tmp;
  l9 = l7 * l7;
  l13 = l10 * l10;
  l17 = 1.0 / l4_tmp;
  l24 = J_min * l2_tmp;
  l25 = J_min * l4_tmp;
  l26_tmp = J_max * l4_tmp;
  l29_tmp = A_init + -A_max;
  l31 = J_min + -J_max;
  l34 = l4_tmp * -J_max;
  l37 = l31 * l31;
  l38 = rt_powd_snf(l31, 3.0);
  l164_im = A_max * A_wayp;
  l94 =
      ((((((l164_im * J_min * 2.0 + l23) + l24) + l25) + -l19) + -l22) + l34) +
      -l32;
  l95 =
      ((((((l164_im * J_max * 2.0 + l23) + l24) + l25) + -l19) + -l22) + l34) +
      -l32;
  l19 = J_min * l5;
  l32 = J_max * l5;
  l55_tmp_tmp = A_max * J_max;
  l55_tmp = l55_tmp_tmp * l2_tmp;
  l60_tmp_tmp = A_max * V_init;
  l60_tmp = l60_tmp_tmp * l7;
  l23 = (((l23 + l24) + l26_tmp) + -l22) + -l25;
  l96 = l94 * l94;
  l114_tmp = J_max * l25 * l37;
  l117_tmp = J_max * V_init;
  l130_tmp = l7 * l10 * l16;
  l108_tmp = l7 * l17;
  l80 = 1.0 / ((A_max * l8 * rt_powd_snf(J_max, 5.0) * 4.0 +
                A_max * rt_powd_snf(J_min, 5.0) * l12 * 8.0) +
               -(A_max * l9 * l13 * 12.0));
  l164_im = l9 * l10;
  l9 = l8 * l12;
  l31 = (((l7 * l13 * l94 * 6.0 + l164_im * l94 * 6.0) + l164_im * l95 * 6.0) +
         -(l9 * l95 * 6.0)) +
        -(l9 * l94 * 12.0);
  l13 = l80 * l80;
  l137 = l80 * l31 / 3.0;
  l34 = rt_powd_snf(l80, 3.0) * rt_powd_snf(l31, 3.0) / 27.0;
  l164_im = ((((((((((((((((l19 * l10 * l38 * 12.0 + l19 * l12 * l37 * 12.0) -
                           l32 * l7 * l38 * 12.0) +
                          l32 * l8 * l37 * 12.0) +
                         l55_tmp * l7 * l38 * 12.0) -
                        l55_tmp * l8 * l37 * 12.0) -
                       l60_tmp * l10 * l38 * 24.0) -
                      l60_tmp * l12 * l37 * 24.0) +
                     l60_tmp_tmp * l8 * l10 * l37 * 24.0) -
                    l5 * l7 * l10 * l37 * 24.0) +
                   A_max * l2_tmp * l7 * l10 * l37 * 12.0) -
                  l32_tmp * l38 * l23 * 12.0) -
                 l32_tmp_tmp * l10 * l37 * l23 * 12.0) +
                l55_tmp_tmp * l7 * l37 * l23 * 12.0) -
               J_min * l12 * l16 * l96 * 3.0) +
              l130_tmp * l96 * 3.0) -
             J_max * l8 * l16 * l94 * l95 * 6.0) +
            l130_tmp * l94 * l95 * 6.0;
  l22 = l13 * l31 * l164_im;
  l19 =
      l80 *
      (((((((((((((((((((((((((((((l22_tmp * l6 * l38 * 12.0 -
                                   l6 * l10 * l38 * 8.0) -
                                  A_init * l5 * l7 * l38 * 12.0) -
                                 A_max * rt_powd_snf(A_init, 3.0) * l7 * l38 *
                                     12.0) -
                                l24 * l26_tmp * l38 * 12.0) -
                               A_max * P_init * l7 * l10 * l38 * 24.0) +
                              A_max * P_wayp * l7 * l10 * l38 * 24.0) +
                             V_init * l10 * l25 * l38 * 24.0) +
                            l2_tmp * l4_tmp * l7 * l38 * 24.0) +
                           A_max * l7 * rt_powd_snf(l29_tmp, 3.0) * l38 * 4.0) +
                          l55_tmp_tmp * V_init * l7 * l29_tmp * l38 * 24.0) -
                         l38 * (l23 * l23) * 3.0) -
                        b_l22_tmp * l38 * l23 * 12.0) +
                       l24 * l38 * l23 * 6.0) -
                      l25 * l38 * l23 * 6.0) +
                     l26_tmp * l38 * l23 * 12.0) +
                    l108_tmp * rt_powd_snf(l95, 3.0) / 2.0) -
                   l10 * l17 * rt_powd_snf(l94, 3.0) / 2.0) +
                  J_max * l24 * l37 * l94 * 6.0) -
                 l114_tmp * l94 * 6.0) -
                l114_tmp * l95 * 6.0) -
               J_min * V_init * l10 * l37 * l94 * 12.0) +
              l117_tmp * l7 * l37 * l95 * 12.0) -
             l2_tmp * l7 * l37 * l95 * 6.0) +
            l4_tmp * l7 * l37 * l95 * 6.0) +
           l4_tmp * l10 * l37 * l94 * 6.0) +
          J_min * l37 * l23 * l95 * 6.0) -
         J_max * l37 * l23 * l94 * 6.0) +
        l22_tmp * l17 * l95 * l96 * 1.5) -
       l108_tmp * l94 * (l95 * l95) * 1.5);
  l32 = (l34 + l22 / 6.0) + l19 * -0.5;
  l13 = l13 * (l31 * l31) / 9.0 + l80 * l164_im / 3.0;
  l157.re = -rt_powd_snf(l13, 3.0) + l32 * l32;
  l157.im = 0.0;
  coder::internal::scalar::b_sqrt(&l157);
  l140.re = ((-l34 + l22 * -0.16666666666666666) + l19 / 2.0) + l157.re;
  l140.im = l157.im;
  l157 = coder::power(l140);
  if (l157.im == 0.0) {
    l22 = 1.0 / l157.re;
    l31 = 0.0;
    l34 = l157.re / 2.0;
    l23 = 0.0;
  } else if (l157.re == 0.0) {
    l22 = 0.0;
    l31 = -(1.0 / l157.im);
    l34 = 0.0;
    l23 = l157.im / 2.0;
  } else {
    l32 = std::abs(l157.re);
    l31 = std::abs(l157.im);
    if (l32 > l31) {
      l31 = l157.im / l157.re;
      l164_im = l157.re + l31 * l157.im;
      l22 = (l31 * 0.0 + 1.0) / l164_im;
      l31 = (0.0 - l31) / l164_im;
    } else if (l31 == l32) {
      if (l157.re > 0.0) {
        l31 = 0.5;
      } else {
        l31 = -0.5;
      }
      if (l157.im > 0.0) {
        l19 = 0.5;
      } else {
        l19 = -0.5;
      }
      l22 = (l31 + 0.0 * l19) / l32;
      l31 = (0.0 * l31 - l19) / l32;
    } else {
      l31 = l157.re / l157.im;
      l164_im = l157.im + l31 * l157.re;
      l22 = l31 / l164_im;
      l31 = (l31 * 0.0 - 1.0) / l164_im;
    }
    l34 = l157.re / 2.0;
    l23 = l157.im / 2.0;
  }
  l19 = l13 * l22;
  l9 = l13 * l31;
  l130_tmp = 1.7320508075688772 * (l157.re + -l22 * l13);
  l164_im = 1.7320508075688772 * (l157.im + -l31 * l13);
  l22 = l130_tmp * 0.0 - l164_im * 0.5;
  l164_im = l130_tmp * 0.5 + l164_im * 0.0;
  t4[0].re = (-l137 + l157.re) + l19;
  t4[0].im = l157.im + l9;
  l32 = (-l137 + -l34) + -0.5 * l19;
  t4[1].re = l32 - l22;
  l31 = -l23 + -0.5 * l9;
  t4[1].im = l31 - l164_im;
  t4[2].re = l32 + l22;
  t4[2].im = l31 + l164_im;
  l13 = 1.0 / A_max * (1.0 / J_max) *
        ((((l2_tmp + J_max * V_max * 2.0) + -(l117_tmp * 2.0)) + -l4_tmp) +
         l26_tmp * (1.0 / J_min)) /
        2.0;
  l22 = 1.0 / J_max;
  l34 = J_min * (J_min * l22 - 1.0);
  l31 = A_max * (1.0 / J_min);
  l6 = -(1.0 / J_max * l29_tmp);
  t[0].re = l6;
  t[0].im = 0.0;
  t[1].re = l6;
  t[1].im = 0.0;
  t[2].re = l6;
  t[2].im = 0.0;
  t[6].re = -l31;
  t[6].im = 0.0;
  t[7].re = -l31;
  t[7].im = 0.0;
  t[8].re = -l31;
  t[8].im = 0.0;
  l32 = A_wayp * l22;
  l19 = ((A_init * J_min - l32_tmp_tmp) + l55_tmp_tmp) + l22_tmp * T;
  l31 = -(l22 * (l19 - l22_tmp * ((l13 + t4[0].re) + l32)));
  l164_im = -(l22 * (0.0 - l22_tmp * t4[0].im));
  if (l164_im == 0.0) {
    l130_tmp = l31 / l34;
    l164_im = 0.0;
  } else if (l31 == 0.0) {
    l130_tmp = 0.0;
    l164_im /= l34;
  } else {
    l130_tmp = l31 / l34;
    l164_im /= l34;
  }
  t[3].re = l13;
  t[3].im = 0.0;
  t[9] = t4[0];
  t[12].re = l130_tmp;
  t[12].im = l164_im;
  t[15].re = 0.0;
  t[15].im = 0.0;
  l31 = A_wayp - J_min * l130_tmp;
  l164_im = 0.0 - J_min * l164_im;
  if (l164_im == 0.0) {
    t[18].re = l31 / J_max;
    t[18].im = 0.0;
  } else if (l31 == 0.0) {
    t[18].re = 0.0;
    t[18].im = l164_im / J_max;
  } else {
    t[18].re = l31 / J_max;
    t[18].im = l164_im / J_max;
  }
  l31 = -(l22 * (l19 - l22_tmp * ((l13 + t4[1].re) + l32)));
  l164_im = -(l22 * (0.0 - l22_tmp * t4[1].im));
  if (l164_im == 0.0) {
    l130_tmp = l31 / l34;
    l164_im = 0.0;
  } else if (l31 == 0.0) {
    l130_tmp = 0.0;
    l164_im /= l34;
  } else {
    l130_tmp = l31 / l34;
    l164_im /= l34;
  }
  t[4].re = l13;
  t[4].im = 0.0;
  t[10] = t4[1];
  t[13].re = l130_tmp;
  t[13].im = l164_im;
  t[16].re = 0.0;
  t[16].im = 0.0;
  l31 = A_wayp - J_min * l130_tmp;
  l164_im = 0.0 - J_min * l164_im;
  if (l164_im == 0.0) {
    t[19].re = l31 / J_max;
    t[19].im = 0.0;
  } else if (l31 == 0.0) {
    t[19].re = 0.0;
    t[19].im = l164_im / J_max;
  } else {
    t[19].re = l31 / J_max;
    t[19].im = l164_im / J_max;
  }
  l31 = -(l22 * (l19 - l22_tmp * ((l13 + t4[2].re) + l32)));
  l164_im = -(l22 * (0.0 - l22_tmp * t4[2].im));
  if (l164_im == 0.0) {
    l130_tmp = l31 / l34;
    l164_im = 0.0;
  } else if (l31 == 0.0) {
    l130_tmp = 0.0;
    l164_im /= l34;
  } else {
    l130_tmp = l31 / l34;
    l164_im /= l34;
  }
  t[5].re = l13;
  t[5].im = 0.0;
  t[11] = t4[2];
  t[14].re = l130_tmp;
  t[14].im = l164_im;
  t[17].re = 0.0;
  t[17].im = 0.0;
  l31 = A_wayp - J_min * l130_tmp;
  l164_im = 0.0 - J_min * l164_im;
  if (l164_im == 0.0) {
    t[20].re = l31 / J_max;
    t[20].im = 0.0;
  } else if (l31 == 0.0) {
    t[20].re = 0.0;
    t[20].im = l164_im / J_max;
  } else {
    t[20].re = l31 / J_max;
    t[20].im = l164_im / J_max;
  }
}

// End of code generation (abcdeg_T_AP.cpp)
