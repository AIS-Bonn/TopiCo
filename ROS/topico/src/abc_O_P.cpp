//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// abc_O_P.cpp
//
// Code generation for function 'abc_O_P'
//

// Include files
#include "abc_O_P.h"
#include "power.h"
#include "rt_nonfinite.h"
#include "sqrt.h"
#include "topico_wrapper_data.h"
#include "topico_wrapper_rtwutil.h"
#include <cmath>

// Function Definitions
void abc_O_P(double P_init, double V_init, double A_init, double P_wayp,
             double V_max, double A_max, double J_max, double J_min,
             creal_T t[21])
{
  creal_T t3[3];
  creal_T b_l24;
  creal_T l60;
  double l13;
  double l14;
  double l15;
  double l17;
  double l19;
  double l2;
  double l24;
  double l5;
  double l6;
  double l67_re;
  double l7;
  double l8_tmp;
  double l9;
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
  //  Generated on 28-Aug-2019 17:25:45
  l2 = A_init * A_init;
  l5 = A_max * A_max;
  l6 = rt_powd_snf(A_max, 3.0);
  l8_tmp = J_min * J_min;
  l9 = J_max * J_max;
  l13 = 1.0 / J_min;
  l7 = l5 * l5;
  l14 = 1.0 / l8_tmp;
  l15 = rt_powd_snf(l13, 3.0);
  l17 = 1.0 / l9;
  l19 = A_max * l13;
  l24 = l6 * l15;
  l13 = J_min * l6 * l9 * 12.0 + A_max * V_max * l8_tmp * l9 * 24.0;
  l6 = l14 * l14 * l17 * l13 / 8.0;
  l67_re = 1.0 / A_max * l15 * l17;
  l14 = l5 * l14 + -(l67_re * l13 / 12.0);
  l17 = J_max * V_init;
  l13 = l8_tmp * l9;
  l13 = l67_re *
        ((((((((((((l7 * l8_tmp + l7 * l9 * 3.0) + -(l2 * l2 * l8_tmp * 3.0)) +
                  A_max * rt_powd_snf(A_init, 3.0) * l8_tmp * 8.0) +
                 -(A_init * A_max * J_max * V_init * l8_tmp * 24.0)) +
                A_max * P_init * l8_tmp * l9 * 24.0) +
               l17 * l2 * l8_tmp * 12.0) +
              l17 * l5 * l8_tmp * 12.0) +
             J_min * V_max * l5 * l9 * 12.0) +
            -(A_max * P_wayp * l8_tmp * l9 * 24.0)) +
           -(l2 * l5 * l8_tmp * 6.0)) +
          l13 * (V_max * V_max) * 12.0) +
         -(l13 * (V_init * V_init) * 12.0)) /
        8.0;
  l17 = (l24 + -l6) + l13;
  l60.re = -rt_powd_snf(l14, 3.0) + l17 * l17;
  l60.im = 0.0;
  coder::internal::scalar::b_sqrt(&l60);
  b_l24.re = ((-l24 + l6) + -l13) + l60.re;
  b_l24.im = l60.im;
  l60 = coder::power(b_l24);
  if (l60.im == 0.0) {
    l5 = l60.re / 2.0;
    l9 = 0.0;
    l7 = 1.0 / l60.re;
    l17 = 0.0;
  } else if (l60.re == 0.0) {
    l5 = 0.0;
    l9 = l60.im / 2.0;
    l7 = 0.0;
    l17 = -(1.0 / l60.im);
  } else {
    l5 = l60.re / 2.0;
    l9 = l60.im / 2.0;
    l15 = std::abs(l60.re);
    l13 = std::abs(l60.im);
    if (l15 > l13) {
      l13 = l60.im / l60.re;
      l17 = l60.re + l13 * l60.im;
      l7 = (l13 * 0.0 + 1.0) / l17;
      l17 = (0.0 - l13) / l17;
    } else if (l13 == l15) {
      if (l60.re > 0.0) {
        l13 = 0.5;
      } else {
        l13 = -0.5;
      }
      if (l60.im > 0.0) {
        l17 = 0.5;
      } else {
        l17 = -0.5;
      }
      l7 = (l13 + 0.0 * l17) / l15;
      l17 = (0.0 * l13 - l17) / l15;
    } else {
      l13 = l60.re / l60.im;
      l17 = l60.im + l13 * l60.re;
      l7 = l13 / l17;
      l17 = (l13 * 0.0 - 1.0) / l17;
    }
  }
  l24 = l14 * l7;
  l13 = l14 * l17;
  if (l13 == 0.0) {
    l15 = l24 / 2.0;
    l14 = 0.0;
  } else if (l24 == 0.0) {
    l15 = 0.0;
    l14 = l13 / 2.0;
  } else {
    l15 = l24 / 2.0;
    l14 = l13 / 2.0;
  }
  l7 = 1.7320508075688772 * (l60.re + -l24);
  l17 = 1.7320508075688772 * (l60.im + -l13);
  l2 = l7 * 0.0 - l17 * 0.5;
  l67_re = l7 * 0.5 + l17 * 0.0;
  t3[0].re = (-l19 + l60.re) + l24;
  t3[0].im = l60.im + l13;
  l17 = (-l19 + -l5) + -l15;
  t3[1].re = l17 - l2;
  l13 = -l9 + -l14;
  t3[1].im = l13 - l67_re;
  t3[2].re = l17 + l2;
  t3[2].im = l13 + l67_re;
  l17 = A_init + -A_max;
  l5 = A_max * J_min * J_max;
  l6 = -(1.0 / J_max * l17);
  l60.re = J_min * (l17 * l17);
  l14 = l8_tmp * J_max;
  l13 = J_min * J_max;
  l2 = l13 * V_init * 2.0;
  l24 = l13 * V_max * 2.0;
  l67_re = A_init * J_min * l17 * 2.0;
  t[0].re = l6;
  t[0].im = 0.0;
  t[1].re = l6;
  t[1].im = 0.0;
  t[2].re = l6;
  t[2].im = 0.0;
  l7 = A_max + J_min * t3[0].re;
  l17 = J_min * t3[0].im;
  l13 = l7 * l17;
  l15 = t3[0].re * t3[0].im;
  l17 = ((((((l60.re - J_max * (l7 * l7 - l17 * l17)) +
             l14 * (t3[0].re * t3[0].re - t3[0].im * t3[0].im)) +
            l2) -
           l24) -
          l67_re) +
         l5 * t3[0].re * 2.0) *
        -0.5;
  l13 = (((0.0 - J_max * (l13 + l13)) + l14 * (l15 + l15)) +
         l5 * t3[0].im * 2.0) *
        -0.5;
  if (l13 == 0.0) {
    t[3].re = l17 / l5;
    t[3].im = 0.0;
  } else if (l17 == 0.0) {
    t[3].re = 0.0;
    t[3].im = l13 / l5;
  } else {
    t[3].re = l17 / l5;
    t[3].im = l13 / l5;
  }
  t[6] = t3[0];
  t[9].re = 0.0;
  t[9].im = 0.0;
  t[12].re = 0.0;
  t[12].im = 0.0;
  t[15].re = 0.0;
  t[15].im = 0.0;
  t[18].re = 0.0;
  t[18].im = 0.0;
  l7 = A_max + J_min * t3[1].re;
  l17 = J_min * t3[1].im;
  l13 = l7 * l17;
  l15 = t3[1].re * t3[1].im;
  l17 = ((((((l60.re - J_max * (l7 * l7 - l17 * l17)) +
             l14 * (t3[1].re * t3[1].re - t3[1].im * t3[1].im)) +
            l2) -
           l24) -
          l67_re) +
         l5 * t3[1].re * 2.0) *
        -0.5;
  l13 = (((0.0 - J_max * (l13 + l13)) + l14 * (l15 + l15)) +
         l5 * t3[1].im * 2.0) *
        -0.5;
  if (l13 == 0.0) {
    t[4].re = l17 / l5;
    t[4].im = 0.0;
  } else if (l17 == 0.0) {
    t[4].re = 0.0;
    t[4].im = l13 / l5;
  } else {
    t[4].re = l17 / l5;
    t[4].im = l13 / l5;
  }
  t[7] = t3[1];
  t[10].re = 0.0;
  t[10].im = 0.0;
  t[13].re = 0.0;
  t[13].im = 0.0;
  t[16].re = 0.0;
  t[16].im = 0.0;
  t[19].re = 0.0;
  t[19].im = 0.0;
  l7 = A_max + J_min * t3[2].re;
  l17 = J_min * t3[2].im;
  l13 = l7 * l17;
  l15 = t3[2].re * t3[2].im;
  l17 = ((((((l60.re - J_max * (l7 * l7 - l17 * l17)) +
             l14 * (t3[2].re * t3[2].re - t3[2].im * t3[2].im)) +
            l2) -
           l24) -
          l67_re) +
         l5 * t3[2].re * 2.0) *
        -0.5;
  l13 = (((0.0 - J_max * (l13 + l13)) + l14 * (l15 + l15)) +
         l5 * t3[2].im * 2.0) *
        -0.5;
  if (l13 == 0.0) {
    t[5].re = l17 / l5;
    t[5].im = 0.0;
  } else if (l17 == 0.0) {
    t[5].re = 0.0;
    t[5].im = l13 / l5;
  } else {
    t[5].re = l17 / l5;
    t[5].im = l13 / l5;
  }
  t[8] = t3[2];
  t[11].re = 0.0;
  t[11].im = 0.0;
  t[14].re = 0.0;
  t[14].im = 0.0;
  t[17].re = 0.0;
  t[17].im = 0.0;
  t[20].re = 0.0;
  t[20].im = 0.0;
}

// End of code generation (abc_O_P.cpp)
