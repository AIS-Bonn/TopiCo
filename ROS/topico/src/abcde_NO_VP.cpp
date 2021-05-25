//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// abcde_NO_VP.cpp
//
// Code generation for function 'abcde_NO_VP'
//

// Include files
#include "abcde_NO_VP.h"
#include "power.h"
#include "rt_nonfinite.h"
#include "sqrt.h"
#include "topico_wrapper_data.h"
#include "topico_wrapper_rtwutil.h"
#include <cmath>

// Function Definitions
void abcde_NO_VP(double P_init, double V_init, double A_init, double P_wayp,
                 double V_wayp, double V_max, double A_min, double J_max,
                 double J_min, creal_T t[14])
{
  creal_T t5[2];
  creal_T x[2];
  creal_T b_A_min;
  creal_T l7;
  double A_init_re;
  double A_min_re;
  double A_min_tmp;
  double J_max_re;
  double J_min_re;
  double b_J_min_re;
  double c_A_min;
  double l13;
  double l3;
  double l4;
  double l5;
  double l6;
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
  //  Generated on 28-Aug-2019 13:51:13
  l7.re = J_min;
  l7.im = 0.0;
  coder::internal::scalar::b_sqrt(&l7);
  b_A_min.re = V_wayp + -V_max;
  b_A_min.im = 0.0;
  coder::internal::scalar::b_sqrt(&b_A_min);
  if (l7.im == 0.0) {
    l3 = 1.0 / l7.re;
    l5 = 0.0;
  } else if (l7.re == 0.0) {
    l3 = 0.0;
    l5 = -(1.0 / l7.im);
  } else {
    l13 = std::abs(l7.re);
    l4 = std::abs(l7.im);
    if (l13 > l4) {
      l4 = l7.im / l7.re;
      l5 = l7.re + l4 * l7.im;
      l3 = (l4 * 0.0 + 1.0) / l5;
      l5 = (0.0 - l4) / l5;
    } else if (l4 == l13) {
      if (l7.re > 0.0) {
        l4 = 0.5;
      } else {
        l4 = -0.5;
      }
      if (l7.im > 0.0) {
        l5 = 0.5;
      } else {
        l5 = -0.5;
      }
      l3 = (l4 + 0.0 * l5) / l13;
      l5 = (0.0 * l4 - l5) / l13;
    } else {
      l4 = l7.re / l7.im;
      l5 = l7.im + l4 * l7.re;
      l3 = l4 / l5;
      l5 = (l4 * 0.0 - 1.0) / l5;
    }
  }
  l3 *= 1.4142135623730951;
  l5 *= 1.4142135623730951;
  l7.re = l3 * b_A_min.re - l5 * b_A_min.im;
  l7.im = l3 * b_A_min.im + l5 * b_A_min.re;
  t5[0] = l7;
  t5[1].re = -l7.re;
  t5[1].im = -l7.im;
  l13 = A_init * A_init;
  l3 = A_min * A_min;
  l5 = J_min * J_min;
  l6 = J_max * J_max;
  l4 = l3 * l3;
  coder::power(t5, x);
  l7.re = (((((-l4 * l5 + l4 * l6) - l13 * l13 * l6 * 3.0) +
             rt_powd_snf(A_init, 3.0) * A_min * l6 * 8.0) -
            l13 * l3 * l6 * 6.0) -
           V_init * V_init * l5 * l6 * 12.0) +
          V_max * V_max * l5 * l6 * 12.0;
  c_A_min = A_min * rt_powd_snf(J_min, 3.0) * l6;
  b_A_min.re = A_min * P_init * l5 * l6 * 24.0;
  A_min_re = A_min * P_wayp * l5 * l6 * 24.0;
  l4 = J_min * V_init;
  J_min_re = l4 * l13 * l6 * 12.0;
  b_J_min_re = l4 * l3 * l6 * 12.0;
  J_max_re = J_max * V_max * l3 * l5 * 12.0;
  A_min_tmp = A_min * V_max * l5 * l6;
  A_init_re = A_init * A_min * J_min * V_init * l6 * 24.0;
  l13 = 1.0 / A_min * (1.0 / J_min) *
        ((((l13 + J_min * V_max * 2.0) + -(l4 * 2.0)) + -l3) +
         J_min * l3 * (1.0 / J_max)) /
        2.0;
  l3 = A_min * (1.0 / J_max);
  l6 = -(1.0 / J_min * (A_init + -A_min));
  t[0].re = l6;
  t[0].im = 0.0;
  t[1].re = l6;
  t[1].im = 0.0;
  t[2].re = l13;
  t[2].im = 0.0;
  t[3].re = l13;
  t[3].im = 0.0;
  t[4].re = -l3;
  t[4].im = 0.0;
  t[5].re = -l3;
  t[5].im = 0.0;
  l3 = -0.041666666666666664 *
       ((((((((l7.re + c_A_min * x[0].re * 4.0) + b_A_min.re) - A_min_re) +
            J_min_re) +
           b_J_min_re) -
          J_max_re) +
         A_min_tmp * t5[0].re * 24.0) -
        A_init_re);
  l5 = -0.041666666666666664 *
       (c_A_min * x[0].im * 4.0 + A_min_tmp * t5[0].im * 24.0);
  if (l5 == 0.0) {
    t[6].re = l3 / A_min_tmp;
    t[6].im = 0.0;
  } else if (l3 == 0.0) {
    t[6].re = 0.0;
    t[6].im = l5 / A_min_tmp;
  } else {
    t[6].re = l3 / A_min_tmp;
    t[6].im = l5 / A_min_tmp;
  }
  t[8] = t5[0];
  t[10].re = 0.0;
  t[10].im = 0.0;
  t[12].re = 0.0;
  t[12].im = 0.0;
  l3 = -0.041666666666666664 *
       ((((((((l7.re + c_A_min * x[1].re * 4.0) + b_A_min.re) - A_min_re) +
            J_min_re) +
           b_J_min_re) -
          J_max_re) +
         A_min_tmp * t5[1].re * 24.0) -
        A_init_re);
  l5 = -0.041666666666666664 *
       (c_A_min * x[1].im * 4.0 + A_min_tmp * t5[1].im * 24.0);
  if (l5 == 0.0) {
    t[7].re = l3 / A_min_tmp;
    t[7].im = 0.0;
  } else if (l3 == 0.0) {
    t[7].re = 0.0;
    t[7].im = l5 / A_min_tmp;
  } else {
    t[7].re = l3 / A_min_tmp;
    t[7].im = l5 / A_min_tmp;
  }
  t[9] = t5[1];
  t[11].re = 0.0;
  t[11].im = 0.0;
  t[13].re = 0.0;
  t[13].im = 0.0;
}

// End of code generation (abcde_NO_VP.cpp)
