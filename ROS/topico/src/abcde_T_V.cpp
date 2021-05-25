//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// abcde_T_V.cpp
//
// Code generation for function 'abcde_T_V'
//

// Include files
#include "abcde_T_V.h"
#include "rt_nonfinite.h"
#include "sqrt.h"
#include "topico_wrapper_data.h"
#include <cmath>

// Function Definitions
void abcde_T_V(double V_init, double A_init, double V_wayp, double V_max,
               double A_max, double J_max, double J_min, double T,
               creal_T t[14])
{
  creal_T t4_tmp[2];
  creal_T t5[2];
  creal_T x[2];
  creal_T l2;
  creal_T l7;
  double J_min_re;
  double b_J_min;
  double b_y_tmp;
  double brm;
  double d;
  double d1;
  double d2;
  double d3;
  double im;
  double l2_tmp;
  double l3;
  double l6;
  double re;
  double y_tmp;
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
  //  Generated on 03-Sep-2019 12:09:59
  l7.re = J_min;
  l7.im = 0.0;
  coder::internal::scalar::b_sqrt(&l7);
  l2.re = V_wayp + -V_max;
  l2.im = 0.0;
  coder::internal::scalar::b_sqrt(&l2);
  if (l7.im == 0.0) {
    re = 1.0 / l7.re;
    im = 0.0;
  } else if (l7.re == 0.0) {
    re = 0.0;
    im = -(1.0 / l7.im);
  } else {
    brm = std::abs(l7.re);
    l3 = std::abs(l7.im);
    if (brm > l3) {
      l3 = l7.im / l7.re;
      l6 = l7.re + l3 * l7.im;
      re = (l3 * 0.0 + 1.0) / l6;
      im = (0.0 - l3) / l6;
    } else if (l3 == brm) {
      if (l7.re > 0.0) {
        l3 = 0.5;
      } else {
        l3 = -0.5;
      }
      if (l7.im > 0.0) {
        l6 = 0.5;
      } else {
        l6 = -0.5;
      }
      re = (l3 + 0.0 * l6) / brm;
      im = (0.0 * l3 - l6) / brm;
    } else {
      l3 = l7.re / l7.im;
      l6 = l7.im + l3 * l7.re;
      re = l3 / l6;
      im = (l3 * 0.0 - 1.0) / l6;
    }
  }
  re *= 1.4142135623730951;
  im *= 1.4142135623730951;
  l7.re = re * l2.re - im * l2.im;
  l7.im = re * l2.im + im * l2.re;
  t5[0] = l7;
  t5[1].re = -l7.re;
  t5[1].im = -l7.im;
  l2_tmp = A_max * A_max;
  y_tmp = A_init * A_init;
  re = l7.re * l7.re - l7.im * l7.im;
  brm = l7.re * l7.im;
  im = brm + brm;
  t4_tmp[0].re = re;
  t4_tmp[0].im = im;
  l3 = A_max * J_max;
  l6 = (l2_tmp + J_max * V_wayp * 2.0) + y_tmp;
  J_min_re = J_min * J_min * J_max;
  d = J_max * l2_tmp;
  b_y_tmp = A_init * A_max * J_min * 2.0;
  b_J_min = J_min * J_max;
  d1 = b_J_min * V_init * 2.0;
  d2 = A_max * J_min * J_max;
  d3 = d2 * T * 2.0;
  x[0].re =
      ((((d - J_min * (l6 + l3 * l7.re * 2.0)) + J_min_re * re) + b_y_tmp) +
       d1) +
      d3;
  x[0].im = (0.0 - J_min * (l3 * l7.im * 2.0)) + J_min_re * im;
  re = -l7.re * -l7.re - -l7.im * -l7.im;
  brm = -l7.re * -l7.im;
  im = brm + brm;
  x[1].re =
      ((((d - J_min * (l6 + l3 * -l7.re * 2.0)) + J_min_re * re) + b_y_tmp) +
       d1) +
      d3;
  x[1].im = (0.0 - J_min * (l3 * -l7.im * 2.0)) + J_min_re * im;
  b_y_tmp = d2 * 2.0;
  l7.re = J_min * y_tmp * 2.0 + d;
  l2.re = (y_tmp + l2_tmp) + J_max * V_init * 2.0;
  J_min_re = b_J_min * V_wayp * 2.0;
  l3 = A_max * (1.0 / J_min);
  l6 = -(1.0 / J_max * (A_init + -A_max));
  t[0].re = l6;
  t[0].im = 0.0;
  t[1].re = l6;
  t[1].im = 0.0;
  t[4].re = -l3;
  t[4].im = 0.0;
  t[5].re = -l3;
  t[5].im = 0.0;
  brm = (l7.re - J_min * (l2.re + b_J_min * t4_tmp[0].re)) + J_min_re;
  l3 = 0.0 - J_min * (b_J_min * t4_tmp[0].im);
  if (l3 == 0.0) {
    t[2].re = brm / b_y_tmp;
    t[2].im = 0.0;
  } else if (brm == 0.0) {
    t[2].re = 0.0;
    t[2].im = l3 / b_y_tmp;
  } else {
    t[2].re = brm / b_y_tmp;
    t[2].im = l3 / b_y_tmp;
  }
  if (x[0].im == 0.0) {
    t[6].re = x[0].re / b_y_tmp;
    t[6].im = 0.0;
  } else if (x[0].re == 0.0) {
    t[6].re = 0.0;
    t[6].im = x[0].im / b_y_tmp;
  } else {
    t[6].re = x[0].re / b_y_tmp;
    t[6].im = x[0].im / b_y_tmp;
  }
  t[8] = t5[0];
  t[10].re = 0.0;
  t[10].im = 0.0;
  t[12].re = 0.0;
  t[12].im = 0.0;
  brm = (l7.re - J_min * (l2.re + b_J_min * re)) + J_min_re;
  l3 = 0.0 - J_min * (b_J_min * im);
  if (l3 == 0.0) {
    t[3].re = brm / b_y_tmp;
    t[3].im = 0.0;
  } else if (brm == 0.0) {
    t[3].re = 0.0;
    t[3].im = l3 / b_y_tmp;
  } else {
    t[3].re = brm / b_y_tmp;
    t[3].im = l3 / b_y_tmp;
  }
  if (x[1].im == 0.0) {
    t[7].re = x[1].re / b_y_tmp;
    t[7].im = 0.0;
  } else if (x[1].re == 0.0) {
    t[7].re = 0.0;
    t[7].im = x[1].im / b_y_tmp;
  } else {
    t[7].re = x[1].re / b_y_tmp;
    t[7].im = x[1].im / b_y_tmp;
  }
  t[9] = t5[1];
  t[11].re = 0.0;
  t[11].im = 0.0;
  t[13].re = 0.0;
  t[13].im = 0.0;
}

// End of code generation (abcde_T_V.cpp)
