//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// abcefg_T_V.cpp
//
// Code generation for function 'abcefg_T_V'
//

// Include files
#include "abcefg_T_V.h"
#include "rt_nonfinite.h"
#include "sqrt.h"
#include "topico_wrapper_data.h"
#include "topico_wrapper_rtwutil.h"
#include "topico_wrapper_types.h"
#include <cmath>

// Function Definitions
void abcefg_T_V(double V_init, double A_init, double V_wayp, double V_min,
                double A_max, double A_min, double J_max, double J_min,
                double T, creal_T t[14])
{
  creal_T t6[2];
  creal_T l26;
  double b_l8_re_tmp_tmp;
  double c_l8_re_tmp_tmp;
  double d_l8_re_tmp_tmp;
  double l10_tmp;
  double l11_tmp;
  double l12_tmp;
  double l13_tmp_tmp;
  double l25;
  double l25_tmp;
  double l3_tmp;
  double l4_tmp;
  double l5;
  double l6;
  double l6_tmp;
  double l8_re_tmp_tmp;
  double l8_tmp;
  double l9_tmp;
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
  //  Generated on 02-Sep-2019 15:45:15
  l3_tmp = A_min * A_min;
  l4_tmp = A_max * A_max;
  l6 = A_max * J_min;
  l6_tmp = l6 * J_max;
  l8_tmp = A_init * A_max * J_min * 2.0;
  l9_tmp = A_min * A_max;
  l10_tmp = l9_tmp * J_max * 2.0;
  l5 = J_min * J_max;
  l11_tmp = l5 * V_init * 2.0;
  l12_tmp = l5 * V_min * 2.0;
  if (J_max < 0.0) {
    f_rtErrorWithMessageID("sqrt", n_emlrtRTEI.fName, n_emlrtRTEI.lineNo);
  }
  l13_tmp_tmp = J_min * (A_init * A_init);
  l25_tmp = A_min * J_min * J_max;
  l25 = 1.0 / (l25_tmp + -l6_tmp);
  l26.re = V_wayp + -V_min;
  l26.im = 0.0;
  coder::internal::scalar::b_sqrt(&l26);
  l5 = l6 * 1.4142135623730951 * std::sqrt(J_max);
  l6 = l5 * l26.re;
  l5 *= l26.im;
  l26.re = 2.0 * l6;
  l26.im = 2.0 * l5;
  l8_re_tmp_tmp = J_min * l3_tmp;
  b_l8_re_tmp_tmp = J_min * l4_tmp;
  c_l8_re_tmp_tmp = J_max * l3_tmp;
  d_l8_re_tmp_tmp = J_max * l4_tmp;
  l5 =
      (((((((((l8_tmp + l9_tmp * J_min * 2.0) - l10_tmp) + l11_tmp) - l12_tmp) -
           l13_tmp_tmp) -
          l8_re_tmp_tmp) +
         c_l8_re_tmp_tmp) -
        b_l8_re_tmp_tmp) +
       d_l8_re_tmp_tmp) +
      T * l6_tmp * 2.0;
  t6[0].re = -0.5 * (l25 * (l5 - l26.re));
  t6[0].im = -0.5 * (l25 * (0.0 - l26.im));
  t6[1].re = -0.5 * (l25 * (l5 + l26.re));
  t6[1].im = -0.5 * (l25 * l26.im);
  l9_tmp = l6_tmp * 2.0;
  l5 = 1.0 / J_min * (A_min + -A_max);
  l6 = -(1.0 / J_max * (A_init + -A_max));
  l26.re = (((((l8_re_tmp_tmp - b_l8_re_tmp_tmp) - c_l8_re_tmp_tmp) +
              d_l8_re_tmp_tmp) +
             l13_tmp_tmp) -
            l11_tmp) +
           l12_tmp;
  t[0].re = l6;
  t[0].im = 0.0;
  t[1].re = l6;
  t[1].im = 0.0;
  t[4].re = l5;
  t[4].im = 0.0;
  t[5].re = l5;
  t[5].im = 0.0;
  l5 = 2.0 * (l25_tmp * t6[0].re);
  l3_tmp = 2.0 * (l25_tmp * t6[0].im);
  l4_tmp = l26.re - l5;
  if (0.0 - l3_tmp == 0.0) {
    t[2].re = l4_tmp / l9_tmp;
    t[2].im = 0.0;
  } else if (l4_tmp == 0.0) {
    t[2].re = 0.0;
    t[2].im = (0.0 - l3_tmp) / l9_tmp;
  } else {
    t[2].re = l4_tmp / l9_tmp;
    t[2].im = (0.0 - l3_tmp) / l9_tmp;
  }
  t[6].re = 0.0;
  t[6].im = 0.0;
  t[8].re = 0.0;
  t[8].im = 0.0;
  t[10] = t6[0];
  l6 = ((((((((l8_re_tmp_tmp + b_l8_re_tmp_tmp) - c_l8_re_tmp_tmp) -
             d_l8_re_tmp_tmp) +
            l13_tmp_tmp) -
           l8_tmp) +
          l10_tmp) -
         l11_tmp) +
        l12_tmp) -
       A_max * J_min * J_max * T * 2.0;
  l4_tmp = ((l6 - l5) + l6_tmp * t6[0].re * 2.0) * -0.5;
  l5 = ((0.0 - l3_tmp) + l6_tmp * t6[0].im * 2.0) * -0.5;
  if (l5 == 0.0) {
    t[12].re = l4_tmp / l6_tmp;
    t[12].im = 0.0;
  } else if (l4_tmp == 0.0) {
    t[12].re = 0.0;
    t[12].im = l5 / l6_tmp;
  } else {
    t[12].re = l4_tmp / l6_tmp;
    t[12].im = l5 / l6_tmp;
  }
  l5 = 2.0 * (l25_tmp * t6[1].re);
  l3_tmp = 2.0 * (l25_tmp * t6[1].im);
  l4_tmp = l26.re - l5;
  if (0.0 - l3_tmp == 0.0) {
    t[3].re = l4_tmp / l9_tmp;
    t[3].im = 0.0;
  } else if (l4_tmp == 0.0) {
    t[3].re = 0.0;
    t[3].im = (0.0 - l3_tmp) / l9_tmp;
  } else {
    t[3].re = l4_tmp / l9_tmp;
    t[3].im = (0.0 - l3_tmp) / l9_tmp;
  }
  t[7].re = 0.0;
  t[7].im = 0.0;
  t[9].re = 0.0;
  t[9].im = 0.0;
  t[11] = t6[1];
  l4_tmp = ((l6 - l5) + l6_tmp * t6[1].re * 2.0) * -0.5;
  l5 = ((0.0 - l3_tmp) + l6_tmp * t6[1].im * 2.0) * -0.5;
  if (l5 == 0.0) {
    t[13].re = l4_tmp / l6_tmp;
    t[13].im = 0.0;
  } else if (l4_tmp == 0.0) {
    t[13].re = 0.0;
    t[13].im = l5 / l6_tmp;
  } else {
    t[13].re = l4_tmp / l6_tmp;
    t[13].im = l5 / l6_tmp;
  }
}

// End of code generation (abcefg_T_V.cpp)
