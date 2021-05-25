//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// abcdeg_T_V.cpp
//
// Code generation for function 'abcdeg_T_V'
//

// Include files
#include "abcdeg_T_V.h"
#include "rt_nonfinite.h"
#include "sqrt.h"
#include "topico_wrapper_data.h"
#include "topico_wrapper_rtwutil.h"
#include "topico_wrapper_types.h"
#include <cmath>

// Function Definitions
void abcdeg_T_V(double V_init, double A_init, double V_wayp, double V_max,
                double V_min, double A_max, double J_max, double J_min,
                double T, creal_T t[14])
{
  creal_T t7[2];
  creal_T dc;
  creal_T dc1;
  creal_T l15;
  creal_T l20;
  double ai;
  double ar_tmp;
  double l13;
  double l18_im;
  double l18_re;
  double l3;
  double l5_tmp;
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
  //  Generated on 02-Sep-2019 16:48:04
  if (J_max < 0.0) {
    f_rtErrorWithMessageID("sqrt", n_emlrtRTEI.fName, n_emlrtRTEI.lineNo);
  }
  l5_tmp = std::sqrt(J_max);
  l15.re = V_wayp + -V_min;
  l15.im = 0.0;
  coder::internal::scalar::b_sqrt(&l15);
  l18_re = J_max * 1.4142135623730951 * l15.re;
  l18_im = J_max * 1.4142135623730951 * l15.im;
  l20.re = -J_min;
  l20.im = 0.0;
  coder::internal::scalar::b_sqrt(&l20);
  dc.re = J_max + -J_min;
  dc.im = 0.0;
  coder::internal::scalar::b_sqrt(&dc);
  dc1.re = V_max + -V_min;
  dc1.im = 0.0;
  coder::internal::scalar::b_sqrt(&dc1);
  l3 = 1.4142135623730951 * l20.re;
  l6 = 1.4142135623730951 * l20.im;
  l13 = l3 * dc.re - l6 * dc.im;
  l6 = l3 * dc.im + l6 * dc.re;
  l20.re = l13 * dc1.re - l6 * dc1.im;
  l20.im = l13 * dc1.im + l6 * dc1.re;
  l6 = (J_min * 1.4142135623730951 * l15.re - l18_re) + l20.re;
  ai = (J_min * 1.4142135623730951 * l15.im - l18_im) + l20.im;
  l3 = rt_powd_snf(l5_tmp, 3.0) - J_min * l5_tmp;
  if (ai == 0.0) {
    t7[0].re = l6 / l3;
    t7[0].im = 0.0;
  } else if (l6 == 0.0) {
    t7[0].re = 0.0;
    t7[0].im = ai / l3;
  } else {
    t7[0].re = l6 / l3;
    t7[0].im = ai / l3;
  }
  ar_tmp = -J_min * 1.4142135623730951;
  l6 = (l18_re + l20.re) + ar_tmp * l15.re;
  ai = (l18_im + l20.im) + ar_tmp * l15.im;
  if (ai == 0.0) {
    t7[1].re = l6 / l3;
    t7[1].im = 0.0;
  } else if (l6 == 0.0) {
    t7[1].re = 0.0;
    t7[1].im = ai / l3;
  } else {
    t7[1].re = l6 / l3;
    t7[1].im = ai / l3;
  }
  l20.re = -(J_min * (J_min + -J_max) * (V_min + -V_max));
  l20.im = 0.0;
  coder::internal::scalar::b_sqrt(&l20);
  l18_re = J_min * J_max;
  l3 = 1.4142135623730951 * l5_tmp * (1.0 / (J_min * J_min + -l18_re));
  l20.re *= l3;
  l20.im *= l3;
  l3 = A_max * A_max;
  l13 = 1.0 / A_max * (1.0 / J_max) *
        ((((A_init * A_init + J_max * V_max * 2.0) + -(J_max * V_init * 2.0)) +
          -l3) +
         J_max * l3 * (1.0 / J_min)) /
        2.0;
  l3 = A_max * (1.0 / J_min);
  l6 = -(1.0 / J_max * (A_init + -A_max));
  l15.re = ((A_init * J_min - A_max * J_min) + A_max * J_max) + l18_re * T;
  t[0].re = l6;
  t[0].im = 0.0;
  t[1].re = l6;
  t[1].im = 0.0;
  t[4].re = -l3;
  t[4].im = 0.0;
  t[5].re = -l3;
  t[5].im = 0.0;
  t[2].re = l13;
  t[2].im = 0.0;
  ar_tmp = l13 + l20.re;
  l6 = l15.re - l18_re * (ar_tmp + t7[0].re);
  ai = 0.0 - l18_re * (l20.im + t7[0].im);
  if (ai == 0.0) {
    t[6].re = l6 / l18_re;
    t[6].im = 0.0;
  } else if (l6 == 0.0) {
    t[6].re = 0.0;
    t[6].im = ai / l18_re;
  } else {
    t[6].re = l6 / l18_re;
    t[6].im = ai / l18_re;
  }
  t[8] = l20;
  t[10].re = 0.0;
  t[10].im = 0.0;
  t[12] = t7[0];
  t[3].re = l13;
  t[3].im = 0.0;
  l6 = l15.re - l18_re * (ar_tmp + t7[1].re);
  ai = 0.0 - l18_re * (l20.im + t7[1].im);
  if (ai == 0.0) {
    t[7].re = l6 / l18_re;
    t[7].im = 0.0;
  } else if (l6 == 0.0) {
    t[7].re = 0.0;
    t[7].im = ai / l18_re;
  } else {
    t[7].re = l6 / l18_re;
    t[7].im = ai / l18_re;
  }
  t[9] = l20;
  t[11].re = 0.0;
  t[11].im = 0.0;
  t[13] = t7[1];
}

// End of code generation (abcdeg_T_V.cpp)
