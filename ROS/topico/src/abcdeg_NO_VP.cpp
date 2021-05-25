//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// abcdeg_NO_VP.cpp
//
// Code generation for function 'abcdeg_NO_VP'
//

// Include files
#include "abcdeg_NO_VP.h"
#include "power.h"
#include "rt_nonfinite.h"
#include "sqrt.h"
#include "topico_wrapper_data.h"
#include "topico_wrapper_rtwutil.h"
#include "topico_wrapper_types.h"
#include <cmath>

// Function Definitions
void abcdeg_NO_VP(double P_init, double V_init, double A_init, double P_wayp,
                  double V_wayp, double V_max, double V_min, double A_min,
                  double J_max, double J_min, creal_T t[14])
{
  creal_T dcv[2];
  creal_T t5[2];
  creal_T t7[2];
  creal_T x[2];
  creal_T b_J_min;
  creal_T c_J_min;
  creal_T l15;
  creal_T l20;
  double A_init_re;
  double A_min_tmp;
  double J_max_re;
  double b_A_min;
  double b_A_min_tmp;
  double l18_im;
  double l18_re;
  double l3;
  double l4;
  double l5_tmp;
  double l7;
  double re;
  double t7_re;
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
  b_J_min.re = J_max + -J_min;
  b_J_min.im = 0.0;
  coder::internal::scalar::b_sqrt(&b_J_min);
  c_J_min.re = V_max + -V_min;
  c_J_min.im = 0.0;
  coder::internal::scalar::b_sqrt(&c_J_min);
  re = 1.4142135623730951 * l20.re;
  l3 = 1.4142135623730951 * l20.im;
  l4 = re * b_J_min.re - l3 * b_J_min.im;
  l3 = re * b_J_min.im + l3 * b_J_min.re;
  l20.re = l4 * c_J_min.re - l3 * c_J_min.im;
  l20.im = l4 * c_J_min.im + l3 * c_J_min.re;
  l7 = (J_min * 1.4142135623730951 * l15.re - l18_re) + l20.re;
  l4 = (J_min * 1.4142135623730951 * l15.im - l18_im) + l20.im;
  l3 = rt_powd_snf(l5_tmp, 3.0) - J_min * l5_tmp;
  if (l4 == 0.0) {
    t7[0].re = l7 / l3;
    t7[0].im = 0.0;
  } else if (l7 == 0.0) {
    t7[0].re = 0.0;
    t7[0].im = l4 / l3;
  } else {
    t7[0].re = l7 / l3;
    t7[0].im = l4 / l3;
  }
  l4 = -J_min * 1.4142135623730951;
  l7 = (l18_re + l20.re) + l4 * l15.re;
  l4 = (l18_im + l20.im) + l4 * l15.im;
  if (l4 == 0.0) {
    t7[1].re = l7 / l3;
    t7[1].im = 0.0;
  } else if (l7 == 0.0) {
    t7[1].re = 0.0;
    t7[1].im = l4 / l3;
  } else {
    t7[1].re = l7 / l3;
    t7[1].im = l4 / l3;
  }
  l20.re = -(J_min * (J_min + -J_max) * (V_min + -V_max));
  l20.im = 0.0;
  coder::internal::scalar::b_sqrt(&l20);
  l18_im = J_min * J_min;
  l4 = 1.4142135623730951 * l5_tmp * (1.0 / (l18_im + -(J_min * J_max)));
  l20.re *= l4;
  l20.im *= l4;
  t5[0] = l20;
  t5[1] = l20;
  l5_tmp = A_init * A_init;
  l3 = A_min * A_min;
  l7 = J_max * J_max;
  l4 = l3 * l3;
  coder::power(t5, x);
  coder::power(t7, dcv);
  l15.re = (((((-l4 * l18_im + l4 * l7) - l5_tmp * l5_tmp * l7 * 3.0) +
              rt_powd_snf(A_init, 3.0) * A_min * l7 * 8.0) -
             l5_tmp * l3 * l7 * 6.0) -
            V_init * V_init * l18_im * l7 * 12.0) +
           V_max * V_max * l18_im * l7 * 12.0;
  A_min_tmp = A_min * rt_powd_snf(J_min, 3.0) * l7;
  b_A_min = A_min * rt_powd_snf(J_max, 3.0) * l18_im;
  l18_re = A_min * P_init * l18_im * l7 * 24.0;
  l20.re = A_min * P_wayp * l18_im * l7 * 24.0;
  l4 = J_min * V_init;
  b_J_min.re = l4 * l5_tmp * l7 * 12.0;
  c_J_min.re = l4 * l3 * l7 * 12.0;
  J_max_re = J_max * V_max * l3 * l18_im * 12.0;
  b_A_min_tmp = A_min * V_max * l18_im * l7;
  A_init_re = A_init * A_min * J_min * V_init * l7 * 24.0;
  l4 = 1.0 / A_min * (1.0 / J_min) *
       ((((l5_tmp + J_min * V_max * 2.0) + -(l4 * 2.0)) + -l3) +
        J_min * l3 * (1.0 / J_max)) /
       2.0;
  l3 = A_min * (1.0 / J_max);
  l7 = -(1.0 / J_min * (A_init + -A_min));
  t[0].re = l7;
  t[0].im = 0.0;
  t[1].re = l7;
  t[1].im = 0.0;
  t[2].re = l4;
  t[2].im = 0.0;
  t[3].re = l4;
  t[3].im = 0.0;
  t[4].re = -l3;
  t[4].im = 0.0;
  t[5].re = -l3;
  t[5].im = 0.0;
  t7_re = t7[0].re * t7[0].re - t7[0].im * t7[0].im;
  l4 = t7[0].re * t7[0].im;
  l18_im = l4 + l4;
  l5_tmp = A_min_tmp * t5[0].re;
  l7 = A_min_tmp * t5[0].im;
  l4 = t5[0].re * t5[0].im;
  l3 = A_min_tmp * (t5[0].re * t5[0].re - t5[0].im * t5[0].im);
  l4 = A_min_tmp * (l4 + l4);
  re = -0.041666666666666664 * ((((((((((((l15.re + A_min_tmp * x[0].re * 4.0) +
                                          b_A_min * dcv[0].re * 4.0) +
                                         l18_re) -
                                        l20.re) +
                                       b_J_min.re) +
                                      c_J_min.re) -
                                     J_max_re) +
                                    b_A_min_tmp * t5[0].re * 24.0) +
                                   b_A_min_tmp * t7[0].re * 24.0) +
                                  (l5_tmp * t7_re - l7 * l18_im) * 12.0) +
                                 (l3 * t7[0].re - l4 * t7[0].im) * 12.0) -
                                A_init_re);
  l3 = -0.041666666666666664 *
       (((((A_min_tmp * x[0].im * 4.0 + b_A_min * dcv[0].im * 4.0) +
           b_A_min_tmp * t5[0].im * 24.0) +
          b_A_min_tmp * t7[0].im * 24.0) +
         (l5_tmp * l18_im + l7 * t7_re) * 12.0) +
        (l3 * t7[0].im + l4 * t7[0].re) * 12.0);
  if (l3 == 0.0) {
    t[6].re = re / b_A_min_tmp;
    t[6].im = 0.0;
  } else if (re == 0.0) {
    t[6].re = 0.0;
    t[6].im = l3 / b_A_min_tmp;
  } else {
    t[6].re = re / b_A_min_tmp;
    t[6].im = l3 / b_A_min_tmp;
  }
  t[8] = t5[0];
  t[10].re = 0.0;
  t[10].im = 0.0;
  t[12] = t7[0];
  t7_re = t7[1].re * t7[1].re - t7[1].im * t7[1].im;
  l4 = t7[1].re * t7[1].im;
  l18_im = l4 + l4;
  l5_tmp = A_min_tmp * t5[1].re;
  l7 = A_min_tmp * t5[1].im;
  l4 = t5[1].re * t5[1].im;
  l3 = A_min_tmp * (t5[1].re * t5[1].re - t5[1].im * t5[1].im);
  l4 = A_min_tmp * (l4 + l4);
  re = -0.041666666666666664 * ((((((((((((l15.re + A_min_tmp * x[1].re * 4.0) +
                                          b_A_min * dcv[1].re * 4.0) +
                                         l18_re) -
                                        l20.re) +
                                       b_J_min.re) +
                                      c_J_min.re) -
                                     J_max_re) +
                                    b_A_min_tmp * t5[1].re * 24.0) +
                                   b_A_min_tmp * t7[1].re * 24.0) +
                                  (l5_tmp * t7_re - l7 * l18_im) * 12.0) +
                                 (l3 * t7[1].re - l4 * t7[1].im) * 12.0) -
                                A_init_re);
  l3 = -0.041666666666666664 *
       (((((A_min_tmp * x[1].im * 4.0 + b_A_min * dcv[1].im * 4.0) +
           b_A_min_tmp * t5[1].im * 24.0) +
          b_A_min_tmp * t7[1].im * 24.0) +
         (l5_tmp * l18_im + l7 * t7_re) * 12.0) +
        (l3 * t7[1].im + l4 * t7[1].re) * 12.0);
  if (l3 == 0.0) {
    t[7].re = re / b_A_min_tmp;
    t[7].im = 0.0;
  } else if (re == 0.0) {
    t[7].re = 0.0;
    t[7].im = l3 / b_A_min_tmp;
  } else {
    t[7].re = re / b_A_min_tmp;
    t[7].im = l3 / b_A_min_tmp;
  }
  t[9] = t5[1];
  t[11].re = 0.0;
  t[11].im = 0.0;
  t[13] = t7[1];
}

// End of code generation (abcdeg_NO_VP.cpp)
