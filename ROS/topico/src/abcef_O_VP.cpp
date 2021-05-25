//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// abcef_O_VP.cpp
//
// Code generation for function 'abcef_O_VP'
//

// Include files
#include "abcef_O_VP.h"
#include "rt_nonfinite.h"
#include "sqrt.h"
#include "topico_wrapper_data.h"
#include "topico_wrapper_rtwutil.h"

// Function Definitions
void abcef_O_VP(double P_init, double V_init, double A_init, double P_wayp,
                double V_wayp, double A_max, double A_min, double J_max,
                double J_min, creal_T t[14])
{
  creal_T t6[2];
  creal_T l54;
  double b_l54_tmp;
  double c_l54_tmp;
  double d_l54_tmp;
  double l11;
  double l12;
  double l20_tmp;
  double l21_tmp;
  double l24_tmp;
  double l2_tmp;
  double l49;
  double l49_tmp;
  double l54_tmp;
  double l5_tmp;
  double l6;
  double l8_tmp;
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
  //  Generated on 28-Aug-2019 18:40:52
  l2_tmp = A_init * A_init;
  l5_tmp = A_min * A_min;
  l6 = rt_powd_snf(A_min, 3.0);
  l8_tmp = A_max * A_max;
  l11 = J_min * J_min;
  l12 = J_max * J_max;
  l20_tmp = A_min * J_min * J_max;
  l21_tmp = A_max * J_min * J_max;
  l24_tmp = A_min * J_max;
  l49_tmp = J_min * J_max;
  l49 = 1.0 / (l49_tmp * l5_tmp + l20_tmp * -A_max);
  l54_tmp = A_min * (l8_tmp * l8_tmp);
  b_l54_tmp = A_min * A_max;
  c_l54_tmp = l24_tmp * V_init;
  d_l54_tmp = A_min + -A_max;
  l54.re = -(d_l54_tmp *
             ((((((((((((((l54_tmp * l11 + A_max * (l5_tmp * l5_tmp) * l12) +
                          -(A_min * (l2_tmp * l2_tmp) * l11 * 3.0)) +
                         -(l54_tmp * l12)) +
                        b_l54_tmp * rt_powd_snf(A_init, 3.0) * l11 * 8.0) +
                       l5_tmp * rt_powd_snf(A_max, 3.0) * l12 * 3.0) +
                      -(l6 * l8_tmp * l12 * 3.0)) +
                     -(A_init * A_min * A_max * J_max * V_init * l11 * 24.0)) +
                    b_l54_tmp * P_init * l11 * l12 * 24.0) +
                   c_l54_tmp * l2_tmp * l11 * 12.0) +
                  c_l54_tmp * l8_tmp * l11 * 12.0) +
                 -(b_l54_tmp * P_wayp * l11 * l12 * 24.0)) +
                -(A_min * l2_tmp * l8_tmp * l11 * 6.0)) +
               A_max * l11 * l12 * (V_wayp * V_wayp) * 12.0) +
              -(A_min * l11 * l12 * (V_init * V_init) * 12.0)));
  l54.im = 0.0;
  coder::internal::scalar::b_sqrt(&l54);
  l54.re *= 1.7320508075688772;
  l54.im *= 1.7320508075688772;
  l12 =
      (((J_max * l6 * 3.0 - l20_tmp * V_wayp * 6.0) + l21_tmp * V_wayp * 6.0) +
       l24_tmp * l8_tmp * 3.0) -
      A_max * J_max * l5_tmp * 6.0;
  t6[0].re = -0.16666666666666666 * (l49 * (l12 - l54.re));
  t6[0].im = -0.16666666666666666 * (l49 * (0.0 - l54.im));
  t6[1].re = -0.16666666666666666 * (l49 * (l12 + l54.re));
  t6[1].im = -0.16666666666666666 * (l49 * l54.im);
  l11 = 1.0 / J_min * d_l54_tmp;
  l6 = -(1.0 / J_max * (A_init + -A_max));
  l54.re =
      ((((J_min * l8_tmp - J_max * l8_tmp) - l2_tmp * J_min) + l5_tmp * J_max) +
       l49_tmp * V_init * 2.0) -
      l49_tmp * V_wayp * 2.0;
  t[0].re = l6;
  t[0].im = 0.0;
  t[1].re = l6;
  t[1].im = 0.0;
  t[4].re = l11;
  t[4].im = 0.0;
  t[5].re = l11;
  t[5].im = 0.0;
  l11 = (l54.re + l20_tmp * t6[0].re * 2.0) * -0.5;
  l12 = l20_tmp * t6[0].im * 2.0 * -0.5;
  if (l12 == 0.0) {
    t[2].re = l11 / l21_tmp;
    t[2].im = 0.0;
  } else if (l11 == 0.0) {
    t[2].re = 0.0;
    t[2].im = l12 / l21_tmp;
  } else {
    t[2].re = l11 / l21_tmp;
    t[2].im = l12 / l21_tmp;
  }
  t[6].re = 0.0;
  t[6].im = 0.0;
  t[8].re = 0.0;
  t[8].im = 0.0;
  t[10] = t6[0];
  t[12].re = 0.0;
  t[12].im = 0.0;
  l11 = (l54.re + l20_tmp * t6[1].re * 2.0) * -0.5;
  l12 = l20_tmp * t6[1].im * 2.0 * -0.5;
  if (l12 == 0.0) {
    t[3].re = l11 / l21_tmp;
    t[3].im = 0.0;
  } else if (l11 == 0.0) {
    t[3].re = 0.0;
    t[3].im = l12 / l21_tmp;
  } else {
    t[3].re = l11 / l21_tmp;
    t[3].im = l12 / l21_tmp;
  }
  t[7].re = 0.0;
  t[7].im = 0.0;
  t[9].re = 0.0;
  t[9].im = 0.0;
  t[11] = t6[1];
  t[13].re = 0.0;
  t[13].im = 0.0;
}

// End of code generation (abcef_O_VP.cpp)
