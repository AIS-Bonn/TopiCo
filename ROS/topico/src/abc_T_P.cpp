//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// abc_T_P.cpp
//
// Code generation for function 'abc_T_P'
//

// Include files
#include "abc_T_P.h"
#include "rt_nonfinite.h"
#include "topico_wrapper_data.h"
#include "topico_wrapper_rtwutil.h"
#include "topico_wrapper_types.h"

// Function Definitions
void abc_T_P(double P_init, double V_init, double A_init, double P_wayp,
             double A_max, double J_max, double J_min, double T, creal_T t[21])
{
  creal_T t2[3];
  double a_tmp;
  double a_tmp_tmp;
  double b_a_tmp;
  double l10;
  double l13;
  double l2;
  double l3;
  double l32;
  double l32_tmp;
  double l33;
  double l35;
  double l4;
  double l5;
  double l59;
  double l6;
  double l61;
  double l61_tmp;
  double l7;
  double l72_im;
  double l72_re;
  double l8;
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
  //  Generated on 29-Aug-2019 14:54:42
  l2 = A_init * A_init;
  l3 = rt_powd_snf(A_init, 3.0);
  l4 = A_max * A_max;
  l5 = rt_powd_snf(A_max, 3.0);
  l6 = J_max * J_max;
  l7 = rt_powd_snf(J_max, 3.0);
  l8 = T * T;
  l10 = 1.0 / J_min;
  l13 = 1.0 / l7;
  l32_tmp = A_init * J_min;
  l32 = l32_tmp * l6 * 3.0;
  l59 = J_min * J_max;
  l33 = l59 * l2 * 3.0;
  l35 = l59 * l4 * 3.0;
  l72_im = A_max * J_min;
  l72_re = l72_im * l6;
  l59 = (l32 + J_min * T * l7 * 3.0) + -(l72_re * 3.0);
  l61_tmp = l10 * l13;
  l61 = l61_tmp * l59 / 3.0;
  a_tmp = J_min * l7;
  a_tmp_tmp = A_init * A_max;
  b_a_tmp = a_tmp_tmp * J_min * J_max;
  l59 = (rt_powd_snf(l10, 3.0) * rt_powd_snf(l13, 3.0) * rt_powd_snf(l59, 3.0) *
             0.07407407407407407 +
         -(l10 * l10 * (1.0 / rt_powd_snf(l6, 3.0)) * l59 *
           (((((-(b_a_tmp * 6.0) + l33) + l35) + l32_tmp * T * l6 * 6.0) +
             a_tmp * l8 * 3.0) +
            -(l72_im * T * l6 * 6.0)) /
           3.0)) +
        l61_tmp *
            ((((((((((((((((((((J_min * l3 + J_max * l5) + P_init * l7 * 6.0) +
                              -(J_max * l3)) +
                             -(J_min * l5)) +
                            -(P_wayp * l7 * 6.0)) +
                           l32_tmp * l4 * 3.0) +
                          A_max * J_max * l2 * 3.0) +
                         T * V_init * l7 * 6.0) +
                        a_tmp * rt_powd_snf(T, 3.0)) +
                       -(l72_im * l2 * 3.0)) +
                      -(A_init * J_max * l4 * 3.0)) +
                     b_a_tmp * T * -6.0) +
                    a_tmp_tmp * T * l6 * 6.0) +
                   T * l33) +
                  T * l35) +
                 A_max * l7 * l8 * 3.0) +
                l8 * l32) +
               -(T * l2 * l6 * 3.0)) +
              -(T * l4 * l6 * 3.0)) +
             l72_re * l8 * -3.0);
  l35 = rt_powd_snf(l59, 0.33333333333333331);
  if (l59 < 0.0) {
    k_rtErrorWithMessageID(o_emlrtRTEI.fName, o_emlrtRTEI.lineNo);
  }
  l72_re = 1.7320508075688772 * l35 * 0.0;
  l72_im = 1.7320508075688772 * l35 * 0.5;
  t2[0].re = l61 + l35;
  t2[0].im = 0.0;
  l59 = l61 + -(l35 / 2.0);
  t2[1].re = l59 - l72_re;
  t2[1].im = 0.0 - l72_im;
  t2[2].re = l59 + l72_re;
  t2[2].im = l72_im;
  l6 = -(1.0 / J_max * (A_init + -A_max));
  l72_re = (A_init - A_max) / J_max;
  t[0].re = l6;
  t[0].im = 0.0;
  t[1].re = l6;
  t[1].im = 0.0;
  t[2].re = l6;
  t[2].im = 0.0;
  t[3] = t2[0];
  t[6].re = (T - t2[0].re) + l72_re;
  t[6].im = 0.0;
  t[9].re = 0.0;
  t[9].im = 0.0;
  t[12].re = 0.0;
  t[12].im = 0.0;
  t[15].re = 0.0;
  t[15].im = 0.0;
  t[18].re = 0.0;
  t[18].im = 0.0;
  t[4] = t2[1];
  t[7].re = (T - t2[1].re) + l72_re;
  t[7].im = 0.0 - t2[1].im;
  t[10].re = 0.0;
  t[10].im = 0.0;
  t[13].re = 0.0;
  t[13].im = 0.0;
  t[16].re = 0.0;
  t[16].im = 0.0;
  t[19].re = 0.0;
  t[19].im = 0.0;
  t[5] = t2[2];
  t[8].re = (T - t2[2].re) + l72_re;
  t[8].im = 0.0 - t2[2].im;
  t[11].re = 0.0;
  t[11].im = 0.0;
  t[14].re = 0.0;
  t[14].im = 0.0;
  t[17].re = 0.0;
  t[17].im = 0.0;
  t[20].re = 0.0;
  t[20].im = 0.0;
}

// End of code generation (abc_T_P.cpp)
