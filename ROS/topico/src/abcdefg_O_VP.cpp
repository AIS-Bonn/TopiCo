//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// abcdefg_O_VP.cpp
//
// Code generation for function 'abcdefg_O_VP'
//

// Include files
#include "abcdefg_O_VP.h"
#include "rt_nonfinite.h"
#include "topico_wrapper_data.h"
#include "topico_wrapper_rtwutil.h"
#include "topico_wrapper_types.h"
#include <cmath>

// Function Definitions
void abcdefg_O_VP(double P_init, double V_init, double A_init, double P_wayp,
                  double V_wayp, double V_max, double V_min, double A_max,
                  double A_min, double J_max, double J_min, double t[14])
{
  double A_min_tmp;
  double J_max_tmp;
  double J_min_tmp;
  double b_A_init;
  double b_A_max;
  double b_A_min;
  double b_J_max;
  double b_J_min;
  double c_A_max;
  double c_A_min;
  double c_J_max;
  double c_J_min;
  double d_A_max;
  double d_A_min;
  double d_J_min;
  double e_A_max;
  double e_A_min;
  double f_A_max;
  double f_A_min;
  double g_A_max;
  double g_A_min;
  double h_A_max;
  double i_A_max;
  double j_A_max;
  double l10;
  double l11;
  double l15;
  double l18;
  double l2;
  double l2_tmp;
  double l3_tmp;
  double l5_tmp;
  double l6;
  double l7;
  double l8;
  double l9;
  double t7_idx_0;
  double t7_idx_1;
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
  //  Generated on 28-Aug-2019 18:40:13
  l2 = 1.0 / J_max;
  if (J_max < 0.0) {
    f_rtErrorWithMessageID("sqrt", n_emlrtRTEI.fName, n_emlrtRTEI.lineNo);
  }
  l7 = V_wayp + -V_min;
  if (l7 < 0.0) {
    f_rtErrorWithMessageID("sqrt", n_emlrtRTEI.fName, n_emlrtRTEI.lineNo);
  }
  l7 = std::sqrt(l7);
  l8 = 1.4142135623730951 * std::sqrt(J_max) * l7;
  t7_idx_0 = -l2 * (A_min + l8);
  t7_idx_1 = -l2 * (A_min - l8);
  l2_tmp = A_init * A_init;
  l3_tmp = A_min * A_min;
  l5_tmp = A_max * A_max;
  l6 = rt_powd_snf(A_max, 3.0);
  l8 = J_min * J_min;
  l9 = J_max * J_max;
  l10 = V_max * V_max;
  l11 = 1.0 / A_min;
  l7 = l5_tmp * l5_tmp;
  l15 = J_max * l3_tmp;
  l2 = J_min * J_max;
  l18 = ((l2 * V_min * 2.0 + J_min * l3_tmp) + -(l2 * V_max * 2.0)) + -l15;
  l2 = A_max * rt_powd_snf(A_min, 3.0);
  l7 = (((((((((l7 * l8 - l7 * l9) - l2_tmp * l2_tmp * l8 * 3.0) +
              l2 * l8 * 3.0) -
             A_min * l6 * l9 * 6.0) +
            l2 * l9) +
           rt_powd_snf(A_init, 3.0) * A_max * l8 * 8.0) -
          l2_tmp * l5_tmp * l8 * 6.0) +
         l8 * l9 * l10 * 12.0) -
        V_init * V_init * l8 * l9 * 12.0) -
       J_max * l6 * l11 * l18 * 6.0;
  b_A_max = A_max * rt_powd_snf(J_max, 3.0) * l8;
  b_A_min = A_min * J_min * J_max * l6 * 6.0;
  A_min_tmp = A_min * A_max;
  c_A_min = A_min_tmp * l2_tmp * l8 * 6.0;
  c_A_max = A_max * P_init * l8 * l9 * 24.0;
  d_A_max = A_max * P_wayp * l8 * l9 * 24.0;
  J_max_tmp = J_max * V_init;
  b_J_max = J_max_tmp * l2_tmp * l8 * 12.0;
  c_J_max = J_max_tmp * l5_tmp * l8 * 12.0;
  J_min_tmp = J_min * V_max;
  b_J_min = J_min_tmp * l5_tmp * l9 * 12.0;
  l2 = A_max * l8;
  e_A_max = l2 * l15;
  f_A_max = A_max * J_min * l2_tmp * l11 * l18 * 6.0;
  g_A_max = A_max * V_min * l8 * l9;
  c_J_min = J_min * V_min * l6 * l9 * l11 * 12.0;
  d_J_min = J_min_tmp * l6 * l9 * l11 * 12.0;
  h_A_max = l2 * l9 * l10 * l11 * 12.0;
  d_A_min = A_min_tmp * l8 * l9;
  i_A_max = A_max * (V_min * V_min) * l8 * l9 * l11 * 12.0;
  l2 = A_min_tmp * J_min;
  e_A_min = l2 * J_max * l2_tmp * 6.0;
  b_A_init = A_init * A_max * J_max * V_init * l8 * 24.0;
  f_A_min = A_min_tmp * J_max * V_min * l8 * 12.0;
  g_A_min = l2 * V_max * l9 * 12.0;
  l2 = A_max * J_max;
  j_A_max = l2 * V_min * l2_tmp * l8 * l11 * 12.0;
  A_min_tmp = l2 * V_max * l2_tmp * l8 * l11 * 12.0;
  l18 = A_max * V_max * l8 * l9;
  l10 = 1.0 / A_min * (1.0 / J_min) * (1.0 / J_max) *
        (J_min * (l3_tmp + J_max * V_min * 2.0) +
         -(J_max * (l3_tmp + J_min_tmp * 2.0))) /
        2.0;
  l2 = 1.0 / A_max * (1.0 / J_max) *
       ((((l2_tmp + J_max * V_max * 2.0) + -(J_max_tmp * 2.0)) + -l5_tmp) +
        J_max * l5_tmp * (1.0 / J_min)) /
       2.0;
  l15 = A_min * (1.0 / J_min);
  l6 = -(1.0 / J_max * (A_init + -A_max));
  t[0] = l6;
  t[1] = l6;
  t[2] = l2;
  t[3] = l2;
  t[6] =
      ((((((((((((((((((((((l7 + b_A_max * rt_powd_snf(t7_idx_0, 3.0) * 4.0) +
                           b_A_min) +
                          c_A_min) +
                         c_A_max) -
                        d_A_max) +
                       b_J_max) +
                      c_J_max) -
                     b_J_min) +
                    e_A_max * t7_idx_0 * 12.0) -
                   f_A_max) +
                  g_A_max * t7_idx_0 * 24.0) +
                 c_J_min) -
                d_J_min) -
               h_A_max) +
              d_A_min * (t7_idx_0 * t7_idx_0) * 12.0) +
             i_A_max) -
            e_A_min) -
           b_A_init) +
          f_A_min) +
         g_A_min) +
        j_A_max) -
       A_min_tmp) *
      -0.041666666666666664 / l18;
  t[7] =
      ((((((((((((((((((((((l7 + b_A_max * rt_powd_snf(t7_idx_1, 3.0) * 4.0) +
                           b_A_min) +
                          c_A_min) +
                         c_A_max) -
                        d_A_max) +
                       b_J_max) +
                      c_J_max) -
                     b_J_min) +
                    e_A_max * t7_idx_1 * 12.0) -
                   f_A_max) +
                  g_A_max * t7_idx_1 * 24.0) +
                 c_J_min) -
                d_J_min) -
               h_A_max) +
              d_A_min * (t7_idx_1 * t7_idx_1) * 12.0) +
             i_A_max) -
            e_A_min) -
           b_A_init) +
          f_A_min) +
         g_A_min) +
        j_A_max) -
       A_min_tmp) *
      -0.041666666666666664 / l18;
  l2 = l15;
  l15 = A_max * (1.0 / J_min);
  t[4] = -l15;
  t[5] = -l15;
  t[10] = l10;
  t[11] = l10;
  t[8] = l2;
  t[12] = t7_idx_0;
  t[9] = l2;
  t[13] = t7_idx_1;
}

// End of code generation (abcdefg_O_VP.cpp)
