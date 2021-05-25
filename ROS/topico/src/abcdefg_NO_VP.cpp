//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// abcdefg_NO_VP.cpp
//
// Code generation for function 'abcdefg_NO_VP'
//

// Include files
#include "abcdefg_NO_VP.h"
#include "rt_nonfinite.h"
#include "topico_wrapper_data.h"
#include "topico_wrapper_rtwutil.h"
#include "topico_wrapper_types.h"
#include <cmath>

// Function Definitions
void abcdefg_NO_VP(double P_init, double V_init, double A_init, double P_wayp,
                   double V_wayp, double V_max, double V_min, double A_min,
                   double J_max, double J_min, double t[14])
{
  double J_min_tmp;
  double b_A_init;
  double b_A_min;
  double b_J_max;
  double b_J_min;
  double c_A_min;
  double c_J_max;
  double c_J_min;
  double d_A_min;
  double d_J_min;
  double e_A_min;
  double f_A_min;
  double l14;
  double l2;
  double l2_tmp;
  double l3;
  double l3_tmp;
  double l4;
  double l6;
  double l7;
  double t7_idx_0;
  double t7_idx_1;
  double y;
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
  l2 = 1.0 / J_max;
  if (J_max < 0.0) {
    f_rtErrorWithMessageID("sqrt", n_emlrtRTEI.fName, n_emlrtRTEI.lineNo);
  }
  l7 = V_wayp + -V_min;
  if (l7 < 0.0) {
    f_rtErrorWithMessageID("sqrt", n_emlrtRTEI.fName, n_emlrtRTEI.lineNo);
  }
  l7 = std::sqrt(l7);
  l7 *= 1.4142135623730951 * std::sqrt(J_max);
  t7_idx_0 = -l2 * (A_min + l7);
  t7_idx_1 = -l2 * (A_min - l7);
  l2_tmp = A_init * A_init;
  l3_tmp = A_min * A_min;
  l7 = J_min * J_min;
  l6 = J_max * J_max;
  l4 = l3_tmp * l3_tmp;
  l4 = (((((l4 * l7 * 2.0 + l4 * l6 * 2.0) - l2_tmp * l2_tmp * l6 * 3.0) +
          rt_powd_snf(A_init, 3.0) * A_min * l6 * 8.0) -
         l2_tmp * l3_tmp * l6 * 6.0) -
        V_init * V_init * l7 * l6 * 12.0) +
       V_min * V_min * l7 * l6 * 12.0;
  y = rt_powd_snf(A_min, 3.0) * J_max * l7;
  l3 = l3_tmp * l7 * l6;
  b_A_min = A_min * rt_powd_snf(J_max, 3.0) * l7;
  c_A_min = A_min * P_init * l7 * l6 * 24.0;
  d_A_min = A_min * P_wayp * l7 * l6 * 24.0;
  J_min_tmp = J_min * V_init;
  b_J_min = J_min_tmp * l2_tmp * l6 * 12.0;
  c_J_min = J_min_tmp * l3_tmp * l6 * 12.0;
  l14 = J_max * V_min;
  b_J_max = l14 * l3_tmp * l7 * 12.0;
  l2 = J_min * V_max;
  d_J_min = l2 * l3_tmp * l6 * 12.0;
  c_J_max = J_max * V_max * l3_tmp * l7 * 12.0;
  e_A_min = A_min * V_min * l7 * l6;
  b_A_init = A_init * A_min * J_min * V_init * l6 * 24.0;
  f_A_min = A_min * V_max * l7 * l6;
  l7 = l2 * 2.0;
  l2 = 1.0 / A_min * (1.0 / J_min);
  l14 = l2 * (1.0 / J_max) *
        (J_min * (l3_tmp + l14 * 2.0) + -(J_max * (l3_tmp + l7))) / 2.0;
  l7 = l2 *
       ((((l2_tmp + l7) + -(J_min_tmp * 2.0)) + -l3_tmp) +
        J_min * l3_tmp * (1.0 / J_max)) /
       2.0;
  l2 = A_min * (1.0 / J_min);
  l6 = -(1.0 / J_min * (A_init + -A_min));
  t[0] = l6;
  t[1] = l6;
  t[2] = l7;
  t[3] = l7;
  t[6] = ((((((((((((l4 + y * t7_idx_0 * 12.0) +
                    l3 * (t7_idx_0 * t7_idx_0) * 12.0) +
                   b_A_min * rt_powd_snf(t7_idx_0, 3.0) * 4.0) +
                  c_A_min) -
                 d_A_min) +
                b_J_min) +
               c_J_min) +
              b_J_max) +
             d_J_min) -
            c_J_max) +
           e_A_min * t7_idx_0 * 24.0) -
          b_A_init) *
         -0.041666666666666664 / f_A_min;
  t[7] = ((((((((((((l4 + y * t7_idx_1 * 12.0) +
                    l3 * (t7_idx_1 * t7_idx_1) * 12.0) +
                   b_A_min * rt_powd_snf(t7_idx_1, 3.0) * 4.0) +
                  c_A_min) -
                 d_A_min) +
                b_J_min) +
               c_J_min) +
              b_J_max) +
             d_J_min) -
            c_J_max) +
           e_A_min * t7_idx_1 * 24.0) -
          b_A_init) *
         -0.041666666666666664 / f_A_min;
  l7 = l2;
  l2 = A_min * (1.0 / J_max);
  t[4] = -l2;
  t[5] = -l2;
  t[10] = l14;
  t[11] = l14;
  t[8] = l7;
  t[12] = t7_idx_0;
  t[9] = l7;
  t[13] = t7_idx_1;
}

// End of code generation (abcdefg_NO_VP.cpp)
