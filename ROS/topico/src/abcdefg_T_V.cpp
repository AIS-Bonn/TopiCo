//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// abcdefg_T_V.cpp
//
// Code generation for function 'abcdefg_T_V'
//

// Include files
#include "abcdefg_T_V.h"
#include "rt_nonfinite.h"
#include "topico_wrapper_data.h"
#include "topico_wrapper_rtwutil.h"
#include "topico_wrapper_types.h"
#include <cmath>

// Function Definitions
void abcdefg_T_V(double V_init, double A_init, double V_wayp, double V_max,
                 double V_min, double A_max, double A_min, double J_max,
                 double J_min, double T, double t[14])
{
  double b_t4_idx_0_tmp;
  double b_t4_idx_0_tmp_tmp;
  double c_t4_idx_0_tmp;
  double c_t4_idx_0_tmp_tmp;
  double d_t4_idx_0_tmp;
  double d_t4_idx_0_tmp_tmp;
  double e_t4_idx_0_tmp;
  double f_t4_idx_0_tmp;
  double l13;
  double l2;
  double l3_tmp;
  double l4_tmp;
  double l7;
  double t4_idx_0_tmp;
  double t4_idx_0_tmp_tmp;
  double t5_idx_1;
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
  //  Generated on 02-Sep-2019 15:54:56
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
  l7 = A_init * A_init;
  l3_tmp = A_min * A_min;
  l4_tmp = A_max * A_max;
  l2 = A_min * J_min;
  t5_idx_1 = A_max * J_min;
  t4_idx_0_tmp = l2 * J_max;
  b_t4_idx_0_tmp = A_min * A_max;
  c_t4_idx_0_tmp = b_t4_idx_0_tmp * J_min * J_max;
  t4_idx_0_tmp_tmp = t5_idx_1 * J_max;
  l2 = ((((((((rt_powd_snf(A_max, 3.0) * J_max + l2 * l7) + l2 * l4_tmp) +
             t5_idx_1 * l7) +
            t5_idx_1 * l3_tmp) +
           A_min * J_max * l4_tmp) +
          A_max * J_max * l3_tmp) -
         t4_idx_0_tmp * V_init * 2.0) +
        t4_idx_0_tmp * V_max * 2.0) +
       t4_idx_0_tmp_tmp * V_min * 2.0;
  b_t4_idx_0_tmp_tmp = J_max * l4_tmp;
  c_t4_idx_0_tmp_tmp = J_min * l7;
  t5_idx_1 = c_t4_idx_0_tmp_tmp + b_t4_idx_0_tmp_tmp;
  d_t4_idx_0_tmp = A_init * A_min * J_min * 2.0;
  b_t4_idx_0_tmp = b_t4_idx_0_tmp * J_max * 2.0;
  d_t4_idx_0_tmp_tmp = J_min * J_max;
  e_t4_idx_0_tmp = d_t4_idx_0_tmp_tmp * V_max * 2.0;
  f_t4_idx_0_tmp = t4_idx_0_tmp * T * 2.0;
  t4_idx_0_tmp *= 2.0;
  l13 = 1.0 / A_max * (1.0 / J_max) *
        ((((l7 + J_max * V_max * 2.0) + -(J_max * V_init * 2.0)) + -l4_tmp) +
         b_t4_idx_0_tmp_tmp * (1.0 / J_min)) /
        2.0;
  l7 = -(1.0 / J_max * (A_init + -A_max));
  t[0] = l7;
  t[1] = l7;
  t[2] = l13;
  t[6] = (((((t5_idx_1 - (l2 + c_t4_idx_0_tmp * t7_idx_0 * 2.0) / A_max) +
             d_t4_idx_0_tmp) +
            b_t4_idx_0_tmp) +
           e_t4_idx_0_tmp) +
          f_t4_idx_0_tmp) /
         t4_idx_0_tmp;
  t[3] = l13;
  t[7] = (((((t5_idx_1 - (l2 + c_t4_idx_0_tmp * t7_idx_1 * 2.0) / A_max) +
             d_t4_idx_0_tmp) +
            b_t4_idx_0_tmp) +
           e_t4_idx_0_tmp) +
          f_t4_idx_0_tmp) /
         t4_idx_0_tmp;
  l2 = A_min * (1.0 / J_min);
  l7 = l2;
  t5_idx_1 = l2;
  l2 = A_max * (1.0 / J_min);
  t[4] = -l2;
  t[5] = -l2;
  t[8] = l7;
  l7 = (((((((J_min * l3_tmp - J_min * l4_tmp) - J_max * l3_tmp) +
            b_t4_idx_0_tmp_tmp) +
           c_t4_idx_0_tmp_tmp) -
          d_t4_idx_0_tmp_tmp * V_init * 2.0) +
         d_t4_idx_0_tmp_tmp * V_min * 2.0) -
        t4_idx_0_tmp_tmp * l13 * 2.0) /
       t4_idx_0_tmp;
  t[10] = l7;
  t[12] = t7_idx_0;
  t[9] = t5_idx_1;
  t[11] = l7;
  t[13] = t7_idx_1;
}

// End of code generation (abcdefg_T_V.cpp)
