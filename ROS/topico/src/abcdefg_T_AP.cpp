//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// abcdefg_T_AP.cpp
//
// Code generation for function 'abcdefg_T_AP'
//

// Include files
#include "abcdefg_T_AP.h"
#include "rt_nonfinite.h"
#include "sqrt.h"
#include "topico_wrapper_data.h"
#include "topico_wrapper_rtwutil.h"

// Function Definitions
void abcdefg_T_AP(double P_init, double V_init, double A_init, double P_wayp,
                  double A_wayp, double V_max, double A_max, double A_min,
                  double J_max, double J_min, double T, creal_T t[14])
{
  creal_T t4[2];
  creal_T l64;
  double b_l29_tmp;
  double b_l64_tmp;
  double c_l64_tmp;
  double d_l64_tmp;
  double l11;
  double l12;
  double l13;
  double l20;
  double l21;
  double l22;
  double l23;
  double l24;
  double l25;
  double l26;
  double l29;
  double l29_tmp;
  double l2_tmp;
  double l5;
  double l64_tmp;
  double l64_tmp_tmp;
  double l7_tmp;
  double l8;
  double t5_idx_0;
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
  //  Generated on 28-Aug-2019 17:25:45
  l2_tmp = A_init * A_init;
  l5 = A_min * A_min;
  l7_tmp = A_max * A_max;
  l11 = J_min * J_min;
  l12 = J_max * J_max;
  l20 = A_init * A_min * A_max * J_min * 6.0;
  l13 = A_min * J_min;
  t5_idx_0 = l13 * J_max;
  l21 = t5_idx_0 * V_init * 6.0;
  l22 = t5_idx_0 * V_max * 6.0;
  l29_tmp = A_min * A_max;
  b_l29_tmp = l29_tmp * J_min;
  l29 = b_l29_tmp * J_max * T * 6.0;
  l8 = l7_tmp * l7_tmp;
  l23 = l13 * l2_tmp * 3.0;
  l24 = l13 * l7_tmp * 3.0;
  l25 = A_min * J_max * l7_tmp * 3.0;
  l26 = A_max * J_max * l5 * 3.0;
  l64_tmp = A_max * rt_powd_snf(A_min, 3.0);
  l64_tmp_tmp = A_init * A_max;
  t5_idx_0 = l64_tmp_tmp * J_max;
  b_l64_tmp = J_max * V_init;
  c_l64_tmp = J_max * V_max;
  l13 = l11 * l12;
  d_l64_tmp = A_max * A_wayp;
  l64.re =
      -A_min * A_max *
      (((((((((((((((((((((l8 * l11 - l8 * l12) - l2_tmp * l2_tmp * l11 * 3.0) +
                         l64_tmp * l12) -
                        l64_tmp * l11 * 4.0) +
                       A_max * rt_powd_snf(A_wayp, 3.0) * l11 * 4.0) -
                      t5_idx_0 * V_init * l11 * 24.0) +
                     t5_idx_0 * V_max * l11 * 24.0) +
                    A_max * rt_powd_snf(A_init, 3.0) * l11 * 8.0) -
                   l29_tmp * (A_wayp * A_wayp) * l11 * 12.0) +
                  d_l64_tmp * l5 * l11 * 12.0) +
                 A_max * P_init * l11 * l12 * 24.0) -
                A_max * P_wayp * l11 * l12 * 24.0) +
               b_l64_tmp * l2_tmp * l11 * 12.0) -
              c_l64_tmp * l2_tmp * l11 * 12.0) +
             b_l64_tmp * l7_tmp * l11 * 12.0) -
            c_l64_tmp * l7_tmp * l11 * 12.0) -
           l2_tmp * l7_tmp * l11 * 6.0) +
          V_init * V_max * l11 * l12 * 24.0) -
         l13 * (V_init * V_init) * 12.0) -
        l13 * (V_max * V_max) * 12.0) +
       A_max * T * V_max * l11 * l12 * 24.0);
  l64.im = 0.0;
  coder::internal::scalar::b_sqrt(&l64);
  l64.re *= 1.7320508075688772;
  l64.im *= 1.7320508075688772;
  t5_idx_0 = 1.0 / A_min * (1.0 / A_max) * (1.0 / J_min) * (1.0 / J_max);
  l8 = t5_idx_0 *
       ((((((((l20 + l21) - l22) - l23) - l24) + l25) - l26) + l29) + l64.re);
  l13 = t5_idx_0 * l64.im;
  if (l13 == 0.0) {
    t4[0].re = l8 / 6.0;
    t4[0].im = 0.0;
  } else if (l8 == 0.0) {
    t4[0].re = 0.0;
    t4[0].im = l13 / 6.0;
  } else {
    t4[0].re = l8 / 6.0;
    t4[0].im = l13 / 6.0;
  }
  t4[1].re =
      -0.16666666666666666 *
      (t5_idx_0 *
       ((((((((-l20 - l21) + l22) + l23) + l24) - l25) + l26) - l29) + l64.re));
  t4[1].im = -0.16666666666666666 * l13;
  l25 = A_max * J_min * J_max;
  l24 = J_max * l7_tmp;
  l13 = 1.0 / A_max * (1.0 / J_max) *
        ((((l2_tmp + c_l64_tmp * 2.0) + -(b_l64_tmp * 2.0)) + -l7_tmp) +
         l24 * (1.0 / J_min)) /
        2.0;
  l5 = 1.0 / J_max * (A_min + -A_wayp);
  l8 = A_min * (1.0 / J_min);
  t5_idx_0 = l8;
  l12 = l8;
  l8 = A_max * (1.0 / J_min);
  l11 = -(1.0 / J_max * (A_init + -A_max));
  l64_tmp = J_min * J_max;
  l64.re = ((((((((J_min * l7_tmp - l24) + l2_tmp * J_min) -
                 l64_tmp_tmp * J_min * 2.0) -
                b_l29_tmp * 2.0) +
               l29_tmp * J_max * 2.0) +
              d_l64_tmp * J_min * 2.0) -
             l64_tmp * V_init * 2.0) +
            l64_tmp * V_max * 2.0) -
           l25 * T * 2.0;
  t[0].re = l11;
  t[0].im = 0.0;
  t[1].re = l11;
  t[1].im = 0.0;
  t[2].re = l13;
  t[2].im = 0.0;
  t[3].re = l13;
  t[3].im = 0.0;
  t[4].re = -l8;
  t[4].im = 0.0;
  t[5].re = -l8;
  t[5].im = 0.0;
  t[6] = t4[0];
  t[8].re = t5_idx_0;
  t[8].im = 0.0;
  l8 = (l64.re + l25 * t4[0].re * 2.0) * -0.5;
  t5_idx_0 = l25 * t4[0].im * 2.0 * -0.5;
  if (t5_idx_0 == 0.0) {
    t[10].re = l8 / l25;
    t[10].im = 0.0;
  } else if (l8 == 0.0) {
    t[10].re = 0.0;
    t[10].im = t5_idx_0 / l25;
  } else {
    t[10].re = l8 / l25;
    t[10].im = t5_idx_0 / l25;
  }
  t[7] = t4[1];
  t[9].re = l12;
  t[9].im = 0.0;
  l8 = (l64.re + l25 * t4[1].re * 2.0) * -0.5;
  t5_idx_0 = l25 * t4[1].im * 2.0 * -0.5;
  if (t5_idx_0 == 0.0) {
    t[11].re = l8 / l25;
    t[11].im = 0.0;
  } else if (l8 == 0.0) {
    t[11].re = 0.0;
    t[11].im = t5_idx_0 / l25;
  } else {
    t[11].re = l8 / l25;
    t[11].im = t5_idx_0 / l25;
  }
  t[12].re = -l5;
  t[12].im = 0.0;
  t[13].re = -l5;
  t[13].im = 0.0;
}

// End of code generation (abcdefg_T_AP.cpp)
