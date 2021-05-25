//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// abcdef_T_P.cpp
//
// Code generation for function 'abcdef_T_P'
//

// Include files
#include "abcdef_T_P.h"
#include "rt_nonfinite.h"
#include "sqrt.h"
#include "topico_wrapper_data.h"
#include "topico_wrapper_rtwutil.h"

// Function Definitions
void abcdef_T_P(double P_init, double V_init, double A_init, double P_wayp,
                double V_max, double A_max, double A_min, double J_max,
                double J_min, double T, creal_T t[14])
{
  creal_T t6[2];
  creal_T l49;
  double b_l49_tmp;
  double c_l49_tmp;
  double d_l49_tmp;
  double l10;
  double l15_tmp;
  double l18;
  double l18_tmp;
  double l2_tmp;
  double l49_tmp;
  double l7_tmp;
  double l8;
  double l9;
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
  //  Generated on 29-Aug-2019 14:36:33
  l2_tmp = A_init * A_init;
  l7_tmp = A_max * A_max;
  l9 = J_min * J_min;
  l10 = J_max * J_max;
  l15_tmp = 1.0 / J_min;
  l8 = l7_tmp * l7_tmp;
  l18_tmp = A_max * J_max;
  l18 = l18_tmp * (A_min * A_min) * 3.0;
  l49_tmp = A_init * A_max * J_max;
  b_l49_tmp = J_max * V_init;
  c_l49_tmp = J_max * V_max;
  d_l49_tmp = l9 * l10;
  l49.re = -A_min * A_max *
           (((((((((((((((((l8 * l9 - l8 * l10) - l2_tmp * l2_tmp * l9 * 3.0) +
                          A_max * rt_powd_snf(A_min, 3.0) * l10) -
                         l49_tmp * V_init * l9 * 24.0) +
                        l49_tmp * V_max * l9 * 24.0) +
                       A_max * rt_powd_snf(A_init, 3.0) * l9 * 8.0) +
                      A_max * P_init * l9 * l10 * 24.0) -
                     A_max * P_wayp * l9 * l10 * 24.0) +
                    b_l49_tmp * l2_tmp * l9 * 12.0) -
                   c_l49_tmp * l2_tmp * l9 * 12.0) +
                  b_l49_tmp * l7_tmp * l9 * 12.0) -
                 c_l49_tmp * l7_tmp * l9 * 12.0) -
                l2_tmp * l7_tmp * l9 * 6.0) +
               V_init * V_max * l9 * l10 * 24.0) -
              d_l49_tmp * (V_init * V_init) * 12.0) -
             d_l49_tmp * (V_max * V_max) * 12.0) +
            A_max * T * V_max * l9 * l10 * 24.0);
  l49.im = 0.0;
  coder::internal::scalar::b_sqrt(&l49);
  l49.re *= 1.7320508075688772;
  l49.im *= 1.7320508075688772;
  l9 = 1.0 / A_min * (1.0 / A_max) * l15_tmp * (1.0 / J_max);
  t6[0].re = -0.16666666666666666 * (l9 * (l18 - l49.re));
  t6[0].im = -0.16666666666666666 * (l9 * (0.0 - l49.im));
  t6[1].re = -0.16666666666666666 * (l9 * (l18 + l49.re));
  t6[1].im = -0.16666666666666666 * (l9 * l49.im);
  l8 = 1.0 / A_max * (1.0 / J_max) *
       ((((l2_tmp + c_l49_tmp * 2.0) + -(b_l49_tmp * 2.0)) + -l7_tmp) +
        J_max * l7_tmp * (1.0 / J_min)) /
       2.0;
  d_l49_tmp = A_min * (1.0 / J_min);
  l9 = A_max * (1.0 / J_min);
  l10 = -(1.0 / J_max * (A_init + -A_max));
  l49_tmp = J_min * J_max;
  l49.re = ((A_init * J_min - A_max * J_min) + l18_tmp) + l49_tmp * T;
  t[0].re = l10;
  t[0].im = 0.0;
  t[1].re = l10;
  t[1].im = 0.0;
  t[4].re = -l9;
  t[4].im = 0.0;
  t[5].re = -l9;
  t[5].im = 0.0;
  t[2].re = l8;
  t[2].im = 0.0;
  l9 = l15_tmp * (l49.re - l49_tmp * ((l8 + t6[0].re) + d_l49_tmp));
  l10 = l15_tmp * (0.0 - l49_tmp * t6[0].im);
  if (l10 == 0.0) {
    t[6].re = l9 / J_max;
    t[6].im = 0.0;
  } else if (l9 == 0.0) {
    t[6].re = 0.0;
    t[6].im = l10 / J_max;
  } else {
    t[6].re = l9 / J_max;
    t[6].im = l10 / J_max;
  }
  t[8].re = d_l49_tmp;
  t[8].im = 0.0;
  t[10] = t6[0];
  t[12].re = 0.0;
  t[12].im = 0.0;
  t[3].re = l8;
  t[3].im = 0.0;
  l9 = l15_tmp * (l49.re - l49_tmp * ((l8 + t6[1].re) + d_l49_tmp));
  l10 = l15_tmp * (0.0 - l49_tmp * t6[1].im);
  if (l10 == 0.0) {
    t[7].re = l9 / J_max;
    t[7].im = 0.0;
  } else if (l9 == 0.0) {
    t[7].re = 0.0;
    t[7].im = l10 / J_max;
  } else {
    t[7].re = l9 / J_max;
    t[7].im = l10 / J_max;
  }
  t[9].re = d_l49_tmp;
  t[9].im = 0.0;
  t[11] = t6[1];
  t[13].re = 0.0;
  t[13].im = 0.0;
}

// End of code generation (abcdef_T_P.cpp)
