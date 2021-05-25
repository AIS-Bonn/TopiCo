//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// abcef_T_P.cpp
//
// Code generation for function 'abcef_T_P'
//

// Include files
#include "abcef_T_P.h"
#include "rt_nonfinite.h"
#include "sqrt.h"
#include "topico_wrapper_data.h"
#include "topico_wrapper_rtwutil.h"

// Function Definitions
void abcef_T_P(double P_init, double V_init, double A_init, double P_wayp,
               double A_max, double A_min, double J_max, double J_min, double T,
               creal_T t[14])
{
  creal_T t6[2];
  creal_T l48;
  double b_l48_tmp;
  double l14;
  double l18;
  double l19;
  double l2;
  double l4;
  double l43;
  double l48_tmp;
  double l6;
  double l7;
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
  //  Generated on 29-Aug-2019 13:28:28
  l2 = A_init * A_init;
  l4 = A_min * A_min;
  l6 = A_max * A_max;
  l7 = rt_powd_snf(A_max, 3.0);
  l8 = J_min * J_min;
  l9 = J_max * J_max;
  l14 = A_min * A_max * J_max * 6.0;
  l18 = J_max * l4 * 3.0;
  l19 = J_max * l6 * 3.0;
  l43 = 1.0 / (A_min * J_min * J_max + -(A_max * J_min * J_max));
  l48_tmp = J_max * T;
  b_l48_tmp = A_min + -A_max;
  l48.re = -(b_l48_tmp *
             ((((((((((((((rt_powd_snf(A_min, 3.0) * l9 + l7 * l8 * 4.0) +
                          -(rt_powd_snf(A_init, 3.0) * l8 * 4.0)) +
                         -(l7 * l9)) +
                        A_min * l6 * l9 * 3.0) +
                       P_init * l8 * l9 * 24.0) +
                      A_init * A_max * J_max * T * l8 * 24.0) +
                     A_max * l2 * l8 * 12.0) +
                    -(A_max * l4 * l9 * 3.0)) +
                   -(P_wayp * l8 * l9 * 24.0)) +
                  T * V_init * l8 * l9 * 24.0) +
                 -(A_init * l6 * l8 * 12.0)) +
                -(l48_tmp * l2 * l8 * 12.0)) +
               -(l48_tmp * l6 * l8 * 12.0)) +
              A_max * l8 * l9 * (T * T) * 12.0));
  l48.im = 0.0;
  coder::internal::scalar::b_sqrt(&l48);
  l48.re *= 1.7320508075688772;
  l48.im *= 1.7320508075688772;
  l7 = l43 * (((l14 - l18) - l19) + l48.re);
  l2 = l43 * l48.im;
  if (l2 == 0.0) {
    t6[0].re = l7 / 6.0;
    t6[0].im = 0.0;
  } else if (l7 == 0.0) {
    t6[0].re = 0.0;
    t6[0].im = l2 / 6.0;
  } else {
    t6[0].re = l7 / 6.0;
    t6[0].im = l2 / 6.0;
  }
  t6[1].re = -0.16666666666666666 * (l43 * (((-l14 + l18) + l19) + l48.re));
  t6[1].im = -0.16666666666666666 * l2;
  l4 = J_min * J_max;
  l2 = 1.0 / J_min * b_l48_tmp;
  l7 = A_init + -A_max;
  l6 = -(1.0 / J_max * l7);
  l48.re = J_min * (l7 + l48_tmp);
  t[0].re = l6;
  t[0].im = 0.0;
  t[1].re = l6;
  t[1].im = 0.0;
  t[4].re = l2;
  t[4].im = 0.0;
  t[5].re = l2;
  t[5].im = 0.0;
  l7 = l48.re - J_max * (b_l48_tmp + J_min * t6[0].re);
  l2 = 0.0 - J_max * (J_min * t6[0].im);
  if (l2 == 0.0) {
    t[2].re = l7 / l4;
    t[2].im = 0.0;
  } else if (l7 == 0.0) {
    t[2].re = 0.0;
    t[2].im = l2 / l4;
  } else {
    t[2].re = l7 / l4;
    t[2].im = l2 / l4;
  }
  t[6].re = 0.0;
  t[6].im = 0.0;
  t[8].re = 0.0;
  t[8].im = 0.0;
  t[10] = t6[0];
  t[12].re = 0.0;
  t[12].im = 0.0;
  l7 = l48.re - J_max * (b_l48_tmp + J_min * t6[1].re);
  l2 = 0.0 - J_max * (J_min * t6[1].im);
  if (l2 == 0.0) {
    t[3].re = l7 / l4;
    t[3].im = 0.0;
  } else if (l7 == 0.0) {
    t[3].re = 0.0;
    t[3].im = l2 / l4;
  } else {
    t[3].re = l7 / l4;
    t[3].im = l2 / l4;
  }
  t[7].re = 0.0;
  t[7].im = 0.0;
  t[9].re = 0.0;
  t[9].im = 0.0;
  t[11] = t6[1];
  t[13].re = 0.0;
  t[13].im = 0.0;
}

// End of code generation (abcef_T_P.cpp)
