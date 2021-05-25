//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// abcefg_T_AP.cpp
//
// Code generation for function 'abcefg_T_AP'
//

// Include files
#include "abcefg_T_AP.h"
#include "rt_nonfinite.h"
#include "sqrt.h"
#include "topico_wrapper_data.h"
#include "topico_wrapper_rtwutil.h"

// Function Definitions
void abcefg_T_AP(double P_init, double V_init, double A_init, double P_wayp,
                 double A_wayp, double A_max, double A_min, double J_max,
                 double J_min, double T, creal_T t[14])
{
  creal_T t2[2];
  creal_T l66;
  double b_l66_tmp;
  double l10;
  double l11;
  double l13;
  double l14;
  double l16;
  double l17;
  double l17_tmp;
  double l18;
  double l19;
  double l2;
  double l25;
  double l26;
  double l27;
  double l28;
  double l29;
  double l4;
  double l5;
  double l6;
  double l61;
  double l66_tmp;
  double l7;
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
  l2 = A_init * A_init;
  l4 = A_min * A_min;
  l5 = rt_powd_snf(A_min, 3.0);
  l6 = A_max * A_max;
  l7 = rt_powd_snf(A_max, 3.0);
  l10 = J_min * J_min;
  l11 = J_max * J_max;
  l13 = A_min * J_min * J_max;
  l14 = A_max * J_min * J_max;
  l16 = A_init * A_min * J_min * 6.0;
  l17_tmp = A_init * A_max;
  l17 = l17_tmp * J_min * 6.0;
  l61 = A_min * A_max;
  l18 = l61 * J_min * 6.0;
  l19 = l61 * J_max * 6.0;
  l25 = J_max * l4 * 3.0;
  l26 = J_max * l6 * 3.0;
  l27 = J_min * l6 * 6.0;
  l28 = T * l13 * 6.0;
  l29 = T * l14 * 6.0;
  l61 = 1.0 / (l13 + -l14);
  l66_tmp = J_max * T;
  b_l66_tmp = A_min + -A_max;
  l66.re =
      -(b_l66_tmp * ((((((((((((((((((l5 * l11 + l7 * l10 * 4.0) +
                                     rt_powd_snf(A_wayp, 3.0) * l10 * 4.0) +
                                    -(rt_powd_snf(A_init, 3.0) * l10 * 4.0)) +
                                   -(l5 * l10 * 4.0)) +
                                  -(l7 * l11)) +
                                 A_min * l6 * l11 * 3.0) +
                                P_init * l10 * l11 * 24.0) +
                               l17_tmp * J_max * T * l10 * 24.0) +
                              A_max * l2 * l10 * 12.0) +
                             -(A_max * l4 * l11 * 3.0)) +
                            A_wayp * l4 * l10 * 12.0) +
                           -(P_wayp * l10 * l11 * 24.0)) +
                          T * V_init * l10 * l11 * 24.0) +
                         -(A_init * l6 * l10 * 12.0)) +
                        -(A_min * (A_wayp * A_wayp) * l10 * 12.0)) +
                       -(l66_tmp * l2 * l10 * 12.0)) +
                      -(l66_tmp * l6 * l10 * 12.0)) +
                     A_max * l10 * l11 * (T * T) * 12.0));
  l66.im = 0.0;
  coder::internal::scalar::b_sqrt(&l66);
  l66.re *= 1.7320508075688772;
  l66.im *= 1.7320508075688772;
  t2[0].re =
      -0.16666666666666666 *
      (l61 * (((((((((l17 + l18) + -l16) + -l19) + l25) + l26) + l29) + -l27) +
               -l28) +
              l66.re));
  l2 = l61 * l66.im;
  t2[0].im = -0.16666666666666666 * l2;
  l61 *= ((((((((l16 - l17) - l18) + l19) - l25) - l26) + l27) + l28) - l29) +
         l66.re;
  if (l2 == 0.0) {
    t2[1].re = l61 / 6.0;
    t2[1].im = 0.0;
  } else if (l61 == 0.0) {
    t2[1].re = 0.0;
    t2[1].im = l2 / 6.0;
  } else {
    t2[1].re = l61 / 6.0;
    t2[1].im = l2 / 6.0;
  }
  l13 = J_min * J_max;
  l14 = -(1.0 / J_max * (A_min + -A_wayp));
  l5 = 1.0 / J_min * b_l66_tmp;
  l6 = -(1.0 / J_max * (A_init + -A_max));
  l66.re = J_min * ((((A_init + A_min) - A_wayp) + -A_max) + l66_tmp);
  t[0].re = l6;
  t[0].im = 0.0;
  t[1].re = l6;
  t[1].im = 0.0;
  t[4].re = l5;
  t[4].im = 0.0;
  t[5].re = l5;
  t[5].im = 0.0;
  t[2] = t2[0];
  t[6].re = 0.0;
  t[6].im = 0.0;
  t[8].re = 0.0;
  t[8].im = 0.0;
  l61 = l66.re - J_max * (b_l66_tmp + J_min * t2[0].re);
  l2 = 0.0 - J_max * (J_min * t2[0].im);
  if (l2 == 0.0) {
    t[10].re = l61 / l13;
    t[10].im = 0.0;
  } else if (l61 == 0.0) {
    t[10].re = 0.0;
    t[10].im = l2 / l13;
  } else {
    t[10].re = l61 / l13;
    t[10].im = l2 / l13;
  }
  t[12].re = l14;
  t[12].im = 0.0;
  t[3] = t2[1];
  t[7].re = 0.0;
  t[7].im = 0.0;
  t[9].re = 0.0;
  t[9].im = 0.0;
  l61 = l66.re - J_max * (b_l66_tmp + J_min * t2[1].re);
  l2 = 0.0 - J_max * (J_min * t2[1].im);
  if (l2 == 0.0) {
    t[11].re = l61 / l13;
    t[11].im = 0.0;
  } else if (l61 == 0.0) {
    t[11].re = 0.0;
    t[11].im = l2 / l13;
  } else {
    t[11].re = l61 / l13;
    t[11].im = l2 / l13;
  }
  t[13].re = l14;
  t[13].im = 0.0;
}

// End of code generation (abcefg_T_AP.cpp)
