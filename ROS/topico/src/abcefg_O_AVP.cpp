//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// abcefg_O_AVP.cpp
//
// Code generation for function 'abcefg_O_AVP'
//

// Include files
#include "abcefg_O_AVP.h"
#include "rt_nonfinite.h"
#include "sqrt.h"
#include "topico_wrapper_data.h"
#include "topico_wrapper_rtwutil.h"

// Function Definitions
void abcefg_O_AVP(double P_init, double V_init, double A_init, double P_wayp,
                  double V_wayp, double A_wayp, double A_max, double A_min,
                  double J_max, double J_min, creal_T t[14])
{
  creal_T t6[2];
  creal_T l71;
  double b_l27_tmp;
  double l11_tmp;
  double l14;
  double l15;
  double l23_tmp;
  double l24;
  double l25;
  double l26;
  double l26_tmp;
  double l27;
  double l27_tmp;
  double l28;
  double l29;
  double l29_tmp;
  double l2_tmp;
  double l30;
  double l30_tmp;
  double l31;
  double l32;
  double l5;
  double l5_tmp;
  double l6;
  double l7;
  double l71_tmp;
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
  //  Generated on 28-Aug-2019 12:13:20
  l2_tmp = A_init * A_init;
  l5_tmp = A_min * A_min;
  l6 = rt_powd_snf(A_min, 3.0);
  l8_tmp = A_max * A_max;
  l11_tmp = A_wayp * A_wayp;
  l14 = J_min * J_min;
  l15 = J_max * J_max;
  l5 = A_min * J_min;
  l26_tmp = l5 * J_max;
  l26 = l26_tmp * V_wayp * 6.0;
  l27_tmp = A_max * J_min;
  b_l27_tmp = l27_tmp * J_max;
  l27 = b_l27_tmp * V_wayp * 6.0;
  l7 = l5_tmp * l5_tmp;
  l23_tmp = A_min + -A_max;
  l24 = J_min * l6 * 3.0;
  l25 = J_max * l6 * 3.0;
  l28 = l27_tmp * l5_tmp * 3.0;
  l29_tmp = A_min * J_max;
  l29 = l29_tmp * l8_tmp * 3.0;
  l30_tmp = A_max * J_max;
  l30 = l30_tmp * l5_tmp * 6.0;
  l31 = l5 * l11_tmp * 3.0;
  l32 = l27_tmp * l11_tmp * 3.0;
  l71_tmp = A_min * (l8_tmp * l8_tmp);
  l27_tmp = A_min * A_max;
  l29_tmp *= V_init;
  l5 = l30_tmp * V_wayp;
  l71.re = l23_tmp *
           (((((((((((((((((((((-(l71_tmp * l14) + A_max * l7 * l14) +
                               l71_tmp * l15) +
                              A_min * (l2_tmp * l2_tmp) * l14 * 3.0) -
                             A_max * (l11_tmp * l11_tmp) * l14 * 3.0) -
                            l27_tmp * rt_powd_snf(A_init, 3.0) * l14 * 8.0) +
                           l27_tmp * rt_powd_snf(A_wayp, 3.0) * l14 * 8.0) -
                          l5_tmp * rt_powd_snf(A_max, 3.0) * l15 * 3.0) +
                         l6 * l8_tmp * l15 * 3.0) +
                        A_init * A_min * A_max * J_max * V_init * l14 * 24.0) -
                       l27_tmp * A_wayp * J_max * V_wayp * l14 * 24.0) -
                      l27_tmp * P_init * l14 * l15 * 24.0) +
                     l27_tmp * P_wayp * l14 * l15 * 24.0) -
                    l29_tmp * l2_tmp * l14 * 12.0) -
                   l29_tmp * l8_tmp * l14 * 12.0) +
                  l5 * l5_tmp * l14 * 12.0) +
                 l5 * l11_tmp * l14 * 12.0) +
                A_min * l2_tmp * l8_tmp * l14 * 6.0) -
               A_max * l5_tmp * l11_tmp * l14 * 6.0) +
              A_min * l14 * l15 * (V_init * V_init) * 12.0) -
             A_max * l14 * l15 * (V_wayp * V_wayp) * 12.0) +
            l7 * l15 * -A_max);
  l71.im = 0.0;
  coder::internal::scalar::b_sqrt(&l71);
  l71.re *= 1.7320508075688772;
  l71.im *= 1.7320508075688772;
  l5 = 1.0 / A_min * (1.0 / J_min) * (1.0 / J_max) * (1.0 / l23_tmp);
  t6[0].re =
      -0.16666666666666666 *
      (l5 *
       (((((((((-l24 + l25) - l26) + l27) + l28) + l29) - l30) + l31) - l32) +
        l71.re));
  l29_tmp = l5 * l71.im;
  t6[0].im = -0.16666666666666666 * l29_tmp;
  l5 *= ((((((((l24 - l25) + l26) - l27) - l28) - l29) + l30) - l31) + l32) +
        l71.re;
  if (l29_tmp == 0.0) {
    t6[1].re = l5 / 6.0;
    t6[1].im = 0.0;
  } else if (l5 == 0.0) {
    t6[1].re = 0.0;
    t6[1].im = l29_tmp / 6.0;
  } else {
    t6[1].re = l5 / 6.0;
    t6[1].im = l29_tmp / 6.0;
  }
  l27_tmp = -(1.0 / J_max * (A_min + -A_wayp));
  l5 = 1.0 / J_min * l23_tmp;
  l6 = -(1.0 / J_max * (A_init + -A_max));
  l71_tmp = J_min * J_max;
  l71.re = ((((((-J_min * l5_tmp + J_min * l8_tmp) + J_max * l5_tmp) -
               J_max * l8_tmp) -
              l2_tmp * J_min) +
             l11_tmp * J_min) +
            l71_tmp * V_init * 2.0) -
           l71_tmp * V_wayp * 2.0;
  t[0].re = l6;
  t[0].im = 0.0;
  t[1].re = l6;
  t[1].im = 0.0;
  t[4].re = l5;
  t[4].im = 0.0;
  t[5].re = l5;
  t[5].im = 0.0;
  l5 = (l71.re + l26_tmp * t6[0].re * 2.0) * -0.5;
  l29_tmp = l26_tmp * t6[0].im * 2.0 * -0.5;
  if (l29_tmp == 0.0) {
    t[2].re = l5 / b_l27_tmp;
    t[2].im = 0.0;
  } else if (l5 == 0.0) {
    t[2].re = 0.0;
    t[2].im = l29_tmp / b_l27_tmp;
  } else {
    t[2].re = l5 / b_l27_tmp;
    t[2].im = l29_tmp / b_l27_tmp;
  }
  t[6].re = 0.0;
  t[6].im = 0.0;
  t[8].re = 0.0;
  t[8].im = 0.0;
  t[10] = t6[0];
  t[12].re = l27_tmp;
  t[12].im = 0.0;
  l5 = (l71.re + l26_tmp * t6[1].re * 2.0) * -0.5;
  l29_tmp = l26_tmp * t6[1].im * 2.0 * -0.5;
  if (l29_tmp == 0.0) {
    t[3].re = l5 / b_l27_tmp;
    t[3].im = 0.0;
  } else if (l5 == 0.0) {
    t[3].re = 0.0;
    t[3].im = l29_tmp / b_l27_tmp;
  } else {
    t[3].re = l5 / b_l27_tmp;
    t[3].im = l29_tmp / b_l27_tmp;
  }
  t[7].re = 0.0;
  t[7].im = 0.0;
  t[9].re = 0.0;
  t[9].im = 0.0;
  t[11] = t6[1];
  t[13].re = l27_tmp;
  t[13].im = 0.0;
}

// End of code generation (abcefg_O_AVP.cpp)
