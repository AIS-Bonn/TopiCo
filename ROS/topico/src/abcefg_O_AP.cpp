//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// abcefg_O_AP.cpp
//
// Code generation for function 'abcefg_O_AP'
//

// Include files
#include "abcefg_O_AP.h"
#include "rt_nonfinite.h"
#include "sqrt.h"
#include "topico_wrapper_data.h"
#include "topico_wrapper_rtwutil.h"

// Function Definitions
void abcefg_O_AP(double P_init, double V_init, double A_init, double P_wayp,
                 double A_wayp, double V_max, double A_max, double A_min,
                 double J_max, double J_min, creal_T t[14])
{
  creal_T t2[2];
  creal_T l79;
  double b_l25_tmp;
  double b_l79_tmp;
  double c_l79_tmp;
  double d_l79_tmp;
  double e_l79_tmp;
  double l11_tmp;
  double l12;
  double l13;
  double l14;
  double l15;
  double l22;
  double l23;
  double l24;
  double l24_tmp;
  double l25;
  double l25_tmp;
  double l28;
  double l29;
  double l2_tmp;
  double l30;
  double l31;
  double l32;
  double l5;
  double l5_tmp;
  double l6;
  double l7;
  double l74;
  double l74_tmp;
  double l79_tmp;
  double l8_tmp;
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
  //  Generated on 28-Aug-2019 17:25:45
  l2_tmp = A_init * A_init;
  l5_tmp = A_min * A_min;
  l8_tmp = A_max * A_max;
  l9 = rt_powd_snf(A_max, 3.0);
  l11_tmp = A_wayp * A_wayp;
  l12 = rt_powd_snf(A_wayp, 3.0);
  l14 = J_min * J_min;
  l15 = J_max * J_max;
  l5 = A_min * J_min;
  l24_tmp = l5 * J_max;
  l24 = l24_tmp * V_init * 6.0;
  l25_tmp = A_max * J_min;
  b_l25_tmp = l25_tmp * J_max;
  l25 = b_l25_tmp * V_init * 6.0;
  l7 = l5_tmp * l5_tmp;
  l13 = l11_tmp * l11_tmp;
  l22 = J_min * l9 * 3.0;
  l23 = J_max * l9 * 3.0;
  l28 = l5 * l2_tmp * 3.0;
  l29 = l25_tmp * l2_tmp * 3.0;
  l30 = l5 * l8_tmp * 3.0;
  l6 = A_max * J_max;
  l31 = l6 * l5_tmp * 3.0;
  l5 = A_min * J_max;
  l32 = l5 * l8_tmp * 6.0;
  l74_tmp = J_min * J_max;
  l74 = 1.0 / (l74_tmp * l8_tmp + l24_tmp * -A_max);
  l79_tmp = A_max * l13;
  b_l79_tmp = A_min * (l8_tmp * l8_tmp);
  c_l79_tmp = A_min * A_max;
  d_l79_tmp = l5 * V_init;
  l5 = l6 * V_max;
  e_l79_tmp = A_min + -A_max;
  l79.re =
      -(e_l79_tmp *
        ((((((((((((((((((((((((((b_l79_tmp * l14 + A_max * l7 * l15) +
                                 -(b_l25_tmp * l13 * 6.0)) +
                                l79_tmp * l14 * 3.0) +
                               l79_tmp * l15 * 3.0) +
                              l12 * (c_l79_tmp * J_min * J_max) * 12.0) +
                             -(A_min * (l2_tmp * l2_tmp) * l14 * 3.0)) +
                            l7 * l14 * -A_max) +
                           -(b_l79_tmp * l15)) +
                          c_l79_tmp * rt_powd_snf(A_init, 3.0) * l14 * 8.0) +
                         l5_tmp * l9 * l15 * 3.0) +
                        c_l79_tmp * A_wayp * J_max * V_max * l14 * 24.0) +
                       -(c_l79_tmp * l12 * l14 * 8.0)) +
                      -(rt_powd_snf(A_min, 3.0) * l8_tmp * l15 * 3.0)) +
                     -(A_init * A_min * A_max * J_max * V_init * l14 * 24.0)) +
                    -(b_l25_tmp * l5_tmp * l11_tmp * 6.0)) +
                   c_l79_tmp * P_init * l14 * l15 * 24.0) +
                  d_l79_tmp * l2_tmp * l14 * 12.0) +
                 d_l79_tmp * l8_tmp * l14 * 12.0) +
                l25_tmp * V_max * l11_tmp * l15 * 12.0) +
               A_max * l5_tmp * l11_tmp * l14 * 6.0) +
              -(c_l79_tmp * P_wayp * l14 * l15 * 24.0)) +
             -(l5 * l5_tmp * l14 * 12.0)) +
            -(l5 * l11_tmp * l14 * 12.0)) +
           -(A_min * l2_tmp * l8_tmp * l14 * 6.0)) +
          A_max * l14 * l15 * (V_max * V_max) * 12.0) +
         -(A_min * l14 * l15 * (V_init * V_init) * 12.0)));
  l79.im = 0.0;
  coder::internal::scalar::b_sqrt(&l79);
  l79.re *= 1.7320508075688772;
  l79.im *= 1.7320508075688772;
  l6 = l74 *
       (((((((((-l22 + l23) + l24) - l25) - l28) + l29) + l30) + l31) - l32) +
        l79.re);
  l5 = l74 * l79.im;
  if (l5 == 0.0) {
    t2[0].re = l6 / 6.0;
    t2[0].im = 0.0;
  } else if (l6 == 0.0) {
    t2[0].re = 0.0;
    t2[0].im = l5 / 6.0;
  } else {
    t2[0].re = l6 / 6.0;
    t2[0].im = l5 / 6.0;
  }
  t2[1].re =
      -0.16666666666666666 *
      (l74 *
       (((((((((l22 - l23) - l24) + l25) + l28) - l29) - l30) - l31) + l32) +
        l79.re));
  t2[1].im = -0.16666666666666666 * l5;
  d_l79_tmp = l24_tmp * 2.0;
  l79_tmp = -(1.0 / J_max * (A_min + -A_wayp));
  l5 = 1.0 / J_min * e_l79_tmp;
  l6 = -(1.0 / J_max * (A_init + -A_max));
  l79.re = (((((((J_min * l5_tmp - J_min * l8_tmp) - J_min * l11_tmp) -
                J_max * l5_tmp) +
               J_max * l8_tmp) +
              J_max * l11_tmp) +
             l2_tmp * J_min) -
            l74_tmp * V_init * 2.0) +
           l74_tmp * V_max * 2.0;
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
  l6 = l79.re - b_l25_tmp * t2[0].re * 2.0;
  l5 = 0.0 - b_l25_tmp * t2[0].im * 2.0;
  if (l5 == 0.0) {
    t[10].re = l6 / d_l79_tmp;
    t[10].im = 0.0;
  } else if (l6 == 0.0) {
    t[10].re = 0.0;
    t[10].im = l5 / d_l79_tmp;
  } else {
    t[10].re = l6 / d_l79_tmp;
    t[10].im = l5 / d_l79_tmp;
  }
  t[12].re = l79_tmp;
  t[12].im = 0.0;
  t[3] = t2[1];
  t[7].re = 0.0;
  t[7].im = 0.0;
  t[9].re = 0.0;
  t[9].im = 0.0;
  l6 = l79.re - b_l25_tmp * t2[1].re * 2.0;
  l5 = 0.0 - b_l25_tmp * t2[1].im * 2.0;
  if (l5 == 0.0) {
    t[11].re = l6 / d_l79_tmp;
    t[11].im = 0.0;
  } else if (l6 == 0.0) {
    t[11].re = 0.0;
    t[11].im = l5 / d_l79_tmp;
  } else {
    t[11].re = l6 / d_l79_tmp;
    t[11].im = l5 / d_l79_tmp;
  }
  t[13].re = l79_tmp;
  t[13].im = 0.0;
}

// End of code generation (abcefg_O_AP.cpp)
