//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// abcdefg_TV_AVP.cpp
//
// Code generation for function 'abcdefg_TV_AVP'
//

// Include files
#include "abcdefg_TV_AVP.h"
#include "rt_nonfinite.h"
#include "sqrt.h"
#include "topico_wrapper_data.h"
#include "topico_wrapper_rtwutil.h"

// Function Definitions
void abcdefg_TV_AVP(double P_init, double V_init, double A_init, double P_wayp,
                    double V_wayp, double A_wayp, double A_max, double A_min,
                    double J_max, double J_min, double T, creal_T t[14])
{
  creal_T t4[2];
  creal_T t6[2];
  creal_T l114;
  double ar_tmp;
  double b_l114_tmp;
  double c_l114_tmp;
  double d;
  double d1;
  double d2;
  double d3;
  double d4;
  double d5;
  double d6;
  double d_l114_tmp;
  double e_l114_tmp;
  double f_l114_tmp;
  double l10;
  double l114_tmp;
  double l11_tmp;
  double l12;
  double l14;
  double l15;
  double l21_tmp;
  double l22_tmp;
  double l24;
  double l25;
  double l25_tmp;
  double l2_tmp;
  double l3;
  double l5_tmp;
  double l6;
  double l7;
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
  //  Generated on 28-Aug-2019 12:21:18
  l2_tmp = A_init * A_init;
  l3 = rt_powd_snf(A_init, 3.0);
  l5_tmp = A_min * A_min;
  l8_tmp = A_max * A_max;
  l11_tmp = A_wayp * A_wayp;
  l12 = rt_powd_snf(A_wayp, 3.0);
  l14 = J_min * J_min;
  l15 = J_max * J_max;
  l21_tmp = 1.0 / J_min;
  l22_tmp = 1.0 / J_max;
  l7 = l5_tmp * l5_tmp;
  l10 = l8_tmp * l8_tmp;
  l6 = A_min * J_max;
  l24 = l6 * l8_tmp * 3.0;
  l25_tmp = A_max * J_max;
  l25 = l25_tmp * l5_tmp * 3.0;
  d = A_min * rt_powd_snf(A_max, 3.0);
  ar_tmp = A_max * rt_powd_snf(A_min, 3.0);
  d1 = A_min * A_wayp;
  d2 = d1 * J_max;
  d3 = A_init * A_max;
  d4 = d3 * J_max;
  d5 = A_init * A_min;
  d6 = d5 * A_max;
  l114_tmp = A_min * A_max;
  b_l114_tmp = J_max * V_init;
  c_l114_tmp = J_max * V_wayp;
  d_l114_tmp = l14 * l15;
  e_l114_tmp = l25_tmp * T;
  l6 *= T;
  f_l114_tmp = A_max * A_wayp;
    l114.re = l114_tmp * ((((((((((((((((((((((((((((((((((((((((((((((((((((((((l7 * l15 + l10 * l15) + l2_tmp * l2_tmp * l14 * 3.0) + l11_tmp * l11_tmp * l14 * 3.0) + -(l7 * l14)) + -(l10 * l14)) + d * l14 * 4.0) + ar_tmp * l14 * 4.0) + d4 * V_init * l14 * 24.0) + d2 * V_wayp * l14 * 24.0) + -(A_min * l3 * l14 * 4.0)) + -(d * l15)) + -(ar_tmp * l15)) + -(A_max * l12 * l14 * 4.0)) + -(d6 * A_wayp * l14 * 24.0)) + -(d2 * V_init * l14 * 24.0)) + -(d4 * V_wayp * l14 * 24.0)) + d3 * l5_tmp * l14 * 12.0) + l114_tmp * l2_tmp * l14 * 12.0) + d1 * l2_tmp * l14 * 12.0) + d3 * l11_tmp * l14 * 12.0) + l114_tmp * l11_tmp * l14 * 12.0) + d1 * l8_tmp * l14 * 12.0) + A_min * P_init * l14 * l15 * 24.0) + A_max * P_wayp * l14 * l15 * 24.0) + b_l114_tmp * l5_tmp * l14 * 12.0) + c_l114_tmp * l2_tmp * l14 * 12.0) + b_l114_tmp * l11_tmp * l14 * 12.0) + c_l114_tmp * l8_tmp * l14 * 12.0) + l2_tmp * l8_tmp * l14 * 6.0) + l5_tmp * l11_tmp * l14 * 6.0) + d6 * J_max * T * l14 * 24.0) + -(A_max * l3 * l14 * 8.0)) + -(A_min * l12 * l14 * 8.0)) + -(d5 * l8_tmp * l14 * 12.0)) + -(f_l114_tmp * l5_tmp * l14 * 12.0)) + -(A_max * P_init * l14 * l15 * 24.0)) + -(A_min * P_wayp * l14 * l15 * 24.0)) + -(b_l114_tmp * l2_tmp * l14 * 12.0)) + -(b_l114_tmp * l8_tmp * l14 * 12.0)) + -(c_l114_tmp * l5_tmp * l14 * 12.0)) + -(c_l114_tmp * l11_tmp * l14 * 12.0)) + -(l2_tmp * l5_tmp * l14 * 6.0)) + -(l5_tmp * l8_tmp * l14 * 6.0)) + -(l2_tmp * l11_tmp * l14 * 6.0)) + -(l8_tmp * l11_tmp * l14 * 6.0)) + -(V_init * V_wayp * l14 * l15 * 24.0)) + d_l114_tmp * (V_init * V_init) * 12.0) + d_l114_tmp * (V_wayp * V_wayp) * 12.0) + -(l114_tmp * A_wayp * J_max * T * l14 * 24.0)) + e_l114_tmp * l5_tmp * l14 * 12.0) + e_l114_tmp * l11_tmp * l14 * 12.0) + A_min * T * V_init * l14 * l15 * 24.0) + -(l6 * l2_tmp * l14 * 12.0)) + -(l6 * l8_tmp * l14 * 12.0)) + -(A_max * T * V_wayp * l14 * l15 * 24.0)) + l114_tmp * l14 * l15 * (T * T) * 12.0);
    l114.im = 0.0;
    coder::internal::scalar::b_sqrt(&l114);
    l114.re *= 1.7320508075688772;
    l114.im *= 1.7320508075688772;
    l6 = 1.0 / A_min * (1.0 / A_max) * l21_tmp * l22_tmp;
    t4[0].re = -0.16666666666666666 * (l6 * ((-l24 + l25) + l114.re));
    l7 = l6 * l114.im;
    t4[0].im = -0.16666666666666666 * l7;
    l14 = l6 * ((l24 - l25) + l114.re);
    if (l7 == 0.0) {
      t4[1].re = l14 / 6.0;
      t4[1].im = 0.0;
    } else if (l14 == 0.0) {
      t4[1].re = 0.0;
      t4[1].im = l7 / 6.0;
    } else {
      t4[1].re = l14 / 6.0;
      t4[1].im = l7 / 6.0;
    }
    l10 = A_max * J_min;
    l15 = l10 * J_max;
    l12 = A_min * J_min * J_max * 2.0 - l15 * 2.0;
    ar_tmp = J_min * J_max;
    l6 = (((((((((((J_min * l5_tmp + J_min * l8_tmp) - J_max * l5_tmp) -
                  J_max * l8_tmp) +
                 l2_tmp * J_min) -
                l11_tmp * J_min) -
               d3 * J_min * 2.0) -
              l114_tmp * J_min * 2.0) +
             l114_tmp * J_max * 2.0) +
            f_l114_tmp * J_min * 2.0) -
           ar_tmp * V_init * 2.0) +
          ar_tmp * V_wayp * 2.0) -
         l15 * T * 2.0;
    l14 = l6 + l15 * t4[0].re * 2.0;
    l7 = l15 * t4[0].im * 2.0;
    if (l7 == 0.0) {
      t6[0].re = l14 / l12;
      t6[0].im = 0.0;
    } else if (l14 == 0.0) {
      t6[0].re = 0.0;
      t6[0].im = l7 / l12;
    } else {
      t6[0].re = l14 / l12;
      t6[0].im = l7 / l12;
    }
    l14 = l6 + l15 * t4[1].re * 2.0;
    l7 = l15 * t4[1].im * 2.0;
    if (l7 == 0.0) {
      t6[1].re = l14 / l12;
      t6[1].im = 0.0;
    } else if (l14 == 0.0) {
      t6[1].re = 0.0;
      t6[1].im = l7 / l12;
    } else {
      t6[1].re = l14 / l12;
      t6[1].im = l7 / l12;
    }
    l12 = 1.0 / J_max * (A_min + -A_wayp);
    l6 = -(1.0 / J_max * (A_init + -A_max));
    t[0].re = l6;
    t[0].im = 0.0;
    t[1].re = l6;
    t[1].im = 0.0;
    l14 = A_min * l21_tmp;
    l15 = l22_tmp * (A_min - A_wayp);
    l6 = ((A_init * J_min - l10) + l25_tmp) + ar_tmp * T;
    d = l21_tmp * l22_tmp;
    t[2].re = d * (l6 - ar_tmp * (((t4[0].re + t6[0].re) + l14) - l15));
    t[2].im = d * (0.0 - ar_tmp * (t4[0].im + t6[0].im));
    t[6] = t4[0];
    t[3].re = d * (l6 - ar_tmp * (((t4[1].re + t6[1].re) + l14) - l15));
    t[3].im = d * (0.0 - ar_tmp * (t4[1].im + t6[1].im));
    t[7] = t4[1];
    l3 = A_min * (1.0 / J_min);
    l6 = l3;
    l3 = A_max * (1.0 / J_min);
    t[4].re = -l3;
    t[4].im = 0.0;
    t[5].re = -l3;
    t[5].im = 0.0;
    t[8].re = l6;
    t[8].im = 0.0;
    t[10] = t6[0];
    t[9].re = l6;
    t[9].im = 0.0;
    t[11] = t6[1];
    t[12].re = -l12;
    t[12].im = 0.0;
    t[13].re = -l12;
    t[13].im = 0.0;
}

// End of code generation (abcdefg_TV_AVP.cpp)
