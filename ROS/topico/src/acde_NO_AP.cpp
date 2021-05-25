//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// acde_NO_AP.cpp
//
// Code generation for function 'acde_NO_AP'
//

// Include files
#include "acde_NO_AP.h"
#include "power.h"
#include "rt_nonfinite.h"
#include "sqrt.h"
#include "topico_wrapper_data.h"
#include "topico_wrapper_rtwutil.h"
#include "topico_wrapper_types.h"
#include <cmath>

// Function Definitions
void acde_NO_AP(double P_init, double V_init, double A_init, double P_wayp,
                double A_wayp, double V_max, double J_max, double J_min,
                creal_T t[14])
{
  creal_T l7[2];
  creal_T l8[2];
  creal_T t1[2];
  creal_T l20;
  double A_init_re;
  double A_init_tmp;
  double A_wayp_re;
  double J_max_tmp;
  double P_init_re;
  double P_wayp_re;
  double V_init_tmp;
  double b_A_init;
  double b_A_init_tmp;
  double b_A_wayp;
  double b_A_wayp_re;
  double b_J_max_tmp;
  double l15;
  double l15_tmp;
  double l2;
  double l20_tmp;
  double l4;
  double l4_tmp;
  double l5_tmp;
  double l6;
  double re;
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
  //  Generated on 28-Aug-2019 17:25:45
  l4 = A_init * J_min;
  l5_tmp = A_init * J_max;
  if (J_max < 0.0) {
    f_rtErrorWithMessageID("sqrt", n_emlrtRTEI.fName, n_emlrtRTEI.lineNo);
  }
  l15_tmp = J_min * J_min;
  l15 = 1.0 / (l15_tmp + -(J_min * J_max));
  l20_tmp = A_init * A_init;
  l20.re = -((J_min + -J_max) *
             ((l20_tmp + J_min * V_max * 2.0) + -(J_min * V_init * 2.0)));
  l20.im = 0.0;
  coder::internal::scalar::b_sqrt(&l20);
  l6 = std::sqrt(J_max);
  l20.re *= l6;
  l20.im *= l6;
  t1[0].re = l15 * ((-l4 + l5_tmp) + l20.re);
  t1[0].im = l15 * l20.im;
  t1[1].re = -l15 * ((l4 - l5_tmp) + l20.re);
  t1[1].im = -l15 * l20.im;
  l4 = rt_powd_snf(J_min, 3.0);
  l6 = J_max * J_max;
  coder::power(t1, l8);
  l15 = l15_tmp * l15_tmp;
  y = rt_powd_snf(J_min, 5.0);
  l20.re =
      rt_powd_snf(A_init, 3.0) * l15_tmp * 2.0 + rt_powd_snf(A_wayp, 3.0) * l6;
  b_A_init = A_init * l15;
  J_max_tmp = J_max * l15;
  P_init_re = P_init * l15_tmp * l6 * 6.0;
  P_wayp_re = P_wayp * l15_tmp * l6 * 6.0;
  l4_tmp = l4 * l6;
  l2 = l20_tmp * l4;
  b_J_max_tmp = J_max * l20_tmp * l15_tmp;
  V_init_tmp = V_init * l15_tmp * l6;
  l15 = A_wayp * J_min;
  A_wayp_re = l15 * J_max * l20_tmp * 3.0;
  A_init_re = l5_tmp * V_init * l15_tmp * 6.0;
  b_A_wayp_re = l15 * V_init * l6 * 6.0;
  A_init_tmp = l5_tmp * l4;
  b_A_wayp = A_wayp * J_max * l4;
  l5_tmp = J_max * V_init * l4;
  b_A_init_tmp = A_init * l15_tmp * l6;
  l4 = A_wayp * l15_tmp * l6;
  l15 = A_init * A_wayp;
  l20_tmp = l15 * J_max * l15_tmp;
  l6 *= l15 * J_min;
  re = t1[0].re * t1[0].re - t1[0].im * t1[0].im;
  l15 = t1[0].re * t1[0].im;
  l15_tmp = l15 + l15;
  l7[0].re = re;
  l7[0].im = l15_tmp;
  l8[0].re =
      ((((((((((((((((((l20.re + y * l8[0].re * 2.0) + b_A_init * re * 6.0) -
                      J_max_tmp * l8[0].re * 3.0) +
                     P_init_re) -
                    P_wayp_re) +
                   l4_tmp * l8[0].re) +
                  l2 * t1[0].re * 6.0) -
                 b_J_max_tmp * t1[0].re * 6.0) +
                V_init_tmp * t1[0].re * 6.0) -
               A_wayp_re) -
              A_init_re) +
             b_A_wayp_re) -
            A_init_tmp * re * 9.0) -
           b_A_wayp * re * 3.0) -
          l5_tmp * t1[0].re * 6.0) +
         b_A_init_tmp * re * 3.0) +
        l4 * re * 3.0) -
       l20_tmp * t1[0].re * 6.0) +
      l6 * t1[0].re * 6.0;
  l8[0].im = ((((((((((((y * l8[0].im * 2.0 + b_A_init * l15_tmp * 6.0) -
                        J_max_tmp * l8[0].im * 3.0) +
                       l4_tmp * l8[0].im) +
                      l2 * t1[0].im * 6.0) -
                     b_J_max_tmp * t1[0].im * 6.0) +
                    V_init_tmp * t1[0].im * 6.0) -
                   A_init_tmp * l15_tmp * 9.0) -
                  b_A_wayp * l15_tmp * 3.0) -
                 l5_tmp * t1[0].im * 6.0) +
                b_A_init_tmp * l15_tmp * 3.0) +
               l4 * l15_tmp * 3.0) -
              l20_tmp * t1[0].im * 6.0) +
             l6 * t1[0].im * 6.0;
  re = t1[1].re * t1[1].re - t1[1].im * t1[1].im;
  l15 = t1[1].re * t1[1].im;
  l15_tmp = l15 + l15;
  l8[1].re =
      ((((((((((((((((((l20.re + y * l8[1].re * 2.0) + b_A_init * re * 6.0) -
                      J_max_tmp * l8[1].re * 3.0) +
                     P_init_re) -
                    P_wayp_re) +
                   l4_tmp * l8[1].re) +
                  l2 * t1[1].re * 6.0) -
                 b_J_max_tmp * t1[1].re * 6.0) +
                V_init_tmp * t1[1].re * 6.0) -
               A_wayp_re) -
              A_init_re) +
             b_A_wayp_re) -
            A_init_tmp * re * 9.0) -
           b_A_wayp * re * 3.0) -
          l5_tmp * t1[1].re * 6.0) +
         b_A_init_tmp * re * 3.0) +
        l4 * re * 3.0) -
       l20_tmp * t1[1].re * 6.0) +
      l6 * t1[1].re * 6.0;
  l8[1].im = ((((((((((((y * l8[1].im * 2.0 + b_A_init * l15_tmp * 6.0) -
                        J_max_tmp * l8[1].im * 3.0) +
                       l4_tmp * l8[1].im) +
                      l2 * t1[1].im * 6.0) -
                     b_J_max_tmp * t1[1].im * 6.0) +
                    V_init_tmp * t1[1].im * 6.0) -
                   A_init_tmp * l15_tmp * 9.0) -
                  b_A_wayp * l15_tmp * 3.0) -
                 l5_tmp * t1[1].im * 6.0) +
                b_A_init_tmp * l15_tmp * 3.0) +
               l4 * l15_tmp * 3.0) -
              l20_tmp * t1[1].im * 6.0) +
             l6 * t1[1].im * 6.0;
  l20.re = b_J_max_tmp * 3.0;
  P_init_re = V_init_tmp * 6.0;
  l15 = A_wayp * (1.0 / J_min);
  t[8].re = l15;
  t[8].im = 0.0;
  t[9].re = l15;
  t[9].im = 0.0;
  l5_tmp = ((((l20.re + J_max_tmp * l7[0].re * 3.0) - P_init_re) -
             l4_tmp * l7[0].re * 3.0) +
            A_init_tmp * t1[0].re * 6.0) -
           b_A_init_tmp * t1[0].re * 6.0;
  l4 = ((J_max_tmp * l7[0].im * 3.0 - l4_tmp * l7[0].im * 3.0) +
        A_init_tmp * t1[0].im * 6.0) -
       b_A_init_tmp * t1[0].im * 6.0;
  t[0] = t1[0];
  t[2].re = 0.0;
  t[2].im = 0.0;
  l15 = -(A_init + J_min * t1[0].re);
  l6 = -(J_min * t1[0].im);
  if (l6 == 0.0) {
    t[4].re = l15 / J_max;
    t[4].im = 0.0;
  } else if (l15 == 0.0) {
    t[4].re = 0.0;
    t[4].im = l6 / J_max;
  } else {
    t[4].re = l15 / J_max;
    t[4].im = l6 / J_max;
  }
  if (l4 == 0.0) {
    if (l8[0].im == 0.0) {
      t[6].re = l8[0].re / l5_tmp;
      t[6].im = 0.0;
    } else if (l8[0].re == 0.0) {
      t[6].re = 0.0;
      t[6].im = l8[0].im / l5_tmp;
    } else {
      t[6].re = l8[0].re / l5_tmp;
      t[6].im = l8[0].im / l5_tmp;
    }
  } else if (l5_tmp == 0.0) {
    if (l8[0].re == 0.0) {
      t[6].re = l8[0].im / l4;
      t[6].im = 0.0;
    } else if (l8[0].im == 0.0) {
      t[6].re = 0.0;
      t[6].im = -(l8[0].re / l4);
    } else {
      t[6].re = l8[0].im / l4;
      t[6].im = -(l8[0].re / l4);
    }
  } else {
    l20_tmp = std::abs(l5_tmp);
    l15 = std::abs(l4);
    if (l20_tmp > l15) {
      l6 = l4 / l5_tmp;
      l15 = l5_tmp + l6 * l4;
      t[6].re = (l8[0].re + l6 * l8[0].im) / l15;
      t[6].im = (l8[0].im - l6 * l8[0].re) / l15;
    } else if (l15 == l20_tmp) {
      if (l5_tmp > 0.0) {
        l6 = 0.5;
      } else {
        l6 = -0.5;
      }
      if (l4 > 0.0) {
        l15 = 0.5;
      } else {
        l15 = -0.5;
      }
      t[6].re = (l8[0].re * l6 + l8[0].im * l15) / l20_tmp;
      t[6].im = (l8[0].im * l6 - l8[0].re * l15) / l20_tmp;
    } else {
      l6 = l5_tmp / l4;
      l15 = l4 + l6 * l5_tmp;
      t[6].re = (l6 * l8[0].re + l8[0].im) / l15;
      t[6].im = (l6 * l8[0].im - l8[0].re) / l15;
    }
  }
  t[10].re = 0.0;
  t[10].im = 0.0;
  t[12].re = 0.0;
  t[12].im = 0.0;
  l5_tmp =
      ((((l20.re + J_max_tmp * re * 3.0) - P_init_re) - l4_tmp * re * 3.0) +
       A_init_tmp * t1[1].re * 6.0) -
      b_A_init_tmp * t1[1].re * 6.0;
  l4 = ((J_max_tmp * l15_tmp * 3.0 - l4_tmp * l15_tmp * 3.0) +
        A_init_tmp * t1[1].im * 6.0) -
       b_A_init_tmp * t1[1].im * 6.0;
  t[1] = t1[1];
  t[3].re = 0.0;
  t[3].im = 0.0;
  l15 = -(A_init + J_min * t1[1].re);
  l6 = -(J_min * t1[1].im);
  if (l6 == 0.0) {
    t[5].re = l15 / J_max;
    t[5].im = 0.0;
  } else if (l15 == 0.0) {
    t[5].re = 0.0;
    t[5].im = l6 / J_max;
  } else {
    t[5].re = l15 / J_max;
    t[5].im = l6 / J_max;
  }
  if (l4 == 0.0) {
    if (l8[1].im == 0.0) {
      t[7].re = l8[1].re / l5_tmp;
      t[7].im = 0.0;
    } else if (l8[1].re == 0.0) {
      t[7].re = 0.0;
      t[7].im = l8[1].im / l5_tmp;
    } else {
      t[7].re = l8[1].re / l5_tmp;
      t[7].im = l8[1].im / l5_tmp;
    }
  } else if (l5_tmp == 0.0) {
    if (l8[1].re == 0.0) {
      t[7].re = l8[1].im / l4;
      t[7].im = 0.0;
    } else if (l8[1].im == 0.0) {
      t[7].re = 0.0;
      t[7].im = -(l8[1].re / l4);
    } else {
      t[7].re = l8[1].im / l4;
      t[7].im = -(l8[1].re / l4);
    }
  } else {
    l20_tmp = std::abs(l5_tmp);
    l15 = std::abs(l4);
    if (l20_tmp > l15) {
      l6 = l4 / l5_tmp;
      l15 = l5_tmp + l6 * l4;
      t[7].re = (l8[1].re + l6 * l8[1].im) / l15;
      t[7].im = (l8[1].im - l6 * l8[1].re) / l15;
    } else if (l15 == l20_tmp) {
      if (l5_tmp > 0.0) {
        l6 = 0.5;
      } else {
        l6 = -0.5;
      }
      if (l4 > 0.0) {
        l15 = 0.5;
      } else {
        l15 = -0.5;
      }
      t[7].re = (l8[1].re * l6 + l8[1].im * l15) / l20_tmp;
      t[7].im = (l8[1].im * l6 - l8[1].re * l15) / l20_tmp;
    } else {
      l6 = l5_tmp / l4;
      l15 = l4 + l6 * l5_tmp;
      t[7].re = (l6 * l8[1].re + l8[1].im) / l15;
      t[7].im = (l6 * l8[1].im - l8[1].re) / l15;
    }
  }
  t[11].re = 0.0;
  t[11].im = 0.0;
  t[13].re = 0.0;
  t[13].im = 0.0;
}

// End of code generation (acde_NO_AP.cpp)
