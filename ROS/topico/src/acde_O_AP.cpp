//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// acde_O_AP.cpp
//
// Code generation for function 'acde_O_AP'
//

// Include files
#include "acde_O_AP.h"
#include "power.h"
#include "rt_nonfinite.h"
#include "sqrt.h"
#include "topico_wrapper_data.h"
#include "topico_wrapper_rtwutil.h"
#include <cmath>

// Function Definitions
void acde_O_AP(double P_init, double V_init, double A_init, double P_wayp,
               double A_wayp, double V_max, double J_max, double J_min,
               creal_T t[14])
{
  creal_T b_l5[2];
  creal_T l6[2];
  creal_T t1[2];
  creal_T l17;
  double A_init_re;
  double A_init_tmp;
  double A_wayp_re;
  double J_max_tmp;
  double J_min_tmp;
  double b_A_init;
  double b_A_init_tmp;
  double b_A_wayp;
  double b_J_max;
  double b_J_min_tmp;
  double b_l14_tmp;
  double b_y;
  double c_A_init;
  double l14;
  double l14_tmp;
  double l17_tmp;
  double l4_tmp;
  double l5;
  double re;
  double y;
  double y_re;
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
  l4_tmp = A_init * J_min;
  l5 = A_init * J_max;
  l14_tmp = J_max * J_max;
  b_l14_tmp = J_min * J_max;
  l14 = 1.0 / (l14_tmp + -b_l14_tmp);
  l17_tmp = A_init * A_init;
  l17.re = J_min * (J_min + -J_max) *
           ((l17_tmp + J_max * V_max * 2.0) + -(J_max * V_init * 2.0));
  l17.im = 0.0;
  coder::internal::scalar::b_sqrt(&l17);
  t1[0].re = l14 * ((l4_tmp - l5) + l17.re);
  t1[0].im = l14 * l17.im;
  t1[1].re = -l14 * ((-l4_tmp + l5) + l17.re);
  t1[1].im = -l14 * l17.im;
  l5 = J_min * J_min;
  coder::power(t1, l6);
  y = rt_powd_snf(J_max, 3.0);
  b_y = rt_powd_snf(A_wayp, 3.0);
  l17.re = (A_wayp * l17_tmp * -3.0 + P_init * l5 * 6.0) - P_wayp * l5 * 6.0;
  y_re = rt_powd_snf(A_init, 3.0) * 2.0;
  A_init_re = l4_tmp * V_init * 6.0;
  l14 = A_wayp * J_min;
  A_wayp_re = l14 * V_init * 6.0;
  A_init_tmp = A_init * l5;
  b_A_init = A_init * l14_tmp;
  b_A_wayp = A_wayp * l14_tmp;
  J_min_tmp = J_min * l14_tmp;
  J_max_tmp = J_max * l5;
  b_J_min_tmp = J_min * l17_tmp;
  b_J_max = J_max * l17_tmp;
  l14_tmp = V_init * l5;
  b_A_init_tmp = A_init * A_wayp;
  l17_tmp = b_A_init_tmp * J_min;
  c_A_init = b_A_init_tmp * J_max;
  b_A_init_tmp = l4_tmp * J_max;
  l4_tmp = l14 * J_max;
  l14 = b_l14_tmp * V_init;
  re = t1[0].re * t1[0].re - t1[0].im * t1[0].im;
  l5 = t1[0].re * t1[0].im;
  b_l14_tmp = l5 + l5;
  b_l5[0].re = re;
  b_l5[0].im = b_l14_tmp;
  l6[0].re = (((((((((((((((((l17.re + y * l6[0].re * 2.0) + y_re) + b_y) -
                           A_init_re) +
                          A_wayp_re) +
                         A_init_tmp * re * 3.0) +
                        b_A_init * re * 6.0) -
                       b_A_wayp * re * 3.0) -
                      J_min_tmp * l6[0].re * 3.0) +
                     J_max_tmp * l6[0].re) -
                    b_J_min_tmp * t1[0].re * 6.0) +
                   b_J_max * t1[0].re * 6.0) +
                  l14_tmp * t1[0].re * 6.0) +
                 l17_tmp * t1[0].re * 6.0) -
                c_A_init * t1[0].re * 6.0) -
               b_A_init_tmp * re * 9.0) +
              l4_tmp * re * 3.0) -
             l14 * t1[0].re * 6.0;
  l6[0].im = ((((((((((((y * l6[0].im * 2.0 + A_init_tmp * b_l14_tmp * 3.0) +
                        b_A_init * b_l14_tmp * 6.0) -
                       b_A_wayp * b_l14_tmp * 3.0) -
                      J_min_tmp * l6[0].im * 3.0) +
                     J_max_tmp * l6[0].im) -
                    b_J_min_tmp * t1[0].im * 6.0) +
                   b_J_max * t1[0].im * 6.0) +
                  l14_tmp * t1[0].im * 6.0) +
                 l17_tmp * t1[0].im * 6.0) -
                c_A_init * t1[0].im * 6.0) -
               b_A_init_tmp * b_l14_tmp * 9.0) +
              l4_tmp * b_l14_tmp * 3.0) -
             l14 * t1[0].im * 6.0;
  re = t1[1].re * t1[1].re - t1[1].im * t1[1].im;
  l5 = t1[1].re * t1[1].im;
  b_l14_tmp = l5 + l5;
  l6[1].re = (((((((((((((((((l17.re + y * l6[1].re * 2.0) + y_re) + b_y) -
                           A_init_re) +
                          A_wayp_re) +
                         A_init_tmp * re * 3.0) +
                        b_A_init * re * 6.0) -
                       b_A_wayp * re * 3.0) -
                      J_min_tmp * l6[1].re * 3.0) +
                     J_max_tmp * l6[1].re) -
                    b_J_min_tmp * t1[1].re * 6.0) +
                   b_J_max * t1[1].re * 6.0) +
                  l14_tmp * t1[1].re * 6.0) +
                 l17_tmp * t1[1].re * 6.0) -
                c_A_init * t1[1].re * 6.0) -
               b_A_init_tmp * re * 9.0) +
              l4_tmp * re * 3.0) -
             l14 * t1[1].re * 6.0;
  l6[1].im = ((((((((((((y * l6[1].im * 2.0 + A_init_tmp * b_l14_tmp * 3.0) +
                        b_A_init * b_l14_tmp * 6.0) -
                       b_A_wayp * b_l14_tmp * 3.0) -
                      J_min_tmp * l6[1].im * 3.0) +
                     J_max_tmp * l6[1].im) -
                    b_J_min_tmp * t1[1].im * 6.0) +
                   b_J_max * t1[1].im * 6.0) +
                  l14_tmp * t1[1].im * 6.0) +
                 l17_tmp * t1[1].im * 6.0) -
                c_A_init * t1[1].im * 6.0) -
               b_A_init_tmp * b_l14_tmp * 9.0) +
              l4_tmp * b_l14_tmp * 3.0) -
             l14 * t1[1].im * 6.0;
  l17.re = b_J_min_tmp * 3.0 - l14_tmp * 6.0;
  l5 = A_wayp * (1.0 / J_min);
  t[8].re = l5;
  t[8].im = 0.0;
  t[9].re = l5;
  t[9].im = 0.0;
  l17_tmp =
      (((l17.re - A_init_tmp * t1[0].re * 6.0) + J_min_tmp * b_l5[0].re * 3.0) -
       J_max_tmp * b_l5[0].re * 3.0) +
      b_A_init_tmp * t1[0].re * 6.0;
  l4_tmp =
      (((0.0 - A_init_tmp * t1[0].im * 6.0) + J_min_tmp * b_l5[0].im * 3.0) -
       J_max_tmp * b_l5[0].im * 3.0) +
      b_A_init_tmp * t1[0].im * 6.0;
  t[0] = t1[0];
  t[2].re = 0.0;
  t[2].im = 0.0;
  l5 = -(A_init + J_max * t1[0].re);
  l14 = -(J_max * t1[0].im);
  if (l14 == 0.0) {
    t[4].re = l5 / J_min;
    t[4].im = 0.0;
  } else if (l5 == 0.0) {
    t[4].re = 0.0;
    t[4].im = l14 / J_min;
  } else {
    t[4].re = l5 / J_min;
    t[4].im = l14 / J_min;
  }
  if (l4_tmp == 0.0) {
    if (l6[0].im == 0.0) {
      t[6].re = l6[0].re / l17_tmp;
      t[6].im = 0.0;
    } else if (l6[0].re == 0.0) {
      t[6].re = 0.0;
      t[6].im = l6[0].im / l17_tmp;
    } else {
      t[6].re = l6[0].re / l17_tmp;
      t[6].im = l6[0].im / l17_tmp;
    }
  } else if (l17_tmp == 0.0) {
    if (l6[0].re == 0.0) {
      t[6].re = l6[0].im / l4_tmp;
      t[6].im = 0.0;
    } else if (l6[0].im == 0.0) {
      t[6].re = 0.0;
      t[6].im = -(l6[0].re / l4_tmp);
    } else {
      t[6].re = l6[0].im / l4_tmp;
      t[6].im = -(l6[0].re / l4_tmp);
    }
  } else {
    l14_tmp = std::abs(l17_tmp);
    l5 = std::abs(l4_tmp);
    if (l14_tmp > l5) {
      l14 = l4_tmp / l17_tmp;
      l5 = l17_tmp + l14 * l4_tmp;
      t[6].re = (l6[0].re + l14 * l6[0].im) / l5;
      t[6].im = (l6[0].im - l14 * l6[0].re) / l5;
    } else if (l5 == l14_tmp) {
      if (l17_tmp > 0.0) {
        l14 = 0.5;
      } else {
        l14 = -0.5;
      }
      if (l4_tmp > 0.0) {
        l5 = 0.5;
      } else {
        l5 = -0.5;
      }
      t[6].re = (l6[0].re * l14 + l6[0].im * l5) / l14_tmp;
      t[6].im = (l6[0].im * l14 - l6[0].re * l5) / l14_tmp;
    } else {
      l14 = l17_tmp / l4_tmp;
      l5 = l4_tmp + l14 * l17_tmp;
      t[6].re = (l14 * l6[0].re + l6[0].im) / l5;
      t[6].im = (l14 * l6[0].im - l6[0].re) / l5;
    }
  }
  t[10].re = 0.0;
  t[10].im = 0.0;
  t[12].re = 0.0;
  t[12].im = 0.0;
  l17_tmp = (((l17.re - A_init_tmp * t1[1].re * 6.0) + J_min_tmp * re * 3.0) -
             J_max_tmp * re * 3.0) +
            b_A_init_tmp * t1[1].re * 6.0;
  l4_tmp =
      (((0.0 - A_init_tmp * t1[1].im * 6.0) + J_min_tmp * b_l14_tmp * 3.0) -
       J_max_tmp * b_l14_tmp * 3.0) +
      b_A_init_tmp * t1[1].im * 6.0;
  t[1] = t1[1];
  t[3].re = 0.0;
  t[3].im = 0.0;
  l5 = -(A_init + J_max * t1[1].re);
  l14 = -(J_max * t1[1].im);
  if (l14 == 0.0) {
    t[5].re = l5 / J_min;
    t[5].im = 0.0;
  } else if (l5 == 0.0) {
    t[5].re = 0.0;
    t[5].im = l14 / J_min;
  } else {
    t[5].re = l5 / J_min;
    t[5].im = l14 / J_min;
  }
  if (l4_tmp == 0.0) {
    if (l6[1].im == 0.0) {
      t[7].re = l6[1].re / l17_tmp;
      t[7].im = 0.0;
    } else if (l6[1].re == 0.0) {
      t[7].re = 0.0;
      t[7].im = l6[1].im / l17_tmp;
    } else {
      t[7].re = l6[1].re / l17_tmp;
      t[7].im = l6[1].im / l17_tmp;
    }
  } else if (l17_tmp == 0.0) {
    if (l6[1].re == 0.0) {
      t[7].re = l6[1].im / l4_tmp;
      t[7].im = 0.0;
    } else if (l6[1].im == 0.0) {
      t[7].re = 0.0;
      t[7].im = -(l6[1].re / l4_tmp);
    } else {
      t[7].re = l6[1].im / l4_tmp;
      t[7].im = -(l6[1].re / l4_tmp);
    }
  } else {
    l14_tmp = std::abs(l17_tmp);
    l5 = std::abs(l4_tmp);
    if (l14_tmp > l5) {
      l14 = l4_tmp / l17_tmp;
      l5 = l17_tmp + l14 * l4_tmp;
      t[7].re = (l6[1].re + l14 * l6[1].im) / l5;
      t[7].im = (l6[1].im - l14 * l6[1].re) / l5;
    } else if (l5 == l14_tmp) {
      if (l17_tmp > 0.0) {
        l14 = 0.5;
      } else {
        l14 = -0.5;
      }
      if (l4_tmp > 0.0) {
        l5 = 0.5;
      } else {
        l5 = -0.5;
      }
      t[7].re = (l6[1].re * l14 + l6[1].im * l5) / l14_tmp;
      t[7].im = (l6[1].im * l14 - l6[1].re * l5) / l14_tmp;
    } else {
      l14 = l17_tmp / l4_tmp;
      l5 = l4_tmp + l14 * l17_tmp;
      t[7].re = (l14 * l6[1].re + l6[1].im) / l5;
      t[7].im = (l14 * l6[1].im - l6[1].re) / l5;
    }
  }
  t[11].re = 0.0;
  t[11].im = 0.0;
  t[13].re = 0.0;
  t[13].im = 0.0;
}

// End of code generation (acde_O_AP.cpp)
