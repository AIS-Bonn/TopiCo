//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// acd_NO_P.cpp
//
// Code generation for function 'acd_NO_P'
//

// Include files
#include "acd_NO_P.h"
#include "power.h"
#include "rt_nonfinite.h"
#include "sqrt.h"
#include "topico_wrapper_data.h"
#include "topico_wrapper_rtwutil.h"
#include "topico_wrapper_types.h"
#include <cmath>

// Function Definitions
void acd_NO_P(double P_init, double V_init, double A_init, double P_wayp,
              double V_max, double J_max, double J_min, creal_T t[14])
{
  creal_T l6[2];
  creal_T t1[2];
  creal_T l20;
  double A_init_re;
  double A_init_tmp;
  double J_max_re;
  double J_max_tmp;
  double J_min_tmp;
  double V_init_tmp;
  double b_A_init;
  double b_A_init_tmp;
  double b_J_max_tmp;
  double b_J_min;
  double b_l15_tmp;
  double b_l20_tmp;
  double c_J_min;
  double l15;
  double l15_tmp;
  double l20_tmp;
  double l4_tmp;
  double l5_tmp;
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
  l5_tmp = A_init * J_max;
  if (J_max < 0.0) {
    f_rtErrorWithMessageID("sqrt", n_emlrtRTEI.fName, n_emlrtRTEI.lineNo);
  }
  l15_tmp = J_min * J_min;
  b_l15_tmp = J_min * J_max;
  l15 = 1.0 / (l15_tmp + -b_l15_tmp);
  l20_tmp = A_init * A_init;
  l20.re = -((J_min + -J_max) *
             ((l20_tmp + J_min * V_max * 2.0) + -(J_min * V_init * 2.0)));
  l20.im = 0.0;
  coder::internal::scalar::b_sqrt(&l20);
  b_l20_tmp = std::sqrt(J_max);
  l20.re *= b_l20_tmp;
  l20.im *= b_l20_tmp;
  t1[0].re = l15 * ((-l4_tmp + l5_tmp) + l20.re);
  t1[0].im = l15 * l20.im;
  t1[1].re = -l15 * ((l4_tmp - l5_tmp) + l20.re);
  t1[1].im = -l15 * l20.im;
  l15 = J_max * J_max;
  coder::power(t1, l6);
  y = rt_powd_snf(J_min, 3.0);
  l20.re = P_init * l15 * 6.0 - P_wayp * l15 * 6.0;
  y_re = rt_powd_snf(A_init, 3.0) * 2.0;
  A_init_re = l5_tmp * V_init * 6.0;
  b_A_init = A_init * l15_tmp;
  A_init_tmp = A_init * l15;
  J_min_tmp = J_min * l15;
  J_max_tmp = J_max * l15_tmp;
  b_J_min = J_min * l20_tmp;
  b_J_max_tmp = J_max * l20_tmp;
  V_init_tmp = V_init * l15;
  b_A_init_tmp = l4_tmp * J_max;
  c_J_min = b_l15_tmp * V_init;
  J_max_re = b_J_max_tmp * 3.0 - V_init_tmp * 6.0;
  b_l20_tmp = t1[0].re * t1[0].re - t1[0].im * t1[0].im;
  l15 = t1[0].re * t1[0].im;
  l15 += l15;
  t[0] = t1[0];
  t[2].re = 0.0;
  t[2].im = 0.0;
  l4_tmp = -(A_init + J_min * t1[0].re);
  b_l15_tmp = -(J_min * t1[0].im);
  if (b_l15_tmp == 0.0) {
    t[4].re = l4_tmp / J_max;
    t[4].im = 0.0;
  } else if (l4_tmp == 0.0) {
    t[4].re = 0.0;
    t[4].im = b_l15_tmp / J_max;
  } else {
    t[4].re = l4_tmp / J_max;
    t[4].im = b_l15_tmp / J_max;
  }
  l4_tmp = (((((((((((l20.re + y * l6[0].re * 2.0) + y_re) - A_init_re) +
                   b_A_init * b_l20_tmp * 6.0) +
                  A_init_tmp * b_l20_tmp * 3.0) +
                 J_min_tmp * l6[0].re) -
                J_max_tmp * l6[0].re * 3.0) +
               b_J_min * t1[0].re * 6.0) -
              b_J_max_tmp * t1[0].re * 6.0) +
             V_init_tmp * t1[0].re * 6.0) -
            b_A_init_tmp * b_l20_tmp * 9.0) -
           c_J_min * t1[0].re * 6.0;
  b_l15_tmp = ((((((((y * l6[0].im * 2.0 + b_A_init * l15 * 6.0) +
                     A_init_tmp * l15 * 3.0) +
                    J_min_tmp * l6[0].im) -
                   J_max_tmp * l6[0].im * 3.0) +
                  b_J_min * t1[0].im * 6.0) -
                 b_J_max_tmp * t1[0].im * 6.0) +
                V_init_tmp * t1[0].im * 6.0) -
               b_A_init_tmp * l15 * 9.0) -
              c_J_min * t1[0].im * 6.0;
  l5_tmp = (((J_max_re - A_init_tmp * t1[0].re * 6.0) -
             J_min_tmp * b_l20_tmp * 3.0) +
            J_max_tmp * b_l20_tmp * 3.0) +
           b_A_init_tmp * t1[0].re * 6.0;
  l15_tmp = (((0.0 - A_init_tmp * t1[0].im * 6.0) - J_min_tmp * l15 * 3.0) +
             J_max_tmp * l15 * 3.0) +
            b_A_init_tmp * t1[0].im * 6.0;
  if (l15_tmp == 0.0) {
    if (b_l15_tmp == 0.0) {
      t[6].re = l4_tmp / l5_tmp;
      t[6].im = 0.0;
    } else if (l4_tmp == 0.0) {
      t[6].re = 0.0;
      t[6].im = b_l15_tmp / l5_tmp;
    } else {
      t[6].re = l4_tmp / l5_tmp;
      t[6].im = b_l15_tmp / l5_tmp;
    }
  } else if (l5_tmp == 0.0) {
    if (l4_tmp == 0.0) {
      t[6].re = b_l15_tmp / l15_tmp;
      t[6].im = 0.0;
    } else if (b_l15_tmp == 0.0) {
      t[6].re = 0.0;
      t[6].im = -(l4_tmp / l15_tmp);
    } else {
      t[6].re = b_l15_tmp / l15_tmp;
      t[6].im = -(l4_tmp / l15_tmp);
    }
  } else {
    l20_tmp = std::abs(l5_tmp);
    l15 = std::abs(l15_tmp);
    if (l20_tmp > l15) {
      b_l20_tmp = l15_tmp / l5_tmp;
      l15 = l5_tmp + b_l20_tmp * l15_tmp;
      t[6].re = (l4_tmp + b_l20_tmp * b_l15_tmp) / l15;
      t[6].im = (b_l15_tmp - b_l20_tmp * l4_tmp) / l15;
    } else if (l15 == l20_tmp) {
      if (l5_tmp > 0.0) {
        b_l20_tmp = 0.5;
      } else {
        b_l20_tmp = -0.5;
      }
      if (l15_tmp > 0.0) {
        l15 = 0.5;
      } else {
        l15 = -0.5;
      }
      t[6].re = (l4_tmp * b_l20_tmp + b_l15_tmp * l15) / l20_tmp;
      t[6].im = (b_l15_tmp * b_l20_tmp - l4_tmp * l15) / l20_tmp;
    } else {
      b_l20_tmp = l5_tmp / l15_tmp;
      l15 = l15_tmp + b_l20_tmp * l5_tmp;
      t[6].re = (b_l20_tmp * l4_tmp + b_l15_tmp) / l15;
      t[6].im = (b_l20_tmp * b_l15_tmp - l4_tmp) / l15;
    }
  }
  t[8].re = 0.0;
  t[8].im = 0.0;
  t[10].re = 0.0;
  t[10].im = 0.0;
  t[12].re = 0.0;
  t[12].im = 0.0;
  b_l20_tmp = t1[1].re * t1[1].re - t1[1].im * t1[1].im;
  l15 = t1[1].re * t1[1].im;
  l15 += l15;
  t[1] = t1[1];
  t[3].re = 0.0;
  t[3].im = 0.0;
  l4_tmp = -(A_init + J_min * t1[1].re);
  b_l15_tmp = -(J_min * t1[1].im);
  if (b_l15_tmp == 0.0) {
    t[5].re = l4_tmp / J_max;
    t[5].im = 0.0;
  } else if (l4_tmp == 0.0) {
    t[5].re = 0.0;
    t[5].im = b_l15_tmp / J_max;
  } else {
    t[5].re = l4_tmp / J_max;
    t[5].im = b_l15_tmp / J_max;
  }
  l4_tmp = (((((((((((l20.re + y * l6[1].re * 2.0) + y_re) - A_init_re) +
                   b_A_init * b_l20_tmp * 6.0) +
                  A_init_tmp * b_l20_tmp * 3.0) +
                 J_min_tmp * l6[1].re) -
                J_max_tmp * l6[1].re * 3.0) +
               b_J_min * t1[1].re * 6.0) -
              b_J_max_tmp * t1[1].re * 6.0) +
             V_init_tmp * t1[1].re * 6.0) -
            b_A_init_tmp * b_l20_tmp * 9.0) -
           c_J_min * t1[1].re * 6.0;
  b_l15_tmp = ((((((((y * l6[1].im * 2.0 + b_A_init * l15 * 6.0) +
                     A_init_tmp * l15 * 3.0) +
                    J_min_tmp * l6[1].im) -
                   J_max_tmp * l6[1].im * 3.0) +
                  b_J_min * t1[1].im * 6.0) -
                 b_J_max_tmp * t1[1].im * 6.0) +
                V_init_tmp * t1[1].im * 6.0) -
               b_A_init_tmp * l15 * 9.0) -
              c_J_min * t1[1].im * 6.0;
  l5_tmp = (((J_max_re - A_init_tmp * t1[1].re * 6.0) -
             J_min_tmp * b_l20_tmp * 3.0) +
            J_max_tmp * b_l20_tmp * 3.0) +
           b_A_init_tmp * t1[1].re * 6.0;
  l15_tmp = (((0.0 - A_init_tmp * t1[1].im * 6.0) - J_min_tmp * l15 * 3.0) +
             J_max_tmp * l15 * 3.0) +
            b_A_init_tmp * t1[1].im * 6.0;
  if (l15_tmp == 0.0) {
    if (b_l15_tmp == 0.0) {
      t[7].re = l4_tmp / l5_tmp;
      t[7].im = 0.0;
    } else if (l4_tmp == 0.0) {
      t[7].re = 0.0;
      t[7].im = b_l15_tmp / l5_tmp;
    } else {
      t[7].re = l4_tmp / l5_tmp;
      t[7].im = b_l15_tmp / l5_tmp;
    }
  } else if (l5_tmp == 0.0) {
    if (l4_tmp == 0.0) {
      t[7].re = b_l15_tmp / l15_tmp;
      t[7].im = 0.0;
    } else if (b_l15_tmp == 0.0) {
      t[7].re = 0.0;
      t[7].im = -(l4_tmp / l15_tmp);
    } else {
      t[7].re = b_l15_tmp / l15_tmp;
      t[7].im = -(l4_tmp / l15_tmp);
    }
  } else {
    l20_tmp = std::abs(l5_tmp);
    l15 = std::abs(l15_tmp);
    if (l20_tmp > l15) {
      b_l20_tmp = l15_tmp / l5_tmp;
      l15 = l5_tmp + b_l20_tmp * l15_tmp;
      t[7].re = (l4_tmp + b_l20_tmp * b_l15_tmp) / l15;
      t[7].im = (b_l15_tmp - b_l20_tmp * l4_tmp) / l15;
    } else if (l15 == l20_tmp) {
      if (l5_tmp > 0.0) {
        b_l20_tmp = 0.5;
      } else {
        b_l20_tmp = -0.5;
      }
      if (l15_tmp > 0.0) {
        l15 = 0.5;
      } else {
        l15 = -0.5;
      }
      t[7].re = (l4_tmp * b_l20_tmp + b_l15_tmp * l15) / l20_tmp;
      t[7].im = (b_l15_tmp * b_l20_tmp - l4_tmp * l15) / l20_tmp;
    } else {
      b_l20_tmp = l5_tmp / l15_tmp;
      l15 = l15_tmp + b_l20_tmp * l5_tmp;
      t[7].re = (b_l20_tmp * l4_tmp + b_l15_tmp) / l15;
      t[7].im = (b_l20_tmp * b_l15_tmp - l4_tmp) / l15;
    }
  }
  t[9].re = 0.0;
  t[9].im = 0.0;
  t[11].re = 0.0;
  t[11].im = 0.0;
  t[13].re = 0.0;
  t[13].im = 0.0;
}

// End of code generation (acd_NO_P.cpp)
