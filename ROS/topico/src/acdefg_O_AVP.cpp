//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// acdefg_O_AVP.cpp
//
// Code generation for function 'acdefg_O_AVP'
//

// Include files
#include "acdefg_O_AVP.h"
#include "power.h"
#include "rt_nonfinite.h"
#include "sqrt.h"
#include "topico_wrapper_data.h"
#include "topico_wrapper_rtwutil.h"
#include <cmath>

// Function Definitions
void acdefg_O_AVP(double P_init, double V_init, double A_init, double P_wayp,
                  double V_wayp, double A_wayp, double V_max, double A_min,
                  double J_max, double J_min, creal_T t[14])
{
  creal_T l11[2];
  creal_T l12[2];
  creal_T t3[2];
  creal_T l16;
  double A_init_re;
  double A_min_re;
  double A_min_tmp;
  double J_max_re;
  double J_max_re_tmp;
  double J_max_tmp;
  double J_min_re;
  double b_A_min;
  double b_A_min_re;
  double b_J_max;
  double b_J_max_re;
  double b_J_max_re_tmp;
  double b_J_min_re;
  double b_V_init;
  double b_im;
  double b_l16_tmp;
  double b_l16_tmp_tmp;
  double b_l3;
  double b_l8;
  double b_re;
  double b_y;
  double b_y_re;
  double c_A_min;
  double c_A_min_re;
  double c_J_max;
  double c_J_max_re;
  double c_J_max_re_tmp;
  double c_l16_tmp;
  double c_y_re;
  double d_A_min;
  double d_A_min_re;
  double d_J_max;
  double e_A_min;
  double e_J_max;
  double f_A_min;
  double g_A_min;
  double h_A_min;
  double im;
  double l10;
  double l16_tmp;
  double l16_tmp_tmp;
  double l2;
  double l3;
  double l3_re;
  double l4;
  double l4_tmp;
  double l5;
  double l7;
  double l8;
  double l9;
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
  //  Generated on 28-Aug-2019 12:21:18
  l16_tmp = A_init * A_init;
  b_l16_tmp = J_max * V_init;
  c_l16_tmp = b_l16_tmp * 2.0;
  l16.re =
      J_min * (J_min + -J_max) * ((l16_tmp + J_max * V_max * 2.0) + -c_l16_tmp);
  l16.im = 0.0;
  coder::internal::scalar::b_sqrt(&l16);
  l16_tmp_tmp = J_min * J_min;
  b_l16_tmp_tmp = J_min * J_max;
  l4 = 1.0 / (l16_tmp_tmp + -b_l16_tmp_tmp);
  l16.re *= l4;
  l16.im *= l4;
  t3[0] = l16;
  t3[1].re = -l16.re;
  t3[1].im = -l16.im;
  l3 = A_min * A_min;
  l5 = A_wayp * A_wayp;
  l7 = rt_powd_snf(J_min, 3.0);
  l9 = J_max * J_max;
  l10 = rt_powd_snf(J_min, 5.0);
  coder::power(t3, l12);
  l4 = l3 * l3;
  l8 = l16_tmp_tmp * l16_tmp_tmp;
  y = rt_powd_snf(l16_tmp_tmp, 3.0);
  l16.re =
      ((l4 * l16_tmp_tmp - l4 * l9) + l16_tmp * l16_tmp * l16_tmp_tmp * 3.0) -
      l5 * l5 * l16_tmp_tmp * 3.0;
  b_A_min = A_min * l10;
  b_J_max = J_max * l10;
  y_re = rt_powd_snf(A_init, 3.0) * A_min * l16_tmp_tmp * 8.0;
  A_min_re = A_min * rt_powd_snf(A_wayp, 3.0) * l16_tmp_tmp * 8.0;
  l3_re = l3 * l5 * l16_tmp_tmp * 6.0;
  l2 = l16_tmp * l8;
  b_l8 = l8 * l9;
  b_y_re = V_init * V_init * l16_tmp_tmp * l9 * 12.0;
  c_y_re = V_wayp * V_wayp * l16_tmp_tmp * l9 * 12.0;
  J_max_tmp = J_max * l16_tmp;
  c_J_max = J_max_tmp * l7;
  d_J_max = J_max * l3 * l7;
  b_V_init = V_init * l7 * l9;
  b_l3 = l3 * l16_tmp_tmp * l9;
  A_min_tmp = A_min * J_max;
  c_A_min = A_min_tmp * l8;
  b_A_min_re = A_min * P_init * l16_tmp_tmp * l9 * 24.0;
  c_A_min_re = A_min * P_wayp * l16_tmp_tmp * l9 * 24.0;
  J_min_re = b_l16_tmp_tmp * l16_tmp * l3 * 6.0;
  J_max_re = b_l16_tmp * l16_tmp * l16_tmp_tmp * 12.0;
  b_J_min_re = J_min * V_init * l3 * l9 * 12.0;
  e_J_max = b_l16_tmp * l8;
  J_max_re_tmp = J_max * V_wayp;
  b_J_max_re = J_max_re_tmp * l3 * l16_tmp_tmp * 12.0;
  c_J_max_re = J_max_re_tmp * l5 * l16_tmp_tmp * 12.0;
  d_A_min = A_min * l7 * l9;
  e_A_min = A_min * l16_tmp * l7;
  f_A_min = A_min_tmp * V_init * l7;
  g_A_min = A_min_tmp * l16_tmp * l16_tmp_tmp;
  h_A_min = A_min * V_init * l16_tmp_tmp * l9;
  A_init_re = A_init * A_min * J_max * V_init * l16_tmp_tmp * 24.0;
  d_A_min_re = A_min * A_wayp * J_max * V_wayp * l16_tmp_tmp * 24.0;
  l4_tmp = A_min + -A_wayp;
  b_y = l4_tmp * l4_tmp;
  l3 = A_min * (1.0 / J_min);
  re = t3[0].re * t3[0].re - t3[0].im * t3[0].im;
  l4 = t3[0].re * t3[0].im;
  im = l4 + l4;
  l11[0].re = re;
  l11[0].im = im;
  b_re = re * re - im * im;
  l4 = re * im;
  b_im = l4 + l4;
  l8 = y * b_re;
  b_l16_tmp = y * b_im;
  l5 = b_J_max * b_re;
  l7 = b_J_max * b_im;
  l4 = b_l8 * b_re;
  l10 = b_l8 * b_im;
  b_re = -0.083333333333333329 *
         ((((((((((((((((((((((((((((((l16.re + l8 * 3.0) +
                                      b_A_min * l12[0].re * 4.0) -
                                     l5 * 6.0) -
                                    y_re) +
                                   A_min_re) -
                                  l3_re) -
                                 l2 * re * 6.0) +
                                l4 * 3.0) +
                               b_y_re) -
                              c_y_re) +
                             c_J_max * re * 6.0) -
                            d_J_max * re * 6.0) -
                           b_V_init * re * 12.0) +
                          b_l3 * re * 6.0) -
                         c_A_min * l12[0].re * 12.0) -
                        b_A_min_re) +
                       c_A_min_re) +
                      J_min_re) -
                     J_max_re) -
                    b_J_min_re) +
                   e_J_max * re * 12.0) +
                  b_J_max_re) +
                 c_J_max_re) +
                d_A_min * l12[0].re * 8.0) -
               e_A_min * t3[0].re * 12.0) +
              f_A_min * t3[0].re * 24.0) +
             g_A_min * t3[0].re * 12.0) -
            h_A_min * t3[0].re * 24.0) +
           A_init_re) -
          d_A_min_re);
  b_im =
      -0.083333333333333329 *
      (((((((((((((((b_l16_tmp * 3.0 + b_A_min * l12[0].im * 4.0) - l7 * 6.0) -
                   l2 * im * 6.0) +
                  l10 * 3.0) +
                 c_J_max * im * 6.0) -
                d_J_max * im * 6.0) -
               b_V_init * im * 12.0) +
              b_l3 * im * 6.0) -
             c_A_min * l12[0].im * 12.0) +
            e_J_max * im * 12.0) +
           d_A_min * l12[0].im * 8.0) -
          e_A_min * t3[0].im * 12.0) +
         f_A_min * t3[0].im * 24.0) +
        g_A_min * t3[0].im * 12.0) -
       h_A_min * t3[0].im * 24.0);
  b_J_max_re_tmp = J_max * l16_tmp_tmp;
  c_J_max_re_tmp = J_min * l9;
  J_max_tmp -= V_init * l9 * 2.0;
  l9 = A_min * l16_tmp_tmp;
  b_l16_tmp = l9 * ((J_max_tmp - b_J_max_re_tmp * re) + c_J_max_re_tmp * re);
  l5 = l9 * ((0.0 - b_J_max_re_tmp * im) + c_J_max_re_tmp * im);
  l10 = -(A_init + J_min * t3[0].re);
  l8 = -(J_min * t3[0].im);
  if (l8 == 0.0) {
    t[0].re = l10 / J_max;
    t[0].im = 0.0;
  } else if (l10 == 0.0) {
    t[0].re = 0.0;
    t[0].im = l8 / J_max;
  } else {
    t[0].re = l10 / J_max;
    t[0].im = l8 / J_max;
  }
  t[2].re = 0.0;
  t[2].im = 0.0;
  t[4] = t3[0];
  if (l5 == 0.0) {
    if (b_im == 0.0) {
      t[6].re = b_re / b_l16_tmp;
      t[6].im = 0.0;
    } else if (b_re == 0.0) {
      t[6].re = 0.0;
      t[6].im = b_im / b_l16_tmp;
    } else {
      t[6].re = b_re / b_l16_tmp;
      t[6].im = b_im / b_l16_tmp;
    }
  } else if (b_l16_tmp == 0.0) {
    if (b_re == 0.0) {
      t[6].re = b_im / l5;
      t[6].im = 0.0;
    } else if (b_im == 0.0) {
      t[6].re = 0.0;
      t[6].im = -(b_re / l5);
    } else {
      t[6].re = b_im / l5;
      t[6].im = -(b_re / l5);
    }
  } else {
    l8 = std::abs(b_l16_tmp);
    l4 = std::abs(l5);
    if (l8 > l4) {
      l10 = l5 / b_l16_tmp;
      l4 = b_l16_tmp + l10 * l5;
      t[6].re = (b_re + l10 * b_im) / l4;
      t[6].im = (b_im - l10 * b_re) / l4;
    } else if (l4 == l8) {
      if (b_l16_tmp > 0.0) {
        l10 = 0.5;
      } else {
        l10 = -0.5;
      }
      if (l5 > 0.0) {
        l4 = 0.5;
      } else {
        l4 = -0.5;
      }
      t[6].re = (b_re * l10 + b_im * l4) / l8;
      t[6].im = (b_im * l10 - b_re * l4) / l8;
    } else {
      l10 = b_l16_tmp / l5;
      l4 = l5 + l10 * b_l16_tmp;
      t[6].re = (l10 * b_re + b_im) / l4;
      t[6].im = (l10 * b_im - b_re) / l4;
    }
  }
  re = t3[1].re * t3[1].re - t3[1].im * t3[1].im;
  l4 = t3[1].re * t3[1].im;
  im = l4 + l4;
  b_re = re * re - im * im;
  l4 = re * im;
  b_im = l4 + l4;
  l8 = y * b_re;
  b_l16_tmp = y * b_im;
  l5 = b_J_max * b_re;
  l7 = b_J_max * b_im;
  l4 = b_l8 * b_re;
  l10 = b_l8 * b_im;
  b_re = -0.083333333333333329 *
         ((((((((((((((((((((((((((((((l16.re + l8 * 3.0) +
                                      b_A_min * l12[1].re * 4.0) -
                                     l5 * 6.0) -
                                    y_re) +
                                   A_min_re) -
                                  l3_re) -
                                 l2 * re * 6.0) +
                                l4 * 3.0) +
                               b_y_re) -
                              c_y_re) +
                             c_J_max * re * 6.0) -
                            d_J_max * re * 6.0) -
                           b_V_init * re * 12.0) +
                          b_l3 * re * 6.0) -
                         c_A_min * l12[1].re * 12.0) -
                        b_A_min_re) +
                       c_A_min_re) +
                      J_min_re) -
                     J_max_re) -
                    b_J_min_re) +
                   e_J_max * re * 12.0) +
                  b_J_max_re) +
                 c_J_max_re) +
                d_A_min * l12[1].re * 8.0) -
               e_A_min * t3[1].re * 12.0) +
              f_A_min * t3[1].re * 24.0) +
             g_A_min * t3[1].re * 12.0) -
            h_A_min * t3[1].re * 24.0) +
           A_init_re) -
          d_A_min_re);
  b_im =
      -0.083333333333333329 *
      (((((((((((((((b_l16_tmp * 3.0 + b_A_min * l12[1].im * 4.0) - l7 * 6.0) -
                   l2 * im * 6.0) +
                  l10 * 3.0) +
                 c_J_max * im * 6.0) -
                d_J_max * im * 6.0) -
               b_V_init * im * 12.0) +
              b_l3 * im * 6.0) -
             c_A_min * l12[1].im * 12.0) +
            e_J_max * im * 12.0) +
           d_A_min * l12[1].im * 8.0) -
          e_A_min * t3[1].im * 12.0) +
         f_A_min * t3[1].im * 24.0) +
        g_A_min * t3[1].im * 12.0) -
       h_A_min * t3[1].im * 24.0);
  b_l16_tmp = l9 * ((J_max_tmp - b_J_max_re_tmp * re) + c_J_max_re_tmp * re);
  l5 = l9 * ((0.0 - b_J_max_re_tmp * im) + c_J_max_re_tmp * im);
  l10 = -(A_init + J_min * t3[1].re);
  l8 = -(J_min * t3[1].im);
  if (l8 == 0.0) {
    t[1].re = l10 / J_max;
    t[1].im = 0.0;
  } else if (l10 == 0.0) {
    t[1].re = 0.0;
    t[1].im = l8 / J_max;
  } else {
    t[1].re = l10 / J_max;
    t[1].im = l8 / J_max;
  }
  t[3].re = 0.0;
  t[3].im = 0.0;
  t[5] = t3[1];
  if (l5 == 0.0) {
    if (b_im == 0.0) {
      t[7].re = b_re / b_l16_tmp;
      t[7].im = 0.0;
    } else if (b_re == 0.0) {
      t[7].re = 0.0;
      t[7].im = b_im / b_l16_tmp;
    } else {
      t[7].re = b_re / b_l16_tmp;
      t[7].im = b_im / b_l16_tmp;
    }
  } else if (b_l16_tmp == 0.0) {
    if (b_re == 0.0) {
      t[7].re = b_im / l5;
      t[7].im = 0.0;
    } else if (b_im == 0.0) {
      t[7].re = 0.0;
      t[7].im = -(b_re / l5);
    } else {
      t[7].re = b_im / l5;
      t[7].im = -(b_re / l5);
    }
  } else {
    l8 = std::abs(b_l16_tmp);
    l4 = std::abs(l5);
    if (l8 > l4) {
      l10 = l5 / b_l16_tmp;
      l4 = b_l16_tmp + l10 * l5;
      t[7].re = (b_re + l10 * b_im) / l4;
      t[7].im = (b_im - l10 * b_re) / l4;
    } else if (l4 == l8) {
      if (b_l16_tmp > 0.0) {
        l10 = 0.5;
      } else {
        l10 = -0.5;
      }
      if (l5 > 0.0) {
        l4 = 0.5;
      } else {
        l4 = -0.5;
      }
      t[7].re = (b_re * l10 + b_im * l4) / l8;
      t[7].im = (b_im * l10 - b_re * l4) / l8;
    } else {
      l10 = b_l16_tmp / l5;
      l4 = l5 + l10 * b_l16_tmp;
      t[7].re = (l10 * b_re + b_im) / l4;
      t[7].im = (l10 * b_im - b_re) / l4;
    }
  }
  l4 = -(1.0 / J_max * l4_tmp);
  J_max_re = (c_l16_tmp - J_max_re_tmp * 2.0) - A_min * l4_tmp * 2.0;
  y_re = A_min * A_min * J_max / J_min;
  t[8].re = l3;
  t[8].im = 0.0;
  t[9].re = l3;
  t[9].im = 0.0;
  l10 = (((((J_max_re + l16_tmp_tmp * l11[0].re) - l16_tmp) + b_y) -
          b_l16_tmp_tmp * l11[0].re) +
         y_re) *
        -0.5;
  l8 = (l16_tmp_tmp * l11[0].im - b_l16_tmp_tmp * l11[0].im) * -0.5;
  if (l8 == 0.0) {
    t[10].re = l10 / A_min_tmp;
    t[10].im = 0.0;
  } else if (l10 == 0.0) {
    t[10].re = 0.0;
    t[10].im = l8 / A_min_tmp;
  } else {
    t[10].re = l10 / A_min_tmp;
    t[10].im = l8 / A_min_tmp;
  }
  l10 = (((((J_max_re + l16_tmp_tmp * re) - l16_tmp) + b_y) -
          b_l16_tmp_tmp * re) +
         y_re) *
        -0.5;
  l8 = (l16_tmp_tmp * im - b_l16_tmp_tmp * im) * -0.5;
  if (l8 == 0.0) {
    t[11].re = l10 / A_min_tmp;
    t[11].im = 0.0;
  } else if (l10 == 0.0) {
    t[11].re = 0.0;
    t[11].im = l8 / A_min_tmp;
  } else {
    t[11].re = l10 / A_min_tmp;
    t[11].im = l8 / A_min_tmp;
  }
  t[12].re = l4;
  t[12].im = 0.0;
  t[13].re = l4;
  t[13].im = 0.0;
}

// End of code generation (acdefg_O_AVP.cpp)
