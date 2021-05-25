//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// acdefg_NO_AVP.cpp
//
// Code generation for function 'acdefg_NO_AVP'
//

// Include files
#include "acdefg_NO_AVP.h"
#include "power.h"
#include "rt_nonfinite.h"
#include "sqrt.h"
#include "topico_wrapper_data.h"
#include "topico_wrapper_rtwutil.h"
#include "topico_wrapper_types.h"
#include <cmath>

// Function Definitions
void acdefg_NO_AVP(double P_init, double V_init, double A_init, double P_wayp,
                   double V_wayp, double A_wayp, double V_max, double A_min,
                   double J_max, double J_min, creal_T t[14])
{
  creal_T l11[2];
  creal_T l12[2];
  creal_T t3[2];
  creal_T l17;
  double A_init_re;
  double A_min_re;
  double A_min_tmp;
  double J_max_re;
  double J_min_re;
  double J_min_tmp;
  double b_A_min;
  double b_A_min_re;
  double b_A_min_tmp;
  double b_J_max_re;
  double b_J_min;
  double b_J_min_re;
  double b_V_init;
  double b_im;
  double b_l17_tmp;
  double b_l3;
  double b_l6;
  double b_re;
  double b_y;
  double b_y_re;
  double c_A_min;
  double c_A_min_re;
  double c_A_min_tmp;
  double c_J_min;
  double c_y_re;
  double d_A_min;
  double d_A_min_re;
  double d_A_min_tmp;
  double d_J_min;
  double e_J_min;
  double f_J_min;
  double im;
  double l10;
  double l17_tmp;
  double l2;
  double l2_re;
  double l3;
  double l3_re;
  double l3_tmp;
  double l5_tmp;
  double l6;
  double l7;
  double l7_tmp;
  double l8_tmp;
  double l9;
  double re;
  double re_tmp;
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
  if (J_max < 0.0) {
    f_rtErrorWithMessageID("sqrt", n_emlrtRTEI.fName, n_emlrtRTEI.lineNo);
  }
  l7 = std::sqrt(J_max);
  l17_tmp = A_init * A_init;
  b_l17_tmp = J_min * V_init;
  l17.re = -((J_min + -J_max) *
             ((l17_tmp + J_min * V_max * 2.0) + -(b_l17_tmp * 2.0)));
  l17.im = 0.0;
  coder::internal::scalar::b_sqrt(&l17);
  l7 = rt_powd_snf(l7, 3.0) - J_min * l7;
  l10 = -1.0 / l7;
  t3[0].re = l10 * l17.re;
  t3[0].im = l10 * l17.im;
  if (l17.im == 0.0) {
    t3[1].re = l17.re / l7;
    t3[1].im = 0.0;
  } else if (l17.re == 0.0) {
    t3[1].re = 0.0;
    t3[1].im = l17.im / l7;
  } else {
    t3[1].re = l17.re / l7;
    t3[1].im = l17.im / l7;
  }
  l3_tmp = A_min * A_min;
  l5_tmp = A_wayp * A_wayp;
  l6 = J_min * J_min;
  l7_tmp = J_max * J_max;
  l8_tmp = rt_powd_snf(J_max, 3.0);
  l10 = rt_powd_snf(J_max, 5.0);
  coder::power(t3, l12);
  l7 = l3_tmp * l3_tmp;
  l9 = l7_tmp * l7_tmp;
  y = rt_powd_snf(l7_tmp, 3.0);
  l17.re = ((l7 * l6 - l7 * l7_tmp) + l17_tmp * l17_tmp * l7_tmp * 3.0) -
           l5_tmp * l5_tmp * l6 * 3.0;
  b_A_min = A_min * l10;
  b_J_min = J_min * l10;
  y_re = rt_powd_snf(A_init, 3.0) * A_min * l7_tmp * 8.0;
  A_min_re = A_min * rt_powd_snf(A_wayp, 3.0) * l6 * 8.0;
  l2_re = l17_tmp * l3_tmp * l7_tmp * 6.0;
  l3_re = l3_tmp * l5_tmp * l6 * 6.0;
  l2 = l17_tmp * l9;
  l3 = l3_tmp * l9;
  b_l6 = l6 * l9;
  b_y_re = V_init * V_init * l6 * l7_tmp * 12.0;
  c_y_re = V_wayp * V_wayp * l6 * l7_tmp * 12.0;
  c_J_min = J_min * l17_tmp * l8_tmp;
  J_min_tmp = J_min * l3_tmp;
  d_J_min = J_min_tmp * l8_tmp;
  b_V_init = V_init * l6 * l8_tmp;
  l10 = A_min * J_min;
  A_min_tmp = l10 * l9;
  b_A_min_re = A_min * P_init * l6 * l7_tmp * 24.0;
  c_A_min_re = A_min * P_wayp * l6 * l7_tmp * 24.0;
  J_min_re = b_l17_tmp * l17_tmp * l7_tmp * 12.0;
  b_J_min_re = b_l17_tmp * l3_tmp * l7_tmp * 12.0;
  e_J_min = b_l17_tmp * l9;
  l7 = J_max * V_wayp;
  J_max_re = l7 * l3_tmp * l6 * 12.0;
  b_J_max_re = l7 * l5_tmp * l6 * 12.0;
  b_A_min_tmp = A_min * l6 * l8_tmp;
  c_A_min = A_min * l17_tmp * l8_tmp;
  d_A_min = l10 * V_init * l8_tmp;
  c_A_min_tmp = l10 * l17_tmp * l7_tmp;
  d_A_min_tmp = A_min * V_init * l6 * l7_tmp;
  A_init_re = A_init * A_min * J_min * V_init * l7_tmp * 24.0;
  d_A_min_re = A_min * A_wayp * J_max * V_wayp * l6 * 24.0;
  b_y = l10 * J_max * 2.0;
  b_l3 = A_min * (1.0 / J_min);
  f_J_min = J_min * l7_tmp;
  re = t3[0].re * t3[0].re - t3[0].im * t3[0].im;
  l7 = t3[0].re * t3[0].im;
  im = l7 + l7;
  l11[0].re = re;
  l11[0].im = im;
  b_re = re * re - im * im;
  l7 = re * im;
  b_im = l7 + l7;
  l9 = y * b_re;
  b_l17_tmp = y * b_im;
  l7_tmp = b_J_min * b_re;
  l6 = b_J_min * b_im;
  l7 = b_l6 * b_re;
  l10 = b_l6 * b_im;
  b_re = -0.083333333333333329 *
         ((((((((((((((((((((((((((((((l17.re + l9 * 3.0) +
                                      b_A_min * l12[0].re * 4.0) -
                                     l7_tmp * 6.0) -
                                    y_re) +
                                   A_min_re) +
                                  l2_re) -
                                 l3_re) -
                                l2 * re * 6.0) -
                               l3 * re * 6.0) +
                              l7 * 3.0) +
                             b_y_re) -
                            c_y_re) +
                           c_J_min * re * 6.0) +
                          d_J_min * re * 6.0) -
                         b_V_init * re * 12.0) -
                        A_min_tmp * l12[0].re * 12.0) -
                       b_A_min_re) +
                      c_A_min_re) -
                     J_min_re) -
                    b_J_min_re) +
                   e_J_min * re * 12.0) +
                  J_max_re) +
                 b_J_max_re) +
                b_A_min_tmp * l12[0].re * 8.0) -
               c_A_min * t3[0].re * 12.0) +
              d_A_min * t3[0].re * 24.0) +
             c_A_min_tmp * t3[0].re * 12.0) -
            d_A_min_tmp * t3[0].re * 24.0) +
           A_init_re) -
          d_A_min_re);
  b_im =
      -0.083333333333333329 *
      (((((((((((((((b_l17_tmp * 3.0 + b_A_min * l12[0].im * 4.0) - l6 * 6.0) -
                   l2 * im * 6.0) -
                  l3 * im * 6.0) +
                 l10 * 3.0) +
                c_J_min * im * 6.0) +
               d_J_min * im * 6.0) -
              b_V_init * im * 12.0) -
             A_min_tmp * l12[0].im * 12.0) +
            e_J_min * im * 12.0) +
           b_A_min_tmp * l12[0].im * 8.0) -
          c_A_min * t3[0].im * 12.0) +
         d_A_min * t3[0].im * 24.0) +
        c_A_min_tmp * t3[0].im * 12.0) -
       d_A_min_tmp * t3[0].im * 24.0);
  re_tmp = d_A_min_tmp * 2.0;
  b_l17_tmp = ((c_A_min_tmp - A_min_tmp * re) - re_tmp) + b_A_min_tmp * re;
  l7_tmp = (0.0 - A_min_tmp * im) + b_A_min_tmp * im;
  l7 = -(A_init + J_max * t3[0].re);
  l10 = -(J_max * t3[0].im);
  if (l10 == 0.0) {
    t[0].re = l7 / J_min;
    t[0].im = 0.0;
  } else if (l7 == 0.0) {
    t[0].re = 0.0;
    t[0].im = l10 / J_min;
  } else {
    t[0].re = l7 / J_min;
    t[0].im = l10 / J_min;
  }
  t[2].re = 0.0;
  t[2].im = 0.0;
  t[4] = t3[0];
  if (l7_tmp == 0.0) {
    if (b_im == 0.0) {
      t[6].re = b_re / b_l17_tmp;
      t[6].im = 0.0;
    } else if (b_re == 0.0) {
      t[6].re = 0.0;
      t[6].im = b_im / b_l17_tmp;
    } else {
      t[6].re = b_re / b_l17_tmp;
      t[6].im = b_im / b_l17_tmp;
    }
  } else if (b_l17_tmp == 0.0) {
    if (b_re == 0.0) {
      t[6].re = b_im / l7_tmp;
      t[6].im = 0.0;
    } else if (b_im == 0.0) {
      t[6].re = 0.0;
      t[6].im = -(b_re / l7_tmp);
    } else {
      t[6].re = b_im / l7_tmp;
      t[6].im = -(b_re / l7_tmp);
    }
  } else {
    l9 = std::abs(b_l17_tmp);
    l7 = std::abs(l7_tmp);
    if (l9 > l7) {
      l10 = l7_tmp / b_l17_tmp;
      l7 = b_l17_tmp + l10 * l7_tmp;
      t[6].re = (b_re + l10 * b_im) / l7;
      t[6].im = (b_im - l10 * b_re) / l7;
    } else if (l7 == l9) {
      if (b_l17_tmp > 0.0) {
        l10 = 0.5;
      } else {
        l10 = -0.5;
      }
      if (l7_tmp > 0.0) {
        l7 = 0.5;
      } else {
        l7 = -0.5;
      }
      t[6].re = (b_re * l10 + b_im * l7) / l9;
      t[6].im = (b_im * l10 - b_re * l7) / l9;
    } else {
      l10 = b_l17_tmp / l7_tmp;
      l7 = l7_tmp + l10 * b_l17_tmp;
      t[6].re = (l10 * b_re + b_im) / l7;
      t[6].im = (l10 * b_im - b_re) / l7;
    }
  }
  re = t3[1].re * t3[1].re - t3[1].im * t3[1].im;
  l7 = t3[1].re * t3[1].im;
  im = l7 + l7;
  b_re = re * re - im * im;
  l7 = re * im;
  b_im = l7 + l7;
  l9 = y * b_re;
  b_l17_tmp = y * b_im;
  l7_tmp = b_J_min * b_re;
  l6 = b_J_min * b_im;
  l7 = b_l6 * b_re;
  l10 = b_l6 * b_im;
  b_re = -0.083333333333333329 *
         ((((((((((((((((((((((((((((((l17.re + l9 * 3.0) +
                                      b_A_min * l12[1].re * 4.0) -
                                     l7_tmp * 6.0) -
                                    y_re) +
                                   A_min_re) +
                                  l2_re) -
                                 l3_re) -
                                l2 * re * 6.0) -
                               l3 * re * 6.0) +
                              l7 * 3.0) +
                             b_y_re) -
                            c_y_re) +
                           c_J_min * re * 6.0) +
                          d_J_min * re * 6.0) -
                         b_V_init * re * 12.0) -
                        A_min_tmp * l12[1].re * 12.0) -
                       b_A_min_re) +
                      c_A_min_re) -
                     J_min_re) -
                    b_J_min_re) +
                   e_J_min * re * 12.0) +
                  J_max_re) +
                 b_J_max_re) +
                b_A_min_tmp * l12[1].re * 8.0) -
               c_A_min * t3[1].re * 12.0) +
              d_A_min * t3[1].re * 24.0) +
             c_A_min_tmp * t3[1].re * 12.0) -
            d_A_min_tmp * t3[1].re * 24.0) +
           A_init_re) -
          d_A_min_re);
  b_im =
      -0.083333333333333329 *
      (((((((((((((((b_l17_tmp * 3.0 + b_A_min * l12[1].im * 4.0) - l6 * 6.0) -
                   l2 * im * 6.0) -
                  l3 * im * 6.0) +
                 l10 * 3.0) +
                c_J_min * im * 6.0) +
               d_J_min * im * 6.0) -
              b_V_init * im * 12.0) -
             A_min_tmp * l12[1].im * 12.0) +
            e_J_min * im * 12.0) +
           b_A_min_tmp * l12[1].im * 8.0) -
          c_A_min * t3[1].im * 12.0) +
         d_A_min * t3[1].im * 24.0) +
        c_A_min_tmp * t3[1].im * 12.0) -
       d_A_min_tmp * t3[1].im * 24.0);
  b_l17_tmp = ((c_A_min_tmp - A_min_tmp * re) - re_tmp) + b_A_min_tmp * re;
  l7_tmp = (0.0 - A_min_tmp * im) + b_A_min_tmp * im;
  l7 = -(A_init + J_max * t3[1].re);
  l10 = -(J_max * t3[1].im);
  if (l10 == 0.0) {
    t[1].re = l7 / J_min;
    t[1].im = 0.0;
  } else if (l7 == 0.0) {
    t[1].re = 0.0;
    t[1].im = l10 / J_min;
  } else {
    t[1].re = l7 / J_min;
    t[1].im = l10 / J_min;
  }
  t[3].re = 0.0;
  t[3].im = 0.0;
  t[5] = t3[1];
  if (l7_tmp == 0.0) {
    if (b_im == 0.0) {
      t[7].re = b_re / b_l17_tmp;
      t[7].im = 0.0;
    } else if (b_re == 0.0) {
      t[7].re = 0.0;
      t[7].im = b_im / b_l17_tmp;
    } else {
      t[7].re = b_re / b_l17_tmp;
      t[7].im = b_im / b_l17_tmp;
    }
  } else if (b_l17_tmp == 0.0) {
    if (b_re == 0.0) {
      t[7].re = b_im / l7_tmp;
      t[7].im = 0.0;
    } else if (b_im == 0.0) {
      t[7].re = 0.0;
      t[7].im = -(b_re / l7_tmp);
    } else {
      t[7].re = b_im / l7_tmp;
      t[7].im = -(b_re / l7_tmp);
    }
  } else {
    l9 = std::abs(b_l17_tmp);
    l7 = std::abs(l7_tmp);
    if (l9 > l7) {
      l10 = l7_tmp / b_l17_tmp;
      l7 = b_l17_tmp + l10 * l7_tmp;
      t[7].re = (b_re + l10 * b_im) / l7;
      t[7].im = (b_im - l10 * b_re) / l7;
    } else if (l7 == l9) {
      if (b_l17_tmp > 0.0) {
        l10 = 0.5;
      } else {
        l10 = -0.5;
      }
      if (l7_tmp > 0.0) {
        l7 = 0.5;
      } else {
        l7 = -0.5;
      }
      t[7].re = (b_re * l10 + b_im * l7) / l9;
      t[7].im = (b_im * l10 - b_re * l7) / l9;
    } else {
      l10 = b_l17_tmp / l7_tmp;
      l7 = l7_tmp + l10 * b_l17_tmp;
      t[7].re = (l10 * b_re + b_im) / l7;
      t[7].im = (l10 * b_im - b_re) / l7;
    }
  }
  l6 = -(1.0 / J_max * (A_min + -A_wayp));
  J_min_re = ((J_min_tmp - J_max * l3_tmp) + l17_tmp * J_max) - l5_tmp * J_min;
  l7 = J_min * J_max;
  b_J_min_re = l7 * V_init * 2.0;
  l17.re = l7 * V_wayp * 2.0;
  t[8].re = b_l3;
  t[8].im = 0.0;
  t[9].re = b_l3;
  t[9].im = 0.0;
  l7 = (((J_min_re - l8_tmp * l11[0].re) - b_J_min_re) + l17.re) +
       f_J_min * l11[0].re;
  l10 = (0.0 - l8_tmp * l11[0].im) + f_J_min * l11[0].im;
  if (l10 == 0.0) {
    t[10].re = l7 / b_y;
    t[10].im = 0.0;
  } else if (l7 == 0.0) {
    t[10].re = 0.0;
    t[10].im = l10 / b_y;
  } else {
    t[10].re = l7 / b_y;
    t[10].im = l10 / b_y;
  }
  l7 = (((J_min_re - l8_tmp * re) - b_J_min_re) + l17.re) + f_J_min * re;
  l10 = (0.0 - l8_tmp * im) + f_J_min * im;
  if (l10 == 0.0) {
    t[11].re = l7 / b_y;
    t[11].im = 0.0;
  } else if (l7 == 0.0) {
    t[11].re = 0.0;
    t[11].im = l10 / b_y;
  } else {
    t[11].re = l7 / b_y;
    t[11].im = l10 / b_y;
  }
  t[12].re = l6;
  t[12].im = 0.0;
  t[13].re = l6;
  t[13].im = 0.0;
}

// End of code generation (acdefg_NO_AVP.cpp)
