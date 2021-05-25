//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// acdef_NO_VP.cpp
//
// Code generation for function 'acdef_NO_VP'
//

// Include files
#include "acdef_NO_VP.h"
#include "power.h"
#include "rt_nonfinite.h"
#include "sqrt.h"
#include "topico_wrapper_data.h"
#include "topico_wrapper_rtwutil.h"
#include "topico_wrapper_types.h"
#include <cmath>

// Function Definitions
void acdef_NO_VP(double P_init, double V_init, double A_init, double P_wayp,
                 double V_wayp, double V_max, double A_min, double J_max,
                 double J_min, creal_T t[14])
{
  creal_T b_l7[2];
  creal_T l8[2];
  creal_T l9[2];
  creal_T t3[2];
  creal_T l17;
  double A_init_re;
  double A_min_re;
  double A_min_tmp;
  double J_min_re;
  double J_min_tmp;
  double b_A_min;
  double b_A_min_re;
  double b_A_min_tmp;
  double b_J_max;
  double b_J_min;
  double b_J_min_re;
  double b_l17_tmp;
  double b_l3;
  double b_l4;
  double b_y;
  double br;
  double c_A_min;
  double c_A_min_tmp;
  double c_J_min;
  double c_l17_tmp;
  double d_A_min;
  double d_A_min_tmp;
  double d_J_min;
  double e_A_min_tmp;
  double e_J_min;
  double im;
  double im_tmp;
  double l17_tmp;
  double l2;
  double l3;
  double l3_tmp;
  double l4;
  double l5_tmp;
  double l7;
  double re;
  double y;
  double y_im;
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
  //  Generated on 28-Aug-2019 13:51:13
  if (J_max < 0.0) {
    f_rtErrorWithMessageID("sqrt", n_emlrtRTEI.fName, n_emlrtRTEI.lineNo);
  }
  l7 = std::sqrt(J_max);
  l17_tmp = A_init * A_init;
  b_l17_tmp = J_min * V_init;
  c_l17_tmp = b_l17_tmp * 2.0;
  l17.re = -((J_min + -J_max) * ((l17_tmp + J_min * V_max * 2.0) + -c_l17_tmp));
  l17.im = 0.0;
  coder::internal::scalar::b_sqrt(&l17);
  br = rt_powd_snf(l7, 3.0) - J_min * l7;
  l7 = -1.0 / br;
  t3[0].re = l7 * l17.re;
  t3[0].im = l7 * l17.im;
  if (l17.im == 0.0) {
    t3[1].re = l17.re / br;
    t3[1].im = 0.0;
  } else if (l17.re == 0.0) {
    t3[1].re = 0.0;
    t3[1].im = l17.im / br;
  } else {
    t3[1].re = l17.re / br;
    t3[1].im = l17.im / br;
  }
  l3_tmp = A_min * A_min;
  l4 = J_min * J_min;
  l5_tmp = J_max * J_max;
  l7 = rt_powd_snf(J_max, 3.0);
  coder::power(t3, l8);
  y = l5_tmp * l5_tmp;
  b_y = l3_tmp * l3_tmp;
  l17.re = ((rt_powd_snf(A_init, 3.0) * A_min * -8.0 + l17_tmp * l3_tmp * 6.0) +
            V_init * V_init * l4 * 12.0) -
           V_wayp * V_wayp * l4 * 12.0;
  y_re = l17_tmp * l17_tmp * 3.0;
  A_min_re = A_min * P_init * l4 * 24.0;
  b_A_min_re = A_min * P_wayp * l4 * 24.0;
  J_min_re = b_l17_tmp * l17_tmp * 12.0;
  b_J_min_re = b_l17_tmp * l3_tmp * 12.0;
  b_A_min = A_min * l7;
  b_J_min = J_min * l7;
  l2 = l17_tmp * l5_tmp;
  l3 = l3_tmp * l5_tmp;
  b_l4 = l4 * l5_tmp;
  A_init_re = A_init * A_min * J_min * V_init * 24.0;
  A_min_tmp = A_min * J_min;
  b_A_min_tmp = A_min_tmp * l5_tmp;
  c_A_min_tmp = A_min * J_max;
  d_A_min_tmp = c_A_min_tmp * l4;
  e_A_min_tmp = A_min_tmp * l17_tmp;
  c_A_min = c_A_min_tmp * l17_tmp;
  J_min_tmp = J_min * J_max;
  c_J_min = J_min_tmp * l17_tmp;
  d_J_min = J_min_tmp * l3_tmp;
  c_A_min_tmp = A_min * V_init * l4;
  e_J_min = b_l17_tmp * l5_tmp;
  b_J_max = J_max * V_init * l4;
  d_A_min = A_min_tmp * J_max * V_init;
  b_l3 = A_min * (1.0 / J_min);
  re = t3[0].re * t3[0].re - t3[0].im * t3[0].im;
  im_tmp = t3[0].re * t3[0].im;
  im = im_tmp + im_tmp;
  b_l7[0].re = re;
  b_l7[0].im = im;
  l7 = re * re - im * im;
  im_tmp = re * im;
  br = im_tmp + im_tmp;
  l4 = y * l7;
  y_im = y * br;
  b_l17_tmp = b_J_min * l7;
  im_tmp = b_J_min * br;
  l7 *= b_l4;
  br *= b_l4;
  l9[0].re =
      -(((((((((((((((((((((((l17.re + l4 * 3.0) + y_re) - b_y) - A_min_re) +
                          b_A_min_re) -
                         J_min_re) -
                        b_J_min_re) +
                       b_A_min * l8[0].re * 4.0) -
                      b_l17_tmp * 6.0) -
                     l2 * re * 6.0) -
                    l3 * re * 6.0) +
                   l7 * 3.0) +
                  A_init_re) -
                 b_A_min_tmp * l8[0].re * 12.0) +
                d_A_min_tmp * l8[0].re * 8.0) +
               e_A_min_tmp * t3[0].re * 12.0) -
              c_A_min * t3[0].re * 12.0) +
             c_J_min * re * 6.0) +
            d_J_min * re * 6.0) -
           c_A_min_tmp * t3[0].re * 24.0) +
          e_J_min * re * 12.0) -
         b_J_max * re * 12.0) +
        d_A_min * t3[0].re * 24.0);
  l9[0].im =
      -(((((((((((((((y_im * 3.0 + b_A_min * l8[0].im * 4.0) - im_tmp * 6.0) -
                    l2 * im * 6.0) -
                   l3 * im * 6.0) +
                  br * 3.0) -
                 b_A_min_tmp * l8[0].im * 12.0) +
                d_A_min_tmp * l8[0].im * 8.0) +
               e_A_min_tmp * t3[0].im * 12.0) -
              c_A_min * t3[0].im * 12.0) +
             c_J_min * im * 6.0) +
            d_J_min * im * 6.0) -
           c_A_min_tmp * t3[0].im * 24.0) +
          e_J_min * im * 12.0) -
         b_J_max * im * 12.0) +
        d_A_min * t3[0].im * 24.0);
  l7 = -(A_init + J_max * t3[0].re);
  y_im = -(J_max * t3[0].im);
  if (y_im == 0.0) {
    t[0].re = l7 / J_min;
    t[0].im = 0.0;
  } else if (l7 == 0.0) {
    t[0].re = 0.0;
    t[0].im = y_im / J_min;
  } else {
    t[0].re = l7 / J_min;
    t[0].im = y_im / J_min;
  }
  t[2].re = 0.0;
  t[2].im = 0.0;
  t[4] = t3[0];
  re = t3[1].re * t3[1].re - t3[1].im * t3[1].im;
  im_tmp = t3[1].re * t3[1].im;
  im = im_tmp + im_tmp;
  l7 = re * re - im * im;
  im_tmp = re * im;
  br = im_tmp + im_tmp;
  l4 = y * l7;
  y_im = y * br;
  b_l17_tmp = b_J_min * l7;
  im_tmp = b_J_min * br;
  l7 *= b_l4;
  br *= b_l4;
  b_J_min =
      -(((((((((((((((((((((((l17.re + l4 * 3.0) + y_re) - b_y) - A_min_re) +
                          b_A_min_re) -
                         J_min_re) -
                        b_J_min_re) +
                       b_A_min * l8[1].re * 4.0) -
                      b_l17_tmp * 6.0) -
                     l2 * re * 6.0) -
                    l3 * re * 6.0) +
                   l7 * 3.0) +
                  A_init_re) -
                 b_A_min_tmp * l8[1].re * 12.0) +
                d_A_min_tmp * l8[1].re * 8.0) +
               e_A_min_tmp * t3[1].re * 12.0) -
              c_A_min * t3[1].re * 12.0) +
             c_J_min * re * 6.0) +
            d_J_min * re * 6.0) -
           c_A_min_tmp * t3[1].re * 24.0) +
          e_J_min * re * 12.0) -
         b_J_max * re * 12.0) +
        d_A_min * t3[1].re * 24.0);
  im_tmp =
      -(((((((((((((((y_im * 3.0 + b_A_min * l8[1].im * 4.0) - im_tmp * 6.0) -
                    l2 * im * 6.0) -
                   l3 * im * 6.0) +
                  br * 3.0) -
                 b_A_min_tmp * l8[1].im * 12.0) +
                d_A_min_tmp * l8[1].im * 8.0) +
               e_A_min_tmp * t3[1].im * 12.0) -
              c_A_min * t3[1].im * 12.0) +
             c_J_min * im * 6.0) +
            d_J_min * im * 6.0) -
           c_A_min_tmp * t3[1].im * 24.0) +
          e_J_min * im * 12.0) -
         b_J_max * im * 12.0) +
        d_A_min * t3[1].im * 24.0);
  l7 = -(A_init + J_max * t3[1].re);
  y_im = -(J_max * t3[1].im);
  if (y_im == 0.0) {
    t[1].re = l7 / J_min;
    t[1].im = 0.0;
  } else if (l7 == 0.0) {
    t[1].re = 0.0;
    t[1].im = y_im / J_min;
  } else {
    t[1].re = l7 / J_min;
    t[1].im = y_im / J_min;
  }
  t[3].re = 0.0;
  t[3].im = 0.0;
  t[5] = t3[1];
  A_min_re = e_A_min_tmp * 12.0 - c_A_min_tmp * 24.0;
  J_min_re = c_l17_tmp - J_min * V_wayp * 2.0;
  t[8].re = b_l3;
  t[8].im = 0.0;
  t[9].re = b_l3;
  t[9].im = 0.0;
  br = (A_min_re - b_A_min_tmp * b_l7[0].re * 12.0) +
       d_A_min_tmp * b_l7[0].re * 12.0;
  b_l17_tmp =
      (0.0 - b_A_min_tmp * b_l7[0].im * 12.0) + d_A_min_tmp * b_l7[0].im * 12.0;
  if (b_l17_tmp == 0.0) {
    if (l9[0].im == 0.0) {
      t[6].re = l9[0].re / br;
      t[6].im = 0.0;
    } else if (l9[0].re == 0.0) {
      t[6].re = 0.0;
      t[6].im = l9[0].im / br;
    } else {
      t[6].re = l9[0].re / br;
      t[6].im = l9[0].im / br;
    }
  } else if (br == 0.0) {
    if (l9[0].re == 0.0) {
      t[6].re = l9[0].im / b_l17_tmp;
      t[6].im = 0.0;
    } else if (l9[0].im == 0.0) {
      t[6].re = 0.0;
      t[6].im = -(l9[0].re / b_l17_tmp);
    } else {
      t[6].re = l9[0].im / b_l17_tmp;
      t[6].im = -(l9[0].re / b_l17_tmp);
    }
  } else {
    y_im = std::abs(br);
    l7 = std::abs(b_l17_tmp);
    if (y_im > l7) {
      l4 = b_l17_tmp / br;
      l7 = br + l4 * b_l17_tmp;
      t[6].re = (l9[0].re + l4 * l9[0].im) / l7;
      t[6].im = (l9[0].im - l4 * l9[0].re) / l7;
    } else if (l7 == y_im) {
      if (br > 0.0) {
        l4 = 0.5;
      } else {
        l4 = -0.5;
      }
      if (b_l17_tmp > 0.0) {
        l7 = 0.5;
      } else {
        l7 = -0.5;
      }
      t[6].re = (l9[0].re * l4 + l9[0].im * l7) / y_im;
      t[6].im = (l9[0].im * l4 - l9[0].re * l7) / y_im;
    } else {
      l4 = br / b_l17_tmp;
      l7 = b_l17_tmp + l4 * br;
      t[6].re = (l4 * l9[0].re + l9[0].im) / l7;
      t[6].im = (l4 * l9[0].im - l9[0].re) / l7;
    }
  }
  l7 = ((((J_min_re + l5_tmp * b_l7[0].re) - l17_tmp) + l3_tmp) -
        J_min_tmp * b_l7[0].re) *
       -0.5;
  y_im = (l5_tmp * b_l7[0].im - J_min_tmp * b_l7[0].im) * -0.5;
  if (y_im == 0.0) {
    t[10].re = l7 / A_min_tmp;
    t[10].im = 0.0;
  } else if (l7 == 0.0) {
    t[10].re = 0.0;
    t[10].im = y_im / A_min_tmp;
  } else {
    t[10].re = l7 / A_min_tmp;
    t[10].im = y_im / A_min_tmp;
  }
  t[12].re = 0.0;
  t[12].im = 0.0;
  br = (A_min_re - b_A_min_tmp * re * 12.0) + d_A_min_tmp * re * 12.0;
  b_l17_tmp = (0.0 - b_A_min_tmp * im * 12.0) + d_A_min_tmp * im * 12.0;
  if (b_l17_tmp == 0.0) {
    if (im_tmp == 0.0) {
      t[7].re = b_J_min / br;
      t[7].im = 0.0;
    } else if (b_J_min == 0.0) {
      t[7].re = 0.0;
      t[7].im = im_tmp / br;
    } else {
      t[7].re = b_J_min / br;
      t[7].im = im_tmp / br;
    }
  } else if (br == 0.0) {
    if (b_J_min == 0.0) {
      t[7].re = im_tmp / b_l17_tmp;
      t[7].im = 0.0;
    } else if (im_tmp == 0.0) {
      t[7].re = 0.0;
      t[7].im = -(b_J_min / b_l17_tmp);
    } else {
      t[7].re = im_tmp / b_l17_tmp;
      t[7].im = -(b_J_min / b_l17_tmp);
    }
  } else {
    y_im = std::abs(br);
    l7 = std::abs(b_l17_tmp);
    if (y_im > l7) {
      l4 = b_l17_tmp / br;
      l7 = br + l4 * b_l17_tmp;
      t[7].re = (b_J_min + l4 * im_tmp) / l7;
      t[7].im = (im_tmp - l4 * b_J_min) / l7;
    } else if (l7 == y_im) {
      if (br > 0.0) {
        l4 = 0.5;
      } else {
        l4 = -0.5;
      }
      if (b_l17_tmp > 0.0) {
        l7 = 0.5;
      } else {
        l7 = -0.5;
      }
      t[7].re = (b_J_min * l4 + im_tmp * l7) / y_im;
      t[7].im = (im_tmp * l4 - b_J_min * l7) / y_im;
    } else {
      l4 = br / b_l17_tmp;
      l7 = b_l17_tmp + l4 * br;
      t[7].re = (l4 * b_J_min + im_tmp) / l7;
      t[7].im = (l4 * im_tmp - b_J_min) / l7;
    }
  }
  l7 =
      ((((J_min_re + l5_tmp * re) - l17_tmp) + l3_tmp) - J_min_tmp * re) * -0.5;
  y_im = (l5_tmp * im - J_min_tmp * im) * -0.5;
  if (y_im == 0.0) {
    t[11].re = l7 / A_min_tmp;
    t[11].im = 0.0;
  } else if (l7 == 0.0) {
    t[11].re = 0.0;
    t[11].im = y_im / A_min_tmp;
  } else {
    t[11].re = l7 / A_min_tmp;
    t[11].im = y_im / A_min_tmp;
  }
  t[13].re = 0.0;
  t[13].im = 0.0;
}

// End of code generation (acdef_NO_VP.cpp)
