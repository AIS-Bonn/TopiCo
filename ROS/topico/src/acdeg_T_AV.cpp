//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// acdeg_T_AV.cpp
//
// Code generation for function 'acdeg_T_AV'
//

// Include files
#include "acdeg_T_AV.h"
#include "rt_nonfinite.h"
#include "sqrt.h"
#include "topico_wrapper_data.h"
#include <cmath>

// Function Definitions
void acdeg_T_AV(double V_init, double A_init, double V_wayp, double A_wayp,
                double V_max, double J_max, double J_min, double T,
                creal_T t[28])
{
  creal_T t4[4];
  creal_T l35;
  creal_T l41;
  double A_init_re;
  double A_wayp_re;
  double J_min_re;
  double J_min_tmp;
  double T_re;
  double ar;
  double ar_tmp;
  double b_A_init_re;
  double b_A_wayp_re;
  double b_ar_tmp;
  double b_l2_re_tmp_tmp;
  double b_l35_tmp;
  double l13;
  double l2_im;
  double l2_re_tmp_tmp;
  double l2_tmp;
  double l35_tmp;
  double l3_tmp;
  double l41_tmp;
  double l4_tmp;
  double l8_tmp;
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
  //  Generated on 03-Sep-2019 11:48:12
  l2_tmp = A_init * A_init;
  l3_tmp = A_wayp * A_wayp;
  l4_tmp = J_max * J_max;
  l8_tmp = J_max * V_max;
  l13 = 1.0 / J_max;
  l35_tmp = J_max * V_init;
  b_l35_tmp = J_max * V_wayp;
  l35.re =
      (((((((l8_tmp * l2_tmp * 2.0 + l8_tmp * l3_tmp * 2.0) + l2_tmp * l3_tmp) +
           V_init * V_wayp * l4_tmp * 4.0) +
          b_l35_tmp * l2_tmp * -2.0) +
         l35_tmp * l3_tmp * -2.0) +
        -(V_init * V_max * l4_tmp * 4.0)) +
       -(V_max * V_wayp * l4_tmp * 4.0)) +
      l4_tmp * (V_max * V_max) * 4.0;
  l35.im = 0.0;
  coder::internal::scalar::b_sqrt(&l35);
  l35.re *= 2.0;
  l35.im *= 2.0;
  l2_re_tmp_tmp = l35_tmp * 2.0;
  b_l2_re_tmp_tmp = b_l35_tmp * 2.0;
  l8_tmp =
      (((l2_tmp + l3_tmp) + l8_tmp * 4.0) + -l2_re_tmp_tmp) + -b_l2_re_tmp_tmp;
  l41_tmp = 1.0 / J_min * (J_min + -J_max);
  l41.re = l41_tmp * (l8_tmp - l35.re);
  l41.im = l41_tmp * (0.0 - l35.im);
  coder::internal::scalar::b_sqrt(&l41);
  l8_tmp += l35.re;
  l2_im = l35.im;
  l35.re = l41_tmp * l8_tmp;
  l35.im = l41_tmp * l2_im;
  coder::internal::scalar::b_sqrt(&l35);
  l8_tmp = (A_init + J_max * T) + -A_wayp;
  t4[0].re = l13 * (l8_tmp - l35.re);
  t4[0].im = l13 * (0.0 - l35.im);
  t4[1].re = l13 * (l8_tmp + l41.re);
  t4[1].im = l13 * l41.im;
  t4[2].re = l13 * (l8_tmp + l35.re);
  t4[2].im = l13 * l35.im;
  t4[3].re = l13 * (l8_tmp - l41.re);
  t4[3].im = l13 * (0.0 - l41.im);
  l35_tmp = A_init * J_max;
  b_l35_tmp = A_wayp * J_min;
  l35.re =
      (((((((J_min * V_init * -2.0 + l2_re_tmp_tmp) + J_min * V_wayp * 2.0) -
           b_l2_re_tmp_tmp) +
          l2_tmp) +
         l3_tmp) -
        A_init * A_wayp * 2.0) +
       l35_tmp * T * 2.0) -
      b_l35_tmp * T * 2.0;
  l41_tmp = J_min * J_max;
  l41.re = l41_tmp * (T * T);
  J_min_tmp = l41_tmp * T;
  T_re = T * l4_tmp * 2.0;
  A_init_re = A_init * J_min * 2.0;
  b_A_init_re = l35_tmp * 2.0;
  A_wayp_re = b_l35_tmp * 2.0;
  b_A_wayp_re = A_wayp * J_max * 2.0;
  J_min_re = J_min_tmp * 2.0;
  y = J_min - J_max;
  l8_tmp = t4[0].re * t4[0].im;
  ar = -(((((l35.re - l35_tmp * t4[0].re * 2.0) + b_l35_tmp * t4[0].re * 2.0) +
           l41.re) +
          l41_tmp * (t4[0].re * t4[0].re - t4[0].im * t4[0].im)) -
         J_min_tmp * t4[0].re * 2.0);
  b_l2_re_tmp_tmp =
      -((((0.0 - l35_tmp * t4[0].im * 2.0) + b_l35_tmp * t4[0].im * 2.0) +
         l41_tmp * (l8_tmp + l8_tmp)) -
        J_min_tmp * t4[0].im * 2.0);
  l2_tmp = ((((((T_re - l4_tmp * t4[0].re * 2.0) - A_init_re) + b_A_init_re) +
              A_wayp_re) -
             b_A_wayp_re) -
            J_min_re) +
           l41_tmp * t4[0].re * 2.0;
  l3_tmp = (0.0 - l4_tmp * t4[0].im * 2.0) + l41_tmp * t4[0].im * 2.0;
  if (l3_tmp == 0.0) {
    if (b_l2_re_tmp_tmp == 0.0) {
      l2_re_tmp_tmp = ar / l2_tmp;
      l8_tmp = 0.0;
    } else if (ar == 0.0) {
      l2_re_tmp_tmp = 0.0;
      l8_tmp = b_l2_re_tmp_tmp / l2_tmp;
    } else {
      l2_re_tmp_tmp = ar / l2_tmp;
      l8_tmp = b_l2_re_tmp_tmp / l2_tmp;
    }
  } else if (l2_tmp == 0.0) {
    if (ar == 0.0) {
      l2_re_tmp_tmp = b_l2_re_tmp_tmp / l3_tmp;
      l8_tmp = 0.0;
    } else if (b_l2_re_tmp_tmp == 0.0) {
      l2_re_tmp_tmp = 0.0;
      l8_tmp = -(ar / l3_tmp);
    } else {
      l2_re_tmp_tmp = b_l2_re_tmp_tmp / l3_tmp;
      l8_tmp = -(ar / l3_tmp);
    }
  } else {
    l13 = std::abs(l2_tmp);
    l8_tmp = std::abs(l3_tmp);
    if (l13 > l8_tmp) {
      l2_im = l3_tmp / l2_tmp;
      l8_tmp = l2_tmp + l2_im * l3_tmp;
      l2_re_tmp_tmp = (ar + l2_im * b_l2_re_tmp_tmp) / l8_tmp;
      l8_tmp = (b_l2_re_tmp_tmp - l2_im * ar) / l8_tmp;
    } else if (l8_tmp == l13) {
      if (l2_tmp > 0.0) {
        l2_im = 0.5;
      } else {
        l2_im = -0.5;
      }
      if (l3_tmp > 0.0) {
        l8_tmp = 0.5;
      } else {
        l8_tmp = -0.5;
      }
      l2_re_tmp_tmp = (ar * l2_im + b_l2_re_tmp_tmp * l8_tmp) / l13;
      l8_tmp = (b_l2_re_tmp_tmp * l2_im - ar * l8_tmp) / l13;
    } else {
      l2_im = l2_tmp / l3_tmp;
      l8_tmp = l3_tmp + l2_im * l2_tmp;
      l2_re_tmp_tmp = (l2_im * ar + b_l2_re_tmp_tmp) / l8_tmp;
      l8_tmp = (l2_im * b_l2_re_tmp_tmp - ar) / l8_tmp;
    }
  }
  l2_im = J_max * l2_re_tmp_tmp;
  l2_tmp = J_max * l8_tmp;
  ar_tmp = A_init - A_wayp;
  b_ar_tmp = J_min * T;
  ar = (((ar_tmp - J_min * l2_re_tmp_tmp) + l2_im) - J_min * t4[0].re) +
       b_ar_tmp;
  b_l2_re_tmp_tmp = ((0.0 - J_min * l8_tmp) + l2_tmp) - J_min * t4[0].im;
  if (b_l2_re_tmp_tmp == 0.0) {
    l3_tmp = ar / y;
    l13 = 0.0;
  } else if (ar == 0.0) {
    l3_tmp = 0.0;
    l13 = b_l2_re_tmp_tmp / y;
  } else {
    l3_tmp = ar / y;
    l13 = b_l2_re_tmp_tmp / y;
  }
  t[0].re = l2_re_tmp_tmp;
  t[0].im = l8_tmp;
  t[4].re = 0.0;
  t[4].im = 0.0;
  ar = -(A_init + l2_im);
  if (-l2_tmp == 0.0) {
    t[8].re = ar / J_min;
    t[8].im = 0.0;
  } else if (ar == 0.0) {
    t[8].re = 0.0;
    t[8].im = -l2_tmp / J_min;
  } else {
    t[8].re = ar / J_min;
    t[8].im = -l2_tmp / J_min;
  }
  t[12] = t4[0];
  ar = A_wayp - J_max * l3_tmp;
  b_l2_re_tmp_tmp = 0.0 - J_max * l13;
  if (b_l2_re_tmp_tmp == 0.0) {
    t[16].re = ar / J_min;
    t[16].im = 0.0;
  } else if (ar == 0.0) {
    t[16].re = 0.0;
    t[16].im = b_l2_re_tmp_tmp / J_min;
  } else {
    t[16].re = ar / J_min;
    t[16].im = b_l2_re_tmp_tmp / J_min;
  }
  t[20].re = 0.0;
  t[20].im = 0.0;
  t[24].re = l3_tmp;
  t[24].im = l13;
  l8_tmp = t4[1].re * t4[1].im;
  ar = -(((((l35.re - l35_tmp * t4[1].re * 2.0) + b_l35_tmp * t4[1].re * 2.0) +
           l41.re) +
          l41_tmp * (t4[1].re * t4[1].re - t4[1].im * t4[1].im)) -
         J_min_tmp * t4[1].re * 2.0);
  b_l2_re_tmp_tmp =
      -((((0.0 - l35_tmp * t4[1].im * 2.0) + b_l35_tmp * t4[1].im * 2.0) +
         l41_tmp * (l8_tmp + l8_tmp)) -
        J_min_tmp * t4[1].im * 2.0);
  l2_tmp = ((((((T_re - l4_tmp * t4[1].re * 2.0) - A_init_re) + b_A_init_re) +
              A_wayp_re) -
             b_A_wayp_re) -
            J_min_re) +
           l41_tmp * t4[1].re * 2.0;
  l3_tmp = (0.0 - l4_tmp * t4[1].im * 2.0) + l41_tmp * t4[1].im * 2.0;
  if (l3_tmp == 0.0) {
    if (b_l2_re_tmp_tmp == 0.0) {
      l2_re_tmp_tmp = ar / l2_tmp;
      l8_tmp = 0.0;
    } else if (ar == 0.0) {
      l2_re_tmp_tmp = 0.0;
      l8_tmp = b_l2_re_tmp_tmp / l2_tmp;
    } else {
      l2_re_tmp_tmp = ar / l2_tmp;
      l8_tmp = b_l2_re_tmp_tmp / l2_tmp;
    }
  } else if (l2_tmp == 0.0) {
    if (ar == 0.0) {
      l2_re_tmp_tmp = b_l2_re_tmp_tmp / l3_tmp;
      l8_tmp = 0.0;
    } else if (b_l2_re_tmp_tmp == 0.0) {
      l2_re_tmp_tmp = 0.0;
      l8_tmp = -(ar / l3_tmp);
    } else {
      l2_re_tmp_tmp = b_l2_re_tmp_tmp / l3_tmp;
      l8_tmp = -(ar / l3_tmp);
    }
  } else {
    l13 = std::abs(l2_tmp);
    l8_tmp = std::abs(l3_tmp);
    if (l13 > l8_tmp) {
      l2_im = l3_tmp / l2_tmp;
      l8_tmp = l2_tmp + l2_im * l3_tmp;
      l2_re_tmp_tmp = (ar + l2_im * b_l2_re_tmp_tmp) / l8_tmp;
      l8_tmp = (b_l2_re_tmp_tmp - l2_im * ar) / l8_tmp;
    } else if (l8_tmp == l13) {
      if (l2_tmp > 0.0) {
        l2_im = 0.5;
      } else {
        l2_im = -0.5;
      }
      if (l3_tmp > 0.0) {
        l8_tmp = 0.5;
      } else {
        l8_tmp = -0.5;
      }
      l2_re_tmp_tmp = (ar * l2_im + b_l2_re_tmp_tmp * l8_tmp) / l13;
      l8_tmp = (b_l2_re_tmp_tmp * l2_im - ar * l8_tmp) / l13;
    } else {
      l2_im = l2_tmp / l3_tmp;
      l8_tmp = l3_tmp + l2_im * l2_tmp;
      l2_re_tmp_tmp = (l2_im * ar + b_l2_re_tmp_tmp) / l8_tmp;
      l8_tmp = (l2_im * b_l2_re_tmp_tmp - ar) / l8_tmp;
    }
  }
  l2_im = J_max * l2_re_tmp_tmp;
  l2_tmp = J_max * l8_tmp;
  ar = (((ar_tmp - J_min * l2_re_tmp_tmp) + l2_im) - J_min * t4[1].re) +
       b_ar_tmp;
  b_l2_re_tmp_tmp = ((0.0 - J_min * l8_tmp) + l2_tmp) - J_min * t4[1].im;
  if (b_l2_re_tmp_tmp == 0.0) {
    l3_tmp = ar / y;
    l13 = 0.0;
  } else if (ar == 0.0) {
    l3_tmp = 0.0;
    l13 = b_l2_re_tmp_tmp / y;
  } else {
    l3_tmp = ar / y;
    l13 = b_l2_re_tmp_tmp / y;
  }
  t[1].re = l2_re_tmp_tmp;
  t[1].im = l8_tmp;
  t[5].re = 0.0;
  t[5].im = 0.0;
  ar = -(A_init + l2_im);
  if (-l2_tmp == 0.0) {
    t[9].re = ar / J_min;
    t[9].im = 0.0;
  } else if (ar == 0.0) {
    t[9].re = 0.0;
    t[9].im = -l2_tmp / J_min;
  } else {
    t[9].re = ar / J_min;
    t[9].im = -l2_tmp / J_min;
  }
  t[13] = t4[1];
  ar = A_wayp - J_max * l3_tmp;
  b_l2_re_tmp_tmp = 0.0 - J_max * l13;
  if (b_l2_re_tmp_tmp == 0.0) {
    t[17].re = ar / J_min;
    t[17].im = 0.0;
  } else if (ar == 0.0) {
    t[17].re = 0.0;
    t[17].im = b_l2_re_tmp_tmp / J_min;
  } else {
    t[17].re = ar / J_min;
    t[17].im = b_l2_re_tmp_tmp / J_min;
  }
  t[21].re = 0.0;
  t[21].im = 0.0;
  t[25].re = l3_tmp;
  t[25].im = l13;
  l8_tmp = t4[2].re * t4[2].im;
  ar = -(((((l35.re - l35_tmp * t4[2].re * 2.0) + b_l35_tmp * t4[2].re * 2.0) +
           l41.re) +
          l41_tmp * (t4[2].re * t4[2].re - t4[2].im * t4[2].im)) -
         J_min_tmp * t4[2].re * 2.0);
  b_l2_re_tmp_tmp =
      -((((0.0 - l35_tmp * t4[2].im * 2.0) + b_l35_tmp * t4[2].im * 2.0) +
         l41_tmp * (l8_tmp + l8_tmp)) -
        J_min_tmp * t4[2].im * 2.0);
  l2_tmp = ((((((T_re - l4_tmp * t4[2].re * 2.0) - A_init_re) + b_A_init_re) +
              A_wayp_re) -
             b_A_wayp_re) -
            J_min_re) +
           l41_tmp * t4[2].re * 2.0;
  l3_tmp = (0.0 - l4_tmp * t4[2].im * 2.0) + l41_tmp * t4[2].im * 2.0;
  if (l3_tmp == 0.0) {
    if (b_l2_re_tmp_tmp == 0.0) {
      l2_re_tmp_tmp = ar / l2_tmp;
      l8_tmp = 0.0;
    } else if (ar == 0.0) {
      l2_re_tmp_tmp = 0.0;
      l8_tmp = b_l2_re_tmp_tmp / l2_tmp;
    } else {
      l2_re_tmp_tmp = ar / l2_tmp;
      l8_tmp = b_l2_re_tmp_tmp / l2_tmp;
    }
  } else if (l2_tmp == 0.0) {
    if (ar == 0.0) {
      l2_re_tmp_tmp = b_l2_re_tmp_tmp / l3_tmp;
      l8_tmp = 0.0;
    } else if (b_l2_re_tmp_tmp == 0.0) {
      l2_re_tmp_tmp = 0.0;
      l8_tmp = -(ar / l3_tmp);
    } else {
      l2_re_tmp_tmp = b_l2_re_tmp_tmp / l3_tmp;
      l8_tmp = -(ar / l3_tmp);
    }
  } else {
    l13 = std::abs(l2_tmp);
    l8_tmp = std::abs(l3_tmp);
    if (l13 > l8_tmp) {
      l2_im = l3_tmp / l2_tmp;
      l8_tmp = l2_tmp + l2_im * l3_tmp;
      l2_re_tmp_tmp = (ar + l2_im * b_l2_re_tmp_tmp) / l8_tmp;
      l8_tmp = (b_l2_re_tmp_tmp - l2_im * ar) / l8_tmp;
    } else if (l8_tmp == l13) {
      if (l2_tmp > 0.0) {
        l2_im = 0.5;
      } else {
        l2_im = -0.5;
      }
      if (l3_tmp > 0.0) {
        l8_tmp = 0.5;
      } else {
        l8_tmp = -0.5;
      }
      l2_re_tmp_tmp = (ar * l2_im + b_l2_re_tmp_tmp * l8_tmp) / l13;
      l8_tmp = (b_l2_re_tmp_tmp * l2_im - ar * l8_tmp) / l13;
    } else {
      l2_im = l2_tmp / l3_tmp;
      l8_tmp = l3_tmp + l2_im * l2_tmp;
      l2_re_tmp_tmp = (l2_im * ar + b_l2_re_tmp_tmp) / l8_tmp;
      l8_tmp = (l2_im * b_l2_re_tmp_tmp - ar) / l8_tmp;
    }
  }
  l2_im = J_max * l2_re_tmp_tmp;
  l2_tmp = J_max * l8_tmp;
  ar = (((ar_tmp - J_min * l2_re_tmp_tmp) + l2_im) - J_min * t4[2].re) +
       b_ar_tmp;
  b_l2_re_tmp_tmp = ((0.0 - J_min * l8_tmp) + l2_tmp) - J_min * t4[2].im;
  if (b_l2_re_tmp_tmp == 0.0) {
    l3_tmp = ar / y;
    l13 = 0.0;
  } else if (ar == 0.0) {
    l3_tmp = 0.0;
    l13 = b_l2_re_tmp_tmp / y;
  } else {
    l3_tmp = ar / y;
    l13 = b_l2_re_tmp_tmp / y;
  }
  t[2].re = l2_re_tmp_tmp;
  t[2].im = l8_tmp;
  t[6].re = 0.0;
  t[6].im = 0.0;
  ar = -(A_init + l2_im);
  if (-l2_tmp == 0.0) {
    t[10].re = ar / J_min;
    t[10].im = 0.0;
  } else if (ar == 0.0) {
    t[10].re = 0.0;
    t[10].im = -l2_tmp / J_min;
  } else {
    t[10].re = ar / J_min;
    t[10].im = -l2_tmp / J_min;
  }
  t[14] = t4[2];
  ar = A_wayp - J_max * l3_tmp;
  b_l2_re_tmp_tmp = 0.0 - J_max * l13;
  if (b_l2_re_tmp_tmp == 0.0) {
    t[18].re = ar / J_min;
    t[18].im = 0.0;
  } else if (ar == 0.0) {
    t[18].re = 0.0;
    t[18].im = b_l2_re_tmp_tmp / J_min;
  } else {
    t[18].re = ar / J_min;
    t[18].im = b_l2_re_tmp_tmp / J_min;
  }
  t[22].re = 0.0;
  t[22].im = 0.0;
  t[26].re = l3_tmp;
  t[26].im = l13;
  l8_tmp = t4[3].re * t4[3].im;
  ar = -(((((l35.re - l35_tmp * t4[3].re * 2.0) + b_l35_tmp * t4[3].re * 2.0) +
           l41.re) +
          l41_tmp * (t4[3].re * t4[3].re - t4[3].im * t4[3].im)) -
         J_min_tmp * t4[3].re * 2.0);
  b_l2_re_tmp_tmp =
      -((((0.0 - l35_tmp * t4[3].im * 2.0) + b_l35_tmp * t4[3].im * 2.0) +
         l41_tmp * (l8_tmp + l8_tmp)) -
        J_min_tmp * t4[3].im * 2.0);
  l2_tmp = ((((((T_re - l4_tmp * t4[3].re * 2.0) - A_init_re) + b_A_init_re) +
              A_wayp_re) -
             b_A_wayp_re) -
            J_min_re) +
           l41_tmp * t4[3].re * 2.0;
  l3_tmp = (0.0 - l4_tmp * t4[3].im * 2.0) + l41_tmp * t4[3].im * 2.0;
  if (l3_tmp == 0.0) {
    if (b_l2_re_tmp_tmp == 0.0) {
      l2_re_tmp_tmp = ar / l2_tmp;
      l8_tmp = 0.0;
    } else if (ar == 0.0) {
      l2_re_tmp_tmp = 0.0;
      l8_tmp = b_l2_re_tmp_tmp / l2_tmp;
    } else {
      l2_re_tmp_tmp = ar / l2_tmp;
      l8_tmp = b_l2_re_tmp_tmp / l2_tmp;
    }
  } else if (l2_tmp == 0.0) {
    if (ar == 0.0) {
      l2_re_tmp_tmp = b_l2_re_tmp_tmp / l3_tmp;
      l8_tmp = 0.0;
    } else if (b_l2_re_tmp_tmp == 0.0) {
      l2_re_tmp_tmp = 0.0;
      l8_tmp = -(ar / l3_tmp);
    } else {
      l2_re_tmp_tmp = b_l2_re_tmp_tmp / l3_tmp;
      l8_tmp = -(ar / l3_tmp);
    }
  } else {
    l13 = std::abs(l2_tmp);
    l8_tmp = std::abs(l3_tmp);
    if (l13 > l8_tmp) {
      l2_im = l3_tmp / l2_tmp;
      l8_tmp = l2_tmp + l2_im * l3_tmp;
      l2_re_tmp_tmp = (ar + l2_im * b_l2_re_tmp_tmp) / l8_tmp;
      l8_tmp = (b_l2_re_tmp_tmp - l2_im * ar) / l8_tmp;
    } else if (l8_tmp == l13) {
      if (l2_tmp > 0.0) {
        l2_im = 0.5;
      } else {
        l2_im = -0.5;
      }
      if (l3_tmp > 0.0) {
        l8_tmp = 0.5;
      } else {
        l8_tmp = -0.5;
      }
      l2_re_tmp_tmp = (ar * l2_im + b_l2_re_tmp_tmp * l8_tmp) / l13;
      l8_tmp = (b_l2_re_tmp_tmp * l2_im - ar * l8_tmp) / l13;
    } else {
      l2_im = l2_tmp / l3_tmp;
      l8_tmp = l3_tmp + l2_im * l2_tmp;
      l2_re_tmp_tmp = (l2_im * ar + b_l2_re_tmp_tmp) / l8_tmp;
      l8_tmp = (l2_im * b_l2_re_tmp_tmp - ar) / l8_tmp;
    }
  }
  l2_im = J_max * l2_re_tmp_tmp;
  l2_tmp = J_max * l8_tmp;
  ar = (((ar_tmp - J_min * l2_re_tmp_tmp) + l2_im) - J_min * t4[3].re) +
       b_ar_tmp;
  b_l2_re_tmp_tmp = ((0.0 - J_min * l8_tmp) + l2_tmp) - J_min * t4[3].im;
  if (b_l2_re_tmp_tmp == 0.0) {
    l3_tmp = ar / y;
    l13 = 0.0;
  } else if (ar == 0.0) {
    l3_tmp = 0.0;
    l13 = b_l2_re_tmp_tmp / y;
  } else {
    l3_tmp = ar / y;
    l13 = b_l2_re_tmp_tmp / y;
  }
  t[3].re = l2_re_tmp_tmp;
  t[3].im = l8_tmp;
  t[7].re = 0.0;
  t[7].im = 0.0;
  ar = -(A_init + l2_im);
  if (-l2_tmp == 0.0) {
    t[11].re = ar / J_min;
    t[11].im = 0.0;
  } else if (ar == 0.0) {
    t[11].re = 0.0;
    t[11].im = -l2_tmp / J_min;
  } else {
    t[11].re = ar / J_min;
    t[11].im = -l2_tmp / J_min;
  }
  t[15] = t4[3];
  ar = A_wayp - J_max * l3_tmp;
  b_l2_re_tmp_tmp = 0.0 - J_max * l13;
  if (b_l2_re_tmp_tmp == 0.0) {
    t[19].re = ar / J_min;
    t[19].im = 0.0;
  } else if (ar == 0.0) {
    t[19].re = 0.0;
    t[19].im = b_l2_re_tmp_tmp / J_min;
  } else {
    t[19].re = ar / J_min;
    t[19].im = b_l2_re_tmp_tmp / J_min;
  }
  t[23].re = 0.0;
  t[23].im = 0.0;
  t[27].re = l3_tmp;
  t[27].im = l13;
}

// End of code generation (acdeg_T_AV.cpp)
