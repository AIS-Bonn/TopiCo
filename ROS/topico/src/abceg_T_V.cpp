//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// abceg_T_V.cpp
//
// Code generation for function 'abceg_T_V'
//

// Include files
#include "abceg_T_V.h"
#include "rt_nonfinite.h"
#include "sqrt.h"
#include "topico_wrapper_data.h"
#include "topico_wrapper_rtwutil.h"
#include "topico_wrapper_types.h"
#include <cmath>

// Function Definitions
void abceg_T_V(double V_init, double A_init, double V_wayp, double V_min,
               double A_max, double J_max, double J_min, double T,
               creal_T t[28])
{
  creal_T c_y[4];
  creal_T l2[4];
  creal_T l3[4];
  creal_T t3[4];
  creal_T t7[4];
  creal_T t7_tmp[4];
  creal_T l29;
  double A_max_re;
  double ar;
  double b_y;
  double d;
  double d1;
  double d2;
  double d3;
  double d4;
  double d5;
  double im;
  double im_tmp;
  double l11_tmp;
  double l11_tmp_tmp;
  double l16;
  double l20;
  double l21_im;
  double l21_re;
  double l29_tmp;
  double l2_tmp;
  double l30_im;
  double l30_re;
  double l4_tmp;
  double l5_tmp;
  double l6_tmp;
  double l9;
  double re;
  double re_tmp;
  double re_tmp_tmp;
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
  //  Generated on 02-Sep-2019 15:40:56
  l2_tmp = A_init * A_init;
  l5_tmp = J_max * V_init * 2.0;
  l6_tmp = J_max * V_min * 2.0;
  l9 = A_init * A_max * 2.0;
  l11_tmp_tmp = A_max * J_max;
  l11_tmp = l11_tmp_tmp * T * 2.0;
  if (J_max < 0.0) {
    f_rtErrorWithMessageID("sqrt", n_emlrtRTEI.fName, n_emlrtRTEI.lineNo);
  }
  l16 = J_min + -J_max;
  l20 = 1.0 / (J_min * J_min + -(J_min * J_max));
  l29.re = V_min + -V_wayp;
  l29.im = 0.0;
  coder::internal::scalar::b_sqrt(&l29);
  l21_im = A_max * 1.4142135623730951 * std::sqrt(J_max);
  A_max_re = l21_im * l29.re;
  l21_im *= l29.im;
  l21_re = A_max_re * 0.0 - l21_im * 2.0;
  l21_im = A_max_re * 2.0 + l21_im * 0.0;
  l29_tmp = -J_min * l16;
  l29.re = l29_tmp * (((((l2_tmp - l5_tmp) + l6_tmp) - l9) - l11_tmp) + l21_re);
  l29.im = l29_tmp * l21_im;
  coder::internal::scalar::b_sqrt(&l29);
  l30_re = l20 * l29.re;
  l30_im = l20 * l29.im;
  l29_tmp = J_min * l16;
  l29.re =
      l29_tmp * (((((l5_tmp + l9) + -l6_tmp) + l11_tmp) + -l2_tmp) + l21_re);
  l29.im = l29_tmp * l21_im;
  coder::internal::scalar::b_sqrt(&l29);
  l29.re *= l20;
  l29.im *= l20;
  t3[0].re = l30_re;
  t3[0].im = l30_im;
  t3[1] = l29;
  t3[2].re = -l30_re;
  t3[2].im = -l30_im;
  t3[3].re = -l29.re;
  t3[3].im = -l29.im;
  l29_tmp = l5_tmp - l6_tmp;
  l29.re = (l29_tmp + l2_tmp) - A_max * A_max;
  l21_re = A_init * (A_init - A_max) * 2.0;
  l4_tmp = A_init + -A_max;
  y = l4_tmp * l4_tmp;
  b_y = J_max * J_max;
  l5_tmp = J_min * t3[0].re;
  im = J_min * t3[0].im;
  l2[0].re = l5_tmp;
  l2[0].im = im;
  re_tmp = A_max + l5_tmp;
  l20 = J_max * l5_tmp;
  im_tmp = J_max * im;
  l21_im = re_tmp * im;
  ar = -((((l29.re - (re_tmp * re_tmp - im * im)) - l21_re) + l11_tmp) +
         (l20 * t3[0].re - im_tmp * t3[0].im));
  A_max_re =
      -((0.0 - (l21_im + l21_im)) + (l20 * t3[0].im + im_tmp * t3[0].re));
  l30_im = l20 * 2.0 - J_max * re_tmp * 2.0;
  l2_tmp = im_tmp * 2.0 - im_tmp * 2.0;
  if (l2_tmp == 0.0) {
    if (A_max_re == 0.0) {
      re = ar / l30_im;
      l16 = 0.0;
    } else if (ar == 0.0) {
      re = 0.0;
      l16 = A_max_re / l30_im;
    } else {
      re = ar / l30_im;
      l16 = A_max_re / l30_im;
    }
  } else if (l30_im == 0.0) {
    if (ar == 0.0) {
      re = A_max_re / l2_tmp;
      l16 = 0.0;
    } else if (A_max_re == 0.0) {
      re = 0.0;
      l16 = -(ar / l2_tmp);
    } else {
      re = A_max_re / l2_tmp;
      l16 = -(ar / l2_tmp);
    }
  } else {
    l5_tmp = std::abs(l30_im);
    l21_im = std::abs(l2_tmp);
    if (l5_tmp > l21_im) {
      l30_re = l2_tmp / l30_im;
      l21_im = l30_im + l30_re * l2_tmp;
      re = (ar + l30_re * A_max_re) / l21_im;
      l16 = (A_max_re - l30_re * ar) / l21_im;
    } else if (l21_im == l5_tmp) {
      if (l30_im > 0.0) {
        l30_re = 0.5;
      } else {
        l30_re = -0.5;
      }
      if (l2_tmp > 0.0) {
        l21_im = 0.5;
      } else {
        l21_im = -0.5;
      }
      re = (ar * l30_re + A_max_re * l21_im) / l5_tmp;
      l16 = (A_max_re * l30_re - ar * l21_im) / l5_tmp;
    } else {
      l30_re = l30_im / l2_tmp;
      l21_im = l2_tmp + l30_re * l30_im;
      re = (l30_re * ar + A_max_re) / l21_im;
      l16 = (l30_re * A_max_re - ar) / l21_im;
    }
  }
  t7[0].re = re;
  t7[0].im = l16;
  l20 = J_max * re;
  l9 = J_max * l16;
  t7_tmp[0].re = l20;
  t7_tmp[0].im = l9;
  l5_tmp = re_tmp + l20;
  l6_tmp = im + l9;
  c_y[0].re = l5_tmp * l5_tmp - l6_tmp * l6_tmp;
  d = l5_tmp * l6_tmp;
  l3[0].re = re * re - l16 * l16;
  d1 = re * l16;
  l5_tmp = J_min * t3[1].re;
  im = J_min * t3[1].im;
  l2[1].re = l5_tmp;
  l2[1].im = im;
  re_tmp = A_max + l5_tmp;
  l20 = J_max * l5_tmp;
  im_tmp = J_max * im;
  l21_im = re_tmp * im;
  ar = -((((l29.re - (re_tmp * re_tmp - im * im)) - l21_re) + l11_tmp) +
         (l20 * t3[1].re - im_tmp * t3[1].im));
  A_max_re =
      -((0.0 - (l21_im + l21_im)) + (l20 * t3[1].im + im_tmp * t3[1].re));
  l30_im = l20 * 2.0 - J_max * re_tmp * 2.0;
  l2_tmp = im_tmp * 2.0 - im_tmp * 2.0;
  if (l2_tmp == 0.0) {
    if (A_max_re == 0.0) {
      re = ar / l30_im;
      l16 = 0.0;
    } else if (ar == 0.0) {
      re = 0.0;
      l16 = A_max_re / l30_im;
    } else {
      re = ar / l30_im;
      l16 = A_max_re / l30_im;
    }
  } else if (l30_im == 0.0) {
    if (ar == 0.0) {
      re = A_max_re / l2_tmp;
      l16 = 0.0;
    } else if (A_max_re == 0.0) {
      re = 0.0;
      l16 = -(ar / l2_tmp);
    } else {
      re = A_max_re / l2_tmp;
      l16 = -(ar / l2_tmp);
    }
  } else {
    l5_tmp = std::abs(l30_im);
    l21_im = std::abs(l2_tmp);
    if (l5_tmp > l21_im) {
      l30_re = l2_tmp / l30_im;
      l21_im = l30_im + l30_re * l2_tmp;
      re = (ar + l30_re * A_max_re) / l21_im;
      l16 = (A_max_re - l30_re * ar) / l21_im;
    } else if (l21_im == l5_tmp) {
      if (l30_im > 0.0) {
        l30_re = 0.5;
      } else {
        l30_re = -0.5;
      }
      if (l2_tmp > 0.0) {
        l21_im = 0.5;
      } else {
        l21_im = -0.5;
      }
      re = (ar * l30_re + A_max_re * l21_im) / l5_tmp;
      l16 = (A_max_re * l30_re - ar * l21_im) / l5_tmp;
    } else {
      l30_re = l30_im / l2_tmp;
      l21_im = l2_tmp + l30_re * l30_im;
      re = (l30_re * ar + A_max_re) / l21_im;
      l16 = (l30_re * A_max_re - ar) / l21_im;
    }
  }
  t7[1].re = re;
  t7[1].im = l16;
  l20 = J_max * re;
  l9 = J_max * l16;
  t7_tmp[1].re = l20;
  t7_tmp[1].im = l9;
  l5_tmp = re_tmp + l20;
  l6_tmp = im + l9;
  c_y[1].re = l5_tmp * l5_tmp - l6_tmp * l6_tmp;
  d2 = l5_tmp * l6_tmp;
  l3[1].re = re * re - l16 * l16;
  d3 = re * l16;
  l5_tmp = J_min * t3[2].re;
  im = J_min * t3[2].im;
  l2[2].re = l5_tmp;
  l2[2].im = im;
  re_tmp = A_max + l5_tmp;
  l20 = J_max * l5_tmp;
  im_tmp = J_max * im;
  l21_im = re_tmp * im;
  ar = -((((l29.re - (re_tmp * re_tmp - im * im)) - l21_re) + l11_tmp) +
         (l20 * t3[2].re - im_tmp * t3[2].im));
  A_max_re =
      -((0.0 - (l21_im + l21_im)) + (l20 * t3[2].im + im_tmp * t3[2].re));
  l30_im = l20 * 2.0 - J_max * re_tmp * 2.0;
  l2_tmp = im_tmp * 2.0 - im_tmp * 2.0;
  if (l2_tmp == 0.0) {
    if (A_max_re == 0.0) {
      re = ar / l30_im;
      l16 = 0.0;
    } else if (ar == 0.0) {
      re = 0.0;
      l16 = A_max_re / l30_im;
    } else {
      re = ar / l30_im;
      l16 = A_max_re / l30_im;
    }
  } else if (l30_im == 0.0) {
    if (ar == 0.0) {
      re = A_max_re / l2_tmp;
      l16 = 0.0;
    } else if (A_max_re == 0.0) {
      re = 0.0;
      l16 = -(ar / l2_tmp);
    } else {
      re = A_max_re / l2_tmp;
      l16 = -(ar / l2_tmp);
    }
  } else {
    l5_tmp = std::abs(l30_im);
    l21_im = std::abs(l2_tmp);
    if (l5_tmp > l21_im) {
      l30_re = l2_tmp / l30_im;
      l21_im = l30_im + l30_re * l2_tmp;
      re = (ar + l30_re * A_max_re) / l21_im;
      l16 = (A_max_re - l30_re * ar) / l21_im;
    } else if (l21_im == l5_tmp) {
      if (l30_im > 0.0) {
        l30_re = 0.5;
      } else {
        l30_re = -0.5;
      }
      if (l2_tmp > 0.0) {
        l21_im = 0.5;
      } else {
        l21_im = -0.5;
      }
      re = (ar * l30_re + A_max_re * l21_im) / l5_tmp;
      l16 = (A_max_re * l30_re - ar * l21_im) / l5_tmp;
    } else {
      l30_re = l30_im / l2_tmp;
      l21_im = l2_tmp + l30_re * l30_im;
      re = (l30_re * ar + A_max_re) / l21_im;
      l16 = (l30_re * A_max_re - ar) / l21_im;
    }
  }
  t7[2].re = re;
  t7[2].im = l16;
  l20 = J_max * re;
  l9 = J_max * l16;
  t7_tmp[2].re = l20;
  t7_tmp[2].im = l9;
  l5_tmp = re_tmp + l20;
  l6_tmp = im + l9;
  c_y[2].re = l5_tmp * l5_tmp - l6_tmp * l6_tmp;
  d4 = l5_tmp * l6_tmp;
  l3[2].re = re * re - l16 * l16;
  d5 = re * l16;
  l5_tmp = J_min * t3[3].re;
  im = J_min * t3[3].im;
  re_tmp_tmp = A_max + l5_tmp;
  re_tmp = J_max * l5_tmp;
  im_tmp = J_max * im;
  l21_im = re_tmp_tmp * im;
  ar = -((((l29.re - (re_tmp_tmp * re_tmp_tmp - im * im)) - l21_re) + l11_tmp) +
         (re_tmp * t3[3].re - im_tmp * t3[3].im));
  A_max_re =
      -((0.0 - (l21_im + l21_im)) + (re_tmp * t3[3].im + im_tmp * t3[3].re));
  l30_im = re_tmp * 2.0 - J_max * re_tmp_tmp * 2.0;
  l2_tmp = im_tmp * 2.0 - im_tmp * 2.0;
  if (l2_tmp == 0.0) {
    if (A_max_re == 0.0) {
      re = ar / l30_im;
      l16 = 0.0;
    } else if (ar == 0.0) {
      re = 0.0;
      l16 = A_max_re / l30_im;
    } else {
      re = ar / l30_im;
      l16 = A_max_re / l30_im;
    }
  } else if (l30_im == 0.0) {
    if (ar == 0.0) {
      re = A_max_re / l2_tmp;
      l16 = 0.0;
    } else if (A_max_re == 0.0) {
      re = 0.0;
      l16 = -(ar / l2_tmp);
    } else {
      re = A_max_re / l2_tmp;
      l16 = -(ar / l2_tmp);
    }
  } else {
    l5_tmp = std::abs(l30_im);
    l21_im = std::abs(l2_tmp);
    if (l5_tmp > l21_im) {
      l30_re = l2_tmp / l30_im;
      l21_im = l30_im + l30_re * l2_tmp;
      re = (ar + l30_re * A_max_re) / l21_im;
      l16 = (A_max_re - l30_re * ar) / l21_im;
    } else if (l21_im == l5_tmp) {
      if (l30_im > 0.0) {
        l30_re = 0.5;
      } else {
        l30_re = -0.5;
      }
      if (l2_tmp > 0.0) {
        l21_im = 0.5;
      } else {
        l21_im = -0.5;
      }
      re = (ar * l30_re + A_max_re * l21_im) / l5_tmp;
      l16 = (A_max_re * l30_re - ar * l21_im) / l5_tmp;
    } else {
      l30_re = l30_im / l2_tmp;
      l21_im = l2_tmp + l30_re * l30_im;
      re = (l30_re * ar + A_max_re) / l21_im;
      l16 = (l30_re * A_max_re - ar) / l21_im;
    }
  }
  t7[3].re = re;
  t7[3].im = l16;
  l20 = J_max * re;
  l9 = J_max * l16;
  l5_tmp = re_tmp_tmp + l20;
  l6_tmp = im + l9;
  l30_im = l5_tmp * l6_tmp;
  l2_tmp = re * l16;
  l21_im = -(1.0 / J_max * l4_tmp);
  l29.re = l29_tmp - A_init * l4_tmp * 2.0;
  t[0].re = l21_im;
  t[0].im = 0.0;
  t[1].re = l21_im;
  t[1].im = 0.0;
  t[2].re = l21_im;
  t[2].im = 0.0;
  t[3].re = l21_im;
  t[3].im = 0.0;
  A_max_re = A_max + l2[0].re;
  l21_im = J_max * l2[0].re;
  l30_re = J_max * l2[0].im;
  ar = ((((((l29.re - c_y[0].re) + y) + b_y * l3[0].re) +
          (t7_tmp[0].re * A_max_re - t7_tmp[0].im * l2[0].im) * 2.0) +
         l11_tmp_tmp * t3[0].re * 2.0) +
        (l21_im * t3[0].re - l30_re * t3[0].im)) *
       -0.5;
  A_max_re = (((((0.0 - (d + d)) + b_y * (d1 + d1)) +
                (t7_tmp[0].re * l2[0].im + t7_tmp[0].im * A_max_re) * 2.0) +
               l11_tmp_tmp * t3[0].im * 2.0) +
              (l21_im * t3[0].im + l30_re * t3[0].re)) *
             -0.5;
  if (A_max_re == 0.0) {
    t[4].re = ar / l11_tmp_tmp;
    t[4].im = 0.0;
  } else if (ar == 0.0) {
    t[4].re = 0.0;
    t[4].im = A_max_re / l11_tmp_tmp;
  } else {
    t[4].re = ar / l11_tmp_tmp;
    t[4].im = A_max_re / l11_tmp_tmp;
  }
  t[8] = t3[0];
  t[12].re = 0.0;
  t[12].im = 0.0;
  t[16].re = 0.0;
  t[16].im = 0.0;
  t[20].re = 0.0;
  t[20].im = 0.0;
  t[24] = t7[0];
  A_max_re = A_max + l2[1].re;
  l21_im = J_max * l2[1].re;
  l30_re = J_max * l2[1].im;
  ar = ((((((l29.re - c_y[1].re) + y) + b_y * l3[1].re) +
          (t7_tmp[1].re * A_max_re - t7_tmp[1].im * l2[1].im) * 2.0) +
         l11_tmp_tmp * t3[1].re * 2.0) +
        (l21_im * t3[1].re - l30_re * t3[1].im)) *
       -0.5;
  A_max_re = (((((0.0 - (d2 + d2)) + b_y * (d3 + d3)) +
                (t7_tmp[1].re * l2[1].im + t7_tmp[1].im * A_max_re) * 2.0) +
               l11_tmp_tmp * t3[1].im * 2.0) +
              (l21_im * t3[1].im + l30_re * t3[1].re)) *
             -0.5;
  if (A_max_re == 0.0) {
    t[5].re = ar / l11_tmp_tmp;
    t[5].im = 0.0;
  } else if (ar == 0.0) {
    t[5].re = 0.0;
    t[5].im = A_max_re / l11_tmp_tmp;
  } else {
    t[5].re = ar / l11_tmp_tmp;
    t[5].im = A_max_re / l11_tmp_tmp;
  }
  t[9] = t3[1];
  t[13].re = 0.0;
  t[13].im = 0.0;
  t[17].re = 0.0;
  t[17].im = 0.0;
  t[21].re = 0.0;
  t[21].im = 0.0;
  t[25] = t7[1];
  A_max_re = A_max + l2[2].re;
  l21_im = J_max * l2[2].re;
  l30_re = J_max * l2[2].im;
  ar = ((((((l29.re - c_y[2].re) + y) + b_y * l3[2].re) +
          (t7_tmp[2].re * A_max_re - t7_tmp[2].im * l2[2].im) * 2.0) +
         l11_tmp_tmp * t3[2].re * 2.0) +
        (l21_im * t3[2].re - l30_re * t3[2].im)) *
       -0.5;
  A_max_re = (((((0.0 - (d4 + d4)) + b_y * (d5 + d5)) +
                (t7_tmp[2].re * l2[2].im + t7_tmp[2].im * A_max_re) * 2.0) +
               l11_tmp_tmp * t3[2].im * 2.0) +
              (l21_im * t3[2].im + l30_re * t3[2].re)) *
             -0.5;
  if (A_max_re == 0.0) {
    t[6].re = ar / l11_tmp_tmp;
    t[6].im = 0.0;
  } else if (ar == 0.0) {
    t[6].re = 0.0;
    t[6].im = A_max_re / l11_tmp_tmp;
  } else {
    t[6].re = ar / l11_tmp_tmp;
    t[6].im = A_max_re / l11_tmp_tmp;
  }
  t[10] = t3[2];
  t[14].re = 0.0;
  t[14].im = 0.0;
  t[18].re = 0.0;
  t[18].im = 0.0;
  t[22].re = 0.0;
  t[22].im = 0.0;
  t[26] = t7[2];
  ar = ((((((l29.re - (l5_tmp * l5_tmp - l6_tmp * l6_tmp)) + y) +
           b_y * (re * re - l16 * l16)) +
          (l20 * re_tmp_tmp - l9 * im) * 2.0) +
         l11_tmp_tmp * t3[3].re * 2.0) +
        (re_tmp * t3[3].re - im_tmp * t3[3].im)) *
       -0.5;
  A_max_re = (((((0.0 - (l30_im + l30_im)) + b_y * (l2_tmp + l2_tmp)) +
                (l20 * im + l9 * re_tmp_tmp) * 2.0) +
               l11_tmp_tmp * t3[3].im * 2.0) +
              (re_tmp * t3[3].im + im_tmp * t3[3].re)) *
             -0.5;
  if (A_max_re == 0.0) {
    t[7].re = ar / l11_tmp_tmp;
    t[7].im = 0.0;
  } else if (ar == 0.0) {
    t[7].re = 0.0;
    t[7].im = A_max_re / l11_tmp_tmp;
  } else {
    t[7].re = ar / l11_tmp_tmp;
    t[7].im = A_max_re / l11_tmp_tmp;
  }
  t[11] = t3[3];
  t[15].re = 0.0;
  t[15].im = 0.0;
  t[19].re = 0.0;
  t[19].im = 0.0;
  t[23].re = 0.0;
  t[23].im = 0.0;
  t[27] = t7[3];
}

// End of code generation (abceg_T_V.cpp)
