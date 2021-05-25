//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// acde_NO_VP.cpp
//
// Code generation for function 'acde_NO_VP'
//

// Include files
#include "acde_NO_VP.h"
#include "power.h"
#include "rt_nonfinite.h"
#include "sqrt.h"
#include "topico_wrapper_data.h"
#include "topico_wrapper_rtwutil.h"
#include "topico_wrapper_types.h"
#include <cmath>

// Function Definitions
void acde_NO_VP(double P_init, double V_init, double A_init, double P_wayp,
                double V_wayp, double V_max, double J_max, double J_min,
                creal_T t[14])
{
  creal_T dcv[2];
  creal_T l2[2];
  creal_T l6[2];
  creal_T t3[2];
  creal_T t4_tmp[2];
  creal_T t5[2];
  creal_T l17;
  double A_init_re;
  double J_max_tmp;
  double J_min_re;
  double J_min_tmp;
  double V_init_tmp;
  double ai;
  double ar;
  double b_J_max;
  double b_J_min;
  double b_J_min_tmp;
  double b_l17_tmp;
  double b_re;
  double b_y;
  double br;
  double im;
  double l17_tmp;
  double l7;
  double re;
  double y;
  double y_re;
  double y_tmp;
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
  l17.re = -((J_min + -J_max) *
             ((l17_tmp + J_min * V_max * 2.0) + -(b_l17_tmp * 2.0)));
  l17.im = 0.0;
  coder::internal::scalar::b_sqrt(&l17);
  br = rt_powd_snf(l7, 3.0) - J_min * l7;
  b_J_min = -1.0 / br;
  t3[0].re = b_J_min * l17.re;
  t3[0].im = b_J_min * l17.im;
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
  y_tmp = J_max * J_max;
  re = t3[0].re * t3[0].re - t3[0].im * t3[0].im;
  l7 = t3[0].re * t3[0].im;
  br = l7 + l7;
  l2[0].re = re;
  l2[0].im = br;
  b_J_min = J_min * J_max;
  b_l17_tmp = b_l17_tmp * -2.0 + J_min * V_wayp * 2.0;
  t5[0].re = ((b_l17_tmp - y_tmp * re) + l17_tmp) + b_J_min * re;
  t5[0].im = (0.0 - y_tmp * br) + b_J_min * br;
  re = t3[1].re * t3[1].re - t3[1].im * t3[1].im;
  l7 = t3[1].re * t3[1].im;
  br = l7 + l7;
  t5[1].re = ((b_l17_tmp - y_tmp * re) + l17_tmp) + b_J_min * re;
  t5[1].im = (0.0 - y_tmp * br) + b_J_min * br;
  coder::internal::scalar::b_sqrt(&t5[0]);
  coder::internal::scalar::b_sqrt(&t5[1]);
  l7 = J_min * J_min;
  coder::power(t3, l6);
  J_max_tmp = J_max * l7;
  y = rt_powd_snf(J_max, 3.0);
  b_y = rt_powd_snf(J_min, 3.0);
  l17.re = P_init * l7 * 6.0 - P_wayp * l7 * 6.0;
  y_re = rt_powd_snf(A_init, 3.0) * 2.0;
  A_init_re = A_init * J_min * V_init * 6.0;
  J_min_tmp = J_min * y_tmp;
  b_J_min_tmp = J_min * l17_tmp;
  b_J_max = J_max * l17_tmp;
  V_init_tmp = V_init * l7;
  b_J_min *= V_init;
  J_min_re = b_J_min_tmp * 3.0 - V_init_tmp * 6.0;
  if (-t5[0].im == 0.0) {
    b_re = -t5[0].re / J_min;
    im = 0.0;
  } else if (-t5[0].re == 0.0) {
    b_re = 0.0;
    im = -t5[0].im / J_min;
  } else {
    b_re = -t5[0].re / J_min;
    im = -t5[0].im / J_min;
  }
  t5[0].re = b_re;
  t5[0].im = im;
  t4_tmp[0].re = J_min_tmp * l2[0].re;
  t4_tmp[0].im = J_min_tmp * l2[0].im;
  l2[0].re *= J_max_tmp;
  l2[0].im *= J_max_tmp;
  ar = -(A_init + J_max * t3[0].re);
  ai = -(J_max * t3[0].im);
  if (ai == 0.0) {
    t[0].re = ar / J_min;
    t[0].im = 0.0;
  } else if (ar == 0.0) {
    t[0].re = 0.0;
    t[0].im = ai / J_min;
  } else {
    t[0].re = ar / J_min;
    t[0].im = ai / J_min;
  }
  t[2].re = 0.0;
  t[2].im = 0.0;
  t[4] = t3[0];
  if (-t5[1].im == 0.0) {
    b_re = -t5[1].re / J_min;
    im = 0.0;
  } else if (-t5[1].re == 0.0) {
    b_re = 0.0;
    im = -t5[1].im / J_min;
  } else {
    b_re = -t5[1].re / J_min;
    im = -t5[1].im / J_min;
  }
  t5[1].re = b_re;
  t5[1].im = im;
  t4_tmp[1].re = J_min_tmp * re;
  t4_tmp[1].im = J_min_tmp * br;
  l2[1].re = J_max_tmp * re;
  l2[1].im = J_max_tmp * br;
  ar = -(A_init + J_max * t3[1].re);
  ai = -(J_max * t3[1].im);
  if (ai == 0.0) {
    t[1].re = ar / J_min;
    t[1].im = 0.0;
  } else if (ar == 0.0) {
    t[1].re = 0.0;
    t[1].im = ai / J_min;
  } else {
    t[1].re = ar / J_min;
    t[1].im = ai / J_min;
  }
  t[3].re = 0.0;
  t[3].im = 0.0;
  t[5] = t3[1];
  coder::power(t5, dcv);
  ar = (((((((((((((l17.re - y * l6[0].re) + y_re) + b_y * dcv[0].re) -
                 A_init_re) +
                J_min_tmp * l6[0].re * 3.0) -
               J_max_tmp * l6[0].re * 2.0) -
              b_J_min_tmp * t3[0].re * 3.0) -
             b_J_min_tmp * t5[0].re * 3.0) +
            b_J_max * t3[0].re * 3.0) +
           V_init_tmp * t3[0].re * 6.0) +
          V_init_tmp * t5[0].re * 6.0) +
         (t4_tmp[0].re * t5[0].re - t4_tmp[0].im * t5[0].im) * 3.0) -
        (l2[0].re * t5[0].re - l2[0].im * t5[0].im) * 3.0) -
       b_J_min * t3[0].re * 6.0;
  ai = (((((((((((0.0 - y * l6[0].im) + b_y * dcv[0].im) +
                J_min_tmp * l6[0].im * 3.0) -
               J_max_tmp * l6[0].im * 2.0) -
              b_J_min_tmp * t3[0].im * 3.0) -
             b_J_min_tmp * t5[0].im * 3.0) +
            b_J_max * t3[0].im * 3.0) +
           V_init_tmp * t3[0].im * 6.0) +
          V_init_tmp * t5[0].im * 6.0) +
         (t4_tmp[0].re * t5[0].im + t4_tmp[0].im * t5[0].re) * 3.0) -
        (l2[0].re * t5[0].im + l2[0].im * t5[0].re) * 3.0) -
       b_J_min * t3[0].im * 6.0;
  br = (J_min_re - t4_tmp[0].re * 3.0) + l2[0].re * 3.0;
  y_tmp = (0.0 - t4_tmp[0].im * 3.0) + l2[0].im * 3.0;
  if (y_tmp == 0.0) {
    if (ai == 0.0) {
      t[6].re = ar / br;
      t[6].im = 0.0;
    } else if (ar == 0.0) {
      t[6].re = 0.0;
      t[6].im = ai / br;
    } else {
      t[6].re = ar / br;
      t[6].im = ai / br;
    }
  } else if (br == 0.0) {
    if (ar == 0.0) {
      t[6].re = ai / y_tmp;
      t[6].im = 0.0;
    } else if (ai == 0.0) {
      t[6].re = 0.0;
      t[6].im = -(ar / y_tmp);
    } else {
      t[6].re = ai / y_tmp;
      t[6].im = -(ar / y_tmp);
    }
  } else {
    l17_tmp = std::abs(br);
    l7 = std::abs(y_tmp);
    if (l17_tmp > l7) {
      b_l17_tmp = y_tmp / br;
      l7 = br + b_l17_tmp * y_tmp;
      t[6].re = (ar + b_l17_tmp * ai) / l7;
      t[6].im = (ai - b_l17_tmp * ar) / l7;
    } else if (l7 == l17_tmp) {
      if (br > 0.0) {
        b_l17_tmp = 0.5;
      } else {
        b_l17_tmp = -0.5;
      }
      if (y_tmp > 0.0) {
        l7 = 0.5;
      } else {
        l7 = -0.5;
      }
      t[6].re = (ar * b_l17_tmp + ai * l7) / l17_tmp;
      t[6].im = (ai * b_l17_tmp - ar * l7) / l17_tmp;
    } else {
      b_l17_tmp = br / y_tmp;
      l7 = y_tmp + b_l17_tmp * br;
      t[6].re = (b_l17_tmp * ar + ai) / l7;
      t[6].im = (b_l17_tmp * ai - ar) / l7;
    }
  }
  t[8] = t5[0];
  t[10].re = 0.0;
  t[10].im = 0.0;
  t[12].re = 0.0;
  t[12].im = 0.0;
  ar = (((((((((((((l17.re - y * l6[1].re) + y_re) + b_y * dcv[1].re) -
                 A_init_re) +
                J_min_tmp * l6[1].re * 3.0) -
               J_max_tmp * l6[1].re * 2.0) -
              b_J_min_tmp * t3[1].re * 3.0) -
             b_J_min_tmp * b_re * 3.0) +
            b_J_max * t3[1].re * 3.0) +
           V_init_tmp * t3[1].re * 6.0) +
          V_init_tmp * b_re * 6.0) +
         (t4_tmp[1].re * b_re - t4_tmp[1].im * im) * 3.0) -
        (l2[1].re * b_re - l2[1].im * im) * 3.0) -
       b_J_min * t3[1].re * 6.0;
  ai = (((((((((((0.0 - y * l6[1].im) + b_y * dcv[1].im) +
                J_min_tmp * l6[1].im * 3.0) -
               J_max_tmp * l6[1].im * 2.0) -
              b_J_min_tmp * t3[1].im * 3.0) -
             b_J_min_tmp * im * 3.0) +
            b_J_max * t3[1].im * 3.0) +
           V_init_tmp * t3[1].im * 6.0) +
          V_init_tmp * im * 6.0) +
         (t4_tmp[1].re * im + t4_tmp[1].im * b_re) * 3.0) -
        (l2[1].re * im + l2[1].im * b_re) * 3.0) -
       b_J_min * t3[1].im * 6.0;
  br = (J_min_re - t4_tmp[1].re * 3.0) + l2[1].re * 3.0;
  y_tmp = (0.0 - t4_tmp[1].im * 3.0) + l2[1].im * 3.0;
  if (y_tmp == 0.0) {
    if (ai == 0.0) {
      t[7].re = ar / br;
      t[7].im = 0.0;
    } else if (ar == 0.0) {
      t[7].re = 0.0;
      t[7].im = ai / br;
    } else {
      t[7].re = ar / br;
      t[7].im = ai / br;
    }
  } else if (br == 0.0) {
    if (ar == 0.0) {
      t[7].re = ai / y_tmp;
      t[7].im = 0.0;
    } else if (ai == 0.0) {
      t[7].re = 0.0;
      t[7].im = -(ar / y_tmp);
    } else {
      t[7].re = ai / y_tmp;
      t[7].im = -(ar / y_tmp);
    }
  } else {
    l17_tmp = std::abs(br);
    l7 = std::abs(y_tmp);
    if (l17_tmp > l7) {
      b_l17_tmp = y_tmp / br;
      l7 = br + b_l17_tmp * y_tmp;
      t[7].re = (ar + b_l17_tmp * ai) / l7;
      t[7].im = (ai - b_l17_tmp * ar) / l7;
    } else if (l7 == l17_tmp) {
      if (br > 0.0) {
        b_l17_tmp = 0.5;
      } else {
        b_l17_tmp = -0.5;
      }
      if (y_tmp > 0.0) {
        l7 = 0.5;
      } else {
        l7 = -0.5;
      }
      t[7].re = (ar * b_l17_tmp + ai * l7) / l17_tmp;
      t[7].im = (ai * b_l17_tmp - ar * l7) / l17_tmp;
    } else {
      b_l17_tmp = br / y_tmp;
      l7 = y_tmp + b_l17_tmp * br;
      t[7].re = (b_l17_tmp * ar + ai) / l7;
      t[7].im = (b_l17_tmp * ai - ar) / l7;
    }
  }
  t[9] = t5[1];
  t[11].re = 0.0;
  t[11].im = 0.0;
  t[13].re = 0.0;
  t[13].im = 0.0;
}

// End of code generation (acde_NO_VP.cpp)
