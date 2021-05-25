//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// acde_O_VP.cpp
//
// Code generation for function 'acde_O_VP'
//

// Include files
#include "acde_O_VP.h"
#include "power.h"
#include "rt_nonfinite.h"
#include "sqrt.h"
#include "topico_wrapper_data.h"
#include "topico_wrapper_rtwutil.h"
#include <cmath>

// Function Definitions
void acde_O_VP(double P_init, double V_init, double A_init, double P_wayp,
               double V_wayp, double V_max, double J_max, double J_min,
               creal_T t[14])
{
  creal_T b_t4_tmp[2];
  creal_T c_t4_tmp[2];
  creal_T d_t4_tmp[2];
  creal_T dcv[2];
  creal_T l2[2];
  creal_T l6[2];
  creal_T t1[2];
  creal_T t4_tmp[2];
  creal_T t5[2];
  creal_T l17;
  double A_init_re;
  double A_init_tmp;
  double J_max_tmp;
  double J_min_tmp;
  double V_init_tmp;
  double ai;
  double ar;
  double b_A_init;
  double b_A_init_tmp;
  double b_J_max;
  double b_J_min_tmp;
  double b_im;
  double b_l14_tmp;
  double b_re;
  double b_y;
  double im;
  double l14;
  double l14_tmp;
  double l17_tmp;
  double l4_tmp;
  double l5_tmp;
  double re;
  double s;
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
  //  Generated on 29-Aug-2019 12:21:02
  l4_tmp = A_init * J_min;
  l5_tmp = A_init * J_max;
  l14_tmp = J_max * J_max;
  b_l14_tmp = J_min * J_max;
  l14 = 1.0 / (l14_tmp + -b_l14_tmp);
  l17_tmp = A_init * A_init;
  l17.re = J_min * (J_min + -J_max) *
           ((l17_tmp + J_max * V_max * 2.0) + -(J_max * V_init * 2.0));
  l17.im = 0.0;
  coder::internal::scalar::b_sqrt(&l17);
  t1[0].re = l14 * ((l4_tmp - l5_tmp) + l17.re);
  t1[0].im = l14 * l17.im;
  t1[1].re = -l14 * ((-l4_tmp + l5_tmp) + l17.re);
  t1[1].im = -l14 * l17.im;
  re = t1[0].re * t1[0].re - t1[0].im * t1[0].im;
  l14 = t1[0].re * t1[0].im;
  im = l14 + l14;
  l2[0].re = re;
  l2[0].im = im;
  s = J_min * V_init * -2.0 + J_min * V_wayp * 2.0;
  t5[0].re = ((((s + l14_tmp * re) + l17_tmp) - l4_tmp * t1[0].re * 2.0) +
              l5_tmp * t1[0].re * 2.0) -
             b_l14_tmp * re;
  t5[0].im =
      ((l14_tmp * im - l4_tmp * t1[0].im * 2.0) + l5_tmp * t1[0].im * 2.0) -
      b_l14_tmp * im;
  re = t1[1].re * t1[1].re - t1[1].im * t1[1].im;
  l14 = t1[1].re * t1[1].im;
  im = l14 + l14;
  t5[1].re = ((((s + l14_tmp * re) + l17_tmp) - l4_tmp * t1[1].re * 2.0) +
              l5_tmp * t1[1].re * 2.0) -
             b_l14_tmp * re;
  t5[1].im =
      ((l14_tmp * im - l4_tmp * t1[1].im * 2.0) + l5_tmp * t1[1].im * 2.0) -
      b_l14_tmp * im;
  coder::internal::scalar::b_sqrt(&t5[0]);
  coder::internal::scalar::b_sqrt(&t5[1]);
  l14 = J_min * J_min;
  coder::power(t1, l6);
  y = rt_powd_snf(J_max, 3.0);
  b_y = rt_powd_snf(J_min, 3.0);
  l17.re = P_init * l14 * 6.0 - P_wayp * l14 * 6.0;
  y_re = rt_powd_snf(A_init, 3.0) * 2.0;
  A_init_re = l4_tmp * V_init * 6.0;
  A_init_tmp = A_init * l14;
  b_A_init = A_init * l14_tmp;
  J_min_tmp = J_min * l14_tmp;
  J_max_tmp = J_max * l14;
  b_J_min_tmp = J_min * l17_tmp;
  b_J_max = J_max * l17_tmp;
  V_init_tmp = V_init * l14;
  b_A_init_tmp = l4_tmp * J_max;
  l4_tmp = b_l14_tmp * V_init;
  b_l14_tmp = b_J_min_tmp * 3.0 - V_init_tmp * 6.0;
  if (-t5[0].im == 0.0) {
    b_re = -t5[0].re / J_min;
    b_im = 0.0;
  } else if (-t5[0].re == 0.0) {
    b_re = 0.0;
    b_im = -t5[0].im / J_min;
  } else {
    b_re = -t5[0].re / J_min;
    b_im = -t5[0].im / J_min;
  }
  t5[0].re = b_re;
  t5[0].im = b_im;
  t4_tmp[0].re = A_init_tmp * t1[0].re;
  t4_tmp[0].im = A_init_tmp * t1[0].im;
  b_t4_tmp[0].re = J_min_tmp * l2[0].re;
  b_t4_tmp[0].im = J_min_tmp * l2[0].im;
  c_t4_tmp[0].re = J_max_tmp * l2[0].re;
  c_t4_tmp[0].im = J_max_tmp * l2[0].im;
  d_t4_tmp[0].re = b_A_init_tmp * t1[0].re;
  d_t4_tmp[0].im = b_A_init_tmp * t1[0].im;
  t[0] = t1[0];
  t[2].re = 0.0;
  t[2].im = 0.0;
  ar = -(A_init + J_max * t1[0].re);
  ai = -(J_max * t1[0].im);
  if (ai == 0.0) {
    t[4].re = ar / J_min;
    t[4].im = 0.0;
  } else if (ar == 0.0) {
    t[4].re = 0.0;
    t[4].im = ai / J_min;
  } else {
    t[4].re = ar / J_min;
    t[4].im = ai / J_min;
  }
  if (-t5[1].im == 0.0) {
    b_re = -t5[1].re / J_min;
    b_im = 0.0;
  } else if (-t5[1].re == 0.0) {
    b_re = 0.0;
    b_im = -t5[1].im / J_min;
  } else {
    b_re = -t5[1].re / J_min;
    b_im = -t5[1].im / J_min;
  }
  t5[1].re = b_re;
  t5[1].im = b_im;
  t4_tmp[1].re = A_init_tmp * t1[1].re;
  t4_tmp[1].im = A_init_tmp * t1[1].im;
  b_t4_tmp[1].re = J_min_tmp * re;
  b_t4_tmp[1].im = J_min_tmp * im;
  c_t4_tmp[1].re = J_max_tmp * re;
  c_t4_tmp[1].im = J_max_tmp * im;
  d_t4_tmp[1].re = b_A_init_tmp * t1[1].re;
  d_t4_tmp[1].im = b_A_init_tmp * t1[1].im;
  t[1] = t1[1];
  t[3].re = 0.0;
  t[3].im = 0.0;
  ar = -(A_init + J_max * t1[1].re);
  ai = -(J_max * t1[1].im);
  if (ai == 0.0) {
    t[5].re = ar / J_min;
    t[5].im = 0.0;
  } else if (ar == 0.0) {
    t[5].re = 0.0;
    t[5].im = ai / J_min;
  } else {
    t[5].re = ar / J_min;
    t[5].im = ai / J_min;
  }
  coder::power(t5, dcv);
  ar = ((((((((((((((((((l17.re + y * l6[0].re * 2.0) + y_re) +
                       b_y * dcv[0].re) -
                      A_init_re) +
                     A_init_tmp * l2[0].re * 3.0) +
                    b_A_init * l2[0].re * 6.0) -
                   J_min_tmp * l6[0].re * 3.0) +
                  J_max_tmp * l6[0].re) -
                 b_J_min_tmp * t1[0].re * 6.0) +
                b_J_max * t1[0].re * 6.0) -
               b_J_min_tmp * t5[0].re * 3.0) +
              V_init_tmp * t1[0].re * 6.0) +
             V_init_tmp * t5[0].re * 6.0) +
            (t4_tmp[0].re * t5[0].re - t4_tmp[0].im * t5[0].im) * 6.0) -
           (b_t4_tmp[0].re * t5[0].re - b_t4_tmp[0].im * t5[0].im) * 3.0) +
          (c_t4_tmp[0].re * t5[0].re - c_t4_tmp[0].im * t5[0].im) * 3.0) -
         b_A_init_tmp * l2[0].re * 9.0) -
        l4_tmp * t1[0].re * 6.0) -
       (d_t4_tmp[0].re * t5[0].re - d_t4_tmp[0].im * t5[0].im) * 6.0;
  ai = (((((((((((((((y * l6[0].im * 2.0 + b_y * dcv[0].im) +
                     A_init_tmp * l2[0].im * 3.0) +
                    b_A_init * l2[0].im * 6.0) -
                   J_min_tmp * l6[0].im * 3.0) +
                  J_max_tmp * l6[0].im) -
                 b_J_min_tmp * t1[0].im * 6.0) +
                b_J_max * t1[0].im * 6.0) -
               b_J_min_tmp * t5[0].im * 3.0) +
              V_init_tmp * t1[0].im * 6.0) +
             V_init_tmp * t5[0].im * 6.0) +
            (t4_tmp[0].re * t5[0].im + t4_tmp[0].im * t5[0].re) * 6.0) -
           (b_t4_tmp[0].re * t5[0].im + b_t4_tmp[0].im * t5[0].re) * 3.0) +
          (c_t4_tmp[0].re * t5[0].im + c_t4_tmp[0].im * t5[0].re) * 3.0) -
         b_A_init_tmp * l2[0].im * 9.0) -
        l4_tmp * t1[0].im * 6.0) -
       (d_t4_tmp[0].re * t5[0].im + d_t4_tmp[0].im * t5[0].re) * 6.0;
  l5_tmp = (((b_l14_tmp - t4_tmp[0].re * 6.0) + b_t4_tmp[0].re * 3.0) -
            c_t4_tmp[0].re * 3.0) +
           d_t4_tmp[0].re * 6.0;
  l14_tmp = (((0.0 - t4_tmp[0].im * 6.0) + b_t4_tmp[0].im * 3.0) -
             c_t4_tmp[0].im * 3.0) +
            d_t4_tmp[0].im * 6.0;
  if (l14_tmp == 0.0) {
    if (ai == 0.0) {
      t[6].re = ar / l5_tmp;
      t[6].im = 0.0;
    } else if (ar == 0.0) {
      t[6].re = 0.0;
      t[6].im = ai / l5_tmp;
    } else {
      t[6].re = ar / l5_tmp;
      t[6].im = ai / l5_tmp;
    }
  } else if (l5_tmp == 0.0) {
    if (ar == 0.0) {
      t[6].re = ai / l14_tmp;
      t[6].im = 0.0;
    } else if (ai == 0.0) {
      t[6].re = 0.0;
      t[6].im = -(ar / l14_tmp);
    } else {
      t[6].re = ai / l14_tmp;
      t[6].im = -(ar / l14_tmp);
    }
  } else {
    l17_tmp = std::abs(l5_tmp);
    l14 = std::abs(l14_tmp);
    if (l17_tmp > l14) {
      s = l14_tmp / l5_tmp;
      l14 = l5_tmp + s * l14_tmp;
      t[6].re = (ar + s * ai) / l14;
      t[6].im = (ai - s * ar) / l14;
    } else if (l14 == l17_tmp) {
      if (l5_tmp > 0.0) {
        l5_tmp = 0.5;
      } else {
        l5_tmp = -0.5;
      }
      if (l14_tmp > 0.0) {
        l14 = 0.5;
      } else {
        l14 = -0.5;
      }
      t[6].re = (ar * l5_tmp + ai * l14) / l17_tmp;
      t[6].im = (ai * l5_tmp - ar * l14) / l17_tmp;
    } else {
      s = l5_tmp / l14_tmp;
      l14 = l14_tmp + s * l5_tmp;
      t[6].re = (s * ar + ai) / l14;
      t[6].im = (s * ai - ar) / l14;
    }
  }
  t[8] = t5[0];
  t[10].re = 0.0;
  t[10].im = 0.0;
  t[12].re = 0.0;
  t[12].im = 0.0;
  ar = ((((((((((((((((((l17.re + y * l6[1].re * 2.0) + y_re) +
                       b_y * dcv[1].re) -
                      A_init_re) +
                     A_init_tmp * re * 3.0) +
                    b_A_init * re * 6.0) -
                   J_min_tmp * l6[1].re * 3.0) +
                  J_max_tmp * l6[1].re) -
                 b_J_min_tmp * t1[1].re * 6.0) +
                b_J_max * t1[1].re * 6.0) -
               b_J_min_tmp * b_re * 3.0) +
              V_init_tmp * t1[1].re * 6.0) +
             V_init_tmp * b_re * 6.0) +
            (t4_tmp[1].re * b_re - t4_tmp[1].im * b_im) * 6.0) -
           (b_t4_tmp[1].re * b_re - b_t4_tmp[1].im * b_im) * 3.0) +
          (c_t4_tmp[1].re * b_re - c_t4_tmp[1].im * b_im) * 3.0) -
         b_A_init_tmp * re * 9.0) -
        l4_tmp * t1[1].re * 6.0) -
       (d_t4_tmp[1].re * b_re - d_t4_tmp[1].im * b_im) * 6.0;
  ai = (((((((((((((((y * l6[1].im * 2.0 + b_y * dcv[1].im) +
                     A_init_tmp * im * 3.0) +
                    b_A_init * im * 6.0) -
                   J_min_tmp * l6[1].im * 3.0) +
                  J_max_tmp * l6[1].im) -
                 b_J_min_tmp * t1[1].im * 6.0) +
                b_J_max * t1[1].im * 6.0) -
               b_J_min_tmp * b_im * 3.0) +
              V_init_tmp * t1[1].im * 6.0) +
             V_init_tmp * b_im * 6.0) +
            (t4_tmp[1].re * b_im + t4_tmp[1].im * b_re) * 6.0) -
           (b_t4_tmp[1].re * b_im + b_t4_tmp[1].im * b_re) * 3.0) +
          (c_t4_tmp[1].re * b_im + c_t4_tmp[1].im * b_re) * 3.0) -
         b_A_init_tmp * im * 9.0) -
        l4_tmp * t1[1].im * 6.0) -
       (d_t4_tmp[1].re * b_im + d_t4_tmp[1].im * b_re) * 6.0;
  l5_tmp = (((b_l14_tmp - t4_tmp[1].re * 6.0) + b_t4_tmp[1].re * 3.0) -
            c_t4_tmp[1].re * 3.0) +
           d_t4_tmp[1].re * 6.0;
  l14_tmp = (((0.0 - t4_tmp[1].im * 6.0) + b_t4_tmp[1].im * 3.0) -
             c_t4_tmp[1].im * 3.0) +
            d_t4_tmp[1].im * 6.0;
  if (l14_tmp == 0.0) {
    if (ai == 0.0) {
      t[7].re = ar / l5_tmp;
      t[7].im = 0.0;
    } else if (ar == 0.0) {
      t[7].re = 0.0;
      t[7].im = ai / l5_tmp;
    } else {
      t[7].re = ar / l5_tmp;
      t[7].im = ai / l5_tmp;
    }
  } else if (l5_tmp == 0.0) {
    if (ar == 0.0) {
      t[7].re = ai / l14_tmp;
      t[7].im = 0.0;
    } else if (ai == 0.0) {
      t[7].re = 0.0;
      t[7].im = -(ar / l14_tmp);
    } else {
      t[7].re = ai / l14_tmp;
      t[7].im = -(ar / l14_tmp);
    }
  } else {
    l17_tmp = std::abs(l5_tmp);
    l14 = std::abs(l14_tmp);
    if (l17_tmp > l14) {
      s = l14_tmp / l5_tmp;
      l14 = l5_tmp + s * l14_tmp;
      t[7].re = (ar + s * ai) / l14;
      t[7].im = (ai - s * ar) / l14;
    } else if (l14 == l17_tmp) {
      if (l5_tmp > 0.0) {
        l5_tmp = 0.5;
      } else {
        l5_tmp = -0.5;
      }
      if (l14_tmp > 0.0) {
        l14 = 0.5;
      } else {
        l14 = -0.5;
      }
      t[7].re = (ar * l5_tmp + ai * l14) / l17_tmp;
      t[7].im = (ai * l5_tmp - ar * l14) / l17_tmp;
    } else {
      s = l5_tmp / l14_tmp;
      l14 = l14_tmp + s * l5_tmp;
      t[7].re = (s * ar + ai) / l14;
      t[7].im = (s * ai - ar) / l14;
    }
  }
  t[9] = t5[1];
  t[11].re = 0.0;
  t[11].im = 0.0;
  t[13].re = 0.0;
  t[13].im = 0.0;
}

// End of code generation (acde_O_VP.cpp)
