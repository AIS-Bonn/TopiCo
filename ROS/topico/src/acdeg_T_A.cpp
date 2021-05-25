//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// acdeg_T_A.cpp
//
// Code generation for function 'acdeg_T_A'
//

// Include files
#include "acdeg_T_A.h"
#include "rt_nonfinite.h"
#include "sqrt.h"
#include "topico_wrapper_data.h"
#include "topico_wrapper_rtwutil.h"
#include <cmath>

// Function Definitions
void acdeg_T_A(double V_init, double A_init, double A_wayp, double V_max,
               double J_max, double J_min, double T, creal_T t[14])
{
  creal_T l23[2];
  creal_T t3[2];
  creal_T z;
  double J_max_re;
  double b_d;
  double b_l9_tmp;
  double bim;
  double brm;
  double d;
  double d1;
  double d2;
  double d3;
  double d4;
  double im;
  double l12_tmp;
  double l13;
  double l14;
  double l16;
  double l16_tmp;
  double l3_tmp;
  double l4_tmp;
  double l8;
  double l9_tmp;
  double re;
  double s;
  double sgnbi;
  double sgnbr;
  double t7_idx_1;
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
  //  Generated on 29-Aug-2019 16:29:55
  t7_idx_1 = A_wayp * 2.0 / J_max;
  l3_tmp = J_max * J_max;
  l4_tmp = rt_powd_snf(J_max, 3.0);
  l8 = 1.0 / J_min;
  l9_tmp = J_min * J_max;
  b_l9_tmp = l9_tmp * V_init * 2.0;
  l12_tmp = l9_tmp * V_max * 2.0;
  l13 = J_min + -J_max;
  l14 = J_max + -J_min;
  l16_tmp = A_init * A_init;
  l16 = l16_tmp * -J_min;
  d = A_wayp * J_min * J_max;
  J_max_re = b_l9_tmp + -l12_tmp;
  d1 = A_wayp * l3_tmp;
  d2 = d * 0.0 * 2.0;
  l23[0].re = ((((J_max_re + d2) + l16) + l4_tmp * 0.0) + -(d1 * 0.0 * 2.0)) +
              l3_tmp * 0.0 * -J_min;
  l23[0].im = 0.0;
  d3 = t7_idx_1 * t7_idx_1;
  d = d * t7_idx_1 * 2.0;
  d4 = l4_tmp * d3;
  l23[1].re = ((((J_max_re + d) + l16) + d4) + -(d1 * t7_idx_1 * 2.0)) +
              l3_tmp * d3 * -J_min;
  l23[1].im = 0.0;
  coder::internal::scalar::b_sqrt(&l23[0]);
  coder::internal::scalar::b_sqrt(&l23[1]);
  z.re = l14;
  z.im = 0.0;
  coder::internal::scalar::b_sqrt(&z);
  if (z.im == 0.0) {
    re = l8 / z.re;
    im = 0.0;
  } else if (z.re == 0.0) {
    if (l8 == 0.0) {
      re = 0.0 / z.im;
      im = 0.0;
    } else {
      re = 0.0;
      im = -(l8 / z.im);
    }
  } else {
    brm = std::abs(z.re);
    bim = std::abs(z.im);
    if (brm > bim) {
      s = z.im / z.re;
      b_d = z.re + s * z.im;
      re = (l8 + s * 0.0) / b_d;
      im = (0.0 - s * l8) / b_d;
    } else if (bim == brm) {
      if (z.re > 0.0) {
        sgnbr = 0.5;
      } else {
        sgnbr = -0.5;
      }
      if (z.im > 0.0) {
        sgnbi = 0.5;
      } else {
        sgnbi = -0.5;
      }
      re = (l8 * sgnbr + 0.0 * sgnbi) / brm;
      im = (0.0 * sgnbr - l8 * sgnbi) / brm;
    } else {
      s = z.re / z.im;
      b_d = z.im + s * z.re;
      re = s * l8 / b_d;
      im = (s * 0.0 - l8) / b_d;
    }
  }
  z.re = l14;
  z.im = 0.0;
  coder::internal::scalar::b_sqrt(&z);
  l14 = J_max * l13;
  J_max_re = l16_tmp * J_min;
  sgnbr = J_min * l3_tmp;
  sgnbi = J_min - J_max;
  t3[0].re =
      sgnbi *
      ((((((J_max_re - l4_tmp * 0.0) - b_l9_tmp) + l12_tmp) + d1 * 0.0 * 2.0) +
        sgnbr * 0.0) -
       d2);
  t3[0].im = 0.0;
  t3[1].re =
      sgnbi *
      ((((((J_max_re - d4) - b_l9_tmp) + l12_tmp) + d1 * t7_idx_1 * 2.0) +
        sgnbr * d3) -
       d);
  t3[1].im = 0.0;
  coder::internal::scalar::b_sqrt(&t3[0]);
  coder::internal::scalar::b_sqrt(&t3[1]);
  sgnbi = J_min * J_min - l9_tmp;
  brm = A_init * l13;
  J_max_re = J_max * T * l13;
  l16 = -J_max * l13;
  if (t3[0].im == 0.0) {
    bim = t3[0].re / sgnbi;
    s = 0.0;
  } else if (t3[0].re == 0.0) {
    bim = 0.0;
    s = t3[0].im / sgnbi;
  } else {
    bim = t3[0].re / sgnbi;
    s = t3[0].im / sgnbi;
  }
  b_d = -(A_init + J_min * bim);
  sgnbr = -(J_min * s);
  if (sgnbr == 0.0) {
    t[0].re = b_d / J_max;
    t[0].im = 0.0;
  } else if (b_d == 0.0) {
    t[0].re = 0.0;
    t[0].im = sgnbr / J_max;
  } else {
    t[0].re = b_d / J_max;
    t[0].im = sgnbr / J_max;
  }
  t[2].re = 0.0;
  t[2].im = 0.0;
  t[4].re = bim;
  t[4].im = s;
  b_d =
      ((brm + (z.re * l23[0].re - z.im * l23[0].im)) + J_max_re) +
      l16 * (l8 * (A_wayp + -J_max * 0.0) - (re * l23[0].re - im * l23[0].im));
  sgnbr = (z.re * l23[0].im + z.im * l23[0].re) +
          l16 * (0.0 - (re * l23[0].im + im * l23[0].re));
  if (sgnbr == 0.0) {
    t[6].re = b_d / l14;
    t[6].im = 0.0;
  } else if (b_d == 0.0) {
    t[6].re = 0.0;
    t[6].im = sgnbr / l14;
  } else {
    t[6].re = b_d / l14;
    t[6].im = sgnbr / l14;
  }
  t[8].re = (A_wayp - J_max * 0.0) / J_min;
  t[8].im = 0.0;
  t[10].re = 0.0;
  t[10].im = 0.0;
  t[12].re = 0.0;
  t[12].im = 0.0;
  if (t3[1].im == 0.0) {
    bim = t3[1].re / sgnbi;
    s = 0.0;
  } else if (t3[1].re == 0.0) {
    bim = 0.0;
    s = t3[1].im / sgnbi;
  } else {
    bim = t3[1].re / sgnbi;
    s = t3[1].im / sgnbi;
  }
  b_d = -(A_init + J_min * bim);
  sgnbr = -(J_min * s);
  if (sgnbr == 0.0) {
    t[1].re = b_d / J_max;
    t[1].im = 0.0;
  } else if (b_d == 0.0) {
    t[1].re = 0.0;
    t[1].im = sgnbr / J_max;
  } else {
    t[1].re = b_d / J_max;
    t[1].im = sgnbr / J_max;
  }
  t[3].re = 0.0;
  t[3].im = 0.0;
  t[5].re = bim;
  t[5].im = s;
  b_d = ((brm + (z.re * l23[1].re - z.im * l23[1].im)) + J_max_re) +
        l16 * ((t7_idx_1 + l8 * (A_wayp + -J_max * t7_idx_1)) -
               (re * l23[1].re - im * l23[1].im));
  sgnbr = (z.re * l23[1].im + z.im * l23[1].re) +
          l16 * (0.0 - (re * l23[1].im + im * l23[1].re));
  if (sgnbr == 0.0) {
    t[7].re = b_d / l14;
    t[7].im = 0.0;
  } else if (b_d == 0.0) {
    t[7].re = 0.0;
    t[7].im = sgnbr / l14;
  } else {
    t[7].re = b_d / l14;
    t[7].im = sgnbr / l14;
  }
  t[9].re = (A_wayp - J_max * t7_idx_1) / J_min;
  t[9].im = 0.0;
  t[11].re = 0.0;
  t[11].im = 0.0;
  t[13].re = t7_idx_1;
  t[13].im = 0.0;
}

// End of code generation (acdeg_T_A.cpp)
