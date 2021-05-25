//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// acde_T_V.cpp
//
// Code generation for function 'acde_T_V'
//

// Include files
#include "acde_T_V.h"
#include "rt_nonfinite.h"
#include "sqrt.h"
#include "topico_wrapper_data.h"

// Function Definitions
void acde_T_V(double V_init, double A_init, double V_wayp, double V_max,
              double J_max, double J_min, double T, creal_T t[14])
{
  creal_T b_t4_tmp[2];
  creal_T c_t4_tmp[2];
  creal_T l2[2];
  creal_T t1[2];
  creal_T t4_tmp[2];
  creal_T x[2];
  creal_T x_tmp[2];
  creal_T l17;
  double b_im;
  double b_l14_tmp;
  double b_re;
  double c_im;
  double c_re;
  double d;
  double d1;
  double d_im;
  double d_re;
  double e_im;
  double im;
  double l14;
  double l14_tmp;
  double l17_tmp;
  double l4_tmp;
  double l5_tmp;
  double re;
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
  //  Generated on 03-Sep-2019 12:19:04
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
  l14 = 2.0 * (l4_tmp * t1[0].re);
  b_im = 2.0 * (l4_tmp * t1[0].im);
  t4_tmp[0].re = l14;
  t4_tmp[0].im = b_im;
  b_re = 2.0 * (l5_tmp * t1[0].re);
  c_im = 2.0 * (l5_tmp * t1[0].im);
  b_t4_tmp[0].re = b_re;
  b_t4_tmp[0].im = c_im;
  c_re = J_max * t1[0].re;
  d_im = J_max * t1[0].im;
  c_t4_tmp[0].re = c_re;
  c_t4_tmp[0].im = d_im;
  d_re = b_l14_tmp * re;
  e_im = b_l14_tmp * im;
  x_tmp[0].re = d_re;
  x_tmp[0].im = e_im;
  d = J_min * V_init * -2.0 + J_min * V_wayp * 2.0;
  d1 = J_min * T;
  x[0].re = (((((((d + l14_tmp * re) + l17_tmp) - l14) + b_re) - d_re) -
              J_min * t1[0].re) +
             c_re) +
            d1;
  x[0].im = ((((l14_tmp * im - b_im) + c_im) - e_im) - J_min * t1[0].im) + d_im;
  re = t1[1].re * t1[1].re - t1[1].im * t1[1].im;
  l14 = t1[1].re * t1[1].im;
  im = l14 + l14;
  c_re = J_max * t1[1].re;
  d_im = J_max * t1[1].im;
  l14 = ((((d + l14_tmp * re) + l17_tmp) - 2.0 * (l4_tmp * t1[1].re)) +
         2.0 * (l5_tmp * t1[1].re)) -
        b_l14_tmp * re;
  x[1].re = ((l14 - J_min * t1[1].re) + c_re) + d1;
  d1 =
      ((l14_tmp * im - 2.0 * (l4_tmp * t1[1].im)) + 2.0 * (l5_tmp * t1[1].im)) -
      b_l14_tmp * im;
  x[1].im = (d1 - J_min * t1[1].im) + d_im;
  coder::internal::scalar::b_sqrt(&x[0]);
  coder::internal::scalar::b_sqrt(&x[1]);
  l2[0].re =
      ((((d + l14_tmp * l2[0].re) + l17_tmp) - t4_tmp[0].re) + b_t4_tmp[0].re) -
      x_tmp[0].re;
  l2[0].im =
      ((l14_tmp * l2[0].im - t4_tmp[0].im) + b_t4_tmp[0].im) - x_tmp[0].im;
  l2[1].re = l14;
  l2[1].im = d1;
  coder::internal::scalar::b_sqrt(&l2[0]);
  coder::internal::scalar::b_sqrt(&l2[1]);
  t[0] = t1[0];
  t[2].re = 0.0;
  t[2].im = 0.0;
  l14 = -(A_init + c_t4_tmp[0].re);
  if (-c_t4_tmp[0].im == 0.0) {
    t[4].re = l14 / J_min;
    t[4].im = 0.0;
  } else if (l14 == 0.0) {
    t[4].re = 0.0;
    t[4].im = -c_t4_tmp[0].im / J_min;
  } else {
    t[4].re = l14 / J_min;
    t[4].im = -c_t4_tmp[0].im / J_min;
  }
  l14 = A_init + x[0].re;
  if (x[0].im == 0.0) {
    t[6].re = l14 / J_min;
    t[6].im = 0.0;
  } else if (l14 == 0.0) {
    t[6].re = 0.0;
    t[6].im = x[0].im / J_min;
  } else {
    t[6].re = l14 / J_min;
    t[6].im = x[0].im / J_min;
  }
  if (-l2[0].im == 0.0) {
    t[8].re = -l2[0].re / J_min;
    t[8].im = 0.0;
  } else if (-l2[0].re == 0.0) {
    t[8].re = 0.0;
    t[8].im = -l2[0].im / J_min;
  } else {
    t[8].re = -l2[0].re / J_min;
    t[8].im = -l2[0].im / J_min;
  }
  t[10].re = 0.0;
  t[10].im = 0.0;
  t[12].re = 0.0;
  t[12].im = 0.0;
  t[1] = t1[1];
  t[3].re = 0.0;
  t[3].im = 0.0;
  l14 = -(A_init + c_re);
  if (-d_im == 0.0) {
    t[5].re = l14 / J_min;
    t[5].im = 0.0;
  } else if (l14 == 0.0) {
    t[5].re = 0.0;
    t[5].im = -d_im / J_min;
  } else {
    t[5].re = l14 / J_min;
    t[5].im = -d_im / J_min;
  }
  l14 = A_init + x[1].re;
  if (x[1].im == 0.0) {
    t[7].re = l14 / J_min;
    t[7].im = 0.0;
  } else if (l14 == 0.0) {
    t[7].re = 0.0;
    t[7].im = x[1].im / J_min;
  } else {
    t[7].re = l14 / J_min;
    t[7].im = x[1].im / J_min;
  }
  if (-l2[1].im == 0.0) {
    t[9].re = -l2[1].re / J_min;
    t[9].im = 0.0;
  } else if (-l2[1].re == 0.0) {
    t[9].re = 0.0;
    t[9].im = -l2[1].im / J_min;
  } else {
    t[9].re = -l2[1].re / J_min;
    t[9].im = -l2[1].im / J_min;
  }
  t[11].re = 0.0;
  t[11].im = 0.0;
  t[13].re = 0.0;
  t[13].im = 0.0;
}

// End of code generation (acde_T_V.cpp)
