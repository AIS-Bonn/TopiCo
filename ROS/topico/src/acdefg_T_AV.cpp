//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// acdefg_T_AV.cpp
//
// Code generation for function 'acdefg_T_AV'
//

// Include files
#include "acdefg_T_AV.h"
#include "rt_nonfinite.h"
#include "sqrt.h"
#include "topico_wrapper_data.h"
#include "topico_wrapper_rtwutil.h"

// Function Definitions
void acdefg_T_AV(double V_init, double A_init, double V_wayp, double A_wayp,
                 double V_max, double A_min, double J_max, double J_min,
                 double T, creal_T t[14])
{
  creal_T a[2];
  creal_T l4[2];
  creal_T t1[2];
  creal_T t4_tmp[2];
  creal_T t6_tmp[2];
  creal_T x[2];
  creal_T l17;
  double A_init_re_tmp;
  double A_min_re_tmp;
  double J_min_re;
  double b_A_init_re_tmp;
  double b_J_min_re;
  double b_im;
  double b_l14_tmp;
  double b_re;
  double b_y;
  double d;
  double im;
  double l14;
  double l14_tmp;
  double l17_tmp;
  double l5;
  double l6;
  double re;
  double y;
  double y_tmp;
  double y_tmp_tmp;
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
  //  Generated on 03-Sep-2019 11:41:15
  l6 = A_init * J_min;
  l5 = A_init * J_max;
  l14_tmp = J_max * J_max;
  b_l14_tmp = J_min * J_max;
  l14 = 1.0 / (l14_tmp + -b_l14_tmp);
  l17_tmp = A_init * A_init;
  l17.re = J_min * (J_min + -J_max) *
           ((l17_tmp + J_max * V_max * 2.0) + -(J_max * V_init * 2.0));
  l17.im = 0.0;
  coder::internal::scalar::b_sqrt(&l17);
  t1[0].re = l14 * ((l6 - l5) + l17.re);
  t1[0].im = l14 * l17.im;
  t1[1].re = -l14 * ((-l6 + l5) + l17.re);
  t1[1].im = -l14 * l17.im;
  l5 = A_min * A_min;
  y = rt_powd_snf(J_max, 3.0);
  y_tmp_tmp = A_min * J_min;
  y_tmp = y_tmp_tmp * J_max;
  b_y = y_tmp * 2.0;
  re = t1[0].re * t1[0].re - t1[0].im * t1[0].im;
  l14 = t1[0].re * t1[0].im;
  im = l14 + l14;
  l4[0].re = re;
  l4[0].im = im;
  A_init_re_tmp = l6 * J_max;
  b_re = 2.0 * (A_init_re_tmp * t1[0].re);
  b_im = 2.0 * (A_init_re_tmp * t1[0].im);
  t4_tmp[0].re = b_re;
  t4_tmp[0].im = b_im;
  b_A_init_re_tmp = A_init * l14_tmp;
  A_min_re_tmp = A_min * l14_tmp;
  d = J_min * l14_tmp;
  l6 = ((J_min * l5 - J_max * l5) - l17_tmp * J_max) + A_wayp * A_wayp * J_min;
  l14_tmp = A_init * A_min * J_max * 2.0;
  l17_tmp = A_min * A_wayp * J_min * 2.0;
  J_min_re = b_l14_tmp * V_init * 2.0;
  b_J_min_re = b_l14_tmp * V_wayp * 2.0;
  l5 = y_tmp * T * 2.0;
  x[0].re =
      ((((((((((l6 - y * re) + l14_tmp) - l17_tmp) + J_min_re) - b_J_min_re) -
           b_A_init_re_tmp * t1[0].re * 2.0) +
          A_min_re_tmp * t1[0].re * 2.0) +
         d * re) +
        l5) +
       b_re) -
      y_tmp * t1[0].re * 2.0;
  x[0].im = (((((0.0 - y * im) - b_A_init_re_tmp * t1[0].im * 2.0) +
               A_min_re_tmp * t1[0].im * 2.0) +
              d * im) +
             b_im) -
            y_tmp * t1[0].im * 2.0;
  re = J_max * t1[0].re;
  b_l14_tmp = J_max * t1[0].im;
  t6_tmp[0].re = re;
  t6_tmp[0].im = b_l14_tmp;
  a[0].re = A_init + re;
  a[0].im = b_l14_tmp;
  re = t1[1].re * t1[1].re - t1[1].im * t1[1].im;
  l14 = t1[1].re * t1[1].im;
  im = l14 + l14;
  b_re = 2.0 * (A_init_re_tmp * t1[1].re);
  b_im = 2.0 * (A_init_re_tmp * t1[1].im);
  x[1].re =
      ((((((((((l6 - y * re) + l14_tmp) - l17_tmp) + J_min_re) - b_J_min_re) -
           b_A_init_re_tmp * t1[1].re * 2.0) +
          A_min_re_tmp * t1[1].re * 2.0) +
         d * re) +
        l5) +
       b_re) -
      y_tmp * t1[1].re * 2.0;
  x[1].im = (((((0.0 - y * im) - b_A_init_re_tmp * t1[1].im * 2.0) +
               A_min_re_tmp * t1[1].im * 2.0) +
              d * im) +
             b_im) -
            y_tmp * t1[1].im * 2.0;
  b_l14_tmp = J_max * t1[1].im;
  d = A_init + J_max * t1[1].re;
  l5 = A_min + -A_wayp;
  l17.re = A_min * A_min * J_max + J_min * (l5 * l5);
  l17_tmp = J_min * (J_max * J_max);
  l14_tmp = y_tmp_tmp * l5 * 2.0;
  l6 = -(1.0 / J_max * l5);
  l5 = A_min * (1.0 / J_min);
  t[8].re = l5;
  t[8].im = 0.0;
  t[9].re = l5;
  t[9].im = 0.0;
  t[0] = t1[0];
  t[2].re = 0.0;
  t[2].im = 0.0;
  l14 = -(A_init + t6_tmp[0].re);
  if (-t6_tmp[0].im == 0.0) {
    t[4].re = l14 / J_min;
    t[4].im = 0.0;
  } else if (l14 == 0.0) {
    t[4].re = 0.0;
    t[4].im = -t6_tmp[0].im / J_min;
  } else {
    t[4].re = l14 / J_min;
    t[4].im = -t6_tmp[0].im / J_min;
  }
  if (x[0].im == 0.0) {
    t[6].re = x[0].re / b_y;
    t[6].im = 0.0;
  } else if (x[0].re == 0.0) {
    t[6].re = 0.0;
    t[6].im = x[0].im / b_y;
  } else {
    t[6].re = x[0].re / b_y;
    t[6].im = x[0].im / b_y;
  }
  l5 = a[0].re * a[0].im;
  l14 = ((((((l17.re - J_max * (a[0].re * a[0].re - a[0].im * a[0].im)) +
             l17_tmp * l4[0].re) +
            J_min_re) -
           b_J_min_re) -
          l14_tmp) +
         t4_tmp[0].re) *
        -0.5;
  l5 = (((0.0 - J_max * (l5 + l5)) + l17_tmp * l4[0].im) + t4_tmp[0].im) * -0.5;
  if (l5 == 0.0) {
    t[10].re = l14 / y_tmp;
    t[10].im = 0.0;
  } else if (l14 == 0.0) {
    t[10].re = 0.0;
    t[10].im = l5 / y_tmp;
  } else {
    t[10].re = l14 / y_tmp;
    t[10].im = l5 / y_tmp;
  }
  t[1] = t1[1];
  t[3].re = 0.0;
  t[3].im = 0.0;
  if (-b_l14_tmp == 0.0) {
    t[5].re = -d / J_min;
    t[5].im = 0.0;
  } else if (-d == 0.0) {
    t[5].re = 0.0;
    t[5].im = -b_l14_tmp / J_min;
  } else {
    t[5].re = -d / J_min;
    t[5].im = -b_l14_tmp / J_min;
  }
  if (x[1].im == 0.0) {
    t[7].re = x[1].re / b_y;
    t[7].im = 0.0;
  } else if (x[1].re == 0.0) {
    t[7].re = 0.0;
    t[7].im = x[1].im / b_y;
  } else {
    t[7].re = x[1].re / b_y;
    t[7].im = x[1].im / b_y;
  }
  l5 = d * b_l14_tmp;
  l14 =
      ((((((l17.re - J_max * (d * d - b_l14_tmp * b_l14_tmp)) + l17_tmp * re) +
          J_min_re) -
         b_J_min_re) -
        l14_tmp) +
       b_re) *
      -0.5;
  l5 = (((0.0 - J_max * (l5 + l5)) + l17_tmp * im) + b_im) * -0.5;
  if (l5 == 0.0) {
    t[11].re = l14 / y_tmp;
    t[11].im = 0.0;
  } else if (l14 == 0.0) {
    t[11].re = 0.0;
    t[11].im = l5 / y_tmp;
  } else {
    t[11].re = l14 / y_tmp;
    t[11].im = l5 / y_tmp;
  }
  t[12].re = l6;
  t[12].im = 0.0;
  t[13].re = l6;
  t[13].im = 0.0;
}

// End of code generation (acdefg_T_AV.cpp)
