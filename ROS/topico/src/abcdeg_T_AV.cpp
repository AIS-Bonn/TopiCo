//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// abcdeg_T_AV.cpp
//
// Code generation for function 'abcdeg_T_AV'
//

// Include files
#include "abcdeg_T_AV.h"
#include "rt_nonfinite.h"
#include "sqrt.h"
#include "topico_wrapper_data.h"
#include "topico_wrapper_rtwutil.h"

// Function Definitions
void abcdeg_T_AV(double V_init, double A_init, double V_wayp, double A_wayp,
                 double V_max, double A_max, double J_max, double J_min,
                 double T, creal_T t[14])
{
  creal_T l4[2];
  creal_T t2_tmp[2];
  creal_T t7[2];
  creal_T l17;
  double A_init_re;
  double A_max_re;
  double J_min_re;
  double b_A_max;
  double b_A_max_re;
  double b_A_wayp;
  double b_J_min;
  double b_J_min_re;
  double b_l14_tmp;
  double b_y;
  double l14;
  double l14_tmp;
  double l17_tmp;
  double l2;
  double l4_tmp;
  double l5;
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
  //  Generated on 03-Sep-2019 11:54:15
  l4_tmp = A_wayp * J_min;
  l5 = A_wayp * J_max;
  l14_tmp = J_max * J_max;
  b_l14_tmp = J_min * J_max;
  l14 = 1.0 / (l14_tmp + -b_l14_tmp);
  l17_tmp = A_wayp * A_wayp;
  l17.re = J_min * (J_min + -J_max) *
           ((l17_tmp + J_max * V_max * 2.0) + -(J_max * V_wayp * 2.0));
  l17.im = 0.0;
  coder::internal::scalar::b_sqrt(&l17);
  t7[0].re = l14 * ((-l4_tmp + l5) + l17.re);
  t7[0].im = l14 * l17.im;
  t7[1].re = -l14 * ((l4_tmp - l5) + l17.re);
  t7[1].im = -l14 * l17.im;
  l2 = A_max * A_max;
  y = rt_powd_snf(J_max, 3.0);
  y_tmp_tmp = A_max * J_min;
  y_tmp = y_tmp_tmp * J_max;
  b_y = y_tmp * 2.0;
  l17.re =
      ((-J_min * l2 + J_max * l2) - A_init * A_init * J_min) + l17_tmp * J_max;
  A_init_re = A_init * A_max * J_min * 2.0;
  A_max_re = A_max * A_wayp * J_max * 2.0;
  J_min_re = b_l14_tmp * V_init * 2.0;
  b_J_min_re = b_l14_tmp * V_wayp * 2.0;
  b_A_max = A_max * l14_tmp;
  b_A_wayp = A_wayp * l14_tmp;
  b_J_min = J_min * l14_tmp;
  b_A_max_re = y_tmp * T * 2.0;
  l14 = l4_tmp * J_max;
  l2 = 1.0 / J_min;
  re = t7[0].re * t7[0].re - t7[0].im * t7[0].im;
  l5 = t7[0].re * t7[0].im;
  l4_tmp = l5 + l5;
  l17_tmp = ((((((((((l17.re + y * re) + A_init_re) - A_max_re) + J_min_re) -
                  b_J_min_re) +
                 b_A_max * t7[0].re * 2.0) -
                b_A_wayp * t7[0].re * 2.0) -
               b_J_min * re) +
              b_A_max_re) -
             y_tmp * t7[0].re * 2.0) +
            l14 * t7[0].re * 2.0;
  l14_tmp =
      ((((y * l4_tmp + b_A_max * t7[0].im * 2.0) - b_A_wayp * t7[0].im * 2.0) -
        b_J_min * l4_tmp) -
       y_tmp * t7[0].im * 2.0) +
      l14 * t7[0].im * 2.0;
  if (l14_tmp == 0.0) {
    re = l17_tmp / b_y;
    l4_tmp = 0.0;
  } else if (l17_tmp == 0.0) {
    re = 0.0;
    l4_tmp = l14_tmp / b_y;
  } else {
    re = l17_tmp / b_y;
    l4_tmp = l14_tmp / b_y;
  }
  l4[0].re = re;
  l4[0].im = l4_tmp;
  t2_tmp[0].im = J_max * t7[0].im;
  re = t7[1].re * t7[1].re - t7[1].im * t7[1].im;
  l5 = t7[1].re * t7[1].im;
  l4_tmp = l5 + l5;
  l17_tmp = ((((((((((l17.re + y * re) + A_init_re) - A_max_re) + J_min_re) -
                  b_J_min_re) +
                 b_A_max * t7[1].re * 2.0) -
                b_A_wayp * t7[1].re * 2.0) -
               b_J_min * re) +
              b_A_max_re) -
             y_tmp * t7[1].re * 2.0) +
            l14 * t7[1].re * 2.0;
  l14_tmp =
      ((((y * l4_tmp + b_A_max * t7[1].im * 2.0) - b_A_wayp * t7[1].im * 2.0) -
        b_J_min * l4_tmp) -
       y_tmp * t7[1].im * 2.0) +
      l14 * t7[1].im * 2.0;
  if (l14_tmp == 0.0) {
    re = l17_tmp / b_y;
    l4_tmp = 0.0;
  } else if (l17_tmp == 0.0) {
    re = 0.0;
    l4_tmp = l14_tmp / b_y;
  } else {
    re = l17_tmp / b_y;
    l4_tmp = l14_tmp / b_y;
  }
  l4[1].re = re;
  l4[1].im = l4_tmp;
  t2_tmp[1].im = J_max * t7[1].im;
  l5 = A_max * (1.0 / J_min);
  l14 = -(1.0 / J_max * (A_init + -A_max));
  A_init_re = ((A_init * J_min - y_tmp_tmp) + A_max * J_max) + b_l14_tmp * T;
  t[0].re = l14;
  t[0].im = 0.0;
  t[1].re = l14;
  t[1].im = 0.0;
  t[4].re = -l5;
  t[4].im = 0.0;
  t[5].re = -l5;
  t[5].im = 0.0;
  l5 = A_wayp - J_max * t7[0].re;
  l17_tmp = l2 * (A_init_re - b_l14_tmp * ((l4[0].re + t7[0].re) + l2 * l5));
  l14_tmp =
      l2 *
      (0.0 - b_l14_tmp * ((l4[0].im + t7[0].im) + l2 * (0.0 - t2_tmp[0].im)));
  if (l14_tmp == 0.0) {
    t[2].re = l17_tmp / J_max;
    t[2].im = 0.0;
  } else if (l17_tmp == 0.0) {
    t[2].re = 0.0;
    t[2].im = l14_tmp / J_max;
  } else {
    t[2].re = l17_tmp / J_max;
    t[2].im = l14_tmp / J_max;
  }
  t[6] = l4[0];
  if (0.0 - t2_tmp[0].im == 0.0) {
    t[8].re = l5 / J_min;
    t[8].im = 0.0;
  } else if (l5 == 0.0) {
    t[8].re = 0.0;
    t[8].im = (0.0 - t2_tmp[0].im) / J_min;
  } else {
    t[8].re = l5 / J_min;
    t[8].im = (0.0 - t2_tmp[0].im) / J_min;
  }
  t[10].re = 0.0;
  t[10].im = 0.0;
  t[12] = t7[0];
  l5 = A_wayp - J_max * t7[1].re;
  l17_tmp = l2 * (A_init_re - b_l14_tmp * ((re + t7[1].re) + l2 * l5));
  l14_tmp = l2 * (0.0 - b_l14_tmp *
                            ((l4_tmp + t7[1].im) + l2 * (0.0 - t2_tmp[1].im)));
  if (l14_tmp == 0.0) {
    t[3].re = l17_tmp / J_max;
    t[3].im = 0.0;
  } else if (l17_tmp == 0.0) {
    t[3].re = 0.0;
    t[3].im = l14_tmp / J_max;
  } else {
    t[3].re = l17_tmp / J_max;
    t[3].im = l14_tmp / J_max;
  }
  t[7] = l4[1];
  if (0.0 - t2_tmp[1].im == 0.0) {
    t[9].re = l5 / J_min;
    t[9].im = 0.0;
  } else if (l5 == 0.0) {
    t[9].re = 0.0;
    t[9].im = (0.0 - t2_tmp[1].im) / J_min;
  } else {
    t[9].re = l5 / J_min;
    t[9].im = (0.0 - t2_tmp[1].im) / J_min;
  }
  t[11].re = 0.0;
  t[11].im = 0.0;
  t[13] = t7[1];
}

// End of code generation (abcdeg_T_AV.cpp)
