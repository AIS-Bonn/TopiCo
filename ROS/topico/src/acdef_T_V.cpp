//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// acdef_T_V.cpp
//
// Code generation for function 'acdef_T_V'
//

// Include files
#include "acdef_T_V.h"
#include "rt_nonfinite.h"
#include "sqrt.h"
#include "topico_wrapper_data.h"

// Function Definitions
void acdef_T_V(double V_init, double A_init, double V_wayp, double V_max,
               double A_min, double J_max, double J_min, double T,
               creal_T t[14])
{
  creal_T l2[2];
  creal_T t1[2];
  creal_T t4[2];
  creal_T t4_tmp[2];
  creal_T l17;
  double A_init_re_tmp;
  double J_min_im;
  double J_min_re_tmp;
  double ar;
  double b_im;
  double b_re;
  double b_y;
  double im;
  double l14;
  double l3;
  double l4_tmp;
  double l5;
  double re;
  double re_tmp;
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
  //  Generated on 02-Sep-2019 16:00:04
  l4_tmp = A_init * J_min;
  l5 = A_init * J_max;
  l14 = 1.0 / (J_max * J_max + -(J_min * J_max));
  l17.re = J_min * (J_min + -J_max) *
           ((A_init * A_init + J_max * V_max * 2.0) + -(J_max * V_init * 2.0));
  l17.im = 0.0;
  coder::internal::scalar::b_sqrt(&l17);
  t1[0].re = l14 * ((l4_tmp - l5) + l17.re);
  t1[0].im = l14 * l17.im;
  t1[1].re = -l14 * ((-l4_tmp + l5) + l17.re);
  t1[1].im = -l14 * l17.im;
  y = A_min * A_min;
  b_y = A_min * J_min * 2.0;
  l17.re = J_min * V_init * 2.0 - J_min * V_wayp * 2.0;
  A_init_re_tmp = A_init - A_min;
  J_min_re_tmp = J_min * T;
  l3 = A_min * (1.0 / J_min);
  re = J_max * t1[0].re;
  im = J_max * t1[0].im;
  l2[0].re = re;
  l2[0].im = im;
  b_re = J_min * t1[0].re;
  b_im = J_min * t1[0].im;
  t4_tmp[0].re = b_re;
  t4_tmp[0].im = b_im;
  re_tmp = A_init + re;
  l5 = re_tmp * im;
  l14 = J_min * re;
  J_min_im = J_min * im;
  ar = ((((l17.re - (re_tmp * re_tmp - im * im)) +
          A_min * (((A_init_re_tmp + re) - b_re) + J_min_re_tmp) * 2.0) +
         y) +
        l4_tmp * t1[0].re * 2.0) +
       (l14 * t1[0].re - J_min_im * t1[0].im);
  J_min_im = (((0.0 - (l5 + l5)) + A_min * (im - b_im) * 2.0) +
              l4_tmp * t1[0].im * 2.0) +
             (l14 * t1[0].im + J_min_im * t1[0].re);
  if (J_min_im == 0.0) {
    l5 = ar / b_y;
    l14 = 0.0;
  } else if (ar == 0.0) {
    l5 = 0.0;
    l14 = J_min_im / b_y;
  } else {
    l5 = ar / b_y;
    l14 = J_min_im / b_y;
  }
  t4[0].re = l5;
  t4[0].im = l14;
  t[0] = t1[0];
  t[2].re = 0.0;
  t[2].im = 0.0;
  if (-im == 0.0) {
    t[4].re = -re_tmp / J_min;
    t[4].im = 0.0;
  } else if (-re_tmp == 0.0) {
    t[4].re = 0.0;
    t[4].im = -im / J_min;
  } else {
    t[4].re = -re_tmp / J_min;
    t[4].im = -im / J_min;
  }
  t[6].re = l5;
  t[6].im = l14;
  re = J_max * t1[1].re;
  im = J_max * t1[1].im;
  b_re = J_min * t1[1].re;
  b_im = J_min * t1[1].im;
  re_tmp = A_init + re;
  l5 = re_tmp * im;
  l14 = J_min * re;
  J_min_im = J_min * im;
  ar = ((((l17.re - (re_tmp * re_tmp - im * im)) +
          A_min * (((A_init_re_tmp + re) - b_re) + J_min_re_tmp) * 2.0) +
         y) +
        l4_tmp * t1[1].re * 2.0) +
       (l14 * t1[1].re - J_min_im * t1[1].im);
  J_min_im = (((0.0 - (l5 + l5)) + A_min * (im - b_im) * 2.0) +
              l4_tmp * t1[1].im * 2.0) +
             (l14 * t1[1].im + J_min_im * t1[1].re);
  if (J_min_im == 0.0) {
    l5 = ar / b_y;
    l14 = 0.0;
  } else if (ar == 0.0) {
    l5 = 0.0;
    l14 = J_min_im / b_y;
  } else {
    l5 = ar / b_y;
    l14 = J_min_im / b_y;
  }
  t[1] = t1[1];
  t[3].re = 0.0;
  t[3].im = 0.0;
  if (-im == 0.0) {
    t[5].re = -re_tmp / J_min;
    t[5].im = 0.0;
  } else if (-re_tmp == 0.0) {
    t[5].re = 0.0;
    t[5].im = -im / J_min;
  } else {
    t[5].re = -re_tmp / J_min;
    t[5].im = -im / J_min;
  }
  t[7].re = l5;
  t[7].im = l14;
  t[8].re = l3;
  t[8].im = 0.0;
  t[9].re = l3;
  t[9].im = 0.0;
  ar = (((A_init_re_tmp - t4_tmp[0].re) + l2[0].re) - J_min * t4[0].re) +
       J_min_re_tmp;
  J_min_im = ((0.0 - t4_tmp[0].im) + l2[0].im) - J_min * t4[0].im;
  if (J_min_im == 0.0) {
    t[10].re = ar / J_min;
    t[10].im = 0.0;
  } else if (ar == 0.0) {
    t[10].re = 0.0;
    t[10].im = J_min_im / J_min;
  } else {
    t[10].re = ar / J_min;
    t[10].im = J_min_im / J_min;
  }
  t[12].re = 0.0;
  t[12].im = 0.0;
  ar = (((A_init_re_tmp - b_re) + re) - J_min * l5) + J_min_re_tmp;
  J_min_im = ((0.0 - b_im) + im) - J_min * l14;
  if (J_min_im == 0.0) {
    t[11].re = ar / J_min;
    t[11].im = 0.0;
  } else if (ar == 0.0) {
    t[11].re = 0.0;
    t[11].im = J_min_im / J_min;
  } else {
    t[11].re = ar / J_min;
    t[11].im = J_min_im / J_min;
  }
  t[13].re = 0.0;
  t[13].im = 0.0;
}

// End of code generation (acdef_T_V.cpp)
