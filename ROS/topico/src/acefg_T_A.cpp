//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// acefg_T_A.cpp
//
// Code generation for function 'acefg_T_A'
//

// Include files
#include "acefg_T_A.h"
#include "rt_nonfinite.h"
#include "sqrt.h"
#include "topico_wrapper_data.h"

// Function Definitions
void acefg_T_A(double V_init, double A_init, double A_wayp, double V_max,
               double A_min, double J_max, double J_min, double T,
               creal_T t[14])
{
  creal_T t3[2];
  creal_T l26;
  double J_max_re;
  double ar;
  double ar_tmp;
  double b_im;
  double im;
  double l26_tmp;
  double l3;
  double l5;
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
  //  Generated on 03-Sep-2019 12:41:01
  l3 = A_wayp * A_wayp;
  l5 = J_min * J_max;
  l26.re = -(J_min + -J_max) *
           (((((((A_init * A_min * J_min * 2.0 - A_min * A_wayp * J_min * 2.0) +
                 V_init * l5 * 2.0) -
                V_max * l5 * 2.0) -
               J_min * (A_init * A_init)) +
              J_min * l3) -
             J_max * l3) +
            A_min * T * l5 * 2.0);
  l26.im = 0.0;
  coder::internal::scalar::b_sqrt(&l26);
  l26_tmp = 1.0 / (J_min * J_min + -l5);
  l26.re *= l26_tmp;
  l26.im *= l26_tmp;
  t3[0] = l26;
  t3[1].re = -l26.re;
  t3[1].im = -l26.im;
  l26_tmp = A_init - A_wayp;
  J_max_re = J_max * T;
  l3 = J_min * t3[0].re;
  im = J_min * t3[0].im;
  ar_tmp = A_init - A_min;
  ar = -(ar_tmp + l3);
  if (-im == 0.0) {
    re = ar / J_max;
    b_im = 0.0;
  } else if (ar == 0.0) {
    re = 0.0;
    b_im = -im / J_max;
  } else {
    re = ar / J_max;
    b_im = -im / J_max;
  }
  t[0].re = re;
  t[0].im = b_im;
  t[2].re = 0.0;
  t[2].im = 0.0;
  t[4] = t3[0];
  t[6].re = 0.0;
  t[6].im = 0.0;
  t[8].re = 0.0;
  t[8].im = 0.0;
  l3 += l26_tmp;
  ar = (l3 - J_max * t3[0].re) + J_max_re;
  l5 = im - J_max * t3[0].im;
  if (l5 == 0.0) {
    t[10].re = ar / J_max;
    t[10].im = 0.0;
  } else if (ar == 0.0) {
    t[10].re = 0.0;
    t[10].im = l5 / J_max;
  } else {
    t[10].re = ar / J_max;
    t[10].im = l5 / J_max;
  }
  ar = -(l3 + J_max * re);
  l5 = -(im + J_max * b_im);
  if (l5 == 0.0) {
    t[12].re = ar / J_max;
    t[12].im = 0.0;
  } else if (ar == 0.0) {
    t[12].re = 0.0;
    t[12].im = l5 / J_max;
  } else {
    t[12].re = ar / J_max;
    t[12].im = l5 / J_max;
  }
  l3 = J_min * t3[1].re;
  im = J_min * t3[1].im;
  ar = -(ar_tmp + l3);
  if (-im == 0.0) {
    re = ar / J_max;
    b_im = 0.0;
  } else if (ar == 0.0) {
    re = 0.0;
    b_im = -im / J_max;
  } else {
    re = ar / J_max;
    b_im = -im / J_max;
  }
  t[1].re = re;
  t[1].im = b_im;
  t[3].re = 0.0;
  t[3].im = 0.0;
  t[5] = t3[1];
  t[7].re = 0.0;
  t[7].im = 0.0;
  t[9].re = 0.0;
  t[9].im = 0.0;
  ar_tmp = l26_tmp + l3;
  ar = (ar_tmp - J_max * t3[1].re) + J_max_re;
  l5 = im - J_max * t3[1].im;
  if (l5 == 0.0) {
    t[11].re = ar / J_max;
    t[11].im = 0.0;
  } else if (ar == 0.0) {
    t[11].re = 0.0;
    t[11].im = l5 / J_max;
  } else {
    t[11].re = ar / J_max;
    t[11].im = l5 / J_max;
  }
  ar = -(ar_tmp + J_max * re);
  l5 = -(im + J_max * b_im);
  if (l5 == 0.0) {
    t[13].re = ar / J_max;
    t[13].im = 0.0;
  } else if (ar == 0.0) {
    t[13].re = 0.0;
    t[13].im = l5 / J_max;
  } else {
    t[13].re = ar / J_max;
    t[13].im = l5 / J_max;
  }
}

// End of code generation (acefg_T_A.cpp)
