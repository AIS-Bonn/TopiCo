//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// aceg_T_V.cpp
//
// Code generation for function 'aceg_T_V'
//

// Include files
#include "aceg_T_V.h"
#include "rt_nonfinite.h"
#include "sqrt.h"
#include "topico_wrapper_data.h"
#include "topico_wrapper_rtwutil.h"
#include "topico_wrapper_types.h"
#include <cmath>

// Function Definitions
void aceg_T_V(double V_init, double A_init, double V_wayp, double V_min,
              double J_max, double J_min, double T, creal_T t[14])
{
  creal_T t3[2];
  creal_T l11;
  double A_init_re;
  double T_re;
  double ai;
  double ar;
  double b_re;
  double im;
  double l11_tmp;
  double l9;
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
  //  Generated on 02-Sep-2019 15:22:52
  if (J_max < 0.0) {
    f_rtErrorWithMessageID("sqrt", n_emlrtRTEI.fName, n_emlrtRTEI.lineNo);
  }
  l9 = 1.0 / (J_min + -J_max);
  l11.re = V_wayp + -V_min;
  l11.im = 0.0;
  coder::internal::scalar::b_sqrt(&l11);
  l11_tmp = 1.4142135623730951 * std::sqrt(J_max);
  l11.re *= l11_tmp;
  l11.im *= l11_tmp;
  l11_tmp = A_init + J_max * T;
  t3[0].re = -l9 * (l11_tmp + l11.re);
  t3[0].im = -l9 * l11.im;
  t3[1].re = -l9 * (l11_tmp - l11.re);
  t3[1].im = -l9 * (0.0 - l11.im);
  l11.re = V_init * 2.0 - V_wayp * 2.0;
  T_re = T * 2.0;
  re = T + -t3[0].re;
  b_re = J_min * t3[0].re;
  im = J_min * t3[0].im;
  l11_tmp = re * -t3[0].im;
  l9 = t3[0].re * t3[0].im;
  A_init_re = A_init + b_re;
  ar = -((((l11.re + A_init * t3[0].re * 2.0) +
           J_max * (re * re - -t3[0].im * -t3[0].im)) +
          J_min * (t3[0].re * t3[0].re - t3[0].im * t3[0].im)) +
         (re * A_init_re - -t3[0].im * im) * 2.0);
  ai = -(((A_init * t3[0].im * 2.0 + J_max * (l11_tmp + l11_tmp)) +
          J_min * (l9 + l9)) +
         (re * im + -t3[0].im * A_init_re) * 2.0);
  A_init_re = ((J_max * re * 2.0 - b_re * 2.0) + J_max * t3[0].re * 2.0) -
              J_max * (T_re - t3[0].re * 2.0);
  b_re = ((J_max * -t3[0].im * 2.0 - im * 2.0) + J_max * t3[0].im * 2.0) -
         J_max * (0.0 - t3[0].im * 2.0);
  if (b_re == 0.0) {
    if (ai == 0.0) {
      re = ar / A_init_re;
      im = 0.0;
    } else if (ar == 0.0) {
      re = 0.0;
      im = ai / A_init_re;
    } else {
      re = ar / A_init_re;
      im = ai / A_init_re;
    }
  } else if (A_init_re == 0.0) {
    if (ar == 0.0) {
      re = ai / b_re;
      im = 0.0;
    } else if (ai == 0.0) {
      re = 0.0;
      im = -(ar / b_re);
    } else {
      re = ai / b_re;
      im = -(ar / b_re);
    }
  } else {
    im = std::abs(A_init_re);
    l11_tmp = std::abs(b_re);
    if (im > l11_tmp) {
      l9 = b_re / A_init_re;
      l11_tmp = A_init_re + l9 * b_re;
      re = (ar + l9 * ai) / l11_tmp;
      im = (ai - l9 * ar) / l11_tmp;
    } else if (l11_tmp == im) {
      if (A_init_re > 0.0) {
        l9 = 0.5;
      } else {
        l9 = -0.5;
      }
      if (b_re > 0.0) {
        l11_tmp = 0.5;
      } else {
        l11_tmp = -0.5;
      }
      re = (ar * l9 + ai * l11_tmp) / im;
      im = (ai * l9 - ar * l11_tmp) / im;
    } else {
      l9 = A_init_re / b_re;
      l11_tmp = b_re + l9 * A_init_re;
      re = (l9 * ar + ai) / l11_tmp;
      im = (l9 * ai - ar) / l11_tmp;
    }
  }
  t[0].re = re;
  t[0].im = im;
  t[2].re = 0.0;
  t[2].im = 0.0;
  t[4] = t3[0];
  t[6].re = 0.0;
  t[6].im = 0.0;
  t[8].re = 0.0;
  t[8].im = 0.0;
  t[10].re = 0.0;
  t[10].im = 0.0;
  t[12].re = (T - re) - t3[0].re;
  t[12].im = (0.0 - im) - t3[0].im;
  re = T + -t3[1].re;
  b_re = J_min * t3[1].re;
  im = J_min * t3[1].im;
  l11_tmp = re * -t3[1].im;
  l9 = t3[1].re * t3[1].im;
  A_init_re = A_init + b_re;
  ar = -((((l11.re + A_init * t3[1].re * 2.0) +
           J_max * (re * re - -t3[1].im * -t3[1].im)) +
          J_min * (t3[1].re * t3[1].re - t3[1].im * t3[1].im)) +
         (re * A_init_re - -t3[1].im * im) * 2.0);
  ai = -(((A_init * t3[1].im * 2.0 + J_max * (l11_tmp + l11_tmp)) +
          J_min * (l9 + l9)) +
         (re * im + -t3[1].im * A_init_re) * 2.0);
  A_init_re = ((J_max * re * 2.0 - b_re * 2.0) + J_max * t3[1].re * 2.0) -
              J_max * (T_re - t3[1].re * 2.0);
  b_re = ((J_max * -t3[1].im * 2.0 - im * 2.0) + J_max * t3[1].im * 2.0) -
         J_max * (0.0 - t3[1].im * 2.0);
  if (b_re == 0.0) {
    if (ai == 0.0) {
      re = ar / A_init_re;
      im = 0.0;
    } else if (ar == 0.0) {
      re = 0.0;
      im = ai / A_init_re;
    } else {
      re = ar / A_init_re;
      im = ai / A_init_re;
    }
  } else if (A_init_re == 0.0) {
    if (ar == 0.0) {
      re = ai / b_re;
      im = 0.0;
    } else if (ai == 0.0) {
      re = 0.0;
      im = -(ar / b_re);
    } else {
      re = ai / b_re;
      im = -(ar / b_re);
    }
  } else {
    im = std::abs(A_init_re);
    l11_tmp = std::abs(b_re);
    if (im > l11_tmp) {
      l9 = b_re / A_init_re;
      l11_tmp = A_init_re + l9 * b_re;
      re = (ar + l9 * ai) / l11_tmp;
      im = (ai - l9 * ar) / l11_tmp;
    } else if (l11_tmp == im) {
      if (A_init_re > 0.0) {
        l9 = 0.5;
      } else {
        l9 = -0.5;
      }
      if (b_re > 0.0) {
        l11_tmp = 0.5;
      } else {
        l11_tmp = -0.5;
      }
      re = (ar * l9 + ai * l11_tmp) / im;
      im = (ai * l9 - ar * l11_tmp) / im;
    } else {
      l9 = A_init_re / b_re;
      l11_tmp = b_re + l9 * A_init_re;
      re = (l9 * ar + ai) / l11_tmp;
      im = (l9 * ai - ar) / l11_tmp;
    }
  }
  t[1].re = re;
  t[1].im = im;
  t[3].re = 0.0;
  t[3].im = 0.0;
  t[5] = t3[1];
  t[7].re = 0.0;
  t[7].im = 0.0;
  t[9].re = 0.0;
  t[9].im = 0.0;
  t[11].re = 0.0;
  t[11].im = 0.0;
  t[13].re = (T - re) - t3[1].re;
  t[13].im = (0.0 - im) - t3[1].im;
}

// End of code generation (aceg_T_V.cpp)
