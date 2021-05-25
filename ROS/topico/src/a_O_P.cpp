//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// a_O_P.cpp
//
// Code generation for function 'a_O_P'
//

// Include files
#include "a_O_P.h"
#include "power.h"
#include "rt_nonfinite.h"
#include "sqrt.h"
#include "topico_wrapper_data.h"
#include "topico_wrapper_rtwutil.h"
#include <cmath>

// Function Definitions
void a_O_P(double P_init, double V_init, double A_init, double P_wayp,
           double J_max, creal_T t[21])
{
  creal_T b_l14;
  creal_T l30;
  double l13;
  double l14;
  double l16;
  double l21;
  double l23;
  double l30_im;
  double l30_re;
  double l37_re;
  double l6;
  double l7;
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
  //  Generated on 28-Aug-2019 17:25:45
  l6 = 1.0 / J_max;
  l7 = l6 * l6;
  l13 = -(A_init * l6);
  l14 = A_init * V_init * l7 * 3.0;
  l16 = rt_powd_snf(A_init, 3.0) * rt_powd_snf(l6, 3.0);
  l21 = l6 * (P_init * 6.0 + -(P_wayp * 6.0)) / 2.0;
  l23 = V_init * l6 * 2.0 + -(A_init * A_init * l7);
  l6 = (l16 + -l14) + l21;
  l30.re = rt_powd_snf(l23, 3.0) + l6 * l6;
  l30.im = 0.0;
  coder::internal::scalar::b_sqrt(&l30);
  b_l14.re = ((l14 + -l16) + -l21) + l30.re;
  b_l14.im = l30.im;
  l30 = coder::power(b_l14);
  if (l30.im == 0.0) {
    l30_re = l30.re / 2.0;
    l30_im = 0.0;
    l16 = 1.0 / l30.re;
    l6 = 0.0;
  } else if (l30.re == 0.0) {
    l30_re = 0.0;
    l30_im = l30.im / 2.0;
    l16 = 0.0;
    l6 = -(1.0 / l30.im);
  } else {
    l30_re = l30.re / 2.0;
    l30_im = l30.im / 2.0;
    l14 = std::abs(l30.re);
    l6 = std::abs(l30.im);
    if (l14 > l6) {
      l6 = l30.im / l30.re;
      l7 = l30.re + l6 * l30.im;
      l16 = (l6 * 0.0 + 1.0) / l7;
      l6 = (0.0 - l6) / l7;
    } else if (l6 == l14) {
      if (l30.re > 0.0) {
        l6 = 0.5;
      } else {
        l6 = -0.5;
      }
      if (l30.im > 0.0) {
        l7 = 0.5;
      } else {
        l7 = -0.5;
      }
      l16 = (l6 + 0.0 * l7) / l14;
      l6 = (0.0 * l6 - l7) / l14;
    } else {
      l6 = l30.re / l30.im;
      l7 = l30.im + l6 * l30.re;
      l16 = l6 / l7;
      l6 = (l6 * 0.0 - 1.0) / l7;
    }
  }
  l21 = l23 * l16;
  l7 = l23 * l6;
  if (l7 == 0.0) {
    l14 = l21 / 2.0;
    l23 = 0.0;
  } else if (l21 == 0.0) {
    l14 = 0.0;
    l23 = l7 / 2.0;
  } else {
    l14 = l21 / 2.0;
    l23 = l7 / 2.0;
  }
  l16 = 1.7320508075688772 * (l30.re + l21);
  l6 = 1.7320508075688772 * (l30.im + l7);
  l37_re = l16 * 0.0 - l6 * 0.5;
  l16 = l16 * 0.5 + l6 * 0.0;
  t[0].re = (l13 + l30.re) - l21;
  t[0].im = l30.im - l7;
  l7 = (l13 + -l30_re) + l14;
  t[1].re = l7 - l37_re;
  l6 = -l30_im + l23;
  t[1].im = l6 - l16;
  t[2].re = l7 + l37_re;
  t[2].im = l6 + l16;
  t[3].re = 0.0;
  t[3].im = 0.0;
  t[6].re = 0.0;
  t[6].im = 0.0;
  t[9].re = 0.0;
  t[9].im = 0.0;
  t[12].re = 0.0;
  t[12].im = 0.0;
  t[15].re = 0.0;
  t[15].im = 0.0;
  t[18].re = 0.0;
  t[18].im = 0.0;
  t[4].re = 0.0;
  t[4].im = 0.0;
  t[7].re = 0.0;
  t[7].im = 0.0;
  t[10].re = 0.0;
  t[10].im = 0.0;
  t[13].re = 0.0;
  t[13].im = 0.0;
  t[16].re = 0.0;
  t[16].im = 0.0;
  t[19].re = 0.0;
  t[19].im = 0.0;
  t[5].re = 0.0;
  t[5].im = 0.0;
  t[8].re = 0.0;
  t[8].im = 0.0;
  t[11].re = 0.0;
  t[11].im = 0.0;
  t[14].re = 0.0;
  t[14].im = 0.0;
  t[17].re = 0.0;
  t[17].im = 0.0;
  t[20].re = 0.0;
  t[20].im = 0.0;
}

// End of code generation (a_O_P.cpp)
