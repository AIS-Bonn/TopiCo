//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// abceg_T_AV.cpp
//
// Code generation for function 'abceg_T_AV'
//

// Include files
#include "abceg_T_AV.h"
#include "rt_nonfinite.h"
#include "sqrt.h"
#include "topico_wrapper_data.h"

// Function Definitions
void abceg_T_AV(double V_init, double A_init, double V_wayp, double A_wayp,
                double A_max, double J_max, double J_min, double T,
                creal_T t[14])
{
  creal_T t7[2];
  creal_T l25;
  double A_max_re_tmp;
  double l22;
  double l5;
  double l6;
  double l7;
  double l8;
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
  //  Generated on 03-Sep-2019 11:17:57
  l5 = A_max * J_min;
  l6 = A_max * J_max;
  l7 = A_wayp * J_min;
  l8 = A_wayp * J_max;
  l22 = 1.0 / (J_max * J_max + -(J_min * J_max));
  l25.re =
      J_min * (J_min + -J_max) *
      ((((((A_wayp * A_wayp + J_max * V_init * 2.0) + A_init * A_max * 2.0) +
          -(J_max * V_wayp * 2.0)) +
         T * l6 * 2.0) +
        -(A_max * A_wayp * 2.0)) +
       -(A_init * A_init));
  l25.im = 0.0;
  coder::internal::scalar::b_sqrt(&l25);
  t7[0].re = l22 * ((((l5 - l6) - l7) + l8) + l25.re);
  t7[0].im = l22 * l25.im;
  t7[1].re = -l22 * ((((-l5 + l6) + l7) - l8) + l25.re);
  t7[1].im = -l22 * l25.im;
  l6 = -(1.0 / J_max * (A_init + -A_max));
  l25.re = (A_init - A_max) / J_max;
  A_max_re_tmp = A_max - A_wayp;
  t[0].re = l6;
  t[0].im = 0.0;
  t[1].re = l6;
  t[1].im = 0.0;
  l5 = J_max * t7[0].im;
  l22 = A_max_re_tmp + J_max * t7[0].re;
  if (l5 == 0.0) {
    l7 = l22 / J_min;
    l8 = 0.0;
  } else if (l22 == 0.0) {
    l7 = 0.0;
    l8 = l5 / J_min;
  } else {
    l7 = l22 / J_min;
    l8 = l5 / J_min;
  }
  t[2].re = ((T - t7[0].re) + l25.re) + l7;
  t[2].im = (0.0 - t7[0].im) + l8;
  if (-l5 == 0.0) {
    t[4].re = -l22 / J_min;
    t[4].im = 0.0;
  } else if (-l22 == 0.0) {
    t[4].re = 0.0;
    t[4].im = -l5 / J_min;
  } else {
    t[4].re = -l22 / J_min;
    t[4].im = -l5 / J_min;
  }
  t[6].re = 0.0;
  t[6].im = 0.0;
  t[8].re = 0.0;
  t[8].im = 0.0;
  t[10].re = 0.0;
  t[10].im = 0.0;
  t[12] = t7[0];
  l5 = J_max * t7[1].im;
  l22 = A_max_re_tmp + J_max * t7[1].re;
  if (l5 == 0.0) {
    l7 = l22 / J_min;
    l8 = 0.0;
  } else if (l22 == 0.0) {
    l7 = 0.0;
    l8 = l5 / J_min;
  } else {
    l7 = l22 / J_min;
    l8 = l5 / J_min;
  }
  t[3].re = ((T - t7[1].re) + l25.re) + l7;
  t[3].im = (0.0 - t7[1].im) + l8;
  if (-l5 == 0.0) {
    t[5].re = -l22 / J_min;
    t[5].im = 0.0;
  } else if (-l22 == 0.0) {
    t[5].re = 0.0;
    t[5].im = -l5 / J_min;
  } else {
    t[5].re = -l22 / J_min;
    t[5].im = -l5 / J_min;
  }
  t[7].re = 0.0;
  t[7].im = 0.0;
  t[9].re = 0.0;
  t[9].im = 0.0;
  t[11].re = 0.0;
  t[11].im = 0.0;
  t[13] = t7[1];
}

// End of code generation (abceg_T_AV.cpp)
