//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// acefg_T_AV.cpp
//
// Code generation for function 'acefg_T_AV'
//

// Include files
#include "acefg_T_AV.h"
#include "rt_nonfinite.h"
#include "sqrt.h"
#include "topico_wrapper_data.h"

// Function Definitions
void acefg_T_AV(double V_init, double A_init, double V_wayp, double A_wayp,
                double A_min, double J_max, double J_min, double T,
                creal_T t[14])
{
  creal_T t1[2];
  creal_T l26;
  double J_min_re;
  double ar;
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
  //  Generated on 03-Sep-2019 11:28:47
  l5 = A_init * J_min;
  l6 = A_init * J_max;
  l7 = A_min * J_min;
  l8 = A_min * J_max;
  l22 = 1.0 / (J_max * J_max + -(J_min * J_max));
  l26.re =
      -(J_min * (J_min + -J_max) *
        ((((((A_wayp * A_wayp + J_max * V_init * 2.0) + A_init * A_min * 2.0) +
            -(J_max * V_wayp * 2.0)) +
           T * l8 * 2.0) +
          -(A_min * A_wayp * 2.0)) +
         -(A_init * A_init)));
  l26.im = 0.0;
  coder::internal::scalar::b_sqrt(&l26);
  t1[0].re = l22 * ((((l5 - l6) - l7) + l8) + l26.re);
  t1[0].im = l22 * l26.im;
  t1[1].re = -l22 * ((((-l5 + l6) + l7) - l8) + l26.re);
  t1[1].im = -l22 * l26.im;
  l6 = -(1.0 / J_max * (A_min + -A_wayp));
  l8 = A_init - A_min;
  l22 = J_min * T;
  J_min_re = J_min * (A_min - A_wayp) / J_max;
  l5 = J_max * t1[0].re;
  l7 = J_max * t1[0].im;
  t[0] = t1[0];
  t[2].re = 0.0;
  t[2].im = 0.0;
  ar = -(l8 + l5);
  if (-l7 == 0.0) {
    t[4].re = ar / J_min;
    t[4].im = 0.0;
  } else if (ar == 0.0) {
    t[4].re = 0.0;
    t[4].im = -l7 / J_min;
  } else {
    t[4].re = ar / J_min;
    t[4].im = -l7 / J_min;
  }
  t[6].re = 0.0;
  t[6].im = 0.0;
  t[8].re = 0.0;
  t[8].im = 0.0;
  ar = (((l8 - J_min * t1[0].re) + l5) + l22) + J_min_re;
  l5 = (0.0 - J_min * t1[0].im) + l7;
  if (l5 == 0.0) {
    t[10].re = ar / J_min;
    t[10].im = 0.0;
  } else if (ar == 0.0) {
    t[10].re = 0.0;
    t[10].im = l5 / J_min;
  } else {
    t[10].re = ar / J_min;
    t[10].im = l5 / J_min;
  }
  l5 = J_max * t1[1].re;
  l7 = J_max * t1[1].im;
  t[1] = t1[1];
  t[3].re = 0.0;
  t[3].im = 0.0;
  ar = -(l8 + l5);
  if (-l7 == 0.0) {
    t[5].re = ar / J_min;
    t[5].im = 0.0;
  } else if (ar == 0.0) {
    t[5].re = 0.0;
    t[5].im = -l7 / J_min;
  } else {
    t[5].re = ar / J_min;
    t[5].im = -l7 / J_min;
  }
  t[7].re = 0.0;
  t[7].im = 0.0;
  t[9].re = 0.0;
  t[9].im = 0.0;
  ar = (((l8 - J_min * t1[1].re) + l5) + l22) + J_min_re;
  l5 = (0.0 - J_min * t1[1].im) + l7;
  if (l5 == 0.0) {
    t[11].re = ar / J_min;
    t[11].im = 0.0;
  } else if (ar == 0.0) {
    t[11].re = 0.0;
    t[11].im = l5 / J_min;
  } else {
    t[11].re = ar / J_min;
    t[11].im = l5 / J_min;
  }
  t[12].re = l6;
  t[12].im = 0.0;
  t[13].re = l6;
  t[13].im = 0.0;
}

// End of code generation (acefg_T_AV.cpp)
