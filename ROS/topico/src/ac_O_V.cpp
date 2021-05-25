//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// ac_O_V.cpp
//
// Code generation for function 'ac_O_V'
//

// Include files
#include "ac_O_V.h"
#include "rt_nonfinite.h"
#include "sqrt.h"
#include "topico_wrapper_data.h"

// Function Definitions
void ac_O_V(double V_init, double A_init, double V_wayp, double V_max,
            double J_max, double J_min, creal_T t[14])
{
  creal_T t1[2];
  creal_T l17;
  creal_T x;
  double l14;
  double l4;
  double l5;
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
  //  Generated on 05-Sep-2019 15:00:15
  l4 = A_init * J_min;
  l5 = A_init * J_max;
  l14 = 1.0 / (J_max * J_max + -(J_min * J_max));
  l17.re = J_min * (J_min + -J_max) *
           ((A_init * A_init + J_max * V_max * 2.0) + -(J_max * V_init * 2.0));
  l17.im = 0.0;
  coder::internal::scalar::b_sqrt(&l17);
  t1[0].re = l14 * ((l4 - l5) + l17.re);
  t1[0].im = l14 * l17.im;
  t1[1].re = -l14 * ((-l4 + l5) + l17.re);
  t1[1].im = -l14 * l17.im;
  l17.re = -J_min;
  l17.im = 0.0;
  coder::internal::scalar::b_sqrt(&l17);
  x.re = V_max - V_wayp;
  x.im = 0.0;
  coder::internal::scalar::b_sqrt(&x);
  l4 = 1.4142135623730951 * l17.re;
  l5 = 1.4142135623730951 * l17.im;
  l17.re = l4 * x.re - l5 * x.im;
  l17.im = l4 * x.im + l5 * x.re;
  t[0] = t1[0];
  t[2].re = 0.0;
  t[2].im = 0.0;
  l4 = -((A_init + J_max * t1[0].re) - l17.re);
  l5 = -(J_max * t1[0].im - l17.im);
  if (l5 == 0.0) {
    t[4].re = l4 / J_min;
    t[4].im = 0.0;
  } else if (l4 == 0.0) {
    t[4].re = 0.0;
    t[4].im = l5 / J_min;
  } else {
    t[4].re = l4 / J_min;
    t[4].im = l5 / J_min;
  }
  t[6].re = 0.0;
  t[6].im = 0.0;
  t[8].re = 0.0;
  t[8].im = 0.0;
  t[10].re = 0.0;
  t[10].im = 0.0;
  t[12].re = 0.0;
  t[12].im = 0.0;
  t[1] = t1[1];
  t[3].re = 0.0;
  t[3].im = 0.0;
  l4 = -((A_init + J_max * t1[1].re) - l17.re);
  l5 = -(J_max * t1[1].im - l17.im);
  if (l5 == 0.0) {
    t[5].re = l4 / J_min;
    t[5].im = 0.0;
  } else if (l4 == 0.0) {
    t[5].re = 0.0;
    t[5].im = l5 / J_min;
  } else {
    t[5].re = l4 / J_min;
    t[5].im = l5 / J_min;
  }
  t[7].re = 0.0;
  t[7].im = 0.0;
  t[9].re = 0.0;
  t[9].im = 0.0;
  t[11].re = 0.0;
  t[11].im = 0.0;
  t[13].re = 0.0;
  t[13].im = 0.0;
}

// End of code generation (ac_O_V.cpp)
