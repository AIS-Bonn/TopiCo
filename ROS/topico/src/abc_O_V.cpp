//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// abc_O_V.cpp
//
// Code generation for function 'abc_O_V'
//

// Include files
#include "abc_O_V.h"
#include "rt_nonfinite.h"
#include "sqrt.h"
#include "topico_wrapper_data.h"
#include "topico_wrapper_rtwutil.h"

// Function Definitions
void abc_O_V(double V_init, double A_init, double V_wayp, double V_max,
             double A_max, double J_max, double J_min, creal_T t[14])
{
  creal_T t3[2];
  creal_T l11;
  double b_J_min;
  double l3;
  double l4;
  double y_tmp;
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
  l3 = A_max * J_min;
  l4 = 1.0 / (J_min * J_min);
  l11.re = -(rt_powd_snf(J_min, 3.0) * (V_max + -V_wayp));
  l11.im = 0.0;
  coder::internal::scalar::b_sqrt(&l11);
  l11.re *= 1.4142135623730951;
  l11.im *= 1.4142135623730951;
  t3[0].re = -l4 * (l3 - l11.re);
  t3[0].im = -l4 * (0.0 - l11.im);
  t3[1].re = -l4 * (l3 + l11.re);
  t3[1].im = -l4 * l11.im;
  y_tmp = A_max * J_max;
  l3 = -(1.0 / J_max * (A_init + -A_max));
  l11.re = ((J_max * V_init * 2.0 - J_max * V_wayp * 2.0) - A_init * A_init) +
           A_max * A_max;
  b_J_min = J_min * J_max;
  t[0].re = l3;
  t[0].im = 0.0;
  t[1].re = l3;
  t[1].im = 0.0;
  l3 = t3[0].re * t3[0].im;
  l4 = ((l11.re + y_tmp * t3[0].re * 2.0) +
        b_J_min * (t3[0].re * t3[0].re - t3[0].im * t3[0].im)) *
       -0.5;
  l3 = (y_tmp * t3[0].im * 2.0 + b_J_min * (l3 + l3)) * -0.5;
  if (l3 == 0.0) {
    t[2].re = l4 / y_tmp;
    t[2].im = 0.0;
  } else if (l4 == 0.0) {
    t[2].re = 0.0;
    t[2].im = l3 / y_tmp;
  } else {
    t[2].re = l4 / y_tmp;
    t[2].im = l3 / y_tmp;
  }
  t[4] = t3[0];
  t[6].re = 0.0;
  t[6].im = 0.0;
  t[8].re = 0.0;
  t[8].im = 0.0;
  t[10].re = 0.0;
  t[10].im = 0.0;
  t[12].re = 0.0;
  t[12].im = 0.0;
  l3 = t3[1].re * t3[1].im;
  l4 = ((l11.re + y_tmp * t3[1].re * 2.0) +
        b_J_min * (t3[1].re * t3[1].re - t3[1].im * t3[1].im)) *
       -0.5;
  l3 = (y_tmp * t3[1].im * 2.0 + b_J_min * (l3 + l3)) * -0.5;
  if (l3 == 0.0) {
    t[3].re = l4 / y_tmp;
    t[3].im = 0.0;
  } else if (l4 == 0.0) {
    t[3].re = 0.0;
    t[3].im = l3 / y_tmp;
  } else {
    t[3].re = l4 / y_tmp;
    t[3].im = l3 / y_tmp;
  }
  t[5] = t3[1];
  t[7].re = 0.0;
  t[7].im = 0.0;
  t[9].re = 0.0;
  t[9].im = 0.0;
  t[11].re = 0.0;
  t[11].im = 0.0;
  t[13].re = 0.0;
  t[13].im = 0.0;
}

// End of code generation (abc_O_V.cpp)
