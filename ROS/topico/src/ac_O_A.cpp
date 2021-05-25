//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// ac_O_A.cpp
//
// Code generation for function 'ac_O_A'
//

// Include files
#include "ac_O_A.h"
#include "rt_nonfinite.h"
#include "sqrt.h"
#include "topico_wrapper_data.h"

// Function Definitions
void ac_O_A(double V_init, double A_init, double A_wayp, double V_min,
            double J_max, double J_min, creal_T t[14])
{
  creal_T t3[2];
  creal_T l22;
  double l19;
  double l3;
  double l5;
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
  l3 = A_wayp * A_wayp;
  l5 = A_wayp * J_min;
  l6 = A_wayp * J_max;
  l7 = J_min * J_max;
  l19 = 1.0 / (J_min * J_min + -l7);
  l22.re = (J_min + -J_max) *
           ((((V_min * l7 * 2.0 + J_min * (A_init * A_init)) + J_min * l3) +
             -(V_init * l7 * 2.0)) +
            l3 * -J_max);
  l22.im = 0.0;
  coder::internal::scalar::b_sqrt(&l22);
  t3[0].re = l19 * ((l5 - l6) + l22.re);
  t3[0].im = l19 * l22.im;
  t3[1].re = -l19 * ((-l5 + l6) + l22.re);
  t3[1].im = -l19 * l22.im;
  l22.re = A_init - A_wayp;
  l3 = -(l22.re + J_min * t3[0].re);
  l5 = -(J_min * t3[0].im);
  if (l5 == 0.0) {
    t[0].re = l3 / J_max;
    t[0].im = 0.0;
  } else if (l3 == 0.0) {
    t[0].re = 0.0;
    t[0].im = l5 / J_max;
  } else {
    t[0].re = l3 / J_max;
    t[0].im = l5 / J_max;
  }
  t[2].re = 0.0;
  t[2].im = 0.0;
  t[4] = t3[0];
  t[6].re = 0.0;
  t[6].im = 0.0;
  t[8].re = 0.0;
  t[8].im = 0.0;
  t[10].re = 0.0;
  t[10].im = 0.0;
  t[12].re = 0.0;
  t[12].im = 0.0;
  l3 = -(l22.re + J_min * t3[1].re);
  l5 = -(J_min * t3[1].im);
  if (l5 == 0.0) {
    t[1].re = l3 / J_max;
    t[1].im = 0.0;
  } else if (l3 == 0.0) {
    t[1].re = 0.0;
    t[1].im = l5 / J_max;
  } else {
    t[1].re = l3 / J_max;
    t[1].im = l5 / J_max;
  }
  t[3].re = 0.0;
  t[3].im = 0.0;
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

// End of code generation (ac_O_A.cpp)
