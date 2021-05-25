//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// ab_O_P.cpp
//
// Code generation for function 'ab_O_P'
//

// Include files
#include "ab_O_P.h"
#include "rt_nonfinite.h"
#include "sqrt.h"
#include "topico_wrapper_rtwutil.h"

// Function Definitions
void ab_O_P(double P_init, double V_init, double A_init, double P_wayp,
            double A_max, double J_max, creal_T t[14])
{
  creal_T l32;
  double l2;
  double l5;
  double l6;
  double l7;
  double l9_tmp;
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
  l2 = A_init * A_init;
  l5 = A_max * A_max;
  l7 = J_max * J_max;
  l9_tmp = J_max * V_init;
  l32.re = ((((((((l2 * l2 * 3.0 + -(l5 * l5)) +
                  A_init * A_max * J_max * V_init * 24.0) +
                 -(A_max * rt_powd_snf(A_init, 3.0) * 8.0)) +
                A_max * P_wayp * l7 * 24.0) +
               l2 * l5 * 6.0) +
              -(A_max * P_init * l7 * 24.0)) +
             -(l9_tmp * l2 * 12.0)) +
            -(l9_tmp * l5 * 12.0)) +
           l7 * (V_init * V_init) * 12.0;
  l32.im = 0.0;
  coder::internal::scalar::b_sqrt(&l32);
  l32.re *= 1.7320508075688772;
  l32.im *= 1.7320508075688772;
  l6 = -(1.0 / J_max * (A_init + -A_max));
  t[0].re = l6;
  t[0].im = 0.0;
  t[1].re = l6;
  t[1].im = 0.0;
  l7 = (l9_tmp * 6.0 + l5 * 3.0) + -(l2 * 3.0);
  l6 = 1.0 / A_max * (1.0 / J_max);
  t[2].re = -0.16666666666666666 * (l6 * (l7 + l32.re));
  t[2].im = -0.16666666666666666 * (l6 * l32.im);
  t[3].re = -0.16666666666666666 * (l6 * (l7 - l32.re));
  t[3].im = -0.16666666666666666 * (l6 * (0.0 - l32.im));
  t[4].re = 0.0;
  t[4].im = 0.0;
  t[6].re = 0.0;
  t[6].im = 0.0;
  t[8].re = 0.0;
  t[8].im = 0.0;
  t[10].re = 0.0;
  t[10].im = 0.0;
  t[12].re = 0.0;
  t[12].im = 0.0;
  t[5].re = 0.0;
  t[5].im = 0.0;
  t[7].re = 0.0;
  t[7].im = 0.0;
  t[9].re = 0.0;
  t[9].im = 0.0;
  t[11].re = 0.0;
  t[11].im = 0.0;
  t[13].re = 0.0;
  t[13].im = 0.0;
}

// End of code generation (ab_O_P.cpp)
