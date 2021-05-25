//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// abc_O_AP.cpp
//
// Code generation for function 'abc_O_AP'
//

// Include files
#include "abc_O_AP.h"
#include "rt_nonfinite.h"
#include "sqrt.h"
#include "topico_wrapper_rtwutil.h"

// Function Definitions
void abc_O_AP(double P_init, double V_init, double A_init, double P_wayp,
              double A_wayp, double A_max, double J_max, double J_min,
              creal_T t[14])
{
  creal_T l45;
  double l10;
  double l11;
  double l2;
  double l20;
  double l21;
  double l45_tmp;
  double l5;
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
  l2 = A_init * A_init;
  l5 = A_max * A_max;
  l10 = J_min * J_min;
  l11 = J_max * J_max;
  l7 = l5 * l5;
  l20 = J_min * l5 * 3.0;
  l21 = J_max * l5 * 6.0;
  l45_tmp = J_max * V_init;
  l45.re = ((((((((((((l2 * l2 * l10 * 3.0 + l7 * l11 * 4.0) + -(l7 * l10)) +
                     A_init * A_max * J_max * V_init * l10 * 24.0) +
                    -(A_max * rt_powd_snf(A_wayp, 3.0) * l11 * 4.0)) +
                   A_max * P_wayp * l10 * l11 * 24.0) +
                  l2 * l5 * l10 * 6.0) +
                 -(A_max * rt_powd_snf(A_init, 3.0) * l10 * 8.0)) +
                -(A_wayp * rt_powd_snf(A_max, 3.0) * l11 * 12.0)) +
               -(A_max * P_init * l10 * l11 * 24.0)) +
              -(l45_tmp * l2 * l10 * 12.0)) +
             -(l45_tmp * l5 * l10 * 12.0)) +
            l5 * (A_wayp * A_wayp) * l11 * 12.0) +
           l10 * l11 * (V_init * V_init) * 12.0;
  l45.im = 0.0;
  coder::internal::scalar::b_sqrt(&l45);
  l45.re *= 1.7320508075688772;
  l45.im *= 1.7320508075688772;
  l5 = 1.0 / J_min * (A_max + -A_wayp);
  l11 = -(1.0 / J_max * (A_init + -A_max));
  t[0].re = l11;
  t[0].im = 0.0;
  t[1].re = l11;
  t[1].im = 0.0;
  l11 = (((A_max * A_wayp * J_max * 6.0 + J_min * J_max * V_init * 6.0) -
          J_min * l2 * 3.0) +
         l20) -
        l21;
  l10 = 1.0 / A_max * (1.0 / J_min) * (1.0 / J_max);
  t[2].re = -0.16666666666666666 * (l10 * (l11 + l45.re));
  t[2].im = -0.16666666666666666 * (l10 * l45.im);
  t[3].re = -0.16666666666666666 * (l10 * (l11 - l45.re));
  t[3].im = -0.16666666666666666 * (l10 * (0.0 - l45.im));
  t[4].re = -l5;
  t[4].im = 0.0;
  t[5].re = -l5;
  t[5].im = 0.0;
  t[6].re = 0.0;
  t[6].im = 0.0;
  t[8].re = 0.0;
  t[8].im = 0.0;
  t[10].re = 0.0;
  t[10].im = 0.0;
  t[12].re = 0.0;
  t[12].im = 0.0;
  t[7].re = 0.0;
  t[7].im = 0.0;
  t[9].re = 0.0;
  t[9].im = 0.0;
  t[11].re = 0.0;
  t[11].im = 0.0;
  t[13].re = 0.0;
  t[13].im = 0.0;
}

// End of code generation (abc_O_AP.cpp)
