//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// aceg_T_AP.cpp
//
// Code generation for function 'aceg_T_AP'
//

// Include files
#include "aceg_T_AP.h"
#include "rt_nonfinite.h"
#include "sqrt.h"
#include "topico_wrapper_data.h"
#include "topico_wrapper_rtwutil.h"

// Function Definitions
void aceg_T_AP(double P_init, double V_init, double A_init, double P_wayp,
               double A_wayp, double J_max, double J_min, double T,
               creal_T t[14])
{
  creal_T t7[2];
  creal_T l68;
  double l10;
  double l11;
  double l12;
  double l14;
  double l15;
  double l16;
  double l19;
  double l2;
  double l21;
  double l23;
  double l4;
  double l52;
  double l6;
  double l68_tmp;
  double l7;
  double l9;
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
  l4 = A_wayp * A_wayp;
  l6 = J_min * J_min;
  l7 = J_max * J_max;
  l9 = T * T;
  l10 = rt_powd_snf(T, 3.0);
  l11 = A_init * J_min;
  l12 = A_init * J_max;
  l14 = A_wayp * J_max;
  l15 = J_max * T;
  l16 = J_min * l15;
  l19 = l2 * 3.0;
  l21 = T * l12 * 6.0;
  l23 = T * l7;
  l52 = l7 * l9 * 3.0;
  l68_tmp = J_min * J_max;
  l68.re = ((A_init + l15) + -A_wayp) *
           ((((((((((((((((((((((rt_powd_snf(A_wayp, 3.0) +
                                 -rt_powd_snf(A_init, 3.0)) +
                                A_wayp * l19) +
                               l68_tmp * P_wayp * 48.0) +
                              A_wayp * l21) +
                             A_init * l4 * -3.0) +
                            -(l68_tmp * P_init * 48.0)) +
                           P_init * l6 * 24.0) +
                          P_init * l7 * 24.0) +
                         -(P_wayp * l6 * 24.0)) +
                        -(P_wayp * l7 * 24.0)) +
                       -(V_init * l16 * 48.0)) +
                      l2 * l15 * -3.0) +
                     l4 * l15 * -3.0) +
                    T * V_init * l6 * 24.0) +
                   V_init * l23 * 24.0) +
                  rt_powd_snf(J_max, 3.0) * l10 * 3.0) +
                 J_max * l6 * l10 * 4.0) +
                A_wayp * l52) +
               -(J_max * l9 * l11 * 24.0)) +
              A_init * l7 * l9 * 9.0) +
             A_init * l6 * l9 * 12.0) +
            -(J_min * l7 * l10 * 8.0));
  l68.im = 0.0;
  coder::internal::scalar::b_sqrt(&l68);
  l68.re *= 1.7320508075688772;
  l68.im *= 1.7320508075688772;
  l6 = ((((l19 + l4 * 3.0) + l21) + -(A_init * A_wayp * 6.0)) +
        -(T * l14 * 6.0)) +
       l52;
  l7 = l6 + l68.re;
  l2 = ((((l11 * 6.0 - l12 * 6.0) - A_wayp * J_min * 6.0) + l14 * 6.0) +
        l16 * 6.0) -
       l23 * 6.0;
  if (l68.im == 0.0) {
    t7[0].re = l7 / l2;
    t7[0].im = 0.0;
  } else if (l7 == 0.0) {
    t7[0].re = 0.0;
    t7[0].im = l68.im / l2;
  } else {
    t7[0].re = l7 / l2;
    t7[0].im = l68.im / l2;
  }
  l7 = l6 - l68.re;
  if (0.0 - l68.im == 0.0) {
    t7[1].re = l7 / l2;
    t7[1].im = 0.0;
  } else if (l7 == 0.0) {
    t7[1].re = 0.0;
    t7[1].im = (0.0 - l68.im) / l2;
  } else {
    t7[1].re = l7 / l2;
    t7[1].im = (0.0 - l68.im) / l2;
  }
  l2 = ((A_init + J_max * T) + -A_wayp) * (1.0 / (J_min + -J_max));
  l6 = (A_init - A_wayp) + J_min * -l2;
  l7 = -(l6 + J_max * t7[0].re);
  l4 = -(J_max * t7[0].im);
  if (l4 == 0.0) {
    t[0].re = l7 / J_max;
    t[0].im = 0.0;
  } else if (l7 == 0.0) {
    t[0].re = 0.0;
    t[0].im = l4 / J_max;
  } else {
    t[0].re = l7 / J_max;
    t[0].im = l4 / J_max;
  }
  t[2].re = 0.0;
  t[2].im = 0.0;
  t[4].re = -l2;
  t[4].im = 0.0;
  t[6].re = 0.0;
  t[6].im = 0.0;
  t[8].re = 0.0;
  t[8].im = 0.0;
  t[10].re = 0.0;
  t[10].im = 0.0;
  t[12] = t7[0];
  l7 = -(l6 + J_max * t7[1].re);
  l4 = -(J_max * t7[1].im);
  if (l4 == 0.0) {
    t[1].re = l7 / J_max;
    t[1].im = 0.0;
  } else if (l7 == 0.0) {
    t[1].re = 0.0;
    t[1].im = l4 / J_max;
  } else {
    t[1].re = l7 / J_max;
    t[1].im = l4 / J_max;
  }
  t[3].re = 0.0;
  t[3].im = 0.0;
  t[5].re = -l2;
  t[5].im = 0.0;
  t[7].re = 0.0;
  t[7].im = 0.0;
  t[9].re = 0.0;
  t[9].im = 0.0;
  t[11].re = 0.0;
  t[11].im = 0.0;
  t[13] = t7[1];
}

// End of code generation (aceg_T_AP.cpp)
