//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// acefg_T_AP.cpp
//
// Code generation for function 'acefg_T_AP'
//

// Include files
#include "acefg_T_AP.h"
#include "power.h"
#include "rt_nonfinite.h"
#include "sqrt.h"
#include "topico_wrapper_data.h"
#include "topico_wrapper_rtwutil.h"
#include <cmath>

// Function Definitions
void acefg_T_AP(double P_init, double V_init, double A_init, double P_wayp,
                double A_wayp, double A_min, double J_max, double J_min,
                double T, creal_T t[21])
{
  creal_T t1[3];
  creal_T l78;
  creal_T l92;
  double l10;
  double l11;
  double l17;
  double l2;
  double l20_tmp;
  double l22;
  double l33;
  double l4;
  double l49;
  double l5;
  double l68;
  double l69;
  double l71;
  double l71_tmp;
  double l75;
  double l8;
  double l83_tmp;
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
  l4 = A_min * A_min;
  l5 = rt_powd_snf(A_min, 3.0);
  l8 = J_min * J_min;
  l9 = J_max * J_max;
  l10 = rt_powd_snf(J_max, 3.0);
  l11 = l9 * l9;
  l20_tmp = A_init * A_min;
  l22 = A_min * J_min * l10 * 6.0;
  l33 = l20_tmp * J_min * l9 * 6.0;
  l49 = A_min * l8 * l9 * 3.0;
  l17 = J_min * l11 * 3.0;
  l71_tmp = A_init * J_min;
  l71 = ((((((A_init * l11 * 3.0 + -(A_min * l11 * 3.0)) + l22) + T * l17) +
           -(l71_tmp * l10 * 6.0)) +
          A_init * l8 * l9 * 3.0) +
         -(T * l8 * l10 * 3.0)) +
        -l49;
  l68 = 1.0 / ((rt_powd_snf(J_max, 5.0) + -l17) + l8 * l10 * 2.0);
  l69 = l68 * l68;
  l75 = l68 * l71 / 3.0;
  l83_tmp = J_min * T;
  l5 = l68 *
       ((((((((((((((l5 * l9 + rt_powd_snf(A_wayp, 3.0) * l8) +
                    P_init * l8 * l9 * 6.0) +
                   T * l33) +
                  -(rt_powd_snf(A_init, 3.0) * l9)) +
                 -(l5 * l8)) +
                A_min * l2 * l9 * 3.0) +
               A_wayp * l4 * l8 * 3.0) +
              -(P_wayp * l8 * l9 * 6.0)) +
             T * V_init * l8 * l9 * 6.0) +
            -(A_init * l4 * l9 * 3.0)) +
           -(A_min * (A_wayp * A_wayp) * l8 * 3.0)) +
          l83_tmp * l2 * l9 * -3.0) +
         l83_tmp * l4 * l9 * -3.0) +
        T * T * l49) /
       2.0;
  l49 = rt_powd_snf(l68, 3.0) * rt_powd_snf(l71, 3.0) / 27.0;
  l11 = ((((((((l20_tmp * l10 * 6.0 - l2 * l10 * 3.0) - l4 * l10 * 3.0) - l33) +
             T * l22) +
            J_min * l2 * l9 * 3.0) +
           J_min * l4 * l9 * 3.0) +
          A_init * T * l8 * l9 * 6.0) -
         A_min * T * l8 * l9 * 6.0) -
        l71_tmp * T * l10 * 6.0;
  l4 = l69 * l71 * l11;
  l17 = (l49 - l5) + l4 / 6.0;
  l20_tmp = l69 * (l71 * l71) / 9.0 + l68 * l11 / 3.0;
  l92.re = -rt_powd_snf(l20_tmp, 3.0) + l17 * l17;
  l92.im = 0.0;
  coder::internal::scalar::b_sqrt(&l92);
  l78.re = ((-l49 + l5) + l4 * -0.16666666666666666) + l92.re;
  l78.im = l92.im;
  l92 = coder::power(l78);
  if (l92.im == 0.0) {
    l4 = 1.0 / l92.re;
    l17 = 0.0;
    l11 = l92.re / 2.0;
    l8 = 0.0;
  } else if (l92.re == 0.0) {
    l4 = 0.0;
    l17 = -(1.0 / l92.im);
    l11 = 0.0;
    l8 = l92.im / 2.0;
  } else {
    l5 = std::abs(l92.re);
    l11 = std::abs(l92.im);
    if (l5 > l11) {
      l11 = l92.im / l92.re;
      l17 = l92.re + l11 * l92.im;
      l4 = (l11 * 0.0 + 1.0) / l17;
      l17 = (0.0 - l11) / l17;
    } else if (l11 == l5) {
      if (l92.re > 0.0) {
        l11 = 0.5;
      } else {
        l11 = -0.5;
      }
      if (l92.im > 0.0) {
        l17 = 0.5;
      } else {
        l17 = -0.5;
      }
      l4 = (l11 + 0.0 * l17) / l5;
      l17 = (0.0 * l11 - l17) / l5;
    } else {
      l11 = l92.re / l92.im;
      l17 = l92.im + l11 * l92.re;
      l4 = l11 / l17;
      l17 = (l11 * 0.0 - 1.0) / l17;
    }
    l11 = l92.re / 2.0;
    l8 = l92.im / 2.0;
  }
  l49 = l20_tmp * l4;
  l9 = l20_tmp * l17;
  l10 = 1.7320508075688772 * (l92.re + -l4 * l20_tmp);
  l2 = 1.7320508075688772 * (l92.im + -l17 * l20_tmp);
  l4 = l10 * 0.0 - l2 * 0.5;
  l5 = l10 * 0.5 + l2 * 0.0;
  t1[0].re = (-l75 + l92.re) + l49;
  t1[0].im = l92.im + l9;
  l17 = (-l75 + -l11) + -0.5 * l49;
  t1[1].re = l17 - l4;
  l11 = -l8 + -0.5 * l9;
  t1[1].im = l11 - l5;
  t1[2].re = l17 + l4;
  t1[2].im = l11 + l5;
  l5 = -(1.0 / J_max * (A_min + -A_wayp));
  l20_tmp = A_init - A_min;
  l17 = J_min * (A_min - A_wayp) / J_max;
  l10 = J_max * t1[0].re;
  l2 = J_max * t1[0].im;
  t[0] = t1[0];
  t[3].re = 0.0;
  t[3].im = 0.0;
  l4 = -(l20_tmp + l10);
  if (-l2 == 0.0) {
    t[6].re = l4 / J_min;
    t[6].im = 0.0;
  } else if (l4 == 0.0) {
    t[6].re = 0.0;
    t[6].im = -l2 / J_min;
  } else {
    t[6].re = l4 / J_min;
    t[6].im = -l2 / J_min;
  }
  t[9].re = 0.0;
  t[9].im = 0.0;
  t[12].re = 0.0;
  t[12].im = 0.0;
  l4 = (((l20_tmp - J_min * t1[0].re) + l10) + l83_tmp) + l17;
  l11 = (0.0 - J_min * t1[0].im) + l2;
  if (l11 == 0.0) {
    t[15].re = l4 / J_min;
    t[15].im = 0.0;
  } else if (l4 == 0.0) {
    t[15].re = 0.0;
    t[15].im = l11 / J_min;
  } else {
    t[15].re = l4 / J_min;
    t[15].im = l11 / J_min;
  }
  l10 = J_max * t1[1].re;
  l2 = J_max * t1[1].im;
  t[1] = t1[1];
  t[4].re = 0.0;
  t[4].im = 0.0;
  l4 = -(l20_tmp + l10);
  if (-l2 == 0.0) {
    t[7].re = l4 / J_min;
    t[7].im = 0.0;
  } else if (l4 == 0.0) {
    t[7].re = 0.0;
    t[7].im = -l2 / J_min;
  } else {
    t[7].re = l4 / J_min;
    t[7].im = -l2 / J_min;
  }
  t[10].re = 0.0;
  t[10].im = 0.0;
  t[13].re = 0.0;
  t[13].im = 0.0;
  l4 = (((l20_tmp - J_min * t1[1].re) + l10) + l83_tmp) + l17;
  l11 = (0.0 - J_min * t1[1].im) + l2;
  if (l11 == 0.0) {
    t[16].re = l4 / J_min;
    t[16].im = 0.0;
  } else if (l4 == 0.0) {
    t[16].re = 0.0;
    t[16].im = l11 / J_min;
  } else {
    t[16].re = l4 / J_min;
    t[16].im = l11 / J_min;
  }
  l10 = J_max * t1[2].re;
  l2 = J_max * t1[2].im;
  t[2] = t1[2];
  t[5].re = 0.0;
  t[5].im = 0.0;
  l4 = -(l20_tmp + l10);
  if (-l2 == 0.0) {
    t[8].re = l4 / J_min;
    t[8].im = 0.0;
  } else if (l4 == 0.0) {
    t[8].re = 0.0;
    t[8].im = -l2 / J_min;
  } else {
    t[8].re = l4 / J_min;
    t[8].im = -l2 / J_min;
  }
  t[11].re = 0.0;
  t[11].im = 0.0;
  t[14].re = 0.0;
  t[14].im = 0.0;
  l4 = (((l20_tmp - J_min * t1[2].re) + l10) + l83_tmp) + l17;
  l11 = (0.0 - J_min * t1[2].im) + l2;
  if (l11 == 0.0) {
    t[17].re = l4 / J_min;
    t[17].im = 0.0;
  } else if (l4 == 0.0) {
    t[17].re = 0.0;
    t[17].im = l11 / J_min;
  } else {
    t[17].re = l4 / J_min;
    t[17].im = l11 / J_min;
  }
  t[18].re = l5;
  t[18].im = 0.0;
  t[19].re = l5;
  t[19].im = 0.0;
  t[20].re = l5;
  t[20].im = 0.0;
}

// End of code generation (acefg_T_AP.cpp)
