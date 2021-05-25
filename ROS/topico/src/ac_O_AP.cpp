//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// ac_O_AP.cpp
//
// Code generation for function 'ac_O_AP'
//

// Include files
#include "ac_O_AP.h"
#include "power.h"
#include "rt_nonfinite.h"
#include "sqrt.h"
#include "topico_wrapper_data.h"
#include "topico_wrapper_rtwutil.h"
#include <cmath>

// Function Definitions
void ac_O_AP(double P_init, double V_init, double A_init, double P_wayp,
             double A_wayp, double J_max, double J_min, creal_T t[21])
{
  creal_T t3[3];
  creal_T l53;
  creal_T l58;
  double l2;
  double l36;
  double l37;
  double l38;
  double l4;
  double l44;
  double l50;
  double l58_im;
  double l58_re;
  double l6;
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
  //  Generated on 28-Aug-2019 17:25:45
  l2 = A_init * A_init;
  l4 = A_wayp * A_wayp;
  l6 = J_min * J_min;
  l8 = J_max * J_max;
  l36 =
      (-(A_wayp * J_min * J_max * 6.0) + A_wayp * l6 * 3.0) + A_wayp * l8 * 3.0;
  l37 = 1.0 /
        ((rt_powd_snf(J_min, 3.0) + J_min * l8 * 2.0) + -(J_max * l6 * 3.0));
  l38 = l37 * l37;
  l44 = l36 * l37 / 3.0;
  l6 =
      ((((J_min * J_max * V_init * 6.0 - J_min * l2 * 3.0) + J_max * l2 * 3.0) +
        J_min * l4 * 3.0) -
       J_max * l4 * 3.0) -
      V_init * l8 * 6.0;
  l50 = l38 * (l36 * l36) / 9.0 + l37 * l6 * -0.33333333333333331;
  l4 = (rt_powd_snf(l37, 3.0) * rt_powd_snf(l36, 3.0) / 27.0 +
        l37 *
            ((((((rt_powd_snf(A_wayp, 3.0) + rt_powd_snf(A_init, 3.0) * 2.0) +
                 A_wayp * J_max * V_init * 6.0) +
                -(A_init * J_max * V_init * 6.0)) +
               P_init * l8 * 6.0) +
              -(A_wayp * l2 * 3.0)) +
             -(P_wayp * l8 * 6.0)) /
            2.0) +
       l36 * l38 * l6 * -0.16666666666666666;
  l58.re = -rt_powd_snf(l50, 3.0) + l4 * l4;
  l58.im = 0.0;
  coder::internal::scalar::b_sqrt(&l58);
  l53.re = l4 + l58.re;
  l53.im = l58.im;
  l58 = coder::power(l53);
  if (l58.im == 0.0) {
    l58_re = l58.re / 2.0;
    l58_im = 0.0;
    l38 = 1.0 / l58.re;
    l6 = 0.0;
  } else if (l58.re == 0.0) {
    l58_re = 0.0;
    l58_im = l58.im / 2.0;
    l38 = 0.0;
    l6 = -(1.0 / l58.im);
  } else {
    l58_re = l58.re / 2.0;
    l58_im = l58.im / 2.0;
    l37 = std::abs(l58.re);
    l4 = std::abs(l58.im);
    if (l37 > l4) {
      l4 = l58.im / l58.re;
      l6 = l58.re + l4 * l58.im;
      l38 = (l4 * 0.0 + 1.0) / l6;
      l6 = (0.0 - l4) / l6;
    } else if (l4 == l37) {
      if (l58.re > 0.0) {
        l6 = 0.5;
      } else {
        l6 = -0.5;
      }
      if (l58.im > 0.0) {
        l4 = 0.5;
      } else {
        l4 = -0.5;
      }
      l38 = (l6 + 0.0 * l4) / l37;
      l6 = (0.0 * l6 - l4) / l37;
    } else {
      l4 = l58.re / l58.im;
      l6 = l58.im + l4 * l58.re;
      l38 = l4 / l6;
      l6 = (l4 * 0.0 - 1.0) / l6;
    }
  }
  l2 = l50 * l38;
  l4 = l50 * l6;
  if (l4 == 0.0) {
    l37 = l2 / 2.0;
    l8 = 0.0;
  } else if (l2 == 0.0) {
    l37 = 0.0;
    l8 = l4 / 2.0;
  } else {
    l37 = l2 / 2.0;
    l8 = l4 / 2.0;
  }
  l38 = 1.7320508075688772 * (l58.re + -l2);
  l6 = 1.7320508075688772 * (l58.im + -l4);
  l36 = l38 * 0.0 - l6 * 0.5;
  l38 = l38 * 0.5 + l6 * 0.0;
  t3[0].re = (l44 + l58.re) + l2;
  t3[0].im = l58.im + l4;
  l4 = (l44 + -l58_re) + -l37;
  t3[1].re = l4 - l36;
  l6 = -l58_im + -l8;
  t3[1].im = l6 - l38;
  t3[2].re = l4 + l36;
  t3[2].im = l6 + l38;
  l58.re = A_init - A_wayp;
  l4 = -(l58.re + J_min * t3[0].re);
  l6 = -(J_min * t3[0].im);
  if (l6 == 0.0) {
    t[0].re = l4 / J_max;
    t[0].im = 0.0;
  } else if (l4 == 0.0) {
    t[0].re = 0.0;
    t[0].im = l6 / J_max;
  } else {
    t[0].re = l4 / J_max;
    t[0].im = l6 / J_max;
  }
  t[3].re = 0.0;
  t[3].im = 0.0;
  t[6] = t3[0];
  t[9].re = 0.0;
  t[9].im = 0.0;
  t[12].re = 0.0;
  t[12].im = 0.0;
  t[15].re = 0.0;
  t[15].im = 0.0;
  t[18].re = 0.0;
  t[18].im = 0.0;
  l4 = -(l58.re + J_min * t3[1].re);
  l6 = -(J_min * t3[1].im);
  if (l6 == 0.0) {
    t[1].re = l4 / J_max;
    t[1].im = 0.0;
  } else if (l4 == 0.0) {
    t[1].re = 0.0;
    t[1].im = l6 / J_max;
  } else {
    t[1].re = l4 / J_max;
    t[1].im = l6 / J_max;
  }
  t[4].re = 0.0;
  t[4].im = 0.0;
  t[7] = t3[1];
  t[10].re = 0.0;
  t[10].im = 0.0;
  t[13].re = 0.0;
  t[13].im = 0.0;
  t[16].re = 0.0;
  t[16].im = 0.0;
  t[19].re = 0.0;
  t[19].im = 0.0;
  l4 = -(l58.re + J_min * t3[2].re);
  l6 = -(J_min * t3[2].im);
  if (l6 == 0.0) {
    t[2].re = l4 / J_max;
    t[2].im = 0.0;
  } else if (l4 == 0.0) {
    t[2].re = 0.0;
    t[2].im = l6 / J_max;
  } else {
    t[2].re = l4 / J_max;
    t[2].im = l6 / J_max;
  }
  t[5].re = 0.0;
  t[5].im = 0.0;
  t[8] = t3[2];
  t[11].re = 0.0;
  t[11].im = 0.0;
  t[14].re = 0.0;
  t[14].im = 0.0;
  t[17].re = 0.0;
  t[17].im = 0.0;
  t[20].re = 0.0;
  t[20].im = 0.0;
}

// End of code generation (ac_O_AP.cpp)
