//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// acef_T_P.cpp
//
// Code generation for function 'acef_T_P'
//

// Include files
#include "acef_T_P.h"
#include "power.h"
#include "rt_nonfinite.h"
#include "sqrt.h"
#include "topico_wrapper_data.h"
#include "topico_wrapper_rtwutil.h"
#include <cmath>

// Function Definitions
void acef_T_P(double P_init, double V_init, double A_init, double P_wayp,
              double A_min, double J_max, double J_min, double T, creal_T t[21])
{
  creal_T t6[3];
  creal_T l66;
  creal_T l82;
  double b_l62_tmp;
  double b_l75_tmp;
  double l11;
  double l19;
  double l2;
  double l4;
  double l56;
  double l56_tmp;
  double l57;
  double l59;
  double l59_tmp;
  double l6;
  double l62;
  double l62_tmp;
  double l63;
  double l7;
  double l75_tmp;
  double l8;
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
  //  Generated on 29-Aug-2019 13:28:28
  l2 = A_init * A_init;
  l4 = A_min * A_min;
  l6 = J_min * J_min;
  l7 = J_max * J_max;
  l8 = T * T;
  l9 = rt_powd_snf(T, 3.0);
  l11 = J_min * l7;
  l19 = A_init * l6 * 3.0;
  l59_tmp = J_max * T;
  l59 = (l19 + -(A_min * l6 * 3.0)) + l59_tmp * l6 * 3.0;
  l62_tmp = A_init * A_min;
  b_l62_tmp = A_init * J_min * J_max;
  l62 = ((((-(l62_tmp * J_min * 6.0) + J_min * l2 * 3.0) + J_min * l4 * 3.0) +
          b_l62_tmp * T * 6.0) +
         -(A_min * J_min * J_max * T * 6.0)) +
        l8 * l11 * 3.0;
  l56_tmp = J_max * l6;
  l56 = 1.0 / (l11 + -(l56_tmp * 2.0));
  l57 = l56 * l56;
  l63 = l56 * l59 / 3.0;
  l75_tmp = J_min * J_max;
  b_l75_tmp = T * V_init;
  l4 =
      l56 *
      ((((((((((((((((((((rt_powd_snf(A_min, 3.0) + -rt_powd_snf(A_init, 3.0)) +
                         A_min * l2 * 3.0) +
                        l75_tmp * P_wayp * 12.0) +
                       P_init * l6 * 6.0) +
                      P_init * l7 * 6.0) +
                     l62_tmp * J_max * T * 6.0) +
                    -(A_init * l4 * 3.0)) +
                   -(l75_tmp * P_init * 12.0)) +
                  -(P_wayp * l6 * 6.0)) +
                 -(P_wayp * l7 * 6.0)) +
                b_l75_tmp * l6 * 6.0) +
               b_l75_tmp * l7 * 6.0) +
              l56_tmp * l9) +
             -(l75_tmp * T * V_init * 12.0)) +
            -(l59_tmp * l2 * 3.0)) +
           -(l59_tmp * l4 * 3.0)) +
          -(b_l62_tmp * l8 * 6.0)) +
         l8 * l19) +
        A_min * l7 * l8 * 3.0) +
       -(l9 * l11 * 2.0)) /
      2.0;
  l6 = rt_powd_snf(l56, 3.0) * rt_powd_snf(l59, 3.0) / 27.0;
  l7 = l57 * l59 * l62 / 6.0;
  l11 = l57 * (l59 * l59) / 9.0 + l56 * l62 / 3.0;
  l2 = (l6 + l7) + -l4;
  l82.re = -rt_powd_snf(l11, 3.0) + l2 * l2;
  l82.im = 0.0;
  coder::internal::scalar::b_sqrt(&l82);
  l66.re = ((-l6 + -l7) + l4) + l82.re;
  l66.im = l82.im;
  l82 = coder::power(l66);
  if (l82.im == 0.0) {
    l19 = l82.re / 2.0;
    l62_tmp = 0.0;
    l9 = 1.0 / l82.re;
    l8 = 0.0;
  } else if (l82.re == 0.0) {
    l19 = 0.0;
    l62_tmp = l82.im / 2.0;
    l9 = 0.0;
    l8 = -(1.0 / l82.im);
  } else {
    l19 = l82.re / 2.0;
    l62_tmp = l82.im / 2.0;
    l6 = std::abs(l82.re);
    l2 = std::abs(l82.im);
    if (l6 > l2) {
      l2 = l82.im / l82.re;
      l7 = l82.re + l2 * l82.im;
      l9 = (l2 * 0.0 + 1.0) / l7;
      l8 = (0.0 - l2) / l7;
    } else if (l2 == l6) {
      if (l82.re > 0.0) {
        l2 = 0.5;
      } else {
        l2 = -0.5;
      }
      if (l82.im > 0.0) {
        l4 = 0.5;
      } else {
        l4 = -0.5;
      }
      l9 = (l2 + 0.0 * l4) / l6;
      l8 = (0.0 * l2 - l4) / l6;
    } else {
      l2 = l82.re / l82.im;
      l7 = l82.im + l2 * l82.re;
      l9 = l2 / l7;
      l8 = (l2 * 0.0 - 1.0) / l7;
    }
  }
  l7 = l11 * l9;
  l2 = l11 * l8;
  if (l2 == 0.0) {
    l4 = l7 / 2.0;
    l8 = 0.0;
  } else if (l7 == 0.0) {
    l4 = 0.0;
    l8 = l2 / 2.0;
  } else {
    l4 = l7 / 2.0;
    l8 = l2 / 2.0;
  }
  t6[0].re = (-l63 + l82.re) + l7;
  t6[0].im = l82.im + l2;
  l9 = 1.7320508075688772 * (l82.re - l7);
  l6 = 1.7320508075688772 * (l82.im - l2);
  l7 = (-l63 + -l19) + -l4;
  t6[1].re = l7 + (l9 * 0.0 - l6 * 0.5);
  l2 = -l62_tmp + -l8;
  t6[1].im = l2 + (l9 * 0.5 + l6 * 0.0);
  t6[2].re = l7 + (l9 * -0.0 - l6 * -0.5);
  t6[2].im = l2 + (l9 * -0.5 + l6 * -0.0);
  l2 = J_min + -J_max;
  l7 = A_init - A_min;
  l4 = -((l7 + -J_max * t6[0].re) + l59_tmp);
  l6 = -(-J_max * t6[0].im);
  if (l6 == 0.0) {
    l9 = l4 / l2;
    l8 = 0.0;
  } else if (l4 == 0.0) {
    l9 = 0.0;
    l8 = l6 / l2;
  } else {
    l9 = l4 / l2;
    l8 = l6 / l2;
  }
  l4 = -(l7 + J_min * l9);
  l6 = -(J_min * l8);
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
  t[6].re = l9;
  t[6].im = l8;
  t[9].re = 0.0;
  t[9].im = 0.0;
  t[12].re = 0.0;
  t[12].im = 0.0;
  t[15] = t6[0];
  t[18].re = 0.0;
  t[18].im = 0.0;
  l4 = -((l7 + -J_max * t6[1].re) + l59_tmp);
  l6 = -(-J_max * t6[1].im);
  if (l6 == 0.0) {
    l9 = l4 / l2;
    l8 = 0.0;
  } else if (l4 == 0.0) {
    l9 = 0.0;
    l8 = l6 / l2;
  } else {
    l9 = l4 / l2;
    l8 = l6 / l2;
  }
  l4 = -(l7 + J_min * l9);
  l6 = -(J_min * l8);
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
  t[7].re = l9;
  t[7].im = l8;
  t[10].re = 0.0;
  t[10].im = 0.0;
  t[13].re = 0.0;
  t[13].im = 0.0;
  t[16] = t6[1];
  t[19].re = 0.0;
  t[19].im = 0.0;
  l4 = -((l7 + -J_max * t6[2].re) + l59_tmp);
  l6 = -(-J_max * t6[2].im);
  if (l6 == 0.0) {
    l9 = l4 / l2;
    l8 = 0.0;
  } else if (l4 == 0.0) {
    l9 = 0.0;
    l8 = l6 / l2;
  } else {
    l9 = l4 / l2;
    l8 = l6 / l2;
  }
  l4 = -(l7 + J_min * l9);
  l6 = -(J_min * l8);
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
  t[8].re = l9;
  t[8].im = l8;
  t[11].re = 0.0;
  t[11].im = 0.0;
  t[14].re = 0.0;
  t[14].im = 0.0;
  t[17] = t6[2];
  t[20].re = 0.0;
  t[20].im = 0.0;
}

// End of code generation (acef_T_P.cpp)
