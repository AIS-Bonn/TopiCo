//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// abceg_T_AP.cpp
//
// Code generation for function 'abceg_T_AP'
//

// Include files
#include "abceg_T_AP.h"
#include "power.h"
#include "rt_nonfinite.h"
#include "sqrt.h"
#include "topico_wrapper_data.h"
#include "topico_wrapper_rtwutil.h"
#include <cmath>

// Function Definitions
void abceg_T_AP(double P_init, double V_init, double A_init, double P_wayp,
                double A_wayp, double A_max, double J_max, double J_min,
                double T, creal_T t[21])
{
  creal_T t2[3];
  creal_T b_l141;
  creal_T l146;
  double b_l125_tmp;
  double b_l131_tmp;
  double b_l141_tmp;
  double c_l125_tmp;
  double c_l131_tmp;
  double d;
  double d1;
  double d_l125_tmp;
  double d_l131_tmp;
  double l10;
  double l11;
  double l12;
  double l122;
  double l122_tmp;
  double l123;
  double l125;
  double l125_tmp;
  double l128;
  double l13;
  double l131;
  double l131_tmp;
  double l136;
  double l141;
  double l141_tmp;
  double l150_re;
  double l155_im;
  double l155_re;
  double l18;
  double l18_tmp;
  double l2;
  double l3;
  double l37;
  double l39;
  double l4;
  double l5;
  double l7;
  double l75;
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
  //  Generated on 28-Aug-2019 17:25:45
  l2 = A_init * A_init;
  l3 = rt_powd_snf(A_init, 3.0);
  l4 = A_max * A_max;
  l5 = A_wayp * A_wayp;
  l7 = J_min * J_min;
  l8 = J_max * J_max;
  l9 = rt_powd_snf(J_max, 3.0);
  l11 = T * T;
  l12 = rt_powd_snf(T, 3.0);
  l10 = l8 * l8;
  l18_tmp = A_init * J_min;
  l18 = l18_tmp * l9 * 3.0;
  l37 = A_init * A_wayp * J_max * l7 * 6.0;
  l141 = A_max * A_wayp;
  l39 = l141 * J_min * l8 * 6.0;
  l75 = A_wayp * l7 * l8 * 3.0;
  l13 = J_min * l10;
  l122_tmp = l7 * l9;
  l122 = 1.0 / (l13 + -(l122_tmp * 2.0));
  l125_tmp = A_max * J_min;
  b_l125_tmp = l125_tmp * l9;
  c_l125_tmp = A_max * l7 * l8;
  d_l125_tmp = A_init * l7 * l8;
  l125 = (((((l18 + T * l13 * 3.0) + -(b_l125_tmp * 3.0)) + c_l125_tmp * 3.0) +
           l75) +
          -(T * l7 * l9 * 6.0)) +
         -(d_l125_tmp * 6.0);
  l131_tmp = A_init * A_max;
  b_l131_tmp = l131_tmp * J_max;
  c_l131_tmp = l141 * J_max;
  d_l131_tmp = l131_tmp * J_min;
  l131 = (((((((((((((b_l131_tmp * l7 * 6.0 + l37) + l39) +
                    l18_tmp * T * l9 * 6.0) +
                   l11 * l13 * 3.0) +
                  -(d_l131_tmp * l8 * 6.0)) +
                 -(c_l131_tmp * l7 * 6.0)) +
                -(l125_tmp * T * l9 * 6.0)) +
               J_min * l2 * l8 * 3.0) +
              A_max * T * l7 * l8 * 6.0) +
             A_wayp * T * l7 * l8 * 6.0) +
            -(J_max * l2 * l7 * 6.0)) +
           -(J_min * l5 * l8 * 3.0)) +
          -(A_init * T * l7 * l8 * 12.0)) +
         -(l122_tmp * l11 * 6.0);
  l123 = l122 * l122;
  l128 = l122 * l125 / 3.0;
  l136 = l123 * (l125 * l125) / 9.0 + -(l122 * l131 / 3.0);
  d = T * V_init;
  d1 = l131_tmp * A_wayp;
  l141 = l18_tmp * J_max;
  l155_im = A_max * l2;
  l150_re = A_wayp * l4;
  l141_tmp = A_init * l4;
  l155_re = J_min * T;
  b_l141_tmp = J_max * T;
  l141 =
      (rt_powd_snf(l122, 3.0) * rt_powd_snf(l125, 3.0) / 27.0 +
       -(l123 * l125 * l131 / 6.0)) +
      l122 *
          ((((((((((((((((((((((((((((((((((((((((((((((((P_init * l10 * 6.0 +
                                                          -(P_wayp * l10 *
                                                            6.0)) +
                                                         J_min * J_max * l3 *
                                                             3.0) +
                                                        rt_powd_snf(A_wayp,
                                                                    3.0) *
                                                            l8) +
                                                       d * l10 * 6.0) +
                                                      d1 * J_min * J_max *
                                                          6.0) +
                                                     l12 * l13) +
                                                    J_min * P_wayp * l9 *
                                                        12.0) +
                                                   l141 * l4 * 6.0) +
                                                  l125_tmp * J_max * l5 * 3.0) +
                                                 l131_tmp * T * l9 * 6.0) +
                                                P_init * l7 * l8 * 6.0) +
                                               T * l37) +
                                              T * l39) +
                                             -(J_min * P_init * l9 * 12.0)) +
                                            -(l3 * l7 * 3.0)) +
                                           -(l3 * l8)) +
                                          -(d1 * l7 * 6.0)) +
                                         -(l141 * l5 * 3.0)) +
                                        -(A_wayp * J_min * J_max * l4 * 6.0)) +
                                       l155_im * l8 * 3.0) +
                                      l155_im * l7 * 6.0) +
                                     A_wayp * l2 * l7 * 3.0) +
                                    l150_re * l7 * 3.0) +
                                   l150_re * l8 * 3.0) +
                                  A_max * l10 * l11 * 3.0) +
                                 -(P_wayp * l7 * l8 * 6.0)) +
                                b_l131_tmp * T * l7 * 12.0) +
                               c_l131_tmp * T * l7 * -6.0) +
                              l11 * l18) +
                             l155_re * l4 * l8 * 6.0) +
                            d * l7 * l8 * 6.0) +
                           -(A_max * J_min * J_max * l2 * 9.0)) +
                          -(l141_tmp * l7 * 3.0)) +
                         -(l141_tmp * l8 * 3.0)) +
                        -(A_max * l5 * l8 * 3.0)) +
                       -(l155_re * V_init * l9 * 12.0)) +
                      -(T * l2 * l9 * 3.0)) +
                     -(T * l4 * l9 * 3.0)) +
                    -(d_l131_tmp * T * l8 * 18.0)) +
                   l155_re * l2 * l8 * 9.0) +
                  -(b_l141_tmp * l4 * l7 * 3.0)) +
                 l155_re * l5 * l8 * -3.0) +
                -(b_l141_tmp * l2 * l7 * 9.0)) +
               -(b_l125_tmp * l11 * 9.0)) +
              l122_tmp * l12 * -2.0) +
             c_l125_tmp * l11 * 6.0) +
            l11 * l75) +
           d_l125_tmp * l11 * -6.0) /
          2.0;
  l146.re = -rt_powd_snf(l136, 3.0) + l141 * l141;
  l146.im = 0.0;
  coder::internal::scalar::b_sqrt(&l146);
  b_l141.re = l141 + l146.re;
  b_l141.im = l146.im;
  l146 = coder::power(b_l141);
  if (l146.im == 0.0) {
    d_l131_tmp = l146.re / 2.0;
    l131 = 0.0;
    l123 = 1.0 / l146.re;
    c_l131_tmp = 0.0;
  } else if (l146.re == 0.0) {
    d_l131_tmp = 0.0;
    l131 = l146.im / 2.0;
    l123 = 0.0;
    c_l131_tmp = -(1.0 / l146.im);
  } else {
    d_l131_tmp = l146.re / 2.0;
    l131 = l146.im / 2.0;
    l150_re = std::abs(l146.re);
    l141 = std::abs(l146.im);
    if (l150_re > l141) {
      l141 = l146.im / l146.re;
      l150_re = l146.re + l141 * l146.im;
      l123 = (l141 * 0.0 + 1.0) / l150_re;
      c_l131_tmp = (0.0 - l141) / l150_re;
    } else if (l141 == l150_re) {
      if (l146.re > 0.0) {
        l141 = 0.5;
      } else {
        l141 = -0.5;
      }
      if (l146.im > 0.0) {
        l155_im = 0.5;
      } else {
        l155_im = -0.5;
      }
      l123 = (l141 + 0.0 * l155_im) / l150_re;
      c_l131_tmp = (0.0 * l141 - l155_im) / l150_re;
    } else {
      l141 = l146.re / l146.im;
      l150_re = l146.im + l141 * l146.re;
      l123 = l141 / l150_re;
      c_l131_tmp = (l141 * 0.0 - 1.0) / l150_re;
    }
  }
  l150_re = l136 * l123;
  l141 = l136 * c_l131_tmp;
  if (l141 == 0.0) {
    l18_tmp = l150_re / 2.0;
    l141_tmp = 0.0;
  } else if (l150_re == 0.0) {
    l18_tmp = 0.0;
    l141_tmp = l141 / 2.0;
  } else {
    l18_tmp = l150_re / 2.0;
    l141_tmp = l141 / 2.0;
  }
  l123 = 1.7320508075688772 * (l146.re + -l150_re);
  c_l131_tmp = 1.7320508075688772 * (l146.im + -l141);
  l155_re = l123 * 0.0 - c_l131_tmp * 0.5;
  l155_im = l123 * 0.5 + c_l131_tmp * 0.0;
  t2[0].re = (l128 + l146.re) + l150_re;
  t2[0].im = l146.im + l141;
  d = (l128 + -d_l131_tmp) + -l18_tmp;
  t2[1].re = d - l155_re;
  d1 = -l131 + -l141_tmp;
  t2[1].im = d1 - l155_im;
  t2[2].re = d + l155_re;
  t2[2].im = d1 + l155_im;
  l18_tmp = J_min + -J_max;
  l141 = -(1.0 / J_max * (A_init + -A_max));
  l146.re = A_max - A_wayp;
  t[0].re = l141;
  t[0].im = 0.0;
  t[1].re = l141;
  t[1].im = 0.0;
  t[2].re = l141;
  t[2].im = 0.0;
  l155_im = A_init - A_wayp;
  l141 = -((l155_im + -J_max * t2[0].re) + b_l141_tmp);
  l150_re = -(-J_max * t2[0].im);
  if (l150_re == 0.0) {
    l123 = l141 / l18_tmp;
    c_l131_tmp = 0.0;
  } else if (l141 == 0.0) {
    l123 = 0.0;
    c_l131_tmp = l150_re / l18_tmp;
  } else {
    l123 = l141 / l18_tmp;
    c_l131_tmp = l150_re / l18_tmp;
  }
  t[3] = t2[0];
  t[6].re = l123;
  t[6].im = c_l131_tmp;
  t[9].re = 0.0;
  t[9].im = 0.0;
  t[12].re = 0.0;
  t[12].im = 0.0;
  t[15].re = 0.0;
  t[15].im = 0.0;
  l141 = -(l146.re + J_min * l123);
  l150_re = -(J_min * c_l131_tmp);
  if (l150_re == 0.0) {
    t[18].re = l141 / J_max;
    t[18].im = 0.0;
  } else if (l141 == 0.0) {
    t[18].re = 0.0;
    t[18].im = l150_re / J_max;
  } else {
    t[18].re = l141 / J_max;
    t[18].im = l150_re / J_max;
  }
  l141 = -((l155_im + -J_max * t2[1].re) + b_l141_tmp);
  l150_re = -(-J_max * t2[1].im);
  if (l150_re == 0.0) {
    l123 = l141 / l18_tmp;
    c_l131_tmp = 0.0;
  } else if (l141 == 0.0) {
    l123 = 0.0;
    c_l131_tmp = l150_re / l18_tmp;
  } else {
    l123 = l141 / l18_tmp;
    c_l131_tmp = l150_re / l18_tmp;
  }
  t[4] = t2[1];
  t[7].re = l123;
  t[7].im = c_l131_tmp;
  t[10].re = 0.0;
  t[10].im = 0.0;
  t[13].re = 0.0;
  t[13].im = 0.0;
  t[16].re = 0.0;
  t[16].im = 0.0;
  l141 = -(l146.re + J_min * l123);
  l150_re = -(J_min * c_l131_tmp);
  if (l150_re == 0.0) {
    t[19].re = l141 / J_max;
    t[19].im = 0.0;
  } else if (l141 == 0.0) {
    t[19].re = 0.0;
    t[19].im = l150_re / J_max;
  } else {
    t[19].re = l141 / J_max;
    t[19].im = l150_re / J_max;
  }
  l141 = -((l155_im + -J_max * t2[2].re) + b_l141_tmp);
  l150_re = -(-J_max * t2[2].im);
  if (l150_re == 0.0) {
    l123 = l141 / l18_tmp;
    c_l131_tmp = 0.0;
  } else if (l141 == 0.0) {
    l123 = 0.0;
    c_l131_tmp = l150_re / l18_tmp;
  } else {
    l123 = l141 / l18_tmp;
    c_l131_tmp = l150_re / l18_tmp;
  }
  t[5] = t2[2];
  t[8].re = l123;
  t[8].im = c_l131_tmp;
  t[11].re = 0.0;
  t[11].im = 0.0;
  t[14].re = 0.0;
  t[14].im = 0.0;
  t[17].re = 0.0;
  t[17].im = 0.0;
  l141 = -(l146.re + J_min * l123);
  l150_re = -(J_min * c_l131_tmp);
  if (l150_re == 0.0) {
    t[20].re = l141 / J_max;
    t[20].im = 0.0;
  } else if (l141 == 0.0) {
    t[20].re = 0.0;
    t[20].im = l150_re / J_max;
  } else {
    t[20].re = l141 / J_max;
    t[20].im = l150_re / J_max;
  }
}

// End of code generation (abceg_T_AP.cpp)
