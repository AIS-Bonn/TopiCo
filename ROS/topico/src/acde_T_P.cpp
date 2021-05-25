//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// acde_T_P.cpp
//
// Code generation for function 'acde_T_P'
//

// Include files
#include "acde_T_P.h"
#include "power.h"
#include "rt_nonfinite.h"
#include "sqrt.h"
#include "topico_wrapper_data.h"
#include "topico_wrapper_rtwutil.h"
#include <cmath>

// Function Definitions
void acde_T_P(double P_init, double V_init, double A_init, double P_wayp,
              double V_max, double J_max, double J_min, double T, creal_T t[21])
{
  creal_T b_l6[3];
  creal_T dcv[3];
  creal_T t3[3];
  creal_T t5[3];
  creal_T dc;
  creal_T l52;
  creal_T l53;
  double J_max_re_tmp;
  double J_max_re_tmp_tmp;
  double V_init_re_tmp;
  double V_init_tmp;
  double V_max_re_tmp;
  double b_J_max_re_tmp;
  double b_l3_re_tmp;
  double b_l52_tmp;
  double c_l3_re_tmp;
  double d_l3_re_tmp;
  double e_l3_re_tmp;
  double l10;
  double l2;
  double l3_re_tmp;
  double l3_tmp;
  double l4_tmp;
  double l52_tmp;
  double l5_tmp;
  double l6;
  double l7_tmp;
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
  //  Generated on 03-Sep-2019 15:48:59
  l2 = A_init * A_init;
  l3_tmp = rt_powd_snf(A_init, 3.0);
  l4_tmp = J_min * J_min;
  l5_tmp = rt_powd_snf(J_min, 3.0);
  l7_tmp = J_max * J_max;
  l9 = rt_powd_snf(J_max, 3.0);
  l6 = l4_tmp * l4_tmp;
  l10 = l7_tmp * l7_tmp;
  l52_tmp = J_max * V_init;
  b_l52_tmp = J_max * V_max;
  l52.re =
      J_min * (J_min + -J_max) * ((l2 + b_l52_tmp * 2.0) + -(l52_tmp * 2.0));
  l52.im = 0.0;
  coder::internal::scalar::b_sqrt(&l52);
  l53 = coder::d_power(l52);
  J_max_re_tmp = l52_tmp * l5_tmp;
  b_l52_tmp *= l5_tmp;
  l52_tmp = l2 * l5_tmp;
  J_max_re_tmp_tmp = J_max * l2;
  b_J_max_re_tmp = J_max_re_tmp_tmp * l4_tmp;
  V_init_re_tmp = V_init * l4_tmp * l7_tmp;
  V_max_re_tmp = V_max * l4_tmp * l7_tmp;
  l3_re_tmp = A_init * J_max;
  b_l3_re_tmp = T * V_max;
  c_l3_re_tmp = A_init * V_init;
  d_l3_re_tmp = A_init * V_max;
  e_l3_re_tmp = l3_re_tmp * V_init;
  V_init_tmp = -(1.0 / ((l5_tmp * l10 + l7_tmp * rt_powd_snf(J_min, 5.0)) +
                        -(l6 * l9 * 2.0)));
  dc.re = V_init_tmp *
          (((((((((((((((((((((((((l3_tmp * l6 * 2.0 - e_l3_re_tmp * l6 * 6.0) +
                                  l3_re_tmp * V_max * l6 * 6.0) +
                                 P_init * l4_tmp * l10 * 6.0) +
                                P_init * l6 * l7_tmp * 6.0) -
                               P_wayp * l4_tmp * l10 * 6.0) -
                              P_wayp * l6 * l7_tmp * 6.0) -
                             J_max * l3_tmp * l5_tmp * 4.0) -
                            P_init * l5_tmp * l9 * 12.0) +
                           P_wayp * l5_tmp * l9 * 12.0) -
                          c_l3_re_tmp * l4_tmp * l9 * 6.0) +
                         d_l3_re_tmp * l4_tmp * l9 * 6.0) +
                        b_l3_re_tmp * l4_tmp * l10 * 6.0) +
                       b_l3_re_tmp * l6 * l7_tmp * 6.0) +
                      c_l3_re_tmp * l5_tmp * l7_tmp * 12.0) -
                     d_l3_re_tmp * l5_tmp * l7_tmp * 12.0) +
                    l3_tmp * l4_tmp * l7_tmp * 2.0) -
                   b_l3_re_tmp * l5_tmp * l9 * 12.0) -
                  J_min * l53.re) -
                 J_max * l53.re) -
                J_max_re_tmp * l52.re * 6.0) +
               b_l52_tmp * l52.re * 6.0) +
              l52_tmp * l52.re * 3.0) -
             b_J_max_re_tmp * l52.re * 3.0) +
            V_init_re_tmp * l52.re * 6.0) -
           V_max_re_tmp * l52.re * 6.0);
  dc.im = V_init_tmp * ((((((((0.0 - J_min * l53.im) - J_max * l53.im) -
                             J_max_re_tmp * l52.im * 6.0) +
                            b_l52_tmp * l52.im * 6.0) +
                           l52_tmp * l52.im * 3.0) -
                          b_J_max_re_tmp * l52.im * 3.0) +
                         V_init_re_tmp * l52.im * 6.0) -
                        V_max_re_tmp * l52.im * 6.0);
  l52 = coder::power(dc);
  b_J_max_re_tmp = 1.7320508075688772 * l52.re;
  V_init_re_tmp = 1.7320508075688772 * l52.im;
  l53.re = b_J_max_re_tmp * 0.0 - V_init_re_tmp * 0.5;
  l53.im = b_J_max_re_tmp * 0.5 + V_init_re_tmp * 0.0;
  if (l52.im == 0.0) {
    b_l52_tmp = l52.re / 2.0;
    l52_tmp = 0.0;
  } else if (l52.re == 0.0) {
    b_l52_tmp = 0.0;
    l52_tmp = l52.im / 2.0;
  } else {
    b_l52_tmp = l52.re / 2.0;
    l52_tmp = l52.im / 2.0;
  }
  t5[0].re = -b_l52_tmp - l53.re;
  t5[0].im = -l52_tmp - l53.im;
  t5[1].re = -b_l52_tmp + l53.re;
  t5[1].im = -l52_tmp + l53.im;
  t5[2] = l52;
  l52.re = J_min * (J_min + -J_max) *
           ((A_init * A_init + J_max * V_max * 2.0) + -(J_max * V_init * 2.0));
  l52.im = 0.0;
  coder::internal::scalar::b_sqrt(&l52);
  b_l52_tmp = J_min * J_max;
  l52_tmp = 1.0 / (l4_tmp + -b_l52_tmp);
  l52.re *= l52_tmp;
  l52.im *= l52_tmp;
  t3[0] = l52;
  t3[1] = l52;
  t3[2] = l52;
  coder::c_power(t3, b_l6);
  l10 = J_max * l4_tmp;
  coder::c_power(t5, dcv);
  l52.re = P_init * l7_tmp * 6.0 - P_wayp * l7_tmp * 6.0;
  l53.re = l3_tmp * 2.0;
  l9 = e_l3_re_tmp * 6.0;
  l6 = J_min * l7_tmp;
  J_max_re_tmp = J_min * l2;
  V_init_tmp = V_init * l7_tmp;
  c_l3_re_tmp = b_l52_tmp * V_init;
  d_l3_re_tmp = J_max_re_tmp_tmp * 3.0 - V_init_tmp * 6.0;
  b_J_max_re_tmp = t3[0].re * t3[0].re - t3[0].im * t3[0].im;
  l52_tmp = t3[0].re * t3[0].im;
  V_init_re_tmp = l52_tmp + l52_tmp;
  b_l52_tmp = l6 * b_J_max_re_tmp;
  l52_tmp = l6 * V_init_re_tmp;
  b_J_max_re_tmp *= l10;
  V_init_re_tmp *= l10;
  l3_re_tmp = -(A_init + J_min * t3[0].re);
  b_l3_re_tmp = -(J_min * t3[0].im);
  if (b_l3_re_tmp == 0.0) {
    t[0].re = l3_re_tmp / J_max;
    t[0].im = 0.0;
  } else if (l3_re_tmp == 0.0) {
    t[0].re = 0.0;
    t[0].im = b_l3_re_tmp / J_max;
  } else {
    t[0].re = l3_re_tmp / J_max;
    t[0].im = b_l3_re_tmp / J_max;
  }
  t[3].re = 0.0;
  t[3].im = 0.0;
  t[6] = t3[0];
  l3_re_tmp = (((((((((((((l52.re - l5_tmp * b_l6[0].re) + l53.re) - l9) -
                        l6 * b_l6[0].re * 2.0) +
                       l10 * b_l6[0].re * 3.0) +
                      J_max_re_tmp * t3[0].re * 3.0) -
                     J_max_re_tmp_tmp * t3[0].re * 3.0) -
                    J_max_re_tmp_tmp * t5[0].re * 3.0) +
                   V_init_tmp * t3[0].re * 6.0) +
                  V_init_tmp * t5[0].re * 6.0) +
                 l6 * dcv[0].re) -
                (b_l52_tmp * t5[0].re - l52_tmp * t5[0].im) * 3.0) +
               (b_J_max_re_tmp * t5[0].re - V_init_re_tmp * t5[0].im) * 3.0) -
              c_l3_re_tmp * t3[0].re * 6.0;
  b_l3_re_tmp = (((((((((((0.0 - l5_tmp * b_l6[0].im) - l6 * b_l6[0].im * 2.0) +
                         l10 * b_l6[0].im * 3.0) +
                        J_max_re_tmp * t3[0].im * 3.0) -
                       J_max_re_tmp_tmp * t3[0].im * 3.0) -
                      J_max_re_tmp_tmp * t5[0].im * 3.0) +
                     V_init_tmp * t3[0].im * 6.0) +
                    V_init_tmp * t5[0].im * 6.0) +
                   l6 * dcv[0].im) -
                  (b_l52_tmp * t5[0].im + l52_tmp * t5[0].re) * 3.0) +
                 (b_J_max_re_tmp * t5[0].im + V_init_re_tmp * t5[0].re) * 3.0) -
                c_l3_re_tmp * t3[0].im * 6.0;
  V_max_re_tmp = (d_l3_re_tmp + b_l52_tmp * 3.0) - b_J_max_re_tmp * 3.0;
  b_J_max_re_tmp = l52_tmp * 3.0 - V_init_re_tmp * 3.0;
  if (b_J_max_re_tmp == 0.0) {
    if (b_l3_re_tmp == 0.0) {
      t[9].re = l3_re_tmp / V_max_re_tmp;
      t[9].im = 0.0;
    } else if (l3_re_tmp == 0.0) {
      t[9].re = 0.0;
      t[9].im = b_l3_re_tmp / V_max_re_tmp;
    } else {
      t[9].re = l3_re_tmp / V_max_re_tmp;
      t[9].im = b_l3_re_tmp / V_max_re_tmp;
    }
  } else if (V_max_re_tmp == 0.0) {
    if (l3_re_tmp == 0.0) {
      t[9].re = b_l3_re_tmp / b_J_max_re_tmp;
      t[9].im = 0.0;
    } else if (b_l3_re_tmp == 0.0) {
      t[9].re = 0.0;
      t[9].im = -(l3_re_tmp / b_J_max_re_tmp);
    } else {
      t[9].re = b_l3_re_tmp / b_J_max_re_tmp;
      t[9].im = -(l3_re_tmp / b_J_max_re_tmp);
    }
  } else {
    V_init_re_tmp = std::abs(V_max_re_tmp);
    b_l52_tmp = std::abs(b_J_max_re_tmp);
    if (V_init_re_tmp > b_l52_tmp) {
      l52_tmp = b_J_max_re_tmp / V_max_re_tmp;
      b_l52_tmp = V_max_re_tmp + l52_tmp * b_J_max_re_tmp;
      t[9].re = (l3_re_tmp + l52_tmp * b_l3_re_tmp) / b_l52_tmp;
      t[9].im = (b_l3_re_tmp - l52_tmp * l3_re_tmp) / b_l52_tmp;
    } else if (b_l52_tmp == V_init_re_tmp) {
      if (V_max_re_tmp > 0.0) {
        l52_tmp = 0.5;
      } else {
        l52_tmp = -0.5;
      }
      if (b_J_max_re_tmp > 0.0) {
        b_l52_tmp = 0.5;
      } else {
        b_l52_tmp = -0.5;
      }
      t[9].re = (l3_re_tmp * l52_tmp + b_l3_re_tmp * b_l52_tmp) / V_init_re_tmp;
      t[9].im = (b_l3_re_tmp * l52_tmp - l3_re_tmp * b_l52_tmp) / V_init_re_tmp;
    } else {
      l52_tmp = V_max_re_tmp / b_J_max_re_tmp;
      b_l52_tmp = b_J_max_re_tmp + l52_tmp * V_max_re_tmp;
      t[9].re = (l52_tmp * l3_re_tmp + b_l3_re_tmp) / b_l52_tmp;
      t[9].im = (l52_tmp * b_l3_re_tmp - l3_re_tmp) / b_l52_tmp;
    }
  }
  t[12] = t5[0];
  t[15].re = 0.0;
  t[15].im = 0.0;
  t[18].re = 0.0;
  t[18].im = 0.0;
  b_J_max_re_tmp = t3[1].re * t3[1].re - t3[1].im * t3[1].im;
  l52_tmp = t3[1].re * t3[1].im;
  V_init_re_tmp = l52_tmp + l52_tmp;
  b_l52_tmp = l6 * b_J_max_re_tmp;
  l52_tmp = l6 * V_init_re_tmp;
  b_J_max_re_tmp *= l10;
  V_init_re_tmp *= l10;
  l3_re_tmp = -(A_init + J_min * t3[1].re);
  b_l3_re_tmp = -(J_min * t3[1].im);
  if (b_l3_re_tmp == 0.0) {
    t[1].re = l3_re_tmp / J_max;
    t[1].im = 0.0;
  } else if (l3_re_tmp == 0.0) {
    t[1].re = 0.0;
    t[1].im = b_l3_re_tmp / J_max;
  } else {
    t[1].re = l3_re_tmp / J_max;
    t[1].im = b_l3_re_tmp / J_max;
  }
  t[4].re = 0.0;
  t[4].im = 0.0;
  t[7] = t3[1];
  l3_re_tmp = (((((((((((((l52.re - l5_tmp * b_l6[1].re) + l53.re) - l9) -
                        l6 * b_l6[1].re * 2.0) +
                       l10 * b_l6[1].re * 3.0) +
                      J_max_re_tmp * t3[1].re * 3.0) -
                     J_max_re_tmp_tmp * t3[1].re * 3.0) -
                    J_max_re_tmp_tmp * t5[1].re * 3.0) +
                   V_init_tmp * t3[1].re * 6.0) +
                  V_init_tmp * t5[1].re * 6.0) +
                 l6 * dcv[1].re) -
                (b_l52_tmp * t5[1].re - l52_tmp * t5[1].im) * 3.0) +
               (b_J_max_re_tmp * t5[1].re - V_init_re_tmp * t5[1].im) * 3.0) -
              c_l3_re_tmp * t3[1].re * 6.0;
  b_l3_re_tmp = (((((((((((0.0 - l5_tmp * b_l6[1].im) - l6 * b_l6[1].im * 2.0) +
                         l10 * b_l6[1].im * 3.0) +
                        J_max_re_tmp * t3[1].im * 3.0) -
                       J_max_re_tmp_tmp * t3[1].im * 3.0) -
                      J_max_re_tmp_tmp * t5[1].im * 3.0) +
                     V_init_tmp * t3[1].im * 6.0) +
                    V_init_tmp * t5[1].im * 6.0) +
                   l6 * dcv[1].im) -
                  (b_l52_tmp * t5[1].im + l52_tmp * t5[1].re) * 3.0) +
                 (b_J_max_re_tmp * t5[1].im + V_init_re_tmp * t5[1].re) * 3.0) -
                c_l3_re_tmp * t3[1].im * 6.0;
  V_max_re_tmp = (d_l3_re_tmp + b_l52_tmp * 3.0) - b_J_max_re_tmp * 3.0;
  b_J_max_re_tmp = l52_tmp * 3.0 - V_init_re_tmp * 3.0;
  if (b_J_max_re_tmp == 0.0) {
    if (b_l3_re_tmp == 0.0) {
      t[10].re = l3_re_tmp / V_max_re_tmp;
      t[10].im = 0.0;
    } else if (l3_re_tmp == 0.0) {
      t[10].re = 0.0;
      t[10].im = b_l3_re_tmp / V_max_re_tmp;
    } else {
      t[10].re = l3_re_tmp / V_max_re_tmp;
      t[10].im = b_l3_re_tmp / V_max_re_tmp;
    }
  } else if (V_max_re_tmp == 0.0) {
    if (l3_re_tmp == 0.0) {
      t[10].re = b_l3_re_tmp / b_J_max_re_tmp;
      t[10].im = 0.0;
    } else if (b_l3_re_tmp == 0.0) {
      t[10].re = 0.0;
      t[10].im = -(l3_re_tmp / b_J_max_re_tmp);
    } else {
      t[10].re = b_l3_re_tmp / b_J_max_re_tmp;
      t[10].im = -(l3_re_tmp / b_J_max_re_tmp);
    }
  } else {
    V_init_re_tmp = std::abs(V_max_re_tmp);
    b_l52_tmp = std::abs(b_J_max_re_tmp);
    if (V_init_re_tmp > b_l52_tmp) {
      l52_tmp = b_J_max_re_tmp / V_max_re_tmp;
      b_l52_tmp = V_max_re_tmp + l52_tmp * b_J_max_re_tmp;
      t[10].re = (l3_re_tmp + l52_tmp * b_l3_re_tmp) / b_l52_tmp;
      t[10].im = (b_l3_re_tmp - l52_tmp * l3_re_tmp) / b_l52_tmp;
    } else if (b_l52_tmp == V_init_re_tmp) {
      if (V_max_re_tmp > 0.0) {
        l52_tmp = 0.5;
      } else {
        l52_tmp = -0.5;
      }
      if (b_J_max_re_tmp > 0.0) {
        b_l52_tmp = 0.5;
      } else {
        b_l52_tmp = -0.5;
      }
      t[10].re =
          (l3_re_tmp * l52_tmp + b_l3_re_tmp * b_l52_tmp) / V_init_re_tmp;
      t[10].im =
          (b_l3_re_tmp * l52_tmp - l3_re_tmp * b_l52_tmp) / V_init_re_tmp;
    } else {
      l52_tmp = V_max_re_tmp / b_J_max_re_tmp;
      b_l52_tmp = b_J_max_re_tmp + l52_tmp * V_max_re_tmp;
      t[10].re = (l52_tmp * l3_re_tmp + b_l3_re_tmp) / b_l52_tmp;
      t[10].im = (l52_tmp * b_l3_re_tmp - l3_re_tmp) / b_l52_tmp;
    }
  }
  t[13] = t5[1];
  t[16].re = 0.0;
  t[16].im = 0.0;
  t[19].re = 0.0;
  t[19].im = 0.0;
  b_J_max_re_tmp = t3[2].re * t3[2].re - t3[2].im * t3[2].im;
  l52_tmp = t3[2].re * t3[2].im;
  V_init_re_tmp = l52_tmp + l52_tmp;
  b_l52_tmp = l6 * b_J_max_re_tmp;
  l52_tmp = l6 * V_init_re_tmp;
  b_J_max_re_tmp *= l10;
  V_init_re_tmp *= l10;
  l3_re_tmp = -(A_init + J_min * t3[2].re);
  b_l3_re_tmp = -(J_min * t3[2].im);
  if (b_l3_re_tmp == 0.0) {
    t[2].re = l3_re_tmp / J_max;
    t[2].im = 0.0;
  } else if (l3_re_tmp == 0.0) {
    t[2].re = 0.0;
    t[2].im = b_l3_re_tmp / J_max;
  } else {
    t[2].re = l3_re_tmp / J_max;
    t[2].im = b_l3_re_tmp / J_max;
  }
  t[5].re = 0.0;
  t[5].im = 0.0;
  t[8] = t3[2];
  l3_re_tmp = (((((((((((((l52.re - l5_tmp * b_l6[2].re) + l53.re) - l9) -
                        l6 * b_l6[2].re * 2.0) +
                       l10 * b_l6[2].re * 3.0) +
                      J_max_re_tmp * t3[2].re * 3.0) -
                     J_max_re_tmp_tmp * t3[2].re * 3.0) -
                    J_max_re_tmp_tmp * t5[2].re * 3.0) +
                   V_init_tmp * t3[2].re * 6.0) +
                  V_init_tmp * t5[2].re * 6.0) +
                 l6 * dcv[2].re) -
                (b_l52_tmp * t5[2].re - l52_tmp * t5[2].im) * 3.0) +
               (b_J_max_re_tmp * t5[2].re - V_init_re_tmp * t5[2].im) * 3.0) -
              c_l3_re_tmp * t3[2].re * 6.0;
  b_l3_re_tmp = (((((((((((0.0 - l5_tmp * b_l6[2].im) - l6 * b_l6[2].im * 2.0) +
                         l10 * b_l6[2].im * 3.0) +
                        J_max_re_tmp * t3[2].im * 3.0) -
                       J_max_re_tmp_tmp * t3[2].im * 3.0) -
                      J_max_re_tmp_tmp * t5[2].im * 3.0) +
                     V_init_tmp * t3[2].im * 6.0) +
                    V_init_tmp * t5[2].im * 6.0) +
                   l6 * dcv[2].im) -
                  (b_l52_tmp * t5[2].im + l52_tmp * t5[2].re) * 3.0) +
                 (b_J_max_re_tmp * t5[2].im + V_init_re_tmp * t5[2].re) * 3.0) -
                c_l3_re_tmp * t3[2].im * 6.0;
  V_max_re_tmp = (d_l3_re_tmp + b_l52_tmp * 3.0) - b_J_max_re_tmp * 3.0;
  b_J_max_re_tmp = l52_tmp * 3.0 - V_init_re_tmp * 3.0;
  if (b_J_max_re_tmp == 0.0) {
    if (b_l3_re_tmp == 0.0) {
      t[11].re = l3_re_tmp / V_max_re_tmp;
      t[11].im = 0.0;
    } else if (l3_re_tmp == 0.0) {
      t[11].re = 0.0;
      t[11].im = b_l3_re_tmp / V_max_re_tmp;
    } else {
      t[11].re = l3_re_tmp / V_max_re_tmp;
      t[11].im = b_l3_re_tmp / V_max_re_tmp;
    }
  } else if (V_max_re_tmp == 0.0) {
    if (l3_re_tmp == 0.0) {
      t[11].re = b_l3_re_tmp / b_J_max_re_tmp;
      t[11].im = 0.0;
    } else if (b_l3_re_tmp == 0.0) {
      t[11].re = 0.0;
      t[11].im = -(l3_re_tmp / b_J_max_re_tmp);
    } else {
      t[11].re = b_l3_re_tmp / b_J_max_re_tmp;
      t[11].im = -(l3_re_tmp / b_J_max_re_tmp);
    }
  } else {
    V_init_re_tmp = std::abs(V_max_re_tmp);
    b_l52_tmp = std::abs(b_J_max_re_tmp);
    if (V_init_re_tmp > b_l52_tmp) {
      l52_tmp = b_J_max_re_tmp / V_max_re_tmp;
      b_l52_tmp = V_max_re_tmp + l52_tmp * b_J_max_re_tmp;
      t[11].re = (l3_re_tmp + l52_tmp * b_l3_re_tmp) / b_l52_tmp;
      t[11].im = (b_l3_re_tmp - l52_tmp * l3_re_tmp) / b_l52_tmp;
    } else if (b_l52_tmp == V_init_re_tmp) {
      if (V_max_re_tmp > 0.0) {
        l52_tmp = 0.5;
      } else {
        l52_tmp = -0.5;
      }
      if (b_J_max_re_tmp > 0.0) {
        b_l52_tmp = 0.5;
      } else {
        b_l52_tmp = -0.5;
      }
      t[11].re =
          (l3_re_tmp * l52_tmp + b_l3_re_tmp * b_l52_tmp) / V_init_re_tmp;
      t[11].im =
          (b_l3_re_tmp * l52_tmp - l3_re_tmp * b_l52_tmp) / V_init_re_tmp;
    } else {
      l52_tmp = V_max_re_tmp / b_J_max_re_tmp;
      b_l52_tmp = b_J_max_re_tmp + l52_tmp * V_max_re_tmp;
      t[11].re = (l52_tmp * l3_re_tmp + b_l3_re_tmp) / b_l52_tmp;
      t[11].im = (l52_tmp * b_l3_re_tmp - l3_re_tmp) / b_l52_tmp;
    }
  }
  t[14] = t5[2];
  t[17].re = 0.0;
  t[17].im = 0.0;
  t[20].re = 0.0;
  t[20].im = 0.0;
}

// End of code generation (acde_T_P.cpp)
