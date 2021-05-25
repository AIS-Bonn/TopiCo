//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// abcdeg_NO_AP.cpp
//
// Code generation for function 'abcdeg_NO_AP'
//

// Include files
#include "abcdeg_NO_AP.h"
#include "rt_nonfinite.h"
#include "topico_wrapper_rtwutil.h"

// Function Definitions
void abcdeg_NO_AP(double P_init, double V_init, double A_init, double P_wayp,
                  double A_wayp, double V_max, double A_min, double J_max,
                  double J_min, double t[14])
{
  double A_min_tmp;
  double A_min_tmp_tmp;
  double A_wayp_tmp;
  double J_min_tmp;
  double V_init_tmp;
  double b_A_init;
  double b_A_min;
  double b_A_min_tmp;
  double b_A_wayp;
  double b_A_wayp_tmp;
  double b_J_min;
  double b_V_init;
  double b_V_max;
  double b_l3;
  double b_l5;
  double b_l6;
  double b_y;
  double c_A_min;
  double c_A_min_tmp;
  double c_A_wayp;
  double c_J_min;
  double c_V_init;
  double d;
  double d1;
  double d2;
  double d3;
  double d4;
  double d5;
  double d6;
  double d7;
  double d8;
  double d_A_min;
  double d_A_min_tmp;
  double d_A_wayp;
  double d_V_init;
  double e_A_min;
  double f_A_min;
  double g_A_min;
  double h_A_min;
  double i_A_min;
  double j_A_min;
  double k_A_min;
  double l15_tmp;
  double l17;
  double l18;
  double l19;
  double l26_idx_0;
  double l2_idx_0;
  double l3;
  double l4;
  double l5;
  double l6;
  double l7_tmp;
  double l8;
  double l9;
  double t7_idx_1;
  double y;
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
  //  Generated on 04-Sep-2019 17:17:26
  t7_idx_1 = A_wayp * 2.0 / J_max;
  l3 = A_init * A_init;
  l4 = A_min * A_min;
  l5 = rt_powd_snf(A_min, 3.0);
  l7_tmp = A_wayp * A_wayp;
  l8 = J_min * J_min;
  l9 = J_max * J_max;
  l15_tmp = J_min * J_max;
  l6 = l4 * l4;
  l17 = J_max * l3;
  l18 = J_min * l4;
  l19 = J_max * l4;
  b_l6 = (l6 * l8 * 8.0 - l6 * l9 * 2.0) + l17 * l18 * 12.0;
  b_J_min = l15_tmp * l6 * 6.0;
  b_A_wayp = A_wayp * l5 * l9 * 12.0;
  V_init_tmp = V_init * l8;
  b_V_init = V_init_tmp * l19 * 12.0;
  c_V_init = V_init * l9;
  b_V_max = V_max * l9 * l18 * 12.0;
  y = rt_powd_snf(A_init, 3.0) * A_min * l9 * 8.0;
  b_l3 = l3 * l4 * l9 * 6.0;
  b_l5 = l5 * l9;
  A_min_tmp = A_min * l9;
  b_y = V_init * V_init * l8 * l9 * 24.0;
  A_min_tmp_tmp = A_min * A_wayp;
  b_A_min_tmp = A_min_tmp_tmp * J_min;
  c_A_min_tmp = A_min * J_min;
  b_A_min = A_min * P_init * l8 * l9 * 24.0;
  c_A_min = A_min * P_wayp * l8 * l9 * 24.0;
  J_min_tmp = J_min * V_init;
  c_J_min = J_min_tmp * l3 * l9 * 12.0;
  d_V_init = V_init * V_max * l8 * l9 * 24.0;
  d_A_min_tmp = c_A_min_tmp * V_init;
  d_A_min = d_A_min_tmp * l9;
  e_A_min = c_A_min_tmp * V_max;
  f_A_min = A_min * J_max * V_max;
  A_wayp_tmp = A_wayp * J_min;
  c_A_wayp = A_wayp_tmp * V_init;
  b_A_wayp_tmp = A_wayp * J_max;
  d_A_wayp = b_A_wayp_tmp * V_init;
  b_A_init = A_init * A_min * J_min * V_init * l9 * 24.0;
  g_A_min = b_A_min_tmp * V_init * l9 * 24.0;
  h_A_min = b_A_min_tmp * V_max * l9 * 24.0;
  i_A_min = A_min * l8;
  j_A_min = A_min * V_max * l8 * l9 * 24.0;
  k_A_min = A_min_tmp_tmp * J_max;
  d = J_max * 0.0;
  l2_idx_0 = d;
  d1 = b_A_wayp_tmp * d;
  d2 = d * d;
  d3 = J_max * d2;
  d4 = J_min * d2;
  d5 = A_wayp + -d;
  d6 = (((l15_tmp * V_max * 2.0 + l17) + l18) + -(l15_tmp * V_init * 2.0)) +
       -l19;
  l6 = (((d6 + -(A_wayp_tmp * d * 2.0)) + d1 * 2.0) + d4) + J_max * d * -d;
  l3 = A_min * rt_powd_snf(d, 3.0);
  l15_tmp = A_min_tmp_tmp * d2;
  l19 = c_A_min_tmp * J_max;
  d7 = l19 * d;
  d8 = d5 * d5;
  d3 = -((((((((((((((((((((((((((((((((((((((((((b_l6 + l18 * d3 * 6.0) -
                                                 l17 * l6 * 6.0) -
                                                l18 * l6 * 12.0) +
                                               l6 * l6 * 3.0) -
                                              d2 * l4 * l9 * 6.0) -
                                             b_J_min) -
                                            b_A_wayp) -
                                           b_V_init) +
                                          V_init_tmp * d3 * 12.0) -
                                         c_V_init * d4 * 12.0) +
                                        b_V_max) +
                                       y) +
                                      d * l5 * l9 * 12.0) -
                                     b_l3) +
                                    b_l5 * d5 * 12.0) +
                                   l3 * l8 * 16.0) +
                                  l3 * l9 * 12.0) +
                                 A_min_tmp * rt_powd_snf(d5, 3.0) * 4.0) -
                                b_y) -
                               l15_tmp * l8 * 24.0) -
                              l15_tmp * l9 * 36.0) +
                             A_min * d2 * l8 * d5 * 12.0) +
                            b_A_min_tmp * d3 * 60.0) -
                           c_A_min_tmp * d * d3 * 24.0) -
                          d1 * l18 * 12.0) +
                         b_A_min) -
                        c_A_min) +
                       c_J_min) +
                      d_V_init) +
                     A_min * d * l7_tmp * l9 * 24.0) +
                    A_wayp * d * l4 * l9 * 12.0) -
                   d7 * l7_tmp * 24.0) +
                  d_A_min_tmp * d * l9 * 24.0) +
                 d_A_min * d5 * 24.0) -
                e_A_min * d * l9 * 24.0) +
               f_A_min * d * l8 * 24.0) +
              c_A_wayp * d * l9 * 24.0) -
             d_A_wayp * d * l8 * 24.0) +
            d7 * d8 * 12.0) -
           b_A_init) -
          g_A_min) +
         h_A_min) /
       ((((i_A_min * d3 * 12.0 - A_min_tmp * d4 * 12.0) + j_A_min) +
         b_A_min_tmp * d * l9 * 24.0) -
        k_A_min * d * l8 * 24.0);
  l26_idx_0 = d3;
  d = J_max * t7_idx_1;
  d1 = b_A_wayp_tmp * d;
  d2 = d * d;
  d3 = J_max * d2;
  d4 = J_min * d2;
  b_A_wayp_tmp = A_wayp + -d;
  l6 = (((d6 + -(A_wayp_tmp * d * 2.0)) + d1 * 2.0) + d4) + J_max * d * -d;
  l3 = A_min * rt_powd_snf(d, 3.0);
  l15_tmp = A_min_tmp_tmp * d2;
  d7 = l19 * d;
  d6 = b_A_wayp_tmp * b_A_wayp_tmp;
  d3 = -((((((((((((((((((((((((((((((((((((((((((b_l6 + l18 * d3 * 6.0) -
                                                 l17 * l6 * 6.0) -
                                                l18 * l6 * 12.0) +
                                               l6 * l6 * 3.0) -
                                              d2 * l4 * l9 * 6.0) -
                                             b_J_min) -
                                            b_A_wayp) -
                                           b_V_init) +
                                          V_init_tmp * d3 * 12.0) -
                                         c_V_init * d4 * 12.0) +
                                        b_V_max) +
                                       y) +
                                      d * l5 * l9 * 12.0) -
                                     b_l3) +
                                    b_l5 * b_A_wayp_tmp * 12.0) +
                                   l3 * l8 * 16.0) +
                                  l3 * l9 * 12.0) +
                                 A_min_tmp * rt_powd_snf(b_A_wayp_tmp, 3.0) *
                                     4.0) -
                                b_y) -
                               l15_tmp * l8 * 24.0) -
                              l15_tmp * l9 * 36.0) +
                             A_min * d2 * l8 * b_A_wayp_tmp * 12.0) +
                            b_A_min_tmp * d3 * 60.0) -
                           c_A_min_tmp * d * d3 * 24.0) -
                          d1 * l18 * 12.0) +
                         b_A_min) -
                        c_A_min) +
                       c_J_min) +
                      d_V_init) +
                     A_min * d * l7_tmp * l9 * 24.0) +
                    A_wayp * d * l4 * l9 * 12.0) -
                   d7 * l7_tmp * 24.0) +
                  d_A_min_tmp * d * l9 * 24.0) +
                 d_A_min * b_A_wayp_tmp * 24.0) -
                e_A_min * d * l9 * 24.0) +
               f_A_min * d * l8 * 24.0) +
              c_A_wayp * d * l9 * 24.0) -
             d_A_wayp * d * l8 * 24.0) +
            d7 * d6 * 12.0) -
           b_A_init) -
          g_A_min) +
         h_A_min) /
       ((((i_A_min * d3 * 12.0 - A_min_tmp * d4 * 12.0) + j_A_min) +
         b_A_min_tmp * d * l9 * 24.0) -
        k_A_min * d * l8 * 24.0);
  l15_tmp = A_init + -A_min;
  l3 = A_min * (1.0 / J_max);
  l6 = -(1.0 / J_min * l15_tmp);
  b_J_min =
      (((J_min_tmp * 2.0 - J_min * V_max * 2.0) - A_init * l15_tmp * 2.0) -
       l7_tmp) +
      l15_tmp * l15_tmp;
  b_A_min = c_A_min_tmp * -A_min / J_max;
  t[0] = l6;
  t[1] = l6;
  t[4] = -l3;
  t[5] = -l3;
  t[2] = ((((b_J_min + d8) + J_min * l2_idx_0 * 0.0) + J_min * d5 * 0.0 * 2.0) +
          b_A_min) *
         -0.5 / c_A_min_tmp;
  t[6] = l26_idx_0;
  t[8] = (A_wayp - l2_idx_0) / J_min;
  t[10] = 0.0;
  t[12] = 0.0;
  t[3] = ((((b_J_min + d6) + J_min * d * t7_idx_1) +
           J_min * b_A_wayp_tmp * t7_idx_1 * 2.0) +
          b_A_min) *
         -0.5 / c_A_min_tmp;
  t[7] = d3;
  t[9] = (A_wayp - d) / J_min;
  t[11] = 0.0;
  t[13] = t7_idx_1;
}

// End of code generation (abcdeg_NO_AP.cpp)
