//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// abcdeg_O_AP.cpp
//
// Code generation for function 'abcdeg_O_AP'
//

// Include files
#include "abcdeg_O_AP.h"
#include "rt_nonfinite.h"
#include "topico_wrapper_rtwutil.h"

// Function Definitions
void abcdeg_O_AP(double P_init, double V_init, double A_init, double P_wayp,
                 double A_wayp, double V_max, double A_max, double J_max,
                 double J_min, double t[14])
{
  double A_max_tmp;
  double A_wayp_tmp;
  double b_A_init;
  double b_A_max;
  double b_A_max_tmp;
  double b_A_wayp;
  double b_J_max;
  double b_J_min;
  double b_V_max;
  double b_l5;
  double b_l6;
  double b_y;
  double c_A_max;
  double c_A_max_tmp;
  double c_A_wayp;
  double c_J_max;
  double c_J_min;
  double c_l5;
  double c_y;
  double d_A_max;
  double d_A_max_tmp;
  double d_A_wayp;
  double d_J_min;
  double d_y;
  double e_A_max;
  double e_A_max_tmp;
  double e_A_wayp;
  double e_J_min;
  double f_A_max;
  double f_A_max_tmp;
  double f_A_wayp;
  double f_J_min;
  double g_A_max;
  double g_A_wayp;
  double g_J_min;
  double h_A_max;
  double h_A_wayp;
  double h_J_min;
  double i_A_max;
  double i_A_wayp;
  double i_J_min;
  double j_A_max;
  double k_A_max;
  double l10;
  double l2;
  double l2_tmp;
  double l3;
  double l3_tmp;
  double l4;
  double l5;
  double l6;
  double l7_tmp;
  double l8_tmp;
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
  //  Generated on 04-Sep-2019 17:09:23
  t7_idx_1 = A_wayp * 2.0 / J_max;
  l2_tmp = A_init * A_init;
  l3_tmp = A_max * A_max;
  l5 = A_wayp * A_wayp;
  l6 = J_min * J_min;
  l7_tmp = J_max * J_max;
  l8_tmp = rt_powd_snf(J_max, 3.0);
  l10 = rt_powd_snf(J_max, 5.0);
  l4 = l3_tmp * l3_tmp;
  l9 = l7_tmp * l7_tmp;
  y = rt_powd_snf(l7_tmp, 3.0);
  l4 = (l4 * l6 - l4 * l7_tmp) - l2_tmp * l2_tmp * l6 * 3.0;
  b_A_max = A_max * l10;
  b_A_wayp = A_wayp * l10;
  b_J_min = J_min * l10;
  b_y = rt_powd_snf(A_init, 3.0) * A_max * l6 * 8.0;
  c_A_max = A_max * rt_powd_snf(A_wayp, 3.0) * l7_tmp * 4.0;
  l2 = l2_tmp * l3_tmp * l6 * 6.0;
  l3 = l3_tmp * l9;
  b_l5 = l5 * l9;
  b_l6 = l6 * l9;
  c_y = V_init * V_init * l6 * l7_tmp * 12.0;
  d_y = V_max * V_max * l6 * l7_tmp * 12.0;
  c_J_min = J_min * l3_tmp * l8_tmp;
  d_J_min = J_min * l5 * l8_tmp;
  b_V_max = V_max * l6 * l8_tmp;
  c_l5 = l5 * l6 * l7_tmp;
  A_max_tmp = A_max * A_wayp;
  d_A_max = A_max_tmp * l9;
  b_A_max_tmp = A_max * J_min;
  c_A_max_tmp = b_A_max_tmp * l9;
  A_wayp_tmp = A_wayp * J_min;
  c_A_wayp = A_wayp_tmp * l9;
  e_A_max = A_max * P_init * l6 * l7_tmp * 24.0;
  f_A_max = A_max * P_wayp * l6 * l7_tmp * 24.0;
  l10 = J_max * V_init;
  b_J_max = l10 * l2_tmp * l6 * 12.0;
  c_J_max = l10 * l3_tmp * l6 * 12.0;
  l10 = J_min * V_max;
  e_J_min = l10 * l3_tmp * l7_tmp * 12.0;
  f_J_min = l10 * l9;
  d_A_max_tmp = A_max * l6 * l8_tmp;
  d_A_wayp = A_wayp * l6 * l8_tmp;
  g_A_max = A_max * l5 * l8_tmp;
  e_A_wayp = A_wayp * l3_tmp * l8_tmp;
  l10 = A_max_tmp * J_min;
  e_A_max_tmp = l10 * l8_tmp;
  h_A_max = b_A_max_tmp * V_max * l8_tmp;
  f_A_wayp = A_wayp_tmp * V_max * l8_tmp;
  A_max_tmp = A_max_tmp * l6 * l7_tmp;
  i_A_max = b_A_max_tmp * l5 * l7_tmp;
  g_A_wayp = A_wayp_tmp * l3_tmp * l7_tmp;
  f_A_max_tmp = A_max * V_max * l6 * l7_tmp;
  h_A_wayp = A_wayp * V_max * l6 * l7_tmp;
  b_A_init = A_init * A_max * J_max * V_init * l6 * 24.0;
  j_A_max = l10 * V_max * l7_tmp * 24.0;
  k_A_max = f_A_max_tmp * 24.0;
  g_J_min = (-J_min * l3_tmp + J_max * l3_tmp) + l2_tmp * J_min;
  l10 = J_min * J_max;
  h_J_min = l10 * V_init * 2.0;
  i_J_min = l10 * V_max * 2.0;
  i_A_wayp = A_wayp * l7_tmp;
  l7_tmp *= J_min;
  l3_tmp = A_wayp_tmp * J_max;
  l2_tmp = b_A_max_tmp * J_max * 2.0;
  l10 = A_max * (1.0 / J_min);
  l6 = -(1.0 / J_max * (A_init + -A_max));
  t[0] = l6;
  t[1] = l6;
  t[4] = -l10;
  t[5] = -l10;
  t[2] = ((((((g_J_min - l8_tmp * 0.0) - h_J_min) + i_J_min) +
            i_A_wayp * 0.0 * 2.0) +
           l7_tmp * 0.0) -
          l3_tmp * 0.0 * 2.0) /
         l2_tmp;
  t[6] = -(((((((((((((((((((((((((((((((((((((((l4 + y * 0.0 * 3.0) +
                                                b_A_max * 0.0 * 8.0) -
                                               b_A_wayp * 0.0 * 12.0) -
                                              b_J_min * 0.0 * 6.0) +
                                             b_y) +
                                            c_A_max) -
                                           l2) +
                                          l3 * 0.0 * 6.0) +
                                         b_l5 * 0.0 * 12.0) +
                                        b_l6 * 0.0 * 3.0) -
                                       c_y) +
                                      d_y) -
                                     c_J_min * 0.0 * 6.0) -
                                    d_J_min * 0.0 * 24.0) +
                                   b_V_max * 0.0 * 12.0) +
                                  c_l5 * 0.0 * 12.0) -
                                 d_A_max * 0.0 * 24.0) -
                                c_A_max_tmp * 0.0 * 12.0) +
                               c_A_wayp * 0.0 * 24.0) +
                              e_A_max) -
                             f_A_max) +
                            b_J_max) +
                           c_J_max) -
                          e_J_min) -
                         f_J_min * 0.0 * 12.0) +
                        d_A_max_tmp * 0.0 * 4.0) -
                       d_A_wayp * 0.0 * 12.0) +
                      g_A_max * 0.0 * 12.0) -
                     e_A_wayp * 0.0 * 12.0) +
                    e_A_max_tmp * 0.0 * 36.0) -
                   h_A_max * 0.0 * 24.0) +
                  f_A_wayp * 0.0 * 24.0) -
                 A_max_tmp * 0.0 * 12.0) -
                i_A_max * 0.0 * 12.0) +
               g_A_wayp * 0.0 * 12.0) +
              f_A_max_tmp * 0.0 * 24.0) -
             h_A_wayp * 0.0 * 24.0) -
            b_A_init) +
           j_A_max) /
         ((((c_A_max_tmp * 0.0 * -12.0 + k_A_max) + d_A_max_tmp * 0.0 * 12.0) +
           e_A_max_tmp * 0.0 * 24.0) -
          A_max_tmp * 0.0 * 24.0);
  t[8] = (A_wayp - J_max * 0.0) / J_min;
  t[10] = 0.0;
  t[12] = 0.0;
  l10 = t7_idx_1 * t7_idx_1;
  l9 = rt_powd_snf(t7_idx_1, 3.0);
  l5 = l10 * l10;
  l5 = -(((((((((((((((((((((((((((((((((((((((l4 + y * l5 * 3.0) +
                                              b_A_max * l9 * 8.0) -
                                             b_A_wayp * l9 * 12.0) -
                                            b_J_min * l5 * 6.0) +
                                           b_y) +
                                          c_A_max) -
                                         l2) +
                                        l3 * l10 * 6.0) +
                                       b_l5 * l10 * 12.0) +
                                      b_l6 * l5 * 3.0) -
                                     c_y) +
                                    d_y) -
                                   c_J_min * l10 * 6.0) -
                                  d_J_min * l10 * 24.0) +
                                 b_V_max * l10 * 12.0) +
                                c_l5 * l10 * 12.0) -
                               d_A_max * l10 * 24.0) -
                              c_A_max_tmp * l9 * 12.0) +
                             c_A_wayp * l9 * 24.0) +
                            e_A_max) -
                           f_A_max) +
                          b_J_max) +
                         c_J_max) -
                        e_J_min) -
                       f_J_min * l10 * 12.0) +
                      d_A_max_tmp * l9 * 4.0) -
                     d_A_wayp * l9 * 12.0) +
                    g_A_max * t7_idx_1 * 12.0) -
                   e_A_wayp * t7_idx_1 * 12.0) +
                  e_A_max_tmp * l10 * 36.0) -
                 h_A_max * t7_idx_1 * 24.0) +
                f_A_wayp * t7_idx_1 * 24.0) -
               A_max_tmp * l10 * 12.0) -
              i_A_max * t7_idx_1 * 12.0) +
             g_A_wayp * t7_idx_1 * 12.0) +
            f_A_max_tmp * t7_idx_1 * 24.0) -
           h_A_wayp * t7_idx_1 * 24.0) -
          b_A_init) +
         j_A_max) /
       ((((c_A_max_tmp * l10 * -12.0 + k_A_max) + d_A_max_tmp * l10 * 12.0) +
         e_A_max_tmp * t7_idx_1 * 24.0) -
        A_max_tmp * t7_idx_1 * 24.0);
  l10 = ((((((g_J_min - l8_tmp * l10) - h_J_min) + i_J_min) +
           i_A_wayp * t7_idx_1 * 2.0) +
          l7_tmp * l10) -
         l3_tmp * t7_idx_1 * 2.0) /
        l2_tmp;
  t[3] = l10;
  t[7] = l5;
  t[9] = (A_wayp - J_max * t7_idx_1) / J_min;
  t[11] = 0.0;
  t[13] = t7_idx_1;
}

// End of code generation (abcdeg_O_AP.cpp)
