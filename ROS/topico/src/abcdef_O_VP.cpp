//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// abcdef_O_VP.cpp
//
// Code generation for function 'abcdef_O_VP'
//

// Include files
#include "abcdef_O_VP.h"
#include "rt_nonfinite.h"
#include "topico_wrapper_rtwutil.h"

// Function Definitions
void abcdef_O_VP(double P_init, double V_init, double A_init, double P_wayp,
                 double V_wayp, double V_max, double A_max, double A_min,
                 double J_max, double J_min, double t[7])
{
  double b_t2_tmp;
  double b_t4_tmp;
  double c_t2_tmp;
  double c_t4_tmp;
  double d_t2_tmp;
  double d_t4_tmp;
  double e_t4_tmp;
  double f_t4_tmp;
  double g_t4_tmp;
  double h_t4_tmp;
  double i_t4_tmp;
  double j_t4_tmp;
  double k_t4_tmp;
  double l2_tmp;
  double l3_tmp;
  double l5;
  double l6;
  double l7;
  double l8;
  double l9;
  double l_t4_tmp;
  double m_t4_tmp;
  double t2;
  double t2_tmp;
  double t4_tmp;
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
  //  Generated on 04-Sep-2019 13:54:44
  l2_tmp = A_max * A_max;
  t2_tmp = A_init * A_init;
  b_t2_tmp = J_max * V_init;
  c_t2_tmp = A_max * J_max;
  d_t2_tmp = J_max * l2_tmp;
  t2 = ((((-l2_tmp - b_t2_tmp * 2.0) + J_max * V_max * 2.0) + t2_tmp) +
        d_t2_tmp / J_min) /
       (c_t2_tmp * 2.0);
  l3_tmp = A_min * A_min;
  l5 = rt_powd_snf(A_max, 3.0);
  l7 = J_min * J_min;
  l8 = J_max * J_max;
  l9 = t2 * t2;
  l6 = l2_tmp * l2_tmp;
  t4_tmp = A_min * l5;
  b_t4_tmp = J_min * J_max;
  c_t4_tmp = b_t4_tmp * t2_tmp;
  d_t4_tmp = J_min * V_init;
  e_t4_tmp = A_min * A_max;
  f_t4_tmp = A_min * J_min;
  g_t4_tmp = A_min * J_max;
  h_t4_tmp = e_t4_tmp * J_min;
  i_t4_tmp = g_t4_tmp * t2_tmp * l7;
  j_t4_tmp = f_t4_tmp * l2_tmp * l8;
  g_t4_tmp = g_t4_tmp * l2_tmp * l7;
  k_t4_tmp = A_min * V_init * l7 * l8;
  l_t4_tmp = e_t4_tmp * l7 * l8;
  m_t4_tmp = A_max * J_min;
  f_t4_tmp *= J_max;
  t[0] = -(A_init - A_max) / J_max;
  t[1] = t2;
  t[2] = -A_max / J_min;
  t[3] = -(((((((((((((((((((((((((((((((((((((l6 * l7 * -3.0 - l6 * l8 * 3.0) -
                                              t2_tmp * t2_tmp * l7 * 3.0) +
                                             l3_tmp * l3_tmp * l8) +
                                            b_t4_tmp * l6 * 6.0) +
                                           t4_tmp * l7 * 4.0) +
                                          t4_tmp * l8 * 8.0) +
                                         rt_powd_snf(A_init, 3.0) * A_min * l7 *
                                             8.0) +
                                        t2_tmp * l2_tmp * l7 * 6.0) -
                                       l3_tmp * l2_tmp * l8 * 6.0) -
                                      V_init * V_init * l7 * l8 * 12.0) +
                                     V_wayp * V_wayp * l7 * l8 * 12.0) +
                                    J_min * l5 * l8 * t2 * 12.0) -
                                   J_max * l5 * l7 * t2 * 12.0) -
                                  l2_tmp * l7 * l8 * l9 * 12.0) -
                                 f_t4_tmp * l5 * 12.0) -
                                e_t4_tmp * t2_tmp * l7 * 12.0) +
                               A_min * P_init * l7 * l8 * 24.0) -
                              A_min * P_wayp * l7 * l8 * 24.0) -
                             c_t4_tmp * l3_tmp * 6.0) -
                            c_t4_tmp * l2_tmp * 6.0) +
                           b_t4_tmp * l3_tmp * l2_tmp * 6.0) +
                          d_t4_tmp * l3_tmp * l8 * 12.0) +
                         b_t2_tmp * t2_tmp * l7 * 12.0) +
                        d_t4_tmp * l2_tmp * l8 * 12.0) -
                       b_t2_tmp * l2_tmp * l7 * 12.0) +
                      l_t4_tmp * l9 * 12.0) -
                     i_t4_tmp * t2 * 12.0) -
                    j_t4_tmp * t2 * 24.0) +
                   g_t4_tmp * t2 * 12.0) +
                  m_t4_tmp * l3_tmp * l8 * t2 * 12.0) +
                 c_t2_tmp * t2_tmp * l7 * t2 * 12.0) +
                k_t4_tmp * t2 * 24.0) -
               A_max * V_init * l7 * l8 * t2 * 24.0) +
              h_t4_tmp * J_max * t2_tmp * 12.0) -
             A_init * A_min * J_max * V_init * l7 * 24.0) -
            h_t4_tmp * V_init * l8 * 24.0) +
           e_t4_tmp * J_max * V_init * l7 * 24.0) /
         ((((i_t4_tmp * -12.0 - j_t4_tmp * 12.0) + g_t4_tmp * 12.0) +
           k_t4_tmp * 24.0) +
          l_t4_tmp * t2 * 24.0);
  t[4] = A_min / J_min;
  t[5] = ((((((J_min * l2_tmp - d_t2_tmp) - t2_tmp * J_min) + l3_tmp * J_max) +
            b_t4_tmp * V_init * 2.0) -
           b_t4_tmp * V_wayp * 2.0) +
          m_t4_tmp * J_max * t2 * 2.0) *
         -0.5 / f_t4_tmp;
  t[6] = 0.0;
}

// End of code generation (abcdef_O_VP.cpp)
