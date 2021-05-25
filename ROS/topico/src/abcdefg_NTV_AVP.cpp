//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// abcdefg_NTV_AVP.cpp
//
// Code generation for function 'abcdefg_NTV_AVP'
//

// Include files
#include "abcdefg_NTV_AVP.h"
#include "rt_nonfinite.h"
#include "topico_wrapper_rtwutil.h"

// Function Definitions
void abcdefg_NTV_AVP(double P_init, double V_init, double A_init, double P_wayp,
                     double V_wayp, double A_wayp, double A_min, double J_max,
                     double J_min, double T, double t[7])
{
  double b_t6_tmp;
  double c_t6_tmp;
  double d;
  double d1;
  double d_t6_tmp;
  double e_t6_tmp;
  double f_t6_tmp;
  double g_t6_tmp;
  double h_t6_tmp;
  double i_t6_tmp;
  double j_t6_tmp;
  double l2_tmp;
  double l3_tmp;
  double l4;
  double l5;
  double l6_tmp;
  double l7;
  double l8;
  double l9;
  double t6_tmp;
  double t6_tmp_tmp;
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
  //  Generated on 28-Aug-2019 13:55:05
  l2_tmp = A_init * A_init;
  l3_tmp = A_min * A_min;
  l4 = rt_powd_snf(A_min, 3.0);
  l6_tmp = A_wayp * A_wayp;
  l7 = J_min * J_min;
  l8 = J_max * J_max;
  l9 = T * T;
  l5 = l3_tmp * l3_tmp;
  d = J_min * J_max;
  d1 = A_init * J_min;
  t6_tmp = A_min * J_max;
  b_t6_tmp = d * l2_tmp;
  c_t6_tmp = J_max * T;
  d_t6_tmp = J_max * V_init;
  e_t6_tmp = J_max * V_wayp;
  f_t6_tmp = A_min * T;
  t6_tmp_tmp = A_init * A_min;
  g_t6_tmp = t6_tmp_tmp * J_min;
  h_t6_tmp = A_min * J_min;
  i_t6_tmp = h_t6_tmp * J_max;
  j_t6_tmp = A_wayp * J_min;
  l7 =
      -((((((((((((((((((((((((((((((((l5 * l7 * 32.0 - l5 * l8 * 8.0) +
                                      l2_tmp * l2_tmp * l8 * 3.0) +
                                     l6_tmp * l6_tmp * l7 * 3.0) -
                                    A_min *
                                        ((((((((((((((l4 * l7 * 18.0 +
                                                      rt_powd_snf(A_wayp, 3.0) *
                                                          l7 * 2.0) -
                                                     d * l4 * 6.0) +
                                                    A_min * l2_tmp * l8 * 6.0) +
                                                   A_min * l6_tmp * l7 * 9.0) -
                                                  A_wayp * l3_tmp * l7 * 6.0) +
                                                 P_init * l7 * l8 * 12.0) +
                                                d1 * J_max * l3_tmp * 18.0) -
                                               i_t6_tmp * l2_tmp * 9.0) +
                                              t6_tmp * V_init * l7 * 18.0) -
                                             t6_tmp * V_wayp * l7 * 18.0) +
                                            c_t6_tmp * l3_tmp * l7 * 18.0) +
                                           T * V_init * l7 * l8 * 12.0) +
                                          A_min * l7 * l8 * l9 * 6.0) +
                                         g_t6_tmp * T * l8 * 12.0) *
                                        2.0) +
                                   A_init * l4 * l8 * 12.0) -
                                  rt_powd_snf(A_init, 3.0) * A_min * l8 * 8.0) +
                                 l2_tmp * l3_tmp * l8 * 12.0) +
                                l3_tmp * l6_tmp * l7 * 12.0) +
                               V_init * V_init * l7 * l8 * 12.0) +
                              V_wayp * V_wayp * l7 * l8 * 12.0) +
                             l3_tmp * l7 * l8 * l9 * 12.0) +
                            A_init * J_min * J_max * l4 * 24.0) -
                           j_t6_tmp * J_max * l4 * 12.0) +
                          A_min * P_wayp * l7 * l8 * 24.0) -
                         b_t6_tmp * l3_tmp * 12.0) -
                        b_t6_tmp * l6_tmp * 6.0) +
                       J_min * T * l4 * l8 * 12.0) +
                      c_t6_tmp * l4 * l7 * 24.0) -
                     J_min * V_init * l2_tmp * l8 * 12.0) +
                    d_t6_tmp * l3_tmp * l7 * 24.0) +
                   d_t6_tmp * l6_tmp * l7 * 12.0) +
                  J_min * V_wayp * l2_tmp * l8 * 12.0) -
                 e_t6_tmp * l3_tmp * l7 * 24.0) -
                e_t6_tmp * l6_tmp * l7 * 12.0) -
               V_init * V_wayp * l7 * l8 * 24.0) +
              d1 * T * l3_tmp * l8 * 24.0) +
             t6_tmp * T * l6_tmp * l7 * 12.0) +
            f_t6_tmp * V_init * l7 * l8 * 24.0) -
           f_t6_tmp * V_wayp * l7 * l8 * 24.0) +
          g_t6_tmp * J_max * l6_tmp * 12.0) +
         g_t6_tmp * V_init * l8 * 24.0) -
        g_t6_tmp * V_wayp * l8 * 24.0) /
      ((((((((J_min * l4 * l8 * -12.0 + J_max * l4 * l7 * 12.0) +
             T * l3_tmp * l7 * l8 * 24.0) +
            d1 * l3_tmp * l8 * 24.0) -
           h_t6_tmp * l2_tmp * l8 * 12.0) +
          t6_tmp * l6_tmp * l7 * 12.0) -
         A_wayp * J_max * l3_tmp * l7 * 24.0) +
        A_min * V_init * l7 * l8 * 24.0) -
       A_min * V_wayp * l7 * l8 * 24.0);
  l4 = ((((((((J_min * l3_tmp * 2.0 - J_max * l3_tmp * 2.0) - l2_tmp * J_max) +
             l6_tmp * J_min) +
            t6_tmp_tmp * J_max * 2.0) -
           A_min * A_wayp * J_min * 2.0) +
          d * V_init * 2.0) -
         d * V_wayp * 2.0) +
        i_t6_tmp * T * 2.0) /
       (i_t6_tmp * 2.0);
  t[0] = -(A_init - A_min) / J_min;
  t[1] = -((((((-A_init * J_max - h_t6_tmp * 2.0) + t6_tmp * 2.0) + j_t6_tmp) -
             d * T) +
            d * l4) +
           d * l7) /
         d;
  t[2] = -A_min / J_max;
  t[3] = l4;
  t[4] = A_min / J_min;
  t[5] = l7;
  t[6] = -(A_min - A_wayp) / J_max;
}

// End of code generation (abcdefg_NTV_AVP.cpp)
