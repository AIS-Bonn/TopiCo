//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// abcdefg_T_AV.cpp
//
// Code generation for function 'abcdefg_T_AV'
//

// Include files
#include "abcdefg_T_AV.h"
#include "rt_nonfinite.h"
#include "topico_wrapper_rtwutil.h"

// Function Definitions
void abcdefg_T_AV(double V_init, double A_init, double V_wayp, double A_wayp,
                  double V_max, double A_max, double A_min, double J_max,
                  double J_min, double T, double t[7])
{
  double b_t2_tmp;
  double b_t4_tmp;
  double c_t2_tmp;
  double c_t4_tmp;
  double d_t4_tmp;
  double e_t4_tmp;
  double f_t4_tmp;
  double g_t4_tmp;
  double h_t4_tmp;
  double i_t4_tmp;
  double j_t4_tmp;
  double l2;
  double l3;
  double t2;
  double t2_tmp;
  double t4;
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
  //  Generated on 03-Sep-2019 11:38:50
  l2 = A_min * A_min;
  l3 = A_max * A_max;
  t2 = A_max * J_min;
  t4_tmp = A_min * J_min;
  b_t4_tmp = t2 * J_max;
  c_t4_tmp = A_min * A_max;
  d_t4_tmp = J_min * J_min;
  e_t4_tmp = A_init * A_init;
  f_t4_tmp = A_wayp * A_wayp;
  g_t4_tmp = A_init * A_min;
  h_t4_tmp = c_t4_tmp * J_min;
  i_t4_tmp = J_min * J_max;
  j_t4_tmp = t4_tmp * J_max;
  t4 =
      1.0 / d_t4_tmp / (J_max * J_max) *
      (i_t4_tmp *
           (((((((((((((rt_powd_snf(A_max, 3.0) * J_min - t4_tmp * l3 * 2.0) +
                       t2 * l2) +
                      A_min * J_max * l3) -
                     A_max * J_max * l2) -
                    e_t4_tmp * A_min * J_min) +
                   A_max * f_t4_tmp * J_min) +
                  g_t4_tmp * A_max * J_min * 2.0) -
                 c_t4_tmp * A_wayp * J_min * 2.0) +
                b_t4_tmp * V_init * 2.0) -
               j_t4_tmp * V_max * 2.0) +
              b_t4_tmp * V_max * 2.0) -
             b_t4_tmp * V_wayp * 2.0) +
            h_t4_tmp * J_max * T * 2.0) *
           2.0 +
       d_t4_tmp * J_max * (A_min - A_max) * (l3 + J_max * V_init * 2.0) * 2.0) /
      (c_t4_tmp * 4.0);
  t2_tmp = J_min * l3;
  b_t2_tmp = J_max * l2;
  c_t2_tmp = J_max * l3;
  e_t4_tmp *= J_min;
  d_t4_tmp = f_t4_tmp * J_min;
  l3 = i_t4_tmp * V_init * 2.0;
  t4_tmp = i_t4_tmp * V_wayp * 2.0;
  t2 = (((((((((((((J_min * l2 + t2_tmp) - b_t2_tmp) - c_t2_tmp) - e_t4_tmp) +
                d_t4_tmp) +
               g_t4_tmp * J_min * 2.0) -
              h_t4_tmp * 2.0) +
             c_t4_tmp * J_max * 2.0) -
            A_min * A_wayp * J_min * 2.0) +
           l3) -
          t4_tmp) +
         j_t4_tmp * T * 2.0) -
        j_t4_tmp * t4 * 2.0) /
       (j_t4_tmp * 2.0 - b_t4_tmp * 2.0);
  t[0] = -(A_init - A_max) / J_max;
  t[1] = t2;
  t[2] = -A_max / J_min;
  t[3] = t4;
  t[4] = A_min / J_min;
  t[5] = ((((((((-J_min * l2 + t2_tmp) + b_t2_tmp) - c_t2_tmp) - e_t4_tmp) +
             d_t4_tmp) +
            l3) -
           t4_tmp) +
          b_t4_tmp * t2 * 2.0) *
         -0.5 / j_t4_tmp;
  t[6] = -(A_min - A_wayp) / J_max;
}

// End of code generation (abcdefg_T_AV.cpp)
