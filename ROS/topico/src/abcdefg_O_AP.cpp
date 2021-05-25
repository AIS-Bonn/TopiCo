//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// abcdefg_O_AP.cpp
//
// Code generation for function 'abcdefg_O_AP'
//

// Include files
#include "abcdefg_O_AP.h"
#include "rt_nonfinite.h"
#include "topico_wrapper_rtwutil.h"

// Function Definitions
void abcdefg_O_AP(double P_init, double V_init, double A_init, double P_wayp,
                  double A_wayp, double V_max, double A_max, double A_min,
                  double J_max, double J_min, double t[7])
{
  double b_l2_tmp;
  double b_t4_tmp;
  double c_t4_tmp;
  double d_t4_tmp;
  double e_t4_tmp;
  double l2_tmp;
  double l3;
  double l4_tmp;
  double l5;
  double l6;
  double l7;
  double l9;
  double t4_tmp;
  double t4_tmp_tmp;
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
  l2_tmp = A_min * A_min;
  l3 = A_wayp * A_wayp;
  l3 = (((J_min * l2_tmp - J_min * l3) - J_max * l2_tmp) + J_max * l3) /
       (A_min * J_min * J_max * 2.0);
  b_l2_tmp = A_init * A_init;
  l4_tmp = A_max * A_max;
  l6 = J_min * J_min;
  l7 = J_max * J_max;
  l5 = l4_tmp * l4_tmp;
  l9 = A_min + -A_wayp;
  t4_tmp = J_max * V_init;
  b_t4_tmp = A_max * J_min;
  t4_tmp_tmp = A_min * A_max;
  c_t4_tmp = t4_tmp_tmp * l6;
  d_t4_tmp = A_max * V_max * l6 * l7;
  e_t4_tmp = A_max * J_max;
  t[0] = -(A_init - A_max) / J_max;
  t[1] = ((((-l4_tmp - t4_tmp * 2.0) + J_max * V_max * 2.0) + b_l2_tmp) +
          J_max * l4_tmp / J_min) /
         (e_t4_tmp * 2.0);
  t[2] = -A_max / J_min;
  t[3] = ((((((((((((((((((((((l5 * l6 - l5 * l7) -
                              b_l2_tmp * b_l2_tmp * l6 * 3.0) +
                             rt_powd_snf(A_init, 3.0) * A_max * l6 * 8.0) +
                            rt_powd_snf(A_min, 3.0) * A_max * l7 * 4.0) -
                           b_l2_tmp * l4_tmp * l6 * 6.0) -
                          A_max * l6 * rt_powd_snf(l9, 3.0) * 4.0) -
                         V_init * V_init * l6 * l7 * 12.0) +
                        V_max * V_max * l6 * l7 * 12.0) +
                       c_t4_tmp * (l9 * l9) * 12.0) +
                      A_max * P_init * l6 * l7 * 24.0) -
                     A_max * P_wayp * l6 * l7 * 24.0) +
                    t4_tmp * b_l2_tmp * l6 * 12.0) +
                   t4_tmp * l4_tmp * l6 * 12.0) -
                  J_min * V_max * l4_tmp * l7 * 12.0) -
                 b_t4_tmp * J_max * l2_tmp * l9 * 12.0) -
                e_t4_tmp * V_max * l6 * l9 * 24.0) +
               b_t4_tmp * l2_tmp * l7 * l3 * 12.0) +
              d_t4_tmp * l3 * 24.0) +
             c_t4_tmp * l7 * (l3 * l3) * 12.0) -
            A_init * A_max * J_max * V_init * l6 * 24.0) +
           t4_tmp_tmp * J_min * V_max * l7 * 24.0) -
          t4_tmp_tmp * J_max * l6 * l9 * l3 * 24.0) *
         -0.041666666666666664 / d_t4_tmp;
  t[4] = A_min / J_min;
  t[5] = l3;
  t[6] = -(A_min - A_wayp) / J_max;
}

// End of code generation (abcdefg_O_AP.cpp)
