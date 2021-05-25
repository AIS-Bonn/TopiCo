//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// abcdefg_NO_AVP.cpp
//
// Code generation for function 'abcdefg_NO_AVP'
//

// Include files
#include "abcdefg_NO_AVP.h"
#include "rt_nonfinite.h"
#include "topico_wrapper_rtwutil.h"

// Function Definitions
void abcdefg_NO_AVP(double P_init, double V_init, double A_init, double P_wayp,
                    double V_wayp, double A_wayp, double V_max, double A_min,
                    double J_max, double J_min, double t[7])
{
  double b_t4_tmp;
  double c_t4_tmp;
  double l2_tmp;
  double l3_tmp;
  double l4;
  double l5_tmp;
  double l6;
  double l7;
  double t4_tmp;
  double t_tmp;
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
  //  Generated on 28-Aug-2019 12:21:18
  l2_tmp = A_init * A_init;
  l3_tmp = A_min * A_min;
  l5_tmp = A_wayp * A_wayp;
  l6 = J_min * J_min;
  l7 = J_max * J_max;
  l4 = l3_tmp * l3_tmp;
  t4_tmp = J_min * V_init;
  b_t4_tmp = J_max * V_wayp;
  c_t4_tmp = J_min * V_max;
  t[0] = -(A_init - A_min) / J_min;
  t_tmp = A_min * J_min;
  t[1] = ((((-l3_tmp - t4_tmp * 2.0) + c_t4_tmp * 2.0) + l2_tmp) +
          J_min * l3_tmp / J_max) /
         (t_tmp * 2.0);
  t[2] = -A_min / J_max;
  t[3] = (((((((((((((((((((l4 * l6 * 2.0 - l4 * l7 * 2.0) +
                           l2_tmp * l2_tmp * l7 * 3.0) -
                          l5_tmp * l5_tmp * l6 * 3.0) -
                         rt_powd_snf(A_init, 3.0) * A_min * l7 * 8.0) +
                        A_min * rt_powd_snf(A_wayp, 3.0) * l6 * 8.0) +
                       l2_tmp * l3_tmp * l7 * 6.0) -
                      l3_tmp * l5_tmp * l6 * 6.0) +
                     V_init * V_init * l6 * l7 * 12.0) -
                    V_wayp * V_wayp * l6 * l7 * 12.0) -
                   A_min * P_init * l6 * l7 * 24.0) +
                  A_min * P_wayp * l6 * l7 * 24.0) -
                 t4_tmp * l2_tmp * l7 * 12.0) -
                t4_tmp * l3_tmp * l7 * 12.0) -
               c_t4_tmp * l3_tmp * l7 * 12.0) +
              J_max * V_max * l3_tmp * l6 * 12.0) +
             b_t4_tmp * l3_tmp * l6 * 12.0) +
            b_t4_tmp * l5_tmp * l6 * 12.0) +
           A_init * A_min * J_min * V_init * l7 * 24.0) -
          A_min * A_wayp * J_max * V_wayp * l6 * 24.0) /
         (A_min * V_max * l6 * l7 * 24.0);
  t[4] = A_min / J_min;
  t[5] =
      (((J_max * l3_tmp + l5_tmp * J_min) - J_min * (l3_tmp + b_t4_tmp * 2.0)) +
       J_min * J_max * V_max * 2.0) *
      -0.5 / (t_tmp * J_max);
  t[6] = -(A_min - A_wayp) / J_max;
}

// End of code generation (abcdefg_NO_AVP.cpp)
