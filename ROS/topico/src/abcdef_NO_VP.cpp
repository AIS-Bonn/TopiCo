//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// abcdef_NO_VP.cpp
//
// Code generation for function 'abcdef_NO_VP'
//

// Include files
#include "abcdef_NO_VP.h"
#include "rt_nonfinite.h"
#include "topico_wrapper_rtwutil.h"

// Function Definitions
void abcdef_NO_VP(double P_init, double V_init, double A_init, double P_wayp,
                  double V_wayp, double V_max, double A_min, double J_max,
                  double J_min, double t[7])
{
  double b_t2_tmp;
  double b_t_tmp;
  double c_t2_tmp;
  double d_t2_tmp;
  double e_t2_tmp;
  double l2_tmp;
  double l4;
  double l5;
  double l6;
  double l7;
  double t2;
  double t2_tmp;
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
  //  Generated on 28-Aug-2019 13:51:13
  l2_tmp = A_min * A_min;
  t2_tmp = A_init * A_init;
  b_t2_tmp = J_min * V_init;
  c_t2_tmp = A_min * J_min;
  d_t2_tmp = b_t2_tmp * 2.0;
  e_t2_tmp = J_min * l2_tmp;
  t2 = ((((-l2_tmp - d_t2_tmp) + J_min * V_max * 2.0) + t2_tmp) +
        e_t2_tmp / J_max) /
       (c_t2_tmp * 2.0);
  l4 = rt_powd_snf(A_min, 3.0);
  l6 = J_min * J_min;
  l7 = J_max * J_max;
  l5 = l2_tmp * l2_tmp;
  t[0] = -(A_init - A_min) / J_min;
  t[1] = t2;
  t[2] = -A_min / J_max;
  t_tmp = J_min * J_max;
  b_t_tmp = J_min * l4 * l7;
  l4 = J_max * l4 * l6;
  t[3] = -((((((((((((((((l5 * l6 * 5.0 + l5 * l7 * 8.0) -
                         t2_tmp * t2_tmp * l7 * 3.0) -
                        t_tmp * l5 * 12.0) +
                       rt_powd_snf(A_init, 3.0) * A_min * l7 * 8.0) -
                      t2_tmp * l2_tmp * l7 * 12.0) -
                     V_init * V_init * l6 * l7 * 12.0) +
                    V_wayp * V_wayp * l6 * l7 * 12.0) +
                   b_t_tmp * t2 * 12.0) -
                  l4 * t2 * 12.0) +
                 A_min * P_init * l6 * l7 * 24.0) -
                A_min * P_wayp * l6 * l7 * 24.0) +
               t_tmp * t2_tmp * l2_tmp * 6.0) +
              b_t2_tmp * t2_tmp * l7 * 12.0) +
             b_t2_tmp * l2_tmp * l7 * 24.0) -
            J_max * V_init * l2_tmp * l6 * 12.0) -
           A_init * A_min * J_min * V_init * l7 * 24.0) /
         ((((b_t_tmp * 12.0 - l4 * 12.0) + l2_tmp * l6 * l7 * t2 * 24.0) -
           c_t2_tmp * t2_tmp * l7 * 12.0) +
          A_min * V_init * l6 * l7 * 24.0);
  t[4] = A_min / J_min;
  t[5] =
      (((e_t2_tmp + J_max * t2_tmp * 2.0) -
        J_max * (((t2_tmp + l2_tmp * 2.0) + d_t2_tmp) + c_t2_tmp * t2 * 2.0)) +
       t_tmp * V_wayp * 2.0) /
      (c_t2_tmp * J_max * 2.0);
  t[6] = 0.0;
}

// End of code generation (abcdef_NO_VP.cpp)
