//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// abcefg_T_A.cpp
//
// Code generation for function 'abcefg_T_A'
//

// Include files
#include "abcefg_T_A.h"
#include "rt_nonfinite.h"
#include "topico_wrapper_rtwutil.h"

// Function Definitions
void abcefg_T_A(double V_init, double A_init, double A_wayp, double V_max,
                double A_max, double A_min, double J_max, double J_min,
                double T, double t[7])
{
  double b_t2_tmp;
  double c_t2_tmp;
  double d_t2_tmp;
  double e_t2_tmp;
  double f_t2_tmp;
  double l2;
  double l3;
  double l3_tmp;
  double l4;
  double t2_tmp;
  double t7;
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
  //  Generated on 03-Sep-2019 13:25:55
  l2 = A_min * A_min;
  l3_tmp = A_max * A_max;
  l4 = A_wayp * A_wayp;
  t7 = A_min * A_max;
  t2_tmp = A_init * A_init * J_min;
  b_t2_tmp = J_max * V_init * 2.0;
  c_t2_tmp = A_min * J_max;
  d_t2_tmp = J_min * J_max * V_max * 2.0;
  e_t2_tmp = A_min * J_min * J_max * 2.0;
  f_t2_tmp = J_max * l3_tmp;
  l2 = -((((((((J_min * l2 * -2.0 + J_max * l2) + f_t2_tmp) + J_max * l4) +
             t2_tmp) -
            J_min *
                ((((((-l2 + l3_tmp) + l4) + b_t2_tmp) + A_init * A_min * 2.0) -
                  t7 * 2.0) +
                 c_t2_tmp * T * 2.0)) -
           t7 * J_max * 2.0) +
          A_min * A_wayp * J_min * 2.0) +
         d_t2_tmp) /
       (e_t2_tmp - A_max * J_min * J_max * 2.0);
  t7 = -(A_min - A_wayp) / J_max;
  l3 = J_max * J_max;
  l4 = t7 * t7;
  t[0] = -(A_init - A_max) / J_max;
  t[1] = l2;
  t[2] = (A_min - A_max) / J_min;
  t[3] = 0.0;
  t[4] = 0.0;
  t[5] =
      (((((f_t2_tmp + t2_tmp) + rt_powd_snf(J_max, 3.0) * l4) -
         J_min * ((((l3_tmp + b_t2_tmp) + l3 * l4) + A_max * J_max * l2 * 2.0) +
                  c_t2_tmp * t7 * 2.0)) +
        d_t2_tmp) +
       A_min * l3 * t7 * 2.0) /
      e_t2_tmp;
  t[6] = t7;
}

// End of code generation (abcefg_T_A.cpp)
