//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// abcdef_T_V.cpp
//
// Code generation for function 'abcdef_T_V'
//

// Include files
#include "abcdef_T_V.h"
#include "rt_nonfinite.h"
#include "topico_wrapper_rtwutil.h"

// Function Definitions
void abcdef_T_V(double V_init, double A_init, double V_wayp, double V_max,
                double A_max, double A_min, double J_max, double J_min,
                double T, double t[7])
{
  double b_t4_tmp;
  double c_t4_tmp;
  double l2;
  double l2_tmp;
  double l3_tmp;
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
  //  Generated on 02-Sep-2019 15:55:12
  l2_tmp = A_init * A_init;
  l3_tmp = A_max * A_max;
  t2 = A_min * J_min;
  l2 = t2 * J_max;
  t4_tmp = A_max * J_min;
  t4 = A_min * A_max;
  b_t4_tmp = J_min * J_max;
  c_t4_tmp = J_max * l3_tmp;
  t4 = (((((((((rt_powd_snf(A_max, 3.0) * J_max -
                A_max * (((((J_min * l2_tmp + c_t4_tmp) +
                            A_init * A_min * J_min * 2.0) +
                           t4 * J_max * 2.0) +
                          b_t4_tmp * V_max * 2.0) +
                         l2 * T * 2.0)) +
               t2 * l2_tmp) +
              t2 * l3_tmp) +
             t4_tmp * l2_tmp) +
            A_min * J_max * l3_tmp) +
           A_min * A_min * A_max * J_max) -
          l2 * V_init * 2.0) +
         l2 * V_max * 2.0) +
        t4_tmp * J_max * V_wayp * 2.0) *
       -0.5 / (t4 * J_min * J_max);
  t2_tmp = A_max * J_max;
  t2 = ((((-l3_tmp - J_max * V_init * 2.0) + J_max * V_max * 2.0) + l2_tmp) +
        c_t4_tmp / J_min) /
       (t2_tmp * 2.0);
  l2 = 1.0 / J_min;
  t[0] = -(A_init - A_max) / J_max;
  t[1] = t2;
  t[2] = -A_max / J_min;
  t[3] = t4;
  t[4] = A_min / J_min;
  t[5] = l2 *
         ((((A_init * J_min - t4_tmp) + t2_tmp) + b_t4_tmp * T) -
          b_t4_tmp * ((t2 + t4) + A_min * l2)) /
         J_max;
  t[6] = 0.0;
}

// End of code generation (abcdef_T_V.cpp)
