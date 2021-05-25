//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// abcdeg_T_A.cpp
//
// Code generation for function 'abcdeg_T_A'
//

// Include files
#include "abcdeg_T_A.h"
#include "rt_nonfinite.h"

// Function Definitions
void abcdeg_T_A(double V_init, double A_init, double A_wayp, double V_max,
                double A_max, double J_max, double J_min, double T,
                double t[14])
{
  double A_init_tmp;
  double b_A_init;
  double l13;
  double l3;
  double l6;
  double t5_idx_0;
  double t5_idx_1;
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
  //  Generated on 29-Aug-2019 15:28:04
  l3 = A_wayp * (1.0 / J_min);
  t5_idx_0 = l3;
  t5_idx_1 = -l3;
  l3 = A_max * A_max;
  l13 = 1.0 / A_max * (1.0 / J_max) *
        ((((A_init * A_init + J_max * V_max * 2.0) + -(J_max * V_init * 2.0)) +
          -l3) +
         J_max * l3 * (1.0 / J_min)) /
        2.0;
  l3 = A_max * (1.0 / J_min);
  l6 = -(1.0 / J_max * (A_init + -A_max));
  A_init_tmp = J_min * J_max;
  b_A_init =
      ((A_init * J_min - A_max * J_min) + A_max * J_max) + A_init_tmp * T;
  t[0] = l6;
  t[1] = l6;
  t[4] = -l3;
  t[5] = -l3;
  l3 = (A_wayp - J_min * t5_idx_0) / J_max;
  t[2] = l13;
  t[6] = (b_A_init - A_init_tmp * ((l13 + t5_idx_0) + l3)) / A_init_tmp;
  t[8] = t5_idx_0;
  t[10] = 0.0;
  t[12] = l3;
  l3 = (A_wayp - J_min * t5_idx_1) / J_max;
  t[3] = l13;
  t[7] = (b_A_init - A_init_tmp * ((l13 + t5_idx_1) + l3)) / A_init_tmp;
  t[9] = t5_idx_1;
  t[11] = 0.0;
  t[13] = l3;
}

// End of code generation (abcdeg_T_A.cpp)
