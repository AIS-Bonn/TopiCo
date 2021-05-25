//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// abcef_T_V.cpp
//
// Code generation for function 'abcef_T_V'
//

// Include files
#include "abcef_T_V.h"
#include "rt_nonfinite.h"

// Function Definitions
void abcef_T_V(double V_init, double A_init, double V_wayp, double A_max,
               double A_min, double J_max, double J_min, double T, double t[7])
{
  double b_t6_tmp;
  double c_t6_tmp;
  double l2;
  double t6_tmp;
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
  //  Generated on 29-Aug-2019 16:37:18
  l2 = A_max * A_max;
  t6_tmp = J_min * J_max;
  b_t6_tmp = A_min + -A_max;
  c_t6_tmp = J_min * ((A_init + -A_max) + J_max * T);
  l2 = -((((((J_min * l2 - J_max * l2) - A_init * A_init * J_min) +
            A_min * A_min * J_max) +
           A_max * (c_t6_tmp - J_max * b_t6_tmp) * 2.0) +
          t6_tmp * V_init * 2.0) -
         t6_tmp * V_wayp * 2.0) /
       (A_min * J_min * J_max * 2.0 - A_max * J_min * J_max * 2.0);
  t[0] = -(A_init - A_max) / J_max;
  t[1] = (c_t6_tmp - J_max * (b_t6_tmp + J_min * l2)) / t6_tmp;
  t[2] = (A_min - A_max) / J_min;
  t[3] = 0.0;
  t[4] = 0.0;
  t[5] = l2;
  t[6] = 0.0;
}

// End of code generation (abcef_T_V.cpp)
