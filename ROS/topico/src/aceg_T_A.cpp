//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// aceg_T_A.cpp
//
// Code generation for function 'aceg_T_A'
//

// Include files
#include "aceg_T_A.h"
#include "rt_nonfinite.h"

// Function Definitions
void aceg_T_A(double V_init, double A_init, double A_wayp, double V_max,
              double J_max, double J_min, double T, double t[7])
{
  double b_t7_tmp;
  double c_t7_tmp;
  double l2;
  double l6;
  double l7;
  double l8;
  double l9;
  double l9_tmp;
  double t7_tmp;
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
  //  Generated on 03-Sep-2019 12:37:25
  l2 = J_min * T;
  l6 = J_min + -J_max;
  l8 = (A_init + l2) + -A_wayp;
  l9_tmp = J_max * T;
  l9 = (A_init + l9_tmp) + -A_wayp;
  l7 = l6 * l6;
  t7_tmp = A_init * J_min;
  b_t7_tmp = J_min * l9;
  c_t7_tmp = J_min * J_max;
  l6 = ((((((J_min * J_min * (l9 * l9) + J_min * V_init * l7 * 2.0) -
            J_min * V_max * l7 * 2.0) +
           A_wayp * -A_wayp * l7) -
          b_t7_tmp * ((J_max * l2 + J_max * -A_wayp) + t7_tmp) * 2.0) +
         c_t7_tmp * (l8 * l8)) +
        t7_tmp * l6 * l8 * 2.0) /
       (((b_t7_tmp * (J_min * -J_max + J_max * J_max) * 2.0 +
          t7_tmp * l7 * 2.0) -
         A_wayp * J_min * l7 * 2.0) +
        c_t7_tmp * l6 * l8 * 2.0);
  l8 = A_init - A_wayp;
  l2 = -(l8 + l9_tmp) / (J_min - J_max);
  t[0] = -((l8 + J_min * l2) + J_max * l6) / J_max;
  t[1] = 0.0;
  t[2] = l2;
  t[3] = 0.0;
  t[4] = 0.0;
  t[5] = 0.0;
  t[6] = l6;
}

// End of code generation (aceg_T_A.cpp)
