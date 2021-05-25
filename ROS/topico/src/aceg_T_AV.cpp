//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// aceg_T_AV.cpp
//
// Code generation for function 'aceg_T_AV'
//

// Include files
#include "aceg_T_AV.h"
#include "rt_nonfinite.h"

// Function Definitions
void aceg_T_AV(double V_init, double A_init, double V_wayp, double A_wayp,
               double J_max, double J_min, double T, double t[7])
{
  double l5;
  double l6;
  double l7;
  double l7_tmp;
  double l8;
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
  //  Generated on 03-Sep-2019 11:12:37
  l5 = J_min + -J_max;
  l7_tmp = J_max * T;
  l7 = (A_init + l7_tmp) + -A_wayp;
  l6 = l5 * l5;
  l8 = l7 * l7;
  t7_tmp = J_min * J_max;
  l7 = (((((((A_init * A_init * l6 - A_wayp * A_wayp * l6) -
             J_min * J_min * l8) +
            t7_tmp * l8) -
           J_max * V_init * l6 * 2.0) +
          J_max * V_wayp * l6 * 2.0) -
         A_wayp * J_min * l5 * l7 * 2.0) +
        A_wayp * J_max * l5 * l7 * 2.0) /
       (J_max * J_max * l5 * l7 * 2.0 - t7_tmp * l5 * l7 * 2.0);
  l6 = A_init - A_wayp;
  l5 = -(l6 + l7_tmp) / (J_min - J_max);
  t[0] = -((l6 + J_min * l5) + J_max * l7) / J_max;
  t[1] = 0.0;
  t[2] = l5;
  t[3] = 0.0;
  t[4] = 0.0;
  t[5] = 0.0;
  t[6] = l7;
}

// End of code generation (aceg_T_AV.cpp)
