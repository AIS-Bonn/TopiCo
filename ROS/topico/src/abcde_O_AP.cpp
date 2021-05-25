//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// abcde_O_AP.cpp
//
// Code generation for function 'abcde_O_AP'
//

// Include files
#include "abcde_O_AP.h"
#include "rt_nonfinite.h"
#include "topico_wrapper_rtwutil.h"

// Function Definitions
void abcde_O_AP(double P_init, double V_init, double A_init, double P_wayp,
                double A_wayp, double V_max, double A_max, double J_max,
                double J_min, double t[7])
{
  double l2_tmp;
  double l3_tmp;
  double l4;
  double l5;
  double l6;
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
  //  Generated on 28-Aug-2019 17:25:45
  l2_tmp = A_init * A_init;
  l3_tmp = A_max * A_max;
  l5 = J_min * J_min;
  l6 = J_max * J_max;
  l4 = l3_tmp * l3_tmp;
  t4_tmp = J_max * V_init;
  t[0] = -(A_init - A_max) / J_max;
  t[1] = ((((-l3_tmp - t4_tmp * 2.0) + J_max * V_max * 2.0) + l2_tmp) +
          J_max * l3_tmp / J_min) /
         (A_max * J_max * 2.0);
  t[2] = -A_max / J_min;
  t[3] = ((((((((((((((l4 * l5 - l4 * l6) - l2_tmp * l2_tmp * l5 * 3.0) +
                     rt_powd_snf(A_init, 3.0) * A_max * l5 * 8.0) +
                    A_max * rt_powd_snf(A_wayp, 3.0) * l6 * 4.0) -
                   l2_tmp * l3_tmp * l5 * 6.0) -
                  V_init * V_init * l5 * l6 * 12.0) +
                 V_max * V_max * l5 * l6 * 12.0) +
                A_max * P_init * l5 * l6 * 24.0) -
               A_max * P_wayp * l5 * l6 * 24.0) +
              t4_tmp * l2_tmp * l5 * 12.0) +
             t4_tmp * l3_tmp * l5 * 12.0) -
            J_min * V_max * l3_tmp * l6 * 12.0) -
           A_init * A_max * J_max * V_init * l5 * 24.0) +
          A_max * A_wayp * J_min * V_max * l6 * 24.0) *
         -0.041666666666666664 / (A_max * V_max * l5 * l6);
  t[4] = A_wayp / J_min;
  t[5] = 0.0;
  t[6] = 0.0;
}

// End of code generation (abcde_O_AP.cpp)
