//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// ac_T_P.cpp
//
// Code generation for function 'ac_T_P'
//

// Include files
#include "ac_T_P.h"
#include "rt_nonfinite.h"
#include "topico_wrapper_data.h"
#include "topico_wrapper_rtwutil.h"
#include "topico_wrapper_types.h"

// Function Definitions
void ac_T_P(double P_init, double V_init, double A_init, double P_wayp,
            double J_max, double J_min, double T, creal_T t[21])
{
  creal_T t1[3];
  double l19;
  double l22;
  double l25;
  double l35_im;
  double l4;
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
  //  Generated on 29-Aug-2019 14:04:13
  l4 = T * T;
  l19 = 1.0 / (J_min + -J_max);
  l22 = J_min * T * 3.0 + -(J_max * T * 3.0);
  l25 = l19 * l22 / 3.0;
  l4 = (rt_powd_snf(l19, 3.0) * rt_powd_snf(l22, 3.0) * 0.07407407407407407 +
        l19 * ((((P_init * 6.0 + T * V_init * 6.0) + -(P_wayp * 6.0)) +
                J_min * rt_powd_snf(T, 3.0)) +
               A_init * l4 * 3.0)) +
       -(l19 * l19 * l22 * (J_min * l4 * 3.0 + -(J_max * l4 * 3.0)) / 3.0);
  l19 = rt_powd_snf(l4, 0.33333333333333331);
  if (l4 < 0.0) {
    k_rtErrorWithMessageID(o_emlrtRTEI.fName, o_emlrtRTEI.lineNo);
  }
  l22 = 1.7320508075688772 * l19 * 0.0;
  l35_im = 1.7320508075688772 * l19 * 0.5;
  t1[0].re = l25 + l19;
  t1[0].im = 0.0;
  l4 = l25 + -(l19 / 2.0);
  t1[1].re = l4 - l22;
  t1[1].im = 0.0 - l35_im;
  t1[2].re = l4 + l22;
  t1[2].im = l35_im;
  t[0] = t1[0];
  t[3].re = 0.0;
  t[3].im = 0.0;
  t[6].re = T - t1[0].re;
  t[6].im = 0.0;
  t[9].re = 0.0;
  t[9].im = 0.0;
  t[12].re = 0.0;
  t[12].im = 0.0;
  t[15].re = 0.0;
  t[15].im = 0.0;
  t[18].re = 0.0;
  t[18].im = 0.0;
  t[1] = t1[1];
  t[4].re = 0.0;
  t[4].im = 0.0;
  t[7].re = T - t1[1].re;
  t[7].im = 0.0 - (0.0 - l35_im);
  t[10].re = 0.0;
  t[10].im = 0.0;
  t[13].re = 0.0;
  t[13].im = 0.0;
  t[16].re = 0.0;
  t[16].im = 0.0;
  t[19].re = 0.0;
  t[19].im = 0.0;
  t[2] = t1[2];
  t[5].re = 0.0;
  t[5].im = 0.0;
  t[8].re = T - t1[2].re;
  t[8].im = 0.0 - l35_im;
  t[11].re = 0.0;
  t[11].im = 0.0;
  t[14].re = 0.0;
  t[14].im = 0.0;
  t[17].re = 0.0;
  t[17].im = 0.0;
  t[20].re = 0.0;
  t[20].im = 0.0;
}

// End of code generation (ac_T_P.cpp)
