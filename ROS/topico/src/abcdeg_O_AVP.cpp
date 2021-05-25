//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// abcdeg_O_AVP.cpp
//
// Code generation for function 'abcdeg_O_AVP'
//

// Include files
#include "abcdeg_O_AVP.h"
#include "power.h"
#include "rt_nonfinite.h"
#include "sqrt.h"
#include "topico_wrapper_data.h"
#include "topico_wrapper_rtwutil.h"

// Function Definitions
void abcdeg_O_AVP(double P_init, double V_init, double A_init, double P_wayp,
                  double V_wayp, double A_wayp, double V_max, double A_max,
                  double J_max, double J_min, creal_T t[14])
{
  creal_T b_y[2];
  creal_T dcv[2];
  creal_T l2[2];
  creal_T l9[2];
  creal_T t7[2];
  creal_T x[2];
  creal_T y[2];
  creal_T l19;
  double A_init_re;
  double A_max_re;
  double J_max_re;
  double J_min_re;
  double b_A_max;
  double b_A_max_re;
  double b_J_max_re;
  double b_re;
  double b_y_re;
  double c_A_max;
  double d;
  double d1;
  double d_A_max;
  double e_A_max;
  double im;
  double l15;
  double l15_tmp;
  double l19_tmp;
  double l3_tmp;
  double l4;
  double l4_tmp;
  double l5;
  double l6;
  double re;
  double y_re;
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
  //  Generated on 28-Aug-2019 12:13:20
  l4 = A_wayp * J_min;
  l5 = A_wayp * J_max;
  l15_tmp = J_max * J_max;
  l15 = 1.0 / (l15_tmp + -(J_min * J_max));
  l19_tmp = J_max * V_max * 2.0;
  l19.re = 1.0 / J_min * (J_min + -J_max) *
           ((A_wayp * A_wayp + l19_tmp) + -(J_max * V_wayp * 2.0));
  l19.im = 0.0;
  coder::internal::scalar::b_sqrt(&l19);
  l19.re *= J_min;
  l19.im *= J_min;
  t7[0].re = l15 * ((-l4 + l5) + l19.re);
  t7[0].im = l15 * l19.im;
  t7[1].re = -l15 * ((l4 - l5) + l19.re);
  t7[1].im = -l15 * l19.im;
  l3_tmp = A_init * A_init;
  l4_tmp = A_max * A_max;
  l6 = J_min * J_min;
  l5 = l4_tmp * l4_tmp;
  re = J_max * t7[0].re;
  im = J_max * t7[0].im;
  l2[0].re = re;
  l2[0].im = im;
  b_re = A_wayp + -re;
  l9[0].re = b_re;
  l9[0].im = -im;
  y[0].re = re * re - im * im;
  d = re * im;
  b_y[0].re = b_re * b_re - -im * -im;
  d1 = b_re * -im;
  b_y[0].im = d1 + d1;
  re = J_max * t7[1].re;
  im = J_max * t7[1].im;
  l2[1].re = re;
  l2[1].im = im;
  b_re = A_wayp + -re;
  l9[1].re = b_re;
  l9[1].im = -im;
  d1 = re * im;
  b_y[1].re = b_re * b_re - -im * -im;
  l4 = b_re * -im;
  b_y[1].im = l4 + l4;
  coder::power(l2, x);
  coder::power(l9, dcv);
  l19.re = (((l5 * l6 - l5 * l15_tmp) - l3_tmp * l3_tmp * l6 * 3.0) +
            rt_powd_snf(A_init, 3.0) * A_max * l6 * 8.0) -
           l3_tmp * l4_tmp * l6 * 6.0;
  b_A_max = A_max * l15_tmp;
  y_re = V_init * V_init * l6 * l15_tmp * 12.0;
  b_y_re = V_max * V_max * l6 * l15_tmp * 12.0;
  A_max_re = A_max * P_init * l6 * l15_tmp * 24.0;
  b_A_max_re = A_max * P_wayp * l6 * l15_tmp * 24.0;
  l15 = J_max * V_init;
  J_max_re = l15 * l3_tmp * l6 * 12.0;
  b_J_max_re = l15 * l4_tmp * l6 * 12.0;
  J_min_re = J_min * V_max * l4_tmp * l15_tmp * 12.0;
  c_A_max = A_max * J_max * V_max;
  l4 = A_max * J_min;
  d_A_max = l4 * V_max * l15_tmp;
  e_A_max = l4 * J_max;
  A_init_re = A_init * A_max * J_max * V_init * l6 * 24.0;
  l15_tmp *= A_max * V_max * l6;
  l4 = 1.0 / A_max * (1.0 / J_max) *
       ((((l3_tmp + l19_tmp) + -(l15 * 2.0)) + -l4_tmp) +
        J_max * l4_tmp * (1.0 / J_min)) /
       2.0;
  l15 = A_max * (1.0 / J_min);
  t[2].re = l4;
  t[2].im = 0.0;
  t[3].re = l4;
  t[3].im = 0.0;
  t[4].re = -l15;
  t[4].im = 0.0;
  t[5].re = -l15;
  t[5].im = 0.0;
  l3_tmp = l6 * (A_max * y[0].re);
  l4 = l6 * (A_max * (d + d));
  l15 = e_A_max * l2[0].re;
  l5 = e_A_max * l2[0].im;
  l4_tmp = -0.041666666666666664 *
           ((((((((((((((l19.re + l6 * (A_max * x[0].re) * 4.0) +
                        b_A_max * dcv[0].re * 4.0) -
                       y_re) +
                      b_y_re) +
                     (l3_tmp * l9[0].re - l4 * l9[0].im) * 12.0) +
                    A_max_re) -
                   b_A_max_re) +
                  J_max_re) +
                 b_J_max_re) -
                J_min_re) +
               l6 * (c_A_max * l2[0].re) * 24.0) +
              d_A_max * l9[0].re * 24.0) +
             (l15 * b_y[0].re - l5 * b_y[0].im) * 12.0) -
            A_init_re);
  l4 = -0.041666666666666664 *
       (((((l6 * (A_max * x[0].im) * 4.0 + b_A_max * dcv[0].im * 4.0) +
           (l3_tmp * l9[0].im + l4 * l9[0].re) * 12.0) +
          l6 * (c_A_max * l2[0].im) * 24.0) +
         d_A_max * l9[0].im * 24.0) +
        (l15 * b_y[0].im + l5 * b_y[0].re) * 12.0);
  if (l4 == 0.0) {
    t[6].re = l4_tmp / l15_tmp;
    t[6].im = 0.0;
  } else if (l4_tmp == 0.0) {
    t[6].re = 0.0;
    t[6].im = l4 / l15_tmp;
  } else {
    t[6].re = l4_tmp / l15_tmp;
    t[6].im = l4 / l15_tmp;
  }
  l4 = A_wayp - l2[0].re;
  if (0.0 - l2[0].im == 0.0) {
    t[8].re = l4 / J_min;
    t[8].im = 0.0;
  } else if (l4 == 0.0) {
    t[8].re = 0.0;
    t[8].im = (0.0 - l2[0].im) / J_min;
  } else {
    t[8].re = l4 / J_min;
    t[8].im = (0.0 - l2[0].im) / J_min;
  }
  t[10].re = 0.0;
  t[10].im = 0.0;
  t[12] = t7[0];
  l3_tmp = l6 * (A_max * (re * re - im * im));
  l4 = l6 * (A_max * (d1 + d1));
  l15 = e_A_max * re;
  l5 = e_A_max * im;
  l4_tmp = -0.041666666666666664 *
           ((((((((((((((l19.re + l6 * (A_max * x[1].re) * 4.0) +
                        b_A_max * dcv[1].re * 4.0) -
                       y_re) +
                      b_y_re) +
                     (l3_tmp * b_re - l4 * -im) * 12.0) +
                    A_max_re) -
                   b_A_max_re) +
                  J_max_re) +
                 b_J_max_re) -
                J_min_re) +
               l6 * (c_A_max * re) * 24.0) +
              d_A_max * b_re * 24.0) +
             (l15 * b_y[1].re - l5 * b_y[1].im) * 12.0) -
            A_init_re);
  l4 = -0.041666666666666664 *
       (((((l6 * (A_max * x[1].im) * 4.0 + b_A_max * dcv[1].im * 4.0) +
           (l3_tmp * -im + l4 * b_re) * 12.0) +
          l6 * (c_A_max * im) * 24.0) +
         d_A_max * -im * 24.0) +
        (l15 * b_y[1].im + l5 * b_y[1].re) * 12.0);
  if (l4 == 0.0) {
    t[7].re = l4_tmp / l15_tmp;
    t[7].im = 0.0;
  } else if (l4_tmp == 0.0) {
    t[7].re = 0.0;
    t[7].im = l4 / l15_tmp;
  } else {
    t[7].re = l4_tmp / l15_tmp;
    t[7].im = l4 / l15_tmp;
  }
  l4 = A_wayp - re;
  if (0.0 - im == 0.0) {
    t[9].re = l4 / J_min;
    t[9].im = 0.0;
  } else if (l4 == 0.0) {
    t[9].re = 0.0;
    t[9].im = (0.0 - im) / J_min;
  } else {
    t[9].re = l4 / J_min;
    t[9].im = (0.0 - im) / J_min;
  }
  t[11].re = 0.0;
  t[11].im = 0.0;
  t[13] = t7[1];
  l6 = -(1.0 / J_max * (A_init + -A_max));
  t[0].re = l6;
  t[0].im = 0.0;
  t[1].re = l6;
  t[1].im = 0.0;
}

// End of code generation (abcdeg_O_AVP.cpp)
