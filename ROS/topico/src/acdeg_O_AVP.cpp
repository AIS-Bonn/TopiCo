//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// acdeg_O_AVP.cpp
//
// Code generation for function 'acdeg_O_AVP'
//

// Include files
#include "acdeg_O_AVP.h"
#include "power.h"
#include "rt_nonfinite.h"
#include "sqrt.h"
#include "topico_wrapper_data.h"
#include "topico_wrapper_rtwutil.h"
#include <cmath>

// Function Definitions
void acdeg_O_AVP(double P_init, double V_init, double A_init, double P_wayp,
                 double V_wayp, double A_wayp, double V_max, double J_max,
                 double J_min, creal_T t[28])
{
  creal_T l11[4];
  creal_T l13[4];
  creal_T t3[4];
  creal_T t7[4];
  creal_T l19;
  double A_init_re;
  double A_wayp_re;
  double A_wayp_tmp;
  double J_max_re;
  double J_max_tmp;
  double P_wayp_re;
  double V_init_re;
  double V_init_tmp;
  double b_A_wayp;
  double b_A_wayp_re;
  double b_J_max;
  double b_J_max_tmp;
  double b_J_min;
  double b_l19_tmp;
  double b_l4_tmp;
  double b_l5_tmp;
  double b_y;
  double c_A_wayp;
  double c_J_min;
  double c_l19_tmp;
  double d_A_wayp;
  double d_J_min;
  double e_J_min;
  double l16;
  double l16_tmp;
  double l19_tmp;
  double l19_tmp_tmp;
  double l2;
  double l23_im_tmp;
  double l23_re;
  double l3;
  double l4_re;
  double l4_tmp;
  double l5;
  double l5_im_tmp;
  double l5_tmp;
  double l8;
  double y;
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
  //  Generated on 28-Aug-2019 12:21:18
  l4_tmp = A_wayp * J_min;
  l5_tmp = A_wayp * J_max;
  l8 = -(J_min * J_max);
  l16_tmp = J_max * J_max;
  l16 = 1.0 / (l16_tmp + l8);
  l19_tmp = J_max * V_max * 2.0;
  l5 = J_min * (J_min + -J_max);
  b_l19_tmp = A_wayp * A_wayp;
  l19.re = l5 * ((b_l19_tmp + l19_tmp) + -(J_max * V_wayp * 2.0));
  l19.im = 0.0;
  coder::internal::scalar::b_sqrt(&l19);
  l5_im_tmp = l19.im;
  l23_re = l16 * ((l5_tmp + -l4_tmp) + l19.re);
  l23_im_tmp = l16 * l19.im;
  l4_re = (l4_tmp + -l5_tmp) + l19.re;
  l19.re = -(l16 * l4_re);
  l19.im = -(l16 * l5_im_tmp);
  t7[0].re = l23_re;
  t7[0].im = l23_im_tmp;
  t7[1] = l19;
  t7[2].re = l23_re;
  t7[2].im = l23_im_tmp;
  t7[3] = l19;
  l5_im_tmp = A_init * A_init;
  c_l19_tmp = J_max * V_init;
  l19.re = l5 * ((l5_im_tmp + l19_tmp) + -(c_l19_tmp * 2.0));
  l19.im = 0.0;
  coder::internal::scalar::b_sqrt(&l19);
  l19_tmp_tmp = J_min * J_min;
  l19_tmp = 1.0 / (l19_tmp_tmp + l8);
  l19.re *= l19_tmp;
  l19.im *= l19_tmp;
  t3[0] = l19;
  t3[1] = l19;
  t3[2].re = -l19.re;
  t3[2].im = -l19.im;
  t3[3].re = -l19.re;
  t3[3].im = -l19.im;
  l5 = rt_powd_snf(J_min, 3.0);
  l8 = rt_powd_snf(J_max, 3.0);
  coder::b_power(t3, l11);
  coder::b_power(t7, l13);
  l16 = l16_tmp * l16_tmp;
  y = rt_powd_snf(J_min, 5.0);
  b_y = rt_powd_snf(J_max, 5.0);
  l19.re = rt_powd_snf(A_init, 3.0) * l19_tmp_tmp * 2.0 +
           rt_powd_snf(A_wayp, 3.0) * l16_tmp;
  b_A_wayp = A_wayp * l16;
  J_max_tmp = J_max * (l19_tmp_tmp * l19_tmp_tmp);
  b_J_min = J_min * l16;
  l23_re = P_init * l19_tmp_tmp * l16_tmp * 6.0;
  P_wayp_re = P_wayp * l19_tmp_tmp * l16_tmp * 6.0;
  b_l5_tmp = l5 * l16_tmp;
  b_l4_tmp = l19_tmp_tmp * l8;
  l2 = l5_im_tmp * l5;
  l3 = b_l19_tmp * l8;
  b_J_max_tmp = J_max * l5_im_tmp * l19_tmp_tmp;
  c_J_min = J_min * l5_im_tmp * l16_tmp;
  d_J_min = J_min * b_l19_tmp * l16_tmp;
  V_init_tmp = V_init * l19_tmp_tmp * l16_tmp;
  A_wayp_re = l4_tmp * J_max * l5_im_tmp * 3.0;
  A_init_re = A_init * J_max * V_init * l19_tmp_tmp * 6.0;
  b_A_wayp_re = l4_tmp * V_init * l16_tmp * 6.0;
  c_A_wayp = l5_tmp * l5;
  d_A_wayp = l4_tmp * l8;
  b_J_max = c_l19_tmp * l5;
  e_J_min = J_min * V_init * l8;
  A_wayp_tmp = A_wayp * l19_tmp_tmp * l16_tmp;
  J_max_re = b_J_max_tmp * 3.0;
  V_init_re = V_init_tmp * 6.0;
  l5 = t3[0].re * t3[0].re - t3[0].im * t3[0].im;
  l16 = t3[0].re * t3[0].im;
  l8 = l16 + l16;
  c_l19_tmp = t7[0].re * t7[0].re - l23_im_tmp * l23_im_tmp;
  l16 = t7[0].re * l23_im_tmp;
  l5_im_tmp = l16 + l16;
  l19_tmp = J_max_tmp * l5;
  l5_tmp = J_max_tmp * l8;
  b_l19_tmp = b_l5_tmp * l5;
  l19_tmp_tmp = b_l5_tmp * l8;
  l16_tmp = -(A_init + J_min * t3[0].re);
  l4_tmp = -(J_min * t3[0].im);
  if (l4_tmp == 0.0) {
    t[0].re = l16_tmp / J_max;
    t[0].im = 0.0;
  } else if (l16_tmp == 0.0) {
    t[0].re = 0.0;
    t[0].im = l4_tmp / J_max;
  } else {
    t[0].re = l16_tmp / J_max;
    t[0].im = l4_tmp / J_max;
  }
  t[4].re = 0.0;
  t[4].im = 0.0;
  t[8] = t3[0];
  l4_re = b_l4_tmp * l5;
  l16 = b_l4_tmp * l8;
  l16_tmp =
      ((((((((((((((((((((((((((((l19.re - y * l11[0].re) - b_y * l13[0].re) +
                                b_A_wayp * c_l19_tmp * 3.0) +
                               J_max_tmp * l11[0].re * 3.0) +
                              b_J_min * l13[0].re * 3.0) +
                             l23_re) -
                            P_wayp_re) -
                           b_l5_tmp * l11[0].re * 2.0) -
                          b_l4_tmp * l13[0].re * 2.0) +
                         l2 * t3[0].re * 3.0) -
                        l3 * t7[0].re * 3.0) -
                       b_J_max_tmp * t3[0].re * 3.0) -
                      b_J_max_tmp * t7[0].re * 3.0) +
                     c_J_min * t7[0].re * 3.0) +
                    d_J_min * t7[0].re * 3.0) +
                   (l19_tmp * t7[0].re - l5_tmp * l23_im_tmp) * 3.0) +
                  V_init_tmp * t3[0].re * 6.0) +
                 V_init_tmp * t7[0].re * 6.0) +
                (l4_re * t7[0].re - l16 * l23_im_tmp) * 3.0) -
               (b_l19_tmp * t7[0].re - l19_tmp_tmp * l23_im_tmp) * 6.0) -
              A_wayp_re) -
             A_init_re) +
            b_A_wayp_re) +
           c_A_wayp * l5 * 3.0) -
          d_A_wayp * c_l19_tmp * 6.0) -
         b_J_max * t3[0].re * 6.0) -
        e_J_min * t7[0].re * 6.0) -
       A_wayp_tmp * l5 * 3.0) +
      A_wayp_tmp * c_l19_tmp * 3.0;
  l4_tmp = (((((((((((((((((((((((0.0 - y * l11[0].im) - b_y * l13[0].im) +
                                b_A_wayp * l5_im_tmp * 3.0) +
                               J_max_tmp * l11[0].im * 3.0) +
                              b_J_min * l13[0].im * 3.0) -
                             b_l5_tmp * l11[0].im * 2.0) -
                            b_l4_tmp * l13[0].im * 2.0) +
                           l2 * t3[0].im * 3.0) -
                          l3 * l23_im_tmp * 3.0) -
                         b_J_max_tmp * t3[0].im * 3.0) -
                        b_J_max_tmp * l23_im_tmp * 3.0) +
                       c_J_min * l23_im_tmp * 3.0) +
                      d_J_min * l23_im_tmp * 3.0) +
                     (l19_tmp * l23_im_tmp + l5_tmp * t7[0].re) * 3.0) +
                    V_init_tmp * t3[0].im * 6.0) +
                   V_init_tmp * l23_im_tmp * 6.0) +
                  (l4_re * l23_im_tmp + l16 * t7[0].re) * 3.0) -
                 (b_l19_tmp * l23_im_tmp + l19_tmp_tmp * t7[0].re) * 6.0) +
                c_A_wayp * l8 * 3.0) -
               d_A_wayp * l5_im_tmp * 6.0) -
              b_J_max * t3[0].im * 6.0) -
             e_J_min * l23_im_tmp * 6.0) -
            A_wayp_tmp * l8 * 3.0) +
           A_wayp_tmp * l5_im_tmp * 3.0;
  l5 = ((J_max_re - l19_tmp * 3.0) - V_init_re) + b_l19_tmp * 3.0;
  l19_tmp = (0.0 - l5_tmp * 3.0) + l19_tmp_tmp * 3.0;
  if (l19_tmp == 0.0) {
    if (l4_tmp == 0.0) {
      t[12].re = l16_tmp / l5;
      t[12].im = 0.0;
    } else if (l16_tmp == 0.0) {
      t[12].re = 0.0;
      t[12].im = l4_tmp / l5;
    } else {
      t[12].re = l16_tmp / l5;
      t[12].im = l4_tmp / l5;
    }
  } else if (l5 == 0.0) {
    if (l16_tmp == 0.0) {
      t[12].re = l4_tmp / l19_tmp;
      t[12].im = 0.0;
    } else if (l4_tmp == 0.0) {
      t[12].re = 0.0;
      t[12].im = -(l16_tmp / l19_tmp);
    } else {
      t[12].re = l4_tmp / l19_tmp;
      t[12].im = -(l16_tmp / l19_tmp);
    }
  } else {
    b_l19_tmp = std::abs(l5);
    l16 = std::abs(l19_tmp);
    if (b_l19_tmp > l16) {
      l5_im_tmp = l19_tmp / l5;
      l16 = l5 + l5_im_tmp * l19_tmp;
      t[12].re = (l16_tmp + l5_im_tmp * l4_tmp) / l16;
      t[12].im = (l4_tmp - l5_im_tmp * l16_tmp) / l16;
    } else if (l16 == b_l19_tmp) {
      if (l5 > 0.0) {
        l5_im_tmp = 0.5;
      } else {
        l5_im_tmp = -0.5;
      }
      if (l19_tmp > 0.0) {
        l16 = 0.5;
      } else {
        l16 = -0.5;
      }
      t[12].re = (l16_tmp * l5_im_tmp + l4_tmp * l16) / b_l19_tmp;
      t[12].im = (l4_tmp * l5_im_tmp - l16_tmp * l16) / b_l19_tmp;
    } else {
      l5_im_tmp = l5 / l19_tmp;
      l16 = l19_tmp + l5_im_tmp * l5;
      t[12].re = (l5_im_tmp * l16_tmp + l4_tmp) / l16;
      t[12].im = (l5_im_tmp * l4_tmp - l16_tmp) / l16;
    }
  }
  l16_tmp = A_wayp - J_max * t7[0].re;
  l4_tmp = 0.0 - J_max * l23_im_tmp;
  if (l4_tmp == 0.0) {
    t[16].re = l16_tmp / J_min;
    t[16].im = 0.0;
  } else if (l16_tmp == 0.0) {
    t[16].re = 0.0;
    t[16].im = l4_tmp / J_min;
  } else {
    t[16].re = l16_tmp / J_min;
    t[16].im = l4_tmp / J_min;
  }
  t[20].re = 0.0;
  t[20].im = 0.0;
  t[24] = t7[0];
  l5 = t3[1].re * t3[1].re - t3[1].im * t3[1].im;
  l16 = t3[1].re * t3[1].im;
  l8 = l16 + l16;
  c_l19_tmp = t7[1].re * t7[1].re - t7[1].im * t7[1].im;
  l16 = t7[1].re * t7[1].im;
  l5_im_tmp = l16 + l16;
  l19_tmp = J_max_tmp * l5;
  l5_tmp = J_max_tmp * l8;
  b_l19_tmp = b_l5_tmp * l5;
  l19_tmp_tmp = b_l5_tmp * l8;
  l16_tmp = -(A_init + J_min * t3[1].re);
  l4_tmp = -(J_min * t3[1].im);
  if (l4_tmp == 0.0) {
    t[1].re = l16_tmp / J_max;
    t[1].im = 0.0;
  } else if (l16_tmp == 0.0) {
    t[1].re = 0.0;
    t[1].im = l4_tmp / J_max;
  } else {
    t[1].re = l16_tmp / J_max;
    t[1].im = l4_tmp / J_max;
  }
  t[5].re = 0.0;
  t[5].im = 0.0;
  t[9] = t3[1];
  l4_re = b_l4_tmp * l5;
  l16 = b_l4_tmp * l8;
  l16_tmp =
      ((((((((((((((((((((((((((((l19.re - y * l11[1].re) - b_y * l13[1].re) +
                                b_A_wayp * c_l19_tmp * 3.0) +
                               J_max_tmp * l11[1].re * 3.0) +
                              b_J_min * l13[1].re * 3.0) +
                             l23_re) -
                            P_wayp_re) -
                           b_l5_tmp * l11[1].re * 2.0) -
                          b_l4_tmp * l13[1].re * 2.0) +
                         l2 * t3[1].re * 3.0) -
                        l3 * t7[1].re * 3.0) -
                       b_J_max_tmp * t3[1].re * 3.0) -
                      b_J_max_tmp * t7[1].re * 3.0) +
                     c_J_min * t7[1].re * 3.0) +
                    d_J_min * t7[1].re * 3.0) +
                   (l19_tmp * t7[1].re - l5_tmp * t7[1].im) * 3.0) +
                  V_init_tmp * t3[1].re * 6.0) +
                 V_init_tmp * t7[1].re * 6.0) +
                (l4_re * t7[1].re - l16 * t7[1].im) * 3.0) -
               (b_l19_tmp * t7[1].re - l19_tmp_tmp * t7[1].im) * 6.0) -
              A_wayp_re) -
             A_init_re) +
            b_A_wayp_re) +
           c_A_wayp * l5 * 3.0) -
          d_A_wayp * c_l19_tmp * 6.0) -
         b_J_max * t3[1].re * 6.0) -
        e_J_min * t7[1].re * 6.0) -
       A_wayp_tmp * l5 * 3.0) +
      A_wayp_tmp * c_l19_tmp * 3.0;
  l4_tmp = (((((((((((((((((((((((0.0 - y * l11[1].im) - b_y * l13[1].im) +
                                b_A_wayp * l5_im_tmp * 3.0) +
                               J_max_tmp * l11[1].im * 3.0) +
                              b_J_min * l13[1].im * 3.0) -
                             b_l5_tmp * l11[1].im * 2.0) -
                            b_l4_tmp * l13[1].im * 2.0) +
                           l2 * t3[1].im * 3.0) -
                          l3 * t7[1].im * 3.0) -
                         b_J_max_tmp * t3[1].im * 3.0) -
                        b_J_max_tmp * t7[1].im * 3.0) +
                       c_J_min * t7[1].im * 3.0) +
                      d_J_min * t7[1].im * 3.0) +
                     (l19_tmp * t7[1].im + l5_tmp * t7[1].re) * 3.0) +
                    V_init_tmp * t3[1].im * 6.0) +
                   V_init_tmp * t7[1].im * 6.0) +
                  (l4_re * t7[1].im + l16 * t7[1].re) * 3.0) -
                 (b_l19_tmp * t7[1].im + l19_tmp_tmp * t7[1].re) * 6.0) +
                c_A_wayp * l8 * 3.0) -
               d_A_wayp * l5_im_tmp * 6.0) -
              b_J_max * t3[1].im * 6.0) -
             e_J_min * t7[1].im * 6.0) -
            A_wayp_tmp * l8 * 3.0) +
           A_wayp_tmp * l5_im_tmp * 3.0;
  l5 = ((J_max_re - l19_tmp * 3.0) - V_init_re) + b_l19_tmp * 3.0;
  l19_tmp = (0.0 - l5_tmp * 3.0) + l19_tmp_tmp * 3.0;
  if (l19_tmp == 0.0) {
    if (l4_tmp == 0.0) {
      t[13].re = l16_tmp / l5;
      t[13].im = 0.0;
    } else if (l16_tmp == 0.0) {
      t[13].re = 0.0;
      t[13].im = l4_tmp / l5;
    } else {
      t[13].re = l16_tmp / l5;
      t[13].im = l4_tmp / l5;
    }
  } else if (l5 == 0.0) {
    if (l16_tmp == 0.0) {
      t[13].re = l4_tmp / l19_tmp;
      t[13].im = 0.0;
    } else if (l4_tmp == 0.0) {
      t[13].re = 0.0;
      t[13].im = -(l16_tmp / l19_tmp);
    } else {
      t[13].re = l4_tmp / l19_tmp;
      t[13].im = -(l16_tmp / l19_tmp);
    }
  } else {
    b_l19_tmp = std::abs(l5);
    l16 = std::abs(l19_tmp);
    if (b_l19_tmp > l16) {
      l5_im_tmp = l19_tmp / l5;
      l16 = l5 + l5_im_tmp * l19_tmp;
      t[13].re = (l16_tmp + l5_im_tmp * l4_tmp) / l16;
      t[13].im = (l4_tmp - l5_im_tmp * l16_tmp) / l16;
    } else if (l16 == b_l19_tmp) {
      if (l5 > 0.0) {
        l5_im_tmp = 0.5;
      } else {
        l5_im_tmp = -0.5;
      }
      if (l19_tmp > 0.0) {
        l16 = 0.5;
      } else {
        l16 = -0.5;
      }
      t[13].re = (l16_tmp * l5_im_tmp + l4_tmp * l16) / b_l19_tmp;
      t[13].im = (l4_tmp * l5_im_tmp - l16_tmp * l16) / b_l19_tmp;
    } else {
      l5_im_tmp = l5 / l19_tmp;
      l16 = l19_tmp + l5_im_tmp * l5;
      t[13].re = (l5_im_tmp * l16_tmp + l4_tmp) / l16;
      t[13].im = (l5_im_tmp * l4_tmp - l16_tmp) / l16;
    }
  }
  l16_tmp = A_wayp - J_max * t7[1].re;
  l4_tmp = 0.0 - J_max * t7[1].im;
  if (l4_tmp == 0.0) {
    t[17].re = l16_tmp / J_min;
    t[17].im = 0.0;
  } else if (l16_tmp == 0.0) {
    t[17].re = 0.0;
    t[17].im = l4_tmp / J_min;
  } else {
    t[17].re = l16_tmp / J_min;
    t[17].im = l4_tmp / J_min;
  }
  t[21].re = 0.0;
  t[21].im = 0.0;
  t[25] = t7[1];
  l5 = t3[2].re * t3[2].re - t3[2].im * t3[2].im;
  l16 = t3[2].re * t3[2].im;
  l8 = l16 + l16;
  c_l19_tmp = t7[2].re * t7[2].re - l23_im_tmp * l23_im_tmp;
  l16 = t7[2].re * l23_im_tmp;
  l5_im_tmp = l16 + l16;
  l19_tmp = J_max_tmp * l5;
  l5_tmp = J_max_tmp * l8;
  b_l19_tmp = b_l5_tmp * l5;
  l19_tmp_tmp = b_l5_tmp * l8;
  l16_tmp = -(A_init + J_min * t3[2].re);
  l4_tmp = -(J_min * t3[2].im);
  if (l4_tmp == 0.0) {
    t[2].re = l16_tmp / J_max;
    t[2].im = 0.0;
  } else if (l16_tmp == 0.0) {
    t[2].re = 0.0;
    t[2].im = l4_tmp / J_max;
  } else {
    t[2].re = l16_tmp / J_max;
    t[2].im = l4_tmp / J_max;
  }
  t[6].re = 0.0;
  t[6].im = 0.0;
  t[10] = t3[2];
  l4_re = b_l4_tmp * l5;
  l16 = b_l4_tmp * l8;
  l16_tmp =
      ((((((((((((((((((((((((((((l19.re - y * l11[2].re) - b_y * l13[2].re) +
                                b_A_wayp * c_l19_tmp * 3.0) +
                               J_max_tmp * l11[2].re * 3.0) +
                              b_J_min * l13[2].re * 3.0) +
                             l23_re) -
                            P_wayp_re) -
                           b_l5_tmp * l11[2].re * 2.0) -
                          b_l4_tmp * l13[2].re * 2.0) +
                         l2 * t3[2].re * 3.0) -
                        l3 * t7[2].re * 3.0) -
                       b_J_max_tmp * t3[2].re * 3.0) -
                      b_J_max_tmp * t7[2].re * 3.0) +
                     c_J_min * t7[2].re * 3.0) +
                    d_J_min * t7[2].re * 3.0) +
                   (l19_tmp * t7[2].re - l5_tmp * l23_im_tmp) * 3.0) +
                  V_init_tmp * t3[2].re * 6.0) +
                 V_init_tmp * t7[2].re * 6.0) +
                (l4_re * t7[2].re - l16 * l23_im_tmp) * 3.0) -
               (b_l19_tmp * t7[2].re - l19_tmp_tmp * l23_im_tmp) * 6.0) -
              A_wayp_re) -
             A_init_re) +
            b_A_wayp_re) +
           c_A_wayp * l5 * 3.0) -
          d_A_wayp * c_l19_tmp * 6.0) -
         b_J_max * t3[2].re * 6.0) -
        e_J_min * t7[2].re * 6.0) -
       A_wayp_tmp * l5 * 3.0) +
      A_wayp_tmp * c_l19_tmp * 3.0;
  l4_tmp = (((((((((((((((((((((((0.0 - y * l11[2].im) - b_y * l13[2].im) +
                                b_A_wayp * l5_im_tmp * 3.0) +
                               J_max_tmp * l11[2].im * 3.0) +
                              b_J_min * l13[2].im * 3.0) -
                             b_l5_tmp * l11[2].im * 2.0) -
                            b_l4_tmp * l13[2].im * 2.0) +
                           l2 * t3[2].im * 3.0) -
                          l3 * l23_im_tmp * 3.0) -
                         b_J_max_tmp * t3[2].im * 3.0) -
                        b_J_max_tmp * l23_im_tmp * 3.0) +
                       c_J_min * l23_im_tmp * 3.0) +
                      d_J_min * l23_im_tmp * 3.0) +
                     (l19_tmp * l23_im_tmp + l5_tmp * t7[2].re) * 3.0) +
                    V_init_tmp * t3[2].im * 6.0) +
                   V_init_tmp * l23_im_tmp * 6.0) +
                  (l4_re * l23_im_tmp + l16 * t7[2].re) * 3.0) -
                 (b_l19_tmp * l23_im_tmp + l19_tmp_tmp * t7[2].re) * 6.0) +
                c_A_wayp * l8 * 3.0) -
               d_A_wayp * l5_im_tmp * 6.0) -
              b_J_max * t3[2].im * 6.0) -
             e_J_min * l23_im_tmp * 6.0) -
            A_wayp_tmp * l8 * 3.0) +
           A_wayp_tmp * l5_im_tmp * 3.0;
  l5 = ((J_max_re - l19_tmp * 3.0) - V_init_re) + b_l19_tmp * 3.0;
  l19_tmp = (0.0 - l5_tmp * 3.0) + l19_tmp_tmp * 3.0;
  if (l19_tmp == 0.0) {
    if (l4_tmp == 0.0) {
      t[14].re = l16_tmp / l5;
      t[14].im = 0.0;
    } else if (l16_tmp == 0.0) {
      t[14].re = 0.0;
      t[14].im = l4_tmp / l5;
    } else {
      t[14].re = l16_tmp / l5;
      t[14].im = l4_tmp / l5;
    }
  } else if (l5 == 0.0) {
    if (l16_tmp == 0.0) {
      t[14].re = l4_tmp / l19_tmp;
      t[14].im = 0.0;
    } else if (l4_tmp == 0.0) {
      t[14].re = 0.0;
      t[14].im = -(l16_tmp / l19_tmp);
    } else {
      t[14].re = l4_tmp / l19_tmp;
      t[14].im = -(l16_tmp / l19_tmp);
    }
  } else {
    b_l19_tmp = std::abs(l5);
    l16 = std::abs(l19_tmp);
    if (b_l19_tmp > l16) {
      l5_im_tmp = l19_tmp / l5;
      l16 = l5 + l5_im_tmp * l19_tmp;
      t[14].re = (l16_tmp + l5_im_tmp * l4_tmp) / l16;
      t[14].im = (l4_tmp - l5_im_tmp * l16_tmp) / l16;
    } else if (l16 == b_l19_tmp) {
      if (l5 > 0.0) {
        l5_im_tmp = 0.5;
      } else {
        l5_im_tmp = -0.5;
      }
      if (l19_tmp > 0.0) {
        l16 = 0.5;
      } else {
        l16 = -0.5;
      }
      t[14].re = (l16_tmp * l5_im_tmp + l4_tmp * l16) / b_l19_tmp;
      t[14].im = (l4_tmp * l5_im_tmp - l16_tmp * l16) / b_l19_tmp;
    } else {
      l5_im_tmp = l5 / l19_tmp;
      l16 = l19_tmp + l5_im_tmp * l5;
      t[14].re = (l5_im_tmp * l16_tmp + l4_tmp) / l16;
      t[14].im = (l5_im_tmp * l4_tmp - l16_tmp) / l16;
    }
  }
  l16_tmp = A_wayp - J_max * t7[2].re;
  l4_tmp = 0.0 - J_max * l23_im_tmp;
  if (l4_tmp == 0.0) {
    t[18].re = l16_tmp / J_min;
    t[18].im = 0.0;
  } else if (l16_tmp == 0.0) {
    t[18].re = 0.0;
    t[18].im = l4_tmp / J_min;
  } else {
    t[18].re = l16_tmp / J_min;
    t[18].im = l4_tmp / J_min;
  }
  t[22].re = 0.0;
  t[22].im = 0.0;
  t[26] = t7[2];
  l5 = t3[3].re * t3[3].re - t3[3].im * t3[3].im;
  l16 = t3[3].re * t3[3].im;
  l8 = l16 + l16;
  c_l19_tmp = t7[3].re * t7[3].re - t7[3].im * t7[3].im;
  l16 = t7[3].re * t7[3].im;
  l5_im_tmp = l16 + l16;
  l19_tmp = J_max_tmp * l5;
  l5_tmp = J_max_tmp * l8;
  b_l19_tmp = b_l5_tmp * l5;
  l19_tmp_tmp = b_l5_tmp * l8;
  l16_tmp = -(A_init + J_min * t3[3].re);
  l4_tmp = -(J_min * t3[3].im);
  if (l4_tmp == 0.0) {
    t[3].re = l16_tmp / J_max;
    t[3].im = 0.0;
  } else if (l16_tmp == 0.0) {
    t[3].re = 0.0;
    t[3].im = l4_tmp / J_max;
  } else {
    t[3].re = l16_tmp / J_max;
    t[3].im = l4_tmp / J_max;
  }
  t[7].re = 0.0;
  t[7].im = 0.0;
  t[11] = t3[3];
  l4_re = b_l4_tmp * l5;
  l16 = b_l4_tmp * l8;
  l16_tmp =
      ((((((((((((((((((((((((((((l19.re - y * l11[3].re) - b_y * l13[3].re) +
                                b_A_wayp * c_l19_tmp * 3.0) +
                               J_max_tmp * l11[3].re * 3.0) +
                              b_J_min * l13[3].re * 3.0) +
                             l23_re) -
                            P_wayp_re) -
                           b_l5_tmp * l11[3].re * 2.0) -
                          b_l4_tmp * l13[3].re * 2.0) +
                         l2 * t3[3].re * 3.0) -
                        l3 * t7[3].re * 3.0) -
                       b_J_max_tmp * t3[3].re * 3.0) -
                      b_J_max_tmp * t7[3].re * 3.0) +
                     c_J_min * t7[3].re * 3.0) +
                    d_J_min * t7[3].re * 3.0) +
                   (l19_tmp * t7[3].re - l5_tmp * t7[3].im) * 3.0) +
                  V_init_tmp * t3[3].re * 6.0) +
                 V_init_tmp * t7[3].re * 6.0) +
                (l4_re * t7[3].re - l16 * t7[3].im) * 3.0) -
               (b_l19_tmp * t7[3].re - l19_tmp_tmp * t7[3].im) * 6.0) -
              A_wayp_re) -
             A_init_re) +
            b_A_wayp_re) +
           c_A_wayp * l5 * 3.0) -
          d_A_wayp * c_l19_tmp * 6.0) -
         b_J_max * t3[3].re * 6.0) -
        e_J_min * t7[3].re * 6.0) -
       A_wayp_tmp * l5 * 3.0) +
      A_wayp_tmp * c_l19_tmp * 3.0;
  l4_tmp = (((((((((((((((((((((((0.0 - y * l11[3].im) - b_y * l13[3].im) +
                                b_A_wayp * l5_im_tmp * 3.0) +
                               J_max_tmp * l11[3].im * 3.0) +
                              b_J_min * l13[3].im * 3.0) -
                             b_l5_tmp * l11[3].im * 2.0) -
                            b_l4_tmp * l13[3].im * 2.0) +
                           l2 * t3[3].im * 3.0) -
                          l3 * t7[3].im * 3.0) -
                         b_J_max_tmp * t3[3].im * 3.0) -
                        b_J_max_tmp * t7[3].im * 3.0) +
                       c_J_min * t7[3].im * 3.0) +
                      d_J_min * t7[3].im * 3.0) +
                     (l19_tmp * t7[3].im + l5_tmp * t7[3].re) * 3.0) +
                    V_init_tmp * t3[3].im * 6.0) +
                   V_init_tmp * t7[3].im * 6.0) +
                  (l4_re * t7[3].im + l16 * t7[3].re) * 3.0) -
                 (b_l19_tmp * t7[3].im + l19_tmp_tmp * t7[3].re) * 6.0) +
                c_A_wayp * l8 * 3.0) -
               d_A_wayp * l5_im_tmp * 6.0) -
              b_J_max * t3[3].im * 6.0) -
             e_J_min * t7[3].im * 6.0) -
            A_wayp_tmp * l8 * 3.0) +
           A_wayp_tmp * l5_im_tmp * 3.0;
  l5 = ((J_max_re - l19_tmp * 3.0) - V_init_re) + b_l19_tmp * 3.0;
  l19_tmp = (0.0 - l5_tmp * 3.0) + l19_tmp_tmp * 3.0;
  if (l19_tmp == 0.0) {
    if (l4_tmp == 0.0) {
      t[15].re = l16_tmp / l5;
      t[15].im = 0.0;
    } else if (l16_tmp == 0.0) {
      t[15].re = 0.0;
      t[15].im = l4_tmp / l5;
    } else {
      t[15].re = l16_tmp / l5;
      t[15].im = l4_tmp / l5;
    }
  } else if (l5 == 0.0) {
    if (l16_tmp == 0.0) {
      t[15].re = l4_tmp / l19_tmp;
      t[15].im = 0.0;
    } else if (l4_tmp == 0.0) {
      t[15].re = 0.0;
      t[15].im = -(l16_tmp / l19_tmp);
    } else {
      t[15].re = l4_tmp / l19_tmp;
      t[15].im = -(l16_tmp / l19_tmp);
    }
  } else {
    b_l19_tmp = std::abs(l5);
    l16 = std::abs(l19_tmp);
    if (b_l19_tmp > l16) {
      l5_im_tmp = l19_tmp / l5;
      l16 = l5 + l5_im_tmp * l19_tmp;
      t[15].re = (l16_tmp + l5_im_tmp * l4_tmp) / l16;
      t[15].im = (l4_tmp - l5_im_tmp * l16_tmp) / l16;
    } else if (l16 == b_l19_tmp) {
      if (l5 > 0.0) {
        l5_im_tmp = 0.5;
      } else {
        l5_im_tmp = -0.5;
      }
      if (l19_tmp > 0.0) {
        l16 = 0.5;
      } else {
        l16 = -0.5;
      }
      t[15].re = (l16_tmp * l5_im_tmp + l4_tmp * l16) / b_l19_tmp;
      t[15].im = (l4_tmp * l5_im_tmp - l16_tmp * l16) / b_l19_tmp;
    } else {
      l5_im_tmp = l5 / l19_tmp;
      l16 = l19_tmp + l5_im_tmp * l5;
      t[15].re = (l5_im_tmp * l16_tmp + l4_tmp) / l16;
      t[15].im = (l5_im_tmp * l4_tmp - l16_tmp) / l16;
    }
  }
  l16_tmp = A_wayp - J_max * t7[3].re;
  l4_tmp = 0.0 - J_max * t7[3].im;
  if (l4_tmp == 0.0) {
    t[19].re = l16_tmp / J_min;
    t[19].im = 0.0;
  } else if (l16_tmp == 0.0) {
    t[19].re = 0.0;
    t[19].im = l4_tmp / J_min;
  } else {
    t[19].re = l16_tmp / J_min;
    t[19].im = l4_tmp / J_min;
  }
  t[23].re = 0.0;
  t[23].im = 0.0;
  t[27] = t7[3];
}

// End of code generation (acdeg_O_AVP.cpp)
