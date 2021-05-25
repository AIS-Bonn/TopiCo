//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// acefg_O_AP.cpp
//
// Code generation for function 'acefg_O_AP'
//

// Include files
#include "acefg_O_AP.h"
#include "power.h"
#include "rt_nonfinite.h"
#include "sqrt.h"
#include "topico_wrapper_data.h"
#include "topico_wrapper_rtwutil.h"
#include <cmath>

// Function Definitions
void acefg_O_AP(double P_init, double V_init, double A_init, double P_wayp,
                double A_wayp, double V_max, double A_min, double J_max,
                double J_min, creal_T t[28])
{
  creal_T b_l2[4];
  creal_T t1[4];
  creal_T y[4];
  creal_T dc;
  creal_T l153;
  creal_T l158;
  creal_T l159;
  double a_tmp;
  double b_l115_tmp;
  double b_l133_tmp;
  double l10;
  double l100;
  double l102;
  double l103;
  double l106;
  double l11;
  double l115;
  double l115_tmp;
  double l12;
  double l127;
  double l13;
  double l133;
  double l133_tmp;
  double l137;
  double l14;
  double l165_im;
  double l175_im;
  double l2;
  double l3;
  double l33_tmp;
  double l5;
  double l6;
  double l7;
  double l8;
  double l9;
  double l98;
  double l99;
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
  l2 = A_init * A_init;
  l3 = rt_powd_snf(A_init, 3.0);
  l5 = A_min * A_min;
  l7 = A_wayp * A_wayp;
  l8 = rt_powd_snf(A_wayp, 3.0);
  l10 = J_min * J_min;
  l11 = J_max * J_max;
  l12 = rt_powd_snf(J_max, 3.0);
  l14 = rt_powd_snf(J_max, 5.0);
  l6 = l5 * l5;
  l9 = l7 * l7;
  l13 = l11 * l11;
  l165_im = A_init * A_min;
  l137 = l165_im * J_min;
  l33_tmp = J_min * V_init;
  l115_tmp = A_min * J_min;
  b_l115_tmp = A_init * J_min;
  l115 = ((((((((l3 * l12 * 12.0 + l115_tmp * V_init * l12 * 24.0) +
                -(b_l115_tmp * V_init * l12 * 24.0)) +
               A_init * l5 * l12 * 12.0) +
              l115_tmp * l2 * l11 * 24.0) +
             A_init * V_init * l10 * l11 * 24.0) +
            -(A_min * l2 * l12 * 24.0)) +
           -(J_min * l3 * l11 * 12.0)) +
          -(b_l115_tmp * l5 * l11 * 12.0)) +
         -(A_min * V_init * l10 * l11 * 24.0);
  l98 = 1.0 / ((rt_powd_snf(l11, 3.0) * 3.0 + -(J_min * l14 * 6.0)) +
               l10 * l13 * 3.0);
  l102 =
      ((((A_init * l14 * 12.0 + -(A_min * l14 * 8.0)) + l115_tmp * l13 * 12.0) +
        -(b_l115_tmp * l13 * 24.0)) +
       A_init * l10 * l12 * 12.0) +
      -(A_min * l10 * l12 * 4.0);
  l99 = l98 * l98;
  l100 = rt_powd_snf(l98, 3.0);
  l103 = l102 * l102;
  l106 = l98 * l102 / 4.0;
  l14 = J_min * J_max;
  l133_tmp = J_max * V_max;
  l175_im = l10 * l11;
  b_l133_tmp = J_min * V_max;
  l133 = l98 *
         ((((((((((((((((((((((l6 * l11 + -(l14 * l9 * 6.0)) + l9 * l10 * 3.0) +
                             l9 * l11 * 3.0) +
                            -(l2 * l2 * l11 * 3.0)) +
                           -(l6 * l10)) +
                          l115_tmp * J_max * l8 * 12.0) +
                         A_min * A_wayp * J_max * V_max * l10 * 24.0) +
                        A_min * l3 * l11 * 8.0) +
                       -(l137 * V_init * l11 * 24.0)) +
                      -(l14 * l5 * l7 * 6.0)) +
                     A_min * P_init * l10 * l11 * 24.0) +
                    l33_tmp * l2 * l11 * 12.0) +
                   l33_tmp * l5 * l11 * 12.0) +
                  b_l133_tmp * l7 * l11 * 12.0) +
                 l5 * l7 * l10 * 6.0) +
                -(A_min * l8 * l10 * 8.0)) +
               -(A_min * P_wayp * l10 * l11 * 24.0)) +
              -(l133_tmp * l5 * l10 * 12.0)) +
             -(l133_tmp * l7 * l10 * 12.0)) +
            -(l2 * l5 * l11 * 6.0)) +
           l175_im * (V_max * V_max) * 12.0) +
          -(l175_im * (V_init * V_init) * 12.0));
  l175_im =
      ((((((((l165_im * l13 * 24.0 + l33_tmp * l13 * 12.0) - l5 * l13 * 6.0) -
            l2 * l13 * 18.0) -
           l137 * l12 * 36.0) +
          J_min * l5 * l12 * 6.0) -
         V_init * l10 * l12 * 12.0) +
        J_min * l2 * l12 * 30.0) +
       l165_im * l10 * l11 * 12.0) -
      l2 * l10 * l11 * 12.0;
  a_tmp = l99 * l103 * 0.375 + l98 * l175_im;
  l127 = a_tmp * a_tmp;
  l7 = l99 * l102;
  l102 =
      (l100 * rt_powd_snf(l102, 3.0) / 8.0 + l98 * l115) + l7 * l175_im / 2.0;
  l137 = l102 * l102;
  l98 = l99 * l99 * (l103 * l103);
  l12 = l100 * l103 * l175_im;
  l11 = l7 * l115;
  l13 = ((l98 * 0.01171875 + l12 / 16.0) + l11 / 4.0) + l133;
  l3 = rt_powd_snf(a_tmp, 3.0);
  l153.re = ((((l137 * l137 * 27.0 + l137 * l3 * -4.0) +
               rt_powd_snf(l13, 3.0) * 256.0) +
              l127 * l127 * l13 * 16.0) +
             l127 * (l13 * l13) * 128.0) +
            l137 * l13 * a_tmp * -144.0;
  l153.im = 0.0;
  coder::internal::scalar::b_sqrt(&l153);
  l7 = 1.7320508075688772 * l153.re;
  l9 = 1.7320508075688772 * l153.im;
  if (l9 == 0.0) {
    l10 = l7 / 18.0;
    l8 = 0.0;
  } else if (l7 == 0.0) {
    l10 = 0.0;
    l8 = l9 / 18.0;
  } else {
    l10 = l7 / 18.0;
    l8 = l9 / 18.0;
  }
  l2 = l13 * a_tmp;
  l158.re =
      ((l3 * -0.037037037037037035 + l137 / 2.0) + l2 * -1.3333333333333333) +
      l10;
  l158.im = l8;
  l159 = coder::power(l158);
  dc = coder::b_power(l158);
  if (dc.im == 0.0) {
    l103 = 1.0 / dc.re;
    l6 = 0.0;
  } else if (dc.re == 0.0) {
    l103 = 0.0;
    l6 = -(1.0 / dc.im);
  } else {
    l133_tmp = std::abs(dc.re);
    l14 = std::abs(dc.im);
    if (l133_tmp > l14) {
      l14 = dc.im / dc.re;
      l175_im = dc.re + l14 * dc.im;
      l103 = (l14 * 0.0 + 1.0) / l175_im;
      l6 = (0.0 - l14) / l175_im;
    } else if (l14 == l133_tmp) {
      if (dc.re > 0.0) {
        l14 = 0.5;
      } else {
        l14 = -0.5;
      }
      if (dc.im > 0.0) {
        l175_im = 0.5;
      } else {
        l175_im = -0.5;
      }
      l103 = (l14 + 0.0 * l175_im) / l133_tmp;
      l6 = (0.0 * l14 - l175_im) / l133_tmp;
    } else {
      l14 = dc.re / dc.im;
      l175_im = dc.im + l14 * dc.re;
      l103 = l14 / l175_im;
      l6 = (l14 * 0.0 - 1.0) / l175_im;
    }
  }
  l100 = l159.re * l159.re - l159.im * l159.im;
  l14 = l159.re * l159.im;
  l99 = l14 + l14;
  dc.re = ((l3 * -2.0 + l137 * 27.0) + l2 * -72.0) + l7 * 3.0;
  dc.im = l9 * 3.0;
  coder::internal::scalar::b_sqrt(&dc);
  l5 = 3.0 * (2.4494897427831779 * l102 * dc.re);
  l165_im = 3.0 * (2.4494897427831779 * l102 * dc.im);
  l158.re = (((((-(l98 * 0.140625) + l12 * -0.75) + -(l11 * 3.0)) + l127) +
              -(l133 * 12.0)) +
             l100 * 9.0) +
            a_tmp * l159.re * 6.0;
  l158.im = l99 * 9.0 + a_tmp * l159.im * 6.0;
  l153 = l158;
  coder::internal::scalar::b_sqrt(&l153);
  dc = coder::c_power(l158);
  if (dc.im == 0.0) {
    l11 = 1.0 / dc.re;
    l3 = 0.0;
  } else if (dc.re == 0.0) {
    l11 = 0.0;
    l3 = -(1.0 / dc.im);
  } else {
    l133_tmp = std::abs(dc.re);
    l14 = std::abs(dc.im);
    if (l133_tmp > l14) {
      l14 = dc.im / dc.re;
      l175_im = dc.re + l14 * dc.im;
      l11 = (l14 * 0.0 + 1.0) / l175_im;
      l3 = (0.0 - l14) / l175_im;
    } else if (l14 == l133_tmp) {
      if (dc.re > 0.0) {
        l14 = 0.5;
      } else {
        l14 = -0.5;
      }
      if (dc.im > 0.0) {
        l175_im = 0.5;
      } else {
        l175_im = -0.5;
      }
      l11 = (l14 + 0.0 * l175_im) / l133_tmp;
      l3 = (0.0 * l14 - l175_im) / l133_tmp;
    } else {
      l14 = dc.re / dc.im;
      l175_im = dc.im + l14 * dc.re;
      l11 = l14 / l175_im;
      l3 = (l14 * 0.0 - 1.0) / l175_im;
    }
  }
  l10 = 12.0 * (l13 * l153.re);
  l7 = 12.0 * (l13 * l153.im);
  l8 = -9.0 * (l100 * l153.re - l99 * l153.im);
  l9 = -9.0 * (l100 * l153.im + l99 * l153.re);
  l2 = l103 * l153.re - l6 * l153.im;
  l133_tmp = l103 * l153.im + l6 * l153.re;
  if (l133_tmp == 0.0) {
    l2 /= 6.0;
    l175_im = 0.0;
  } else if (l2 == 0.0) {
    l2 = 0.0;
    l175_im = l133_tmp / 6.0;
  } else {
    l2 /= 6.0;
    l175_im = l133_tmp / 6.0;
  }
  l100 = 12.0 * (a_tmp * (l159.re * l153.re - l159.im * l153.im));
  l99 = 12.0 * (a_tmp * (l159.re * l153.im + l159.im * l153.re));
  l158.re = -(l127 * l153.re);
  l158.im = -(l127 * l153.im);
  dc.re = (((l5 + l158.re) + l10) + l8) + l100;
  dc.im = (((l165_im + l158.im) + l7) + l9) + l99;
  coder::internal::scalar::b_sqrt(&dc);
  l14 = l103 * l11 - l6 * l3;
  l3 = l103 * l3 + l6 * l11;
  l103 = l14 * dc.re - l3 * dc.im;
  l6 = l14 * dc.im + l3 * dc.re;
  if (l6 == 0.0) {
    l159.re = l103 / 6.0;
    l159.im = 0.0;
  } else if (l103 == 0.0) {
    l159.re = 0.0;
    l159.im = l6 / 6.0;
  } else {
    l159.re = l103 / 6.0;
    l159.im = l6 / 6.0;
  }
  dc.re = (((-l5 + l158.re) + l10) + l8) + l100;
  dc.im = (((-l165_im + l158.im) + l7) + l9) + l99;
  coder::internal::scalar::b_sqrt(&dc);
  l103 = l14 * dc.re - l3 * dc.im;
  l6 = l14 * dc.im + l3 * dc.re;
  if (l6 == 0.0) {
    l158.re = l103 / 6.0;
    l158.im = 0.0;
  } else if (l103 == 0.0) {
    l158.re = 0.0;
    l158.im = l6 / 6.0;
  } else {
    l158.re = l103 / 6.0;
    l158.im = l6 / 6.0;
  }
  l133_tmp = -l106 + -l2;
  t1[0].re = l133_tmp - l159.re;
  t1[0].im = -l175_im - l159.im;
  t1[1].re = l133_tmp + l159.re;
  t1[1].im = -l175_im + l159.im;
  l133_tmp = -l106 + l2;
  t1[2].re = l133_tmp - l158.re;
  t1[2].im = l175_im - l158.im;
  t1[3].re = l133_tmp + l158.re;
  t1[3].im = l175_im + l158.im;
  l5 = 1.0 / J_max;
  l14 = A_min + -A_wayp;
  l153.re = (l33_tmp * 2.0 - b_l133_tmp * 2.0) + A_wayp * -A_wayp;
  l158.re = J_min * l5 * (l14 * l14);
  l159.re = l115_tmp * l5 * l14 * 2.0;
  l6 = -(1.0 / J_max * l14);
  l10 = J_max * t1[0].re;
  l8 = J_max * t1[0].im;
  b_l2[0].re = l10;
  b_l2[0].im = l8;
  l175_im = A_init + l10;
  l14 = l175_im + -A_min;
  l2 = l14 * l8;
  l3 = J_min * l10;
  l7 = J_min * l8;
  l133_tmp = l8 * l8;
  y[0].re = -0.5 * ((((((l153.re + (l14 * l14 - l133_tmp)) -
                        (l14 * l175_im - l133_tmp) * 2.0) +
                       b_l115_tmp * t1[0].re * 2.0) +
                      (l3 * t1[0].re - l7 * t1[0].im)) +
                     l158.re) -
                    l159.re);
  y[0].im =
      -0.5 *
      ((((l2 + l2) - (l2 + l8 * l175_im) * 2.0) + b_l115_tmp * t1[0].im * 2.0) +
       (l3 * t1[0].im + l7 * t1[0].re));
  t[0] = t1[0];
  t[4].re = 0.0;
  t[4].im = 0.0;
  l10 = J_max * t1[1].re;
  l8 = J_max * t1[1].im;
  b_l2[1].re = l10;
  b_l2[1].im = l8;
  l175_im = A_init + l10;
  l14 = l175_im + -A_min;
  l2 = l14 * l8;
  l3 = J_min * l10;
  l7 = J_min * l8;
  l133_tmp = l8 * l8;
  y[1].re = -0.5 * ((((((l153.re + (l14 * l14 - l133_tmp)) -
                        (l14 * l175_im - l133_tmp) * 2.0) +
                       b_l115_tmp * t1[1].re * 2.0) +
                      (l3 * t1[1].re - l7 * t1[1].im)) +
                     l158.re) -
                    l159.re);
  y[1].im =
      -0.5 *
      ((((l2 + l2) - (l2 + l8 * l175_im) * 2.0) + b_l115_tmp * t1[1].im * 2.0) +
       (l3 * t1[1].im + l7 * t1[1].re));
  t[1] = t1[1];
  t[5].re = 0.0;
  t[5].im = 0.0;
  l10 = J_max * t1[2].re;
  l8 = J_max * t1[2].im;
  b_l2[2].re = l10;
  b_l2[2].im = l8;
  l175_im = A_init + l10;
  l14 = l175_im + -A_min;
  l2 = l14 * l8;
  l3 = J_min * l10;
  l7 = J_min * l8;
  l133_tmp = l8 * l8;
  y[2].re = -0.5 * ((((((l153.re + (l14 * l14 - l133_tmp)) -
                        (l14 * l175_im - l133_tmp) * 2.0) +
                       b_l115_tmp * t1[2].re * 2.0) +
                      (l3 * t1[2].re - l7 * t1[2].im)) +
                     l158.re) -
                    l159.re);
  y[2].im =
      -0.5 *
      ((((l2 + l2) - (l2 + l8 * l175_im) * 2.0) + b_l115_tmp * t1[2].im * 2.0) +
       (l3 * t1[2].im + l7 * t1[2].re));
  t[2] = t1[2];
  t[6].re = 0.0;
  t[6].im = 0.0;
  l10 = J_max * t1[3].re;
  l8 = J_max * t1[3].im;
  l175_im = A_init + l10;
  l14 = l175_im + -A_min;
  l2 = l14 * l8;
  l3 = J_min * l10;
  l7 = J_min * l8;
  l133_tmp = l8 * l8;
  y[3].re = -0.5 * ((((((l153.re + (l14 * l14 - l133_tmp)) -
                        (l14 * l175_im - l133_tmp) * 2.0) +
                       b_l115_tmp * t1[3].re * 2.0) +
                      (l3 * t1[3].re - l7 * t1[3].im)) +
                     l158.re) -
                    l159.re);
  y[3].im =
      -0.5 *
      ((((l2 + l2) - (l2 + l8 * l175_im) * 2.0) + b_l115_tmp * t1[3].im * 2.0) +
       (l3 * t1[3].im + l7 * t1[3].re));
  t[3] = t1[3];
  t[7].re = 0.0;
  t[7].im = 0.0;
  l153.re = A_init - A_min;
  l14 = -(l153.re + b_l2[0].re);
  if (-b_l2[0].im == 0.0) {
    t[8].re = l14 / J_min;
    t[8].im = 0.0;
  } else if (l14 == 0.0) {
    t[8].re = 0.0;
    t[8].im = -b_l2[0].im / J_min;
  } else {
    t[8].re = l14 / J_min;
    t[8].im = -b_l2[0].im / J_min;
  }
  t[12].re = 0.0;
  t[12].im = 0.0;
  t[16].re = 0.0;
  t[16].im = 0.0;
  if (y[0].im == 0.0) {
    t[20].re = y[0].re / l115_tmp;
    t[20].im = 0.0;
  } else if (y[0].re == 0.0) {
    t[20].re = 0.0;
    t[20].im = y[0].im / l115_tmp;
  } else {
    t[20].re = y[0].re / l115_tmp;
    t[20].im = y[0].im / l115_tmp;
  }
  l14 = -(l153.re + b_l2[1].re);
  if (-b_l2[1].im == 0.0) {
    t[9].re = l14 / J_min;
    t[9].im = 0.0;
  } else if (l14 == 0.0) {
    t[9].re = 0.0;
    t[9].im = -b_l2[1].im / J_min;
  } else {
    t[9].re = l14 / J_min;
    t[9].im = -b_l2[1].im / J_min;
  }
  t[13].re = 0.0;
  t[13].im = 0.0;
  t[17].re = 0.0;
  t[17].im = 0.0;
  if (y[1].im == 0.0) {
    t[21].re = y[1].re / l115_tmp;
    t[21].im = 0.0;
  } else if (y[1].re == 0.0) {
    t[21].re = 0.0;
    t[21].im = y[1].im / l115_tmp;
  } else {
    t[21].re = y[1].re / l115_tmp;
    t[21].im = y[1].im / l115_tmp;
  }
  l14 = -(l153.re + b_l2[2].re);
  if (-b_l2[2].im == 0.0) {
    t[10].re = l14 / J_min;
    t[10].im = 0.0;
  } else if (l14 == 0.0) {
    t[10].re = 0.0;
    t[10].im = -b_l2[2].im / J_min;
  } else {
    t[10].re = l14 / J_min;
    t[10].im = -b_l2[2].im / J_min;
  }
  t[14].re = 0.0;
  t[14].im = 0.0;
  t[18].re = 0.0;
  t[18].im = 0.0;
  if (y[2].im == 0.0) {
    t[22].re = y[2].re / l115_tmp;
    t[22].im = 0.0;
  } else if (y[2].re == 0.0) {
    t[22].re = 0.0;
    t[22].im = y[2].im / l115_tmp;
  } else {
    t[22].re = y[2].re / l115_tmp;
    t[22].im = y[2].im / l115_tmp;
  }
  l14 = -(l153.re + l10);
  if (-l8 == 0.0) {
    t[11].re = l14 / J_min;
    t[11].im = 0.0;
  } else if (l14 == 0.0) {
    t[11].re = 0.0;
    t[11].im = -l8 / J_min;
  } else {
    t[11].re = l14 / J_min;
    t[11].im = -l8 / J_min;
  }
  t[15].re = 0.0;
  t[15].im = 0.0;
  t[19].re = 0.0;
  t[19].im = 0.0;
  if (y[3].im == 0.0) {
    t[23].re = y[3].re / l115_tmp;
    t[23].im = 0.0;
  } else if (y[3].re == 0.0) {
    t[23].re = 0.0;
    t[23].im = y[3].im / l115_tmp;
  } else {
    t[23].re = y[3].re / l115_tmp;
    t[23].im = y[3].im / l115_tmp;
  }
  t[24].re = l6;
  t[24].im = 0.0;
  t[25].re = l6;
  t[25].im = 0.0;
  t[26].re = l6;
  t[26].im = 0.0;
  t[27].re = l6;
  t[27].im = 0.0;
}

// End of code generation (acefg_O_AP.cpp)
