//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// abcdeg_NTV_AVP.cpp
//
// Code generation for function 'abcdeg_NTV_AVP'
//

// Include files
#include "abcdeg_NTV_AVP.h"
#include "power.h"
#include "rt_nonfinite.h"
#include "sqrt.h"
#include "topico_wrapper_data.h"
#include "topico_wrapper_rtwutil.h"
#include <cmath>

// Function Definitions
void abcdeg_NTV_AVP(double P_init, double V_init, double A_init, double P_wayp,
                    double V_wayp, double A_wayp, double A_min, double J_max,
                    double J_min, double T, creal_T t[28])
{
  creal_T l2[4];
  creal_T t2[4];
  creal_T t7[4];
  creal_T dc;
  creal_T l169;
  creal_T l174;
  double a_tmp;
  double b_l144_tmp;
  double b_l156_tmp;
  double c_l156_tmp;
  double l10;
  double l103;
  double l103_tmp;
  double l104;
  double l11;
  double l113;
  double l12;
  double l120;
  double l124;
  double l136;
  double l144_tmp;
  double l148;
  double l149;
  double l156;
  double l156_tmp;
  double l15_tmp;
  double l16_tmp;
  double l17_tmp;
  double l180_im;
  double l184_re;
  double l187_im;
  double l187_re;
  double l189_re;
  double l18_tmp;
  double l19;
  double l20;
  double l21;
  double l2_tmp;
  double l3;
  double l4;
  double l5;
  double l6;
  double l7;
  double l88;
  double l89;
  double l9;
  double l90;
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
  //  Generated on 28-Aug-2019 13:55:05
  l2_tmp = A_init * A_init;
  l3 = rt_powd_snf(A_init, 3.0);
  l4 = A_min * A_min;
  l5 = rt_powd_snf(A_min, 3.0);
  l7 = A_wayp * A_wayp;
  l9 = J_min * J_min;
  l10 = J_max * J_max;
  l11 = rt_powd_snf(J_max, 3.0);
  l120 = A_wayp * J_min;
  l15_tmp = J_min * J_max;
  l16_tmp = l15_tmp * V_wayp * 2.0;
  l6 = l4 * l4;
  l12 = l10 * l10;
  l17_tmp = J_max * l2_tmp;
  l18_tmp = J_min * l4;
  l19 = J_max * l4;
  l20 = J_max * l7;
  l21 = J_min * l10;
  l187_im = l11 - l21;
  l88 = 1.0 / (l187_im * l187_im);
  l189_re = l120 * J_max * 2.0 - A_wayp * l10 * 2.0;
  a_tmp =
      ((((l15_tmp * V_init * 2.0 - l16_tmp) - l17_tmp) - l18_tmp) + l19) + l20;
  l89 = l88 * l88;
  l90 = rt_powd_snf(l88, 3.0);
  l103_tmp = A_min * J_min;
  l149 = A_min * l9;
  l103 = ((A_min * rt_powd_snf(J_max, 5.0) * 4.0 + -(l103_tmp * l12 * 12.0)) +
          l149 * l11 * 8.0) +
         l187_im * l189_re * 6.0;
  l104 = l103 * l103;
  l113 = l88 * l103 / 12.0;
  l187_re = A_min * A_wayp;
  l124 = A_init * A_wayp;
  l120 = ((A_min * l7 * l11 * 12.0 + l19 * l189_re * -12.0) +
          l189_re * a_tmp * 6.0) +
         -(A_min *
           ((((((l124 * l11 * 12.0 + l187_re * l21 * 6.0) +
                -(l187_re * J_max * l9 * 6.0)) +
               l120 * T * l11 * 12.0) +
              l7 * l21 * 6.0) +
             -(l124 * l21 * 12.0)) +
            -(A_wayp * T * l9 * l10 * 12.0)) *
           2.0);
  l180_im = A_init * J_min;
  l156 = T * l9;
  l124 = (((l187_re * J_min * l11 * 24.0 + l189_re * l189_re * 3.0) +
           l19 * l187_im * -12.0) +
          l187_im * a_tmp * 6.0) +
         -(A_min *
           (((((((A_wayp * l12 * 6.0 + -(A_init * l12 * 6.0)) +
                 l180_im * l11 * 6.0) +
                -(l103_tmp * l11 * 3.0)) +
               -(J_min * T * l12 * 6.0)) +
              l156 * l11 * 6.0) +
             l149 * l10 * 3.0) +
            A_wayp * l9 * l10 * 6.0) *
           2.0);
  l144_tmp = A_min * J_max;
  b_l144_tmp = A_init * J_max;
  l136 = l89 * l104 / 24.0 + -(l88 * l124 / 3.0);
  l184_re = l89 * l103;
  l148 = (l90 * rt_powd_snf(l103, 3.0) / 216.0 + l88 * l120 / 3.0) +
         -(l184_re * l124 / 18.0);
  l156_tmp = l89 * l89 * (l104 * l104);
  b_l156_tmp = l184_re * l120;
  c_l156_tmp = l90 * l104 * l124;
  l124 =
      l88 *
      (((((((((l6 * l9 * 4.0 + l6 * l10 * 8.0) + l180_im * J_max * l5 * 12.0) +
             A_min * l3 * l10 * 4.0) +
            A_min * P_wayp * l9 * l10 * 24.0) +
           J_max * T * l5 * l9 * 12.0) +
          A_min * T * l2_tmp * l21 * 12.0) +
         a_tmp * a_tmp * 3.0) +
        l19 * a_tmp * -12.0) +
       -(A_min *
         ((((((((((((((((l15_tmp * l5 * 3.0 + l3 * l10 * 6.0) + l5 * l9 * 3.0) +
                       rt_powd_snf(A_wayp, 3.0) * l10 * 2.0) +
                      l103_tmp * l17_tmp * 3.0) +
                     b_l144_tmp * l18_tmp * 6.0) +
                    l144_tmp * V_wayp * l9 * 6.0) +
                   -(l103_tmp * l20 * 3.0)) +
                  -(l144_tmp * V_init * l9 * 6.0)) +
                 A_init * V_wayp * l21 * 12.0) +
                P_init * l9 * l10 * 12.0) +
               T * l2_tmp * l21 * 6.0) +
              l156 * l19 * 6.0) +
             -(A_init * V_init * l21 * 12.0)) +
            -(A_init * l7 * l10 * 6.0)) +
           T * l7 * l21 * -6.0) +
          T * V_wayp * l9 * l10 * 12.0) *
         2.0));
  l156 = ((l156_tmp / 6912.0 + b_l156_tmp / 36.0) + -(c_l156_tmp / 432.0)) +
         -(l124 / 3.0);
  l7 = l136 * l136;
  l120 = rt_powd_snf(l136, 3.0);
  l149 = l148 * l148;
  l169.re = ((((l149 * l149 * 27.0 + -(l120 * l149 * 4.0)) +
               rt_powd_snf(l156, 3.0) * 256.0) +
              l7 * l7 * l156 * 16.0) +
             l7 * (l156 * l156) * 128.0) +
            -(l136 * l149 * l156 * 144.0);
  l169.im = 0.0;
  coder::internal::scalar::b_sqrt(&l169);
  l187_re = 1.7320508075688772 * l169.re;
  l184_re = 1.7320508075688772 * l169.im;
  if (l184_re == 0.0) {
    l89 = l187_re / 18.0;
    l180_im = 0.0;
  } else if (l187_re == 0.0) {
    l89 = 0.0;
    l180_im = l184_re / 18.0;
  } else {
    l89 = l187_re / 18.0;
    l180_im = l184_re / 18.0;
  }
  l11 = l136 * l156;
  l174.re = ((-(l120 / 27.0) + l149 / 2.0) + -(l11 * 1.3333333333333333)) + l89;
  l174.im = l180_im;
  l169 = coder::power(l174);
  dc = coder::b_power(l174);
  if (dc.im == 0.0) {
    l9 = 1.0 / dc.re;
    l10 = 0.0;
  } else if (dc.re == 0.0) {
    l9 = 0.0;
    l10 = -(1.0 / dc.im);
  } else {
    l12 = std::abs(dc.re);
    l103 = std::abs(dc.im);
    if (l12 > l103) {
      l187_im = dc.im / dc.re;
      l189_re = dc.re + l187_im * dc.im;
      l9 = (l187_im * 0.0 + 1.0) / l189_re;
      l10 = (0.0 - l187_im) / l189_re;
    } else if (l103 == l12) {
      if (dc.re > 0.0) {
        l187_im = 0.5;
      } else {
        l187_im = -0.5;
      }
      if (dc.im > 0.0) {
        l189_re = 0.5;
      } else {
        l189_re = -0.5;
      }
      l9 = (l187_im + 0.0 * l189_re) / l12;
      l10 = (0.0 * l187_im - l189_re) / l12;
    } else {
      l187_im = dc.re / dc.im;
      l189_re = dc.im + l187_im * dc.re;
      l9 = l187_im / l189_re;
      l10 = (l187_im * 0.0 - 1.0) / l189_re;
    }
  }
  l5 = l169.re * l169.re - l169.im * l169.im;
  l189_re = l169.re * l169.im;
  l104 = l189_re + l189_re;
  dc.re = ((-(l120 * 2.0) + l149 * 27.0) + -(l11 * 72.0)) + l187_re * 3.0;
  dc.im = l184_re * 3.0;
  coder::internal::scalar::b_sqrt(&dc);
  l90 = 3.0 * (2.4494897427831779 * l148 * dc.re);
  l180_im = 3.0 * (2.4494897427831779 * l148 * dc.im);
  l89 = l136 * l169.re;
  l11 = l136 * l169.im;
  l174.re =
      (((((-(l156_tmp / 576.0) + -(b_l156_tmp / 3.0)) + c_l156_tmp / 36.0) +
         l7) +
        l124 * 4.0) +
       l5 * 9.0) +
      l89 * 6.0;
  l174.im = l104 * 9.0 + l11 * 6.0;
  l169 = l174;
  coder::internal::scalar::b_sqrt(&l169);
  dc = coder::c_power(l174);
  if (dc.im == 0.0) {
    l184_re = 1.0 / dc.re;
    l124 = 0.0;
  } else if (dc.re == 0.0) {
    l184_re = 0.0;
    l124 = -(1.0 / dc.im);
  } else {
    l12 = std::abs(dc.re);
    l103 = std::abs(dc.im);
    if (l12 > l103) {
      l187_im = dc.im / dc.re;
      l189_re = dc.re + l187_im * dc.im;
      l184_re = (l187_im * 0.0 + 1.0) / l189_re;
      l124 = (0.0 - l187_im) / l189_re;
    } else if (l103 == l12) {
      if (dc.re > 0.0) {
        l187_im = 0.5;
      } else {
        l187_im = -0.5;
      }
      if (dc.im > 0.0) {
        l189_re = 0.5;
      } else {
        l189_re = -0.5;
      }
      l184_re = (l187_im + 0.0 * l189_re) / l12;
      l124 = (0.0 * l187_im - l189_re) / l12;
    } else {
      l187_im = dc.re / dc.im;
      l189_re = dc.im + l187_im * dc.re;
      l184_re = l187_im / l189_re;
      l124 = (l187_im * 0.0 - 1.0) / l189_re;
    }
  }
  l187_re = 12.0 * (l156 * l169.re);
  l187_im = 12.0 * (l156 * l169.im);
  l189_re = -9.0 * (l5 * l169.re - l104 * l169.im);
  l120 = -9.0 * (l5 * l169.im + l104 * l169.re);
  l149 = l9 * l169.re - l10 * l169.im;
  l103 = l9 * l169.im + l10 * l169.re;
  if (l103 == 0.0) {
    l149 /= 6.0;
    l12 = 0.0;
  } else if (l149 == 0.0) {
    l149 = 0.0;
    l12 = l103 / 6.0;
  } else {
    l149 /= 6.0;
    l12 = l103 / 6.0;
  }
  l5 = 12.0 * (l89 * l169.re - l11 * l169.im);
  l104 = 12.0 * (l89 * l169.im + l11 * l169.re);
  l174.re = -(l7 * l169.re);
  l174.im = -(l7 * l169.im);
  dc.re = (((l90 + l174.re) + l187_re) + l189_re) + l5;
  dc.im = (((l180_im + l174.im) + l187_im) + l120) + l104;
  coder::internal::scalar::b_sqrt(&dc);
  l11 = l9 * l184_re - l10 * l124;
  l184_re = l9 * l124 + l10 * l184_re;
  l9 = l11 * dc.re - l184_re * dc.im;
  l10 = l11 * dc.im + l184_re * dc.re;
  if (l10 == 0.0) {
    l169.re = l9 / 6.0;
    l169.im = 0.0;
  } else if (l9 == 0.0) {
    l169.re = 0.0;
    l169.im = l10 / 6.0;
  } else {
    l169.re = l9 / 6.0;
    l169.im = l10 / 6.0;
  }
  dc.re = (((-l90 + l174.re) + l187_re) + l189_re) + l5;
  dc.im = (((-l180_im + l174.im) + l187_im) + l120) + l104;
  coder::internal::scalar::b_sqrt(&dc);
  l9 = l11 * dc.re - l184_re * dc.im;
  l10 = l11 * dc.im + l184_re * dc.re;
  if (l10 == 0.0) {
    l174.re = l9 / 6.0;
    l174.im = 0.0;
  } else if (l9 == 0.0) {
    l174.re = 0.0;
    l174.im = l10 / 6.0;
  } else {
    l174.re = l9 / 6.0;
    l174.im = l10 / 6.0;
  }
  l184_re = -l113 + -l149;
  t7[0].re = l184_re - l169.re;
  t7[0].im = -l12 - l169.im;
  t7[1].re = l184_re + l169.re;
  t7[1].im = -l12 + l169.im;
  l184_re = -l113 + l149;
  t7[2].re = l184_re - l174.re;
  t7[2].im = l12 - l174.im;
  t7[3].re = l184_re + l174.re;
  t7[3].im = l12 + l174.im;
  l187_im = l103_tmp * J_max * 2.0;
  l169.re = l18_tmp + l17_tmp * 2.0;
  l174.re = (l2_tmp + l4) + J_min * V_init * 2.0;
  l124 = 1.0 / J_min;
  l3 = A_min * (1.0 / J_max);
  l6 = -(1.0 / J_min * (A_init + -A_min));
  t[0].re = l6;
  t[0].im = 0.0;
  t[1].re = l6;
  t[1].im = 0.0;
  t[2].re = l6;
  t[2].im = 0.0;
  t[3].re = l6;
  t[3].im = 0.0;
  l89 = J_max * t7[0].re;
  l180_im = J_max * t7[0].im;
  l2[0].re = l89;
  l2[0].im = l180_im;
  l12 = A_wayp + -l89;
  l184_re = l12 * -l180_im;
  l149 = J_min * l89;
  l103 = J_min * l180_im;
  l11 = J_min * l12;
  l189_re = J_min * -l180_im;
  l120 = (l169.re - J_max * (((l174.re + (l12 * l12 - -l180_im * -l180_im)) +
                              (l149 * t7[0].re - l103 * t7[0].im)) +
                             (l11 * t7[0].re - l189_re * t7[0].im) * 2.0)) +
         l16_tmp;
  l103 = 0.0 -
         J_max * (((l184_re + l184_re) + (l149 * t7[0].im + l103 * t7[0].re)) +
                  (l11 * t7[0].im + l189_re * t7[0].re) * 2.0);
  if (l103 == 0.0) {
    l12 = l120 / l187_im;
    l149 = 0.0;
  } else if (l120 == 0.0) {
    l12 = 0.0;
    l149 = l103 / l187_im;
  } else {
    l12 = l120 / l187_im;
    l149 = l103 / l187_im;
  }
  t2[0].re = l12;
  t2[0].im = l149;
  t[4].re = l12;
  t[4].im = l149;
  l89 = J_max * t7[1].re;
  l180_im = J_max * t7[1].im;
  l2[1].re = l89;
  l2[1].im = l180_im;
  l12 = A_wayp + -l89;
  l184_re = l12 * -l180_im;
  l149 = J_min * l89;
  l103 = J_min * l180_im;
  l11 = J_min * l12;
  l189_re = J_min * -l180_im;
  l120 = (l169.re - J_max * (((l174.re + (l12 * l12 - -l180_im * -l180_im)) +
                              (l149 * t7[1].re - l103 * t7[1].im)) +
                             (l11 * t7[1].re - l189_re * t7[1].im) * 2.0)) +
         l16_tmp;
  l103 = 0.0 -
         J_max * (((l184_re + l184_re) + (l149 * t7[1].im + l103 * t7[1].re)) +
                  (l11 * t7[1].im + l189_re * t7[1].re) * 2.0);
  if (l103 == 0.0) {
    l12 = l120 / l187_im;
    l149 = 0.0;
  } else if (l120 == 0.0) {
    l12 = 0.0;
    l149 = l103 / l187_im;
  } else {
    l12 = l120 / l187_im;
    l149 = l103 / l187_im;
  }
  t2[1].re = l12;
  t2[1].im = l149;
  t[5].re = l12;
  t[5].im = l149;
  l89 = J_max * t7[2].re;
  l180_im = J_max * t7[2].im;
  l2[2].re = l89;
  l2[2].im = l180_im;
  l12 = A_wayp + -l89;
  l184_re = l12 * -l180_im;
  l149 = J_min * l89;
  l103 = J_min * l180_im;
  l11 = J_min * l12;
  l189_re = J_min * -l180_im;
  l120 = (l169.re - J_max * (((l174.re + (l12 * l12 - -l180_im * -l180_im)) +
                              (l149 * t7[2].re - l103 * t7[2].im)) +
                             (l11 * t7[2].re - l189_re * t7[2].im) * 2.0)) +
         l16_tmp;
  l103 = 0.0 -
         J_max * (((l184_re + l184_re) + (l149 * t7[2].im + l103 * t7[2].re)) +
                  (l11 * t7[2].im + l189_re * t7[2].re) * 2.0);
  if (l103 == 0.0) {
    l12 = l120 / l187_im;
    l149 = 0.0;
  } else if (l120 == 0.0) {
    l12 = 0.0;
    l149 = l103 / l187_im;
  } else {
    l12 = l120 / l187_im;
    l149 = l103 / l187_im;
  }
  t2[2].re = l12;
  t2[2].im = l149;
  t[6].re = l12;
  t[6].im = l149;
  l89 = J_max * t7[3].re;
  l180_im = J_max * t7[3].im;
  l12 = A_wayp + -l89;
  l184_re = l12 * -l180_im;
  l149 = J_min * l89;
  l103 = J_min * l180_im;
  l11 = J_min * l12;
  l189_re = J_min * -l180_im;
  l120 = (l169.re - J_max * (((l174.re + (l12 * l12 - -l180_im * -l180_im)) +
                              (l149 * t7[3].re - l103 * t7[3].im)) +
                             (l11 * t7[3].re - l189_re * t7[3].im) * 2.0)) +
         l16_tmp;
  l103 = 0.0 -
         J_max * (((l184_re + l184_re) + (l149 * t7[3].im + l103 * t7[3].re)) +
                  (l11 * t7[3].im + l189_re * t7[3].re) * 2.0);
  if (l103 == 0.0) {
    l12 = l120 / l187_im;
    l149 = 0.0;
  } else if (l120 == 0.0) {
    l12 = 0.0;
    l149 = l103 / l187_im;
  } else {
    l12 = l120 / l187_im;
    l149 = l103 / l187_im;
  }
  t[7].re = l12;
  t[7].im = l149;
  l169.re = ((b_l144_tmp + l103_tmp) - l144_tmp) + l15_tmp * T;
  t[8].re = -l3;
  t[8].im = 0.0;
  t[9].re = -l3;
  t[9].im = 0.0;
  t[10].re = -l3;
  t[10].im = 0.0;
  t[11].re = -l3;
  t[11].im = 0.0;
  l184_re = A_wayp - l2[0].re;
  l120 = l124 * (l169.re - l15_tmp * ((t2[0].re + t7[0].re) + l124 * l184_re));
  l103 = l124 *
         (0.0 - l15_tmp * ((t2[0].im + t7[0].im) + l124 * (0.0 - l2[0].im)));
  if (l103 == 0.0) {
    t[12].re = l120 / J_max;
    t[12].im = 0.0;
  } else if (l120 == 0.0) {
    t[12].re = 0.0;
    t[12].im = l103 / J_max;
  } else {
    t[12].re = l120 / J_max;
    t[12].im = l103 / J_max;
  }
  if (0.0 - l2[0].im == 0.0) {
    t[16].re = l184_re / J_min;
    t[16].im = 0.0;
  } else if (l184_re == 0.0) {
    t[16].re = 0.0;
    t[16].im = (0.0 - l2[0].im) / J_min;
  } else {
    t[16].re = l184_re / J_min;
    t[16].im = (0.0 - l2[0].im) / J_min;
  }
  t[20].re = 0.0;
  t[20].im = 0.0;
  t[24] = t7[0];
  l184_re = A_wayp - l2[1].re;
  l120 = l124 * (l169.re - l15_tmp * ((t2[1].re + t7[1].re) + l124 * l184_re));
  l103 = l124 *
         (0.0 - l15_tmp * ((t2[1].im + t7[1].im) + l124 * (0.0 - l2[1].im)));
  if (l103 == 0.0) {
    t[13].re = l120 / J_max;
    t[13].im = 0.0;
  } else if (l120 == 0.0) {
    t[13].re = 0.0;
    t[13].im = l103 / J_max;
  } else {
    t[13].re = l120 / J_max;
    t[13].im = l103 / J_max;
  }
  if (0.0 - l2[1].im == 0.0) {
    t[17].re = l184_re / J_min;
    t[17].im = 0.0;
  } else if (l184_re == 0.0) {
    t[17].re = 0.0;
    t[17].im = (0.0 - l2[1].im) / J_min;
  } else {
    t[17].re = l184_re / J_min;
    t[17].im = (0.0 - l2[1].im) / J_min;
  }
  t[21].re = 0.0;
  t[21].im = 0.0;
  t[25] = t7[1];
  l184_re = A_wayp - l2[2].re;
  l120 = l124 * (l169.re - l15_tmp * ((t2[2].re + t7[2].re) + l124 * l184_re));
  l103 = l124 *
         (0.0 - l15_tmp * ((t2[2].im + t7[2].im) + l124 * (0.0 - l2[2].im)));
  if (l103 == 0.0) {
    t[14].re = l120 / J_max;
    t[14].im = 0.0;
  } else if (l120 == 0.0) {
    t[14].re = 0.0;
    t[14].im = l103 / J_max;
  } else {
    t[14].re = l120 / J_max;
    t[14].im = l103 / J_max;
  }
  if (0.0 - l2[2].im == 0.0) {
    t[18].re = l184_re / J_min;
    t[18].im = 0.0;
  } else if (l184_re == 0.0) {
    t[18].re = 0.0;
    t[18].im = (0.0 - l2[2].im) / J_min;
  } else {
    t[18].re = l184_re / J_min;
    t[18].im = (0.0 - l2[2].im) / J_min;
  }
  t[22].re = 0.0;
  t[22].im = 0.0;
  t[26] = t7[2];
  l184_re = A_wayp - l89;
  l120 = l124 * (l169.re - l15_tmp * ((l12 + t7[3].re) + l124 * l184_re));
  l103 = l124 * (0.0 - l15_tmp * ((l149 + t7[3].im) + l124 * (0.0 - l180_im)));
  if (l103 == 0.0) {
    t[15].re = l120 / J_max;
    t[15].im = 0.0;
  } else if (l120 == 0.0) {
    t[15].re = 0.0;
    t[15].im = l103 / J_max;
  } else {
    t[15].re = l120 / J_max;
    t[15].im = l103 / J_max;
  }
  if (0.0 - l180_im == 0.0) {
    t[19].re = l184_re / J_min;
    t[19].im = 0.0;
  } else if (l184_re == 0.0) {
    t[19].re = 0.0;
    t[19].im = (0.0 - l180_im) / J_min;
  } else {
    t[19].re = l184_re / J_min;
    t[19].im = (0.0 - l180_im) / J_min;
  }
  t[23].re = 0.0;
  t[23].im = 0.0;
  t[27] = t7[3];
}

// End of code generation (abcdeg_NTV_AVP.cpp)
