//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// abcdeg_TV_AVP.cpp
//
// Code generation for function 'abcdeg_TV_AVP'
//

// Include files
#include "abcdeg_TV_AVP.h"
#include "power.h"
#include "rt_nonfinite.h"
#include "sqrt.h"
#include "topico_wrapper_data.h"
#include "topico_wrapper_rtwutil.h"
#include <cmath>

// Function Definitions
void abcdeg_TV_AVP(double P_init, double V_init, double A_init, double P_wayp,
                   double V_wayp, double A_wayp, double A_max, double J_max,
                   double J_min, double T, creal_T t[28])
{
  creal_T b_l7[4];
  creal_T l2[4];
  creal_T t7[4];
  creal_T dc;
  creal_T l169;
  creal_T l174;
  double a_tmp;
  double b_l156_tmp;
  double l10;
  double l103;
  double l103_tmp;
  double l104;
  double l11;
  double l113;
  double l12;
  double l120;
  double l120_tmp;
  double l124;
  double l124_tmp;
  double l136;
  double l138;
  double l144_tmp;
  double l148;
  double l156_tmp;
  double l15_tmp;
  double l16_tmp;
  double l17_tmp;
  double l18;
  double l184_im;
  double l187_im;
  double l190_im;
  double l19_tmp;
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
  l4 = A_max * A_max;
  l5 = rt_powd_snf(A_max, 3.0);
  l7 = A_wayp * A_wayp;
  l9 = J_min * J_min;
  l10 = J_max * J_max;
  l11 = rt_powd_snf(J_max, 3.0);
  l124 = A_wayp * J_min;
  l15_tmp = J_min * J_max;
  l16_tmp = l15_tmp * V_wayp * 2.0;
  l6 = l4 * l4;
  l12 = l10 * l10;
  l17_tmp = J_min * l2_tmp;
  l18 = J_min * l4;
  l19_tmp = J_max * l4;
  l20 = J_max * l7;
  l21 = J_min * l10;
  l190_im = l11 - l21;
  l88 = 1.0 / (l190_im * l190_im);
  l184_im = l124 * J_max * 2.0 - A_wayp * l10 * 2.0;
  a_tmp =
      ((((l15_tmp * V_init * 2.0 - l16_tmp) - l17_tmp) + l18) - l19_tmp) + l20;
  l89 = l88 * l88;
  l90 = rt_powd_snf(l88, 3.0);
  l103_tmp = A_max * J_min;
  l103 = ((A_max * rt_powd_snf(J_max, 5.0) * 4.0 + -(l103_tmp * l12 * 12.0)) +
          A_max * l9 * l11 * 8.0) +
         l190_im * l184_im * 6.0;
  l104 = l103 * l103;
  l113 = l88 * l103 / 12.0;
  l187_im = A_max * A_wayp;
  l138 = A_init * A_wayp;
  l120_tmp = A_max * l7;
  l120 = ((l120_tmp * l11 * 12.0 + l18 * l184_im * -12.0) +
          l184_im * a_tmp * 6.0) +
         -(A_max *
           ((((((l187_im * l11 * 6.0 + l138 * l21 * 12.0) +
                -(l187_im * l21 * 6.0)) +
               l124 * T * l11 * 12.0) +
              l7 * l21 * 6.0) +
             -(l138 * J_max * l9 * 12.0)) +
            -(A_wayp * T * l9 * l10 * 12.0)) *
           2.0);
  l124_tmp = A_init * J_min;
  l124 = (((l187_im * J_min * l11 * 24.0 + l184_im * l184_im * 3.0) +
           l18 * l190_im * -12.0) +
          l190_im * a_tmp * 6.0) +
         -(A_max *
           (((((((A_wayp * l12 * 6.0 + -(A_max * l12 * 3.0)) +
                 l103_tmp * l11 * 3.0) +
                -(l124_tmp * l11 * 6.0)) +
               -(J_min * T * l12 * 6.0)) +
              T * l9 * l11 * 6.0) +
             A_init * l9 * l10 * 6.0) +
            A_wayp * l9 * l10 * 6.0) *
           2.0);
  l144_tmp = A_max * J_max;
  l184_im = A_init * J_max;
  l136 = l89 * l104 / 24.0 + -(l88 * l124 / 3.0);
  l190_im = l89 * l103;
  l148 = (l90 * rt_powd_snf(l103, 3.0) / 216.0 + l88 * l120 / 3.0) +
         -(l190_im * l124 / 18.0);
  l156_tmp = l89 * l89 * (l104 * l104);
  b_l156_tmp = l190_im * l120;
  l124 *= l90 * l104;
  l12 =
      l88 *
      (((((((((l6 * l10 * 4.0 + l6 * l9 * 8.0) + l124_tmp * J_max * l5 * 12.0) +
             A_max * l3 * l9 * 4.0) +
            A_max * P_wayp * l9 * l10 * 24.0) +
           T * l5 * l21 * 12.0) +
          l144_tmp * T * l2_tmp * l9 * 12.0) +
         a_tmp * a_tmp * 3.0) +
        l18 * a_tmp * -12.0) +
       -(A_max *
         ((((((((((((((((l15_tmp * l5 * 3.0 + l3 * l9 * 6.0) + l5 * l10 * 3.0) +
                       rt_powd_snf(A_wayp, 3.0) * l10 * 2.0) +
                      l144_tmp * l17_tmp * 3.0) +
                     l184_im * l18 * 6.0) +
                    A_max * V_wayp * l21 * 6.0) +
                   -(l124_tmp * l20 * 6.0)) +
                  -(A_max * V_init * l21 * 6.0)) +
                 l184_im * V_wayp * l9 * 12.0) +
                P_init * l9 * l10 * 12.0) +
               J_max * T * l2_tmp * l9 * 6.0) +
              T * l10 * l18 * 6.0) +
             -(l184_im * V_init * l9 * 12.0)) +
            -(l120_tmp * l10 * 3.0)) +
           T * l7 * l21 * -6.0) +
          T * V_wayp * l9 * l10 * 12.0) *
         2.0));
  l5 = ((l156_tmp / 6912.0 + b_l156_tmp / 36.0) + -(l124 / 432.0)) +
       -(l12 / 3.0);
  l20 = l136 * l136;
  l138 = rt_powd_snf(l136, 3.0);
  l89 = l148 * l148;
  l169.re = ((((l89 * l89 * 27.0 + -(l138 * l89 * 4.0)) +
               rt_powd_snf(l5, 3.0) * 256.0) +
              l20 * l20 * l5 * 16.0) +
             l20 * (l5 * l5) * 128.0) +
            -(l136 * l89 * l5 * 144.0);
  l169.im = 0.0;
  coder::internal::scalar::b_sqrt(&l169);
  l103 = 1.7320508075688772 * l169.re;
  l120 = 1.7320508075688772 * l169.im;
  if (l120 == 0.0) {
    l90 = l103 / 18.0;
    l104 = 0.0;
  } else if (l103 == 0.0) {
    l90 = 0.0;
    l104 = l120 / 18.0;
  } else {
    l90 = l103 / 18.0;
    l104 = l120 / 18.0;
  }
  l11 = l136 * l5;
  l174.re = ((-(l138 / 27.0) + l89 / 2.0) + -(l11 * 1.3333333333333333)) + l90;
  l174.im = l104;
  l169 = coder::power(l174);
  dc = coder::b_power(l174);
  if (dc.im == 0.0) {
    l21 = 1.0 / dc.re;
    l88 = 0.0;
  } else if (dc.re == 0.0) {
    l21 = 0.0;
    l88 = -(1.0 / dc.im);
  } else {
    l187_im = std::abs(dc.re);
    l190_im = std::abs(dc.im);
    if (l187_im > l190_im) {
      l190_im = dc.im / dc.re;
      l184_im = dc.re + l190_im * dc.im;
      l21 = (l190_im * 0.0 + 1.0) / l184_im;
      l88 = (0.0 - l190_im) / l184_im;
    } else if (l190_im == l187_im) {
      if (dc.re > 0.0) {
        l190_im = 0.5;
      } else {
        l190_im = -0.5;
      }
      if (dc.im > 0.0) {
        l184_im = 0.5;
      } else {
        l184_im = -0.5;
      }
      l21 = (l190_im + 0.0 * l184_im) / l187_im;
      l88 = (0.0 * l190_im - l184_im) / l187_im;
    } else {
      l190_im = dc.re / dc.im;
      l184_im = dc.im + l190_im * dc.re;
      l21 = l190_im / l184_im;
      l88 = (l190_im * 0.0 - 1.0) / l184_im;
    }
  }
  l18 = l169.re * l169.re - l169.im * l169.im;
  l187_im = l169.re * l169.im;
  l10 = l187_im + l187_im;
  dc.re = ((-(l138 * 2.0) + l89 * 27.0) + -(l11 * 72.0)) + l103 * 3.0;
  dc.im = l120 * 3.0;
  coder::internal::scalar::b_sqrt(&dc);
  l9 = 3.0 * (2.4494897427831779 * l148 * dc.re);
  l7 = 3.0 * (2.4494897427831779 * l148 * dc.im);
  l104 = l136 * l169.re;
  l90 = l136 * l169.im;
  l174.re =
      (((((-(l156_tmp / 576.0) + -(b_l156_tmp / 3.0)) + l124 / 36.0) + l20) +
        l12 * 4.0) +
       l18 * 9.0) +
      l104 * 6.0;
  l174.im = l10 * 9.0 + l90 * 6.0;
  l169 = l174;
  coder::internal::scalar::b_sqrt(&l169);
  dc = coder::c_power(l174);
  if (dc.im == 0.0) {
    l124 = 1.0 / dc.re;
    l184_im = 0.0;
  } else if (dc.re == 0.0) {
    l124 = 0.0;
    l184_im = -(1.0 / dc.im);
  } else {
    l187_im = std::abs(dc.re);
    l190_im = std::abs(dc.im);
    if (l187_im > l190_im) {
      l190_im = dc.im / dc.re;
      l184_im = dc.re + l190_im * dc.im;
      l124 = (l190_im * 0.0 + 1.0) / l184_im;
      l184_im = (0.0 - l190_im) / l184_im;
    } else if (l190_im == l187_im) {
      if (dc.re > 0.0) {
        l190_im = 0.5;
      } else {
        l190_im = -0.5;
      }
      if (dc.im > 0.0) {
        l184_im = 0.5;
      } else {
        l184_im = -0.5;
      }
      l124 = (l190_im + 0.0 * l184_im) / l187_im;
      l184_im = (0.0 * l190_im - l184_im) / l187_im;
    } else {
      l190_im = dc.re / dc.im;
      l184_im = dc.im + l190_im * dc.re;
      l124 = l190_im / l184_im;
      l184_im = (l190_im * 0.0 - 1.0) / l184_im;
    }
  }
  l120 = 12.0 * (l5 * l169.re);
  l187_im = 12.0 * (l5 * l169.im);
  l138 = -9.0 * (l18 * l169.re - l10 * l169.im);
  l89 = -9.0 * (l18 * l169.im + l10 * l169.re);
  l103 = l21 * l169.re - l88 * l169.im;
  l11 = l21 * l169.im + l88 * l169.re;
  if (l11 == 0.0) {
    l103 /= 6.0;
    l190_im = 0.0;
  } else if (l103 == 0.0) {
    l103 = 0.0;
    l190_im = l11 / 6.0;
  } else {
    l103 /= 6.0;
    l190_im = l11 / 6.0;
  }
  l18 = 12.0 * (l104 * l169.re - l90 * l169.im);
  l10 = 12.0 * (l104 * l169.im + l90 * l169.re);
  l174.re = -(l20 * l169.re);
  l174.im = -(l20 * l169.im);
  dc.re = (((l9 + l174.re) + l120) + l138) + l18;
  dc.im = (((l7 + l174.im) + l187_im) + l89) + l10;
  coder::internal::scalar::b_sqrt(&dc);
  l12 = l21 * l124 - l88 * l184_im;
  l124 = l21 * l184_im + l88 * l124;
  l21 = l12 * dc.re - l124 * dc.im;
  l88 = l12 * dc.im + l124 * dc.re;
  if (l88 == 0.0) {
    l169.re = l21 / 6.0;
    l169.im = 0.0;
  } else if (l21 == 0.0) {
    l169.re = 0.0;
    l169.im = l88 / 6.0;
  } else {
    l169.re = l21 / 6.0;
    l169.im = l88 / 6.0;
  }
  dc.re = (((-l9 + l174.re) + l120) + l138) + l18;
  dc.im = (((-l7 + l174.im) + l187_im) + l89) + l10;
  coder::internal::scalar::b_sqrt(&dc);
  l21 = l12 * dc.re - l124 * dc.im;
  l88 = l12 * dc.im + l124 * dc.re;
  if (l88 == 0.0) {
    l174.re = l21 / 6.0;
    l174.im = 0.0;
  } else if (l21 == 0.0) {
    l174.re = 0.0;
    l174.im = l88 / 6.0;
  } else {
    l174.re = l21 / 6.0;
    l174.im = l88 / 6.0;
  }
  l124 = -l113 + -l103;
  t7[0].re = l124 - l169.re;
  t7[0].im = -l190_im - l169.im;
  t7[1].re = l124 + l169.re;
  t7[1].im = -l190_im + l169.im;
  l124 = -l113 + l103;
  t7[2].re = l124 - l174.re;
  t7[2].im = l190_im - l174.im;
  t7[3].re = l124 + l174.re;
  t7[3].im = l190_im + l174.im;
  l184_im = 1.0 / J_min;
  l138 = l144_tmp * 2.0;
  l169.re = l17_tmp * 2.0 + l19_tmp;
  l174.re = (l2_tmp + l4) + J_max * V_init * 2.0;
  l190_im = J_max * l184_im;
  l3 = A_max * (1.0 / J_min);
  l6 = -(1.0 / J_max * (A_init + -A_max));
  t[0].re = l6;
  t[0].im = 0.0;
  t[1].re = l6;
  t[1].im = 0.0;
  t[2].re = l6;
  t[2].im = 0.0;
  t[3].re = l6;
  t[3].im = 0.0;
  l90 = J_max * t7[0].re;
  l104 = J_max * t7[0].im;
  l2[0].re = l90;
  l2[0].im = l104;
  l187_im = A_wayp + -l90;
  l124 = l90 * l104;
  l103 = l187_im * -l104;
  l11 = ((l174.re + (l90 * l187_im - l104 * -l104) * 2.0) +
         (l90 * l90 - l104 * l104)) +
        l190_im * (l187_im * l187_im - -l104 * -l104);
  l124 = ((l90 * -l104 + l104 * l187_im) * 2.0 + (l124 + l124)) +
         l190_im * (l103 + l103);
  l12 = l184_im * ((l169.re - J_min * l11) + l16_tmp);
  l11 = l184_im * (0.0 - J_min * l124);
  if (l11 == 0.0) {
    l187_im = l12 / l138;
    l103 = 0.0;
  } else if (l12 == 0.0) {
    l187_im = 0.0;
    l103 = l11 / l138;
  } else {
    l187_im = l12 / l138;
    l103 = l11 / l138;
  }
  b_l7[0].re = l187_im;
  b_l7[0].im = l103;
  t[4].re = l187_im;
  t[4].im = l103;
  l90 = J_max * t7[1].re;
  l104 = J_max * t7[1].im;
  l2[1].re = l90;
  l2[1].im = l104;
  l187_im = A_wayp + -l90;
  l124 = l90 * l104;
  l103 = l187_im * -l104;
  l11 = ((l174.re + (l90 * l187_im - l104 * -l104) * 2.0) +
         (l90 * l90 - l104 * l104)) +
        l190_im * (l187_im * l187_im - -l104 * -l104);
  l124 = ((l90 * -l104 + l104 * l187_im) * 2.0 + (l124 + l124)) +
         l190_im * (l103 + l103);
  l12 = l184_im * ((l169.re - J_min * l11) + l16_tmp);
  l11 = l184_im * (0.0 - J_min * l124);
  if (l11 == 0.0) {
    l187_im = l12 / l138;
    l103 = 0.0;
  } else if (l12 == 0.0) {
    l187_im = 0.0;
    l103 = l11 / l138;
  } else {
    l187_im = l12 / l138;
    l103 = l11 / l138;
  }
  b_l7[1].re = l187_im;
  b_l7[1].im = l103;
  t[5].re = l187_im;
  t[5].im = l103;
  l90 = J_max * t7[2].re;
  l104 = J_max * t7[2].im;
  l2[2].re = l90;
  l2[2].im = l104;
  l187_im = A_wayp + -l90;
  l124 = l90 * l104;
  l103 = l187_im * -l104;
  l11 = ((l174.re + (l90 * l187_im - l104 * -l104) * 2.0) +
         (l90 * l90 - l104 * l104)) +
        l190_im * (l187_im * l187_im - -l104 * -l104);
  l124 = ((l90 * -l104 + l104 * l187_im) * 2.0 + (l124 + l124)) +
         l190_im * (l103 + l103);
  l12 = l184_im * ((l169.re - J_min * l11) + l16_tmp);
  l11 = l184_im * (0.0 - J_min * l124);
  if (l11 == 0.0) {
    l187_im = l12 / l138;
    l103 = 0.0;
  } else if (l12 == 0.0) {
    l187_im = 0.0;
    l103 = l11 / l138;
  } else {
    l187_im = l12 / l138;
    l103 = l11 / l138;
  }
  b_l7[2].re = l187_im;
  b_l7[2].im = l103;
  t[6].re = l187_im;
  t[6].im = l103;
  l90 = J_max * t7[3].re;
  l104 = J_max * t7[3].im;
  l187_im = A_wayp + -l90;
  l124 = l90 * l104;
  l103 = l187_im * -l104;
  l11 = ((l174.re + (l90 * l187_im - l104 * -l104) * 2.0) +
         (l90 * l90 - l104 * l104)) +
        l190_im * (l187_im * l187_im - -l104 * -l104);
  l124 = ((l90 * -l104 + l104 * l187_im) * 2.0 + (l124 + l124)) +
         l190_im * (l103 + l103);
  l12 = l184_im * ((l169.re - J_min * l11) + l16_tmp);
  l11 = l184_im * (0.0 - J_min * l124);
  if (l11 == 0.0) {
    l187_im = l12 / l138;
    l103 = 0.0;
  } else if (l12 == 0.0) {
    l187_im = 0.0;
    l103 = l11 / l138;
  } else {
    l187_im = l12 / l138;
    l103 = l11 / l138;
  }
  t[7].re = l187_im;
  t[7].im = l103;
  l169.re = ((l124_tmp - l103_tmp) + l144_tmp) + l15_tmp * T;
  t[8].re = -l3;
  t[8].im = 0.0;
  t[9].re = -l3;
  t[9].im = 0.0;
  t[10].re = -l3;
  t[10].im = 0.0;
  t[11].re = -l3;
  t[11].im = 0.0;
  l124 = A_wayp - l2[0].re;
  l12 = l184_im *
        (l169.re - l15_tmp * ((b_l7[0].re + t7[0].re) + l184_im * l124));
  l11 =
      l184_im *
      (0.0 - l15_tmp * ((b_l7[0].im + t7[0].im) + l184_im * (0.0 - l2[0].im)));
  if (l11 == 0.0) {
    t[12].re = l12 / J_max;
    t[12].im = 0.0;
  } else if (l12 == 0.0) {
    t[12].re = 0.0;
    t[12].im = l11 / J_max;
  } else {
    t[12].re = l12 / J_max;
    t[12].im = l11 / J_max;
  }
  if (0.0 - l2[0].im == 0.0) {
    t[16].re = l124 / J_min;
    t[16].im = 0.0;
  } else if (l124 == 0.0) {
    t[16].re = 0.0;
    t[16].im = (0.0 - l2[0].im) / J_min;
  } else {
    t[16].re = l124 / J_min;
    t[16].im = (0.0 - l2[0].im) / J_min;
  }
  t[20].re = 0.0;
  t[20].im = 0.0;
  t[24] = t7[0];
  l124 = A_wayp - l2[1].re;
  l12 = l184_im *
        (l169.re - l15_tmp * ((b_l7[1].re + t7[1].re) + l184_im * l124));
  l11 =
      l184_im *
      (0.0 - l15_tmp * ((b_l7[1].im + t7[1].im) + l184_im * (0.0 - l2[1].im)));
  if (l11 == 0.0) {
    t[13].re = l12 / J_max;
    t[13].im = 0.0;
  } else if (l12 == 0.0) {
    t[13].re = 0.0;
    t[13].im = l11 / J_max;
  } else {
    t[13].re = l12 / J_max;
    t[13].im = l11 / J_max;
  }
  if (0.0 - l2[1].im == 0.0) {
    t[17].re = l124 / J_min;
    t[17].im = 0.0;
  } else if (l124 == 0.0) {
    t[17].re = 0.0;
    t[17].im = (0.0 - l2[1].im) / J_min;
  } else {
    t[17].re = l124 / J_min;
    t[17].im = (0.0 - l2[1].im) / J_min;
  }
  t[21].re = 0.0;
  t[21].im = 0.0;
  t[25] = t7[1];
  l124 = A_wayp - l2[2].re;
  l12 = l184_im *
        (l169.re - l15_tmp * ((b_l7[2].re + t7[2].re) + l184_im * l124));
  l11 =
      l184_im *
      (0.0 - l15_tmp * ((b_l7[2].im + t7[2].im) + l184_im * (0.0 - l2[2].im)));
  if (l11 == 0.0) {
    t[14].re = l12 / J_max;
    t[14].im = 0.0;
  } else if (l12 == 0.0) {
    t[14].re = 0.0;
    t[14].im = l11 / J_max;
  } else {
    t[14].re = l12 / J_max;
    t[14].im = l11 / J_max;
  }
  if (0.0 - l2[2].im == 0.0) {
    t[18].re = l124 / J_min;
    t[18].im = 0.0;
  } else if (l124 == 0.0) {
    t[18].re = 0.0;
    t[18].im = (0.0 - l2[2].im) / J_min;
  } else {
    t[18].re = l124 / J_min;
    t[18].im = (0.0 - l2[2].im) / J_min;
  }
  t[22].re = 0.0;
  t[22].im = 0.0;
  t[26] = t7[2];
  l124 = A_wayp - l90;
  l12 = l184_im * (l169.re - l15_tmp * ((l187_im + t7[3].re) + l184_im * l124));
  l11 =
      l184_im * (0.0 - l15_tmp * ((l103 + t7[3].im) + l184_im * (0.0 - l104)));
  if (l11 == 0.0) {
    t[15].re = l12 / J_max;
    t[15].im = 0.0;
  } else if (l12 == 0.0) {
    t[15].re = 0.0;
    t[15].im = l11 / J_max;
  } else {
    t[15].re = l12 / J_max;
    t[15].im = l11 / J_max;
  }
  if (0.0 - l104 == 0.0) {
    t[19].re = l124 / J_min;
    t[19].im = 0.0;
  } else if (l124 == 0.0) {
    t[19].re = 0.0;
    t[19].im = (0.0 - l104) / J_min;
  } else {
    t[19].re = l124 / J_min;
    t[19].im = (0.0 - l104) / J_min;
  }
  t[23].re = 0.0;
  t[23].im = 0.0;
  t[27] = t7[3];
}

// End of code generation (abcdeg_TV_AVP.cpp)
