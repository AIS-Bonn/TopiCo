//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// abceg_O_AVP.cpp
//
// Code generation for function 'abceg_O_AVP'
//

// Include files
#include "abceg_O_AVP.h"
#include "power.h"
#include "rt_nonfinite.h"
#include "sqrt.h"
#include "topico_wrapper_data.h"
#include "topico_wrapper_rtwutil.h"
#include <cmath>

// Function Definitions
void abceg_O_AVP(double P_init, double V_init, double A_init, double P_wayp,
                 double V_wayp, double A_wayp, double A_max, double J_max,
                 double J_min, creal_T t[28])
{
  creal_T t7[4];
  creal_T dc;
  creal_T l146;
  creal_T l149;
  creal_T l151;
  creal_T l152;
  double a_tmp;
  double a_tmp_tmp;
  double b_a_tmp;
  double b_l109_tmp;
  double b_l136_tmp;
  double c_a_tmp;
  double c_l136_tmp;
  double l10;
  double l109;
  double l109_tmp;
  double l11_tmp;
  double l121;
  double l12_tmp;
  double l13;
  double l130;
  double l136;
  double l136_tmp;
  double l14;
  double l154_im;
  double l162_im;
  double l2_tmp;
  double l5_tmp;
  double l6;
  double l66_tmp;
  double l68_tmp;
  double l7_tmp;
  double l8;
  double l87_tmp;
  double l91;
  double l92;
  double l93;
  double l96;
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
  //  Generated on 28-Aug-2019 12:21:18
  l2_tmp = A_init * A_init;
  l5_tmp = A_max * A_max;
  l7_tmp = A_wayp * A_wayp;
  l8 = rt_powd_snf(A_wayp, 3.0);
  l10 = J_min * J_min;
  l11_tmp = J_max * J_max;
  l12_tmp = rt_powd_snf(J_max, 3.0);
  l14 = rt_powd_snf(J_max, 5.0);
  l6 = l5_tmp * l5_tmp;
  l13 = l11_tmp * l11_tmp;
  l162_im = A_max * A_wayp;
  l154_im = l162_im * J_min;
  l66_tmp = J_max * V_init;
  l68_tmp = J_min * V_wayp;
  l87_tmp = l10 * l11_tmp;
  l109_tmp = A_max * J_min;
  b_l109_tmp = A_wayp * J_min;
  l109 = ((((((((l8 * l12_tmp * 12.0 + l109_tmp * V_wayp * l12_tmp * 24.0) +
                -(b_l109_tmp * V_wayp * l12_tmp * 24.0)) +
               A_wayp * l5_tmp * l12_tmp * 12.0) +
              l109_tmp * l7_tmp * l11_tmp * 24.0) +
             A_wayp * V_wayp * l10 * l11_tmp * 24.0) +
            -(A_max * l7_tmp * l12_tmp * 24.0)) +
           -(J_min * l8 * l11_tmp * 12.0)) +
          -(b_l109_tmp * l5_tmp * l11_tmp * 12.0)) +
         -(A_max * V_wayp * l10 * l11_tmp * 24.0);
  l91 = 1.0 / ((rt_powd_snf(l11_tmp, 3.0) * 3.0 + -(J_min * l14 * 6.0)) +
               l10 * l13 * 3.0);
  l14 = ((((A_max * l14 * 8.0 + -(A_wayp * l14 * 12.0)) +
           b_l109_tmp * l13 * 24.0) +
          -(l109_tmp * l13 * 12.0)) +
         A_max * l10 * l12_tmp * 4.0) +
        -(A_wayp * l10 * l12_tmp * 12.0);
  l92 = l91 * l91;
  l93 = rt_powd_snf(l91, 3.0);
  l96 = l14 * l14;
  l99 = l91 * l14 / 4.0;
  a_tmp_tmp = J_min * l5_tmp;
  a_tmp = ((((((((l162_im * l13 * 24.0 + l68_tmp * l13 * 12.0) -
                 l5_tmp * l13 * 6.0) -
                l7_tmp * l13 * 18.0) -
               l154_im * l12_tmp * 36.0) +
              a_tmp_tmp * l12_tmp * 6.0) -
             V_wayp * l10 * l12_tmp * 12.0) +
            J_min * l7_tmp * l12_tmp * 30.0) +
           l162_im * l10 * l11_tmp * 12.0) -
          l7_tmp * l10 * l11_tmp * 12.0;
  b_a_tmp = l92 * l96 * 0.375 + l91 * a_tmp;
  l121 = b_a_tmp * b_a_tmp;
  l136 = l92 * l14;
  c_a_tmp =
      (l93 * rt_powd_snf(l14, 3.0) / 8.0 - l91 * l109) + l136 * a_tmp / 2.0;
  l130 = c_a_tmp * c_a_tmp;
  l136_tmp = l92 * l92 * (l96 * l96);
  b_l136_tmp = l93 * l96 * a_tmp;
  c_l136_tmp = l136 * l109;
  l13 =
      ((((((((((((((((l6 * l10 - l6 * l11_tmp) - l2_tmp * l2_tmp * l10 * 3.0) +
                    l7_tmp * l7_tmp * l11_tmp * 3.0) -
                   A_init * A_max * J_max * V_init * l10 * 24.0) +
                  l154_im * V_wayp * l11_tmp * 24.0) +
                 A_max * rt_powd_snf(A_init, 3.0) * l10 * 8.0) -
                A_max * l8 * l11_tmp * 8.0) +
               A_max * P_init * l10 * l11_tmp * 24.0) -
              A_max * P_wayp * l10 * l11_tmp * 24.0) +
             l66_tmp * l2_tmp * l10 * 12.0) +
            l66_tmp * l5_tmp * l10 * 12.0) -
           l68_tmp * l5_tmp * l11_tmp * 12.0) -
          l68_tmp * l7_tmp * l11_tmp * 12.0) -
         l2_tmp * l5_tmp * l10 * 6.0) +
        l5_tmp * l7_tmp * l11_tmp * 6.0) -
       l87_tmp * (V_init * V_init) * 12.0) +
      l87_tmp * (V_wayp * V_wayp) * 12.0;
  l136 = ((l136_tmp * 0.01171875 + b_l136_tmp / 16.0) + -(c_l136_tmp / 4.0)) +
         -l91 * l13;
  l14 = rt_powd_snf(b_a_tmp, 3.0);
  l146.re = ((((l130 * l130 * 27.0 + l130 * l14 * -4.0) +
               rt_powd_snf(l136, 3.0) * 256.0) +
              l121 * l121 * l136 * 16.0) +
             l121 * (l136 * l136) * 128.0) +
            l130 * l136 * b_a_tmp * -144.0;
  l146.im = 0.0;
  coder::internal::scalar::b_sqrt(&l146);
  l162_im = 1.7320508075688772 * l146.re;
  l93 = 1.7320508075688772 * l146.im;
  if (l93 == 0.0) {
    l96 = l162_im / 18.0;
    l109 = 0.0;
  } else if (l162_im == 0.0) {
    l96 = 0.0;
    l109 = l93 / 18.0;
  } else {
    l96 = l162_im / 18.0;
    l109 = l93 / 18.0;
  }
  l92 = l136 * b_a_tmp;
  l151.re =
      ((l14 * -0.037037037037037035 + l130 / 2.0) + l92 * -1.3333333333333333) +
      l96;
  l151.im = l109;
  l149.re = ((l14 * -2.0 + l130 * 27.0) + l92 * -72.0) + l162_im * 3.0;
  l149.im = l93 * 3.0;
  coder::internal::scalar::b_sqrt(&l149);
  l152 = coder::power(l151);
  dc = coder::b_power(l151);
  if (dc.im == 0.0) {
    l10 = 1.0 / dc.re;
    l154_im = 0.0;
  } else if (dc.re == 0.0) {
    l10 = 0.0;
    l154_im = -(1.0 / dc.im);
  } else {
    l162_im = std::abs(dc.re);
    l14 = std::abs(dc.im);
    if (l162_im > l14) {
      l14 = dc.im / dc.re;
      l93 = dc.re + l14 * dc.im;
      l10 = (l14 * 0.0 + 1.0) / l93;
      l154_im = (0.0 - l14) / l93;
    } else if (l14 == l162_im) {
      if (dc.re > 0.0) {
        l14 = 0.5;
      } else {
        l14 = -0.5;
      }
      if (dc.im > 0.0) {
        a_tmp = 0.5;
      } else {
        a_tmp = -0.5;
      }
      l10 = (l14 + 0.0 * a_tmp) / l162_im;
      l154_im = (0.0 * l14 - a_tmp) / l162_im;
    } else {
      l14 = dc.re / dc.im;
      l93 = dc.im + l14 * dc.re;
      l10 = l14 / l93;
      l154_im = (l14 * 0.0 - 1.0) / l93;
    }
  }
  l151.re = l152.re * l152.re - l152.im * l152.im;
  l92 = l152.re * l152.im;
  l151.im = l92 + l92;
  l146.re =
      (((((-(l136_tmp * 0.140625) + b_l136_tmp * -0.75) + c_l136_tmp * 3.0) +
         l121) +
        l91 * l13 * 12.0) +
       l151.re * 9.0) +
      b_a_tmp * l152.re * 6.0;
  l146.im = l151.im * 9.0 + b_a_tmp * l152.im * 6.0;
  dc = l146;
  coder::internal::scalar::b_sqrt(&dc);
  l146 = coder::c_power(l146);
  if (l146.im == 0.0) {
    l13 = 1.0 / l146.re;
    l162_im = 0.0;
  } else if (l146.re == 0.0) {
    l13 = 0.0;
    l162_im = -(1.0 / l146.im);
  } else {
    l162_im = std::abs(l146.re);
    l14 = std::abs(l146.im);
    if (l162_im > l14) {
      l14 = l146.im / l146.re;
      l93 = l146.re + l14 * l146.im;
      l13 = (l14 * 0.0 + 1.0) / l93;
      l162_im = (0.0 - l14) / l93;
    } else if (l14 == l162_im) {
      if (l146.re > 0.0) {
        l14 = 0.5;
      } else {
        l14 = -0.5;
      }
      if (l146.im > 0.0) {
        a_tmp = 0.5;
      } else {
        a_tmp = -0.5;
      }
      l13 = (l14 + 0.0 * a_tmp) / l162_im;
      l162_im = (0.0 * l14 - a_tmp) / l162_im;
    } else {
      l14 = l146.re / l146.im;
      l93 = l146.im + l14 * l146.re;
      l13 = l14 / l93;
      l162_im = (l14 * 0.0 - 1.0) / l93;
    }
  }
  l8 = l136 * dc.re * 12.0;
  l93 = dc.re * l151.re;
  a_tmp = dc.re * l151.im;
  l151.re = -9.0 * l93;
  l151.im = -9.0 * a_tmp;
  l14 = l10 * dc.re;
  l92 = l154_im * dc.re;
  if (l92 == 0.0) {
    l109 = l14 / 6.0;
    l96 = 0.0;
  } else if (l14 == 0.0) {
    l109 = 0.0;
    l96 = l92 / 6.0;
  } else {
    l109 = l14 / 6.0;
    l96 = l92 / 6.0;
  }
  l146.re = 12.0 * (b_a_tmp * (dc.re * l152.re));
  l146.im = 12.0 * (b_a_tmp * (dc.re * l152.im));
  l92 = -(l121 * dc.re);
  l136 = c_a_tmp * (2.4494897427831779 * l149.re);
  dc.re = (((l136 * -3.0 + l92) + l8) + l151.re) + l146.re;
  a_tmp = c_a_tmp * (2.4494897427831779 * l149.im);
  dc.im = (a_tmp * -3.0 + l151.im) + l146.im;
  coder::internal::scalar::b_sqrt(&dc);
  l93 = l10 * l13 - l154_im * l162_im;
  l13 = l10 * l162_im + l154_im * l13;
  l10 = l93 * dc.re - l13 * dc.im;
  l154_im = l93 * dc.im + l13 * dc.re;
  if (l154_im == 0.0) {
    l162_im = l10 / 6.0;
    l14 = 0.0;
  } else if (l10 == 0.0) {
    l162_im = 0.0;
    l14 = l154_im / 6.0;
  } else {
    l162_im = l10 / 6.0;
    l14 = l154_im / 6.0;
  }
  dc.re = (((l136 * 3.0 + l92) + l8) + l151.re) + l146.re;
  dc.im = (a_tmp * 3.0 + l151.im) + l146.im;
  coder::internal::scalar::b_sqrt(&dc);
  l10 = l93 * dc.re - l13 * dc.im;
  l154_im = l93 * dc.im + l13 * dc.re;
  if (l154_im == 0.0) {
    l146.re = l10 / 6.0;
    l146.im = 0.0;
  } else if (l10 == 0.0) {
    l146.re = 0.0;
    l146.im = l154_im / 6.0;
  } else {
    l146.re = l10 / 6.0;
    l146.im = l154_im / 6.0;
  }
  l136 = -l99 + -l109;
  t7[0].re = l136 - l146.re;
  t7[0].im = -l96 - l146.im;
  t7[1].re = l136 + l146.re;
  t7[1].im = -l96 + l146.im;
  l136 = -l99 + l109;
  t7[2].re = l136 - l162_im;
  t7[2].im = l96 - l14;
  t7[3].re = l136 + l162_im;
  t7[3].im = l96 + l14;
  l162_im = l109_tmp * J_max;
  l6 = -(1.0 / J_max * (A_init + -A_max));
  l146.re = ((a_tmp_tmp - J_max * l5_tmp) - l2_tmp * J_min) + l7_tmp * J_max;
  l92 = J_min * J_max;
  l151.re = l92 * V_init * 2.0;
  l149.re = l92 * V_wayp * 2.0;
  l13 = A_wayp * l11_tmp;
  a_tmp = J_min * l11_tmp;
  l93 = b_l109_tmp * J_max;
  l152.re = A_max - A_wayp;
  t[0].re = l6;
  t[0].im = 0.0;
  t[1].re = l6;
  t[1].im = 0.0;
  t[2].re = l6;
  t[2].im = 0.0;
  t[3].re = l6;
  t[3].im = 0.0;
  l96 = t7[0].re * t7[0].re - t7[0].im * t7[0].im;
  l14 = t7[0].re * t7[0].im;
  l109 = l14 + l14;
  l14 = ((((((l146.re + l12_tmp * l96) + l151.re) - l149.re) -
           l13 * t7[0].re * 2.0) -
          a_tmp * l96) +
         l93 * t7[0].re * 2.0) *
        -0.5;
  l92 = (((l12_tmp * l109 - l13 * t7[0].im * 2.0) - a_tmp * l109) +
         l93 * t7[0].im * 2.0) *
        -0.5;
  if (l92 == 0.0) {
    t[4].re = l14 / l162_im;
    t[4].im = 0.0;
  } else if (l14 == 0.0) {
    t[4].re = 0.0;
    t[4].im = l92 / l162_im;
  } else {
    t[4].re = l14 / l162_im;
    t[4].im = l92 / l162_im;
  }
  l14 = -(l152.re + J_max * t7[0].re);
  l92 = -(J_max * t7[0].im);
  if (l92 == 0.0) {
    t[8].re = l14 / J_min;
    t[8].im = 0.0;
  } else if (l14 == 0.0) {
    t[8].re = 0.0;
    t[8].im = l92 / J_min;
  } else {
    t[8].re = l14 / J_min;
    t[8].im = l92 / J_min;
  }
  t[12].re = 0.0;
  t[12].im = 0.0;
  t[16].re = 0.0;
  t[16].im = 0.0;
  t[20].re = 0.0;
  t[20].im = 0.0;
  t[24] = t7[0];
  l96 = t7[1].re * t7[1].re - t7[1].im * t7[1].im;
  l14 = t7[1].re * t7[1].im;
  l109 = l14 + l14;
  l14 = ((((((l146.re + l12_tmp * l96) + l151.re) - l149.re) -
           l13 * t7[1].re * 2.0) -
          a_tmp * l96) +
         l93 * t7[1].re * 2.0) *
        -0.5;
  l92 = (((l12_tmp * l109 - l13 * t7[1].im * 2.0) - a_tmp * l109) +
         l93 * t7[1].im * 2.0) *
        -0.5;
  if (l92 == 0.0) {
    t[5].re = l14 / l162_im;
    t[5].im = 0.0;
  } else if (l14 == 0.0) {
    t[5].re = 0.0;
    t[5].im = l92 / l162_im;
  } else {
    t[5].re = l14 / l162_im;
    t[5].im = l92 / l162_im;
  }
  l14 = -(l152.re + J_max * t7[1].re);
  l92 = -(J_max * t7[1].im);
  if (l92 == 0.0) {
    t[9].re = l14 / J_min;
    t[9].im = 0.0;
  } else if (l14 == 0.0) {
    t[9].re = 0.0;
    t[9].im = l92 / J_min;
  } else {
    t[9].re = l14 / J_min;
    t[9].im = l92 / J_min;
  }
  t[13].re = 0.0;
  t[13].im = 0.0;
  t[17].re = 0.0;
  t[17].im = 0.0;
  t[21].re = 0.0;
  t[21].im = 0.0;
  t[25] = t7[1];
  l96 = t7[2].re * t7[2].re - t7[2].im * t7[2].im;
  l14 = t7[2].re * t7[2].im;
  l109 = l14 + l14;
  l14 = ((((((l146.re + l12_tmp * l96) + l151.re) - l149.re) -
           l13 * t7[2].re * 2.0) -
          a_tmp * l96) +
         l93 * t7[2].re * 2.0) *
        -0.5;
  l92 = (((l12_tmp * l109 - l13 * t7[2].im * 2.0) - a_tmp * l109) +
         l93 * t7[2].im * 2.0) *
        -0.5;
  if (l92 == 0.0) {
    t[6].re = l14 / l162_im;
    t[6].im = 0.0;
  } else if (l14 == 0.0) {
    t[6].re = 0.0;
    t[6].im = l92 / l162_im;
  } else {
    t[6].re = l14 / l162_im;
    t[6].im = l92 / l162_im;
  }
  l14 = -(l152.re + J_max * t7[2].re);
  l92 = -(J_max * t7[2].im);
  if (l92 == 0.0) {
    t[10].re = l14 / J_min;
    t[10].im = 0.0;
  } else if (l14 == 0.0) {
    t[10].re = 0.0;
    t[10].im = l92 / J_min;
  } else {
    t[10].re = l14 / J_min;
    t[10].im = l92 / J_min;
  }
  t[14].re = 0.0;
  t[14].im = 0.0;
  t[18].re = 0.0;
  t[18].im = 0.0;
  t[22].re = 0.0;
  t[22].im = 0.0;
  t[26] = t7[2];
  l96 = t7[3].re * t7[3].re - t7[3].im * t7[3].im;
  l14 = t7[3].re * t7[3].im;
  l109 = l14 + l14;
  l14 = ((((((l146.re + l12_tmp * l96) + l151.re) - l149.re) -
           l13 * t7[3].re * 2.0) -
          a_tmp * l96) +
         l93 * t7[3].re * 2.0) *
        -0.5;
  l92 = (((l12_tmp * l109 - l13 * t7[3].im * 2.0) - a_tmp * l109) +
         l93 * t7[3].im * 2.0) *
        -0.5;
  if (l92 == 0.0) {
    t[7].re = l14 / l162_im;
    t[7].im = 0.0;
  } else if (l14 == 0.0) {
    t[7].re = 0.0;
    t[7].im = l92 / l162_im;
  } else {
    t[7].re = l14 / l162_im;
    t[7].im = l92 / l162_im;
  }
  l14 = -(l152.re + J_max * t7[3].re);
  l92 = -(J_max * t7[3].im);
  if (l92 == 0.0) {
    t[11].re = l14 / J_min;
    t[11].im = 0.0;
  } else if (l14 == 0.0) {
    t[11].re = 0.0;
    t[11].im = l92 / J_min;
  } else {
    t[11].re = l14 / J_min;
    t[11].im = l92 / J_min;
  }
  t[15].re = 0.0;
  t[15].im = 0.0;
  t[19].re = 0.0;
  t[19].im = 0.0;
  t[23].re = 0.0;
  t[23].im = 0.0;
  t[27] = t7[3];
}

// End of code generation (abceg_O_AVP.cpp)
