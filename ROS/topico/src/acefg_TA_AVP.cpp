//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// acefg_TA_AVP.cpp
//
// Code generation for function 'acefg_TA_AVP'
//

// Include files
#include "acefg_TA_AVP.h"
#include "power.h"
#include "rt_nonfinite.h"
#include "sqrt.h"
#include "topico_wrapper_data.h"
#include "topico_wrapper_rtwutil.h"
#include <cmath>

// Function Definitions
void acefg_TA_AVP(double P_init, double V_init, double A_init, double P_wayp,
                  double V_wayp, double A_wayp, double J_max, double J_min,
                  double T, creal_T t[28])
{
  creal_T b_l11[4];
  creal_T l2[4];
  creal_T t6[4];
  creal_T dc;
  creal_T l177;
  creal_T l182;
  creal_T l183;
  double b_l126_tmp;
  double b_l130_tmp;
  double b_l160_tmp;
  double c_l126_tmp;
  double c_l130_tmp;
  double c_l160_tmp;
  double d;
  double d1;
  double d_l126_tmp;
  double d_l160_tmp;
  double e_l126_tmp;
  double l10;
  double l11;
  double l12;
  double l126;
  double l126_tmp;
  double l127;
  double l13;
  double l130;
  double l130_tmp;
  double l130_tmp_tmp;
  double l131;
  double l131_tmp;
  double l14;
  double l144;
  double l144_im_tmp;
  double l145;
  double l15;
  double l152;
  double l153;
  double l16;
  double l160_tmp;
  double l164;
  double l17;
  double l18;
  double l184_im;
  double l184_re;
  double l188_im;
  double l188_re;
  double l195_im;
  double l198_re;
  double l20;
  double l2_tmp;
  double l3;
  double l39;
  double l39_tmp;
  double l5_tmp;
  double l6;
  double l8_tmp;
  double l9_tmp;
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
  l5_tmp = A_wayp * A_wayp;
  l6 = rt_powd_snf(A_wayp, 3.0);
  l8_tmp = J_min * J_min;
  l9_tmp = J_max * J_max;
  l10 = rt_powd_snf(J_max, 3.0);
  l11 = T * T;
  l12 = rt_powd_snf(T, 3.0);
  l14 = V_init * V_init;
  l15 = V_wayp * V_wayp;
  l13 = l11 * l11;
  l16 = 1.0 / l8_tmp;
  l18 = 1.0 / l9_tmp;
  l39_tmp = A_wayp * J_max;
  l39 = l39_tmp * l8_tmp * 4.0;
  l17 = l16 * l16;
  l20 = l18 * l18;
  l184_im = A_init * J_min;
  l126_tmp = A_wayp * J_min;
  b_l126_tmp = A_init * J_max;
  c_l126_tmp = b_l126_tmp * l8_tmp;
  d_l126_tmp = l184_im * l9_tmp;
  e_l126_tmp = l126_tmp * l9_tmp;
  l126 = ((((d_l126_tmp * 4.0 + c_l126_tmp * 4.0) + J_min * T * l10 * 4.0) +
           -(e_l126_tmp * 4.0)) +
          -l39) +
         T * l8_tmp * l9_tmp * 4.0;
  l130_tmp = J_min * J_max;
  b_l130_tmp = J_min * l10;
  l130_tmp_tmp = A_init * A_wayp;
  c_l130_tmp = l130_tmp_tmp * J_min * J_max;
  l130 = ((((l130_tmp * l2_tmp * 6.0 + l130_tmp * l5_tmp * 6.0) +
            -(c_l130_tmp * 12.0)) +
           b_l130_tmp * l11 * 6.0) +
          l184_im * T * l9_tmp * 12.0) +
         -(l126_tmp * T * l9_tmp * 12.0);
  l127 = l126 * l126;
  l131_tmp = l16 * l18;
  l131 = l131_tmp * l126 / 12.0;
  d = l184_im * J_max;
  l144_im_tmp = l126_tmp * J_max;
  d1 = J_max * T;
  l164 = A_init * P_wayp;
  l188_im = A_wayp * P_init;
  l188_re = A_init * P_init;
  l160_tmp = A_wayp * P_wayp;
  l184_im = V_init * V_wayp;
  l184_re = A_wayp * T * V_init;
  b_l160_tmp = A_init * T * V_wayp;
  c_l160_tmp = J_max * V_init;
  d_l160_tmp = J_max * V_wayp;
  l195_im = l17 * l20;
  l144 = l195_im * l127 / 24.0 + -(l131_tmp * l130 / 3.0);
  l198_re = rt_powd_snf(l16, 3.0) * rt_powd_snf(l18, 3.0);
  l152 = l198_re * rt_powd_snf(l126, 3.0) / 216.0 +
         -(l195_im * l126 * l130 / 18.0);
  l145 = l144 * l144;
  l126 = rt_powd_snf(l144, 3.0);
  l153 = l152 * l152;
  l17 = l17 * l17 * (l20 * l20) * (l127 * l127);
  l198_re = l198_re * l127 * l130;
    l18 = l131_tmp * (((((((((((((((((((((((((((((((((((((((((((((((((((((((((l2_tmp * l2_tmp + l5_tmp * l5_tmp) + -(A_init * l6 * 4.0)) + -(A_wayp * l3 * 4.0)) + d * P_init * 48.0) + l144_im_tmp * P_wayp * 48.0) + d1 * l3 * 4.0) + l130_tmp * V_init * V_wayp * 48.0) + -(d * P_wayp * 48.0)) + -(l144_im_tmp * P_init * 48.0)) + l164 * l8_tmp * 24.0) + l188_im * l8_tmp * 24.0) + l164 * l9_tmp * 24.0) + l188_im * l9_tmp * 24.0) + -(d1 * l6 * 4.0)) + l2_tmp * l5_tmp * 6.0) + P_wayp * T * l10 * 24.0) + d * T * V_wayp * 48.0) + b_l130_tmp * l13 * 2.0) + -(l188_re * l8_tmp * 24.0)) + -(l188_re * l9_tmp * 24.0)) + -(l160_tmp * l8_tmp * 24.0)) + -(l160_tmp * l9_tmp * 24.0)) + -(l130_tmp * l14 * 24.0)) + -(l130_tmp * l15 * 24.0)) + -(P_init * T * l10 * 24.0)) + -(l184_im * l8_tmp * 24.0)) + -(l184_im * l9_tmp * 24.0)) + l8_tmp * l14 * 12.0) + l9_tmp * l14 * 12.0) + l8_tmp * l15 * 12.0) + l9_tmp * l15 * 12.0) + -(l144_im_tmp * T * V_init * 48.0)) + b_l126_tmp * T * l5_tmp * 12.0) + l184_re * l8_tmp * 24.0) + l184_re * l9_tmp * 24.0) + J_min * P_init * T * l9_tmp * 48.0) + J_max * P_wayp * T * l8_tmp * 24.0) + l12 * l39) + -(l39_tmp * T * l2_tmp * 12.0)) + -(b_l160_tmp * l8_tmp * 24.0)) + -(b_l160_tmp * l9_tmp * 24.0)) + -(J_max * P_init * T * l8_tmp * 24.0)) + -(J_min * P_wayp * T * l9_tmp * 48.0)) + -(V_init * l10 * l11 * 12.0)) + -(V_wayp * l10 * l11 * 12.0)) + -(c_l130_tmp * l11 * 24.0)) + l130_tmp_tmp * l8_tmp * l11 * 12.0) + c_l126_tmp * l12 * -4.0) + d_l126_tmp * l12 * 8.0) + J_min * V_init * l9_tmp * l11 * 24.0) + J_min * V_wayp * l9_tmp * l11 * 24.0) + l2_tmp * l9_tmp * l11 * 6.0) + l5_tmp * l9_tmp * l11 * 6.0) + -(e_l126_tmp * l12 * 8.0)) + -(c_l160_tmp * l8_tmp * l11 * 12.0)) + -(d_l160_tmp * l8_tmp * l11 * 12.0)) + -(l8_tmp * l9_tmp * l13));
    l164 = (l17 / 6912.0 + -(l198_re / 432.0)) + l18 / 3.0;
    l177.re = ((((l153 * l153 * 27.0 + -(l126 * l153 * 4.0)) +
                 rt_powd_snf(l164, 3.0) * 256.0) +
                l145 * l145 * l164 * 16.0) +
               l145 * (l164 * l164) * 128.0) +
              -(l144 * l153 * l164 * 144.0);
    l177.im = 0.0;
    coder::internal::scalar::b_sqrt(&l177);
    l11 = 1.7320508075688772 * l177.re;
    l130 = 1.7320508075688772 * l177.im;
    if (l130 == 0.0) {
      l188_re = l11 / 18.0;
      l188_im = 0.0;
    } else if (l11 == 0.0) {
      l188_re = 0.0;
      l188_im = l130 / 18.0;
    } else {
      l188_re = l11 / 18.0;
      l188_im = l130 / 18.0;
    }
    l16 = l144 * l164;
    l182.re =
        ((-(l126 / 27.0) + l153 / 2.0) + -(l16 * 1.3333333333333333)) + l188_re;
    l182.im = l188_im;
    l183 = coder::power(l182);
    dc = coder::b_power(l182);
    if (dc.im == 0.0) {
      l6 = 1.0 / dc.re;
      l10 = 0.0;
    } else if (dc.re == 0.0) {
      l6 = 0.0;
      l10 = -(1.0 / dc.im);
    } else {
      b_l160_tmp = std::abs(dc.re);
      l195_im = std::abs(dc.im);
      if (b_l160_tmp > l195_im) {
        l160_tmp = dc.im / dc.re;
        l195_im = dc.re + l160_tmp * dc.im;
        l6 = (l160_tmp * 0.0 + 1.0) / l195_im;
        l10 = (0.0 - l160_tmp) / l195_im;
      } else if (l195_im == b_l160_tmp) {
        if (dc.re > 0.0) {
          l160_tmp = 0.5;
        } else {
          l160_tmp = -0.5;
        }
        if (dc.im > 0.0) {
          l195_im = 0.5;
        } else {
          l195_im = -0.5;
        }
        l6 = (l160_tmp + 0.0 * l195_im) / b_l160_tmp;
        l10 = (0.0 * l160_tmp - l195_im) / b_l160_tmp;
      } else {
        l160_tmp = dc.re / dc.im;
        l195_im = dc.im + l160_tmp * dc.re;
        l6 = l160_tmp / l195_im;
        l10 = (l160_tmp * 0.0 - 1.0) / l195_im;
      }
    }
    l184_re = l183.re * l183.re - l183.im * l183.im;
    l195_im = l183.re * l183.im;
    l184_im = l195_im + l195_im;
    dc.re = ((-(l126 * 2.0) + l153 * 27.0) + -(l16 * 72.0)) + l11 * 3.0;
    dc.im = l130 * 3.0;
    coder::internal::scalar::b_sqrt(&dc);
    l188_re = 3.0 * (2.4494897427831779 * l152 * dc.re);
    l188_im = 3.0 * (2.4494897427831779 * l152 * dc.im);
    l20 = l144 * l183.re;
    l144_im_tmp = l144 * l183.im;
    l182.re = ((((-(l17 / 576.0) + l198_re / 36.0) + l145) + -(l18 * 4.0)) +
               l184_re * 9.0) +
              l20 * 6.0;
    l182.im = l184_im * 9.0 + l144_im_tmp * 6.0;
    l177 = l182;
    coder::internal::scalar::b_sqrt(&l177);
    dc = coder::c_power(l182);
    if (dc.im == 0.0) {
      l127 = 1.0 / dc.re;
      l18 = 0.0;
    } else if (dc.re == 0.0) {
      l127 = 0.0;
      l18 = -(1.0 / dc.im);
    } else {
      b_l160_tmp = std::abs(dc.re);
      l195_im = std::abs(dc.im);
      if (b_l160_tmp > l195_im) {
        l160_tmp = dc.im / dc.re;
        l195_im = dc.re + l160_tmp * dc.im;
        l127 = (l160_tmp * 0.0 + 1.0) / l195_im;
        l18 = (0.0 - l160_tmp) / l195_im;
      } else if (l195_im == b_l160_tmp) {
        if (dc.re > 0.0) {
          l160_tmp = 0.5;
        } else {
          l160_tmp = -0.5;
        }
        if (dc.im > 0.0) {
          l195_im = 0.5;
        } else {
          l195_im = -0.5;
        }
        l127 = (l160_tmp + 0.0 * l195_im) / b_l160_tmp;
        l18 = (0.0 * l160_tmp - l195_im) / b_l160_tmp;
      } else {
        l160_tmp = dc.re / dc.im;
        l195_im = dc.im + l160_tmp * dc.re;
        l127 = l160_tmp / l195_im;
        l18 = (l160_tmp * 0.0 - 1.0) / l195_im;
      }
    }
    l17 = 12.0 * (l164 * l177.re);
    l195_im = 12.0 * (l164 * l177.im);
    l164 = -9.0 * (l184_re * l177.re - l184_im * l177.im);
    l126 = -9.0 * (l184_re * l177.im + l184_im * l177.re);
    l16 = l6 * l177.re - l10 * l177.im;
    l184_im = l6 * l177.im + l10 * l177.re;
    if (l184_im == 0.0) {
      l198_re = l16 / 6.0;
      l16 = 0.0;
    } else if (l16 == 0.0) {
      l198_re = 0.0;
      l16 = l184_im / 6.0;
    } else {
      l198_re = l16 / 6.0;
      l16 = l184_im / 6.0;
    }
    l184_re = 12.0 * (l20 * l177.re - l144_im_tmp * l177.im);
    l184_im = 12.0 * (l20 * l177.im + l144_im_tmp * l177.re);
    l182.re = -(l145 * l177.re);
    l182.im = -(l145 * l177.im);
    dc.re = (((l188_re + l182.re) + l17) + l164) + l184_re;
    dc.im = (((l188_im + l182.im) + l195_im) + l126) + l184_im;
    coder::internal::scalar::b_sqrt(&dc);
    l130 = l6 * l127 - l10 * l18;
    l18 = l6 * l18 + l10 * l127;
    l6 = l130 * dc.re - l18 * dc.im;
    l10 = l130 * dc.im + l18 * dc.re;
    if (l10 == 0.0) {
      l183.re = l6 / 6.0;
      l183.im = 0.0;
    } else if (l6 == 0.0) {
      l183.re = 0.0;
      l183.im = l10 / 6.0;
    } else {
      l183.re = l6 / 6.0;
      l183.im = l10 / 6.0;
    }
    dc.re = (((-l188_re + l182.re) + l17) + l164) + l184_re;
    dc.im = (((-l188_im + l182.im) + l195_im) + l126) + l184_im;
    coder::internal::scalar::b_sqrt(&dc);
    l6 = l130 * dc.re - l18 * dc.im;
    l10 = l130 * dc.im + l18 * dc.re;
    if (l10 == 0.0) {
      l182.re = l6 / 6.0;
      l182.im = 0.0;
    } else if (l6 == 0.0) {
      l182.re = 0.0;
      l182.im = l10 / 6.0;
    } else {
      l182.re = l6 / 6.0;
      l182.im = l10 / 6.0;
    }
    d = l131 + -l198_re;
    t6[0].re = d - l182.re;
    t6[0].im = -l16 - l182.im;
    t6[1].re = d + l182.re;
    t6[1].im = -l16 + l182.im;
    d = l131 + l198_re;
    t6[2].re = d - l183.re;
    t6[2].im = l16 - l183.im;
    t6[3].re = d + l183.re;
    t6[3].im = l16 + l183.im;
    l144_im_tmp = J_min + -J_max;
    l20 = l144_im_tmp * l144_im_tmp;
    l177.re = -l2_tmp * l20 + l5_tmp * l20;
    l182.re = c_l160_tmp * l20 * 2.0;
    l183.re = d_l160_tmp * l20 * 2.0;
    l127 = J_min * -J_max;
    l17 = l126_tmp * l144_im_tmp;
    l198_re = l39_tmp * l144_im_tmp;
    l3 = l9_tmp * l144_im_tmp;
    l164 = l130_tmp * l144_im_tmp;
    l188_re = J_max * t6[0].re;
    l188_im = J_max * t6[0].im;
    l126 = (A_init + d1) + -A_wayp;
    l195_im = l126 + -l188_re;
    l18 = l195_im * l195_im - -l188_im * -l188_im;
    l16 = l195_im * -l188_im;
    l16 += l16;
    l130 = ((((((l177.re + l8_tmp * l18) + l182.re) - l183.re) +
              l20 * (A_wayp * l188_re) * 2.0) +
             l127 * l18) +
            l17 * l195_im * 2.0) -
           l198_re * l195_im * 2.0;
    l184_im = (((l8_tmp * l16 + l20 * (A_wayp * l188_im) * 2.0) + l127 * l16) +
               l17 * -l188_im * 2.0) -
              l198_re * -l188_im * 2.0;
    l18 = (l20 * (J_max * l188_re) * 2.0 - l3 * l195_im * 2.0) +
          l164 * l195_im * 2.0;
    l16 = (l20 * (J_max * l188_im) * 2.0 - l3 * -l188_im * 2.0) +
          l164 * -l188_im * 2.0;
    if (l16 == 0.0) {
      if (l184_im == 0.0) {
        l188_re = l130 / l18;
        l188_im = 0.0;
      } else if (l130 == 0.0) {
        l188_re = 0.0;
        l188_im = l184_im / l18;
      } else {
        l188_re = l130 / l18;
        l188_im = l184_im / l18;
      }
    } else if (l18 == 0.0) {
      if (l130 == 0.0) {
        l188_re = l184_im / l16;
        l188_im = 0.0;
      } else if (l184_im == 0.0) {
        l188_re = 0.0;
        l188_im = -(l130 / l16);
      } else {
        l188_re = l184_im / l16;
        l188_im = -(l130 / l16);
      }
    } else {
      b_l160_tmp = std::abs(l18);
      l195_im = std::abs(l16);
      if (b_l160_tmp > l195_im) {
        l160_tmp = l16 / l18;
        l195_im = l18 + l160_tmp * l16;
        l188_re = (l130 + l160_tmp * l184_im) / l195_im;
        l188_im = (l184_im - l160_tmp * l130) / l195_im;
      } else if (l195_im == b_l160_tmp) {
        if (l18 > 0.0) {
          l160_tmp = 0.5;
        } else {
          l160_tmp = -0.5;
        }
        if (l16 > 0.0) {
          l195_im = 0.5;
        } else {
          l195_im = -0.5;
        }
        l188_re = (l130 * l160_tmp + l184_im * l195_im) / b_l160_tmp;
        l188_im = (l184_im * l160_tmp - l130 * l195_im) / b_l160_tmp;
      } else {
        l160_tmp = l18 / l16;
        l195_im = l16 + l160_tmp * l18;
        l188_re = (l160_tmp * l130 + l184_im) / l195_im;
        l188_im = (l160_tmp * l184_im - l130) / l195_im;
      }
    }
    b_l11[0].re = l188_re;
    b_l11[0].im = l188_im;
    l11 = A_init - A_wayp;
    l130 = -((l11 + -J_max * t6[0].re) + d1);
    l184_im = -(-J_max * t6[0].im);
    if (l184_im == 0.0) {
      l2[0].re = l130 / l144_im_tmp;
      l2[0].im = 0.0;
    } else if (l130 == 0.0) {
      l2[0].re = 0.0;
      l2[0].im = l184_im / l144_im_tmp;
    } else {
      l2[0].re = l130 / l144_im_tmp;
      l2[0].im = l184_im / l144_im_tmp;
    }
    l188_re = J_max * t6[1].re;
    l188_im = J_max * t6[1].im;
    l195_im = l126 + -l188_re;
    l18 = l195_im * l195_im - -l188_im * -l188_im;
    l16 = l195_im * -l188_im;
    l16 += l16;
    l130 = ((((((l177.re + l8_tmp * l18) + l182.re) - l183.re) +
              l20 * (A_wayp * l188_re) * 2.0) +
             l127 * l18) +
            l17 * l195_im * 2.0) -
           l198_re * l195_im * 2.0;
    l184_im = (((l8_tmp * l16 + l20 * (A_wayp * l188_im) * 2.0) + l127 * l16) +
               l17 * -l188_im * 2.0) -
              l198_re * -l188_im * 2.0;
    l18 = (l20 * (J_max * l188_re) * 2.0 - l3 * l195_im * 2.0) +
          l164 * l195_im * 2.0;
    l16 = (l20 * (J_max * l188_im) * 2.0 - l3 * -l188_im * 2.0) +
          l164 * -l188_im * 2.0;
    if (l16 == 0.0) {
      if (l184_im == 0.0) {
        l188_re = l130 / l18;
        l188_im = 0.0;
      } else if (l130 == 0.0) {
        l188_re = 0.0;
        l188_im = l184_im / l18;
      } else {
        l188_re = l130 / l18;
        l188_im = l184_im / l18;
      }
    } else if (l18 == 0.0) {
      if (l130 == 0.0) {
        l188_re = l184_im / l16;
        l188_im = 0.0;
      } else if (l184_im == 0.0) {
        l188_re = 0.0;
        l188_im = -(l130 / l16);
      } else {
        l188_re = l184_im / l16;
        l188_im = -(l130 / l16);
      }
    } else {
      b_l160_tmp = std::abs(l18);
      l195_im = std::abs(l16);
      if (b_l160_tmp > l195_im) {
        l160_tmp = l16 / l18;
        l195_im = l18 + l160_tmp * l16;
        l188_re = (l130 + l160_tmp * l184_im) / l195_im;
        l188_im = (l184_im - l160_tmp * l130) / l195_im;
      } else if (l195_im == b_l160_tmp) {
        if (l18 > 0.0) {
          l160_tmp = 0.5;
        } else {
          l160_tmp = -0.5;
        }
        if (l16 > 0.0) {
          l195_im = 0.5;
        } else {
          l195_im = -0.5;
        }
        l188_re = (l130 * l160_tmp + l184_im * l195_im) / b_l160_tmp;
        l188_im = (l184_im * l160_tmp - l130 * l195_im) / b_l160_tmp;
      } else {
        l160_tmp = l18 / l16;
        l195_im = l16 + l160_tmp * l18;
        l188_re = (l160_tmp * l130 + l184_im) / l195_im;
        l188_im = (l160_tmp * l184_im - l130) / l195_im;
      }
    }
    b_l11[1].re = l188_re;
    b_l11[1].im = l188_im;
    l130 = -((l11 + -J_max * t6[1].re) + d1);
    l184_im = -(-J_max * t6[1].im);
    if (l184_im == 0.0) {
      l2[1].re = l130 / l144_im_tmp;
      l2[1].im = 0.0;
    } else if (l130 == 0.0) {
      l2[1].re = 0.0;
      l2[1].im = l184_im / l144_im_tmp;
    } else {
      l2[1].re = l130 / l144_im_tmp;
      l2[1].im = l184_im / l144_im_tmp;
    }
    l188_re = J_max * t6[2].re;
    l188_im = J_max * t6[2].im;
    l195_im = l126 + -l188_re;
    l18 = l195_im * l195_im - -l188_im * -l188_im;
    l16 = l195_im * -l188_im;
    l16 += l16;
    l130 = ((((((l177.re + l8_tmp * l18) + l182.re) - l183.re) +
              l20 * (A_wayp * l188_re) * 2.0) +
             l127 * l18) +
            l17 * l195_im * 2.0) -
           l198_re * l195_im * 2.0;
    l184_im = (((l8_tmp * l16 + l20 * (A_wayp * l188_im) * 2.0) + l127 * l16) +
               l17 * -l188_im * 2.0) -
              l198_re * -l188_im * 2.0;
    l18 = (l20 * (J_max * l188_re) * 2.0 - l3 * l195_im * 2.0) +
          l164 * l195_im * 2.0;
    l16 = (l20 * (J_max * l188_im) * 2.0 - l3 * -l188_im * 2.0) +
          l164 * -l188_im * 2.0;
    if (l16 == 0.0) {
      if (l184_im == 0.0) {
        l188_re = l130 / l18;
        l188_im = 0.0;
      } else if (l130 == 0.0) {
        l188_re = 0.0;
        l188_im = l184_im / l18;
      } else {
        l188_re = l130 / l18;
        l188_im = l184_im / l18;
      }
    } else if (l18 == 0.0) {
      if (l130 == 0.0) {
        l188_re = l184_im / l16;
        l188_im = 0.0;
      } else if (l184_im == 0.0) {
        l188_re = 0.0;
        l188_im = -(l130 / l16);
      } else {
        l188_re = l184_im / l16;
        l188_im = -(l130 / l16);
      }
    } else {
      b_l160_tmp = std::abs(l18);
      l195_im = std::abs(l16);
      if (b_l160_tmp > l195_im) {
        l160_tmp = l16 / l18;
        l195_im = l18 + l160_tmp * l16;
        l188_re = (l130 + l160_tmp * l184_im) / l195_im;
        l188_im = (l184_im - l160_tmp * l130) / l195_im;
      } else if (l195_im == b_l160_tmp) {
        if (l18 > 0.0) {
          l160_tmp = 0.5;
        } else {
          l160_tmp = -0.5;
        }
        if (l16 > 0.0) {
          l195_im = 0.5;
        } else {
          l195_im = -0.5;
        }
        l188_re = (l130 * l160_tmp + l184_im * l195_im) / b_l160_tmp;
        l188_im = (l184_im * l160_tmp - l130 * l195_im) / b_l160_tmp;
      } else {
        l160_tmp = l18 / l16;
        l195_im = l16 + l160_tmp * l18;
        l188_re = (l160_tmp * l130 + l184_im) / l195_im;
        l188_im = (l160_tmp * l184_im - l130) / l195_im;
      }
    }
    b_l11[2].re = l188_re;
    b_l11[2].im = l188_im;
    l130 = -((l11 + -J_max * t6[2].re) + d1);
    l184_im = -(-J_max * t6[2].im);
    if (l184_im == 0.0) {
      l2[2].re = l130 / l144_im_tmp;
      l2[2].im = 0.0;
    } else if (l130 == 0.0) {
      l2[2].re = 0.0;
      l2[2].im = l184_im / l144_im_tmp;
    } else {
      l2[2].re = l130 / l144_im_tmp;
      l2[2].im = l184_im / l144_im_tmp;
    }
    l188_re = J_max * t6[3].re;
    l188_im = J_max * t6[3].im;
    l195_im = l126 + -l188_re;
    l18 = l195_im * l195_im - -l188_im * -l188_im;
    l16 = l195_im * -l188_im;
    l16 += l16;
    l130 = ((((((l177.re + l8_tmp * l18) + l182.re) - l183.re) +
              l20 * (A_wayp * l188_re) * 2.0) +
             l127 * l18) +
            l17 * l195_im * 2.0) -
           l198_re * l195_im * 2.0;
    l184_im = (((l8_tmp * l16 + l20 * (A_wayp * l188_im) * 2.0) + l127 * l16) +
               l17 * -l188_im * 2.0) -
              l198_re * -l188_im * 2.0;
    l18 = (l20 * (J_max * l188_re) * 2.0 - l3 * l195_im * 2.0) +
          l164 * l195_im * 2.0;
    l16 = (l20 * (J_max * l188_im) * 2.0 - l3 * -l188_im * 2.0) +
          l164 * -l188_im * 2.0;
    if (l16 == 0.0) {
      if (l184_im == 0.0) {
        l188_re = l130 / l18;
        l188_im = 0.0;
      } else if (l130 == 0.0) {
        l188_re = 0.0;
        l188_im = l184_im / l18;
      } else {
        l188_re = l130 / l18;
        l188_im = l184_im / l18;
      }
    } else if (l18 == 0.0) {
      if (l130 == 0.0) {
        l188_re = l184_im / l16;
        l188_im = 0.0;
      } else if (l184_im == 0.0) {
        l188_re = 0.0;
        l188_im = -(l130 / l16);
      } else {
        l188_re = l184_im / l16;
        l188_im = -(l130 / l16);
      }
    } else {
      b_l160_tmp = std::abs(l18);
      l195_im = std::abs(l16);
      if (b_l160_tmp > l195_im) {
        l160_tmp = l16 / l18;
        l195_im = l18 + l160_tmp * l16;
        l188_re = (l130 + l160_tmp * l184_im) / l195_im;
        l188_im = (l184_im - l160_tmp * l130) / l195_im;
      } else if (l195_im == b_l160_tmp) {
        if (l18 > 0.0) {
          l160_tmp = 0.5;
        } else {
          l160_tmp = -0.5;
        }
        if (l16 > 0.0) {
          l195_im = 0.5;
        } else {
          l195_im = -0.5;
        }
        l188_re = (l130 * l160_tmp + l184_im * l195_im) / b_l160_tmp;
        l188_im = (l184_im * l160_tmp - l130 * l195_im) / b_l160_tmp;
      } else {
        l160_tmp = l18 / l16;
        l195_im = l16 + l160_tmp * l18;
        l188_re = (l160_tmp * l130 + l184_im) / l195_im;
        l188_im = (l160_tmp * l184_im - l130) / l195_im;
      }
    }
    b_l11[3].re = l188_re;
    b_l11[3].im = l188_im;
    l130 = -((l11 + -J_max * t6[3].re) + d1);
    l184_im = -(-J_max * t6[3].im);
    if (l184_im == 0.0) {
      l2[3].re = l130 / l144_im_tmp;
      l2[3].im = 0.0;
    } else if (l130 == 0.0) {
      l2[3].re = 0.0;
      l2[3].im = l184_im / l144_im_tmp;
    } else {
      l2[3].re = l130 / l144_im_tmp;
      l2[3].im = l184_im / l144_im_tmp;
    }
    l130 = -((l11 + J_min * l2[0].re) + J_max * b_l11[0].re);
    l184_im = -(J_min * l2[0].im + J_max * b_l11[0].im);
    if (l184_im == 0.0) {
      t[0].re = l130 / J_max;
      t[0].im = 0.0;
    } else if (l130 == 0.0) {
      t[0].re = 0.0;
      t[0].im = l184_im / J_max;
    } else {
      t[0].re = l130 / J_max;
      t[0].im = l184_im / J_max;
    }
    t[4].re = 0.0;
    t[4].im = 0.0;
    t[8] = l2[0];
    t[12].re = 0.0;
    t[12].im = 0.0;
    t[16].re = 0.0;
    t[16].im = 0.0;
    t[20] = t6[0];
    t[24] = b_l11[0];
    l130 = -((l11 + J_min * l2[1].re) + J_max * b_l11[1].re);
    l184_im = -(J_min * l2[1].im + J_max * b_l11[1].im);
    if (l184_im == 0.0) {
      t[1].re = l130 / J_max;
      t[1].im = 0.0;
    } else if (l130 == 0.0) {
      t[1].re = 0.0;
      t[1].im = l184_im / J_max;
    } else {
      t[1].re = l130 / J_max;
      t[1].im = l184_im / J_max;
    }
    t[5].re = 0.0;
    t[5].im = 0.0;
    t[9] = l2[1];
    t[13].re = 0.0;
    t[13].im = 0.0;
    t[17].re = 0.0;
    t[17].im = 0.0;
    t[21] = t6[1];
    t[25] = b_l11[1];
    l130 = -((l11 + J_min * l2[2].re) + J_max * b_l11[2].re);
    l184_im = -(J_min * l2[2].im + J_max * b_l11[2].im);
    if (l184_im == 0.0) {
      t[2].re = l130 / J_max;
      t[2].im = 0.0;
    } else if (l130 == 0.0) {
      t[2].re = 0.0;
      t[2].im = l184_im / J_max;
    } else {
      t[2].re = l130 / J_max;
      t[2].im = l184_im / J_max;
    }
    t[6].re = 0.0;
    t[6].im = 0.0;
    t[10] = l2[2];
    t[14].re = 0.0;
    t[14].im = 0.0;
    t[18].re = 0.0;
    t[18].im = 0.0;
    t[22] = t6[2];
    t[26] = b_l11[2];
    l130 = -((l11 + J_min * l2[3].re) + J_max * l188_re);
    l184_im = -(J_min * l2[3].im + J_max * l188_im);
    if (l184_im == 0.0) {
      t[3].re = l130 / J_max;
      t[3].im = 0.0;
    } else if (l130 == 0.0) {
      t[3].re = 0.0;
      t[3].im = l184_im / J_max;
    } else {
      t[3].re = l130 / J_max;
      t[3].im = l184_im / J_max;
    }
    t[7].re = 0.0;
    t[7].im = 0.0;
    t[11] = l2[3];
    t[15].re = 0.0;
    t[15].im = 0.0;
    t[19].re = 0.0;
    t[19].im = 0.0;
    t[23] = t6[3];
    t[27] = b_l11[3];
}

// End of code generation (acefg_TA_AVP.cpp)
