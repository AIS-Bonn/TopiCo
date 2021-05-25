//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// abceg_TA_AVP.cpp
//
// Code generation for function 'abceg_TA_AVP'
//

// Include files
#include "abceg_TA_AVP.h"
#include "power.h"
#include "rt_nonfinite.h"
#include "sqrt.h"
#include "topico_wrapper_data.h"
#include "topico_wrapper_rtwutil.h"
#include <cmath>

// Function Definitions
void abceg_TA_AVP(double P_init, double V_init, double A_init, double P_wayp,
                  double V_wayp, double A_wayp, double J_max, double J_min,
                  double T, creal_T t[28])
{
  creal_T t2[4];
  creal_T dc;
  creal_T l177;
  creal_T l182;
  creal_T l183;
  double b_l126_tmp;
  double b_l130_tmp;
  double b_l160_tmp;
  double c_l126_tmp;
  double c_l130_tmp;
  double d;
  double d1;
  double d_l126_tmp;
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
  double l145;
  double l15;
  double l152;
  double l16;
  double l160_tmp;
  double l160_tmp_tmp;
  double l164;
  double l17;
  double l18;
  double l182_tmp;
  double l188_im;
  double l188_re;
  double l192_re;
  double l198_im;
  double l198_re;
  double l2;
  double l20;
  double l3;
  double l39;
  double l39_tmp;
  double l5;
  double l6;
  double l8;
  double l9;
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
  l2 = A_init * A_init;
  l3 = rt_powd_snf(A_init, 3.0);
  l5 = A_wayp * A_wayp;
  l6 = rt_powd_snf(A_wayp, 3.0);
  l8 = J_min * J_min;
  l9 = J_max * J_max;
  l10 = rt_powd_snf(J_max, 3.0);
  l11 = T * T;
  l12 = rt_powd_snf(T, 3.0);
  l14 = V_init * V_init;
  l15 = V_wayp * V_wayp;
  l13 = l11 * l11;
  l16 = 1.0 / l8;
  l18 = 1.0 / l9;
  l39_tmp = A_wayp * J_max;
  l39 = l39_tmp * l8 * 4.0;
  l17 = l16 * l16;
  l20 = l18 * l18;
  l126_tmp = A_init * J_min;
  l198_im = A_wayp * J_min;
  b_l126_tmp = A_init * J_max;
  c_l126_tmp = b_l126_tmp * l8;
  d_l126_tmp = l126_tmp * l9;
  e_l126_tmp = l198_im * l9;
  l126 = ((((d_l126_tmp * 4.0 + c_l126_tmp * 4.0) + J_min * T * l10 * 4.0) +
           -(e_l126_tmp * 4.0)) +
          -l39) +
         T * l8 * l9 * 4.0;
  l130_tmp = J_min * J_max;
  b_l130_tmp = J_min * l10;
  l130_tmp_tmp = A_init * A_wayp;
  c_l130_tmp = l130_tmp_tmp * J_min * J_max;
  l130 =
      ((((l130_tmp * l2 * 6.0 + l130_tmp * l5 * 6.0) + -(c_l130_tmp * 12.0)) +
        b_l130_tmp * l11 * 6.0) +
       l126_tmp * T * l9 * 12.0) +
      -(l198_im * T * l9 * 12.0);
  l127 = l126 * l126;
  l131_tmp = l16 * l18;
  l131 = l131_tmp * l126 / 12.0;
  d = l126_tmp * J_max;
  l182_tmp = l198_im * J_max;
  d1 = J_max * T;
  l192_re = A_init * P_wayp;
  l126_tmp = A_wayp * P_init;
  l160_tmp = A_init * P_init;
  b_l160_tmp = A_wayp * P_wayp;
  l164 = V_init * V_wayp;
  l188_im = A_wayp * T * V_init;
  l160_tmp_tmp = A_init * T;
  l188_re = l160_tmp_tmp * V_wayp;
  l198_im = l17 * l20;
  l144 = l198_im * l127 / 24.0 + -(l131_tmp * l130 / 3.0);
  l198_re = rt_powd_snf(l16, 3.0) * rt_powd_snf(l18, 3.0);
  l152 = l198_re * rt_powd_snf(l126, 3.0) / 216.0 +
         -(l198_im * l126 * l130 / 18.0);
  l145 = l144 * l144;
  l18 = rt_powd_snf(l144, 3.0);
  l126 = l152 * l152;
  l17 = l17 * l17 * (l20 * l20) * (l127 * l127);
  l198_re = l198_re * l127 * l130;
    l198_im = l131_tmp * (((((((((((((((((((((((((((((((((((((((((((((((((((((((((l2 * l2 + l5 * l5) + -(A_init * l6 * 4.0)) + -(A_wayp * l3 * 4.0)) + d * P_init * 48.0) + l182_tmp * P_wayp * 48.0) + d1 * l3 * 4.0) + l130_tmp * V_init * V_wayp * 48.0) + -(d * P_wayp * 48.0)) + -(l182_tmp * P_init * 48.0)) + l192_re * l8 * 24.0) + l126_tmp * l8 * 24.0) + l192_re * l9 * 24.0) + l126_tmp * l9 * 24.0) + -(d1 * l6 * 4.0)) + l2 * l5 * 6.0) + P_wayp * T * l10 * 24.0) + d * T * V_wayp * 48.0) + b_l130_tmp * l13 * 2.0) + -(l160_tmp * l8 * 24.0)) + -(l160_tmp * l9 * 24.0)) + -(b_l160_tmp * l8 * 24.0)) + -(b_l160_tmp * l9 * 24.0)) + -(l130_tmp * l14 * 24.0)) + -(l130_tmp * l15 * 24.0)) + -(P_init * T * l10 * 24.0)) + -(l164 * l8 * 24.0)) + -(l164 * l9 * 24.0)) + l8 * l14 * 12.0) + l9 * l14 * 12.0) + l8 * l15 * 12.0) + l9 * l15 * 12.0) + -(l182_tmp * T * V_init * 48.0)) + b_l126_tmp * T * l5 * 12.0) + l188_im * l8 * 24.0) + l188_im * l9 * 24.0) + J_min * P_init * T * l9 * 48.0) + J_max * P_wayp * T * l8 * 24.0) + l12 * l39) + -(l39_tmp * T * l2 * 12.0)) + -(l188_re * l8 * 24.0)) + -(l188_re * l9 * 24.0)) + -(J_max * P_init * T * l8 * 24.0)) + -(J_min * P_wayp * T * l9 * 48.0)) + -(V_init * l10 * l11 * 12.0)) + -(V_wayp * l10 * l11 * 12.0)) + -(c_l130_tmp * l11 * 24.0)) + l130_tmp_tmp * l8 * l11 * 12.0) + c_l126_tmp * l12 * -4.0) + d_l126_tmp * l12 * 8.0) + J_min * V_init * l9 * l11 * 24.0) + J_min * V_wayp * l9 * l11 * 24.0) + l2 * l9 * l11 * 6.0) + l5 * l9 * l11 * 6.0) + -(e_l126_tmp * l12 * 8.0)) + -(J_max * V_init * l8 * l11 * 12.0)) + -(J_max * V_wayp * l8 * l11 * 12.0)) + -(l8 * l9 * l13));
    l164 = (l17 / 6912.0 + -(l198_re / 432.0)) + l198_im / 3.0;
    l177.re = ((((l126 * l126 * 27.0 + -(l18 * l126 * 4.0)) +
                 rt_powd_snf(l164, 3.0) * 256.0) +
                l145 * l145 * l164 * 16.0) +
               l145 * (l164 * l164) * 128.0) +
              -(l144 * l126 * l164 * 144.0);
    l177.im = 0.0;
    coder::internal::scalar::b_sqrt(&l177);
    l188_re = 1.7320508075688772 * l177.re;
    l20 = 1.7320508075688772 * l177.im;
    if (l20 == 0.0) {
      l188_im = l188_re / 18.0;
      b_l160_tmp = 0.0;
    } else if (l188_re == 0.0) {
      l188_im = 0.0;
      b_l160_tmp = l20 / 18.0;
    } else {
      l188_im = l188_re / 18.0;
      b_l160_tmp = l20 / 18.0;
    }
    l182_tmp = l144 * l164;
    l182.re =
        ((-(l18 / 27.0) + l126 / 2.0) + -(l182_tmp * 1.3333333333333333)) +
        l188_im;
    l182.im = b_l160_tmp;
    l183 = coder::power(l182);
    dc = coder::b_power(l182);
    if (dc.im == 0.0) {
      l3 = 1.0 / dc.re;
      l2 = 0.0;
    } else if (dc.re == 0.0) {
      l3 = 0.0;
      l2 = -(1.0 / dc.im);
    } else {
      b_l160_tmp = std::abs(dc.re);
      l16 = std::abs(dc.im);
      if (b_l160_tmp > l16) {
        b_l160_tmp = dc.im / dc.re;
        l16 = dc.re + b_l160_tmp * dc.im;
        l3 = (b_l160_tmp * 0.0 + 1.0) / l16;
        l2 = (0.0 - b_l160_tmp) / l16;
      } else if (l16 == b_l160_tmp) {
        if (dc.re > 0.0) {
          l126_tmp = 0.5;
        } else {
          l126_tmp = -0.5;
        }
        if (dc.im > 0.0) {
          l16 = 0.5;
        } else {
          l16 = -0.5;
        }
        l3 = (l126_tmp + 0.0 * l16) / b_l160_tmp;
        l2 = (0.0 * l126_tmp - l16) / b_l160_tmp;
      } else {
        b_l160_tmp = dc.re / dc.im;
        l16 = dc.im + b_l160_tmp * dc.re;
        l3 = b_l160_tmp / l16;
        l2 = (b_l160_tmp * 0.0 - 1.0) / l16;
      }
    }
    l127 = l183.re * l183.re - l183.im * l183.im;
    l126_tmp = l183.re * l183.im;
    l130 = l126_tmp + l126_tmp;
    dc.re = ((-(l18 * 2.0) + l126 * 27.0) + -(l182_tmp * 72.0)) + l188_re * 3.0;
    dc.im = l20 * 3.0;
    coder::internal::scalar::b_sqrt(&dc);
    l188_re = 3.0 * (2.4494897427831779 * l152 * dc.re);
    l188_im = 3.0 * (2.4494897427831779 * l152 * dc.im);
    l160_tmp = l144 * l183.re;
    l182_tmp = l144 * l183.im;
    l182.re = ((((-(l17 / 576.0) + l198_re / 36.0) + l145) + -(l198_im * 4.0)) +
               l127 * 9.0) +
              l160_tmp * 6.0;
    l182.im = l130 * 9.0 + l182_tmp * 6.0;
    l177 = l182;
    coder::internal::scalar::b_sqrt(&l177);
    dc = coder::c_power(l182);
    if (dc.im == 0.0) {
      l192_re = 1.0 / dc.re;
      l16 = 0.0;
    } else if (dc.re == 0.0) {
      l192_re = 0.0;
      l16 = -(1.0 / dc.im);
    } else {
      b_l160_tmp = std::abs(dc.re);
      l16 = std::abs(dc.im);
      if (b_l160_tmp > l16) {
        b_l160_tmp = dc.im / dc.re;
        l16 = dc.re + b_l160_tmp * dc.im;
        l192_re = (b_l160_tmp * 0.0 + 1.0) / l16;
        l16 = (0.0 - b_l160_tmp) / l16;
      } else if (l16 == b_l160_tmp) {
        if (dc.re > 0.0) {
          l126_tmp = 0.5;
        } else {
          l126_tmp = -0.5;
        }
        if (dc.im > 0.0) {
          l16 = 0.5;
        } else {
          l16 = -0.5;
        }
        l192_re = (l126_tmp + 0.0 * l16) / b_l160_tmp;
        l16 = (0.0 * l126_tmp - l16) / b_l160_tmp;
      } else {
        b_l160_tmp = dc.re / dc.im;
        l16 = dc.im + b_l160_tmp * dc.re;
        l192_re = b_l160_tmp / l16;
        l16 = (b_l160_tmp * 0.0 - 1.0) / l16;
      }
    }
    l17 = 12.0 * (l164 * l177.re);
    l164 = 12.0 * (l164 * l177.im);
    l18 = -9.0 * (l127 * l177.re - l130 * l177.im);
    l126 = -9.0 * (l127 * l177.im + l130 * l177.re);
    l126_tmp = l3 * l177.re - l2 * l177.im;
    l20 = l3 * l177.im + l2 * l177.re;
    if (l20 == 0.0) {
      l198_re = l126_tmp / 6.0;
      l198_im = 0.0;
    } else if (l126_tmp == 0.0) {
      l198_re = 0.0;
      l198_im = l20 / 6.0;
    } else {
      l198_re = l126_tmp / 6.0;
      l198_im = l20 / 6.0;
    }
    l127 = 12.0 * (l160_tmp * l177.re - l182_tmp * l177.im);
    l130 = 12.0 * (l160_tmp * l177.im + l182_tmp * l177.re);
    l182.re = -(l145 * l177.re);
    l182.im = -(l145 * l177.im);
    dc.re = (((l188_re + l182.re) + l17) + l18) + l127;
    dc.im = (((l188_im + l182.im) + l164) + l126) + l130;
    coder::internal::scalar::b_sqrt(&dc);
    l182_tmp = l3 * l192_re - l2 * l16;
    l126_tmp = l3 * l16 + l2 * l192_re;
    l3 = l182_tmp * dc.re - l126_tmp * dc.im;
    l2 = l182_tmp * dc.im + l126_tmp * dc.re;
    if (l2 == 0.0) {
      l183.re = l3 / 6.0;
      l183.im = 0.0;
    } else if (l3 == 0.0) {
      l183.re = 0.0;
      l183.im = l2 / 6.0;
    } else {
      l183.re = l3 / 6.0;
      l183.im = l2 / 6.0;
    }
    dc.re = (((-l188_re + l182.re) + l17) + l18) + l127;
    dc.im = (((-l188_im + l182.im) + l164) + l126) + l130;
    coder::internal::scalar::b_sqrt(&dc);
    l3 = l182_tmp * dc.re - l126_tmp * dc.im;
    l2 = l182_tmp * dc.im + l126_tmp * dc.re;
    if (l2 == 0.0) {
      l182.re = l3 / 6.0;
      l182.im = 0.0;
    } else if (l3 == 0.0) {
      l182.re = 0.0;
      l182.im = l2 / 6.0;
    } else {
      l182.re = l3 / 6.0;
      l182.im = l2 / 6.0;
    }
    d = l131 + -l198_re;
    t2[0].re = d - l182.re;
    t2[0].im = -l198_im - l182.im;
    t2[1].re = d + l182.re;
    t2[1].im = -l198_im + l182.im;
    d = l131 + l198_re;
    t2[2].re = d - l183.re;
    t2[2].im = l198_im - l183.im;
    t2[3].re = d + l183.re;
    t2[3].im = l198_im + l183.im;
    l160_tmp = J_min + -J_max;
    l8 = l160_tmp * l160_tmp;
    l177.re = V_init * l8 * 2.0;
    l182.re = V_wayp * l8 * 2.0;
    l183.re = l160_tmp_tmp * l8 * 2.0;
    l3 = T * d1 * l8;
    l198_re = J_min * l160_tmp;
    l17 = J_max * l160_tmp;
    l188_im = J_max * t2[0].re;
    b_l160_tmp = J_max * t2[0].im;
    l192_re = (A_init + d1) + -A_wayp;
    l126 = l192_re + -l188_im;
    l20 = l126 * l126 - -b_l160_tmp * -b_l160_tmp;
    l126_tmp = l126 * -b_l160_tmp;
    l164 = l126_tmp + l126_tmp;
    l126_tmp = l160_tmp * l188_im;
    l182_tmp = l160_tmp * b_l160_tmp;
    l198_im = l8 * -l188_im;
    l16 = l8 * -b_l160_tmp;
    l18 = ((((((J_min * l20 + l177.re) - l182.re) + -J_max * l20) + l183.re) +
            l3) +
           (l126_tmp * l126 - l182_tmp * -b_l160_tmp) * 2.0) +
          (l198_im * t2[0].re - l16 * t2[0].im);
    l198_im = ((J_min * l164 + -J_max * l164) +
               (l126_tmp * -b_l160_tmp + l182_tmp * l126) * 2.0) +
              (l198_im * t2[0].im + l16 * t2[0].re);
    l126_tmp = (l8 * l188_im * 2.0 + l198_re * l126 * 2.0) - l17 * l126 * 2.0;
    l20 = (l8 * b_l160_tmp * 2.0 + l198_re * -b_l160_tmp * 2.0) -
          l17 * -b_l160_tmp * 2.0;
    if (l20 == 0.0) {
      if (l198_im == 0.0) {
        l126 = l18 / l126_tmp;
        l164 = 0.0;
      } else if (l18 == 0.0) {
        l126 = 0.0;
        l164 = l198_im / l126_tmp;
      } else {
        l126 = l18 / l126_tmp;
        l164 = l198_im / l126_tmp;
      }
    } else if (l126_tmp == 0.0) {
      if (l18 == 0.0) {
        l126 = l198_im / l20;
        l164 = 0.0;
      } else if (l198_im == 0.0) {
        l126 = 0.0;
        l164 = -(l18 / l20);
      } else {
        l126 = l198_im / l20;
        l164 = -(l18 / l20);
      }
    } else {
      b_l160_tmp = std::abs(l126_tmp);
      l16 = std::abs(l20);
      if (b_l160_tmp > l16) {
        b_l160_tmp = l20 / l126_tmp;
        l16 = l126_tmp + b_l160_tmp * l20;
        l126 = (l18 + b_l160_tmp * l198_im) / l16;
        l164 = (l198_im - b_l160_tmp * l18) / l16;
      } else if (l16 == b_l160_tmp) {
        if (l126_tmp > 0.0) {
          l126_tmp = 0.5;
        } else {
          l126_tmp = -0.5;
        }
        if (l20 > 0.0) {
          l16 = 0.5;
        } else {
          l16 = -0.5;
        }
        l126 = (l18 * l126_tmp + l198_im * l16) / b_l160_tmp;
        l164 = (l198_im * l126_tmp - l18 * l16) / b_l160_tmp;
      } else {
        b_l160_tmp = l126_tmp / l20;
        l16 = l20 + b_l160_tmp * l126_tmp;
        l126 = (b_l160_tmp * l18 + l198_im) / l16;
        l164 = (b_l160_tmp * l198_im - l18) / l16;
      }
    }
    l188_re = A_init - A_wayp;
    l18 = -((l188_re + -J_max * t2[0].re) + d1);
    l198_im = -(-J_max * t2[0].im);
    if (l198_im == 0.0) {
      l188_im = l18 / l160_tmp;
      b_l160_tmp = 0.0;
    } else if (l18 == 0.0) {
      l188_im = 0.0;
      b_l160_tmp = l198_im / l160_tmp;
    } else {
      l188_im = l18 / l160_tmp;
      b_l160_tmp = l198_im / l160_tmp;
    }
    t[0].re = ((T - t2[0].re) - l188_im) - l126;
    t[0].im = ((0.0 - t2[0].im) - b_l160_tmp) - l164;
    t[4] = t2[0];
    t[8].re = l188_im;
    t[8].im = b_l160_tmp;
    t[12].re = 0.0;
    t[12].im = 0.0;
    t[16].re = 0.0;
    t[16].im = 0.0;
    t[20].re = 0.0;
    t[20].im = 0.0;
    t[24].re = l126;
    t[24].im = l164;
    l188_im = J_max * t2[1].re;
    b_l160_tmp = J_max * t2[1].im;
    l126 = l192_re + -l188_im;
    l20 = l126 * l126 - -b_l160_tmp * -b_l160_tmp;
    l126_tmp = l126 * -b_l160_tmp;
    l164 = l126_tmp + l126_tmp;
    l126_tmp = l160_tmp * l188_im;
    l182_tmp = l160_tmp * b_l160_tmp;
    l198_im = l8 * -l188_im;
    l16 = l8 * -b_l160_tmp;
    l18 = ((((((J_min * l20 + l177.re) - l182.re) + -J_max * l20) + l183.re) +
            l3) +
           (l126_tmp * l126 - l182_tmp * -b_l160_tmp) * 2.0) +
          (l198_im * t2[1].re - l16 * t2[1].im);
    l198_im = ((J_min * l164 + -J_max * l164) +
               (l126_tmp * -b_l160_tmp + l182_tmp * l126) * 2.0) +
              (l198_im * t2[1].im + l16 * t2[1].re);
    l126_tmp = (l8 * l188_im * 2.0 + l198_re * l126 * 2.0) - l17 * l126 * 2.0;
    l20 = (l8 * b_l160_tmp * 2.0 + l198_re * -b_l160_tmp * 2.0) -
          l17 * -b_l160_tmp * 2.0;
    if (l20 == 0.0) {
      if (l198_im == 0.0) {
        l126 = l18 / l126_tmp;
        l164 = 0.0;
      } else if (l18 == 0.0) {
        l126 = 0.0;
        l164 = l198_im / l126_tmp;
      } else {
        l126 = l18 / l126_tmp;
        l164 = l198_im / l126_tmp;
      }
    } else if (l126_tmp == 0.0) {
      if (l18 == 0.0) {
        l126 = l198_im / l20;
        l164 = 0.0;
      } else if (l198_im == 0.0) {
        l126 = 0.0;
        l164 = -(l18 / l20);
      } else {
        l126 = l198_im / l20;
        l164 = -(l18 / l20);
      }
    } else {
      b_l160_tmp = std::abs(l126_tmp);
      l16 = std::abs(l20);
      if (b_l160_tmp > l16) {
        b_l160_tmp = l20 / l126_tmp;
        l16 = l126_tmp + b_l160_tmp * l20;
        l126 = (l18 + b_l160_tmp * l198_im) / l16;
        l164 = (l198_im - b_l160_tmp * l18) / l16;
      } else if (l16 == b_l160_tmp) {
        if (l126_tmp > 0.0) {
          l126_tmp = 0.5;
        } else {
          l126_tmp = -0.5;
        }
        if (l20 > 0.0) {
          l16 = 0.5;
        } else {
          l16 = -0.5;
        }
        l126 = (l18 * l126_tmp + l198_im * l16) / b_l160_tmp;
        l164 = (l198_im * l126_tmp - l18 * l16) / b_l160_tmp;
      } else {
        b_l160_tmp = l126_tmp / l20;
        l16 = l20 + b_l160_tmp * l126_tmp;
        l126 = (b_l160_tmp * l18 + l198_im) / l16;
        l164 = (b_l160_tmp * l198_im - l18) / l16;
      }
    }
    l18 = -((l188_re + -J_max * t2[1].re) + d1);
    l198_im = -(-J_max * t2[1].im);
    if (l198_im == 0.0) {
      l188_im = l18 / l160_tmp;
      b_l160_tmp = 0.0;
    } else if (l18 == 0.0) {
      l188_im = 0.0;
      b_l160_tmp = l198_im / l160_tmp;
    } else {
      l188_im = l18 / l160_tmp;
      b_l160_tmp = l198_im / l160_tmp;
    }
    t[1].re = ((T - t2[1].re) - l188_im) - l126;
    t[1].im = ((0.0 - t2[1].im) - b_l160_tmp) - l164;
    t[5] = t2[1];
    t[9].re = l188_im;
    t[9].im = b_l160_tmp;
    t[13].re = 0.0;
    t[13].im = 0.0;
    t[17].re = 0.0;
    t[17].im = 0.0;
    t[21].re = 0.0;
    t[21].im = 0.0;
    t[25].re = l126;
    t[25].im = l164;
    l188_im = J_max * t2[2].re;
    b_l160_tmp = J_max * t2[2].im;
    l126 = l192_re + -l188_im;
    l20 = l126 * l126 - -b_l160_tmp * -b_l160_tmp;
    l126_tmp = l126 * -b_l160_tmp;
    l164 = l126_tmp + l126_tmp;
    l126_tmp = l160_tmp * l188_im;
    l182_tmp = l160_tmp * b_l160_tmp;
    l198_im = l8 * -l188_im;
    l16 = l8 * -b_l160_tmp;
    l18 = ((((((J_min * l20 + l177.re) - l182.re) + -J_max * l20) + l183.re) +
            l3) +
           (l126_tmp * l126 - l182_tmp * -b_l160_tmp) * 2.0) +
          (l198_im * t2[2].re - l16 * t2[2].im);
    l198_im = ((J_min * l164 + -J_max * l164) +
               (l126_tmp * -b_l160_tmp + l182_tmp * l126) * 2.0) +
              (l198_im * t2[2].im + l16 * t2[2].re);
    l126_tmp = (l8 * l188_im * 2.0 + l198_re * l126 * 2.0) - l17 * l126 * 2.0;
    l20 = (l8 * b_l160_tmp * 2.0 + l198_re * -b_l160_tmp * 2.0) -
          l17 * -b_l160_tmp * 2.0;
    if (l20 == 0.0) {
      if (l198_im == 0.0) {
        l126 = l18 / l126_tmp;
        l164 = 0.0;
      } else if (l18 == 0.0) {
        l126 = 0.0;
        l164 = l198_im / l126_tmp;
      } else {
        l126 = l18 / l126_tmp;
        l164 = l198_im / l126_tmp;
      }
    } else if (l126_tmp == 0.0) {
      if (l18 == 0.0) {
        l126 = l198_im / l20;
        l164 = 0.0;
      } else if (l198_im == 0.0) {
        l126 = 0.0;
        l164 = -(l18 / l20);
      } else {
        l126 = l198_im / l20;
        l164 = -(l18 / l20);
      }
    } else {
      b_l160_tmp = std::abs(l126_tmp);
      l16 = std::abs(l20);
      if (b_l160_tmp > l16) {
        b_l160_tmp = l20 / l126_tmp;
        l16 = l126_tmp + b_l160_tmp * l20;
        l126 = (l18 + b_l160_tmp * l198_im) / l16;
        l164 = (l198_im - b_l160_tmp * l18) / l16;
      } else if (l16 == b_l160_tmp) {
        if (l126_tmp > 0.0) {
          l126_tmp = 0.5;
        } else {
          l126_tmp = -0.5;
        }
        if (l20 > 0.0) {
          l16 = 0.5;
        } else {
          l16 = -0.5;
        }
        l126 = (l18 * l126_tmp + l198_im * l16) / b_l160_tmp;
        l164 = (l198_im * l126_tmp - l18 * l16) / b_l160_tmp;
      } else {
        b_l160_tmp = l126_tmp / l20;
        l16 = l20 + b_l160_tmp * l126_tmp;
        l126 = (b_l160_tmp * l18 + l198_im) / l16;
        l164 = (b_l160_tmp * l198_im - l18) / l16;
      }
    }
    l18 = -((l188_re + -J_max * t2[2].re) + d1);
    l198_im = -(-J_max * t2[2].im);
    if (l198_im == 0.0) {
      l188_im = l18 / l160_tmp;
      b_l160_tmp = 0.0;
    } else if (l18 == 0.0) {
      l188_im = 0.0;
      b_l160_tmp = l198_im / l160_tmp;
    } else {
      l188_im = l18 / l160_tmp;
      b_l160_tmp = l198_im / l160_tmp;
    }
    t[2].re = ((T - t2[2].re) - l188_im) - l126;
    t[2].im = ((0.0 - t2[2].im) - b_l160_tmp) - l164;
    t[6] = t2[2];
    t[10].re = l188_im;
    t[10].im = b_l160_tmp;
    t[14].re = 0.0;
    t[14].im = 0.0;
    t[18].re = 0.0;
    t[18].im = 0.0;
    t[22].re = 0.0;
    t[22].im = 0.0;
    t[26].re = l126;
    t[26].im = l164;
    l188_im = J_max * t2[3].re;
    b_l160_tmp = J_max * t2[3].im;
    l126 = l192_re + -l188_im;
    l20 = l126 * l126 - -b_l160_tmp * -b_l160_tmp;
    l126_tmp = l126 * -b_l160_tmp;
    l164 = l126_tmp + l126_tmp;
    l126_tmp = l160_tmp * l188_im;
    l182_tmp = l160_tmp * b_l160_tmp;
    l198_im = l8 * -l188_im;
    l16 = l8 * -b_l160_tmp;
    l18 = ((((((J_min * l20 + l177.re) - l182.re) + -J_max * l20) + l183.re) +
            l3) +
           (l126_tmp * l126 - l182_tmp * -b_l160_tmp) * 2.0) +
          (l198_im * t2[3].re - l16 * t2[3].im);
    l198_im = ((J_min * l164 + -J_max * l164) +
               (l126_tmp * -b_l160_tmp + l182_tmp * l126) * 2.0) +
              (l198_im * t2[3].im + l16 * t2[3].re);
    l126_tmp = (l8 * l188_im * 2.0 + l198_re * l126 * 2.0) - l17 * l126 * 2.0;
    l20 = (l8 * b_l160_tmp * 2.0 + l198_re * -b_l160_tmp * 2.0) -
          l17 * -b_l160_tmp * 2.0;
    if (l20 == 0.0) {
      if (l198_im == 0.0) {
        l126 = l18 / l126_tmp;
        l164 = 0.0;
      } else if (l18 == 0.0) {
        l126 = 0.0;
        l164 = l198_im / l126_tmp;
      } else {
        l126 = l18 / l126_tmp;
        l164 = l198_im / l126_tmp;
      }
    } else if (l126_tmp == 0.0) {
      if (l18 == 0.0) {
        l126 = l198_im / l20;
        l164 = 0.0;
      } else if (l198_im == 0.0) {
        l126 = 0.0;
        l164 = -(l18 / l20);
      } else {
        l126 = l198_im / l20;
        l164 = -(l18 / l20);
      }
    } else {
      b_l160_tmp = std::abs(l126_tmp);
      l16 = std::abs(l20);
      if (b_l160_tmp > l16) {
        b_l160_tmp = l20 / l126_tmp;
        l16 = l126_tmp + b_l160_tmp * l20;
        l126 = (l18 + b_l160_tmp * l198_im) / l16;
        l164 = (l198_im - b_l160_tmp * l18) / l16;
      } else if (l16 == b_l160_tmp) {
        if (l126_tmp > 0.0) {
          l126_tmp = 0.5;
        } else {
          l126_tmp = -0.5;
        }
        if (l20 > 0.0) {
          l16 = 0.5;
        } else {
          l16 = -0.5;
        }
        l126 = (l18 * l126_tmp + l198_im * l16) / b_l160_tmp;
        l164 = (l198_im * l126_tmp - l18 * l16) / b_l160_tmp;
      } else {
        b_l160_tmp = l126_tmp / l20;
        l16 = l20 + b_l160_tmp * l126_tmp;
        l126 = (b_l160_tmp * l18 + l198_im) / l16;
        l164 = (b_l160_tmp * l198_im - l18) / l16;
      }
    }
    l18 = -((l188_re + -J_max * t2[3].re) + d1);
    l198_im = -(-J_max * t2[3].im);
    if (l198_im == 0.0) {
      l188_im = l18 / l160_tmp;
      b_l160_tmp = 0.0;
    } else if (l18 == 0.0) {
      l188_im = 0.0;
      b_l160_tmp = l198_im / l160_tmp;
    } else {
      l188_im = l18 / l160_tmp;
      b_l160_tmp = l198_im / l160_tmp;
    }
    t[3].re = ((T - t2[3].re) - l188_im) - l126;
    t[3].im = ((0.0 - t2[3].im) - b_l160_tmp) - l164;
    t[7] = t2[3];
    t[11].re = l188_im;
    t[11].im = b_l160_tmp;
    t[15].re = 0.0;
    t[15].im = 0.0;
    t[19].re = 0.0;
    t[19].im = 0.0;
    t[23].re = 0.0;
    t[23].im = 0.0;
    t[27].re = l126;
    t[27].im = l164;
}

// End of code generation (abceg_TA_AVP.cpp)
