//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// abceg_O_AP.cpp
//
// Code generation for function 'abceg_O_AP'
//

// Include files
#include "abceg_O_AP.h"
#include "power.h"
#include "rt_nonfinite.h"
#include "sqrt.h"
#include "topico_wrapper_data.h"
#include "topico_wrapper_rtwutil.h"
#include <cmath>

// Function Definitions
void abceg_O_AP(double P_init, double V_init, double A_init, double P_wayp,
                double A_wayp, double V_max, double A_max, double J_max,
                double J_min, creal_T t[28])
{
  creal_T t7[4];
  creal_T dc;
  creal_T l142;
  creal_T l147;
  creal_T l148;
  double a_tmp;
  double b_a_tmp;
  double l106_tmp;
  double l10_tmp;
  double l112;
  double l115;
  double l11_tmp;
  double l12;
  double l120;
  double l13;
  double l154_im;
  double l158_im;
  double l162_re;
  double l164_im;
  double l165_im;
  double l2_tmp;
  double l41_tmp;
  double l42_tmp;
  double l5_tmp;
  double l6;
  double l7;
  double l82;
  double l83;
  double l84;
  double l86;
  double l87;
  double l9;
  double l90;
  double l95_tmp;
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
  l5_tmp = A_max * A_max;
  l7 = A_wayp * A_wayp;
  l9 = J_min * J_min;
  l10_tmp = J_max * J_max;
  l11_tmp = rt_powd_snf(J_max, 3.0);
  l13 = rt_powd_snf(J_max, 5.0);
  l6 = l5_tmp * l5_tmp;
  l12 = l10_tmp * l10_tmp;
  l164_im = A_max * A_wayp;
  l162_re = l164_im * J_min;
  l41_tmp = A_max * J_min;
  l42_tmp = A_wayp * J_min;
  l154_im = J_min * V_max;
  l82 = 1.0 / ((rt_powd_snf(l10_tmp, 3.0) * 3.0 + -(J_min * l13 * 6.0)) +
               l9 * l12 * 3.0);
  l86 =
      ((((A_max * l13 * 8.0 + -(A_wayp * l13 * 12.0)) + l42_tmp * l12 * 24.0) +
        -(l41_tmp * l12 * 12.0)) +
       A_max * l9 * l11_tmp * 4.0) +
      -(A_wayp * l9 * l11_tmp * 12.0);
  l83 = l82 * l82;
  l84 = rt_powd_snf(l82, 3.0);
  l87 = l86 * l86;
  l90 = l82 * l86 / 4.0;
  l165_im = J_max * V_init;
  l158_im = l9 * l10_tmp;
  l112 = l82 * ((((((((((((((l6 * l9 + -(l2_tmp * l2_tmp * l9 * 3.0)) +
                            -(l6 * l10_tmp)) +
                           A_max * rt_powd_snf(A_wayp, 3.0) * l10_tmp * 4.0) +
                          l162_re * V_max * l10_tmp * 24.0) +
                         A_max * rt_powd_snf(A_init, 3.0) * l9 * 8.0) +
                        -(A_init * A_max * J_max * V_init * l9 * 24.0)) +
                       A_max * P_init * l9 * l10_tmp * 24.0) +
                      l165_im * l2_tmp * l9 * 12.0) +
                     l165_im * l5_tmp * l9 * 12.0) +
                    -(A_max * P_wayp * l9 * l10_tmp * 24.0)) +
                   -(l154_im * l5_tmp * l10_tmp * 12.0)) +
                  -(l2_tmp * l5_tmp * l9 * 6.0)) +
                 l158_im * (V_max * V_max) * 12.0) +
                -(l158_im * (V_init * V_init) * 12.0));
  l13 = ((((((l41_tmp * V_max * l11_tmp * 24.0 -
              l42_tmp * V_max * l11_tmp * 24.0) -
             A_max * l7 * l11_tmp * 12.0) +
            A_wayp * l5_tmp * l11_tmp * 12.0) +
           l41_tmp * l7 * l10_tmp * 12.0) -
          l42_tmp * l5_tmp * l10_tmp * 12.0) -
         A_max * V_max * l9 * l10_tmp * 24.0) +
        A_wayp * V_max * l9 * l10_tmp * 24.0;
  l120 = l83 * l86;
  l106_tmp = l120 * l13;
  l95_tmp = l83 * l83 * (l87 * l87);
  a_tmp = ((((((((l164_im * l12 * 24.0 + l154_im * l12 * 12.0) -
                 l5_tmp * l12 * 6.0) -
                l7 * l12 * 12.0) -
               l162_re * l11_tmp * 36.0) +
              J_min * l5_tmp * l11_tmp * 6.0) -
             V_max * l9 * l11_tmp * 12.0) +
            J_min * l7 * l11_tmp * 24.0) +
           l164_im * l9 * l10_tmp * 12.0) -
          l7 * l9 * l10_tmp * 12.0;
  b_a_tmp = l83 * l87 * 0.375 + l82 * a_tmp;
  l115 = b_a_tmp * b_a_tmp;
  l120 = (l84 * rt_powd_snf(l86, 3.0) / 8.0 + -l82 * l13) + l120 * a_tmp / 2.0;
  l83 = l120 * l120;
  l158_im = l84 * l87 * a_tmp;
  a_tmp = ((l95_tmp * 0.01171875 + l106_tmp * -0.25) - l112) + l158_im / 16.0;
  l9 = rt_powd_snf(b_a_tmp, 3.0);
  l142.re = ((((l83 * l83 * 27.0 + l83 * l9 * -4.0) +
               rt_powd_snf(a_tmp, 3.0) * 256.0) +
              l115 * l115 * a_tmp * 16.0) +
             l115 * (a_tmp * a_tmp) * 128.0) +
            -(l83 * b_a_tmp * a_tmp * 144.0);
  l142.im = 0.0;
  coder::internal::scalar::b_sqrt(&l142);
  l12 = 1.7320508075688772 * l142.re;
  l154_im = 1.7320508075688772 * l142.im;
  if (l154_im == 0.0) {
    l165_im = l12 / 18.0;
    l13 = 0.0;
  } else if (l12 == 0.0) {
    l165_im = 0.0;
    l13 = l154_im / 18.0;
  } else {
    l165_im = l12 / 18.0;
    l13 = l154_im / 18.0;
  }
  l164_im = b_a_tmp * a_tmp;
  l147.re = ((l9 * -0.037037037037037035 + l83 / 2.0) +
             -(l164_im * 1.3333333333333333)) +
            l165_im;
  l147.im = l13;
  l148 = coder::power(l147);
  dc = coder::b_power(l147);
  if (dc.im == 0.0) {
    l84 = 1.0 / dc.re;
    l82 = 0.0;
  } else if (dc.re == 0.0) {
    l84 = 0.0;
    l82 = -(1.0 / dc.im);
  } else {
    l7 = std::abs(dc.re);
    l13 = std::abs(dc.im);
    if (l7 > l13) {
      l13 = dc.im / dc.re;
      l165_im = dc.re + l13 * dc.im;
      l84 = (l13 * 0.0 + 1.0) / l165_im;
      l82 = (0.0 - l13) / l165_im;
    } else if (l13 == l7) {
      if (dc.re > 0.0) {
        l13 = 0.5;
      } else {
        l13 = -0.5;
      }
      if (dc.im > 0.0) {
        l165_im = 0.5;
      } else {
        l165_im = -0.5;
      }
      l84 = (l13 + 0.0 * l165_im) / l7;
      l82 = (0.0 * l13 - l165_im) / l7;
    } else {
      l13 = dc.re / dc.im;
      l165_im = dc.im + l13 * dc.re;
      l84 = l13 / l165_im;
      l82 = (l13 * 0.0 - 1.0) / l165_im;
    }
  }
  l86 = l148.re * l148.re - l148.im * l148.im;
  l13 = l148.re * l148.im;
  l6 = l13 + l13;
  dc.re = ((l9 * -2.0 + l83 * 27.0) + -(l164_im * 72.0)) + l12 * 3.0;
  dc.im = l154_im * 3.0;
  coder::internal::scalar::b_sqrt(&dc);
  l83 = 3.0 * (2.4494897427831779 * l120 * dc.re);
  l154_im = 3.0 * (2.4494897427831779 * l120 * dc.im);
  l147.re = (((((-(l95_tmp * 0.140625) + l106_tmp * 3.0) + l158_im * -0.75) +
               l112 * 12.0) +
              l115) +
             l86 * 9.0) +
            b_a_tmp * l148.re * 6.0;
  l147.im = l6 * 9.0 + b_a_tmp * l148.im * 6.0;
  l142 = l147;
  coder::internal::scalar::b_sqrt(&l142);
  dc = coder::c_power(l147);
  if (dc.im == 0.0) {
    l9 = 1.0 / dc.re;
    l158_im = 0.0;
  } else if (dc.re == 0.0) {
    l9 = 0.0;
    l158_im = -(1.0 / dc.im);
  } else {
    l7 = std::abs(dc.re);
    l13 = std::abs(dc.im);
    if (l7 > l13) {
      l13 = dc.im / dc.re;
      l165_im = dc.re + l13 * dc.im;
      l9 = (l13 * 0.0 + 1.0) / l165_im;
      l158_im = (0.0 - l13) / l165_im;
    } else if (l13 == l7) {
      if (dc.re > 0.0) {
        l13 = 0.5;
      } else {
        l13 = -0.5;
      }
      if (dc.im > 0.0) {
        l165_im = 0.5;
      } else {
        l165_im = -0.5;
      }
      l9 = (l13 + 0.0 * l165_im) / l7;
      l158_im = (0.0 * l13 - l165_im) / l7;
    } else {
      l13 = dc.re / dc.im;
      l165_im = dc.im + l13 * dc.re;
      l9 = l13 / l165_im;
      l158_im = (l13 * 0.0 - 1.0) / l165_im;
    }
  }
  l162_re = 12.0 * (a_tmp * l142.re);
  l120 = 12.0 * (a_tmp * l142.im);
  l12 = -9.0 * (l86 * l142.re - l6 * l142.im);
  l164_im = -9.0 * (l86 * l142.im + l6 * l142.re);
  l7 = l84 * l142.re - l82 * l142.im;
  l13 = l84 * l142.im + l82 * l142.re;
  if (l13 == 0.0) {
    l7 /= 6.0;
    l165_im = 0.0;
  } else if (l7 == 0.0) {
    l7 = 0.0;
    l165_im = l13 / 6.0;
  } else {
    l7 /= 6.0;
    l165_im = l13 / 6.0;
  }
  l86 = 12.0 * (b_a_tmp * (l148.re * l142.re - l148.im * l142.im));
  l6 = 12.0 * (b_a_tmp * (l148.re * l142.im + l148.im * l142.re));
  l147.re = -(l115 * l142.re);
  l147.im = -(l115 * l142.im);
  dc.re = (((l83 + l147.re) + l162_re) + l12) + l86;
  dc.im = (((l154_im + l147.im) + l120) + l164_im) + l6;
  coder::internal::scalar::b_sqrt(&dc);
  l13 = l84 * l9 - l82 * l158_im;
  l9 = l84 * l158_im + l82 * l9;
  l84 = l13 * dc.re - l9 * dc.im;
  l82 = l13 * dc.im + l9 * dc.re;
  if (l82 == 0.0) {
    l148.re = l84 / 6.0;
    l148.im = 0.0;
  } else if (l84 == 0.0) {
    l148.re = 0.0;
    l148.im = l82 / 6.0;
  } else {
    l148.re = l84 / 6.0;
    l148.im = l82 / 6.0;
  }
  dc.re = (((-l83 + l147.re) + l162_re) + l12) + l86;
  dc.im = (((-l154_im + l147.im) + l120) + l164_im) + l6;
  coder::internal::scalar::b_sqrt(&dc);
  l84 = l13 * dc.re - l9 * dc.im;
  l82 = l13 * dc.im + l9 * dc.re;
  if (l82 == 0.0) {
    l147.re = l84 / 6.0;
    l147.im = 0.0;
  } else if (l84 == 0.0) {
    l147.re = 0.0;
    l147.im = l82 / 6.0;
  } else {
    l147.re = l84 / 6.0;
    l147.im = l82 / 6.0;
  }
  l13 = -l90 + -l7;
  t7[0].re = l13 - l148.re;
  t7[0].im = -l165_im - l148.im;
  t7[1].re = l13 + l148.re;
  t7[1].im = -l165_im + l148.im;
  l13 = -l90 + l7;
  t7[2].re = l13 - l147.re;
  t7[2].im = l165_im - l147.im;
  t7[3].re = l13 + l147.re;
  t7[3].im = l165_im + l147.im;
  l120 = l41_tmp * J_max * 2.0;
  l6 = -(1.0 / J_max * (A_init + -A_max));
  l142.re = (-J_min * l5_tmp + J_max * l5_tmp) + l2_tmp * J_min;
  l164_im = J_min * J_max;
  l147.re = l164_im * V_init * 2.0;
  l148.re = l164_im * V_max * 2.0;
  l9 = A_wayp * l10_tmp;
  l7 = J_min * l10_tmp;
  l158_im = l42_tmp * J_max;
  l84 = A_max - A_wayp;
  t[0].re = l6;
  t[0].im = 0.0;
  t[1].re = l6;
  t[1].im = 0.0;
  t[2].re = l6;
  t[2].im = 0.0;
  t[3].re = l6;
  t[3].im = 0.0;
  l165_im = t7[0].re * t7[0].re - t7[0].im * t7[0].im;
  l13 = t7[0].re * t7[0].im;
  l13 += l13;
  l165_im = (((((l142.re - l11_tmp * l165_im) - l147.re) + l148.re) +
              l9 * t7[0].re * 2.0) +
             l7 * l165_im) -
            l158_im * t7[0].re * 2.0;
  l13 = (((0.0 - l11_tmp * l13) + l9 * t7[0].im * 2.0) + l7 * l13) -
        l158_im * t7[0].im * 2.0;
  if (l13 == 0.0) {
    t[4].re = l165_im / l120;
    t[4].im = 0.0;
  } else if (l165_im == 0.0) {
    t[4].re = 0.0;
    t[4].im = l13 / l120;
  } else {
    t[4].re = l165_im / l120;
    t[4].im = l13 / l120;
  }
  l165_im = -(l84 + J_max * t7[0].re);
  l13 = -(J_max * t7[0].im);
  if (l13 == 0.0) {
    t[8].re = l165_im / J_min;
    t[8].im = 0.0;
  } else if (l165_im == 0.0) {
    t[8].re = 0.0;
    t[8].im = l13 / J_min;
  } else {
    t[8].re = l165_im / J_min;
    t[8].im = l13 / J_min;
  }
  t[12].re = 0.0;
  t[12].im = 0.0;
  t[16].re = 0.0;
  t[16].im = 0.0;
  t[20].re = 0.0;
  t[20].im = 0.0;
  t[24] = t7[0];
  l165_im = t7[1].re * t7[1].re - t7[1].im * t7[1].im;
  l13 = t7[1].re * t7[1].im;
  l13 += l13;
  l165_im = (((((l142.re - l11_tmp * l165_im) - l147.re) + l148.re) +
              l9 * t7[1].re * 2.0) +
             l7 * l165_im) -
            l158_im * t7[1].re * 2.0;
  l13 = (((0.0 - l11_tmp * l13) + l9 * t7[1].im * 2.0) + l7 * l13) -
        l158_im * t7[1].im * 2.0;
  if (l13 == 0.0) {
    t[5].re = l165_im / l120;
    t[5].im = 0.0;
  } else if (l165_im == 0.0) {
    t[5].re = 0.0;
    t[5].im = l13 / l120;
  } else {
    t[5].re = l165_im / l120;
    t[5].im = l13 / l120;
  }
  l165_im = -(l84 + J_max * t7[1].re);
  l13 = -(J_max * t7[1].im);
  if (l13 == 0.0) {
    t[9].re = l165_im / J_min;
    t[9].im = 0.0;
  } else if (l165_im == 0.0) {
    t[9].re = 0.0;
    t[9].im = l13 / J_min;
  } else {
    t[9].re = l165_im / J_min;
    t[9].im = l13 / J_min;
  }
  t[13].re = 0.0;
  t[13].im = 0.0;
  t[17].re = 0.0;
  t[17].im = 0.0;
  t[21].re = 0.0;
  t[21].im = 0.0;
  t[25] = t7[1];
  l165_im = t7[2].re * t7[2].re - t7[2].im * t7[2].im;
  l13 = t7[2].re * t7[2].im;
  l13 += l13;
  l165_im = (((((l142.re - l11_tmp * l165_im) - l147.re) + l148.re) +
              l9 * t7[2].re * 2.0) +
             l7 * l165_im) -
            l158_im * t7[2].re * 2.0;
  l13 = (((0.0 - l11_tmp * l13) + l9 * t7[2].im * 2.0) + l7 * l13) -
        l158_im * t7[2].im * 2.0;
  if (l13 == 0.0) {
    t[6].re = l165_im / l120;
    t[6].im = 0.0;
  } else if (l165_im == 0.0) {
    t[6].re = 0.0;
    t[6].im = l13 / l120;
  } else {
    t[6].re = l165_im / l120;
    t[6].im = l13 / l120;
  }
  l165_im = -(l84 + J_max * t7[2].re);
  l13 = -(J_max * t7[2].im);
  if (l13 == 0.0) {
    t[10].re = l165_im / J_min;
    t[10].im = 0.0;
  } else if (l165_im == 0.0) {
    t[10].re = 0.0;
    t[10].im = l13 / J_min;
  } else {
    t[10].re = l165_im / J_min;
    t[10].im = l13 / J_min;
  }
  t[14].re = 0.0;
  t[14].im = 0.0;
  t[18].re = 0.0;
  t[18].im = 0.0;
  t[22].re = 0.0;
  t[22].im = 0.0;
  t[26] = t7[2];
  l165_im = t7[3].re * t7[3].re - t7[3].im * t7[3].im;
  l13 = t7[3].re * t7[3].im;
  l13 += l13;
  l165_im = (((((l142.re - l11_tmp * l165_im) - l147.re) + l148.re) +
              l9 * t7[3].re * 2.0) +
             l7 * l165_im) -
            l158_im * t7[3].re * 2.0;
  l13 = (((0.0 - l11_tmp * l13) + l9 * t7[3].im * 2.0) + l7 * l13) -
        l158_im * t7[3].im * 2.0;
  if (l13 == 0.0) {
    t[7].re = l165_im / l120;
    t[7].im = 0.0;
  } else if (l165_im == 0.0) {
    t[7].re = 0.0;
    t[7].im = l13 / l120;
  } else {
    t[7].re = l165_im / l120;
    t[7].im = l13 / l120;
  }
  l165_im = -(l84 + J_max * t7[3].re);
  l13 = -(J_max * t7[3].im);
  if (l13 == 0.0) {
    t[11].re = l165_im / J_min;
    t[11].im = 0.0;
  } else if (l165_im == 0.0) {
    t[11].re = 0.0;
    t[11].im = l13 / J_min;
  } else {
    t[11].re = l165_im / J_min;
    t[11].im = l13 / J_min;
  }
  t[15].re = 0.0;
  t[15].im = 0.0;
  t[19].re = 0.0;
  t[19].im = 0.0;
  t[23].re = 0.0;
  t[23].im = 0.0;
  t[27] = t7[3];
}

// End of code generation (abceg_O_AP.cpp)
