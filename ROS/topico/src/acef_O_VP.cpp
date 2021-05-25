//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// acef_O_VP.cpp
//
// Code generation for function 'acef_O_VP'
//

// Include files
#include "acef_O_VP.h"
#include "power.h"
#include "rt_nonfinite.h"
#include "sqrt.h"
#include "topico_wrapper_data.h"
#include "topico_wrapper_rtwutil.h"
#include <cmath>

// Function Definitions
void acef_O_VP(double P_init, double V_init, double A_init, double P_wayp,
               double V_wayp, double A_min, double J_max, double J_min,
               creal_T t[28])
{
  creal_T t3[4];
  creal_T dc;
  creal_T l101;
  creal_T l106;
  double b_l52_tmp;
  double l10;
  double l108_im;
  double l108_re;
  double l109_im;
  double l109_re;
  double l112_im;
  double l112_re;
  double l116_re;
  double l121_im;
  double l23_tmp;
  double l2_tmp;
  double l35_tmp;
  double l49;
  double l50;
  double l52;
  double l52_tmp;
  double l53;
  double l57;
  double l5_tmp;
  double l69;
  double l72;
  double l72_re_tmp;
  double l73;
  double l74;
  double l7_tmp;
  double l8;
  double l80;
  double l81;
  double l86;
  double l86_tmp;
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
  l5_tmp = A_min * A_min;
  l7_tmp = J_min * J_min;
  l8 = rt_powd_snf(J_min, 3.0);
  l10 = J_max * J_max;
  l23_tmp = J_min * J_max;
  l35_tmp = J_max * V_init;
  l52_tmp = A_min * J_max;
  b_l52_tmp = A_min * J_min;
  l52 = (A_min * l8 * 8.0 + b_l52_tmp * l10 * 4.0) + -(l52_tmp * l7_tmp * 12.0);
  l8 = 1.0 /
       ((l7_tmp * l7_tmp * 3.0 + -(J_max * l8 * 6.0)) + l7_tmp * l10 * 3.0);
  l53 = l52 * l52;
  l49 = l8 * l8;
  l50 = rt_powd_snf(l8, 3.0);
  l57 = l8 * l52 / 4.0;
  l69 = l8 * ((((((((((l5_tmp * l5_tmp + -(l2_tmp * l2_tmp * 3.0)) +
                      A_min * rt_powd_snf(A_init, 3.0) * 8.0) +
                     -(A_init * A_min * J_max * V_init * 24.0)) +
                    A_min * P_init * l10 * 24.0) +
                   l35_tmp * l2_tmp * 12.0) +
                  l35_tmp * l5_tmp * 12.0) +
                 -(A_min * P_wayp * l10 * 24.0)) +
                -(l2_tmp * l5_tmp * 6.0)) +
               l10 * (V_wayp * V_wayp) * 12.0) +
              -(l10 * (V_init * V_init) * 12.0));
  l10 = ((((l23_tmp * l2_tmp * 6.0 - l23_tmp * l5_tmp * 6.0) -
           J_min * V_init * l10 * 12.0) +
          l35_tmp * l7_tmp * 12.0) -
         l2_tmp * l7_tmp * 6.0) +
        l5_tmp * l7_tmp * 6.0;
  l72 = l49 * l53 * 0.375 + -l8 * l10;
  l80 = l50 * rt_powd_snf(l52, 3.0) / 8.0 + l49 * l52 * l10 * -0.5;
  l73 = l72 * l72;
  l74 = rt_powd_snf(l72, 3.0);
  l81 = l80 * l80;
  l86_tmp = l49 * l49 * (l53 * l53);
  l116_re = l50 * l53 * l10;
  l86 = (l86_tmp * 0.01171875 + l116_re * -0.0625) + l69;
  l101.re = ((((l81 * l81 * 27.0 + rt_powd_snf(l86, 3.0) * 256.0) +
               -(l74 * l81 * 4.0)) +
              l73 * l73 * l86 * 16.0) +
             l73 * (l86 * l86) * 128.0) +
            -(l72 * l81 * l86 * 144.0);
  l101.im = 0.0;
  coder::internal::scalar::b_sqrt(&l101);
  l53 = 1.7320508075688772 * l101.re;
  l50 = 1.7320508075688772 * l101.im;
  if (l50 == 0.0) {
    l49 = l53 / 18.0;
    l52 = 0.0;
  } else if (l53 == 0.0) {
    l49 = 0.0;
    l52 = l50 / 18.0;
  } else {
    l49 = l53 / 18.0;
    l52 = l50 / 18.0;
  }
  l121_im = l72 * l86;
  l106.re =
      ((-(l74 / 27.0) + l81 / 2.0) + -(l121_im * 1.3333333333333333)) + l49;
  l106.im = l52;
  l101 = coder::power(l106);
  dc = coder::b_power(l106);
  if (dc.im == 0.0) {
    l109_re = 1.0 / dc.re;
    l109_im = 0.0;
  } else if (dc.re == 0.0) {
    l109_re = 0.0;
    l109_im = -(1.0 / dc.im);
  } else {
    l52 = std::abs(dc.re);
    l8 = std::abs(dc.im);
    if (l52 > l8) {
      l8 = dc.im / dc.re;
      l10 = dc.re + l8 * dc.im;
      l109_re = (l8 * 0.0 + 1.0) / l10;
      l109_im = (0.0 - l8) / l10;
    } else if (l8 == l52) {
      if (dc.re > 0.0) {
        l8 = 0.5;
      } else {
        l8 = -0.5;
      }
      if (dc.im > 0.0) {
        l10 = 0.5;
      } else {
        l10 = -0.5;
      }
      l109_re = (l8 + 0.0 * l10) / l52;
      l109_im = (0.0 * l8 - l10) / l52;
    } else {
      l8 = dc.re / dc.im;
      l10 = dc.im + l8 * dc.re;
      l109_re = l8 / l10;
      l109_im = (l8 * 0.0 - 1.0) / l10;
    }
  }
  l108_re = l101.re * l101.re - l101.im * l101.im;
  l8 = l101.re * l101.im;
  l108_im = l8 + l8;
  dc.re = ((-(l74 * 2.0) + l81 * 27.0) + -(l121_im * 72.0)) + l53 * 3.0;
  dc.im = l50 * 3.0;
  coder::internal::scalar::b_sqrt(&dc);
  l112_re = 3.0 * (2.4494897427831779 * l80 * dc.re);
  l112_im = 3.0 * (2.4494897427831779 * l80 * dc.im);
  l72_re_tmp = l72 * l101.re;
  l80 = l72 * l101.im;
  l106.re =
      ((((-(l86_tmp * 0.140625) + l116_re * 0.75) + -(l69 * 12.0)) + l73) +
       l108_re * 9.0) +
      l72_re_tmp * 6.0;
  l106.im = l108_im * 9.0 + l80 * 6.0;
  l101 = l106;
  coder::internal::scalar::b_sqrt(&l101);
  dc = coder::c_power(l106);
  if (dc.im == 0.0) {
    l116_re = 1.0 / dc.re;
    l49 = 0.0;
  } else if (dc.re == 0.0) {
    l116_re = 0.0;
    l49 = -(1.0 / dc.im);
  } else {
    l52 = std::abs(dc.re);
    l8 = std::abs(dc.im);
    if (l52 > l8) {
      l8 = dc.im / dc.re;
      l10 = dc.re + l8 * dc.im;
      l116_re = (l8 * 0.0 + 1.0) / l10;
      l49 = (0.0 - l8) / l10;
    } else if (l8 == l52) {
      if (dc.re > 0.0) {
        l8 = 0.5;
      } else {
        l8 = -0.5;
      }
      if (dc.im > 0.0) {
        l10 = 0.5;
      } else {
        l10 = -0.5;
      }
      l116_re = (l8 + 0.0 * l10) / l52;
      l49 = (0.0 * l8 - l10) / l52;
    } else {
      l8 = dc.re / dc.im;
      l10 = dc.im + l8 * dc.re;
      l116_re = l8 / l10;
      l49 = (l8 * 0.0 - 1.0) / l10;
    }
  }
  l81 = 12.0 * (l86 * l101.re);
  l53 = 12.0 * (l86 * l101.im);
  l50 = -9.0 * (l108_re * l101.re - l108_im * l101.im);
  l121_im = -9.0 * (l108_re * l101.im + l108_im * l101.re);
  l8 = l109_re * l101.re - l109_im * l101.im;
  l10 = l109_re * l101.im + l109_im * l101.re;
  if (l10 == 0.0) {
    l74 = l8 / 6.0;
    l52 = 0.0;
  } else if (l8 == 0.0) {
    l74 = 0.0;
    l52 = l10 / 6.0;
  } else {
    l74 = l8 / 6.0;
    l52 = l10 / 6.0;
  }
  l108_re = 12.0 * (l72_re_tmp * l101.re - l80 * l101.im);
  l108_im = 12.0 * (l72_re_tmp * l101.im + l80 * l101.re);
  l106.re = -(l73 * l101.re);
  l106.im = -(l73 * l101.im);
  dc.re = (((l112_re + l106.re) + l81) + l50) + l108_re;
  dc.im = (((l112_im + l106.im) + l53) + l121_im) + l108_im;
  coder::internal::scalar::b_sqrt(&dc);
  l10 = l109_re * l116_re - l109_im * l49;
  l8 = l109_re * l49 + l109_im * l116_re;
  l109_re = l10 * dc.re - l8 * dc.im;
  l109_im = l10 * dc.im + l8 * dc.re;
  if (l109_im == 0.0) {
    l101.re = l109_re / 6.0;
    l101.im = 0.0;
  } else if (l109_re == 0.0) {
    l101.re = 0.0;
    l101.im = l109_im / 6.0;
  } else {
    l101.re = l109_re / 6.0;
    l101.im = l109_im / 6.0;
  }
  dc.re = (((-l112_re + l106.re) + l81) + l50) + l108_re;
  dc.im = (((-l112_im + l106.im) + l53) + l121_im) + l108_im;
  coder::internal::scalar::b_sqrt(&dc);
  l109_re = l10 * dc.re - l8 * dc.im;
  l109_im = l10 * dc.im + l8 * dc.re;
  if (l109_im == 0.0) {
    l106.re = l109_re / 6.0;
    l106.im = 0.0;
  } else if (l109_re == 0.0) {
    l106.re = 0.0;
    l106.im = l109_im / 6.0;
  } else {
    l106.re = l109_re / 6.0;
    l106.im = l109_im / 6.0;
  }
  l8 = l57 + -l74;
  t3[0].re = l8 - l106.re;
  t3[0].im = -l52 - l106.im;
  t3[1].re = l8 + l106.re;
  t3[1].im = -l52 + l106.im;
  l8 = l57 + l74;
  t3[2].re = l8 - l101.re;
  t3[2].im = l52 - l101.im;
  t3[3].re = l8 + l101.re;
  t3[3].im = l52 + l101.im;
  l101.re = A_init - A_min;
  l106.re = l35_tmp * 2.0 - J_max * V_wayp * 2.0;
  l49 = t3[0].re * t3[0].re - t3[0].im * t3[0].im;
  l8 = t3[0].re * t3[0].im;
  l52 = l8 + l8;
  l8 = -(l101.re + J_min * t3[0].re);
  l10 = -(J_min * t3[0].im);
  if (l10 == 0.0) {
    t[0].re = l8 / J_max;
    t[0].im = 0.0;
  } else if (l8 == 0.0) {
    t[0].re = 0.0;
    t[0].im = l10 / J_max;
  } else {
    t[0].re = l8 / J_max;
    t[0].im = l10 / J_max;
  }
  t[4].re = 0.0;
  t[4].im = 0.0;
  t[8] = t3[0];
  t[12].re = 0.0;
  t[12].im = 0.0;
  t[16].re = 0.0;
  t[16].im = 0.0;
  l8 = ((((((l106.re + l7_tmp * l49) - l2_tmp) + l5_tmp) -
          b_l52_tmp * t3[0].re * 2.0) +
         l52_tmp * t3[0].re * 2.0) -
        l23_tmp * l49) *
       -0.5;
  l10 = (((l7_tmp * l52 - b_l52_tmp * t3[0].im * 2.0) +
          l52_tmp * t3[0].im * 2.0) -
         l23_tmp * l52) *
        -0.5;
  if (l10 == 0.0) {
    t[20].re = l8 / l52_tmp;
    t[20].im = 0.0;
  } else if (l8 == 0.0) {
    t[20].re = 0.0;
    t[20].im = l10 / l52_tmp;
  } else {
    t[20].re = l8 / l52_tmp;
    t[20].im = l10 / l52_tmp;
  }
  t[24].re = 0.0;
  t[24].im = 0.0;
  l49 = t3[1].re * t3[1].re - t3[1].im * t3[1].im;
  l8 = t3[1].re * t3[1].im;
  l52 = l8 + l8;
  l8 = -(l101.re + J_min * t3[1].re);
  l10 = -(J_min * t3[1].im);
  if (l10 == 0.0) {
    t[1].re = l8 / J_max;
    t[1].im = 0.0;
  } else if (l8 == 0.0) {
    t[1].re = 0.0;
    t[1].im = l10 / J_max;
  } else {
    t[1].re = l8 / J_max;
    t[1].im = l10 / J_max;
  }
  t[5].re = 0.0;
  t[5].im = 0.0;
  t[9] = t3[1];
  t[13].re = 0.0;
  t[13].im = 0.0;
  t[17].re = 0.0;
  t[17].im = 0.0;
  l8 = ((((((l106.re + l7_tmp * l49) - l2_tmp) + l5_tmp) -
          b_l52_tmp * t3[1].re * 2.0) +
         l52_tmp * t3[1].re * 2.0) -
        l23_tmp * l49) *
       -0.5;
  l10 = (((l7_tmp * l52 - b_l52_tmp * t3[1].im * 2.0) +
          l52_tmp * t3[1].im * 2.0) -
         l23_tmp * l52) *
        -0.5;
  if (l10 == 0.0) {
    t[21].re = l8 / l52_tmp;
    t[21].im = 0.0;
  } else if (l8 == 0.0) {
    t[21].re = 0.0;
    t[21].im = l10 / l52_tmp;
  } else {
    t[21].re = l8 / l52_tmp;
    t[21].im = l10 / l52_tmp;
  }
  t[25].re = 0.0;
  t[25].im = 0.0;
  l49 = t3[2].re * t3[2].re - t3[2].im * t3[2].im;
  l8 = t3[2].re * t3[2].im;
  l52 = l8 + l8;
  l8 = -(l101.re + J_min * t3[2].re);
  l10 = -(J_min * t3[2].im);
  if (l10 == 0.0) {
    t[2].re = l8 / J_max;
    t[2].im = 0.0;
  } else if (l8 == 0.0) {
    t[2].re = 0.0;
    t[2].im = l10 / J_max;
  } else {
    t[2].re = l8 / J_max;
    t[2].im = l10 / J_max;
  }
  t[6].re = 0.0;
  t[6].im = 0.0;
  t[10] = t3[2];
  t[14].re = 0.0;
  t[14].im = 0.0;
  t[18].re = 0.0;
  t[18].im = 0.0;
  l8 = ((((((l106.re + l7_tmp * l49) - l2_tmp) + l5_tmp) -
          b_l52_tmp * t3[2].re * 2.0) +
         l52_tmp * t3[2].re * 2.0) -
        l23_tmp * l49) *
       -0.5;
  l10 = (((l7_tmp * l52 - b_l52_tmp * t3[2].im * 2.0) +
          l52_tmp * t3[2].im * 2.0) -
         l23_tmp * l52) *
        -0.5;
  if (l10 == 0.0) {
    t[22].re = l8 / l52_tmp;
    t[22].im = 0.0;
  } else if (l8 == 0.0) {
    t[22].re = 0.0;
    t[22].im = l10 / l52_tmp;
  } else {
    t[22].re = l8 / l52_tmp;
    t[22].im = l10 / l52_tmp;
  }
  t[26].re = 0.0;
  t[26].im = 0.0;
  l49 = t3[3].re * t3[3].re - t3[3].im * t3[3].im;
  l8 = t3[3].re * t3[3].im;
  l52 = l8 + l8;
  l8 = -(l101.re + J_min * t3[3].re);
  l10 = -(J_min * t3[3].im);
  if (l10 == 0.0) {
    t[3].re = l8 / J_max;
    t[3].im = 0.0;
  } else if (l8 == 0.0) {
    t[3].re = 0.0;
    t[3].im = l10 / J_max;
  } else {
    t[3].re = l8 / J_max;
    t[3].im = l10 / J_max;
  }
  t[7].re = 0.0;
  t[7].im = 0.0;
  t[11] = t3[3];
  t[15].re = 0.0;
  t[15].im = 0.0;
  t[19].re = 0.0;
  t[19].im = 0.0;
  l8 = ((((((l106.re + l7_tmp * l49) - l2_tmp) + l5_tmp) -
          b_l52_tmp * t3[3].re * 2.0) +
         l52_tmp * t3[3].re * 2.0) -
        l23_tmp * l49) *
       -0.5;
  l10 = (((l7_tmp * l52 - b_l52_tmp * t3[3].im * 2.0) +
          l52_tmp * t3[3].im * 2.0) -
         l23_tmp * l52) *
        -0.5;
  if (l10 == 0.0) {
    t[23].re = l8 / l52_tmp;
    t[23].im = 0.0;
  } else if (l8 == 0.0) {
    t[23].re = 0.0;
    t[23].im = l10 / l52_tmp;
  } else {
    t[23].re = l8 / l52_tmp;
    t[23].im = l10 / l52_tmp;
  }
  t[27].re = 0.0;
  t[27].im = 0.0;
}

// End of code generation (acef_O_VP.cpp)
