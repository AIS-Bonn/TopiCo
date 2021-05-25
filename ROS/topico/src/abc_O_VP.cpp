//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// abc_O_VP.cpp
//
// Code generation for function 'abc_O_VP'
//

// Include files
#include "abc_O_VP.h"
#include "power.h"
#include "rt_nonfinite.h"
#include "sqrt.h"
#include "topico_wrapper_data.h"
#include "topico_wrapper_rtwutil.h"
#include <cmath>

// Function Definitions
void abc_O_VP(double P_init, double V_init, double A_init, double P_wayp,
              double V_wayp, double A_max, double J_max, double J_min,
              creal_T t[28])
{
  creal_T t3[4];
  creal_T dc;
  creal_T l78;
  creal_T l83;
  double b_l65_tmp;
  double l11;
  double l12;
  double l13;
  double l29;
  double l2_tmp;
  double l45;
  double l45_re_tmp;
  double l46;
  double l47;
  double l51;
  double l53;
  double l5_tmp;
  double l61_tmp;
  double l65;
  double l65_tmp;
  double l7;
  double l8;
  double l85_im;
  double l85_re;
  double l86_im;
  double l86_re;
  double l89_im;
  double l89_re;
  double l96_re;
  double l98_re;
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
  l8 = J_max * J_max;
  l11 = 1.0 / J_min;
  l7 = l5_tmp * l5_tmp;
  l12 = l11 * l11;
  l13 = rt_powd_snf(l11, 3.0);
  l29 = A_max * l11 / 3.0;
  l45 = V_wayp * l11 * 4.0 + l5_tmp * l12 * 0.66666666666666663;
  l51 = A_max * V_wayp * l12 * 2.6666666666666665 +
        rt_powd_snf(A_max, 3.0) * l13 * 0.29629629629629628;
  l46 = l45 * l45;
  l47 = rt_powd_snf(l45, 3.0);
  l53 = l51 * l51;
  l61_tmp = J_max * V_init;
  l65_tmp = l7 * (l12 * l12);
  b_l65_tmp = V_wayp * l5_tmp * l13;
  l96_re = l12 * (1.0 / l8) *
           ((((((((((l7 + -(l2_tmp * l2_tmp * 3.0)) +
                    A_max * rt_powd_snf(A_init, 3.0) * 8.0) +
                   -(A_init * A_max * J_max * V_init * 24.0)) +
                  A_max * P_init * l8 * 24.0) +
                 l61_tmp * l2_tmp * 12.0) +
                l61_tmp * l5_tmp * 12.0) +
               -(A_max * P_wayp * l8 * 24.0)) +
              -(l2_tmp * l5_tmp * 6.0)) +
             l8 * (V_wayp * V_wayp) * 12.0) +
            -(l8 * (V_init * V_init) * 12.0));
  l65 = (l65_tmp / 27.0 + b_l65_tmp * 0.44444444444444442) + -(l96_re / 3.0);
  l78.re = ((((l53 * l53 * 27.0 + -(l47 * l53 * 4.0)) +
              rt_powd_snf(l65, 3.0) * 256.0) +
             l46 * l46 * l65 * 16.0) +
            l46 * (l65 * l65) * 128.0) +
           -(l45 * l53 * l65 * 144.0);
  l78.im = 0.0;
  coder::internal::scalar::b_sqrt(&l78);
  l8 = 1.7320508075688772 * l78.re;
  l7 = 1.7320508075688772 * l78.im;
  if (l7 == 0.0) {
    l11 = l8 / 18.0;
    l13 = 0.0;
  } else if (l8 == 0.0) {
    l11 = 0.0;
    l13 = l7 / 18.0;
  } else {
    l11 = l8 / 18.0;
    l13 = l7 / 18.0;
  }
  l12 = l45 * l65;
  l83.re = ((-(l47 / 27.0) + l53 / 2.0) + -(l12 * 1.3333333333333333)) + l11;
  l83.im = l13;
  l78 = coder::power(l83);
  dc = coder::b_power(l83);
  if (dc.im == 0.0) {
    l86_re = 1.0 / dc.re;
    l86_im = 0.0;
  } else if (dc.re == 0.0) {
    l86_re = 0.0;
    l86_im = -(1.0 / dc.im);
  } else {
    l98_re = std::abs(dc.re);
    l11 = std::abs(dc.im);
    if (l98_re > l11) {
      l11 = dc.im / dc.re;
      l13 = dc.re + l11 * dc.im;
      l86_re = (l11 * 0.0 + 1.0) / l13;
      l86_im = (0.0 - l11) / l13;
    } else if (l11 == l98_re) {
      if (dc.re > 0.0) {
        l11 = 0.5;
      } else {
        l11 = -0.5;
      }
      if (dc.im > 0.0) {
        l13 = 0.5;
      } else {
        l13 = -0.5;
      }
      l86_re = (l11 + 0.0 * l13) / l98_re;
      l86_im = (0.0 * l11 - l13) / l98_re;
    } else {
      l11 = dc.re / dc.im;
      l13 = dc.im + l11 * dc.re;
      l86_re = l11 / l13;
      l86_im = (l11 * 0.0 - 1.0) / l13;
    }
  }
  l85_re = l78.re * l78.re - l78.im * l78.im;
  l11 = l78.re * l78.im;
  l85_im = l11 + l11;
  dc.re = ((-(l47 * 2.0) + l53 * 27.0) + -(l12 * 72.0)) + l8 * 3.0;
  dc.im = l7 * 3.0;
  coder::internal::scalar::b_sqrt(&dc);
  l89_re = 3.0 * (2.4494897427831779 * l51 * dc.re);
  l89_im = 3.0 * (2.4494897427831779 * l51 * dc.im);
  l45_re_tmp = l45 * l78.re;
  l45 *= l78.im;
  l83.re =
      ((((-(l65_tmp * 0.44444444444444442) + -(b_l65_tmp * 5.333333333333333)) +
         l46) +
        l96_re * 4.0) +
       l85_re * 9.0) +
      l45_re_tmp * 6.0;
  l83.im = l85_im * 9.0 + l45 * 6.0;
  l78 = l83;
  coder::internal::scalar::b_sqrt(&l78);
  dc = coder::c_power(l83);
  if (dc.im == 0.0) {
    l51 = 1.0 / dc.re;
    l7 = 0.0;
  } else if (dc.re == 0.0) {
    l51 = 0.0;
    l7 = -(1.0 / dc.im);
  } else {
    l98_re = std::abs(dc.re);
    l11 = std::abs(dc.im);
    if (l98_re > l11) {
      l11 = dc.im / dc.re;
      l13 = dc.re + l11 * dc.im;
      l51 = (l11 * 0.0 + 1.0) / l13;
      l7 = (0.0 - l11) / l13;
    } else if (l11 == l98_re) {
      if (dc.re > 0.0) {
        l11 = 0.5;
      } else {
        l11 = -0.5;
      }
      if (dc.im > 0.0) {
        l13 = 0.5;
      } else {
        l13 = -0.5;
      }
      l51 = (l11 + 0.0 * l13) / l98_re;
      l7 = (0.0 * l11 - l13) / l98_re;
    } else {
      l11 = dc.re / dc.im;
      l13 = dc.im + l11 * dc.re;
      l51 = l11 / l13;
      l7 = (l11 * 0.0 - 1.0) / l13;
    }
  }
  l96_re = 12.0 * (l65 * l78.re);
  l12 = 12.0 * (l65 * l78.im);
  l98_re = -9.0 * (l85_re * l78.re - l85_im * l78.im);
  l47 = -9.0 * (l85_re * l78.im + l85_im * l78.re);
  l11 = l86_re * l78.re - l86_im * l78.im;
  l13 = l86_re * l78.im + l86_im * l78.re;
  if (l13 == 0.0) {
    l53 = l11 / 6.0;
    l8 = 0.0;
  } else if (l11 == 0.0) {
    l53 = 0.0;
    l8 = l13 / 6.0;
  } else {
    l53 = l11 / 6.0;
    l8 = l13 / 6.0;
  }
  l85_re = 12.0 * (l45_re_tmp * l78.re - l45 * l78.im);
  l85_im = 12.0 * (l45_re_tmp * l78.im + l45 * l78.re);
  l83.re = -(l46 * l78.re);
  l83.im = -(l46 * l78.im);
  dc.re = (((l89_re + l83.re) + l96_re) + l98_re) + l85_re;
  dc.im = (((l89_im + l83.im) + l12) + l47) + l85_im;
  coder::internal::scalar::b_sqrt(&dc);
  l13 = l86_re * l51 - l86_im * l7;
  l11 = l86_re * l7 + l86_im * l51;
  l86_re = l13 * dc.re - l11 * dc.im;
  l86_im = l13 * dc.im + l11 * dc.re;
  if (l86_im == 0.0) {
    l78.re = l86_re / 6.0;
    l78.im = 0.0;
  } else if (l86_re == 0.0) {
    l78.re = 0.0;
    l78.im = l86_im / 6.0;
  } else {
    l78.re = l86_re / 6.0;
    l78.im = l86_im / 6.0;
  }
  dc.re = (((-l89_re + l83.re) + l96_re) + l98_re) + l85_re;
  dc.im = (((-l89_im + l83.im) + l12) + l47) + l85_im;
  coder::internal::scalar::b_sqrt(&dc);
  l86_re = l13 * dc.re - l11 * dc.im;
  l86_im = l13 * dc.im + l11 * dc.re;
  if (l86_im == 0.0) {
    l83.re = l86_re / 6.0;
    l83.im = 0.0;
  } else if (l86_re == 0.0) {
    l83.re = 0.0;
    l83.im = l86_im / 6.0;
  } else {
    l83.re = l86_re / 6.0;
    l83.im = l86_im / 6.0;
  }
  l11 = -l29 + -l53;
  t3[0].re = l11 - l78.re;
  t3[0].im = -l8 - l78.im;
  t3[1].re = l11 + l78.re;
  t3[1].im = -l8 + l78.im;
  l11 = -l29 + l53;
  t3[2].re = l11 - l83.re;
  t3[2].im = l8 - l83.im;
  t3[3].re = l11 + l83.re;
  t3[3].im = l8 + l83.im;
  l7 = A_max * J_max;
  l13 = -(1.0 / J_max * (A_init + -A_max));
  l78.re = ((l61_tmp * 2.0 - J_max * V_wayp * 2.0) - l2_tmp) + l5_tmp;
  l8 = J_min * J_max;
  t[0].re = l13;
  t[0].im = 0.0;
  t[1].re = l13;
  t[1].im = 0.0;
  t[2].re = l13;
  t[2].im = 0.0;
  t[3].re = l13;
  t[3].im = 0.0;
  l13 = t3[0].re * t3[0].im;
  l11 = ((l78.re + l7 * t3[0].re * 2.0) +
         l8 * (t3[0].re * t3[0].re - t3[0].im * t3[0].im)) *
        -0.5;
  l13 = (l7 * t3[0].im * 2.0 + l8 * (l13 + l13)) * -0.5;
  if (l13 == 0.0) {
    t[4].re = l11 / l7;
    t[4].im = 0.0;
  } else if (l11 == 0.0) {
    t[4].re = 0.0;
    t[4].im = l13 / l7;
  } else {
    t[4].re = l11 / l7;
    t[4].im = l13 / l7;
  }
  t[8] = t3[0];
  t[12].re = 0.0;
  t[12].im = 0.0;
  t[16].re = 0.0;
  t[16].im = 0.0;
  t[20].re = 0.0;
  t[20].im = 0.0;
  t[24].re = 0.0;
  t[24].im = 0.0;
  l13 = t3[1].re * t3[1].im;
  l11 = ((l78.re + l7 * t3[1].re * 2.0) +
         l8 * (t3[1].re * t3[1].re - t3[1].im * t3[1].im)) *
        -0.5;
  l13 = (l7 * t3[1].im * 2.0 + l8 * (l13 + l13)) * -0.5;
  if (l13 == 0.0) {
    t[5].re = l11 / l7;
    t[5].im = 0.0;
  } else if (l11 == 0.0) {
    t[5].re = 0.0;
    t[5].im = l13 / l7;
  } else {
    t[5].re = l11 / l7;
    t[5].im = l13 / l7;
  }
  t[9] = t3[1];
  t[13].re = 0.0;
  t[13].im = 0.0;
  t[17].re = 0.0;
  t[17].im = 0.0;
  t[21].re = 0.0;
  t[21].im = 0.0;
  t[25].re = 0.0;
  t[25].im = 0.0;
  l13 = t3[2].re * t3[2].im;
  l11 = ((l78.re + l7 * t3[2].re * 2.0) +
         l8 * (t3[2].re * t3[2].re - t3[2].im * t3[2].im)) *
        -0.5;
  l13 = (l7 * t3[2].im * 2.0 + l8 * (l13 + l13)) * -0.5;
  if (l13 == 0.0) {
    t[6].re = l11 / l7;
    t[6].im = 0.0;
  } else if (l11 == 0.0) {
    t[6].re = 0.0;
    t[6].im = l13 / l7;
  } else {
    t[6].re = l11 / l7;
    t[6].im = l13 / l7;
  }
  t[10] = t3[2];
  t[14].re = 0.0;
  t[14].im = 0.0;
  t[18].re = 0.0;
  t[18].im = 0.0;
  t[22].re = 0.0;
  t[22].im = 0.0;
  t[26].re = 0.0;
  t[26].im = 0.0;
  l13 = t3[3].re * t3[3].im;
  l11 = ((l78.re + l7 * t3[3].re * 2.0) +
         l8 * (t3[3].re * t3[3].re - t3[3].im * t3[3].im)) *
        -0.5;
  l13 = (l7 * t3[3].im * 2.0 + l8 * (l13 + l13)) * -0.5;
  if (l13 == 0.0) {
    t[7].re = l11 / l7;
    t[7].im = 0.0;
  } else if (l11 == 0.0) {
    t[7].re = 0.0;
    t[7].im = l13 / l7;
  } else {
    t[7].re = l11 / l7;
    t[7].im = l13 / l7;
  }
  t[11] = t3[3];
  t[15].re = 0.0;
  t[15].im = 0.0;
  t[19].re = 0.0;
  t[19].im = 0.0;
  t[23].re = 0.0;
  t[23].im = 0.0;
  t[27].re = 0.0;
  t[27].im = 0.0;
}

// End of code generation (abc_O_VP.cpp)
