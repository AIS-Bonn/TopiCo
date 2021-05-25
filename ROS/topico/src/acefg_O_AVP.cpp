//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// acefg_O_AVP.cpp
//
// Code generation for function 'acefg_O_AVP'
//

// Include files
#include "acefg_O_AVP.h"
#include "power.h"
#include "rt_nonfinite.h"
#include "sqrt.h"
#include "topico_wrapper_data.h"
#include "topico_wrapper_rtwutil.h"
#include <cmath>

// Function Definitions
void acefg_O_AVP(double P_init, double V_init, double A_init, double P_wayp,
                 double V_wayp, double A_wayp, double A_min, double J_max,
                 double J_min, creal_T t[28])
{
  creal_T t3[4];
  creal_T dc;
  creal_T l112;
  creal_T l117;
  double a_tmp;
  double a_tmp_tmp;
  double b_l63_tmp;
  double l10;
  double l119_im;
  double l119_re;
  double l12;
  double l120_im;
  double l120_re;
  double l123_im;
  double l123_re;
  double l130_im;
  double l28_tmp;
  double l2_tmp;
  double l37_tmp;
  double l39_tmp;
  double l5;
  double l59;
  double l60;
  double l61;
  double l63;
  double l63_tmp;
  double l68;
  double l6_tmp;
  double l71_tmp;
  double l80;
  double l80_re_tmp;
  double l81;
  double l82;
  double l91;
  double l92;
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
  //  Generated on 28-Aug-2019 12:21:18
  l2_tmp = A_init * A_init;
  l5 = A_min * A_min;
  l6_tmp = A_wayp * A_wayp;
  l9_tmp = J_min * J_min;
  l10 = rt_powd_snf(J_min, 3.0);
  l12 = J_max * J_max;
  l28_tmp = J_min * J_max;
  l37_tmp = J_max * V_init;
  l39_tmp = J_max * V_wayp;
  l63_tmp = A_min * J_max;
  b_l63_tmp = A_min * J_min;
  l63 =
      (A_min * l10 * 8.0 + b_l63_tmp * l12 * 4.0) + -(l63_tmp * l9_tmp * 12.0);
  l59 = 1.0 /
        ((l9_tmp * l9_tmp * 3.0 + -(J_max * l10 * 6.0)) + l9_tmp * l12 * 3.0);
  l10 = l63 * l63;
  l60 = l59 * l59;
  l61 = rt_powd_snf(l59, 3.0);
  l68 = l59 * l63 / 4.0;
  l71_tmp = l60 * l60 * (l10 * l10);
  l130_im = ((((l28_tmp * l2_tmp * 6.0 - l28_tmp * l5 * 6.0) -
               J_min * V_init * l12 * 12.0) +
              l37_tmp * l9_tmp * 12.0) -
             l2_tmp * l9_tmp * 6.0) +
            l5 * l9_tmp * 6.0;
  l80 = l60 * l10 * 0.375 + -l59 * l130_im;
  l91 = l61 * rt_powd_snf(l63, 3.0) / 8.0 + l60 * l63 * l130_im * -0.5;
  l81 = l80 * l80;
  l82 = rt_powd_snf(l80, 3.0);
  l92 = l91 * l91;
  a_tmp_tmp = l61 * l10 * l130_im;
  l12 = l59 * (((((((((((((((l2_tmp * l2_tmp * 3.0 - l6_tmp * l6_tmp * 3.0) -
                            A_min * rt_powd_snf(A_init, 3.0) * 8.0) +
                           A_min * rt_powd_snf(A_wayp, 3.0) * 8.0) +
                          A_init * A_min * J_max * V_init * 24.0) -
                         A_min * A_wayp * J_max * V_wayp * 24.0) -
                        A_min * P_init * l12 * 24.0) +
                       A_min * P_wayp * l12 * 24.0) -
                      l37_tmp * l2_tmp * 12.0) -
                     l37_tmp * l5 * 12.0) +
                    l39_tmp * l5 * 12.0) +
                   l39_tmp * l6_tmp * 12.0) +
                  l2_tmp * l5 * 6.0) -
                 l5 * l6_tmp * 6.0) +
                l12 * (V_init * V_init) * 12.0) -
               l12 * (V_wayp * V_wayp) * 12.0);
  a_tmp = (-(l71_tmp * 0.01171875) + l12) + a_tmp_tmp / 16.0;
  l112.re = ((((l92 * l92 * 27.0 + -(l82 * l92 * 4.0)) +
               rt_powd_snf(a_tmp, 3.0) * -256.0) +
              l81 * l81 * a_tmp * -16.0) +
             l81 * (a_tmp * a_tmp) * 128.0) +
            l80 * l92 * a_tmp * 144.0;
  l112.im = 0.0;
  coder::internal::scalar::b_sqrt(&l112);
  l63 = 1.7320508075688772 * l112.re;
  l59 = 1.7320508075688772 * l112.im;
  if (l59 == 0.0) {
    l130_im = l63 / 18.0;
    l61 = 0.0;
  } else if (l63 == 0.0) {
    l130_im = 0.0;
    l61 = l59 / 18.0;
  } else {
    l130_im = l63 / 18.0;
    l61 = l59 / 18.0;
  }
  l5 = l80 * a_tmp;
  l117.re = ((-(l82 / 27.0) + l92 / 2.0) + l5 * 1.3333333333333333) + l130_im;
  l117.im = l61;
  l112 = coder::power(l117);
  dc = coder::b_power(l117);
  if (dc.im == 0.0) {
    l120_re = 1.0 / dc.re;
    l120_im = 0.0;
  } else if (dc.re == 0.0) {
    l120_re = 0.0;
    l120_im = -(1.0 / dc.im);
  } else {
    l61 = std::abs(dc.re);
    l10 = std::abs(dc.im);
    if (l61 > l10) {
      l10 = dc.im / dc.re;
      l60 = dc.re + l10 * dc.im;
      l120_re = (l10 * 0.0 + 1.0) / l60;
      l120_im = (0.0 - l10) / l60;
    } else if (l10 == l61) {
      if (dc.re > 0.0) {
        l10 = 0.5;
      } else {
        l10 = -0.5;
      }
      if (dc.im > 0.0) {
        l60 = 0.5;
      } else {
        l60 = -0.5;
      }
      l120_re = (l10 + 0.0 * l60) / l61;
      l120_im = (0.0 * l10 - l60) / l61;
    } else {
      l10 = dc.re / dc.im;
      l60 = dc.im + l10 * dc.re;
      l120_re = l10 / l60;
      l120_im = (l10 * 0.0 - 1.0) / l60;
    }
  }
  l119_re = l112.re * l112.re - l112.im * l112.im;
  l10 = l112.re * l112.im;
  l119_im = l10 + l10;
  dc.re = ((-(l82 * 2.0) + l92 * 27.0) + l5 * 72.0) + l63 * 3.0;
  dc.im = l59 * 3.0;
  coder::internal::scalar::b_sqrt(&dc);
  l123_re = 3.0 * (2.4494897427831779 * l91 * dc.re);
  l123_im = 3.0 * (2.4494897427831779 * l91 * dc.im);
  l80_re_tmp = l80 * l112.re;
  l91 = l80 * l112.im;
  l117.re = ((((-(l71_tmp * 0.140625) + a_tmp_tmp * 0.75) + l81) + l12 * 12.0) +
             l119_re * 9.0) +
            l80_re_tmp * 6.0;
  l117.im = l119_im * 9.0 + l91 * 6.0;
  l112 = l117;
  coder::internal::scalar::b_sqrt(&l112);
  dc = coder::c_power(l117);
  if (dc.im == 0.0) {
    l92 = 1.0 / dc.re;
    l61 = 0.0;
  } else if (dc.re == 0.0) {
    l92 = 0.0;
    l61 = -(1.0 / dc.im);
  } else {
    l61 = std::abs(dc.re);
    l10 = std::abs(dc.im);
    if (l61 > l10) {
      l10 = dc.im / dc.re;
      l60 = dc.re + l10 * dc.im;
      l92 = (l10 * 0.0 + 1.0) / l60;
      l61 = (0.0 - l10) / l60;
    } else if (l10 == l61) {
      if (dc.re > 0.0) {
        l10 = 0.5;
      } else {
        l10 = -0.5;
      }
      if (dc.im > 0.0) {
        l60 = 0.5;
      } else {
        l60 = -0.5;
      }
      l92 = (l10 + 0.0 * l60) / l61;
      l61 = (0.0 * l10 - l60) / l61;
    } else {
      l10 = dc.re / dc.im;
      l60 = dc.im + l10 * dc.re;
      l92 = l10 / l60;
      l61 = (l10 * 0.0 - 1.0) / l60;
    }
  }
  l82 = -12.0 * (a_tmp * l112.re);
  l130_im = -12.0 * (a_tmp * l112.im);
  l59 = -9.0 * (l119_re * l112.re - l119_im * l112.im);
  l5 = -9.0 * (l119_re * l112.im + l119_im * l112.re);
  l10 = l120_re * l112.re - l120_im * l112.im;
  l60 = l120_re * l112.im + l120_im * l112.re;
  if (l60 == 0.0) {
    l12 = l10 / 6.0;
    l63 = 0.0;
  } else if (l10 == 0.0) {
    l12 = 0.0;
    l63 = l60 / 6.0;
  } else {
    l12 = l10 / 6.0;
    l63 = l60 / 6.0;
  }
  l119_re = 12.0 * (l80_re_tmp * l112.re - l91 * l112.im);
  l119_im = 12.0 * (l80_re_tmp * l112.im + l91 * l112.re);
  l117.re = -(l81 * l112.re);
  l117.im = -(l81 * l112.im);
  dc.re = (((l123_re + l117.re) + l82) + l59) + l119_re;
  dc.im = (((l123_im + l117.im) + l130_im) + l5) + l119_im;
  coder::internal::scalar::b_sqrt(&dc);
  l60 = l120_re * l92 - l120_im * l61;
  l10 = l120_re * l61 + l120_im * l92;
  l120_re = l60 * dc.re - l10 * dc.im;
  l120_im = l60 * dc.im + l10 * dc.re;
  if (l120_im == 0.0) {
    l112.re = l120_re / 6.0;
    l112.im = 0.0;
  } else if (l120_re == 0.0) {
    l112.re = 0.0;
    l112.im = l120_im / 6.0;
  } else {
    l112.re = l120_re / 6.0;
    l112.im = l120_im / 6.0;
  }
  dc.re = (((-l123_re + l117.re) + l82) + l59) + l119_re;
  dc.im = (((-l123_im + l117.im) + l130_im) + l5) + l119_im;
  coder::internal::scalar::b_sqrt(&dc);
  l120_re = l60 * dc.re - l10 * dc.im;
  l120_im = l60 * dc.im + l10 * dc.re;
  if (l120_im == 0.0) {
    l117.re = l120_re / 6.0;
    l117.im = 0.0;
  } else if (l120_re == 0.0) {
    l117.re = 0.0;
    l117.im = l120_im / 6.0;
  } else {
    l117.re = l120_re / 6.0;
    l117.im = l120_im / 6.0;
  }
  l60 = l68 + -l12;
  t3[0].re = l60 - l117.re;
  t3[0].im = -l63 - l117.im;
  t3[1].re = l60 + l117.re;
  t3[1].im = -l63 + l117.im;
  l60 = l68 + l12;
  t3[2].re = l60 - l112.re;
  t3[2].im = l63 - l112.im;
  t3[3].re = l60 + l112.re;
  t3[3].im = l63 + l112.im;
  l63 = -(1.0 / J_max * (A_min + -A_wayp));
  l112.re = A_init - A_min;
  l117.re = l37_tmp * 2.0 - l39_tmp * 2.0;
  l130_im = t3[0].re * t3[0].re - t3[0].im * t3[0].im;
  l10 = t3[0].re * t3[0].im;
  l61 = l10 + l10;
  l10 = -(l112.re + J_min * t3[0].re);
  l60 = -(J_min * t3[0].im);
  if (l60 == 0.0) {
    t[0].re = l10 / J_max;
    t[0].im = 0.0;
  } else if (l10 == 0.0) {
    t[0].re = 0.0;
    t[0].im = l60 / J_max;
  } else {
    t[0].re = l10 / J_max;
    t[0].im = l60 / J_max;
  }
  t[4].re = 0.0;
  t[4].im = 0.0;
  t[8] = t3[0];
  t[12].re = 0.0;
  t[12].im = 0.0;
  t[16].re = 0.0;
  t[16].im = 0.0;
  l10 = ((((((l117.re + l9_tmp * l130_im) - l2_tmp) + l6_tmp) -
           b_l63_tmp * t3[0].re * 2.0) +
          l63_tmp * t3[0].re * 2.0) -
         l28_tmp * l130_im) *
        -0.5;
  l60 = (((l9_tmp * l61 - b_l63_tmp * t3[0].im * 2.0) +
          l63_tmp * t3[0].im * 2.0) -
         l28_tmp * l61) *
        -0.5;
  if (l60 == 0.0) {
    t[20].re = l10 / l63_tmp;
    t[20].im = 0.0;
  } else if (l10 == 0.0) {
    t[20].re = 0.0;
    t[20].im = l60 / l63_tmp;
  } else {
    t[20].re = l10 / l63_tmp;
    t[20].im = l60 / l63_tmp;
  }
  l130_im = t3[1].re * t3[1].re - t3[1].im * t3[1].im;
  l10 = t3[1].re * t3[1].im;
  l61 = l10 + l10;
  l10 = -(l112.re + J_min * t3[1].re);
  l60 = -(J_min * t3[1].im);
  if (l60 == 0.0) {
    t[1].re = l10 / J_max;
    t[1].im = 0.0;
  } else if (l10 == 0.0) {
    t[1].re = 0.0;
    t[1].im = l60 / J_max;
  } else {
    t[1].re = l10 / J_max;
    t[1].im = l60 / J_max;
  }
  t[5].re = 0.0;
  t[5].im = 0.0;
  t[9] = t3[1];
  t[13].re = 0.0;
  t[13].im = 0.0;
  t[17].re = 0.0;
  t[17].im = 0.0;
  l10 = ((((((l117.re + l9_tmp * l130_im) - l2_tmp) + l6_tmp) -
           b_l63_tmp * t3[1].re * 2.0) +
          l63_tmp * t3[1].re * 2.0) -
         l28_tmp * l130_im) *
        -0.5;
  l60 = (((l9_tmp * l61 - b_l63_tmp * t3[1].im * 2.0) +
          l63_tmp * t3[1].im * 2.0) -
         l28_tmp * l61) *
        -0.5;
  if (l60 == 0.0) {
    t[21].re = l10 / l63_tmp;
    t[21].im = 0.0;
  } else if (l10 == 0.0) {
    t[21].re = 0.0;
    t[21].im = l60 / l63_tmp;
  } else {
    t[21].re = l10 / l63_tmp;
    t[21].im = l60 / l63_tmp;
  }
  l130_im = t3[2].re * t3[2].re - t3[2].im * t3[2].im;
  l10 = t3[2].re * t3[2].im;
  l61 = l10 + l10;
  l10 = -(l112.re + J_min * t3[2].re);
  l60 = -(J_min * t3[2].im);
  if (l60 == 0.0) {
    t[2].re = l10 / J_max;
    t[2].im = 0.0;
  } else if (l10 == 0.0) {
    t[2].re = 0.0;
    t[2].im = l60 / J_max;
  } else {
    t[2].re = l10 / J_max;
    t[2].im = l60 / J_max;
  }
  t[6].re = 0.0;
  t[6].im = 0.0;
  t[10] = t3[2];
  t[14].re = 0.0;
  t[14].im = 0.0;
  t[18].re = 0.0;
  t[18].im = 0.0;
  l10 = ((((((l117.re + l9_tmp * l130_im) - l2_tmp) + l6_tmp) -
           b_l63_tmp * t3[2].re * 2.0) +
          l63_tmp * t3[2].re * 2.0) -
         l28_tmp * l130_im) *
        -0.5;
  l60 = (((l9_tmp * l61 - b_l63_tmp * t3[2].im * 2.0) +
          l63_tmp * t3[2].im * 2.0) -
         l28_tmp * l61) *
        -0.5;
  if (l60 == 0.0) {
    t[22].re = l10 / l63_tmp;
    t[22].im = 0.0;
  } else if (l10 == 0.0) {
    t[22].re = 0.0;
    t[22].im = l60 / l63_tmp;
  } else {
    t[22].re = l10 / l63_tmp;
    t[22].im = l60 / l63_tmp;
  }
  l130_im = t3[3].re * t3[3].re - t3[3].im * t3[3].im;
  l10 = t3[3].re * t3[3].im;
  l61 = l10 + l10;
  l10 = -(l112.re + J_min * t3[3].re);
  l60 = -(J_min * t3[3].im);
  if (l60 == 0.0) {
    t[3].re = l10 / J_max;
    t[3].im = 0.0;
  } else if (l10 == 0.0) {
    t[3].re = 0.0;
    t[3].im = l60 / J_max;
  } else {
    t[3].re = l10 / J_max;
    t[3].im = l60 / J_max;
  }
  t[7].re = 0.0;
  t[7].im = 0.0;
  t[11] = t3[3];
  t[15].re = 0.0;
  t[15].im = 0.0;
  t[19].re = 0.0;
  t[19].im = 0.0;
  l10 = ((((((l117.re + l9_tmp * l130_im) - l2_tmp) + l6_tmp) -
           b_l63_tmp * t3[3].re * 2.0) +
          l63_tmp * t3[3].re * 2.0) -
         l28_tmp * l130_im) *
        -0.5;
  l60 = (((l9_tmp * l61 - b_l63_tmp * t3[3].im * 2.0) +
          l63_tmp * t3[3].im * 2.0) -
         l28_tmp * l61) *
        -0.5;
  if (l60 == 0.0) {
    t[23].re = l10 / l63_tmp;
    t[23].im = 0.0;
  } else if (l10 == 0.0) {
    t[23].re = 0.0;
    t[23].im = l60 / l63_tmp;
  } else {
    t[23].re = l10 / l63_tmp;
    t[23].im = l60 / l63_tmp;
  }
  t[24].re = l63;
  t[24].im = 0.0;
  t[25].re = l63;
  t[25].im = 0.0;
  t[26].re = l63;
  t[26].im = 0.0;
  t[27].re = l63;
  t[27].im = 0.0;
}

// End of code generation (acefg_O_AVP.cpp)
