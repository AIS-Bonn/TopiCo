//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// aceg_T_P.cpp
//
// Code generation for function 'aceg_T_P'
//

// Include files
#include "aceg_T_P.h"
#include "power.h"
#include "rt_nonfinite.h"
#include "sqrt.h"
#include "topico_wrapper_data.h"
#include "topico_wrapper_rtwutil.h"
#include <cmath>

// Function Definitions
void aceg_T_P(double P_init, double V_init, double A_init, double P_wayp,
              double V_min, double J_max, double J_min, double T, creal_T t[28])
{
  creal_T t3[4];
  creal_T dc;
  creal_T l153;
  creal_T l158;
  creal_T l159;
  double b_l112_tmp;
  double c_l112_tmp;
  double d_l112_tmp;
  double e_l112_tmp;
  double l10;
  double l102;
  double l104;
  double l11;
  double l112;
  double l112_tmp;
  double l113;
  double l12;
  double l125;
  double l126;
  double l127;
  double l158_tmp;
  double l160_re;
  double l161_im;
  double l161_re;
  double l164_im;
  double l174_re;
  double l175_im;
  double l24;
  double l2_tmp;
  double l3;
  double l56;
  double l5_tmp;
  double l6;
  double l8_tmp;
  double l9;
  double l93;
  double l98_tmp;
  double re;
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
  //  Generated on 03-Sep-2019 14:44:33
  l2_tmp = A_init * A_init;
  l3 = rt_powd_snf(A_init, 3.0);
  l5_tmp = J_min * J_min;
  l6 = rt_powd_snf(J_min, 3.0);
  l8_tmp = J_max * J_max;
  l9 = rt_powd_snf(J_max, 3.0);
  l11 = T * T;
  l12 = rt_powd_snf(T, 3.0);
  l10 = l8_tmp * l8_tmp;
  l24 = J_max * l6 * 12.0;
  l56 = J_min * V_min * l8_tmp * 24.0;
  l112_tmp = A_init * J_min;
  l93 = J_min * l9;
  b_l112_tmp = J_min * J_max;
  l174_re = l5_tmp * l8_tmp;
  c_l112_tmp = A_init * J_max;
  d_l112_tmp = J_max * V_min;
  e_l112_tmp = J_max * V_init;
  l112 = (((((((((((((V_min * l9 * 12.0 + -(V_init * l9 * 12.0)) +
                     A_init * T * l9 * 24.0) +
                    J_min * V_init * l8_tmp * 24.0) +
                   d_l112_tmp * l5_tmp * 12.0) +
                  -(b_l112_tmp * l2_tmp * 36.0)) +
                 -(e_l112_tmp * l5_tmp * 12.0)) +
                -l56) +
               l2_tmp * l5_tmp * 18.0) +
              l2_tmp * l8_tmp * 18.0) +
             l10 * l11 * 12.0) +
            c_l112_tmp * T * l5_tmp * 24.0) +
           -(l112_tmp * T * l8_tmp * 48.0)) +
          -(l93 * l11 * 24.0)) +
         l174_re * l11 * 12.0;
  l175_im = l112_tmp * J_max;
  l164_im = A_init * l9;
  l127 = l112_tmp * l8_tmp;
  l113 = ((((((((((((((((J_min * l3 * 12.0 + P_wayp * l9 * 24.0) +
                        l175_im * V_min * 24.0) +
                       -(J_max * l3 * 12.0)) +
                      -(P_init * l9 * 24.0)) +
                     -(l175_im * V_init * 24.0)) +
                    A_init * V_init * l8_tmp * 24.0) +
                   J_min * P_init * l8_tmp * 24.0) +
                  l93 * l12 * 4.0) +
                 -(A_init * V_min * l8_tmp * 24.0)) +
                -(J_min * P_wayp * l8_tmp * 24.0)) +
               -(T * V_min * l9 * 24.0)) +
              l10 * l12 * -4.0) +
             b_l112_tmp * T * l2_tmp * 12.0) +
            T * l56) +
           l127 * l11 * 12.0) +
          -(T * l2_tmp * l8_tmp * 12.0)) +
         l164_im * l11 * -12.0;
  l93 = 1.0 / ((((l5_tmp * l5_tmp * 3.0 + l10 * 4.0) + -l24) + -(l93 * 14.0)) +
               l174_re * 19.0);
  l98_tmp = J_min * T;
  l3 = ((((((A_init * l6 * 12.0 + -(l164_im * 12.0)) + -(T * l10 * 12.0)) +
           l127 * 36.0) +
          T * l24) +
         l98_tmp * l9 * 36.0) +
        -(c_l112_tmp * l5_tmp * 36.0)) +
       -(T * l5_tmp * l8_tmp * 36.0);
  l11 = l93 * l93;
  l12 = rt_powd_snf(l93, 3.0);
  l56 = l3 * l3;
  l102 = l93 * (((((l2_tmp * l2_tmp * 3.0 + d_l112_tmp * l2_tmp * 12.0) +
                   -(e_l112_tmp * l2_tmp * 12.0)) +
                  -(V_init * V_min * l8_tmp * 24.0)) +
                 l8_tmp * (V_init * V_init) * 12.0) +
                l8_tmp * (V_min * V_min) * 12.0);
  l104 = l93 * l3 / 4.0;
  l125 = l11 * l56 * 0.375 + -(l93 * l112);
  l126 = l125 * l125;
  l127 = rt_powd_snf(l125, 3.0);
  l175_im = l11 * l3;
  l24 =
      (l12 * rt_powd_snf(l3, 3.0) / 8.0 + l93 * l113) + -(l175_im * l112 / 2.0);
  l9 = l24 * l24;
  l174_re = l11 * l11 * (l56 * l56);
  l10 = l12 * l56 * l112;
  l6 = l175_im * l113;
  l112 = ((l102 + -(l174_re * 0.01171875)) + l10 / 16.0) + -(l6 / 4.0);
  l153.re = ((((l9 * l9 * 27.0 + -(rt_powd_snf(l112, 3.0) * 256.0)) +
               -(l127 * l9 * 4.0)) +
              -(l126 * l126 * l112 * 16.0)) +
             l126 * (l112 * l112) * 128.0) +
            l125 * l9 * l112 * 144.0;
  l153.im = 0.0;
  coder::internal::scalar::b_sqrt(&l153);
  l11 = 1.7320508075688772 * l153.re;
  l12 = 1.7320508075688772 * l153.im;
  if (l12 == 0.0) {
    re = l11 / 18.0;
    l160_re = 0.0;
  } else if (l11 == 0.0) {
    re = 0.0;
    l160_re = l12 / 18.0;
  } else {
    re = l11 / 18.0;
    l160_re = l12 / 18.0;
  }
  l158_tmp = l125 * l112;
  l158.re = ((-(l127 / 27.0) + l9 / 2.0) + l158_tmp * 1.3333333333333333) + re;
  l158.im = l160_re;
  l159 = coder::power(l158);
  dc = coder::b_power(l158);
  if (dc.im == 0.0) {
    l161_re = 1.0 / dc.re;
    l161_im = 0.0;
  } else if (dc.re == 0.0) {
    l161_re = 0.0;
    l161_im = -(1.0 / dc.im);
  } else {
    re = std::abs(dc.re);
    l175_im = std::abs(dc.im);
    if (re > l175_im) {
      l175_im = dc.im / dc.re;
      l3 = dc.re + l175_im * dc.im;
      l161_re = (l175_im * 0.0 + 1.0) / l3;
      l161_im = (0.0 - l175_im) / l3;
    } else if (l175_im == re) {
      if (dc.re > 0.0) {
        l175_im = 0.5;
      } else {
        l175_im = -0.5;
      }
      if (dc.im > 0.0) {
        l3 = 0.5;
      } else {
        l3 = -0.5;
      }
      l161_re = (l175_im + 0.0 * l3) / re;
      l161_im = (0.0 * l175_im - l3) / re;
    } else {
      l175_im = dc.re / dc.im;
      l3 = dc.im + l175_im * dc.re;
      l161_re = l175_im / l3;
      l161_im = (l175_im * 0.0 - 1.0) / l3;
    }
  }
  l160_re = l159.re * l159.re - l159.im * l159.im;
  l3 = l159.re * l159.im;
  l113 = l3 + l3;
  dc.re = ((-(l127 * 2.0) + l9 * 27.0) + l158_tmp * 72.0) + l11 * 3.0;
  dc.im = l12 * 3.0;
  coder::internal::scalar::b_sqrt(&dc);
  l93 = 3.0 * (2.4494897427831779 * l24 * dc.re);
  l164_im = 3.0 * (2.4494897427831779 * l24 * dc.im);
  l24 = l125 * l159.re;
  l56 = l125 * l159.im;
  l158.re =
      (((((l102 * 12.0 + -(l174_re * 0.140625)) + l10 * 0.75) + -(l6 * 3.0)) +
        l126) +
       l160_re * 9.0) +
      l24 * 6.0;
  l158.im = l113 * 9.0 + l56 * 6.0;
  l153 = l158;
  coder::internal::scalar::b_sqrt(&l153);
  dc = coder::c_power(l158);
  if (dc.im == 0.0) {
    l10 = 1.0 / dc.re;
    l12 = 0.0;
  } else if (dc.re == 0.0) {
    l10 = 0.0;
    l12 = -(1.0 / dc.im);
  } else {
    re = std::abs(dc.re);
    l175_im = std::abs(dc.im);
    if (re > l175_im) {
      l175_im = dc.im / dc.re;
      l3 = dc.re + l175_im * dc.im;
      l10 = (l175_im * 0.0 + 1.0) / l3;
      l12 = (0.0 - l175_im) / l3;
    } else if (l175_im == re) {
      if (dc.re > 0.0) {
        l175_im = 0.5;
      } else {
        l175_im = -0.5;
      }
      if (dc.im > 0.0) {
        l3 = 0.5;
      } else {
        l3 = -0.5;
      }
      l10 = (l175_im + 0.0 * l3) / re;
      l12 = (0.0 * l175_im - l3) / re;
    } else {
      l175_im = dc.re / dc.im;
      l3 = dc.im + l175_im * dc.re;
      l10 = l175_im / l3;
      l12 = (l175_im * 0.0 - 1.0) / l3;
    }
  }
  l174_re = -9.0 * (l160_re * l153.re - l113 * l153.im);
  l6 = -9.0 * (l160_re * l153.im + l113 * l153.re);
  l3 = l161_re * l153.re - l161_im * l153.im;
  l11 = l161_re * l153.im + l161_im * l153.re;
  if (l11 == 0.0) {
    l9 = l3 / 6.0;
    l175_im = 0.0;
  } else if (l3 == 0.0) {
    l9 = 0.0;
    l175_im = l11 / 6.0;
  } else {
    l9 = l3 / 6.0;
    l175_im = l11 / 6.0;
  }
  l160_re = 12.0 * (l24 * l153.re - l56 * l153.im);
  l113 = 12.0 * (l24 * l153.im + l56 * l153.re);
  l159.re = -(l126 * l153.re);
  l159.im = -(l126 * l153.im);
  l158.re = -(l112 * l153.re * 12.0);
  l158.im = -(l112 * l153.im * 12.0);
  dc.re = (((l93 + l159.re) + l158.re) + l174_re) + l160_re;
  dc.im = (((l164_im + l159.im) + l158.im) + l6) + l113;
  coder::internal::scalar::b_sqrt(&dc);
  l56 = l161_re * l10 - l161_im * l12;
  l3 = l161_re * l12 + l161_im * l10;
  l161_re = l56 * dc.re - l3 * dc.im;
  l161_im = l56 * dc.im + l3 * dc.re;
  if (l161_im == 0.0) {
    l11 = l161_re / 6.0;
    l12 = 0.0;
  } else if (l161_re == 0.0) {
    l11 = 0.0;
    l12 = l161_im / 6.0;
  } else {
    l11 = l161_re / 6.0;
    l12 = l161_im / 6.0;
  }
  dc.re = (((-l93 + l159.re) + l158.re) + l174_re) + l160_re;
  dc.im = (((-l164_im + l159.im) + l158.im) + l6) + l113;
  coder::internal::scalar::b_sqrt(&dc);
  l161_re = l56 * dc.re - l3 * dc.im;
  l161_im = l56 * dc.im + l3 * dc.re;
  if (l161_im == 0.0) {
    l158.re = l161_re / 6.0;
    l158.im = 0.0;
  } else if (l161_re == 0.0) {
    l158.re = 0.0;
    l158.im = l161_im / 6.0;
  } else {
    l158.re = l161_re / 6.0;
    l158.im = l161_im / 6.0;
  }
  l3 = -l104 + -l9;
  t3[0].re = l3 - l11;
  t3[0].im = -l175_im - l12;
  t3[1].re = l3 + l11;
  t3[1].im = -l175_im + l12;
  l3 = -l104 + l9;
  t3[2].re = l3 - l158.re;
  t3[2].im = l175_im - l158.im;
  t3[3].re = l3 + l158.re;
  t3[3].im = l175_im + l158.im;
  l125 = d_l112_tmp * 2.0;
  l153.re = e_l112_tmp * 2.0 - l125;
  l158_tmp = c_l112_tmp * 2.0;
  re = t3[0].re * t3[0].re - t3[0].im * t3[0].im;
  l12 = t3[0].re * t3[0].im;
  l160_re = l12 + l12;
  l6 = J_min * t3[0].re;
  l9 = J_min * t3[0].im;
  l10 = J_max * T;
  l127 = -((((((l125 + l8_tmp * re * 2.0) + l5_tmp * re) + l2_tmp) -
             J_max * ((((V_init * 2.0 + A_init * t3[0].re * 2.0) + J_min * re) -
                       l98_tmp * t3[0].re * 2.0) +
                      l10 * t3[0].re * 2.0)) +
            l112_tmp * t3[0].re * 2.0) -
           b_l112_tmp * re * 2.0);
  l12 = -((((l8_tmp * l160_re * 2.0 + l5_tmp * l160_re) -
            J_max * (((A_init * t3[0].im * 2.0 + J_min * l160_re) -
                      l98_tmp * t3[0].im * 2.0) +
                     l10 * t3[0].im * 2.0)) +
           l112_tmp * t3[0].im * 2.0) -
          b_l112_tmp * l160_re * 2.0);
  l113 = l8_tmp * t3[0].re * 2.0;
  l11 = (l113 - J_max * (A_init * 2.0 + l6 * 2.0)) + l158_tmp;
  l112 = l8_tmp * t3[0].im * 2.0;
  l3 = l112 - J_max * (l9 * 2.0);
  if (l3 == 0.0) {
    if (l12 == 0.0) {
      l24 = l127 / l11;
      l164_im = 0.0;
    } else if (l127 == 0.0) {
      l24 = 0.0;
      l164_im = l12 / l11;
    } else {
      l24 = l127 / l11;
      l164_im = l12 / l11;
    }
  } else if (l11 == 0.0) {
    if (l127 == 0.0) {
      l24 = l12 / l3;
      l164_im = 0.0;
    } else if (l12 == 0.0) {
      l24 = 0.0;
      l164_im = -(l127 / l3);
    } else {
      l24 = l12 / l3;
      l164_im = -(l127 / l3);
    }
  } else {
    re = std::abs(l11);
    l175_im = std::abs(l3);
    if (re > l175_im) {
      l175_im = l3 / l11;
      l3 = l11 + l175_im * l3;
      l24 = (l127 + l175_im * l12) / l3;
      l164_im = (l12 - l175_im * l127) / l3;
    } else if (l175_im == re) {
      if (l11 > 0.0) {
        l175_im = 0.5;
      } else {
        l175_im = -0.5;
      }
      if (l3 > 0.0) {
        l3 = 0.5;
      } else {
        l3 = -0.5;
      }
      l24 = (l127 * l175_im + l12 * l3) / re;
      l164_im = (l12 * l175_im - l127 * l3) / re;
    } else {
      l175_im = l11 / l3;
      l3 += l175_im * l11;
      l24 = (l175_im * l127 + l12) / l3;
      l164_im = (l175_im * l12 - l127) / l3;
    }
  }
  re = J_max * l24;
  l160_re = J_max * l164_im;
  l56 = A_init + l6;
  l93 = l56 + re;
  l174_re = l9 + l160_re;
  l12 = re * l160_re;
  l175_im = l93 * l174_re;
  l3 = J_max * l6;
  l11 = J_max * l9;
  l127 = -(((((l153.re + (re * re - l160_re * l160_re)) -
              (l93 * l93 - l174_re * l174_re)) +
             (re * l56 - l160_re * l9) * 2.0) +
            c_l112_tmp * t3[0].re * 2.0) +
           (l3 * t3[0].re - l11 * t3[0].im));
  l12 = -(
      ((((l12 + l12) - (l175_im + l175_im)) + (re * l9 + l160_re * l56) * 2.0) +
       c_l112_tmp * t3[0].im * 2.0) +
      (l3 * t3[0].im + l11 * t3[0].re));
  l11 = ((J_max * re * 2.0 - J_max * l93 * 2.0) + l113) + l158_tmp;
  l3 = (J_max * l160_re * 2.0 - J_max * l174_re * 2.0) + l112;
  if (l3 == 0.0) {
    if (l12 == 0.0) {
      t[0].re = l127 / l11;
      t[0].im = 0.0;
    } else if (l127 == 0.0) {
      t[0].re = 0.0;
      t[0].im = l12 / l11;
    } else {
      t[0].re = l127 / l11;
      t[0].im = l12 / l11;
    }
  } else if (l11 == 0.0) {
    if (l127 == 0.0) {
      t[0].re = l12 / l3;
      t[0].im = 0.0;
    } else if (l12 == 0.0) {
      t[0].re = 0.0;
      t[0].im = -(l127 / l3);
    } else {
      t[0].re = l12 / l3;
      t[0].im = -(l127 / l3);
    }
  } else {
    re = std::abs(l11);
    l175_im = std::abs(l3);
    if (re > l175_im) {
      l175_im = l3 / l11;
      l3 = l11 + l175_im * l3;
      t[0].re = (l127 + l175_im * l12) / l3;
      t[0].im = (l12 - l175_im * l127) / l3;
    } else if (l175_im == re) {
      if (l11 > 0.0) {
        l175_im = 0.5;
      } else {
        l175_im = -0.5;
      }
      if (l3 > 0.0) {
        l3 = 0.5;
      } else {
        l3 = -0.5;
      }
      t[0].re = (l127 * l175_im + l12 * l3) / re;
      t[0].im = (l12 * l175_im - l127 * l3) / re;
    } else {
      l175_im = l11 / l3;
      l3 += l175_im * l11;
      t[0].re = (l175_im * l127 + l12) / l3;
      t[0].im = (l175_im * l12 - l127) / l3;
    }
  }
  t[4].re = 0.0;
  t[4].im = 0.0;
  t[8] = t3[0];
  t[12].re = 0.0;
  t[12].im = 0.0;
  t[16].re = 0.0;
  t[16].im = 0.0;
  t[20].re = 0.0;
  t[20].im = 0.0;
  t[24].re = l24;
  t[24].im = l164_im;
  re = t3[1].re * t3[1].re - t3[1].im * t3[1].im;
  l12 = t3[1].re * t3[1].im;
  l160_re = l12 + l12;
  l6 = J_min * t3[1].re;
  l9 = J_min * t3[1].im;
  l127 = -((((((l125 + l8_tmp * re * 2.0) + l5_tmp * re) + l2_tmp) -
             J_max * ((((V_init * 2.0 + A_init * t3[1].re * 2.0) + J_min * re) -
                       l98_tmp * t3[1].re * 2.0) +
                      l10 * t3[1].re * 2.0)) +
            l112_tmp * t3[1].re * 2.0) -
           b_l112_tmp * re * 2.0);
  l12 = -((((l8_tmp * l160_re * 2.0 + l5_tmp * l160_re) -
            J_max * (((A_init * t3[1].im * 2.0 + J_min * l160_re) -
                      l98_tmp * t3[1].im * 2.0) +
                     l10 * t3[1].im * 2.0)) +
           l112_tmp * t3[1].im * 2.0) -
          b_l112_tmp * l160_re * 2.0);
  l113 = l8_tmp * t3[1].re * 2.0;
  l11 = (l113 - J_max * (A_init * 2.0 + l6 * 2.0)) + l158_tmp;
  l112 = l8_tmp * t3[1].im * 2.0;
  l3 = l112 - J_max * (l9 * 2.0);
  if (l3 == 0.0) {
    if (l12 == 0.0) {
      l24 = l127 / l11;
      l164_im = 0.0;
    } else if (l127 == 0.0) {
      l24 = 0.0;
      l164_im = l12 / l11;
    } else {
      l24 = l127 / l11;
      l164_im = l12 / l11;
    }
  } else if (l11 == 0.0) {
    if (l127 == 0.0) {
      l24 = l12 / l3;
      l164_im = 0.0;
    } else if (l12 == 0.0) {
      l24 = 0.0;
      l164_im = -(l127 / l3);
    } else {
      l24 = l12 / l3;
      l164_im = -(l127 / l3);
    }
  } else {
    re = std::abs(l11);
    l175_im = std::abs(l3);
    if (re > l175_im) {
      l175_im = l3 / l11;
      l3 = l11 + l175_im * l3;
      l24 = (l127 + l175_im * l12) / l3;
      l164_im = (l12 - l175_im * l127) / l3;
    } else if (l175_im == re) {
      if (l11 > 0.0) {
        l175_im = 0.5;
      } else {
        l175_im = -0.5;
      }
      if (l3 > 0.0) {
        l3 = 0.5;
      } else {
        l3 = -0.5;
      }
      l24 = (l127 * l175_im + l12 * l3) / re;
      l164_im = (l12 * l175_im - l127 * l3) / re;
    } else {
      l175_im = l11 / l3;
      l3 += l175_im * l11;
      l24 = (l175_im * l127 + l12) / l3;
      l164_im = (l175_im * l12 - l127) / l3;
    }
  }
  re = J_max * l24;
  l160_re = J_max * l164_im;
  l56 = A_init + l6;
  l93 = l56 + re;
  l174_re = l9 + l160_re;
  l12 = re * l160_re;
  l175_im = l93 * l174_re;
  l3 = J_max * l6;
  l11 = J_max * l9;
  l127 = -(((((l153.re + (re * re - l160_re * l160_re)) -
              (l93 * l93 - l174_re * l174_re)) +
             (re * l56 - l160_re * l9) * 2.0) +
            c_l112_tmp * t3[1].re * 2.0) +
           (l3 * t3[1].re - l11 * t3[1].im));
  l12 = -(
      ((((l12 + l12) - (l175_im + l175_im)) + (re * l9 + l160_re * l56) * 2.0) +
       c_l112_tmp * t3[1].im * 2.0) +
      (l3 * t3[1].im + l11 * t3[1].re));
  l11 = ((J_max * re * 2.0 - J_max * l93 * 2.0) + l113) + l158_tmp;
  l3 = (J_max * l160_re * 2.0 - J_max * l174_re * 2.0) + l112;
  if (l3 == 0.0) {
    if (l12 == 0.0) {
      t[1].re = l127 / l11;
      t[1].im = 0.0;
    } else if (l127 == 0.0) {
      t[1].re = 0.0;
      t[1].im = l12 / l11;
    } else {
      t[1].re = l127 / l11;
      t[1].im = l12 / l11;
    }
  } else if (l11 == 0.0) {
    if (l127 == 0.0) {
      t[1].re = l12 / l3;
      t[1].im = 0.0;
    } else if (l12 == 0.0) {
      t[1].re = 0.0;
      t[1].im = -(l127 / l3);
    } else {
      t[1].re = l12 / l3;
      t[1].im = -(l127 / l3);
    }
  } else {
    re = std::abs(l11);
    l175_im = std::abs(l3);
    if (re > l175_im) {
      l175_im = l3 / l11;
      l3 = l11 + l175_im * l3;
      t[1].re = (l127 + l175_im * l12) / l3;
      t[1].im = (l12 - l175_im * l127) / l3;
    } else if (l175_im == re) {
      if (l11 > 0.0) {
        l175_im = 0.5;
      } else {
        l175_im = -0.5;
      }
      if (l3 > 0.0) {
        l3 = 0.5;
      } else {
        l3 = -0.5;
      }
      t[1].re = (l127 * l175_im + l12 * l3) / re;
      t[1].im = (l12 * l175_im - l127 * l3) / re;
    } else {
      l175_im = l11 / l3;
      l3 += l175_im * l11;
      t[1].re = (l175_im * l127 + l12) / l3;
      t[1].im = (l175_im * l12 - l127) / l3;
    }
  }
  t[5].re = 0.0;
  t[5].im = 0.0;
  t[9] = t3[1];
  t[13].re = 0.0;
  t[13].im = 0.0;
  t[17].re = 0.0;
  t[17].im = 0.0;
  t[21].re = 0.0;
  t[21].im = 0.0;
  t[25].re = l24;
  t[25].im = l164_im;
  re = t3[2].re * t3[2].re - t3[2].im * t3[2].im;
  l12 = t3[2].re * t3[2].im;
  l160_re = l12 + l12;
  l6 = J_min * t3[2].re;
  l9 = J_min * t3[2].im;
  l127 = -((((((l125 + l8_tmp * re * 2.0) + l5_tmp * re) + l2_tmp) -
             J_max * ((((V_init * 2.0 + A_init * t3[2].re * 2.0) + J_min * re) -
                       l98_tmp * t3[2].re * 2.0) +
                      l10 * t3[2].re * 2.0)) +
            l112_tmp * t3[2].re * 2.0) -
           b_l112_tmp * re * 2.0);
  l12 = -((((l8_tmp * l160_re * 2.0 + l5_tmp * l160_re) -
            J_max * (((A_init * t3[2].im * 2.0 + J_min * l160_re) -
                      l98_tmp * t3[2].im * 2.0) +
                     l10 * t3[2].im * 2.0)) +
           l112_tmp * t3[2].im * 2.0) -
          b_l112_tmp * l160_re * 2.0);
  l113 = l8_tmp * t3[2].re * 2.0;
  l11 = (l113 - J_max * (A_init * 2.0 + l6 * 2.0)) + l158_tmp;
  l112 = l8_tmp * t3[2].im * 2.0;
  l3 = l112 - J_max * (l9 * 2.0);
  if (l3 == 0.0) {
    if (l12 == 0.0) {
      l24 = l127 / l11;
      l164_im = 0.0;
    } else if (l127 == 0.0) {
      l24 = 0.0;
      l164_im = l12 / l11;
    } else {
      l24 = l127 / l11;
      l164_im = l12 / l11;
    }
  } else if (l11 == 0.0) {
    if (l127 == 0.0) {
      l24 = l12 / l3;
      l164_im = 0.0;
    } else if (l12 == 0.0) {
      l24 = 0.0;
      l164_im = -(l127 / l3);
    } else {
      l24 = l12 / l3;
      l164_im = -(l127 / l3);
    }
  } else {
    re = std::abs(l11);
    l175_im = std::abs(l3);
    if (re > l175_im) {
      l175_im = l3 / l11;
      l3 = l11 + l175_im * l3;
      l24 = (l127 + l175_im * l12) / l3;
      l164_im = (l12 - l175_im * l127) / l3;
    } else if (l175_im == re) {
      if (l11 > 0.0) {
        l175_im = 0.5;
      } else {
        l175_im = -0.5;
      }
      if (l3 > 0.0) {
        l3 = 0.5;
      } else {
        l3 = -0.5;
      }
      l24 = (l127 * l175_im + l12 * l3) / re;
      l164_im = (l12 * l175_im - l127 * l3) / re;
    } else {
      l175_im = l11 / l3;
      l3 += l175_im * l11;
      l24 = (l175_im * l127 + l12) / l3;
      l164_im = (l175_im * l12 - l127) / l3;
    }
  }
  re = J_max * l24;
  l160_re = J_max * l164_im;
  l56 = A_init + l6;
  l93 = l56 + re;
  l174_re = l9 + l160_re;
  l12 = re * l160_re;
  l175_im = l93 * l174_re;
  l3 = J_max * l6;
  l11 = J_max * l9;
  l127 = -(((((l153.re + (re * re - l160_re * l160_re)) -
              (l93 * l93 - l174_re * l174_re)) +
             (re * l56 - l160_re * l9) * 2.0) +
            c_l112_tmp * t3[2].re * 2.0) +
           (l3 * t3[2].re - l11 * t3[2].im));
  l12 = -(
      ((((l12 + l12) - (l175_im + l175_im)) + (re * l9 + l160_re * l56) * 2.0) +
       c_l112_tmp * t3[2].im * 2.0) +
      (l3 * t3[2].im + l11 * t3[2].re));
  l11 = ((J_max * re * 2.0 - J_max * l93 * 2.0) + l113) + l158_tmp;
  l3 = (J_max * l160_re * 2.0 - J_max * l174_re * 2.0) + l112;
  if (l3 == 0.0) {
    if (l12 == 0.0) {
      t[2].re = l127 / l11;
      t[2].im = 0.0;
    } else if (l127 == 0.0) {
      t[2].re = 0.0;
      t[2].im = l12 / l11;
    } else {
      t[2].re = l127 / l11;
      t[2].im = l12 / l11;
    }
  } else if (l11 == 0.0) {
    if (l127 == 0.0) {
      t[2].re = l12 / l3;
      t[2].im = 0.0;
    } else if (l12 == 0.0) {
      t[2].re = 0.0;
      t[2].im = -(l127 / l3);
    } else {
      t[2].re = l12 / l3;
      t[2].im = -(l127 / l3);
    }
  } else {
    re = std::abs(l11);
    l175_im = std::abs(l3);
    if (re > l175_im) {
      l175_im = l3 / l11;
      l3 = l11 + l175_im * l3;
      t[2].re = (l127 + l175_im * l12) / l3;
      t[2].im = (l12 - l175_im * l127) / l3;
    } else if (l175_im == re) {
      if (l11 > 0.0) {
        l175_im = 0.5;
      } else {
        l175_im = -0.5;
      }
      if (l3 > 0.0) {
        l3 = 0.5;
      } else {
        l3 = -0.5;
      }
      t[2].re = (l127 * l175_im + l12 * l3) / re;
      t[2].im = (l12 * l175_im - l127 * l3) / re;
    } else {
      l175_im = l11 / l3;
      l3 += l175_im * l11;
      t[2].re = (l175_im * l127 + l12) / l3;
      t[2].im = (l175_im * l12 - l127) / l3;
    }
  }
  t[6].re = 0.0;
  t[6].im = 0.0;
  t[10] = t3[2];
  t[14].re = 0.0;
  t[14].im = 0.0;
  t[18].re = 0.0;
  t[18].im = 0.0;
  t[22].re = 0.0;
  t[22].im = 0.0;
  t[26].re = l24;
  t[26].im = l164_im;
  re = t3[3].re * t3[3].re - t3[3].im * t3[3].im;
  l12 = t3[3].re * t3[3].im;
  l160_re = l12 + l12;
  l6 = J_min * t3[3].re;
  l9 = J_min * t3[3].im;
  l127 = -((((((l125 + l8_tmp * re * 2.0) + l5_tmp * re) + l2_tmp) -
             J_max * ((((V_init * 2.0 + A_init * t3[3].re * 2.0) + J_min * re) -
                       l98_tmp * t3[3].re * 2.0) +
                      l10 * t3[3].re * 2.0)) +
            l112_tmp * t3[3].re * 2.0) -
           b_l112_tmp * re * 2.0);
  l12 = -((((l8_tmp * l160_re * 2.0 + l5_tmp * l160_re) -
            J_max * (((A_init * t3[3].im * 2.0 + J_min * l160_re) -
                      l98_tmp * t3[3].im * 2.0) +
                     l10 * t3[3].im * 2.0)) +
           l112_tmp * t3[3].im * 2.0) -
          b_l112_tmp * l160_re * 2.0);
  l113 = l8_tmp * t3[3].re * 2.0;
  l11 = (l113 - J_max * (A_init * 2.0 + l6 * 2.0)) + l158_tmp;
  l112 = l8_tmp * t3[3].im * 2.0;
  l3 = l112 - J_max * (l9 * 2.0);
  if (l3 == 0.0) {
    if (l12 == 0.0) {
      l24 = l127 / l11;
      l164_im = 0.0;
    } else if (l127 == 0.0) {
      l24 = 0.0;
      l164_im = l12 / l11;
    } else {
      l24 = l127 / l11;
      l164_im = l12 / l11;
    }
  } else if (l11 == 0.0) {
    if (l127 == 0.0) {
      l24 = l12 / l3;
      l164_im = 0.0;
    } else if (l12 == 0.0) {
      l24 = 0.0;
      l164_im = -(l127 / l3);
    } else {
      l24 = l12 / l3;
      l164_im = -(l127 / l3);
    }
  } else {
    re = std::abs(l11);
    l175_im = std::abs(l3);
    if (re > l175_im) {
      l175_im = l3 / l11;
      l3 = l11 + l175_im * l3;
      l24 = (l127 + l175_im * l12) / l3;
      l164_im = (l12 - l175_im * l127) / l3;
    } else if (l175_im == re) {
      if (l11 > 0.0) {
        l175_im = 0.5;
      } else {
        l175_im = -0.5;
      }
      if (l3 > 0.0) {
        l3 = 0.5;
      } else {
        l3 = -0.5;
      }
      l24 = (l127 * l175_im + l12 * l3) / re;
      l164_im = (l12 * l175_im - l127 * l3) / re;
    } else {
      l175_im = l11 / l3;
      l3 += l175_im * l11;
      l24 = (l175_im * l127 + l12) / l3;
      l164_im = (l175_im * l12 - l127) / l3;
    }
  }
  re = J_max * l24;
  l160_re = J_max * l164_im;
  l56 = A_init + l6;
  l93 = l56 + re;
  l174_re = l9 + l160_re;
  l12 = re * l160_re;
  l175_im = l93 * l174_re;
  l3 = J_max * l6;
  l11 = J_max * l9;
  l127 = -(((((l153.re + (re * re - l160_re * l160_re)) -
              (l93 * l93 - l174_re * l174_re)) +
             (re * l56 - l160_re * l9) * 2.0) +
            c_l112_tmp * t3[3].re * 2.0) +
           (l3 * t3[3].re - l11 * t3[3].im));
  l12 = -(
      ((((l12 + l12) - (l175_im + l175_im)) + (re * l9 + l160_re * l56) * 2.0) +
       c_l112_tmp * t3[3].im * 2.0) +
      (l3 * t3[3].im + l11 * t3[3].re));
  l11 = ((J_max * re * 2.0 - J_max * l93 * 2.0) + l113) + l158_tmp;
  l3 = (J_max * l160_re * 2.0 - J_max * l174_re * 2.0) + l112;
  if (l3 == 0.0) {
    if (l12 == 0.0) {
      t[3].re = l127 / l11;
      t[3].im = 0.0;
    } else if (l127 == 0.0) {
      t[3].re = 0.0;
      t[3].im = l12 / l11;
    } else {
      t[3].re = l127 / l11;
      t[3].im = l12 / l11;
    }
  } else if (l11 == 0.0) {
    if (l127 == 0.0) {
      t[3].re = l12 / l3;
      t[3].im = 0.0;
    } else if (l12 == 0.0) {
      t[3].re = 0.0;
      t[3].im = -(l127 / l3);
    } else {
      t[3].re = l12 / l3;
      t[3].im = -(l127 / l3);
    }
  } else {
    re = std::abs(l11);
    l175_im = std::abs(l3);
    if (re > l175_im) {
      l175_im = l3 / l11;
      l3 = l11 + l175_im * l3;
      t[3].re = (l127 + l175_im * l12) / l3;
      t[3].im = (l12 - l175_im * l127) / l3;
    } else if (l175_im == re) {
      if (l11 > 0.0) {
        l175_im = 0.5;
      } else {
        l175_im = -0.5;
      }
      if (l3 > 0.0) {
        l3 = 0.5;
      } else {
        l3 = -0.5;
      }
      t[3].re = (l127 * l175_im + l12 * l3) / re;
      t[3].im = (l12 * l175_im - l127 * l3) / re;
    } else {
      l175_im = l11 / l3;
      l3 += l175_im * l11;
      t[3].re = (l175_im * l127 + l12) / l3;
      t[3].im = (l175_im * l12 - l127) / l3;
    }
  }
  t[7].re = 0.0;
  t[7].im = 0.0;
  t[11] = t3[3];
  t[15].re = 0.0;
  t[15].im = 0.0;
  t[19].re = 0.0;
  t[19].im = 0.0;
  t[23].re = 0.0;
  t[23].im = 0.0;
  t[27].re = l24;
  t[27].im = l164_im;
}

// End of code generation (aceg_T_P.cpp)
