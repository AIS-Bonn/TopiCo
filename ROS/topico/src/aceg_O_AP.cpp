//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// aceg_O_AP.cpp
//
// Code generation for function 'aceg_O_AP'
//

// Include files
#include "aceg_O_AP.h"
#include "power.h"
#include "rt_nonfinite.h"
#include "sqrt.h"
#include "topico_wrapper_data.h"
#include "topico_wrapper_rtwutil.h"
#include <cmath>

// Function Definitions
void aceg_O_AP(double P_init, double V_init, double A_init, double P_wayp,
               double A_wayp, double V_max, double J_max, double J_min,
               creal_T t[28])
{
  creal_T b_l2[4];
  creal_T t3[4];
  creal_T t7[4];
  creal_T l121;
  creal_T l124;
  creal_T l126;
  creal_T l127;
  creal_T l99;
  double a_tmp;
  double ai;
  double ar;
  double im;
  double l10;
  double l11_tmp;
  double l129_im;
  double l129_re;
  double l13;
  double l141_im;
  double l146_re;
  double l2;
  double l3;
  double l5;
  double l6;
  double l7;
  double l8;
  double l85;
  double l87;
  double l89;
  double l9;
  double l93;
  double l93_tmp;
  double l96_tmp;
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
  //  Generated on 28-Aug-2019 17:25:45
  l2 = A_init * A_init;
  l3 = rt_powd_snf(A_init, 3.0);
  l5 = A_wayp * A_wayp;
  l6 = rt_powd_snf(A_wayp, 3.0);
  l8 = J_min * J_min;
  l9 = rt_powd_snf(J_min, 3.0);
  l11_tmp = J_max * J_max;
  l13 = rt_powd_snf(J_max, 3.0);
  l7 = l5 * l5;
  l10 = l8 * l8;
  a_tmp = ((rt_powd_snf(l8, 3.0) - J_max * rt_powd_snf(J_min, 5.0) * 4.0) -
           l9 * l13 * 2.0) +
          l10 * l11_tmp * 5.0;
  l85 = 1.0 / (a_tmp * a_tmp);
  l93_tmp = A_wayp * J_max;
  l93 = (((((((((((l6 * l9 * 8.0 + A_init * J_max * V_init * l9 * 24.0) +
                  P_init * l8 * l13 * 24.0) +
                 P_wayp * l9 * l11_tmp * 24.0) +
                -(l3 * l9 * 8.0)) +
               -(l93_tmp * V_max * l9 * 24.0)) +
              J_max * l3 * l8 * 8.0) +
             J_min * l6 * l11_tmp * 12.0) +
            -(P_init * l9 * l11_tmp * 24.0)) +
           -(P_wayp * l8 * l13 * 24.0)) +
          A_wayp * V_max * l8 * l11_tmp * 24.0) +
         -(J_max * l6 * l8 * 20.0)) +
        -(A_init * V_init * l8 * l11_tmp * 24.0);
  l87 = l85 * l85;
  l6 = J_max * V_max;
  l146_re = J_max * V_init;
  l89 =
      (((((((((((l146_re * l10 * 12.0 + l6 * l10 * 12.0) + -(l2 * l10 * 6.0)) +
               -(l5 * l10 * 6.0)) +
              J_min * l5 * l13 * 6.0) +
             V_init * l8 * l13 * 12.0) +
            V_max * l8 * l13 * 12.0) +
           J_max * l2 * l9 * 12.0) +
          J_max * l5 * l9 * 18.0) +
         -(V_init * l9 * l11_tmp * 24.0)) +
        -(V_max * l9 * l11_tmp * 24.0)) +
       -(l2 * l8 * l11_tmp * 6.0)) +
      -(l5 * l8 * l11_tmp * 18.0);
  l10 = l93 * l93;
  l96_tmp = J_min * J_max;
  l3 = l8 * l11_tmp;
  l7 = (((((((((((((-(l96_tmp * l7 * 6.0) + l2 * l2 * l8 * 3.0) +
                   l7 * l8 * 3.0) +
                  l7 * l11_tmp * 3.0) +
                 l96_tmp * l2 * l5 * 6.0) +
                l6 * l2 * l8 * 12.0) +
               l146_re * l5 * l8 * 12.0) +
              J_min * V_max * l5 * l11_tmp * 12.0) +
             -(l146_re * l2 * l8 * 12.0)) +
            -(J_min * V_init * l5 * l11_tmp * 12.0)) +
           -(l6 * l5 * l8 * 12.0)) +
          -(l2 * l5 * l8 * 6.0)) +
         -(V_init * V_max * l8 * l11_tmp * 24.0)) +
        l3 * (V_init * V_init) * 12.0) +
       l3 * (V_max * V_max) * 12.0;
  l8 = l89 * l89;
  l6 = rt_powd_snf(l89, 3.0);
  l5 = l85 * l8;
  l3 = rt_powd_snf(a_tmp, 5.0);
  l9 = rt_powd_snf(a_tmp, 3.0);
  l121.re = ((((l87 * (l10 * l10) * 27.0 + rt_powd_snf(l7, 3.0) / l9 * 256.0) +
               l6 * l10 / l3 * 4.0) +
              l8 * l8 * l7 / l3 * 16.0) +
             l87 * l8 * (l7 * l7) * 128.0) +
            l87 * l89 * l10 * l7 * 144.0;
  l121.im = 0.0;
  coder::internal::scalar::b_sqrt(&l121);
  l2 = 1.7320508075688772 * l121.re;
  l13 = 1.7320508075688772 * l121.im;
  if (l13 == 0.0) {
    re = l2 / 18.0;
    im = 0.0;
  } else if (l2 == 0.0) {
    re = 0.0;
    im = l13 / 18.0;
  } else {
    re = l2 / 18.0;
    im = l13 / 18.0;
  }
  l9 = l6 / l9;
  l6 = l85 * l10;
  l3 = l85 * l89 * l7;
  l126.re = ((l9 / 27.0 + l6 / 2.0) + l3 * 1.3333333333333333) + re;
  l126.im = im;
  l124.re = ((l9 * 2.0 + l6 * 27.0) + l3 * 72.0) + l2 * 3.0;
  l124.im = l13 * 3.0;
  coder::internal::scalar::b_sqrt(&l124);
  l127 = coder::power(l126);
  l99 = coder::b_power(l126);
  if (l99.im == 0.0) {
    l129_re = 1.0 / l99.re;
    l129_im = 0.0;
  } else if (l99.re == 0.0) {
    l129_re = 0.0;
    l129_im = -(1.0 / l99.im);
  } else {
    l13 = std::abs(l99.re);
    l9 = std::abs(l99.im);
    if (l13 > l9) {
      l9 = l99.im / l99.re;
      l3 = l99.re + l9 * l99.im;
      l129_re = (l9 * 0.0 + 1.0) / l3;
      l129_im = (0.0 - l9) / l3;
    } else if (l9 == l13) {
      if (l99.re > 0.0) {
        l9 = 0.5;
      } else {
        l9 = -0.5;
      }
      if (l99.im > 0.0) {
        l3 = 0.5;
      } else {
        l3 = -0.5;
      }
      l129_re = (l9 + 0.0 * l3) / l13;
      l129_im = (0.0 * l9 - l3) / l13;
    } else {
      l9 = l99.re / l99.im;
      l3 = l99.im + l9 * l99.re;
      l129_re = l9 / l3;
      l129_im = (l9 * 0.0 - 1.0) / l3;
    }
  }
  l121.re = l127.re * l127.re - l127.im * l127.im;
  l3 = l127.re * l127.im;
  l121.im = l3 + l3;
  l9 = l89 * l127.re;
  l13 = l89 * l127.im;
  ar = l9 * -2.0;
  ai = l13 * -2.0;
  if (ai == 0.0) {
    l6 = ar / a_tmp;
    l3 = 0.0;
  } else if (ar == 0.0) {
    l6 = 0.0;
    l3 = ai / a_tmp;
  } else {
    l6 = ar / a_tmp;
    l3 = ai / a_tmp;
  }
  l126.re = ((l5 / 3.0 + l7 * -4.0 / a_tmp) + l121.re * 3.0) + l6;
  l126.im = l121.im * 3.0 + l3;
  coder::internal::scalar::b_sqrt(&l126);
  ar = l9 * -6.0;
  ai = l13 * -6.0;
  if (ai == 0.0) {
    l6 = ar / a_tmp;
    l3 = 0.0;
  } else if (ar == 0.0) {
    l6 = 0.0;
    l3 = ai / a_tmp;
  } else {
    l6 = ar / a_tmp;
    l3 = ai / a_tmp;
  }
  l99.re = ((l5 + l7 * -12.0 / a_tmp) + l121.re * 9.0) + l6;
  l99.im = l121.im * 9.0 + l3;
  l99 = coder::c_power(l99);
  if (l99.im == 0.0) {
    l6 = 1.0 / l99.re;
    l3 = 0.0;
  } else if (l99.re == 0.0) {
    l6 = 0.0;
    l3 = -(1.0 / l99.im);
  } else {
    l13 = std::abs(l99.re);
    l9 = std::abs(l99.im);
    if (l13 > l9) {
      l9 = l99.im / l99.re;
      l3 = l99.re + l9 * l99.im;
      l6 = (l9 * 0.0 + 1.0) / l3;
      l3 = (0.0 - l9) / l3;
    } else if (l9 == l13) {
      if (l99.re > 0.0) {
        l9 = 0.5;
      } else {
        l9 = -0.5;
      }
      if (l99.im > 0.0) {
        l3 = 0.5;
      } else {
        l3 = -0.5;
      }
      l6 = (l9 + 0.0 * l3) / l13;
      l3 = (0.0 * l9 - l3) / l13;
    } else {
      l9 = l99.re / l99.im;
      l3 = l99.im + l9 * l99.re;
      l6 = l9 / l3;
      l3 = (l9 * 0.0 - 1.0) / l3;
    }
  }
  l2 = 1.7320508075688772 * (-1.0 / a_tmp) * l8;
  ar = l2 * l126.re;
  ai = l2 * l126.im;
  if (ai == 0.0) {
    l85 = ar / a_tmp;
    l141_im = 0.0;
  } else if (ar == 0.0) {
    l85 = 0.0;
    l141_im = ai / a_tmp;
  } else {
    l85 = ar / a_tmp;
    l141_im = ai / a_tmp;
  }
  ar = 1.7320508075688772 * l7 * l126.re * 12.0;
  ai = 1.7320508075688772 * l7 * l126.im * 12.0;
  if (ai == 0.0) {
    l5 = ar / a_tmp;
    l8 = 0.0;
  } else if (ar == 0.0) {
    l5 = 0.0;
    l8 = ai / a_tmp;
  } else {
    l5 = ar / a_tmp;
    l8 = ai / a_tmp;
  }
  re = 1.7320508075688772 * l121.re;
  im = 1.7320508075688772 * l121.im;
  l7 = -9.0 * (re * l126.re - im * l126.im);
  l87 = -9.0 * (re * l126.im + im * l126.re);
  re = 1.7320508075688772 * l129_re;
  im = 1.7320508075688772 * l129_im;
  l146_re = re * l126.re - im * l126.im;
  im = re * l126.im + im * l126.re;
  if (im == 0.0) {
    l146_re /= 6.0;
    l2 = 0.0;
  } else if (l146_re == 0.0) {
    l146_re = 0.0;
    l2 = im / 6.0;
  } else {
    l146_re /= 6.0;
    l2 = im / 6.0;
  }
  re = 1.7320508075688772 * l89 * l127.re;
  im = 1.7320508075688772 * l89 * l127.im;
  ar = (re * l126.re - im * l126.im) * -12.0;
  ai = (re * l126.im + im * l126.re) * -12.0;
  if (ai == 0.0) {
    l121.re = ar / a_tmp;
    l121.im = 0.0;
  } else if (ar == 0.0) {
    l121.re = 0.0;
    l121.im = ai / a_tmp;
  } else {
    l121.re = ar / a_tmp;
    l121.im = ai / a_tmp;
  }
  l10 = 2.4494897427831779 * l93 * l124.re;
  l89 = 2.4494897427831779 * l93 * l124.im;
  ar = l10 * -3.0;
  ai = l89 * -3.0;
  if (ai == 0.0) {
    re = ar / a_tmp;
    im = 0.0;
  } else if (ar == 0.0) {
    re = 0.0;
    im = ai / a_tmp;
  } else {
    re = ar / a_tmp;
    im = ai / a_tmp;
  }
  l126.re = (((re + l85) + l5) + l7) + l121.re;
  l126.im = (((im + l141_im) + l8) + l87) + l121.im;
  coder::internal::scalar::b_sqrt(&l126);
  l13 = l129_re * l6 - l129_im * l3;
  l3 = l129_re * l3 + l129_im * l6;
  l129_re = l13 * l126.re - l3 * l126.im;
  l129_im = l13 * l126.im + l3 * l126.re;
  if (l129_im == 0.0) {
    l6 = l129_re / 6.0;
    l9 = 0.0;
  } else if (l129_re == 0.0) {
    l6 = 0.0;
    l9 = l129_im / 6.0;
  } else {
    l6 = l129_re / 6.0;
    l9 = l129_im / 6.0;
  }
  ar = l10 * 3.0;
  ai = l89 * 3.0;
  if (ai == 0.0) {
    re = ar / a_tmp;
    im = 0.0;
  } else if (ar == 0.0) {
    re = 0.0;
    im = ai / a_tmp;
  } else {
    re = ar / a_tmp;
    im = ai / a_tmp;
  }
  l126.re = (((re + l85) + l5) + l7) + l121.re;
  l126.im = (((im + l141_im) + l8) + l87) + l121.im;
  coder::internal::scalar::b_sqrt(&l126);
  l129_re = l13 * l126.re - l3 * l126.im;
  l129_im = l13 * l126.im + l3 * l126.re;
  if (l129_im == 0.0) {
    l121.re = l129_re / 6.0;
    l121.im = 0.0;
  } else if (l129_re == 0.0) {
    l121.re = 0.0;
    l121.im = l129_im / 6.0;
  } else {
    l121.re = l129_re / 6.0;
    l121.im = l129_im / 6.0;
  }
  t3[0].re = -l146_re - l121.re;
  t3[0].im = -l2 - l121.im;
  t3[1].re = -l146_re + l121.re;
  t3[1].im = -l2 + l121.im;
  t3[2].re = l146_re - l6;
  t3[2].im = l2 - l9;
  t3[3].re = l146_re + l6;
  t3[3].im = l2 + l9;
  l121.re = l96_tmp * V_init * 2.0;
  l126.re = l96_tmp * V_max * 2.0;
  l10 = A_init * J_min;
  l124.re = l93_tmp * -A_wayp;
  l127.re = l10 * J_max * 2.0;
  l129_re = A_wayp * J_min * J_max * 2.0;
  re = J_min * t3[0].re;
  im = J_min * t3[0].im;
  b_l2[0].re = re;
  b_l2[0].im = im;
  l146_re = (A_init + re) + -A_wayp;
  l89 = l146_re * im;
  l3 = re * im;
  l6 = J_max * re;
  l9 = J_max * im;
  l13 = A_wayp - re;
  l2 = im * im;
  ar = (((((J_min * (l146_re * l146_re - l2) + J_max * (re * re - l2)) +
           l121.re) -
          l126.re) -
         l10 * l146_re * 2.0) +
        l124.re) +
       (l6 * l13 - l9 * (0.0 - im)) * 2.0;
  ai = ((J_min * (l89 + l89) + J_max * (l3 + l3)) - l10 * im * 2.0) +
       (l6 * (0.0 - im) + l9 * l13) * 2.0;
  l3 = ((l11_tmp * re * 2.0 + l127.re) - l129_re) - l96_tmp * l146_re * 2.0;
  l6 = l11_tmp * im * 2.0 - l96_tmp * im * 2.0;
  if (l6 == 0.0) {
    if (ai == 0.0) {
      t7[0].re = ar / l3;
      t7[0].im = 0.0;
    } else if (ar == 0.0) {
      t7[0].re = 0.0;
      t7[0].im = ai / l3;
    } else {
      t7[0].re = ar / l3;
      t7[0].im = ai / l3;
    }
  } else if (l3 == 0.0) {
    if (ar == 0.0) {
      t7[0].re = ai / l6;
      t7[0].im = 0.0;
    } else if (ai == 0.0) {
      t7[0].re = 0.0;
      t7[0].im = -(ar / l6);
    } else {
      t7[0].re = ai / l6;
      t7[0].im = -(ar / l6);
    }
  } else {
    l13 = std::abs(l3);
    l9 = std::abs(l6);
    if (l13 > l9) {
      l9 = l6 / l3;
      l3 += l9 * l6;
      t7[0].re = (ar + l9 * ai) / l3;
      t7[0].im = (ai - l9 * ar) / l3;
    } else if (l9 == l13) {
      if (l3 > 0.0) {
        l9 = 0.5;
      } else {
        l9 = -0.5;
      }
      if (l6 > 0.0) {
        l3 = 0.5;
      } else {
        l3 = -0.5;
      }
      t7[0].re = (ar * l9 + ai * l3) / l13;
      t7[0].im = (ai * l9 - ar * l3) / l13;
    } else {
      l9 = l3 / l6;
      l3 = l6 + l9 * l3;
      t7[0].re = (l9 * ar + ai) / l3;
      t7[0].im = (l9 * ai - ar) / l3;
    }
  }
  re = J_min * t3[1].re;
  im = J_min * t3[1].im;
  b_l2[1].re = re;
  b_l2[1].im = im;
  l146_re = (A_init + re) + -A_wayp;
  l89 = l146_re * im;
  l3 = re * im;
  l6 = J_max * re;
  l9 = J_max * im;
  l13 = A_wayp - re;
  l2 = im * im;
  ar = (((((J_min * (l146_re * l146_re - l2) + J_max * (re * re - l2)) +
           l121.re) -
          l126.re) -
         l10 * l146_re * 2.0) +
        l124.re) +
       (l6 * l13 - l9 * (0.0 - im)) * 2.0;
  ai = ((J_min * (l89 + l89) + J_max * (l3 + l3)) - l10 * im * 2.0) +
       (l6 * (0.0 - im) + l9 * l13) * 2.0;
  l3 = ((l11_tmp * re * 2.0 + l127.re) - l129_re) - l96_tmp * l146_re * 2.0;
  l6 = l11_tmp * im * 2.0 - l96_tmp * im * 2.0;
  if (l6 == 0.0) {
    if (ai == 0.0) {
      t7[1].re = ar / l3;
      t7[1].im = 0.0;
    } else if (ar == 0.0) {
      t7[1].re = 0.0;
      t7[1].im = ai / l3;
    } else {
      t7[1].re = ar / l3;
      t7[1].im = ai / l3;
    }
  } else if (l3 == 0.0) {
    if (ar == 0.0) {
      t7[1].re = ai / l6;
      t7[1].im = 0.0;
    } else if (ai == 0.0) {
      t7[1].re = 0.0;
      t7[1].im = -(ar / l6);
    } else {
      t7[1].re = ai / l6;
      t7[1].im = -(ar / l6);
    }
  } else {
    l13 = std::abs(l3);
    l9 = std::abs(l6);
    if (l13 > l9) {
      l9 = l6 / l3;
      l3 += l9 * l6;
      t7[1].re = (ar + l9 * ai) / l3;
      t7[1].im = (ai - l9 * ar) / l3;
    } else if (l9 == l13) {
      if (l3 > 0.0) {
        l9 = 0.5;
      } else {
        l9 = -0.5;
      }
      if (l6 > 0.0) {
        l3 = 0.5;
      } else {
        l3 = -0.5;
      }
      t7[1].re = (ar * l9 + ai * l3) / l13;
      t7[1].im = (ai * l9 - ar * l3) / l13;
    } else {
      l9 = l3 / l6;
      l3 = l6 + l9 * l3;
      t7[1].re = (l9 * ar + ai) / l3;
      t7[1].im = (l9 * ai - ar) / l3;
    }
  }
  re = J_min * t3[2].re;
  im = J_min * t3[2].im;
  b_l2[2].re = re;
  b_l2[2].im = im;
  l146_re = (A_init + re) + -A_wayp;
  l89 = l146_re * im;
  l3 = re * im;
  l6 = J_max * re;
  l9 = J_max * im;
  l13 = A_wayp - re;
  l2 = im * im;
  ar = (((((J_min * (l146_re * l146_re - l2) + J_max * (re * re - l2)) +
           l121.re) -
          l126.re) -
         l10 * l146_re * 2.0) +
        l124.re) +
       (l6 * l13 - l9 * (0.0 - im)) * 2.0;
  ai = ((J_min * (l89 + l89) + J_max * (l3 + l3)) - l10 * im * 2.0) +
       (l6 * (0.0 - im) + l9 * l13) * 2.0;
  l3 = ((l11_tmp * re * 2.0 + l127.re) - l129_re) - l96_tmp * l146_re * 2.0;
  l6 = l11_tmp * im * 2.0 - l96_tmp * im * 2.0;
  if (l6 == 0.0) {
    if (ai == 0.0) {
      t7[2].re = ar / l3;
      t7[2].im = 0.0;
    } else if (ar == 0.0) {
      t7[2].re = 0.0;
      t7[2].im = ai / l3;
    } else {
      t7[2].re = ar / l3;
      t7[2].im = ai / l3;
    }
  } else if (l3 == 0.0) {
    if (ar == 0.0) {
      t7[2].re = ai / l6;
      t7[2].im = 0.0;
    } else if (ai == 0.0) {
      t7[2].re = 0.0;
      t7[2].im = -(ar / l6);
    } else {
      t7[2].re = ai / l6;
      t7[2].im = -(ar / l6);
    }
  } else {
    l13 = std::abs(l3);
    l9 = std::abs(l6);
    if (l13 > l9) {
      l9 = l6 / l3;
      l3 += l9 * l6;
      t7[2].re = (ar + l9 * ai) / l3;
      t7[2].im = (ai - l9 * ar) / l3;
    } else if (l9 == l13) {
      if (l3 > 0.0) {
        l9 = 0.5;
      } else {
        l9 = -0.5;
      }
      if (l6 > 0.0) {
        l3 = 0.5;
      } else {
        l3 = -0.5;
      }
      t7[2].re = (ar * l9 + ai * l3) / l13;
      t7[2].im = (ai * l9 - ar * l3) / l13;
    } else {
      l9 = l3 / l6;
      l3 = l6 + l9 * l3;
      t7[2].re = (l9 * ar + ai) / l3;
      t7[2].im = (l9 * ai - ar) / l3;
    }
  }
  re = J_min * t3[3].re;
  im = J_min * t3[3].im;
  l146_re = (A_init + re) + -A_wayp;
  l89 = l146_re * im;
  l3 = re * im;
  l6 = J_max * re;
  l9 = J_max * im;
  l13 = A_wayp - re;
  l2 = im * im;
  ar = (((((J_min * (l146_re * l146_re - l2) + J_max * (re * re - l2)) +
           l121.re) -
          l126.re) -
         l10 * l146_re * 2.0) +
        l124.re) +
       (l6 * l13 - l9 * (0.0 - im)) * 2.0;
  ai = ((J_min * (l89 + l89) + J_max * (l3 + l3)) - l10 * im * 2.0) +
       (l6 * (0.0 - im) + l9 * l13) * 2.0;
  l3 = ((l11_tmp * re * 2.0 + l127.re) - l129_re) - l96_tmp * l146_re * 2.0;
  l6 = l11_tmp * im * 2.0 - l96_tmp * im * 2.0;
  if (l6 == 0.0) {
    if (ai == 0.0) {
      t7[3].re = ar / l3;
      t7[3].im = 0.0;
    } else if (ar == 0.0) {
      t7[3].re = 0.0;
      t7[3].im = ai / l3;
    } else {
      t7[3].re = ar / l3;
      t7[3].im = ai / l3;
    }
  } else if (l3 == 0.0) {
    if (ar == 0.0) {
      t7[3].re = ai / l6;
      t7[3].im = 0.0;
    } else if (ai == 0.0) {
      t7[3].re = 0.0;
      t7[3].im = -(ar / l6);
    } else {
      t7[3].re = ai / l6;
      t7[3].im = -(ar / l6);
    }
  } else {
    l13 = std::abs(l3);
    l9 = std::abs(l6);
    if (l13 > l9) {
      l9 = l6 / l3;
      l3 += l9 * l6;
      t7[3].re = (ar + l9 * ai) / l3;
      t7[3].im = (ai - l9 * ar) / l3;
    } else if (l9 == l13) {
      if (l3 > 0.0) {
        l9 = 0.5;
      } else {
        l9 = -0.5;
      }
      if (l6 > 0.0) {
        l3 = 0.5;
      } else {
        l3 = -0.5;
      }
      t7[3].re = (ar * l9 + ai * l3) / l13;
      t7[3].im = (ai * l9 - ar * l3) / l13;
    } else {
      l9 = l3 / l6;
      l3 = l6 + l9 * l3;
      t7[3].re = (l9 * ar + ai) / l3;
      t7[3].im = (l9 * ai - ar) / l3;
    }
  }
  l127.re = A_init - A_wayp;
  ar = -((l127.re + b_l2[0].re) + J_max * t7[0].re);
  ai = -(b_l2[0].im + J_max * t7[0].im);
  if (ai == 0.0) {
    t[0].re = ar / J_max;
    t[0].im = 0.0;
  } else if (ar == 0.0) {
    t[0].re = 0.0;
    t[0].im = ai / J_max;
  } else {
    t[0].re = ar / J_max;
    t[0].im = ai / J_max;
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
  t[24] = t7[0];
  ar = -((l127.re + b_l2[1].re) + J_max * t7[1].re);
  ai = -(b_l2[1].im + J_max * t7[1].im);
  if (ai == 0.0) {
    t[1].re = ar / J_max;
    t[1].im = 0.0;
  } else if (ar == 0.0) {
    t[1].re = 0.0;
    t[1].im = ai / J_max;
  } else {
    t[1].re = ar / J_max;
    t[1].im = ai / J_max;
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
  t[25] = t7[1];
  ar = -((l127.re + b_l2[2].re) + J_max * t7[2].re);
  ai = -(b_l2[2].im + J_max * t7[2].im);
  if (ai == 0.0) {
    t[2].re = ar / J_max;
    t[2].im = 0.0;
  } else if (ar == 0.0) {
    t[2].re = 0.0;
    t[2].im = ai / J_max;
  } else {
    t[2].re = ar / J_max;
    t[2].im = ai / J_max;
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
  t[26] = t7[2];
  ar = -((l127.re + re) + J_max * t7[3].re);
  ai = -(im + J_max * t7[3].im);
  if (ai == 0.0) {
    t[3].re = ar / J_max;
    t[3].im = 0.0;
  } else if (ar == 0.0) {
    t[3].re = 0.0;
    t[3].im = ai / J_max;
  } else {
    t[3].re = ar / J_max;
    t[3].im = ai / J_max;
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
  t[27] = t7[3];
}

// End of code generation (aceg_O_AP.cpp)
