//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// aceg_O_AVP.cpp
//
// Code generation for function 'aceg_O_AVP'
//

// Include files
#include "aceg_O_AVP.h"
#include "power.h"
#include "rt_nonfinite.h"
#include "sqrt.h"
#include "topico_wrapper_data.h"
#include "topico_wrapper_rtwutil.h"
#include <cmath>

// Function Definitions
void aceg_O_AVP(double P_init, double V_init, double A_init, double P_wayp,
                double V_wayp, double A_wayp, double J_max, double J_min,
                creal_T t[35])
{
  creal_T l2[5];
  creal_T t3[5];
  creal_T b_l78;
  creal_T l110;
  creal_T l113;
  creal_T l115;
  creal_T l116;
  double a_tmp;
  double ar;
  double b_l78_tmp;
  double im;
  double l118_im;
  double l118_re;
  double l11_tmp;
  double l12;
  double l130_im;
  double l130_re;
  double l132_re;
  double l135_re;
  double l2_tmp;
  double l3;
  double l5_tmp;
  double l6;
  double l74;
  double l76;
  double l78;
  double l78_tmp;
  double l81;
  double l81_tmp;
  double l84;
  double l84_tmp;
  double l85;
  double l86;
  double l8_tmp;
  double l95;
  double re;
  int k;
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
  l3 = rt_powd_snf(A_init, 3.0);
  l5_tmp = A_wayp * A_wayp;
  l6 = rt_powd_snf(A_wayp, 3.0);
  l8_tmp = J_min * J_min;
  l11_tmp = J_max * J_max;
  l12 = rt_powd_snf(J_max, 3.0);
  a_tmp = ((l8_tmp * l8_tmp - J_min * l12 * 2.0) -
           J_max * rt_powd_snf(J_min, 3.0) * 4.0) +
          l8_tmp * l11_tmp * 5.0;
  l74 = 1.0 / (a_tmp * a_tmp);
  l76 = l74 * l74;
  l78_tmp = J_max * V_init;
  b_l78_tmp = J_max * V_wayp;
  l78 = ((((((((l2_tmp * l2_tmp * 3.0 + l5_tmp * l5_tmp * 3.0) +
               b_l78_tmp * l2_tmp * 12.0) +
              l78_tmp * l5_tmp * 12.0) +
             -(l78_tmp * l2_tmp * 12.0)) +
            -(b_l78_tmp * l5_tmp * 12.0)) +
           -(l2_tmp * l5_tmp * 6.0)) +
          -(V_init * V_wayp * l11_tmp * 24.0)) +
         l11_tmp * (V_init * V_init) * 12.0) +
        l11_tmp * (V_wayp * V_wayp) * 12.0;
  l81_tmp = A_wayp * J_min;
  l81 = ((((((((((J_min * l3 * 8.0 + J_max * l6 * 8.0) + P_wayp * l12 * 24.0) +
                l81_tmp * J_max * V_wayp * 24.0) +
               -(J_max * l3 * 8.0)) +
              -(J_min * l6 * 8.0)) +
             -(P_init * l12 * 24.0)) +
            -(A_init * J_min * J_max * V_init * 24.0)) +
           A_init * V_init * l11_tmp * 24.0) +
          J_min * P_init * l11_tmp * 24.0) +
         -(A_wayp * V_wayp * l11_tmp * 24.0)) +
        -(J_min * P_wayp * l11_tmp * 24.0);
  l84_tmp = J_min * J_max;
  l84 = ((((((((((V_init * l12 * 12.0 + V_wayp * l12 * 12.0) +
                 l84_tmp * l2_tmp * 12.0) +
                l84_tmp * l5_tmp * 12.0) +
               l78_tmp * l8_tmp * 12.0) +
              b_l78_tmp * l8_tmp * 12.0) +
             -(J_min * V_init * l11_tmp * 24.0)) +
            -(J_min * V_wayp * l11_tmp * 24.0)) +
           -(l2_tmp * l8_tmp * 6.0)) +
          -(l2_tmp * l11_tmp * 6.0)) +
         -(l5_tmp * l8_tmp * 6.0)) +
        -(l5_tmp * l11_tmp * 6.0);
  l6 = l81 * l81;
  l85 = l84 * l84;
  l86 = rt_powd_snf(l84, 3.0);
  l95 = l74 * l85;
  l3 = rt_powd_snf(a_tmp, 5.0);
  l135_re = rt_powd_snf(a_tmp, 3.0);
  l110.re =
      ((((rt_powd_snf(l78, 3.0) / l135_re * 256.0 + l76 * (l6 * l6) * 27.0) +
         l78 * (l85 * l85) / l3 * 16.0) +
        l76 * (l78 * l78) * l85 * 128.0) +
       l6 * l86 / l3 * 4.0) +
      l76 * l78 * l6 * l84 * 144.0;
  l110.im = 0.0;
  coder::internal::scalar::b_sqrt(&l110);
  l132_re = 1.7320508075688772 * l110.re;
  l76 = 1.7320508075688772 * l110.im;
  if (l76 == 0.0) {
    re = l132_re / 18.0;
    im = 0.0;
  } else if (l132_re == 0.0) {
    re = 0.0;
    im = l76 / 18.0;
  } else {
    re = l132_re / 18.0;
    im = l76 / 18.0;
  }
  l12 = l74 * l6;
  l6 = l86 / l135_re;
  l3 = l74 * l78 * l84;
  l115.re = ((l12 / 2.0 + l6 / 27.0) + l3 * 1.3333333333333333) + re;
  l115.im = im;
  l113.re = ((l12 * 27.0 + l6 * 2.0) + l3 * 72.0) + l132_re * 3.0;
  l113.im = l76 * 3.0;
  coder::internal::scalar::b_sqrt(&l113);
  l116 = coder::power(l115);
  b_l78 = coder::b_power(l115);
  if (b_l78.im == 0.0) {
    l118_re = 1.0 / b_l78.re;
    l118_im = 0.0;
  } else if (b_l78.re == 0.0) {
    l118_re = 0.0;
    l118_im = -(1.0 / b_l78.im);
  } else {
    l86 = std::abs(b_l78.re);
    l76 = std::abs(b_l78.im);
    if (l86 > l76) {
      l76 = b_l78.im / b_l78.re;
      l3 = b_l78.re + l76 * b_l78.im;
      l118_re = (l76 * 0.0 + 1.0) / l3;
      l118_im = (0.0 - l76) / l3;
    } else if (l76 == l86) {
      if (b_l78.re > 0.0) {
        l76 = 0.5;
      } else {
        l76 = -0.5;
      }
      if (b_l78.im > 0.0) {
        l3 = 0.5;
      } else {
        l3 = -0.5;
      }
      l118_re = (l76 + 0.0 * l3) / l86;
      l118_im = (0.0 * l76 - l3) / l86;
    } else {
      l76 = b_l78.re / b_l78.im;
      l3 = b_l78.im + l76 * b_l78.re;
      l118_re = l76 / l3;
      l118_im = (l76 * 0.0 - 1.0) / l3;
    }
  }
  l110.re = l116.re * l116.re - l116.im * l116.im;
  l3 = l116.re * l116.im;
  l110.im = l3 + l3;
  l12 = l84 * l116.re;
  l76 = l84 * l116.im;
  ar = l12 * -2.0;
  im = l76 * -2.0;
  if (im == 0.0) {
    l6 = ar / a_tmp;
    l3 = 0.0;
  } else if (ar == 0.0) {
    l6 = 0.0;
    l3 = im / a_tmp;
  } else {
    l6 = ar / a_tmp;
    l3 = im / a_tmp;
  }
  l115.re = ((l78 * -4.0 / a_tmp + l95 / 3.0) + l110.re * 3.0) + l6;
  l115.im = l110.im * 3.0 + l3;
  coder::internal::scalar::b_sqrt(&l115);
  ar = l12 * -6.0;
  im = l76 * -6.0;
  if (im == 0.0) {
    l6 = ar / a_tmp;
    l3 = 0.0;
  } else if (ar == 0.0) {
    l6 = 0.0;
    l3 = im / a_tmp;
  } else {
    l6 = ar / a_tmp;
    l3 = im / a_tmp;
  }
  b_l78.re = ((l78 * -12.0 / a_tmp + l95) + l110.re * 9.0) + l6;
  b_l78.im = l110.im * 9.0 + l3;
  b_l78 = coder::c_power(b_l78);
  if (b_l78.im == 0.0) {
    l12 = 1.0 / b_l78.re;
    l6 = 0.0;
  } else if (b_l78.re == 0.0) {
    l12 = 0.0;
    l6 = -(1.0 / b_l78.im);
  } else {
    l86 = std::abs(b_l78.re);
    l76 = std::abs(b_l78.im);
    if (l86 > l76) {
      l76 = b_l78.im / b_l78.re;
      l3 = b_l78.re + l76 * b_l78.im;
      l12 = (l76 * 0.0 + 1.0) / l3;
      l6 = (0.0 - l76) / l3;
    } else if (l76 == l86) {
      if (b_l78.re > 0.0) {
        l76 = 0.5;
      } else {
        l76 = -0.5;
      }
      if (b_l78.im > 0.0) {
        l3 = 0.5;
      } else {
        l3 = -0.5;
      }
      l12 = (l76 + 0.0 * l3) / l86;
      l6 = (0.0 * l76 - l3) / l86;
    } else {
      l76 = b_l78.re / b_l78.im;
      l3 = b_l78.im + l76 * b_l78.re;
      l12 = l76 / l3;
      l6 = (l76 * 0.0 - 1.0) / l3;
    }
  }
  ar = 1.7320508075688772 * l78 * l115.re * 12.0;
  im = 1.7320508075688772 * l78 * l115.im * 12.0;
  if (im == 0.0) {
    l130_re = ar / a_tmp;
    l130_im = 0.0;
  } else if (ar == 0.0) {
    l130_re = 0.0;
    l130_im = im / a_tmp;
  } else {
    l130_re = ar / a_tmp;
    l130_im = im / a_tmp;
  }
  l132_re = 1.7320508075688772 * (-1.0 / a_tmp) * l85;
  ar = l132_re * l115.re;
  im = l132_re * l115.im;
  if (im == 0.0) {
    l132_re = ar / a_tmp;
    l95 = 0.0;
  } else if (ar == 0.0) {
    l132_re = 0.0;
    l95 = im / a_tmp;
  } else {
    l132_re = ar / a_tmp;
    l95 = im / a_tmp;
  }
  re = 1.7320508075688772 * l110.re;
  im = 1.7320508075688772 * l110.im;
  l85 = -9.0 * (re * l115.re - im * l115.im);
  l78 = -9.0 * (re * l115.im + im * l115.re);
  re = 1.7320508075688772 * l118_re;
  im = 1.7320508075688772 * l118_im;
  l3 = re * l115.re - im * l115.im;
  im = re * l115.im + im * l115.re;
  if (im == 0.0) {
    l135_re = l3 / 6.0;
    l74 = 0.0;
  } else if (l3 == 0.0) {
    l135_re = 0.0;
    l74 = im / 6.0;
  } else {
    l135_re = l3 / 6.0;
    l74 = im / 6.0;
  }
  re = 1.7320508075688772 * l84 * l116.re;
  im = 1.7320508075688772 * l84 * l116.im;
  ar = (re * l115.re - im * l115.im) * -12.0;
  im = (re * l115.im + im * l115.re) * -12.0;
  if (im == 0.0) {
    l110.re = ar / a_tmp;
    l110.im = 0.0;
  } else if (ar == 0.0) {
    l110.re = 0.0;
    l110.im = im / a_tmp;
  } else {
    l110.re = ar / a_tmp;
    l110.im = im / a_tmp;
  }
  l86 = 2.4494897427831779 * l81 * l113.re;
  l84 = 2.4494897427831779 * l81 * l113.im;
  ar = l86 * -3.0;
  im = l84 * -3.0;
  if (im == 0.0) {
    re = ar / a_tmp;
    im = 0.0;
  } else if (ar == 0.0) {
    re = 0.0;
    im /= a_tmp;
  } else {
    re = ar / a_tmp;
    im /= a_tmp;
  }
  l115.re = (((re + l130_re) + l132_re) + l85) + l110.re;
  l115.im = (((im + l130_im) + l95) + l78) + l110.im;
  coder::internal::scalar::b_sqrt(&l115);
  l76 = l118_re * l12 - l118_im * l6;
  l3 = l118_re * l6 + l118_im * l12;
  l118_re = l76 * l115.re - l3 * l115.im;
  l118_im = l76 * l115.im + l3 * l115.re;
  if (l118_im == 0.0) {
    l6 = l118_re / 6.0;
    l12 = 0.0;
  } else if (l118_re == 0.0) {
    l6 = 0.0;
    l12 = l118_im / 6.0;
  } else {
    l6 = l118_re / 6.0;
    l12 = l118_im / 6.0;
  }
  ar = l86 * 3.0;
  im = l84 * 3.0;
  if (im == 0.0) {
    re = ar / a_tmp;
    im = 0.0;
  } else if (ar == 0.0) {
    re = 0.0;
    im /= a_tmp;
  } else {
    re = ar / a_tmp;
    im /= a_tmp;
  }
  l115.re = (((re + l130_re) + l132_re) + l85) + l110.re;
  l115.im = (((im + l130_im) + l95) + l78) + l110.im;
  coder::internal::scalar::b_sqrt(&l115);
  l118_re = l76 * l115.re - l3 * l115.im;
  l118_im = l76 * l115.im + l3 * l115.re;
  if (l118_im == 0.0) {
    l110.re = l118_re / 6.0;
    l110.im = 0.0;
  } else if (l118_re == 0.0) {
    l110.re = 0.0;
    l110.im = l118_im / 6.0;
  } else {
    l110.re = l118_re / 6.0;
    l110.im = l118_im / 6.0;
  }
  t3[0].re = 0.0;
  t3[0].im = 0.0;
  t3[1].re = l135_re - l110.re;
  t3[1].im = l74 - l110.im;
  t3[2].re = l135_re + l110.re;
  t3[2].im = l74 + l110.im;
  t3[3].re = -l135_re - l6;
  t3[3].im = -l74 - l12;
  t3[4].re = -l135_re + l6;
  t3[4].im = -l74 + l12;
  l110.re = l78_tmp * 2.0 - b_l78_tmp * 2.0;
  l12 = A_wayp * J_max;
  for (k = 0; k < 5; k++) {
    l3 = t3[k].re;
    l6 = t3[k].im;
    re = l3 * l3 - l6 * l6;
    l84 = l3 * l6;
    im = l84 + l84;
    ar = (((((l110.re + l8_tmp * re) - l2_tmp) + l5_tmp) - l81_tmp * l3 * 2.0) +
          l12 * l3 * 2.0) -
         l84_tmp * re;
    im = ((l8_tmp * im - l81_tmp * l6 * 2.0) + l12 * l6 * 2.0) - l84_tmp * im;
    l3 = l11_tmp * l3 * 2.0 - l84_tmp * l3 * 2.0;
    l6 = l11_tmp * l6 * 2.0 - l84_tmp * l6 * 2.0;
    if (l6 == 0.0) {
      if (im == 0.0) {
        re = ar / l3;
        im = 0.0;
      } else if (ar == 0.0) {
        re = 0.0;
        im /= l3;
      } else {
        re = ar / l3;
        im /= l3;
      }
    } else if (l3 == 0.0) {
      if (ar == 0.0) {
        re = im / l6;
        im = 0.0;
      } else if (im == 0.0) {
        re = 0.0;
        im = -(ar / l6);
      } else {
        re = im / l6;
        im = -(ar / l6);
      }
    } else {
      l86 = std::abs(l3);
      l76 = std::abs(l6);
      if (l86 > l76) {
        l76 = l6 / l3;
        l3 += l76 * l6;
        re = (ar + l76 * im) / l3;
        im = (im - l76 * ar) / l3;
      } else if (l76 == l86) {
        if (l3 > 0.0) {
          l76 = 0.5;
        } else {
          l76 = -0.5;
        }
        if (l6 > 0.0) {
          l3 = 0.5;
        } else {
          l3 = -0.5;
        }
        re = (ar * l76 + im * l3) / l86;
        im = (im * l76 - ar * l3) / l86;
      } else {
        l76 = l3 / l6;
        l3 = l6 + l76 * l3;
        re = (l76 * ar + im) / l3;
        im = (l76 * im - ar) / l3;
      }
    }
    l2[k].re = re;
    l2[k].im = im;
  }
  l110.re = A_init - A_wayp;
  for (k = 0; k < 5; k++) {
    ar = -((l110.re + J_min * t3[k].re) + J_max * l2[k].re);
    im = -(J_min * t3[k].im + J_max * l2[k].im);
    if (im == 0.0) {
      t[k].re = ar / J_max;
      t[k].im = 0.0;
    } else if (ar == 0.0) {
      t[k].re = 0.0;
      t[k].im = im / J_max;
    } else {
      t[k].re = ar / J_max;
      t[k].im = im / J_max;
    }
    t[k + 5].re = 0.0;
    t[k + 5].im = 0.0;
    t[k + 10] = t3[k];
    t[k + 15].re = 0.0;
    t[k + 15].im = 0.0;
    t[k + 20].re = 0.0;
    t[k + 20].im = 0.0;
    t[k + 25].re = 0.0;
    t[k + 25].im = 0.0;
    t[k + 30] = l2[k];
  }
}

// End of code generation (aceg_O_AVP.cpp)
