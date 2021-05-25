//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// acdefg_NTV_AVP.cpp
//
// Code generation for function 'acdefg_NTV_AVP'
//

// Include files
#include "acdefg_NTV_AVP.h"
#include "power.h"
#include "rt_nonfinite.h"
#include "sqrt.h"
#include "topico_wrapper_data.h"
#include "topico_wrapper_rtwutil.h"
#include <cmath>

// Function Definitions
void acdefg_NTV_AVP(double P_init, double V_init, double A_init, double P_wayp,
                    double V_wayp, double A_wayp, double A_min, double J_max,
                    double J_min, double T, creal_T t[28])
{
  creal_T l3[4];
  creal_T t3[4];
  creal_T t4_tmp[4];
  creal_T dc;
  creal_T l124;
  creal_T l129;
  creal_T l130;
  double a_tmp_tmp;
  double b_l112_tmp;
  double l10;
  double l101;
  double l112;
  double l112_tmp;
  double l11_tmp;
  double l12;
  double l131_im;
  double l139_im;
  double l14_tmp;
  double l15_tmp;
  double l16;
  double l18_tmp;
  double l2;
  double l20;
  double l33;
  double l33_tmp;
  double l5;
  double l56;
  double l57;
  double l58;
  double l6;
  double l62;
  double l63;
  double l7;
  double l74;
  double l86;
  double l9;
  double l93;
  double l94;
  double l95;
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
  l94 = A_min * A_min;
  l5 = rt_powd_snf(A_min, 3.0);
  l7 = A_wayp * A_wayp;
  l9 = J_min * J_min;
  l10 = J_max * J_max;
  l11_tmp = rt_powd_snf(J_max, 3.0);
  l12 = J_min * J_max;
  l14_tmp = l12 * V_init * 2.0;
  l15_tmp = l12 * V_wayp * 2.0;
  l6 = l94 * l94;
  l12 = l10 * l10;
  l16 = J_max * l2;
  l18_tmp = J_max * l94;
  l20 = J_min * l10;
  l33_tmp = A_min * J_min;
  l33 = l33_tmp * l12 * 12.0;
  l86 = l11_tmp - l20;
  l56 = 1.0 / (l86 * l86);
  a_tmp_tmp = J_min * l94;
  l131_im = ((((l14_tmp - l15_tmp) - l16) - a_tmp_tmp) + l18_tmp) + J_min * l7;
  l57 = l56 * l56;
  l58 = rt_powd_snf(l56, 3.0);
  l95 = A_min * l9;
  l62 = (A_min * rt_powd_snf(J_max, 5.0) * 8.0 + -l33) + l95 * l11_tmp * 4.0;
  l63 = l62 * l62;
  l74 = l56 * l62 / 12.0;
  l112 = A_min * A_wayp * J_min;
  l86 = ((((A_init * A_min * l12 * 12.0 + T * l33) + l18_tmp * l86 * -6.0) +
          l112 * l86 * -12.0) +
         -(A_min *
           (((A_init * J_min * l11_tmp * 3.0 + -(l33_tmp * l11_tmp * 3.0)) +
             T * l9 * l11_tmp * 3.0) +
            l95 * l10 * 3.0) *
           4.0)) +
        l86 * l131_im * 6.0;
  l139_im = A_min * J_max;
  l93 = l57 * l63 / 24.0 + -(l56 * l86 / 3.0);
  l101 = l58 * rt_powd_snf(l62, 3.0) / 216.0 + -(l57 * l62 * l86 / 18.0);
  l112_tmp = l57 * l57 * (l63 * l63);
  b_l112_tmp = l58 * l63 * l86;
  l16 = l56 * (((((((((l6 * l10 * 4.0 + l6 * l9 * 8.0) +
                      A_wayp * J_min * J_max * l5 * 12.0) +
                     A_min * rt_powd_snf(A_wayp, 3.0) * l9 * 4.0) +
                    A_min * P_init * l9 * l10 * 24.0) +
                   A_min * T * V_init * l9 * l10 * 24.0) +
                  l131_im * l131_im * 3.0) +
                 l112 * l131_im * -12.0) +
                l18_tmp * l131_im * -6.0) +
               -(A_min *
                 ((((((((rt_powd_snf(A_init, 3.0) * l10 + l5 * l9 * 3.0) +
                        l33_tmp * l16 * 3.0) +
                       l139_im * V_wayp * l9 * 6.0) +
                      P_wayp * l9 * l10 * 6.0) +
                     -(l139_im * V_init * l9 * 6.0)) +
                    A_wayp * l94 * l9 * 3.0) +
                   T * l2 * l20 * 3.0) +
                  -(A_min * l7 * l9 * 3.0)) *
                 4.0));
  l112 = (-(l112_tmp / 6912.0) + b_l112_tmp / 432.0) + l16 / 3.0;
  l94 = l93 * l93;
  l95 = rt_powd_snf(l93, 3.0);
  l86 = l101 * l101;
  l124.re = ((((l86 * l86 * 27.0 + -(l95 * l86 * 4.0)) +
               -(rt_powd_snf(l112, 3.0) * 256.0)) +
              -(l94 * l94 * l112 * 16.0)) +
             l94 * (l112 * l112) * 128.0) +
            l93 * l86 * l112 * 144.0;
  l124.im = 0.0;
  coder::internal::scalar::b_sqrt(&l124);
  l63 = 1.7320508075688772 * l124.re;
  l57 = 1.7320508075688772 * l124.im;
  if (l57 == 0.0) {
    l33 = l63 / 18.0;
    l62 = 0.0;
  } else if (l63 == 0.0) {
    l33 = 0.0;
    l62 = l57 / 18.0;
  } else {
    l33 = l63 / 18.0;
    l62 = l57 / 18.0;
  }
  l58 = l93 * l112;
  l129.re = ((-(l95 / 27.0) + l86 / 2.0) + l58 * 1.3333333333333333) + l33;
  l129.im = l62;
  l130 = coder::power(l129);
  dc = coder::b_power(l129);
  if (dc.im == 0.0) {
    l7 = 1.0 / dc.re;
    l9 = 0.0;
  } else if (dc.re == 0.0) {
    l7 = 0.0;
    l9 = -(1.0 / dc.im);
  } else {
    l33 = std::abs(dc.re);
    l139_im = std::abs(dc.im);
    if (l33 > l139_im) {
      l139_im = dc.im / dc.re;
      l12 = dc.re + l139_im * dc.im;
      l7 = (l139_im * 0.0 + 1.0) / l12;
      l9 = (0.0 - l139_im) / l12;
    } else if (l139_im == l33) {
      if (dc.re > 0.0) {
        l139_im = 0.5;
      } else {
        l139_im = -0.5;
      }
      if (dc.im > 0.0) {
        l12 = 0.5;
      } else {
        l12 = -0.5;
      }
      l7 = (l139_im + 0.0 * l12) / l33;
      l9 = (0.0 * l139_im - l12) / l33;
    } else {
      l139_im = dc.re / dc.im;
      l12 = dc.im + l139_im * dc.re;
      l7 = l139_im / l12;
      l9 = (l139_im * 0.0 - 1.0) / l12;
    }
  }
  l5 = l130.re * l130.re - l130.im * l130.im;
  l12 = l130.re * l130.im;
  l131_im = l12 + l12;
  dc.re = ((-(l95 * 2.0) + l86 * 27.0) + l58 * 72.0) + l63 * 3.0;
  dc.im = l57 * 3.0;
  coder::internal::scalar::b_sqrt(&dc);
  l56 = 3.0 * (2.4494897427831779 * l101 * dc.re);
  l20 = 3.0 * (2.4494897427831779 * l101 * dc.im);
  l95 = l93 * l130.re;
  l63 = l93 * l130.im;
  l129.re = ((((-(l112_tmp / 576.0) + b_l112_tmp / 36.0) + l94) + l16 * 4.0) +
             l5 * 9.0) +
            l95 * 6.0;
  l129.im = l131_im * 9.0 + l63 * 6.0;
  l124 = l129;
  coder::internal::scalar::b_sqrt(&l124);
  dc = coder::c_power(l129);
  if (dc.im == 0.0) {
    l86 = 1.0 / dc.re;
    l139_im = 0.0;
  } else if (dc.re == 0.0) {
    l86 = 0.0;
    l139_im = -(1.0 / dc.im);
  } else {
    l33 = std::abs(dc.re);
    l139_im = std::abs(dc.im);
    if (l33 > l139_im) {
      l139_im = dc.im / dc.re;
      l12 = dc.re + l139_im * dc.im;
      l86 = (l139_im * 0.0 + 1.0) / l12;
      l139_im = (0.0 - l139_im) / l12;
    } else if (l139_im == l33) {
      if (dc.re > 0.0) {
        l139_im = 0.5;
      } else {
        l139_im = -0.5;
      }
      if (dc.im > 0.0) {
        l12 = 0.5;
      } else {
        l12 = -0.5;
      }
      l86 = (l139_im + 0.0 * l12) / l33;
      l139_im = (0.0 * l139_im - l12) / l33;
    } else {
      l139_im = dc.re / dc.im;
      l12 = dc.im + l139_im * dc.re;
      l86 = l139_im / l12;
      l139_im = (l139_im * 0.0 - 1.0) / l12;
    }
  }
  l62 = -9.0 * (l5 * l124.re - l131_im * l124.im);
  l57 = -9.0 * (l5 * l124.im + l131_im * l124.re);
  l12 = l7 * l124.re - l9 * l124.im;
  l33 = l7 * l124.im + l9 * l124.re;
  if (l33 == 0.0) {
    l58 = l12 / 6.0;
    l33 = 0.0;
  } else if (l12 == 0.0) {
    l58 = 0.0;
    l33 /= 6.0;
  } else {
    l58 = l12 / 6.0;
    l33 /= 6.0;
  }
  l5 = 12.0 * (l95 * l124.re - l63 * l124.im);
  l131_im = 12.0 * (l95 * l124.im + l63 * l124.re);
  l130.re = -(l94 * l124.re);
  l130.im = -(l94 * l124.im);
  l129.re = -(l112 * l124.re * 12.0);
  l129.im = -(l112 * l124.im * 12.0);
  dc.re = (((l56 + l130.re) + l129.re) + l62) + l5;
  dc.im = (((l20 + l130.im) + l129.im) + l57) + l131_im;
  coder::internal::scalar::b_sqrt(&dc);
  l95 = l7 * l86 - l9 * l139_im;
  l12 = l7 * l139_im + l9 * l86;
  l7 = l95 * dc.re - l12 * dc.im;
  l9 = l95 * dc.im + l12 * dc.re;
  if (l9 == 0.0) {
    l139_im = l7 / 6.0;
    l86 = 0.0;
  } else if (l7 == 0.0) {
    l139_im = 0.0;
    l86 = l9 / 6.0;
  } else {
    l139_im = l7 / 6.0;
    l86 = l9 / 6.0;
  }
  dc.re = (((-l56 + l130.re) + l129.re) + l62) + l5;
  dc.im = (((-l20 + l130.im) + l129.im) + l57) + l131_im;
  coder::internal::scalar::b_sqrt(&dc);
  l7 = l95 * dc.re - l12 * dc.im;
  l9 = l95 * dc.im + l12 * dc.re;
  if (l9 == 0.0) {
    l129.re = l7 / 6.0;
    l129.im = 0.0;
  } else if (l7 == 0.0) {
    l129.re = 0.0;
    l129.im = l9 / 6.0;
  } else {
    l129.re = l7 / 6.0;
    l129.im = l9 / 6.0;
  }
  l12 = -l74 + -l58;
  t3[0].re = l12 - l139_im;
  t3[0].im = -l33 - l86;
  t3[1].re = l12 + l139_im;
  t3[1].im = -l33 + l86;
  l12 = -l74 + l58;
  t3[2].re = l12 - l129.re;
  t3[2].im = l33 - l129.im;
  t3[3].re = l12 + l129.re;
  t3[3].im = l33 + l129.im;
  l95 = l33_tmp * J_max * 2.0;
  l130.re = ((a_tmp_tmp - l18_tmp) + A_init * A_init * J_max) -
            A_wayp * A_wayp * J_min;
  l139_im = J_min * (J_max * J_max);
  l2 = 1.0 / J_min;
  l6 = -(1.0 / J_max * (A_min + -A_wayp));
  l86 = A_min * (1.0 / J_min);
  l33 = t3[0].re * t3[0].re - t3[0].im * t3[0].im;
  l12 = t3[0].re * t3[0].im;
  l62 = l12 + l12;
  l33 = (((l130.re - l11_tmp * l33) - l14_tmp) + l15_tmp) + l139_im * l33;
  l12 = (0.0 - l11_tmp * l62) + l139_im * l62;
  if (l12 == 0.0) {
    l33 /= l95;
    l62 = 0.0;
  } else if (l33 == 0.0) {
    l33 = 0.0;
    l62 = l12 / l95;
  } else {
    l33 /= l95;
    l62 = l12 / l95;
  }
  l3[0].re = l33;
  l3[0].im = l62;
  l33 = J_max * t3[0].re;
  l12 = J_max * t3[0].im;
  t4_tmp[0].re = l33;
  t4_tmp[0].im = l12;
  l33 = -(A_init + l33);
  if (-l12 == 0.0) {
    t[0].re = l33 / J_min;
    t[0].im = 0.0;
  } else if (l33 == 0.0) {
    t[0].re = 0.0;
    t[0].im = -l12 / J_min;
  } else {
    t[0].re = l33 / J_min;
    t[0].im = -l12 / J_min;
  }
  t[4].re = 0.0;
  t[4].im = 0.0;
  t[8] = t3[0];
  l33 = t3[1].re * t3[1].re - t3[1].im * t3[1].im;
  l12 = t3[1].re * t3[1].im;
  l62 = l12 + l12;
  l33 = (((l130.re - l11_tmp * l33) - l14_tmp) + l15_tmp) + l139_im * l33;
  l12 = (0.0 - l11_tmp * l62) + l139_im * l62;
  if (l12 == 0.0) {
    l33 /= l95;
    l62 = 0.0;
  } else if (l33 == 0.0) {
    l33 = 0.0;
    l62 = l12 / l95;
  } else {
    l33 /= l95;
    l62 = l12 / l95;
  }
  l3[1].re = l33;
  l3[1].im = l62;
  l33 = J_max * t3[1].re;
  l12 = J_max * t3[1].im;
  t4_tmp[1].re = l33;
  t4_tmp[1].im = l12;
  l33 = -(A_init + l33);
  if (-l12 == 0.0) {
    t[1].re = l33 / J_min;
    t[1].im = 0.0;
  } else if (l33 == 0.0) {
    t[1].re = 0.0;
    t[1].im = -l12 / J_min;
  } else {
    t[1].re = l33 / J_min;
    t[1].im = -l12 / J_min;
  }
  t[5].re = 0.0;
  t[5].im = 0.0;
  t[9] = t3[1];
  l33 = t3[2].re * t3[2].re - t3[2].im * t3[2].im;
  l12 = t3[2].re * t3[2].im;
  l62 = l12 + l12;
  l33 = (((l130.re - l11_tmp * l33) - l14_tmp) + l15_tmp) + l139_im * l33;
  l12 = (0.0 - l11_tmp * l62) + l139_im * l62;
  if (l12 == 0.0) {
    l33 /= l95;
    l62 = 0.0;
  } else if (l33 == 0.0) {
    l33 = 0.0;
    l62 = l12 / l95;
  } else {
    l33 /= l95;
    l62 = l12 / l95;
  }
  l3[2].re = l33;
  l3[2].im = l62;
  l33 = J_max * t3[2].re;
  l12 = J_max * t3[2].im;
  t4_tmp[2].re = l33;
  t4_tmp[2].im = l12;
  l33 = -(A_init + l33);
  if (-l12 == 0.0) {
    t[2].re = l33 / J_min;
    t[2].im = 0.0;
  } else if (l33 == 0.0) {
    t[2].re = 0.0;
    t[2].im = -l12 / J_min;
  } else {
    t[2].re = l33 / J_min;
    t[2].im = -l12 / J_min;
  }
  t[6].re = 0.0;
  t[6].im = 0.0;
  t[10] = t3[2];
  l33 = t3[3].re * t3[3].re - t3[3].im * t3[3].im;
  l12 = t3[3].re * t3[3].im;
  l62 = l12 + l12;
  l33 = (((l130.re - l11_tmp * l33) - l14_tmp) + l15_tmp) + l139_im * l33;
  l12 = (0.0 - l11_tmp * l62) + l139_im * l62;
  if (l12 == 0.0) {
    l33 /= l95;
    l62 = 0.0;
  } else if (l33 == 0.0) {
    l33 = 0.0;
    l62 = l12 / l95;
  } else {
    l33 /= l95;
    l62 = l12 / l95;
  }
  l3[3].re = l33;
  l3[3].im = l62;
  l12 = J_max * t3[3].im;
  l63 = A_init + J_max * t3[3].re;
  if (-l12 == 0.0) {
    t[3].re = -l63 / J_min;
    t[3].im = 0.0;
  } else if (-l63 == 0.0) {
    t[3].re = 0.0;
    t[3].im = -l12 / J_min;
  } else {
    t[3].re = -l63 / J_min;
    t[3].im = -l12 / J_min;
  }
  t[7].re = 0.0;
  t[7].im = 0.0;
  t[11] = t3[3];
  l129.re = (A_min - A_wayp) / J_max;
  l130.re = J_min * T;
  t[16].re = l86;
  t[16].im = 0.0;
  t[17].re = l86;
  t[17].im = 0.0;
  t[18].re = l86;
  t[18].im = 0.0;
  t[19].re = l86;
  t[19].im = 0.0;
  t[12].re = l2 * (((A_init + t4_tmp[0].re) -
                    J_min * (((t3[0].re + l3[0].re) + l86) - l129.re)) +
                   l130.re);
  t[12].im = l2 * (t4_tmp[0].im - J_min * (t3[0].im + l3[0].im));
  t[20] = l3[0];
  t[13].re = l2 * (((A_init + t4_tmp[1].re) -
                    J_min * (((t3[1].re + l3[1].re) + l86) - l129.re)) +
                   l130.re);
  t[13].im = l2 * (t4_tmp[1].im - J_min * (t3[1].im + l3[1].im));
  t[21] = l3[1];
  t[14].re = l2 * (((A_init + t4_tmp[2].re) -
                    J_min * (((t3[2].re + l3[2].re) + l86) - l129.re)) +
                   l130.re);
  t[14].im = l2 * (t4_tmp[2].im - J_min * (t3[2].im + l3[2].im));
  t[22] = l3[2];
  t[15].re =
      l2 * ((l63 - J_min * (((t3[3].re + l33) + l86) - l129.re)) + l130.re);
  t[15].im = l2 * (l12 - J_min * (t3[3].im + l62));
  t[23] = l3[3];
  t[24].re = l6;
  t[24].im = 0.0;
  t[25].re = l6;
  t[25].im = 0.0;
  t[26].re = l6;
  t[26].im = 0.0;
  t[27].re = l6;
  t[27].im = 0.0;
}

// End of code generation (acdefg_NTV_AVP.cpp)
