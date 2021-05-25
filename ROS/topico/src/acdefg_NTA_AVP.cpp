//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// acdefg_NTA_AVP.cpp
//
// Code generation for function 'acdefg_NTA_AVP'
//

// Include files
#include "acdefg_NTA_AVP.h"
#include "power.h"
#include "rt_nonfinite.h"
#include "sqrt.h"
#include "topico_wrapper_data.h"
#include "topico_wrapper_rtwutil.h"
#include "topico_wrapper_types.h"
#include <cmath>

// Function Definitions
void acdefg_NTA_AVP(double P_init, double V_init, double A_init, double P_wayp,
                    double V_wayp, double A_wayp, double V_max, double J_max,
                    double J_min, double T, creal_T t[28])
{
  creal_T l10[4];
  creal_T l12[4];
  creal_T t3[4];
  creal_T t5[4];
  creal_T l128;
  creal_T l55;
  creal_T l56;
  creal_T l66;
  creal_T l67;
  creal_T y;
  double J_max_tmp;
  double J_min_re_tmp;
  double J_min_tmp;
  double V_init_tmp;
  double b_A_wayp;
  double b_J_max;
  double b_l5_re_tmp;
  double b_l95_tmp;
  double b_y;
  double bim;
  double brm;
  double c_l5_re_tmp;
  double c_l95_tmp;
  double c_y;
  double d_l5_re_tmp;
  double d_l95_tmp;
  double e_l5_re_tmp;
  double im;
  double l102;
  double l11;
  double l12_tmp;
  double l130_re;
  double l133_re;
  double l137_im;
  double l139_im;
  double l139_re;
  double l144_re;
  double l2;
  double l24;
  double l2_tmp;
  double l38;
  double l38_tmp;
  double l45;
  double l4_tmp;
  double l57;
  double l57_tmp;
  double l58;
  double l5_re_tmp;
  double l60;
  double l67_tmp;
  double l6_tmp;
  double l7_im;
  double l7_re;
  double l7_tmp;
  double l85;
  double l85_tmp;
  double l86;
  double l87;
  double l88;
  double l89;
  double l8_tmp;
  double l95;
  double l95_tmp;
  double l96;
  double l97;
  double l9_tmp;
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
  //  Generated on 17-Sep-2019 10:06:26
  l2_tmp = A_init * A_init;
  l4_tmp = A_wayp * A_wayp;
  l133_re = rt_powd_snf(A_wayp, 3.0);
  l6_tmp = J_min * J_min;
  l7_tmp = rt_powd_snf(J_min, 3.0);
  l9_tmp = J_max * J_max;
  l12_tmp = J_min * J_max;
  if (J_max < 0.0) {
    k_rtErrorWithMessageID(o_emlrtRTEI.fName, o_emlrtRTEI.lineNo);
  }
  l24 = rt_powd_snf(J_max, 2.5);
  l8_tmp = l6_tmp * l6_tmp;
  l11 = rt_powd_snf(l6_tmp, 3.0);
  l38_tmp = J_min + -J_max;
  l38 = 1.0 / l38_tmp;
  l45 = l6_tmp + -l12_tmp;
  y.re = J_max + -J_min;
  y.im = 0.0;
  coder::internal::scalar::b_sqrt(&y);
  if (y.im == 0.0) {
    l55.re = 1.0 / y.re;
    l55.im = 0.0;
  } else if (y.re == 0.0) {
    l55.re = 0.0;
    l55.im = -(1.0 / y.im);
  } else {
    brm = std::abs(y.re);
    bim = std::abs(y.im);
    if (brm > bim) {
      bim = y.im / y.re;
      l58 = y.re + bim * y.im;
      l55.re = (bim * 0.0 + 1.0) / l58;
      l55.im = (0.0 - bim) / l58;
    } else if (bim == brm) {
      if (y.re > 0.0) {
        bim = 0.5;
      } else {
        bim = -0.5;
      }
      if (y.im > 0.0) {
        l58 = 0.5;
      } else {
        l58 = -0.5;
      }
      l55.re = (bim + 0.0 * l58) / brm;
      l55.im = (0.0 * bim - l58) / brm;
    } else {
      bim = y.re / y.im;
      l58 = y.im + bim * y.re;
      l55.re = bim / l58;
      l55.im = (bim * 0.0 - 1.0) / l58;
    }
  }
  l57_tmp = J_min * V_init;
  l57 = (l2_tmp + J_min * V_max * 2.0) + -(l57_tmp * 2.0);
  l58 = (l4_tmp + J_max * V_max * 2.0) + -(J_max * V_wayp * 2.0);
  l56 = coder::d_power(l55);
  l60 = l58 * l58;
  l66.re = l57;
  l66.im = 0.0;
  coder::internal::scalar::b_sqrt(&l66);
  l67 = coder::d_power(l66);
  l85_tmp = l7_tmp * l9_tmp;
  l85 = 1.0 / ((((l11 * 8.0 + l8_tmp * l9_tmp * 4.0) + -(l8_tmp * l45 * 12.0)) +
                l6_tmp * (l45 * l45) * 3.0) +
               -(J_min * (J_max * l8_tmp * 3.0 + l85_tmp * 3.0) * 2.0));
  l95_tmp = J_min * l2_tmp;
  b_l95_tmp = rt_powd_snf(J_max, 3.0);
  c_l95_tmp = l95_tmp * l9_tmp;
  d_l95_tmp = V_init * l6_tmp;
  l95 = (-(l8_tmp * l58 * 12.0) + l6_tmp * l45 * l58 * 6.0) +
        J_min *
            (((((d_l95_tmp * l9_tmp * 6.0 + J_max * l4_tmp * l6_tmp * 3.0) +
                -(V_wayp * l6_tmp * l9_tmp * 6.0)) +
               -(c_l95_tmp * 3.0)) +
              -(J_min * b_l95_tmp * l38 * l57 * 3.0)) +
             l6_tmp * l9_tmp * l38 * l57 * 3.0) *
            2.0;
  l86 = l85 * l85;
  l87 = rt_powd_snf(l85, 3.0);
  l89 = rt_powd_snf(l85, 5.0);
  l96 = l95 * l95;
  l97 = rt_powd_snf(l95, 3.0);
  l88 = l86 * l86;
  l102 = l86 * l96;
  l58 = l7_tmp * rt_powd_snf(J_max, 1.5);
  l45 = l58 * l56.re;
  l7_im = l58 * l56.im;
  l7_re = l45 * l67.re - l7_im * l67.im;
  l7_im = l45 * l67.im + l7_im * l67.re;
  J_min_re_tmp = l95_tmp * l24;
  l137_im = J_min_re_tmp * l55.re;
  l139_im = J_min_re_tmp * l55.im;
  l58 = V_max * l6_tmp * l24;
  l139_re = l58 * l55.re;
  l144_re = l58 * l55.im;
  l58 = rt_powd_snf(J_max, 3.5);
  re = l58 * l56.re;
  im = l58 * l56.im;
  J_min_re_tmp = l57_tmp * l24;
  l24 = J_min_re_tmp * l55.re;
  l58 = J_min_re_tmp * l55.im;
  J_min_re_tmp = A_wayp * J_max;
  l5_re_tmp = A_wayp * J_min;
  b_l5_re_tmp = l133_re * l6_tmp;
  c_l5_re_tmp = P_wayp * l6_tmp * l9_tmp;
  d_l5_re_tmp = A_init * J_min * V_init * l9_tmp;
  e_l5_re_tmp = J_min_re_tmp * V_init * l6_tmp;
  l45 = ((((((((b_l5_re_tmp * 6.0 + -(A_wayp * l2_tmp * l12_tmp * 6.0)) +
               d_l5_re_tmp * 12.0) +
              e_l5_re_tmp * 12.0) +
             c_l5_re_tmp * 12.0) +
            -(J_min_re_tmp * V_wayp * l6_tmp * 12.0)) +
           J_min_re_tmp * l6_tmp * l38 * l57 * 6.0) +
          -(l5_re_tmp * l9_tmp * l38 * l57 * 6.0)) +
         (re * l67.re - im * l67.im) * 2.0) +
        (l24 * l66.re - l58 * l66.im) * 12.0;
  l58 =
      (re * l67.im + im * l67.re) * 2.0 + (l24 * l66.im + l58 * l66.re) * 12.0;
  l67_tmp = rt_powd_snf(A_init, 3.0);
  l67.re = (((((((l133_re * l7_tmp * 4.0 + P_init * l7_tmp * l9_tmp * 24.0) +
                 J_min * l67_tmp * l9_tmp * 8.0) +
                A_init * V_max * l6_tmp * l9_tmp * 24.0) +
               T * V_max * l7_tmp * l9_tmp * 24.0) +
              l7_re * 4.0) +
             (l137_im * l66.re - l139_im * l66.im) * 12.0) +
            (l139_re * l66.re - l144_re * l66.im) * 24.0) +
           -(J_min * l45 * 2.0);
  l67.im = ((l7_im * 4.0 + (l137_im * l66.im + l139_im * l66.re) * 12.0) +
            (l139_re * l66.im + l144_re * l66.re) * 24.0) +
           -(J_min * l58 * 2.0);
  l56.re = l67.re * l67.re - l67.im * l67.im;
  l58 = l67.re * l67.im;
  l56.im = l58 + l58;
  l58 = l56.re * l56.im;
  l45 = l89 * l97;
  l57 = l6_tmp * l60;
  l24 = l57 * l88 * l95;
  l66.re = ((((-(l11 * rt_powd_snf(l60, 3.0) * l87 * 6912.0) +
               -(l57 * l89 * (l96 * l96) * 48.0)) +
              l8_tmp * (l60 * l60) * l88 * l96 * 1152.0) +
             l88 * (l56.re * l56.re - l56.im * l56.im) * 27.0) +
            -(l45 * l56.re * 4.0)) +
           l24 * l56.re * 432.0;
  l66.im =
      (l88 * (l58 + l58) * 27.0 + -(l45 * l56.im * 4.0)) + l24 * l56.im * 432.0;
  coder::internal::scalar::b_sqrt(&l66);
  l144_re = l86 * l56.re;
  l137_im = l86 * l56.im;
  if (l137_im == 0.0) {
    l58 = l144_re / 2.0;
    l45 = 0.0;
  } else if (l144_re == 0.0) {
    l58 = 0.0;
    l45 = l137_im / 2.0;
  } else {
    l58 = l144_re / 2.0;
    l45 = l137_im / 2.0;
  }
  l139_im = 1.7320508075688772 * l66.re;
  l139_re = 1.7320508075688772 * l66.im;
  if (l139_re == 0.0) {
    re = l139_im / 18.0;
    im = 0.0;
  } else if (l139_im == 0.0) {
    re = 0.0;
    im = l139_re / 18.0;
  } else {
    re = l139_im / 18.0;
    im = l139_re / 18.0;
  }
  l60 = l87 * l97;
  l24 = l57 * l86 * l95;
  l55.re = ((-(l60 / 27.0) + l24 * 4.0) + l58) + re;
  l55.im = l45 + im;
  l128 = coder::power(l55);
  y = coder::b_power(l55);
  if (y.im == 0.0) {
    l130_re = 1.0 / y.re;
    l96 = 0.0;
  } else if (y.re == 0.0) {
    l130_re = 0.0;
    l96 = -(1.0 / y.im);
  } else {
    brm = std::abs(y.re);
    bim = std::abs(y.im);
    if (brm > bim) {
      bim = y.im / y.re;
      l58 = y.re + bim * y.im;
      l130_re = (bim * 0.0 + 1.0) / l58;
      l96 = (0.0 - bim) / l58;
    } else if (bim == brm) {
      if (y.re > 0.0) {
        bim = 0.5;
      } else {
        bim = -0.5;
      }
      if (y.im > 0.0) {
        l58 = 0.5;
      } else {
        l58 = -0.5;
      }
      l130_re = (bim + 0.0 * l58) / brm;
      l96 = (0.0 * bim - l58) / brm;
    } else {
      bim = y.re / y.im;
      l58 = y.im + bim * y.re;
      l130_re = bim / l58;
      l96 = (bim * 0.0 - 1.0) / l58;
    }
  }
  l88 = l128.re * l128.re - l128.im * l128.im;
  l58 = l128.re * l128.im;
  l89 = l58 + l58;
  y.re = ((-(l60 * 2.0) + l24 * 216.0) + l144_re * 27.0) + l139_im * 3.0;
  y.im = l137_im * 27.0 + l139_re * 3.0;
  coder::internal::scalar::b_sqrt(&y);
  re = 2.4494897427831779 * l85 * l67.re;
  im = 2.4494897427831779 * l85 * l67.im;
  l133_re = 3.0 * (re * y.re - im * y.im);
  l38 = 3.0 * (re * y.im + im * y.re);
  l58 = l85 * l95;
  l7_re = l58 * l128.re;
  l7_im = l58 * l128.im;
  l60 = l57 * l85;
  l55.re = ((l60 * 36.0 + l102) + l88 * 9.0) + l7_re * 6.0;
  l55.im = l89 * 9.0 + l7_im * 6.0;
  l56 = l55;
  coder::internal::scalar::b_sqrt(&l56);
  y = coder::c_power(l55);
  if (y.im == 0.0) {
    l57 = 1.0 / y.re;
    l137_im = 0.0;
  } else if (y.re == 0.0) {
    l57 = 0.0;
    l137_im = -(1.0 / y.im);
  } else {
    brm = std::abs(y.re);
    bim = std::abs(y.im);
    if (brm > bim) {
      bim = y.im / y.re;
      l58 = y.re + bim * y.im;
      l57 = (bim * 0.0 + 1.0) / l58;
      l137_im = (0.0 - bim) / l58;
    } else if (bim == brm) {
      if (y.re > 0.0) {
        bim = 0.5;
      } else {
        bim = -0.5;
      }
      if (y.im > 0.0) {
        l58 = 0.5;
      } else {
        l58 = -0.5;
      }
      l57 = (bim + 0.0 * l58) / brm;
      l137_im = (0.0 * bim - l58) / brm;
    } else {
      bim = y.re / y.im;
      l58 = y.im + bim * y.re;
      l57 = bim / l58;
      l137_im = (bim * 0.0 - 1.0) / l58;
    }
  }
  l139_re = -36.0 * (l60 * l56.re);
  l139_im = -36.0 * (l60 * l56.im);
  l58 = l88 * l56.re - l89 * l56.im;
  l89 = l88 * l56.im + l89 * l56.re;
  l88 = -9.0 * l58;
  l89 *= -9.0;
  l58 = l130_re * l56.re - l96 * l56.im;
  l45 = l130_re * l56.im + l96 * l56.re;
  if (l45 == 0.0) {
    l144_re = l58 / 6.0;
    l24 = 0.0;
  } else if (l58 == 0.0) {
    l144_re = 0.0;
    l24 = l45 / 6.0;
  } else {
    l144_re = l58 / 6.0;
    l24 = l45 / 6.0;
  }
  l66.re = 12.0 * (l7_re * l56.re - l7_im * l56.im);
  l66.im = 12.0 * (l7_re * l56.im + l7_im * l56.re);
  l55.re = -(l102 * l56.re);
  l55.im = -(l102 * l56.im);
  y.re = (((l133_re + l139_re) + l55.re) + l88) + l66.re;
  y.im = (((l38 + l139_im) + l55.im) + l89) + l66.im;
  coder::internal::scalar::b_sqrt(&y);
  l45 = l130_re * l57 - l96 * l137_im;
  l58 = l130_re * l137_im + l96 * l57;
  l130_re = l45 * y.re - l58 * y.im;
  l96 = l45 * y.im + l58 * y.re;
  if (l96 == 0.0) {
    l67.re = l130_re / 6.0;
    l67.im = 0.0;
  } else if (l130_re == 0.0) {
    l67.re = 0.0;
    l67.im = l96 / 6.0;
  } else {
    l67.re = l130_re / 6.0;
    l67.im = l96 / 6.0;
  }
  y.re = (((-l133_re + l139_re) + l55.re) + l88) + l66.re;
  y.im = (((-l38 + l139_im) + l55.im) + l89) + l66.im;
  coder::internal::scalar::b_sqrt(&y);
  l130_re = l45 * y.re - l58 * y.im;
  l96 = l45 * y.im + l58 * y.re;
  if (l96 == 0.0) {
    l55.re = l130_re / 6.0;
    l55.im = 0.0;
  } else if (l130_re == 0.0) {
    l55.re = 0.0;
    l55.im = l96 / 6.0;
  } else {
    l55.re = l130_re / 6.0;
    l55.im = l96 / 6.0;
  }
  t5[0].re = -l144_re - l67.re;
  t5[0].im = -l24 - l67.im;
  t5[1].re = -l144_re + l67.re;
  t5[1].im = -l24 + l67.im;
  t5[2].re = l144_re - l55.re;
  t5[2].im = l24 - l55.im;
  t5[3].re = l144_re + l55.re;
  t5[3].im = l24 + l55.im;
  l58 = std::sqrt(J_max);
  y.re = -(l38_tmp *
           ((A_init * A_init + J_min * V_max * 2.0) + -(J_min * V_init * 2.0)));
  y.im = 0.0;
  coder::internal::scalar::b_sqrt(&y);
  l144_re = rt_powd_snf(l58, 3.0) - J_min * l58;
  if (y.im == 0.0) {
    l55.re = y.re / l144_re;
    l55.im = 0.0;
  } else if (y.re == 0.0) {
    l55.re = 0.0;
    l55.im = y.im / l144_re;
  } else {
    l55.re = y.re / l144_re;
    l55.im = y.im / l144_re;
  }
  t3[0] = l55;
  t3[1] = l55;
  t3[2] = l55;
  t3[3] = l55;
  coder::b_power(t3, l10);
  coder::b_power(t5, l12);
  b_y = rt_powd_snf(J_min, 5.0);
  c_y = rt_powd_snf(J_max, 5.0);
  y.re = l67_tmp * l9_tmp * 2.0 + b_l5_re_tmp;
  b_A_wayp = A_wayp * l8_tmp;
  J_min_tmp = J_min * (l9_tmp * l9_tmp);
  J_max_tmp = J_max * l8_tmp;
  l55.re = P_init * l6_tmp * l9_tmp * 6.0;
  l56.re = c_l5_re_tmp * 6.0;
  l8_tmp = l6_tmp * b_l95_tmp;
  l2 = l2_tmp * b_l95_tmp;
  b_J_max = J_max * l2_tmp * l6_tmp;
  V_init_tmp = d_l95_tmp * l9_tmp;
  l66.re = l5_re_tmp * J_max * l2_tmp * 3.0;
  l67.re = d_l5_re_tmp * 6.0;
  l128.re = e_l5_re_tmp * 6.0;
  e_l5_re_tmp = l5_re_tmp * b_l95_tmp;
  d_l5_re_tmp = J_min_re_tmp * l7_tmp;
  c_l5_re_tmp = l57_tmp * b_l95_tmp;
  l38_tmp = J_max * V_init * l7_tmp;
  l5_re_tmp = A_wayp * l6_tmp * l9_tmp;
  l130_re = l95_tmp - d_l95_tmp * 2.0;
  l85 = J_max * l6_tmp;
  l67_tmp = J_min * l9_tmp;
  re = t3[0].re * t3[0].re - t3[0].im * t3[0].im;
  l58 = t3[0].re * t3[0].im;
  im = l58 + l58;
  l102 = t5[0].re * t5[0].re - t5[0].im * t5[0].im;
  l58 = t5[0].re * t5[0].im;
  b_l5_re_tmp = l58 + l58;
  J_min_re_tmp = J_min * V_wayp * 2.0 + l2_tmp;
  l97 = l4_tmp * J_min;
  l86 = l12_tmp * V_init * 2.0;
  l87 = 1.0 / l6_tmp *
        (((((-J_max * (J_min_re_tmp + l12_tmp * re) + l97) - l7_tmp * l102) +
           b_l95_tmp * re) +
          l86) +
         l85 * l102) *
        -0.5;
  l45 = 1.0 / l6_tmp *
        (((-J_max * (l12_tmp * im) - l7_tmp * b_l5_re_tmp) + b_l95_tmp * im) +
         l85 * b_l5_re_tmp) *
        -0.5;
  l144_re = J_max * t5[0].re;
  l58 = J_max * t5[0].im;
  if (l58 == 0.0) {
    if (l45 == 0.0) {
      l11 = l87 / l144_re;
      l95 = 0.0;
    } else if (l87 == 0.0) {
      l11 = 0.0;
      l95 = l45 / l144_re;
    } else {
      l11 = l87 / l144_re;
      l95 = l45 / l144_re;
    }
  } else if (l144_re == 0.0) {
    if (l87 == 0.0) {
      l11 = l45 / l58;
      l95 = 0.0;
    } else if (l45 == 0.0) {
      l11 = 0.0;
      l95 = -(l87 / l58);
    } else {
      l11 = l45 / l58;
      l95 = -(l87 / l58);
    }
  } else {
    brm = std::abs(l144_re);
    bim = std::abs(l58);
    if (brm > bim) {
      bim = l58 / l144_re;
      l58 = l144_re + bim * l58;
      l11 = (l87 + bim * l45) / l58;
      l95 = (l45 - bim * l87) / l58;
    } else if (bim == brm) {
      if (l144_re > 0.0) {
        bim = 0.5;
      } else {
        bim = -0.5;
      }
      if (l58 > 0.0) {
        l58 = 0.5;
      } else {
        l58 = -0.5;
      }
      l11 = (l87 * bim + l45 * l58) / brm;
      l95 = (l45 * bim - l87 * l58) / brm;
    } else {
      bim = l144_re / l58;
      l58 += bim * l144_re;
      l11 = (bim * l87 + l45) / l58;
      l95 = (bim * l45 - l87) / l58;
    }
  }
  l133_re = J_min_tmp * re;
  l89 = J_min_tmp * im;
  l96 = l8_tmp * re;
  l88 = l8_tmp * im;
  l87 = -(A_init + J_max * t3[0].re);
  l45 = -(J_max * t3[0].im);
  if (l45 == 0.0) {
    t[0].re = l87 / J_min;
    t[0].im = 0.0;
  } else if (l87 == 0.0) {
    t[0].re = 0.0;
    t[0].im = l45 / J_min;
  } else {
    t[0].re = l87 / J_min;
    t[0].im = l45 / J_min;
  }
  t[4].re = 0.0;
  t[4].im = 0.0;
  t[8] = t3[0];
  l139_re = J_max_tmp * l102;
  l57 = J_max_tmp * b_l5_re_tmp;
  l7_im = l85_tmp * re;
  l7_re = l85_tmp * im;
  l24 = l85_tmp * l102;
  l38 = l85_tmp * b_l5_re_tmp;
  l60 = l11 * l11 - l95 * l95;
  l58 = l11 * l95;
  l58 += l58;
  l45 = l85_tmp * t5[0].re;
  l144_re = l85_tmp * t5[0].im;
  l137_im = d_l5_re_tmp * t5[0].re;
  l139_im = d_l5_re_tmp * t5[0].im;
  l87 = (((((((((((((((((((((((((((((((((y.re + b_y * l12[0].re * 2.0) -
                                        c_y * l10[0].re) -
                                       b_A_wayp * l102 * 3.0) +
                                      J_min_tmp * l10[0].re * 3.0) -
                                     J_max_tmp * l12[0].re * 3.0) +
                                    l55.re) -
                                   l56.re) -
                                  l8_tmp * l10[0].re * 2.0) +
                                 l85_tmp * l12[0].re) +
                                l2 * t3[0].re * 3.0) -
                               c_l95_tmp * t3[0].re * 3.0) +
                              b_J_max * t5[0].re * 3.0) -
                             c_l95_tmp * t5[0].re * 3.0) -
                            c_l95_tmp * l11 * 3.0) +
                           (l133_re * t5[0].re - l89 * t5[0].im) * 3.0) +
                          (l133_re * l11 - l89 * l95) * 3.0) -
                         (l139_re * l11 - l57 * l95) * 6.0) +
                        V_init_tmp * t3[0].re * 6.0) +
                       V_init_tmp * t5[0].re * 6.0) +
                      V_init_tmp * l11 * 6.0) -
                     (l96 * t5[0].re - l88 * t5[0].im) * 6.0) +
                    (l7_im * t5[0].re - l7_re * t5[0].im) * 3.0) -
                   (l96 * l11 - l88 * l95) * 3.0) +
                  (l24 * l11 - l38 * l95) * 3.0) +
                 (l45 * l60 - l144_re * l58) * 3.0) -
                l66.re) -
               l67.re) +
              l128.re) +
             e_l5_re_tmp * re * 3.0) +
            d_l5_re_tmp * l102 * 3.0) -
           c_l5_re_tmp * t3[0].re * 6.0) -
          l38_tmp * t5[0].re * 6.0) -
         l5_re_tmp * re * 3.0) +
        (l137_im * l11 - l139_im * l95) * 6.0;
  l45 = (((((((((((((((((((((((((((b_y * l12[0].im * 2.0 - c_y * l10[0].im) -
                                  b_A_wayp * b_l5_re_tmp * 3.0) +
                                 J_min_tmp * l10[0].im * 3.0) -
                                J_max_tmp * l12[0].im * 3.0) -
                               l8_tmp * l10[0].im * 2.0) +
                              l85_tmp * l12[0].im) +
                             l2 * t3[0].im * 3.0) -
                            c_l95_tmp * t3[0].im * 3.0) +
                           b_J_max * t5[0].im * 3.0) -
                          c_l95_tmp * t5[0].im * 3.0) -
                         c_l95_tmp * l95 * 3.0) +
                        (l133_re * t5[0].im + l89 * t5[0].re) * 3.0) +
                       (l133_re * l95 + l89 * l11) * 3.0) -
                      (l139_re * l95 + l57 * l11) * 6.0) +
                     V_init_tmp * t3[0].im * 6.0) +
                    V_init_tmp * t5[0].im * 6.0) +
                   V_init_tmp * l95 * 6.0) -
                  (l96 * t5[0].im + l88 * t5[0].re) * 6.0) +
                 (l7_im * t5[0].im + l7_re * t5[0].re) * 3.0) -
                (l96 * l95 + l88 * l11) * 3.0) +
               (l24 * l95 + l38 * l11) * 3.0) +
              (l45 * l58 + l144_re * l60) * 3.0) +
             e_l5_re_tmp * im * 3.0) +
            d_l5_re_tmp * b_l5_re_tmp * 3.0) -
           c_l5_re_tmp * t3[0].im * 6.0) -
          l38_tmp * t5[0].im * 6.0) -
         l5_re_tmp * im * 3.0) +
        (l137_im * l95 + l139_im * l11) * 6.0;
  l144_re = l9_tmp * ((l130_re + l85 * re) - l67_tmp * re) * 3.0;
  l58 = l9_tmp * (l85 * im - l67_tmp * im) * 3.0;
  if (l58 == 0.0) {
    if (l45 == 0.0) {
      t[12].re = l87 / l144_re;
      t[12].im = 0.0;
    } else if (l87 == 0.0) {
      t[12].re = 0.0;
      t[12].im = l45 / l144_re;
    } else {
      t[12].re = l87 / l144_re;
      t[12].im = l45 / l144_re;
    }
  } else if (l144_re == 0.0) {
    if (l87 == 0.0) {
      t[12].re = l45 / l58;
      t[12].im = 0.0;
    } else if (l45 == 0.0) {
      t[12].re = 0.0;
      t[12].im = -(l87 / l58);
    } else {
      t[12].re = l45 / l58;
      t[12].im = -(l87 / l58);
    }
  } else {
    brm = std::abs(l144_re);
    bim = std::abs(l58);
    if (brm > bim) {
      bim = l58 / l144_re;
      l58 = l144_re + bim * l58;
      t[12].re = (l87 + bim * l45) / l58;
      t[12].im = (l45 - bim * l87) / l58;
    } else if (bim == brm) {
      if (l144_re > 0.0) {
        bim = 0.5;
      } else {
        bim = -0.5;
      }
      if (l58 > 0.0) {
        l58 = 0.5;
      } else {
        l58 = -0.5;
      }
      t[12].re = (l87 * bim + l45 * l58) / brm;
      t[12].im = (l45 * bim - l87 * l58) / brm;
    } else {
      bim = l144_re / l58;
      l58 += bim * l144_re;
      t[12].re = (bim * l87 + l45) / l58;
      t[12].im = (bim * l45 - l87) / l58;
    }
  }
  t[16] = t5[0];
  t[20].re = l11;
  t[20].im = l95;
  l87 = A_wayp - J_min * t5[0].re;
  l45 = 0.0 - J_min * t5[0].im;
  if (l45 == 0.0) {
    t[24].re = l87 / J_max;
    t[24].im = 0.0;
  } else if (l87 == 0.0) {
    t[24].re = 0.0;
    t[24].im = l45 / J_max;
  } else {
    t[24].re = l87 / J_max;
    t[24].im = l45 / J_max;
  }
  re = t3[1].re * t3[1].re - t3[1].im * t3[1].im;
  l58 = t3[1].re * t3[1].im;
  im = l58 + l58;
  l102 = t5[1].re * t5[1].re - t5[1].im * t5[1].im;
  l58 = t5[1].re * t5[1].im;
  b_l5_re_tmp = l58 + l58;
  l87 = 1.0 / l6_tmp *
        (((((-J_max * (J_min_re_tmp + l12_tmp * re) + l97) - l7_tmp * l102) +
           b_l95_tmp * re) +
          l86) +
         l85 * l102) *
        -0.5;
  l45 = 1.0 / l6_tmp *
        (((-J_max * (l12_tmp * im) - l7_tmp * b_l5_re_tmp) + b_l95_tmp * im) +
         l85 * b_l5_re_tmp) *
        -0.5;
  l144_re = J_max * t5[1].re;
  l58 = J_max * t5[1].im;
  if (l58 == 0.0) {
    if (l45 == 0.0) {
      l11 = l87 / l144_re;
      l95 = 0.0;
    } else if (l87 == 0.0) {
      l11 = 0.0;
      l95 = l45 / l144_re;
    } else {
      l11 = l87 / l144_re;
      l95 = l45 / l144_re;
    }
  } else if (l144_re == 0.0) {
    if (l87 == 0.0) {
      l11 = l45 / l58;
      l95 = 0.0;
    } else if (l45 == 0.0) {
      l11 = 0.0;
      l95 = -(l87 / l58);
    } else {
      l11 = l45 / l58;
      l95 = -(l87 / l58);
    }
  } else {
    brm = std::abs(l144_re);
    bim = std::abs(l58);
    if (brm > bim) {
      bim = l58 / l144_re;
      l58 = l144_re + bim * l58;
      l11 = (l87 + bim * l45) / l58;
      l95 = (l45 - bim * l87) / l58;
    } else if (bim == brm) {
      if (l144_re > 0.0) {
        bim = 0.5;
      } else {
        bim = -0.5;
      }
      if (l58 > 0.0) {
        l58 = 0.5;
      } else {
        l58 = -0.5;
      }
      l11 = (l87 * bim + l45 * l58) / brm;
      l95 = (l45 * bim - l87 * l58) / brm;
    } else {
      bim = l144_re / l58;
      l58 += bim * l144_re;
      l11 = (bim * l87 + l45) / l58;
      l95 = (bim * l45 - l87) / l58;
    }
  }
  l133_re = J_min_tmp * re;
  l89 = J_min_tmp * im;
  l96 = l8_tmp * re;
  l88 = l8_tmp * im;
  l87 = -(A_init + J_max * t3[1].re);
  l45 = -(J_max * t3[1].im);
  if (l45 == 0.0) {
    t[1].re = l87 / J_min;
    t[1].im = 0.0;
  } else if (l87 == 0.0) {
    t[1].re = 0.0;
    t[1].im = l45 / J_min;
  } else {
    t[1].re = l87 / J_min;
    t[1].im = l45 / J_min;
  }
  t[5].re = 0.0;
  t[5].im = 0.0;
  t[9] = t3[1];
  l139_re = J_max_tmp * l102;
  l57 = J_max_tmp * b_l5_re_tmp;
  l7_im = l85_tmp * re;
  l7_re = l85_tmp * im;
  l24 = l85_tmp * l102;
  l38 = l85_tmp * b_l5_re_tmp;
  l60 = l11 * l11 - l95 * l95;
  l58 = l11 * l95;
  l58 += l58;
  l45 = l85_tmp * t5[1].re;
  l144_re = l85_tmp * t5[1].im;
  l137_im = d_l5_re_tmp * t5[1].re;
  l139_im = d_l5_re_tmp * t5[1].im;
  l87 = (((((((((((((((((((((((((((((((((y.re + b_y * l12[1].re * 2.0) -
                                        c_y * l10[1].re) -
                                       b_A_wayp * l102 * 3.0) +
                                      J_min_tmp * l10[1].re * 3.0) -
                                     J_max_tmp * l12[1].re * 3.0) +
                                    l55.re) -
                                   l56.re) -
                                  l8_tmp * l10[1].re * 2.0) +
                                 l85_tmp * l12[1].re) +
                                l2 * t3[1].re * 3.0) -
                               c_l95_tmp * t3[1].re * 3.0) +
                              b_J_max * t5[1].re * 3.0) -
                             c_l95_tmp * t5[1].re * 3.0) -
                            c_l95_tmp * l11 * 3.0) +
                           (l133_re * t5[1].re - l89 * t5[1].im) * 3.0) +
                          (l133_re * l11 - l89 * l95) * 3.0) -
                         (l139_re * l11 - l57 * l95) * 6.0) +
                        V_init_tmp * t3[1].re * 6.0) +
                       V_init_tmp * t5[1].re * 6.0) +
                      V_init_tmp * l11 * 6.0) -
                     (l96 * t5[1].re - l88 * t5[1].im) * 6.0) +
                    (l7_im * t5[1].re - l7_re * t5[1].im) * 3.0) -
                   (l96 * l11 - l88 * l95) * 3.0) +
                  (l24 * l11 - l38 * l95) * 3.0) +
                 (l45 * l60 - l144_re * l58) * 3.0) -
                l66.re) -
               l67.re) +
              l128.re) +
             e_l5_re_tmp * re * 3.0) +
            d_l5_re_tmp * l102 * 3.0) -
           c_l5_re_tmp * t3[1].re * 6.0) -
          l38_tmp * t5[1].re * 6.0) -
         l5_re_tmp * re * 3.0) +
        (l137_im * l11 - l139_im * l95) * 6.0;
  l45 = (((((((((((((((((((((((((((b_y * l12[1].im * 2.0 - c_y * l10[1].im) -
                                  b_A_wayp * b_l5_re_tmp * 3.0) +
                                 J_min_tmp * l10[1].im * 3.0) -
                                J_max_tmp * l12[1].im * 3.0) -
                               l8_tmp * l10[1].im * 2.0) +
                              l85_tmp * l12[1].im) +
                             l2 * t3[1].im * 3.0) -
                            c_l95_tmp * t3[1].im * 3.0) +
                           b_J_max * t5[1].im * 3.0) -
                          c_l95_tmp * t5[1].im * 3.0) -
                         c_l95_tmp * l95 * 3.0) +
                        (l133_re * t5[1].im + l89 * t5[1].re) * 3.0) +
                       (l133_re * l95 + l89 * l11) * 3.0) -
                      (l139_re * l95 + l57 * l11) * 6.0) +
                     V_init_tmp * t3[1].im * 6.0) +
                    V_init_tmp * t5[1].im * 6.0) +
                   V_init_tmp * l95 * 6.0) -
                  (l96 * t5[1].im + l88 * t5[1].re) * 6.0) +
                 (l7_im * t5[1].im + l7_re * t5[1].re) * 3.0) -
                (l96 * l95 + l88 * l11) * 3.0) +
               (l24 * l95 + l38 * l11) * 3.0) +
              (l45 * l58 + l144_re * l60) * 3.0) +
             e_l5_re_tmp * im * 3.0) +
            d_l5_re_tmp * b_l5_re_tmp * 3.0) -
           c_l5_re_tmp * t3[1].im * 6.0) -
          l38_tmp * t5[1].im * 6.0) -
         l5_re_tmp * im * 3.0) +
        (l137_im * l95 + l139_im * l11) * 6.0;
  l144_re = l9_tmp * ((l130_re + l85 * re) - l67_tmp * re) * 3.0;
  l58 = l9_tmp * (l85 * im - l67_tmp * im) * 3.0;
  if (l58 == 0.0) {
    if (l45 == 0.0) {
      t[13].re = l87 / l144_re;
      t[13].im = 0.0;
    } else if (l87 == 0.0) {
      t[13].re = 0.0;
      t[13].im = l45 / l144_re;
    } else {
      t[13].re = l87 / l144_re;
      t[13].im = l45 / l144_re;
    }
  } else if (l144_re == 0.0) {
    if (l87 == 0.0) {
      t[13].re = l45 / l58;
      t[13].im = 0.0;
    } else if (l45 == 0.0) {
      t[13].re = 0.0;
      t[13].im = -(l87 / l58);
    } else {
      t[13].re = l45 / l58;
      t[13].im = -(l87 / l58);
    }
  } else {
    brm = std::abs(l144_re);
    bim = std::abs(l58);
    if (brm > bim) {
      bim = l58 / l144_re;
      l58 = l144_re + bim * l58;
      t[13].re = (l87 + bim * l45) / l58;
      t[13].im = (l45 - bim * l87) / l58;
    } else if (bim == brm) {
      if (l144_re > 0.0) {
        bim = 0.5;
      } else {
        bim = -0.5;
      }
      if (l58 > 0.0) {
        l58 = 0.5;
      } else {
        l58 = -0.5;
      }
      t[13].re = (l87 * bim + l45 * l58) / brm;
      t[13].im = (l45 * bim - l87 * l58) / brm;
    } else {
      bim = l144_re / l58;
      l58 += bim * l144_re;
      t[13].re = (bim * l87 + l45) / l58;
      t[13].im = (bim * l45 - l87) / l58;
    }
  }
  t[17] = t5[1];
  t[21].re = l11;
  t[21].im = l95;
  l87 = A_wayp - J_min * t5[1].re;
  l45 = 0.0 - J_min * t5[1].im;
  if (l45 == 0.0) {
    t[25].re = l87 / J_max;
    t[25].im = 0.0;
  } else if (l87 == 0.0) {
    t[25].re = 0.0;
    t[25].im = l45 / J_max;
  } else {
    t[25].re = l87 / J_max;
    t[25].im = l45 / J_max;
  }
  re = t3[2].re * t3[2].re - t3[2].im * t3[2].im;
  l58 = t3[2].re * t3[2].im;
  im = l58 + l58;
  l102 = t5[2].re * t5[2].re - t5[2].im * t5[2].im;
  l58 = t5[2].re * t5[2].im;
  b_l5_re_tmp = l58 + l58;
  l87 = 1.0 / l6_tmp *
        (((((-J_max * (J_min_re_tmp + l12_tmp * re) + l97) - l7_tmp * l102) +
           b_l95_tmp * re) +
          l86) +
         l85 * l102) *
        -0.5;
  l45 = 1.0 / l6_tmp *
        (((-J_max * (l12_tmp * im) - l7_tmp * b_l5_re_tmp) + b_l95_tmp * im) +
         l85 * b_l5_re_tmp) *
        -0.5;
  l144_re = J_max * t5[2].re;
  l58 = J_max * t5[2].im;
  if (l58 == 0.0) {
    if (l45 == 0.0) {
      l11 = l87 / l144_re;
      l95 = 0.0;
    } else if (l87 == 0.0) {
      l11 = 0.0;
      l95 = l45 / l144_re;
    } else {
      l11 = l87 / l144_re;
      l95 = l45 / l144_re;
    }
  } else if (l144_re == 0.0) {
    if (l87 == 0.0) {
      l11 = l45 / l58;
      l95 = 0.0;
    } else if (l45 == 0.0) {
      l11 = 0.0;
      l95 = -(l87 / l58);
    } else {
      l11 = l45 / l58;
      l95 = -(l87 / l58);
    }
  } else {
    brm = std::abs(l144_re);
    bim = std::abs(l58);
    if (brm > bim) {
      bim = l58 / l144_re;
      l58 = l144_re + bim * l58;
      l11 = (l87 + bim * l45) / l58;
      l95 = (l45 - bim * l87) / l58;
    } else if (bim == brm) {
      if (l144_re > 0.0) {
        bim = 0.5;
      } else {
        bim = -0.5;
      }
      if (l58 > 0.0) {
        l58 = 0.5;
      } else {
        l58 = -0.5;
      }
      l11 = (l87 * bim + l45 * l58) / brm;
      l95 = (l45 * bim - l87 * l58) / brm;
    } else {
      bim = l144_re / l58;
      l58 += bim * l144_re;
      l11 = (bim * l87 + l45) / l58;
      l95 = (bim * l45 - l87) / l58;
    }
  }
  l133_re = J_min_tmp * re;
  l89 = J_min_tmp * im;
  l96 = l8_tmp * re;
  l88 = l8_tmp * im;
  l87 = -(A_init + J_max * t3[2].re);
  l45 = -(J_max * t3[2].im);
  if (l45 == 0.0) {
    t[2].re = l87 / J_min;
    t[2].im = 0.0;
  } else if (l87 == 0.0) {
    t[2].re = 0.0;
    t[2].im = l45 / J_min;
  } else {
    t[2].re = l87 / J_min;
    t[2].im = l45 / J_min;
  }
  t[6].re = 0.0;
  t[6].im = 0.0;
  t[10] = t3[2];
  l139_re = J_max_tmp * l102;
  l57 = J_max_tmp * b_l5_re_tmp;
  l7_im = l85_tmp * re;
  l7_re = l85_tmp * im;
  l24 = l85_tmp * l102;
  l38 = l85_tmp * b_l5_re_tmp;
  l60 = l11 * l11 - l95 * l95;
  l58 = l11 * l95;
  l58 += l58;
  l45 = l85_tmp * t5[2].re;
  l144_re = l85_tmp * t5[2].im;
  l137_im = d_l5_re_tmp * t5[2].re;
  l139_im = d_l5_re_tmp * t5[2].im;
  l87 = (((((((((((((((((((((((((((((((((y.re + b_y * l12[2].re * 2.0) -
                                        c_y * l10[2].re) -
                                       b_A_wayp * l102 * 3.0) +
                                      J_min_tmp * l10[2].re * 3.0) -
                                     J_max_tmp * l12[2].re * 3.0) +
                                    l55.re) -
                                   l56.re) -
                                  l8_tmp * l10[2].re * 2.0) +
                                 l85_tmp * l12[2].re) +
                                l2 * t3[2].re * 3.0) -
                               c_l95_tmp * t3[2].re * 3.0) +
                              b_J_max * t5[2].re * 3.0) -
                             c_l95_tmp * t5[2].re * 3.0) -
                            c_l95_tmp * l11 * 3.0) +
                           (l133_re * t5[2].re - l89 * t5[2].im) * 3.0) +
                          (l133_re * l11 - l89 * l95) * 3.0) -
                         (l139_re * l11 - l57 * l95) * 6.0) +
                        V_init_tmp * t3[2].re * 6.0) +
                       V_init_tmp * t5[2].re * 6.0) +
                      V_init_tmp * l11 * 6.0) -
                     (l96 * t5[2].re - l88 * t5[2].im) * 6.0) +
                    (l7_im * t5[2].re - l7_re * t5[2].im) * 3.0) -
                   (l96 * l11 - l88 * l95) * 3.0) +
                  (l24 * l11 - l38 * l95) * 3.0) +
                 (l45 * l60 - l144_re * l58) * 3.0) -
                l66.re) -
               l67.re) +
              l128.re) +
             e_l5_re_tmp * re * 3.0) +
            d_l5_re_tmp * l102 * 3.0) -
           c_l5_re_tmp * t3[2].re * 6.0) -
          l38_tmp * t5[2].re * 6.0) -
         l5_re_tmp * re * 3.0) +
        (l137_im * l11 - l139_im * l95) * 6.0;
  l45 = (((((((((((((((((((((((((((b_y * l12[2].im * 2.0 - c_y * l10[2].im) -
                                  b_A_wayp * b_l5_re_tmp * 3.0) +
                                 J_min_tmp * l10[2].im * 3.0) -
                                J_max_tmp * l12[2].im * 3.0) -
                               l8_tmp * l10[2].im * 2.0) +
                              l85_tmp * l12[2].im) +
                             l2 * t3[2].im * 3.0) -
                            c_l95_tmp * t3[2].im * 3.0) +
                           b_J_max * t5[2].im * 3.0) -
                          c_l95_tmp * t5[2].im * 3.0) -
                         c_l95_tmp * l95 * 3.0) +
                        (l133_re * t5[2].im + l89 * t5[2].re) * 3.0) +
                       (l133_re * l95 + l89 * l11) * 3.0) -
                      (l139_re * l95 + l57 * l11) * 6.0) +
                     V_init_tmp * t3[2].im * 6.0) +
                    V_init_tmp * t5[2].im * 6.0) +
                   V_init_tmp * l95 * 6.0) -
                  (l96 * t5[2].im + l88 * t5[2].re) * 6.0) +
                 (l7_im * t5[2].im + l7_re * t5[2].re) * 3.0) -
                (l96 * l95 + l88 * l11) * 3.0) +
               (l24 * l95 + l38 * l11) * 3.0) +
              (l45 * l58 + l144_re * l60) * 3.0) +
             e_l5_re_tmp * im * 3.0) +
            d_l5_re_tmp * b_l5_re_tmp * 3.0) -
           c_l5_re_tmp * t3[2].im * 6.0) -
          l38_tmp * t5[2].im * 6.0) -
         l5_re_tmp * im * 3.0) +
        (l137_im * l95 + l139_im * l11) * 6.0;
  l144_re = l9_tmp * ((l130_re + l85 * re) - l67_tmp * re) * 3.0;
  l58 = l9_tmp * (l85 * im - l67_tmp * im) * 3.0;
  if (l58 == 0.0) {
    if (l45 == 0.0) {
      t[14].re = l87 / l144_re;
      t[14].im = 0.0;
    } else if (l87 == 0.0) {
      t[14].re = 0.0;
      t[14].im = l45 / l144_re;
    } else {
      t[14].re = l87 / l144_re;
      t[14].im = l45 / l144_re;
    }
  } else if (l144_re == 0.0) {
    if (l87 == 0.0) {
      t[14].re = l45 / l58;
      t[14].im = 0.0;
    } else if (l45 == 0.0) {
      t[14].re = 0.0;
      t[14].im = -(l87 / l58);
    } else {
      t[14].re = l45 / l58;
      t[14].im = -(l87 / l58);
    }
  } else {
    brm = std::abs(l144_re);
    bim = std::abs(l58);
    if (brm > bim) {
      bim = l58 / l144_re;
      l58 = l144_re + bim * l58;
      t[14].re = (l87 + bim * l45) / l58;
      t[14].im = (l45 - bim * l87) / l58;
    } else if (bim == brm) {
      if (l144_re > 0.0) {
        bim = 0.5;
      } else {
        bim = -0.5;
      }
      if (l58 > 0.0) {
        l58 = 0.5;
      } else {
        l58 = -0.5;
      }
      t[14].re = (l87 * bim + l45 * l58) / brm;
      t[14].im = (l45 * bim - l87 * l58) / brm;
    } else {
      bim = l144_re / l58;
      l58 += bim * l144_re;
      t[14].re = (bim * l87 + l45) / l58;
      t[14].im = (bim * l45 - l87) / l58;
    }
  }
  t[18] = t5[2];
  t[22].re = l11;
  t[22].im = l95;
  l87 = A_wayp - J_min * t5[2].re;
  l45 = 0.0 - J_min * t5[2].im;
  if (l45 == 0.0) {
    t[26].re = l87 / J_max;
    t[26].im = 0.0;
  } else if (l87 == 0.0) {
    t[26].re = 0.0;
    t[26].im = l45 / J_max;
  } else {
    t[26].re = l87 / J_max;
    t[26].im = l45 / J_max;
  }
  re = t3[3].re * t3[3].re - t3[3].im * t3[3].im;
  l58 = t3[3].re * t3[3].im;
  im = l58 + l58;
  l102 = t5[3].re * t5[3].re - t5[3].im * t5[3].im;
  l58 = t5[3].re * t5[3].im;
  b_l5_re_tmp = l58 + l58;
  l87 = 1.0 / l6_tmp *
        (((((-J_max * (J_min_re_tmp + l12_tmp * re) + l97) - l7_tmp * l102) +
           b_l95_tmp * re) +
          l86) +
         l85 * l102) *
        -0.5;
  l45 = 1.0 / l6_tmp *
        (((-J_max * (l12_tmp * im) - l7_tmp * b_l5_re_tmp) + b_l95_tmp * im) +
         l85 * b_l5_re_tmp) *
        -0.5;
  l144_re = J_max * t5[3].re;
  l58 = J_max * t5[3].im;
  if (l58 == 0.0) {
    if (l45 == 0.0) {
      l11 = l87 / l144_re;
      l95 = 0.0;
    } else if (l87 == 0.0) {
      l11 = 0.0;
      l95 = l45 / l144_re;
    } else {
      l11 = l87 / l144_re;
      l95 = l45 / l144_re;
    }
  } else if (l144_re == 0.0) {
    if (l87 == 0.0) {
      l11 = l45 / l58;
      l95 = 0.0;
    } else if (l45 == 0.0) {
      l11 = 0.0;
      l95 = -(l87 / l58);
    } else {
      l11 = l45 / l58;
      l95 = -(l87 / l58);
    }
  } else {
    brm = std::abs(l144_re);
    bim = std::abs(l58);
    if (brm > bim) {
      bim = l58 / l144_re;
      l58 = l144_re + bim * l58;
      l11 = (l87 + bim * l45) / l58;
      l95 = (l45 - bim * l87) / l58;
    } else if (bim == brm) {
      if (l144_re > 0.0) {
        bim = 0.5;
      } else {
        bim = -0.5;
      }
      if (l58 > 0.0) {
        l58 = 0.5;
      } else {
        l58 = -0.5;
      }
      l11 = (l87 * bim + l45 * l58) / brm;
      l95 = (l45 * bim - l87 * l58) / brm;
    } else {
      bim = l144_re / l58;
      l58 += bim * l144_re;
      l11 = (bim * l87 + l45) / l58;
      l95 = (bim * l45 - l87) / l58;
    }
  }
  l133_re = J_min_tmp * re;
  l89 = J_min_tmp * im;
  l96 = l8_tmp * re;
  l88 = l8_tmp * im;
  l87 = -(A_init + J_max * t3[3].re);
  l45 = -(J_max * t3[3].im);
  if (l45 == 0.0) {
    t[3].re = l87 / J_min;
    t[3].im = 0.0;
  } else if (l87 == 0.0) {
    t[3].re = 0.0;
    t[3].im = l45 / J_min;
  } else {
    t[3].re = l87 / J_min;
    t[3].im = l45 / J_min;
  }
  t[7].re = 0.0;
  t[7].im = 0.0;
  t[11] = t3[3];
  l139_re = J_max_tmp * l102;
  l57 = J_max_tmp * b_l5_re_tmp;
  l7_im = l85_tmp * re;
  l7_re = l85_tmp * im;
  l24 = l85_tmp * l102;
  l38 = l85_tmp * b_l5_re_tmp;
  l60 = l11 * l11 - l95 * l95;
  l58 = l11 * l95;
  l58 += l58;
  l45 = l85_tmp * t5[3].re;
  l144_re = l85_tmp * t5[3].im;
  l137_im = d_l5_re_tmp * t5[3].re;
  l139_im = d_l5_re_tmp * t5[3].im;
  l87 = (((((((((((((((((((((((((((((((((y.re + b_y * l12[3].re * 2.0) -
                                        c_y * l10[3].re) -
                                       b_A_wayp * l102 * 3.0) +
                                      J_min_tmp * l10[3].re * 3.0) -
                                     J_max_tmp * l12[3].re * 3.0) +
                                    l55.re) -
                                   l56.re) -
                                  l8_tmp * l10[3].re * 2.0) +
                                 l85_tmp * l12[3].re) +
                                l2 * t3[3].re * 3.0) -
                               c_l95_tmp * t3[3].re * 3.0) +
                              b_J_max * t5[3].re * 3.0) -
                             c_l95_tmp * t5[3].re * 3.0) -
                            c_l95_tmp * l11 * 3.0) +
                           (l133_re * t5[3].re - l89 * t5[3].im) * 3.0) +
                          (l133_re * l11 - l89 * l95) * 3.0) -
                         (l139_re * l11 - l57 * l95) * 6.0) +
                        V_init_tmp * t3[3].re * 6.0) +
                       V_init_tmp * t5[3].re * 6.0) +
                      V_init_tmp * l11 * 6.0) -
                     (l96 * t5[3].re - l88 * t5[3].im) * 6.0) +
                    (l7_im * t5[3].re - l7_re * t5[3].im) * 3.0) -
                   (l96 * l11 - l88 * l95) * 3.0) +
                  (l24 * l11 - l38 * l95) * 3.0) +
                 (l45 * l60 - l144_re * l58) * 3.0) -
                l66.re) -
               l67.re) +
              l128.re) +
             e_l5_re_tmp * re * 3.0) +
            d_l5_re_tmp * l102 * 3.0) -
           c_l5_re_tmp * t3[3].re * 6.0) -
          l38_tmp * t5[3].re * 6.0) -
         l5_re_tmp * re * 3.0) +
        (l137_im * l11 - l139_im * l95) * 6.0;
  l45 = (((((((((((((((((((((((((((b_y * l12[3].im * 2.0 - c_y * l10[3].im) -
                                  b_A_wayp * b_l5_re_tmp * 3.0) +
                                 J_min_tmp * l10[3].im * 3.0) -
                                J_max_tmp * l12[3].im * 3.0) -
                               l8_tmp * l10[3].im * 2.0) +
                              l85_tmp * l12[3].im) +
                             l2 * t3[3].im * 3.0) -
                            c_l95_tmp * t3[3].im * 3.0) +
                           b_J_max * t5[3].im * 3.0) -
                          c_l95_tmp * t5[3].im * 3.0) -
                         c_l95_tmp * l95 * 3.0) +
                        (l133_re * t5[3].im + l89 * t5[3].re) * 3.0) +
                       (l133_re * l95 + l89 * l11) * 3.0) -
                      (l139_re * l95 + l57 * l11) * 6.0) +
                     V_init_tmp * t3[3].im * 6.0) +
                    V_init_tmp * t5[3].im * 6.0) +
                   V_init_tmp * l95 * 6.0) -
                  (l96 * t5[3].im + l88 * t5[3].re) * 6.0) +
                 (l7_im * t5[3].im + l7_re * t5[3].re) * 3.0) -
                (l96 * l95 + l88 * l11) * 3.0) +
               (l24 * l95 + l38 * l11) * 3.0) +
              (l45 * l58 + l144_re * l60) * 3.0) +
             e_l5_re_tmp * im * 3.0) +
            d_l5_re_tmp * b_l5_re_tmp * 3.0) -
           c_l5_re_tmp * t3[3].im * 6.0) -
          l38_tmp * t5[3].im * 6.0) -
         l5_re_tmp * im * 3.0) +
        (l137_im * l95 + l139_im * l11) * 6.0;
  l144_re = l9_tmp * ((l130_re + l85 * re) - l67_tmp * re) * 3.0;
  l58 = l9_tmp * (l85 * im - l67_tmp * im) * 3.0;
  if (l58 == 0.0) {
    if (l45 == 0.0) {
      t[15].re = l87 / l144_re;
      t[15].im = 0.0;
    } else if (l87 == 0.0) {
      t[15].re = 0.0;
      t[15].im = l45 / l144_re;
    } else {
      t[15].re = l87 / l144_re;
      t[15].im = l45 / l144_re;
    }
  } else if (l144_re == 0.0) {
    if (l87 == 0.0) {
      t[15].re = l45 / l58;
      t[15].im = 0.0;
    } else if (l45 == 0.0) {
      t[15].re = 0.0;
      t[15].im = -(l87 / l58);
    } else {
      t[15].re = l45 / l58;
      t[15].im = -(l87 / l58);
    }
  } else {
    brm = std::abs(l144_re);
    bim = std::abs(l58);
    if (brm > bim) {
      bim = l58 / l144_re;
      l58 = l144_re + bim * l58;
      t[15].re = (l87 + bim * l45) / l58;
      t[15].im = (l45 - bim * l87) / l58;
    } else if (bim == brm) {
      if (l144_re > 0.0) {
        bim = 0.5;
      } else {
        bim = -0.5;
      }
      if (l58 > 0.0) {
        l58 = 0.5;
      } else {
        l58 = -0.5;
      }
      t[15].re = (l87 * bim + l45 * l58) / brm;
      t[15].im = (l45 * bim - l87 * l58) / brm;
    } else {
      bim = l144_re / l58;
      l58 += bim * l144_re;
      t[15].re = (bim * l87 + l45) / l58;
      t[15].im = (bim * l45 - l87) / l58;
    }
  }
  t[19] = t5[3];
  t[23].re = l11;
  t[23].im = l95;
  l87 = A_wayp - J_min * t5[3].re;
  l45 = 0.0 - J_min * t5[3].im;
  if (l45 == 0.0) {
    t[27].re = l87 / J_max;
    t[27].im = 0.0;
  } else if (l87 == 0.0) {
    t[27].re = 0.0;
    t[27].im = l45 / J_max;
  } else {
    t[27].re = l87 / J_max;
    t[27].im = l45 / J_max;
  }
}

// End of code generation (acdefg_NTA_AVP.cpp)
