//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// acdefg_TV_AVP.cpp
//
// Code generation for function 'acdefg_TV_AVP'
//

// Include files
#include "acdefg_TV_AVP.h"
#include "power.h"
#include "rt_nonfinite.h"
#include "sqrt.h"
#include "topico_wrapper_data.h"
#include "topico_wrapper_rtwutil.h"
#include <cmath>

// Function Definitions
void acdefg_TV_AVP(double P_init, double V_init, double A_init, double P_wayp,
                   double V_wayp, double A_wayp, double A_min, double J_max,
                   double J_min, double T, creal_T t[28])
{
  creal_T l2[4];
  creal_T t3[4];
  creal_T t4_tmp[4];
  creal_T dc;
  creal_T l134;
  creal_T l139;
  creal_T l140;
  double a_tmp;
  double b_l122_tmp;
  double l10;
  double l103;
  double l11;
  double l111;
  double l12;
  double l122_tmp;
  double l14_tmp;
  double l16;
  double l163_im;
  double l17;
  double l19;
  double l20;
  double l2_tmp;
  double l34;
  double l34_tmp;
  double l4;
  double l5;
  double l6;
  double l66;
  double l67;
  double l68;
  double l72;
  double l73;
  double l8;
  double l81;
  double l96_tmp;
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
  //  Generated on 28-Aug-2019 13:55:05
  l2_tmp = A_init * A_init;
  l4 = A_min * A_min;
  l5 = rt_powd_snf(A_min, 3.0);
  l8 = rt_powd_snf(A_wayp, 3.0);
  l9_tmp = J_min * J_min;
  l10 = rt_powd_snf(J_min, 3.0);
  l12 = J_max * J_max;
  l14_tmp = J_min * J_max;
  l6 = l4 * l4;
  l11 = l9_tmp * l9_tmp;
  l16 = J_min * l2_tmp;
  l17 = J_min * l4;
  l19 = J_min * (A_wayp * A_wayp);
  l20 = J_max * l9_tmp;
  l34_tmp = A_min * J_max;
  l34 = l34_tmp * l11 * 12.0;
  l163_im = l10 - l20;
  l66 = 1.0 / (l163_im * l163_im);
  a_tmp = ((((l14_tmp * V_init * 2.0 - l14_tmp * V_wayp * 2.0) - l16) - l17) +
           J_max * l4) +
          l19;
  l67 = l66 * l66;
  l68 = rt_powd_snf(l66, 3.0);
  l72 =
      (A_min * rt_powd_snf(J_min, 5.0) * 8.0 + -l34) + A_min * l10 * l12 * 4.0;
  l73 = l72 * l72;
  l81 = l66 * l72 / 12.0;
  l96_tmp = A_wayp * J_max;
  l4 = (((A_init * A_min * l11 * 12.0 + T * l34) + l17 * l163_im * 12.0) +
        l163_im * a_tmp * 6.0) +
       -(A_min *
         (((((A_wayp * l11 * 6.0 + A_init * J_max * l10 * 6.0) +
             l34_tmp * l10 * 3.0) +
            -(l96_tmp * l10 * 6.0)) +
           T * l10 * l12 * 6.0) +
          -(A_min * l9_tmp * l12 * 3.0)) *
         2.0);
  l163_im = A_min * J_min;
  l103 = l67 * l73 / 24.0 + -(l66 * l4 / 3.0);
  l111 = l68 * rt_powd_snf(l72, 3.0) / 216.0 + -(l67 * l72 * l4 / 18.0);
  l122_tmp = l67 * l67 * (l73 * l73);
  b_l122_tmp = l68 * l73 * l4;
  l73 = l66 * ((((((((l6 * l12 * 4.0 + l6 * l9_tmp * 8.0) +
                     A_wayp * J_min * J_max * l5 * 12.0) +
                    A_min * l8 * l9_tmp * 4.0) +
                   A_min * P_init * l9_tmp * l12 * 24.0) +
                  A_min * T * V_init * l9_tmp * l12 * 24.0) +
                 a_tmp * a_tmp * 3.0) +
                l17 * a_tmp * 12.0) +
               -(A_min *
                 (((((((((((((l14_tmp * l5 * 3.0 +
                              rt_powd_snf(A_init, 3.0) * l9_tmp * 2.0) +
                             l5 * l12 * 3.0) +
                            l8 * l9_tmp * 6.0) +
                           l34_tmp * l19 * 3.0) +
                          l96_tmp * l17 * 6.0) +
                         l163_im * V_init * l12 * 6.0) +
                        -(l34_tmp * l16 * 3.0)) +
                       A_wayp * V_init * l20 * 12.0) +
                      -(l163_im * V_wayp * l12 * 6.0)) +
                     P_wayp * l9_tmp * l12 * 12.0) +
                    T * l2_tmp * l20 * 6.0) +
                   -(A_wayp * V_wayp * l20 * 12.0)) +
                  -(A_wayp * l2_tmp * l9_tmp * 6.0)) *
                 2.0));
  l16 = (-(l122_tmp / 6912.0) + b_l122_tmp / 432.0) + l73 / 3.0;
  l17 = l103 * l103;
  l11 = rt_powd_snf(l103, 3.0);
  l34 = l111 * l111;
  l134.re = ((((l34 * l34 * 27.0 + -(l11 * l34 * 4.0)) +
               -(rt_powd_snf(l16, 3.0) * 256.0)) +
              -(l17 * l17 * l16 * 16.0)) +
             l17 * (l16 * l16) * 128.0) +
            l103 * l34 * l16 * 144.0;
  l134.im = 0.0;
  coder::internal::scalar::b_sqrt(&l134);
  a_tmp = 1.7320508075688772 * l134.re;
  l67 = 1.7320508075688772 * l134.im;
  if (l67 == 0.0) {
    l10 = a_tmp / 18.0;
    l72 = 0.0;
  } else if (a_tmp == 0.0) {
    l10 = 0.0;
    l72 = l67 / 18.0;
  } else {
    l10 = a_tmp / 18.0;
    l72 = l67 / 18.0;
  }
  l68 = l103 * l16;
  l139.re = ((-(l11 / 27.0) + l34 / 2.0) + l68 * 1.3333333333333333) + l10;
  l139.im = l72;
  l140 = coder::power(l139);
  dc = coder::b_power(l139);
  if (dc.im == 0.0) {
    l20 = 1.0 / dc.re;
    l66 = 0.0;
  } else if (dc.re == 0.0) {
    l20 = 0.0;
    l66 = -(1.0 / dc.im);
  } else {
    l10 = std::abs(dc.re);
    l4 = std::abs(dc.im);
    if (l10 > l4) {
      l4 = dc.im / dc.re;
      l163_im = dc.re + l4 * dc.im;
      l20 = (l4 * 0.0 + 1.0) / l163_im;
      l66 = (0.0 - l4) / l163_im;
    } else if (l4 == l10) {
      if (dc.re > 0.0) {
        l4 = 0.5;
      } else {
        l4 = -0.5;
      }
      if (dc.im > 0.0) {
        l163_im = 0.5;
      } else {
        l163_im = -0.5;
      }
      l20 = (l4 + 0.0 * l163_im) / l10;
      l66 = (0.0 * l4 - l163_im) / l10;
    } else {
      l4 = dc.re / dc.im;
      l163_im = dc.im + l4 * dc.re;
      l20 = l4 / l163_im;
      l66 = (l4 * 0.0 - 1.0) / l163_im;
    }
  }
  l19 = l140.re * l140.re - l140.im * l140.im;
  l4 = l140.re * l140.im;
  l12 = l4 + l4;
  dc.re = ((-(l11 * 2.0) + l34 * 27.0) + l68 * 72.0) + a_tmp * 3.0;
  dc.im = l67 * 3.0;
  coder::internal::scalar::b_sqrt(&dc);
  l8 = 3.0 * (2.4494897427831779 * l111 * dc.re);
  l5 = 3.0 * (2.4494897427831779 * l111 * dc.im);
  l96_tmp = l103 * l140.re;
  a_tmp = l103 * l140.im;
  l139.re = ((((-(l122_tmp / 576.0) + b_l122_tmp / 36.0) + l17) + l73 * 4.0) +
             l19 * 9.0) +
            l96_tmp * 6.0;
  l139.im = l12 * 9.0 + a_tmp * 6.0;
  l134 = l139;
  coder::internal::scalar::b_sqrt(&l134);
  dc = coder::c_power(l139);
  if (dc.im == 0.0) {
    l34 = 1.0 / dc.re;
    l72 = 0.0;
  } else if (dc.re == 0.0) {
    l34 = 0.0;
    l72 = -(1.0 / dc.im);
  } else {
    l10 = std::abs(dc.re);
    l4 = std::abs(dc.im);
    if (l10 > l4) {
      l4 = dc.im / dc.re;
      l163_im = dc.re + l4 * dc.im;
      l34 = (l4 * 0.0 + 1.0) / l163_im;
      l72 = (0.0 - l4) / l163_im;
    } else if (l4 == l10) {
      if (dc.re > 0.0) {
        l4 = 0.5;
      } else {
        l4 = -0.5;
      }
      if (dc.im > 0.0) {
        l163_im = 0.5;
      } else {
        l163_im = -0.5;
      }
      l34 = (l4 + 0.0 * l163_im) / l10;
      l72 = (0.0 * l4 - l163_im) / l10;
    } else {
      l4 = dc.re / dc.im;
      l163_im = dc.im + l4 * dc.re;
      l34 = l4 / l163_im;
      l72 = (l4 * 0.0 - 1.0) / l163_im;
    }
  }
  l67 = -9.0 * (l19 * l134.re - l12 * l134.im);
  l68 = -9.0 * (l19 * l134.im + l12 * l134.re);
  l10 = l20 * l134.re - l66 * l134.im;
  l11 = l20 * l134.im + l66 * l134.re;
  if (l11 == 0.0) {
    l73 = l10 / 6.0;
    l10 = 0.0;
  } else if (l10 == 0.0) {
    l73 = 0.0;
    l10 = l11 / 6.0;
  } else {
    l73 = l10 / 6.0;
    l10 = l11 / 6.0;
  }
  l19 = 12.0 * (l96_tmp * l134.re - a_tmp * l134.im);
  l12 = 12.0 * (l96_tmp * l134.im + a_tmp * l134.re);
  l140.re = -(l17 * l134.re);
  l140.im = -(l17 * l134.im);
  l139.re = -(l16 * l134.re * 12.0);
  l139.im = -(l16 * l134.im * 12.0);
  dc.re = (((l8 + l140.re) + l139.re) + l67) + l19;
  dc.im = (((l5 + l140.im) + l139.im) + l68) + l12;
  coder::internal::scalar::b_sqrt(&dc);
  l4 = l20 * l34 - l66 * l72;
  l72 = l20 * l72 + l66 * l34;
  l20 = l4 * dc.re - l72 * dc.im;
  l66 = l4 * dc.im + l72 * dc.re;
  if (l66 == 0.0) {
    l34 = l20 / 6.0;
    l163_im = 0.0;
  } else if (l20 == 0.0) {
    l34 = 0.0;
    l163_im = l66 / 6.0;
  } else {
    l34 = l20 / 6.0;
    l163_im = l66 / 6.0;
  }
  dc.re = (((-l8 + l140.re) + l139.re) + l67) + l19;
  dc.im = (((-l5 + l140.im) + l139.im) + l68) + l12;
  coder::internal::scalar::b_sqrt(&dc);
  l20 = l4 * dc.re - l72 * dc.im;
  l66 = l4 * dc.im + l72 * dc.re;
  if (l66 == 0.0) {
    l139.re = l20 / 6.0;
    l139.im = 0.0;
  } else if (l20 == 0.0) {
    l139.re = 0.0;
    l139.im = l66 / 6.0;
  } else {
    l139.re = l20 / 6.0;
    l139.im = l66 / 6.0;
  }
  l4 = -l81 + -l73;
  t3[0].re = l4 - l34;
  t3[0].im = -l10 - l163_im;
  t3[1].re = l4 + l34;
  t3[1].im = -l10 + l163_im;
  l4 = -l81 + l73;
  t3[2].re = l4 - l139.re;
  t3[2].im = l10 - l139.im;
  t3[3].re = l4 + l139.re;
  t3[3].im = l10 + l139.im;
  l4 = A_min + -A_wayp;
  l11 = l4 * l4;
  l140.re = (J_max * V_init * 2.0 - J_max * V_wayp * 2.0) - A_min * l4 * 2.0;
  l134.re = A_min * A_min * J_max / J_min;
  l163_im = 1.0 / J_max;
  l6 = -(1.0 / J_max * l4);
  l34 = A_min * (1.0 / J_min);
  l139.re = A_min / J_min;
  l10 = t3[0].re * t3[0].re - t3[0].im * t3[0].im;
  l4 = t3[0].re * t3[0].im;
  l72 = l4 + l4;
  l4 = ((((l140.re + l9_tmp * l10) - l2_tmp) + l11) - l14_tmp * l10) + l134.re;
  l10 = l9_tmp * l72 - l14_tmp * l72;
  l72 = l4 * -0.5;
  l4 = l10 * -0.5;
  if (l4 == 0.0) {
    l10 = l72 / l34_tmp;
    l72 = 0.0;
  } else if (l72 == 0.0) {
    l10 = 0.0;
    l72 = l4 / l34_tmp;
  } else {
    l10 = l72 / l34_tmp;
    l72 = l4 / l34_tmp;
  }
  l2[0].re = l10;
  l2[0].im = l72;
  l10 = J_min * t3[0].re;
  l4 = J_min * t3[0].im;
  t4_tmp[0].re = l10;
  t4_tmp[0].im = l4;
  l72 = -(A_init + l10);
  if (-l4 == 0.0) {
    t[0].re = l72 / J_max;
    t[0].im = 0.0;
  } else if (l72 == 0.0) {
    t[0].re = 0.0;
    t[0].im = -l4 / J_max;
  } else {
    t[0].re = l72 / J_max;
    t[0].im = -l4 / J_max;
  }
  t[4].re = 0.0;
  t[4].im = 0.0;
  t[8] = t3[0];
  l10 = t3[1].re * t3[1].re - t3[1].im * t3[1].im;
  l4 = t3[1].re * t3[1].im;
  l72 = l4 + l4;
  l4 = ((((l140.re + l9_tmp * l10) - l2_tmp) + l11) - l14_tmp * l10) + l134.re;
  l10 = l9_tmp * l72 - l14_tmp * l72;
  l72 = l4 * -0.5;
  l4 = l10 * -0.5;
  if (l4 == 0.0) {
    l10 = l72 / l34_tmp;
    l72 = 0.0;
  } else if (l72 == 0.0) {
    l10 = 0.0;
    l72 = l4 / l34_tmp;
  } else {
    l10 = l72 / l34_tmp;
    l72 = l4 / l34_tmp;
  }
  l2[1].re = l10;
  l2[1].im = l72;
  l10 = J_min * t3[1].re;
  l4 = J_min * t3[1].im;
  t4_tmp[1].re = l10;
  t4_tmp[1].im = l4;
  l72 = -(A_init + l10);
  if (-l4 == 0.0) {
    t[1].re = l72 / J_max;
    t[1].im = 0.0;
  } else if (l72 == 0.0) {
    t[1].re = 0.0;
    t[1].im = -l4 / J_max;
  } else {
    t[1].re = l72 / J_max;
    t[1].im = -l4 / J_max;
  }
  t[5].re = 0.0;
  t[5].im = 0.0;
  t[9] = t3[1];
  l10 = t3[2].re * t3[2].re - t3[2].im * t3[2].im;
  l4 = t3[2].re * t3[2].im;
  l72 = l4 + l4;
  l4 = ((((l140.re + l9_tmp * l10) - l2_tmp) + l11) - l14_tmp * l10) + l134.re;
  l10 = l9_tmp * l72 - l14_tmp * l72;
  l72 = l4 * -0.5;
  l4 = l10 * -0.5;
  if (l4 == 0.0) {
    l10 = l72 / l34_tmp;
    l72 = 0.0;
  } else if (l72 == 0.0) {
    l10 = 0.0;
    l72 = l4 / l34_tmp;
  } else {
    l10 = l72 / l34_tmp;
    l72 = l4 / l34_tmp;
  }
  l2[2].re = l10;
  l2[2].im = l72;
  l10 = J_min * t3[2].re;
  l4 = J_min * t3[2].im;
  t4_tmp[2].re = l10;
  t4_tmp[2].im = l4;
  l72 = -(A_init + l10);
  if (-l4 == 0.0) {
    t[2].re = l72 / J_max;
    t[2].im = 0.0;
  } else if (l72 == 0.0) {
    t[2].re = 0.0;
    t[2].im = -l4 / J_max;
  } else {
    t[2].re = l72 / J_max;
    t[2].im = -l4 / J_max;
  }
  t[6].re = 0.0;
  t[6].im = 0.0;
  t[10] = t3[2];
  l10 = t3[3].re * t3[3].re - t3[3].im * t3[3].im;
  l4 = t3[3].re * t3[3].im;
  l72 = l4 + l4;
  l4 = ((((l140.re + l9_tmp * l10) - l2_tmp) + l11) - l14_tmp * l10) + l134.re;
  l10 = l9_tmp * l72 - l14_tmp * l72;
  l72 = l4 * -0.5;
  l4 = l10 * -0.5;
  if (l4 == 0.0) {
    l10 = l72 / l34_tmp;
    l72 = 0.0;
  } else if (l72 == 0.0) {
    l10 = 0.0;
    l72 = l4 / l34_tmp;
  } else {
    l10 = l72 / l34_tmp;
    l72 = l4 / l34_tmp;
  }
  l2[3].re = l10;
  l2[3].im = l72;
  l4 = J_min * t3[3].im;
  a_tmp = A_init + J_min * t3[3].re;
  if (-l4 == 0.0) {
    t[3].re = -a_tmp / J_max;
    t[3].im = 0.0;
  } else if (-a_tmp == 0.0) {
    t[3].re = 0.0;
    t[3].im = -l4 / J_max;
  } else {
    t[3].re = -a_tmp / J_max;
    t[3].im = -l4 / J_max;
  }
  t[7].re = 0.0;
  t[7].im = 0.0;
  t[11] = t3[3];
  l134.re = l163_im * (A_min - A_wayp);
  l140.re = J_max * T;
  t[16].re = l34;
  t[16].im = 0.0;
  t[17].re = l34;
  t[17].im = 0.0;
  t[18].re = l34;
  t[18].im = 0.0;
  t[19].re = l34;
  t[19].im = 0.0;
  t[12].re =
      l163_im * (((A_init + t4_tmp[0].re) -
                  J_max * (((t3[0].re + l2[0].re) - l134.re) + l139.re)) +
                 l140.re);
  t[12].im = l163_im * (t4_tmp[0].im - J_max * (t3[0].im + l2[0].im));
  t[20] = l2[0];
  t[13].re =
      l163_im * (((A_init + t4_tmp[1].re) -
                  J_max * (((t3[1].re + l2[1].re) - l134.re) + l139.re)) +
                 l140.re);
  t[13].im = l163_im * (t4_tmp[1].im - J_max * (t3[1].im + l2[1].im));
  t[21] = l2[1];
  t[14].re =
      l163_im * (((A_init + t4_tmp[2].re) -
                  J_max * (((t3[2].re + l2[2].re) - l134.re) + l139.re)) +
                 l140.re);
  t[14].im = l163_im * (t4_tmp[2].im - J_max * (t3[2].im + l2[2].im));
  t[22] = l2[2];
  t[15].re =
      l163_im *
      ((a_tmp - J_max * (((t3[3].re + l10) - l134.re) + l139.re)) + l140.re);
  t[15].im = l163_im * (l4 - J_max * (t3[3].im + l72));
  t[23] = l2[3];
  t[24].re = l6;
  t[24].im = 0.0;
  t[25].re = l6;
  t[25].im = 0.0;
  t[26].re = l6;
  t[26].im = 0.0;
  t[27].re = l6;
  t[27].im = 0.0;
}

// End of code generation (acdefg_TV_AVP.cpp)
