//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// abceg_T_P.cpp
//
// Code generation for function 'abceg_T_P'
//

// Include files
#include "abceg_T_P.h"
#include "roots.h"
#include "rt_nonfinite.h"
#include "topico_wrapper_data.h"
#include "topico_wrapper_rtwutil.h"
#include "topico_wrapper_types.h"
#include <algorithm>
#include <cmath>

// Function Definitions
void abceg_T_P(double P_init, double V_init, double A_init, double P_wayp,
               double V_min, double A_max, double J_max, double J_min, double T,
               creal_T t[42])
{
  creal_T b_tmp_data[6];
  creal_T l2_data[6];
  creal_T l3_data[6];
  creal_T t2_data[6];
  creal_T t3_data[6];
  creal_T t7_data[6];
  creal_T tmp_data[6];
  creal_T y_data[6];
  double b_l9[7];
  double b_l9_tmp;
  double c_l9_tmp;
  double d_l9_tmp;
  double e_l9_tmp;
  double f_l9_tmp;
  double g_l9_tmp;
  double h_l9_tmp;
  double i_l9_tmp;
  double j_l9_tmp;
  double k_l9_tmp;
  double l10;
  double l11_tmp;
  double l12;
  double l13;
  double l14;
  double l15;
  double l2_tmp;
  double l3;
  double l4;
  double l5_tmp;
  double l6;
  double l7;
  double l8;
  double l9;
  double l9_tmp;
  double l9_tmp_tmp;
  double l_l9_tmp;
  double m_l9_tmp;
  double n_l9_tmp;
  double o_l9_tmp;
  double p_l9_tmp;
  double q_l9_tmp;
  double r_l9_tmp;
  double s_l9_tmp;
  int i;
  int i1;
  int k;
  int t3_size;
  signed char z_size_idx_0;
  bool b_p;
  bool exitg1;
  bool p;
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
  //  Generated on 03-Sep-2019 15:31:15
  l2_tmp = A_init * A_init;
  l3 = rt_powd_snf(A_init, 3.0);
  l5_tmp = A_max * A_max;
  l6 = rt_powd_snf(A_max, 3.0);
  l8 = J_min * J_min;
  l9 = rt_powd_snf(J_min, 3.0);
  l11_tmp = J_max * J_max;
  l12 = rt_powd_snf(J_max, 3.0);
  l13 = T * T;
  l14 = V_init * V_init;
  l15 = V_min * V_min;
  l4 = l2_tmp * l2_tmp;
  l7 = l5_tmp * l5_tmp;
  l10 = l8 * l8;
  b_l9[0] = ((-l9 * l12 + l10 * l11_tmp * 3.0) -
             rt_powd_snf(J_min, 5.0) * J_max * 3.0) +
            rt_powd_snf(l8, 3.0);
  b_l9[1] = 0.0;
  l9_tmp = A_init * A_max;
  b_l9_tmp = A_max * T;
  c_l9_tmp = l5_tmp * l8 * l11_tmp;
  d_l9_tmp = J_max * V_init;
  e_l9_tmp = J_max * V_min;
  l9_tmp_tmp = A_max * J_max;
  f_l9_tmp = l9_tmp_tmp * T;
  g_l9_tmp = l9_tmp * J_max;
  b_l9[2] = ((((((((((((((((l2_tmp * l10 * 3.0 - l5_tmp * l10 * 6.0) -
                           l9_tmp * l10 * 6.0) -
                          d_l9_tmp * l10 * 6.0) +
                         e_l9_tmp * l10 * 6.0) -
                        J_max * l2_tmp * l9 * 6.0) +
                       J_max * l5_tmp * l9 * 12.0) -
                      V_init * l8 * l12 * 6.0) +
                     V_init * l9 * l11_tmp * 12.0) +
                    V_min * l8 * l12 * 6.0) -
                   V_min * l9 * l11_tmp * 12.0) +
                  l2_tmp * l8 * l11_tmp * 3.0) -
                 c_l9_tmp * 6.0) +
                g_l9_tmp * l9 * 12.0) -
               f_l9_tmp * l10 * 6.0) -
              l9_tmp * l8 * l11_tmp * 6.0) -
             b_l9_tmp * l8 * l12 * 6.0) +
            b_l9_tmp * l9 * l11_tmp * 12.0;
  b_l9[3] =
      (l6 * l9 * -16.0 + J_max * l6 * l8 * 24.0) - J_min * l6 * l11_tmp * 8.0;
  h_l9_tmp = J_min * J_max;
  i_l9_tmp = J_min * l12;
  j_l9_tmp = l8 * l11_tmp;
  k_l9_tmp = J_min * V_init;
  l_l9_tmp = A_max * J_min;
  m_l9_tmp = l_l9_tmp * T;
  l9 = l9_tmp * J_min;
  l10 = l2_tmp * l5_tmp;
  n_l9_tmp = A_init * J_max;
  o_l9_tmp = d_l9_tmp * l2_tmp;
  p_l9_tmp = e_l9_tmp * l2_tmp;
  q_l9_tmp = V_init * V_min;
  r_l9_tmp = b_l9_tmp * V_init;
  s_l9_tmp = b_l9_tmp * V_min;
  b_l9[4] = ((((((((((((((((((((((((((((((l4 * l8 * 3.0 - l7 * l8 * 12.0) -
                                         h_l9_tmp * l4 * 3.0) +
                                        h_l9_tmp * l7 * 12.0) -
                                       A_max * l3 * l8 * 12.0) -
                                      i_l9_tmp * l14 * 12.0) -
                                     i_l9_tmp * l15 * 12.0) +
                                    l10 * l8 * 12.0) +
                                   j_l9_tmp * l14 * 12.0) +
                                  j_l9_tmp * l15 * 12.0) -
                                 J_min * l5_tmp * l12 * l13 * 12.0) +
                                c_l9_tmp * l13 * 12.0) +
                               l_l9_tmp * J_max * l3 * 12.0) +
                              k_l9_tmp * V_min * l12 * 24.0) -
                             h_l9_tmp * l2_tmp * l5_tmp * 12.0) -
                            o_l9_tmp * l8 * 12.0) +
                           k_l9_tmp * l2_tmp * l11_tmp * 12.0) +
                          p_l9_tmp * l8 * 12.0) -
                         J_min * V_min * l2_tmp * l11_tmp * 12.0) -
                        q_l9_tmp * l8 * l11_tmp * 24.0) -
                       m_l9_tmp * V_init * l12 * 24.0) +
                      m_l9_tmp * V_min * l12 * 24.0) +
                     n_l9_tmp * T * l5_tmp * l8 * 24.0) -
                    A_init * J_min * T * l5_tmp * l11_tmp * 24.0) -
                   f_l9_tmp * l2_tmp * l8 * 12.0) +
                  m_l9_tmp * l2_tmp * l11_tmp * 12.0) +
                 r_l9_tmp * l8 * l11_tmp * 24.0) -
                s_l9_tmp * l8 * l11_tmp * 24.0) +
               g_l9_tmp * V_init * l8 * 24.0) -
              l9 * V_init * l11_tmp * 24.0) -
             g_l9_tmp * V_min * l8 * 24.0) +
            l9 * V_min * l11_tmp * 24.0;
  b_l9[5] = 0.0;
  c_l9_tmp = l2_tmp * l11_tmp;
  g_l9_tmp = l5_tmp * l11_tmp;
  h_l9_tmp = l9_tmp * l11_tmp;
  b_l9_tmp *= l12;
  i_l9_tmp = J_max * T;
  j_l9_tmp = A_init * T;
  b_l9[6] =
      ((((((((((((((((((((((((((((((((((((((((((((rt_powd_snf(A_init, 5.0) *
                                                      A_max * -6.0 +
                                                  l2_tmp * l7 * 12.0) -
                                                 l3 * l6 * 24.0) +
                                                l4 * l5_tmp * 18.0) -
                                               rt_powd_snf(V_init, 3.0) * l12 *
                                                   8.0) +
                                              rt_powd_snf(V_min, 3.0) * l12 *
                                                  8.0) +
                                             rt_powd_snf(l2_tmp, 3.0)) -
                                            d_l9_tmp * l4 * 6.0) -
                                           d_l9_tmp * l7 * 24.0) +
                                          e_l9_tmp * l4 * 6.0) +
                                         e_l9_tmp * l7 * 24.0) -
                                        P_init * l6 * l11_tmp * 48.0) +
                                       P_wayp * l6 * l11_tmp * 48.0) -
                                      V_init * l12 * l15 * 24.0) +
                                     V_min * l12 * l14 * 24.0) +
                                    c_l9_tmp * l14 * 12.0) +
                                   c_l9_tmp * l15 * 12.0) +
                                  g_l9_tmp * l14 * 24.0) +
                                 g_l9_tmp * l15 * 24.0) -
                                rt_powd_snf(T, 3.0) * l6 * l12 * 8.0) -
                               V_init * l5_tmp * l12 * l13 * 24.0) +
                              V_min * l5_tmp * l12 * l13 * 24.0) +
                             l10 * l11_tmp * l13 * 12.0) -
                            f_l9_tmp * l4 * 6.0) +
                           n_l9_tmp * V_init * l6 * 48.0) +
                          l9_tmp_tmp * V_init * l3 * 24.0) -
                         n_l9_tmp * V_min * l6 * 48.0) -
                        l9_tmp_tmp * V_min * l3 * 24.0) -
                       h_l9_tmp * l14 * 24.0) -
                      h_l9_tmp * l15 * 24.0) -
                     b_l9_tmp * l14 * 24.0) -
                    b_l9_tmp * l15 * 24.0) -
                   i_l9_tmp * l2_tmp * l6 * 24.0) +
                  i_l9_tmp * l3 * l5_tmp * 24.0) -
                 o_l9_tmp * l5_tmp * 48.0) +
                p_l9_tmp * l5_tmp * 48.0) -
               T * V_min * l6 * l11_tmp * 48.0) -
              q_l9_tmp * l2_tmp * l11_tmp * 24.0) -
             q_l9_tmp * l5_tmp * l11_tmp * 48.0) -
            A_init * l6 * l11_tmp * l13 * 24.0) +
           l9_tmp * V_init * V_min * l11_tmp * 48.0) +
          r_l9_tmp * V_min * l12 * 48.0) -
         j_l9_tmp * V_init * l5_tmp * l11_tmp * 48.0) +
        r_l9_tmp * l2_tmp * l11_tmp * 24.0) +
       j_l9_tmp * V_min * l5_tmp * l11_tmp * 48.0) -
      s_l9_tmp * l2_tmp * l11_tmp * 24.0;
  coder::b_roots(b_l9, t3_data, &t3_size);
  for (i = 0; i < t3_size; i++) {
    l2_data[i].re = J_min * t3_data[i].re;
    l2_data[i].im = J_min * t3_data[i].im;
  }
  for (i = 0; i < t3_size; i++) {
    l3_data[i].re = A_max + l2_data[i].re;
    l3_data[i].im = l2_data[i].im;
  }
  for (i = 0; i < t3_size; i++) {
    tmp_data[i].re = J_max * l2_data[i].re;
    tmp_data[i].im = J_max * l2_data[i].im;
  }
  for (k = 0; k < t3_size; k++) {
    j_l9_tmp = l3_data[k].re;
    k_l9_tmp = l3_data[k].im;
    t7_data[k].re = j_l9_tmp * j_l9_tmp - k_l9_tmp * k_l9_tmp;
    j_l9_tmp *= k_l9_tmp;
    t7_data[k].im = j_l9_tmp + j_l9_tmp;
  }
  for (i = 0; i < t3_size; i++) {
    j_l9_tmp = tmp_data[i].re;
    k_l9_tmp = tmp_data[i].im;
    l9 = t3_data[i].re;
    h_l9_tmp = t3_data[i].im;
    b_tmp_data[i].re = j_l9_tmp * l9 - k_l9_tmp * h_l9_tmp;
    b_tmp_data[i].im = j_l9_tmp * h_l9_tmp + k_l9_tmp * l9;
  }
  for (i = 0; i < t3_size; i++) {
    y_data[i].re = 2.0 * tmp_data[i].re;
    y_data[i].im = 2.0 * tmp_data[i].im;
  }
  for (i = 0; i < t3_size; i++) {
    l3_data[i].re = 2.0 * (J_max * l3_data[i].re);
    l3_data[i].im = 2.0 * (J_max * l3_data[i].im);
  }
  n_l9_tmp = d_l9_tmp * 2.0 - e_l9_tmp * 2.0;
  l10 = (n_l9_tmp + l2_tmp) - l5_tmp;
  l9 = A_init * (A_init - A_max) * 2.0;
  h_l9_tmp = f_l9_tmp * 2.0;
  for (i = 0; i < t3_size; i++) {
    t7_data[i].re =
        -((((l10 - t7_data[i].re) - l9) + h_l9_tmp) + b_tmp_data[i].re);
    t7_data[i].im = -((0.0 - t7_data[i].im) + b_tmp_data[i].im);
  }
  for (i = 0; i < t3_size; i++) {
    y_data[i].re -= l3_data[i].re;
    y_data[i].im -= l3_data[i].im;
  }
  z_size_idx_0 = static_cast<signed char>(t3_size);
  p = true;
  b_p = true;
  k = 0;
  exitg1 = false;
  while ((!exitg1) && (k < 2)) {
    if (k + 1 <= 1) {
      i = z_size_idx_0;
      i1 = t3_size;
    } else {
      i = 1;
      i1 = 1;
    }
    if (i != i1) {
      b_p = false;
      exitg1 = true;
    } else {
      k++;
    }
  }
  if (b_p) {
    b_p = true;
    k = 0;
    exitg1 = false;
    while ((!exitg1) && (k < 2)) {
      if (k + 1 <= 1) {
        i = z_size_idx_0;
        i1 = t3_size;
      } else {
        i = 1;
        i1 = 1;
      }
      if (i != i1) {
        b_p = false;
        exitg1 = true;
      } else {
        k++;
      }
    }
    if (!b_p) {
      p = false;
    }
  } else {
    p = false;
  }
  if (!p) {
    m_rtErrorWithMessageID(p_emlrtRTEI.fName, p_emlrtRTEI.lineNo);
  }
  z_size_idx_0 = static_cast<signed char>(t3_size);
  p = true;
  b_p = true;
  k = 0;
  exitg1 = false;
  while ((!exitg1) && (k < 2)) {
    if (k + 1 <= 1) {
      i = z_size_idx_0;
      i1 = t3_size;
    } else {
      i = 1;
      i1 = 1;
    }
    if (i != i1) {
      b_p = false;
      exitg1 = true;
    } else {
      k++;
    }
  }
  if (b_p) {
    b_p = true;
    k = 0;
    exitg1 = false;
    while ((!exitg1) && (k < 2)) {
      if (k + 1 <= 1) {
        i = z_size_idx_0;
        i1 = t3_size;
      } else {
        i = 1;
        i1 = 1;
      }
      if (i != i1) {
        b_p = false;
        exitg1 = true;
      } else {
        k++;
      }
    }
    if (!b_p) {
      p = false;
    }
  } else {
    p = false;
  }
  if (!p) {
    m_rtErrorWithMessageID(p_emlrtRTEI.fName, p_emlrtRTEI.lineNo);
  }
  for (i = 0; i < t3_size; i++) {
    l_l9_tmp = t7_data[i].re;
    m_l9_tmp = t7_data[i].im;
    h_l9_tmp = y_data[i].re;
    i_l9_tmp = y_data[i].im;
    if (i_l9_tmp == 0.0) {
      if (m_l9_tmp == 0.0) {
        k_l9_tmp = l_l9_tmp / h_l9_tmp;
        l10 = 0.0;
      } else if (l_l9_tmp == 0.0) {
        k_l9_tmp = 0.0;
        l10 = m_l9_tmp / h_l9_tmp;
      } else {
        k_l9_tmp = l_l9_tmp / h_l9_tmp;
        l10 = m_l9_tmp / h_l9_tmp;
      }
    } else if (h_l9_tmp == 0.0) {
      if (l_l9_tmp == 0.0) {
        k_l9_tmp = m_l9_tmp / i_l9_tmp;
        l10 = 0.0;
      } else if (m_l9_tmp == 0.0) {
        k_l9_tmp = 0.0;
        l10 = -(l_l9_tmp / i_l9_tmp);
      } else {
        k_l9_tmp = m_l9_tmp / i_l9_tmp;
        l10 = -(l_l9_tmp / i_l9_tmp);
      }
    } else {
      j_l9_tmp = std::abs(h_l9_tmp);
      l9 = std::abs(i_l9_tmp);
      if (j_l9_tmp > l9) {
        l10 = i_l9_tmp / h_l9_tmp;
        l9 = h_l9_tmp + l10 * i_l9_tmp;
        k_l9_tmp = (l_l9_tmp + l10 * m_l9_tmp) / l9;
        l10 = (m_l9_tmp - l10 * l_l9_tmp) / l9;
      } else if (l9 == j_l9_tmp) {
        if (h_l9_tmp > 0.0) {
          l10 = 0.5;
        } else {
          l10 = -0.5;
        }
        if (i_l9_tmp > 0.0) {
          l9 = 0.5;
        } else {
          l9 = -0.5;
        }
        k_l9_tmp = (l_l9_tmp * l10 + m_l9_tmp * l9) / j_l9_tmp;
        l10 = (m_l9_tmp * l10 - l_l9_tmp * l9) / j_l9_tmp;
      } else {
        l10 = h_l9_tmp / i_l9_tmp;
        l9 = i_l9_tmp + l10 * h_l9_tmp;
        k_l9_tmp = (l10 * l_l9_tmp + m_l9_tmp) / l9;
        l10 = (l10 * m_l9_tmp - l_l9_tmp) / l9;
      }
    }
    t7_data[i].re = k_l9_tmp;
    t7_data[i].im = l10;
  }
  i_l9_tmp = A_init + -A_max;
  for (i = 0; i < t3_size; i++) {
    b_tmp_data[i].re = J_max * t7_data[i].re;
    b_tmp_data[i].im = J_max * t7_data[i].im;
  }
  for (i = 0; i < t3_size; i++) {
    l3_data[i].re = (A_max + l2_data[i].re) + b_tmp_data[i].re;
    l3_data[i].im = l2_data[i].im + b_tmp_data[i].im;
  }
  for (k = 0; k < t3_size; k++) {
    j_l9_tmp = l3_data[k].re;
    k_l9_tmp = l3_data[k].im;
    t2_data[k].re = j_l9_tmp * j_l9_tmp - k_l9_tmp * k_l9_tmp;
    j_l9_tmp *= k_l9_tmp;
    t2_data[k].im = j_l9_tmp + j_l9_tmp;
  }
  for (k = 0; k < t3_size; k++) {
    j_l9_tmp = t7_data[k].re;
    k_l9_tmp = t7_data[k].im;
    y_data[k].re = j_l9_tmp * j_l9_tmp - k_l9_tmp * k_l9_tmp;
    j_l9_tmp *= k_l9_tmp;
    y_data[k].im = j_l9_tmp + j_l9_tmp;
  }
  for (i = 0; i < t3_size; i++) {
    y_data[i].re *= l11_tmp;
    y_data[i].im *= l11_tmp;
  }
  for (i = 0; i < t3_size; i++) {
    h_l9_tmp = A_max + l2_data[i].re;
    l9 = l2_data[i].im;
    j_l9_tmp = b_tmp_data[i].re;
    k_l9_tmp = b_tmp_data[i].im;
    l10 = j_l9_tmp * l9 + k_l9_tmp * h_l9_tmp;
    j_l9_tmp = 2.0 * (j_l9_tmp * h_l9_tmp - k_l9_tmp * l9);
    b_tmp_data[i].re = j_l9_tmp;
    b_tmp_data[i].im = 2.0 * l10;
  }
  for (i = 0; i < t3_size; i++) {
    l3_data[i].re = 2.0 * (l9_tmp_tmp * t3_data[i].re);
    l3_data[i].im = 2.0 * (l9_tmp_tmp * t3_data[i].im);
  }
  for (i = 0; i < t3_size; i++) {
    j_l9_tmp = tmp_data[i].re;
    k_l9_tmp = tmp_data[i].im;
    l9 = t3_data[i].im;
    h_l9_tmp = t3_data[i].re;
    l10 = j_l9_tmp * l9 + k_l9_tmp * h_l9_tmp;
    j_l9_tmp = j_l9_tmp * h_l9_tmp - k_l9_tmp * l9;
    tmp_data[i].re = j_l9_tmp;
    tmp_data[i].im = l10;
  }
  l9 = i_l9_tmp * i_l9_tmp;
  l10 = n_l9_tmp - A_init * i_l9_tmp * 2.0;
  for (i = 0; i < t3_size; i++) {
    l_l9_tmp =
        ((((((l10 - t2_data[i].re) + l9) + y_data[i].re) + b_tmp_data[i].re) +
          l3_data[i].re) +
         tmp_data[i].re) *
        -0.5;
    m_l9_tmp = (((((0.0 - t2_data[i].im) + y_data[i].im) + b_tmp_data[i].im) +
                 l3_data[i].im) +
                tmp_data[i].im) *
               -0.5;
    if (m_l9_tmp == 0.0) {
      t2_data[i].re = l_l9_tmp / l9_tmp_tmp;
      t2_data[i].im = 0.0;
    } else if (l_l9_tmp == 0.0) {
      t2_data[i].re = 0.0;
      t2_data[i].im = m_l9_tmp / l9_tmp_tmp;
    } else {
      t2_data[i].re = l_l9_tmp / l9_tmp_tmp;
      t2_data[i].im = m_l9_tmp / l9_tmp_tmp;
    }
  }
  l9 = 1.0 / J_max * i_l9_tmp;
  p = (t3_size == 6);
  if (!p) {
    h_rtErrorWithMessageID(i_emlrtRTEI.fName, i_emlrtRTEI.lineNo);
  }
  p = ((t3_size == 6) && p);
  if (!p) {
    h_rtErrorWithMessageID(i_emlrtRTEI.fName, i_emlrtRTEI.lineNo);
  }
  p = ((t3_size == 6) && p);
  if (!p) {
    h_rtErrorWithMessageID(i_emlrtRTEI.fName, i_emlrtRTEI.lineNo);
  }
  t[0].re = -l9;
  t[0].im = 0.0;
  t[1].re = -l9;
  t[1].im = 0.0;
  t[2].re = -l9;
  t[2].im = 0.0;
  t[3].re = -l9;
  t[3].im = 0.0;
  t[4].re = -l9;
  t[4].im = 0.0;
  t[5].re = -l9;
  t[5].im = 0.0;
  if (0 <= t3_size - 1) {
    std::copy(&t2_data[0], &t2_data[t3_size], &t[6]);
    std::copy(&t3_data[0], &t3_data[t3_size], &t[12]);
  }
  for (i = 0; i < 6; i++) {
    t[i + 18].re = 0.0;
    t[i + 18].im = 0.0;
    t[i + 24].re = 0.0;
    t[i + 24].im = 0.0;
    t[i + 30].re = 0.0;
    t[i + 30].im = 0.0;
  }
  if (0 <= t3_size - 1) {
    std::copy(&t7_data[0], &t7_data[t3_size], &t[36]);
  }
}

// End of code generation (abceg_T_P.cpp)
