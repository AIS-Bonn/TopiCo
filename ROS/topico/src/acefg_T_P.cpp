//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// acefg_T_P.cpp
//
// Code generation for function 'acefg_T_P'
//

// Include files
#include "acefg_T_P.h"
#include "roots.h"
#include "rt_nonfinite.h"
#include "topico_wrapper_data.h"
#include "topico_wrapper_rtwutil.h"
#include "topico_wrapper_types.h"
#include <algorithm>
#include <cstring>

// Function Definitions
void acefg_T_P(double P_init, double V_init, double A_init, double P_wayp,
               double V_min, double A_min, double J_max, double J_min, double T,
               creal_T t[42])
{
  creal_T a_data[6];
  creal_T b_tmp_data[6];
  creal_T b_y_data[6];
  creal_T l2_data[6];
  creal_T l5_data[6];
  creal_T t1_data[6];
  creal_T t6_data[6];
  creal_T t7_data[6];
  creal_T tmp_data[6];
  creal_T y_data[6];
  double b_J_min[7];
  double J_min_tmp;
  double J_min_tmp_tmp;
  double ab_J_min_tmp;
  double ac_J_min_tmp;
  double b_J_min_tmp;
  double b_J_min_tmp_tmp;
  double bb_J_min_tmp;
  double c_J_min_tmp;
  double c_J_min_tmp_tmp;
  double cb_J_min_tmp;
  double d;
  double d1;
  double d2;
  double d3;
  double d_J_min_tmp;
  double d_J_min_tmp_tmp;
  double db_J_min_tmp;
  double e_J_min_tmp;
  double e_J_min_tmp_tmp;
  double eb_J_min_tmp;
  double f_J_min_tmp;
  double f_J_min_tmp_tmp;
  double fb_J_min_tmp;
  double g_J_min_tmp;
  double g_J_min_tmp_tmp;
  double gb_J_min_tmp;
  double h_J_min_tmp;
  double h_J_min_tmp_tmp;
  double hb_J_min_tmp;
  double i_J_min_tmp;
  double ib_J_min_tmp;
  double j_J_min_tmp;
  double jb_J_min_tmp;
  double k_J_min_tmp;
  double kb_J_min_tmp;
  double l10;
  double l11;
  double l12;
  double l13_tmp;
  double l14;
  double l15;
  double l16;
  double l17;
  double l18;
  double l19;
  double l20;
  double l21;
  double l22;
  double l2_tmp;
  double l3;
  double l3_re_tmp;
  double l4;
  double l5;
  double l6_tmp;
  double l7;
  double l8;
  double l9;
  double l_J_min_tmp;
  double lb_J_min_tmp;
  double m_J_min_tmp;
  double mb_J_min_tmp;
  double n_J_min_tmp;
  double nb_J_min_tmp;
  double o_J_min_tmp;
  double ob_J_min_tmp;
  double p_J_min_tmp;
  double pb_J_min_tmp;
  double q_J_min_tmp;
  double qb_J_min_tmp;
  double r_J_min_tmp;
  double rb_J_min_tmp;
  double s_J_min_tmp;
  double sb_J_min_tmp;
  double t_J_min_tmp;
  double tb_J_min_tmp;
  double u_J_min_tmp;
  double ub_J_min_tmp;
  double v_J_min_tmp;
  double vb_J_min_tmp;
  double w_J_min_tmp;
  double wb_J_min_tmp;
  double x_J_min_tmp;
  double xb_J_min_tmp;
  double y_J_min_tmp;
  double yb_J_min_tmp;
  int k;
  int t1_size;
  bool b;
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
  //  Generated on 03-Sep-2019 15:34:01
  l2_tmp = A_init * A_init;
  l3 = rt_powd_snf(A_init, 3.0);
  l5 = rt_powd_snf(A_init, 5.0);
  l6_tmp = A_min * A_min;
  l7 = rt_powd_snf(A_min, 3.0);
  l9 = rt_powd_snf(A_min, 5.0);
  l11 = J_min * J_min;
  l12 = rt_powd_snf(J_min, 3.0);
  l13_tmp = J_max * J_max;
  l14 = rt_powd_snf(J_max, 3.0);
  l16 = rt_powd_snf(J_max, 5.0);
  l18 = rt_powd_snf(J_max, 7.0);
  l19 = T * T;
  l20 = V_init * V_init;
  l21 = V_min * V_min;
  l4 = l2_tmp * l2_tmp;
  l8 = l6_tmp * l6_tmp;
  l10 = rt_powd_snf(l6_tmp, 3.0);
  l15 = l13_tmp * l13_tmp;
  l17 = rt_powd_snf(l13_tmp, 3.0);
  l22 = l15 * l15;
  b_J_min[0] = ((J_min * l22 * -3.0 + l11 * l18 * 3.0) - l12 * l17) +
               rt_powd_snf(l14, 3.0);
  J_min_tmp = A_min * J_min;
  b_J_min_tmp = A_init * J_min;
  c_J_min_tmp = A_init * l11;
  d_J_min_tmp = A_init * l12;
  e_J_min_tmp = A_min * l11;
  f_J_min_tmp = A_min * l12;
  b_J_min[1] = ((((((A_init * l22 * 6.0 - A_min * l22 * 6.0) -
                    b_J_min_tmp * l18 * 18.0) +
                   J_min_tmp * l18 * 18.0) +
                  c_J_min_tmp * l17 * 18.0) -
                 d_J_min_tmp * l16 * 6.0) -
                e_J_min_tmp * l17 * 18.0) +
               f_J_min_tmp * l16 * 6.0;
  g_J_min_tmp = A_init * A_min;
  h_J_min_tmp = A_min * T;
  i_J_min_tmp = g_J_min_tmp * J_min;
  J_min_tmp_tmp = l6_tmp * l11;
  j_J_min_tmp = J_min_tmp_tmp * l16;
  b_J_min_tmp_tmp = l6_tmp * l12;
  k_J_min_tmp = b_J_min_tmp_tmp * l15;
  c_J_min_tmp_tmp = J_min * V_init;
  d_J_min_tmp_tmp = J_min * V_min;
  l_J_min_tmp = J_min_tmp * T;
  e_J_min_tmp_tmp = J_min * l2_tmp;
  m_J_min_tmp = V_init * l12;
  n_J_min_tmp = V_min * l12;
  o_J_min_tmp = l2_tmp * l11;
  p_J_min_tmp = g_J_min_tmp * l11;
  q_J_min_tmp = h_J_min_tmp * l12;
  r_J_min_tmp = J_min * l6_tmp;
  b_J_min[2] = (((((((((((((((((((l2_tmp * l18 * 15.0 + l6_tmp * l18 * 15.0) -
                                 g_J_min_tmp * l18 * 30.0) -
                                c_J_min_tmp_tmp * l18 * 6.0) +
                               d_J_min_tmp_tmp * l18 * 6.0) -
                              e_J_min_tmp_tmp * l17 * 42.0) -
                             r_J_min_tmp * l17 * 39.0) +
                            V_init * l11 * l17 * 12.0) -
                           m_J_min_tmp * l16 * 6.0) -
                          V_min * l11 * l17 * 12.0) +
                         n_J_min_tmp * l16 * 6.0) +
                        o_J_min_tmp * l16 * 39.0) -
                       l2_tmp * l12 * l15 * 12.0) +
                      j_J_min_tmp * 33.0) -
                     k_J_min_tmp * 9.0) +
                    i_J_min_tmp * l17 * 84.0) -
                   l_J_min_tmp * l18 * 6.0) -
                  p_J_min_tmp * l16 * 78.0) +
                 g_J_min_tmp * l12 * l15 * 24.0) +
                h_J_min_tmp * l11 * l17 * 12.0) -
               q_J_min_tmp * l16 * 6.0;
  s_J_min_tmp = T * l6_tmp;
  t_J_min_tmp = A_init * V_init;
  u_J_min_tmp = A_init * V_min;
  v_J_min_tmp = A_min * V_init;
  w_J_min_tmp = A_min * V_min;
  x_J_min_tmp = A_init * l6_tmp;
  y_J_min_tmp = A_min * l2_tmp;
  ab_J_min_tmp = g_J_min_tmp * T;
  bb_J_min_tmp = J_min * T;
  cb_J_min_tmp = J_min * l3;
  db_J_min_tmp = l7 * l11 * l15;
  eb_J_min_tmp = l7 * l12 * l14;
  fb_J_min_tmp = b_J_min_tmp * V_init;
  gb_J_min_tmp = J_min_tmp * V_init;
  hb_J_min_tmp = b_J_min_tmp * V_min;
  ib_J_min_tmp = J_min_tmp * V_min;
  jb_J_min_tmp = x_J_min_tmp * l11 * l15;
  kb_J_min_tmp = x_J_min_tmp * l12 * l14;
  b_J_min[3] =
      ((((((((((((((((((((((((((((((((l3 * l17 * 20.0 - l7 * l17 * 20.0) +
                                     x_J_min_tmp * l17 * 60.0) -
                                    y_J_min_tmp * l17 * 60.0) -
                                   cb_J_min_tmp * l16 * 48.0) +
                                  J_min * l7 * l16 * 44.0) +
                                 l3 * l11 * l15 * 36.0) -
                                l3 * l12 * l14 * 8.0) -
                               db_J_min_tmp * 36.0) +
                              eb_J_min_tmp * 12.0) -
                             s_J_min_tmp * l11 * l16 * 48.0) +
                            s_J_min_tmp * l12 * l15 * 24.0) -
                           fb_J_min_tmp * l17 * 24.0) +
                          hb_J_min_tmp * l17 * 24.0) +
                         gb_J_min_tmp * l17 * 24.0) -
                        ib_J_min_tmp * l17 * 24.0) -
                       b_J_min_tmp * l6_tmp * l16 * 132.0) +
                      J_min_tmp * l2_tmp * l16 * 144.0) +
                     t_J_min_tmp * l11 * l16 * 48.0) -
                    t_J_min_tmp * l12 * l15 * 24.0) -
                   u_J_min_tmp * l11 * l16 * 48.0) +
                  u_J_min_tmp * l12 * l15 * 24.0) -
                 v_J_min_tmp * l11 * l16 * 48.0) +
                v_J_min_tmp * l12 * l15 * 24.0) +
               w_J_min_tmp * l11 * l16 * 48.0) -
              w_J_min_tmp * l12 * l15 * 24.0) +
             bb_J_min_tmp * l6_tmp * l17 * 24.0) +
            jb_J_min_tmp * 84.0) -
           kb_J_min_tmp * 12.0) -
          y_J_min_tmp * l11 * l15 * 108.0) +
         y_J_min_tmp * l12 * l14 * 24.0) +
        ab_J_min_tmp * l11 * l16 * 48.0) -
       ab_J_min_tmp * l12 * l15 * 24.0) -
      i_J_min_tmp * T * l17 * 24.0;
  d = l2_tmp * l6_tmp;
  d1 = A_init * l7;
  d2 = A_min * l3;
  d3 = J_min * l4;
  l3_re_tmp = l8 * l11 * l14;
  l22 = l8 * l12 * l13_tmp;
  l18 = l12 * l15;
  s_J_min_tmp = T * l7;
  x_J_min_tmp = V_init * l2_tmp;
  y_J_min_tmp = V_init * l6_tmp;
  l17 = V_min * l2_tmp;
  lb_J_min_tmp = V_min * l6_tmp;
  mb_J_min_tmp = V_init * V_min;
  nb_J_min_tmp = g_J_min_tmp * V_init;
  g_J_min_tmp *= V_min;
  ob_J_min_tmp = h_J_min_tmp * V_init;
  pb_J_min_tmp = h_J_min_tmp * V_min;
  f_J_min_tmp_tmp = A_init * T;
  qb_J_min_tmp = f_J_min_tmp_tmp * l6_tmp;
  rb_J_min_tmp = h_J_min_tmp * l2_tmp;
  sb_J_min_tmp = b_J_min_tmp * T;
  tb_J_min_tmp = c_J_min_tmp_tmp * l2_tmp;
  ub_J_min_tmp = d_J_min_tmp_tmp * l2_tmp;
  vb_J_min_tmp = d1 * l11 * l14;
  wb_J_min_tmp = y_J_min_tmp * l12 * l14;
  xb_J_min_tmp = lb_J_min_tmp * l12 * l14;
  yb_J_min_tmp = d * l11 * l14;
  ac_J_min_tmp = l11 * l16;
    b_J_min[4] = ((((((((((((((((((((((((((((((((((((((((((((((((((((((l4 * l16 * 15.0 + l8 * l16 * 15.0) - d1 * l16 * 60.0) - d2 * l16 * 60.0) - d3 * l15 * 27.0) - J_min * l8 * l15 * 33.0) + d * l16 * 90.0) + l4 * l11 * l14 * 12.0) + l3_re_tmp * 33.0) - l22 * 15.0) + ac_J_min_tmp * l20 * 12.0) - l18 * l20 * 12.0) + ac_J_min_tmp * l21 * 12.0) - l18 * l21 * 12.0) - e_J_min_tmp_tmp * l6_tmp * l15 * 144.0) + s_J_min_tmp * l11 * l15 * 72.0) - s_J_min_tmp * l12 * l14 * 36.0) + x_J_min_tmp * l11 * l15 * 60.0) - x_J_min_tmp * l12 * l14 * 24.0) + y_J_min_tmp * l11 * l15 * 48.0) - wb_J_min_tmp * 12.0) - l17 * l11 * l15 * 60.0) + l17 * l12 * l14 * 24.0) - lb_J_min_tmp * l11 * l15 * 48.0) + xb_J_min_tmp * 12.0) + yb_J_min_tmp * 42.0) + d * l12 * l13_tmp * 12.0) + j_J_min_tmp * l19 * 12.0) - k_J_min_tmp * l19 * 12.0) + b_J_min_tmp * l7 * l15 * 96.0) + J_min_tmp * l3 * l15 * 108.0) - bb_J_min_tmp * l7 * l16 * 36.0) - tb_J_min_tmp * l16 * 36.0) - c_J_min_tmp_tmp * l6_tmp * l16 * 36.0) + ub_J_min_tmp * l16 * 36.0) + d_J_min_tmp_tmp * l6_tmp * l16 * 36.0) - mb_J_min_tmp * l11 * l16 * 24.0) + mb_J_min_tmp * l12 * l15 * 24.0) - vb_J_min_tmp * 36.0) - d2 * l11 * l14 * 48.0) - nb_J_min_tmp * l11 * l15 * 120.0) + nb_J_min_tmp * l12 * l14 * 48.0) + g_J_min_tmp * l11 * l15 * 120.0) - g_J_min_tmp * l12 * l14 * 48.0) + sb_J_min_tmp * l6_tmp * l16 * 72.0) - l_J_min_tmp * l2_tmp * l16 * 36.0) + ob_J_min_tmp * l11 * l16 * 24.0) - ob_J_min_tmp * l12 * l15 * 24.0) - pb_J_min_tmp * l11 * l16 * 24.0) + pb_J_min_tmp * l12 * l15 * 24.0) - qb_J_min_tmp * l11 * l15 * 120.0) + qb_J_min_tmp * l12 * l14 * 48.0) + rb_J_min_tmp * l11 * l15 * 60.0) - rb_J_min_tmp * l12 * l14 * 24.0) + i_J_min_tmp * V_init * l16 * 72.0) - i_J_min_tmp * V_min * l16 * 72.0;
    d = T * l8;
    d1 = l2_tmp * l7;
    d2 = l3 * l6_tmp;
    l18 = A_init * l8;
    l16 = A_init * J_max;
    c_J_min_tmp *= l15;
    d_J_min_tmp *= l14;
    e_J_min_tmp *= l15;
    f_J_min_tmp *= l14;
    g_J_min_tmp = t_J_min_tmp * V_min;
    i_J_min_tmp = v_J_min_tmp * V_min;
    j_J_min_tmp = f_J_min_tmp_tmp * l7;
    k_J_min_tmp = t_J_min_tmp * l6_tmp;
    s_J_min_tmp = u_J_min_tmp * l6_tmp;
    g_J_min_tmp_tmp = T * V_init;
    y_J_min_tmp = g_J_min_tmp_tmp * l6_tmp;
    h_J_min_tmp_tmp = T * V_min;
    lb_J_min_tmp = h_J_min_tmp_tmp * l6_tmp;
    qb_J_min_tmp = ab_J_min_tmp * V_init;
    ab_J_min_tmp *= V_min;
    rb_J_min_tmp = bb_J_min_tmp * l2_tmp;
    ac_J_min_tmp = T * l2_tmp;
    b_J_min[5] = ((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((l5 * l15 * 6.0 - l9 * l15 * 6.0) + l18 * l15 * 30.0) - A_min * l4 * l15 * 30.0) - J_min * l5 * l14 * 6.0) + J_min * l9 * l14 * 18.0) + J_max * l9 * l12 * 6.0) - d1 * l15 * 60.0) + d2 * l15 * 60.0) - l9 * l11 * l13_tmp * 18.0) + e_J_min_tmp_tmp * l7 * l14 * 48.0) - cb_J_min_tmp * l6_tmp * l14 * 48.0) - d * l11 * l14 * 48.0) + d * l12 * l13_tmp * 24.0) + V_init * l3 * l11 * l14 * 24.0) - V_init * l7 * l12 * l13_tmp * 24.0) - V_min * l3 * l11 * l14 * 24.0) + V_min * l7 * l12 * l13_tmp * 24.0) + d1 * l11 * l13_tmp * 12.0) - d2 * l11 * l13_tmp * 12.0) - db_J_min_tmp * l19 * 24.0) + eb_J_min_tmp * l19 * 24.0) - b_J_min_tmp * l8 * l14 * 42.0) - l16 * l8 * l12 * 6.0) + J_min_tmp * l4 * l14 * 30.0) + bb_J_min_tmp * l8 * l15 * 24.0) - c_J_min_tmp_tmp * l3 * l15 * 24.0) + c_J_min_tmp_tmp * l7 * l15 * 24.0) + d_J_min_tmp_tmp * l3 * l15 * 24.0) - d_J_min_tmp_tmp * l7 * l15 * 24.0) + l18 * l11 * l13_tmp * 18.0) + c_J_min_tmp * l20 * 24.0) - d_J_min_tmp * l20 * 24.0) + c_J_min_tmp * l21 * 24.0) - d_J_min_tmp * l21 * 24.0) - e_J_min_tmp * l20 * 24.0) + f_J_min_tmp * l20 * 24.0) - e_J_min_tmp * l21 * 24.0) + f_J_min_tmp * l21 * 24.0) - sb_J_min_tmp * l7 * l15 * 72.0) - l_J_min_tmp * l3 * l15 * 24.0) - fb_J_min_tmp * l6_tmp * l15 * 72.0) + gb_J_min_tmp * l2_tmp * l15 * 72.0) + hb_J_min_tmp * l6_tmp * l15 * 72.0) - ib_J_min_tmp * l2_tmp * l15 * 72.0) - g_J_min_tmp * l11 * l15 * 48.0) + g_J_min_tmp * l12 * l14 * 48.0) + i_J_min_tmp * l11 * l15 * 48.0) - i_J_min_tmp * l12 * l14 * 48.0) + j_J_min_tmp * l11 * l14 * 96.0) - j_J_min_tmp * l12 * l13_tmp * 24.0) + h_J_min_tmp * l3 * l11 * l14 * 24.0) + k_J_min_tmp * l11 * l14 * 48.0) + k_J_min_tmp * l12 * l13_tmp * 24.0) - v_J_min_tmp * l2_tmp * l11 * l14 * 72.0) - s_J_min_tmp * l11 * l14 * 48.0) - s_J_min_tmp * l12 * l13_tmp * 24.0) + w_J_min_tmp * l2_tmp * l11 * l14 * 72.0) + rb_J_min_tmp * l6_tmp * l15 * 72.0) - y_J_min_tmp * l11 * l15 * 48.0) + y_J_min_tmp * l12 * l14 * 48.0) + lb_J_min_tmp * l11 * l15 * 48.0) - lb_J_min_tmp * l12 * l14 * 48.0) + jb_J_min_tmp * l19 * 24.0) - kb_J_min_tmp * l19 * 24.0) - ac_J_min_tmp * l6_tmp * l11 * l14 * 72.0) + qb_J_min_tmp * l11 * l15 * 48.0) - qb_J_min_tmp * l12 * l14 * 48.0) - ab_J_min_tmp * l11 * l15 * 48.0) + ab_J_min_tmp * l12 * l14 * 48.0;
    d = o_J_min_tmp * l14;
    d1 = J_min_tmp_tmp * l14;
    d2 = b_J_min_tmp_tmp * l13_tmp;
    c_J_min_tmp = p_J_min_tmp * l14;
    d_J_min_tmp = q_J_min_tmp * l14;
    e_J_min_tmp = g_J_min_tmp_tmp * l7;
    f_J_min_tmp = h_J_min_tmp_tmp * l7;
    g_J_min_tmp = mb_J_min_tmp * l6_tmp;
    b_J_min[6] = (((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((l10 * l12 + l10 * l14) + rt_powd_snf(l2_tmp, 3.0) * l14) - A_init * l9 * l14 * 6.0) - A_min * l5 * l14 * 6.0) - J_min * l10 * l13_tmp * 5.0) + J_max * l10 * l11 * 3.0) + l2_tmp * l8 * l14 * 15.0) - l3 * l7 * l14 * 20.0) + l4 * l6_tmp * l14 * 15.0) - rt_powd_snf(V_init, 3.0) * l12 * l14 * 8.0) + rt_powd_snf(V_min, 3.0) * l12 * l14 * 8.0) - e_J_min_tmp_tmp * l8 * l13_tmp * 6.0) - cb_J_min_tmp * l7 * l13_tmp * 4.0) + d3 * l6_tmp * l13_tmp * 3.0) + J_max * l2_tmp * l8 * l11 * 3.0) - P_init * l7 * l12 * l13_tmp * 48.0) + P_wayp * l7 * l12 * l13_tmp * 48.0) + T * l9 * l11 * l13_tmp * 12.0) - V_init * l8 * l11 * l13_tmp * 12.0) - m_J_min_tmp * l14 * l21 * 24.0) + V_min * l8 * l11 * l13_tmp * 12.0) + n_J_min_tmp * l14 * l20 * 24.0) + d * l20 * 12.0) + d * l21 * 12.0) + d1 * l20 * 12.0) + d2 * l20 * 12.0) + d1 * l21 * 12.0) + d2 * l21 * 12.0) + l3_re_tmp * l19 * 12.0) - l22 * l19 * 12.0) - rt_powd_snf(T, 3.0) * l7 * l12 * l14 * 8.0) + b_J_min_tmp * l9 * l13_tmp * 12.0) - l16 * l9 * l11 * 6.0) - bb_J_min_tmp * l9 * l14 * 6.0) - J_max * T * l9 * l12 * 6.0) - c_J_min_tmp_tmp * l4 * l14 * 6.0) - c_J_min_tmp_tmp * l8 * l14 * 6.0) - J_max * V_init * l8 * l12 * 6.0) + d_J_min_tmp_tmp * l4 * l14 * 6.0) + d_J_min_tmp_tmp * l8 * l14 * 6.0) + J_max * V_min * l8 * l12 * 6.0) + sb_J_min_tmp * l8 * l14 * 24.0) - l_J_min_tmp * l4 * l14 * 6.0) + fb_J_min_tmp * l7 * l14 * 24.0) + gb_J_min_tmp * l3 * l14 * 24.0) - hb_J_min_tmp * l7 * l14 * 24.0) - ib_J_min_tmp * l3 * l14 * 24.0) - c_J_min_tmp * l20 * 24.0) - c_J_min_tmp * l21 * 24.0) - f_J_min_tmp_tmp * l8 * l11 * l13_tmp * 24.0) - d_J_min_tmp * l20 * 24.0) - d_J_min_tmp * l21 * 24.0) + t_J_min_tmp * l7 * l11 * l13_tmp * 24.0) - u_J_min_tmp * l7 * l11 * l13_tmp * 24.0) - rb_J_min_tmp * l7 * l14 * 36.0) + bb_J_min_tmp * l3 * l6_tmp * l14 * 24.0) - tb_J_min_tmp * l6_tmp * l14 * 36.0) + ub_J_min_tmp * l6_tmp * l14 * 36.0) + e_J_min_tmp * l11 * l14 * 24.0) - e_J_min_tmp * l12 * l13_tmp * 24.0) - f_J_min_tmp * l11 * l14 * 24.0) - f_J_min_tmp * l12 * l13_tmp * 24.0) - mb_J_min_tmp * l2_tmp * l11 * l14 * 24.0) - g_J_min_tmp * l11 * l14 * 24.0) - g_J_min_tmp * l12 * l13_tmp * 24.0) - vb_J_min_tmp * l19 * 24.0) + ac_J_min_tmp * l7 * l11 * l13_tmp * 12.0) - x_J_min_tmp * l6_tmp * l11 * l13_tmp * 12.0) - wb_J_min_tmp * l19 * 24.0) + l17 * l6_tmp * l11 * l13_tmp * 12.0) + xb_J_min_tmp * l19 * 24.0) + yb_J_min_tmp * l19 * 12.0) + nb_J_min_tmp * V_min * l11 * l14 * 48.0) + ob_J_min_tmp * V_min * l12 * l14 * 48.0) - f_J_min_tmp_tmp * V_init * l6_tmp * l11 * l14 * 48.0) + ob_J_min_tmp * l2_tmp * l11 * l14 * 24.0) + f_J_min_tmp_tmp * V_min * l6_tmp * l11 * l14 * 48.0) - pb_J_min_tmp * l2_tmp * l11 * l14 * 24.0;
    coder::b_roots(b_J_min, t1_data, &t1_size);
    for (k = 0; k < t1_size; k++) {
      l2_data[k].re = J_max * t1_data[k].re;
      l2_data[k].im = J_max * t1_data[k].im;
    }
    for (k = 0; k < t1_size; k++) {
      t7_data[k].re = 2.0 * (A_init * l2_data[k].re);
      t7_data[k].im = 2.0 * (A_init * l2_data[k].im);
    }
    for (k = 0; k < t1_size; k++) {
      d = l2_data[k].re;
      d1 = l2_data[k].im;
      b_tmp_data[k].re = d * d - d1 * d1;
      d *= d1;
      b_tmp_data[k].im = d + d;
    }
    for (k = 0; k < t1_size; k++) {
      tmp_data[k].re = 2.0 * (b_J_min_tmp * t1_data[k].re);
      tmp_data[k].im = 2.0 * (b_J_min_tmp * t1_data[k].im);
    }
    for (k = 0; k < t1_size; k++) {
      a_data[k].re = J_min * l2_data[k].re;
      a_data[k].im = J_min * l2_data[k].im;
    }
    for (k = 0; k < t1_size; k++) {
      d = a_data[k].re;
      d1 = a_data[k].im;
      d2 = t1_data[k].im;
      d3 = t1_data[k].re;
      l16 = d * d2 + d1 * d3;
      d = d * d3 - d1 * d2;
      a_data[k].re = d;
      a_data[k].im = l16;
    }
    h_J_min_tmp_tmp = l6_tmp + c_J_min_tmp_tmp * 2.0;
    l22 = l_J_min_tmp * 2.0;
    for (k = 0; k < t1_size; k++) {
      t7_data[k].re =
          J_max *
          ((((((h_J_min_tmp_tmp + t7_data[k].re) + l2_tmp) + b_tmp_data[k].re) +
             l22) +
            tmp_data[k].re) +
           a_data[k].re);
      t7_data[k].im =
          J_max * (((t7_data[k].im + b_tmp_data[k].im) + tmp_data[k].im) +
                   a_data[k].im);
    }
    for (k = 0; k < t1_size; k++) {
      b_tmp_data[k].re = 2.0 * (J_min_tmp * l2_data[k].re);
      b_tmp_data[k].im = 2.0 * (J_min_tmp * l2_data[k].im);
    }
    for (k = 0; k < t1_size; k++) {
      tmp_data[k].re = J_max * (A_init + l2_data[k].re);
      tmp_data[k].im = J_max * l2_data[k].im;
    }
    l3_re_tmp = A_init - A_min;
    for (k = 0; k < t1_size; k++) {
      h_J_min_tmp_tmp = l3_re_tmp + l2_data[k].re;
      l22 = l2_data[k].im;
      d = tmp_data[k].re;
      d1 = tmp_data[k].im;
      l16 = d * l22 + d1 * h_J_min_tmp_tmp;
      d = 2.0 * (d * h_J_min_tmp_tmp - d1 * l22);
      tmp_data[k].re = d;
      tmp_data[k].im = 2.0 * l16;
    }
    g_J_min_tmp_tmp = J_min_tmp * J_max;
    h_J_min_tmp_tmp = r_J_min_tmp + J_max * l6_tmp * 2.0;
    l18 = J_min * J_max;
    l17 = l18 * V_min * 2.0;
    for (k = 0; k < t1_size; k++) {
      l22 = ((((h_J_min_tmp_tmp - t7_data[k].re) + l17) + b_tmp_data[k].re) +
             tmp_data[k].re) *
            -0.5;
      l16 =
          (((0.0 - t7_data[k].im) + b_tmp_data[k].im) + tmp_data[k].im) * -0.5;
      if (l16 == 0.0) {
        t7_data[k].re = l22 / g_J_min_tmp_tmp;
        t7_data[k].im = 0.0;
      } else if (l22 == 0.0) {
        t7_data[k].re = 0.0;
        t7_data[k].im = l16 / g_J_min_tmp_tmp;
      } else {
        t7_data[k].re = l22 / g_J_min_tmp_tmp;
        t7_data[k].im = l16 / g_J_min_tmp_tmp;
      }
    }
    for (k = 0; k < t1_size; k++) {
      l5_data[k].re = (A_init + l2_data[k].re) + -A_min;
      l5_data[k].im = l2_data[k].im;
    }
    for (k = 0; k < t1_size; k++) {
      d = l2_data[k].re;
      d1 = l2_data[k].im;
      t6_data[k].re = d * d - d1 * d1;
      d *= d1;
      t6_data[k].im = d + d;
    }
    for (k = 0; k < t1_size; k++) {
      t6_data[k].re *= J_min;
      t6_data[k].im *= J_min;
    }
    for (k = 0; k < t1_size; k++) {
      d = l5_data[k].re;
      d1 = l5_data[k].im;
      y_data[k].re = d * d - d1 * d1;
      d *= d1;
      y_data[k].im = d + d;
    }
    for (k = 0; k < t1_size; k++) {
      y_data[k].re *= J_max;
      y_data[k].im *= J_max;
    }
    for (k = 0; k < t1_size; k++) {
      a_data[k].re = A_min + J_max * t7_data[k].re;
      a_data[k].im = J_max * t7_data[k].im;
    }
    for (k = 0; k < t1_size; k++) {
      d = a_data[k].re;
      d1 = a_data[k].im;
      b_y_data[k].re = d * d - d1 * d1;
      d *= d1;
      b_y_data[k].im = d + d;
    }
    for (k = 0; k < t1_size; k++) {
      b_y_data[k].re *= J_min;
      b_y_data[k].im *= J_min;
    }
    for (k = 0; k < t1_size; k++) {
      l5_data[k].re *= J_max;
      l5_data[k].im *= J_max;
    }
    for (k = 0; k < t1_size; k++) {
      l22 = A_init + l2_data[k].re;
      l16 = l2_data[k].im;
      d = l5_data[k].re;
      d1 = l5_data[k].im;
      h_J_min_tmp_tmp = d * l16 + d1 * l22;
      d = 2.0 * (d * l22 - d1 * l16);
      l5_data[k].re = d;
      l5_data[k].im = 2.0 * h_J_min_tmp_tmp;
    }
    for (k = 0; k < t1_size; k++) {
      b_tmp_data[k].re = 2.0 * (b_J_min_tmp * l2_data[k].re);
      b_tmp_data[k].im = 2.0 * (b_J_min_tmp * l2_data[k].im);
    }
    for (k = 0; k < t1_size; k++) {
      d = t7_data[k].re;
      d1 = t7_data[k].im;
      a_data[k].re = d * d - d1 * d1;
      d *= d1;
      a_data[k].im = d + d;
    }
    l22 = J_min * l13_tmp;
    for (k = 0; k < t1_size; k++) {
      a_data[k].re *= l22;
      a_data[k].im *= l22;
    }
    for (k = 0; k < t1_size; k++) {
      tmp_data[k].re = 2.0 * (g_J_min_tmp_tmp * t7_data[k].re);
      tmp_data[k].im = 2.0 * (g_J_min_tmp_tmp * t7_data[k].im);
    }
    h_J_min_tmp_tmp = l18 * V_init * 2.0;
    for (k = 0; k < t1_size; k++) {
      l22 = ((((((((t6_data[k].re + y_data[k].re) - b_y_data[k].re) -
                  l5_data[k].re) +
                 h_J_min_tmp_tmp) -
                l17) +
               b_tmp_data[k].re) +
              a_data[k].re) +
             tmp_data[k].re) *
            -0.5;
      l16 = ((((((t6_data[k].im + y_data[k].im) - b_y_data[k].im) -
                l5_data[k].im) +
               b_tmp_data[k].im) +
              a_data[k].im) +
             tmp_data[k].im) *
            -0.5;
      if (l16 == 0.0) {
        t6_data[k].re = l22 / g_J_min_tmp_tmp;
        t6_data[k].im = 0.0;
      } else if (l22 == 0.0) {
        t6_data[k].re = 0.0;
        t6_data[k].im = l16 / g_J_min_tmp_tmp;
      } else {
        t6_data[k].re = l22 / g_J_min_tmp_tmp;
        t6_data[k].im = l16 / g_J_min_tmp_tmp;
      }
    }
    for (k = 0; k < t1_size; k++) {
      l22 = -(l3_re_tmp + l2_data[k].re);
      l16 = -l2_data[k].im;
      if (l16 == 0.0) {
        l22 /= J_min;
        l16 = 0.0;
      } else if (l22 == 0.0) {
        l22 = 0.0;
        l16 /= J_min;
      } else {
        l22 /= J_min;
        l16 /= J_min;
      }
      l2_data[k].re = l22;
      l2_data[k].im = l16;
    }
    b = (t1_size == 6);
    if (!b) {
      h_rtErrorWithMessageID(i_emlrtRTEI.fName, i_emlrtRTEI.lineNo);
    }
    b = ((t1_size == 6) && b);
    if (!b) {
      h_rtErrorWithMessageID(i_emlrtRTEI.fName, i_emlrtRTEI.lineNo);
    }
    b = ((t1_size == 6) && b);
    if (!b) {
      h_rtErrorWithMessageID(i_emlrtRTEI.fName, i_emlrtRTEI.lineNo);
    }
    b = ((t1_size == 6) && b);
    if (!b) {
      h_rtErrorWithMessageID(i_emlrtRTEI.fName, i_emlrtRTEI.lineNo);
    }
    if (0 <= t1_size - 1) {
      std::copy(&t1_data[0], &t1_data[t1_size], &t[0]);
    }
    std::memset(&t[6], 0, 6U * sizeof(creal_T));
    if (0 <= t1_size - 1) {
      std::copy(&l2_data[0], &l2_data[t1_size], &t[12]);
    }
    for (k = 0; k < 6; k++) {
      t[k + 18].re = 0.0;
      t[k + 18].im = 0.0;
      t[k + 24].re = 0.0;
      t[k + 24].im = 0.0;
    }
    if (0 <= t1_size - 1) {
      std::copy(&t6_data[0], &t6_data[t1_size], &t[30]);
      std::copy(&t7_data[0], &t7_data[t1_size], &t[36]);
    }
}

// End of code generation (acefg_T_P.cpp)
