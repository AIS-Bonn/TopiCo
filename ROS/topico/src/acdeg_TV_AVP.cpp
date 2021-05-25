//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// acdeg_TV_AVP.cpp
//
// Code generation for function 'acdeg_TV_AVP'
//

// Include files
#include "acdeg_TV_AVP.h"
#include "roots.h"
#include "rt_nonfinite.h"
#include "topico_wrapper_data.h"
#include "topico_wrapper_rtwutil.h"
#include "topico_wrapper_types.h"
#include <algorithm>
#include <cmath>
#include <cstring>

// Function Definitions
void acdeg_TV_AVP(double P_init, double V_init, double A_init, double P_wayp,
                  double V_wayp, double A_wayp, double J_max, double J_min,
                  double T, creal_T t[35])
{
  creal_T b_z_data[6];
  creal_T x_data[6];
  creal_T y_data[6];
  creal_T z_data[6];
  creal_T b_tmp_data[5];
  creal_T c_tmp_data[5];
  creal_T d_tmp_data[5];
  creal_T e_tmp_data[5];
  creal_T f_tmp_data[5];
  creal_T t4_data[5];
  creal_T tmp_data[5];
  double l8[6];
  double ab_l8_tmp;
  double ac_l8_tmp;
  double b_A_init;
  double b_A_wayp;
  double b_l8_tmp;
  double b_l8_tmp_tmp;
  double bb_l8_tmp;
  double bc_l8_tmp;
  double c_l8_tmp;
  double c_l8_tmp_tmp;
  double cb_l8_tmp;
  double cc_l8_tmp;
  double d;
  double d1;
  double d10;
  double d11;
  double d12;
  double d13;
  double d14;
  double d15;
  double d16;
  double d17;
  double d18;
  double d19;
  double d2;
  double d20;
  double d21;
  double d22;
  double d23;
  double d3;
  double d4;
  double d5;
  double d6;
  double d7;
  double d8;
  double d9;
  double d_l8_tmp;
  double d_l8_tmp_tmp;
  double db_l8_tmp;
  double dc_l8_tmp;
  double e_l8_tmp;
  double e_l8_tmp_tmp;
  double eb_l8_tmp;
  double ec_l8_tmp;
  double f_l8_tmp;
  double f_l8_tmp_tmp;
  double fb_l8_tmp;
  double fc_l8_tmp;
  double g_l8_tmp;
  double g_l8_tmp_tmp;
  double gb_l8_tmp;
  double gc_l8_tmp;
  double h_l8_tmp;
  double h_l8_tmp_tmp;
  double hb_l8_tmp;
  double hc_l8_tmp;
  double i_l8_tmp;
  double ib_l8_tmp;
  double ic_l8_tmp;
  double j_l8_tmp;
  double jb_l8_tmp;
  double jc_l8_tmp;
  double k_l8_tmp;
  double kb_l8_tmp;
  double kc_l8_tmp;
  double l10_tmp;
  double l11;
  double l12;
  double l13_tmp;
  double l14;
  double l15;
  double l16;
  double l17;
  double l18;
  double l2;
  double l3;
  double l4;
  double l5;
  double l6;
  double l7;
  double l8_tmp;
  double l8_tmp_tmp;
  double l9;
  double l_l8_tmp;
  double lb_l8_tmp;
  double lc_l8_tmp;
  double m_l8_tmp;
  double mb_l8_tmp;
  double mc_l8_tmp;
  double n_l8_tmp;
  double nb_l8_tmp;
  double nc_l8_tmp;
  double o_l8_tmp;
  double ob_l8_tmp;
  double oc_l8_tmp;
  double p_l8_tmp;
  double pb_l8_tmp;
  double pc_l8_tmp;
  double q_l8_tmp;
  double qb_l8_tmp;
  double qc_l8_tmp;
  double r_l8_tmp;
  double rb_l8_tmp;
  double rc_l8_tmp;
  double s_l8_tmp;
  double sb_l8_tmp;
  double sc_l8_tmp;
  double t_l8_tmp;
  double tb_l8_tmp;
  double tc_l8_tmp;
  double u_l8_tmp;
  double ub_l8_tmp;
  double uc_l8_tmp;
  double v_l8_tmp;
  double vb_l8_tmp;
  double vc_l8_tmp;
  double w_l8_tmp;
  double wb_l8_tmp;
  double wc_l8_tmp;
  double x_l8_tmp;
  double xb_l8_tmp;
  double y_l8_tmp;
  double yb_l8_tmp;
  int i;
  int i1;
  int k;
  int t4_size;
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
  //  Generated on 29-Aug-2019 10:09:55
  l2 = A_init * A_init;
  l3 = rt_powd_snf(A_init, 3.0);
  l5 = A_wayp * A_wayp;
  l6 = rt_powd_snf(A_wayp, 3.0);
  l8_tmp = J_min * J_min;
  l9 = rt_powd_snf(J_min, 3.0);
  l10_tmp = J_max * J_max;
  l11 = rt_powd_snf(J_max, 3.0);
  l13_tmp = T * T;
  l14 = rt_powd_snf(T, 3.0);
  l16 = rt_powd_snf(T, 5.0);
  l17 = V_init * V_init;
  l18 = V_wayp * V_wayp;
  l4 = l2 * l2;
  l7 = l5 * l5;
  l12 = l10_tmp * l10_tmp;
  l15 = l13_tmp * l13_tmp;
  b_l8_tmp = l9 * l11;
  l8[0] = -l8_tmp * l12 + b_l8_tmp * 2.0;
  l8_tmp_tmp = A_init * l8_tmp;
  c_l8_tmp = l8_tmp_tmp * l11;
  b_l8_tmp_tmp = A_init * l9;
  d_l8_tmp = b_l8_tmp_tmp * l10_tmp;
  c_l8_tmp_tmp = A_wayp * l8_tmp;
  e_l8_tmp = c_l8_tmp_tmp * l11;
  d_l8_tmp_tmp = A_wayp * l9;
  f_l8_tmp = d_l8_tmp_tmp * l10_tmp;
  g_l8_tmp = T * l8_tmp;
  l8[1] =
      ((((c_l8_tmp * 2.0 - d_l8_tmp * 7.0) - e_l8_tmp * 2.0) + f_l8_tmp * 7.0) +
       g_l8_tmp * l12 * 2.0) -
      T * l9 * l11 * 7.0;
  h_l8_tmp = A_init * A_wayp;
  i_l8_tmp = A_init * T;
  j_l8_tmp = A_wayp * T;
  k_l8_tmp = l8_tmp * l12;
  l_l8_tmp = h_l8_tmp * J_max;
  m_l8_tmp = l2 * l8_tmp * l10_tmp;
  e_l8_tmp_tmp = l5 * l8_tmp * l10_tmp;
  n_l8_tmp = l_l8_tmp * l9;
  o_l8_tmp = h_l8_tmp * l8_tmp * l10_tmp;
  f_l8_tmp_tmp = J_max * l2;
  p_l8_tmp = f_l8_tmp_tmp * l9;
  q_l8_tmp = J_max * l5 * l9;
  l8[2] = ((((((((((p_l8_tmp * 8.0 + q_l8_tmp * 8.0) + m_l8_tmp * 2.0) +
                  e_l8_tmp_tmp * 2.0) +
                 k_l8_tmp * l13_tmp * 2.0) +
                b_l8_tmp * l13_tmp * 8.0) -
               n_l8_tmp * 16.0) -
              o_l8_tmp * 4.0) +
             i_l8_tmp * l8_tmp * l11 * 4.0) +
            i_l8_tmp * l9 * l10_tmp * 16.0) -
           j_l8_tmp * l8_tmp * l11 * 4.0) -
          j_l8_tmp * l9 * l10_tmp * 16.0;
  d = J_min * P_init;
  d1 = d * l12;
  d2 = J_min * P_wayp;
  d3 = d2 * l12;
  r_l8_tmp = A_init * J_min;
  s_l8_tmp = A_init * J_max;
  t_l8_tmp = A_wayp * J_min;
  u_l8_tmp = A_wayp * J_max;
  v_l8_tmp = J_min * T;
  w_l8_tmp = J_max * T;
  x_l8_tmp = T * V_init;
  y_l8_tmp = T * V_wayp;
  ab_l8_tmp = J_min * l6 * l10_tmp;
  bb_l8_tmp = P_init * l8_tmp * l11;
  cb_l8_tmp = P_init * l9 * l10_tmp;
  db_l8_tmp = P_wayp * l8_tmp * l11;
  eb_l8_tmp = P_wayp * l9 * l10_tmp;
  fb_l8_tmp = A_init * V_init;
  gb_l8_tmp = A_wayp * V_init;
  hb_l8_tmp = h_l8_tmp * T;
  ib_l8_tmp = r_l8_tmp * V_init;
  jb_l8_tmp = s_l8_tmp * V_init;
  kb_l8_tmp = v_l8_tmp * V_init;
  lb_l8_tmp = ib_l8_tmp * l11;
  mb_l8_tmp = jb_l8_tmp * l9;
  nb_l8_tmp = r_l8_tmp * V_wayp * l11;
  ob_l8_tmp = s_l8_tmp * V_wayp * l9;
  g_l8_tmp_tmp = t_l8_tmp * V_init;
  pb_l8_tmp = g_l8_tmp_tmp * l11;
  h_l8_tmp_tmp = u_l8_tmp * V_init;
  qb_l8_tmp = h_l8_tmp_tmp * l9;
  rb_l8_tmp = t_l8_tmp * V_wayp * l11;
  sb_l8_tmp = u_l8_tmp * V_wayp * l9;
  tb_l8_tmp = v_l8_tmp * V_wayp;
  ub_l8_tmp = r_l8_tmp * l5 * l10_tmp;
  vb_l8_tmp = s_l8_tmp * l5 * l8_tmp;
  wb_l8_tmp = t_l8_tmp * l2 * l10_tmp;
  xb_l8_tmp = u_l8_tmp * l2 * l8_tmp;
  yb_l8_tmp = fb_l8_tmp * l8_tmp * l10_tmp;
  ac_l8_tmp = A_init * V_wayp * l8_tmp * l10_tmp;
  bc_l8_tmp = gb_l8_tmp * l8_tmp * l10_tmp;
  cc_l8_tmp = A_wayp * V_wayp * l8_tmp * l10_tmp;
  dc_l8_tmp = l_l8_tmp * T;
  ec_l8_tmp = J_min * l3;
  fc_l8_tmp = A_init * l5 * l9;
  gc_l8_tmp = A_wayp * l2 * l9;
  l8[3] =
      ((((((((((((((((((((((((((((((((((((((((((((((d1 * 24.0 - d3 * 24.0) -
                                                   fc_l8_tmp * 12.0) +
                                                  gc_l8_tmp * 12.0) +
                                                 ec_l8_tmp * l10_tmp * 2.0) -
                                                J_max * l3 * l8_tmp * 12.0) -
                                               ab_l8_tmp * 2.0) +
                                              J_max * l6 * l8_tmp * 12.0) -
                                             bb_l8_tmp * 48.0) +
                                            cb_l8_tmp * 24.0) +
                                           db_l8_tmp * 48.0) -
                                          eb_l8_tmp * 24.0) -
                                         k_l8_tmp * l14 * 8.0) -
                                        b_l8_tmp * l14 * 2.0) -
                                       T * l2 * l8_tmp * l10_tmp * 12.0) -
                                      T * l5 * l8_tmp * l10_tmp * 12.0) -
                                     lb_l8_tmp * 12.0) -
                                    mb_l8_tmp * 12.0) +
                                   nb_l8_tmp * 12.0) +
                                  ob_l8_tmp * 12.0) -
                                 pb_l8_tmp * 12.0) -
                                qb_l8_tmp * 12.0) +
                               rb_l8_tmp * 12.0) +
                              sb_l8_tmp * 12.0) +
                             kb_l8_tmp * l12 * 12.0) +
                            tb_l8_tmp * l12 * 12.0) -
                           ub_l8_tmp * 6.0) -
                          vb_l8_tmp * 12.0) +
                         wb_l8_tmp * 6.0) +
                        xb_l8_tmp * 12.0) +
                       yb_l8_tmp * 24.0) -
                      ac_l8_tmp * 24.0) +
                     bc_l8_tmp * 24.0) -
                    cc_l8_tmp * 24.0) -
                   v_l8_tmp * l2 * l11 * 6.0) -
                  w_l8_tmp * l2 * l9 * 12.0) -
                 v_l8_tmp * l5 * l11 * 6.0) -
                w_l8_tmp * l5 * l9 * 12.0) -
               x_l8_tmp * l8_tmp * l11 * 24.0) +
              x_l8_tmp * l9 * l10_tmp * 12.0) -
             y_l8_tmp * l8_tmp * l11 * 24.0) +
            y_l8_tmp * l9 * l10_tmp * 12.0) -
           c_l8_tmp * l13_tmp * 24.0) -
          d_l8_tmp * l13_tmp * 6.0) +
         e_l8_tmp * l13_tmp * 24.0) +
        f_l8_tmp * l13_tmp * 6.0) +
       hb_l8_tmp * l8_tmp * l10_tmp * 48.0) +
      dc_l8_tmp * l9 * 12.0;
  d4 = J_min * J_max;
  d5 = J_min * l11;
  d6 = J_max * l9;
  d7 = l2 * l5;
  d8 = l8_tmp * l10_tmp;
  d9 = V_init * V_wayp;
  d10 = J_min * l2;
  d11 = d10 * l11;
  d12 = J_min * l5 * l11;
  d13 = V_init * l8_tmp * l11;
  d14 = V_init * l9 * l10_tmp;
  d15 = V_wayp * l8_tmp * l11;
  d16 = V_wayp * l9 * l10_tmp;
  d17 = r_l8_tmp * P_init;
  d18 = s_l8_tmp * P_init;
  b_A_init = r_l8_tmp * J_max;
  b_A_wayp = t_l8_tmp * J_max;
  d19 = r_l8_tmp * P_wayp;
  d20 = s_l8_tmp * P_wayp;
  d21 = t_l8_tmp * P_init;
  d22 = u_l8_tmp * P_init;
  d23 = t_l8_tmp * P_wayp;
  hc_l8_tmp = J_min * V_init;
  ic_l8_tmp = J_max * V_init;
  jc_l8_tmp = J_min * V_wayp;
  kc_l8_tmp = J_max * V_wayp;
  lc_l8_tmp = P_init * T;
  mc_l8_tmp = P_wayp * T;
  nc_l8_tmp = r_l8_tmp * T;
  oc_l8_tmp = s_l8_tmp * T;
  pc_l8_tmp = t_l8_tmp * T;
  qc_l8_tmp = u_l8_tmp * T;
  rc_l8_tmp = hc_l8_tmp * l12;
  sc_l8_tmp = jc_l8_tmp * l12;
  tc_l8_tmp = A_init * P_init;
  uc_l8_tmp = A_init * P_wayp;
  vc_l8_tmp = A_wayp * P_init;
  wc_l8_tmp = A_wayp * P_wayp;
    l8[4] = ((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((l4 * l10_tmp * -3.0 - l7 * l10_tmp * 3.0) - l12 * l17 * 12.0) - l12 * l18 * 12.0) + d4 * l4 * 8.0) + d4 * l7 * 8.0) + d9 * l12 * 24.0) - A_init * l6 * l8_tmp * 12.0) - A_wayp * l3 * l8_tmp * 12.0) + d5 * l17 * 48.0) + d6 * l17 * 24.0) + d5 * l18 * 48.0) + d6 * l18 * 24.0) + V_init * l2 * l11 * 12.0) - V_init * l5 * l11 * 12.0) - V_wayp * l2 * l11 * 12.0) + V_wayp * l5 * l11 * 12.0) + d7 * l8_tmp * 24.0) + d7 * l10_tmp * 6.0) - d8 * l17 * 60.0) + k_l8_tmp * l15 * 7.0) - b_l8_tmp * l15 * 2.0) - d8 * l18 * 60.0) + d11 * l13_tmp * 12.0) + d12 * l13_tmp * 12.0) + d13 * l13_tmp * 48.0) - d14 * l13_tmp * 24.0) + d15 * l13_tmp * 48.0) - d16 * l13_tmp * 24.0) + m_l8_tmp * l13_tmp * 18.0) + e_l8_tmp_tmp * l13_tmp * 18.0) - b_A_init * l6 * 8.0) - b_A_wayp * l3 * 8.0) - d17 * l11 * 48.0) - d18 * l9 * 48.0) + d19 * l11 * 48.0) + d20 * l9 * 48.0) + d21 * l11 * 48.0) + d22 * l9 * 48.0) - d23 * l11 * 48.0) - u_l8_tmp * P_wayp * l9 * 48.0) - d * T * l12 * 48.0) + d2 * T * l12 * 48.0) - hc_l8_tmp * V_wayp * l11 * 96.0) - ic_l8_tmp * V_wayp * l9 * 48.0) + tc_l8_tmp * l8_tmp * l10_tmp * 96.0) - uc_l8_tmp * l8_tmp * l10_tmp * 96.0) - vc_l8_tmp * l8_tmp * l10_tmp * 96.0) + wc_l8_tmp * l8_tmp * l10_tmp * 96.0) + v_l8_tmp * l3 * l10_tmp * 8.0) + w_l8_tmp * l3 * l8_tmp * 12.0) - v_l8_tmp * l6 * l10_tmp * 8.0) - w_l8_tmp * l6 * l8_tmp * 12.0) - hc_l8_tmp * l2 * l10_tmp * 24.0) + ic_l8_tmp * l2 * l8_tmp * 12.0) + hc_l8_tmp * l5 * l10_tmp * 24.0) - ic_l8_tmp * l5 * l8_tmp * 12.0) - rc_l8_tmp * l13_tmp * 24.0) + jc_l8_tmp * l2 * l10_tmp * 24.0) - kc_l8_tmp * l2 * l8_tmp * 12.0) - jc_l8_tmp * l5 * l10_tmp * 24.0) + kc_l8_tmp * l5 * l8_tmp * 12.0) - sc_l8_tmp * l13_tmp * 24.0) + lc_l8_tmp * l8_tmp * l11 * 96.0) - lc_l8_tmp * l9 * l10_tmp * 48.0) - mc_l8_tmp * l8_tmp * l11 * 96.0) + mc_l8_tmp * l9 * l10_tmp * 48.0) + d9 * l8_tmp * l10_tmp * 120.0) + c_l8_tmp * l14 * 28.0) - d_l8_tmp * l14 * 8.0) - e_l8_tmp * l14 * 28.0) + f_l8_tmp * l14 * 8.0) - nc_l8_tmp * V_wayp * l11 * 48.0) - oc_l8_tmp * V_wayp * l9 * 48.0) + pc_l8_tmp * V_init * l11 * 48.0) + qc_l8_tmp * V_init * l9 * 48.0) + n_l8_tmp * l13_tmp * 24.0) + nc_l8_tmp * l5 * l10_tmp * 24.0) + oc_l8_tmp * l5 * l8_tmp * 36.0) - pc_l8_tmp * l2 * l10_tmp * 24.0) - qc_l8_tmp * l2 * l8_tmp * 36.0) + i_l8_tmp * V_wayp * l8_tmp * l10_tmp * 96.0) - j_l8_tmp * V_init * l8_tmp * l10_tmp * 96.0) - o_l8_tmp * l13_tmp * 84.0;
    d5 = g_l8_tmp * l10_tmp;
    d6 = d4 * T;
    d7 = r_l8_tmp * l10_tmp;
    d8 = s_l8_tmp * l8_tmp;
    d9 = t_l8_tmp * l10_tmp;
    l12 = u_l8_tmp * l8_tmp;
    u_l8_tmp = J_max * P_init;
    hc_l8_tmp = J_max * P_wayp;
    ic_l8_tmp = v_l8_tmp * l11;
    jc_l8_tmp = w_l8_tmp * l9;
    g_l8_tmp = w_l8_tmp * V_init;
    i_l8_tmp = h_l8_tmp * J_min;
    j_l8_tmp = i_l8_tmp * T;
    n_l8_tmp = b_A_init * T;
    s_l8_tmp = b_A_wayp * T;
    l8[5] = ((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((-rt_powd_snf(A_init, 5.0) * J_min + rt_powd_snf(A_wayp, 5.0) * J_min) - r_l8_tmp * l7 * 5.0) + t_l8_tmp * l4 * 5.0) - b_l8_tmp_tmp * l17 * 12.0) - b_l8_tmp_tmp * l18 * 12.0) + d_l8_tmp_tmp * l17 * 12.0) + d_l8_tmp_tmp * l18 * 12.0) + d10 * l6 * 10.0) - ec_l8_tmp * l5 * 10.0) + P_init * l2 * l9 * 24.0) + P_init * l5 * l9 * 24.0) - P_wayp * l2 * l9 * 24.0) - P_wayp * l5 * l9 * 24.0) - k_l8_tmp * l16 * 2.0) + b_l8_tmp * l16) - ec_l8_tmp * l10_tmp * l13_tmp * 10.0) - d11 * l14 * 6.0) + p_l8_tmp * l14 * 4.0) + ab_l8_tmp * l13_tmp * 10.0) - d12 * l14 * 6.0) + q_l8_tmp * l14 * 4.0) - bb_l8_tmp * l13_tmp * 48.0) + cb_l8_tmp * l13_tmp * 24.0) + db_l8_tmp * l13_tmp * 48.0) - eb_l8_tmp * l13_tmp * 24.0) + d5 * l17 * 24.0) + d5 * l18 * 24.0) - d13 * l14 * 24.0) + d14 * l14 * 12.0) - d15 * l14 * 24.0) + d16 * l14 * 12.0) - m_l8_tmp * l14 * 8.0) - e_l8_tmp_tmp * l14 * 8.0) - h_l8_tmp * P_init * l9 * 48.0) + h_l8_tmp * P_wayp * l9 * 48.0) - d6 * l4 * 5.0) - d6 * l7 * 5.0) + fb_l8_tmp * V_wayp * l9 * 24.0) - gb_l8_tmp * V_wayp * l9 * 24.0) - d7 * l17 * 12.0) + d8 * l17 * 24.0) - d7 * l18 * 12.0) + d8 * l18 * 24.0) + d9 * l17 * 12.0) - l12 * l17 * 24.0) + d9 * l18 * 12.0) - l12 * l18 * 24.0) + d * l2 * l10_tmp * 24.0) - u_l8_tmp * l2 * l8_tmp * 48.0) + d * l5 * l10_tmp * 24.0) - u_l8_tmp * l5 * l8_tmp * 48.0) + d1 * l13_tmp * 24.0) - d2 * l2 * l10_tmp * 24.0) + hc_l8_tmp * l2 * l8_tmp * 48.0) - d2 * l5 * l10_tmp * 24.0) + hc_l8_tmp * l5 * l8_tmp * 48.0) - d3 * l13_tmp * 24.0) - ic_l8_tmp * l17 * 12.0) - jc_l8_tmp * l17 * 12.0) - ic_l8_tmp * l18 * 12.0) - jc_l8_tmp * l18 * 12.0) + rc_l8_tmp * l14 * 12.0) + sc_l8_tmp * l14 * 12.0) + x_l8_tmp * l5 * l9 * 24.0) + y_l8_tmp * l2 * l9 * 24.0) + fc_l8_tmp * l13_tmp * 12.0) - c_l8_tmp * l15 * 10.0) + d_l8_tmp * l15 * 5.0) - gc_l8_tmp * l13_tmp * 12.0) + e_l8_tmp * l15 * 10.0) - f_l8_tmp * l15 * 5.0) - hb_l8_tmp * V_init * l9 * 24.0) - hb_l8_tmp * V_wayp * l9 * 24.0) + d17 * T * l11 * 48.0) + d18 * T * l9 * 48.0) - d19 * T * l11 * 48.0) - d20 * T * l9 * 48.0) - d21 * T * l11 * 48.0) - d22 * T * l9 * 48.0) + d23 * T * l11 * 48.0) + A_wayp * J_max * P_wayp * T * l9 * 48.0) + ib_l8_tmp * V_wayp * l10_tmp * 24.0) - jb_l8_tmp * V_wayp * l8_tmp * 48.0) - g_l8_tmp_tmp * V_wayp * l10_tmp * 24.0) + h_l8_tmp_tmp * V_wayp * l8_tmp * 48.0) - A_init * A_wayp * J_max * l9 * l14 * 20.0) + kb_l8_tmp * V_wayp * l11 * 24.0) + g_l8_tmp * V_wayp * l9 * 24.0) + lb_l8_tmp * l13_tmp * 12.0) + mb_l8_tmp * l13_tmp * 12.0) + nb_l8_tmp * l13_tmp * 36.0) + ob_l8_tmp * l13_tmp * 36.0) - pb_l8_tmp * l13_tmp * 36.0) - qb_l8_tmp * l13_tmp * 36.0) - rb_l8_tmp * l13_tmp * 12.0) - sb_l8_tmp * l13_tmp * 12.0) - tc_l8_tmp * T * l8_tmp * l10_tmp * 96.0) + uc_l8_tmp * T * l8_tmp * l10_tmp * 96.0) + vc_l8_tmp * T * l8_tmp * l10_tmp * 96.0) - wc_l8_tmp * T * l8_tmp * l10_tmp * 96.0) - d6 * l2 * l5 * 30.0) + kb_l8_tmp * l5 * l10_tmp * 24.0) - g_l8_tmp * l5 * l8_tmp * 48.0) + tb_l8_tmp * l2 * l10_tmp * 24.0) - w_l8_tmp * V_wayp * l2 * l8_tmp * 48.0) + o_l8_tmp * l14 * 40.0) - ub_l8_tmp * l13_tmp * 18.0) - vb_l8_tmp * l13_tmp * 24.0) + wb_l8_tmp * l13_tmp * 18.0) + xb_l8_tmp * l13_tmp * 24.0) - x_l8_tmp * V_wayp * l8_tmp * l10_tmp * 48.0) - yb_l8_tmp * l13_tmp * 24.0) - ac_l8_tmp * l13_tmp * 72.0) + bc_l8_tmp * l13_tmp * 72.0) + cc_l8_tmp * l13_tmp * 24.0) - i_l8_tmp * P_init * l10_tmp * 48.0) + l_l8_tmp * P_init * l8_tmp * 96.0) + i_l8_tmp * P_wayp * l10_tmp * 48.0) - l_l8_tmp * P_wayp * l8_tmp * 96.0) + n_l8_tmp * l6 * 20.0) + s_l8_tmp * l3 * 20.0) - j_l8_tmp * V_init * l10_tmp * 24.0) + dc_l8_tmp * V_init * l8_tmp * 48.0) - j_l8_tmp * V_wayp * l10_tmp * 24.0) + dc_l8_tmp * V_wayp * l8_tmp * 48.0;
    coder::roots(l8, t4_data, &t4_size);
    for (k = 0; k < t4_size; k++) {
      d = t4_data[k].re;
      d1 = t4_data[k].im;
      y_data[k].re = d * d - d1 * d1;
      d *= d1;
      y_data[k].im = d + d;
    }
    lc_l8_tmp = J_min * l10_tmp;
    for (i = 0; i < t4_size; i++) {
      tmp_data[i].re = lc_l8_tmp * y_data[i].re;
      tmp_data[i].im = lc_l8_tmp * y_data[i].im;
    }
    for (i = 0; i < t4_size; i++) {
      b_tmp_data[i].re = 2.0 * (b_A_init * t4_data[i].re);
      b_tmp_data[i].im = 2.0 * (b_A_init * t4_data[i].im);
    }
    for (i = 0; i < t4_size; i++) {
      c_tmp_data[i].re = 2.0 * (b_A_wayp * t4_data[i].re);
      c_tmp_data[i].im = 2.0 * (b_A_wayp * t4_data[i].im);
    }
    mc_l8_tmp = v_l8_tmp * l10_tmp;
    for (i = 0; i < t4_size; i++) {
      d_tmp_data[i].re = 2.0 * (mc_l8_tmp * t4_data[i].re);
      d_tmp_data[i].im = 2.0 * (mc_l8_tmp * t4_data[i].im);
    }
    for (i = 0; i < t4_size; i++) {
      e_tmp_data[i].re = 2.0 * (lc_l8_tmp * t4_data[i].re);
      e_tmp_data[i].im = 2.0 * (lc_l8_tmp * t4_data[i].im);
    }
    kc_l8_tmp = J_max * l8_tmp;
    for (i = 0; i < t4_size; i++) {
      f_tmp_data[i].re = 2.0 * (kc_l8_tmp * t4_data[i].re);
      f_tmp_data[i].im = 2.0 * (kc_l8_tmp * t4_data[i].im);
    }
    u_l8_tmp = (((((((d10 * 2.0 - f_l8_tmp_tmp) + V_init * l10_tmp * 2.0) -
                    V_wayp * l10_tmp * 2.0) +
                   A_wayp * A_wayp * J_max) -
                  i_l8_tmp * 2.0) -
                 d4 * V_init * 2.0) +
                d4 * V_wayp * 2.0) +
               J_min * l13_tmp * l10_tmp;
    hc_l8_tmp = n_l8_tmp * 2.0;
    l12 = s_l8_tmp * 2.0;
    for (i = 0; i < t4_size; i++) {
      x_data[i].re = -((((((u_l8_tmp + tmp_data[i].re) + hc_l8_tmp) - l12) -
                         b_tmp_data[i].re) +
                        c_tmp_data[i].re) -
                       d_tmp_data[i].re);
      x_data[i].im =
          -(((tmp_data[i].im - b_tmp_data[i].im) + c_tmp_data[i].im) -
            d_tmp_data[i].im);
    }
    hc_l8_tmp = ((((l8_tmp_tmp * 2.0 - c_l8_tmp_tmp * 2.0) - b_A_init * 2.0) +
                  b_A_wayp * 2.0) -
                 mc_l8_tmp * 2.0) +
                w_l8_tmp * l8_tmp * 2.0;
    for (i = 0; i < t4_size; i++) {
      y_data[i].re = (hc_l8_tmp + e_tmp_data[i].re) - f_tmp_data[i].re;
      y_data[i].im = e_tmp_data[i].im - f_tmp_data[i].im;
    }
    z_size_idx_0 = static_cast<signed char>(t4_size);
    p = true;
    b_p = true;
    k = 0;
    exitg1 = false;
    while ((!exitg1) && (k < 2)) {
      if (k + 1 <= 1) {
        i = z_size_idx_0;
        i1 = t4_size;
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
          i1 = t4_size;
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
    z_size_idx_0 = static_cast<signed char>(t4_size);
    p = true;
    b_p = true;
    k = 0;
    exitg1 = false;
    while ((!exitg1) && (k < 2)) {
      if (k + 1 <= 1) {
        i = z_size_idx_0;
        i1 = t4_size;
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
          i1 = t4_size;
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
    for (i = 0; i < t4_size; i++) {
      ic_l8_tmp = x_data[i].re;
      jc_l8_tmp = x_data[i].im;
      hc_l8_tmp = y_data[i].re;
      kc_l8_tmp = y_data[i].im;
      if (kc_l8_tmp == 0.0) {
        if (jc_l8_tmp == 0.0) {
          mc_l8_tmp = ic_l8_tmp / hc_l8_tmp;
          l12 = 0.0;
        } else if (ic_l8_tmp == 0.0) {
          mc_l8_tmp = 0.0;
          l12 = jc_l8_tmp / hc_l8_tmp;
        } else {
          mc_l8_tmp = ic_l8_tmp / hc_l8_tmp;
          l12 = jc_l8_tmp / hc_l8_tmp;
        }
      } else if (hc_l8_tmp == 0.0) {
        if (ic_l8_tmp == 0.0) {
          mc_l8_tmp = jc_l8_tmp / kc_l8_tmp;
          l12 = 0.0;
        } else if (jc_l8_tmp == 0.0) {
          mc_l8_tmp = 0.0;
          l12 = -(ic_l8_tmp / kc_l8_tmp);
        } else {
          mc_l8_tmp = jc_l8_tmp / kc_l8_tmp;
          l12 = -(ic_l8_tmp / kc_l8_tmp);
        }
      } else {
        lc_l8_tmp = std::abs(hc_l8_tmp);
        l12 = std::abs(kc_l8_tmp);
        if (lc_l8_tmp > l12) {
          u_l8_tmp = kc_l8_tmp / hc_l8_tmp;
          l12 = hc_l8_tmp + u_l8_tmp * kc_l8_tmp;
          mc_l8_tmp = (ic_l8_tmp + u_l8_tmp * jc_l8_tmp) / l12;
          l12 = (jc_l8_tmp - u_l8_tmp * ic_l8_tmp) / l12;
        } else if (l12 == lc_l8_tmp) {
          if (hc_l8_tmp > 0.0) {
            u_l8_tmp = 0.5;
          } else {
            u_l8_tmp = -0.5;
          }
          if (kc_l8_tmp > 0.0) {
            l12 = 0.5;
          } else {
            l12 = -0.5;
          }
          mc_l8_tmp = (ic_l8_tmp * u_l8_tmp + jc_l8_tmp * l12) / lc_l8_tmp;
          l12 = (jc_l8_tmp * u_l8_tmp - ic_l8_tmp * l12) / lc_l8_tmp;
        } else {
          u_l8_tmp = hc_l8_tmp / kc_l8_tmp;
          l12 = kc_l8_tmp + u_l8_tmp * hc_l8_tmp;
          mc_l8_tmp = (u_l8_tmp * ic_l8_tmp + jc_l8_tmp) / l12;
          l12 = (u_l8_tmp * jc_l8_tmp - ic_l8_tmp) / l12;
        }
      }
      x_data[i].re = mc_l8_tmp;
      x_data[i].im = l12;
    }
    l2 = 1.0 / J_min;
    l12 = A_wayp * l2;
    for (i = 0; i < t4_size; i++) {
      tmp_data[i].re = J_max * ((x_data[i].re + t4_data[i].re) + l12);
      tmp_data[i].im = J_max * (x_data[i].im + t4_data[i].im);
    }
    for (i = 0; i < t4_size; i++) {
      b_tmp_data[i].re = J_min * x_data[i].re;
      b_tmp_data[i].im = J_min * x_data[i].im;
    }
    u_l8_tmp = J_max * (J_max * l2 - 1.0);
    for (i = 0; i < t4_size; i++) {
      ic_l8_tmp = -(((A_init - tmp_data[i].re) + b_tmp_data[i].re) + w_l8_tmp);
      jc_l8_tmp = -((0.0 - tmp_data[i].im) + b_tmp_data[i].im);
      if (jc_l8_tmp == 0.0) {
        y_data[i].re = ic_l8_tmp / u_l8_tmp;
        y_data[i].im = 0.0;
      } else if (ic_l8_tmp == 0.0) {
        y_data[i].re = 0.0;
        y_data[i].im = jc_l8_tmp / u_l8_tmp;
      } else {
        y_data[i].re = ic_l8_tmp / u_l8_tmp;
        y_data[i].im = jc_l8_tmp / u_l8_tmp;
      }
    }
    for (i = 0; i < t4_size; i++) {
      ic_l8_tmp = -(((A_init - tmp_data[i].re) + b_tmp_data[i].re) + w_l8_tmp);
      jc_l8_tmp = -((0.0 - tmp_data[i].im) + b_tmp_data[i].im);
      if (jc_l8_tmp == 0.0) {
        hc_l8_tmp = ic_l8_tmp / u_l8_tmp;
        l12 = 0.0;
      } else if (ic_l8_tmp == 0.0) {
        hc_l8_tmp = 0.0;
        l12 = jc_l8_tmp / u_l8_tmp;
      } else {
        hc_l8_tmp = ic_l8_tmp / u_l8_tmp;
        l12 = jc_l8_tmp / u_l8_tmp;
      }
      ic_l8_tmp = A_wayp - J_max * hc_l8_tmp;
      jc_l8_tmp = 0.0 - J_max * l12;
      if (jc_l8_tmp == 0.0) {
        b_z_data[i].re = ic_l8_tmp / J_min;
        b_z_data[i].im = 0.0;
      } else if (ic_l8_tmp == 0.0) {
        b_z_data[i].re = 0.0;
        b_z_data[i].im = jc_l8_tmp / J_min;
      } else {
        b_z_data[i].re = ic_l8_tmp / J_min;
        b_z_data[i].im = jc_l8_tmp / J_min;
      }
    }
    for (i = 0; i < t4_size; i++) {
      ic_l8_tmp = -(A_init + b_tmp_data[i].re);
      jc_l8_tmp = -b_tmp_data[i].im;
      if (jc_l8_tmp == 0.0) {
        z_data[i].re = ic_l8_tmp / J_max;
        z_data[i].im = 0.0;
      } else if (ic_l8_tmp == 0.0) {
        z_data[i].re = 0.0;
        z_data[i].im = jc_l8_tmp / J_max;
      } else {
        z_data[i].re = ic_l8_tmp / J_max;
        z_data[i].im = jc_l8_tmp / J_max;
      }
    }
    p = (t4_size == 5);
    if (!p) {
      h_rtErrorWithMessageID(i_emlrtRTEI.fName, i_emlrtRTEI.lineNo);
    }
    p = ((t4_size == 5) && p);
    if (!p) {
      h_rtErrorWithMessageID(i_emlrtRTEI.fName, i_emlrtRTEI.lineNo);
    }
    p = ((t4_size == 5) && p);
    if (!p) {
      h_rtErrorWithMessageID(i_emlrtRTEI.fName, i_emlrtRTEI.lineNo);
    }
    p = ((t4_size == 5) && p);
    if (!p) {
      h_rtErrorWithMessageID(i_emlrtRTEI.fName, i_emlrtRTEI.lineNo);
    }
    p = ((t4_size == 5) && p);
    if (!p) {
      h_rtErrorWithMessageID(i_emlrtRTEI.fName, i_emlrtRTEI.lineNo);
    }
    if (0 <= t4_size - 1) {
      std::copy(&z_data[0], &z_data[t4_size], &t[0]);
    }
    std::memset(&t[5], 0, 5U * sizeof(creal_T));
    if (0 <= t4_size - 1) {
      std::copy(&x_data[0], &x_data[t4_size], &t[10]);
      std::copy(&t4_data[0], &t4_data[t4_size], &t[15]);
      std::copy(&b_z_data[0], &b_z_data[t4_size], &t[20]);
    }
    std::memset(&t[25], 0, 5U * sizeof(creal_T));
    if (0 <= t4_size - 1) {
      std::copy(&y_data[0], &y_data[t4_size], &t[30]);
    }
}

// End of code generation (acdeg_TV_AVP.cpp)
