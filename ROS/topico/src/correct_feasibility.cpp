//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// correct_feasibility.cpp
//
// Code generation for function 'correct_feasibility'
//

// Include files
#include "correct_feasibility.h"
#include "check_feasibility.h"
#include "eml_int_forloop_overflow_check.h"
#include "evaluate_to_time.h"
#include "indexShapeCheck.h"
#include "rt_nonfinite.h"
#include "simplify_setp.h"
#include "solve_O.h"
#include "topico_wrapper_rtwutil.h"
#include "topico_wrapper_types.h"
#include "useConstantDim.h"
#include "coder_array.h"
#include <algorithm>

// Function Definitions
void correct_feasibility(double P_init, double V_init, double A_init,
                         double V_max, double V_min, double A_max, double A_min,
                         double J_max, double J_min, bool b_hard_V_lim,
                         double t[4], double J[4])
{
  static rtBoundsCheckInfo nb_emlrtBCI = {
      -1,                                                         // iFirst
      -1,                                                         // iLast
      62,                                                         // lineNo
      25,                                                         // colNo
      "t_test",                                                   // aName
      "correct_feasibility",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/correct_feasibility.m", // pName
      0                                                           // checkKind
  };
  static rtBoundsCheckInfo ob_emlrtBCI = {
      -1,                                                         // iFirst
      -1,                                                         // iLast
      63,                                                         // lineNo
      25,                                                         // colNo
      "J_test",                                                   // aName
      "correct_feasibility",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/correct_feasibility.m", // pName
      0                                                           // checkKind
  };
  static rtBoundsCheckInfo pb_emlrtBCI = {
      -1,                                                         // iFirst
      -1,                                                         // iLast
      66,                                                         // lineNo
      25,                                                         // colNo
      "t_test",                                                   // aName
      "correct_feasibility",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/correct_feasibility.m", // pName
      0                                                           // checkKind
  };
  static rtBoundsCheckInfo qb_emlrtBCI = {
      -1,                                                         // iFirst
      -1,                                                         // iLast
      67,                                                         // lineNo
      25,                                                         // colNo
      "J_test",                                                   // aName
      "correct_feasibility",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/correct_feasibility.m", // pName
      0                                                           // checkKind
  };
  static rtBoundsCheckInfo rb_emlrtBCI = {
      -1,                                                         // iFirst
      -1,                                                         // iLast
      73,                                                         // lineNo
      25,                                                         // colNo
      "t_test",                                                   // aName
      "correct_feasibility",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/correct_feasibility.m", // pName
      0                                                           // checkKind
  };
  static rtBoundsCheckInfo sb_emlrtBCI = {
      -1,                                                         // iFirst
      -1,                                                         // iLast
      74,                                                         // lineNo
      25,                                                         // colNo
      "J_test",                                                   // aName
      "correct_feasibility",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/correct_feasibility.m", // pName
      0                                                           // checkKind
  };
  static rtBoundsCheckInfo tb_emlrtBCI = {
      -1,                                                         // iFirst
      -1,                                                         // iLast
      80,                                                         // lineNo
      25,                                                         // colNo
      "t_test",                                                   // aName
      "correct_feasibility",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/correct_feasibility.m", // pName
      0                                                           // checkKind
  };
  static rtBoundsCheckInfo ub_emlrtBCI = {
      -1,                                                         // iFirst
      -1,                                                         // iLast
      81,                                                         // lineNo
      25,                                                         // colNo
      "J_test",                                                   // aName
      "correct_feasibility",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/correct_feasibility.m", // pName
      0                                                           // checkKind
  };
  coder::array<double, 2U> J_2;
  coder::array<double, 2U> J_setp_struct_time;
  coder::array<double, 2U> b_expl_temp_values;
  coder::array<double, 2U> c_expl_temp_values;
  coder::array<double, 2U> c_tmp_data;
  coder::array<double, 2U> d_tmp_data;
  coder::array<double, 2U> expl_temp_values;
  coder::array<double, 2U> g_tmp_data;
  coder::array<double, 2U> h_tmp_data;
  coder::array<double, 2U> i_tmp_data;
  coder::array<double, 2U> j_tmp_data;
  coder::array<double, 2U> r;
  coder::array<double, 2U> t_2;
  double J_test_data[700];
  double t_test_data[700];
  double e_tmp_data[2];
  double f_tmp_data[2];
  double b_A_init;
  double b_P_init;
  double b_V_init;
  double b_tmp_data;
  double tmp_data;
  int a__4_data[100];
  int J_test_size[2];
  int t_test_size[2];
  int b;
  int i;
  int j;
  unsigned char feasibility;
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
  t[0] = 0.0;
  J[0] = 0.0;
  t[1] = 0.0;
  J[1] = 0.0;
  t[2] = 0.0;
  J[2] = 0.0;
  t[3] = 0.0;
  J[3] = 0.0;
  feasibility = check_feasibility(V_init, A_init, V_max, V_min, A_max, A_min,
                                  J_max, J_min);
  if (feasibility == 1) {
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
    t[0] = -(-A_init - (-A_max)) / -J_min;
    J[0] = J_min;
  } else if (feasibility == 2) {
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
    t[0] = -(A_init - A_min) / J_max;
    J[0] = J_max;
  }
  if ((feasibility == 1) || (feasibility == 2)) {
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
    tmp_data = t[0];
    b_tmp_data = J[0];
    c_tmp_data.set(&tmp_data, 1, 1);
    d_tmp_data.set(&b_tmp_data, 1, 1);
    simplify_setp(c_tmp_data, d_tmp_data, t_2, J_2);
    coder::internal::useConstantDim(t_2);
    r.set_size(1, t_2.size(1) + 1);
    r[0] = 0.0;
    b = t_2.size(1);
    if ((1 <= t_2.size(1)) && (t_2.size(1) > 2147483646)) {
      coder::check_forloop_overflow_error();
    }
    for (j = 0; j < b; j++) {
      r[j + 1] = t_2[j];
    }
    b = r.size(1);
    j = r.size(1);
    for (i = 0; i < j; i++) {
      e_tmp_data[i] = r[i];
    }
    if (0 <= b - 1) {
      std::copy(&e_tmp_data[0], &e_tmp_data[b], &f_tmp_data[0]);
    }
    J_setp_struct_time.set_size(b, 1);
    for (i = 0; i < b; i++) {
      J_setp_struct_time[i] = f_tmp_data[i];
    }
    expl_temp_values.set_size(1, J_2.size(1));
    j = J_2.size(1);
    for (i = 0; i < j; i++) {
      expl_temp_values[i] = J_2[i];
    }
    expl_temp_values.set_size(1, J_2.size(1) + 1);
    expl_temp_values[J_2.size(1)] = 0.0;
    evaluate_to_time(P_init, V_init, A_init, J_setp_struct_time,
                     expl_temp_values, &P_init, &b_V_init, &b_A_init);
    V_init = b_V_init;
    A_init = b_A_init;
    feasibility = check_feasibility(b_V_init, b_A_init, V_max, V_min, A_max,
                                    A_min, J_max, J_min);
  }
  if (b_hard_V_lim && (feasibility == 3)) {
    solve_O(P_init, V_init, A_init, V_max, V_min, A_max, A_min, J_max, J_min,
            t_test_data, t_test_size, J_test_data, J_test_size, a__4_data, &b);
    // V_max =  Inf wegen check, V_min muss bleiben für Prädiktion und
    // 3-phasiges t
    b = t_test_size[0] * 7;
    if (1 > b) {
      rtDynamicBoundsError(1, 1, b, &nb_emlrtBCI);
    }
    t[1] = t_test_data[0];
    if (2 > b) {
      rtDynamicBoundsError(2, 1, 1, &nb_emlrtBCI);
    }
    t[2] = t_test_data[1];
    if (3 > b) {
      rtDynamicBoundsError(3, 1, 2, &nb_emlrtBCI);
    }
    t[3] = t_test_data[2];
    b = J_test_size[0] * 7;
    if (1 > b) {
      rtDynamicBoundsError(1, 1, b, &ob_emlrtBCI);
    }
    J[1] = J_test_data[0];
    if (2 > b) {
      rtDynamicBoundsError(2, 1, 1, &ob_emlrtBCI);
    }
    J[2] = J_test_data[1];
    if (3 > b) {
      rtDynamicBoundsError(3, 1, 2, &ob_emlrtBCI);
    }
    J[3] = J_test_data[2];
  } else if (b_hard_V_lim && (feasibility == 4)) {
    b_solve_O(P_init, V_init, A_init, V_min, V_max, A_max, A_min, J_max, J_min,
              t_test_data, t_test_size, J_test_data, J_test_size, a__4_data,
              &b);
    // V_min = -Inf wegen check, V_max muss bleiben für Prädiktion und
    // 3-phasiges t
    b = t_test_size[0] * 7;
    if (1 > b) {
      rtDynamicBoundsError(1, 1, b, &pb_emlrtBCI);
    }
    t[1] = t_test_data[0];
    if (2 > b) {
      rtDynamicBoundsError(2, 1, 1, &pb_emlrtBCI);
    }
    t[2] = t_test_data[1];
    if (3 > b) {
      rtDynamicBoundsError(3, 1, 2, &pb_emlrtBCI);
    }
    t[3] = t_test_data[2];
    b = J_test_size[0] * 7;
    if (1 > b) {
      rtDynamicBoundsError(1, 1, b, &qb_emlrtBCI);
    }
    J[1] = J_test_data[0];
    if (2 > b) {
      rtDynamicBoundsError(2, 1, 1, &qb_emlrtBCI);
    }
    J[2] = J_test_data[1];
    if (3 > b) {
      rtDynamicBoundsError(3, 1, 2, &qb_emlrtBCI);
    }
    J[3] = J_test_data[2];
  } else if (b_hard_V_lim && (feasibility == 5)) {
    double t_test1_idx_0;
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
    t_test1_idx_0 = A_init / -J_min;
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
    tmp_data = t_test1_idx_0;
    b_tmp_data = J_min;
    g_tmp_data.set(&tmp_data, 1, 1);
    i_tmp_data.set(&b_tmp_data, 1, 1);
    simplify_setp(g_tmp_data, i_tmp_data, t_2, J_2);
    coder::internal::useConstantDim(t_2);
    r.set_size(1, t_2.size(1) + 1);
    r[0] = 0.0;
    b = t_2.size(1);
    if ((1 <= t_2.size(1)) && (t_2.size(1) > 2147483646)) {
      coder::check_forloop_overflow_error();
    }
    for (j = 0; j < b; j++) {
      r[j + 1] = t_2[j];
    }
    b = r.size(1);
    j = r.size(1);
    for (i = 0; i < j; i++) {
      e_tmp_data[i] = r[i];
    }
    if (0 <= b - 1) {
      std::copy(&e_tmp_data[0], &e_tmp_data[b], &f_tmp_data[0]);
    }
    J_setp_struct_time.set_size(b, 1);
    for (i = 0; i < b; i++) {
      J_setp_struct_time[i] = f_tmp_data[i];
    }
    b_expl_temp_values.set_size(1, J_2.size(1));
    j = J_2.size(1);
    for (i = 0; i < j; i++) {
      b_expl_temp_values[i] = J_2[i];
    }
    b_expl_temp_values.set_size(1, J_2.size(1) + 1);
    b_expl_temp_values[J_2.size(1)] = 0.0;
    evaluate_to_time(P_init, V_init, A_init, J_setp_struct_time,
                     b_expl_temp_values, &b_P_init, &b_V_init, &b_A_init);
    solve_O(b_P_init, b_V_init, b_A_init, V_max, V_min, A_max, A_min, J_max,
            J_min, t_test_data, t_test_size, J_test_data, J_test_size,
            a__4_data, &b);
    // V_max =  Inf wegen check, V_min muss bleiben für Prädiktion und
    // 3-phasiges t
    coder::internal::indexShapeCheck(t_test_size);
    b = t_test_size[0] * 7;
    if (1 > b) {
      rtDynamicBoundsError(1, 1, b, &rb_emlrtBCI);
    }
    t[1] = t_test_data[0] + t_test1_idx_0;
    if (2 > b) {
      rtDynamicBoundsError(2, 1, 1, &rb_emlrtBCI);
    }
    t[2] = t_test_data[1];
    if (3 > b) {
      rtDynamicBoundsError(3, 1, 2, &rb_emlrtBCI);
    }
    t[3] = t_test_data[2];
    b = J_test_size[0] * 7;
    if (1 > b) {
      rtDynamicBoundsError(1, 1, b, &sb_emlrtBCI);
    }
    J[1] = J_test_data[0];
    if (2 > b) {
      rtDynamicBoundsError(2, 1, 1, &sb_emlrtBCI);
    }
    J[2] = J_test_data[1];
    if (3 > b) {
      rtDynamicBoundsError(3, 1, 2, &sb_emlrtBCI);
    }
    J[3] = J_test_data[2];
  } else if (b_hard_V_lim && (feasibility == 6)) {
    double t_test1_idx_0;
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
    t_test1_idx_0 = -A_init / J_max;
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
    tmp_data = t_test1_idx_0;
    b_tmp_data = J_max;
    h_tmp_data.set(&tmp_data, 1, 1);
    j_tmp_data.set(&b_tmp_data, 1, 1);
    simplify_setp(h_tmp_data, j_tmp_data, t_2, J_2);
    coder::internal::useConstantDim(t_2);
    r.set_size(1, t_2.size(1) + 1);
    r[0] = 0.0;
    b = t_2.size(1);
    if ((1 <= t_2.size(1)) && (t_2.size(1) > 2147483646)) {
      coder::check_forloop_overflow_error();
    }
    for (j = 0; j < b; j++) {
      r[j + 1] = t_2[j];
    }
    b = r.size(1);
    j = r.size(1);
    for (i = 0; i < j; i++) {
      e_tmp_data[i] = r[i];
    }
    if (0 <= b - 1) {
      std::copy(&e_tmp_data[0], &e_tmp_data[b], &f_tmp_data[0]);
    }
    J_setp_struct_time.set_size(b, 1);
    for (i = 0; i < b; i++) {
      J_setp_struct_time[i] = f_tmp_data[i];
    }
    b_expl_temp_values.set_size(1, J_2.size(1));
    j = J_2.size(1);
    for (i = 0; i < j; i++) {
      b_expl_temp_values[i] = J_2[i];
    }
    b_expl_temp_values.set_size(1, J_2.size(1) + 1);
    b_expl_temp_values[J_2.size(1)] = 0.0;
    c_expl_temp_values.set_size(1, b_expl_temp_values.size(1));
    j = b_expl_temp_values.size(1);
    for (i = 0; i < j; i++) {
      c_expl_temp_values[i] = b_expl_temp_values[i];
    }
    evaluate_to_time(P_init, V_init, A_init, J_setp_struct_time,
                     c_expl_temp_values, &b_P_init, &b_V_init, &b_A_init);
    b_solve_O(b_P_init, b_V_init, b_A_init, V_min, V_max, A_max, A_min, J_max,
              J_min, t_test_data, t_test_size, J_test_data, J_test_size,
              a__4_data, &b);
    // V_min = -Inf wegen check, V_max muss bleiben für Prädiktion und
    // 3-phasiges t
    coder::internal::indexShapeCheck(t_test_size);
    b = t_test_size[0] * 7;
    if (1 > b) {
      rtDynamicBoundsError(1, 1, b, &tb_emlrtBCI);
    }
    t[1] = t_test_data[0] + t_test1_idx_0;
    if (2 > b) {
      rtDynamicBoundsError(2, 1, 1, &tb_emlrtBCI);
    }
    t[2] = t_test_data[1];
    if (3 > b) {
      rtDynamicBoundsError(3, 1, 2, &tb_emlrtBCI);
    }
    t[3] = t_test_data[2];
    b = J_test_size[0] * 7;
    if (1 > b) {
      rtDynamicBoundsError(1, 1, b, &ub_emlrtBCI);
    }
    J[1] = J_test_data[0];
    if (2 > b) {
      rtDynamicBoundsError(2, 1, 1, &ub_emlrtBCI);
    }
    J[2] = J_test_data[1];
    if (3 > b) {
      rtDynamicBoundsError(3, 1, 2, &ub_emlrtBCI);
    }
    J[3] = J_test_data[2];
  }
}

// End of code generation (correct_feasibility.cpp)
