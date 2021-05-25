//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// evaluate_to_time.cpp
//
// Code generation for function 'evaluate_to_time'
//

// Include files
#include "evaluate_to_time.h"
#include "cut_to_time.h"
#include "diff.h"
#include "rt_nonfinite.h"
#include "topico_wrapper_internal_types.h"
#include "topico_wrapper_rtwutil.h"
#include "topico_wrapper_types.h"
#include "coder_array.h"

// Variable Definitions
static rtBoundsCheckInfo k_emlrtBCI = {
    -1,                                              // iFirst
    -1,                                              // iLast
    53,                                              // lineNo
    9,                                               // colNo
    "A",                                             // aName
    "evaluate",                                      // fName
    "/home/lmbeul/Desktop/TopiCo/MATLAB/evaluate.m", // pName
    0                                                // checkKind
};

static rtBoundsCheckInfo l_emlrtBCI = {
    -1,                                              // iFirst
    -1,                                              // iLast
    53,                                              // lineNo
    31,                                              // colNo
    "A",                                             // aName
    "evaluate",                                      // fName
    "/home/lmbeul/Desktop/TopiCo/MATLAB/evaluate.m", // pName
    0                                                // checkKind
};

static rtBoundsCheckInfo m_emlrtBCI = {
    -1,                                              // iFirst
    -1,                                              // iLast
    52,                                              // lineNo
    9,                                               // colNo
    "V",                                             // aName
    "evaluate",                                      // fName
    "/home/lmbeul/Desktop/TopiCo/MATLAB/evaluate.m", // pName
    0                                                // checkKind
};

static rtBoundsCheckInfo n_emlrtBCI = {
    -1,                                              // iFirst
    -1,                                              // iLast
    52,                                              // lineNo
    65,                                              // colNo
    "A",                                             // aName
    "evaluate",                                      // fName
    "/home/lmbeul/Desktop/TopiCo/MATLAB/evaluate.m", // pName
    0                                                // checkKind
};

static rtBoundsCheckInfo o_emlrtBCI = {
    -1,                                              // iFirst
    -1,                                              // iLast
    52,                                              // lineNo
    31,                                              // colNo
    "V",                                             // aName
    "evaluate",                                      // fName
    "/home/lmbeul/Desktop/TopiCo/MATLAB/evaluate.m", // pName
    0                                                // checkKind
};

static rtBoundsCheckInfo p_emlrtBCI = {
    -1,                                              // iFirst
    -1,                                              // iLast
    51,                                              // lineNo
    9,                                               // colNo
    "P",                                             // aName
    "evaluate",                                      // fName
    "/home/lmbeul/Desktop/TopiCo/MATLAB/evaluate.m", // pName
    0                                                // checkKind
};

static rtBoundsCheckInfo q_emlrtBCI = {
    -1,                                              // iFirst
    -1,                                              // iLast
    51,                                              // lineNo
    108,                                             // colNo
    "A",                                             // aName
    "evaluate",                                      // fName
    "/home/lmbeul/Desktop/TopiCo/MATLAB/evaluate.m", // pName
    0                                                // checkKind
};

static rtBoundsCheckInfo r_emlrtBCI = {
    -1,                                              // iFirst
    -1,                                              // iLast
    51,                                              // lineNo
    65,                                              // colNo
    "V",                                             // aName
    "evaluate",                                      // fName
    "/home/lmbeul/Desktop/TopiCo/MATLAB/evaluate.m", // pName
    0                                                // checkKind
};

static rtBoundsCheckInfo s_emlrtBCI = {
    -1,                                              // iFirst
    -1,                                              // iLast
    51,                                              // lineNo
    31,                                              // colNo
    "P",                                             // aName
    "evaluate",                                      // fName
    "/home/lmbeul/Desktop/TopiCo/MATLAB/evaluate.m", // pName
    0                                                // checkKind
};

static rtBoundsCheckInfo t_emlrtBCI = {
    -1,                                                      // iFirst
    -1,                                                      // iLast
    71,                                                      // lineNo
    27,                                                      // colNo
    "A_eval",                                                // aName
    "evaluate_to_time",                                      // fName
    "/home/lmbeul/Desktop/TopiCo/MATLAB/evaluate_to_time.m", // pName
    0                                                        // checkKind
};

static rtBoundsCheckInfo u_emlrtBCI = {
    -1,                                              // iFirst
    -1,                                              // iLast
    50,                                              // lineNo
    18,                                              // colNo
    "J",                                             // aName
    "evaluate",                                      // fName
    "/home/lmbeul/Desktop/TopiCo/MATLAB/evaluate.m", // pName
    0                                                // checkKind
};

static rtBoundsCheckInfo v_emlrtBCI = {
    -1,                                                      // iFirst
    -1,                                                      // iLast
    70,                                                      // lineNo
    27,                                                      // colNo
    "V_eval",                                                // aName
    "evaluate_to_time",                                      // fName
    "/home/lmbeul/Desktop/TopiCo/MATLAB/evaluate_to_time.m", // pName
    0                                                        // checkKind
};

static rtBoundsCheckInfo w_emlrtBCI = {
    -1,                                                      // iFirst
    -1,                                                      // iLast
    69,                                                      // lineNo
    27,                                                      // colNo
    "P_eval",                                                // aName
    "evaluate_to_time",                                      // fName
    "/home/lmbeul/Desktop/TopiCo/MATLAB/evaluate_to_time.m", // pName
    0                                                        // checkKind
};

static rtBoundsCheckInfo x_emlrtBCI = {
    -1,                                                      // iFirst
    -1,                                                      // iLast
    47,                                                      // lineNo
    63,                                                      // colNo
    "J_setp_struct(index_axis).time",                        // aName
    "evaluate_to_time",                                      // fName
    "/home/lmbeul/Desktop/TopiCo/MATLAB/evaluate_to_time.m", // pName
    0                                                        // checkKind
};

static rtBoundsCheckInfo y_emlrtBCI = {
    -1,                                                          // iFirst
    -1,                                                          // iLast
    48,                                                          // lineNo
    72,                                                          // colNo
    "J_setp_struct(index_axis).signals.values",                  // aName
    "destruct_setp_struct",                                      // fName
    "/home/lmbeul/Desktop/TopiCo/MATLAB/destruct_setp_struct.m", // pName
    0                                                            // checkKind
};

static rtBoundsCheckInfo ab_emlrtBCI = {
    -1,                                              // iFirst
    -1,                                              // iLast
    49,                                              // lineNo
    20,                                              // colNo
    "t",                                             // aName
    "evaluate",                                      // fName
    "/home/lmbeul/Desktop/TopiCo/MATLAB/evaluate.m", // pName
    0                                                // checkKind
};

static rtBoundsCheckInfo mb_emlrtBCI = {
    -1,                                                      // iFirst
    -1,                                                      // iLast
    72,                                                      // lineNo
    27,                                                      // colNo
    "J_remaining{index_axis}",                               // aName
    "evaluate_to_time",                                      // fName
    "/home/lmbeul/Desktop/TopiCo/MATLAB/evaluate_to_time.m", // pName
    0                                                        // checkKind
};

// Function Definitions
void evaluate_to_time(
    double P_init, double V_init, double A_init,
    const coder::array<double, 2U> &J_setp_struct_time,
    const coder::array<double, 2U> &J_setp_struct_signals_values, double T_in,
    double *P, double *V, double *A, double *J)
{
  coder::array<double, 2U> A_eval;
  coder::array<double, 2U> P_eval;
  coder::array<double, 2U> V_eval;
  coder::array<double, 1U> b_J_setp_struct_time;
  coder::array<double, 1U> r;
  cell_wrap_17 J_remaining;
  cell_wrap_17 J_temp;
  cell_wrap_17 t_remaining;
  cell_wrap_17 t_temp;
  double J_curr;
  double a;
  int i;
  int loop_ub;
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
  loop_ub = J_setp_struct_time.size(0);
  b_J_setp_struct_time.set_size(J_setp_struct_time.size(0));
  for (i = 0; i < loop_ub; i++) {
    b_J_setp_struct_time[i] = J_setp_struct_time[i];
  }
  coder::diff(b_J_setp_struct_time, r);
  t_temp.f1.set_size(1, r.size(0));
  loop_ub = r.size(0);
  for (i = 0; i < loop_ub; i++) {
    t_temp.f1[i] = r[i];
  }
  if (1 > J_setp_struct_signals_values.size(1) - 1) {
    loop_ub = 0;
  } else {
    if ((J_setp_struct_signals_values.size(1) - 1 < 1) ||
        (J_setp_struct_signals_values.size(1) - 1 >
         J_setp_struct_signals_values.size(1))) {
      rtDynamicBoundsError(J_setp_struct_signals_values.size(1) - 1, 1,
                           J_setp_struct_signals_values.size(1), &y_emlrtBCI);
    }
    loop_ub = J_setp_struct_signals_values.size(1) - 1;
  }
  J_temp.f1.set_size(1, loop_ub);
  for (i = 0; i < loop_ub; i++) {
    J_temp.f1[i] = J_setp_struct_signals_values[i];
  }
  cut_to_time(&t_temp, &J_temp, T_in, &t_remaining, &J_remaining);
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
  J_curr = 0.5 * (t_remaining.f1[0] * t_remaining.f1[0]);
  a = ((P_init + t_remaining.f1[0] * V_init) + J_curr * A_init) +
      0.16666666666666666 * rt_powd_snf(t_remaining.f1[0], 3.0) *
          J_remaining.f1[0];
  P_eval.set_size(1, t_remaining.f1.size(1));
  loop_ub = t_remaining.f1.size(1);
  for (i = 0; i < loop_ub; i++) {
    P_eval[i] = a;
  }
  a = (V_init + t_remaining.f1[0] * A_init) + J_curr * J_remaining.f1[0];
  V_eval.set_size(1, t_remaining.f1.size(1));
  loop_ub = t_remaining.f1.size(1);
  for (i = 0; i < loop_ub; i++) {
    V_eval[i] = a;
  }
  a = A_init + t_remaining.f1[0] * J_remaining.f1[0];
  A_eval.set_size(1, t_remaining.f1.size(1));
  loop_ub = t_remaining.f1.size(1);
  for (i = 0; i < loop_ub; i++) {
    A_eval[i] = a;
  }
  i = t_remaining.f1.size(1);
  for (loop_ub = 0; loop_ub <= i - 2; loop_ub++) {
    double P_eval_tmp;
    if (loop_ub + 2 > t_remaining.f1.size(1)) {
      rtDynamicBoundsError(loop_ub + 2, 1, t_remaining.f1.size(1),
                           &ab_emlrtBCI);
    }
    if (loop_ub + 2 > J_remaining.f1.size(1)) {
      rtDynamicBoundsError(loop_ub + 2, 1, J_remaining.f1.size(1), &u_emlrtBCI);
    }
    J_curr = J_remaining.f1[loop_ub + 1];
    if (loop_ub + 2 > P_eval.size(1)) {
      rtDynamicBoundsError(loop_ub + 2, 1, P_eval.size(1), &p_emlrtBCI);
    }
    if (loop_ub + 1 > A_eval.size(1)) {
      rtDynamicBoundsError(loop_ub + 1, 1, A_eval.size(1), &q_emlrtBCI);
    }
    if (loop_ub + 1 > V_eval.size(1)) {
      rtDynamicBoundsError(loop_ub + 1, 1, V_eval.size(1), &r_emlrtBCI);
    }
    if (loop_ub + 1 > P_eval.size(1)) {
      rtDynamicBoundsError(loop_ub + 1, 1, P_eval.size(1), &s_emlrtBCI);
    }
    a = t_remaining.f1[loop_ub + 1];
    P_eval_tmp = 0.5 * (a * a);
    P_eval[loop_ub + 1] = ((P_eval[loop_ub] + a * V_eval[loop_ub]) +
                           P_eval_tmp * A_eval[loop_ub]) +
                          0.16666666666666666 * rt_powd_snf(a, 3.0) * J_curr;
    if (loop_ub + 2 > V_eval.size(1)) {
      rtDynamicBoundsError(loop_ub + 2, 1, V_eval.size(1), &m_emlrtBCI);
    }
    if (loop_ub + 1 > A_eval.size(1)) {
      rtDynamicBoundsError(loop_ub + 1, 1, A_eval.size(1), &n_emlrtBCI);
    }
    if (loop_ub + 1 > V_eval.size(1)) {
      rtDynamicBoundsError(loop_ub + 1, 1, V_eval.size(1), &o_emlrtBCI);
    }
    V_eval[loop_ub + 1] =
        (V_eval[loop_ub] + a * A_eval[loop_ub]) + P_eval_tmp * J_curr;
    if (loop_ub + 2 > A_eval.size(1)) {
      rtDynamicBoundsError(loop_ub + 2, 1, A_eval.size(1), &k_emlrtBCI);
    }
    if (loop_ub + 1 > A_eval.size(1)) {
      rtDynamicBoundsError(loop_ub + 1, 1, A_eval.size(1), &l_emlrtBCI);
    }
    A_eval[loop_ub + 1] = A_eval[loop_ub] + a * J_curr;
  }
  if (P_eval.size(1) < 1) {
    rtDynamicBoundsError(P_eval.size(1), 1, P_eval.size(1), &w_emlrtBCI);
  }
  *P = P_eval[P_eval.size(1) - 1];
  if (V_eval.size(1) < 1) {
    rtDynamicBoundsError(V_eval.size(1), 1, V_eval.size(1), &v_emlrtBCI);
  }
  *V = V_eval[V_eval.size(1) - 1];
  if (A_eval.size(1) < 1) {
    rtDynamicBoundsError(A_eval.size(1), 1, A_eval.size(1), &t_emlrtBCI);
  }
  *A = A_eval[A_eval.size(1) - 1];
  if (J_remaining.f1.size(1) < 1) {
    rtDynamicBoundsError(J_remaining.f1.size(1), 1, J_remaining.f1.size(1),
                         &mb_emlrtBCI);
  }
  *J = J_remaining.f1[J_remaining.f1.size(1) - 1];
}

void evaluate_to_time(
    double P_init, double V_init, double A_init,
    const coder::array<double, 2U> &J_setp_struct_time,
    const coder::array<double, 2U> &J_setp_struct_signals_values, double *P,
    double *V, double *A, double *J)
{
  coder::array<double, 2U> A_eval;
  coder::array<double, 2U> P_eval;
  coder::array<double, 2U> V_eval;
  coder::array<double, 1U> b_J_setp_struct_time;
  coder::array<double, 1U> r;
  cell_wrap_17 J_remaining;
  cell_wrap_17 J_temp;
  cell_wrap_17 t_remaining;
  cell_wrap_17 t_temp;
  double J_curr;
  double a;
  int i;
  int loop_ub;
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
  if (J_setp_struct_time.size(0) < 1) {
    rtDynamicBoundsError(J_setp_struct_time.size(0), 1,
                         J_setp_struct_time.size(0), &x_emlrtBCI);
  }
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
  loop_ub = J_setp_struct_time.size(0);
  b_J_setp_struct_time.set_size(J_setp_struct_time.size(0));
  for (i = 0; i < loop_ub; i++) {
    b_J_setp_struct_time[i] = J_setp_struct_time[i];
  }
  coder::diff(b_J_setp_struct_time, r);
  t_temp.f1.set_size(1, r.size(0));
  loop_ub = r.size(0);
  for (i = 0; i < loop_ub; i++) {
    t_temp.f1[i] = r[i];
  }
  if (1 > J_setp_struct_signals_values.size(1) - 1) {
    loop_ub = 0;
  } else {
    if ((J_setp_struct_signals_values.size(1) - 1 < 1) ||
        (J_setp_struct_signals_values.size(1) - 1 >
         J_setp_struct_signals_values.size(1))) {
      rtDynamicBoundsError(J_setp_struct_signals_values.size(1) - 1, 1,
                           J_setp_struct_signals_values.size(1), &y_emlrtBCI);
    }
    loop_ub = J_setp_struct_signals_values.size(1) - 1;
  }
  J_temp.f1.set_size(1, loop_ub);
  for (i = 0; i < loop_ub; i++) {
    J_temp.f1[i] = J_setp_struct_signals_values[i];
  }
  cut_to_time(&t_temp, &J_temp,
              J_setp_struct_time[J_setp_struct_time.size(0) - 1], &t_remaining,
              &J_remaining);
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
  J_curr = 0.5 * (t_remaining.f1[0] * t_remaining.f1[0]);
  a = ((P_init + t_remaining.f1[0] * V_init) + J_curr * A_init) +
      0.16666666666666666 * rt_powd_snf(t_remaining.f1[0], 3.0) *
          J_remaining.f1[0];
  P_eval.set_size(1, t_remaining.f1.size(1));
  loop_ub = t_remaining.f1.size(1);
  for (i = 0; i < loop_ub; i++) {
    P_eval[i] = a;
  }
  a = (V_init + t_remaining.f1[0] * A_init) + J_curr * J_remaining.f1[0];
  V_eval.set_size(1, t_remaining.f1.size(1));
  loop_ub = t_remaining.f1.size(1);
  for (i = 0; i < loop_ub; i++) {
    V_eval[i] = a;
  }
  a = A_init + t_remaining.f1[0] * J_remaining.f1[0];
  A_eval.set_size(1, t_remaining.f1.size(1));
  loop_ub = t_remaining.f1.size(1);
  for (i = 0; i < loop_ub; i++) {
    A_eval[i] = a;
  }
  i = t_remaining.f1.size(1);
  for (loop_ub = 0; loop_ub <= i - 2; loop_ub++) {
    double P_eval_tmp;
    if (loop_ub + 2 > t_remaining.f1.size(1)) {
      rtDynamicBoundsError(loop_ub + 2, 1, t_remaining.f1.size(1),
                           &ab_emlrtBCI);
    }
    if (loop_ub + 2 > J_remaining.f1.size(1)) {
      rtDynamicBoundsError(loop_ub + 2, 1, J_remaining.f1.size(1), &u_emlrtBCI);
    }
    J_curr = J_remaining.f1[loop_ub + 1];
    if (loop_ub + 2 > P_eval.size(1)) {
      rtDynamicBoundsError(loop_ub + 2, 1, P_eval.size(1), &p_emlrtBCI);
    }
    if (loop_ub + 1 > A_eval.size(1)) {
      rtDynamicBoundsError(loop_ub + 1, 1, A_eval.size(1), &q_emlrtBCI);
    }
    if (loop_ub + 1 > V_eval.size(1)) {
      rtDynamicBoundsError(loop_ub + 1, 1, V_eval.size(1), &r_emlrtBCI);
    }
    if (loop_ub + 1 > P_eval.size(1)) {
      rtDynamicBoundsError(loop_ub + 1, 1, P_eval.size(1), &s_emlrtBCI);
    }
    a = t_remaining.f1[loop_ub + 1];
    P_eval_tmp = 0.5 * (a * a);
    P_eval[loop_ub + 1] = ((P_eval[loop_ub] + a * V_eval[loop_ub]) +
                           P_eval_tmp * A_eval[loop_ub]) +
                          0.16666666666666666 * rt_powd_snf(a, 3.0) * J_curr;
    if (loop_ub + 2 > V_eval.size(1)) {
      rtDynamicBoundsError(loop_ub + 2, 1, V_eval.size(1), &m_emlrtBCI);
    }
    if (loop_ub + 1 > A_eval.size(1)) {
      rtDynamicBoundsError(loop_ub + 1, 1, A_eval.size(1), &n_emlrtBCI);
    }
    if (loop_ub + 1 > V_eval.size(1)) {
      rtDynamicBoundsError(loop_ub + 1, 1, V_eval.size(1), &o_emlrtBCI);
    }
    V_eval[loop_ub + 1] =
        (V_eval[loop_ub] + a * A_eval[loop_ub]) + P_eval_tmp * J_curr;
    if (loop_ub + 2 > A_eval.size(1)) {
      rtDynamicBoundsError(loop_ub + 2, 1, A_eval.size(1), &k_emlrtBCI);
    }
    if (loop_ub + 1 > A_eval.size(1)) {
      rtDynamicBoundsError(loop_ub + 1, 1, A_eval.size(1), &l_emlrtBCI);
    }
    A_eval[loop_ub + 1] = A_eval[loop_ub] + a * J_curr;
  }
  if (P_eval.size(1) < 1) {
    rtDynamicBoundsError(P_eval.size(1), 1, P_eval.size(1), &w_emlrtBCI);
  }
  *P = P_eval[P_eval.size(1) - 1];
  if (V_eval.size(1) < 1) {
    rtDynamicBoundsError(V_eval.size(1), 1, V_eval.size(1), &v_emlrtBCI);
  }
  *V = V_eval[V_eval.size(1) - 1];
  if (A_eval.size(1) < 1) {
    rtDynamicBoundsError(A_eval.size(1), 1, A_eval.size(1), &t_emlrtBCI);
  }
  *A = A_eval[A_eval.size(1) - 1];
  if (J_remaining.f1.size(1) < 1) {
    rtDynamicBoundsError(J_remaining.f1.size(1), 1, J_remaining.f1.size(1),
                         &mb_emlrtBCI);
  }
  *J = J_remaining.f1[J_remaining.f1.size(1) - 1];
}

void evaluate_to_time(
    double P_init, double V_init, double A_init,
    const coder::array<double, 2U> &J_setp_struct_time,
    const coder::array<double, 2U> &J_setp_struct_signals_values, double *P,
    double *V, double *A)
{
  static rtBoundsCheckInfo nb_emlrtBCI = {
      -1,                                                      // iFirst
      -1,                                                      // iLast
      72,                                                      // lineNo
      51,                                                      // colNo
      "J_remaining{index_axis}",                               // aName
      "evaluate_to_time",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/evaluate_to_time.m", // pName
      0                                                        // checkKind
  };
  coder::array<double, 2U> A_eval;
  coder::array<double, 2U> P_eval;
  coder::array<double, 2U> V_eval;
  coder::array<double, 1U> b_J_setp_struct_time;
  coder::array<double, 1U> r;
  cell_wrap_17 J_remaining;
  cell_wrap_17 J_temp;
  cell_wrap_17 t_remaining;
  cell_wrap_17 t_temp;
  double J_curr;
  double a;
  int i;
  int loop_ub;
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
  if (J_setp_struct_time.size(0) < 1) {
    rtDynamicBoundsError(J_setp_struct_time.size(0), 1,
                         J_setp_struct_time.size(0), &x_emlrtBCI);
  }
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
  loop_ub = J_setp_struct_time.size(0);
  b_J_setp_struct_time.set_size(J_setp_struct_time.size(0));
  for (i = 0; i < loop_ub; i++) {
    b_J_setp_struct_time[i] = J_setp_struct_time[i];
  }
  coder::diff(b_J_setp_struct_time, r);
  t_temp.f1.set_size(1, r.size(0));
  loop_ub = r.size(0);
  for (i = 0; i < loop_ub; i++) {
    t_temp.f1[i] = r[i];
  }
  if (1 > J_setp_struct_signals_values.size(1) - 1) {
    loop_ub = 0;
  } else {
    if ((J_setp_struct_signals_values.size(1) - 1 < 1) ||
        (J_setp_struct_signals_values.size(1) - 1 >
         J_setp_struct_signals_values.size(1))) {
      rtDynamicBoundsError(J_setp_struct_signals_values.size(1) - 1, 1,
                           J_setp_struct_signals_values.size(1), &y_emlrtBCI);
    }
    loop_ub = J_setp_struct_signals_values.size(1) - 1;
  }
  J_temp.f1.set_size(1, loop_ub);
  for (i = 0; i < loop_ub; i++) {
    J_temp.f1[i] = J_setp_struct_signals_values[i];
  }
  cut_to_time(&t_temp, &J_temp,
              J_setp_struct_time[J_setp_struct_time.size(0) - 1], &t_remaining,
              &J_remaining);
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
  J_curr = 0.5 * (t_remaining.f1[0] * t_remaining.f1[0]);
  a = ((P_init + t_remaining.f1[0] * V_init) + J_curr * A_init) +
      0.16666666666666666 * rt_powd_snf(t_remaining.f1[0], 3.0) *
          J_remaining.f1[0];
  P_eval.set_size(1, t_remaining.f1.size(1));
  loop_ub = t_remaining.f1.size(1);
  for (i = 0; i < loop_ub; i++) {
    P_eval[i] = a;
  }
  a = (V_init + t_remaining.f1[0] * A_init) + J_curr * J_remaining.f1[0];
  V_eval.set_size(1, t_remaining.f1.size(1));
  loop_ub = t_remaining.f1.size(1);
  for (i = 0; i < loop_ub; i++) {
    V_eval[i] = a;
  }
  a = A_init + t_remaining.f1[0] * J_remaining.f1[0];
  A_eval.set_size(1, t_remaining.f1.size(1));
  loop_ub = t_remaining.f1.size(1);
  for (i = 0; i < loop_ub; i++) {
    A_eval[i] = a;
  }
  i = t_remaining.f1.size(1);
  for (loop_ub = 0; loop_ub <= i - 2; loop_ub++) {
    double P_eval_tmp;
    if (loop_ub + 2 > t_remaining.f1.size(1)) {
      rtDynamicBoundsError(loop_ub + 2, 1, t_remaining.f1.size(1),
                           &ab_emlrtBCI);
    }
    if (loop_ub + 2 > J_remaining.f1.size(1)) {
      rtDynamicBoundsError(loop_ub + 2, 1, J_remaining.f1.size(1), &u_emlrtBCI);
    }
    J_curr = J_remaining.f1[loop_ub + 1];
    if (loop_ub + 2 > P_eval.size(1)) {
      rtDynamicBoundsError(loop_ub + 2, 1, P_eval.size(1), &p_emlrtBCI);
    }
    if (loop_ub + 1 > A_eval.size(1)) {
      rtDynamicBoundsError(loop_ub + 1, 1, A_eval.size(1), &q_emlrtBCI);
    }
    if (loop_ub + 1 > V_eval.size(1)) {
      rtDynamicBoundsError(loop_ub + 1, 1, V_eval.size(1), &r_emlrtBCI);
    }
    if (loop_ub + 1 > P_eval.size(1)) {
      rtDynamicBoundsError(loop_ub + 1, 1, P_eval.size(1), &s_emlrtBCI);
    }
    a = t_remaining.f1[loop_ub + 1];
    P_eval_tmp = 0.5 * (a * a);
    P_eval[loop_ub + 1] = ((P_eval[loop_ub] + a * V_eval[loop_ub]) +
                           P_eval_tmp * A_eval[loop_ub]) +
                          0.16666666666666666 * rt_powd_snf(a, 3.0) * J_curr;
    if (loop_ub + 2 > V_eval.size(1)) {
      rtDynamicBoundsError(loop_ub + 2, 1, V_eval.size(1), &m_emlrtBCI);
    }
    if (loop_ub + 1 > A_eval.size(1)) {
      rtDynamicBoundsError(loop_ub + 1, 1, A_eval.size(1), &n_emlrtBCI);
    }
    if (loop_ub + 1 > V_eval.size(1)) {
      rtDynamicBoundsError(loop_ub + 1, 1, V_eval.size(1), &o_emlrtBCI);
    }
    V_eval[loop_ub + 1] =
        (V_eval[loop_ub] + a * A_eval[loop_ub]) + P_eval_tmp * J_curr;
    if (loop_ub + 2 > A_eval.size(1)) {
      rtDynamicBoundsError(loop_ub + 2, 1, A_eval.size(1), &k_emlrtBCI);
    }
    if (loop_ub + 1 > A_eval.size(1)) {
      rtDynamicBoundsError(loop_ub + 1, 1, A_eval.size(1), &l_emlrtBCI);
    }
    A_eval[loop_ub + 1] = A_eval[loop_ub] + a * J_curr;
  }
  if (P_eval.size(1) < 1) {
    rtDynamicBoundsError(P_eval.size(1), 1, P_eval.size(1), &w_emlrtBCI);
  }
  *P = P_eval[P_eval.size(1) - 1];
  if (V_eval.size(1) < 1) {
    rtDynamicBoundsError(V_eval.size(1), 1, V_eval.size(1), &v_emlrtBCI);
  }
  *V = V_eval[V_eval.size(1) - 1];
  if (A_eval.size(1) < 1) {
    rtDynamicBoundsError(A_eval.size(1), 1, A_eval.size(1), &t_emlrtBCI);
  }
  *A = A_eval[A_eval.size(1) - 1];
  if (J_remaining.f1.size(1) < 1) {
    rtDynamicBoundsError(J_remaining.f1.size(1), 1, J_remaining.f1.size(1),
                         &nb_emlrtBCI);
  }
}

// End of code generation (evaluate_to_time.cpp)
