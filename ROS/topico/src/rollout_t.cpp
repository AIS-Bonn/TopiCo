//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// rollout_t.cpp
//
// Code generation for function 'rollout_t'
//

// Include files
#include "rollout_t.h"
#include "eml_int_forloop_overflow_check.h"
#include "evaluate_to_time.h"
#include "printint.h"
#include "rt_nonfinite.h"
#include "topico_wrapper_internal_types.h"
#include "topico_wrapper_rtwutil.h"
#include "topico_wrapper_types.h"
#include "coder_array.h"
#include "rt_nonfinite.h"
#include <cmath>
#include <sstream>
#include <stdexcept>
#include <stdio.h>
#include <string>

// Function Declarations
static unsigned int _u32_d_(double b);

static unsigned int _u32_minus__(unsigned int b, unsigned int c);

static void q_rtErrorWithMessageID(const char *aFcnName, int aLineNum);

static void rtIntegerError(const double aInteger,
                           const rtDoubleCheckInfo *aInfo);

// Function Definitions
static unsigned int _u32_d_(double b)
{
  unsigned int a;
  a = static_cast<unsigned int>(b);
  if ((b < 0.0) || (a != std::floor(b))) {
    rtIntegerOverflowErrorN();
  }
  return a;
}

static unsigned int _u32_minus__(unsigned int b, unsigned int c)
{
  unsigned int a;
  a = b - c;
  if (b < c) {
    rtIntegerOverflowErrorN();
  }
  return a;
}

static void q_rtErrorWithMessageID(const char *aFcnName, int aLineNum)
{
  std::stringstream outStream;
  outStream << "Maximum variable size allowed by the program is exceeded.";
  outStream << "\n";
  ((((outStream << "Error in ") << aFcnName) << " (line ") << aLineNum) << ")";
  throw std::runtime_error(outStream.str());
}

static void rtIntegerError(const double aInteger,
                           const rtDoubleCheckInfo *aInfo)
{
  std::stringstream outStream;
  ((outStream
    << "Expected a value representable in the C type \'int\'.  Found ")
   << aInteger)
      << " instead.";
  outStream << "\n";
  ((((outStream << "Error in ") << aInfo->fName) << " (line ") << aInfo->lineNo)
      << ")";
  throw std::runtime_error(outStream.str());
}

void rollout_t(const coder::array<double, 1U> &P_init,
               const coder::array<double, 1U> &V_init,
               const coder::array<double, 1U> &A_init,
               const coder::array<struct0_T, 2U> &J_setp_struct, double ts,
               coder::array<double, 2U> &P, coder::array<double, 2U> &V,
               coder::array<double, 2U> &A, coder::array<double, 2U> &J,
               coder::array<double, 2U> &t)
{
  static rtBoundsCheckInfo nb_emlrtBCI = {
      -1,                                               // iFirst
      -1,                                               // iLast
      47,                                               // lineNo
      37,                                               // colNo
      "J_setp_struct",                                  // aName
      "rollout_t",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/rollout_t.m", // pName
      0                                                 // checkKind
  };
  static rtBoundsCheckInfo ob_emlrtBCI = {
      -1,                                               // iFirst
      -1,                                               // iLast
      47,                                               // lineNo
      54,                                               // colNo
      "J_setp_struct(index_axis).time",                 // aName
      "rollout_t",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/rollout_t.m", // pName
      0                                                 // checkKind
  };
  static rtBoundsCheckInfo pb_emlrtBCI = {
      -1,                                               // iFirst
      -1,                                               // iLast
      78,                                               // lineNo
      239,                                              // colNo
      "t",                                              // aName
      "rollout_t",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/rollout_t.m", // pName
      0                                                 // checkKind
  };
  static rtBoundsCheckInfo qb_emlrtBCI = {
      -1,                                               // iFirst
      -1,                                               // iLast
      78,                                               // lineNo
      211,                                              // colNo
      "J_setp_struct",                                  // aName
      "rollout_t",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/rollout_t.m", // pName
      0                                                 // checkKind
  };
  static rtBoundsCheckInfo rb_emlrtBCI = {
      -1,                                               // iFirst
      -1,                                               // iLast
      78,                                               // lineNo
      104,                                              // colNo
      "J",                                              // aName
      "rollout_t",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/rollout_t.m", // pName
      0                                                 // checkKind
  };
  static rtBoundsCheckInfo sb_emlrtBCI = {
      -1,                                               // iFirst
      -1,                                               // iLast
      78,                                               // lineNo
      74,                                               // colNo
      "A",                                              // aName
      "rollout_t",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/rollout_t.m", // pName
      0                                                 // checkKind
  };
  static rtBoundsCheckInfo tb_emlrtBCI = {
      -1,                                               // iFirst
      -1,                                               // iLast
      78,                                               // lineNo
      44,                                               // colNo
      "V",                                              // aName
      "rollout_t",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/rollout_t.m", // pName
      0                                                 // checkKind
  };
  static rtBoundsCheckInfo ub_emlrtBCI = {
      -1,                                               // iFirst
      -1,                                               // iLast
      78,                                               // lineNo
      14,                                               // colNo
      "P",                                              // aName
      "rollout_t",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/rollout_t.m", // pName
      0                                                 // checkKind
  };
  static rtBoundsCheckInfo vb_emlrtBCI = {
      -1,                                               // iFirst
      -1,                                               // iLast
      78,                                               // lineNo
      237,                                              // colNo
      "t",                                              // aName
      "rollout_t",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/rollout_t.m", // pName
      0                                                 // checkKind
  };
  static rtBoundsCheckInfo wb_emlrtBCI = {
      -1,                                               // iFirst
      -1,                                               // iLast
      78,                                               // lineNo
      192,                                              // colNo
      "A_init",                                         // aName
      "rollout_t",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/rollout_t.m", // pName
      0                                                 // checkKind
  };
  static rtBoundsCheckInfo xb_emlrtBCI = {
      -1,                                               // iFirst
      -1,                                               // iLast
      78,                                               // lineNo
      173,                                              // colNo
      "V_init",                                         // aName
      "rollout_t",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/rollout_t.m", // pName
      0                                                 // checkKind
  };
  static rtBoundsCheckInfo yb_emlrtBCI = {
      -1,                                               // iFirst
      -1,                                               // iLast
      78,                                               // lineNo
      154,                                              // colNo
      "P_init",                                         // aName
      "rollout_t",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/rollout_t.m", // pName
      0                                                 // checkKind
  };
  static rtDoubleCheckInfo b_emlrtDCI = {
      71,                                               // lineNo
      24,                                               // colNo
      "rollout_t",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/rollout_t.m", // pName
      1                                                 // checkKind
  };
  static rtDoubleCheckInfo c_emlrtDCI = {
      72,                                               // lineNo
      24,                                               // colNo
      "rollout_t",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/rollout_t.m", // pName
      1                                                 // checkKind
  };
  static rtDoubleCheckInfo d_emlrtDCI = {
      73,                                               // lineNo
      24,                                               // colNo
      "rollout_t",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/rollout_t.m", // pName
      1                                                 // checkKind
  };
  static rtDoubleCheckInfo emlrtDCI = {
      70,                                               // lineNo
      24,                                               // colNo
      "rollout_t",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/rollout_t.m", // pName
      1                                                 // checkKind
  };
  static rtRunTimeErrorInfo s_emlrtRTEI = {
      417,                                                          // lineNo
      15,                                                           // colNo
      "assert_pmaxsize",                                            // fName
      "/usr/local/MATLAB/R2021a/toolbox/eml/lib/matlab/ops/colon.m" // pName
  };
  coder::array<double, 2U> y;
  double T;
  double apnd;
  double cdiff;
  double ndbl;
  int ibmat;
  int jcol;
  int ncols;
  int nm1d2;
  unsigned int num_iterations;
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
  T = 0.0;
  jcol = J_setp_struct.size(1);
  for (ibmat = 0; ibmat < jcol; ibmat++) {
    if (ibmat + 1 > J_setp_struct.size(1)) {
      rtDynamicBoundsError(ibmat + 1, 1, J_setp_struct.size(1), &nb_emlrtBCI);
    }
    nm1d2 = J_setp_struct[ibmat].time.size(0);
    ncols = J_setp_struct[ibmat].time.size(0);
    if ((ncols < 1) || (ncols > nm1d2)) {
      rtDynamicBoundsError(ncols, 1, nm1d2, &ob_emlrtBCI);
    }
    if ((!(T >
           J_setp_struct[ibmat].time[J_setp_struct[ibmat].time.size(0) - 1])) &&
        (!rtIsNaN(J_setp_struct[ibmat]
                      .time[J_setp_struct[ibmat].time.size(0) - 1]))) {
      T = J_setp_struct[ibmat].time[J_setp_struct[ibmat].time.size(0) - 1];
    }
  }
  if (ts < 0.001) {
    printf("Warning: Very small ts!\n");
    fflush(stdout);
  }
  num_iterations = _u32_d_(std::ceil(T / ts) + 1.0);
  // Better to make one iteration too much -> Trajectory rollout never ends
  // before reaching final waypoint
  if (rtIsInf(ts)) {
    num_iterations = 1U;
    ts = 1.0;
  }
  if (num_iterations > 10000U) {
    printf("Warning: Very many rollout iterations (");
    fflush(stdout);
    printint(num_iterations);
    printf(" > ");
    fflush(stdout);
    printint(10000U);
    printf(")!\n");
    fflush(stdout);
    num_iterations = static_cast<unsigned int>(
        rt_roundd_snf(static_cast<double>(num_iterations) * 10000.0 /
                      static_cast<double>(num_iterations)));
  }
  P.set_size(J_setp_struct.size(1), P.size(1));
  if (static_cast<double>(num_iterations) != static_cast<int>(num_iterations)) {
    rtIntegerError(static_cast<double>(num_iterations), &emlrtDCI);
  }
  P.set_size(P.size(0), static_cast<int>(num_iterations));
  V.set_size(J_setp_struct.size(1), V.size(1));
  if (static_cast<double>(num_iterations) != static_cast<int>(num_iterations)) {
    rtIntegerError(static_cast<double>(num_iterations), &b_emlrtDCI);
  }
  V.set_size(V.size(0), static_cast<int>(num_iterations));
  A.set_size(J_setp_struct.size(1), A.size(1));
  if (static_cast<double>(num_iterations) != static_cast<int>(num_iterations)) {
    rtIntegerError(static_cast<double>(num_iterations), &c_emlrtDCI);
  }
  A.set_size(A.size(0), static_cast<int>(num_iterations));
  J.set_size(J_setp_struct.size(1), J.size(1));
  if (static_cast<double>(num_iterations) != static_cast<int>(num_iterations)) {
    rtIntegerError(static_cast<double>(num_iterations), &d_emlrtDCI);
  }
  J.set_size(J.size(0), static_cast<int>(num_iterations));
  T = ts * static_cast<double>(_u32_minus__(num_iterations, 1U));
  if (rtIsNaN(ts) || rtIsNaN(T)) {
    y.set_size(1, 1);
    y[0] = rtNaN;
  } else if ((ts == 0.0) || ((0.0 < T) && (ts < 0.0)) ||
             ((T < 0.0) && (ts > 0.0))) {
    y.set_size(1, 0);
  } else if (rtIsInf(T) && (rtIsInf(ts) || (0.0 == T))) {
    y.set_size(1, 1);
    y[0] = rtNaN;
  } else if (rtIsInf(ts)) {
    y.set_size(1, 1);
    y[0] = 0.0;
  } else if (std::floor(ts) == ts) {
    ncols = static_cast<int>(std::floor(T / ts));
    y.set_size(1, ncols + 1);
    for (jcol = 0; jcol <= ncols; jcol++) {
      y[jcol] = ts * static_cast<double>(jcol);
    }
  } else {
    ndbl = std::floor(T / ts + 0.5);
    apnd = ndbl * ts;
    if (ts > 0.0) {
      cdiff = apnd - T;
    } else {
      cdiff = T - apnd;
    }
    if (std::abs(cdiff) < 4.4408920985006262E-16 * std::abs(T)) {
      ndbl++;
      apnd = T;
    } else if (cdiff > 0.0) {
      apnd = (ndbl - 1.0) * ts;
    } else {
      ndbl++;
    }
    if (ndbl >= 0.0) {
      ncols = static_cast<int>(ndbl);
    } else {
      ncols = 0;
    }
    if (ndbl > 2.147483647E+9) {
      q_rtErrorWithMessageID(s_emlrtRTEI.fName, s_emlrtRTEI.lineNo);
    }
    y.set_size(1, ncols);
    if (ncols > 0) {
      y[0] = 0.0;
      if (ncols > 1) {
        y[ncols - 1] = apnd;
        nm1d2 = (ncols - 1) / 2;
        for (ibmat = 0; ibmat <= nm1d2 - 2; ibmat++) {
          T = (static_cast<double>(ibmat) + 1.0) * ts;
          y[ibmat + 1] = T;
          y[(ncols - ibmat) - 2] = apnd - T;
        }
        if (nm1d2 << 1 == ncols - 1) {
          y[nm1d2] = apnd / 2.0;
        } else {
          T = static_cast<double>(nm1d2) * ts;
          y[nm1d2] = T;
          y[nm1d2 + 1] = apnd - T;
        }
      }
    }
  }
  t.set_size(J_setp_struct.size(1), y.size(1));
  ncols = y.size(1);
  nm1d2 = J_setp_struct.size(1);
  if ((1 <= y.size(1)) && (y.size(1) > 2147483646)) {
    coder::check_forloop_overflow_error();
  }
  for (jcol = 0; jcol < ncols; jcol++) {
    ibmat = jcol * nm1d2;
    if ((1 <= nm1d2) && (nm1d2 > 2147483646)) {
      coder::check_forloop_overflow_error();
    }
    for (int itilerow = 0; itilerow < nm1d2; itilerow++) {
      t[ibmat + itilerow] = y[jcol];
    }
  }
  jcol = J_setp_struct.size(1);
  for (ibmat = 0; ibmat < jcol; ibmat++) {
    nm1d2 = static_cast<int>(num_iterations);
    for (ncols = 0; ncols < nm1d2; ncols++) {
      if (1 > t.size(0)) {
        rtDynamicBoundsError(1, 1, t.size(0), &pb_emlrtBCI);
      }
      if (ibmat + 1 > J_setp_struct.size(1)) {
        rtDynamicBoundsError(ibmat + 1, 1, J_setp_struct.size(1), &qb_emlrtBCI);
      }
      if ((static_cast<int>(ncols + 1U) < 1) ||
          (static_cast<int>(ncols + 1U) > J.size(1))) {
        rtDynamicBoundsError(static_cast<int>(ncols + 1U), 1, J.size(1),
                             &rb_emlrtBCI);
      }
      if (ibmat + 1 > J.size(0)) {
        rtDynamicBoundsError(ibmat + 1, 1, J.size(0), &rb_emlrtBCI);
      }
      if ((static_cast<int>(ncols + 1U) < 1) ||
          (static_cast<int>(ncols + 1U) > A.size(1))) {
        rtDynamicBoundsError(static_cast<int>(ncols + 1U), 1, A.size(1),
                             &sb_emlrtBCI);
      }
      if (ibmat + 1 > A.size(0)) {
        rtDynamicBoundsError(ibmat + 1, 1, A.size(0), &sb_emlrtBCI);
      }
      if ((static_cast<int>(ncols + 1U) < 1) ||
          (static_cast<int>(ncols + 1U) > V.size(1))) {
        rtDynamicBoundsError(static_cast<int>(ncols + 1U), 1, V.size(1),
                             &tb_emlrtBCI);
      }
      if (ibmat + 1 > V.size(0)) {
        rtDynamicBoundsError(ibmat + 1, 1, V.size(0), &tb_emlrtBCI);
      }
      if ((static_cast<int>(ncols + 1U) < 1) ||
          (static_cast<int>(ncols + 1U) > P.size(1))) {
        rtDynamicBoundsError(static_cast<int>(ncols + 1U), 1, P.size(1),
                             &ub_emlrtBCI);
      }
      if (ibmat + 1 > P.size(0)) {
        rtDynamicBoundsError(ibmat + 1, 1, P.size(0), &ub_emlrtBCI);
      }
      if ((static_cast<int>(ncols + 1U) < 1) ||
          (static_cast<int>(ncols + 1U) > t.size(1))) {
        rtDynamicBoundsError(static_cast<int>(ncols + 1U), 1, t.size(1),
                             &vb_emlrtBCI);
      }
      if (ibmat + 1 > A_init.size(0)) {
        rtDynamicBoundsError(ibmat + 1, 1, A_init.size(0), &wb_emlrtBCI);
      }
      if (ibmat + 1 > V_init.size(0)) {
        rtDynamicBoundsError(ibmat + 1, 1, V_init.size(0), &xb_emlrtBCI);
      }
      if (ibmat + 1 > P_init.size(0)) {
        rtDynamicBoundsError(ibmat + 1, 1, P_init.size(0), &yb_emlrtBCI);
      }
      evaluate_to_time(P_init[ibmat], V_init[ibmat], A_init[ibmat],
                       J_setp_struct[ibmat].time,
                       J_setp_struct[ibmat].signals.values,
                       t[t.size(0) * ncols], &T, &ndbl, &apnd, &cdiff);
      J[ibmat + J.size(0) * ncols] = cdiff;
      A[ibmat + A.size(0) * ncols] = apnd;
      V[ibmat + V.size(0) * ncols] = ndbl;
      P[ibmat + P.size(0) * ncols] = T;
    }
  }
}

// End of code generation (rollout_t.cpp)
