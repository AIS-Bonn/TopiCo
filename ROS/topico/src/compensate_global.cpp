//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// compensate_global.cpp
//
// Code generation for function 'compensate_global'
//

// Include files
#include "compensate_global.h"
#include "repmat.h"
#include "rt_nonfinite.h"
#include "topico_wrapper_rtwutil.h"
#include "topico_wrapper_types.h"
#include "coder_array.h"

// Function Definitions
void compensate_global(coder::array<double, 2U> &State_in,
                       const coder::array<double, 1U> &A_global,
                       const coder::array<double, 2U> &A_max_in,
                       const coder::array<double, 2U> &A_min_in,
                       coder::array<double, 2U> &A_max_out,
                       coder::array<double, 2U> &A_min_out)
{
  static rtEqualityCheckInfo c_emlrtECI = {
      -1,                                                      // nDims
      45,                                                      // lineNo
      22,                                                      // colNo
      "compensate_global",                                     // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/compensate_global.m" // pName
  };
  static rtEqualityCheckInfo d_emlrtECI = {
      -1,                                                      // nDims
      45,                                                      // lineNo
      5,                                                       // colNo
      "compensate_global",                                     // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/compensate_global.m" // pName
  };
  static rtEqualityCheckInfo e_emlrtECI = {
      2,                                                       // nDims
      46,                                                      // lineNo
      17,                                                      // colNo
      "compensate_global",                                     // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/compensate_global.m" // pName
  };
  static rtEqualityCheckInfo f_emlrtECI = {
      2,                                                       // nDims
      47,                                                      // lineNo
      17,                                                      // colNo
      "compensate_global",                                     // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/compensate_global.m" // pName
  };
  coder::array<double, 1U> c_State_in;
  int iv[2];
  int iv1[2];
  int b_State_in;
  int i;
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
  i = State_in.size(0);
  if (i != A_global.size(0)) {
    rtSizeEq1DError(i, A_global.size(0), &c_emlrtECI);
  }
  rtSubAssignSizeCheck(State_in.size(), 1, State_in.size(), 1, &d_emlrtECI);
  b_State_in = State_in.size(0) - 1;
  c_State_in.set_size(b_State_in + 1);
  for (i = 0; i <= b_State_in; i++) {
    c_State_in[i] = State_in[i + State_in.size(0) * 2] + A_global[i];
  }
  b_State_in = c_State_in.size(0);
  for (i = 0; i < b_State_in; i++) {
    State_in[i + State_in.size(0) * 2] = c_State_in[i];
  }
  coder::repmat(A_global, static_cast<double>(A_max_in.size(1)), A_max_out);
  iv[0] = (*(int(*)[2])((coder::array<double, 2U> *)&A_max_in)->size())[0];
  iv[1] = (*(int(*)[2])((coder::array<double, 2U> *)&A_max_in)->size())[1];
  iv1[0] = (*(int(*)[2])A_max_out.size())[0];
  iv1[1] = (*(int(*)[2])A_max_out.size())[1];
  rtSizeEqNDCheck(&iv[0], &iv1[0], &e_emlrtECI);
  b_State_in = A_max_in.size(0) * A_max_in.size(1);
  A_max_out.set_size(A_max_in.size(0), A_max_in.size(1));
  for (i = 0; i < b_State_in; i++) {
    A_max_out[i] = A_max_in[i] + A_max_out[i];
  }
  coder::repmat(A_global, static_cast<double>(A_max_in.size(1)), A_min_out);
  iv[0] = (*(int(*)[2])((coder::array<double, 2U> *)&A_min_in)->size())[0];
  iv[1] = (*(int(*)[2])((coder::array<double, 2U> *)&A_min_in)->size())[1];
  iv1[0] = (*(int(*)[2])A_min_out.size())[0];
  iv1[1] = (*(int(*)[2])A_min_out.size())[1];
  rtSizeEqNDCheck(&iv[0], &iv1[0], &f_emlrtECI);
  b_State_in = A_min_in.size(0) * A_min_in.size(1);
  A_min_out.set_size(A_min_in.size(0), A_min_in.size(1));
  for (i = 0; i < b_State_in; i++) {
    A_min_out[i] = A_min_in[i] + A_min_out[i];
  }
}

// End of code generation (compensate_global.cpp)
