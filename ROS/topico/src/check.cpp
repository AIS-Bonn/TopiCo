//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// check.cpp
//
// Code generation for function 'check'
//

// Include files
#include "check.h"
#include "check_feasibility.h"
#include "rt_nonfinite.h"
#include "topico_wrapper_rtwutil.h"
#include "topico_wrapper_types.h"
#include "rt_nonfinite.h"
#include <cstring>

// Variable Definitions
static rtBoundsCheckInfo ib_emlrtBCI = {
    -1,                                           // iFirst
    -1,                                           // iLast
    65,                                           // lineNo
    27,                                           // colNo
    "t",                                          // aName
    "check",                                      // fName
    "/home/lmbeul/Desktop/TopiCo/MATLAB/check.m", // pName
    0                                             // checkKind
};

static rtBoundsCheckInfo jb_emlrtBCI = {
    -1,                                           // iFirst
    -1,                                           // iLast
    64,                                           // lineNo
    59,                                           // colNo
    "t",                                          // aName
    "check",                                      // fName
    "/home/lmbeul/Desktop/TopiCo/MATLAB/check.m", // pName
    0                                             // checkKind
};

static rtBoundsCheckInfo kb_emlrtBCI = {
    -1,                                           // iFirst
    -1,                                           // iLast
    64,                                           // lineNo
    24,                                           // colNo
    "t",                                          // aName
    "check",                                      // fName
    "/home/lmbeul/Desktop/TopiCo/MATLAB/check.m", // pName
    0                                             // checkKind
};

static rtBoundsCheckInfo lb_emlrtBCI = {
    -1,                                           // iFirst
    -1,                                           // iLast
    82,                                           // lineNo
    25,                                           // colNo
    "valid",                                      // aName
    "check",                                      // fName
    "/home/lmbeul/Desktop/TopiCo/MATLAB/check.m", // pName
    0                                             // checkKind
};

// Function Definitions
void b_check(const creal_T t_data[], const int t_size[2], const double J[7],
             double V_init, double A_init, double V_wayp, double V_max,
             double A_max, double A_min, double J_max, double J_min,
             bool valid_data[], int valid_size[2])
{
  double c_a[7];
  double t[7];
  double v[7];
  int i;
  int loop_ub;
  bool x[7];
  bool b_x[5];
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
  // numerical
  // numerical
  // numerical
  // numerical
  V_max *= 1.0001;
  // numerical
  // numerical
  A_max *= 1.0001;
  // numerical
  A_min *= 1.0001;
  // numerical
  J_max *= 1.0001;
  // numerical
  J_min *= 1.0001;
  // numerical
  // numerical
  // numerical
  valid_size[0] = 1;
  valid_size[1] = t_size[0];
  loop_ub = t_size[0];
  if (0 <= loop_ub - 1) {
    std::memset(&valid_data[0], 0, loop_ub * sizeof(bool));
  }
  i = t_size[0];
  for (int b_index = 0; b_index < i; b_index++) {
    bool exitg1;
    bool y;
    if (b_index + 1 > t_size[0]) {
      rtDynamicBoundsError(b_index + 1, 1, t_size[0], &kb_emlrtBCI);
    }
    for (loop_ub = 0; loop_ub < 7; loop_ub++) {
      x[loop_ub] = (t_data[b_index + t_size[0] * loop_ub].im <= 0.0001);
    }
    y = true;
    loop_ub = 0;
    exitg1 = false;
    while ((!exitg1) && (loop_ub <= 6)) {
      if (!x[loop_ub]) {
        y = false;
        exitg1 = true;
      } else {
        loop_ub++;
      }
    }
    if (y) {
      if (b_index + 1 > t_size[0]) {
        rtDynamicBoundsError(b_index + 1, 1, t_size[0], &jb_emlrtBCI);
      }
      for (loop_ub = 0; loop_ub < 7; loop_ub++) {
        x[loop_ub] = (t_data[b_index + t_size[0] * loop_ub].re >= -0.0001);
      }
      y = true;
      loop_ub = 0;
      exitg1 = false;
      while ((!exitg1) && (loop_ub <= 6)) {
        if (!x[loop_ub]) {
          y = false;
          exitg1 = true;
        } else {
          loop_ub++;
        }
      }
      if (y) {
        double a;
        double b_a;
        unsigned char valid_0;
        bool guard1 = false;
        bool guard2 = false;
        if (b_index + 1 > t_size[0]) {
          rtDynamicBoundsError(b_index + 1, 1, t_size[0], &ib_emlrtBCI);
        }
        for (loop_ub = 0; loop_ub < 7; loop_ub++) {
          t[loop_ub] = t_data[b_index + t_size[0] * loop_ub].re;
        }
        //  ---------------------------------------------------------------------
        //  Package:    TopiCo (https://github.com/AIS-Bonn/TopiCo)
        //  Version:    2021-03-18 12:09:55
        //  Maintainer: Marius Beul (mbeul@ais.uni-bonn.de)
        //  License:    BSD
        //  ---------------------------------------------------------------------
        //  Software License Agreement (BSD License)
        //  Copyright (c) 2021, Computer Science Institute VI, University of
        //  Bonn All rights reserved. Redistribution and use in source and
        //  binary forms, with or without modification, are permitted provided
        //  that the following conditions are met:
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
        a = (V_init + t[0] * A_init) + 0.5 * (t[0] * t[0]) * J[0];
        b_a = A_init + t[0] * J[0];
        for (loop_ub = 0; loop_ub < 7; loop_ub++) {
          v[loop_ub] = a;
          c_a[loop_ub] = b_a;
        }
        for (loop_ub = 0; loop_ub < 6; loop_ub++) {
          a = t[loop_ub + 1];
          b_a = J[loop_ub + 1];
          v[loop_ub + 1] =
              (v[loop_ub] + a * c_a[loop_ub]) + 0.5 * (a * a) * b_a;
          c_a[loop_ub + 1] = c_a[loop_ub] + a * b_a;
        }
        valid_0 = check_feasibility(V_init, A_init, V_max, rtMinusInf, A_max,
                                    A_min, J_max, J_min);
        for (loop_ub = 0; loop_ub < 7; loop_ub++) {
          x[loop_ub] = (v[loop_ub] <= V_max);
        }
        y = true;
        loop_ub = 0;
        exitg1 = false;
        while ((!exitg1) && (loop_ub <= 6)) {
          if (!x[loop_ub]) {
            y = false;
            exitg1 = true;
          } else {
            loop_ub++;
          }
        }
        guard1 = false;
        guard2 = false;
        if (y) {
          for (loop_ub = 0; loop_ub < 7; loop_ub++) {
            x[loop_ub] = (v[loop_ub] >= rtMinusInf);
          }
          y = true;
          loop_ub = 0;
          exitg1 = false;
          while ((!exitg1) && (loop_ub <= 6)) {
            if (!x[loop_ub]) {
              y = false;
              exitg1 = true;
            } else {
              loop_ub++;
            }
          }
          if (y && (valid_0 == 0) &&
              (check_feasibility(v[1], c_a[1], V_max, rtMinusInf, A_max, A_min,
                                 J_max, J_min) == 0) &&
              (check_feasibility(v[6], c_a[6], V_max, rtMinusInf, A_max, A_min,
                                 J_max, J_min) == 0)) {
            guard1 = true;
          } else {
            guard2 = true;
          }
        } else {
          guard2 = true;
        }
        if (guard2) {
          for (loop_ub = 0; loop_ub < 5; loop_ub++) {
            b_x[loop_ub] = (v[loop_ub + 2] <= V_max);
          }
          y = true;
          loop_ub = 0;
          exitg1 = false;
          while ((!exitg1) && (loop_ub <= 4)) {
            if (!b_x[loop_ub]) {
              y = false;
              exitg1 = true;
            } else {
              loop_ub++;
            }
          }
          if (y) {
            for (loop_ub = 0; loop_ub < 5; loop_ub++) {
              b_x[loop_ub] = (v[loop_ub + 2] >= rtMinusInf);
            }
            y = true;
            loop_ub = 0;
            exitg1 = false;
            while ((!exitg1) && (loop_ub <= 4)) {
              if (!b_x[loop_ub]) {
                y = false;
                exitg1 = true;
              } else {
                loop_ub++;
              }
            }
            if (y && (valid_0 != 0)) {
              guard1 = true;
            }
          }
        }
        if (guard1) {
          for (loop_ub = 0; loop_ub < 7; loop_ub++) {
            x[loop_ub] = (c_a[loop_ub] <= A_max);
          }
          y = true;
          loop_ub = 0;
          exitg1 = false;
          while ((!exitg1) && (loop_ub <= 6)) {
            if (!x[loop_ub]) {
              y = false;
              exitg1 = true;
            } else {
              loop_ub++;
            }
          }
          if (y) {
            for (loop_ub = 0; loop_ub < 7; loop_ub++) {
              x[loop_ub] = (c_a[loop_ub] >= A_min);
            }
            y = true;
            loop_ub = 0;
            exitg1 = false;
            while ((!exitg1) && (loop_ub <= 6)) {
              if (!x[loop_ub]) {
                y = false;
                exitg1 = true;
              } else {
                loop_ub++;
              }
            }
            if (y) {
              for (loop_ub = 0; loop_ub < 7; loop_ub++) {
                x[loop_ub] = (J[loop_ub] <= J_max);
              }
              y = true;
              loop_ub = 0;
              exitg1 = false;
              while ((!exitg1) && (loop_ub <= 6)) {
                if (!x[loop_ub]) {
                  y = false;
                  exitg1 = true;
                } else {
                  loop_ub++;
                }
              }
              if (y) {
                for (loop_ub = 0; loop_ub < 7; loop_ub++) {
                  x[loop_ub] = (J[loop_ub] >= J_min);
                }
                y = true;
                loop_ub = 0;
                exitg1 = false;
                while ((!exitg1) && (loop_ub <= 6)) {
                  if (!x[loop_ub]) {
                    y = false;
                    exitg1 = true;
                  } else {
                    loop_ub++;
                  }
                }
                if (y &&
                    (((v[6] <= V_wayp + 0.0001) && (v[6] >= V_wayp - 0.0001)) ||
                     rtIsNaN(V_wayp))) {
                  if (b_index + 1 > valid_size[1]) {
                    rtDynamicBoundsError(b_index + 1, 1, valid_size[1],
                                         &lb_emlrtBCI);
                  }
                  valid_data[b_index] = true;
                }
              }
            }
          }
        }
      }
    }
  }
}

void c_check(const creal_T t_data[], const int t_size[2], const double J[7],
             double P_init, double V_init, double A_init, double P_wayp,
             double V_wayp, double A_wayp, double V_max, double V_min,
             double A_max, double A_min, double J_max, double J_min,
             bool valid_data[], int valid_size[2])
{
  double c_a[7];
  double p[7];
  double t[7];
  double v[7];
  int i;
  int loop_ub;
  bool x[7];
  bool b_x[5];
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
  // numerical
  // numerical
  // numerical
  // numerical
  V_max *= 1.0001;
  // numerical
  V_min *= 1.0001;
  // numerical
  A_max *= 1.0001;
  // numerical
  A_min *= 1.0001;
  // numerical
  J_max *= 1.0001;
  // numerical
  J_min *= 1.0001;
  // numerical
  // numerical
  // numerical
  valid_size[0] = 1;
  valid_size[1] = t_size[0];
  loop_ub = t_size[0];
  if (0 <= loop_ub - 1) {
    std::memset(&valid_data[0], 0, loop_ub * sizeof(bool));
  }
  i = t_size[0];
  for (int b_index = 0; b_index < i; b_index++) {
    bool exitg1;
    bool y;
    if (b_index + 1 > t_size[0]) {
      rtDynamicBoundsError(b_index + 1, 1, t_size[0], &kb_emlrtBCI);
    }
    for (loop_ub = 0; loop_ub < 7; loop_ub++) {
      x[loop_ub] = (t_data[b_index + t_size[0] * loop_ub].im <= 0.0001);
    }
    y = true;
    loop_ub = 0;
    exitg1 = false;
    while ((!exitg1) && (loop_ub <= 6)) {
      if (!x[loop_ub]) {
        y = false;
        exitg1 = true;
      } else {
        loop_ub++;
      }
    }
    if (y) {
      if (b_index + 1 > t_size[0]) {
        rtDynamicBoundsError(b_index + 1, 1, t_size[0], &jb_emlrtBCI);
      }
      for (loop_ub = 0; loop_ub < 7; loop_ub++) {
        x[loop_ub] = (t_data[b_index + t_size[0] * loop_ub].re >= -0.0001);
      }
      y = true;
      loop_ub = 0;
      exitg1 = false;
      while ((!exitg1) && (loop_ub <= 6)) {
        if (!x[loop_ub]) {
          y = false;
          exitg1 = true;
        } else {
          loop_ub++;
        }
      }
      if (y) {
        double a;
        double a_tmp;
        double b_a;
        unsigned char valid_0;
        bool guard1 = false;
        bool guard2 = false;
        if (b_index + 1 > t_size[0]) {
          rtDynamicBoundsError(b_index + 1, 1, t_size[0], &ib_emlrtBCI);
        }
        for (loop_ub = 0; loop_ub < 7; loop_ub++) {
          t[loop_ub] = t_data[b_index + t_size[0] * loop_ub].re;
        }
        //  ---------------------------------------------------------------------
        //  Package:    TopiCo (https://github.com/AIS-Bonn/TopiCo)
        //  Version:    2021-03-18 12:09:55
        //  Maintainer: Marius Beul (mbeul@ais.uni-bonn.de)
        //  License:    BSD
        //  ---------------------------------------------------------------------
        //  Software License Agreement (BSD License)
        //  Copyright (c) 2021, Computer Science Institute VI, University of
        //  Bonn All rights reserved. Redistribution and use in source and
        //  binary forms, with or without modification, are permitted provided
        //  that the following conditions are met:
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
        a_tmp = 0.5 * (t[0] * t[0]);
        a = ((P_init + t[0] * V_init) + a_tmp * A_init) +
            0.16666666666666666 * rt_powd_snf(t[0], 3.0) * J[0];
        a_tmp = (V_init + t[0] * A_init) + a_tmp * J[0];
        b_a = A_init + t[0] * J[0];
        for (loop_ub = 0; loop_ub < 7; loop_ub++) {
          p[loop_ub] = a;
          v[loop_ub] = a_tmp;
          c_a[loop_ub] = b_a;
        }
        for (loop_ub = 0; loop_ub < 6; loop_ub++) {
          double d;
          a_tmp = t[loop_ub + 1];
          b_a = 0.5 * (a_tmp * a_tmp);
          a = J[loop_ub + 1];
          d = c_a[loop_ub];
          p[loop_ub + 1] = ((p[loop_ub] + a_tmp * v[loop_ub]) + b_a * d) +
                           0.16666666666666666 * rt_powd_snf(a_tmp, 3.0) * a;
          v[loop_ub + 1] = (v[loop_ub] + a_tmp * d) + b_a * a;
          c_a[loop_ub + 1] = c_a[loop_ub] + a_tmp * a;
        }
        valid_0 = check_feasibility(V_init, A_init, V_max, V_min, A_max, A_min,
                                    J_max, J_min);
        for (loop_ub = 0; loop_ub < 7; loop_ub++) {
          x[loop_ub] = (v[loop_ub] <= V_max);
        }
        y = true;
        loop_ub = 0;
        exitg1 = false;
        while ((!exitg1) && (loop_ub <= 6)) {
          if (!x[loop_ub]) {
            y = false;
            exitg1 = true;
          } else {
            loop_ub++;
          }
        }
        guard1 = false;
        guard2 = false;
        if (y) {
          for (loop_ub = 0; loop_ub < 7; loop_ub++) {
            x[loop_ub] = (v[loop_ub] >= V_min);
          }
          y = true;
          loop_ub = 0;
          exitg1 = false;
          while ((!exitg1) && (loop_ub <= 6)) {
            if (!x[loop_ub]) {
              y = false;
              exitg1 = true;
            } else {
              loop_ub++;
            }
          }
          if (y && (valid_0 == 0) &&
              (check_feasibility(v[1], c_a[1], V_max, V_min, A_max, A_min,
                                 J_max, J_min) == 0) &&
              (check_feasibility(v[6], c_a[6], V_max, V_min, A_max, A_min,
                                 J_max, J_min) == 0)) {
            guard1 = true;
          } else {
            guard2 = true;
          }
        } else {
          guard2 = true;
        }
        if (guard2) {
          for (loop_ub = 0; loop_ub < 5; loop_ub++) {
            b_x[loop_ub] = (v[loop_ub + 2] <= V_max);
          }
          y = true;
          loop_ub = 0;
          exitg1 = false;
          while ((!exitg1) && (loop_ub <= 4)) {
            if (!b_x[loop_ub]) {
              y = false;
              exitg1 = true;
            } else {
              loop_ub++;
            }
          }
          if (y) {
            for (loop_ub = 0; loop_ub < 5; loop_ub++) {
              b_x[loop_ub] = (v[loop_ub + 2] >= V_min);
            }
            y = true;
            loop_ub = 0;
            exitg1 = false;
            while ((!exitg1) && (loop_ub <= 4)) {
              if (!b_x[loop_ub]) {
                y = false;
                exitg1 = true;
              } else {
                loop_ub++;
              }
            }
            if (y && (valid_0 != 0)) {
              guard1 = true;
            }
          }
        }
        if (guard1) {
          for (loop_ub = 0; loop_ub < 7; loop_ub++) {
            x[loop_ub] = (c_a[loop_ub] <= A_max);
          }
          y = true;
          loop_ub = 0;
          exitg1 = false;
          while ((!exitg1) && (loop_ub <= 6)) {
            if (!x[loop_ub]) {
              y = false;
              exitg1 = true;
            } else {
              loop_ub++;
            }
          }
          if (y) {
            for (loop_ub = 0; loop_ub < 7; loop_ub++) {
              x[loop_ub] = (c_a[loop_ub] >= A_min);
            }
            y = true;
            loop_ub = 0;
            exitg1 = false;
            while ((!exitg1) && (loop_ub <= 6)) {
              if (!x[loop_ub]) {
                y = false;
                exitg1 = true;
              } else {
                loop_ub++;
              }
            }
            if (y) {
              for (loop_ub = 0; loop_ub < 7; loop_ub++) {
                x[loop_ub] = (J[loop_ub] <= J_max);
              }
              y = true;
              loop_ub = 0;
              exitg1 = false;
              while ((!exitg1) && (loop_ub <= 6)) {
                if (!x[loop_ub]) {
                  y = false;
                  exitg1 = true;
                } else {
                  loop_ub++;
                }
              }
              if (y) {
                for (loop_ub = 0; loop_ub < 7; loop_ub++) {
                  x[loop_ub] = (J[loop_ub] >= J_min);
                }
                y = true;
                loop_ub = 0;
                exitg1 = false;
                while ((!exitg1) && (loop_ub <= 6)) {
                  if (!x[loop_ub]) {
                    y = false;
                    exitg1 = true;
                  } else {
                    loop_ub++;
                  }
                }
                if (y &&
                    (((p[6] <= P_wayp + 0.0001) && (p[6] >= P_wayp - 0.0001)) ||
                     rtIsNaN(P_wayp)) &&
                    (((v[6] <= V_wayp + 0.0001) && (v[6] >= V_wayp - 0.0001)) ||
                     rtIsNaN(V_wayp)) &&
                    (((c_a[6] <= A_wayp + 0.0001) &&
                      (c_a[6] >= A_wayp - 0.0001)) ||
                     rtIsNaN(A_wayp))) {
                  if (b_index + 1 > valid_size[1]) {
                    rtDynamicBoundsError(b_index + 1, 1, valid_size[1],
                                         &lb_emlrtBCI);
                  }
                  valid_data[b_index] = true;
                }
              }
            }
          }
        }
      }
    }
  }
}

void check(const creal_T t_data[], const int t_size[2], const double J[7],
           double V_init, double A_init, double V_wayp, double V_min,
           double A_max, double A_min, double J_max, double J_min,
           bool valid_data[], int valid_size[2])
{
  double c_a[7];
  double t[7];
  double v[7];
  int i;
  int loop_ub;
  bool x[7];
  bool b_x[5];
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
  // numerical
  // numerical
  // numerical
  // numerical
  // numerical
  V_min *= 1.0001;
  // numerical
  A_max *= 1.0001;
  // numerical
  A_min *= 1.0001;
  // numerical
  J_max *= 1.0001;
  // numerical
  J_min *= 1.0001;
  // numerical
  // numerical
  // numerical
  valid_size[0] = 1;
  valid_size[1] = t_size[0];
  loop_ub = t_size[0];
  if (0 <= loop_ub - 1) {
    std::memset(&valid_data[0], 0, loop_ub * sizeof(bool));
  }
  i = t_size[0];
  for (int b_index = 0; b_index < i; b_index++) {
    bool exitg1;
    bool y;
    if (b_index + 1 > t_size[0]) {
      rtDynamicBoundsError(b_index + 1, 1, t_size[0], &kb_emlrtBCI);
    }
    for (loop_ub = 0; loop_ub < 7; loop_ub++) {
      x[loop_ub] = (t_data[b_index + t_size[0] * loop_ub].im <= 0.0001);
    }
    y = true;
    loop_ub = 0;
    exitg1 = false;
    while ((!exitg1) && (loop_ub <= 6)) {
      if (!x[loop_ub]) {
        y = false;
        exitg1 = true;
      } else {
        loop_ub++;
      }
    }
    if (y) {
      if (b_index + 1 > t_size[0]) {
        rtDynamicBoundsError(b_index + 1, 1, t_size[0], &jb_emlrtBCI);
      }
      for (loop_ub = 0; loop_ub < 7; loop_ub++) {
        x[loop_ub] = (t_data[b_index + t_size[0] * loop_ub].re >= -0.0001);
      }
      y = true;
      loop_ub = 0;
      exitg1 = false;
      while ((!exitg1) && (loop_ub <= 6)) {
        if (!x[loop_ub]) {
          y = false;
          exitg1 = true;
        } else {
          loop_ub++;
        }
      }
      if (y) {
        double a;
        double b_a;
        unsigned char valid_0;
        bool guard1 = false;
        bool guard2 = false;
        if (b_index + 1 > t_size[0]) {
          rtDynamicBoundsError(b_index + 1, 1, t_size[0], &ib_emlrtBCI);
        }
        for (loop_ub = 0; loop_ub < 7; loop_ub++) {
          t[loop_ub] = t_data[b_index + t_size[0] * loop_ub].re;
        }
        //  ---------------------------------------------------------------------
        //  Package:    TopiCo (https://github.com/AIS-Bonn/TopiCo)
        //  Version:    2021-03-18 12:09:55
        //  Maintainer: Marius Beul (mbeul@ais.uni-bonn.de)
        //  License:    BSD
        //  ---------------------------------------------------------------------
        //  Software License Agreement (BSD License)
        //  Copyright (c) 2021, Computer Science Institute VI, University of
        //  Bonn All rights reserved. Redistribution and use in source and
        //  binary forms, with or without modification, are permitted provided
        //  that the following conditions are met:
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
        a = (V_init + t[0] * A_init) + 0.5 * (t[0] * t[0]) * J[0];
        b_a = A_init + t[0] * J[0];
        for (loop_ub = 0; loop_ub < 7; loop_ub++) {
          v[loop_ub] = a;
          c_a[loop_ub] = b_a;
        }
        for (loop_ub = 0; loop_ub < 6; loop_ub++) {
          a = t[loop_ub + 1];
          b_a = J[loop_ub + 1];
          v[loop_ub + 1] =
              (v[loop_ub] + a * c_a[loop_ub]) + 0.5 * (a * a) * b_a;
          c_a[loop_ub + 1] = c_a[loop_ub] + a * b_a;
        }
        valid_0 = check_feasibility(V_init, A_init, rtInf, V_min, A_max, A_min,
                                    J_max, J_min);
        for (loop_ub = 0; loop_ub < 7; loop_ub++) {
          x[loop_ub] = (v[loop_ub] <= rtInf);
        }
        y = true;
        loop_ub = 0;
        exitg1 = false;
        while ((!exitg1) && (loop_ub <= 6)) {
          if (!x[loop_ub]) {
            y = false;
            exitg1 = true;
          } else {
            loop_ub++;
          }
        }
        guard1 = false;
        guard2 = false;
        if (y) {
          for (loop_ub = 0; loop_ub < 7; loop_ub++) {
            x[loop_ub] = (v[loop_ub] >= V_min);
          }
          y = true;
          loop_ub = 0;
          exitg1 = false;
          while ((!exitg1) && (loop_ub <= 6)) {
            if (!x[loop_ub]) {
              y = false;
              exitg1 = true;
            } else {
              loop_ub++;
            }
          }
          if (y && (valid_0 == 0) &&
              (check_feasibility(v[1], c_a[1], rtInf, V_min, A_max, A_min,
                                 J_max, J_min) == 0) &&
              (check_feasibility(v[6], c_a[6], rtInf, V_min, A_max, A_min,
                                 J_max, J_min) == 0)) {
            guard1 = true;
          } else {
            guard2 = true;
          }
        } else {
          guard2 = true;
        }
        if (guard2) {
          for (loop_ub = 0; loop_ub < 5; loop_ub++) {
            b_x[loop_ub] = (v[loop_ub + 2] <= rtInf);
          }
          y = true;
          loop_ub = 0;
          exitg1 = false;
          while ((!exitg1) && (loop_ub <= 4)) {
            if (!b_x[loop_ub]) {
              y = false;
              exitg1 = true;
            } else {
              loop_ub++;
            }
          }
          if (y) {
            for (loop_ub = 0; loop_ub < 5; loop_ub++) {
              b_x[loop_ub] = (v[loop_ub + 2] >= V_min);
            }
            y = true;
            loop_ub = 0;
            exitg1 = false;
            while ((!exitg1) && (loop_ub <= 4)) {
              if (!b_x[loop_ub]) {
                y = false;
                exitg1 = true;
              } else {
                loop_ub++;
              }
            }
            if (y && (valid_0 != 0)) {
              guard1 = true;
            }
          }
        }
        if (guard1) {
          for (loop_ub = 0; loop_ub < 7; loop_ub++) {
            x[loop_ub] = (c_a[loop_ub] <= A_max);
          }
          y = true;
          loop_ub = 0;
          exitg1 = false;
          while ((!exitg1) && (loop_ub <= 6)) {
            if (!x[loop_ub]) {
              y = false;
              exitg1 = true;
            } else {
              loop_ub++;
            }
          }
          if (y) {
            for (loop_ub = 0; loop_ub < 7; loop_ub++) {
              x[loop_ub] = (c_a[loop_ub] >= A_min);
            }
            y = true;
            loop_ub = 0;
            exitg1 = false;
            while ((!exitg1) && (loop_ub <= 6)) {
              if (!x[loop_ub]) {
                y = false;
                exitg1 = true;
              } else {
                loop_ub++;
              }
            }
            if (y) {
              for (loop_ub = 0; loop_ub < 7; loop_ub++) {
                x[loop_ub] = (J[loop_ub] <= J_max);
              }
              y = true;
              loop_ub = 0;
              exitg1 = false;
              while ((!exitg1) && (loop_ub <= 6)) {
                if (!x[loop_ub]) {
                  y = false;
                  exitg1 = true;
                } else {
                  loop_ub++;
                }
              }
              if (y) {
                for (loop_ub = 0; loop_ub < 7; loop_ub++) {
                  x[loop_ub] = (J[loop_ub] >= J_min);
                }
                y = true;
                loop_ub = 0;
                exitg1 = false;
                while ((!exitg1) && (loop_ub <= 6)) {
                  if (!x[loop_ub]) {
                    y = false;
                    exitg1 = true;
                  } else {
                    loop_ub++;
                  }
                }
                if (y &&
                    (((v[6] <= V_wayp + 0.0001) && (v[6] >= V_wayp - 0.0001)) ||
                     rtIsNaN(V_wayp))) {
                  if (b_index + 1 > valid_size[1]) {
                    rtDynamicBoundsError(b_index + 1, 1, valid_size[1],
                                         &lb_emlrtBCI);
                  }
                  valid_data[b_index] = true;
                }
              }
            }
          }
        }
      }
    }
  }
}

void d_check(const creal_T t_data[], const int t_size[2], const double J[7],
             double V_init, double A_init, double V_max, double V_min,
             double A_max, double A_min, double J_max, double J_min,
             bool valid_data[], int valid_size[2])
{
  double c_a[7];
  double t[7];
  double v[7];
  int i;
  int loop_ub;
  bool x[7];
  bool b_x[5];
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
  // numerical
  // numerical
  // numerical
  // numerical
  V_max *= 1.0001;
  // numerical
  V_min *= 1.0001;
  // numerical
  A_max *= 1.0001;
  // numerical
  A_min *= 1.0001;
  // numerical
  J_max *= 1.0001;
  // numerical
  J_min *= 1.0001;
  // numerical
  // numerical
  // numerical
  valid_size[0] = 1;
  valid_size[1] = t_size[0];
  loop_ub = t_size[0];
  if (0 <= loop_ub - 1) {
    std::memset(&valid_data[0], 0, loop_ub * sizeof(bool));
  }
  i = t_size[0];
  for (int b_index = 0; b_index < i; b_index++) {
    bool exitg1;
    bool y;
    if (b_index + 1 > t_size[0]) {
      rtDynamicBoundsError(b_index + 1, 1, t_size[0], &kb_emlrtBCI);
    }
    for (loop_ub = 0; loop_ub < 7; loop_ub++) {
      x[loop_ub] = (t_data[b_index + t_size[0] * loop_ub].im <= 0.0001);
    }
    y = true;
    loop_ub = 0;
    exitg1 = false;
    while ((!exitg1) && (loop_ub <= 6)) {
      if (!x[loop_ub]) {
        y = false;
        exitg1 = true;
      } else {
        loop_ub++;
      }
    }
    if (y) {
      if (b_index + 1 > t_size[0]) {
        rtDynamicBoundsError(b_index + 1, 1, t_size[0], &jb_emlrtBCI);
      }
      for (loop_ub = 0; loop_ub < 7; loop_ub++) {
        x[loop_ub] = (t_data[b_index + t_size[0] * loop_ub].re >= -0.0001);
      }
      y = true;
      loop_ub = 0;
      exitg1 = false;
      while ((!exitg1) && (loop_ub <= 6)) {
        if (!x[loop_ub]) {
          y = false;
          exitg1 = true;
        } else {
          loop_ub++;
        }
      }
      if (y) {
        double a;
        double b_a;
        unsigned char valid_0;
        bool guard1 = false;
        bool guard2 = false;
        if (b_index + 1 > t_size[0]) {
          rtDynamicBoundsError(b_index + 1, 1, t_size[0], &ib_emlrtBCI);
        }
        for (loop_ub = 0; loop_ub < 7; loop_ub++) {
          t[loop_ub] = t_data[b_index + t_size[0] * loop_ub].re;
        }
        //  ---------------------------------------------------------------------
        //  Package:    TopiCo (https://github.com/AIS-Bonn/TopiCo)
        //  Version:    2021-03-18 12:09:55
        //  Maintainer: Marius Beul (mbeul@ais.uni-bonn.de)
        //  License:    BSD
        //  ---------------------------------------------------------------------
        //  Software License Agreement (BSD License)
        //  Copyright (c) 2021, Computer Science Institute VI, University of
        //  Bonn All rights reserved. Redistribution and use in source and
        //  binary forms, with or without modification, are permitted provided
        //  that the following conditions are met:
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
        a = (V_init + t[0] * A_init) + 0.5 * (t[0] * t[0]) * J[0];
        b_a = A_init + t[0] * J[0];
        for (loop_ub = 0; loop_ub < 7; loop_ub++) {
          v[loop_ub] = a;
          c_a[loop_ub] = b_a;
        }
        for (loop_ub = 0; loop_ub < 6; loop_ub++) {
          a = t[loop_ub + 1];
          b_a = J[loop_ub + 1];
          v[loop_ub + 1] =
              (v[loop_ub] + a * c_a[loop_ub]) + 0.5 * (a * a) * b_a;
          c_a[loop_ub + 1] = c_a[loop_ub] + a * b_a;
        }
        valid_0 = check_feasibility(V_init, A_init, V_max, V_min, A_max, A_min,
                                    J_max, J_min);
        for (loop_ub = 0; loop_ub < 7; loop_ub++) {
          x[loop_ub] = (v[loop_ub] <= V_max);
        }
        y = true;
        loop_ub = 0;
        exitg1 = false;
        while ((!exitg1) && (loop_ub <= 6)) {
          if (!x[loop_ub]) {
            y = false;
            exitg1 = true;
          } else {
            loop_ub++;
          }
        }
        guard1 = false;
        guard2 = false;
        if (y) {
          for (loop_ub = 0; loop_ub < 7; loop_ub++) {
            x[loop_ub] = (v[loop_ub] >= V_min);
          }
          y = true;
          loop_ub = 0;
          exitg1 = false;
          while ((!exitg1) && (loop_ub <= 6)) {
            if (!x[loop_ub]) {
              y = false;
              exitg1 = true;
            } else {
              loop_ub++;
            }
          }
          if (y && (valid_0 == 0) &&
              (check_feasibility(v[1], c_a[1], V_max, V_min, A_max, A_min,
                                 J_max, J_min) == 0) &&
              (check_feasibility(v[6], c_a[6], V_max, V_min, A_max, A_min,
                                 J_max, J_min) == 0)) {
            guard1 = true;
          } else {
            guard2 = true;
          }
        } else {
          guard2 = true;
        }
        if (guard2) {
          for (loop_ub = 0; loop_ub < 5; loop_ub++) {
            b_x[loop_ub] = (v[loop_ub + 2] <= V_max);
          }
          y = true;
          loop_ub = 0;
          exitg1 = false;
          while ((!exitg1) && (loop_ub <= 4)) {
            if (!b_x[loop_ub]) {
              y = false;
              exitg1 = true;
            } else {
              loop_ub++;
            }
          }
          if (y) {
            for (loop_ub = 0; loop_ub < 5; loop_ub++) {
              b_x[loop_ub] = (v[loop_ub + 2] >= V_min);
            }
            y = true;
            loop_ub = 0;
            exitg1 = false;
            while ((!exitg1) && (loop_ub <= 4)) {
              if (!b_x[loop_ub]) {
                y = false;
                exitg1 = true;
              } else {
                loop_ub++;
              }
            }
            if (y && (valid_0 != 0)) {
              guard1 = true;
            }
          }
        }
        if (guard1) {
          for (loop_ub = 0; loop_ub < 7; loop_ub++) {
            x[loop_ub] = (c_a[loop_ub] <= A_max);
          }
          y = true;
          loop_ub = 0;
          exitg1 = false;
          while ((!exitg1) && (loop_ub <= 6)) {
            if (!x[loop_ub]) {
              y = false;
              exitg1 = true;
            } else {
              loop_ub++;
            }
          }
          if (y) {
            for (loop_ub = 0; loop_ub < 7; loop_ub++) {
              x[loop_ub] = (c_a[loop_ub] >= A_min);
            }
            y = true;
            loop_ub = 0;
            exitg1 = false;
            while ((!exitg1) && (loop_ub <= 6)) {
              if (!x[loop_ub]) {
                y = false;
                exitg1 = true;
              } else {
                loop_ub++;
              }
            }
            if (y) {
              for (loop_ub = 0; loop_ub < 7; loop_ub++) {
                x[loop_ub] = (J[loop_ub] <= J_max);
              }
              y = true;
              loop_ub = 0;
              exitg1 = false;
              while ((!exitg1) && (loop_ub <= 6)) {
                if (!x[loop_ub]) {
                  y = false;
                  exitg1 = true;
                } else {
                  loop_ub++;
                }
              }
              if (y) {
                for (loop_ub = 0; loop_ub < 7; loop_ub++) {
                  x[loop_ub] = (J[loop_ub] >= J_min);
                }
                y = true;
                loop_ub = 0;
                exitg1 = false;
                while ((!exitg1) && (loop_ub <= 6)) {
                  if (!x[loop_ub]) {
                    y = false;
                    exitg1 = true;
                  } else {
                    loop_ub++;
                  }
                }
                if (y && (v[6] <= 0.0001) && (v[6] >= -0.0001) &&
                    (c_a[6] <= 0.0001) && (c_a[6] >= -0.0001)) {
                  if (b_index + 1 > valid_size[1]) {
                    rtDynamicBoundsError(b_index + 1, 1, valid_size[1],
                                         &lb_emlrtBCI);
                  }
                  valid_data[b_index] = true;
                }
              }
            }
          }
        }
      }
    }
  }
}

void e_check(const creal_T t_data[], const int t_size[2], const double J[7],
             double P_init, double V_init, double A_init, double P_wayp,
             double V_wayp, double A_wayp, double V_max, double V_min,
             double A_max, double A_min, double J_max, double J_min,
             double t_sync, bool valid_data[], int valid_size[2])
{
  static rtBoundsCheckInfo nb_emlrtBCI = {
      -1,                                           // iFirst
      -1,                                           // iLast
      67,                                           // lineNo
      43,                                           // colNo
      "t",                                          // aName
      "check",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/check.m", // pName
      0                                             // checkKind
  };
  double c_a[7];
  double p[7];
  double t[7];
  double v[7];
  int i;
  int loop_ub;
  bool x[7];
  bool b_x[5];
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
  // numerical
  // numerical
  // numerical
  // numerical
  V_max *= 1.0001;
  // numerical
  V_min *= 1.0001;
  // numerical
  A_max *= 1.0001;
  // numerical
  A_min *= 1.0001;
  // numerical
  J_max *= 1.0001;
  // numerical
  J_min *= 1.0001;
  // numerical
  // numerical
  // numerical
  valid_size[0] = 1;
  valid_size[1] = t_size[0];
  loop_ub = t_size[0];
  if (0 <= loop_ub - 1) {
    std::memset(&valid_data[0], 0, loop_ub * sizeof(bool));
  }
  i = t_size[0];
  for (int b_index = 0; b_index < i; b_index++) {
    bool exitg1;
    bool y;
    if (b_index + 1 > t_size[0]) {
      rtDynamicBoundsError(b_index + 1, 1, t_size[0], &kb_emlrtBCI);
    }
    for (loop_ub = 0; loop_ub < 7; loop_ub++) {
      x[loop_ub] = (t_data[b_index + t_size[0] * loop_ub].im <= 0.0001);
    }
    y = true;
    loop_ub = 0;
    exitg1 = false;
    while ((!exitg1) && (loop_ub <= 6)) {
      if (!x[loop_ub]) {
        y = false;
        exitg1 = true;
      } else {
        loop_ub++;
      }
    }
    if (y) {
      if (b_index + 1 > t_size[0]) {
        rtDynamicBoundsError(b_index + 1, 1, t_size[0], &jb_emlrtBCI);
      }
      for (loop_ub = 0; loop_ub < 7; loop_ub++) {
        x[loop_ub] = (t_data[b_index + t_size[0] * loop_ub].re >= -0.0001);
      }
      y = true;
      loop_ub = 0;
      exitg1 = false;
      while ((!exitg1) && (loop_ub <= 6)) {
        if (!x[loop_ub]) {
          y = false;
          exitg1 = true;
        } else {
          loop_ub++;
        }
      }
      if (y) {
        double sum_t_re;
        if (b_index + 1 > t_size[0]) {
          rtDynamicBoundsError(b_index + 1, 1, t_size[0], &ib_emlrtBCI);
        }
        sum_t_re = t_data[b_index].re;
        for (loop_ub = 0; loop_ub < 6; loop_ub++) {
          sum_t_re += t_data[b_index + t_size[0] * (loop_ub + 1)].re;
        }
        if ((t_sync - 0.001 < sum_t_re) && (t_sync + 0.001 > sum_t_re)) {
          double a;
          double b_a;
          unsigned char valid_0;
          bool guard1 = false;
          bool guard2 = false;
          if (b_index + 1 > t_size[0]) {
            rtDynamicBoundsError(b_index + 1, 1, t_size[0], &nb_emlrtBCI);
          }
          for (loop_ub = 0; loop_ub < 7; loop_ub++) {
            t[loop_ub] = t_data[b_index + t_size[0] * loop_ub].re;
          }
          //  ---------------------------------------------------------------------
          //  Package:    TopiCo (https://github.com/AIS-Bonn/TopiCo)
          //  Version:    2021-03-18 12:09:55
          //  Maintainer: Marius Beul (mbeul@ais.uni-bonn.de)
          //  License:    BSD
          //  ---------------------------------------------------------------------
          //  Software License Agreement (BSD License)
          //  Copyright (c) 2021, Computer Science Institute VI, University of
          //  Bonn All rights reserved. Redistribution and use in source and
          //  binary forms, with or without modification, are permitted provided
          //  that the following conditions are met:
          //
          //  * Redistributions of source code must retain the above copyright
          //    notice, this list of conditions and the following disclaimer.
          //  * Redistributions in binary form must reproduce the above
          //    copyright notice, this list of conditions and the following
          //    disclaimer in the documentation and/or other materials provided
          //    with the distribution.
          //  * Neither the name of University of Bonn, Computer Science
          //  Institute
          //    VI nor the names of its contributors may be used to endorse or
          //    promote products derived from this software without specific
          //    prior written permission.
          //
          //  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
          //  CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
          //  INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
          //  MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
          //  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS
          //  BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
          //  EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
          //  TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
          //  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
          //  ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
          //  TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF
          //  THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
          //  SUCH DAMAGE.
          //  --------------------------------------------------------------------
          sum_t_re = 0.5 * (t[0] * t[0]);
          a = ((P_init + t[0] * V_init) + sum_t_re * A_init) +
              0.16666666666666666 * rt_powd_snf(t[0], 3.0) * J[0];
          sum_t_re = (V_init + t[0] * A_init) + sum_t_re * J[0];
          b_a = A_init + t[0] * J[0];
          for (loop_ub = 0; loop_ub < 7; loop_ub++) {
            p[loop_ub] = a;
            v[loop_ub] = sum_t_re;
            c_a[loop_ub] = b_a;
          }
          for (loop_ub = 0; loop_ub < 6; loop_ub++) {
            double d;
            sum_t_re = t[loop_ub + 1];
            b_a = 0.5 * (sum_t_re * sum_t_re);
            a = J[loop_ub + 1];
            d = c_a[loop_ub];
            p[loop_ub + 1] =
                ((p[loop_ub] + sum_t_re * v[loop_ub]) + b_a * d) +
                0.16666666666666666 * rt_powd_snf(sum_t_re, 3.0) * a;
            v[loop_ub + 1] = (v[loop_ub] + sum_t_re * d) + b_a * a;
            c_a[loop_ub + 1] = c_a[loop_ub] + sum_t_re * a;
          }
          valid_0 = check_feasibility(V_init, A_init, V_max, V_min, A_max,
                                      A_min, J_max, J_min);
          for (loop_ub = 0; loop_ub < 7; loop_ub++) {
            x[loop_ub] = (v[loop_ub] <= V_max);
          }
          y = true;
          loop_ub = 0;
          exitg1 = false;
          while ((!exitg1) && (loop_ub <= 6)) {
            if (!x[loop_ub]) {
              y = false;
              exitg1 = true;
            } else {
              loop_ub++;
            }
          }
          guard1 = false;
          guard2 = false;
          if (y) {
            for (loop_ub = 0; loop_ub < 7; loop_ub++) {
              x[loop_ub] = (v[loop_ub] >= V_min);
            }
            y = true;
            loop_ub = 0;
            exitg1 = false;
            while ((!exitg1) && (loop_ub <= 6)) {
              if (!x[loop_ub]) {
                y = false;
                exitg1 = true;
              } else {
                loop_ub++;
              }
            }
            if (y && (valid_0 == 0) &&
                (check_feasibility(v[1], c_a[1], V_max, V_min, A_max, A_min,
                                   J_max, J_min) == 0) &&
                (check_feasibility(v[6], c_a[6], V_max, V_min, A_max, A_min,
                                   J_max, J_min) == 0)) {
              guard1 = true;
            } else {
              guard2 = true;
            }
          } else {
            guard2 = true;
          }
          if (guard2) {
            for (loop_ub = 0; loop_ub < 5; loop_ub++) {
              b_x[loop_ub] = (v[loop_ub + 2] <= V_max);
            }
            y = true;
            loop_ub = 0;
            exitg1 = false;
            while ((!exitg1) && (loop_ub <= 4)) {
              if (!b_x[loop_ub]) {
                y = false;
                exitg1 = true;
              } else {
                loop_ub++;
              }
            }
            if (y) {
              for (loop_ub = 0; loop_ub < 5; loop_ub++) {
                b_x[loop_ub] = (v[loop_ub + 2] >= V_min);
              }
              y = true;
              loop_ub = 0;
              exitg1 = false;
              while ((!exitg1) && (loop_ub <= 4)) {
                if (!b_x[loop_ub]) {
                  y = false;
                  exitg1 = true;
                } else {
                  loop_ub++;
                }
              }
              if (y && (valid_0 != 0)) {
                guard1 = true;
              }
            }
          }
          if (guard1) {
            for (loop_ub = 0; loop_ub < 7; loop_ub++) {
              x[loop_ub] = (c_a[loop_ub] <= A_max);
            }
            y = true;
            loop_ub = 0;
            exitg1 = false;
            while ((!exitg1) && (loop_ub <= 6)) {
              if (!x[loop_ub]) {
                y = false;
                exitg1 = true;
              } else {
                loop_ub++;
              }
            }
            if (y) {
              for (loop_ub = 0; loop_ub < 7; loop_ub++) {
                x[loop_ub] = (c_a[loop_ub] >= A_min);
              }
              y = true;
              loop_ub = 0;
              exitg1 = false;
              while ((!exitg1) && (loop_ub <= 6)) {
                if (!x[loop_ub]) {
                  y = false;
                  exitg1 = true;
                } else {
                  loop_ub++;
                }
              }
              if (y) {
                for (loop_ub = 0; loop_ub < 7; loop_ub++) {
                  x[loop_ub] = (J[loop_ub] <= J_max);
                }
                y = true;
                loop_ub = 0;
                exitg1 = false;
                while ((!exitg1) && (loop_ub <= 6)) {
                  if (!x[loop_ub]) {
                    y = false;
                    exitg1 = true;
                  } else {
                    loop_ub++;
                  }
                }
                if (y) {
                  for (loop_ub = 0; loop_ub < 7; loop_ub++) {
                    x[loop_ub] = (J[loop_ub] >= J_min);
                  }
                  y = true;
                  loop_ub = 0;
                  exitg1 = false;
                  while ((!exitg1) && (loop_ub <= 6)) {
                    if (!x[loop_ub]) {
                      y = false;
                      exitg1 = true;
                    } else {
                      loop_ub++;
                    }
                  }
                  if (y &&
                      (((p[6] <= P_wayp + 0.0001) &&
                        (p[6] >= P_wayp - 0.0001)) ||
                       rtIsNaN(P_wayp)) &&
                      (((v[6] <= V_wayp + 0.0001) &&
                        (v[6] >= V_wayp - 0.0001)) ||
                       rtIsNaN(V_wayp)) &&
                      (((c_a[6] <= A_wayp + 0.0001) &&
                        (c_a[6] >= A_wayp - 0.0001)) ||
                       rtIsNaN(A_wayp))) {
                    if (b_index + 1 > valid_size[1]) {
                      rtDynamicBoundsError(b_index + 1, 1, valid_size[1],
                                           &lb_emlrtBCI);
                    }
                    valid_data[b_index] = true;
                  }
                }
              }
            }
          }
        }
      }
    }
  }
}

// End of code generation (check.cpp)
