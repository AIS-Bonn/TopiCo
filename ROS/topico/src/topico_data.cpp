//
// Student License - for use by students to meet course requirements and
// perform academic research at degree granting institutions only.  Not
// for government, commercial, or other organizational use.
//
// topico_data.cpp
//
// Code generation for function 'topico_data'
//

// Include files
#include "topico_data.h"
#include "rt_nonfinite.h"
#include "topico_types.h"

// Variable Definitions
unsigned char nlut[8];

unsigned char condlut[48];

rtRunTimeErrorInfo h_emlrtRTEI = {
    103,                                                            // lineNo
    9,                                                              // colNo
    "eml_idivide",                                                  // fName
    "/usr/local/MATLAB/R2021a/toolbox/eml/lib/matlab/ops/idivide.m" // pName
};

rtRunTimeErrorInfo k_emlrtRTEI = {
    271,                                                              // lineNo
    27,                                                               // colNo
    "check_non_axis_size",                                            // fName
    "/usr/local/MATLAB/R2021a/toolbox/eml/eml/+coder/+internal/cat.m" // pName
};

rtRunTimeErrorInfo o_emlrtRTEI = {
    13,                                                            // lineNo
    9,                                                             // colNo
    "sqrt",                                                        // fName
    "/usr/local/MATLAB/R2021a/toolbox/eml/lib/matlab/elfun/sqrt.m" // pName
};

rtRunTimeErrorInfo p_emlrtRTEI = {
    83,                                                           // lineNo
    5,                                                            // colNo
    "fltpower",                                                   // fName
    "/usr/local/MATLAB/R2021a/toolbox/eml/lib/matlab/ops/power.m" // pName
};

rtRunTimeErrorInfo q_emlrtRTEI =
    {
        19,             // lineNo
        23,             // colNo
        "scalexpAlloc", // fName
        "/usr/local/MATLAB/R2021a/toolbox/eml/eml/+coder/+internal/"
        "scalexpAlloc.m" // pName
};

bool isInitialized_topico_mex = false;

// End of code generation (topico_data.cpp)
