//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// sub2ind.cpp
//
// Code generation for function 'sub2ind'
//

// Include files
#include "sub2ind.h"
#include "rt_nonfinite.h"
#include "topico_wrapper_types.h"
#include <sstream>
#include <stdexcept>
#include <string>

// Variable Definitions
static rtRunTimeErrorInfo m_emlrtRTEI = {
    41,                                                               // lineNo
    19,                                                               // colNo
    "eml_sub2ind",                                                    // fName
    "/usr/local/MATLAB/R2021a/toolbox/eml/lib/matlab/elmat/sub2ind.m" // pName
};

// Function Declarations
static void j_rtErrorWithMessageID(const char *aFcnName, int aLineNum);

// Function Definitions
static void j_rtErrorWithMessageID(const char *aFcnName, int aLineNum)
{
  std::stringstream outStream;
  outStream << "Out of range subscript.";
  outStream << "\n";
  ((((outStream << "Error in ") << aFcnName) << " (line ") << aLineNum) << ")";
  throw std::runtime_error(outStream.str());
}

namespace coder {
int b_eml_sub2ind(double varargin_1, double varargin_2, double varargin_4,
                  double varargin_5)
{
  if (!(varargin_1 <= 3.0)) {
    j_rtErrorWithMessageID(m_emlrtRTEI.fName, m_emlrtRTEI.lineNo);
  }
  if (!(varargin_2 <= 2.0)) {
    j_rtErrorWithMessageID(m_emlrtRTEI.fName, m_emlrtRTEI.lineNo);
  }
  if (!(varargin_4 <= 2.0)) {
    j_rtErrorWithMessageID(m_emlrtRTEI.fName, m_emlrtRTEI.lineNo);
  }
  if (!(varargin_5 <= 2.0)) {
    j_rtErrorWithMessageID(m_emlrtRTEI.fName, m_emlrtRTEI.lineNo);
  }
  return ((static_cast<int>(varargin_1) +
           3 * (static_cast<int>(varargin_2) - 1)) +
          12 * (static_cast<int>(varargin_4) - 1)) +
         24 * (static_cast<int>(varargin_5) - 1);
}

int eml_sub2ind(double varargin_1, double varargin_3, double varargin_4,
                double varargin_5)
{
  if (!(varargin_1 <= 3.0)) {
    j_rtErrorWithMessageID(m_emlrtRTEI.fName, m_emlrtRTEI.lineNo);
  }
  if (!(varargin_3 <= 2.0)) {
    j_rtErrorWithMessageID(m_emlrtRTEI.fName, m_emlrtRTEI.lineNo);
  }
  if (!(varargin_4 <= 2.0)) {
    j_rtErrorWithMessageID(m_emlrtRTEI.fName, m_emlrtRTEI.lineNo);
  }
  if (!(varargin_5 <= 2.0)) {
    j_rtErrorWithMessageID(m_emlrtRTEI.fName, m_emlrtRTEI.lineNo);
  }
  return ((static_cast<int>(varargin_1) +
           6 * (static_cast<int>(varargin_3) - 1)) +
          12 * (static_cast<int>(varargin_4) - 1)) +
         24 * (static_cast<int>(varargin_5) - 1);
}

int eml_sub2ind(double varargin_1, double varargin_2, double varargin_3)
{
  if (!(varargin_1 <= 2.0)) {
    j_rtErrorWithMessageID(m_emlrtRTEI.fName, m_emlrtRTEI.lineNo);
  }
  if (!(varargin_2 <= 2.0)) {
    j_rtErrorWithMessageID(m_emlrtRTEI.fName, m_emlrtRTEI.lineNo);
  }
  if (!(varargin_3 <= 2.0)) {
    j_rtErrorWithMessageID(m_emlrtRTEI.fName, m_emlrtRTEI.lineNo);
  }
  return (static_cast<int>(varargin_1) +
          ((static_cast<int>(varargin_2) - 1) << 1)) +
         ((static_cast<int>(varargin_3) - 1) << 2);
}

int eml_sub2ind(double varargin_1, double varargin_2, double varargin_3,
                double varargin_4, double varargin_5)
{
  if (!(varargin_1 <= 3.0)) {
    j_rtErrorWithMessageID(m_emlrtRTEI.fName, m_emlrtRTEI.lineNo);
  }
  if (!(varargin_2 <= 2.0)) {
    j_rtErrorWithMessageID(m_emlrtRTEI.fName, m_emlrtRTEI.lineNo);
  }
  if (!(varargin_3 <= 2.0)) {
    j_rtErrorWithMessageID(m_emlrtRTEI.fName, m_emlrtRTEI.lineNo);
  }
  if (!(varargin_4 <= 2.0)) {
    j_rtErrorWithMessageID(m_emlrtRTEI.fName, m_emlrtRTEI.lineNo);
  }
  if (!(varargin_5 <= 2.0)) {
    j_rtErrorWithMessageID(m_emlrtRTEI.fName, m_emlrtRTEI.lineNo);
  }
  return (((static_cast<int>(varargin_1) +
            3 * (static_cast<int>(varargin_2) - 1)) +
           6 * (static_cast<int>(varargin_3) - 1)) +
          12 * (static_cast<int>(varargin_4) - 1)) +
         24 * (static_cast<int>(varargin_5) - 1);
}

} // namespace coder

// End of code generation (sub2ind.cpp)
