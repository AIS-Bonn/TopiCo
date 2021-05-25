//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// validateattributes.cpp
//
// Code generation for function 'validateattributes'
//

// Include files
#include "validateattributes.h"
#include "rt_nonfinite.h"
#include "topico_wrapper_types.h"
#include "coder_array.h"
#include "rt_nonfinite.h"
#include <cmath>
#include <sstream>
#include <stdexcept>
#include <string>

// Variable Definitions
static rtRunTimeErrorInfo emlrtRTEI = {
    10,             // lineNo
    23,             // colNo
    "validatesize", // fName
    "/usr/local/MATLAB/R2021a/toolbox/eml/eml/+coder/+internal/+valattr/"
    "validatesize.m" // pName
};

static rtRunTimeErrorInfo b_emlrtRTEI = {
    15,             // lineNo
    19,             // colNo
    "validatesize", // fName
    "/usr/local/MATLAB/R2021a/toolbox/eml/eml/+coder/+internal/+valattr/"
    "validatesize.m" // pName
};

static rtRunTimeErrorInfo c_emlrtRTEI = {
    14,               // lineNo
    37,               // colNo
    "validatefinite", // fName
    "/usr/local/MATLAB/R2021a/toolbox/eml/eml/+coder/+internal/+valattr/"
    "validatefinite.m" // pName
};

static rtRunTimeErrorInfo d_emlrtRTEI = {
    14,               // lineNo
    37,               // colNo
    "validatenonnan", // fName
    "/usr/local/MATLAB/R2021a/toolbox/eml/eml/+coder/+internal/+valattr/"
    "validatenonnan.m" // pName
};

// Function Declarations
static void b_rtErrorWithMessageID(const char *b, const char *aFcnName,
                                   int aLineNum);

static void c_rtErrorWithMessageID(const char *aFcnName, int aLineNum);

static void c_rtErrorWithMessageID(const char *b, const char *aFcnName,
                                   int aLineNum);

static void d_rtErrorWithMessageID(const char *b, const char *aFcnName,
                                   int aLineNum);

static void rtErrorWithMessageID(const char *b, const char *aFcnName,
                                 int aLineNum);

// Function Definitions
static void b_rtErrorWithMessageID(const char *b, const char *aFcnName,
                                   int aLineNum)
{
  std::stringstream outStream;
  ((outStream << "Expected ") << b) << " to be finite.";
  outStream << "\n";
  ((((outStream << "Error in ") << aFcnName) << " (line ") << aLineNum) << ")";
  throw std::runtime_error(outStream.str());
}

static void c_rtErrorWithMessageID(const char *b, const char *aFcnName,
                                   int aLineNum)
{
  std::stringstream outStream;
  ((outStream << "Expected ") << b) << " to be non-NaN.";
  outStream << "\n";
  ((((outStream << "Error in ") << aFcnName) << " (line ") << aLineNum) << ")";
  throw std::runtime_error(outStream.str());
}

static void c_rtErrorWithMessageID(const char *aFcnName, int aLineNum)
{
  std::stringstream outStream;
  outStream
      << "Argument \'size\' must be followed by a real array of integers.";
  outStream << "\n";
  ((((outStream << "Error in ") << aFcnName) << " (line ") << aLineNum) << ")";
  throw std::runtime_error(outStream.str());
}

static void d_rtErrorWithMessageID(const char *b, const char *aFcnName,
                                   int aLineNum)
{
  std::stringstream outStream;
  ((outStream << "Expected ") << b) << " to be positive.";
  outStream << "\n";
  ((((outStream << "Error in ") << aFcnName) << " (line ") << aLineNum) << ")";
  throw std::runtime_error(outStream.str());
}

static void rtErrorWithMessageID(const char *b, const char *aFcnName,
                                 int aLineNum)
{
  std::stringstream outStream;
  ((outStream << "") << b) << " does not have the expected size.";
  outStream << "\n";
  ((((outStream << "Error in ") << aFcnName) << " (line ") << aLineNum) << ")";
  throw std::runtime_error(outStream.str());
}

namespace coder {
void b_validateattributes(const ::coder::array<double, 2U> &a,
                          const double attributes_f2[2])
{
  int i;
  int k;
  bool exitg1;
  bool p;
  if (((!(attributes_f2[0] != attributes_f2[0])) &&
       (rtIsInf(attributes_f2[0]) || (!(attributes_f2[0] >= 0.0)) ||
        (!(attributes_f2[0] == std::floor(attributes_f2[0]))))) ||
      ((!(attributes_f2[1] != attributes_f2[1])) &&
       (rtIsInf(attributes_f2[1]) || (!(attributes_f2[1] >= 0.0)) ||
        (!(attributes_f2[1] == std::floor(attributes_f2[1])))))) {
    c_rtErrorWithMessageID(emlrtRTEI.fName, emlrtRTEI.lineNo);
  }
  if (((!(attributes_f2[0] != attributes_f2[0])) &&
       (!(attributes_f2[0] == a.size(0)))) ||
      ((!(attributes_f2[1] != attributes_f2[1])) &&
       (!(attributes_f2[1] == a.size(1))))) {
    rtErrorWithMessageID("Input number 3, V_max,", b_emlrtRTEI.fName,
                         b_emlrtRTEI.lineNo);
  }
  p = true;
  i = a.size(0) * a.size(1);
  k = 0;
  exitg1 = false;
  while ((!exitg1) && (k <= i - 1)) {
    if (!rtIsNaN(a[k])) {
      k++;
    } else {
      p = false;
      exitg1 = true;
    }
  }
  if (!p) {
    c_rtErrorWithMessageID("input number 3, V_max,", d_emlrtRTEI.fName,
                           d_emlrtRTEI.lineNo);
  }
}

void c_validateattributes(const ::coder::array<double, 2U> &a,
                          const double attributes_f2[2])
{
  int i;
  int k;
  bool exitg1;
  bool p;
  if (((!(attributes_f2[0] != attributes_f2[0])) &&
       (rtIsInf(attributes_f2[0]) || (!(attributes_f2[0] >= 0.0)) ||
        (!(attributes_f2[0] == std::floor(attributes_f2[0]))))) ||
      ((!(attributes_f2[1] != attributes_f2[1])) &&
       (rtIsInf(attributes_f2[1]) || (!(attributes_f2[1] >= 0.0)) ||
        (!(attributes_f2[1] == std::floor(attributes_f2[1])))))) {
    c_rtErrorWithMessageID(emlrtRTEI.fName, emlrtRTEI.lineNo);
  }
  if (((!(attributes_f2[0] != attributes_f2[0])) &&
       (!(attributes_f2[0] == a.size(0)))) ||
      ((!(attributes_f2[1] != attributes_f2[1])) &&
       (!(attributes_f2[1] == a.size(1))))) {
    rtErrorWithMessageID("Input number 4, V_min,", b_emlrtRTEI.fName,
                         b_emlrtRTEI.lineNo);
  }
  p = true;
  i = a.size(0) * a.size(1);
  k = 0;
  exitg1 = false;
  while ((!exitg1) && (k <= i - 1)) {
    if (!rtIsNaN(a[k])) {
      k++;
    } else {
      p = false;
      exitg1 = true;
    }
  }
  if (!p) {
    c_rtErrorWithMessageID("input number 4, V_min,", d_emlrtRTEI.fName,
                           d_emlrtRTEI.lineNo);
  }
}

void d_validateattributes(const ::coder::array<double, 2U> &a,
                          const double attributes_f2[2])
{
  int i;
  int k;
  bool exitg1;
  bool p;
  if (((!(attributes_f2[0] != attributes_f2[0])) &&
       (rtIsInf(attributes_f2[0]) || (!(attributes_f2[0] >= 0.0)) ||
        (!(attributes_f2[0] == std::floor(attributes_f2[0]))))) ||
      ((!(attributes_f2[1] != attributes_f2[1])) &&
       (rtIsInf(attributes_f2[1]) || (!(attributes_f2[1] >= 0.0)) ||
        (!(attributes_f2[1] == std::floor(attributes_f2[1])))))) {
    c_rtErrorWithMessageID(emlrtRTEI.fName, emlrtRTEI.lineNo);
  }
  if (((!(attributes_f2[0] != attributes_f2[0])) &&
       (!(attributes_f2[0] == a.size(0)))) ||
      ((!(attributes_f2[1] != attributes_f2[1])) &&
       (!(attributes_f2[1] == a.size(1))))) {
    rtErrorWithMessageID("Input number 5, A_max,", b_emlrtRTEI.fName,
                         b_emlrtRTEI.lineNo);
  }
  p = true;
  i = a.size(0) * a.size(1);
  k = 0;
  exitg1 = false;
  while ((!exitg1) && (k <= i - 1)) {
    if (!rtIsNaN(a[k])) {
      k++;
    } else {
      p = false;
      exitg1 = true;
    }
  }
  if (!p) {
    c_rtErrorWithMessageID("input number 5, A_max,", d_emlrtRTEI.fName,
                           d_emlrtRTEI.lineNo);
  }
}

void e_validateattributes(const ::coder::array<double, 2U> &a,
                          const double attributes_f2[2])
{
  int i;
  int k;
  bool exitg1;
  bool p;
  if (((!(attributes_f2[0] != attributes_f2[0])) &&
       (rtIsInf(attributes_f2[0]) || (!(attributes_f2[0] >= 0.0)) ||
        (!(attributes_f2[0] == std::floor(attributes_f2[0]))))) ||
      ((!(attributes_f2[1] != attributes_f2[1])) &&
       (rtIsInf(attributes_f2[1]) || (!(attributes_f2[1] >= 0.0)) ||
        (!(attributes_f2[1] == std::floor(attributes_f2[1])))))) {
    c_rtErrorWithMessageID(emlrtRTEI.fName, emlrtRTEI.lineNo);
  }
  if (((!(attributes_f2[0] != attributes_f2[0])) &&
       (!(attributes_f2[0] == a.size(0)))) ||
      ((!(attributes_f2[1] != attributes_f2[1])) &&
       (!(attributes_f2[1] == a.size(1))))) {
    rtErrorWithMessageID("Input number 6, A_min,", b_emlrtRTEI.fName,
                         b_emlrtRTEI.lineNo);
  }
  p = true;
  i = a.size(0) * a.size(1);
  k = 0;
  exitg1 = false;
  while ((!exitg1) && (k <= i - 1)) {
    if (!rtIsNaN(a[k])) {
      k++;
    } else {
      p = false;
      exitg1 = true;
    }
  }
  if (!p) {
    c_rtErrorWithMessageID("input number 6, A_min,", d_emlrtRTEI.fName,
                           d_emlrtRTEI.lineNo);
  }
}

void f_validateattributes(const ::coder::array<double, 2U> &a,
                          const double attributes_f2[2])
{
  int i;
  int k;
  bool exitg1;
  bool p;
  if (((!(attributes_f2[0] != attributes_f2[0])) &&
       (rtIsInf(attributes_f2[0]) || (!(attributes_f2[0] >= 0.0)) ||
        (!(attributes_f2[0] == std::floor(attributes_f2[0]))))) ||
      ((!(attributes_f2[1] != attributes_f2[1])) &&
       (rtIsInf(attributes_f2[1]) || (!(attributes_f2[1] >= 0.0)) ||
        (!(attributes_f2[1] == std::floor(attributes_f2[1])))))) {
    c_rtErrorWithMessageID(emlrtRTEI.fName, emlrtRTEI.lineNo);
  }
  if (((!(attributes_f2[0] != attributes_f2[0])) &&
       (!(attributes_f2[0] == a.size(0)))) ||
      ((!(attributes_f2[1] != attributes_f2[1])) &&
       (!(attributes_f2[1] == a.size(1))))) {
    rtErrorWithMessageID("Input number 7, J_max,", b_emlrtRTEI.fName,
                         b_emlrtRTEI.lineNo);
  }
  p = true;
  i = a.size(0) * a.size(1);
  k = 0;
  exitg1 = false;
  while ((!exitg1) && (k <= i - 1)) {
    if (!rtIsNaN(a[k])) {
      k++;
    } else {
      p = false;
      exitg1 = true;
    }
  }
  if (!p) {
    c_rtErrorWithMessageID("input number 7, J_max,", d_emlrtRTEI.fName,
                           d_emlrtRTEI.lineNo);
  }
}

void g_validateattributes(const ::coder::array<double, 2U> &a,
                          const double attributes_f2[2])
{
  int i;
  int k;
  bool exitg1;
  bool p;
  if (((!(attributes_f2[0] != attributes_f2[0])) &&
       (rtIsInf(attributes_f2[0]) || (!(attributes_f2[0] >= 0.0)) ||
        (!(attributes_f2[0] == std::floor(attributes_f2[0]))))) ||
      ((!(attributes_f2[1] != attributes_f2[1])) &&
       (rtIsInf(attributes_f2[1]) || (!(attributes_f2[1] >= 0.0)) ||
        (!(attributes_f2[1] == std::floor(attributes_f2[1])))))) {
    c_rtErrorWithMessageID(emlrtRTEI.fName, emlrtRTEI.lineNo);
  }
  if (((!(attributes_f2[0] != attributes_f2[0])) &&
       (!(attributes_f2[0] == a.size(0)))) ||
      ((!(attributes_f2[1] != attributes_f2[1])) &&
       (!(attributes_f2[1] == a.size(1))))) {
    rtErrorWithMessageID("Input number 8, J_min,", b_emlrtRTEI.fName,
                         b_emlrtRTEI.lineNo);
  }
  p = true;
  i = a.size(0) * a.size(1);
  k = 0;
  exitg1 = false;
  while ((!exitg1) && (k <= i - 1)) {
    if (!rtIsNaN(a[k])) {
      k++;
    } else {
      p = false;
      exitg1 = true;
    }
  }
  if (!p) {
    c_rtErrorWithMessageID("input number 8, J_min,", d_emlrtRTEI.fName,
                           d_emlrtRTEI.lineNo);
  }
}

void h_validateattributes(const ::coder::array<bool, 2U> &a,
                          const double attributes_f2[2])
{
  if (((!(attributes_f2[0] != attributes_f2[0])) &&
       (rtIsInf(attributes_f2[0]) || (!(attributes_f2[0] >= 0.0)) ||
        (!(attributes_f2[0] == std::floor(attributes_f2[0]))))) ||
      ((!(attributes_f2[1] != attributes_f2[1])) &&
       (rtIsInf(attributes_f2[1]) || (!(attributes_f2[1] >= 0.0)) ||
        (!(attributes_f2[1] == std::floor(attributes_f2[1])))))) {
    c_rtErrorWithMessageID(emlrtRTEI.fName, emlrtRTEI.lineNo);
  }
  if (((!(attributes_f2[0] != attributes_f2[0])) &&
       (!(attributes_f2[0] == a.size(0)))) ||
      ((!(attributes_f2[1] != attributes_f2[1])) &&
       (!(attributes_f2[1] == a.size(1))))) {
    rtErrorWithMessageID("Input number 10, b_sync_V,", b_emlrtRTEI.fName,
                         b_emlrtRTEI.lineNo);
  }
}

void i_validateattributes(const ::coder::array<bool, 2U> &a,
                          const double attributes_f2[2])
{
  if (((!(attributes_f2[0] != attributes_f2[0])) &&
       (rtIsInf(attributes_f2[0]) || (!(attributes_f2[0] >= 0.0)) ||
        (!(attributes_f2[0] == std::floor(attributes_f2[0]))))) ||
      ((!(attributes_f2[1] != attributes_f2[1])) &&
       (rtIsInf(attributes_f2[1]) || (!(attributes_f2[1] >= 0.0)) ||
        (!(attributes_f2[1] == std::floor(attributes_f2[1])))))) {
    c_rtErrorWithMessageID(emlrtRTEI.fName, emlrtRTEI.lineNo);
  }
  if (((!(attributes_f2[0] != attributes_f2[0])) &&
       (!(attributes_f2[0] == a.size(0)))) ||
      ((!(attributes_f2[1] != attributes_f2[1])) &&
       (!(attributes_f2[1] == a.size(1))))) {
    rtErrorWithMessageID("Input number 11, b_sync_A,", b_emlrtRTEI.fName,
                         b_emlrtRTEI.lineNo);
  }
}

void j_validateattributes(const ::coder::array<bool, 2U> &a,
                          const double attributes_f2[2])
{
  if (((!(attributes_f2[0] != attributes_f2[0])) &&
       (rtIsInf(attributes_f2[0]) || (!(attributes_f2[0] >= 0.0)) ||
        (!(attributes_f2[0] == std::floor(attributes_f2[0]))))) ||
      ((!(attributes_f2[1] != attributes_f2[1])) &&
       (rtIsInf(attributes_f2[1]) || (!(attributes_f2[1] >= 0.0)) ||
        (!(attributes_f2[1] == std::floor(attributes_f2[1])))))) {
    c_rtErrorWithMessageID(emlrtRTEI.fName, emlrtRTEI.lineNo);
  }
  if (((!(attributes_f2[0] != attributes_f2[0])) &&
       (!(attributes_f2[0] == a.size(0)))) ||
      ((!(attributes_f2[1] != attributes_f2[1])) &&
       (!(attributes_f2[1] == a.size(1))))) {
    rtErrorWithMessageID("Input number 12, b_sync_J,", b_emlrtRTEI.fName,
                         b_emlrtRTEI.lineNo);
  }
}

void k_validateattributes(const ::coder::array<bool, 2U> &a,
                          const double attributes_f2[2])
{
  if (((!(attributes_f2[0] != attributes_f2[0])) &&
       (rtIsInf(attributes_f2[0]) || (!(attributes_f2[0] >= 0.0)) ||
        (!(attributes_f2[0] == std::floor(attributes_f2[0]))))) ||
      ((!(attributes_f2[1] != attributes_f2[1])) &&
       (rtIsInf(attributes_f2[1]) || (!(attributes_f2[1] >= 0.0)) ||
        (!(attributes_f2[1] == std::floor(attributes_f2[1])))))) {
    c_rtErrorWithMessageID(emlrtRTEI.fName, emlrtRTEI.lineNo);
  }
  if (((!(attributes_f2[0] != attributes_f2[0])) &&
       (!(attributes_f2[0] == a.size(0)))) ||
      ((!(attributes_f2[1] != attributes_f2[1])) &&
       (!(attributes_f2[1] == a.size(1))))) {
    rtErrorWithMessageID("Input number 13, b_sync_W,", b_emlrtRTEI.fName,
                         b_emlrtRTEI.lineNo);
  }
}

void l_validateattributes(const ::coder::array<bool, 2U> &a,
                          const double attributes_f2[2])
{
  if (((!(attributes_f2[0] != attributes_f2[0])) &&
       (rtIsInf(attributes_f2[0]) || (!(attributes_f2[0] >= 0.0)) ||
        (!(attributes_f2[0] == std::floor(attributes_f2[0]))))) ||
      ((!(attributes_f2[1] != attributes_f2[1])) &&
       (rtIsInf(attributes_f2[1]) || (!(attributes_f2[1] >= 0.0)) ||
        (!(attributes_f2[1] == std::floor(attributes_f2[1])))))) {
    c_rtErrorWithMessageID(emlrtRTEI.fName, emlrtRTEI.lineNo);
  }
  if (((!(attributes_f2[0] != attributes_f2[0])) &&
       (!(attributes_f2[0] == a.size(0)))) ||
      ((!(attributes_f2[1] != attributes_f2[1])) &&
       (!(attributes_f2[1] == a.size(1))))) {
    rtErrorWithMessageID("Input number 14, b_rotate,", b_emlrtRTEI.fName,
                         b_emlrtRTEI.lineNo);
  }
}

void m_validateattributes(const ::coder::array<bool, 2U> &a,
                          const double attributes_f2[2])
{
  if (((!(attributes_f2[0] != attributes_f2[0])) &&
       (rtIsInf(attributes_f2[0]) || (!(attributes_f2[0] >= 0.0)) ||
        (!(attributes_f2[0] == std::floor(attributes_f2[0]))))) ||
      ((!(attributes_f2[1] != attributes_f2[1])) &&
       (rtIsInf(attributes_f2[1]) || (!(attributes_f2[1] >= 0.0)) ||
        (!(attributes_f2[1] == std::floor(attributes_f2[1])))))) {
    c_rtErrorWithMessageID(emlrtRTEI.fName, emlrtRTEI.lineNo);
  }
  if (((!(attributes_f2[0] != attributes_f2[0])) &&
       (!(attributes_f2[0] == a.size(0)))) ||
      ((!(attributes_f2[1] != attributes_f2[1])) &&
       (!(attributes_f2[1] == a.size(1))))) {
    rtErrorWithMessageID("Input number 15, b_hard_V_lim,", b_emlrtRTEI.fName,
                         b_emlrtRTEI.lineNo);
  }
}

void n_validateattributes(const ::coder::array<bool, 2U> &a,
                          const double attributes_f2[2])
{
  if (((!(attributes_f2[0] != attributes_f2[0])) &&
       (rtIsInf(attributes_f2[0]) || (!(attributes_f2[0] >= 0.0)) ||
        (!(attributes_f2[0] == std::floor(attributes_f2[0]))))) ||
      ((!(attributes_f2[1] != attributes_f2[1])) &&
       (rtIsInf(attributes_f2[1]) || (!(attributes_f2[1] >= 0.0)) ||
        (!(attributes_f2[1] == std::floor(attributes_f2[1])))))) {
    c_rtErrorWithMessageID(emlrtRTEI.fName, emlrtRTEI.lineNo);
  }
  if (((!(attributes_f2[0] != attributes_f2[0])) &&
       (!(attributes_f2[0] == a.size(0)))) ||
      ((!(attributes_f2[1] != attributes_f2[1])) &&
       (!(attributes_f2[1] == a.size(1))))) {
    rtErrorWithMessageID("Input number 16, b_catch_up,", b_emlrtRTEI.fName,
                         b_emlrtRTEI.lineNo);
  }
}

void validateattributes(const ::coder::array<signed char, 2U> &a,
                        const double attributes_f2[2])
{
  if (((!(attributes_f2[0] != attributes_f2[0])) &&
       (rtIsInf(attributes_f2[0]) || (!(attributes_f2[0] >= 0.0)) ||
        (!(attributes_f2[0] == std::floor(attributes_f2[0]))))) ||
      ((!(attributes_f2[1] != attributes_f2[1])) &&
       (rtIsInf(attributes_f2[1]) || (!(attributes_f2[1] >= 0.0)) ||
        (!(attributes_f2[1] == std::floor(attributes_f2[1])))))) {
    c_rtErrorWithMessageID(emlrtRTEI.fName, emlrtRTEI.lineNo);
  }
  if (((!(attributes_f2[0] != attributes_f2[0])) &&
       (!(attributes_f2[0] == a.size(0)))) ||
      ((!(attributes_f2[1] != attributes_f2[1])) &&
       (!(attributes_f2[1] == a.size(1))))) {
    rtErrorWithMessageID("Input number 17, direction,", b_emlrtRTEI.fName,
                         b_emlrtRTEI.lineNo);
  }
}

void validateattributes(const ::coder::array<double, 2U> &a,
                        const double attributes_f2[2])
{
  int i;
  int k;
  bool exitg1;
  bool p;
  if (((!(attributes_f2[0] != attributes_f2[0])) &&
       (rtIsInf(attributes_f2[0]) || (!(attributes_f2[0] >= 0.0)) ||
        (!(attributes_f2[0] == std::floor(attributes_f2[0]))))) ||
      ((!(attributes_f2[1] != attributes_f2[1])) &&
       (rtIsInf(attributes_f2[1]) || (!(attributes_f2[1] >= 0.0)) ||
        (!(attributes_f2[1] == std::floor(attributes_f2[1])))))) {
    c_rtErrorWithMessageID(emlrtRTEI.fName, emlrtRTEI.lineNo);
  }
  if (((!(attributes_f2[0] != attributes_f2[0])) &&
       (!(attributes_f2[0] == a.size(0)))) ||
      ((!(attributes_f2[1] != attributes_f2[1])) &&
       (!(attributes_f2[1] == 3.0)))) {
    rtErrorWithMessageID("Input number 1, State_start,", b_emlrtRTEI.fName,
                         b_emlrtRTEI.lineNo);
  }
  p = true;
  i = a.size(0) * 3;
  k = 0;
  exitg1 = false;
  while ((!exitg1) && (k <= i - 1)) {
    if ((!rtIsInf(a[k])) && (!rtIsNaN(a[k]))) {
      k++;
    } else {
      p = false;
      exitg1 = true;
    }
  }
  if (!p) {
    b_rtErrorWithMessageID("input number 1, State_start,", c_emlrtRTEI.fName,
                           c_emlrtRTEI.lineNo);
  }
}

void validateattributes(double a)
{
  static rtRunTimeErrorInfo s_emlrtRTEI = {
      14,                 // lineNo
      37,                 // colNo
      "validatepositive", // fName
      "/usr/local/MATLAB/R2021a/toolbox/eml/eml/+coder/+internal/+valattr/"
      "validatepositive.m" // pName
  };
  bool p;
  p = !rtIsNaN(a);
  if (!p) {
    c_rtErrorWithMessageID("input number 18, ts_rollout,", d_emlrtRTEI.fName,
                           d_emlrtRTEI.lineNo);
  }
  p = !(a <= 0.0);
  if (!p) {
    d_rtErrorWithMessageID("input number 18, ts_rollout,", s_emlrtRTEI.fName,
                           s_emlrtRTEI.lineNo);
  }
}

void validateattributes(const ::coder::array<double, 1U> &a,
                        const double attributes_f2[2])
{
  int k;
  bool exitg1;
  bool p;
  if (((!(attributes_f2[0] != attributes_f2[0])) &&
       (rtIsInf(attributes_f2[0]) || (!(attributes_f2[0] >= 0.0)) ||
        (!(attributes_f2[0] == std::floor(attributes_f2[0]))))) ||
      ((!(attributes_f2[1] != attributes_f2[1])) &&
       (rtIsInf(attributes_f2[1]) || (!(attributes_f2[1] >= 0.0)) ||
        (!(attributes_f2[1] == std::floor(attributes_f2[1])))))) {
    c_rtErrorWithMessageID(emlrtRTEI.fName, emlrtRTEI.lineNo);
  }
  if (((!(attributes_f2[0] != attributes_f2[0])) &&
       (!(attributes_f2[0] == a.size(0)))) ||
      ((!(attributes_f2[1] != attributes_f2[1])) &&
       (!(attributes_f2[1] == 1.0)))) {
    rtErrorWithMessageID("Input number 9, A_global,", b_emlrtRTEI.fName,
                         b_emlrtRTEI.lineNo);
  }
  p = true;
  k = 0;
  exitg1 = false;
  while ((!exitg1) && (k <= a.size(0) - 1)) {
    if ((!rtIsInf(a[k])) && (!rtIsNaN(a[k]))) {
      k++;
    } else {
      p = false;
      exitg1 = true;
    }
  }
  if (!p) {
    b_rtErrorWithMessageID("input number 9, A_global,", c_emlrtRTEI.fName,
                           c_emlrtRTEI.lineNo);
  }
}

void validateattributes(const ::coder::array<double, 3U> &a,
                        const double attributes_f2[3])
{
  if (((!(attributes_f2[0] != attributes_f2[0])) &&
       (rtIsInf(attributes_f2[0]) ||
        (!(attributes_f2[0] == std::floor(attributes_f2[0]))))) ||
      ((!(attributes_f2[1] != attributes_f2[1])) &&
       (rtIsInf(attributes_f2[1]) ||
        (!(attributes_f2[1] == std::floor(attributes_f2[1]))))) ||
      ((!(attributes_f2[2] != attributes_f2[2])) &&
       (rtIsInf(attributes_f2[2]) ||
        (!(attributes_f2[2] == std::floor(attributes_f2[2])))))) {
    c_rtErrorWithMessageID(emlrtRTEI.fName, emlrtRTEI.lineNo);
  }
  if (((!(attributes_f2[0] != attributes_f2[0])) &&
       (!(attributes_f2[0] == a.size(0)))) ||
      ((!(attributes_f2[1] != attributes_f2[1])) &&
       (!(attributes_f2[1] == 5.0))) ||
      ((!(attributes_f2[2] != attributes_f2[2])) &&
       (!(attributes_f2[2] == a.size(2))))) {
    rtErrorWithMessageID("Input number 2, Waypoints,", b_emlrtRTEI.fName,
                         b_emlrtRTEI.lineNo);
  }
}

} // namespace coder

// End of code generation (validateattributes.cpp)
