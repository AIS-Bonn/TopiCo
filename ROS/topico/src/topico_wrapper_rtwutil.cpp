//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// topico_wrapper_rtwutil.cpp
//
// Code generation for function 'topico_wrapper_rtwutil'
//

// Include files
#include "topico_wrapper_rtwutil.h"
#include "rt_nonfinite.h"
#include "topico_wrapper_types.h"
#include "rt_defines.h"
#include "rt_nonfinite.h"
#include <cmath>
#include <sstream>
#include <stdexcept>
#include <string>

// Function Definitions
void d_rtErrorWithMessageID(const char *aFcnName, int aLineNum)
{
  std::stringstream outStream;
  outStream << "Divide by zero.";
  outStream << "\n";
  ((((outStream << "Error in ") << aFcnName) << " (line ") << aLineNum) << ")";
  throw std::runtime_error(outStream.str());
}

void f_rtErrorWithMessageID(const char *b, const char *aFcnName, int aLineNum)
{
  std::stringstream outStream;
  ((outStream << "Domain error. To compute complex results from real x, use \'")
   << b)
      << "(complex(x))\'.";
  outStream << "\n";
  ((((outStream << "Error in ") << aFcnName) << " (line ") << aLineNum) << ")";
  throw std::runtime_error(outStream.str());
}

void h_rtErrorWithMessageID(const char *aFcnName, int aLineNum)
{
  std::stringstream outStream;
  outStream << "Dimensions of arrays being concatenated are not consistent.";
  outStream << "\n";
  ((((outStream << "Error in ") << aFcnName) << " (line ") << aLineNum) << ")";
  throw std::runtime_error(outStream.str());
}

void i_rtErrorWithMessageID(const char *aFcnName, int aLineNum)
{
  std::stringstream outStream;
  outStream << "Assertion failed.";
  outStream << "\n";
  ((((outStream << "Error in ") << aFcnName) << " (line ") << aLineNum) << ")";
  throw std::runtime_error(outStream.str());
}

void k_rtErrorWithMessageID(const char *aFcnName, int aLineNum)
{
  std::stringstream outStream;
  outStream << "Domain error. To compute complex results, make at least one "
               "input complex, e.g. \'power(complex(a),b)\'.";
  outStream << "\n";
  ((((outStream << "Error in ") << aFcnName) << " (line ") << aLineNum) << ")";
  throw std::runtime_error(outStream.str());
}

void m_rtErrorWithMessageID(const char *aFcnName, int aLineNum)
{
  std::stringstream outStream;
  outStream << "Matrix dimensions must agree.";
  outStream << "\n";
  ((((outStream << "Error in ") << aFcnName) << " (line ") << aLineNum) << ")";
  throw std::runtime_error(outStream.str());
}

void rtDynamicBoundsError(int aIndexValue, int aLoBound, int aHiBound,
                          const rtBoundsCheckInfo *aInfo)
{
  std::stringstream b_outStream;
  std::stringstream outStream;
  if (aLoBound == 0) {
    aIndexValue++;
    aLoBound = 1;
    aHiBound++;
  }
  if (rtIsNullOrEmptyString(aInfo->aName)) {
    ((((((b_outStream << "Index exceeds array dimensions.  Index value ")
         << aIndexValue)
        << " exceeds valid range [")
       << aLoBound)
      << "-")
     << aHiBound)
        << "].";
    b_outStream << "\n";
    ((((b_outStream << "Error in ") << aInfo->fName) << " (line ")
     << aInfo->lineNo)
        << ")";
    throw std::runtime_error(b_outStream.str());
  } else {
    ((((((((outStream << "Index exceeds array dimensions. Index value ")
           << aIndexValue)
          << " exceeds valid range [")
         << aLoBound)
        << "-")
       << aHiBound)
      << "] for array \'")
     << aInfo->aName)
        << "\'.";
    outStream << "\n";
    ((((outStream << "Error in ") << aInfo->fName) << " (line ")
     << aInfo->lineNo)
        << ")";
    throw std::runtime_error(outStream.str());
  }
}

std::string rtGenSizeString(const int aNDims, const int *aDims)
{
  std::stringstream outStream;
  for (int i = 0; i < aNDims; i++) {
    outStream << "[";
    outStream << aDims[i];
    outStream << "]";
  }
  return outStream.str();
}

void rtIntegerOverflowErrorN()
{
  std::stringstream outStream;
  outStream << "Integer overflow detected.\nEarly termination due to integer "
               "overflow.";
  outStream << "\n";
  throw std::runtime_error(outStream.str());
}

bool rtIsNullOrEmptyString(const char *aString)
{
  return (aString == NULL) || (*aString == '\x00');
}

void rtSizeEq1DError(const int aDim1, const int aDim2,
                     const rtEqualityCheckInfo *aInfo)
{
  std::stringstream outStream;
  ((((outStream << "Sizes mismatch: ") << aDim1) << " ~= ") << aDim2) << ".";
  outStream << "\n";
  ((((outStream << "Error in ") << aInfo->fName) << " (line ") << aInfo->lineNo)
      << ")";
  throw std::runtime_error(outStream.str());
}

void rtSizeEqNDCheck(const int *aDims1, const int *aDims2,
                     const rtEqualityCheckInfo *aInfo)
{
  std::stringstream outStream;
  for (int i = 0; i < aInfo->nDims; i++) {
    if (aDims1[i] != aDims2[i]) {
      std::string dims1Str;
      std::string dims2Str;
      dims1Str = rtGenSizeString(aInfo->nDims, aDims1);
      dims2Str = rtGenSizeString(aInfo->nDims, aDims2);
      ((((outStream << "Sizes mismatch: ") << dims1Str) << " ~= ") << dims2Str)
          << ".";
      outStream << "\n";
      ((((outStream << "Error in ") << aInfo->fName) << " (line ")
       << aInfo->lineNo)
          << ")";
      throw std::runtime_error(outStream.str());
    }
  }
}

void rtSubAssignSizeCheck(const int *aDims1, const int aNDims1,
                          const int *aDims2, const int aNDims2,
                          const rtEqualityCheckInfo *aInfo)
{
  std::stringstream outStream;
  int i;
  int j;
  i = 0;
  j = 0;
  while ((i < aNDims1) && (j < aNDims2)) {
    while ((i < aNDims1) && (aDims1[i] == 1)) {
      i++;
    }
    while ((j < aNDims2) && (aDims2[j] == 1)) {
      j++;
    }
    if (((i < aNDims1) || (j < aNDims2)) &&
        ((i == aNDims1) || ((j == aNDims2) || ((aDims1[i] != -1) &&
                                               ((aDims2[j] != -1) &&
                                                (aDims1[i] != aDims2[j])))))) {
      std::string dims1Str;
      std::string dims2Str;
      dims1Str = rtGenSizeString(aNDims1, aDims1);
      dims2Str = rtGenSizeString(aNDims1, aDims2);
      ((((outStream << "Subscripted assignment dimension mismatch: ")
         << dims1Str)
        << " ~= ")
       << dims2Str)
          << ".";
      outStream << "\n";
      ((((outStream << "Error in ") << aInfo->fName) << " (line ")
       << aInfo->lineNo)
          << ")";
      throw std::runtime_error(outStream.str());
    }
    i++;
    j++;
  }
}

double rt_atan2d_snf(double u0, double u1)
{
  double y;
  if (rtIsNaN(u0) || rtIsNaN(u1)) {
    y = rtNaN;
  } else if (rtIsInf(u0) && rtIsInf(u1)) {
    int b_u0;
    int b_u1;
    if (u0 > 0.0) {
      b_u0 = 1;
    } else {
      b_u0 = -1;
    }
    if (u1 > 0.0) {
      b_u1 = 1;
    } else {
      b_u1 = -1;
    }
    y = std::atan2(static_cast<double>(b_u0), static_cast<double>(b_u1));
  } else if (u1 == 0.0) {
    if (u0 > 0.0) {
      y = RT_PI / 2.0;
    } else if (u0 < 0.0) {
      y = -(RT_PI / 2.0);
    } else {
      y = 0.0;
    }
  } else {
    y = std::atan2(u0, u1);
  }
  return y;
}

double rt_hypotd_snf(double u0, double u1)
{
  double a;
  double y;
  a = std::abs(u0);
  y = std::abs(u1);
  if (a < y) {
    a /= y;
    y *= std::sqrt(a * a + 1.0);
  } else if (a > y) {
    y /= a;
    y = a * std::sqrt(y * y + 1.0);
  } else if (!rtIsNaN(y)) {
    y = a * 1.4142135623730951;
  }
  return y;
}

double rt_powd_snf(double u0, double u1)
{
  double y;
  if (rtIsNaN(u0) || rtIsNaN(u1)) {
    y = rtNaN;
  } else {
    double d;
    double d1;
    d = std::abs(u0);
    d1 = std::abs(u1);
    if (rtIsInf(u1)) {
      if (d == 1.0) {
        y = 1.0;
      } else if (d > 1.0) {
        if (u1 > 0.0) {
          y = rtInf;
        } else {
          y = 0.0;
        }
      } else if (u1 > 0.0) {
        y = 0.0;
      } else {
        y = rtInf;
      }
    } else if (d1 == 0.0) {
      y = 1.0;
    } else if (d1 == 1.0) {
      if (u1 > 0.0) {
        y = u0;
      } else {
        y = 1.0 / u0;
      }
    } else if (u1 == 2.0) {
      y = u0 * u0;
    } else if ((u1 == 0.5) && (u0 >= 0.0)) {
      y = std::sqrt(u0);
    } else if ((u0 < 0.0) && (u1 > std::floor(u1))) {
      y = rtNaN;
    } else {
      y = std::pow(u0, u1);
    }
  }
  return y;
}

double rt_roundd_snf(double u)
{
  double y;
  if (std::abs(u) < 4.503599627370496E+15) {
    if (u >= 0.5) {
      y = std::floor(u + 0.5);
    } else if (u > -0.5) {
      y = u * 0.0;
    } else {
      y = std::ceil(u - 0.5);
    }
  } else {
    y = u;
  }
  return y;
}

// End of code generation (topico_wrapper_rtwutil.cpp)
