//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// indexShapeCheck.cpp
//
// Code generation for function 'indexShapeCheck'
//

// Include files
#include "indexShapeCheck.h"
#include "rt_nonfinite.h"
#include "topico_wrapper_types.h"
#include <sstream>
#include <stdexcept>
#include <string>

// Variable Definitions
static rtRunTimeErrorInfo h_emlrtRTEI = {
    121,           // lineNo
    5,             // colNo
    "errOrWarnIf", // fName
    "/usr/local/MATLAB/R2021a/toolbox/eml/eml/+coder/+internal/"
    "indexShapeCheck.m" // pName
};

// Function Declarations
static void g_rtErrorWithMessageID(const char *aFcnName, int aLineNum);

static void l_rtErrorWithMessageID(const char *aFcnName, int aLineNum);

// Function Definitions
static void g_rtErrorWithMessageID(const char *aFcnName, int aLineNum)
{
  std::stringstream outStream;
  outStream
      << "Code generation assumption about size violated. Run-time indexing is "
         "scalar(vector), but compile-time assumption was vector(vect"
         "or) indexing.";
  outStream << "\n";
  ((((outStream << "Error in ") << aFcnName) << " (line ") << aLineNum) << ")";
  throw std::runtime_error(outStream.str());
}

static void l_rtErrorWithMessageID(const char *aFcnName, int aLineNum)
{
  std::stringstream outStream;
  outStream
      << "Code generation assumption about size violated. Run-time indexing is "
         "vector(vector) with different orientations, but compile-tim"
         "e assumption was matrix(vector) indexing.";
  outStream << "\n";
  ((((outStream << "Error in ") << aFcnName) << " (line ") << aLineNum) << ")";
  throw std::runtime_error(outStream.str());
}

namespace coder {
namespace internal {
void indexShapeCheck(const int matrixSize[2])
{
  bool nonSingletonDimFound;
  nonSingletonDimFound = (matrixSize[0] != 1);
  if (matrixSize[1] != 1) {
    if (nonSingletonDimFound) {
      nonSingletonDimFound = false;
    } else {
      nonSingletonDimFound = true;
    }
  }
  if (nonSingletonDimFound && (matrixSize[0] != 1)) {
    l_rtErrorWithMessageID(h_emlrtRTEI.fName, h_emlrtRTEI.lineNo);
  }
}

void indexShapeCheck(int matrixSize)
{
  if (matrixSize == 1) {
    g_rtErrorWithMessageID(h_emlrtRTEI.fName, h_emlrtRTEI.lineNo);
  }
}

} // namespace internal
} // namespace coder

// End of code generation (indexShapeCheck.cpp)
