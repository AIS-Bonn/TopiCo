//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// topico_wrapper_rtwutil.h
//
// Code generation for function 'topico_wrapper_rtwutil'
//

#ifndef TOPICO_WRAPPER_RTWUTIL_H
#define TOPICO_WRAPPER_RTWUTIL_H

// Include files
#include "rtwtypes.h"
#include "topico_wrapper_types.h"
#include <cstddef>
#include <cstdlib>
#include <string>

// Function Declarations
extern void d_rtErrorWithMessageID(const char *aFcnName, int aLineNum);

extern void f_rtErrorWithMessageID(const char *b, const char *aFcnName,
                                   int aLineNum);

extern void h_rtErrorWithMessageID(const char *aFcnName, int aLineNum);

extern void i_rtErrorWithMessageID(const char *aFcnName, int aLineNum);

extern void k_rtErrorWithMessageID(const char *aFcnName, int aLineNum);

extern void m_rtErrorWithMessageID(const char *aFcnName, int aLineNum);

extern void rtDynamicBoundsError(int aIndexValue, int aLoBound, int aHiBound,
                                 const rtBoundsCheckInfo *aInfo);

extern std::string rtGenSizeString(const int aNDims, const int *aDims);

extern void rtIntegerOverflowErrorN();

extern bool rtIsNullOrEmptyString(const char *aString);

extern void rtSizeEq1DError(const int aDim1, const int aDim2,
                            const rtEqualityCheckInfo *aInfo);

extern void rtSizeEqNDCheck(const int *aDims1, const int *aDims2,
                            const rtEqualityCheckInfo *aInfo);

extern void rtSubAssignSizeCheck(const int *aDims1, const int aNDims1,
                                 const int *aDims2, const int aNDims2,
                                 const rtEqualityCheckInfo *aInfo);

extern double rt_atan2d_snf(double u0, double u1);

extern double rt_hypotd_snf(double u0, double u1);

extern double rt_powd_snf(double u0, double u1);

extern double rt_roundd_snf(double u);

#endif
// End of code generation (topico_wrapper_rtwutil.h)
