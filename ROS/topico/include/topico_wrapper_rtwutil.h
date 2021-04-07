//
// Student License - for use by students to meet course requirements and
// perform academic research at degree granting institutions only.  Not
// for government, commercial, or other organizational use.
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
extern unsigned long _u64_div__(unsigned long b, unsigned long c);

extern unsigned long _u64_minus__(unsigned long b, unsigned long c);

extern void d_rtErrorWithMessageID(const char *aFcnName, int aLineNum);

extern void f_rtErrorWithMessageID(const char *b, const char *aFcnName,
                                   int aLineNum);

extern void h_rtErrorWithMessageID(const char *aFcnName, int aLineNum);

extern void i_rtErrorWithMessageID(const char *aFcnName, int aLineNum);

extern void k_rtErrorWithMessageID(const char *aFcnName, int aLineNum);

extern void m_rtErrorWithMessageID(const char *aFcnName, int aLineNum);

extern void mul_wide_u64(unsigned long in0, unsigned long in1,
                         unsigned long *ptrOutBitsHi,
                         unsigned long *ptrOutBitsLo);

extern unsigned long mulv_u64(unsigned long a, unsigned long b);

extern void rtDivisionByZeroErrorN();

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

#endif
// End of code generation (topico_wrapper_rtwutil.h)
