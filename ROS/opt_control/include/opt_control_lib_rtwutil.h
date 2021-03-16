/*
 * Student License - for use by students to meet course requirements and
 * perform academic research at degree granting institutions only.  Not
 * for government, commercial, or other organizational use.
 *
 * opt_control_lib_rtwutil.h
 *
 * Code generation for function 'opt_control_lib_rtwutil'
 *
 */

#ifndef OPT_CONTROL_LIB_RTWUTIL_H
#define OPT_CONTROL_LIB_RTWUTIL_H

/* Include files */
#include <stddef.h>
#include <stdlib.h>
#include "rtwtypes.h"
#include "opt_control_lib_types.h"

/* Function Declarations */
extern short _s16_s32_(int b);
extern void b_rtErrorWithMessageID(const rtRunTimeErrorInfo *aInfo);
extern void c_rtErrorWithMessageID(const rtRunTimeErrorInfo *aInfo);
extern void d_rtErrorWithMessageID(const rtRunTimeErrorInfo *aInfo);
extern void e_rtErrorWithMessageID(const rtRunTimeErrorInfo *aInfo);
extern void f_rtErrorWithMessageID(const int b, const char *c, const
  rtRunTimeErrorInfo *aInfo);
extern void g_rtErrorWithMessageID(const int b, const char *c, const
  rtRunTimeErrorInfo *aInfo);
extern void h_rtErrorWithMessageID(const int b, const char *c, const
  rtRunTimeErrorInfo *aInfo);
extern void i_rtErrorWithMessageID(const rtRunTimeErrorInfo *aInfo);
extern void j_rtErrorWithMessageID(const int b, const char *c, const
  rtRunTimeErrorInfo *aInfo);
extern void k_rtErrorWithMessageID(const rtRunTimeErrorInfo *aInfo);
extern void l_rtErrorWithMessageID(const rtRunTimeErrorInfo *aInfo);
extern void m_rtErrorWithMessageID(const rtRunTimeErrorInfo *aInfo);
extern void n_rtErrorWithMessageID(const rtRunTimeErrorInfo *aInfo);
extern void o_rtErrorWithMessageID(const rtRunTimeErrorInfo *aInfo);
extern void p_rtErrorWithMessageID(const int b, const char *c, const
  rtRunTimeErrorInfo *aInfo);
extern void q_rtErrorWithMessageID(const rtRunTimeErrorInfo *aInfo);
extern void r_rtErrorWithMessageID(const rtRunTimeErrorInfo *aInfo);
extern void rtDimSizeEqError(const int aDim1, const int aDim2, const
  rtEqualityCheckInfo *aInfo);
extern void rtDynamicBoundsError(int aIndexValue, int aLoBound, int aHiBound,
  const rtBoundsCheckInfo *aInfo);
extern void rtErrorWithMessageID(const rtRunTimeErrorInfo *aInfo);
extern void rtIntegerError(const double aInteger, const rtDoubleCheckInfo *aInfo);
extern void rtIntegerOverflowError(const rtRunTimeErrorInfo *aInfo);
extern void rtNonNegativeError(const double aPositive, const rtDoubleCheckInfo
  *aInfo);
extern void rtSizeEq1DError(const int aDim1, const int aDim2, const
  rtEqualityCheckInfo *aInfo);
extern void rtSizeEqNDCheck(const int *aDims1, const int *aDims2, const
  rtEqualityCheckInfo *aInfo);
extern void rtSubAssignSizeCheck(const int *aDims1, const int aNDims1, const int
  *aDims2, const int aNDims2, const rtEqualityCheckInfo *aInfo);
extern double rt_atan2d_snf(double u0, double u1);
extern double rt_hypotd_snf(double u0, double u1);
extern double rt_powd_snf(double u0, double u1);
extern void s_rtErrorWithMessageID(const rtRunTimeErrorInfo *aInfo);
extern void t_rtErrorWithMessageID(const rtRunTimeErrorInfo *aInfo);
extern void u_rtErrorWithMessageID(const rtRunTimeErrorInfo *aInfo);
extern void v_rtErrorWithMessageID(const rtRunTimeErrorInfo *aInfo);
extern void w_rtErrorWithMessageID(const int b, const int c, const
  rtRunTimeErrorInfo *aInfo);

#endif

/* End of code generation (opt_control_lib_rtwutil.h) */
