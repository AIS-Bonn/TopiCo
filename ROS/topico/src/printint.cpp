//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// printint.cpp
//
// Code generation for function 'printint'
//

// Include files
#include "printint.h"
#include "i64ddiv.h"
#include "rt_nonfinite.h"
#include "topico_wrapper_data.h"
#include "topico_wrapper_rtwutil.h"
#include "topico_wrapper_types.h"
#include <cmath>
#include <sstream>
#include <stdexcept>
#include <stdio.h>
#include <string>

// Function Declarations
static unsigned long _u64_d_(double b);

static unsigned long _u64_div__(unsigned long b, unsigned long c);

static unsigned long _u64_minus__(unsigned long b, unsigned long c);

static unsigned long _u64_s32_(int b);

static void mul_wide_u64(unsigned long in0, unsigned long in1,
                         unsigned long *ptrOutBitsHi,
                         unsigned long *ptrOutBitsLo);

static unsigned long mulv_u64(unsigned long a, unsigned long b);

static void rtDivisionByZeroErrorN();

// Function Definitions
static unsigned long _u64_d_(double b)
{
  unsigned long a;
  a = static_cast<unsigned long>(b);
  if ((b < 0.0) || (a != std::floor(b))) {
    rtIntegerOverflowErrorN();
  }
  return a;
}

static unsigned long _u64_div__(unsigned long b, unsigned long c)
{
  if (c == 0UL) {
    rtDivisionByZeroErrorN();
  }
  return b / c;
}

static unsigned long _u64_minus__(unsigned long b, unsigned long c)
{
  unsigned long a;
  a = b - c;
  if (b < c) {
    rtIntegerOverflowErrorN();
  }
  return a;
}

static unsigned long _u64_s32_(int b)
{
  unsigned long a;
  a = static_cast<unsigned long>(b);
  if (b < 0) {
    rtIntegerOverflowErrorN();
  }
  return a;
}

static void mul_wide_u64(unsigned long in0, unsigned long in1,
                         unsigned long *ptrOutBitsHi,
                         unsigned long *ptrOutBitsLo)
{
  unsigned long in0Hi;
  unsigned long in0Lo;
  unsigned long in1Hi;
  unsigned long in1Lo;
  unsigned long outBitsLo;
  unsigned long productHiLo;
  unsigned long productLoHi;
  in0Hi = in0 >> 32UL;
  in0Lo = in0 & 4294967295UL;
  in1Hi = in1 >> 32UL;
  in1Lo = in1 & 4294967295UL;
  productHiLo = in0Hi * in1Lo;
  productLoHi = in0Lo * in1Hi;
  in0Lo *= in1Lo;
  in1Lo = 0UL;
  outBitsLo = in0Lo + (productLoHi << 32UL);
  if (outBitsLo < in0Lo) {
    in1Lo = 1UL;
  }
  in0Lo = outBitsLo;
  outBitsLo += productHiLo << 32UL;
  if (outBitsLo < in0Lo) {
    in1Lo++;
  }
  *ptrOutBitsHi =
      ((in1Lo + in0Hi * in1Hi) + (productLoHi >> 32UL)) + (productHiLo >> 32UL);
  *ptrOutBitsLo = outBitsLo;
}

static unsigned long mulv_u64(unsigned long a, unsigned long b)
{
  unsigned long result;
  unsigned long u64_chi;
  mul_wide_u64(a, b, &u64_chi, &result);
  if (u64_chi) {
    rtIntegerOverflowErrorN();
  }
  return result;
}

static void rtDivisionByZeroErrorN()
{
  std::stringstream outStream;
  outStream << "Division by zero detected.\nEarly termination due to division "
               "by zero.";
  outStream << "\n";
  throw std::runtime_error(outStream.str());
}

void printint(unsigned int number)
{
  unsigned long b_number;
  unsigned long divisor;
  bool b_leading;
  b_number = number;
  b_leading = true;
  divisor = 10000000000000000000UL;
  while (divisor >= 1UL) {
    unsigned long d;
    if (divisor == 0UL) {
      d_rtErrorWithMessageID(f_emlrtRTEI.fName, f_emlrtRTEI.lineNo);
    }
    d = _u64_div__(b_number, divisor);
    b_leading = ((d == 0UL) && b_leading);
    if (!b_leading) {
      b_number = _u64_minus__(b_number, mulv_u64(d, divisor));
      switch (d) {
      case 0UL:
        printf("0");
        fflush(stdout);
        break;
      case 1UL:
        printf("1");
        fflush(stdout);
        break;
      case 2UL:
        printf("2");
        fflush(stdout);
        break;
      case 3UL:
        printf("3");
        fflush(stdout);
        break;
      case 4UL:
        printf("4");
        fflush(stdout);
        break;
      case 5UL:
        printf("5");
        fflush(stdout);
        break;
      case 6UL:
        printf("6");
        fflush(stdout);
        break;
      case 7UL:
        printf("7");
        fflush(stdout);
        break;
      case 8UL:
        printf("8");
        fflush(stdout);
        break;
      case 9UL:
        printf("9");
        fflush(stdout);
        break;
      }
    }
    divisor = coder::internal::i64ddiv(divisor);
  }
  if (b_leading) {
    printf("0");
    fflush(stdout);
  }
}

void printint(int number)
{
  unsigned long b_number;
  unsigned long divisor;
  bool b_leading;
  b_number = _u64_s32_(number);
  b_leading = true;
  divisor = 10000000000000000000UL;
  while (divisor >= 1UL) {
    unsigned long d;
    if (divisor == 0UL) {
      d_rtErrorWithMessageID(f_emlrtRTEI.fName, f_emlrtRTEI.lineNo);
    }
    d = _u64_div__(b_number, divisor);
    b_leading = ((d == 0UL) && b_leading);
    if (!b_leading) {
      b_number = _u64_minus__(b_number, mulv_u64(d, divisor));
      switch (d) {
      case 0UL:
        printf("0");
        fflush(stdout);
        break;
      case 1UL:
        printf("1");
        fflush(stdout);
        break;
      case 2UL:
        printf("2");
        fflush(stdout);
        break;
      case 3UL:
        printf("3");
        fflush(stdout);
        break;
      case 4UL:
        printf("4");
        fflush(stdout);
        break;
      case 5UL:
        printf("5");
        fflush(stdout);
        break;
      case 6UL:
        printf("6");
        fflush(stdout);
        break;
      case 7UL:
        printf("7");
        fflush(stdout);
        break;
      case 8UL:
        printf("8");
        fflush(stdout);
        break;
      case 9UL:
        printf("9");
        fflush(stdout);
        break;
      }
    }
    divisor = coder::internal::i64ddiv(divisor);
  }
  if (b_leading) {
    printf("0");
    fflush(stdout);
  }
}

void printint(double number)
{
  unsigned long b_number;
  unsigned long divisor;
  bool b_leading;
  b_number = _u64_d_(rt_roundd_snf(std::abs(number)));
  b_leading = true;
  divisor = 10000000000000000000UL;
  while (divisor >= 1UL) {
    unsigned long d;
    if (divisor == 0UL) {
      d_rtErrorWithMessageID(f_emlrtRTEI.fName, f_emlrtRTEI.lineNo);
    }
    d = _u64_div__(b_number, divisor);
    b_leading = ((d == 0UL) && b_leading);
    if (!b_leading) {
      b_number = _u64_minus__(b_number, mulv_u64(d, divisor));
      switch (d) {
      case 0UL:
        printf("0");
        fflush(stdout);
        break;
      case 1UL:
        printf("1");
        fflush(stdout);
        break;
      case 2UL:
        printf("2");
        fflush(stdout);
        break;
      case 3UL:
        printf("3");
        fflush(stdout);
        break;
      case 4UL:
        printf("4");
        fflush(stdout);
        break;
      case 5UL:
        printf("5");
        fflush(stdout);
        break;
      case 6UL:
        printf("6");
        fflush(stdout);
        break;
      case 7UL:
        printf("7");
        fflush(stdout);
        break;
      case 8UL:
        printf("8");
        fflush(stdout);
        break;
      case 9UL:
        printf("9");
        fflush(stdout);
        break;
      }
    }
    divisor = coder::internal::i64ddiv(divisor);
  }
  if (b_leading) {
    printf("0");
    fflush(stdout);
  }
}

// End of code generation (printint.cpp)
