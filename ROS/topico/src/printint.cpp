//
// Student License - for use by students to meet course requirements and
// perform academic research at degree granting institutions only.  Not
// for government, commercial, or other organizational use.
//
// printint.cpp
//
// Code generation for function 'printint'
//

// Include files
#include "printint.h"
#include "i64ddiv.h"
#include "rt_nonfinite.h"
#include "topico_data.h"
#include "topico_rtwutil.h"
#include "topico_types.h"
#include <cmath>
#include <stdio.h>

// Function Declarations
static unsigned long _u64_d_(double b);

static unsigned long _u64_s32_(int b);

static double rt_roundd_snf(double u);

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

static unsigned long _u64_s32_(int b)
{
  unsigned long a;
  a = static_cast<unsigned long>(b);
  if (b < 0) {
    rtIntegerOverflowErrorN();
  }
  return a;
}

static double rt_roundd_snf(double u)
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
      e_rtErrorWithMessageID(h_emlrtRTEI.fName, h_emlrtRTEI.lineNo);
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
      e_rtErrorWithMessageID(h_emlrtRTEI.fName, h_emlrtRTEI.lineNo);
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
