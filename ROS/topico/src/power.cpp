//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// power.cpp
//
// Code generation for function 'power'
//

// Include files
#include "power.h"
#include "rt_nonfinite.h"
#include "topico_wrapper_rtwutil.h"
#include "rt_nonfinite.h"
#include <cmath>

// Function Definitions
namespace coder {
void b_power(const creal_T a[4], creal_T y[4])
{
  double r;
  double y_im;
  if ((a[0].im == 0.0) && (a[0].re >= 0.0)) {
    y[0].re = rt_powd_snf(a[0].re, 3.0);
    y[0].im = 0.0;
  } else if (a[0].re == 0.0) {
    y[0].re = 0.0;
    y[0].im = -rt_powd_snf(a[0].im, 3.0);
  } else {
    if (a[0].im == 0.0) {
      if (a[0].re < 0.0) {
        r = std::log(std::abs(a[0].re));
        y_im = 3.1415926535897931;
      } else {
        r = std::log(std::abs(a[0].re));
        y_im = 0.0;
      }
    } else if ((std::abs(a[0].re) > 8.9884656743115785E+307) ||
               (std::abs(a[0].im) > 8.9884656743115785E+307)) {
      r = std::log(rt_hypotd_snf(a[0].re / 2.0, a[0].im / 2.0)) +
          0.69314718055994529;
      y_im = rt_atan2d_snf(a[0].im, a[0].re);
    } else {
      r = std::log(rt_hypotd_snf(a[0].re, a[0].im));
      y_im = rt_atan2d_snf(a[0].im, a[0].re);
    }
    r *= 3.0;
    y_im *= 3.0;
    if (y_im == 0.0) {
      y[0].re = std::exp(r);
      y[0].im = 0.0;
    } else if (rtIsInf(y_im) && rtIsInf(r) && (r < 0.0)) {
      y[0].re = 0.0;
      y[0].im = 0.0;
    } else {
      r = std::exp(r / 2.0);
      y[0].re = r * (r * std::cos(y_im));
      y[0].im = r * (r * std::sin(y_im));
    }
  }
  if ((a[1].im == 0.0) && (a[1].re >= 0.0)) {
    y[1].re = rt_powd_snf(a[1].re, 3.0);
    y[1].im = 0.0;
  } else if (a[1].re == 0.0) {
    y[1].re = 0.0;
    y[1].im = -rt_powd_snf(a[1].im, 3.0);
  } else {
    if (a[1].im == 0.0) {
      if (a[1].re < 0.0) {
        r = std::log(std::abs(a[1].re));
        y_im = 3.1415926535897931;
      } else {
        r = std::log(std::abs(a[1].re));
        y_im = 0.0;
      }
    } else if ((std::abs(a[1].re) > 8.9884656743115785E+307) ||
               (std::abs(a[1].im) > 8.9884656743115785E+307)) {
      r = std::log(rt_hypotd_snf(a[1].re / 2.0, a[1].im / 2.0)) +
          0.69314718055994529;
      y_im = rt_atan2d_snf(a[1].im, a[1].re);
    } else {
      r = std::log(rt_hypotd_snf(a[1].re, a[1].im));
      y_im = rt_atan2d_snf(a[1].im, a[1].re);
    }
    r *= 3.0;
    y_im *= 3.0;
    if (y_im == 0.0) {
      y[1].re = std::exp(r);
      y[1].im = 0.0;
    } else if (rtIsInf(y_im) && rtIsInf(r) && (r < 0.0)) {
      y[1].re = 0.0;
      y[1].im = 0.0;
    } else {
      r = std::exp(r / 2.0);
      y[1].re = r * (r * std::cos(y_im));
      y[1].im = r * (r * std::sin(y_im));
    }
  }
  if ((a[2].im == 0.0) && (a[2].re >= 0.0)) {
    y[2].re = rt_powd_snf(a[2].re, 3.0);
    y[2].im = 0.0;
  } else if (a[2].re == 0.0) {
    y[2].re = 0.0;
    y[2].im = -rt_powd_snf(a[2].im, 3.0);
  } else {
    if (a[2].im == 0.0) {
      if (a[2].re < 0.0) {
        r = std::log(std::abs(a[2].re));
        y_im = 3.1415926535897931;
      } else {
        r = std::log(std::abs(a[2].re));
        y_im = 0.0;
      }
    } else if ((std::abs(a[2].re) > 8.9884656743115785E+307) ||
               (std::abs(a[2].im) > 8.9884656743115785E+307)) {
      r = std::log(rt_hypotd_snf(a[2].re / 2.0, a[2].im / 2.0)) +
          0.69314718055994529;
      y_im = rt_atan2d_snf(a[2].im, a[2].re);
    } else {
      r = std::log(rt_hypotd_snf(a[2].re, a[2].im));
      y_im = rt_atan2d_snf(a[2].im, a[2].re);
    }
    r *= 3.0;
    y_im *= 3.0;
    if (y_im == 0.0) {
      y[2].re = std::exp(r);
      y[2].im = 0.0;
    } else if (rtIsInf(y_im) && rtIsInf(r) && (r < 0.0)) {
      y[2].re = 0.0;
      y[2].im = 0.0;
    } else {
      r = std::exp(r / 2.0);
      y[2].re = r * (r * std::cos(y_im));
      y[2].im = r * (r * std::sin(y_im));
    }
  }
  if ((a[3].im == 0.0) && (a[3].re >= 0.0)) {
    y[3].re = rt_powd_snf(a[3].re, 3.0);
    y[3].im = 0.0;
  } else if (a[3].re == 0.0) {
    y[3].re = 0.0;
    y[3].im = -rt_powd_snf(a[3].im, 3.0);
  } else {
    if (a[3].im == 0.0) {
      if (a[3].re < 0.0) {
        r = std::log(std::abs(a[3].re));
        y_im = 3.1415926535897931;
      } else {
        r = std::log(std::abs(a[3].re));
        y_im = 0.0;
      }
    } else if ((std::abs(a[3].re) > 8.9884656743115785E+307) ||
               (std::abs(a[3].im) > 8.9884656743115785E+307)) {
      r = std::log(rt_hypotd_snf(a[3].re / 2.0, a[3].im / 2.0)) +
          0.69314718055994529;
      y_im = rt_atan2d_snf(a[3].im, a[3].re);
    } else {
      r = std::log(rt_hypotd_snf(a[3].re, a[3].im));
      y_im = rt_atan2d_snf(a[3].im, a[3].re);
    }
    r *= 3.0;
    y_im *= 3.0;
    if (y_im == 0.0) {
      y[3].re = std::exp(r);
      y[3].im = 0.0;
    } else if (rtIsInf(y_im) && rtIsInf(r) && (r < 0.0)) {
      y[3].re = 0.0;
      y[3].im = 0.0;
    } else {
      r = std::exp(r / 2.0);
      y[3].re = r * (r * std::cos(y_im));
      y[3].im = r * (r * std::sin(y_im));
    }
  }
}

creal_T b_power(const creal_T a)
{
  creal_T y;
  if ((a.im == 0.0) && (a.re >= 0.0)) {
    y.re = rt_powd_snf(a.re, 0.16666666666666666);
    y.im = 0.0;
  } else {
    double b_im;
    double r;
    if (a.im == 0.0) {
      if (a.re < 0.0) {
        r = std::log(std::abs(a.re));
        b_im = 3.1415926535897931;
      } else {
        r = std::log(a.re);
        b_im = 0.0;
      }
    } else if ((std::abs(a.re) > 8.9884656743115785E+307) ||
               (std::abs(a.im) > 8.9884656743115785E+307)) {
      r = std::log(rt_hypotd_snf(a.re / 2.0, a.im / 2.0)) + 0.69314718055994529;
      b_im = rt_atan2d_snf(a.im, a.re);
    } else {
      r = std::log(rt_hypotd_snf(a.re, a.im));
      b_im = rt_atan2d_snf(a.im, a.re);
    }
    y.re = 0.16666666666666666 * r;
    y.im = 0.16666666666666666 * b_im;
    if (y.im == 0.0) {
      b_im = y.re;
      y.re = std::exp(b_im);
      y.im = 0.0;
    } else if (rtIsInf(y.im) && rtIsInf(y.re) && (y.re < 0.0)) {
      y.re = 0.0;
      y.im = 0.0;
    } else {
      double d;
      r = std::exp(y.re / 2.0);
      b_im = y.im;
      d = y.im;
      y.re = r * (r * std::cos(b_im));
      y.im = r * (r * std::sin(d));
    }
  }
  return y;
}

creal_T c_power(const creal_T a)
{
  creal_T y;
  if ((a.im == 0.0) && (a.re >= 0.0)) {
    y.re = rt_powd_snf(a.re, 0.25);
    y.im = 0.0;
  } else {
    double b_im;
    double r;
    if (a.im == 0.0) {
      if (a.re < 0.0) {
        r = std::log(std::abs(a.re));
        b_im = 3.1415926535897931;
      } else {
        r = std::log(a.re);
        b_im = 0.0;
      }
    } else if ((std::abs(a.re) > 8.9884656743115785E+307) ||
               (std::abs(a.im) > 8.9884656743115785E+307)) {
      r = std::log(rt_hypotd_snf(a.re / 2.0, a.im / 2.0)) + 0.69314718055994529;
      b_im = rt_atan2d_snf(a.im, a.re);
    } else {
      r = std::log(rt_hypotd_snf(a.re, a.im));
      b_im = rt_atan2d_snf(a.im, a.re);
    }
    y.re = 0.25 * r;
    y.im = 0.25 * b_im;
    if (y.im == 0.0) {
      b_im = y.re;
      y.re = std::exp(b_im);
      y.im = 0.0;
    } else if (rtIsInf(y.im) && rtIsInf(y.re) && (y.re < 0.0)) {
      y.re = 0.0;
      y.im = 0.0;
    } else {
      double d;
      r = std::exp(y.re / 2.0);
      b_im = y.im;
      d = y.im;
      y.re = r * (r * std::cos(b_im));
      y.im = r * (r * std::sin(d));
    }
  }
  return y;
}

void c_power(const creal_T a[3], creal_T y[3])
{
  double r;
  double y_im;
  if ((a[0].im == 0.0) && (a[0].re >= 0.0)) {
    y[0].re = rt_powd_snf(a[0].re, 3.0);
    y[0].im = 0.0;
  } else if (a[0].re == 0.0) {
    y[0].re = 0.0;
    y[0].im = -rt_powd_snf(a[0].im, 3.0);
  } else {
    if (a[0].im == 0.0) {
      if (a[0].re < 0.0) {
        r = std::log(std::abs(a[0].re));
        y_im = 3.1415926535897931;
      } else {
        r = std::log(std::abs(a[0].re));
        y_im = 0.0;
      }
    } else if ((std::abs(a[0].re) > 8.9884656743115785E+307) ||
               (std::abs(a[0].im) > 8.9884656743115785E+307)) {
      r = std::log(rt_hypotd_snf(a[0].re / 2.0, a[0].im / 2.0)) +
          0.69314718055994529;
      y_im = rt_atan2d_snf(a[0].im, a[0].re);
    } else {
      r = std::log(rt_hypotd_snf(a[0].re, a[0].im));
      y_im = rt_atan2d_snf(a[0].im, a[0].re);
    }
    r *= 3.0;
    y_im *= 3.0;
    if (y_im == 0.0) {
      y[0].re = std::exp(r);
      y[0].im = 0.0;
    } else if (rtIsInf(y_im) && rtIsInf(r) && (r < 0.0)) {
      y[0].re = 0.0;
      y[0].im = 0.0;
    } else {
      r = std::exp(r / 2.0);
      y[0].re = r * (r * std::cos(y_im));
      y[0].im = r * (r * std::sin(y_im));
    }
  }
  if ((a[1].im == 0.0) && (a[1].re >= 0.0)) {
    y[1].re = rt_powd_snf(a[1].re, 3.0);
    y[1].im = 0.0;
  } else if (a[1].re == 0.0) {
    y[1].re = 0.0;
    y[1].im = -rt_powd_snf(a[1].im, 3.0);
  } else {
    if (a[1].im == 0.0) {
      if (a[1].re < 0.0) {
        r = std::log(std::abs(a[1].re));
        y_im = 3.1415926535897931;
      } else {
        r = std::log(std::abs(a[1].re));
        y_im = 0.0;
      }
    } else if ((std::abs(a[1].re) > 8.9884656743115785E+307) ||
               (std::abs(a[1].im) > 8.9884656743115785E+307)) {
      r = std::log(rt_hypotd_snf(a[1].re / 2.0, a[1].im / 2.0)) +
          0.69314718055994529;
      y_im = rt_atan2d_snf(a[1].im, a[1].re);
    } else {
      r = std::log(rt_hypotd_snf(a[1].re, a[1].im));
      y_im = rt_atan2d_snf(a[1].im, a[1].re);
    }
    r *= 3.0;
    y_im *= 3.0;
    if (y_im == 0.0) {
      y[1].re = std::exp(r);
      y[1].im = 0.0;
    } else if (rtIsInf(y_im) && rtIsInf(r) && (r < 0.0)) {
      y[1].re = 0.0;
      y[1].im = 0.0;
    } else {
      r = std::exp(r / 2.0);
      y[1].re = r * (r * std::cos(y_im));
      y[1].im = r * (r * std::sin(y_im));
    }
  }
  if ((a[2].im == 0.0) && (a[2].re >= 0.0)) {
    y[2].re = rt_powd_snf(a[2].re, 3.0);
    y[2].im = 0.0;
  } else if (a[2].re == 0.0) {
    y[2].re = 0.0;
    y[2].im = -rt_powd_snf(a[2].im, 3.0);
  } else {
    if (a[2].im == 0.0) {
      if (a[2].re < 0.0) {
        r = std::log(std::abs(a[2].re));
        y_im = 3.1415926535897931;
      } else {
        r = std::log(std::abs(a[2].re));
        y_im = 0.0;
      }
    } else if ((std::abs(a[2].re) > 8.9884656743115785E+307) ||
               (std::abs(a[2].im) > 8.9884656743115785E+307)) {
      r = std::log(rt_hypotd_snf(a[2].re / 2.0, a[2].im / 2.0)) +
          0.69314718055994529;
      y_im = rt_atan2d_snf(a[2].im, a[2].re);
    } else {
      r = std::log(rt_hypotd_snf(a[2].re, a[2].im));
      y_im = rt_atan2d_snf(a[2].im, a[2].re);
    }
    r *= 3.0;
    y_im *= 3.0;
    if (y_im == 0.0) {
      y[2].re = std::exp(r);
      y[2].im = 0.0;
    } else if (rtIsInf(y_im) && rtIsInf(r) && (r < 0.0)) {
      y[2].re = 0.0;
      y[2].im = 0.0;
    } else {
      r = std::exp(r / 2.0);
      y[2].re = r * (r * std::cos(y_im));
      y[2].im = r * (r * std::sin(y_im));
    }
  }
}

creal_T d_power(const creal_T a)
{
  creal_T y;
  if ((a.im == 0.0) && (a.re >= 0.0)) {
    y.re = rt_powd_snf(a.re, 3.0);
    y.im = 0.0;
  } else if (a.re == 0.0) {
    y.re = 0.0;
    y.im = -rt_powd_snf(a.im, 3.0);
  } else {
    double b_im;
    double r;
    if (a.im == 0.0) {
      if (a.re < 0.0) {
        r = std::log(std::abs(a.re));
        b_im = 3.1415926535897931;
      } else {
        r = std::log(a.re);
        b_im = 0.0;
      }
    } else if ((std::abs(a.re) > 8.9884656743115785E+307) ||
               (std::abs(a.im) > 8.9884656743115785E+307)) {
      r = std::log(rt_hypotd_snf(a.re / 2.0, a.im / 2.0)) + 0.69314718055994529;
      b_im = rt_atan2d_snf(a.im, a.re);
    } else {
      r = std::log(rt_hypotd_snf(a.re, a.im));
      b_im = rt_atan2d_snf(a.im, a.re);
    }
    y.re = 3.0 * r;
    y.im = 3.0 * b_im;
    if (y.im == 0.0) {
      b_im = y.re;
      y.re = std::exp(b_im);
      y.im = 0.0;
    } else if (rtIsInf(y.im) && rtIsInf(y.re) && (y.re < 0.0)) {
      y.re = 0.0;
      y.im = 0.0;
    } else {
      double d;
      r = std::exp(y.re / 2.0);
      b_im = y.im;
      d = y.im;
      y.re = r * (r * std::cos(b_im));
      y.im = r * (r * std::sin(d));
    }
  }
  return y;
}

creal_T e_power(const creal_T a)
{
  creal_T y;
  if ((a.im == 0.0) && (a.re >= 0.0)) {
    y.re = rt_powd_snf(a.re, 5.0);
    y.im = 0.0;
  } else if (a.re == 0.0) {
    y.re = 0.0;
    y.im = rt_powd_snf(a.im, 5.0);
  } else {
    double b_im;
    double r;
    if (a.im == 0.0) {
      if (a.re < 0.0) {
        r = std::log(std::abs(a.re));
        b_im = 3.1415926535897931;
      } else {
        r = std::log(a.re);
        b_im = 0.0;
      }
    } else if ((std::abs(a.re) > 8.9884656743115785E+307) ||
               (std::abs(a.im) > 8.9884656743115785E+307)) {
      r = std::log(rt_hypotd_snf(a.re / 2.0, a.im / 2.0)) + 0.69314718055994529;
      b_im = rt_atan2d_snf(a.im, a.re);
    } else {
      r = std::log(rt_hypotd_snf(a.re, a.im));
      b_im = rt_atan2d_snf(a.im, a.re);
    }
    y.re = 5.0 * r;
    y.im = 5.0 * b_im;
    if (y.im == 0.0) {
      b_im = y.re;
      y.re = std::exp(b_im);
      y.im = 0.0;
    } else if (rtIsInf(y.im) && rtIsInf(y.re) && (y.re < 0.0)) {
      y.re = 0.0;
      y.im = 0.0;
    } else {
      double d;
      r = std::exp(y.re / 2.0);
      b_im = y.im;
      d = y.im;
      y.re = r * (r * std::cos(b_im));
      y.im = r * (r * std::sin(d));
    }
  }
  return y;
}

creal_T f_power(const creal_T a)
{
  creal_T y;
  if ((a.im == 0.0) && (a.re >= 0.0)) {
    y.re = rt_powd_snf(a.re, 7.0);
    y.im = 0.0;
  } else if (a.re == 0.0) {
    y.re = 0.0;
    y.im = -rt_powd_snf(a.im, 7.0);
  } else {
    double b_im;
    double r;
    if (a.im == 0.0) {
      if (a.re < 0.0) {
        r = std::log(std::abs(a.re));
        b_im = 3.1415926535897931;
      } else {
        r = std::log(a.re);
        b_im = 0.0;
      }
    } else if ((std::abs(a.re) > 8.9884656743115785E+307) ||
               (std::abs(a.im) > 8.9884656743115785E+307)) {
      r = std::log(rt_hypotd_snf(a.re / 2.0, a.im / 2.0)) + 0.69314718055994529;
      b_im = rt_atan2d_snf(a.im, a.re);
    } else {
      r = std::log(rt_hypotd_snf(a.re, a.im));
      b_im = rt_atan2d_snf(a.im, a.re);
    }
    y.re = 7.0 * r;
    y.im = 7.0 * b_im;
    if (y.im == 0.0) {
      b_im = y.re;
      y.re = std::exp(b_im);
      y.im = 0.0;
    } else if (rtIsInf(y.im) && rtIsInf(y.re) && (y.re < 0.0)) {
      y.re = 0.0;
      y.im = 0.0;
    } else {
      double d;
      r = std::exp(y.re / 2.0);
      b_im = y.im;
      d = y.im;
      y.re = r * (r * std::cos(b_im));
      y.im = r * (r * std::sin(d));
    }
  }
  return y;
}

void power(const creal_T a[2], creal_T y[2])
{
  double r;
  double y_im;
  if ((a[0].im == 0.0) && (a[0].re >= 0.0)) {
    y[0].re = rt_powd_snf(a[0].re, 3.0);
    y[0].im = 0.0;
  } else if (a[0].re == 0.0) {
    y[0].re = 0.0;
    y[0].im = -rt_powd_snf(a[0].im, 3.0);
  } else {
    if (a[0].im == 0.0) {
      if (a[0].re < 0.0) {
        r = std::log(std::abs(a[0].re));
        y_im = 3.1415926535897931;
      } else {
        r = std::log(std::abs(a[0].re));
        y_im = 0.0;
      }
    } else if ((std::abs(a[0].re) > 8.9884656743115785E+307) ||
               (std::abs(a[0].im) > 8.9884656743115785E+307)) {
      r = std::log(rt_hypotd_snf(a[0].re / 2.0, a[0].im / 2.0)) +
          0.69314718055994529;
      y_im = rt_atan2d_snf(a[0].im, a[0].re);
    } else {
      r = std::log(rt_hypotd_snf(a[0].re, a[0].im));
      y_im = rt_atan2d_snf(a[0].im, a[0].re);
    }
    r *= 3.0;
    y_im *= 3.0;
    if (y_im == 0.0) {
      y[0].re = std::exp(r);
      y[0].im = 0.0;
    } else if (rtIsInf(y_im) && rtIsInf(r) && (r < 0.0)) {
      y[0].re = 0.0;
      y[0].im = 0.0;
    } else {
      r = std::exp(r / 2.0);
      y[0].re = r * (r * std::cos(y_im));
      y[0].im = r * (r * std::sin(y_im));
    }
  }
  if ((a[1].im == 0.0) && (a[1].re >= 0.0)) {
    y[1].re = rt_powd_snf(a[1].re, 3.0);
    y[1].im = 0.0;
  } else if (a[1].re == 0.0) {
    y[1].re = 0.0;
    y[1].im = -rt_powd_snf(a[1].im, 3.0);
  } else {
    if (a[1].im == 0.0) {
      if (a[1].re < 0.0) {
        r = std::log(std::abs(a[1].re));
        y_im = 3.1415926535897931;
      } else {
        r = std::log(std::abs(a[1].re));
        y_im = 0.0;
      }
    } else if ((std::abs(a[1].re) > 8.9884656743115785E+307) ||
               (std::abs(a[1].im) > 8.9884656743115785E+307)) {
      r = std::log(rt_hypotd_snf(a[1].re / 2.0, a[1].im / 2.0)) +
          0.69314718055994529;
      y_im = rt_atan2d_snf(a[1].im, a[1].re);
    } else {
      r = std::log(rt_hypotd_snf(a[1].re, a[1].im));
      y_im = rt_atan2d_snf(a[1].im, a[1].re);
    }
    r *= 3.0;
    y_im *= 3.0;
    if (y_im == 0.0) {
      y[1].re = std::exp(r);
      y[1].im = 0.0;
    } else if (rtIsInf(y_im) && rtIsInf(r) && (r < 0.0)) {
      y[1].re = 0.0;
      y[1].im = 0.0;
    } else {
      r = std::exp(r / 2.0);
      y[1].re = r * (r * std::cos(y_im));
      y[1].im = r * (r * std::sin(y_im));
    }
  }
}

creal_T power(const creal_T a)
{
  creal_T y;
  if ((a.im == 0.0) && (a.re >= 0.0)) {
    y.re = rt_powd_snf(a.re, 0.33333333333333331);
    y.im = 0.0;
  } else {
    double b_im;
    double r;
    if (a.im == 0.0) {
      if (a.re < 0.0) {
        r = std::log(std::abs(a.re));
        b_im = 3.1415926535897931;
      } else {
        r = std::log(a.re);
        b_im = 0.0;
      }
    } else if ((std::abs(a.re) > 8.9884656743115785E+307) ||
               (std::abs(a.im) > 8.9884656743115785E+307)) {
      r = std::log(rt_hypotd_snf(a.re / 2.0, a.im / 2.0)) + 0.69314718055994529;
      b_im = rt_atan2d_snf(a.im, a.re);
    } else {
      r = std::log(rt_hypotd_snf(a.re, a.im));
      b_im = rt_atan2d_snf(a.im, a.re);
    }
    y.re = 0.33333333333333331 * r;
    y.im = 0.33333333333333331 * b_im;
    if (y.im == 0.0) {
      b_im = y.re;
      y.re = std::exp(b_im);
      y.im = 0.0;
    } else if (rtIsInf(y.im) && rtIsInf(y.re) && (y.re < 0.0)) {
      y.re = 0.0;
      y.im = 0.0;
    } else {
      double d;
      r = std::exp(y.re / 2.0);
      b_im = y.im;
      d = y.im;
      y.re = r * (r * std::cos(b_im));
      y.im = r * (r * std::sin(d));
    }
  }
  return y;
}

} // namespace coder

// End of code generation (power.cpp)
