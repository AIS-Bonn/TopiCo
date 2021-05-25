//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// xzlartg.cpp
//
// Code generation for function 'xzlartg'
//

// Include files
#include "xzlartg.h"
#include "eml_int_forloop_overflow_check.h"
#include "rt_nonfinite.h"
#include "topico_wrapper_data.h"
#include "topico_wrapper_rtwutil.h"
#include "topico_wrapper_types.h"
#include <cmath>

// Function Definitions
namespace coder {
namespace internal {
namespace reflapack {
void xzlartg(const creal_T f, const creal_T g, double *cs, creal_T *sn)
{
  double f2;
  double fs_im;
  double fs_re;
  double gs_im;
  double gs_re;
  double scale;
  double scale_tmp;
  int count;
  int rescaledir;
  bool guard1 = false;
  scale_tmp = std::abs(f.re);
  f2 = std::abs(f.im);
  if (f2 > scale_tmp) {
    scale_tmp = f2;
  }
  f2 = std::abs(g.re);
  scale = std::abs(g.im);
  if (scale > f2) {
    f2 = scale;
  }
  scale = scale_tmp;
  if (f2 > scale_tmp) {
    scale = f2;
  }
  fs_re = f.re;
  fs_im = f.im;
  gs_re = g.re;
  gs_im = g.im;
  count = 0;
  rescaledir = 0;
  guard1 = false;
  if (scale >= 7.4428285367870146E+137) {
    do {
      count++;
      fs_re *= 1.3435752215134178E-138;
      fs_im *= 1.3435752215134178E-138;
      gs_re *= 1.3435752215134178E-138;
      gs_im *= 1.3435752215134178E-138;
      scale *= 1.3435752215134178E-138;
    } while (!(scale < 7.4428285367870146E+137));
    rescaledir = 1;
    guard1 = true;
  } else if (scale <= 1.3435752215134178E-138) {
    if ((g.re == 0.0) && (g.im == 0.0)) {
      *cs = 1.0;
      sn->re = 0.0;
      sn->im = 0.0;
    } else {
      do {
        count++;
        fs_re *= 7.4428285367870146E+137;
        fs_im *= 7.4428285367870146E+137;
        gs_re *= 7.4428285367870146E+137;
        gs_im *= 7.4428285367870146E+137;
        scale *= 7.4428285367870146E+137;
      } while (!(scale > 1.3435752215134178E-138));
      rescaledir = -1;
      guard1 = true;
    }
  } else {
    guard1 = true;
  }
  if (guard1) {
    double g2;
    f2 = fs_re * fs_re + fs_im * fs_im;
    g2 = gs_re * gs_re + gs_im * gs_im;
    scale = g2;
    if (1.0 > g2) {
      scale = 1.0;
    }
    if (f2 <= scale * 2.0041683600089728E-292) {
      if ((f.re == 0.0) && (f.im == 0.0)) {
        *cs = 0.0;
        g2 = rt_hypotd_snf(gs_re, gs_im);
        sn->re = gs_re / g2;
        sn->im = -gs_im / g2;
      } else {
        double g2s;
        if (g2 < 0.0) {
          f_rtErrorWithMessageID("sqrt", n_emlrtRTEI.fName, n_emlrtRTEI.lineNo);
        }
        g2s = std::sqrt(g2);
        *cs = rt_hypotd_snf(fs_re, fs_im) / g2s;
        if (scale_tmp > 1.0) {
          g2 = rt_hypotd_snf(f.re, f.im);
          fs_re = f.re / g2;
          fs_im = f.im / g2;
        } else {
          f2 = 7.4428285367870146E+137 * f.re;
          scale = 7.4428285367870146E+137 * f.im;
          g2 = rt_hypotd_snf(f2, scale);
          fs_re = f2 / g2;
          fs_im = scale / g2;
        }
        gs_re /= g2s;
        gs_im = -gs_im / g2s;
        sn->re = fs_re * gs_re - fs_im * gs_im;
        sn->im = fs_re * gs_im + fs_im * gs_re;
      }
    } else {
      scale = g2 / f2 + 1.0;
      if (scale < 0.0) {
        f_rtErrorWithMessageID("sqrt", n_emlrtRTEI.fName, n_emlrtRTEI.lineNo);
      }
      scale = std::sqrt(scale);
      *cs = 1.0 / scale;
      g2 += f2;
      fs_re = scale * fs_re / g2;
      fs_im = scale * fs_im / g2;
      sn->re = fs_re * gs_re - fs_im * -gs_im;
      sn->im = fs_re * -gs_im + fs_im * gs_re;
      if (rescaledir > 0) {
        if ((1 <= count) && (count > 2147483646)) {
          check_forloop_overflow_error();
        }
      } else if ((rescaledir < 0) && ((1 <= count) && (count > 2147483646))) {
        check_forloop_overflow_error();
      }
    }
  }
}

void xzlartg(const creal_T f, const creal_T g, double *cs, creal_T *sn,
             creal_T *r)
{
  double f2s;
  double fs_im;
  double fs_re;
  double gs_im;
  double gs_re;
  double scale;
  double scale_tmp;
  int count;
  int rescaledir;
  bool guard1 = false;
  scale_tmp = std::abs(f.re);
  f2s = std::abs(f.im);
  if (f2s > scale_tmp) {
    scale_tmp = f2s;
  }
  f2s = std::abs(g.re);
  scale = std::abs(g.im);
  if (scale > f2s) {
    f2s = scale;
  }
  scale = scale_tmp;
  if (f2s > scale_tmp) {
    scale = f2s;
  }
  fs_re = f.re;
  fs_im = f.im;
  gs_re = g.re;
  gs_im = g.im;
  count = 0;
  rescaledir = 0;
  guard1 = false;
  if (scale >= 7.4428285367870146E+137) {
    do {
      count++;
      fs_re *= 1.3435752215134178E-138;
      fs_im *= 1.3435752215134178E-138;
      gs_re *= 1.3435752215134178E-138;
      gs_im *= 1.3435752215134178E-138;
      scale *= 1.3435752215134178E-138;
    } while (!(scale < 7.4428285367870146E+137));
    rescaledir = 1;
    guard1 = true;
  } else if (scale <= 1.3435752215134178E-138) {
    if ((g.re == 0.0) && (g.im == 0.0)) {
      *cs = 1.0;
      sn->re = 0.0;
      sn->im = 0.0;
      *r = f;
    } else {
      do {
        count++;
        fs_re *= 7.4428285367870146E+137;
        fs_im *= 7.4428285367870146E+137;
        gs_re *= 7.4428285367870146E+137;
        gs_im *= 7.4428285367870146E+137;
        scale *= 7.4428285367870146E+137;
      } while (!(scale > 1.3435752215134178E-138));
      rescaledir = -1;
      guard1 = true;
    }
  } else {
    guard1 = true;
  }
  if (guard1) {
    double f2;
    double g2;
    f2 = fs_re * fs_re + fs_im * fs_im;
    g2 = gs_re * gs_re + gs_im * gs_im;
    scale = g2;
    if (1.0 > g2) {
      scale = 1.0;
    }
    if (f2 <= scale * 2.0041683600089728E-292) {
      if ((f.re == 0.0) && (f.im == 0.0)) {
        *cs = 0.0;
        r->re = rt_hypotd_snf(g.re, g.im);
        r->im = 0.0;
        f2 = rt_hypotd_snf(gs_re, gs_im);
        sn->re = gs_re / f2;
        sn->im = -gs_im / f2;
      } else {
        if (g2 < 0.0) {
          f_rtErrorWithMessageID("sqrt", n_emlrtRTEI.fName, n_emlrtRTEI.lineNo);
        }
        g2 = std::sqrt(g2);
        *cs = rt_hypotd_snf(fs_re, fs_im) / g2;
        if (scale_tmp > 1.0) {
          f2 = rt_hypotd_snf(f.re, f.im);
          fs_re = f.re / f2;
          fs_im = f.im / f2;
        } else {
          scale = 7.4428285367870146E+137 * f.re;
          f2s = 7.4428285367870146E+137 * f.im;
          f2 = rt_hypotd_snf(scale, f2s);
          fs_re = scale / f2;
          fs_im = f2s / f2;
        }
        gs_re /= g2;
        gs_im = -gs_im / g2;
        sn->re = fs_re * gs_re - fs_im * gs_im;
        sn->im = fs_re * gs_im + fs_im * gs_re;
        r->re = *cs * f.re + (sn->re * g.re - sn->im * g.im);
        r->im = *cs * f.im + (sn->re * g.im + sn->im * g.re);
      }
    } else {
      f2s = g2 / f2 + 1.0;
      if (f2s < 0.0) {
        f_rtErrorWithMessageID("sqrt", n_emlrtRTEI.fName, n_emlrtRTEI.lineNo);
      }
      f2s = std::sqrt(f2s);
      r->re = f2s * fs_re;
      r->im = f2s * fs_im;
      *cs = 1.0 / f2s;
      f2 += g2;
      f2s = r->re / f2;
      scale = r->im / f2;
      sn->re = f2s * gs_re - scale * -gs_im;
      sn->im = f2s * -gs_im + scale * gs_re;
      if (rescaledir > 0) {
        if ((1 <= count) && (count > 2147483646)) {
          check_forloop_overflow_error();
        }
        for (rescaledir = 0; rescaledir < count; rescaledir++) {
          r->re *= 7.4428285367870146E+137;
          r->im *= 7.4428285367870146E+137;
        }
      } else if (rescaledir < 0) {
        if ((1 <= count) && (count > 2147483646)) {
          check_forloop_overflow_error();
        }
        for (rescaledir = 0; rescaledir < count; rescaledir++) {
          r->re *= 1.3435752215134178E-138;
          r->im *= 1.3435752215134178E-138;
        }
      }
    }
  }
}

} // namespace reflapack
} // namespace internal
} // namespace coder

// End of code generation (xzlartg.cpp)
