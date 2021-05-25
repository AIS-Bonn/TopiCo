//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// xdlanv2.cpp
//
// Code generation for function 'xdlanv2'
//

// Include files
#include "xdlanv2.h"
#include "rt_nonfinite.h"
#include "topico_wrapper_rtwutil.h"
#include "rt_nonfinite.h"
#include <cmath>

// Function Definitions
namespace coder {
namespace internal {
namespace reflapack {
void xdlanv2(double *a, double *b, double *c, double *d, double *rt1r,
             double *rt1i, double *rt2r, double *rt2i, double *cs, double *sn)
{
  if (*c == 0.0) {
    *cs = 1.0;
    *sn = 0.0;
  } else if (*b == 0.0) {
    double z;
    *cs = 0.0;
    *sn = 1.0;
    z = *d;
    *d = *a;
    *a = z;
    *b = -*c;
    *c = 0.0;
  } else {
    double tau;
    tau = *a - *d;
    if ((tau == 0.0) && ((*b < 0.0) != (*c < 0.0))) {
      *cs = 1.0;
      *sn = 0.0;
    } else {
      double bcmax;
      double bcmis;
      double p;
      double scale;
      double z;
      int b_b;
      int b_c;
      p = 0.5 * tau;
      scale = std::abs(*b);
      bcmis = std::abs(*c);
      if ((scale > bcmis) || rtIsNaN(bcmis)) {
        bcmax = scale;
      } else {
        bcmax = bcmis;
      }
      if ((scale < bcmis) || rtIsNaN(bcmis)) {
        bcmis = scale;
      }
      if (!(*b < 0.0)) {
        b_b = 1;
      } else {
        b_b = -1;
      }
      if (!(*c < 0.0)) {
        b_c = 1;
      } else {
        b_c = -1;
      }
      bcmis = bcmis * static_cast<double>(b_b) * static_cast<double>(b_c);
      scale = std::abs(p);
      if ((!(scale > bcmax)) && (!rtIsNaN(bcmax))) {
        scale = bcmax;
      }
      z = p / scale * p + bcmax / scale * bcmis;
      if (z >= 8.8817841970012523E-16) {
        *a = std::sqrt(scale) * std::sqrt(z);
        if (p < 0.0) {
          *a = -*a;
        }
        z = p + *a;
        *a = *d + z;
        *d -= bcmax / z * bcmis;
        tau = rt_hypotd_snf(*c, z);
        *cs = z / tau;
        *sn = *c / tau;
        *b -= *c;
        *c = 0.0;
      } else {
        scale = *b + *c;
        tau = rt_hypotd_snf(scale, tau);
        *cs = std::sqrt(0.5 * (std::abs(scale) / tau + 1.0));
        if (!(scale < 0.0)) {
          b_b = 1;
        } else {
          b_b = -1;
        }
        *sn = -(p / (tau * *cs)) * static_cast<double>(b_b);
        bcmax = *a * *cs + *b * *sn;
        bcmis = -*a * *sn + *b * *cs;
        z = *c * *cs + *d * *sn;
        scale = -*c * *sn + *d * *cs;
        *b = bcmis * *cs + scale * *sn;
        *c = -bcmax * *sn + z * *cs;
        z = 0.5 * ((bcmax * *cs + z * *sn) + (-bcmis * *sn + scale * *cs));
        *a = z;
        *d = z;
        if (*c != 0.0) {
          if (*b != 0.0) {
            if ((*b < 0.0) == (*c < 0.0)) {
              scale = std::sqrt(std::abs(*b));
              bcmis = std::sqrt(std::abs(*c));
              *a = scale * bcmis;
              if (!(*c < 0.0)) {
                p = *a;
              } else {
                p = -*a;
              }
              tau = 1.0 / std::sqrt(std::abs(*b + *c));
              *a = z + p;
              *d = z - p;
              *b -= *c;
              *c = 0.0;
              bcmax = scale * tau;
              scale = bcmis * tau;
              z = *cs * bcmax - *sn * scale;
              *sn = *cs * scale + *sn * bcmax;
              *cs = z;
            }
          } else {
            *b = -*c;
            *c = 0.0;
            z = *cs;
            *cs = -*sn;
            *sn = z;
          }
        }
      }
    }
  }
  *rt1r = *a;
  *rt2r = *d;
  if (*c == 0.0) {
    *rt1i = 0.0;
    *rt2i = 0.0;
  } else {
    *rt1i = std::sqrt(std::abs(*b)) * std::sqrt(std::abs(*c));
    *rt2i = -*rt1i;
  }
}

} // namespace reflapack
} // namespace internal
} // namespace coder

// End of code generation (xdlanv2.cpp)
