//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// xdhseqr.cpp
//
// Code generation for function 'xdhseqr'
//

// Include files
#include "xdhseqr.h"
#include "rt_nonfinite.h"
#include "xdlanv2.h"
#include "xrot.h"
#include "xzlarfg.h"
#include "rt_nonfinite.h"
#include <cmath>

// Function Definitions
namespace coder {
namespace internal {
namespace reflapack {
int eml_dlahqr(double h_data[], const int h_size[2])
{
  double v[3];
  double aa;
  double ab;
  double ba;
  double bb;
  double d;
  double rt1r;
  double rt2r;
  double s;
  double tst;
  int info;
  int ldh;
  int n;
  n = h_size[0];
  ldh = h_size[0];
  info = 0;
  if (1 != n) {
    double SMLNUM;
    int b;
    int i;
    int j;
    bool exitg1;
    v[0] = 0.0;
    v[1] = 0.0;
    v[2] = 0.0;
    for (j = 0; j <= n - 4; j++) {
      b = j + h_size[0] * j;
      h_data[b + 2] = 0.0;
      h_data[b + 3] = 0.0;
    }
    if (1 <= n - 2) {
      h_data[(n + h_size[0] * (n - 3)) - 1] = 0.0;
    }
    SMLNUM = 2.2250738585072014E-308 *
             (static_cast<double>(n) / 2.2204460492503131E-16);
    i = n - 1;
    exitg1 = false;
    while ((!exitg1) && (i + 1 >= 1)) {
      int L;
      int hoffset;
      int its;
      bool exitg2;
      bool goto150;
      L = 1;
      goto150 = false;
      its = 0;
      exitg2 = false;
      while ((!exitg2) && (its <= 300)) {
        int k;
        bool exitg3;
        k = i;
        exitg3 = false;
        while ((!exitg3) && (k + 1 > L)) {
          b = k + h_size[0] * (k - 1);
          ba = std::abs(h_data[b]);
          if (ba <= SMLNUM) {
            exitg3 = true;
          } else {
            hoffset = k + h_size[0] * k;
            bb = std::abs(h_data[hoffset]);
            tst = std::abs(h_data[b - 1]) + bb;
            if (tst == 0.0) {
              if (k - 1 >= 1) {
                tst = std::abs(h_data[(k + h_size[0] * (k - 2)) - 1]);
              }
              if (k + 2 <= n) {
                tst += std::abs(h_data[(k + h_size[0] * k) + 1]);
              }
            }
            if (ba <= 2.2204460492503131E-16 * tst) {
              tst = std::abs(h_data[hoffset - 1]);
              if (ba > tst) {
                ab = ba;
                ba = tst;
              } else {
                ab = tst;
              }
              tst = std::abs(h_data[b - 1] - h_data[hoffset]);
              if (bb > tst) {
                aa = bb;
                bb = tst;
              } else {
                aa = tst;
              }
              s = aa + ab;
              tst = 2.2204460492503131E-16 * (bb * (aa / s));
              if ((SMLNUM > tst) || rtIsNaN(tst)) {
                tst = SMLNUM;
              }
              if (ba * (ab / s) <= tst) {
                exitg3 = true;
              } else {
                k--;
              }
            } else {
              k--;
            }
          }
        }
        L = k + 1;
        if (k + 1 > 1) {
          h_data[k + h_size[0] * (k - 1)] = 0.0;
        }
        if (k + 1 >= i) {
          goto150 = true;
          exitg2 = true;
        } else {
          int m;
          if (its == 10) {
            hoffset = k + h_size[0] * k;
            s = std::abs(h_data[hoffset + 1]) +
                std::abs(h_data[(k + h_size[0] * (k + 1)) + 2]);
            tst = 0.75 * s + h_data[hoffset];
            ab = -0.4375 * s;
            aa = s;
            ba = tst;
          } else if (its == 20) {
            s = std::abs(h_data[i + h_size[0] * (i - 1)]) +
                std::abs(h_data[(i + h_size[0] * (i - 2)) - 1]);
            tst = 0.75 * s + h_data[i + h_size[0] * i];
            ab = -0.4375 * s;
            aa = s;
            ba = tst;
          } else {
            hoffset = i + h_size[0] * (i - 1);
            tst = h_data[hoffset - 1];
            aa = h_data[hoffset];
            ab = h_data[(i + h_size[0] * i) - 1];
            ba = h_data[i + h_size[0] * i];
          }
          s = ((std::abs(tst) + std::abs(ab)) + std::abs(aa)) + std::abs(ba);
          if (s == 0.0) {
            rt1r = 0.0;
            aa = 0.0;
            rt2r = 0.0;
            ab = 0.0;
          } else {
            tst /= s;
            aa /= s;
            ab /= s;
            ba /= s;
            bb = (tst + ba) / 2.0;
            tst = (tst - bb) * (ba - bb) - ab * aa;
            aa = std::sqrt(std::abs(tst));
            if (tst >= 0.0) {
              rt1r = bb * s;
              rt2r = rt1r;
              aa *= s;
              ab = -aa;
            } else {
              rt1r = bb + aa;
              rt2r = bb - aa;
              if (std::abs(rt1r - ba) <= std::abs(rt2r - ba)) {
                rt1r *= s;
                rt2r = rt1r;
              } else {
                rt2r *= s;
                rt1r = rt2r;
              }
              aa = 0.0;
              ab = 0.0;
            }
          }
          m = i - 1;
          exitg3 = false;
          while ((!exitg3) && (m >= k + 1)) {
            hoffset = m + h_size[0] * (m - 1);
            tst = h_data[hoffset];
            bb = h_data[hoffset - 1];
            ba = bb - rt2r;
            s = (std::abs(ba) + std::abs(ab)) + std::abs(tst);
            tst /= s;
            hoffset = m + h_size[0] * m;
            v[0] = (tst * h_data[hoffset - 1] + (bb - rt1r) * (ba / s)) -
                   aa * (ab / s);
            v[1] = tst * (((bb + h_data[hoffset]) - rt1r) - rt2r);
            v[2] = tst * h_data[hoffset + 1];
            s = (std::abs(v[0]) + std::abs(v[1])) + std::abs(v[2]);
            v[0] /= s;
            v[1] /= s;
            v[2] /= s;
            if (m == k + 1) {
              exitg3 = true;
            } else {
              b = m + h_size[0] * (m - 2);
              if (std::abs(h_data[b - 1]) * (std::abs(v[1]) + std::abs(v[2])) <=
                  2.2204460492503131E-16 * std::abs(v[0]) *
                      ((std::abs(h_data[b - 2]) + std::abs(bb)) +
                       std::abs(h_data[hoffset]))) {
                exitg3 = true;
              } else {
                m--;
              }
            }
          }
          for (int b_k = m; b_k <= i; b_k++) {
            int nr;
            nr = (i - b_k) + 2;
            if (3 < nr) {
              nr = 3;
            }
            if (b_k > m) {
              hoffset = (b_k + ldh * (b_k - 2)) - 1;
              for (j = 0; j < nr; j++) {
                v[j] = h_data[j + hoffset];
              }
            }
            tst = v[0];
            bb = xzlarfg(nr, &tst, v);
            v[0] = tst;
            if (b_k > m) {
              b = b_k + h_size[0] * (b_k - 2);
              h_data[b - 1] = tst;
              h_data[b] = 0.0;
              if (b_k < i) {
                h_data[b + 1] = 0.0;
              }
            } else if (m > k + 1) {
              b = (b_k + h_size[0] * (b_k - 2)) - 1;
              h_data[b] *= 1.0 - bb;
            }
            d = v[1];
            aa = bb * v[1];
            if (nr == 3) {
              s = v[2];
              tst = bb * v[2];
              for (j = b_k; j <= n; j++) {
                hoffset = b_k + h_size[0] * (j - 1);
                ab = (h_data[hoffset - 1] + d * h_data[hoffset]) +
                     s * h_data[hoffset + 1];
                h_data[hoffset - 1] -= ab * bb;
                h_data[hoffset] -= ab * aa;
                h_data[hoffset + 1] -= ab * tst;
              }
              if (b_k + 3 < i + 1) {
                b = b_k + 2;
              } else {
                b = i;
              }
              for (j = 0; j <= b; j++) {
                int sum1_tmp;
                hoffset = j + h_size[0] * (b_k - 1);
                nr = j + h_size[0] * b_k;
                sum1_tmp = j + h_size[0] * (b_k + 1);
                ab = (h_data[hoffset] + d * h_data[nr]) + s * h_data[sum1_tmp];
                h_data[hoffset] -= ab * bb;
                h_data[nr] -= ab * aa;
                h_data[sum1_tmp] -= ab * tst;
              }
            } else if (nr == 2) {
              for (j = b_k; j <= n; j++) {
                hoffset = b_k + h_size[0] * (j - 1);
                tst = h_data[hoffset - 1];
                ab = tst + d * h_data[hoffset];
                h_data[hoffset - 1] = tst - ab * bb;
                h_data[hoffset] -= ab * aa;
              }
              for (j = 0; j <= i; j++) {
                hoffset = j + h_size[0] * (b_k - 1);
                nr = j + h_size[0] * b_k;
                ab = h_data[hoffset] + d * h_data[nr];
                h_data[hoffset] -= ab * bb;
                h_data[nr] -= ab * aa;
              }
            }
          }
          its++;
        }
      }
      if (!goto150) {
        info = i + 1;
        exitg1 = true;
      } else {
        if ((L != i + 1) && (L == i)) {
          b = i + h_size[0] * i;
          d = h_data[b - 1];
          hoffset = i + h_size[0] * (i - 1);
          s = h_data[hoffset];
          tst = h_data[b];
          xdlanv2(&h_data[(i + h_size[0] * (i - 1)) - 1], &d, &s, &tst, &aa,
                  &ab, &bb, &ba, &rt2r, &rt1r);
          h_data[b - 1] = d;
          h_data[hoffset] = s;
          h_data[b] = tst;
          if (n > i + 1) {
            blas::xrot((n - i) - 1, h_data, i + (i + 1) * ldh, ldh,
                       (i + (i + 1) * ldh) + 1, ldh, rt2r, rt1r);
          }
          blas::xrot(i - 1, h_data, (i - 1) * ldh + 1, i * ldh + 1, rt2r, rt1r);
        }
        i = L - 2;
      }
    }
  }
  return info;
}

} // namespace reflapack
} // namespace internal
} // namespace coder

// End of code generation (xdhseqr.cpp)
