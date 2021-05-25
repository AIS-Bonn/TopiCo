//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// rt_nonfinite.h
//
// Code generation for function 'topico_wrapper'
//

#ifndef RT_NONFINITE_H
#define RT_NONFINITE_H

// Include files
#include "rtwtypes.h"

#ifdef __cplusplus
extern "C" {
#endif

extern real_T rtInf;
extern real_T rtMinusInf;
extern real_T rtNaN;
extern real32_T rtInfF;
extern real32_T rtMinusInfF;
extern real32_T rtNaNF;

extern boolean_T rtIsInf(real_T value);
extern boolean_T rtIsInfF(real32_T value);
extern boolean_T rtIsNaN(real_T value);
extern boolean_T rtIsNaNF(real32_T value);

#ifdef __cplusplus
}
#endif
#endif
// End of code generation (rt_nonfinite.h)
