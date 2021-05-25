//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// eig.h
//
// Code generation for function 'eig'
//

#ifndef EIG_H
#define EIG_H

// Include files
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

// Function Declarations
namespace coder {
void eig(const double A_data[], const int A_size[2], creal_T V_data[],
         int *V_size);

}

#endif
// End of code generation (eig.h)
