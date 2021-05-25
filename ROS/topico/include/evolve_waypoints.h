//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// evolve_waypoints.h
//
// Code generation for function 'evolve_waypoints'
//

#ifndef EVOLVE_WAYPOINTS_H
#define EVOLVE_WAYPOINTS_H

// Include files
#include "rtwtypes.h"
#include "coder_array.h"
#include <cstddef>
#include <cstdlib>

// Function Declarations
void evolve_waypoints(const coder::array<double, 2U> &Waypoints,
                      const coder::array<double, 1U> &T,
                      coder::array<double, 2U> &Waypoints_evolved);

#endif
// End of code generation (evolve_waypoints.h)
