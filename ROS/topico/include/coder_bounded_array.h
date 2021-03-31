/* Copyright 2020 The Mathworks, Inc. */
/* Copied from fullfile(matlabroot,'extern','include','coder','coder_array','coder_bounded_array.h') */

#ifndef _mw_coder_bounded_array_h
#define _mw_coder_bounded_array_h

#ifdef MATLAB_MEX_FILE
#include "tmwtypes.h"
#else
#include "rtwtypes.h"
#endif

namespace coder {

#if __cplusplus >= 201103L
using SizeType = int32_T;
#else
typedef int32_T SizeType;
#endif

// Bounded array
template <typename T, SizeType UpperBoundSize, SizeType NumDims>
struct bounded_array {
    T data[UpperBoundSize];
    SizeType size[NumDims];
};

template <typename T, SizeType NumDims>
struct empty_bounded_array {
    SizeType size[NumDims];
};
} // namespace coder

#endif
