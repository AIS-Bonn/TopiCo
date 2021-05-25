//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// rtwtypes.h
//
// Code generation for function 'topico_wrapper'
//

#ifndef RTWTYPES_H
#define RTWTYPES_H

// Include files

#ifndef __cplusplus
#include <stdbool.h>
#endif

#ifdef __cplusplus
extern "C" {
#endif

/*=======================================================================*
 * Target hardware information
 *   Device type: Intel->x86-64 (Linux 64)
 *   Number of bits:     char:   8    short:   16    int:  32
 *                       long:  64
 *                       native word size:  64
 *   Byte ordering: LittleEndian
 *   Signed integer division rounds to: Zero
 *   Shift right on a signed integer as arithmetic shift: on
 *=======================================================================*/

/*=======================================================================*
 * Fixed width word size data types:                                     *
 *   int8_T, int16_T, int32_T     - signed 8, 16, or 32 bit integers     *
 *   uint8_T, uint16_T, uint32_T  - unsigned 8, 16, or 32 bit integers   *
 *   real32_T, real64_T           - 32 and 64 bit floating point numbers *
 *=======================================================================*/
typedef signed char int8_T;
typedef unsigned char uint8_T;
typedef short int16_T;
typedef unsigned short uint16_T;
typedef int int32_T;
typedef unsigned int uint32_T;
typedef long int64_T;
typedef unsigned long uint64_T;
typedef float real32_T;
typedef double real64_T;

/*===========================================================================*
 * Generic type definitions: real_T, time_T, boolean_T, int_T, uint_T,       *
 *                           ulong_T, char_T and byte_T.                     *
 *===========================================================================*/

typedef double real_T;
typedef double time_T;
typedef bool boolean_T;
typedef int int_T;
typedef unsigned int uint_T;
typedef unsigned long ulong_T;
typedef char char_T;
typedef char_T byte_T;

/*===========================================================================*
 * Complex number type definitions                                           *
 *===========================================================================*/
typedef struct {
  real32_T re;
  real32_T im;
} creal32_T;

typedef struct {
  real64_T re;
  real64_T im;
} creal64_T;

typedef struct {
  real_T re;
  real_T im;
} creal_T;

typedef struct {
  int8_T re;
  int8_T im;
} cint8_T;

typedef struct {
  uint8_T re;
  uint8_T im;
} cuint8_T;

typedef struct {
  int16_T re;
  int16_T im;
} cint16_T;

typedef struct {
  uint16_T re;
  uint16_T im;
} cuint16_T;

typedef struct {
  int32_T re;
  int32_T im;
} cint32_T;

typedef struct {
  uint32_T re;
  uint32_T im;
} cuint32_T;

typedef struct {
  int64_T re;
  int64_T im;
} cint64_T;

typedef struct {
  uint64_T re;
  uint64_T im;
} cuint64_T;

/*=======================================================================*
 * Min and Max:                                                          *
 *   int8_T, int16_T, int32_T     - signed 8, 16, or 32 bit integers     *
 *   uint8_T, uint16_T, uint32_T  - unsigned 8, 16, or 32 bit integers   *
 *=======================================================================*/

static const int8_T MAX_int8_T = 127;
static const int8_T MIN_int8_T = -128;
static const uint8_T MAX_uint8_T = 255U;
static const uint8_T MIN_uint8_T = 0;
static const int16_T MAX_int16_T = 32767;
static const int16_T MIN_int16_T = -32768;
static const uint16_T MAX_uint16_T = 65535U;
static const uint16_T MIN_uint16_T = 0;
static const int32_T MAX_int32_T = 2147483647;
static const int32_T MIN_int32_T = -2147483647 - 1;
static const uint32_T MAX_uint32_T = 0xFFFFFFFFU;
static const uint32_T MIN_uint32_T = 0;
static const int64_T MAX_int64_T = 9223372036854775807L;
static const int64_T MIN_int64_T = -9223372036854775807L - 1L;
static const uint64_T MAX_uint64_T = 0xFFFFFFFFFFFFFFFFUL;
static const uint64_T MIN_uint64_T = 0UL;

/* Logical type definitions */
#if (!defined(__cplusplus)) && (!defined(__true_false_are_keywords)) &&        \
    (!defined(__bool_true_false_are_defined))
#ifndef false
#define false 0U
#endif
#ifndef true
#define true 1U
#endif
#endif

#ifdef __cplusplus
}
#endif
#endif
// End of code generation (rtwtypes.h)
