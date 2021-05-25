//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// select_cases_O.cpp
//
// Code generation for function 'select_cases_O'
//

// Include files
#include "select_cases_O.h"
#include "rt_nonfinite.h"
#include "topico_wrapper_rtwutil.h"
#include "topico_wrapper_types.h"
#include "coder_bounded_array.h"

// Type Definitions
struct cell_wrap_18 {
  coder::bounded_array<int, 33U, 2U> f1;
};

// Variable Definitions
static bool caselut_not_empty;

// Function Definitions
void select_cases_O(unsigned char n, unsigned char cond, int cases_data[],
                    int cases_size[2])
{
  static cell_wrap_18 caselut[224];
  static rtBoundsCheckInfo nb_emlrtBCI = {
      0,                                                     // iFirst
      6,                                                     // iLast
      271,                                                   // lineNo
      21,                                                    // colNo
      "caselut",                                             // aName
      "select_cases_O",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/select_cases_O.m", // pName
      0                                                      // checkKind
  };
  static rtBoundsCheckInfo ob_emlrtBCI = {
      0,                                                     // iFirst
      31,                                                    // iLast
      271,                                                   // lineNo
      23,                                                    // colNo
      "caselut",                                             // aName
      "select_cases_O",                                      // fName
      "/home/lmbeul/Desktop/TopiCo/MATLAB/select_cases_O.m", // pName
      0                                                      // checkKind
  };
  static const short iv30[25] = {
      0,   202, 204, 206, 208,  210,  212,  214,  216,  218,  220,  222, 224,
      226, 228, 230, 232, 1201, 1203, 1205, 1207, 1209, 1211, 1213, 1215};
  static const short iv38[25] = {
      0,   201, 203, 205, 207,  209,  211,  213,  215,  217,  219,  221, 223,
      225, 227, 229, 231, 1202, 1204, 1206, 1208, 1210, 1212, 1214, 1216};
  static const short iv51[21] = {0,   301, 302, 303, 304, 305, 306,
                                 307, 308, 309, 310, 311, 312, 313,
                                 314, 315, 316, 317, 318, 319, 320};
  static const short iv26[17] = {0,    202,  212,  214,  216,  218,
                                 228,  230,  232,  1201, 1203, 1205,
                                 1207, 1209, 1211, 1213, 1215};
  static const short iv27[17] = {0,    202,  204,  210,  216,  218,
                                 220,  226,  232,  1201, 1203, 1205,
                                 1207, 1209, 1211, 1213, 1215};
  static const short iv34[17] = {0,    201,  211,  213,  215,  217,
                                 227,  229,  231,  1202, 1204, 1206,
                                 1208, 1210, 1212, 1214, 1216};
  static const short iv36[17] = {0,    201,  203,  209,  215,  217,
                                 219,  225,  231,  1202, 1204, 1206,
                                 1208, 1210, 1212, 1214, 1216};
  static const short iv17[13] = {0,   102, 104,  106,  108,  110, 112,
                                 114, 116, 1101, 1103, 1105, 1107};
  static const short iv22[13] = {0,   101, 103,  105,  107,  109, 111,
                                 113, 115, 1102, 1104, 1106, 1108};
  static const short iv23[13] = {0,    202,  216,  218,  232,  1201, 1203,
                                 1205, 1207, 1209, 1211, 1213, 1215};
  static const short iv32[13] = {0,    201,  215,  217,  231,  1202, 1204,
                                 1206, 1208, 1210, 1212, 1214, 1216};
  static const short iv42[13] = {0,   301, 302, 306, 304, 308, 310,
                                 311, 312, 314, 316, 318, 320};
  static const short iv46[13] = {0,   301, 302, 303, 305, 307, 309,
                                 311, 312, 313, 315, 317, 319};
  static const short iv91[13] = {0,   701, 702, 703, 704, 705, 706,
                                 707, 708, 709, 710, 711, 712};
  static const short iv48[12] = {0,   302, 304, 310, 311, 312,
                                 313, 314, 315, 317, 319, 320};
  static const short iv49[12] = {0,   301, 303, 309, 311, 312,
                                 313, 314, 316, 318, 319, 320};
  static const short iv28[11] = {0,   218, 220, 222,  224, 226,
                                 228, 230, 232, 1209, 1215};
  static const short iv35[11] = {0,   217, 219, 221,  223, 225,
                                 227, 229, 231, 1210, 1216};
  static const short iv58[11] = {0,   302, 304, 308,  310, 312,
                                 314, 318, 320, 1301, 1303};
  static const short iv65[11] = {0,   301, 303, 307,  309, 311,
                                 313, 317, 319, 1302, 1304};
  static const short iv88[10] = {0,   701, 702, 704, 706,
                                 708, 709, 710, 711, 712};
  static const short iv89[10] = {0,   701, 702, 703, 705,
                                 707, 709, 710, 711, 712};
  static const short iv14[9] = {0, 106, 108, 114, 116, 1101, 1103, 1105, 1107};
  static const short iv15[9] = {0, 104, 108, 112, 116, 1101, 1103, 1105, 1107};
  static const short iv19[9] = {0, 105, 107, 113, 115, 1102, 1104, 1106, 1108};
  static const short iv21[9] = {0, 103, 107, 111, 115, 1102, 1104, 1106, 1108};
  static const short iv54[9] = {0, 302, 308, 310, 312, 318, 320, 1301, 1303};
  static const short iv55[9] = {0, 302, 304, 310, 312, 314, 320, 1301, 1303};
  static const short iv61[9] = {0, 301, 307, 309, 311, 317, 319, 1302, 1304};
  static const short iv63[9] = {0, 301, 303, 309, 311, 313, 319, 1302, 1304};
  static const short iv76[9] = {0, 601, 602, 603, 604, 605, 606, 607, 608};
  static const short iv82[9] = {0, 701, 702, 703, 704, 706, 708, 710, 712};
  static const short iv86[9] = {0, 701, 702, 703, 704, 705, 707, 709, 711};
  static const short iv93[9] = {0, 702, 704, 706, 708, 710, 712, 1701, 1703};
  static const short iv97[9] = {0, 701, 703, 705, 707, 709, 711, 1702, 1704};
  static const short iv39[8] = {0, 302, 304, 310, 311, 312, 314, 320};
  static const short iv40[8] = {0, 301, 311, 312, 314, 316, 318, 320};
  static const short iv43[8] = {0, 302, 311, 312, 313, 315, 317, 319};
  static const short iv44[8] = {0, 301, 303, 309, 311, 312, 313, 319};
  static const short iv78[8] = {0, 701, 702, 704, 706, 708, 710, 712};
  static const short iv84[8] = {0, 701, 702, 703, 705, 707, 709, 711};
  static const short iv13[7] = {0, 108, 116, 1101, 1103, 1105, 1107};
  static const short iv18[7] = {0, 107, 115, 1102, 1104, 1106, 1108};
  static const short iv24[7] = {0, 218, 228, 230, 232, 1209, 1215};
  static const short iv29[7] = {0, 218, 220, 226, 232, 1209, 1215};
  static const short iv31[7] = {0, 217, 227, 229, 231, 1210, 1216};
  static const short iv37[7] = {0, 217, 219, 225, 231, 1210, 1216};
  static const short iv50[7] = {0, 311, 312, 313, 314, 319, 320};
  static const short iv52[7] = {0, 302, 310, 312, 320, 1301, 1303};
  static const short iv60[7] = {0, 301, 309, 311, 319, 1302, 1304};
  static const short iv72[7] = {0, 501, 502, 503, 504, 505, 506};
  static const short iv73[7] = {0, 601, 602, 604, 606, 607, 608};
  static const short iv74[7] = {0, 601, 602, 603, 605, 607, 608};
  static const short iv90[7] = {0, 701, 702, 709, 710, 711, 712};
  static const short iv92[7] = {0, 702, 704, 706, 710, 1701, 1703};
  static const short iv95[7] = {0, 701, 703, 705, 709, 1702, 1704};
  static const short iv16[6] = {0, 110, 112, 114, 116, 1107};
  static const short iv20[6] = {0, 109, 111, 113, 115, 1108};
  static const short iv56[6] = {0, 312, 314, 318, 320, 1303};
  static const short iv62[6] = {0, 311, 313, 317, 319, 1304};
  static const short iv69[6] = {0, 501, 502, 504, 505, 506};
  static const short iv70[6] = {0, 501, 502, 503, 505, 506};
  static const short iv80[6] = {0, 701, 702, 703, 710, 712};
  static const short iv83[6] = {0, 701, 702, 704, 709, 711};
  static const short iv25[5] = {0, 218, 232, 1209, 1215};
  static const short iv33[5] = {0, 217, 231, 1210, 1216};
  static const short iv41[5] = {0, 311, 312, 314, 320};
  static const short iv45[5] = {0, 311, 312, 313, 319};
  static const short iv47[5] = {0, 301, 302, 311, 312};
  static const short iv53[5] = {0, 312, 318, 320, 1303};
  static const short iv57[5] = {0, 312, 314, 320, 1303};
  static const short iv59[5] = {0, 311, 317, 319, 1304};
  static const short iv64[5] = {0, 311, 313, 319, 1304};
  static const short iv66[5] = {0, 401, 402, 403, 404};
  static const short iv67[5] = {0, 501, 502, 503, 505};
  static const short iv68[5] = {0, 501, 502, 504, 506};
  static const short iv71[5] = {0, 501, 502, 505, 506};
  static const short iv75[5] = {0, 601, 602, 607, 608};
  static const short iv77[5] = {0, 602, 604, 606, 608};
  static const short iv79[5] = {0, 601, 603, 605, 607};
  static const short iv81[5] = {0, 701, 702, 710, 712};
  static const short iv85[5] = {0, 701, 702, 709, 711};
  static const short iv87[5] = {0, 701, 702, 703, 704};
  static const short iv94[5] = {0, 702, 710, 712, 1703};
  static const short iv96[5] = {0, 701, 709, 711, 1704};
  static const unsigned char uv14[33] = {
      0U,   201U, 202U, 203U, 204U, 205U, 206U, 207U, 208U, 209U, 210U,
      211U, 212U, 213U, 214U, 215U, 216U, 217U, 218U, 219U, 220U, 221U,
      222U, 223U, 224U, 225U, 226U, 227U, 228U, 229U, 230U, 231U, 232U};
  static const signed char iv12[17] = {0,   101, 102, 103, 104, 105,
                                       106, 107, 108, 109, 110, 111,
                                       112, 113, 114, 115, 116};
  static const unsigned char uv11[17] = {0U,   202U, 204U, 210U, 216U, 217U,
                                         218U, 219U, 220U, 221U, 223U, 225U,
                                         226U, 227U, 229U, 231U, 232U};
  static const unsigned char uv12[17] = {0U,   201U, 203U, 209U, 215U, 217U,
                                         218U, 219U, 220U, 222U, 224U, 225U,
                                         226U, 228U, 230U, 231U, 232U};
  static const unsigned char uv3[17] = {0U,   201U, 202U, 204U, 206U, 211U,
                                        212U, 213U, 215U, 217U, 218U, 220U,
                                        222U, 227U, 228U, 229U, 231U};
  static const unsigned char uv7[17] = {0U,   201U, 202U, 203U, 205U, 211U,
                                        212U, 214U, 216U, 217U, 218U, 219U,
                                        221U, 227U, 228U, 230U, 232U};
  static const signed char iv2[13] = {0,   102, 104, 105, 106, 107, 108,
                                      110, 112, 113, 114, 115, 116};
  static const signed char iv5[13] = {0,   101, 103, 105, 106, 107, 108,
                                      109, 111, 113, 114, 115, 116};
  static const signed char iv10[9] = {0,   103, 107, 110, 111,
                                      112, 114, 115, 116};
  static const signed char iv8[9] = {0, 105, 106, 107, 108, 113, 114, 115, 116};
  static const signed char iv9[9] = {0, 104, 108, 109, 111, 112, 113, 115, 116};
  static const unsigned char uv[9] = {0U,   202U, 204U, 217U, 218U,
                                      220U, 227U, 229U, 231U};
  static const unsigned char uv1[9] = {0U,   201U, 215U, 217U, 218U,
                                       220U, 222U, 228U, 231U};
  static const unsigned char uv10[9] = {0U,   201U, 202U, 211U, 212U,
                                        217U, 218U, 227U, 228U};
  static const unsigned char uv13[9] = {0U,   217U, 218U, 219U, 220U,
                                        225U, 226U, 231U, 232U};
  static const unsigned char uv4[9] = {0U,   202U, 216U, 217U, 218U,
                                       219U, 221U, 227U, 232U};
  static const unsigned char uv5[9] = {0U,   201U, 203U, 217U, 218U,
                                       219U, 228U, 230U, 232U};
  static const signed char iv[7] = {0, 104, 108, 112, 113, 115, 116};
  static const signed char iv1[7] = {0, 107, 110, 112, 114, 115, 116};
  static const signed char iv3[7] = {0, 108, 109, 111, 113, 115, 116};
  static const signed char iv4[7] = {0, 103, 107, 111, 114, 115, 116};
  static const signed char iv11[5] = {0, 111, 112, 115, 116};
  static const signed char iv6[5] = {0, 108, 113, 115, 116};
  static const signed char iv7[5] = {0, 107, 114, 115, 116};
  static const unsigned char uv2[5] = {0U, 217U, 218U, 220U, 231U};
  static const unsigned char uv6[5] = {0U, 217U, 218U, 219U, 232U};
  static const unsigned char uv8[5] = {0U, 202U, 217U, 218U, 227U};
  static const unsigned char uv9[5] = {0U, 201U, 217U, 218U, 228U};
  int cases_size_tmp;
  int i;
  int loop_ub;
  //  ---------------------------------------------------------------------
  //  Package:    TopiCo (https://github.com/AIS-Bonn/TopiCo)
  //  Version:    2021-03-18 12:09:55
  //  Maintainer: Marius Beul (mbeul@ais.uni-bonn.de)
  //  License:    BSD
  //  ---------------------------------------------------------------------
  //  Software License Agreement (BSD License)
  //  Copyright (c) 2021, Computer Science Institute VI, University of Bonn
  //  All rights reserved.
  //  Redistribution and use in source and binary forms, with or without
  //  modification, are permitted provided that the following conditions
  //  are met:
  //
  //  * Redistributions of source code must retain the above copyright
  //    notice, this list of conditions and the following disclaimer.
  //  * Redistributions in binary form must reproduce the above
  //    copyright notice, this list of conditions and the following
  //    disclaimer in the documentation and/or other materials provided
  //    with the distribution.
  //  * Neither the name of University of Bonn, Computer Science Institute
  //    VI nor the names of its contributors may be used to endorse or
  //    promote products derived from this software without specific
  //    prior written permission.
  //
  //  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
  //  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
  //  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
  //  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
  //  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  //  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
  //  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
  //  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  //  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
  //  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
  //  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
  //  POSSIBILITY OF SUCH DAMAGE.
  //  --------------------------------------------------------------------
  if (!caselut_not_empty) {
    short i1;
    for (i = 0; i < 224; i++) {
      caselut[i].f1.size[0] = 1;
      caselut[i].f1.size[1] = 0;
    }
    caselut_not_empty = true;
    caselut[0].f1.size[0] = 1;
    caselut[0].f1.size[1] = 7;
    caselut[7].f1.size[0] = 1;
    caselut[7].f1.size[1] = 7;
    for (i = 0; i < 7; i++) {
      caselut[0].f1.data[i] = iv[i];
      caselut[7].f1.data[i] = iv1[i];
    }
    caselut[14].f1.size[0] = 1;
    caselut[14].f1.size[1] = 4;
    caselut[14].f1.data[0] = 0;
    caselut[14].f1.data[1] = 112;
    caselut[14].f1.data[2] = 115;
    caselut[14].f1.data[3] = 116;
    caselut[21].f1.size[0] = 1;
    caselut[21].f1.size[1] = 13;
    for (i = 0; i < 13; i++) {
      caselut[21].f1.data[i] = iv2[i];
    }
    caselut[28].f1.size[0] = 1;
    caselut[28].f1.size[1] = 7;
    caselut[35].f1.size[0] = 1;
    caselut[35].f1.size[1] = 7;
    for (i = 0; i < 7; i++) {
      caselut[28].f1.data[i] = iv3[i];
      caselut[35].f1.data[i] = iv4[i];
    }
    caselut[42].f1.size[0] = 1;
    caselut[42].f1.size[1] = 4;
    caselut[42].f1.data[0] = 0;
    caselut[42].f1.data[1] = 111;
    caselut[42].f1.data[2] = 115;
    caselut[42].f1.data[3] = 116;
    caselut[49].f1.size[0] = 1;
    caselut[49].f1.size[1] = 13;
    for (i = 0; i < 13; i++) {
      caselut[49].f1.data[i] = iv5[i];
    }
    caselut[56].f1.size[0] = 1;
    caselut[56].f1.size[1] = 5;
    caselut[63].f1.size[0] = 1;
    caselut[63].f1.size[1] = 5;
    for (i = 0; i < 5; i++) {
      caselut[56].f1.data[i] = iv6[i];
      caselut[63].f1.data[i] = iv7[i];
    }
    caselut[70].f1.size[0] = 1;
    caselut[70].f1.size[1] = 3;
    caselut[70].f1.data[0] = 0;
    caselut[70].f1.data[1] = 115;
    caselut[70].f1.data[2] = 116;
    caselut[77].f1.size[0] = 1;
    caselut[77].f1.size[1] = 9;
    caselut[84].f1.size[0] = 1;
    caselut[84].f1.size[1] = 9;
    caselut[91].f1.size[0] = 1;
    caselut[91].f1.size[1] = 9;
    for (i = 0; i < 9; i++) {
      caselut[77].f1.data[i] = iv8[i];
      caselut[84].f1.data[i] = iv9[i];
      caselut[91].f1.data[i] = iv10[i];
    }
    caselut[98].f1.size[0] = 1;
    caselut[98].f1.size[1] = 5;
    for (i = 0; i < 5; i++) {
      caselut[98].f1.data[i] = iv11[i];
    }
    caselut[105].f1.size[0] = 1;
    caselut[105].f1.size[1] = 17;
    for (i = 0; i < 17; i++) {
      caselut[105].f1.data[i] = iv12[i];
    }
    caselut[112].f1.size[0] = 1;
    caselut[112].f1.size[1] = 7;
    for (i = 0; i < 7; i++) {
      caselut[112].f1.data[i] = iv13[i];
    }
    caselut[119].f1.size[0] = 1;
    caselut[119].f1.size[1] = 4;
    caselut[119].f1.data[0] = 0;
    caselut[119].f1.data[1] = 114;
    caselut[119].f1.data[2] = 116;
    caselut[119].f1.data[3] = 1107;
    caselut[126].f1.size[0] = 1;
    caselut[126].f1.size[1] = 3;
    caselut[126].f1.data[0] = 0;
    caselut[126].f1.data[1] = 116;
    caselut[126].f1.data[2] = 1107;
    caselut[133].f1.size[0] = 1;
    caselut[133].f1.size[1] = 9;
    caselut[140].f1.size[0] = 1;
    caselut[140].f1.size[1] = 9;
    for (i = 0; i < 9; i++) {
      caselut[133].f1.data[i] = iv14[i];
      caselut[140].f1.data[i] = iv15[i];
    }
    caselut[147].f1.size[0] = 1;
    caselut[147].f1.size[1] = 6;
    for (i = 0; i < 6; i++) {
      caselut[147].f1.data[i] = iv16[i];
    }
    caselut[154].f1.size[0] = 1;
    caselut[154].f1.size[1] = 4;
    caselut[154].f1.data[0] = 0;
    caselut[154].f1.data[1] = 112;
    caselut[154].f1.data[2] = 116;
    caselut[154].f1.data[3] = 1107;
    caselut[161].f1.size[0] = 1;
    caselut[161].f1.size[1] = 13;
    for (i = 0; i < 13; i++) {
      caselut[161].f1.data[i] = iv17[i];
    }
    caselut[168].f1.size[0] = 1;
    caselut[168].f1.size[1] = 4;
    caselut[168].f1.data[0] = 0;
    caselut[168].f1.data[1] = 113;
    caselut[168].f1.data[2] = 115;
    caselut[168].f1.data[3] = 1108;
    caselut[175].f1.size[0] = 1;
    caselut[175].f1.size[1] = 7;
    for (i = 0; i < 7; i++) {
      caselut[175].f1.data[i] = iv18[i];
    }
    caselut[182].f1.size[0] = 1;
    caselut[182].f1.size[1] = 3;
    caselut[182].f1.data[0] = 0;
    caselut[182].f1.data[1] = 115;
    caselut[182].f1.data[2] = 1108;
    caselut[189].f1.size[0] = 1;
    caselut[189].f1.size[1] = 9;
    for (i = 0; i < 9; i++) {
      caselut[189].f1.data[i] = iv19[i];
    }
    caselut[196].f1.size[0] = 1;
    caselut[196].f1.size[1] = 6;
    for (i = 0; i < 6; i++) {
      caselut[196].f1.data[i] = iv20[i];
    }
    caselut[203].f1.size[0] = 1;
    caselut[203].f1.size[1] = 9;
    for (i = 0; i < 9; i++) {
      caselut[203].f1.data[i] = iv21[i];
    }
    caselut[210].f1.size[0] = 1;
    caselut[210].f1.size[1] = 4;
    caselut[210].f1.data[0] = 0;
    caselut[210].f1.data[1] = 111;
    caselut[210].f1.data[2] = 115;
    caselut[210].f1.data[3] = 1108;
    caselut[217].f1.size[0] = 1;
    caselut[217].f1.size[1] = 13;
    for (i = 0; i < 13; i++) {
      caselut[217].f1.data[i] = iv22[i];
    }
    caselut[1].f1.size[0] = 1;
    caselut[1].f1.size[1] = 9;
    caselut[8].f1.size[0] = 1;
    caselut[8].f1.size[1] = 9;
    for (i = 0; i < 9; i++) {
      caselut[1].f1.data[i] = uv[i];
      caselut[8].f1.data[i] = uv1[i];
    }
    caselut[15].f1.size[0] = 1;
    caselut[15].f1.size[1] = 5;
    for (i = 0; i < 5; i++) {
      caselut[15].f1.data[i] = uv2[i];
    }
    caselut[22].f1.size[0] = 1;
    caselut[22].f1.size[1] = 17;
    for (i = 0; i < 17; i++) {
      caselut[22].f1.data[i] = uv3[i];
    }
    caselut[29].f1.size[0] = 1;
    caselut[29].f1.size[1] = 9;
    caselut[36].f1.size[0] = 1;
    caselut[36].f1.size[1] = 9;
    for (i = 0; i < 9; i++) {
      caselut[29].f1.data[i] = uv4[i];
      caselut[36].f1.data[i] = uv5[i];
    }
    caselut[43].f1.size[0] = 1;
    caselut[43].f1.size[1] = 5;
    for (i = 0; i < 5; i++) {
      caselut[43].f1.data[i] = uv6[i];
    }
    caselut[50].f1.size[0] = 1;
    caselut[50].f1.size[1] = 17;
    for (i = 0; i < 17; i++) {
      caselut[50].f1.data[i] = uv7[i];
    }
    caselut[57].f1.size[0] = 1;
    caselut[57].f1.size[1] = 5;
    caselut[64].f1.size[0] = 1;
    caselut[64].f1.size[1] = 5;
    for (i = 0; i < 5; i++) {
      caselut[57].f1.data[i] = uv8[i];
      caselut[64].f1.data[i] = uv9[i];
    }
    caselut[71].f1.size[0] = 1;
    caselut[71].f1.size[1] = 3;
    caselut[71].f1.data[0] = 0;
    caselut[71].f1.data[1] = 217;
    caselut[71].f1.data[2] = 218;
    caselut[78].f1.size[0] = 1;
    caselut[78].f1.size[1] = 9;
    for (i = 0; i < 9; i++) {
      caselut[78].f1.data[i] = uv10[i];
    }
    caselut[85].f1.size[0] = 1;
    caselut[85].f1.size[1] = 17;
    caselut[92].f1.size[0] = 1;
    caselut[92].f1.size[1] = 17;
    for (i = 0; i < 17; i++) {
      caselut[85].f1.data[i] = uv11[i];
      caselut[92].f1.data[i] = uv12[i];
    }
    caselut[99].f1.size[0] = 1;
    caselut[99].f1.size[1] = 9;
    for (i = 0; i < 9; i++) {
      caselut[99].f1.data[i] = uv13[i];
    }
    caselut[106].f1.size[0] = 1;
    caselut[106].f1.size[1] = 33;
    for (i = 0; i < 33; i++) {
      caselut[106].f1.data[i] = uv14[i];
    }
    caselut[113].f1.size[0] = 1;
    caselut[113].f1.size[1] = 13;
    for (i = 0; i < 13; i++) {
      caselut[113].f1.data[i] = iv23[i];
    }
    caselut[120].f1.size[0] = 1;
    caselut[120].f1.size[1] = 7;
    for (i = 0; i < 7; i++) {
      caselut[120].f1.data[i] = iv24[i];
    }
    caselut[127].f1.size[0] = 1;
    caselut[127].f1.size[1] = 5;
    for (i = 0; i < 5; i++) {
      caselut[127].f1.data[i] = iv25[i];
    }
    caselut[134].f1.size[0] = 1;
    caselut[134].f1.size[1] = 17;
    caselut[141].f1.size[0] = 1;
    caselut[141].f1.size[1] = 17;
    for (i = 0; i < 17; i++) {
      caselut[134].f1.data[i] = iv26[i];
      caselut[141].f1.data[i] = iv27[i];
    }
    caselut[148].f1.size[0] = 1;
    caselut[148].f1.size[1] = 11;
    for (i = 0; i < 11; i++) {
      caselut[148].f1.data[i] = iv28[i];
    }
    caselut[155].f1.size[0] = 1;
    caselut[155].f1.size[1] = 7;
    for (i = 0; i < 7; i++) {
      caselut[155].f1.data[i] = iv29[i];
    }
    caselut[162].f1.size[0] = 1;
    caselut[162].f1.size[1] = 25;
    for (i = 0; i < 25; i++) {
      caselut[162].f1.data[i] = iv30[i];
    }
    caselut[169].f1.size[0] = 1;
    caselut[169].f1.size[1] = 7;
    for (i = 0; i < 7; i++) {
      caselut[169].f1.data[i] = iv31[i];
    }
    caselut[176].f1.size[0] = 1;
    caselut[176].f1.size[1] = 13;
    for (i = 0; i < 13; i++) {
      caselut[176].f1.data[i] = iv32[i];
    }
    caselut[183].f1.size[0] = 1;
    caselut[183].f1.size[1] = 5;
    for (i = 0; i < 5; i++) {
      caselut[183].f1.data[i] = iv33[i];
    }
    caselut[190].f1.size[0] = 1;
    caselut[190].f1.size[1] = 17;
    for (i = 0; i < 17; i++) {
      caselut[190].f1.data[i] = iv34[i];
    }
    caselut[197].f1.size[0] = 1;
    caselut[197].f1.size[1] = 11;
    for (i = 0; i < 11; i++) {
      caselut[197].f1.data[i] = iv35[i];
    }
    caselut[204].f1.size[0] = 1;
    caselut[204].f1.size[1] = 17;
    for (i = 0; i < 17; i++) {
      caselut[204].f1.data[i] = iv36[i];
    }
    caselut[211].f1.size[0] = 1;
    caselut[211].f1.size[1] = 7;
    for (i = 0; i < 7; i++) {
      caselut[211].f1.data[i] = iv37[i];
    }
    caselut[218].f1.size[0] = 1;
    caselut[218].f1.size[1] = 25;
    for (i = 0; i < 25; i++) {
      caselut[218].f1.data[i] = iv38[i];
    }
    caselut[2].f1.size[0] = 1;
    caselut[2].f1.size[1] = 8;
    caselut[9].f1.size[0] = 1;
    caselut[9].f1.size[1] = 8;
    for (i = 0; i < 8; i++) {
      caselut[2].f1.data[i] = iv39[i];
      caselut[9].f1.data[i] = iv40[i];
    }
    caselut[16].f1.size[0] = 1;
    caselut[16].f1.size[1] = 5;
    for (i = 0; i < 5; i++) {
      caselut[16].f1.data[i] = iv41[i];
    }
    caselut[23].f1.size[0] = 1;
    caselut[23].f1.size[1] = 13;
    for (i = 0; i < 13; i++) {
      caselut[23].f1.data[i] = iv42[i];
    }
    caselut[30].f1.size[0] = 1;
    caselut[30].f1.size[1] = 8;
    caselut[37].f1.size[0] = 1;
    caselut[37].f1.size[1] = 8;
    for (i = 0; i < 8; i++) {
      caselut[30].f1.data[i] = iv43[i];
      caselut[37].f1.data[i] = iv44[i];
    }
    caselut[44].f1.size[0] = 1;
    caselut[44].f1.size[1] = 5;
    for (i = 0; i < 5; i++) {
      caselut[44].f1.data[i] = iv45[i];
    }
    caselut[51].f1.size[0] = 1;
    caselut[51].f1.size[1] = 13;
    for (i = 0; i < 13; i++) {
      caselut[51].f1.data[i] = iv46[i];
    }
    caselut[58].f1.size[0] = 1;
    caselut[58].f1.size[1] = 4;
    caselut[65].f1.size[0] = 1;
    caselut[65].f1.size[1] = 4;
    caselut[58].f1.data[0] = 0;
    caselut[65].f1.data[0] = 0;
    caselut[58].f1.data[1] = 302;
    caselut[65].f1.data[1] = 301;
    caselut[58].f1.data[2] = 311;
    caselut[65].f1.data[2] = 311;
    caselut[58].f1.data[3] = 312;
    caselut[65].f1.data[3] = 312;
    caselut[72].f1.size[0] = 1;
    caselut[72].f1.size[1] = 3;
    caselut[72].f1.data[0] = 0;
    caselut[72].f1.data[1] = 311;
    caselut[72].f1.data[2] = 312;
    caselut[79].f1.size[0] = 1;
    caselut[79].f1.size[1] = 5;
    for (i = 0; i < 5; i++) {
      caselut[79].f1.data[i] = iv47[i];
    }
    caselut[86].f1.size[0] = 1;
    caselut[86].f1.size[1] = 12;
    caselut[93].f1.size[0] = 1;
    caselut[93].f1.size[1] = 12;
    for (i = 0; i < 12; i++) {
      caselut[86].f1.data[i] = iv48[i];
      caselut[93].f1.data[i] = iv49[i];
    }
    caselut[100].f1.size[0] = 1;
    caselut[100].f1.size[1] = 7;
    for (i = 0; i < 7; i++) {
      caselut[100].f1.data[i] = iv50[i];
    }
    caselut[107].f1.size[0] = 1;
    caselut[107].f1.size[1] = 21;
    for (i = 0; i < 21; i++) {
      caselut[107].f1.data[i] = iv51[i];
    }
    caselut[114].f1.size[0] = 1;
    caselut[114].f1.size[1] = 7;
    for (i = 0; i < 7; i++) {
      caselut[114].f1.data[i] = iv52[i];
    }
    caselut[121].f1.size[0] = 1;
    caselut[121].f1.size[1] = 5;
    for (i = 0; i < 5; i++) {
      caselut[121].f1.data[i] = iv53[i];
    }
    caselut[128].f1.size[0] = 1;
    caselut[128].f1.size[1] = 4;
    caselut[128].f1.data[0] = 0;
    caselut[128].f1.data[1] = 312;
    caselut[128].f1.data[2] = 320;
    caselut[128].f1.data[3] = 1303;
    caselut[135].f1.size[0] = 1;
    caselut[135].f1.size[1] = 9;
    caselut[142].f1.size[0] = 1;
    caselut[142].f1.size[1] = 9;
    for (i = 0; i < 9; i++) {
      caselut[135].f1.data[i] = iv54[i];
      caselut[142].f1.data[i] = iv55[i];
    }
    caselut[149].f1.size[0] = 1;
    caselut[149].f1.size[1] = 6;
    for (i = 0; i < 6; i++) {
      caselut[149].f1.data[i] = iv56[i];
    }
    caselut[156].f1.size[0] = 1;
    caselut[156].f1.size[1] = 5;
    for (i = 0; i < 5; i++) {
      caselut[156].f1.data[i] = iv57[i];
    }
    caselut[163].f1.size[0] = 1;
    caselut[163].f1.size[1] = 11;
    for (i = 0; i < 11; i++) {
      caselut[163].f1.data[i] = iv58[i];
    }
    caselut[170].f1.size[0] = 1;
    caselut[170].f1.size[1] = 5;
    for (i = 0; i < 5; i++) {
      caselut[170].f1.data[i] = iv59[i];
    }
    caselut[177].f1.size[0] = 1;
    caselut[177].f1.size[1] = 7;
    for (i = 0; i < 7; i++) {
      caselut[177].f1.data[i] = iv60[i];
    }
    caselut[184].f1.size[0] = 1;
    caselut[184].f1.size[1] = 4;
    caselut[184].f1.data[0] = 0;
    caselut[184].f1.data[1] = 311;
    caselut[184].f1.data[2] = 319;
    caselut[184].f1.data[3] = 1304;
    caselut[191].f1.size[0] = 1;
    caselut[191].f1.size[1] = 9;
    for (i = 0; i < 9; i++) {
      caselut[191].f1.data[i] = iv61[i];
    }
    caselut[198].f1.size[0] = 1;
    caselut[198].f1.size[1] = 6;
    for (i = 0; i < 6; i++) {
      caselut[198].f1.data[i] = iv62[i];
    }
    caselut[205].f1.size[0] = 1;
    caselut[205].f1.size[1] = 9;
    for (i = 0; i < 9; i++) {
      caselut[205].f1.data[i] = iv63[i];
    }
    caselut[212].f1.size[0] = 1;
    caselut[212].f1.size[1] = 5;
    for (i = 0; i < 5; i++) {
      caselut[212].f1.data[i] = iv64[i];
    }
    caselut[219].f1.size[0] = 1;
    caselut[219].f1.size[1] = 11;
    for (i = 0; i < 11; i++) {
      caselut[219].f1.data[i] = iv65[i];
    }
    caselut[3].f1.size[0] = 1;
    caselut[3].f1.size[1] = 4;
    caselut[10].f1.size[0] = 1;
    caselut[10].f1.size[1] = 4;
    caselut[3].f1.data[0] = 0;
    caselut[10].f1.data[0] = 0;
    caselut[3].f1.data[1] = 402;
    caselut[10].f1.data[1] = 401;
    caselut[3].f1.data[2] = 403;
    caselut[10].f1.data[2] = 403;
    caselut[3].f1.data[3] = 404;
    caselut[10].f1.data[3] = 404;
    caselut[17].f1.size[0] = 1;
    caselut[17].f1.size[1] = 3;
    caselut[17].f1.data[0] = 0;
    caselut[17].f1.data[1] = 403;
    caselut[17].f1.data[2] = 404;
    caselut[24].f1.size[0] = 1;
    caselut[24].f1.size[1] = 5;
    caselut[31].f1.size[0] = 1;
    caselut[31].f1.size[1] = 4;
    caselut[38].f1.size[0] = 1;
    caselut[38].f1.size[1] = 4;
    caselut[31].f1.data[0] = 0;
    caselut[38].f1.data[0] = 0;
    caselut[31].f1.data[1] = 402;
    caselut[38].f1.data[1] = 401;
    caselut[31].f1.data[2] = 403;
    caselut[38].f1.data[2] = 403;
    caselut[31].f1.data[3] = 404;
    caselut[38].f1.data[3] = 404;
    caselut[45].f1.size[0] = 1;
    caselut[45].f1.size[1] = 3;
    caselut[45].f1.data[0] = 0;
    caselut[45].f1.data[1] = 403;
    caselut[45].f1.data[2] = 404;
    caselut[52].f1.size[0] = 1;
    caselut[52].f1.size[1] = 5;
    caselut[59].f1.size[0] = 1;
    caselut[59].f1.size[1] = 4;
    caselut[66].f1.size[0] = 1;
    caselut[66].f1.size[1] = 4;
    caselut[59].f1.data[0] = 0;
    caselut[66].f1.data[0] = 0;
    caselut[59].f1.data[1] = 402;
    caselut[66].f1.data[1] = 401;
    caselut[59].f1.data[2] = 403;
    caselut[66].f1.data[2] = 403;
    caselut[59].f1.data[3] = 404;
    caselut[66].f1.data[3] = 404;
    caselut[73].f1.size[0] = 1;
    caselut[73].f1.size[1] = 3;
    caselut[73].f1.data[0] = 0;
    caselut[73].f1.data[1] = 403;
    caselut[73].f1.data[2] = 404;
    caselut[80].f1.size[0] = 1;
    caselut[80].f1.size[1] = 5;
    caselut[87].f1.size[0] = 1;
    caselut[87].f1.size[1] = 4;
    caselut[94].f1.size[0] = 1;
    caselut[94].f1.size[1] = 4;
    caselut[87].f1.data[0] = 0;
    caselut[94].f1.data[0] = 0;
    caselut[87].f1.data[1] = 402;
    caselut[94].f1.data[1] = 401;
    caselut[87].f1.data[2] = 403;
    caselut[94].f1.data[2] = 403;
    caselut[87].f1.data[3] = 404;
    caselut[94].f1.data[3] = 404;
    caselut[101].f1.size[0] = 1;
    caselut[101].f1.size[1] = 3;
    caselut[101].f1.data[0] = 0;
    caselut[101].f1.data[1] = 403;
    caselut[101].f1.data[2] = 404;
    caselut[108].f1.size[0] = 1;
    caselut[108].f1.size[1] = 5;
    caselut[115].f1.size[0] = 1;
    caselut[115].f1.size[1] = 3;
    caselut[115].f1.data[0] = 0;
    caselut[115].f1.data[1] = 402;
    caselut[115].f1.data[2] = 404;
    caselut[122].f1.size[0] = 1;
    caselut[122].f1.size[1] = 2;
    caselut[129].f1.size[0] = 1;
    caselut[129].f1.size[1] = 2;
    caselut[122].f1.data[0] = 0;
    caselut[129].f1.data[0] = 0;
    caselut[122].f1.data[1] = 404;
    caselut[129].f1.data[1] = 404;
    caselut[136].f1.size[0] = 1;
    caselut[136].f1.size[1] = 3;
    caselut[143].f1.size[0] = 1;
    caselut[143].f1.size[1] = 3;
    caselut[136].f1.data[0] = 0;
    caselut[143].f1.data[0] = 0;
    caselut[136].f1.data[1] = 402;
    caselut[143].f1.data[1] = 402;
    caselut[136].f1.data[2] = 404;
    caselut[143].f1.data[2] = 404;
    caselut[150].f1.size[0] = 1;
    caselut[150].f1.size[1] = 2;
    caselut[157].f1.size[0] = 1;
    caselut[157].f1.size[1] = 2;
    caselut[150].f1.data[0] = 0;
    caselut[157].f1.data[0] = 0;
    caselut[150].f1.data[1] = 404;
    caselut[157].f1.data[1] = 404;
    caselut[164].f1.size[0] = 1;
    caselut[164].f1.size[1] = 3;
    caselut[164].f1.data[0] = 0;
    caselut[164].f1.data[1] = 402;
    caselut[164].f1.data[2] = 404;
    caselut[171].f1.size[0] = 1;
    caselut[171].f1.size[1] = 2;
    caselut[171].f1.data[0] = 0;
    caselut[171].f1.data[1] = 403;
    caselut[178].f1.size[0] = 1;
    caselut[178].f1.size[1] = 3;
    caselut[178].f1.data[0] = 0;
    caselut[178].f1.data[1] = 401;
    caselut[178].f1.data[2] = 403;
    caselut[185].f1.size[0] = 1;
    caselut[185].f1.size[1] = 2;
    caselut[185].f1.data[0] = 0;
    caselut[185].f1.data[1] = 403;
    caselut[192].f1.size[0] = 1;
    caselut[192].f1.size[1] = 3;
    caselut[192].f1.data[0] = 0;
    caselut[192].f1.data[1] = 401;
    caselut[192].f1.data[2] = 403;
    caselut[199].f1.size[0] = 1;
    caselut[199].f1.size[1] = 2;
    caselut[199].f1.data[0] = 0;
    caselut[199].f1.data[1] = 403;
    caselut[206].f1.size[0] = 1;
    caselut[206].f1.size[1] = 3;
    caselut[206].f1.data[0] = 0;
    caselut[206].f1.data[1] = 401;
    caselut[206].f1.data[2] = 403;
    caselut[213].f1.size[0] = 1;
    caselut[213].f1.size[1] = 2;
    caselut[213].f1.data[0] = 0;
    caselut[213].f1.data[1] = 403;
    caselut[220].f1.size[0] = 1;
    caselut[220].f1.size[1] = 3;
    caselut[220].f1.data[0] = 0;
    caselut[220].f1.data[1] = 401;
    caselut[220].f1.data[2] = 403;
    caselut[4].f1.size[0] = 1;
    caselut[4].f1.size[1] = 4;
    caselut[4].f1.data[0] = 0;
    caselut[4].f1.data[1] = 501;
    caselut[4].f1.data[2] = 502;
    caselut[4].f1.data[3] = 505;
    caselut[11].f1.size[0] = 1;
    caselut[11].f1.size[1] = 5;
    caselut[18].f1.size[0] = 1;
    caselut[18].f1.size[1] = 4;
    caselut[18].f1.data[0] = 0;
    caselut[18].f1.data[1] = 501;
    caselut[18].f1.data[2] = 502;
    caselut[18].f1.data[3] = 505;
    caselut[25].f1.size[0] = 1;
    caselut[25].f1.size[1] = 5;
    caselut[32].f1.size[0] = 1;
    caselut[32].f1.size[1] = 5;
    caselut[39].f1.size[0] = 1;
    caselut[39].f1.size[1] = 4;
    caselut[46].f1.size[0] = 1;
    caselut[46].f1.size[1] = 4;
    caselut[39].f1.data[0] = 0;
    caselut[46].f1.data[0] = 0;
    caselut[39].f1.data[1] = 501;
    caselut[46].f1.data[1] = 501;
    caselut[39].f1.data[2] = 502;
    caselut[46].f1.data[2] = 502;
    caselut[39].f1.data[3] = 506;
    caselut[46].f1.data[3] = 506;
    caselut[53].f1.size[0] = 1;
    caselut[53].f1.size[1] = 5;
    for (i = 0; i < 5; i++) {
      i1 = iv66[i];
      caselut[24].f1.data[i] = i1;
      caselut[52].f1.data[i] = i1;
      caselut[80].f1.data[i] = i1;
      caselut[108].f1.data[i] = i1;
      i1 = iv67[i];
      caselut[11].f1.data[i] = i1;
      caselut[25].f1.data[i] = i1;
      i1 = iv68[i];
      caselut[32].f1.data[i] = i1;
      caselut[53].f1.data[i] = i1;
    }
    caselut[60].f1.size[0] = 1;
    caselut[60].f1.size[1] = 3;
    caselut[67].f1.size[0] = 1;
    caselut[67].f1.size[1] = 3;
    caselut[74].f1.size[0] = 1;
    caselut[74].f1.size[1] = 3;
    caselut[81].f1.size[0] = 1;
    caselut[81].f1.size[1] = 3;
    caselut[60].f1.data[0] = 0;
    caselut[67].f1.data[0] = 0;
    caselut[74].f1.data[0] = 0;
    caselut[81].f1.data[0] = 0;
    caselut[60].f1.data[1] = 501;
    caselut[67].f1.data[1] = 501;
    caselut[74].f1.data[1] = 501;
    caselut[81].f1.data[1] = 501;
    caselut[60].f1.data[2] = 502;
    caselut[67].f1.data[2] = 502;
    caselut[74].f1.data[2] = 502;
    caselut[81].f1.data[2] = 502;
    caselut[88].f1.size[0] = 1;
    caselut[88].f1.size[1] = 6;
    caselut[95].f1.size[0] = 1;
    caselut[95].f1.size[1] = 6;
    for (i = 0; i < 6; i++) {
      caselut[88].f1.data[i] = iv69[i];
      caselut[95].f1.data[i] = iv70[i];
    }
    caselut[102].f1.size[0] = 1;
    caselut[102].f1.size[1] = 5;
    for (i = 0; i < 5; i++) {
      caselut[102].f1.data[i] = iv71[i];
    }
    caselut[109].f1.size[0] = 1;
    caselut[109].f1.size[1] = 7;
    caselut[116].f1.size[0] = 1;
    caselut[116].f1.size[1] = 4;
    caselut[116].f1.data[0] = 0;
    caselut[116].f1.data[1] = 502;
    caselut[116].f1.data[2] = 504;
    caselut[116].f1.data[3] = 506;
    caselut[123].f1.size[0] = 1;
    caselut[123].f1.size[1] = 3;
    caselut[130].f1.size[0] = 1;
    caselut[130].f1.size[1] = 3;
    caselut[123].f1.data[0] = 0;
    caselut[130].f1.data[0] = 0;
    caselut[123].f1.data[1] = 502;
    caselut[130].f1.data[1] = 502;
    caselut[123].f1.data[2] = 506;
    caselut[130].f1.data[2] = 506;
    caselut[137].f1.size[0] = 1;
    caselut[137].f1.size[1] = 4;
    caselut[144].f1.size[0] = 1;
    caselut[144].f1.size[1] = 4;
    caselut[137].f1.data[0] = 0;
    caselut[144].f1.data[0] = 0;
    caselut[137].f1.data[1] = 502;
    caselut[144].f1.data[1] = 502;
    caselut[137].f1.data[2] = 504;
    caselut[144].f1.data[2] = 504;
    caselut[137].f1.data[3] = 506;
    caselut[144].f1.data[3] = 506;
    caselut[151].f1.size[0] = 1;
    caselut[151].f1.size[1] = 3;
    caselut[158].f1.size[0] = 1;
    caselut[158].f1.size[1] = 3;
    caselut[151].f1.data[0] = 0;
    caselut[158].f1.data[0] = 0;
    caselut[151].f1.data[1] = 502;
    caselut[158].f1.data[1] = 502;
    caselut[151].f1.data[2] = 506;
    caselut[158].f1.data[2] = 506;
    caselut[165].f1.size[0] = 1;
    caselut[165].f1.size[1] = 4;
    caselut[165].f1.data[0] = 0;
    caselut[165].f1.data[1] = 502;
    caselut[165].f1.data[2] = 504;
    caselut[165].f1.data[3] = 506;
    caselut[172].f1.size[0] = 1;
    caselut[172].f1.size[1] = 3;
    caselut[172].f1.data[0] = 0;
    caselut[172].f1.data[1] = 501;
    caselut[172].f1.data[2] = 505;
    caselut[179].f1.size[0] = 1;
    caselut[179].f1.size[1] = 4;
    caselut[179].f1.data[0] = 0;
    caselut[179].f1.data[1] = 501;
    caselut[179].f1.data[2] = 503;
    caselut[179].f1.data[3] = 505;
    caselut[186].f1.size[0] = 1;
    caselut[186].f1.size[1] = 3;
    caselut[186].f1.data[0] = 0;
    caselut[186].f1.data[1] = 501;
    caselut[186].f1.data[2] = 505;
    caselut[193].f1.size[0] = 1;
    caselut[193].f1.size[1] = 4;
    caselut[193].f1.data[0] = 0;
    caselut[193].f1.data[1] = 501;
    caselut[193].f1.data[2] = 503;
    caselut[193].f1.data[3] = 505;
    caselut[200].f1.size[0] = 1;
    caselut[200].f1.size[1] = 3;
    caselut[200].f1.data[0] = 0;
    caselut[200].f1.data[1] = 501;
    caselut[200].f1.data[2] = 505;
    caselut[207].f1.size[0] = 1;
    caselut[207].f1.size[1] = 4;
    caselut[207].f1.data[0] = 0;
    caselut[207].f1.data[1] = 501;
    caselut[207].f1.data[2] = 503;
    caselut[207].f1.data[3] = 505;
    caselut[214].f1.size[0] = 1;
    caselut[214].f1.size[1] = 3;
    caselut[214].f1.data[0] = 0;
    caselut[214].f1.data[1] = 501;
    caselut[214].f1.data[2] = 505;
    caselut[221].f1.size[0] = 1;
    caselut[221].f1.size[1] = 4;
    caselut[221].f1.data[0] = 0;
    caselut[221].f1.data[1] = 501;
    caselut[221].f1.data[2] = 503;
    caselut[221].f1.data[3] = 505;
    caselut[5].f1.size[0] = 1;
    caselut[5].f1.size[1] = 7;
    caselut[12].f1.size[0] = 1;
    caselut[12].f1.size[1] = 7;
    for (i = 0; i < 7; i++) {
      caselut[109].f1.data[i] = iv72[i];
      caselut[5].f1.data[i] = iv73[i];
      caselut[12].f1.data[i] = iv74[i];
    }
    caselut[19].f1.size[0] = 1;
    caselut[19].f1.size[1] = 5;
    for (i = 0; i < 5; i++) {
      caselut[19].f1.data[i] = iv75[i];
    }
    caselut[26].f1.size[0] = 1;
    caselut[26].f1.size[1] = 9;
    for (i = 0; i < 9; i++) {
      caselut[26].f1.data[i] = iv76[i];
    }
    caselut[33].f1.size[0] = 1;
    caselut[33].f1.size[1] = 7;
    caselut[40].f1.size[0] = 1;
    caselut[40].f1.size[1] = 7;
    for (i = 0; i < 7; i++) {
      caselut[33].f1.data[i] = iv73[i];
      caselut[40].f1.data[i] = iv74[i];
    }
    caselut[47].f1.size[0] = 1;
    caselut[47].f1.size[1] = 5;
    for (i = 0; i < 5; i++) {
      caselut[47].f1.data[i] = iv75[i];
    }
    caselut[54].f1.size[0] = 1;
    caselut[54].f1.size[1] = 9;
    for (i = 0; i < 9; i++) {
      caselut[54].f1.data[i] = iv76[i];
    }
    caselut[61].f1.size[0] = 1;
    caselut[61].f1.size[1] = 7;
    caselut[68].f1.size[0] = 1;
    caselut[68].f1.size[1] = 7;
    for (i = 0; i < 7; i++) {
      caselut[61].f1.data[i] = iv73[i];
      caselut[68].f1.data[i] = iv74[i];
    }
    caselut[75].f1.size[0] = 1;
    caselut[75].f1.size[1] = 5;
    for (i = 0; i < 5; i++) {
      caselut[75].f1.data[i] = iv75[i];
    }
    caselut[82].f1.size[0] = 1;
    caselut[82].f1.size[1] = 9;
    for (i = 0; i < 9; i++) {
      caselut[82].f1.data[i] = iv76[i];
    }
    caselut[89].f1.size[0] = 1;
    caselut[89].f1.size[1] = 7;
    caselut[96].f1.size[0] = 1;
    caselut[96].f1.size[1] = 7;
    for (i = 0; i < 7; i++) {
      caselut[89].f1.data[i] = iv73[i];
      caselut[96].f1.data[i] = iv74[i];
    }
    caselut[103].f1.size[0] = 1;
    caselut[103].f1.size[1] = 5;
    for (i = 0; i < 5; i++) {
      caselut[103].f1.data[i] = iv75[i];
    }
    caselut[110].f1.size[0] = 1;
    caselut[110].f1.size[1] = 9;
    for (i = 0; i < 9; i++) {
      caselut[110].f1.data[i] = iv76[i];
    }
    caselut[117].f1.size[0] = 1;
    caselut[117].f1.size[1] = 5;
    caselut[124].f1.size[0] = 1;
    caselut[124].f1.size[1] = 3;
    caselut[131].f1.size[0] = 1;
    caselut[131].f1.size[1] = 3;
    caselut[124].f1.data[0] = 0;
    caselut[131].f1.data[0] = 0;
    caselut[124].f1.data[1] = 602;
    caselut[131].f1.data[1] = 602;
    caselut[124].f1.data[2] = 608;
    caselut[131].f1.data[2] = 608;
    caselut[138].f1.size[0] = 1;
    caselut[138].f1.size[1] = 5;
    caselut[145].f1.size[0] = 1;
    caselut[145].f1.size[1] = 5;
    caselut[152].f1.size[0] = 1;
    caselut[152].f1.size[1] = 3;
    caselut[159].f1.size[0] = 1;
    caselut[159].f1.size[1] = 3;
    caselut[152].f1.data[0] = 0;
    caselut[159].f1.data[0] = 0;
    caselut[152].f1.data[1] = 602;
    caselut[159].f1.data[1] = 602;
    caselut[152].f1.data[2] = 608;
    caselut[159].f1.data[2] = 608;
    caselut[166].f1.size[0] = 1;
    caselut[166].f1.size[1] = 5;
    caselut[173].f1.size[0] = 1;
    caselut[173].f1.size[1] = 3;
    caselut[173].f1.data[0] = 0;
    caselut[173].f1.data[1] = 601;
    caselut[173].f1.data[2] = 607;
    caselut[180].f1.size[0] = 1;
    caselut[180].f1.size[1] = 5;
    caselut[187].f1.size[0] = 1;
    caselut[187].f1.size[1] = 3;
    caselut[187].f1.data[0] = 0;
    caselut[187].f1.data[1] = 601;
    caselut[187].f1.data[2] = 607;
    caselut[194].f1.size[0] = 1;
    caselut[194].f1.size[1] = 5;
    caselut[201].f1.size[0] = 1;
    caselut[201].f1.size[1] = 3;
    caselut[201].f1.data[0] = 0;
    caselut[201].f1.data[1] = 601;
    caselut[201].f1.data[2] = 607;
    caselut[208].f1.size[0] = 1;
    caselut[208].f1.size[1] = 5;
    caselut[215].f1.size[0] = 1;
    caselut[215].f1.size[1] = 3;
    caselut[215].f1.data[0] = 0;
    caselut[215].f1.data[1] = 601;
    caselut[215].f1.data[2] = 607;
    caselut[222].f1.size[0] = 1;
    caselut[222].f1.size[1] = 5;
    for (i = 0; i < 5; i++) {
      i1 = iv77[i];
      caselut[117].f1.data[i] = i1;
      caselut[138].f1.data[i] = i1;
      caselut[145].f1.data[i] = i1;
      caselut[166].f1.data[i] = i1;
      i1 = iv79[i];
      caselut[180].f1.data[i] = i1;
      caselut[194].f1.data[i] = i1;
      caselut[208].f1.data[i] = i1;
      caselut[222].f1.data[i] = i1;
    }
    caselut[6].f1.size[0] = 1;
    caselut[6].f1.size[1] = 8;
    for (i = 0; i < 8; i++) {
      caselut[6].f1.data[i] = iv78[i];
    }
    caselut[13].f1.size[0] = 1;
    caselut[13].f1.size[1] = 6;
    for (i = 0; i < 6; i++) {
      caselut[13].f1.data[i] = iv80[i];
    }
    caselut[20].f1.size[0] = 1;
    caselut[20].f1.size[1] = 5;
    for (i = 0; i < 5; i++) {
      caselut[20].f1.data[i] = iv81[i];
    }
    caselut[27].f1.size[0] = 1;
    caselut[27].f1.size[1] = 9;
    for (i = 0; i < 9; i++) {
      caselut[27].f1.data[i] = iv82[i];
    }
    caselut[34].f1.size[0] = 1;
    caselut[34].f1.size[1] = 6;
    for (i = 0; i < 6; i++) {
      caselut[34].f1.data[i] = iv83[i];
    }
    caselut[41].f1.size[0] = 1;
    caselut[41].f1.size[1] = 8;
    for (i = 0; i < 8; i++) {
      caselut[41].f1.data[i] = iv84[i];
    }
    caselut[48].f1.size[0] = 1;
    caselut[48].f1.size[1] = 5;
    for (i = 0; i < 5; i++) {
      caselut[48].f1.data[i] = iv85[i];
    }
    caselut[55].f1.size[0] = 1;
    caselut[55].f1.size[1] = 9;
    for (i = 0; i < 9; i++) {
      caselut[55].f1.data[i] = iv86[i];
    }
    caselut[62].f1.size[0] = 1;
    caselut[62].f1.size[1] = 4;
    caselut[69].f1.size[0] = 1;
    caselut[69].f1.size[1] = 4;
    caselut[62].f1.data[0] = 0;
    caselut[69].f1.data[0] = 0;
    caselut[62].f1.data[1] = 701;
    caselut[69].f1.data[1] = 701;
    caselut[62].f1.data[2] = 702;
    caselut[69].f1.data[2] = 702;
    caselut[62].f1.data[3] = 704;
    caselut[69].f1.data[3] = 703;
    caselut[76].f1.size[0] = 1;
    caselut[76].f1.size[1] = 3;
    caselut[76].f1.data[0] = 0;
    caselut[76].f1.data[1] = 701;
    caselut[76].f1.data[2] = 702;
    caselut[83].f1.size[0] = 1;
    caselut[83].f1.size[1] = 5;
    for (i = 0; i < 5; i++) {
      caselut[83].f1.data[i] = iv87[i];
    }
    caselut[90].f1.size[0] = 1;
    caselut[90].f1.size[1] = 10;
    caselut[97].f1.size[0] = 1;
    caselut[97].f1.size[1] = 10;
    for (i = 0; i < 10; i++) {
      caselut[90].f1.data[i] = iv88[i];
      caselut[97].f1.data[i] = iv89[i];
    }
    caselut[104].f1.size[0] = 1;
    caselut[104].f1.size[1] = 7;
    for (i = 0; i < 7; i++) {
      caselut[104].f1.data[i] = iv90[i];
    }
    caselut[111].f1.size[0] = 1;
    caselut[111].f1.size[1] = 13;
    for (i = 0; i < 13; i++) {
      caselut[111].f1.data[i] = iv91[i];
    }
    caselut[118].f1.size[0] = 1;
    caselut[118].f1.size[1] = 7;
    caselut[125].f1.size[0] = 1;
    caselut[125].f1.size[1] = 4;
    caselut[132].f1.size[0] = 1;
    caselut[132].f1.size[1] = 4;
    caselut[125].f1.data[0] = 0;
    caselut[132].f1.data[0] = 0;
    caselut[125].f1.data[1] = 702;
    caselut[132].f1.data[1] = 702;
    caselut[125].f1.data[2] = 710;
    caselut[132].f1.data[2] = 710;
    caselut[125].f1.data[3] = 1703;
    caselut[132].f1.data[3] = 1703;
    caselut[139].f1.size[0] = 1;
    caselut[139].f1.size[1] = 7;
    for (i = 0; i < 7; i++) {
      i1 = iv92[i];
      caselut[118].f1.data[i] = i1;
      caselut[139].f1.data[i] = i1;
    }
    caselut[146].f1.size[0] = 1;
    caselut[146].f1.size[1] = 9;
    for (i = 0; i < 9; i++) {
      caselut[146].f1.data[i] = iv93[i];
    }
    caselut[153].f1.size[0] = 1;
    caselut[153].f1.size[1] = 5;
    caselut[160].f1.size[0] = 1;
    caselut[160].f1.size[1] = 5;
    for (i = 0; i < 5; i++) {
      i1 = iv94[i];
      caselut[153].f1.data[i] = i1;
      caselut[160].f1.data[i] = i1;
    }
    caselut[167].f1.size[0] = 1;
    caselut[167].f1.size[1] = 9;
    for (i = 0; i < 9; i++) {
      caselut[167].f1.data[i] = iv93[i];
    }
    caselut[174].f1.size[0] = 1;
    caselut[174].f1.size[1] = 4;
    caselut[174].f1.data[0] = 0;
    caselut[174].f1.data[1] = 701;
    caselut[174].f1.data[2] = 709;
    caselut[174].f1.data[3] = 1704;
    caselut[181].f1.size[0] = 1;
    caselut[181].f1.size[1] = 7;
    caselut[188].f1.size[0] = 1;
    caselut[188].f1.size[1] = 4;
    caselut[188].f1.data[0] = 0;
    caselut[188].f1.data[1] = 701;
    caselut[188].f1.data[2] = 709;
    caselut[188].f1.data[3] = 1704;
    caselut[195].f1.size[0] = 1;
    caselut[195].f1.size[1] = 7;
    for (i = 0; i < 7; i++) {
      i1 = iv95[i];
      caselut[181].f1.data[i] = i1;
      caselut[195].f1.data[i] = i1;
    }
    caselut[202].f1.size[0] = 1;
    caselut[202].f1.size[1] = 5;
    for (i = 0; i < 5; i++) {
      caselut[202].f1.data[i] = iv96[i];
    }
    caselut[209].f1.size[0] = 1;
    caselut[209].f1.size[1] = 9;
    for (i = 0; i < 9; i++) {
      caselut[209].f1.data[i] = iv97[i];
    }
    caselut[216].f1.size[0] = 1;
    caselut[216].f1.size[1] = 5;
    for (i = 0; i < 5; i++) {
      caselut[216].f1.data[i] = iv96[i];
    }
    caselut[223].f1.size[0] = 1;
    caselut[223].f1.size[1] = 9;
    for (i = 0; i < 9; i++) {
      caselut[223].f1.data[i] = iv97[i];
    }
  }
  if ((n - 1 < 0) || (n - 1 > 6)) {
    rtDynamicBoundsError(n - 1, 0, 6, &nb_emlrtBCI);
  }
  if ((cond - 1 < 0) || (cond - 1 > 31)) {
    rtDynamicBoundsError(cond - 1, 0, 31, &ob_emlrtBCI);
  }
  cases_size[0] = 1;
  cases_size_tmp = (n + 7 * (cond - 1)) - 1;
  cases_size[1] = caselut[cases_size_tmp].f1.size[1] + 12;
  loop_ub = caselut[cases_size_tmp].f1.size[1];
  for (i = 0; i < loop_ub; i++) {
    cases_data[i] = caselut[cases_size_tmp].f1.data[i];
  }
  cases_data[caselut[cases_size_tmp].f1.size[1]] = 321;
  cases_data[caselut[cases_size_tmp].f1.size[1] + 1] = 322;
  cases_data[caselut[cases_size_tmp].f1.size[1] + 2] = 323;
  cases_data[caselut[cases_size_tmp].f1.size[1] + 3] = 324;
  cases_data[caselut[cases_size_tmp].f1.size[1] + 4] = 1305;
  cases_data[caselut[cases_size_tmp].f1.size[1] + 5] = 1306;
  cases_data[caselut[cases_size_tmp].f1.size[1] + 6] = 1307;
  cases_data[caselut[cases_size_tmp].f1.size[1] + 7] = 1308;
  cases_data[caselut[cases_size_tmp].f1.size[1] + 8] = 1309;
  cases_data[caselut[cases_size_tmp].f1.size[1] + 9] = 1310;
  cases_data[caselut[cases_size_tmp].f1.size[1] + 10] = 1311;
  cases_data[caselut[cases_size_tmp].f1.size[1] + 11] = 1312;
  // TODO Put these into appropriate places. They are sometimes necessary to
  // find a solution!
}

void select_cases_O_init()
{
  caselut_not_empty = false;
}

// End of code generation (select_cases_O.cpp)
