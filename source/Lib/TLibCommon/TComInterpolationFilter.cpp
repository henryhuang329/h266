/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.
 *
 * Copyright (c) 2010-2015, ITU/ISO/IEC
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *  * Neither the name of the ITU/ISO/IEC nor the names of its contributors may
 *    be used to endorse or promote products derived from this software without
 *    specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * \file
 * \brief Implementation of TComInterpolationFilter class
 */

// ====================================================================================================================
// Includes
// ====================================================================================================================

#include "TComRom.h"
#include "TComInterpolationFilter.h"
#include <assert.h>

#include "TComChromaFormat.h"

#if COM16_C806_SIMD_OPT
#include <emmintrin.h>  
#endif


//! \ingroup TLibCommon
//! \{

// ====================================================================================================================
// Tables
// ====================================================================================================================

#if JVET_B058_HIGH_PRECISION_MOTION_VECTOR_MC
const TFilterCoeff TComInterpolationFilter::m_lumaFilter[LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS << VCEG_AZ07_MV_ADD_PRECISION_BIT_FOR_STORE][NTAPS_LUMA] =
{
  {  0, 0,   0, 64,  0,   0,  0,  0 },
#if VCEG_AZ07_MV_ADD_PRECISION_BIT_FOR_STORE >= 2 
  {  0, 1,  -3, 63,  4,  -2,  1,  0 },
#endif
#if VCEG_AZ07_MV_ADD_PRECISION_BIT_FOR_STORE >= 1 
  { -1, 2,  -5, 62,  8,  -3,  1,  0 },
#endif
#if VCEG_AZ07_MV_ADD_PRECISION_BIT_FOR_STORE >= 2 
  { -1, 3,  -8, 60, 13,  -4,  1,  0 },
#endif
  { -1, 4, -10, 58, 17,  -5,  1,  0 },
#if VCEG_AZ07_MV_ADD_PRECISION_BIT_FOR_STORE >= 2 
  { -1, 4, -11, 52, 26,  -8,  3, -1 }, 
#endif
#if VCEG_AZ07_MV_ADD_PRECISION_BIT_FOR_STORE >= 1
  { -1, 3,  -9, 47, 31, -10,  4, -1 },
#endif
#if VCEG_AZ07_MV_ADD_PRECISION_BIT_FOR_STORE >= 2 
  { -1, 4, -11, 45, 34, -10,  4, -1 },
#endif
  { -1, 4, -11, 40, 40, -11,  4, -1 }, 
#if VCEG_AZ07_MV_ADD_PRECISION_BIT_FOR_STORE >= 2 
  { -1, 4, -10, 34, 45, -11,  4, -1 },
#endif
#if VCEG_AZ07_MV_ADD_PRECISION_BIT_FOR_STORE >= 1 
  { -1, 4, -10, 31, 47,  -9,  3, -1 },
#endif
#if VCEG_AZ07_MV_ADD_PRECISION_BIT_FOR_STORE >= 2 
  { -1, 3,  -8, 26, 52, -11,  4, -1 }, 
#endif
  {  0, 1,  -5, 17, 58, -10,  4, -1 },
#if VCEG_AZ07_MV_ADD_PRECISION_BIT_FOR_STORE >= 2 
  {  0, 1,  -4, 13, 60,  -8,  3, -1 },
#endif
#if VCEG_AZ07_MV_ADD_PRECISION_BIT_FOR_STORE >= 1 
  {  0, 1,  -3,  8, 62,  -5,  2, -1 },
#endif
#if VCEG_AZ07_MV_ADD_PRECISION_BIT_FOR_STORE >= 2 
  {  0, 1,  -2,  4, 63,  -3,  1,  0 }
#endif
};

const TFilterCoeff TComInterpolationFilter::m_chromaFilter[CHROMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS << VCEG_AZ07_MV_ADD_PRECISION_BIT_FOR_STORE][NTAPS_CHROMA] =
{
  {  0, 64,  0,  0 },
#if VCEG_AZ07_MV_ADD_PRECISION_BIT_FOR_STORE >= 2 
  { -1, 63,  2,  0 },
#endif
#if VCEG_AZ07_MV_ADD_PRECISION_BIT_FOR_STORE >= 1 
  { -2, 62,  4,  0 },
#endif
#if VCEG_AZ07_MV_ADD_PRECISION_BIT_FOR_STORE >= 2 
  { -2, 60,  7,  -1 },
#endif
  { -2, 58, 10, -2 },
#if VCEG_AZ07_MV_ADD_PRECISION_BIT_FOR_STORE >= 2 
  { -3, 57, 12, -2 },
#endif
#if VCEG_AZ07_MV_ADD_PRECISION_BIT_FOR_STORE >= 1 
  { -4, 56, 14, -2 },
#endif
#if VCEG_AZ07_MV_ADD_PRECISION_BIT_FOR_STORE >= 2 
  { -4, 55, 15, -2 },
#endif
  { -4, 54, 16, -2 }, 
#if VCEG_AZ07_MV_ADD_PRECISION_BIT_FOR_STORE >= 2 
  { -5, 53, 18, -2 },
#endif
#if VCEG_AZ07_MV_ADD_PRECISION_BIT_FOR_STORE >= 1 
  { -6, 52, 20, -2 }, 
#endif
#if VCEG_AZ07_MV_ADD_PRECISION_BIT_FOR_STORE >= 2 
  { -6, 49, 24, -3 },
#endif
  { -6, 46, 28, -4 }, 
#if VCEG_AZ07_MV_ADD_PRECISION_BIT_FOR_STORE >= 2 
  { -5, 44, 29, -4 },
#endif
#if VCEG_AZ07_MV_ADD_PRECISION_BIT_FOR_STORE >= 1 
  { -4, 42, 30, -4 },
#endif
#if VCEG_AZ07_MV_ADD_PRECISION_BIT_FOR_STORE >= 2 
  { -4, 39, 33, -4 },
#endif
  { -4, 36, 36, -4 }, 
#if VCEG_AZ07_MV_ADD_PRECISION_BIT_FOR_STORE >= 2 
  { -4, 33, 39, -4 },
#endif
#if VCEG_AZ07_MV_ADD_PRECISION_BIT_FOR_STORE >= 1 
  { -4, 30, 42, -4 }, 
#endif
#if VCEG_AZ07_MV_ADD_PRECISION_BIT_FOR_STORE >= 2 
  { -4, 29, 44, -5 },
#endif
  { -4, 28, 46, -6 },
#if VCEG_AZ07_MV_ADD_PRECISION_BIT_FOR_STORE >= 2 
  { -3, 24, 49, -6 },
#endif
#if VCEG_AZ07_MV_ADD_PRECISION_BIT_FOR_STORE >= 1 
  { -2, 20, 52, -6 }, 
#endif
#if VCEG_AZ07_MV_ADD_PRECISION_BIT_FOR_STORE >= 2 
  { -2, 18, 53, -5 },
#endif
  { -2, 16, 54, -4 },
#if VCEG_AZ07_MV_ADD_PRECISION_BIT_FOR_STORE >= 2 
  { -2, 15, 55, -4 },
#endif
#if VCEG_AZ07_MV_ADD_PRECISION_BIT_FOR_STORE >= 1 
  { -2, 14, 56, -4 },
#endif
#if VCEG_AZ07_MV_ADD_PRECISION_BIT_FOR_STORE >= 2 
  { -2, 12, 57, -3 },
#endif
  { -2, 10, 58, -2 }, 
#if VCEG_AZ07_MV_ADD_PRECISION_BIT_FOR_STORE >= 2 
  { -1,  7, 60, -2 },
#endif
#if VCEG_AZ07_MV_ADD_PRECISION_BIT_FOR_STORE >= 1 
  {  0,  4, 62, -2 },
#endif
#if VCEG_AZ07_MV_ADD_PRECISION_BIT_FOR_STORE >= 2 
  { 0,  2, 63, -1 },
#endif
};
#else
#if VCEG_AZ07_MV_ADD_PRECISION_BIT_FOR_STORE == 1
// from SHVC upsampling filter
const Short TComInterpolationFilter::m_lumaFilter[8][NTAPS_LUMA] =
{
  {  0, 0,   0, 64,  0,   0,  0,  0 },
  { -1, 2,  -5, 62,  8,  -3,  1,  0 },
  { -1, 4, -10, 58, 17,  -5,  1,  0 },
  { -1, 3,  -9, 47, 31, -10,  4, -1 },
  { -1, 4, -11, 40, 40, -11,  4, -1 }, 
  { -1, 4, -10, 31, 47,  -9,  3, -1 },
  {  0, 1,  -5, 17, 58, -10,  4, -1 },
  {  0, 1,  -3,  8, 62,  -5,  2, -1 },
};

const Short TComInterpolationFilter::m_chromaFilter[16][NTAPS_CHROMA] =
{
  {  0, 64,  0,  0 },
  { -2, 62,  4,  0 },
  { -2, 58, 10, -2 },
  { -4, 56, 14, -2 },
  { -4, 54, 16, -2 }, 
  { -6, 52, 20, -2 }, 
  { -6, 46, 28, -4 }, 
  { -4, 42, 30, -4 },
  { -4, 36, 36, -4 }, 
  { -4, 30, 42, -4 }, 
  { -4, 28, 46, -6 },
  { -2, 20, 52, -6 }, 
  { -2, 16, 54, -4 },
  { -2, 14, 56, -4 },
  { -2, 10, 58, -2 }, 
  {  0,  4, 62, -2 }  
};
#else
const TFilterCoeff TComInterpolationFilter::m_lumaFilter[LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS][NTAPS_LUMA] =
{
  {  0, 0,   0, 64,  0,   0, 0,  0 },
  { -1, 4, -10, 58, 17,  -5, 1,  0 },
  { -1, 4, -11, 40, 40, -11, 4, -1 },
  {  0, 1,  -5, 17, 58, -10, 4, -1 }
};

const TFilterCoeff TComInterpolationFilter::m_chromaFilter[CHROMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS][NTAPS_CHROMA] =
{
  {  0, 64,  0,  0 },
  { -2, 58, 10, -2 },
  { -4, 54, 16, -2 },
  { -6, 46, 28, -4 },
  { -4, 36, 36, -4 },
  { -4, 28, 46, -6 },
  { -2, 16, 54, -4 },
  { -2, 10, 58, -2 }
};
#endif
#endif

#if VCEG_AZ07_FRUC_MERGE
#if JVET_B058_HIGH_PRECISION_MOTION_VECTOR_MC
const Short TComInterpolationFilter::m_lumaFilterBilinear[LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS << VCEG_AZ07_MV_ADD_PRECISION_BIT_FOR_STORE][NTAPS_LUMA_FRUC] =
{
  { 64,  0, },
#if VCEG_AZ07_MV_ADD_PRECISION_BIT_FOR_STORE >=2 
  { 60,  4, },
#endif
#if VCEG_AZ07_MV_ADD_PRECISION_BIT_FOR_STORE >=1 
  { 56,  8, },
#endif
#if VCEG_AZ07_MV_ADD_PRECISION_BIT_FOR_STORE >=2 
  { 52, 12, },
#endif
  { 48, 16, },
#if VCEG_AZ07_MV_ADD_PRECISION_BIT_FOR_STORE >=2 
  { 44, 20, },
#endif
#if VCEG_AZ07_MV_ADD_PRECISION_BIT_FOR_STORE >=1 
  { 40, 24, },
#endif
#if VCEG_AZ07_MV_ADD_PRECISION_BIT_FOR_STORE >=2 
  { 36, 28, },
#endif
  { 32, 32, },
#if VCEG_AZ07_MV_ADD_PRECISION_BIT_FOR_STORE >=2 
  { 28, 36, },
#endif
#if VCEG_AZ07_MV_ADD_PRECISION_BIT_FOR_STORE >=1 
  { 24, 40, },
#endif
#if VCEG_AZ07_MV_ADD_PRECISION_BIT_FOR_STORE >=2 
  { 20, 44, },
#endif
  { 16, 48, },
#if VCEG_AZ07_MV_ADD_PRECISION_BIT_FOR_STORE >=2 
  { 12, 52, },
#endif
#if VCEG_AZ07_MV_ADD_PRECISION_BIT_FOR_STORE >=1 
  {  8, 56, },
#endif
#if VCEG_AZ07_MV_ADD_PRECISION_BIT_FOR_STORE >=2 
  {  4, 60, },
#endif
};
#else
#if VCEG_AZ07_MV_ADD_PRECISION_BIT_FOR_STORE == 1
const Short TComInterpolationFilter::m_lumaFilterBilinear[8][NTAPS_LUMA_FRUC] =
{
  { 64,  0, },
  { 56,  8, },
  { 48, 16, },
  { 40, 24, },
  { 32, 32, },
  { 24, 40, },
  { 16, 48, },
  {  8, 56, },
};
#else
const Short TComInterpolationFilter::m_lumaFilterBilinear[4][NTAPS_LUMA_FRUC] =
{
  { 64,  0, },
  { 48, 16, },
  { 32, 32, },
  { 16, 48, },
};
#endif
#endif
#endif

#if COM16_C1016_AFFINE
#if !JVET_C0025_AFFINE_FILTER_SIMPLIFICATION
const Short TComInterpolationFilter::m_lumaFilterAffine[(NFRACS_LUMA_AFFINE)*NTAPS_LUMA] =
{
   0,  0,   0, 256,   0,   0,  0,  0,
   0,  1,  -3, 256,   4,  -2,  0,  0,
   0,  2,  -7, 255,   8,  -3,  1,  0,
  -1,  3, -10, 255,  12,  -4,  1,  0,
  -1,  4, -13, 254,  16,  -5,  2, -1,
  -1,  5, -16, 253,  20,  -7,  2,  0,
  -1,  6, -18, 251,  25,  -9,  3, -1,
  -2,  7, -21, 250,  29, -10,  4, -1,
  -2,  8, -23, 248,  34, -12,  4, -1,
  -2,  8, -25, 246,  38, -13,  5, -1,
  -2,  9, -27, 244,  43, -15,  5, -1,
  -2, 10, -30, 242,  48, -16,  6, -2,
  -2, 10, -31, 239,  52, -17,  5,  0,
  -2, 10, -32, 237,  57, -18,  6, -2,
  -2, 11, -34, 234,  63, -21,  7, -2,
  -2, 11, -35, 231,  68, -21,  6, -2,
  -3, 13, -38, 228,  74, -24,  9, -3,
  -2, 12, -38, 224,  78, -24,  7, -1,
  -3, 14, -40, 221,  84, -27, 10, -3,
  -2, 12, -39, 217,  88, -27,  8, -1,
  -3, 13, -40, 213,  94, -28,  9, -2,
  -3, 15, -43, 210, 100, -31, 11, -3,
  -3, 13, -41, 205, 104, -30,  9, -1,
  -3, 12, -41, 201, 110, -31,  9, -1,
  -3, 15, -43, 197, 116, -35, 12, -3,
  -3, 14, -43, 192, 121, -35, 12, -2,
  -2, 13, -42, 187, 126, -35, 10, -1,
  -3, 14, -43, 183, 132, -37, 12, -2,
  -2, 13, -42, 178, 137, -38, 12, -2,
  -3, 14, -42, 173, 143, -39, 12, -2,
  -3, 15, -43, 169, 148, -41, 14, -3,
  -3, 13, -41, 163, 153, -40, 13, -2,
  -3, 13, -40, 158, 158, -40, 13, -3,
  -2, 13, -40, 153, 163, -41, 13, -3,
  -3, 14, -41, 148, 169, -43, 15, -3,
  -2, 12, -39, 143, 173, -42, 14, -3,
  -2, 12, -38, 137, 178, -42, 13, -2,
  -2, 12, -37, 132, 183, -43, 14, -3,
  -1, 10, -35, 126, 187, -42, 13, -2,
  -2, 12, -35, 121, 192, -43, 14, -3,
  -3, 12, -35, 116, 197, -43, 15, -3,
  -1,  9, -31, 110, 201, -41, 12, -3,
  -1,  9, -30, 104, 205, -41, 13, -3,
  -3, 11, -31, 100, 210, -43, 15, -3,
  -2,  9, -28,  94, 213, -40, 13, -3,
  -1,  8, -27,  88, 217, -39, 12, -2,
  -3, 10, -27,  84, 221, -40, 14, -3,
  -1,  7, -24,  78, 224, -38, 12, -2,
  -3,  9, -24,  74, 228, -38, 13, -3,
  -2,  6, -21,  68, 231, -35, 11, -2,
  -2,  7, -21,  63, 234, -34, 11, -2,
  -2,  6, -18,  57, 237, -32, 10, -2,
   0,  5, -17,  52, 239, -31, 10, -2,
  -2,  6, -16,  48, 242, -30, 10, -2,
  -1,  5, -15,  43, 244, -27,  9, -2,
  -1,  5, -13,  38, 246, -25,  8, -2,
  -1,  4, -12,  34, 248, -23,  8, -2,
  -1,  4, -10,  29, 250, -21,  7, -2,
  -1,  3,  -9,  25, 251, -18,  6, -1,
   0,  2,  -7,  20, 253, -16,  5, -1,
  -1,  2,  -5,  16, 254, -13,  4, -1,
   0,  1,  -4,  12, 255, -10,  3, -1,
   0,  1,  -3,   8, 255,  -7,  2,  0,
   0,  0,  -2,   4, 256,  -3,  1,  0
};

const Short TComInterpolationFilter::m_chromaFilterAffine[(NFRACS_CHROMA_AFFINE) * NTAPS_CHROMA] = 
{
    0, 256,   0,   0,
   -2, 255,   4,  -1,
   11, 225,  23,  -3,
    8, 225,  26,  -3,
   -6, 248,  18,  -4,
   -6, 244,  24,  -6,
   -8, 243,  27,  -6,
  -11, 242,  32,  -7,
  -12, 240,  36,  -8,
  -13, 236,  42,  -9,
  -15, 235,  45,  -9,
  -15, 230,  51, -10,
  -15, 226,  56, -11,
  -15, 221,  62, -12,
  -17, 221,  65, -13,
  -19, 219,  69, -13,
  -20, 216,  74, -14,
  -17, 208,  80, -15,
  -19, 207,  84, -16,
  -19, 201,  90, -16,
  -19, 198,  94, -17,
  -20, 195,  98, -17,
  -19, 190, 103, -18,
  -19, 185, 108, -18,
  -20, 182, 112, -18,
  -21, 178, 117, -18,
  -23, 176, 122, -19,
  -21, 170, 126, -19,
  -22, 168, 131, -21,
  -21, 162, 135, -20,
  -19, 156, 139, -20,
  -20, 153, 144, -21,
  -20, 148, 148, -20,
  -21, 144, 153, -20,
  -20, 139, 156, -19,
  -20, 135, 162, -21,
  -21, 131, 168, -22,
  -19, 126, 170, -21,
  -19, 122, 176, -23,
  -18, 117, 178, -21,
  -18, 112, 182, -20,
  -18, 108, 185, -19,
  -18, 103, 190, -19,
  -17,  98, 195, -20,
  -17,  94, 198, -19,
  -16,  90, 201, -19,
  -16,  84, 207, -19,
  -15,  80, 208, -17,
  -14,  74, 216, -20,
  -13,  69, 219, -19,
  -13,  65, 221, -17,
  -12,  62, 221, -15,
  -11,  56, 226, -15,
  -10,  51, 230, -15,
   -9,  45, 235, -15,
   -9,  42, 236, -13,
   -8,  36, 240, -12,
   -7,  32, 242, -11,
   -6,  27, 243,  -8,
   -6,  24, 244,  -6,
   -4,  18, 248,  -6,
   -3,  26, 225,   8,
   -3,  23, 225,  11,
   -1,   4, 255,  -2
};
#endif
#endif


#if COM16_C806_SIMD_OPT
inline __m128i simdInterpolateLuma4( Short const *src , Int srcStride , __m128i *mmCoeff , const __m128i & mmOffset , Int shift )
{
  __m128i sumHi = _mm_setzero_si128();
  __m128i sumLo = _mm_setzero_si128();
  for( Int n = 0 ; n < 8 ; n++ )
  {
    __m128i mmPix = _mm_loadl_epi64( ( __m128i* )src );
    __m128i hi = _mm_mulhi_epi16( mmPix , mmCoeff[n] );
    __m128i lo = _mm_mullo_epi16( mmPix , mmCoeff[n] );
    sumHi = _mm_add_epi32( sumHi , _mm_unpackhi_epi16( lo , hi ) );
    sumLo = _mm_add_epi32( sumLo , _mm_unpacklo_epi16( lo , hi ) );
    src += srcStride;
  }
  sumHi = _mm_srai_epi32( _mm_add_epi32( sumHi , mmOffset ) , shift );
  sumLo = _mm_srai_epi32( _mm_add_epi32( sumLo , mmOffset ) , shift );
  return( _mm_packs_epi32( sumLo , sumHi ) );
}

inline __m128i simdInterpolateChroma4( Short const *src , Int srcStride , __m128i *mmCoeff , const __m128i & mmOffset , Int shift )
{
  __m128i sumHi = _mm_setzero_si128();
  __m128i sumLo = _mm_setzero_si128();
  for( Int n = 0 ; n < 4 ; n++ )
  {
    __m128i mmPix = _mm_loadl_epi64( ( __m128i* )src );
    __m128i hi = _mm_mulhi_epi16( mmPix , mmCoeff[n] );
    __m128i lo = _mm_mullo_epi16( mmPix , mmCoeff[n] );
    sumHi = _mm_add_epi32( sumHi , _mm_unpackhi_epi16( lo , hi ) );
    sumLo = _mm_add_epi32( sumLo , _mm_unpacklo_epi16( lo , hi ) );
    src += srcStride;
  }
  sumHi = _mm_srai_epi32( _mm_add_epi32( sumHi , mmOffset ) , shift );
  sumLo = _mm_srai_epi32( _mm_add_epi32( sumLo , mmOffset ) , shift );
  return( _mm_packs_epi32( sumLo , sumHi ) );
}

inline __m128i simdInterpolateLuma8( Short const *src , Int srcStride , __m128i *mmCoeff , const __m128i & mmOffset , Int shift )
{
  __m128i sumHi = _mm_setzero_si128();
  __m128i sumLo = _mm_setzero_si128();
  for( Int n = 0 ; n < 8 ; n++ )
  {
    __m128i mmPix = _mm_loadu_si128( ( __m128i* )src );
    __m128i hi = _mm_mulhi_epi16( mmPix , mmCoeff[n] );
    __m128i lo = _mm_mullo_epi16( mmPix , mmCoeff[n] );
    sumHi = _mm_add_epi32( sumHi , _mm_unpackhi_epi16( lo , hi ) );
    sumLo = _mm_add_epi32( sumLo , _mm_unpacklo_epi16( lo , hi ) );
    src += srcStride;
  }
  sumHi = _mm_srai_epi32( _mm_add_epi32( sumHi , mmOffset ) , shift );
  sumLo = _mm_srai_epi32( _mm_add_epi32( sumLo , mmOffset ) , shift );
  return( _mm_packs_epi32( sumLo , sumHi ) );
}

inline __m128i simdInterpolateLuma2P8( Short const *src , Int srcStride , __m128i *mmCoeff , const __m128i & mmOffset , Int shift )
{
  __m128i sumHi = _mm_setzero_si128();
  __m128i sumLo = _mm_setzero_si128();
  for( Int n = 0 ; n < 2 ; n++ )
  {
    __m128i mmPix = _mm_loadu_si128( ( __m128i* )src );
    __m128i hi = _mm_mulhi_epi16( mmPix , mmCoeff[n] );
    __m128i lo = _mm_mullo_epi16( mmPix , mmCoeff[n] );
    sumHi = _mm_add_epi32( sumHi , _mm_unpackhi_epi16( lo , hi ) );
    sumLo = _mm_add_epi32( sumLo , _mm_unpacklo_epi16( lo , hi ) );
    src += srcStride;
  }
  sumHi = _mm_srai_epi32( _mm_add_epi32( sumHi , mmOffset ) , shift );
  sumLo = _mm_srai_epi32( _mm_add_epi32( sumLo , mmOffset ) , shift );
  return( _mm_packs_epi32( sumLo , sumHi ) );
}

inline __m128i simdInterpolateLuma2P4( Short const *src , Int srcStride , __m128i *mmCoeff , const __m128i & mmOffset , Int shift )
{
  __m128i sumHi = _mm_setzero_si128();
  __m128i sumLo = _mm_setzero_si128();
  for( Int n = 0 ; n < 2 ; n++ )
  {
    __m128i mmPix = _mm_loadl_epi64( ( __m128i* )src );
    __m128i hi = _mm_mulhi_epi16( mmPix , mmCoeff[n] );
    __m128i lo = _mm_mullo_epi16( mmPix , mmCoeff[n] );
    sumHi = _mm_add_epi32( sumHi , _mm_unpackhi_epi16( lo , hi ) );
    sumLo = _mm_add_epi32( sumLo , _mm_unpacklo_epi16( lo , hi ) );
    src += srcStride;
  }
  sumHi = _mm_srai_epi32( _mm_add_epi32( sumHi , mmOffset ) , shift );
  sumLo = _mm_srai_epi32( _mm_add_epi32( sumLo , mmOffset ) , shift );
  return( _mm_packs_epi32( sumLo , sumHi ) );
}

inline __m128i simdClip3( __m128i mmMin , __m128i mmMax , __m128i mmPix )
{
  __m128i mmMask = _mm_cmpgt_epi16( mmPix , mmMin );
  mmPix = _mm_or_si128( _mm_and_si128( mmMask , mmPix ) , _mm_andnot_si128( mmMask , mmMin ) );
  mmMask = _mm_cmplt_epi16( mmPix , mmMax );
  mmPix = _mm_or_si128( _mm_and_si128( mmMask , mmPix ) , _mm_andnot_si128( mmMask , mmMax ) );
  return( mmPix );
}
#endif

// ====================================================================================================================
// Private member functions
// ====================================================================================================================

/**
 * \brief Apply unit FIR filter to a block of samples
 *
 * \param bitDepth   bitDepth of samples
 * \param src        Pointer to source samples
 * \param srcStride  Stride of source samples
 * \param dst        Pointer to destination samples
 * \param dstStride  Stride of destination samples
 * \param width      Width of block
 * \param height     Height of block
 * \param isFirst    Flag indicating whether it is the first filtering operation
 * \param isLast     Flag indicating whether it is the last filtering operation
 */
#if JVET_D0033_ADAPTIVE_CLIPPING
Void TComInterpolationFilter::filterCopy(Int bitDepth, const Pel *src, Int srcStride, Pel *dst, Int dstStride, Int width, Int height, Bool isFirst, Bool isLast, ComponentID compID)
#else
Void TComInterpolationFilter::filterCopy(Int bitDepth, const Pel *src, Int srcStride, Pel *dst, Int dstStride, Int width, Int height, Bool isFirst, Bool isLast)
#endif
{
  Int row, col;

  if ( isFirst == isLast )
  {
    for (row = 0; row < height; row++)
    {
      for (col = 0; col < width; col++)
      {
        dst[col] = src[col];
      }

      src += srcStride;
      dst += dstStride;
    }
  }
  else if ( isFirst )
  {
    const Int shift = std::max<Int>(2, (IF_INTERNAL_PREC - bitDepth));

    for (row = 0; row < height; row++)
    {
      for (col = 0; col < width; col++)
      {
        Pel val = leftShift_round(src[col], shift);
        dst[col] = val - (Pel)IF_INTERNAL_OFFS;
      }

      src += srcStride;
      dst += dstStride;
    }
  }
  else
  {
    const Int shift = std::max<Int>(2, (IF_INTERNAL_PREC - bitDepth));

#if !JVET_D0033_ADAPTIVE_CLIPPING
    Pel maxVal = (1 << bitDepth) - 1;
    Pel minVal = 0;
#endif
    for (row = 0; row < height; row++)
    {
      for (col = 0; col < width; col++)
      {
        Pel val = src[ col ];
        val = rightShift_round((val + IF_INTERNAL_OFFS), shift);
#if JVET_D0033_ADAPTIVE_CLIPPING
        val=ClipA(val,compID);
#else
        if (val < minVal)
        {
          val = minVal;
        }
        if (val > maxVal)
        {
          val = maxVal;
        }
#endif
        dst[col] = val;
      }

      src += srcStride;
      dst += dstStride;
    }
  }
}

/**
 * \brief Apply FIR filter to a block of samples
 *
 * \tparam N          Number of taps
 * \tparam isVertical Flag indicating filtering along vertical direction
 * \tparam isFirst    Flag indicating whether it is the first filtering operation
 * \tparam isLast     Flag indicating whether it is the last filtering operation
 * \param  bitDepth   Bit depth of samples
 * \param  src        Pointer to source samples
 * \param  srcStride  Stride of source samples
 * \param  dst        Pointer to destination samples
 * \param  dstStride  Stride of destination samples
 * \param  width      Width of block
 * \param  height     Height of block
 * \param  coeff      Pointer to filter taps
 */
#if JVET_D0033_ADAPTIVE_CLIPPING
template<Int N, Bool isVertical, Bool isFirst, Bool isLast>
Void TComInterpolationFilter::filter(Int bitDepth, Pel const *src, Int srcStride, Pel *dst, Int dstStride, Int width, Int height, TFilterCoeff const *coeff, ComponentID compID)
#else
template<Int N, Bool isVertical, Bool isFirst, Bool isLast>
Void TComInterpolationFilter::filter(Int bitDepth, Pel const *src, Int srcStride, Pel *dst, Int dstStride, Int width, Int height, TFilterCoeff const *coeff)
#endif
{
  Int row, col;

  Pel c[8];
  c[0] = coeff[0];
  c[1] = coeff[1];
  if ( N >= 4 )
  {
    c[2] = coeff[2];
    c[3] = coeff[3];
  }
  if ( N >= 6 )
  {
    c[4] = coeff[4];
    c[5] = coeff[5];
  }
  if ( N == 8 )
  {
    c[6] = coeff[6];
    c[7] = coeff[7];
  }

  Int cStride = ( isVertical ) ? srcStride : 1;
  src -= ( N/2 - 1 ) * cStride;

  Int offset;
  Pel maxVal;
  Int headRoom = std::max<Int>(2, (IF_INTERNAL_PREC - bitDepth));
  Int shift    = IF_FILTER_PREC;
  // with the current settings (IF_INTERNAL_PREC = 14 and IF_FILTER_PREC = 6), though headroom can be
  // negative for bit depths greater than 14, shift will remain non-negative for bit depths of 8->20
  assert(shift >= 0);

#if JVET_D0033_ADAPTIVE_CLIPPING
  Short minVal=0;
#endif
  if ( isLast )
  {
    shift += (isFirst) ? 0 : headRoom;
    offset = 1 << (shift - 1);
    offset += (isFirst) ? 0 : IF_INTERNAL_OFFS << IF_FILTER_PREC;
#if JVET_D0033_ADAPTIVE_CLIPPING
    minVal = g_ClipParam.min(compID);
    maxVal = g_ClipParam.max(compID);
#else
    maxVal = (1 << bitDepth) - 1;
#endif
  }
  else
  {
    shift -= (isFirst) ? headRoom : 0;
    offset = (isFirst) ? -IF_INTERNAL_OFFS << shift : 0;
    maxVal = 0;
  }

#if COM16_C806_SIMD_OPT
  if( bitDepth <= 10 )
  {
    if( N == 8 && !( width & 0x07 ) )
    {
#if !JVET_D0033_ADAPTIVE_CLIPPING
      Short minVal = 0;
#endif
      __m128i mmOffset = _mm_set1_epi32( offset );
      __m128i mmCoeff[8];
      __m128i mmMin = _mm_set1_epi16( minVal );
      __m128i mmMax = _mm_set1_epi16( maxVal );
      for( Int n = 0 ; n < 8 ; n++ )
        mmCoeff[n] = _mm_set1_epi16( c[n] );
      for( row = 0 ; row < height ; row++ )
      {
        for( col = 0 ; col < width ; col += 8 )
        {
          __m128i mmFiltered = simdInterpolateLuma8( src + col , cStride , mmCoeff , mmOffset , shift );
          if( isLast )
          {
            mmFiltered = simdClip3( mmMin , mmMax , mmFiltered );
          }
          _mm_storeu_si128( ( __m128i * )( dst + col ) , mmFiltered );
        }
        src += srcStride;
        dst += dstStride;
      }
      return;
    }
    else if( N == 8 && !( width & 0x03 ) )
    {
#if !JVET_D0033_ADAPTIVE_CLIPPING
      Short minVal = 0;
#endif
      __m128i mmOffset = _mm_set1_epi32( offset );
      __m128i mmCoeff[8];
      __m128i mmMin = _mm_set1_epi16( minVal );
      __m128i mmMax = _mm_set1_epi16( maxVal );
      for( Int n = 0 ; n < 8 ; n++ )
        mmCoeff[n] = _mm_set1_epi16( c[n] );
      for( row = 0 ; row < height ; row++ )
      {
        for( col = 0 ; col < width ; col += 4 )
        {
          __m128i mmFiltered = simdInterpolateLuma4( src + col , cStride , mmCoeff , mmOffset , shift );
          if( isLast )
          {
            mmFiltered = simdClip3( mmMin , mmMax , mmFiltered );
          }
          _mm_storel_epi64( ( __m128i * )( dst + col ) , mmFiltered );
        }
        src += srcStride;
        dst += dstStride;
      }
      return;
    }
    else if( N == 4 && !( width & 0x03 ) )
    {
#if !JVET_D0033_ADAPTIVE_CLIPPING
      Short minVal = 0;
#endif
      __m128i mmOffset = _mm_set1_epi32( offset );
      __m128i mmCoeff[8];
      __m128i mmMin = _mm_set1_epi16( minVal );
      __m128i mmMax = _mm_set1_epi16( maxVal );
      for( Int n = 0 ; n < 4 ; n++ )
        mmCoeff[n] = _mm_set1_epi16( c[n] );
      for( row = 0 ; row < height ; row++ )
      {
        for( col = 0 ; col < width ; col += 4 )
        {
          __m128i mmFiltered = simdInterpolateChroma4( src + col , cStride , mmCoeff , mmOffset , shift );
          if( isLast )
          {
            mmFiltered = simdClip3( mmMin , mmMax , mmFiltered );
          }
          _mm_storel_epi64( ( __m128i * )( dst + col ) , mmFiltered );
        }
        src += srcStride;
        dst += dstStride;
      }
      return;
    }
    else if( N == 2 && !( width & 0x07 ) )
    {
#if !JVET_D0033_ADAPTIVE_CLIPPING
      Short minVal = 0;
#endif
      __m128i mmOffset = _mm_set1_epi32( offset );
      __m128i mmCoeff[2];
      __m128i mmMin = _mm_set1_epi16( minVal );
      __m128i mmMax = _mm_set1_epi16( maxVal );
      for( Int n = 0 ; n < 2 ; n++ )
        mmCoeff[n] = _mm_set1_epi16( c[n] );
      for( row = 0 ; row < height ; row++ )
      {
        for( col = 0 ; col < width ; col += 8 )
        {
          __m128i mmFiltered = simdInterpolateLuma2P8( src + col , cStride , mmCoeff , mmOffset , shift );
          if( isLast )
          {
            mmFiltered = simdClip3( mmMin , mmMax , mmFiltered );
          }
          _mm_storeu_si128( ( __m128i * )( dst + col ) , mmFiltered );
        }
        src += srcStride;
        dst += dstStride;
      }
      return;
    }
    else if( N == 2 && !( width & 0x03 ) )
    {
#if !JVET_D0033_ADAPTIVE_CLIPPING
      Short minVal = 0;
#endif
      __m128i mmOffset = _mm_set1_epi32( offset );
      __m128i mmCoeff[8];
      __m128i mmMin = _mm_set1_epi16( minVal );
      __m128i mmMax = _mm_set1_epi16( maxVal );
      for( Int n = 0 ; n < 2 ; n++ )
        mmCoeff[n] = _mm_set1_epi16( c[n] );
      for( row = 0 ; row < height ; row++ )
      {
        for( col = 0 ; col < width ; col += 4 )
        {
          __m128i mmFiltered = simdInterpolateLuma2P4( src + col , cStride , mmCoeff , mmOffset , shift );
          if( isLast )
          {
            mmFiltered = simdClip3( mmMin , mmMax , mmFiltered );
          }
          _mm_storel_epi64( ( __m128i * )( dst + col ) , mmFiltered );
        }
        src += srcStride;
        dst += dstStride;
      }
      return;
    }
  }
#endif

  for (row = 0; row < height; row++)
  {
    for (col = 0; col < width; col++)
    {
      Int sum;

      sum  = src[ col + 0 * cStride] * c[0];
      sum += src[ col + 1 * cStride] * c[1];
      if ( N >= 4 )
      {
        sum += src[ col + 2 * cStride] * c[2];
        sum += src[ col + 3 * cStride] * c[3];
      }
      if ( N >= 6 )
      {
        sum += src[ col + 4 * cStride] * c[4];
        sum += src[ col + 5 * cStride] * c[5];
      }
      if ( N == 8 )
      {
        sum += src[ col + 6 * cStride] * c[6];
        sum += src[ col + 7 * cStride] * c[7];
      }

      Pel val = ( sum + offset ) >> shift;
      if ( isLast )
      {
#if JVET_D0033_ADAPTIVE_CLIPPING
        val = ClipA(val,compID);
#else
        val = ( val < 0 ) ? 0 : val;
        val = ( val > maxVal ) ? maxVal : val;
#endif
      }
      dst[col] = val;
    }

    src += srcStride;
    dst += dstStride;
  }
}

/**
 * \brief Filter a block of samples (horizontal)
 *
 * \tparam N          Number of taps
 * \param  bitDepth   Bit depth of samples
 * \param  src        Pointer to source samples
 * \param  srcStride  Stride of source samples
 * \param  dst        Pointer to destination samples
 * \param  dstStride  Stride of destination samples
 * \param  width      Width of block
 * \param  height     Height of block
 * \param  isLast     Flag indicating whether it is the last filtering operation
 * \param  coeff      Pointer to filter taps
 */
#if JVET_D0033_ADAPTIVE_CLIPPING
template<Int N>
Void TComInterpolationFilter::filterHor(Int bitDepth, Pel *src, Int srcStride, Pel *dst, Int dstStride, Int width, Int height, Bool isLast, TFilterCoeff const *coeff, ComponentID compID)
#else
template<Int N>
Void TComInterpolationFilter::filterHor(Int bitDepth, Pel *src, Int srcStride, Pel *dst, Int dstStride, Int width, Int height, Bool isLast, TFilterCoeff const *coeff)
#endif
{
  if ( isLast )
  {
#if JVET_D0033_ADAPTIVE_CLIPPING
      filter<N, false, true, true>(bitDepth, src, srcStride, dst, dstStride, width, height, coeff, compID);
#else
    filter<N, false, true, true>(bitDepth, src, srcStride, dst, dstStride, width, height, coeff);
#endif
  }
  else
  {
#if JVET_D0033_ADAPTIVE_CLIPPING
      filter<N, false, true, false>(bitDepth, src, srcStride, dst, dstStride, width, height, coeff, compID);
#else
    filter<N, false, true, false>(bitDepth, src, srcStride, dst, dstStride, width, height, coeff);
#endif
  }
}

/**
 * \brief Filter a block of samples (vertical)
 *
 * \tparam N          Number of taps
 * \param  bitDepth   Bit depth
 * \param  src        Pointer to source samples
 * \param  srcStride  Stride of source samples
 * \param  dst        Pointer to destination samples
 * \param  dstStride  Stride of destination samples
 * \param  width      Width of block
 * \param  height     Height of block
 * \param  isFirst    Flag indicating whether it is the first filtering operation
 * \param  isLast     Flag indicating whether it is the last filtering operation
 * \param  coeff      Pointer to filter taps
 */
#if JVET_D0033_ADAPTIVE_CLIPPING
template<Int N>
Void TComInterpolationFilter::filterVer(Int bitDepth, Pel *src, Int srcStride, Pel *dst, Int dstStride, Int width, Int height, Bool isFirst, Bool isLast, TFilterCoeff const *coeff, ComponentID compID)
{
    if (isFirst && isLast)
    {
        filter<N, true, true, true>(bitDepth, src, srcStride, dst, dstStride, width, height, coeff, compID);
    }
    else if (isFirst && !isLast)
    {
        filter<N, true, true, false>(bitDepth, src, srcStride, dst, dstStride, width, height, coeff, compID);
    }
    else if (!isFirst && isLast)
    {
        filter<N, true, false, true>(bitDepth, src, srcStride, dst, dstStride, width, height, coeff, compID);
    }
    else
    {
        filter<N, true, false, false>(bitDepth, src, srcStride, dst, dstStride, width, height, coeff, compID);
    }
}
#else
template<Int N>
Void TComInterpolationFilter::filterVer(Int bitDepth, Pel *src, Int srcStride, Pel *dst, Int dstStride, Int width, Int height, Bool isFirst, Bool isLast, TFilterCoeff const *coeff)
{
  if ( isFirst && isLast )
  {
    filter<N, true, true, true>(bitDepth, src, srcStride, dst, dstStride, width, height, coeff);
  }
  else if ( isFirst && !isLast )
  {
    filter<N, true, true, false>(bitDepth, src, srcStride, dst, dstStride, width, height, coeff);
  }
  else if ( !isFirst && isLast )
  {
    filter<N, true, false, true>(bitDepth, src, srcStride, dst, dstStride, width, height, coeff);
  }
  else
  {
    filter<N, true, false, false>(bitDepth, src, srcStride, dst, dstStride, width, height, coeff);
  }
}
#endif

#if COM16_C1016_AFFINE
#if !JVET_C0025_AFFINE_FILTER_SIMPLIFICATION
/**
 * \brief Apply FIR filter to a block of samples
 *
 * \tparam N          Number of taps
 * \tparam isVertical Flag indicating filtering along vertical direction
 * \tparam isFirst    Flag indicating whether it is the first filtering operation
 * \tparam isLast     Flag indicating whether it is the last filtering operation
 * \param  bitDepth   Bit depth of samples
 * \param  src        Pointer to source samples
 * \param  srcStride  Stride of source samples
 * \param  dst        Pointer to destination samples
 * \param  dstStride  Stride of destination samples
 * \param  width      Width of block
 * \param  height     Height of block
 * \param  coeff      Pointer to filter taps
 */
template<Int N, Bool isVertical, Bool isFirst, Bool isLast>
Void TComInterpolationFilter::filterAffine(Int bitDepth, Pel const *src, Int srcStride, Pel *dst, Int dstStride, Int width, Int height, TFilterCoeff const *coeff
                                           #if JVET_D0033_ADAPTIVE_CLIPPING
                                                       , ComponentID compID
                                           #endif
                                           )
{
  Int row, col;

  Pel c[8];
  c[0] = coeff[0];
  c[1] = coeff[1];
  if ( N >= 4 )
  {
    c[2] = coeff[2];
    c[3] = coeff[3];
  }
  if ( N >= 6 )
  {
    c[4] = coeff[4];
    c[5] = coeff[5];
  }
  if ( N == 8 )
  {
    c[6] = coeff[6];
    c[7] = coeff[7];
  }

  Int cStride = ( isVertical ) ? srcStride : 1;
  src -= ( N/2 - 1 ) * cStride;

  Int offset;
  Pel maxVal;
  Int headRoom = std::max<Int>(2, (IF_INTERNAL_PREC - bitDepth));
  Int shift    = IF_FILTER_PREC_AFFINE;
  // with the current settings (IF_INTERNAL_PREC = 14 and IF_FILTER_PREC = 6), though headroom can be
  // negative for bit depths greater than 14, shift will remain non-negative for bit depths of 8->20
  assert(shift >= 0);

#if JVET_D0033_ADAPTIVE_CLIPPING
    Short minVal=0;
#endif
  if ( isLast )
  {
    shift += (isFirst) ? 0 : headRoom;
    offset = 1 << (shift - 1);
    offset += (isFirst) ? 0 : IF_INTERNAL_OFFS << IF_FILTER_PREC_AFFINE;
#if JVET_D0033_ADAPTIVE_CLIPPING
        minVal = g_ClipParam.min(compID);
        maxVal = g_ClipParam.max(compID);
#else
    maxVal = (1 << bitDepth) - 1;
#endif
  }
  else
  {
    shift -= (isFirst) ? headRoom : 0;
    offset = (isFirst) ? -IF_INTERNAL_OFFS << shift : 0;
    maxVal = 0;
  }

#if COM16_C806_SIMD_OPT
  if( bitDepth <= 10 )
  {
    if( N == 8 && !( width & 0x07 ) )
    {
#if !JVET_D0033_ADAPTIVE_CLIPPING
      Short minVal = 0;
#endif
      __m128i mmOffset = _mm_set1_epi32( offset );
      __m128i mmCoeff[8];
      __m128i mmMin = _mm_set1_epi16( minVal );
      __m128i mmMax = _mm_set1_epi16( maxVal );
      for( Int n = 0 ; n < 8 ; n++ )
        mmCoeff[n] = _mm_set1_epi16( c[n] );
      for( row = 0 ; row < height ; row++ )
      {
        for( col = 0 ; col < width ; col += 8 )
        {
          __m128i mmFiltered = simdInterpolateLuma8( src + col , cStride , mmCoeff , mmOffset , shift );
          if( isLast )
          {
            mmFiltered = simdClip3( mmMin , mmMax , mmFiltered );
          }
          _mm_storeu_si128( ( __m128i * )( dst + col ) , mmFiltered );
        }
        src += srcStride;
        dst += dstStride;
      }
      return;
    }
    else if( N == 8 && !( width & 0x03 ) )
    {
#if !JVET_D0033_ADAPTIVE_CLIPPING
      Short minVal = 0;
#endif
      __m128i mmOffset = _mm_set1_epi32( offset );
      __m128i mmCoeff[8];
      __m128i mmMin = _mm_set1_epi16( minVal );
      __m128i mmMax = _mm_set1_epi16( maxVal );
      for( Int n = 0 ; n < 8 ; n++ )
        mmCoeff[n] = _mm_set1_epi16( c[n] );
      for( row = 0 ; row < height ; row++ )
      {
        for( col = 0 ; col < width ; col += 4 )
        {
          __m128i mmFiltered = simdInterpolateLuma4( src + col , cStride , mmCoeff , mmOffset , shift );
          if( isLast )
          {
            mmFiltered = simdClip3( mmMin , mmMax , mmFiltered );
          }
          _mm_storel_epi64( ( __m128i * )( dst + col ) , mmFiltered );
        }
        src += srcStride;
        dst += dstStride;
      }
      return;
    }
    else if( N == 4 && !( width & 0x03 ) )
    {
#if !JVET_D0033_ADAPTIVE_CLIPPING
      Short minVal = 0;
#endif
      __m128i mmOffset = _mm_set1_epi32( offset );
      __m128i mmCoeff[8];
      __m128i mmMin = _mm_set1_epi16( minVal );
      __m128i mmMax = _mm_set1_epi16( maxVal );
      for( Int n = 0 ; n < 4 ; n++ )
        mmCoeff[n] = _mm_set1_epi16( c[n] );
      for( row = 0 ; row < height ; row++ )
      {
        for( col = 0 ; col < width ; col += 4 )
        {
          __m128i mmFiltered = simdInterpolateChroma4( src + col , cStride , mmCoeff , mmOffset , shift );
          if( isLast )
          {
            mmFiltered = simdClip3( mmMin , mmMax , mmFiltered );
          }
          _mm_storel_epi64( ( __m128i * )( dst + col ) , mmFiltered );
        }
        src += srcStride;
        dst += dstStride;
      }
      return;
    }
    else if( N == 2 && !( width & 0x07 ) )
    {
#if !JVET_D0033_ADAPTIVE_CLIPPING
      Short minVal = 0;
#endif
      __m128i mmOffset = _mm_set1_epi32( offset );
      __m128i mmCoeff[2];
      __m128i mmMin = _mm_set1_epi16( minVal );
      __m128i mmMax = _mm_set1_epi16( maxVal );
      for( Int n = 0 ; n < 2 ; n++ )
        mmCoeff[n] = _mm_set1_epi16( c[n] );
      for( row = 0 ; row < height ; row++ )
      {
        for( col = 0 ; col < width ; col += 8 )
        {
          __m128i mmFiltered = simdInterpolateLuma2P8( src + col , cStride , mmCoeff , mmOffset , shift );
          if( isLast )
          {
            mmFiltered = simdClip3( mmMin , mmMax , mmFiltered );
          }
          _mm_storeu_si128( ( __m128i * )( dst + col ) , mmFiltered );
        }
        src += srcStride;
        dst += dstStride;
      }
      return;
    }
    else if( N == 2 && !( width & 0x03 ) )
    {
#if !JVET_D0033_ADAPTIVE_CLIPPING
      Short minVal = 0;
#endif
      __m128i mmOffset = _mm_set1_epi32( offset );
      __m128i mmCoeff[8];
      __m128i mmMin = _mm_set1_epi16( minVal );
      __m128i mmMax = _mm_set1_epi16( maxVal );
      for( Int n = 0 ; n < 2 ; n++ )
        mmCoeff[n] = _mm_set1_epi16( c[n] );
      for( row = 0 ; row < height ; row++ )
      {
        for( col = 0 ; col < width ; col += 4 )
        {
          __m128i mmFiltered = simdInterpolateLuma2P4( src + col , cStride , mmCoeff , mmOffset , shift );
          if( isLast )
          {
            mmFiltered = simdClip3( mmMin , mmMax , mmFiltered );
          }
          _mm_storel_epi64( ( __m128i * )( dst + col ) , mmFiltered );
        }
        src += srcStride;
        dst += dstStride;
      }
      return;
    }
  }
#endif

  for (row = 0; row < height; row++)
  {
    for (col = 0; col < width; col++)
    {
      Int sum;

      sum  = src[ col + 0 * cStride] * c[0];
      sum += src[ col + 1 * cStride] * c[1];
      if ( N >= 4 )
      {
        sum += src[ col + 2 * cStride] * c[2];
        sum += src[ col + 3 * cStride] * c[3];
      }
      if ( N >= 6 )
      {
        sum += src[ col + 4 * cStride] * c[4];
        sum += src[ col + 5 * cStride] * c[5];
      }
      if ( N == 8 )
      {
        sum += src[ col + 6 * cStride] * c[6];
        sum += src[ col + 7 * cStride] * c[7];
      }

      Pel val = ( sum + offset ) >> shift;
      if ( isLast )
      {
#if JVET_D0033_ADAPTIVE_CLIPPING
                val = ClipA(val,compID);
#else
        val = ( val < 0 ) ? 0 : val;
        val = ( val > maxVal ) ? maxVal : val;
#endif
      }
      dst[col] = val;
    }

    src += srcStride;
    dst += dstStride;
  }
}

/**
 * \brief Filter a block of samples (horizontal)
 *
 * \tparam N          Number of taps
 * \param  bitDepth   Bit depth of samples
 * \param  src        Pointer to source samples
 * \param  srcStride  Stride of source samples
 * \param  dst        Pointer to destination samples
 * \param  dstStride  Stride of destination samples
 * \param  width      Width of block
 * \param  height     Height of block
 * \param  isLast     Flag indicating whether it is the last filtering operation
 * \param  coeff      Pointer to filter taps
 */
template<Int N>
Void TComInterpolationFilter::filterHorAffine(Int bitDepth, Pel *src, Int srcStride, Pel *dst, Int dstStride, Int width, Int height, Bool isLast, TFilterCoeff const *coeff
                                              #if JVET_D0033_ADAPTIVE_CLIPPING
                                                          ,ComponentID compID
                                              #endif
                                              )
{
  if ( isLast )
  {
        filterAffine<N, false, true, true>(bitDepth, src, srcStride, dst, dstStride, width, height, coeff
                                   #if JVET_D0033_ADAPTIVE_CLIPPING
                                               , compID
                                   #endif
                                           );
  }
  else
  {
        filterAffine<N, false, true, false>(bitDepth, src, srcStride, dst, dstStride, width, height, coeff
                                    #if JVET_D0033_ADAPTIVE_CLIPPING
                                                , compID
                                    #endif
                                            );
  }
}

/**
 * \brief Filter a block of samples (vertical)
 *
 * \tparam N          Number of taps
 * \param  bitDepth   Bit depth
 * \param  src        Pointer to source samples
 * \param  srcStride  Stride of source samples
 * \param  dst        Pointer to destination samples
 * \param  dstStride  Stride of destination samples
 * \param  width      Width of block
 * \param  height     Height of block
 * \param  isFirst    Flag indicating whether it is the first filtering operation
 * \param  isLast     Flag indicating whether it is the last filtering operation
 * \param  coeff      Pointer to filter taps
 */
template<Int N>
Void TComInterpolationFilter::filterVerAffine(Int bitDepth, Pel *src, Int srcStride, Pel *dst, Int dstStride, Int width, Int height, Bool isFirst, Bool isLast, TFilterCoeff const *coeff
                                              #if JVET_D0033_ADAPTIVE_CLIPPING
                                              ,ComponentID compID
                                              #endif
                                              )
{
  if ( isFirst && isLast )
  {
        filterAffine<N, true, true, true>(bitDepth, src, srcStride, dst, dstStride, width, height, coeff
                                  #if JVET_D0033_ADAPTIVE_CLIPPING
                                              , compID
                                  #endif
                                          );
  }
  else if ( isFirst && !isLast )
  {
        filterAffine<N, true, true, false>(bitDepth, src, srcStride, dst, dstStride, width, height, coeff
                                   #if JVET_D0033_ADAPTIVE_CLIPPING
                                               , compID
                                   #endif
                                           );
  }
  else if ( !isFirst && isLast )
  {
        filterAffine<N, true, false, true>(bitDepth, src, srcStride, dst, dstStride, width, height, coeff
                                   #if JVET_D0033_ADAPTIVE_CLIPPING
                                               , compID
                                   #endif
                                           );
  }
  else
  {
        filterAffine<N, true, false, false>(bitDepth, src, srcStride, dst, dstStride, width, height, coeff
                                    #if JVET_D0033_ADAPTIVE_CLIPPING
                                                , compID
                                    #endif
                                            );
  }
}
#endif
#endif

// ====================================================================================================================
// Public member functions
// ====================================================================================================================

/**
 * \brief Filter a block of Luma/Chroma samples (horizontal)
 *
 * \param  compID     Chroma component ID
 * \param  src        Pointer to source samples
 * \param  srcStride  Stride of source samples
 * \param  dst        Pointer to destination samples
 * \param  dstStride  Stride of destination samples
 * \param  width      Width of block
 * \param  height     Height of block
 * \param  frac       Fractional sample offset
 * \param  isLast     Flag indicating whether it is the last filtering operation
 * \param  fmt        Chroma format
 * \param  bitDepth   Bit depth
 */
Void TComInterpolationFilter::filterHor(const ComponentID compID, Pel *src, Int srcStride, Pel *dst, Int dstStride, Int width, Int height, Int frac, Bool isLast, const ChromaFormat fmt, const Int bitDepth 
#if VCEG_AZ07_FRUC_MERGE
  , Int nFilterIdx
#endif
  )
{
  if ( frac == 0 )
  {
#if JVET_D0033_ADAPTIVE_CLIPPING
      filterCopy(bitDepth, src, srcStride, dst, dstStride, width, height, true, isLast, compID);
#else
    filterCopy(bitDepth, src, srcStride, dst, dstStride, width, height, true, isLast );
#endif
  }
  else if (isLuma(compID))
  {
#if VCEG_AZ07_MV_ADD_PRECISION_BIT_FOR_STORE
    assert(frac >= 0 && frac < (LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS<<VCEG_AZ07_MV_ADD_PRECISION_BIT_FOR_STORE));
#else
    assert(frac >= 0 && frac < LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS);
#endif
#if VCEG_AZ07_FRUC_MERGE
    if( nFilterIdx == 1 )
      filterHor<NTAPS_LUMA_FRUC>(bitDepth, src, srcStride, dst, dstStride, width, height, isLast, m_lumaFilterBilinear[frac]
#if JVET_D0033_ADAPTIVE_CLIPPING
      , compID
#endif
      );
    else
#endif
    filterHor<NTAPS_LUMA>(bitDepth, src, srcStride, dst, dstStride, width, height, isLast, m_lumaFilter[frac]
#if JVET_D0033_ADAPTIVE_CLIPPING
    , compID
#endif
    );
  }
  else
  {
    const UInt csx = getComponentScaleX(compID, fmt);
#if VCEG_AZ07_MV_ADD_PRECISION_BIT_FOR_STORE
    assert(frac >=0 && csx<2 && (frac<<(1-csx)) < (CHROMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS<<VCEG_AZ07_MV_ADD_PRECISION_BIT_FOR_STORE));
#else
    assert(frac >=0 && csx<2 && (frac<<(1-csx)) < CHROMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS);
#endif
    filterHor<NTAPS_CHROMA>(bitDepth, src, srcStride, dst, dstStride, width, height, isLast, m_chromaFilter[frac<<(1-csx)]
#if JVET_D0033_ADAPTIVE_CLIPPING
        , compID
#endif
        );
  }
}


/**
 * \brief Filter a block of Luma/Chroma samples (vertical)
 *
 * \param  compID     Colour component ID
 * \param  src        Pointer to source samples
 * \param  srcStride  Stride of source samples
 * \param  dst        Pointer to destination samples
 * \param  dstStride  Stride of destination samples
 * \param  width      Width of block
 * \param  height     Height of block
 * \param  frac       Fractional sample offset
 * \param  isFirst    Flag indicating whether it is the first filtering operation
 * \param  isLast     Flag indicating whether it is the last filtering operation
 * \param  fmt        Chroma format
 * \param  bitDepth   Bit depth
 */
Void TComInterpolationFilter::filterVer(const ComponentID compID, Pel *src, Int srcStride, Pel *dst, Int dstStride, Int width, Int height, Int frac, Bool isFirst, Bool isLast, const ChromaFormat fmt, const Int bitDepth 
#if VCEG_AZ07_FRUC_MERGE
  , Int nFilterIdx
#endif
  )
{
  if ( frac == 0 )
  {
#if JVET_D0033_ADAPTIVE_CLIPPING
      filterCopy(bitDepth, src, srcStride, dst, dstStride, width, height, isFirst, isLast, compID);
#else
    filterCopy(bitDepth, src, srcStride, dst, dstStride, width, height, isFirst, isLast );
#endif
  }
  else if (isLuma(compID))
  {
#if VCEG_AZ07_MV_ADD_PRECISION_BIT_FOR_STORE
    assert(frac >= 0 && frac < (LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS<<VCEG_AZ07_MV_ADD_PRECISION_BIT_FOR_STORE));
#else
    assert(frac >= 0 && frac < LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS);
#endif
#if VCEG_AZ07_FRUC_MERGE
    if( nFilterIdx == 1 )
      filterVer<NTAPS_LUMA_FRUC>(bitDepth, src, srcStride, dst, dstStride, width, height, isFirst, isLast, m_lumaFilterBilinear[frac]
#if JVET_D0033_ADAPTIVE_CLIPPING
      , compID
#endif
      );
    else
#endif
    filterVer<NTAPS_LUMA>(bitDepth, src, srcStride, dst, dstStride, width, height, isFirst, isLast, m_lumaFilter[frac]
#if JVET_D0033_ADAPTIVE_CLIPPING
    , compID
#endif
    );
  }
  else
  {
    const UInt csy = getComponentScaleY(compID, fmt);
#if VCEG_AZ07_MV_ADD_PRECISION_BIT_FOR_STORE
    assert(frac >=0 && csy<2 && (frac<<(1-csy)) < (CHROMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS<<VCEG_AZ07_MV_ADD_PRECISION_BIT_FOR_STORE));
#else
    assert(frac >=0 && csy<2 && (frac<<(1-csy)) < CHROMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS);
#endif
    filterVer<NTAPS_CHROMA>(bitDepth, src, srcStride, dst, dstStride, width, height, isFirst, isLast, m_chromaFilter[frac<<(1-csy)]
#if JVET_D0033_ADAPTIVE_CLIPPING
        , compID
#endif
        );
  }
}

#if COM16_C1016_AFFINE
#if !JVET_C0025_AFFINE_FILTER_SIMPLIFICATION
/**
 * \brief Filter a block of Luma/Chroma samples (horizontal)
 *
 * \param  compID     Color component ID
 * \param  src        Pointer to source samples
 * \param  srcStride  Stride of source samples
 * \param  dst        Pointer to destination samples
 * \param  dstStride  Stride of destination samples
 * \param  width      Width of block
 * \param  height     Height of block
 * \param  frac       Fractional sample offset
 * \param  isLast     Flag indicating whether it is the last filtering operation
 * \param  fmt        Chroma format
 * \param  bitDepth   Bit depth
 */
Void TComInterpolationFilter::filterHorAffine(const ComponentID compID, Pel *src, Int srcStride, Pel *dst, Int dstStride, Int width, Int height, Int frac, Bool isLast, const ChromaFormat fmt, const Int bitDepth )
{
  if ( frac == 0 )
  {
        filterCopy(bitDepth, src, srcStride, dst, dstStride, width, height, true, isLast
           #if JVET_D0033_ADAPTIVE_CLIPPING
                   ,compID
           #endif
                   );
  }
  else if (isLuma(compID))
  {
    assert(frac >= 0 && frac < NFRACS_LUMA_AFFINE);
        filterHorAffine<NTAPS_LUMA>( bitDepth, src, srcStride, dst, dstStride, width, height, isLast, m_lumaFilterAffine+frac*NTAPS_LUMA
                             #if JVET_D0033_ADAPTIVE_CLIPPING
                                     ,compID
                             #endif
                                     );
  }
  else
  {
    assert(frac >= 0 && frac < NFRACS_CHROMA_AFFINE);
        filterHorAffine<NTAPS_CHROMA>( bitDepth, src, srcStride, dst, dstStride, width, height, isLast, m_chromaFilterAffine+frac*NTAPS_CHROMA
                               #if JVET_D0033_ADAPTIVE_CLIPPING
                                       ,compID
                               #endif
                                       );
  }
}


/**
 * \brief Filter a block of Luma/Chroma samples (vertical)
 *
 * \param  compID     Color component ID
 * \param  src        Pointer to source samples
 * \param  srcStride  Stride of source samples
 * \param  dst        Pointer to destination samples
 * \param  dstStride  Stride of destination samples
 * \param  width      Width of block
 * \param  height     Height of block
 * \param  frac       Fractional sample offset
 * \param  isFirst    Flag indicating whether it is the first filtering operation
 * \param  isLast     Flag indicating whether it is the last filtering operation
 * \param  fmt        Chroma format
 * \param  bitDepth   Bit depth
 */
Void TComInterpolationFilter::filterVerAffine(const ComponentID compID, Pel *src, Int srcStride, Pel *dst, Int dstStride, Int width, Int height, Int frac, Bool isFirst, Bool isLast, const ChromaFormat fmt, const Int bitDepth )
{
  if ( frac == 0 )
  {
        filterCopy(bitDepth, src, srcStride, dst, dstStride, width, height, isFirst, isLast
           #if JVET_D0033_ADAPTIVE_CLIPPING
                   ,compID
           #endif
                   );
  }
  else if (isLuma(compID))
  {
    assert(frac >= 0 && frac < NFRACS_LUMA_AFFINE);
        filterVerAffine<NTAPS_LUMA>( bitDepth, src, srcStride, dst, dstStride, width, height, isFirst, isLast, m_lumaFilterAffine+frac*NTAPS_LUMA
                             #if JVET_D0033_ADAPTIVE_CLIPPING
                                     ,compID
                             #endif
                                     );
  }
  else
  {
    assert(frac >= 0 && frac < NFRACS_CHROMA_AFFINE);
    filterVerAffine<NTAPS_CHROMA>( bitDepth, src, srcStride, dst, dstStride, width, height, isFirst, isLast, m_chromaFilterAffine+frac*NTAPS_CHROMA );
  }
}
#endif
#endif

//! \}
