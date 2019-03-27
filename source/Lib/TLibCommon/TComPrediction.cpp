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

/** \file     TComPrediction.cpp
    \brief    prediction class
*/

#include <memory.h>
#include "TComPrediction.h"
#include "TComPic.h"
#include "TComTU.h"

//! \ingroup TLibCommon
//! \{

// ====================================================================================================================
// Tables
// ====================================================================================================================

const UChar TComPrediction::m_aucIntraFilter[MAX_NUM_CHANNEL_TYPE][MAX_INTRA_FILTER_DEPTHS] =
{
  { // Luma
#if JVET_C0024_QTBT
    20, //2x2
#endif
#if VCEG_AZ07_INTRA_65ANG_MODES
    20, //4x4
    14, //8x8
    2,  //16x16
    0,  //32x32
    20, //64x64
#if COM16_C806_LARGE_CTU
    0, //128x128
#if !JVET_C0024_QTBT
    0, //256x256
#endif
#endif
#else
    10, //4x4
    7, //8x8
    1, //16x16
    0, //32x32
    10, //64x64
#if COM16_C806_LARGE_CTU
    0, //128x128
#if !JVET_C0024_QTBT
    0, //256x256
#endif
#endif
#endif
  },
  { // Chroma
#if JVET_C0024_QTBT
    20, //2x2
#endif
#if VCEG_AZ07_INTRA_65ANG_MODES
    20, //4xn
    14, //8xn
    2,  //16xn
    0,  //32xn
    20, //64xn
#if COM16_C806_LARGE_CTU
    0, //128xn
#if !JVET_C0024_QTBT
    0, //256xn
#endif
#endif
#else
    10, //4xn
    7, //8xn
    1, //16xn
    0, //32xn
    10, //64xn
#if COM16_C806_LARGE_CTU
    0, //128x128
#if !JVET_C0024_QTBT
    0, //256x256
#endif
#endif
#endif
  }
};

// ====================================================================================================================
// Constructor / destructor / initialize
// ====================================================================================================================

TComPrediction::TComPrediction()
: m_pLumaRecBuffer(0)
, m_iLumaRecStride(0)
{
#if VCEG_AZ05_BIO 
#if JVET_F0028_BIO_NO_BLOCK_EXTENTION // no block extension is needed
#define BIO_TEMP_BUFFER_SIZE      (MAX_CU_SIZE)*(MAX_CU_SIZE)
#else
#define BIO_TEMP_BUFFER_SIZE      (MAX_CU_SIZE+4)*(MAX_CU_SIZE+4) 
#endif
  m_pGradX0 = new Pel [BIO_TEMP_BUFFER_SIZE];
  m_pGradY0 = new Pel [BIO_TEMP_BUFFER_SIZE];
  m_pGradX1 = new Pel [BIO_TEMP_BUFFER_SIZE];
  m_pGradY1 = new Pel [BIO_TEMP_BUFFER_SIZE];
#if !JVET_F0028_BIO_NO_BLOCK_EXTENTION
  m_pPred0  = new Pel [BIO_TEMP_BUFFER_SIZE];
  m_pPred1  = new Pel [BIO_TEMP_BUFFER_SIZE];
#endif
  iRefListIdx = -1;  
#endif
#if COM16_C1046_PDPC_INTRA
  piTempRef = new Int[4 * MAX_CU_SIZE + 1];
  piFiltRef = new Int[4 * MAX_CU_SIZE + 1];
  piBinBuff = new Int[4 * MAX_CU_SIZE + 9];
#endif

#if COM16_C806_VCEG_AZ10_SUB_PU_TMVP
  m_puiW = NULL;
  m_puiH = NULL;
  m_puiSPAddr = NULL;
#endif
#if VCEG_AZ07_FRUC_MERGE && COM16_C806_VCEG_AZ10_SUB_PU_TMVP
  memset( m_cMvFieldSP , 0 , sizeof( m_cMvFieldSP ) );
  memset( m_uhInterDirSP , 0 , sizeof( m_uhInterDirSP ) );
#endif
  for(UInt ch=0; ch<MAX_NUM_COMPONENT; ch++)
  {
    for(UInt buf=0; buf<2; buf++)
    {
      m_piYuvExt[ch][buf] = NULL;
    }
  }
#if VCEG_AZ07_FRUC_MERGE
  m_cFRUCRDCost.init();
#endif

#if VCEG_AZ08_INTER_KLT
  m_tempPicYuv = NULL;
#endif

#if JVET_E0077_LM_MF
  for (Int i = 0; i < LM_FILTER_NUM; i++)
  {
      m_pLumaRecBufferMul[i] = 0;
  }
#endif



}

TComPrediction::~TComPrediction()
{
  destroy();
}

Void TComPrediction::destroy()
{
#if VCEG_AZ05_BIO 
  if( m_pGradX0 != NULL )     {delete [] m_pGradX0 ; m_pGradX0= NULL;}
  if( m_pGradY0 != NULL )     {delete [] m_pGradY0 ; m_pGradY0= NULL;}
  if( m_pGradX1 != NULL )     {delete [] m_pGradX1 ; m_pGradX1= NULL;}
  if( m_pGradY1 != NULL )     {delete [] m_pGradY1 ; m_pGradY1= NULL;}
#if !JVET_F0028_BIO_NO_BLOCK_EXTENTION
  if( m_pPred0  != NULL )     {delete [] m_pPred0  ; m_pPred0 = NULL;}
  if( m_pPred1  != NULL )     {delete [] m_pPred1  ; m_pPred1 = NULL;}
#endif
#endif

#if COM16_C1046_PDPC_INTRA
  if (piTempRef != NULL)    { delete[] piTempRef;  piTempRef = NULL; }
  if (piFiltRef != NULL)    { delete[] piFiltRef;  piFiltRef = NULL; }
  if (piBinBuff != NULL)    { delete[] piBinBuff;  piBinBuff = NULL; }
#endif

#if COM16_C806_VCEG_AZ10_SUB_PU_TMVP
  if( m_puiW != NULL )
  {
    delete [] m_puiW;
    m_puiW = NULL;
  }
  if( m_puiH != NULL )
  {
    delete [] m_puiH;
    m_puiH = NULL;
  }
  if( m_puiSPAddr != NULL )
  {
    delete [] m_puiSPAddr;
    m_puiSPAddr = NULL;
  }
#endif
  for(UInt ch=0; ch<MAX_NUM_COMPONENT; ch++)
  {
    for(UInt buf=0; buf<NUM_PRED_BUF; buf++)
    {
      delete [] m_piYuvExt[ch][buf];
      m_piYuvExt[ch][buf] = NULL;
    }
  }

  for(UInt i=0; i<NUM_REF_PIC_LIST_01; i++)
  {
    m_acYuvPred[i].destroy();
  }

  m_cYuvPredTemp.destroy();

  if( m_pLumaRecBuffer )
  {
    delete [] m_pLumaRecBuffer;
    m_pLumaRecBuffer = 0;
  }
  m_iLumaRecStride = 0;

#if JVET_E0077_LM_MF
  for (Int i = 0; i < LM_FILTER_NUM; i++)
  {
      if (m_pLumaRecBufferMul[i])
      {
          delete[]m_pLumaRecBufferMul[i];
          m_pLumaRecBufferMul[i] = 0;
      }
  }
#endif



  for (UInt i = 0; i < LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS; i++)
  {
    for (UInt j = 0; j < LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS; j++)
    {
      m_filteredBlock[i][j].destroy();
    }
    m_filteredBlockTmp[i].destroy();
  }

#if VCEG_AZ07_FRUC_MERGE
  m_cYuvPredFrucTemplate[0].destroy();
  m_cYuvPredFrucTemplate[1].destroy();
#if COM16_C806_VCEG_AZ10_SUB_PU_TMVP
#if JVET_C0035_ATMVP_SIMPLIFICATION
  for (UInt ui=0;ui<NUM_MGR_TYPE;ui++)
#else
  for (UInt ui=0;ui<2;ui++)
#endif
  {
    if( m_cMvFieldSP[ui] != NULL )
    {
      delete [] m_cMvFieldSP[ui];
      m_cMvFieldSP[ui] = NULL;
    }
    if( m_uhInterDirSP[ui] != NULL )
    {
      delete [] m_uhInterDirSP[ui];
      m_uhInterDirSP[ui] = NULL;
    }
  }
#endif
#endif

#if VCEG_AZ08_INTER_KLT
  if( m_tempPicYuv != NULL )
  {
    m_tempPicYuv->destroy();
    delete m_tempPicYuv;
    m_tempPicYuv = NULL;
  }
#endif
}

#if COM16_C806_LMCHROMA
Void TComPrediction::initTempBuff(ChromaFormat chromaFormatIDC, Int bitDepthY
#if VCEG_AZ08_INTER_KLT
  , bool interKLT , const Int iPicWidth, const Int iPicHeight, const UInt uiMaxCUWidth, const UInt uiMaxCUHeight, const UInt uiMaxCUDepth
#endif
  )
#else
Void TComPrediction::initTempBuff(ChromaFormat chromaFormatIDC
#if VCEG_AZ08_INTER_KLT
  , bool interKLT , const Int iPicWidth, const Int iPicHeight, const UInt uiMaxCUWidth, const UInt uiMaxCUHeight, const UInt uiMaxCUDepth
#endif
  )
#endif
{
  // if it has been initialised before, but the chroma format has changed, release the memory and start again.
  if( m_piYuvExt[COMPONENT_Y][PRED_BUF_UNFILTERED] != NULL && m_cYuvPredTemp.getChromaFormat()!=chromaFormatIDC)
  {
    destroy();
  }

  if( m_piYuvExt[COMPONENT_Y][PRED_BUF_UNFILTERED] == NULL ) // check if first is null (in which case, nothing initialised yet)
  {
    Int extWidth  = MAX_CU_SIZE + 16;
    Int extHeight = MAX_CU_SIZE + 1;

    for (UInt i = 0; i < LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS; i++)
    {
#if VCEG_AZ05_BIO
      m_filteredBlockTmp[i].create(extWidth+4, extHeight + 7+4, chromaFormatIDC);
#else
      m_filteredBlockTmp[i].create(extWidth, extHeight + 7, chromaFormatIDC);
#endif
      for (UInt j = 0; j < LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS; j++)
      {
        m_filteredBlock[i][j].create(extWidth, extHeight, chromaFormatIDC);
      }
    }

    m_iYuvExtSize = (MAX_CU_SIZE*2+1) * (MAX_CU_SIZE*2+1);
    for(UInt ch=0; ch<MAX_NUM_COMPONENT; ch++)
    {
      for(UInt buf=0; buf<NUM_PRED_BUF; buf++)
      {
        m_piYuvExt[ch][buf] = new Pel[ m_iYuvExtSize ];
      }
    }

    // new structure
    for(UInt i=0; i<NUM_REF_PIC_LIST_01; i++)
    {
#if VCEG_AZ05_BIO
      m_acYuvPred[i] .create( MAX_CU_SIZE+4, MAX_CU_SIZE+4, chromaFormatIDC );
#else
      m_acYuvPred[i] .create( MAX_CU_SIZE, MAX_CU_SIZE, chromaFormatIDC );
#endif
    }

#if JVET_E0052_DMVR
    m_cYuvPredTemp.create( MAX_CU_SIZE+ DMVR_INTME_RANGE*2, MAX_CU_SIZE+ DMVR_INTME_RANGE*2, chromaFormatIDC );
#else
    m_cYuvPredTemp.create( MAX_CU_SIZE, MAX_CU_SIZE, chromaFormatIDC );
#endif
#if VCEG_AZ07_FRUC_MERGE
    m_cYuvPredFrucTemplate[0].create( MAX_CU_SIZE, MAX_CU_SIZE, chromaFormatIDC );
    m_cYuvPredFrucTemplate[1].create( MAX_CU_SIZE, MAX_CU_SIZE, chromaFormatIDC );
#endif
  }

#if JVET_E0077_MMLM
  m_iLumaRecStride = (MAX_CU_SIZE >> 1) + MMLM_SAMPLE_NEIGHBOR_LINES;
  if (!m_pLumaRecBuffer)
  {
      m_pLumaRecBuffer = new Pel[m_iLumaRecStride * m_iLumaRecStride];
  }
#else
  if (m_iLumaRecStride != (MAX_CU_SIZE>>1) + 1)
  {
    m_iLumaRecStride =  (MAX_CU_SIZE>>1) + 1;
    if (!m_pLumaRecBuffer)
    {
      m_pLumaRecBuffer = new Pel[ m_iLumaRecStride * m_iLumaRecStride ];
    }
  }
#endif

#if JVET_E0077_LM_MF
  for (Int i = 0; i < LM_FILTER_NUM; i++)
  {
      if (!m_pLumaRecBufferMul[i])
      {
          m_pLumaRecBufferMul[i] = new Pel[m_iLumaRecStride * m_iLumaRecStride];
      }
  }
#endif

#if COM16_C806_LMCHROMA
  Int shift = bitDepthY + 4;   
  for( Int i = 32; i < 64; i++ )
  {
    m_uiaLMShift[i-32] = ( ( 1 << shift ) + i/2 ) / i;
  }
#endif

#if COM16_C806_VCEG_AZ10_SUB_PU_TMVP
  if( m_puiSPAddr == NULL )
  {
    m_puiW = new UInt[MAX_NUM_PART_IDXS_IN_CTU_WIDTH*MAX_NUM_PART_IDXS_IN_CTU_WIDTH];
    m_puiH = new UInt[MAX_NUM_PART_IDXS_IN_CTU_WIDTH*MAX_NUM_PART_IDXS_IN_CTU_WIDTH];
    m_puiSPAddr = new UInt[MAX_NUM_PART_IDXS_IN_CTU_WIDTH*MAX_NUM_PART_IDXS_IN_CTU_WIDTH];
  }
#endif

#if VCEG_AZ07_FRUC_MERGE && COM16_C806_VCEG_AZ10_SUB_PU_TMVP
  if( m_cMvFieldSP[0] == NULL )
  {
#if JVET_C0035_ATMVP_SIMPLIFICATION
    for (Int i=0; i< NUM_MGR_TYPE; i++)
    {
      m_cMvFieldSP[i] = new TComMvField[MAX_NUM_PART_IDXS_IN_CTU_WIDTH*MAX_NUM_PART_IDXS_IN_CTU_WIDTH*2];
      m_uhInterDirSP[i] = new UChar[MAX_NUM_PART_IDXS_IN_CTU_WIDTH*MAX_NUM_PART_IDXS_IN_CTU_WIDTH];
    }
#else
    m_cMvFieldSP[0] = new TComMvField[MAX_NUM_PART_IDXS_IN_CTU_WIDTH*MAX_NUM_PART_IDXS_IN_CTU_WIDTH*2];
    m_cMvFieldSP[1] = new TComMvField[MAX_NUM_PART_IDXS_IN_CTU_WIDTH*MAX_NUM_PART_IDXS_IN_CTU_WIDTH*2];
    m_uhInterDirSP[0] = new UChar[MAX_NUM_PART_IDXS_IN_CTU_WIDTH*MAX_NUM_PART_IDXS_IN_CTU_WIDTH];
    m_uhInterDirSP[1] = new UChar[MAX_NUM_PART_IDXS_IN_CTU_WIDTH*MAX_NUM_PART_IDXS_IN_CTU_WIDTH];
#endif
  }
#endif

#if VCEG_AZ06_IC
  m_uiaICShift[0] = 0;
  for( Int i = 1; i < 64; i++ )
  {
    m_uiaICShift[i] = ( (1 << 15) + i/2 ) / i;
  }
#endif

#if JVET_G0082
  m_uiaBIOShift[0] = 0;
  for (Int i = 1; i < 64; i++)
  {
    m_uiaBIOShift[i] = ((1 << 15) + i / 2) / i;
  }
#endif

#if VCEG_AZ08_INTER_KLT
  if( interKLT )
  {
    if( m_tempPicYuv != NULL )
    {
      m_tempPicYuv->destroy();
      delete m_tempPicYuv;
    }
    m_tempPicYuv = new TComPicYuv;
    m_tempPicYuv->create( iPicWidth , iPicHeight , chromaFormatIDC , uiMaxCUWidth , uiMaxCUHeight , uiMaxCUDepth , true );
  }
#endif
}

// ====================================================================================================================
// Public member functions
// ====================================================================================================================

// Function for calculating DC value of the reference samples used in Intra prediction
//NOTE: Bit-Limit - 25-bit source
Pel TComPrediction::predIntraGetPredValDC( const Pel* pSrc, Int iSrcStride, UInt iWidth, UInt iHeight)
{
  assert(iWidth > 0 && iHeight > 0);
  Int iInd, iSum = 0;
  Pel pDcVal;

  for (iInd = 0;iInd < iWidth;iInd++)
  {
    iSum += pSrc[iInd-iSrcStride];
  }
  for (iInd = 0;iInd < iHeight;iInd++)
  {
    iSum += pSrc[iInd*iSrcStride-1];
  }

#if JVET_C0024_QTBT
  pDcVal = (iSum + ((iWidth+iHeight)>>1)) / (iWidth + iHeight);
#else
  pDcVal = (iSum + iWidth) / (iWidth + iHeight);
#endif

  return pDcVal;
}

// Function for deriving the angular Intra predictions

/** Function for deriving the simplified angular intra predictions.
 * \param bitDepth           bit depth
 * \param pSrc               pointer to reconstructed sample array
 * \param srcStride          the stride of the reconstructed sample array
 * \param pTrueDst           reference to pointer for the prediction sample array
 * \param dstStrideTrue      the stride of the prediction sample array
 * \param uiWidth            the width of the block
 * \param uiHeight           the height of the block
 * \param channelType        type of pel array (luma/chroma)
 * \param format             chroma format
 * \param dirMode            the intra prediction mode index
 * \param blkAboveAvailable  boolean indication if the block above is available
 * \param blkLeftAvailable   boolean indication if the block to the left is available
 * \param bEnableEdgeFilters indication whether to enable edge filters
 *
 * This function derives the prediction samples for the angular mode based on the prediction direction indicated by
 * the prediction mode index. The prediction direction is given by the displacement of the bottom row of the block and
 * the reference row above the block in the case of vertical prediction or displacement of the rightmost column
 * of the block and reference column left from the block in the case of the horizontal prediction. The displacement
 * is signalled at 1/32 pixel accuracy. When projection of the predicted pixel falls inbetween reference samples,
 * the predicted value for the pixel is linearly interpolated from the reference samples. All reference samples are taken
 * from the extended main reference.
 */
//NOTE: Bit-Limit - 25-bit source
Void TComPrediction::xPredIntraAng(       Int bitDepth,
                                    const Pel* pSrc,     Int srcStride,
                                          Pel* pTrueDst, Int dstStrideTrue,
                                          UInt uiWidth, UInt uiHeight,
#if JVET_D0033_ADAPTIVE_CLIPPING
                                          ComponentID compID,
#else
                                          ChannelType channelType,
#endif
                                          UInt dirMode, const Bool bEnableEdgeFilters
#if VCEG_AZ07_INTRA_4TAP_FILTER
                                          , Bool enable4TapFilter
#endif
#if COM16_C983_RSAF_PREVENT_OVERSMOOTHING
                                          , Bool enableRSAF
#endif
                                          )
{
  Int width=Int(uiWidth);
  Int height=Int(uiHeight);

#if JVET_D0033_ADAPTIVE_CLIPPING
    const ChannelType channelType=toChannelType(compID);
#endif
  // Map the mode index to main prediction direction and angle
  assert( dirMode != PLANAR_IDX ); //no planar
  const Bool modeDC        = dirMode==DC_IDX;

  // Do the DC prediction
  if (modeDC)
  {
    const Pel dcval = predIntraGetPredValDC(pSrc, srcStride, width, height);

    for (Int y=height;y>0;y--, pTrueDst+=dstStrideTrue)
    {
      for (Int x=0; x<width;) // width is always a multiple of 4.
      {
        pTrueDst[x++] = dcval;
      }
    }
  }
  else // Do angular predictions
  {
#if VCEG_AZ07_INTRA_65ANG_MODES
    const Bool       bIsModeVer         = (dirMode >= DIA_IDX);
#else
    const Bool       bIsModeVer         = (dirMode >= 18);
#endif
    const Int        intraPredAngleMode = (bIsModeVer) ? (Int)dirMode - VER_IDX :  -((Int)dirMode - HOR_IDX);
    const Int        absAngMode         = abs(intraPredAngleMode);
    const Int        signAng            = intraPredAngleMode < 0 ? -1 : 1;
    const Bool       edgeFilter         = bEnableEdgeFilters && isLuma(channelType) && (width <= MAXIMUM_INTRA_FILTERED_WIDTH) && (height <= MAXIMUM_INTRA_FILTERED_HEIGHT);

    // Set bitshifts and scale the angle parameter to block size
#if VCEG_AZ07_INTRA_65ANG_MODES
    static const Int angTable[17]    = {0,    1,    2,    3,    5,    7,    9,   11,   13,   15,   17,   19,   21,   23,   26,   29,   32};
    static const Int invAngTable[17] = {0, 8192, 4096, 2731, 1638, 1170,  910,  745,  630,  546,  482,  431,  390,  356,  315,  282,  256}; // (256 * 32) / Angle
#else
    static const Int angTable[9]    = {0,    2,    5,   9,  13,  17,  21,  26,  32};
    static const Int invAngTable[9] = {0, 4096, 1638, 910, 630, 482, 390, 315, 256}; // (256 * 32) / Angle
#endif
    Int invAngle                    = invAngTable[absAngMode];
    Int absAng                      = angTable[absAngMode];
    Int intraPredAngle              = signAng * absAng;

    Pel* refMain;
    Pel* refSide;

    Pel  refAbove[2*MAX_CU_SIZE+1];
    Pel  refLeft[2*MAX_CU_SIZE+1];

    // Initialize the Main and Left reference array.
    if (intraPredAngle < 0)
    {
      const Int refMainOffsetPreScale = (bIsModeVer ? height : width ) - 1;
#if !JVET_C0024_QTBT
      const Int refMainOffset         = height - 1;
#endif
      for (Int x=0;x<width+1;x++)
      {
#if JVET_C0024_QTBT
        refAbove[x+height-1] = pSrc[x-srcStride-1];
#else
        refAbove[x+refMainOffset] = pSrc[x-srcStride-1];
#endif
      }
      for (Int y=0;y<height+1;y++)
      {
#if JVET_C0024_QTBT
        refLeft[y+width-1] = pSrc[(y-1)*srcStride-1];
#else
        refLeft[y+refMainOffset] = pSrc[(y-1)*srcStride-1];
#endif
      }
#if JVET_C0024_QTBT
      refMain = (bIsModeVer ? refAbove + height : refLeft + width)  - 1;
      refSide = (bIsModeVer ? refLeft + width  : refAbove + height) - 1;
#else
      refMain = (bIsModeVer ? refAbove : refLeft)  + refMainOffset;
      refSide = (bIsModeVer ? refLeft  : refAbove) + refMainOffset;
#endif

      // Extend the Main reference to the left.
      Int invAngleSum    = 128;       // rounding for (shift by 8)
      for (Int k=-1; k>(refMainOffsetPreScale+1)*intraPredAngle>>5; k--)
      {
        invAngleSum += invAngle;
        refMain[k] = refSide[invAngleSum>>8];
      }
    }
    else
    {
#if JVET_C0024_QTBT
      for (Int x=0;x<width+height+1;x++)
#else
      for (Int x=0;x<2*width+1;x++)
#endif
      {
        refAbove[x] = pSrc[x-srcStride-1];
#if JVET_C0024_QTBT
        refLeft[x] = pSrc[(x-1)*srcStride-1];
#endif
      }
#if !JVET_C0024_QTBT
      for (Int y=0;y<2*height+1;y++)
      {
        refLeft[y] = pSrc[(y-1)*srcStride-1];
      }
#endif
      refMain = bIsModeVer ? refAbove : refLeft ;
      refSide = bIsModeVer ? refLeft  : refAbove;
    }

    // swap width/height if we are doing a horizontal mode:
    Pel tempArray[MAX_CU_SIZE*MAX_CU_SIZE];
    const Int dstStride = bIsModeVer ? dstStrideTrue : MAX_CU_SIZE;
    Pel *pDst = bIsModeVer ? pTrueDst : tempArray;
    if (!bIsModeVer)
    {
      std::swap(width, height);
    }

    if (intraPredAngle == 0)  // pure vertical or pure horizontal
    {
      for (Int y=0;y<height;y++)
      {
        for (Int x=0;x<width;x++)
        {
          pDst[y*dstStride+x] = refMain[x+1];
        }
      }

      if (edgeFilter)
      {
        for (Int y=0;y<height;y++)
        {
#if JVET_D0033_ADAPTIVE_CLIPPING
            pDst[y*dstStride] = ClipA (pDst[y*dstStride] + (( refSide[y+1] - refSide[0] ) >> 1) ,compID);
#else
          pDst[y*dstStride] = Clip3 (0, ((1 << bitDepth) - 1), pDst[y*dstStride] + (( refSide[y+1] - refSide[0] ) >> 1) );
#endif
        }
      }
    }
    else
    {
      Pel *pDsty=pDst;

      for (Int y=0, deltaPos=intraPredAngle; y<height; y++, deltaPos+=intraPredAngle, pDsty+=dstStride)
      {
        const Int deltaInt   = deltaPos >> 5;
        const Int deltaFract = deltaPos & (32 - 1);

        if (deltaFract)
        {
          // Do linear filtering
#if VCEG_AZ07_INTRA_4TAP_FILTER
          if (enable4TapFilter)
          {
            Int p[4], x, refMainIndex;
#if !JVET_D0033_ADAPTIVE_CLIPPING
            const Pel nMin = 0, nMax = (1 << bitDepth) - 1;
#endif
#if COM16_C983_RSAF_PREVENT_OVERSMOOTHING
            Int *f =  ((channelType==CHANNEL_TYPE_LUMA) && enableRSAF) ? g_aiIntraCubicFilter[deltaFract] : ( (width<=8) ? g_aiIntraCubicFilter[deltaFract] : g_aiIntraGaussFilter[deltaFract] );
#else
            Int *f = (width<=8) ? g_aiIntraCubicFilter[deltaFract] : g_aiIntraGaussFilter[deltaFract];
#endif

            
            for (x=0;x<width;x++)
            {
              refMainIndex = x+deltaInt+1;

              p[1] = refMain[refMainIndex];
              p[2] = refMain[refMainIndex+1];

              p[0] = x==0 ? p[1] : refMain[refMainIndex-1];
              p[3] = x==(width-1) ? p[2] : refMain[refMainIndex+2];

              pDst[y*dstStride+x] =  (Pel)( ( f[0]*p[0] + f[1]*p[1] + f[2]*p[2] + f[3]*p[3] + 128 ) >> 8 );

#if COM16_C983_RSAF_PREVENT_OVERSMOOTHING
              if (enableRSAF || width<=8)
#else
              if( width<=8 ) // for blocks larger than 8x8, Gaussian interpolation filter with positive coefficients is used, no Clipping is necessary
#endif
              {
#if JVET_D0033_ADAPTIVE_CLIPPING
                  pDst[y*dstStride+x] =  ClipA( pDst[y*dstStride+x],compID );
#else
                pDst[y*dstStride+x] =  Clip3( nMin, nMax, pDst[y*dstStride+x] );
#endif
              }
            }
          }
          else
          {
#endif
          const Pel *pRM=refMain+deltaInt+1;
          Int lastRefMainPel=*pRM++;
          for (Int x=0;x<width;pRM++,x++)
          {
            Int thisRefMainPel=*pRM;
            pDsty[x+0] = (Pel) ( ((32-deltaFract)*lastRefMainPel + deltaFract*thisRefMainPel +16) >> 5 );
            lastRefMainPel=thisRefMainPel;
          }
#if VCEG_AZ07_INTRA_4TAP_FILTER
          }
#endif
        }
        else
        {
          // Just copy the integer samples
          for (Int x=0;x<width; x++)
          {
            pDsty[x] = refMain[x+deltaInt+1];
          }
        }
      }
#if VCEG_AZ07_INTRA_65ANG_MODES
      if ( edgeFilter && absAng<=1 )
      {
        for (Int y=0;y<height;y++)
        {
#if JVET_D0033_ADAPTIVE_CLIPPING
            pDst[y*dstStride] = ClipA(pDst[y*dstStride] + (( refSide[y+1] - refSide[0] ) >> 2) ,compID);
#else
          pDst[y*dstStride] = Clip3(0, (1<<bitDepth)-1, pDst[y*dstStride] + (( refSide[y+1] - refSide[0] ) >> 2) );
#endif
        }
      }
#endif
    }

    // Flip the block if this is the horizontal mode
    if (!bIsModeVer)
    {
      for (Int y=0; y<height; y++)
      {
        for (Int x=0; x<width; x++)
        {
          pTrueDst[x*dstStrideTrue] = pDst[x];
        }
        pTrueDst++;
        pDst+=dstStride;
      }
    }
  }
}

Void TComPrediction::predIntraAng( const ComponentID compID, UInt uiDirMode, Pel* piOrg /* Will be null for decoding */, UInt uiOrgStride, Pel* piPred, UInt uiStride, TComTU &rTu, const Bool bUseFilteredPredSamples, const Bool bUseLosslessDPCM )
{
  const ChannelType    channelType = toChannelType(compID);
  const TComRectangle &rect        = rTu.getRect(isLuma(compID) ? COMPONENT_Y : COMPONENT_Cb);
  const Int            iWidth      = rect.width;
  const Int            iHeight     = rect.height;

#if JVET_C0024_QTBT
  assert( g_aucConvertToBit[ iWidth ] >= -1 );  //2x2
  assert( g_aucConvertToBit[ iWidth ] <= MAX_CU_DEPTH - MIN_CU_LOG2 ); 
#else
  assert( g_aucConvertToBit[ iWidth ] >= 0 ); //   4x  4
#if COM16_C806_LARGE_CTU
  assert( g_aucConvertToBit[ iWidth ] <= MAX_CU_DEPTH - 2 ); 
#else
  assert( g_aucConvertToBit[ iWidth ] <= 5 ); // 128x128
#endif
  //assert( iWidth == iHeight  );
#endif

        Pel *pDst = piPred;

  // get starting pixel in block
#if JVET_C0024_QTBT
  const Int sw = (iHeight + iWidth + 1);
#else
  const Int sw = (2 * iWidth + 1);
#endif

#if COM16_C1046_PDPC_INTRA
  TComDataCU *const pcCU = rTu.getCU();
  const UInt uiAbsPartIdx = rTu.GetAbsPartIdxTU();
#endif

  if ( bUseLosslessDPCM )
  {
    const Pel *ptrSrc = getPredictorPtr( compID, false );
    // Sample Adaptive intra-Prediction (SAP)
    if (uiDirMode==HOR_IDX)
    {
      // left column filled with reference samples
      // remaining columns filled with piOrg data (if available).
      for(Int y=0; y<iHeight; y++)
      {
        piPred[y*uiStride+0] = ptrSrc[(y+1)*sw];
      }
      if (piOrg!=0)
      {
        piPred+=1; // miss off first column
        for(Int y=0; y<iHeight; y++, piPred+=uiStride, piOrg+=uiOrgStride)
        {
          memcpy(piPred, piOrg, (iWidth-1)*sizeof(Pel));
        }
      }
    }
    else // VER_IDX
    {
      // top row filled with reference samples
      // remaining rows filled with piOrd data (if available)
      for(Int x=0; x<iWidth; x++)
      {
        piPred[x] = ptrSrc[x+1];
      }
      if (piOrg!=0)
      {
        piPred+=uiStride; // miss off the first row
        for(Int y=1; y<iHeight; y++, piPred+=uiStride, piOrg+=uiOrgStride)
        {
          memcpy(piPred, piOrg, iWidth*sizeof(Pel));
        }
      }
    }
  }
  else
  {
#if COM16_C1046_PDPC_INTRA
#if !COM16_C1046_PDPC_RSAF_HARMONIZATION
    Pel *ptrSrc = getPredictorPtr(compID, false);
#endif
#if JVET_G0104_PLANAR_PDPC
    if( uiDirMode == PLANAR_IDX )
#else
#if JVET_C0024_QTBT //different PDPC filter coeff between sizes, w!=h? JCA
    Int iBlkSizeGrp = std::min(4, 1 + std::max((Int)g_aucConvertToBit[iWidth], (Int) g_aucConvertToBit[iHeight]));
    Int blkSizeGroup[2] = { std::min(4, 1 + (Int)g_aucConvertToBit[iWidth]), std::min(4, 1 + (Int)g_aucConvertToBit[iHeight]) };
#else
    Int iBlkSizeGrp = std::min(4, 1 + (Int)g_aucConvertToBit[iWidth]); //Block Size
#endif
    
#if JVET_C0024_QTBT
    Int iPdpcIdx = pcCU->getPDPCIdx(uiAbsPartIdx);

    if( isChroma(pcCU->getTextType()) )
    {
      UInt absPartIdx = pcCU->getZorderIdxInCtu() + uiAbsPartIdx;
      absPartIdx = g_auiRasterToZscan[ g_auiZscanToRaster[absPartIdx] + ( pcCU->getHeight(uiAbsPartIdx)/pcCU->getPic()->getMinCUHeight() )/2*pcCU->getPic()->getNumPartInCtuWidth() + ( pcCU->getWidth(uiAbsPartIdx)/pcCU->getPic()->getMinCUWidth() )/2];
      iPdpcIdx = pcCU->getPic()->getCtu(pcCU->getCtuRsAddr())->isIntra(absPartIdx) ? pcCU->getPic()->getCtu(pcCU->getCtuRsAddr())->getPDPCIdx(absPartIdx) : 0;
    }

    if( iPdpcIdx && pcCU->getSlice()->getSPS()->getUsePDPC() )
#else
    Int iPdpcIdx = 0; //PDPC Idx
    
    if (pcCU->getPDPCIdx(uiAbsPartIdx) && pcCU->getCUPelX() && pcCU->getCUPelY() && pcCU->getSlice()->getSPS()->getUsePDPC())
#endif
    {
#if !JVET_C0024_QTBT
      PartSize eSize = pcCU->getPartitionSize(uiAbsPartIdx);
      iPdpcIdx = pcCU->getPDPCIdx(uiAbsPartIdx);
#endif

      if (iPdpcIdx > 3) iPdpcIdx = 0;
#if JVET_C0024_QTBT
      if (iBlkSizeGrp==1) iBlkSizeGrp = 0;  
#else
      if ((eSize == SIZE_NxN) && (iBlkSizeGrp == 1)) iBlkSizeGrp = 0;
#endif
    }
#if JVET_C0024_QTBT
    else
    {
      iPdpcIdx = 0;
    }
#endif

    //pdpc applied
    if (iPdpcIdx != 0) 
#endif
    {
#if COM16_C1046_PDPC_RSAF_HARMONIZATION 
      Pel *ptrSrc = getPredictorPtr(compID, false);
#endif
#if JVET_C0024_QTBT
      const Int iSrcStride = iWidth + iHeight + 1;
      const Int iDoubleSize = iWidth + iHeight;
#else
      const Int iBlkSize = iWidth;
      const Int iSrcStride = (iWidth<<1) + 1;
      const Int iDoubleWidth = iWidth<<1;
#endif

#if JVET_G0104_PLANAR_PDPC
      const Int blkSizeGroup[2] = { std::min( 4, 1 + (Int)g_aucConvertToBit[iWidth] ), std::min( 4, 1 + (Int)g_aucConvertToBit[iHeight] ) };
      const Short *pdpcParam[2] = { g_pdpcParam[blkSizeGroup[0]], g_pdpcParam[blkSizeGroup[1]] };
      const Short *pPdpcPar = pdpcParam[iWidth < iHeight];
#else
#if VCEG_AZ07_INTRA_65ANG_MODES
      Int   iSelMode = (uiDirMode > 1 ? 18 + ((Int(uiDirMode) - 34)>>1) : uiDirMode);
#if JVET_C0024_QTBT
      const Int *pdpcParam[2] = { g_pdpc_pred_param[blkSizeGroup[0]][iSelMode], g_pdpc_pred_param[blkSizeGroup[1]][iSelMode] };
      const Int *pPdpcPar = pdpcParam[iWidth < iHeight];
#else
      const Int * pPdpcPar = g_pdpc_pred_param[iBlkSizeGrp][iPdpcIdx][iSelMode];
#endif
#else
#if JVET_C0024_QTBT
      const Int *pdpcParam[2] = { g_pdpc_pred_param[blkSizeGroup[0]][uiDirMode], g_pdpc_pred_param[blkSizeGroup[1]][uiDirMode] };
      const Int *pPdpcPar = pdpcParam[iWidth < iHeight];
#else
      const Int * pPdpcPar = g_pdpc_pred_param[iBlkSizeGrp][iPdpcIdx][uiDirMode];
#endif
#endif
#endif

#if JVET_C0024_QTBT
      Int * piRefVector = piTempRef + iDoubleSize;
      Int * piLowpRefer = piFiltRef + iDoubleSize;
#else
      Int * piRefVector = piTempRef + iDoubleWidth;
      Int * piLowpRefer = piFiltRef + iDoubleWidth;
#endif

      //unfiltered reference
#if JVET_C0024_QTBT
      for (Int j = 0; j <= iDoubleSize; j++)
        piRefVector[j] = ptrSrc[j];

      for (Int i = 1; i <= iDoubleSize; i++)
        piRefVector[-i] = ptrSrc[i*iSrcStride];
#else
      for (Int j = 0; j <= iDoubleWidth; j++)
        piRefVector[j] = ptrSrc[j];

      for (Int i = 1; i <= iDoubleWidth; i++)
        piRefVector[-i] = ptrSrc[i*iSrcStride];
#endif


      if (pPdpcPar[5] != 0) 
      { // filter reference samples
#if JVET_C0024_QTBT
        xReferenceFilter(iDoubleSize, pPdpcPar[4], pPdpcPar[5], piRefVector, piLowpRefer);
        for (Int j = 0; j <= iDoubleSize; j++)
          ptrSrc[j] = piLowpRefer[j];
        for (Int i = 1; i <= iDoubleSize; i++)
          ptrSrc[i*iSrcStride] = piLowpRefer[-i];
#else
        xReferenceFilter(iBlkSize, pPdpcPar[4], pPdpcPar[5], piRefVector, piLowpRefer);
        for (Int j = 0; j <= iDoubleWidth; j++)
          ptrSrc[j] = piLowpRefer[j];
        for (Int i = 1; i <= iDoubleWidth; i++)
          ptrSrc[i*iSrcStride] = piLowpRefer[-i];
#endif
      }


      if (uiDirMode == PLANAR_IDX)
        xPredIntraPlanar(ptrSrc + sw + 1, sw, pDst, uiStride, iWidth, iHeight);
      else
      {
        const Bool            enableEdgeFilters = !(pcCU->isRDPCMEnabled(uiAbsPartIdx) && pcCU->getCUTransquantBypass(uiAbsPartIdx));
#if O0043_BEST_EFFORT_DECODING
        const Int channelsBitDepthForPrediction = rTu.getCU()->getSlice()->getSPS()->getStreamBitDepth(channelType);
#else
        const Int channelsBitDepthForPrediction = rTu.getCU()->getSlice()->getSPS()->getBitDepth(channelType);
#endif
#if VCEG_AZ07_INTRA_4TAP_FILTER
        const Bool             enable4TapFilter = pcCU->getSlice()->getSPS()->getUseIntra4TapFilter();
#endif

                xPredIntraAng(channelsBitDepthForPrediction, ptrSrc + sw + 1, sw, pDst, uiStride,
                              iWidth, iHeight,
#if JVET_D0033_ADAPTIVE_CLIPPING
                              compID,
              #else
                              channelType,
              #endif
                              uiDirMode, enableEdgeFilters
#if VCEG_AZ07_INTRA_4TAP_FILTER
          , enable4TapFilter
#endif
          );
      }

      //use unfiltered reference sample for weighted prediction
      if (pPdpcPar[5] != 0) 
      {
#if JVET_C0024_QTBT
        for (int j = 0; j <= iDoubleSize; j++)
          ptrSrc[j] = piRefVector[j];

        for (int i = 1; i <= iDoubleSize; i++)
          ptrSrc[i*iSrcStride] = piRefVector[-i];
#else
        for (int j = 0; j <= iDoubleWidth; j++)
          ptrSrc[j] = piRefVector[j];

        for (int i = 1; i <= iDoubleWidth; i++)
          ptrSrc[i*iSrcStride] = piRefVector[-i];
#endif
      }

#if JVET_C0024_QTBT
      Int scale = g_aucConvertToBit[iWidth] + MIN_CU_LOG2 + g_aucConvertToBit[iHeight] + MIN_CU_LOG2 < 10 ? 0: 1;
#else
      Int scale = (iBlkSize < 32 ? 0 : 1);
#endif
#if !JVET_D0033_ADAPTIVE_CLIPPING
      Int bitDepth = rTu.getCU()->getSlice()->getSPS()->getBitDepth(channelType);
#endif
      Int ParShift = 6; //normalization factor
      Int ParScale = 1 << ParShift;
      Int ParOffset = 1 << (ParShift - 1);

#if JVET_C0024_QTBT
      for (Int row = 0; row < iHeight; row++) 
#else
      for (Int row = 0; row < iBlkSize; row++) 
#endif
      {
        Int pos          = row * uiStride;
        Int shiftRow     = row >> scale;
#if JVET_C0024_QTBT
        Int Coeff_Top    = pdpcParam[1][2] >> shiftRow;
        Int Coeff_offset = pdpcParam[1][3] >> shiftRow;
#else
        Int Coeff_Top    = pPdpcPar[2] >> shiftRow;
        Int Coeff_offset = pPdpcPar[3] >> shiftRow;
#endif

#if JVET_C0024_QTBT
        for (Int col = 0; col < iWidth; col++, pos++) 
#else
        for (Int col = 0; col < iBlkSize; col++, pos++) 
#endif
        {
          Int shiftCol      = col >> scale;
#if JVET_C0024_QTBT
          Int Coeff_Left    = pdpcParam[0][0] >> shiftCol;
          Int Coeff_TopLeft = (pdpcParam[0][1] >> shiftCol) + Coeff_offset;
#else
          Int Coeff_Left    = pPdpcPar[0] >> shiftCol;
          Int Coeff_TopLeft = (pPdpcPar[1] >> shiftCol) + Coeff_offset;
#endif
          Int Coeff_Cur     = ParScale - Coeff_Left - Coeff_Top + Coeff_TopLeft;

          Int sampleVal = (Coeff_Left* piRefVector[-row - 1] + Coeff_Top * piRefVector[col + 1] - Coeff_TopLeft * piRefVector[0] + Coeff_Cur * pDst[pos] + ParOffset) >> ParShift;
#if JVET_D0033_ADAPTIVE_CLIPPING
          pDst[pos] = ClipA(sampleVal,compID);
#else
          pDst[pos] = Clip3(0, ((1 << bitDepth) - 1), sampleVal);
#endif
        }
      }
      return; //terminate the prediction process
    }
#else
    const Pel *ptrSrc = getPredictorPtr( compID, bUseFilteredPredSamples );
#endif

#if JVET_G0104_PLANAR_PDPC
    if( isLuma( compID ) )
    {
      if( pcCU->getROTIdx( CHANNEL_TYPE_LUMA, uiAbsPartIdx ) )
      {
        ptrSrc = getPredictorPtr( compID, bUseFilteredPredSamples );
      }
    }
#elif COM16_C1046_PDPC_RSAF_HARMONIZATION
    const Pel *ptrSrc = getPredictorPtr(compID, bUseFilteredPredSamples);
#endif

#if VCEG_AZ05_INTRA_MPI
    TComDataCU *const pcCU = rTu.getCU();
    const UInt              uiAbsPartIdx = rTu.GetAbsPartIdxTU();
#endif
    if ( uiDirMode == PLANAR_IDX )
    {
      xPredIntraPlanar( ptrSrc+sw+1, sw, pDst, uiStride, iWidth, iHeight );
    }
    else
    {
      // Create the prediction
#if !VCEG_AZ05_INTRA_MPI && !COM16_C1046_PDPC_INTRA
      TComDataCU *const pcCU = rTu.getCU();
      const UInt              uiAbsPartIdx      = rTu.GetAbsPartIdxTU();
#endif
      const Bool              enableEdgeFilters = !(pcCU->isRDPCMEnabled(uiAbsPartIdx) && pcCU->getCUTransquantBypass(uiAbsPartIdx));
#if O0043_BEST_EFFORT_DECODING
      const Int channelsBitDepthForPrediction = rTu.getCU()->getSlice()->getSPS()->getStreamBitDepth(channelType);
#else
      const Int channelsBitDepthForPrediction = rTu.getCU()->getSlice()->getSPS()->getBitDepth(channelType);
#endif
#if VCEG_AZ07_INTRA_4TAP_FILTER
      const Bool              enable4TapFilter     = pcCU->getSlice()->getSPS()->getUseIntra4TapFilter();
#endif

#if VCEG_AZ07_INTRA_BOUNDARY_FILTER

#if COM16_C983_RSAF_PREVENT_OVERSMOOTHING
#if VCEG_AZ05_INTRA_MPI
      const Bool              enableBoundaryFilter = pcCU->getSlice()->getSPS()->getUseIntraBoundaryFilter() && (pcCU->getMPIIdx(uiAbsPartIdx) <= 1 || pcCU->getWidth(uiAbsPartIdx)>=16 || !pcCU->getSlice()->getSPS()->getUseRSAF() );
#else
#if COM16_C1046_PDPC_INTRA
      const Bool              enableBoundaryFilter = pcCU->getSlice()->getSPS()->getUseIntraBoundaryFilter() && (pcCU->getPDPCIdx(uiAbsPartIdx) <= 1 || pcCU->getWidth(uiAbsPartIdx) >= 16 || !pcCU->getSlice()->getSPS()->getUseRSAF());
#else
      const Bool              enableBoundaryFilter = pcCU->getSlice()->getSPS()->getUseIntraBoundaryFilter() && (pcCU->getWidth(uiAbsPartIdx) >= 16 || !pcCU->getSlice()->getSPS()->getUseRSAF());
#endif
#endif
#else
      const Bool              enableBoundaryFilter = pcCU->getSlice()->getSPS()->getUseIntraBoundaryFilter();
#endif

#endif
        xPredIntraAng( channelsBitDepthForPrediction, ptrSrc+sw+1, sw, pDst, uiStride, iWidth, iHeight,
#if JVET_D0033_ADAPTIVE_CLIPPING
             compID,
#else
             channelType,
#endif
        uiDirMode, enableEdgeFilters
#if VCEG_AZ07_INTRA_4TAP_FILTER
        , enable4TapFilter
#endif
#if COM16_C983_RSAF_PREVENT_OVERSMOOTHING
        , pcCU->getSlice()->getSPS()->getUseRSAF()
#endif
        );

#if VCEG_AZ05_INTRA_MPI
#if JVET_C0024_QTBT
      if (!(pcCU->getMPIIdx(uiAbsPartIdx) && isLuma(compID)) && (uiDirMode == DC_IDX))
#else
      if (!pcCU->getMPIIdx(uiAbsPartIdx) && (uiDirMode == DC_IDX))
#endif
#else
      if ( uiDirMode == DC_IDX )
#endif
      {
        xDCPredFiltering( ptrSrc+sw+1, sw, pDst, uiStride, iWidth, iHeight, channelType );
      }
#if VCEG_AZ07_INTRA_BOUNDARY_FILTER
#if JVET_C0024_QTBT
      else if( enableBoundaryFilter && isLuma(compID) && iWidth>2 && iHeight>2)
#else
      else if( enableBoundaryFilter && isLuma(compID) )
#endif
      {
#if VCEG_AZ07_INTRA_65ANG_MODES
        if( uiDirMode == VDIA_IDX )
#else
        if( uiDirMode == 34 )
#endif
        {
          xIntraPredFilteringMode34( ptrSrc+sw+1, sw, pDst, uiStride, iWidth, iHeight);
        }
        else  if( uiDirMode == 2 )
        {
          xIntraPredFilteringMode02( ptrSrc+sw+1, sw, pDst, uiStride, iWidth, iHeight);
        }
#if VCEG_AZ07_INTRA_65ANG_MODES
        else if( ( uiDirMode<=10 && uiDirMode>2 ) || ( uiDirMode>=(VDIA_IDX-8) && uiDirMode<VDIA_IDX ) )
#else
        else if( ( uiDirMode<=6 && uiDirMode>2 ) || ( uiDirMode>=30 && uiDirMode<34 ) )
#endif
        {
          xIntraPredFilteringModeDGL( ptrSrc+sw+1, sw, pDst, uiStride, iWidth, iHeight, uiDirMode );
        }
      }
#endif
    }
#if VCEG_AZ05_INTRA_MPI
#if JVET_C0024_QTBT
    if (pcCU->getMPIIdx(uiAbsPartIdx) && isLuma(compID) && pcCU->getCUPelX() && pcCU->getCUPelY() && pcCU->getSlice()->getSPS()->getUseMPI())
#else
    if (pcCU->getMPIIdx(uiAbsPartIdx) && pcCU->getCUPelX() && pcCU->getCUPelY() && pcCU->getSlice()->getSPS()->getUseMPI())
#endif
    {
      Pel* pRec = pcCU->getPic()->getPicYuvRec()->getAddr(compID, pcCU->getCtuRsAddr(), pcCU->getZorderIdxInCtu() + uiAbsPartIdx);   
      Int iStrideRec = pcCU->getPic()->getPicYuvRec()->getStride(compID);
#if JVET_C0024_QTBT
      PartSize eSize = SIZE_2Nx2N;
#else
      PartSize eSize = pcCU->getPartitionSize(uiAbsPartIdx);
#endif
      Int idexMPI = pcCU->getMPIIdx(uiAbsPartIdx);
      if (idexMPI>3) idexMPI = 0;
#if JVET_C0024_QTBT
      idexMPI += (iWidth*iHeight<64 ? 4 : 0);
#else
      idexMPI += (eSize == SIZE_NxN ? 4 : 0);
#endif
      xMPIredFiltering(pRec, iStrideRec, pDst, uiStride, iWidth, iHeight, idexMPI);
    }
#endif

  }

}

/** Check for identical motion in both motion vector direction of a bi-directional predicted CU
  * \returns true, if motion vectors and reference pictures match
 */
Bool TComPrediction::xCheckIdenticalMotion ( TComDataCU* pcCU, UInt PartAddr )
{
  if( pcCU->getSlice()->isInterB() && !pcCU->getSlice()->getPPS()->getWPBiPred() )
  {
#if VCEG_AZ07_FRUC_MERGE
    if( pcCU->getFRUCMgrMode( PartAddr ) )
      return false;
#endif
    if( pcCU->getCUMvField(REF_PIC_LIST_0)->getRefIdx(PartAddr) >= 0 && pcCU->getCUMvField(REF_PIC_LIST_1)->getRefIdx(PartAddr) >= 0)
    {
      Int RefPOCL0 = pcCU->getSlice()->getRefPic(REF_PIC_LIST_0, pcCU->getCUMvField(REF_PIC_LIST_0)->getRefIdx(PartAddr))->getPOC();
      Int RefPOCL1 = pcCU->getSlice()->getRefPic(REF_PIC_LIST_1, pcCU->getCUMvField(REF_PIC_LIST_1)->getRefIdx(PartAddr))->getPOC();
      if(RefPOCL0 == RefPOCL1 && pcCU->getCUMvField(REF_PIC_LIST_0)->getMv(PartAddr) == pcCU->getCUMvField(REF_PIC_LIST_1)->getMv(PartAddr))
      {
        return true;
      }
    }
  }
  return false;
}

#if JVET_E0052_DMVR
Void TComPrediction::motionCompensation ( TComDataCU* pcCU, TComYuv* pcYuvPred, Bool bRefineflag, RefPicList eRefPicList, Int iPartIdx )
#else
Void TComPrediction::motionCompensation ( TComDataCU* pcCU, TComYuv* pcYuvPred, RefPicList eRefPicList, Int iPartIdx )
#endif
{
  Int         iWidth;
  Int         iHeight;
  UInt        uiPartAddr;

#if JVET_C0024_QTBT
  assert(iPartIdx<=0);
#endif
  if ( iPartIdx >= 0 )
  {
    pcCU->getPartIndexAndSize( iPartIdx, uiPartAddr, iWidth, iHeight );
    if ( eRefPicList != REF_PIC_LIST_X )
    {
      if( pcCU->getSlice()->getPPS()->getUseWP()
#if VCEG_AZ06_IC
        && !pcCU->getICFlag( uiPartAddr )
#endif
        )
      {
        xPredInterUni (pcCU, uiPartAddr, iWidth, iHeight, eRefPicList, pcYuvPred
#if JVET_E0052_DMVR
    , bRefineflag
#endif
#if VCEG_AZ05_BIO                  
          , false
#endif
          , true );
      }
      else
      {
        xPredInterUni (pcCU, uiPartAddr, iWidth, iHeight, eRefPicList, pcYuvPred
#if JVET_E0052_DMVR
    , bRefineflag
#endif
       );
      }
      if ( pcCU->getSlice()->getPPS()->getUseWP() 
#if VCEG_AZ06_IC
        && !pcCU->getICFlag( uiPartAddr )
#endif
        )
      {
        xWeightedPredictionUni( pcCU, pcYuvPred, uiPartAddr, iWidth, iHeight, eRefPicList, pcYuvPred );
      }
    }
    else
    {
#if COM16_C806_VCEG_AZ10_SUB_PU_TMVP && !COM16_C1045_BIO_HARMO_IMPROV
      if ( pcCU->getMergeType(uiPartAddr))  
      {
        Int iNumSPInOneLine, iNumSP, iSPWidth, iSPHeight;

        pcCU->getSPPara(iWidth, iHeight, iNumSP, iNumSPInOneLine, iSPWidth, iSPHeight);

        xGetSubPUAddrAndMerge(pcCU, uiPartAddr, iSPWidth, iSPHeight, iNumSPInOneLine, iNumSP, m_puiW, m_puiH, m_puiSPAddr);

        //MC
        for (Int i = 0; i < iNumSP; i++)
        {
          if (m_puiW[i]==0 || m_puiH[i]==0)
          {
            continue;
          }

          if(xCheckIdenticalMotion( pcCU, m_puiSPAddr[i] ))
          {
            xPredInterUni (pcCU, m_puiSPAddr[i], m_puiW[i], m_puiH[i], REF_PIC_LIST_0, pcYuvPred 
#if JVET_E0052_DMVR
, bRefineflag
#endif
          );
          }
          else
          {
            xPredInterBi  (pcCU, m_puiSPAddr[i], m_puiW[i], m_puiH[i], pcYuvPred
#if JVET_E0052_DMVR
    , bRefineflag
#endif
          );
          }
        }
      }
      else
      {
#endif
#if COM16_C1016_AFFINE
        if ( (  pcCU->isAffine(uiPartAddr) && xCheckIdenticalAffineMotion( pcCU, uiPartAddr, iWidth, iHeight ) )
          || ( ( !pcCU->isAffine(uiPartAddr) && xCheckIdenticalMotion( pcCU, uiPartAddr ) ) 
#if COM16_C1045_BIO_HARMO_IMPROV 
#if VCEG_AZ07_FRUC_MERGE
          && pcCU->getFRUCMgrMode( uiPartAddr ) == 0
#endif
#if COM16_C806_VCEG_AZ10_SUB_PU_TMVP
          && pcCU->getMergeType( uiPartAddr ) == MGR_TYPE_DEFAULT_N
#endif
#endif
          ) )
#else
      if ( xCheckIdenticalMotion( pcCU, uiPartAddr ) 
#if COM16_C1045_BIO_HARMO_IMPROV 
#if VCEG_AZ07_FRUC_MERGE
        && pcCU->getFRUCMgrMode( uiPartAddr ) == 0
#endif
#if COM16_C806_VCEG_AZ10_SUB_PU_TMVP
        && pcCU->getMergeType( uiPartAddr ) == MGR_TYPE_DEFAULT_N
#endif
#endif
        )
#endif
      {
        xPredInterUni (pcCU, uiPartAddr, iWidth, iHeight, REF_PIC_LIST_0, pcYuvPred 
#if JVET_E0052_DMVR
    , bRefineflag
#endif
       );
      }
      else
      {
        xPredInterBi  (pcCU, uiPartAddr, iWidth, iHeight, pcYuvPred 
#if JVET_E0052_DMVR
    , bRefineflag
#endif
      );
      }
#if COM16_C806_VCEG_AZ10_SUB_PU_TMVP && !COM16_C1045_BIO_HARMO_IMPROV
      }
#endif
    }
    return;
  }

  for ( iPartIdx = 0; iPartIdx < pcCU->getNumPartitions(); iPartIdx++ )
  {
    pcCU->getPartIndexAndSize( iPartIdx, uiPartAddr, iWidth, iHeight );

    if ( eRefPicList != REF_PIC_LIST_X )
    {
      if( pcCU->getSlice()->getPPS()->getUseWP()
#if VCEG_AZ06_IC
        && !pcCU->getICFlag( uiPartAddr )
#endif
        )
      {
        xPredInterUni (pcCU, uiPartAddr, iWidth, iHeight, eRefPicList, pcYuvPred
#if JVET_E0052_DMVR
    , bRefineflag
#endif
#if VCEG_AZ05_BIO                  
          , false
#endif
          , true );
      }
      else
      {
        xPredInterUni (pcCU, uiPartAddr, iWidth, iHeight, eRefPicList, pcYuvPred 
#if JVET_E0052_DMVR
     , bRefineflag
#endif
      );
      }
      if ( pcCU->getSlice()->getPPS()->getUseWP() 
#if VCEG_AZ06_IC
        && !pcCU->getICFlag( uiPartAddr )
#endif
        )
      {
        xWeightedPredictionUni( pcCU, pcYuvPred, uiPartAddr, iWidth, iHeight, eRefPicList, pcYuvPred );
      }
    }
    else
    {
#if COM16_C806_VCEG_AZ10_SUB_PU_TMVP && !COM16_C1045_BIO_HARMO_IMPROV
      if (pcCU->getMergeType(uiPartAddr))  
      {
        Int iNumSPInOneLine, iNumSP, iSPWidth, iSPHeight;

        pcCU->getSPPara(iWidth, iHeight, iNumSP, iNumSPInOneLine, iSPWidth, iSPHeight);

        xGetSubPUAddrAndMerge(pcCU, uiPartAddr, iSPWidth, iSPHeight, iNumSPInOneLine, iNumSP, m_puiW, m_puiH, m_puiSPAddr);
      
        for (Int i = 0; i < iNumSP; i++)
        {
          if (m_puiW[i]==0 || m_puiH[i]==0)
          {
            continue;
          }
          if( xCheckIdenticalMotion( pcCU, m_puiSPAddr[i] ))
          {
            xPredInterUni (pcCU, m_puiSPAddr[i], m_puiW[i], m_puiH[i], REF_PIC_LIST_0, pcYuvPred 
#if JVET_E0052_DMVR
    , bRefineflag
#endif
          );
          }
          else
          {
            xPredInterBi  (pcCU, m_puiSPAddr[i], m_puiW[i], m_puiH[i], pcYuvPred
#if JVET_E0052_DMVR
    , bRefineflag
#endif 
          );
          }
        }
      }
      else
      {
#endif
#if COM16_C1016_AFFINE
        if ( (  pcCU->isAffine(uiPartAddr) && xCheckIdenticalAffineMotion( pcCU, uiPartAddr, iWidth, iHeight ) )
          || ( ( !pcCU->isAffine(uiPartAddr) && xCheckIdenticalMotion( pcCU, uiPartAddr ) ) 
#if COM16_C1045_BIO_HARMO_IMPROV
#if VCEG_AZ07_FRUC_MERGE
          && pcCU->getFRUCMgrMode( uiPartAddr ) == 0
#endif
#if COM16_C806_VCEG_AZ10_SUB_PU_TMVP
          && pcCU->getMergeType( uiPartAddr ) == MGR_TYPE_DEFAULT_N
#endif
#endif
          ) )
#else
      if ( xCheckIdenticalMotion( pcCU, uiPartAddr ) 
#if COM16_C1045_BIO_HARMO_IMPROV
#if VCEG_AZ07_FRUC_MERGE
        && pcCU->getFRUCMgrMode( uiPartAddr ) == 0
#endif
#if COM16_C806_VCEG_AZ10_SUB_PU_TMVP
        && pcCU->getMergeType( uiPartAddr ) == MGR_TYPE_DEFAULT_N
#endif
#endif
        )
#endif
      {
        xPredInterUni (pcCU, uiPartAddr, iWidth, iHeight, REF_PIC_LIST_0, pcYuvPred 
#if JVET_E0052_DMVR
    , bRefineflag
#endif
      );
      }
      else
      {
        xPredInterBi  (pcCU, uiPartAddr, iWidth, iHeight, pcYuvPred 
#if JVET_E0052_DMVR
    , bRefineflag
#endif
      );
      }
#if COM16_C806_VCEG_AZ10_SUB_PU_TMVP && !COM16_C1045_BIO_HARMO_IMPROV
      }
#endif
    }
  }
  return;
}

Void TComPrediction::xPredInterUni ( TComDataCU* pcCU, UInt uiPartAddr, Int iWidth, Int iHeight, RefPicList eRefPicList, TComYuv* pcYuvPred
#if JVET_E0052_DMVR
    , Bool bRefineflag
#endif
#if VCEG_AZ05_BIO                  
  ,Bool bBIOapplied
#endif
  , Bool bi 
#if VCEG_AZ07_FRUC_MERGE
  , Bool bOBMC
#endif
  )
{
#if COM16_C1016_AFFINE
  if ( pcCU->isAffine(uiPartAddr) )
  {
    Int iRefIdx = pcCU->getCUMvField( eRefPicList )->getRefIdx( uiPartAddr );
    assert (iRefIdx >= 0);

    // Get Part Index in LCU and get Mv
    TComMv acMv[3];
    UInt uiPartIdxLT, uiPartIdxRT, uiPartIdxLB, uiAbsIndexInLCU;
    uiAbsIndexInLCU = pcCU->getZorderIdxInCtu();
    uiPartIdxLT = uiPartAddr + uiAbsIndexInLCU;
    uiPartIdxRT = g_auiRasterToZscan [ g_auiZscanToRaster[ uiPartIdxLT ] + iWidth / pcCU->getPic()->getMinCUWidth() - 1 ];
    uiPartIdxLB = g_auiRasterToZscan [ g_auiZscanToRaster[ uiPartIdxLT ] + ( (iHeight / pcCU->getPic()->getMinCUHeight()) - 1 ) * pcCU->getPic()->getNumPartInCtuWidth() ];

    acMv[0] = pcCU->getCUMvField( eRefPicList )->getMv( uiPartIdxLT - uiAbsIndexInLCU );  pcCU->clipMv(acMv[0]);
    acMv[1] = pcCU->getCUMvField( eRefPicList )->getMv( uiPartIdxRT - uiAbsIndexInLCU );  pcCU->clipMv(acMv[1]);
    acMv[2] = pcCU->getCUMvField( eRefPicList )->getMv( uiPartIdxLB - uiAbsIndexInLCU );  pcCU->clipMv(acMv[2]);

    for (UInt comp=COMPONENT_Y; comp<pcYuvPred->getNumberValidComponents(); comp++)
    {
      const ComponentID compID=ComponentID(comp);
      xPredAffineBlk( compID, pcCU, pcCU->getSlice()->getRefPic( eRefPicList, iRefIdx )->getPicYuvRec(), uiPartAddr, acMv, iWidth, iHeight, pcYuvPred, bi, pcCU->getSlice()->getSPS()->getBitDepth(toChannelType(compID)) );
    }

    return;
  }
#endif

#if VCEG_AZ07_FRUC_MERGE || COM16_C1045_BIO_HARMO_IMPROV
  Int nBlkWidth = iWidth;
  Int nBlkHeight = iHeight;
  Int nBlkStepX = iWidth;
  Int nBlkStepY = iHeight;
  Int xRasterOffsetStep = 0;
  Int yRasterOffsetStep = 0;
  UInt uiIdxRasterStart = g_auiZscanToRaster[pcCU->getZorderIdxInCtu() + uiPartAddr];
  Int nBlkMCWidth = iWidth;
  if( 
#if VCEG_AZ07_FRUC_MERGE
    !bOBMC 
#else
    true
#endif
#if COM16_C1045_BIO_HARMO_IMPROV
    && ( false
#if VCEG_AZ07_FRUC_MERGE
    || pcCU->getFRUCMgrMode( uiPartAddr )
#endif   
#if COM16_C806_VCEG_AZ10_SUB_PU_TMVP
    || pcCU->getMergeType( uiPartAddr ) != MGR_TYPE_DEFAULT_N
#endif
    )
#else
    && pcCU->getFRUCMgrMode( uiPartAddr ) 
#if VCEG_AZ05_BIO                  
    && !bBIOapplied
#endif
#endif
    )
  {
#if COM16_C1045_BIO_HARMO_IMPROV
    Int nRefineBlkSize = 4;
#if VCEG_AZ07_FRUC_MERGE
    if( pcCU->getFRUCMgrMode( uiPartAddr ) )
    {
      nRefineBlkSize = xFrucGetSubBlkSize( pcCU , uiPartAddr , nBlkWidth , nBlkHeight );
    }
#endif
#else
    Int nRefineBlkSize = xFrucGetSubBlkSize( pcCU , uiPartAddr , nBlkWidth , nBlkHeight );
#endif
    nBlkMCWidth = iWidth = iHeight = nRefineBlkSize;
    nBlkStepX = nBlkStepY = nRefineBlkSize;
    xRasterOffsetStep = nRefineBlkSize >> 2;
    yRasterOffsetStep = xRasterOffsetStep * pcCU->getPic()->getNumPartInCtuWidth();
  }
  for( Int y = 0 , yRasterOffset = 0 ; y < nBlkHeight ; y += nBlkStepY , yRasterOffset += yRasterOffsetStep )
  {
    for( Int x = 0 , xRasterOffset = 0 ; x < nBlkWidth ; x += nBlkStepX , xRasterOffset += xRasterOffsetStep )
    {
      uiPartAddr = g_auiRasterToZscan[uiIdxRasterStart+yRasterOffset+xRasterOffset] - pcCU->getZorderIdxInCtu();
#endif
  Int         iRefIdx     = pcCU->getCUMvField( eRefPicList )->getRefIdx( uiPartAddr );           assert (iRefIdx >= 0);
  TComMv      cMv         = pcCU->getCUMvField( eRefPicList )->getMv( uiPartAddr );
  pcCU->clipMv(cMv);

#if VCEG_AZ07_FRUC_MERGE || COM16_C1045_BIO_HARMO_IMPROV
  // check whether later blocks have the same MV, refidx must been the same
  iWidth = nBlkMCWidth;
  for( Int xLater = x + nBlkStepX , xRasterOffsetLater = xRasterOffset + xRasterOffsetStep ; xLater < nBlkWidth ; xLater += nBlkStepX , xRasterOffsetLater += xRasterOffsetStep )
  {
    UInt uiPartAddrLater = g_auiRasterToZscan[uiIdxRasterStart+yRasterOffset+xRasterOffsetLater] - pcCU->getZorderIdxInCtu();
    if( pcCU->getCUMvField( eRefPicList )->getMv( uiPartAddrLater ) == cMv 
#if COM16_C1045_BIO_HARMO_IMPROV
      && pcCU->getCUMvField( eRefPicList )->getRefIdx( uiPartAddrLater ) == iRefIdx 
#endif
      )
    {
      iWidth += nBlkStepX;
      x += nBlkStepX;
      xRasterOffset += xRasterOffsetStep;
    }
    else
      break;
  }
#endif
#if JVET_E0052_DMVR
  Bool bBIPMVRefine = bi;
  if (bi)
  {
      Int iPOC0=pcCU->getSlice()->getRefPOC(REF_PIC_LIST_0, pcCU->getCUMvField( REF_PIC_LIST_0 )->getRefIdx( uiPartAddr ));
      Int iPOC1=pcCU->getSlice()->getRefPOC(REF_PIC_LIST_1, pcCU->getCUMvField( REF_PIC_LIST_1 )->getRefIdx( uiPartAddr ));
      Int iPOC = pcCU->getSlice()->getPOC();
      bBIPMVRefine = !bOBMC && ((iPOC - iPOC0)*(iPOC - iPOC1) < 0) && pcCU->getMergeFlag(uiPartAddr) && !pcCU->getICFlag(uiPartAddr) && !pcCU->getAffineFlag(uiPartAddr) && bRefineflag;
      bBIPMVRefine &= pcCU->getMergeType(uiPartAddr) == MGR_TYPE_DEFAULT_N;
      bBIPMVRefine &= !pcCU->getFRUCMgrMode(uiPartAddr);
      bBIPMVRefine &= pcCU->getSlice()->getSPS()->getUseDMVR();
  }
  for (UInt comp=COMPONENT_Y; comp<(bBIPMVRefine ? 1: pcYuvPred->getNumberValidComponents()); comp++)
#else
  for (UInt comp=COMPONENT_Y; comp<pcYuvPred->getNumberValidComponents(); comp++)
#endif
  {
    const ComponentID compID=ComponentID(comp);
    xPredInterBlk  ( compID, pcCU, pcCU->getSlice()->getRefPic( eRefPicList, iRefIdx )->getPicYuvRec(), uiPartAddr, &cMv, iWidth, iHeight, pcYuvPred, bi, pcCU->getSlice()->getSPS()->getBitDepth(toChannelType(compID))
#if VCEG_AZ05_BIO                  
#if JVET_E0052_DMVR
    , (bBIPMVRefine ? false : (bBIOapplied && compID == COMPONENT_Y))
#else
    , bBIOapplied && compID == COMPONENT_Y
#endif
#endif
#if VCEG_AZ07_FRUC_MERGE
      , 0 
#endif
#if VCEG_AZ06_IC
      , pcCU->getICFlag( uiPartAddr ) 
#endif
      );
  }
#if VCEG_AZ07_FRUC_MERGE || COM16_C1045_BIO_HARMO_IMPROV
    }
  }
#endif
}

Void TComPrediction::xPredInterBi ( TComDataCU* pcCU, UInt uiPartAddr, Int iWidth, Int iHeight, TComYuv* pcYuvPred 
#if JVET_E0052_DMVR
    , Bool bRefineflag
#endif
#if VCEG_AZ07_FRUC_MERGE || JVET_G0082
  , Bool bOBMC
#endif
  )
{
  TComYuv* pcMbYuv;
  Int      iRefIdx[NUM_REF_PIC_LIST_01] = {-1, -1};
#if COM16_C1045_BIO_HARMO_IMPROV
  Int nBlkWidth = iWidth;
  Int nBlkHeight = iHeight;
  Int nBlkStepX = iWidth;
  Int nBlkStepY = iHeight;
  Int xRasterOffsetStep = 0;
  Int yRasterOffsetStep = 0;
  UInt uiIdxRasterStart = g_auiZscanToRaster[pcCU->getZorderIdxInCtu() + uiPartAddr];
  if( 
#if VCEG_AZ07_FRUC_MERGE
    !bOBMC 
#else
    true
#endif
    && ( false 
#if VCEG_AZ07_FRUC_MERGE
    || pcCU->getFRUCMgrMode( uiPartAddr ) 
#endif
#if COM16_C806_VCEG_AZ10_SUB_PU_TMVP
    || pcCU->getMergeType( uiPartAddr ) != MGR_TYPE_DEFAULT_N
#endif
    ) )
  {
    Int nRefineBlkSize = 4;
#if VCEG_AZ07_FRUC_MERGE
    if( pcCU->getFRUCMgrMode( uiPartAddr ) )
    {
      nRefineBlkSize = xFrucGetSubBlkSize( pcCU , uiPartAddr , nBlkWidth , nBlkHeight );
    }
#endif
    iWidth = iHeight = nRefineBlkSize;
    nBlkStepX = nBlkStepY = nRefineBlkSize;
    xRasterOffsetStep = nRefineBlkSize >> 2;
    yRasterOffsetStep = xRasterOffsetStep * pcCU->getPic()->getNumPartInCtuWidth();
  }
  for( Int y = 0 , yRasterOffset = 0 ; y < nBlkHeight ; y += nBlkStepY , yRasterOffset += yRasterOffsetStep )
  {
    for( Int x = 0 , xRasterOffset = 0 ; x < nBlkWidth ; x += nBlkStepX , xRasterOffset += xRasterOffsetStep )
    {
      uiPartAddr = g_auiRasterToZscan[uiIdxRasterStart+yRasterOffset+xRasterOffset] - pcCU->getZorderIdxInCtu();
#endif
#if VCEG_AZ05_BIO 
  Int      FrameNumber[3] = {-1, -1,-1};
  FrameNumber[2] = pcCU->getSlice()->getPOC();
  bool bBIOcheck0 = pcCU->getSlice()->getPPS()->getWPBiPred()    && pcCU->getSlice()->getSliceType() == B_SLICE; 
  bool bBIOcheck1 =  pcCU->getSlice()->getPPS()->getUseWP() && pcCU->getSlice()->getSliceType() == P_SLICE;
  bool bBIOapplied = false;
  if (pcCU->getSlice()->getSPS()->getUseBIO())
  {
    for ( Int iRefList = 0; iRefList < 2; iRefList++ )
    {
      RefPicList eRefPicList = (iRefList ? REF_PIC_LIST_1 : REF_PIC_LIST_0);
      iRefIdx[iRefList] = pcCU->getCUMvField( eRefPicList )->getRefIdx( uiPartAddr );
      if ( iRefIdx[iRefList] >= 0 )
        FrameNumber[iRefList] = pcCU->getSlice()->getRefPic(eRefPicList,iRefIdx[iRefList])->getPOC() ;
    }

    if ( iRefIdx[0] >= 0 && iRefIdx[1] >= 0 ) // applied for only EL,  only if Bi-pred is from different "time directions"
    {  
      int d1 = FrameNumber[1] - FrameNumber[2], d0 = FrameNumber[2] - FrameNumber[0];
      if (d1 * d0 > 0&&!bBIOcheck0&&! bBIOcheck1 )
      {
        bBIOapplied = true;
      }
    }

#if COM16_C1045_BIO_HARMO_IMPROV
#if VCEG_AZ06_IC
    if( pcCU->getICFlag( uiPartAddr ) )
    {
      bBIOapplied = false;
    }
    else
#endif
    if( pcCU->isBIOLDB( uiPartAddr ) )
    {
      bBIOapplied = true;
    }
#else
#if VCEG_AZ07_FRUC_MERGE 
  if (pcCU->getFRUCMgrMode( uiPartAddr ) ) bBIOapplied = false;
#endif
#endif

#if COM16_C1016_AFFINE
  if ( pcCU->isAffine( uiPartAddr) )       bBIOapplied = false;
#endif

#if JVET_G0082
  if (bOBMC)                               bBIOapplied = false;
#endif

  }
#endif
  for ( UInt refList = 0; refList < NUM_REF_PIC_LIST_01; refList++ )
  {
    RefPicList eRefPicList = (refList ? REF_PIC_LIST_1 : REF_PIC_LIST_0);
    iRefIdx[refList] = pcCU->getCUMvField( eRefPicList )->getRefIdx( uiPartAddr );

    if ( iRefIdx[refList] < 0 )
    {
      continue;
    }

    assert( iRefIdx[refList] < pcCU->getSlice()->getNumRefIdx(eRefPicList) );
#if VCEG_AZ05_BIO 
    iRefListIdx = refList;
#endif
    pcMbYuv = &m_acYuvPred[refList];
    if( pcCU->getCUMvField( REF_PIC_LIST_0 )->getRefIdx( uiPartAddr ) >= 0 && pcCU->getCUMvField( REF_PIC_LIST_1 )->getRefIdx( uiPartAddr ) >= 0 )
    {
      xPredInterUni ( pcCU, uiPartAddr, iWidth, iHeight, eRefPicList, pcMbYuv
#if JVET_E0052_DMVR
    , bRefineflag
#endif
#if VCEG_AZ05_BIO                  
        ,bBIOapplied
#endif
        , true 
#if VCEG_AZ07_FRUC_MERGE
        , bOBMC
#endif
        );
    }
    else
    {
#if VCEG_AZ06_IC
      if ( ( ( pcCU->getSlice()->getPPS()->getUseWP()       && pcCU->getSlice()->getSliceType() == P_SLICE ) || 
        ( pcCU->getSlice()->getPPS()->getWPBiPred() && pcCU->getSlice()->getSliceType() == B_SLICE ) ) 
        && !pcCU->getICFlag( uiPartAddr ) )
#else
      if ( ( pcCU->getSlice()->getPPS()->getUseWP()       && pcCU->getSlice()->getSliceType() == P_SLICE ) ||
           ( pcCU->getSlice()->getPPS()->getWPBiPred()    && pcCU->getSlice()->getSliceType() == B_SLICE ) )
#endif
      {
        xPredInterUni ( pcCU, uiPartAddr, iWidth, iHeight, eRefPicList, pcMbYuv
#if JVET_E0052_DMVR
    , bRefineflag
#endif
#if VCEG_AZ05_BIO                  
          ,bBIOapplied
#endif
          , true 
#if VCEG_AZ07_FRUC_MERGE
          , bOBMC
#endif
          );
      }
      else
      {
        xPredInterUni ( pcCU, uiPartAddr, iWidth, iHeight, eRefPicList, pcMbYuv 
#if JVET_E0052_DMVR
    , bRefineflag
#endif
#if VCEG_AZ05_BIO                  
          ,bBIOapplied
#endif
#if VCEG_AZ07_FRUC_MERGE
          , false , bOBMC
#endif
          );
      }
    }
  }

  if ( pcCU->getSlice()->getPPS()->getWPBiPred()    && pcCU->getSlice()->getSliceType() == B_SLICE  
#if VCEG_AZ06_IC
    && !pcCU->getICFlag( uiPartAddr )
#endif
    )
  {
    xWeightedPredictionBi( pcCU, &m_acYuvPred[REF_PIC_LIST_0], &m_acYuvPred[REF_PIC_LIST_1], iRefIdx[REF_PIC_LIST_0], iRefIdx[REF_PIC_LIST_1], uiPartAddr, iWidth, iHeight, pcYuvPred );
  }
  else if ( pcCU->getSlice()->getPPS()->getUseWP() && pcCU->getSlice()->getSliceType() == P_SLICE
#if VCEG_AZ06_IC
    && !pcCU->getICFlag( uiPartAddr )
#endif
    )
  {
    xWeightedPredictionUni( pcCU, &m_acYuvPred[REF_PIC_LIST_0], uiPartAddr, iWidth, iHeight, REF_PIC_LIST_0, pcYuvPred );
  }
  else
  {
    xWeightedAverage( &m_acYuvPred[REF_PIC_LIST_0], &m_acYuvPred[REF_PIC_LIST_1], iRefIdx[REF_PIC_LIST_0], iRefIdx[REF_PIC_LIST_1], uiPartAddr, iWidth, iHeight, pcYuvPred, pcCU->getSlice()->getSPS()->getBitDepths() 
#if VCEG_AZ05_BIO                  
      ,bBIOapplied
#endif
#if COM16_C1045_BIO_HARMO_IMPROV || JVET_C0027_BIO || JVET_E0052_DMVR
      , pcCU 
#endif
#if JVET_E0052_DMVR
    , bRefineflag
#endif
#if JVET_E0052_DMVR || JVET_G0082
    , bOBMC
#endif
      );
  }
#if COM16_C1045_BIO_HARMO_IMPROV
  }
  }
#endif
}
#if VCEG_AZ05_BIO 
Void  TComPrediction::xGradFilterX(Pel*  piRefY, Int iRefStride,Pel*  piDstY,Int iDstStride,
Int iWidth, Int iHeight,Int iMVyFrac,Int iMVxFrac, const Int bitDepth)
{
  static const Int iBIOGradShift = 4;
  if ( iMVyFrac == 0 )
  {
    gradFilter1DHor (piRefY, iRefStride,  iWidth, iHeight, iDstStride,  piDstY, iMVxFrac, iBIOGradShift   );
    return;
  }

  Int tmpStride = m_filteredBlockTmp[0].getStride(COMPONENT_Y);
  Pel *tmp    = m_filteredBlockTmp[0].getAddr(COMPONENT_Y);
  Int shift0 = bitDepth-8;
  Int shift1 = 6 + iBIOGradShift - shift0;
  fracFilter2DVer (piRefY - BIO_FILTER_HALF_LENGTH_MINUS_1,  iRefStride,  iWidth +BIO_FILTER_LENGTH_MINUS_1 , iHeight, tmpStride,  tmp,  iMVyFrac        ,shift0);
  gradFilter2DHor  (tmp +   BIO_FILTER_HALF_LENGTH_MINUS_1,  tmpStride,  iWidth            , iHeight, iDstStride,  piDstY ,iMVxFrac ,shift1);
}

Void  TComPrediction::xGradFilterY(Pel*  piRefY, Int iRefStride,Pel*  piDstY,Int iDstStride,
  Int iWidth, Int iHeight,Int iMVyFrac,Int iMVxFrac, const Int bitDepth)
{
  static const Int iBIOGradShift = 4;
  if ( iMVxFrac == 0 )
  {
    gradFilter1DVer (piRefY, iRefStride,  iWidth, iHeight, iDstStride,  piDstY, iMVyFrac, iBIOGradShift );
    return;
  }

  Int tmpStride = m_filteredBlockTmp[0].getStride(COMPONENT_Y);
  Pel *tmp    = m_filteredBlockTmp[0].getAddr(COMPONENT_Y);
  Int shift0 = bitDepth-8;
  Int shift1 = 6 + iBIOGradShift - shift0;
  gradFilter2DVer (piRefY  - BIO_FILTER_HALF_LENGTH_MINUS_1,  iRefStride,  iWidth +BIO_FILTER_LENGTH_MINUS_1 , iHeight, tmpStride,  tmp,  iMVyFrac ,shift0);
  fracFilter2DHor (tmp  + BIO_FILTER_HALF_LENGTH_MINUS_1,  tmpStride,  iWidth            , iHeight, iDstStride,  piDstY , iMVxFrac ,shift1);
}
#endif

#if JVET_E0052_DMVR
Void TComPrediction::xPredInterLines(TComDataCU *cu, TComPicYuv *refPic, UInt partAddr, TComMv *mv, Int width, Int height, Pel* dstPix, Int dstStride, Bool bi, const Int bitDepth)
{
  cu->clipMv( *mv );

  ComponentID compID = COMPONENT_Y;
  Int     refStride  = refPic->getStride(compID);
  Int shiftHor=(2+refPic->getComponentScaleX(compID));
  Int shiftVer=(2+refPic->getComponentScaleY(compID));
#if VCEG_AZ07_MV_ADD_PRECISION_BIT_FOR_STORE
  shiftHor += VCEG_AZ07_MV_ADD_PRECISION_BIT_FOR_STORE;
  shiftVer += VCEG_AZ07_MV_ADD_PRECISION_BIT_FOR_STORE;
#endif
  Int     refOffset  = (mv->getHor() >> shiftHor) + (mv->getVer() >> shiftVer) * refStride;

  Pel*    ref     = refPic->getAddr(compID, cu->getCtuRsAddr(), cu->getZorderIdxInCtu() + partAddr ) + refOffset;

  Int     xFrac  = mv->getHor() & ((1<<shiftHor)-1);
  Int     yFrac  = mv->getVer() & ((1<<shiftVer)-1);
  UInt    cxWidth  = width  >> refPic->getComponentScaleX(compID);
  UInt    cxHeight = height >> refPic->getComponentScaleY(compID);

  const ChromaFormat chFmt = cu->getPic()->getChromaFormat();

  if ( yFrac == 0 )
  {
    m_if.filterHor(compID, ref, refStride, dstPix,  dstStride, cxWidth, cxHeight, xFrac, !bi, chFmt, bitDepth);
  }
  else if ( xFrac == 0 )
  {
    m_if.filterVer(compID, ref, refStride, dstPix, dstStride, cxWidth, cxHeight, yFrac, true, !bi, chFmt, bitDepth);
  }
  else
  {
    Int   tmpStride = m_filteredBlockTmp[0].getStride(compID);
    Pel*  tmp       = m_filteredBlockTmp[0].getAddr(compID);

    const Int vFilterSize = isLuma(compID) ? NTAPS_LUMA : NTAPS_CHROMA;

    m_if.filterHor(compID, ref - ((vFilterSize>>1) -1)*refStride, refStride, tmp, tmpStride, cxWidth, cxHeight+vFilterSize-1, xFrac, false,      chFmt, bitDepth);
    m_if.filterVer(compID, tmp + ((vFilterSize>>1) -1)*tmpStride, tmpStride, dstPix, dstStride, cxWidth, cxHeight,               yFrac, false,  !bi, chFmt, bitDepth);
  }
}
#endif
/**
 * \brief Generate motion-compensated block
 *
 * \param compID     Colour component ID
 * \param cu         Pointer to current CU
 * \param refPic     Pointer to reference picture
 * \param partAddr   Address of block within CU
 * \param mv         Motion vector
 * \param width      Width of block
 * \param height     Height of block
 * \param dstPic     Pointer to destination picture
 * \param bi         Flag indicating whether bipred is used
 * \param  bitDepth  Bit depth
 */


Void TComPrediction::xPredInterBlk(const ComponentID compID, TComDataCU *cu, TComPicYuv *refPic, UInt partAddr, TComMv *mv, Int width, Int height, TComYuv *dstPic, Bool bi, const Int bitDepth
#if VCEG_AZ05_BIO                  
  ,bool bBIOapplied
#endif
#if VCEG_AZ07_FRUC_MERGE
  , Int nFRUCMode
#endif
#if VCEG_AZ06_IC
  , Bool bICFlag
#endif
  )
{
#if VCEG_AZ07_FRUC_MERGE
  cu->clipMv( *mv );
  Int nFilterIdx = nFRUCMode ? cu->getSlice()->getSPS()->getFRUCRefineFilter() : 0;
  assert( bitDepth == cu->getSlice()->getSPS()->getBitDepth( compID == COMPONENT_Y ? CHANNEL_TYPE_LUMA : CHANNEL_TYPE_CHROMA ) );
#endif
  Int     refStride  = refPic->getStride(compID);
  Int     dstStride  = dstPic->getStride(compID);
  Int shiftHor=(2+refPic->getComponentScaleX(compID));
  Int shiftVer=(2+refPic->getComponentScaleY(compID));
#if VCEG_AZ07_MV_ADD_PRECISION_BIT_FOR_STORE
  shiftHor += VCEG_AZ07_MV_ADD_PRECISION_BIT_FOR_STORE;
  shiftVer += VCEG_AZ07_MV_ADD_PRECISION_BIT_FOR_STORE;
#endif

  Int     refOffset  = (mv->getHor() >> shiftHor) + (mv->getVer() >> shiftVer) * refStride;

  Pel*    ref     = refPic->getAddr(compID, cu->getCtuRsAddr(), cu->getZorderIdxInCtu() + partAddr ) + refOffset;

  Pel*    dst = dstPic->getAddr( compID, partAddr );
#if VCEG_AZ07_FRUC_MERGE
  if( nFRUCMode )
  {
    dst = dstPic->getAddr( compID , 0 );
  }
#endif

  Int     xFrac  = mv->getHor() & ((1<<shiftHor)-1);
  Int     yFrac  = mv->getVer() & ((1<<shiftVer)-1);
  UInt    cxWidth  = width  >> refPic->getComponentScaleX(compID);
  UInt    cxHeight = height >> refPic->getComponentScaleY(compID);

  const ChromaFormat chFmt = cu->getPic()->getChromaFormat();
#if VCEG_AZ05_BIO 
  if ( bBIOapplied)
  { 
#if JVET_F0028_BIO_NO_BLOCK_EXTENTION // no block extension is needed
    Pel* pGradY = m_pGradY0;  Pel* pGradX = m_pGradX0;
    Int iWidthG = width;
    Int iHeightG = height;
#else
    Pel* pGradY= m_pGradY0;  Pel* pGradX= m_pGradX0;  Pel *pPred= m_pPred0;
    Int iWidthG   = width + 4;
    Int iHeightG = height + 4;
#endif
    if (iRefListIdx == 0)
    {    
      pGradY = m_pGradY0;
      pGradX = m_pGradX0;
#if !JVET_F0028_BIO_NO_BLOCK_EXTENTION
      pPred = m_pPred0;
#endif
    }
    else
    {
      pGradY = m_pGradY1;
      pGradX = m_pGradX1;
#if !JVET_F0028_BIO_NO_BLOCK_EXTENTION
      pPred  = m_pPred1 ;
#endif
    }
#if !JVET_F0028_BIO_NO_BLOCK_EXTENTION
    ref -=(2+2*refStride);
#endif
#if JVET_B058_HIGH_PRECISION_MOTION_VECTOR_MC && !JVET_C0027_BIO
    xGradFilterY(ref , refStride,pGradY,iWidthG,iWidthG,iHeightG, yFrac>>VCEG_AZ07_MV_ADD_PRECISION_BIT_FOR_STORE,  xFrac>>VCEG_AZ07_MV_ADD_PRECISION_BIT_FOR_STORE,  bitDepth);
    xGradFilterX(ref , refStride,pGradX,iWidthG,iWidthG,iHeightG, yFrac>>VCEG_AZ07_MV_ADD_PRECISION_BIT_FOR_STORE,  xFrac>>VCEG_AZ07_MV_ADD_PRECISION_BIT_FOR_STORE,  bitDepth);
#else
#if VCEG_AZ07_MV_ADD_PRECISION_BIT_FOR_STORE == 1 && !JVET_C0027_BIO
    xGradFilterY(ref , refStride,pGradY,iWidthG,iWidthG,iHeightG, yFrac>>1,  xFrac>>1,  bitDepth);
    xGradFilterX(ref , refStride,pGradX,iWidthG,iWidthG,iHeightG, yFrac>>1,  xFrac>>1,  bitDepth);
#else
    xGradFilterY(ref , refStride,pGradY,iWidthG,iWidthG,iHeightG, yFrac,  xFrac,  bitDepth);
    xGradFilterX(ref , refStride,pGradX,iWidthG,iWidthG,iHeightG, yFrac,  xFrac,  bitDepth);
#endif
#endif
#if !JVET_F0028_BIO_NO_BLOCK_EXTENTION // no separate function is needed, BIO uses the MC predicion function and same buffers
    xPredInterFrac( ref , pPred, iWidthG, refStride, xFrac, yFrac, iWidthG, iHeightG,bi, chFmt,  bitDepth);
    ref +=(2+2*refStride);
#endif
  }
#if !JVET_F0028_BIO_NO_BLOCK_EXTENTION
  else
  {   
#endif
#endif
  if ( yFrac == 0 )
  {
    m_if.filterHor(compID, ref, refStride, dst,  dstStride, cxWidth, cxHeight, xFrac, 
#if VCEG_AZ06_IC
      !bi || bICFlag,
#else
      !bi,
#endif
      chFmt, bitDepth
#if VCEG_AZ07_FRUC_MERGE
      , nFilterIdx
#endif
      );
  }
  else if ( xFrac == 0 )
  {
    m_if.filterVer(compID, ref, refStride, dst, dstStride, cxWidth, cxHeight, yFrac, true, 
#if VCEG_AZ06_IC
      !bi || bICFlag,
#else
      !bi,
#endif
      chFmt, bitDepth
#if VCEG_AZ07_FRUC_MERGE
      , nFilterIdx
#endif
      );
  }
  else
  {
    Int   tmpStride = m_filteredBlockTmp[0].getStride(compID);
    Pel*  tmp       = m_filteredBlockTmp[0].getAddr(compID);

#if VCEG_AZ07_FRUC_MERGE
    Int vFilterSize = isLuma(compID) ? NTAPS_LUMA : NTAPS_CHROMA;
    if( isLuma(compID) && nFilterIdx == 1 )
    {
      vFilterSize = NTAPS_LUMA_FRUC;
    }
#else
    const Int vFilterSize = isLuma(compID) ? NTAPS_LUMA : NTAPS_CHROMA;
#endif

    m_if.filterHor(compID, ref - ((vFilterSize>>1) -1)*refStride, refStride, tmp, tmpStride, cxWidth, cxHeight+vFilterSize-1, xFrac, false,      chFmt, bitDepth
#if VCEG_AZ07_FRUC_MERGE
      , nFilterIdx
#endif
      );
    m_if.filterVer(compID, tmp + ((vFilterSize>>1) -1)*tmpStride, tmpStride, dst, dstStride, cxWidth, cxHeight,               yFrac, false, 
#if VCEG_AZ06_IC
      !bi || bICFlag,
#else
      !bi,
#endif
      chFmt, bitDepth
#if VCEG_AZ07_FRUC_MERGE
      , nFilterIdx
#endif
      );
  }
#if VCEG_AZ05_BIO && !JVET_F0028_BIO_NO_BLOCK_EXTENTION
  }
#endif
#if VCEG_AZ06_IC
  if( bICFlag )
  {
    Int a, b, i, j;
    const Int iShift = m_ICConstShift;
    xGetLLSICPrediction( cu, mv, refPic, a, b, compID, bitDepth );
    
    dst = dstPic->getAddr( compID, partAddr );

    for ( i = 0; i < cxHeight; i++ )
    {
      for ( j = 0; j < cxWidth; j++ )
      {
#if JVET_D0033_ADAPTIVE_CLIPPING
          dst[j] = ClipA( ( ( a*dst[j] ) >> iShift ) + b ,  compID);
#else
        dst[j] = Clip3( 0, ( 1 << bitDepth ) - 1, ( ( a*dst[j] ) >> iShift ) + b );
#endif
      }
      dst += dstStride;
    }

    if(bi)
    {
      Pel *dst2      = dstPic->getAddr( compID, partAddr );
      Int shift = IF_INTERNAL_PREC - bitDepth;
      for (i = 0; i < cxHeight; i++)
      {
        for (j = 0; j < cxWidth; j++)
        {
          Short val = dst2[j] << shift;
          dst2[j] = val - (Short)IF_INTERNAL_OFFS;
        }
        dst2 += dstStride;
      }
    }
  }
#endif
}
#if VCEG_AZ05_BIO && !JVET_F0028_BIO_NO_BLOCK_EXTENTION // no separate function is needed, BIO uses the MC predicion function and same buffers
Void TComPrediction::xPredInterFrac(Pel* ref,Pel* dst,Int dstStride,Int refStride,Int xFrac,Int yFrac,Int width, Int height,Bool bi ,ChromaFormat chFmt, const Int bitDepth )
{
  if ( yFrac == 0 )
  {
    m_if.filterHor(COMPONENT_Y, ref, refStride, dst,  dstStride, width, height, xFrac,       !bi,  chFmt, bitDepth );
  }
  else if ( xFrac == 0 )
  {
    m_if.filterVer(COMPONENT_Y, ref, refStride, dst, dstStride, width, height, yFrac, true,   !bi,  chFmt, bitDepth);
  }
  else
  {
    Int   tmpStride = m_filteredBlockTmp[0].getStride(COMPONENT_Y);
    Pel*  tmp       = m_filteredBlockTmp[0].getAddr(COMPONENT_Y);

    const Int vFilterSize = isLuma(COMPONENT_Y) ? NTAPS_LUMA : NTAPS_CHROMA;

    m_if.filterHor(COMPONENT_Y, ref - ((vFilterSize>>1) -1)*refStride, refStride, tmp, tmpStride, width, height+vFilterSize-1, xFrac, false,           chFmt, bitDepth);
    m_if.filterVer(COMPONENT_Y, tmp + ((vFilterSize>>1) -1)*tmpStride, tmpStride, dst, dstStride, width, height,               yFrac, false, !bi,      chFmt, bitDepth);
  }
}
#endif

#if JVET_G0082

__inline Void TComPrediction::calcBlkGradient(Int sx, Int sy, Int64 *arraysGx2, Int64 *arraysGxGy, Int64 *arraysGxdI, Int64 *arraysGy2, Int64 *arraysGydI,
  Int64 &sGx2, Int64 &sGy2, Int64 &sGxGy, Int64 &sGxdI, Int64 &sGydI, Int iWidth, Int iHeight)
{
  static const UInt weightTbl[8][8] = { { 1, 2, 3, 4, 4, 3, 2, 1 },
    { 2, 4, 6, 8, 8, 6, 4, 2 },
    { 3, 6, 9, 12, 12, 9, 6, 3 },
    { 4, 8, 12, 16, 16, 12, 8, 4 },
    { 4, 8, 12, 16, 16, 12, 8, 4 },
    { 3, 6, 9, 12, 12, 9, 6, 3 },
    { 2, 4, 6, 8, 8, 6, 4, 2 },
    { 1, 2, 3, 4, 4, 3, 2, 1 } };

  Int64 *pGx2 = arraysGx2;
  Int64 *pGy2 = arraysGy2;
  Int64 *pGxGy = arraysGxGy;
  Int64 *pGxdI = arraysGxdI;
  Int64 *pGydI = arraysGydI;

#if JVET_F0028_BIO_NO_BLOCK_EXTENTION
  Int x0;

  if (sy > 0 && iHeight > 4)
  {
    pGx2 -= (iWidth << 1);
    pGy2 -= (iWidth << 1);
    pGxGy -= (iWidth << 1);
    pGxdI -= (iWidth << 1);
    pGydI -= (iWidth << 1);
  }

  for (Int y = -2; y < 6; y++)
  {
    for (Int x = -2; x < 6; x++)
    {
      UInt weight = weightTbl[y + 2][x + 2];
      x0 = x;
      if (sx + x < 0)       x0 = 0;
      if (sx + x >= iWidth) x0 = 3;

      sGx2 += weight*pGx2[x0];
      sGy2 += weight*pGy2[x0];
      sGxGy += weight*pGxGy[x0];
      sGxdI += weight*pGxdI[x0];
      sGydI += weight*pGydI[x0];
    }

    if (sy + y < 0 || sy + y >= iHeight - 1)
    {
      continue;
    }

    pGx2 += iWidth;
    pGy2 += iWidth;
    pGxGy += iWidth;
    pGxdI += iWidth;
    pGydI += iWidth;
  }
#else
  for (Int y = 0; y < 8; y++)
  {
    for (Int x = 0; x < 8; x++)
    {
      UInt weight = weightTbl[y][x];

      sGx2 += weight*pGx2[x];
      sGy2 += weight*pGy2[x];
      sGxGy += weight*pGxGy[x];
      sGxdI += weight*pGxdI[x];
      sGydI += weight*pGydI[x];
    }
    pGx2 += iWidth;
    pGy2 += iWidth;
    pGxGy += iWidth;
    pGxdI += iWidth;
    pGydI += iWidth;
  }
#endif

}

#endif

#if VCEG_AZ06_IC
/** Function for deriving the position of first non-zero binary bit of a value
 * \param x input value
 *
 * This function derives the position of first non-zero binary bit of a value
 */
Int GetMSB( UInt x )
{
  Int iMSB = 0, bits = ( sizeof( Int ) << 3 ), y = 1;

  while( x > 1 )
  {
    bits >>= 1;
    y = x >> bits;

    if( y )
    {
      x = y;
      iMSB += bits;
    }
  }

  iMSB+=y;

  return iMSB;
}

/** Function for deriving LM illumination compensation.
 */
Void TComPrediction::xGetLLSICPrediction( TComDataCU* pcCU, TComMv *pMv, TComPicYuv *pRefPic, Int &a, Int &b, const ComponentID eComp, Int nBitDepth )
{
  TComPicYuv *pRecPic = pcCU->getPic()->getPicYuvRec();
  Pel *pRec = NULL, *pRef = NULL;
  UInt uiWidth, uiTmpPartIdx;
  Int iRecStride = pRecPic->getStride( eComp );
  Int iRefStride = pRefPic->getStride( eComp );
  Int iRefOffset, iRecOffset, iHor, iVer;
  Int shiftHor=(2+pRefPic->getComponentScaleX(eComp));
  Int shiftVer=(2+pRefPic->getComponentScaleY(eComp));
#if VCEG_AZ07_MV_ADD_PRECISION_BIT_FOR_STORE
  shiftHor += VCEG_AZ07_MV_ADD_PRECISION_BIT_FOR_STORE;
  shiftVer += VCEG_AZ07_MV_ADD_PRECISION_BIT_FOR_STORE;
#endif

  iHor = ( pMv->getHor() + (1<<(shiftHor-1)) ) >> shiftHor;
  iVer = ( pMv->getVer() + (1<<(shiftVer-1)) ) >> shiftVer;
  uiWidth  = ( eComp == COMPONENT_Y ) ? pcCU->getWidth( 0 )  : ( pcCU->getWidth( 0 )  >> 1 );
#if JVET_C0024_QTBT
  UInt uiHeight = ( eComp == COMPONENT_Y ) ? pcCU->getHeight( 0 )  : ( pcCU->getHeight( 0 )  >> 1 );
#endif
  Int j, iCountShift = 0;

  // LLS parameters estimation -->

  Int x = 0, y = 0, xx = 0, xy = 0;
  Int precShift = std::max( 0, ( nBitDepth - 12 ) );
  Int iTmpRec, iTmpRef;
  Int iRefStep, iRecStep;
#if  JVET_C0024_QTBT
  UInt uiStep = min(uiWidth, uiHeight) > 8 ? 2 : 1;
#else
  UInt uiStep = 2;//uiWidth > 8 ? 2 : 1;
#endif
  TComDataCU* pNeigCu = NULL;
  TComMv cMv;
  Int iMaxNumMinus1 = 30 - 2*min( nBitDepth, 12 ) - 1;
#if  JVET_C0024_QTBT
  while( min(uiWidth, uiHeight)/uiStep > ( 1 << iMaxNumMinus1 ) ) //make sure log2(2*uiWidth/uiStep) + 2*min(g_bitDepthY, 12) <= 30
#else
  while( uiWidth/uiStep > ( 1 << iMaxNumMinus1 ) ) //make sure log2(2*uiWidth/uiStep) + 2*min(g_bitDepthY, 12) <= 30
#endif
  {
    uiStep <<= 1;
  }

#if  JVET_C0024_QTBT
  UInt uiStepX = uiStep, uiStepY = uiStep;
  if (uiWidth > uiHeight)
  {
    uiStepX  *= uiWidth/uiHeight;
  }
  else
  {
    uiStepY  *= uiHeight/uiWidth;
  }
  Int oriStep = uiStep;
#endif

  for( Int iDir = 0; iDir < 2; iDir++ ) //iDir: 0 - above, 1 - left
  {
    if( !iDir )
    {
      pNeigCu = pcCU->getPUAbove( uiTmpPartIdx, pcCU->getZorderIdxInCtu() );

#if  JVET_C0024_QTBT
      uiStep = uiStepX;
#endif
    }
    else
    {
      pNeigCu =  pcCU->getPULeft( uiTmpPartIdx, pcCU->getZorderIdxInCtu() );
#if  JVET_C0024_QTBT
      uiStep = uiStepY;
#endif
    }

    if( pNeigCu == NULL )
    {
      continue;
    }

    cMv.setHor( iHor << shiftHor ); cMv.setVer( iVer << shiftVer );
    pNeigCu->clipMv( cMv );

    if( iDir )
    {
      iRefOffset = ( cMv.getHor() >> shiftHor ) + ( cMv.getVer() >> shiftVer ) * iRefStride - 1;
      iRecOffset = -1;
      iRefStep   = iRefStride*uiStep;
      iRecStep   = iRecStride*uiStep;
    }
    else
    {
      iRefOffset = ( cMv.getHor() >> shiftHor ) + ( cMv.getVer() >> shiftVer ) * iRefStride - iRefStride;
      iRecOffset = -iRecStride;
      iRefStep   = uiStep;
      iRecStep   = uiStep;
    }

    pRef = pRefPic->getAddr( eComp, pcCU->getCtuRsAddr(), pcCU->getZorderIdxInCtu() ) + iRefOffset;
    pRec = pRecPic->getAddr( eComp, pcCU->getCtuRsAddr(), pcCU->getZorderIdxInCtu() ) + iRecOffset;

#if JVET_C0024_QTBT
    for( j = 0; j < (iDir==0 ? uiWidth: uiHeight); j+=uiStep )
#else
    for( j = 0; j < uiWidth; j+=uiStep )
#endif
    {
      iTmpRef = pRef[0] >> precShift;
      iTmpRec = pRec[0] >> precShift;

      x  += iTmpRef;
      y  += iTmpRec;
      xx += iTmpRef*iTmpRef;
      xy += iTmpRef*iTmpRec;

      pRef += iRefStep;
      pRec += iRecStep;
    }

#if JVET_C0024_QTBT
    iCountShift += ( iCountShift ? 1 : g_aucConvertToBit[ min(uiWidth, uiHeight)/oriStep ] + MIN_CU_LOG2 );
#else
    iCountShift += ( iCountShift ? 1 : g_aucConvertToBit[ uiWidth/uiStep ] + 2 );
#endif
  }

  if( iCountShift == 0 )
  {
    a = ( 1 << m_ICConstShift );
    b = 0;
    return;
  }

  xy += xx >> m_ICRegCostShift;
  xx += xx >> m_ICRegCostShift;

  Int  iCropShift = max( 0, ( nBitDepth - precShift + iCountShift ) - 15 );
  Int  x1 = x, y1 = y;

  x  >>= iCropShift;
  y  >>= iCropShift;
  xy >>= ( iCropShift << 1 );
  xx >>= ( iCropShift << 1 );

  Int a1 = ( xy << iCountShift ) - ( y * x );
  Int a2 = ( xx << iCountShift ) - ( x * x );

  x = x1 << precShift;
  y = y1 << precShift;

  const Int iShift = m_ICConstShift;
  const Int iShiftA2 = 6;
  const Int iAccuracyShift = 15;
  Int iScaleShiftA2 = 0;
  Int iScaleShiftA1 = 0;
  Int a1s = a1;
  Int a2s = a2;

  iScaleShiftA2 = GetMSB( abs( a2 ) ) - iShiftA2;
  iScaleShiftA1 = iScaleShiftA2 - m_ICShiftDiff;

  if( iScaleShiftA1 < 0 )
  {
    iScaleShiftA1 = 0;
  }

  if( iScaleShiftA2 < 0 )
  {
    iScaleShiftA2 = 0;
  }

  Int iScaleShiftA = iScaleShiftA2 + iAccuracyShift - iShift - iScaleShiftA1;

  a2s = a2 >> iScaleShiftA2;
  a1s = a1 >> iScaleShiftA1;

  a2s = Clip3( 0 , 63, a2s );
  Int64 aI64 = ( (Int64) a1s * m_uiaICShift[ a2s ] ) >> iScaleShiftA;
  a = (Int) aI64;
  a = Clip3( 0, 1 << ( iShift + 2 ), a );
  b = (  y - ( ( a * x ) >> iShift ) + ( 1 << ( iCountShift - 1 ) ) ) >> iCountShift;
  Int iOffset = 1 << ( nBitDepth - 1 );
  b = Clip3( -iOffset, iOffset - 1, b );
}
#endif
#if VCEG_AZ05_BIO
Pel optical_flow_averaging( Int64 s1,Int64 s2,Int64 s3,Int64 s5,Int64 s6,
  Pel pGradX0 , Pel pGradX1,Pel pGradY0 , Pel pGradY1,
  Pel pSrcY0Temp, Pel pSrcY1Temp, 
  const int shiftNum , const int  offset     , const Int64 limit ,   
  const Int64 denom_min_1 ,   const Int64 denom_min_2,    
  const Int bitDepth)
{
  Int64 vx = 0;  Int64 vy = 0;
  Int64 b=0;

  if (s1>denom_min_1)         
  {
    vx =  s3  / s1;
    vx =vx>limit?limit:vx<-limit?-limit:vx; 
  }
  if (s5>denom_min_2)
  {
    vy = (s6-vx*s2)/ s5;
    vy =vy>limit?limit:vy<-limit?-limit:vy; 
  }

  b = vx * (pGradX0 - pGradX1) +vy * (pGradY0 - pGradY1);       
  b = (b>0)?((b +32)>> 6):(-((-b+32) >> 6));
#if JVET_D0033_ADAPTIVE_CLIPPING
  return( ClipA((Short)((pSrcY0Temp + pSrcY1Temp + b +offset) >> shiftNum),COMPONENT_Y) );
#else
  return( ClipBD((Short)((pSrcY0Temp + pSrcY1Temp + b +offset) >> shiftNum),bitDepth));
#endif
}
#endif

#if JVET_E0052_DMVR
#if DMVR_HALF_ME
Void TComPrediction::xGenerateFracPixel(TComDataCU* pcCU, UInt uiAbsPartIdx, RefPicList eRefPicList, Int iWidth, Int iHeight, UInt nSearchStepShift)
{
  TComPicYuv* pRefPic = pcCU->getSlice()->getRefPic(eRefPicList, pcCU->getCUMvField(eRefPicList)->getRefIdx(uiAbsPartIdx))->getPicYuvRec();
  TComMv cMvOrg = pcCU->getCUMvField(eRefPicList)->getMv(uiAbsPartIdx);

  //(0,-/+1)
  TComMv cMv = cMvOrg;
  TComMv mvOffset = TComMv(0, -1);
  mvOffset <<= nSearchStepShift;
  cMv += mvOffset;
  xPredInterBlk(COMPONENT_Y, pcCU, pRefPic, uiAbsPartIdx, &cMv, iWidth, iHeight + 1, &(m_filteredBlock[0][1]), false, pcCU->getSlice()->getSPS()->getBitDepth(CHANNEL_TYPE_LUMA));

  //(-/+1,0)
  cMv = cMvOrg;
  mvOffset = TComMv(-1, 0);
  mvOffset <<= nSearchStepShift;
  cMv += mvOffset;
  xPredInterBlk(COMPONENT_Y, pcCU, pRefPic, uiAbsPartIdx, &cMv, iWidth + 1, iHeight, &(m_filteredBlock[1][0]), false, pcCU->getSlice()->getSPS()->getBitDepth(CHANNEL_TYPE_LUMA));

  //(-/+1,-/+1)
  cMv = cMvOrg;
  mvOffset = TComMv(-1, -1);
  mvOffset <<= nSearchStepShift;
  cMv += mvOffset;
  xPredInterBlk(COMPONENT_Y, pcCU, pRefPic, uiAbsPartIdx, &cMv, iWidth + 1, iHeight + 1, &(m_filteredBlock[1][1]), false, pcCU->getSlice()->getSPS()->getBitDepth(CHANNEL_TYPE_LUMA));
}
#endif

Void TComPrediction::xFillPredBorder(TComDataCU* pcCU, UInt uiAbsPartIdx, RefPicList eRefPicList, Int iWidth, Int iHeight, TComYuv* pDstYuv)
{
  TComPicYuv* pRefPic = pcCU->getSlice()->getRefPic(eRefPicList, pcCU->getCUMvField(eRefPicList)->getRefIdx(uiAbsPartIdx))->getPicYuvRec();
  TComMv cMvOrg = pcCU->getCUMvField(eRefPicList)->getMv(uiAbsPartIdx);
  Pel* pDst = pDstYuv->getAddr(COMPONENT_Y);
  Int iDstStride = pDstYuv->getStride(COMPONENT_Y);

  Int iMvShift = 2 + VCEG_AZ07_MV_ADD_PRECISION_BIT_FOR_STORE;
  TComMv cMv = cMvOrg;

  //top row
  cMv += TComMv(-(DMVR_INTME_RANGE << iMvShift), -(DMVR_INTME_RANGE << iMvShift));
  Int bitDepth = pcCU->getSlice()->getSPS()->getBitDepth(CHANNEL_TYPE_LUMA);
  xPredInterLines(pcCU, pRefPic, uiAbsPartIdx, &cMv, iWidth + 2 * DMVR_INTME_RANGE, DMVR_INTME_RANGE, pDst, iDstStride, false, bitDepth);

  //left column
  cMv = cMvOrg;
  cMv += TComMv(-(DMVR_INTME_RANGE << iMvShift), 0);
  xPredInterLines(pcCU, pRefPic, uiAbsPartIdx, &cMv, DMVR_INTME_RANGE, iHeight, pDst + iDstStride*DMVR_INTME_RANGE, iDstStride, false, bitDepth);

  //right column
  cMv = cMvOrg;
  cMv += TComMv((iWidth << iMvShift), 0);
  xPredInterLines(pcCU, pRefPic, uiAbsPartIdx, &cMv, DMVR_INTME_RANGE, iHeight, pDst + iDstStride*DMVR_INTME_RANGE + iWidth + DMVR_INTME_RANGE, iDstStride, false, bitDepth);

  //bottom row
  cMv = cMvOrg;
  cMv += TComMv(-(DMVR_INTME_RANGE << iMvShift), (iHeight << iMvShift));
  xPredInterLines(pcCU, pRefPic, uiAbsPartIdx, &cMv, iWidth + 2 * DMVR_INTME_RANGE, DMVR_INTME_RANGE, pDst + iDstStride*(iHeight + DMVR_INTME_RANGE), iDstStride, false, bitDepth);
}

UInt TComPrediction::xDirectMCCost(Int iBitDepth, Pel* pRef, UInt uiRefStride, Pel* pOrg, UInt uiOrgStride, Int iWidth, Int iHeight)
{
  DistParam cDistParam;
  cDistParam.bApplyWeight = false;
  m_cFRUCRDCost.setDistParam(cDistParam, iBitDepth, pRef, uiRefStride,
    pOrg, uiOrgStride, iWidth, iHeight, false);
#if VCEG_AZ06_IC
  cDistParam.bMRFlag = false;
#endif
  UInt uiCost = cDistParam.DistFunc(&cDistParam);
  return uiCost;
}

Void TComPrediction::xBIPMVRefine(TComDataCU* pcCU, UInt uiAbsPartIdx, RefPicList eRefPicList, Int iWidth, Int iHeight, TComYuv* pOrgYuv, TComYuv* pDstYuv, UInt uiMaxSearchRounds, UInt nSearchStepShift, UInt& uiMinCost)
{
  const TComMv mvSearchOffsetSquare[8] = { TComMv(-1 , 1) , TComMv(0 , 1) , TComMv(1 , 1) , TComMv(1 , 0) , TComMv(1 , -1) , TComMv(0 , -1) , TComMv(-1 , -1) , TComMv(-1 , 0) };

  Int nDirectStart = 0, nDirectEnd = 0, nDirectRounding = 0, nDirectMask = 0;
  const TComMv * pSearchOffset;

  nDirectEnd = 7;
  nDirectRounding = 8;
  nDirectMask = 0x07;
  pSearchOffset = mvSearchOffsetSquare;

  TComMv cMvOrg = pcCU->getCUMvField(eRefPicList)->getMv(uiAbsPartIdx);
  TComMv cBestMv = cMvOrg;

  Int nBestDirect;
  for (UInt uiRound = 0; uiRound < uiMaxSearchRounds; uiRound++)
  {
    nBestDirect = -1;
    TComMv cMvCtr = cBestMv;

    for (Int nIdx = nDirectStart; nIdx <= nDirectEnd; nIdx++)
    {
      Int nDirect = (nIdx + nDirectRounding) & nDirectMask;

      TComMv mvOffset = pSearchOffset[nDirect];
      mvOffset <<= nSearchStepShift;

      TComMv cMvTemp = cMvCtr;
      cMvTemp += mvOffset;

      UInt uiCost;

      if (nSearchStepShift == 2 + VCEG_AZ07_MV_ADD_PRECISION_BIT_FOR_STORE)
      {
        TComMv cMvD = cMvTemp;
        cMvD -= cMvOrg;
        cMvD >>= nSearchStepShift;
        assert(cMvD.getAbsHor() <= DMVR_INTME_RANGE && cMvD.getAbsVer() <= DMVR_INTME_RANGE);

        Int iRefStride = m_cYuvPredTemp.getStride(COMPONENT_Y);
        Pel* pRef = m_cYuvPredTemp.getAddrPix(COMPONENT_Y, DMVR_INTME_RANGE + cMvD.getHor(), DMVR_INTME_RANGE + cMvD.getVer());
        uiCost = xDirectMCCost(pcCU->getSlice()->getSPS()->getBitDepth(CHANNEL_TYPE_LUMA), pRef, iRefStride, pOrgYuv->getAddr(COMPONENT_Y, uiAbsPartIdx), pOrgYuv->getStride(COMPONENT_Y), iWidth, iHeight);
      }
      else
      {
        Int iRefStride = m_filteredBlock[0][0].getStride(COMPONENT_Y);
        Pel* pRef = m_filteredBlock[pSearchOffset[nDirect].getAbsHor()][pSearchOffset[nDirect].getAbsVer()].getAddr(COMPONENT_Y, uiAbsPartIdx);
        if (pSearchOffset[nDirect].getHor() == 1)
        {
          pRef++;
        }
        if (pSearchOffset[nDirect].getVer() == 1)
        {
          pRef += iRefStride;
        }
        uiCost = xDirectMCCost(pcCU->getSlice()->getSPS()->getBitDepth(CHANNEL_TYPE_LUMA), pRef, iRefStride, pOrgYuv->getAddr(COMPONENT_Y, uiAbsPartIdx), pOrgYuv->getStride(COMPONENT_Y), iWidth, iHeight);
      }

      if (uiCost < uiMinCost)
      {
        uiMinCost = uiCost;
        nBestDirect = nDirect;
        cBestMv = cMvTemp;
      }
    }

    if (nBestDirect == -1)
      break;
    Int nStep = 2 - (nBestDirect & 0x01);

    nDirectStart = nBestDirect - nStep;
    nDirectEnd = nBestDirect + nStep;
  }

  pcCU->getCUMvField(eRefPicList)->setAllMv(cBestMv, SIZE_2Nx2N, 0, 0);
}
#endif

#if JVET_G0082

__inline Int GetMSB64(UInt64 x)
{
  Int iMSB = 0, bits = (sizeof(Int64) << 3);
  UInt64 y = 1;

  while (x > 1)
  {
    bits >>= 1;
    y = x >> bits;

    if (y)
    {
      x = y;
      iMSB += bits;
    }
  }

  iMSB += (Int)y;

  return iMSB;
}

__inline Int64 TComPrediction::divide64(Int64 numer, Int64 denom)
{
  Int64 d;
  const Int64 iShiftA2 = 6;
  const Int64 iAccuracyShift = 15;
  const Int64 iMaxVal = 63;
  Int64 iScaleShiftA2 = 0;
  Int64 iScaleShiftA1 = 0;

  UChar signA1 = numer < 0;
  UChar signA2 = denom < 0;

  numer = (signA1) ? -numer : numer;
  denom = (signA2) ? -denom : denom;

  iScaleShiftA2 = GetMSB64(denom) - iShiftA2;
  iScaleShiftA1 = iScaleShiftA2 - 12;

  if (iScaleShiftA1 < 0)
  {
    iScaleShiftA1 = 0;
  }

  if (iScaleShiftA2 < 0)
  {
    iScaleShiftA2 = 0;
  }

  Int64 iScaleShiftA = iScaleShiftA2 + iAccuracyShift - iScaleShiftA1;

  Int64 a2s = (denom >> iScaleShiftA2) > iMaxVal ? iMaxVal : (denom >> iScaleShiftA2);
  Int64 a1s = (numer >> iScaleShiftA1);

  Int64 aI64 = (a1s * (Int64)m_uiaBIOShift[a2s]) >> iScaleShiftA;

  d = (signA1 + signA2 == 1) ? -aI64 : aI64;

  return d;
}

#endif

Void TComPrediction::xWeightedAverage( TComYuv* pcYuvSrc0, TComYuv* pcYuvSrc1, Int iRefIdx0, Int iRefIdx1, UInt uiPartIdx, Int iWidth, Int iHeight, TComYuv* pcYuvDst, const BitDepths &clipBitDepths 
#if VCEG_AZ05_BIO                  
  ,bool bBIOapplied
#endif
#if COM16_C1045_BIO_HARMO_IMPROV || JVET_C0027_BIO || JVET_E0052_DMVR
  , TComDataCU * pCu
#endif
#if JVET_E0052_DMVR
  , Bool bRefineflag
#endif
#if JVET_E0052_DMVR || JVET_G0082
  , Bool bOBMC
#endif
)
{
  if( iRefIdx0 >= 0 && iRefIdx1 >= 0 )
  {
#if JVET_G0082
    if (bOBMC)      assert(bBIOapplied == false);
#endif
#if JVET_E0052_DMVR
  Int iPOC0=pCu->getSlice()->getRefPOC(REF_PIC_LIST_0, iRefIdx0);
  Int iPOC1=pCu->getSlice()->getRefPOC(REF_PIC_LIST_1, iRefIdx1);
  Int iPOC = pCu->getSlice()->getPOC();
  Bool bBIPMVRefine = !bOBMC && (iPOC - iPOC0)*(iPOC - iPOC1) < 0 && pCu->getMergeFlag(uiPartIdx) && !pCu->getICFlag(uiPartIdx) && !pCu->getAffineFlag(uiPartIdx) && bRefineflag;
  bBIPMVRefine &= pCu->getMergeType(uiPartIdx) == MGR_TYPE_DEFAULT_N;
  bBIPMVRefine &= !pCu->getFRUCMgrMode(uiPartIdx);
  bBIPMVRefine &= pCu->getSlice()->getSPS()->getUseDMVR();
  if (bBIPMVRefine )
  {
    pcYuvDst->addAvg( pcYuvSrc0, pcYuvSrc1, uiPartIdx, iWidth, iHeight, clipBitDepths
#if VCEG_AZ05_BIO
    , false
#endif
    , true );

    //list 0
    //get init cost
    pcYuvSrc0->toLast(uiPartIdx, iWidth, iHeight, clipBitDepths);
    UInt uiMinCost = xDirectMCCost(pCu->getSlice()->getSPS()->getBitDepth( CHANNEL_TYPE_LUMA ), pcYuvDst->getAddr( COMPONENT_Y , uiPartIdx ) , pcYuvDst->getStride( COMPONENT_Y ) , pcYuvSrc0->getAddr( COMPONENT_Y , uiPartIdx ) , pcYuvSrc0->getStride( COMPONENT_Y ) , iWidth , iHeight);

    //generate/interpolate extended pred with integer pixel interval
    pcYuvSrc0->copyToPartXYComponent(COMPONENT_Y, uiPartIdx, &m_cYuvPredTemp, DMVR_INTME_RANGE, DMVR_INTME_RANGE, iWidth, iHeight);
    xFillPredBorder(pCu, uiPartIdx, REF_PIC_LIST_0, iWidth, iHeight, &m_cYuvPredTemp); 

    //mv refinement
    xBIPMVRefine(pCu, uiPartIdx, REF_PIC_LIST_0, iWidth, iHeight, pcYuvDst, pcYuvSrc0, DMVR_INTME_RANGE, 2+VCEG_AZ07_MV_ADD_PRECISION_BIT_FOR_STORE, uiMinCost);
#if DMVR_HALF_ME
    xGenerateFracPixel(pCu, uiPartIdx, REF_PIC_LIST_0, iWidth, iHeight, 1+VCEG_AZ07_MV_ADD_PRECISION_BIT_FOR_STORE);   
    xBIPMVRefine(pCu, uiPartIdx, REF_PIC_LIST_0, iWidth, iHeight, pcYuvDst, pcYuvSrc0, 1  , 1+VCEG_AZ07_MV_ADD_PRECISION_BIT_FOR_STORE, uiMinCost);
#endif

    //get new prediction
    TComMv cMv = pCu->getCUMvField(REF_PIC_LIST_0)->getMv(uiPartIdx);
    TComPicYuv* pRefPic = pCu->getSlice()->getRefPic(REF_PIC_LIST_0, pCu->getCUMvField(REF_PIC_LIST_0)->getRefIdx(uiPartIdx))->getPicYuvRec();
#if VCEG_AZ05_BIO 
    iRefListIdx = 0;
#endif
    xPredInterBlk( COMPONENT_Y  , pCu , pRefPic , uiPartIdx , &cMv , iWidth , iHeight , pcYuvSrc0 , true , pCu->getSlice()->getSPS()->getBitDepth( CHANNEL_TYPE_LUMA ) 
#if VCEG_AZ05_BIO 
    , bBIOapplied
#endif
    );
    xPredInterBlk( COMPONENT_Cb , pCu , pRefPic , uiPartIdx , &cMv , iWidth , iHeight , pcYuvSrc0 , true , pCu->getSlice()->getSPS()->getBitDepth( CHANNEL_TYPE_CHROMA ));
    xPredInterBlk( COMPONENT_Cr , pCu , pRefPic , uiPartIdx , &cMv , iWidth , iHeight , pcYuvSrc0 , true , pCu->getSlice()->getSPS()->getBitDepth( CHANNEL_TYPE_CHROMA ));

    //list 1
    //get init cost
    pcYuvSrc1->toLast(uiPartIdx, iWidth, iHeight, clipBitDepths);    
    uiMinCost = xDirectMCCost(pCu->getSlice()->getSPS()->getBitDepth( CHANNEL_TYPE_LUMA ), pcYuvDst->getAddr( COMPONENT_Y , uiPartIdx ) , pcYuvDst->getStride( COMPONENT_Y ) ,
    pcYuvSrc1->getAddr( COMPONENT_Y , uiPartIdx ) , pcYuvSrc1->getStride( COMPONENT_Y ) , iWidth , iHeight);

    //generate/interpolate extended pred with integer pixel interval
    pcYuvSrc1->copyToPartXYComponent(COMPONENT_Y, uiPartIdx, &m_cYuvPredTemp, DMVR_INTME_RANGE, DMVR_INTME_RANGE, iWidth, iHeight);
    xFillPredBorder(pCu, uiPartIdx, REF_PIC_LIST_1, iWidth, iHeight, &m_cYuvPredTemp); 
    xBIPMVRefine(pCu, uiPartIdx, REF_PIC_LIST_1, iWidth, iHeight, pcYuvDst, pcYuvSrc1, DMVR_INTME_RANGE, 2+VCEG_AZ07_MV_ADD_PRECISION_BIT_FOR_STORE, uiMinCost);
#if DMVR_HALF_ME
    xGenerateFracPixel(pCu, uiPartIdx, REF_PIC_LIST_1, iWidth, iHeight, 1+VCEG_AZ07_MV_ADD_PRECISION_BIT_FOR_STORE);   
    xBIPMVRefine(pCu, uiPartIdx, REF_PIC_LIST_1, iWidth, iHeight, pcYuvDst, pcYuvSrc1, 1  , 1+VCEG_AZ07_MV_ADD_PRECISION_BIT_FOR_STORE, uiMinCost);
#endif

    //get new prediction
    cMv = pCu->getCUMvField(REF_PIC_LIST_1)->getMv(uiPartIdx);
    pRefPic = pCu->getSlice()->getRefPic(REF_PIC_LIST_1, pCu->getCUMvField(REF_PIC_LIST_1)->getRefIdx(uiPartIdx))->getPicYuvRec();
#if VCEG_AZ05_BIO 
    iRefListIdx = 1;
#endif
    xPredInterBlk( COMPONENT_Y  , pCu , pRefPic , uiPartIdx , &cMv , iWidth , iHeight , pcYuvSrc1 , true , pCu->getSlice()->getSPS()->getBitDepth( CHANNEL_TYPE_LUMA )
#if VCEG_AZ05_BIO 
    , bBIOapplied
#endif
    );  
    xPredInterBlk( COMPONENT_Cb , pCu , pRefPic , uiPartIdx , &cMv , iWidth , iHeight , pcYuvSrc1 , true , pCu->getSlice()->getSPS()->getBitDepth( CHANNEL_TYPE_CHROMA ));
    xPredInterBlk( COMPONENT_Cr , pCu , pRefPic , uiPartIdx , &cMv , iWidth , iHeight , pcYuvSrc1 , true , pCu->getSlice()->getSPS()->getBitDepth( CHANNEL_TYPE_CHROMA ));
}
#endif
#if VCEG_AZ05_BIO 
    if (bBIOapplied)
    {
#if JVET_F0028_BIO_NO_BLOCK_EXTENTION 
#if JVET_G0082
      static Int64 m_piDotProduct1[BIO_TEMP_BUFFER_SIZE];
      static Int64 m_piDotProduct2[BIO_TEMP_BUFFER_SIZE];
      static Int64 m_piDotProduct3[BIO_TEMP_BUFFER_SIZE];
      static Int64 m_piDotProduct5[BIO_TEMP_BUFFER_SIZE];
      static Int64 m_piDotProduct6[BIO_TEMP_BUFFER_SIZE];
#else
      //for weighted averaging implementation few temporal buffers are used
      static Int64 m_piDotProduct1[BIO_TEMP_BUFFER_SIZE]; static Int64 m_piDotProduct1t[BIO_TEMP_BUFFER_SIZE];
      static Int64 m_piDotProduct2[BIO_TEMP_BUFFER_SIZE]; static Int64 m_piDotProduct2t[BIO_TEMP_BUFFER_SIZE];
      static Int64 m_piDotProduct3[BIO_TEMP_BUFFER_SIZE]; static Int64 m_piDotProduct3t[BIO_TEMP_BUFFER_SIZE];
      static Int64 m_piDotProduct5[BIO_TEMP_BUFFER_SIZE]; static Int64 m_piDotProduct5t[BIO_TEMP_BUFFER_SIZE];
      static Int64 m_piDotProduct6[BIO_TEMP_BUFFER_SIZE]; static Int64 m_piDotProduct6t[BIO_TEMP_BUFFER_SIZE];
#endif
#else
#if JVET_G0082
      static Int64 m_piDotProduct1[BIO_TEMP_BUFFER_SIZE];
      static Int64 m_piDotProduct2[BIO_TEMP_BUFFER_SIZE];
      static Int64 m_piDotProduct3[BIO_TEMP_BUFFER_SIZE];
      static Int64 m_piDotProduct5[BIO_TEMP_BUFFER_SIZE];
      static Int64 m_piDotProduct6[BIO_TEMP_BUFFER_SIZE];
#else
      static Int64 m_piDotProduct1[BIO_TEMP_BUFFER_SIZE];
      static Int64 m_piDotProduct2[BIO_TEMP_BUFFER_SIZE];
      static Int64 m_piDotProduct3[BIO_TEMP_BUFFER_SIZE];
      static Int64 m_piDotProduct5[BIO_TEMP_BUFFER_SIZE];
      static Int64 m_piDotProduct6[BIO_TEMP_BUFFER_SIZE];
      static Int64 m_piS1temp[BIO_TEMP_BUFFER_SIZE];
      static Int64 m_piS2temp[BIO_TEMP_BUFFER_SIZE];
      static Int64 m_piS3temp[BIO_TEMP_BUFFER_SIZE];
      static Int64 m_piS5temp[BIO_TEMP_BUFFER_SIZE];
      static Int64 m_piS6temp[BIO_TEMP_BUFFER_SIZE];
      static Int64 m_piS1[BIO_TEMP_BUFFER_SIZE];
      static Int64 m_piS2[BIO_TEMP_BUFFER_SIZE];
      static Int64 m_piS3[BIO_TEMP_BUFFER_SIZE];
      static Int64 m_piS5[BIO_TEMP_BUFFER_SIZE];
      static Int64 m_piS6[BIO_TEMP_BUFFER_SIZE];
#endif
#endif
      Int x=0, y=0;
#if JVET_F0028_BIO_NO_BLOCK_EXTENTION
      Int iHeightG = iHeight;
      Int iWidthG  = iWidth;
#else
      Int iHeightG = iHeight + 4;
      Int iWidthG  = iWidth  + 4;
      Int iStrideTemp = 2+2*iWidthG;
#endif
      Pel* pGradX0 = m_pGradX0; Pel* pGradX1 = m_pGradX1; 
      Pel* pGradY0 = m_pGradY0; Pel* pGradY1 = m_pGradY1;    
#if JVET_F0028_BIO_NO_BLOCK_EXTENTION
      Pel* pSrcY0 = pcYuvSrc0->getAddr(COMPONENT_Y, uiPartIdx);
      Pel* pSrcY1 = pcYuvSrc1->getAddr(COMPONENT_Y, uiPartIdx);
      Int iSrc0Stride = pcYuvSrc0->getStride(COMPONENT_Y);
      Int iSrc1Stride = pcYuvSrc1->getStride(COMPONENT_Y);
#else
      Pel* pSrcY0 = m_pPred0; 
      Pel* pSrcY1 = m_pPred1;
      Int iSrc0Stride = iWidthG;
      Int iSrc1Stride = iWidthG;  
#endif
      Pel* pDstY = pcYuvDst->getAddr(COMPONENT_Y,uiPartIdx); 
      Int iDstStride = pcYuvDst->getStride(COMPONENT_Y); 
      Pel* pSrcY0Temp = pSrcY0; Pel* pSrcY1Temp = pSrcY1;

#if COM16_C1045_BIO_HARMO_IMPROV
      Int dT0 = pCu->getSlice()->getRefPOC( REF_PIC_LIST_0 , iRefIdx0 ) - pCu->getSlice()->getPOC();
      Int dT1 = pCu->getSlice()->getPOC() - pCu->getSlice()->getRefPOC( REF_PIC_LIST_1 , iRefIdx1 );
      if( dT0 * dT1 < 0 )
      {
        Pel * tmpGradX0 = m_pGradX0;
        Pel * tmpGradX1 = m_pGradX1;
        Pel * tmpGradY0 = m_pGradY0;
        Pel * tmpGradY1 = m_pGradY1;
        for( y = 0; y < iHeightG; y++ )    
        {
          for( x =0; x < iWidthG ; x++ )
          {
            tmpGradX0[x] *= dT0;
            tmpGradX1[x] *= dT1;
            tmpGradY0[x] *= dT0;
            tmpGradY1[x] *= dT1;
          }
          tmpGradX0 += iWidthG;
          tmpGradX1 += iWidthG;
          tmpGradY0 += iWidthG;
          tmpGradY1 += iWidthG;
        }
      }
#endif

      static const int  bitDepth =clipBitDepths.recon[toChannelType(COMPONENT_Y)];
      static const int  shiftNum    = IF_INTERNAL_PREC + 1 - bitDepth;
      static const int  offset      = ( 1 << ( shiftNum - 1 ) ) + 2 * IF_INTERNAL_OFFS;
#if JVET_C0027_BIO
      static const bool bShortRefMV =  (pCu->getSlice()->getCheckLDC()
#if COM16_C1045_BIO_HARMO_IMPROV
        && pCu->isBIOLDB(uiPartIdx)
#endif
        );
      static const Int64 limit = (12<<(IF_INTERNAL_PREC -bShortRefMV - bitDepth)); 
#else
      static const Int64 limit = (12<<(IF_INTERNAL_PREC-1- bitDepth)); 
#endif
      static const Int64 regularizator_1 = 500* (1<< (bitDepth-8))* (1<< (bitDepth-8));
      static const Int64 regularizator_2 =regularizator_1<<1;
      static const Int64 denom_min_1 = 700* (1<< (bitDepth-8))* (1<< (bitDepth-8));
      static const Int64 denom_min_2 = denom_min_1<<1;
#if JVET_F0028_BIO_NO_BLOCK_EXTENTION
#if JVET_G0082
      Int64* m_piDotProductTemp1 = m_piDotProduct1;
      Int64* m_piDotProductTemp2 = m_piDotProduct2;
      Int64* m_piDotProductTemp3 = m_piDotProduct3;
      Int64* m_piDotProductTemp5 = m_piDotProduct5;
      Int64* m_piDotProductTemp6 = m_piDotProduct6;
#else
      Int64* m_piDotProductTemp1 = m_piDotProduct1; Int64* m_piDotProductTemp1t = m_piDotProduct1t;
      Int64* m_piDotProductTemp2 = m_piDotProduct2; Int64* m_piDotProductTemp2t = m_piDotProduct2t;
      Int64* m_piDotProductTemp3 = m_piDotProduct3; Int64* m_piDotProductTemp3t = m_piDotProduct3t;
      Int64* m_piDotProductTemp5 = m_piDotProduct5; Int64* m_piDotProductTemp5t = m_piDotProduct5t;
      Int64* m_piDotProductTemp6 = m_piDotProduct6; Int64* m_piDotProductTemp6t = m_piDotProduct6t;
#endif
#else
      Int64* m_piDotProductTemp1 = m_piDotProduct1;Int64* m_piDotProductTemp2 = m_piDotProduct2;Int64* m_piDotProductTemp3 = m_piDotProduct3;Int64* m_piDotProductTemp5 = m_piDotProduct5;Int64* m_piDotProductTemp6 = m_piDotProduct6;
#if !JVET_G0082
      Int64* m_pS1loc=m_piS1temp;Int64* m_pS2loc=m_piS2temp;Int64* m_pS3loc=m_piS3temp;Int64* m_pS5loc=m_piS5temp;Int64* m_pS6loc=m_piS6temp;
      Int64* m_pS1loc_1=m_piS1temp;Int64* m_pS2loc_1=m_piS2temp;Int64* m_pS3loc_1=m_piS3temp;Int64* m_pS5loc_1=m_piS5temp;Int64* m_pS6loc_1=m_piS6temp;
      Int64* m_pS1loc_2=m_piS1temp;Int64* m_pS2loc_2=m_piS2temp;Int64* m_pS3loc_2=m_piS3temp;Int64* m_pS5loc_2=m_piS5temp;Int64* m_pS6loc_2=m_piS6temp;
      Int64* m_pS1loc_3=m_piS1temp;Int64* m_pS2loc_3=m_piS2temp;Int64* m_pS3loc_3=m_piS3temp;Int64* m_pS5loc_3=m_piS5temp;Int64* m_pS6loc_3=m_piS6temp;

      Int64* m_pS1loc1=m_piS1temp;Int64* m_pS2loc1=m_piS2temp;Int64* m_pS3loc1=m_piS3temp;Int64* m_pS5loc1=m_piS5temp;Int64* m_pS6loc1=m_piS6temp;
      Int64* m_pS1loc2=m_piS1temp;Int64* m_pS2loc2=m_piS2temp;Int64* m_pS3loc2=m_piS3temp;Int64* m_pS5loc2=m_piS5temp;Int64* m_pS6loc2=m_piS6temp;
      Int64* m_piSS1loc=m_piS1;Int64* m_piSS2loc=m_piS2;Int64* m_piSS3loc=m_piS3;Int64* m_piSS5loc=m_piS5;Int64* m_piSS6loc=m_piS6;
      Int64* m_piSS1loc_1=m_piS1;Int64* m_piSS2loc_1=m_piS2;Int64* m_piSS3loc_1=m_piS3;Int64* m_piSS5loc_1=m_piS5;Int64* m_piSS6loc_1=m_piS6;
#endif
#endif
      Int64 temp=0, tempX=0, tempY=0;
      for (y = 0; y < iHeightG; y ++)    
      {
        for (x =0; x < iWidthG ; x ++)
        {
          temp =(Int64) (pSrcY0Temp[x ] - pSrcY1Temp[x ]);
          tempX =(Int64) (pGradX0[x] +  pGradX1[x]);
          tempY =(Int64) (pGradY0[x] +  pGradY1[x]);
          m_piDotProductTemp1[x] =  tempX*tempX;
          m_piDotProductTemp2[x] =  tempX*tempY;
          m_piDotProductTemp3[x] = -tempX*temp<<5;
          m_piDotProductTemp5[x] =  tempY*tempY<<1;
          m_piDotProductTemp6[x] = -tempY*temp<<6;
        }
        pSrcY0Temp+=iSrc0Stride;
        pSrcY1Temp+=iSrc1Stride;
        pGradX0+=iWidthG;
        pGradX1+=iWidthG;
        pGradY0+=iWidthG;
        pGradY1+=iWidthG;
        m_piDotProductTemp1+=iWidthG;
        m_piDotProductTemp2+=iWidthG;
        m_piDotProductTemp3+=iWidthG;
        m_piDotProductTemp5+=iWidthG;
        m_piDotProductTemp6+=iWidthG;
      }

#if JVET_G0082
      Int xUnit = (iWidth >> 2);
      Int yUnit = (iHeight >> 2);

      Pel *pDstY0 = pDstY;
      pGradX0 = m_pGradX0; pGradX1 = m_pGradX1;
      pGradY0 = m_pGradY0; pGradY1 = m_pGradY1;

      for (Int yu = 0; yu < yUnit; yu++)
      {
        for (Int xu = 0; xu < xUnit; xu++)
        {
          Int64 sGxdI = 0, sGydI = 0, sGxGy = 0, sGx2 = 0, sGy2 = 0;
          Int64 tmpx = 0, tmpy = 0;

          m_piDotProductTemp1 = m_piDotProduct1 + ((yu*iWidthG + xu) << 2);
          m_piDotProductTemp2 = m_piDotProduct2 + ((yu*iWidthG + xu) << 2);
          m_piDotProductTemp3 = m_piDotProduct3 + ((yu*iWidthG + xu) << 2);
          m_piDotProductTemp5 = m_piDotProduct5 + ((yu*iWidthG + xu) << 2);
          m_piDotProductTemp6 = m_piDotProduct6 + ((yu*iWidthG + xu) << 2);

          calcBlkGradient(xu << 2, yu << 2, m_piDotProductTemp1, m_piDotProductTemp2, m_piDotProductTemp3, m_piDotProductTemp5, m_piDotProductTemp6,
            sGx2, sGy2, sGxGy, sGxdI, sGydI, iWidthG, iHeightG);

          sGxdI >>= 4;
          sGydI >>= 4;
          sGxGy >>= 4;
          sGx2 >>= 4;
          sGy2 >>= 4;

          sGx2 += regularizator_1;
          sGy2 += regularizator_2;

          if (sGx2 > denom_min_1)
          {
            tmpx = divide64(sGxdI, sGx2);
            tmpx = Clip3(-limit, limit, tmpx);
          }
          if (sGy2 > denom_min_2)
          {
            tmpy = divide64((sGydI - tmpx * sGxGy), sGy2);
            tmpy = Clip3(-limit, limit, tmpy);
          }

#if !JVET_F0028_BIO_NO_BLOCK_EXTENTION
          pSrcY0Temp = pSrcY0 + ((yu*iSrc0Stride + xu) << 2) + iStrideTemp;
          pSrcY1Temp = pSrcY1 + ((yu*iSrc0Stride + xu) << 2) + iStrideTemp;
          pGradX0 = m_pGradX0 + ((yu*iWidthG + xu) << 2) + iStrideTemp;
          pGradX1 = m_pGradX1 + ((yu*iWidthG + xu) << 2) + iStrideTemp;
          pGradY0 = m_pGradY0 + ((yu*iWidthG + xu) << 2) + iStrideTemp;
          pGradY1 = m_pGradY1 + ((yu*iWidthG + xu) << 2) + iStrideTemp;
#else
          pSrcY0Temp = pSrcY0 + ((yu*iSrc0Stride + xu) << 2);
          pSrcY1Temp = pSrcY1 + ((yu*iSrc0Stride + xu) << 2);
          pGradX0 = m_pGradX0 + ((yu*iWidthG + xu) << 2);
          pGradX1 = m_pGradX1 + ((yu*iWidthG + xu) << 2);
          pGradY0 = m_pGradY0 + ((yu*iWidthG + xu) << 2);
          pGradY1 = m_pGradY1 + ((yu*iWidthG + xu) << 2);
#endif
          pDstY0 = pDstY + ((yu*iDstStride + xu) << 2);

          // apply BIO offset for the sub-block
          for (y = 0; y < 4; y++)
          {
            for (x = 0; x < 4; x++)
            {
              Int b = (Int)tmpx * (pGradX0[x] - pGradX1[x]) + (Int)tmpy * (pGradY0[x] - pGradY1[x]);
              b = (b > 0) ? ((b + 32) >> 6) : (-((-b + 32) >> 6));

#if JVET_D0033_ADAPTIVE_CLIPPING
              pDstY0[x] = (ClipA((Short)((pSrcY0Temp[x] + pSrcY1Temp[x] + b + offset) >> shiftNum), COMPONENT_Y));
#else
              pDstY0[x] = (ClipBD((Short)((pSrcY0Temp[x] + pSrcY1Temp[x] + b + offset) >> shiftNum), bitDepth));
#endif
            }
            pDstY0 += iDstStride; pSrcY0Temp += iSrc0Stride; pSrcY1Temp += iSrc1Stride;
            pGradX0 += iWidthG; pGradX1 += iWidthG; pGradY0 += iWidthG; pGradY1 += iWidthG;
          }

        }  // xu
      }  // yu

#else

#if JVET_F0028_BIO_NO_BLOCK_EXTENTION
      //major change is here
      //samples out-side of predicted blocks are not used 
      m_piDotProductTemp1 = m_piDotProduct1; m_piDotProductTemp2 = m_piDotProduct2; m_piDotProductTemp3 = m_piDotProduct3; m_piDotProductTemp5 = m_piDotProduct5; m_piDotProductTemp6 = m_piDotProduct6;
      for (int j = 0; j<iHeight; j++)
      {
        int i = 0, i1 = 1, i2 = 2, im1 = 0, im2 = 0;
        m_piDotProductTemp1t[i] = m_piDotProductTemp1[im2] + m_piDotProductTemp1[im1] + (m_piDotProductTemp1[i]) + m_piDotProductTemp1[i1] + m_piDotProductTemp1[i2];
        m_piDotProductTemp2t[i] = m_piDotProductTemp2[im2] + m_piDotProductTemp2[im1] + (m_piDotProductTemp2[i]) + m_piDotProductTemp2[i1] + m_piDotProductTemp2[i2];
        m_piDotProductTemp3t[i] = m_piDotProductTemp3[im2] + m_piDotProductTemp3[im1] + (m_piDotProductTemp3[i]) + m_piDotProductTemp3[i1] + m_piDotProductTemp3[i2];
        m_piDotProductTemp5t[i] = m_piDotProductTemp5[im2] + m_piDotProductTemp5[im1] + (m_piDotProductTemp5[i]) + m_piDotProductTemp5[i1] + m_piDotProductTemp5[i2];
        m_piDotProductTemp6t[i] = m_piDotProductTemp6[im2] + m_piDotProductTemp6[im1] + (m_piDotProductTemp6[i]) + m_piDotProductTemp6[i1] + m_piDotProductTemp6[i2];
        i = 1; i1 = 2; i2 = 3; im1 = 0; im2 = 0;
        m_piDotProductTemp1t[i] = m_piDotProductTemp1[im2] + m_piDotProductTemp1[im1] + (m_piDotProductTemp1[i]) + m_piDotProductTemp1[i1] + m_piDotProductTemp1[i2];
        m_piDotProductTemp2t[i] = m_piDotProductTemp2[im2] + m_piDotProductTemp2[im1] + (m_piDotProductTemp2[i]) + m_piDotProductTemp2[i1] + m_piDotProductTemp2[i2];
        m_piDotProductTemp3t[i] = m_piDotProductTemp3[im2] + m_piDotProductTemp3[im1] + (m_piDotProductTemp3[i]) + m_piDotProductTemp3[i1] + m_piDotProductTemp3[i2];
        m_piDotProductTemp5t[i] = m_piDotProductTemp5[im2] + m_piDotProductTemp5[im1] + (m_piDotProductTemp5[i]) + m_piDotProductTemp5[i1] + m_piDotProductTemp5[i2];
        m_piDotProductTemp6t[i] = m_piDotProductTemp6[im2] + m_piDotProductTemp6[im1] + (m_piDotProductTemp6[i]) + m_piDotProductTemp6[i1] + m_piDotProductTemp6[i2];
        for (i = 2, i1 = 3, i2 = 4, im1 = 1, im2 = 0; i<iWidth - 2; i++, i1++, i2++, im1++, im2++)
        {
          m_piDotProductTemp1t[i] = m_piDotProductTemp1[im2] + m_piDotProductTemp1[im1] + (m_piDotProductTemp1[i]) + m_piDotProductTemp1[i1] + m_piDotProductTemp1[i2];
          m_piDotProductTemp2t[i] = m_piDotProductTemp2[im2] + m_piDotProductTemp2[im1] + (m_piDotProductTemp2[i]) + m_piDotProductTemp2[i1] + m_piDotProductTemp2[i2];
          m_piDotProductTemp3t[i] = m_piDotProductTemp3[im2] + m_piDotProductTemp3[im1] + (m_piDotProductTemp3[i]) + m_piDotProductTemp3[i1] + m_piDotProductTemp3[i2];
          m_piDotProductTemp5t[i] = m_piDotProductTemp5[im2] + m_piDotProductTemp5[im1] + (m_piDotProductTemp5[i]) + m_piDotProductTemp5[i1] + m_piDotProductTemp5[i2];
          m_piDotProductTemp6t[i] = m_piDotProductTemp6[im2] + m_piDotProductTemp6[im1] + (m_piDotProductTemp6[i]) + m_piDotProductTemp6[i1] + m_piDotProductTemp6[i2];
        }
        i = iWidth - 2; i1 = iWidth - 1; i2 = i1; im1 = i - 1; im2 = i - 2;
        m_piDotProductTemp1t[i] = m_piDotProductTemp1[im2] + m_piDotProductTemp1[im1] + (m_piDotProductTemp1[i]) + m_piDotProductTemp1[i1] + m_piDotProductTemp1[i2];
        m_piDotProductTemp2t[i] = m_piDotProductTemp2[im2] + m_piDotProductTemp2[im1] + (m_piDotProductTemp2[i]) + m_piDotProductTemp2[i1] + m_piDotProductTemp2[i2];
        m_piDotProductTemp3t[i] = m_piDotProductTemp3[im2] + m_piDotProductTemp3[im1] + (m_piDotProductTemp3[i]) + m_piDotProductTemp3[i1] + m_piDotProductTemp3[i2];
        m_piDotProductTemp5t[i] = m_piDotProductTemp5[im2] + m_piDotProductTemp5[im1] + (m_piDotProductTemp5[i]) + m_piDotProductTemp5[i1] + m_piDotProductTemp5[i2];
        m_piDotProductTemp6t[i] = m_piDotProductTemp6[im2] + m_piDotProductTemp6[im1] + (m_piDotProductTemp6[i]) + m_piDotProductTemp6[i1] + m_piDotProductTemp6[i2];
        i = iWidth - 1; i1 = i; i2 = i; im1 = i - 1; im2 = i - 2;
        m_piDotProductTemp1t[i] = m_piDotProductTemp1[im2] + m_piDotProductTemp1[im1] + (m_piDotProductTemp1[i]) + m_piDotProductTemp1[i1] + m_piDotProductTemp1[i2];
        m_piDotProductTemp2t[i] = m_piDotProductTemp2[im2] + m_piDotProductTemp2[im1] + (m_piDotProductTemp2[i]) + m_piDotProductTemp2[i1] + m_piDotProductTemp2[i2];
        m_piDotProductTemp3t[i] = m_piDotProductTemp3[im2] + m_piDotProductTemp3[im1] + (m_piDotProductTemp3[i]) + m_piDotProductTemp3[i1] + m_piDotProductTemp3[i2];
        m_piDotProductTemp5t[i] = m_piDotProductTemp5[im2] + m_piDotProductTemp5[im1] + (m_piDotProductTemp5[i]) + m_piDotProductTemp5[i1] + m_piDotProductTemp5[i2];
        m_piDotProductTemp6t[i] = m_piDotProductTemp6[im2] + m_piDotProductTemp6[im1] + (m_piDotProductTemp6[i]) + m_piDotProductTemp6[i1] + m_piDotProductTemp6[i2];

        m_piDotProductTemp1 += iWidth; m_piDotProductTemp1t += iWidth;
        m_piDotProductTemp2 += iWidth; m_piDotProductTemp2t += iWidth;
        m_piDotProductTemp3 += iWidth; m_piDotProductTemp3t += iWidth;
        m_piDotProductTemp5 += iWidth; m_piDotProductTemp5t += iWidth;
        m_piDotProductTemp6 += iWidth; m_piDotProductTemp6t += iWidth;
      }
      m_piDotProductTemp1 = m_piDotProduct1; m_piDotProductTemp2 = m_piDotProduct2; m_piDotProductTemp3 = m_piDotProduct3; m_piDotProductTemp5 = m_piDotProduct5; m_piDotProductTemp6 = m_piDotProduct6;
      m_piDotProductTemp1t = m_piDotProduct1t; m_piDotProductTemp2t = m_piDotProduct2t; m_piDotProductTemp3t = m_piDotProduct3t; m_piDotProductTemp5t = m_piDotProduct5t; m_piDotProductTemp6t = m_piDotProduct6t;

      pGradX0 = m_pGradX0;  pGradX1 = m_pGradX1;
      pGradY0 = m_pGradY0; pGradY1 = m_pGradY1;
      pSrcY0Temp = pSrcY0;
      pSrcY1Temp = pSrcY1;

      Int iWidth2 = iWidth << 1;
      for (int j = 0; j<iWidth; j++)
      {
        int i = 0, i1 = iWidth, i2 = iWidth2, im1 = 0, im2 = 0; int id = 0; int ir0 = 0; int ir1 = 0;
        m_piDotProductTemp1[i] = m_piDotProductTemp1t[im2] + m_piDotProductTemp1t[im1] + (m_piDotProductTemp1t[i]) + m_piDotProductTemp1t[i1] + m_piDotProductTemp1t[i2];
        m_piDotProductTemp2[i] = m_piDotProductTemp2t[im2] + m_piDotProductTemp2t[im1] + (m_piDotProductTemp2t[i]) + m_piDotProductTemp2t[i1] + m_piDotProductTemp2t[i2];
        m_piDotProductTemp3[i] = m_piDotProductTemp3t[im2] + m_piDotProductTemp3t[im1] + (m_piDotProductTemp3t[i]) + m_piDotProductTemp3t[i1] + m_piDotProductTemp3t[i2];
        m_piDotProductTemp5[i] = m_piDotProductTemp5t[im2] + m_piDotProductTemp5t[im1] + (m_piDotProductTemp5t[i]) + m_piDotProductTemp5t[i1] + m_piDotProductTemp5t[i2];
        m_piDotProductTemp6[i] = m_piDotProductTemp6t[im2] + m_piDotProductTemp6t[im1] + (m_piDotProductTemp6t[i]) + m_piDotProductTemp6t[i1] + m_piDotProductTemp6t[i2];

        pDstY[id] = optical_flow_averaging(m_piDotProductTemp1[i] + regularizator_1, m_piDotProductTemp2[i], m_piDotProductTemp3[i], m_piDotProductTemp5[i] + regularizator_2, m_piDotProductTemp6[i],
          pGradX0[i], pGradX1[i], pGradY0[i], pGradY1[i], pSrcY0Temp[ir0], pSrcY1Temp[ir1]
          , shiftNum, offset, limit, denom_min_1, denom_min_2, bitDepth);

        i += iWidth; id += iDstStride; ir0 += iSrc0Stride; ir1 += iSrc1Stride; i1 += iWidth; i2 += iWidth; im1 = 0;
        m_piDotProductTemp1[i] = m_piDotProductTemp1t[im2] + m_piDotProductTemp1t[im1] + (m_piDotProductTemp1t[i]) + m_piDotProductTemp1t[i1] + m_piDotProductTemp1t[i2];
        m_piDotProductTemp2[i] = m_piDotProductTemp2t[im2] + m_piDotProductTemp2t[im1] + (m_piDotProductTemp2t[i]) + m_piDotProductTemp2t[i1] + m_piDotProductTemp2t[i2];
        m_piDotProductTemp3[i] = m_piDotProductTemp3t[im2] + m_piDotProductTemp3t[im1] + (m_piDotProductTemp3t[i]) + m_piDotProductTemp3t[i1] + m_piDotProductTemp3t[i2];
        m_piDotProductTemp5[i] = m_piDotProductTemp5t[im2] + m_piDotProductTemp5t[im1] + (m_piDotProductTemp5t[i]) + m_piDotProductTemp5t[i1] + m_piDotProductTemp5t[i2];
        m_piDotProductTemp6[i] = m_piDotProductTemp6t[im2] + m_piDotProductTemp6t[im1] + (m_piDotProductTemp6t[i]) + m_piDotProductTemp6t[i1] + m_piDotProductTemp6t[i2];

        pDstY[id] = optical_flow_averaging(m_piDotProductTemp1[i] + regularizator_1, m_piDotProductTemp2[i], m_piDotProductTemp3[i], m_piDotProductTemp5[i] + regularizator_2, m_piDotProductTemp6[i],
          pGradX0[i], pGradX1[i], pGradY0[i], pGradY1[i], pSrcY0Temp[ir0], pSrcY1Temp[ir1]
          , shiftNum, offset, limit, denom_min_1, denom_min_2, bitDepth);



        for (i = iWidth2, id = 2 * iDstStride, ir0 = 2 * iSrc0Stride, ir1 = 2 * iSrc1Stride, i1 = i + iWidth, i2 = i1 + iWidth; i<iWidth*(iHeight - 2); i += iWidth, id += iDstStride, ir0 += iSrc0Stride, ir1 += iSrc1Stride, i1 += iWidth, i2 += iWidth, im1 += iWidth, im2 += iWidth)
        {
          m_piDotProductTemp1[i] = m_piDotProductTemp1t[im2] + m_piDotProductTemp1t[im1] + (m_piDotProductTemp1t[i]) + m_piDotProductTemp1t[i1] + m_piDotProductTemp1t[i2];
          m_piDotProductTemp2[i] = m_piDotProductTemp2t[im2] + m_piDotProductTemp2t[im1] + (m_piDotProductTemp2t[i]) + m_piDotProductTemp2t[i1] + m_piDotProductTemp2t[i2];
          m_piDotProductTemp3[i] = m_piDotProductTemp3t[im2] + m_piDotProductTemp3t[im1] + (m_piDotProductTemp3t[i]) + m_piDotProductTemp3t[i1] + m_piDotProductTemp3t[i2];
          m_piDotProductTemp5[i] = m_piDotProductTemp5t[im2] + m_piDotProductTemp5t[im1] + (m_piDotProductTemp5t[i]) + m_piDotProductTemp5t[i1] + m_piDotProductTemp5t[i2];
          m_piDotProductTemp6[i] = m_piDotProductTemp6t[im2] + m_piDotProductTemp6t[im1] + (m_piDotProductTemp6t[i]) + m_piDotProductTemp6t[i1] + m_piDotProductTemp6t[i2];

          pDstY[id] = optical_flow_averaging(m_piDotProductTemp1[i] + regularizator_1, m_piDotProductTemp2[i], m_piDotProductTemp3[i], m_piDotProductTemp5[i] + regularizator_2, m_piDotProductTemp6[i],
            pGradX0[i], pGradX1[i], pGradY0[i], pGradY1[i], pSrcY0Temp[ir0], pSrcY1Temp[ir1]
            , shiftNum, offset, limit, denom_min_1, denom_min_2, bitDepth);
        }

        i = iWidth*(iHeight - 2); id = iDstStride*(iHeight - 2);  ir0 = iSrc0Stride*(iHeight - 2);  ir1 = iSrc1Stride*(iHeight - 2); im2 = i - iWidth2; im1 = i - iWidth; i1 = i + iWidth; i2 = i1;
        m_piDotProductTemp1[i] = m_piDotProductTemp1t[im2] + m_piDotProductTemp1t[im1] + (m_piDotProductTemp1t[i]) + m_piDotProductTemp1t[i1] + m_piDotProductTemp1t[i2];
        m_piDotProductTemp2[i] = m_piDotProductTemp2t[im2] + m_piDotProductTemp2t[im1] + (m_piDotProductTemp2t[i]) + m_piDotProductTemp2t[i1] + m_piDotProductTemp2t[i2];
        m_piDotProductTemp3[i] = m_piDotProductTemp3t[im2] + m_piDotProductTemp3t[im1] + (m_piDotProductTemp3t[i]) + m_piDotProductTemp3t[i1] + m_piDotProductTemp3t[i2];
        m_piDotProductTemp5[i] = m_piDotProductTemp5t[im2] + m_piDotProductTemp5t[im1] + (m_piDotProductTemp5t[i]) + m_piDotProductTemp5t[i1] + m_piDotProductTemp5t[i2];
        m_piDotProductTemp6[i] = m_piDotProductTemp6t[im2] + m_piDotProductTemp6t[im1] + (m_piDotProductTemp6t[i]) + m_piDotProductTemp6t[i1] + m_piDotProductTemp6t[i2];

        pDstY[id] = optical_flow_averaging(m_piDotProductTemp1[i] + regularizator_1, m_piDotProductTemp2[i], m_piDotProductTemp3[i], m_piDotProductTemp5[i] + regularizator_2, m_piDotProductTemp6[i],
          pGradX0[i], pGradX1[i], pGradY0[i], pGradY1[i], pSrcY0Temp[ir0], pSrcY1Temp[ir1]
          , shiftNum, offset, limit, denom_min_1, denom_min_2, bitDepth);

        i = iWidth*(iHeight - 1); id = iDstStride*(iHeight - 1);  ir0 = iSrc0Stride*(iHeight - 1);  ir1 = iSrc1Stride*(iHeight - 1); im2 = i - iWidth2; im1 = i - iWidth; i1 = i; i2 = i;
        m_piDotProductTemp1[i] = m_piDotProductTemp1t[im2] + m_piDotProductTemp1t[im1] + (m_piDotProductTemp1t[i]) + m_piDotProductTemp1t[i1] + m_piDotProductTemp1t[i2];
        m_piDotProductTemp2[i] = m_piDotProductTemp2t[im2] + m_piDotProductTemp2t[im1] + (m_piDotProductTemp2t[i]) + m_piDotProductTemp2t[i1] + m_piDotProductTemp2t[i2];
        m_piDotProductTemp3[i] = m_piDotProductTemp3t[im2] + m_piDotProductTemp3t[im1] + (m_piDotProductTemp3t[i]) + m_piDotProductTemp3t[i1] + m_piDotProductTemp3t[i2];
        m_piDotProductTemp5[i] = m_piDotProductTemp5t[im2] + m_piDotProductTemp5t[im1] + (m_piDotProductTemp5t[i]) + m_piDotProductTemp5t[i1] + m_piDotProductTemp5t[i2];
        m_piDotProductTemp6[i] = m_piDotProductTemp6t[im2] + m_piDotProductTemp6t[im1] + (m_piDotProductTemp6t[i]) + m_piDotProductTemp6t[i1] + m_piDotProductTemp6t[i2];
        pDstY[id] = optical_flow_averaging(m_piDotProductTemp1[i] + regularizator_1, m_piDotProductTemp2[i], m_piDotProductTemp3[i], m_piDotProductTemp5[i] + regularizator_2, m_piDotProductTemp6[i],
          pGradX0[i], pGradX1[i], pGradY0[i], pGradY1[i], pSrcY0Temp[ir0], pSrcY1Temp[ir1]
          , shiftNum, offset, limit, denom_min_1, denom_min_2, bitDepth);

        m_piDotProductTemp1++; m_piDotProductTemp1t++;
        m_piDotProductTemp2++; m_piDotProductTemp2t++;
        m_piDotProductTemp3++; m_piDotProductTemp3t++;
        m_piDotProductTemp5++; m_piDotProductTemp5t++;
        m_piDotProductTemp6++; m_piDotProductTemp6t++;

        pDstY++;
        pGradX0++;  pGradX1++;
        pGradY0++; pGradY1++;
        pSrcY0Temp++;
        pSrcY1Temp++;

      }


#else
      m_piDotProductTemp1 = m_piDotProduct1+2;m_piDotProductTemp2 = m_piDotProduct2+2;m_piDotProductTemp3 = m_piDotProduct3+2;m_piDotProductTemp5 = m_piDotProduct5+2;m_piDotProductTemp6 = m_piDotProduct6+2;
      m_pS1loc=m_piS1temp+2;m_pS2loc=m_piS2temp+2;m_pS3loc=m_piS3temp+2;m_pS5loc=m_piS5temp+2;m_pS6loc=m_piS6temp+2;      

      for (y = 0;y < iHeightG ; y ++)
      {  
        x=0;
        m_pS1loc[x] =  m_piDotProductTemp1[-2] + m_piDotProductTemp1[-1] + m_piDotProductTemp1[0] + m_piDotProductTemp1[1] +  m_piDotProductTemp1[2] ;
        m_pS2loc[x] =  m_piDotProductTemp2[-2] + m_piDotProductTemp2[-1] + m_piDotProductTemp2[0] + m_piDotProductTemp2[1] +  m_piDotProductTemp2[2] ;
        m_pS3loc[x] =  m_piDotProductTemp3[-2] + m_piDotProductTemp3[-1] + m_piDotProductTemp3[0] + m_piDotProductTemp3[1] +  m_piDotProductTemp3[2] ;
        m_pS5loc[x] =  m_piDotProductTemp5[-2] + m_piDotProductTemp5[-1] + m_piDotProductTemp5[0] + m_piDotProductTemp5[1] +  m_piDotProductTemp5[2] ;
        m_pS6loc[x] =  m_piDotProductTemp6[-2] + m_piDotProductTemp6[-1] + m_piDotProductTemp6[0] + m_piDotProductTemp6[1] +  m_piDotProductTemp6[2] ;

        for ( x=1;    x < iWidth  ; x++)
        {
          m_pS1loc[x] =  -m_piDotProductTemp1[x-3]  +  m_piDotProductTemp1[x+2] + m_pS1loc[x-1];
          m_pS2loc[x] =  -m_piDotProductTemp2[x-3]  +  m_piDotProductTemp2[x+2] + m_pS2loc[x-1];  
          m_pS3loc[x] =  -m_piDotProductTemp3[x-3]  +  m_piDotProductTemp3[x+2] + m_pS3loc[x-1];
          m_pS5loc[x] =  -m_piDotProductTemp5[x-3]  +  m_piDotProductTemp5[x+2] + m_pS5loc[x-1];  
          m_pS6loc[x] =  -m_piDotProductTemp6[x-3]  +  m_piDotProductTemp6[x+2] + m_pS6loc[x-1];
        }
        m_piDotProductTemp1+=iWidthG;m_piDotProductTemp2+=iWidthG;m_piDotProductTemp3+=iWidthG;m_piDotProductTemp5+=iWidthG;m_piDotProductTemp6+=iWidthG;
        m_pS1loc+=iWidthG;m_pS2loc+=iWidthG;m_pS3loc+=iWidthG;m_pS5loc+=iWidthG;m_pS6loc+=iWidthG;
      }
      m_pS1loc=m_piS1temp+iStrideTemp;m_pS2loc=m_piS2temp+iStrideTemp;m_pS3loc=m_piS3temp+iStrideTemp;m_pS5loc=m_piS5temp+iStrideTemp;m_pS6loc=m_piS6temp+iStrideTemp;
      m_pS1loc_1=m_pS1loc- iWidthG;m_pS2loc_1=m_pS2loc- iWidthG;m_pS3loc_1=m_pS3loc- iWidthG;m_pS5loc_1=m_pS5loc- iWidthG;m_pS6loc_1=m_pS6loc- iWidthG;
      m_pS1loc_2=m_pS1loc_1- iWidthG;m_pS2loc_2=m_pS2loc_1- iWidthG;m_pS3loc_2=m_pS3loc_1- iWidthG;m_pS5loc_2=m_pS5loc_1- iWidthG;m_pS6loc_2=m_pS6loc_1- iWidthG;
      m_pS1loc1=m_pS1loc+ iWidthG;m_pS2loc1=m_pS2loc+ iWidthG;m_pS3loc1=m_pS3loc+ iWidthG;m_pS5loc1=m_pS5loc+ iWidthG;m_pS6loc1=m_pS6loc+ iWidthG;
      m_pS1loc2=m_pS1loc1+ iWidthG;m_pS2loc2=m_pS2loc1+ iWidthG;m_pS3loc2=m_pS3loc1+ iWidthG;m_pS5loc2=m_pS5loc1+ iWidthG;m_pS6loc2=m_pS6loc1+ iWidthG;
      m_piSS1loc=m_piS1+iStrideTemp;m_piSS2loc=m_piS2+iStrideTemp;m_piSS3loc=m_piS3+iStrideTemp;m_piSS5loc=m_piS5+iStrideTemp;m_piSS6loc=m_piS6+iStrideTemp;
      pGradX0 = m_pGradX0+iStrideTemp ;  pGradX1 = m_pGradX1+iStrideTemp ;
      pGradY0 = m_pGradY0+iStrideTemp  ; pGradY1 = m_pGradY1+iStrideTemp ;
      pSrcY0Temp = pSrcY0 +iStrideTemp;
      pSrcY1Temp = pSrcY1 +iStrideTemp;

      y = 0;
      for (x = 0; x < iWidth ; x ++)
      { 
        m_piSS1loc[x]=m_pS1loc_2[x]+m_pS1loc_1[x]+m_pS1loc[x]+m_pS1loc1[x]+m_pS1loc2[x]+regularizator_1;
        m_piSS2loc[x]=m_pS2loc_2[x]+m_pS2loc_1[x]+m_pS2loc[x]+m_pS2loc1[x]+m_pS2loc2[x];
        m_piSS3loc[x]=m_pS3loc_2[x]+m_pS3loc_1[x]+m_pS3loc[x]+m_pS3loc1[x]+m_pS3loc2[x];
        m_piSS5loc[x]=m_pS5loc_2[x]+m_pS5loc_1[x]+m_pS5loc[x]+m_pS5loc1[x]+m_pS5loc2[x]+regularizator_2;
        m_piSS6loc[x]=m_pS6loc_2[x]+m_pS6loc_1[x]+m_pS6loc[x]+m_pS6loc1[x]+m_pS6loc2[x];
        pDstY[x]=optical_flow_averaging(
          m_piSS1loc[x],m_piSS2loc[x],m_piSS3loc[x],m_piSS5loc[x],m_piSS6loc[x],
          pGradX0[x] , pGradX1[x],pGradY0[x] , pGradY1[x],pSrcY0Temp[x ], pSrcY1Temp[x ]
        ,  shiftNum ,  offset , limit, denom_min_1, denom_min_2,bitDepth);
      }
      m_pS1loc2+=iWidthG;m_pS2loc2+=iWidthG;m_pS3loc2+=iWidthG;m_pS5loc2+=iWidthG;m_pS6loc2+=iWidthG;
      pDstY += iDstStride;pSrcY0Temp+=iSrc0Stride;pSrcY1Temp+=iSrc1Stride;pGradX0+=iWidthG;pGradX1+=iWidthG;pGradY0+=iWidthG;pGradY1+=iWidthG;
      m_piSS1loc_1=m_piSS1loc;m_piSS2loc_1=m_piSS2loc;m_piSS3loc_1=m_piSS3loc;m_piSS5loc_1=m_piSS5loc;m_piSS6loc_1=m_piSS6loc;
      m_pS1loc+=iWidthG;m_pS2loc+=iWidthG;m_pS3loc+=iWidthG;m_pS5loc+=iWidthG;m_pS6loc+=iWidthG;
      m_pS1loc_3=m_pS1loc_2;m_pS2loc_3=m_pS2loc_2;m_pS3loc_3=m_pS3loc_2;m_pS5loc_3=m_pS5loc_2;m_pS6loc_3=m_pS6loc_2;
      m_piSS1loc+=iWidthG;  m_piSS2loc+=iWidthG;  m_piSS3loc+=iWidthG;  m_piSS5loc+=iWidthG;  m_piSS6loc+=iWidthG;
      for (y = 1;  y < iHeight ;   y ++)
      {
        for (x = 0; x < iWidth ; x ++)
        { 
          m_piSS1loc[x]=m_piSS1loc_1[x]-m_pS1loc_3[x]+m_pS1loc2[x];
          m_piSS2loc[x]=m_piSS2loc_1[x]-m_pS2loc_3[x]+m_pS2loc2[x];
          m_piSS3loc[x]=m_piSS3loc_1[x]-m_pS3loc_3[x]+m_pS3loc2[x];
          m_piSS5loc[x]=m_piSS5loc_1[x]-m_pS5loc_3[x]+m_pS5loc2[x];
          m_piSS6loc[x]=m_piSS6loc_1[x]-m_pS6loc_3[x]+m_pS6loc2[x];

          pDstY[x]=optical_flow_averaging(m_piSS1loc[x],m_piSS2loc[x],m_piSS3loc[x],m_piSS5loc[x],m_piSS6loc[x],
            pGradX0[x] , pGradX1[x],pGradY0[x] , pGradY1[x],pSrcY0Temp[x ], pSrcY1Temp[x ]
          ,  shiftNum,  offset, limit, denom_min_1, denom_min_2 ,bitDepth);
        }

        m_piSS1loc_1=m_piSS1loc;m_piSS2loc_1=m_piSS2loc;m_piSS3loc_1=m_piSS3loc;m_piSS5loc_1=m_piSS5loc;m_piSS6loc_1=m_piSS6loc;
        m_piSS1loc+=iWidthG;m_piSS2loc+=iWidthG;m_piSS3loc+=iWidthG;m_piSS5loc+=iWidthG;m_piSS6loc+=iWidthG;
        m_pS1loc2+=iWidthG;m_pS2loc2+=iWidthG;m_pS3loc2+=iWidthG;m_pS5loc2+=iWidthG;m_pS6loc2+=iWidthG;
        m_pS1loc_3+=iWidthG;m_pS2loc_3+=iWidthG;m_pS3loc_3+=iWidthG;m_pS5loc_3+=iWidthG;m_pS6loc_3+=iWidthG;
        pDstY += iDstStride;pSrcY0Temp+=iSrc0Stride;pSrcY1Temp+=iSrc1Stride;
        pGradX0+=iWidthG;pGradX1+=iWidthG;pGradY0+=iWidthG;pGradY1+=iWidthG;
      }
#endif

#endif
    } //bBIOapplied
    pcYuvDst->addAvg( pcYuvSrc0, pcYuvSrc1, uiPartIdx, iWidth, iHeight, clipBitDepths, bBIOapplied);    
#else
    pcYuvDst->addAvg( pcYuvSrc0, pcYuvSrc1, uiPartIdx, iWidth, iHeight, clipBitDepths );
#endif
  }
  else if ( iRefIdx0 >= 0 && iRefIdx1 <  0 )
  {
#if JVET_D0033_ADAPTIVE_CLIPPING
      pcYuvSrc0->clipPartToPartYuv( pcYuvDst, uiPartIdx, iWidth, iHeight );
#else
    pcYuvSrc0->copyPartToPartYuv( pcYuvDst, uiPartIdx, iWidth, iHeight );
#endif
  }
  else if ( iRefIdx0 <  0 && iRefIdx1 >= 0 )
  {
#if JVET_D0033_ADAPTIVE_CLIPPING
    pcYuvSrc1->clipPartToPartYuv(pcYuvDst, uiPartIdx, iWidth, iHeight );
#else
    pcYuvSrc1->copyPartToPartYuv( pcYuvDst, uiPartIdx, iWidth, iHeight );
#endif
  }
}

#if VCEG_AZ05_BIO 
#if JVET_C0027_BIO && JVET_B058_HIGH_PRECISION_MOTION_VECTOR_MC
const Short m_lumaGradientFilter[4<<VCEG_AZ07_MV_ADD_PRECISION_BIT_FOR_STORE][BIO_FILTER_LENGTH] =
{
  {       8,     -39,      -3,      46,     -17,       5 },//0
#if VCEG_AZ07_MV_ADD_PRECISION_BIT_FOR_STORE >= 2 
  {       8,     -32,     -13,      50,     -18,       5,}, //1 -->-->
#endif
#if VCEG_AZ07_MV_ADD_PRECISION_BIT_FOR_STORE >= 1 
  {       7,     -27,     -20,      54,     -19,       5,}, //2-->
#endif
#if VCEG_AZ07_MV_ADD_PRECISION_BIT_FOR_STORE >= 2 
 {       6,     -21,     -29,      57,     -18,       5,}, //3-->-->
#endif
  {       4,     -17,     -36,      60,     -15,       4,},//4
#if VCEG_AZ07_MV_ADD_PRECISION_BIT_FOR_STORE >= 2 
 {       3,      -9,     -44,      61,     -15,       4,},//5-->-->
#endif
#if VCEG_AZ07_MV_ADD_PRECISION_BIT_FOR_STORE >= 1 
 {       1,      -4,     -48,      61,     -13,       3,},//6-->
#endif
  #if VCEG_AZ07_MV_ADD_PRECISION_BIT_FOR_STORE >= 2 
  {       0,       1,     -54,      60,      -9,       2,},//7-->-->
#endif
  {      -1,       4,     -57,      57,      -4,       1 },//8
#if VCEG_AZ07_MV_ADD_PRECISION_BIT_FOR_STORE >= 2 
  {      -2,       9,     -60,      54,      -1,       0,},//9-->-->
#endif
#if VCEG_AZ07_MV_ADD_PRECISION_BIT_FOR_STORE >= 1 
  {      -3,      13,     -61,      48,       4,      -1,},//10-->
#endif
#if VCEG_AZ07_MV_ADD_PRECISION_BIT_FOR_STORE >= 2 
  {      -4,      15,     -61,      44,       9,      -3,},//11-->-->
#endif
  {      -4,      15,     -60,      36,      17,      -4,},//12
#if VCEG_AZ07_MV_ADD_PRECISION_BIT_FOR_STORE >= 2 
  {      -5,      18,     -57,      29,      21,      -6,},//13-->-->
#endif
#if VCEG_AZ07_MV_ADD_PRECISION_BIT_FOR_STORE >= 1 
  {      -5,      19,     -54,      20,      27,      -7,},//14-->
#endif
#if VCEG_AZ07_MV_ADD_PRECISION_BIT_FOR_STORE >= 2 
  {      -5,      18,     -50,      13,      32,      -8,},//15-->-->
#endif
};
const Short m_lumaInterpolationFilter[4<<VCEG_AZ07_MV_ADD_PRECISION_BIT_FOR_STORE][BIO_FILTER_LENGTH] =
{
  {       0,       0,      64,       0,       0,       0,},//0
#if VCEG_AZ07_MV_ADD_PRECISION_BIT_FOR_STORE >= 2 
  {       1,      -3,      64,       4,      -2,       0,},//1 -->-->
#endif
#if VCEG_AZ07_MV_ADD_PRECISION_BIT_FOR_STORE >= 1 
  {       1,      -6,      62,       9,      -3,       1,},//2-->
#endif
#if VCEG_AZ07_MV_ADD_PRECISION_BIT_FOR_STORE >= 2 
  {       2,      -8,      60,      14,      -5,       1,},//3-->-->
#endif
  {       2,      -9,      57,      19,      -7,       2,},//4
#if VCEG_AZ07_MV_ADD_PRECISION_BIT_FOR_STORE >= 2 
  {       3,     -10,      53,      24,      -8,       2,},//5-->-->
#endif
#if VCEG_AZ07_MV_ADD_PRECISION_BIT_FOR_STORE >= 1 
  {       3,     -11,      50,      29,      -9,       2,},//6-->
#endif
#if VCEG_AZ07_MV_ADD_PRECISION_BIT_FOR_STORE >= 2 
  {       3,     -11,      44,      35,     -10,       3,},//7-->-->
#endif
  {       1,      -7,      38,      38,      -7,       1,},//8
#if VCEG_AZ07_MV_ADD_PRECISION_BIT_FOR_STORE >= 2 
 {       3,     -10,      35,      44,     -11,       3,},//9-->-->
#endif
#if VCEG_AZ07_MV_ADD_PRECISION_BIT_FOR_STORE >= 1 
  {       2,      -9,      29,      50,     -11,       3,},//10-->
#endif
#if VCEG_AZ07_MV_ADD_PRECISION_BIT_FOR_STORE >= 2 
  {       2,      -8,      24,      53,     -10,       3,},//11-->-->
#endif
  {       2,      -7,      19,      57,      -9,       2,},//12
#if VCEG_AZ07_MV_ADD_PRECISION_BIT_FOR_STORE >= 2 
  {       1,      -5,      14,      60,      -8,       2,},//13-->-->
#endif
#if VCEG_AZ07_MV_ADD_PRECISION_BIT_FOR_STORE >= 1 
  {       1,      -3,       9,      62,      -6,       1,},//14-->
#endif
#if VCEG_AZ07_MV_ADD_PRECISION_BIT_FOR_STORE >= 2 
  {       0,      -2,       4,      64,      -3,       1,},//15-->-->
#endif
};
#else
const Short m_lumaGradientFilter[4][BIO_FILTER_LENGTH] =
{
  {  8,      -39,       -3,       46,      -17,        5 },
  {  4,      -17,      -36,       60,      -15,        4 },
  { -1,        4,      -57,       57,       -4,        1 },
  { -4,       15,      -60,       36,       17,       -4 }
};
const Short m_lumaInterpolationFilter[4][BIO_FILTER_LENGTH] =
{
  {      0,    0,  64,   0,    0,    0  },
  {      2,   -9,  57,  19,   -7,    2  },
  {      1,   -7,  38,  38,   -7,    1  },
  {      2,   -7,  19,  57,   -9,    2  },
};
#endif

__inline Void TComPrediction::gradFilter2DVer (Pel* piSrc, Int iSrcStride,  Int iWidth, Int iHeight, Int iDstStride,  
  Pel*& rpiDst, Int iMV, const Int iShift)
{
  Pel*   piDst = rpiDst;
  Int   iSum =0;
  Pel*  piSrcTmp  = piSrc-BIO_FILTER_HALF_LENGTH_MINUS_1*iSrcStride;
  Pel*  piSrcTmp1  = piSrcTmp+iSrcStride;
  Pel*  piSrcTmp2  = piSrcTmp1+iSrcStride;
  Pel*  piSrcTmp3  = piSrcTmp2+iSrcStride;
  Pel*  piSrcTmp4  = piSrcTmp3+iSrcStride;
  Pel*  piSrcTmp5  = piSrcTmp4+iSrcStride;

  static const Int iOffSet = iShift>0?(1<<(iShift-1)):0;
  for ( Int y = iHeight; y != 0; y-- )
  {

    for ( Int x = 0; x < iWidth; x++ )
    {
      iSum      = m_lumaGradientFilter[iMV][0]* piSrcTmp[x] 
      +m_lumaGradientFilter[iMV][1]*piSrcTmp1[x]
      +m_lumaGradientFilter[iMV][2]*piSrcTmp2[x]
      +m_lumaGradientFilter[iMV][3]*piSrcTmp3[x] 
      +m_lumaGradientFilter[iMV][4]*piSrcTmp4[x] 
      +m_lumaGradientFilter[iMV][5]*piSrcTmp5[x];
      iSum = (iSum>=0)?((iSum +iOffSet)>>iShift):-(((-iSum +iOffSet)>>iShift));
      piDst[x ] = (Pel) (iSum);
    }
    piSrcTmp+=iSrcStride;
    piSrcTmp1+=iSrcStride;
    piSrcTmp2+=iSrcStride;
    piSrcTmp3+=iSrcStride;
    piSrcTmp4+=iSrcStride;
    piSrcTmp5+=iSrcStride;
    piSrc += iSrcStride;
    piDst += iDstStride;
  }
  return;
}
__inline Void TComPrediction::gradFilter1DVer (Pel* piSrc, Int iSrcStride, Int iWidth, Int iHeight, Int iDstStride, 
  Pel*& rpiDst, Int iMV, const Int iShift)
{
  Pel*  piDst = rpiDst;
  Pel*  piSrcTmp  = piSrc-BIO_FILTER_HALF_LENGTH_MINUS_1*iSrcStride;
  Pel*  piSrcTmp1  = piSrcTmp+iSrcStride;
  Pel*  piSrcTmp2  = piSrcTmp1+iSrcStride;
  Pel*  piSrcTmp3  = piSrcTmp2+iSrcStride;
  Pel*  piSrcTmp4  = piSrcTmp3+iSrcStride;
  Pel*  piSrcTmp5  = piSrcTmp4+iSrcStride;
  Int iSum = 0;    
  static const Int iOffSet = 1<<(iShift-1);  
  for ( Int y = iHeight; y != 0; y-- )
  {
    for ( Int x = 0; x < iWidth; x++ )
    {
      iSum      = m_lumaGradientFilter[iMV][0]* piSrcTmp[x] 
      +m_lumaGradientFilter[iMV][1]*piSrcTmp1[x]
      +m_lumaGradientFilter[iMV][2]*piSrcTmp2[x]
      +m_lumaGradientFilter[iMV][3]*piSrcTmp3[x] 
      +m_lumaGradientFilter[iMV][4]*piSrcTmp4[x] 
      +m_lumaGradientFilter[iMV][5]*piSrcTmp5[x];
      iSum = (iSum>=0)?((iSum +iOffSet)>>iShift):-(((-iSum +iOffSet)>>iShift));
      piDst[x ] = (Pel) (iSum);
    }
    piSrcTmp+=iSrcStride;
    piSrcTmp1+=iSrcStride;
    piSrcTmp2+=iSrcStride;
    piSrcTmp3+=iSrcStride;
    piSrcTmp4+=iSrcStride;
    piSrcTmp5+=iSrcStride;
    piSrc += iSrcStride;
    piDst += iDstStride;
  }
  return;
}
__inline Void TComPrediction::gradFilter1DHor(Pel* piSrc, Int iSrcStride, Int iWidth, Int iHeight, Int iDstStride, 
  Pel*& rpiDst, Int iMV, const Int iShift)
{
  Pel*  piDst    = rpiDst;
  Int   iSum =0;
  Pel*  piSrcTmp;       
  static const Int iOffSet = 1<<(iShift-1);  
  for ( Int y = iHeight; y != 0; y-- )
  {
    piSrcTmp = &piSrc[ -BIO_FILTER_HALF_LENGTH_MINUS_1 ];
    for ( Int x = 0; x < iWidth; x++ )
    {
      iSum      = m_lumaGradientFilter[iMV][0]* piSrcTmp[0] 
      +m_lumaGradientFilter[iMV][1]*piSrcTmp[1]
      +m_lumaGradientFilter[iMV][2]*piSrcTmp[2]
      +m_lumaGradientFilter[iMV][3]*piSrcTmp[3] 
      +m_lumaGradientFilter[iMV][4]*piSrcTmp[4] 
      +m_lumaGradientFilter[iMV][5]*piSrcTmp[5];
      iSum = (iSum>=0)?((iSum +iOffSet)>>iShift):-(((-iSum +iOffSet)>>iShift));
      piDst[x ] = (Pel) (iSum);
      piSrcTmp++;
    }
    piSrc += iSrcStride;
    piDst += iDstStride;
  }

  return;
}
__inline Void TComPrediction::gradFilter2DHor( Pel* piSrc, Int iSrcStride,  Int iWidth, Int iHeight, Int iDstStride,  
  Pel*& rpiDst, Int iMV, const Int iShift)
{
  Pel*  piDst    = rpiDst;
  Int   iSum=0;
  Pel*  piSrcTmp;

  static const Int iOffSet = iShift>0?(1<<(iShift-1)):0;

  for ( Int y = iHeight; y != 0; y-- )
  {
    piSrcTmp = &piSrc[ -BIO_FILTER_HALF_LENGTH_MINUS_1 ];
    for ( Int x = 0; x < iWidth; x++ )
    {

      iSum      = m_lumaGradientFilter[iMV][0]* piSrcTmp[0] 
      +m_lumaGradientFilter[iMV][1]*piSrcTmp[1]
      +m_lumaGradientFilter[iMV][2]*piSrcTmp[2]
      +m_lumaGradientFilter[iMV][3]*piSrcTmp[3] 
      +m_lumaGradientFilter[iMV][4]*piSrcTmp[4] 
      +m_lumaGradientFilter[iMV][5]*piSrcTmp[5];
      iSum = (iSum>=0)?((iSum +iOffSet)>>iShift):-(((-iSum +iOffSet)>>iShift));
      piDst[x ] = (Pel) (iSum);
      piSrcTmp++;
    }
    piSrc += iSrcStride;
    piDst += iDstStride;
  }

  return;
}
__inline Void TComPrediction::fracFilter2DVer (Pel* piSrc, Int iSrcStride,  Int iWidth, Int iHeight, Int iDstStride,  
  Pel*& rpiDst, Int iMV, const Int iShift)
{
  Pel* piDst = rpiDst;
  Int   iSum=0;
  Pel*  piSrcTmp  = piSrc-BIO_FILTER_HALF_LENGTH_MINUS_1*iSrcStride;
  Pel*  piSrcTmp1  = piSrcTmp+iSrcStride;
  Pel*  piSrcTmp2  = piSrcTmp1+iSrcStride;
  Pel*  piSrcTmp3  = piSrcTmp2+iSrcStride;
  Pel*  piSrcTmp4  = piSrcTmp3+iSrcStride;
  Pel*  piSrcTmp5  = piSrcTmp4+iSrcStride;

  static const Int iOffSet = (iShift>0)?((1<<(iShift-1))-(8192<<iShift)):(-8192);

  for ( Int y = iHeight; y != 0; y-- )
  {
    for ( Int x = 0; x < iWidth; x++ )
    {
      iSum      = m_lumaInterpolationFilter[iMV][0]* piSrcTmp[x] 
      +m_lumaInterpolationFilter[iMV][1]*piSrcTmp1[x]
      +m_lumaInterpolationFilter[iMV][2]*piSrcTmp2[x]
      +m_lumaInterpolationFilter[iMV][3]*piSrcTmp3[x] 
      +m_lumaInterpolationFilter[iMV][4]*piSrcTmp4[x] 
      +m_lumaInterpolationFilter[iMV][5]*piSrcTmp5[x];
      iSum = (iSum>=0)?((iSum +iOffSet)>>iShift):-(((-iSum +iOffSet)>>iShift));
      piDst[x ] = (Pel) (iSum);
    }
    piSrcTmp+=iSrcStride;
    piSrcTmp1+=iSrcStride;
    piSrcTmp2+=iSrcStride;
    piSrcTmp3+=iSrcStride;
    piSrcTmp4+=iSrcStride;
    piSrcTmp5+=iSrcStride;
    piSrc += iSrcStride;
    piDst += iDstStride;
  }

  return;
}
__inline Void TComPrediction::fracFilter2DHor( Pel* piSrc, Int iSrcStride,  Int iWidth, Int iHeight, Int iDstStride,  Pel*& rpiDst, Int iMV, const Int iShift)
{
  Pel*  piDst    = rpiDst;
  Int   iSum=0;
  Pel*  piSrcTmp;

  static const Int iOffSet = iShift>0?(1<<(iShift-1)):0;
  for ( Int y = iHeight; y != 0; y-- )
  {
    piSrcTmp = &piSrc[ -BIO_FILTER_HALF_LENGTH_MINUS_1 ];
    for ( Int x = 0; x < iWidth; x++ )
    {
      iSum      = m_lumaInterpolationFilter[iMV][0]* piSrcTmp[0] 
      +m_lumaInterpolationFilter[iMV][1]*piSrcTmp[1]
      +m_lumaInterpolationFilter[iMV][2]*piSrcTmp[2]
      +m_lumaInterpolationFilter[iMV][3]*piSrcTmp[3]
      + m_lumaInterpolationFilter[iMV][4]*piSrcTmp[4] 
      +m_lumaInterpolationFilter[iMV][5]*piSrcTmp[5];
      iSum = (iSum>=0)?((iSum +iOffSet)>>iShift):-(((-iSum +iOffSet)>>iShift));
      piDst[x ] = (Pel) (iSum);
      piSrcTmp++;
    }
    piSrc += iSrcStride;
    piDst += iDstStride;
  }

  return;
}
#endif

// AMVP
Void TComPrediction::getMvPredAMVP( TComDataCU* pcCU, UInt uiPartIdx, UInt uiPartAddr, RefPicList eRefPicList, TComMv& rcMvPred )
{
  AMVPInfo* pcAMVPInfo = pcCU->getCUMvField(eRefPicList)->getAMVPInfo();

  if( pcAMVPInfo->iN <= 1 )
  {
    rcMvPred = pcAMVPInfo->m_acMvCand[0];

    pcCU->setMVPIdxSubParts( 0, eRefPicList, uiPartAddr, uiPartIdx, pcCU->getDepth(uiPartAddr));
    pcCU->setMVPNumSubParts( pcAMVPInfo->iN, eRefPicList, uiPartAddr, uiPartIdx, pcCU->getDepth(uiPartAddr));
    return;
  }

  assert(pcCU->getMVPIdx(eRefPicList,uiPartAddr) >= 0);
  rcMvPred = pcAMVPInfo->m_acMvCand[pcCU->getMVPIdx(eRefPicList,uiPartAddr)];
  return;
}

/** Function for deriving planar intra prediction.
 * \param pSrc        pointer to reconstructed sample array
 * \param srcStride   the stride of the reconstructed sample array
 * \param rpDst       reference to pointer for the prediction sample array
 * \param dstStride   the stride of the prediction sample array
 * \param width       the width of the block
 * \param height      the height of the block
 * \param channelType type of pel array (luma, chroma)
 * \param format      chroma format
 *
 * This function derives the prediction samples for planar mode (intra coding).
 */
//NOTE: Bit-Limit - 24-bit source
Void TComPrediction::xPredIntraPlanar( const Pel* pSrc, Int srcStride, Pel* rpDst, Int dstStride, UInt width, UInt height )
{
#if !JVET_C0024_QTBT
  assert(width <= height);
#endif

  Int leftColumn[MAX_CU_SIZE+1], topRow[MAX_CU_SIZE+1], bottomRow[MAX_CU_SIZE], rightColumn[MAX_CU_SIZE];
#if JVET_C0024_QTBT
  UInt shift1Dhor = g_aucConvertToBit[ width ] + MIN_CU_LOG2;
  UInt shift1Dver = g_aucConvertToBit[ height ] + MIN_CU_LOG2;
  UInt delt = width*height;
#else
  UInt shift1Dhor = g_aucConvertToBit[ width ] + 2;
  UInt shift1Dver = g_aucConvertToBit[ height ] + 2;
#endif
  // Get left and above reference column and row
  for(Int k=0;k<width+1;k++)
  {
    topRow[k] = pSrc[k-srcStride];
  }

  for (Int k=0; k < height+1; k++)
  {
    leftColumn[k] = pSrc[k*srcStride-1];
  }

  // Prepare intermediate variables used in interpolation
  Int bottomLeft = leftColumn[height];
  Int topRight   = topRow[width];

  for(Int k=0;k<width;k++)
  {
    bottomRow[k]  = bottomLeft - topRow[k];
    topRow[k]     <<= shift1Dver;
  }

  for(Int k=0;k<height;k++)
  {
    rightColumn[k]  = topRight - leftColumn[k];
    leftColumn[k]   <<= shift1Dhor;
  }

  const UInt topRowShift = 0;

  // Generate prediction signal
  for (Int y=0;y<height;y++)
  {
#if JVET_C0024_QTBT
    Int horPred = leftColumn[y];
#else
    Int horPred = leftColumn[y] + width;
#endif
    for (Int x=0;x<width;x++)
    {
      horPred += rightColumn[y];
      topRow[x] += bottomRow[x];

      Int vertPred = ((topRow[x] + topRowShift)>>topRowShift);
#if JVET_C0024_QTBT
      rpDst[y*dstStride+x] = ((horPred<<shift1Dver) + (vertPred<<shift1Dhor) + delt) >> (shift1Dhor+shift1Dver+1);
#else
      rpDst[y*dstStride+x] = ( horPred + vertPred ) >> (shift1Dhor+1);
#endif
    }
  }
}

/** Function for filtering intra DC predictor.
 * \param pSrc pointer to reconstructed sample array
 * \param iSrcStride the stride of the reconstructed sample array
 * \param pDst reference to pointer for the prediction sample array
 * \param iDstStride the stride of the prediction sample array
 * \param iWidth the width of the block
 * \param iHeight the height of the block
 * \param channelType type of pel array (luma, chroma)
 *
 * This function performs filtering left and top edges of the prediction samples for DC mode (intra coding).
 */
Void TComPrediction::xDCPredFiltering( const Pel* pSrc, Int iSrcStride, Pel* pDst, Int iDstStride, Int iWidth, Int iHeight, ChannelType channelType )
{
  Int x, y, iDstStride2, iSrcStride2;

  if (isLuma(channelType) && (iWidth <= MAXIMUM_INTRA_FILTERED_WIDTH) && (iHeight <= MAXIMUM_INTRA_FILTERED_HEIGHT))
  {
    //top-left
    pDst[0] = (Pel)((pSrc[-iSrcStride] + pSrc[-1] + 2 * pDst[0] + 2) >> 2);

    //top row (vertical filter)
    for ( x = 1; x < iWidth; x++ )
    {
      pDst[x] = (Pel)((pSrc[x - iSrcStride] +  3 * pDst[x] + 2) >> 2);
    }

    //left column (horizontal filter)
    for ( y = 1, iDstStride2 = iDstStride, iSrcStride2 = iSrcStride-1; y < iHeight; y++, iDstStride2+=iDstStride, iSrcStride2+=iSrcStride )
    {
      pDst[iDstStride2] = (Pel)((pSrc[iSrcStride2] + 3 * pDst[iDstStride2] + 2) >> 2);
    }
  }

  return;
}

#if VCEG_AZ07_INTRA_BOUNDARY_FILTER
Void TComPrediction::xIntraPredFilteringMode34( const Pel* pSrc, Int iSrcStride, Pel*& rpDst, Int iDstStride, Int iWidth, Int iHeight )
{
  Pel* pDst = rpDst;

  for ( Int y = 0, iDstStride2 = 0, iSrcStride2 = -1; y < iHeight; y++, iDstStride2+=iDstStride, iSrcStride2+=iSrcStride )
  {
    pDst[iDstStride2  ] = (  8 * pDst[iDstStride2  ] + 8 * pSrc[iSrcStride2+iSrcStride  ] + 8 ) >> 4;
#if VCEG_AZ07_INTRA_BOUNDARY_FILTER_MULTI_LINE
    pDst[iDstStride2+1] = ( 12 * pDst[iDstStride2+1] + 4 * pSrc[iSrcStride2+iSrcStride*2] + 8 ) >> 4;     
#if JVET_C0024_QTBT
    if (iWidth>2)
    {
#endif
    pDst[iDstStride2+2] = ( 14 * pDst[iDstStride2+2] + 2 * pSrc[iSrcStride2+iSrcStride*3] + 8 ) >> 4;    
    pDst[iDstStride2+3] = ( 15 * pDst[iDstStride2+3] +     pSrc[iSrcStride2+iSrcStride*4] + 8 ) >> 4;
#if JVET_C0024_QTBT
    }
#endif
#endif
  }
  return;
}

Void TComPrediction::xIntraPredFilteringMode02( const Pel* pSrc, Int iSrcStride, Pel*& rpDst, Int iDstStride, Int iWidth, Int iHeight )
{
  Pel* pDst = rpDst;

  for ( Int x = 0; x < iWidth; x++ )
  {
    pDst[x             ] = (  8 * pDst[x             ] + 8 * pSrc[x - iSrcStride + 1] + 8 ) >> 4;
#if VCEG_AZ07_INTRA_BOUNDARY_FILTER_MULTI_LINE
    pDst[x+iDstStride  ] = ( 12 * pDst[x+iDstStride  ] + 4 * pSrc[x - iSrcStride + 2] + 8 ) >> 4;
#if JVET_C0024_QTBT
    if (iHeight>2)
    {
#endif
    pDst[x+iDstStride*2] = ( 14 * pDst[x+iDstStride*2] + 2 * pSrc[x - iSrcStride + 3] + 8 ) >> 4;
    pDst[x+iDstStride*3] = ( 15 * pDst[x+iDstStride*3] +     pSrc[x - iSrcStride + 4] + 8 ) >> 4; 
#if JVET_C0024_QTBT
    }
#endif
#endif
  }
  return;
}

Void TComPrediction::xIntraPredFilteringModeDGL( const Pel* pSrc, Int iSrcStride, Pel*& rpDst, Int iDstStride, Int iWidth, Int iHeight, UInt uiMode )
{
  Pel* pDst = rpDst;

#if VCEG_AZ07_INTRA_65ANG_MODES
  const Int aucAngPredFilterCoef[8][3] = {
    { 12, 3, 1 }, { 12, 3, 1 }, 
    { 12, 1, 3 }, { 12, 2, 2 }, 
    { 12, 2, 2 }, { 12, 3, 1 },
    {  8, 6, 2 }, {  8, 7, 1 },  
  };
  const Int aucAngPredPosiOffset[8][2] = {
    { 2, 3 }, { 2, 3 }, 
    { 1, 2 }, { 1, 2 },
    { 1, 2 }, { 1, 2 },
    { 1, 2 }, { 1, 2 },
  };
  assert( ( uiMode>=(VDIA_IDX-8) && uiMode<VDIA_IDX ) || ( uiMode>2 && uiMode<=(2+8) ) );
#else
  const Int aucAngPredFilterCoef[4][3] = {
    { 12, 3, 1 }, 
    { 12, 1, 3 }, 
    { 12, 2, 2 },
    {  8, 6, 2 },  
  };
  const Int aucAngPredPosiOffset[4][2] = {
    { 2, 3 }, 
    { 1, 2 },
    { 1, 2 },
    { 1, 2 },
  };
  assert( ( uiMode>=30 && uiMode<34 ) || ( uiMode>2 && uiMode<=6 ) );
#endif

#if VCEG_AZ07_INTRA_65ANG_MODES
  Bool bHorz = (uiMode < DIA_IDX);
  UInt deltaAng = bHorz ? ((2+8)-uiMode): (uiMode-(VDIA_IDX-8));
#else
  Bool bHorz = (uiMode < 18);
  UInt deltaAng = bHorz ? (6-uiMode): (uiMode-30);
#endif

  const Int *offset = aucAngPredPosiOffset[deltaAng];
  const Int *filter = aucAngPredFilterCoef[deltaAng];

  if(bHorz)
  {
    for ( Int x = 0; x < iWidth; x++ )
    {
      pDst[x] = ( filter[0] * pDst[x] 
      + filter[1] * pSrc[x - iSrcStride + offset[0]] 
      + filter[2] * pSrc[x - iSrcStride + offset[1]] + 8) >> 4;
    }
  }
  else
  {
    for ( Int y = 0; y < iHeight; y++ )
    {         
      pDst[y * iDstStride] = ( filter[0] * pDst[y * iDstStride] 
      + filter[1] * pSrc[(y + offset[0] ) * iSrcStride -1 ] 
      + filter[2] * pSrc[(y + offset[1] ) * iSrcStride -1 ] + 8) >> 4;        
    }
  }

  return;
}
#endif

/* Static member function */
Bool TComPrediction::UseDPCMForFirstPassIntraEstimation(TComTU &rTu, const UInt uiDirMode)
{
  return (rTu.getCU()->isRDPCMEnabled(rTu.GetAbsPartIdxTU()) ) &&
          rTu.getCU()->getCUTransquantBypass(rTu.GetAbsPartIdxTU()) &&
          (uiDirMode==HOR_IDX || uiDirMode==VER_IDX);
}

#if COM16_C806_VCEG_AZ10_SUB_PU_TMVP
Void TComPrediction::xGetSubPUAddrAndMerge(TComDataCU* pcCU, UInt uiPartAddr, Int iSPWidth, Int iSPHeight, Int iNumSPInOneLine, Int iNumSP, UInt* uiMergedSPW, UInt* uiMergedSPH, UInt* uiSPAddr )
{
  for (Int i = 0; i < iNumSP; i++)
  {
    uiMergedSPW[i] = iSPWidth;
    uiMergedSPH[i] = iSPHeight;
    pcCU->getSPAbsPartIdx(uiPartAddr, iSPWidth, iSPHeight, i, iNumSPInOneLine, uiSPAddr[i]);
  }

  // horizontal sub-PU merge
  for (Int i=0; i<iNumSP; i++)
  {
    if (i % iNumSPInOneLine == iNumSPInOneLine - 1 || uiMergedSPW[i]==0 || uiMergedSPH[i]==0)
    {
      continue;
    }
    for (Int j=i+1; j<i+iNumSPInOneLine-i%iNumSPInOneLine; j++)
    {
      if (xCheckTwoSPMotion(pcCU, uiSPAddr[i], uiSPAddr[j]))
      {
        uiMergedSPW[i] += iSPWidth;
        uiMergedSPW[j] = uiMergedSPH[j] = 0;
      }
      else
      {
        break;
      }
    }
  }
  //vertical sub-PU merge
  for (Int i=0; i<iNumSP-iNumSPInOneLine; i++)
  {
    if (uiMergedSPW[i]==0 || uiMergedSPH[i]==0)
    {
      continue;
    }
    for (Int j=i+iNumSPInOneLine; j<iNumSP; j+=iNumSPInOneLine)
    {
      if (xCheckTwoSPMotion(pcCU, uiSPAddr[i], uiSPAddr[j]) && uiMergedSPW[i]==uiMergedSPW[j])
      {
        uiMergedSPH[i] += iSPHeight;
        uiMergedSPH[j] = uiMergedSPW[j] = 0;
      }
      else
      {
        break;
      }
    }
  }
}

Bool TComPrediction::xCheckTwoSPMotion ( TComDataCU* pcCU, UInt PartAddr0, UInt PartAddr1 )
{
  if( pcCU->getCUMvField(REF_PIC_LIST_0)->getRefIdx(PartAddr0) != pcCU->getCUMvField(REF_PIC_LIST_0)->getRefIdx(PartAddr1))
  {
    return false;
  }
  if( pcCU->getCUMvField(REF_PIC_LIST_1)->getRefIdx(PartAddr0) != pcCU->getCUMvField(REF_PIC_LIST_1)->getRefIdx(PartAddr1))
  {
    return false;
  }

  if (pcCU->getCUMvField(REF_PIC_LIST_0)->getRefIdx(PartAddr0) >= 0)
  {
    if (pcCU->getCUMvField(REF_PIC_LIST_0)->getMv(PartAddr0) != pcCU->getCUMvField(REF_PIC_LIST_0)->getMv(PartAddr1))
    {
      return false;
    }
  }

  if (pcCU->getCUMvField(REF_PIC_LIST_1)->getRefIdx(PartAddr0) >= 0)
  {
    if (pcCU->getCUMvField(REF_PIC_LIST_1)->getMv(PartAddr0) != pcCU->getCUMvField(REF_PIC_LIST_1)->getMv(PartAddr1))
    {
      return false;
    }
  }
  return true;
}
#endif

#if COM16_C806_OBMC
/** Function for sub-block based Overlapped Block Motion Compensation (OBMC).
 *
 * This function can:
 * 1. Perform sub-block OBMC for a CU.
 * 2. Before motion estimation, subtract (scaled) predictors generated by applying neighboring motions to current CU/PU from the original signal of current CU/PU,
 *    to make the motion estimation biased to OBMC.
 */
#if JVET_E0052_DMVR
Void TComPrediction::subBlockOBMC( TComDataCU*  pcCU, UInt uiAbsPartIdx, TComYuv *pcYuvPred, TComYuv *pcYuvTmpPred1, TComYuv *pcYuvTmpPred2, Bool bRefineflag, Bool bOBMC4ME )
#else
Void TComPrediction::subBlockOBMC( TComDataCU*  pcCU, UInt uiAbsPartIdx, TComYuv *pcYuvPred, TComYuv *pcYuvTmpPred1, TComYuv *pcYuvTmpPred2, Bool bOBMC4ME )
#endif
{
  if( !pcCU->getSlice()->getSPS()->getOBMC() || !pcCU->getOBMCFlag( uiAbsPartIdx ) )
  {
    return;
  }
  
#if JVET_C0024_QTBT
  PartSize ePartSize = SIZE_2Nx2N;
#else
  PartSize ePartSize = pcCU->getPartitionSize( uiAbsPartIdx );
#endif
  UInt uiWidth           = pcCU->getWidth ( uiAbsPartIdx );
  UInt uiHeight          = pcCU->getHeight( uiAbsPartIdx );
  UInt uiMinCUW          = pcCU->getPic()->getMinCUWidth();
  UInt uiOBMCBlkSize     = pcCU->getSlice()->getSPS()->getOBMCBlkSize();
  UInt uiChromaOBMCWidth = 0, uiChromaOBMCHeight = 0;
  UInt uiMaxWidthInBlock = pcCU->getPic()->getNumPartInCtuWidth();

  UInt uiHeightInBlock   = uiHeight / uiMinCUW;
  UInt uiWidthInBlock    = uiWidth / uiMinCUW;
  UInt uiStep            = uiOBMCBlkSize / uiMinCUW;
#if !JVET_C0024_QTBT
  UInt uiMaxCUDepth      = pcCU->getSlice()->getSPS()->getMaxTotalCUDepth();
  UInt uiDepth           = uiMaxCUDepth - pcCU->getDepth( uiAbsPartIdx );
#endif

  UInt uiSubPartIdx      = 0;
  UInt uiZeroIdx         = pcCU->getZorderIdxInCtu();
  UInt uiAbsPartIdxLCURaster = g_auiZscanToRaster[uiAbsPartIdx + uiZeroIdx];
#if JVET_C0024_QTBT
  Bool bOBMCSimp         = uiWidth * uiHeight < 64;
#else
  Bool bOBMCSimp             = ( uiWidth == 8 && ePartSize != SIZE_2Nx2N );

  Int  i1stPUWidth  = -1, i1stPUHeight = -1;
  UInt uiPartAddr   = 0;
#endif
#if COM16_C806_VCEG_AZ10_SUB_PU_TMVP
#if JVET_C0035_ATMVP_SIMPLIFICATION 
  Bool bATMVP       = (pcCU->getMergeType( uiAbsPartIdx ) == MGR_TYPE_SUBPU_ATMVP || pcCU->getMergeType( uiAbsPartIdx ) == MGR_TYPE_SUBPU_ATMVP_EXT);
#else
  Bool bATMVP       = (pcCU->getMergeType( uiAbsPartIdx ) == MGR_TYPE_SUBPU_TMVP || pcCU->getMergeType( uiAbsPartIdx ) == MGR_TYPE_SUBPU_TMVP_EXT);
#endif
  
#if JVET_C0024_QTBT
  Bool bNormal2Nx2N = !bATMVP;
  Bool bSubMotion   = bATMVP;
#else
  Bool bNormal2Nx2N = (ePartSize == SIZE_2Nx2N && !bATMVP);
  Bool bSubMotion   = ePartSize == SIZE_NxN   || (ePartSize == SIZE_2Nx2N && bATMVP);
#endif
#else
  Bool bNormal2Nx2N = ePartSize == SIZE_2Nx2N;
#if JVET_C0024_QTBT
  Bool bSubMotion   = false;
#else
  Bool bSubMotion   = ePartSize == SIZE_NxN;
#endif
#endif
#if VCEG_AZ07_FRUC_MERGE
#if JVET_C0024_QTBT
  Int avgLength = 1<<(((g_aucConvertToBit[pcCU->getWidth( 0 )] + g_aucConvertToBit[pcCU->getHeight( 0 )] + 1)>>1) + MIN_CU_LOG2);
  Int nFrucRefineSize = max( avgLength >> pcCU->getSlice()->getSPS()->getFRUCSmallBlkRefineDepth(), FRUC_MERGE_REFINE_MINBLKSIZE );
#else
  Int nFrucRefineSize = max( pcCU->getWidth( 0 ) >> pcCU->getSlice()->getSPS()->getFRUCSmallBlkRefineDepth(), FRUC_MERGE_REFINE_MINBLKSIZE );
#endif
  if( pcCU->getFRUCMgrMode( uiAbsPartIdx ) && ePartSize == SIZE_2Nx2N )
  {
    bNormal2Nx2N = false;
    bSubMotion = true;
  }
#endif
#if JVET_B0038_AFFINE_HARMONIZATION
  if ( pcCU->getAffineFlag( uiAbsPartIdx ) )
  {
    bNormal2Nx2N = false;
    bSubMotion = true;
  }
#endif
#if !JVET_C0024_QTBT
  Bool bVerticalPU  = ( ePartSize == SIZE_2NxN || ePartSize == SIZE_2NxnU || ePartSize == SIZE_2NxnD );
  Bool bHorizonalPU = ( ePartSize == SIZE_Nx2N || ePartSize == SIZE_nLx2N || ePartSize == SIZE_nRx2N );
  Bool bAtmvpPU = false, bNormalTwoPUs = false;
#if VCEG_AZ07_FRUC_MERGE
  Bool bFrucPU = false;
#endif
  Bool bTwoPUs  = ( bVerticalPU || bHorizonalPU );
#endif
  Int  iNeigPredDir = 0, iCurPredDir = 0;
#if JVET_C0024_QTBT && COM16_C1016_AFFINE
  Bool isCurAffine;
#endif

  switch( pcCU->getSlice()->getSPS()->getChromaFormatIdc() )
  {
  case CHROMA_400:
    uiChromaOBMCWidth = uiChromaOBMCHeight = 0;                              break;
  case CHROMA_420:
    uiChromaOBMCWidth = uiChromaOBMCHeight = uiOBMCBlkSize/2;                break;
  case CHROMA_422:
    uiChromaOBMCWidth = uiOBMCBlkSize/2; uiChromaOBMCHeight = uiOBMCBlkSize; break;
  case CHROMA_444:
    uiChromaOBMCWidth = uiChromaOBMCHeight = uiOBMCBlkSize;                  break;
  default:
    break;
  }

#if JVET_C0024_QTBT
  Bool bCurrMotStored = false;
#else
  if( bTwoPUs )
  {
    pcCU->getPartIndexAndSize( 1, uiPartAddr, i1stPUWidth, i1stPUHeight );
#if COM16_C806_VCEG_AZ10_SUB_PU_TMVP
#if JVET_C0035_ATMVP_SIMPLIFICATION 
    bAtmvpPU |= (pcCU->getMergeType( uiPartAddr ) == MGR_TYPE_SUBPU_ATMVP || pcCU->getMergeType( uiPartAddr ) == MGR_TYPE_SUBPU_ATMVP_EXT);
#else
    bAtmvpPU |= (pcCU->getMergeType( uiPartAddr ) == MGR_TYPE_SUBPU_TMVP || pcCU->getMergeType( uiPartAddr ) == MGR_TYPE_SUBPU_TMVP_EXT);
#endif
#endif
#if VCEG_AZ07_FRUC_MERGE
    bFrucPU |= (pcCU->getFRUCMgrMode( uiPartAddr ) != FRUC_MERGE_OFF );
#endif
    pcCU->getPartIndexAndSize( 0, uiPartAddr, i1stPUWidth, i1stPUHeight );
#if COM16_C806_VCEG_AZ10_SUB_PU_TMVP
#if JVET_C0035_ATMVP_SIMPLIFICATION 
    bAtmvpPU |= (pcCU->getMergeType( uiPartAddr ) == MGR_TYPE_SUBPU_ATMVP || pcCU->getMergeType( uiPartAddr ) == MGR_TYPE_SUBPU_ATMVP_EXT);
#else
    bAtmvpPU |= (pcCU->getMergeType( uiPartAddr ) == MGR_TYPE_SUBPU_TMVP || pcCU->getMergeType( uiPartAddr ) == MGR_TYPE_SUBPU_TMVP_EXT);
#endif
#endif
#if VCEG_AZ07_FRUC_MERGE
    bFrucPU |= (pcCU->getFRUCMgrMode( uiPartAddr ) != FRUC_MERGE_OFF );
#endif
    i1stPUWidth  /= pcCU->getPic()->getMinCUWidth();
    i1stPUHeight /= pcCU->getPic()->getMinCUWidth();    

    bNormalTwoPUs = !bAtmvpPU;
#if VCEG_AZ07_FRUC_MERGE
    bNormalTwoPUs &= !bFrucPU;
#endif
  }

  Bool bCurrMotStored = false, bDiffMot[4]= { false, false, false, false };
#endif
  TComMvField cCurMvField[2], cNeigMvField[2];

  Int maxDir = bNormal2Nx2N ? 2 : 4;
  for( Int iSubX = 0; iSubX < uiWidthInBlock; iSubX += uiStep )
  {
    for( Int iSubY = 0; iSubY < uiHeightInBlock; iSubY += uiStep )
    {
      if( bNormal2Nx2N && iSubX && iSubY )
      {
        continue;
      }
      Bool bCURBoundary = ( iSubX == uiWidthInBlock  - 1 );
      Bool bCUBBoundary = ( iSubY == uiHeightInBlock - 1 );

      bCurrMotStored    = false;
      uiSubPartIdx      = g_auiRasterToZscan[uiAbsPartIdxLCURaster + iSubX + iSubY*uiMaxWidthInBlock] - uiZeroIdx;

      for( Int iDir = 0; iDir < maxDir; iDir++ ) //iDir: 0 - above, 1 - left, 2 - below, 3 - right
      {
        if( ( iDir == 3 && bCURBoundary ) || ( iDir == 2 && bCUBBoundary ) )
        {
          continue;
        }
 
#if !JVET_C0024_QTBT
        Bool bVerPUBound  = false;
        Bool bHorPUBound  = false;
#endif

        if( bNormal2Nx2N ) //skip unnecessary check for CU boundary
        {
          if( ( iDir == 1 && !iSubY && iSubX ) || ( iDir == 0 && !iSubX && iSubY ) )
          {
            continue;
          }
        }
        else
        {
          Bool bCheckNeig = bSubMotion || ( iSubX == 0 && iDir == 1 ) || ( iSubY == 0 && iDir == 0 ); //CU boundary or NxN or 2nx2n_ATMVP
#if !JVET_C0024_QTBT
          if( !bCheckNeig && bTwoPUs )
          {
#if COM16_C806_VCEG_AZ10_SUB_PU_TMVP
            if( bAtmvpPU )
            {
              //sub-PU boundary
#if JVET_C0035_ATMVP_SIMPLIFICATION 
              bCheckNeig |= (pcCU->getMergeType( uiSubPartIdx ) == MGR_TYPE_SUBPU_ATMVP || pcCU->getMergeType( uiSubPartIdx ) == MGR_TYPE_SUBPU_ATMVP_EXT);
#else
              bCheckNeig |= (pcCU->getMergeType( uiSubPartIdx ) == MGR_TYPE_SUBPU_TMVP || pcCU->getMergeType( uiSubPartIdx ) == MGR_TYPE_SUBPU_TMVP_EXT);
#endif
            }
#endif
#if VCEG_AZ07_FRUC_MERGE
            if( bFrucPU )
            {
              //sub-PU boundary
              bCheckNeig |= ( pcCU->getFRUCMgrMode( uiSubPartIdx ) != FRUC_MERGE_OFF );
            }
#endif
            if( !bCheckNeig )
            {
              //PU boundary
              bVerPUBound = bVerticalPU  && ( ( iDir == 2 && iSubY == i1stPUHeight - 1 ) || ( iDir == 0 && iSubY == i1stPUHeight ) );
              bHorPUBound = bHorizonalPU && ( ( iDir == 3 && iSubX == i1stPUWidth  - 1 ) || ( iDir == 1 && iSubX == i1stPUWidth ) );

              bCheckNeig  |= ( bVerPUBound || bHorPUBound );
            }
          }
#endif
          if( !bCheckNeig )
          {
            continue;
          }
        }
        
#if !JVET_C0024_QTBT
        Bool bCurSubBkFetched  = bNormalTwoPUs && ( ( bVerPUBound && iSubX ) || ( bHorPUBound && iSubY ) );
#endif

#if COM16_C806_VCEG_AZ10_SUB_PU_TMVP
#if JVET_C0035_ATMVP_SIMPLIFICATION 
        Bool bSubBlockOBMCSimp = (bOBMCSimp || (( pcCU->getMergeType( uiSubPartIdx ) == MGR_TYPE_SUBPU_ATMVP || pcCU->getMergeType( uiSubPartIdx ) == MGR_TYPE_SUBPU_ATMVP_EXT) && ( 1 << pcCU->getSlice()->getSPS()->getSubPUTLog2Size() ) == 4 ));
#else
        Bool bSubBlockOBMCSimp = (bOBMCSimp || (( pcCU->getMergeType( uiSubPartIdx ) == MGR_TYPE_SUBPU_TMVP || pcCU->getMergeType( uiSubPartIdx ) == MGR_TYPE_SUBPU_TMVP_EXT) && ( 1 << pcCU->getSlice()->getSPS()->getSubPUTLog2Size() ) == 4 ));
#endif
#else
        Bool bSubBlockOBMCSimp = bOBMCSimp;
#endif
#if VCEG_AZ07_FRUC_MERGE
        bSubBlockOBMCSimp |= ( bOBMCSimp || ( pcCU->getFRUCMgrMode( uiSubPartIdx ) != FRUC_MERGE_OFF && nFrucRefineSize == 4 ) );
#endif
#if JVET_B0038_AFFINE_HARMONIZATION
        bSubBlockOBMCSimp |= ( bOBMCSimp || pcCU->getAffineFlag( uiSubPartIdx ) );
#endif
#if JVET_C0024_QTBT
        if( pcCU->getNeigMotion( uiSubPartIdx, cNeigMvField, iNeigPredDir, iDir, cCurMvField, iCurPredDir, uiZeroIdx, bCurrMotStored ) )
#else
        if( ( bCurSubBkFetched && bDiffMot[iDir] ) || pcCU->getNeigMotion( uiSubPartIdx, cNeigMvField, iNeigPredDir, iDir, cCurMvField, iCurPredDir, uiZeroIdx, bCurrMotStored ) )
#endif
        {
#if JVET_C0024_QTBT
          //store temporary motion information
#if COM16_C1016_AFFINE
          isCurAffine = pcCU->getAffineFlag(uiSubPartIdx);  //bug fix for affine OBMC
          pcCU->setAffineFlag(uiSubPartIdx, false);
#endif
          pcCU->getCUMvField( REF_PIC_LIST_0 )->setMv(cNeigMvField[0].getMv(), uiSubPartIdx);
          pcCU->getCUMvField( REF_PIC_LIST_0 )->setRefIdx(cNeigMvField[0].getRefIdx(), uiSubPartIdx);
          pcCU->getCUMvField( REF_PIC_LIST_1 )->setMv(cNeigMvField[1].getMv(), uiSubPartIdx);
          pcCU->getCUMvField( REF_PIC_LIST_1 )->setRefIdx(cNeigMvField[1].getRefIdx(), uiSubPartIdx);
          pcCU->setInterDir(uiSubPartIdx, iNeigPredDir);

          //motion compensation and OBMC
          xSubBlockMotionCompensation( pcCU, pcYuvTmpPred1, uiSubPartIdx, uiOBMCBlkSize, uiOBMCBlkSize 
#if JVET_E0052_DMVR
            , bRefineflag 
#endif
          );

          if( bOBMC4ME )
          {
            xSubtractOBMC( pcCU, uiSubPartIdx, pcYuvPred, pcYuvTmpPred1, uiOBMCBlkSize, uiOBMCBlkSize, iDir, bSubBlockOBMCSimp );
          }
          else
          {
            xSubblockOBMC( COMPONENT_Y, pcCU, uiSubPartIdx, pcYuvPred, pcYuvTmpPred1, uiOBMCBlkSize, uiOBMCBlkSize, iDir, bSubBlockOBMCSimp );
            xSubblockOBMC( COMPONENT_Cb, pcCU, uiSubPartIdx, pcYuvPred, pcYuvTmpPred1, uiChromaOBMCWidth, uiChromaOBMCHeight, iDir, bSubBlockOBMCSimp );
            xSubblockOBMC( COMPONENT_Cr, pcCU, uiSubPartIdx, pcYuvPred, pcYuvTmpPred1, uiChromaOBMCWidth, uiChromaOBMCHeight, iDir, bSubBlockOBMCSimp );
          }
          //recover motion information
#if COM16_C1016_AFFINE
          pcCU->setAffineFlag(uiSubPartIdx, isCurAffine);
#endif
          pcCU->getCUMvField( REF_PIC_LIST_0 )->setMv(cCurMvField[0].getMv(), uiSubPartIdx);
          pcCU->getCUMvField( REF_PIC_LIST_0 )->setRefIdx(cCurMvField[0].getRefIdx(), uiSubPartIdx);
          pcCU->getCUMvField( REF_PIC_LIST_1 )->setMv(cCurMvField[1].getMv(), uiSubPartIdx);
          pcCU->getCUMvField( REF_PIC_LIST_1 )->setRefIdx(cCurMvField[1].getRefIdx(), uiSubPartIdx);
          pcCU->setInterDir(uiSubPartIdx, iCurPredDir);

#else
          Bool bFeAllSubBkIn1Line = false; //Fetch all sub-blocks in one row/column
          if( !bCurSubBkFetched )
          {
            //store temporary motion information
            pcCU->setPartSizeSubParts( SIZE_2Nx2N, uiSubPartIdx, uiMaxCUDepth);
            pcCU->getCUMvField( REF_PIC_LIST_0 )->setAllMvField( cNeigMvField[0], SIZE_2Nx2N, uiSubPartIdx, uiDepth, 0 );
            pcCU->getCUMvField( REF_PIC_LIST_1 )->setAllMvField( cNeigMvField[1], SIZE_2Nx2N, uiSubPartIdx, uiDepth, 0 );
            pcCU->setInterDirSubParts( iNeigPredDir, uiSubPartIdx, 0, uiMaxCUDepth );
            if( bNormalTwoPUs )
            {
              bFeAllSubBkIn1Line = ( bHorPUBound || bVerPUBound );
              if( bFeAllSubBkIn1Line )
              {
                bDiffMot[iDir] = true;
              }
            }
          }
          UInt uiSubBlockWidth  = ( bFeAllSubBkIn1Line && bVerticalPU  ) ? uiWidth  : uiOBMCBlkSize;
          UInt uiSubBlockHeight = ( bFeAllSubBkIn1Line && !bVerticalPU ) ? uiHeight : uiOBMCBlkSize;
          //motion compensation and OBMC
          if( !bCurSubBkFetched )
          {
            xSubBlockMotionCompensation( pcCU, bFeAllSubBkIn1Line ? pcYuvTmpPred2 : pcYuvTmpPred1, uiSubPartIdx, uiSubBlockWidth, uiSubBlockHeight
#if JVET_E0052_DMVR
    , bRefineflag 
#endif
);
          }

          if( bOBMC4ME )
          {
            xSubtractOBMC( pcCU, uiSubPartIdx, pcYuvPred, ( bCurSubBkFetched || bFeAllSubBkIn1Line ) ? pcYuvTmpPred2 : pcYuvTmpPred1, uiOBMCBlkSize, uiOBMCBlkSize, iDir, bSubBlockOBMCSimp );
          }
          else
          {
            xSubblockOBMC( COMPONENT_Y, pcCU, uiSubPartIdx, pcYuvPred, ( bCurSubBkFetched || bFeAllSubBkIn1Line ) ? pcYuvTmpPred2 : pcYuvTmpPred1, uiOBMCBlkSize, uiOBMCBlkSize, iDir, bSubBlockOBMCSimp );
            xSubblockOBMC( COMPONENT_Cb, pcCU, uiSubPartIdx, pcYuvPred, ( bCurSubBkFetched || bFeAllSubBkIn1Line ) ? pcYuvTmpPred2 : pcYuvTmpPred1, uiChromaOBMCWidth, uiChromaOBMCHeight, iDir, bSubBlockOBMCSimp );
            xSubblockOBMC( COMPONENT_Cr, pcCU, uiSubPartIdx, pcYuvPred, ( bCurSubBkFetched || bFeAllSubBkIn1Line ) ? pcYuvTmpPred2 : pcYuvTmpPred1, uiChromaOBMCWidth, uiChromaOBMCHeight, iDir, bSubBlockOBMCSimp );
          }
          //recover motion information
          if( !bCurSubBkFetched )
          {
            pcCU->getCUMvField( REF_PIC_LIST_0 )->setAllMvField( cCurMvField[0], SIZE_2Nx2N, uiSubPartIdx, uiDepth, 0 );
            pcCU->getCUMvField( REF_PIC_LIST_1 )->setAllMvField( cCurMvField[1], SIZE_2Nx2N, uiSubPartIdx, uiDepth, 0 );
            pcCU->setInterDirSubParts( iCurPredDir, uiSubPartIdx, 0, uiMaxCUDepth );
            pcCU->setPartSizeSubParts( ePartSize, uiSubPartIdx,      uiMaxCUDepth );
          }
#endif
        }
      }
    }
  }
}

// Function for (weighted) averaging predictors of current block and predictors generated by applying neighboring motions to current block.
Void TComPrediction::xSubblockOBMC( const ComponentID eComp, TComDataCU* pcCU, Int uiAbsPartIdx, TComYuv* pcYuvPredDst, TComYuv* pcYuvPredSrc, Int iWidth, Int iHeight, Int iDir, Bool bOBMCSimp )
{

  if( iWidth == 0 || iHeight == 0 )
  {
    return;
  }

  Int iDstStride  = pcYuvPredDst->getStride( eComp );
  Int iSrcStride  = pcYuvPredSrc->getStride( eComp );

  Pel *pDst   = pcYuvPredDst->getAddr( eComp, uiAbsPartIdx );
  Pel *pSrc   = pcYuvPredSrc->getAddr( eComp, uiAbsPartIdx );

  Int iDstPtrOffset = iDstStride, iScrPtrOffset = iSrcStride, ioffsetDst = 1, ioffsetSrc = 1;

  if( iDir ) //0: above; 1:left; 2: below; 3:right
  {
    if( iDir == 1 )  
    {
      iDstPtrOffset = iScrPtrOffset = 1;
      ioffsetDst = iDstStride; ioffsetSrc = iSrcStride;
      iWidth = iHeight;
    }
    else if( iDir == 2 )
    {
      Int iHMinus1 = iHeight - 1;
      pDst += iHMinus1*iDstStride;
      pSrc += iHMinus1*iSrcStride;
      iDstPtrOffset = -iDstStride; iScrPtrOffset = -iSrcStride; ioffsetDst = ioffsetSrc = 1;
    }
    else
    {
      Int iWMinus1 = iWidth - 1;
      pDst += iWMinus1;
      pSrc += iWMinus1;
      iDstPtrOffset = iScrPtrOffset = -1;
      ioffsetDst = iDstStride; ioffsetSrc = iSrcStride;
      iWidth  = iHeight;
    }
  }

  Pel *pDst1 = pDst  + iDstPtrOffset;
  Pel *pDst2 = pDst1 + iDstPtrOffset;
  Pel *pDst3 = pDst2 + iDstPtrOffset;
  Pel *pSrc1 = pSrc  + iScrPtrOffset;
  Pel *pSrc2 = pSrc1 + iScrPtrOffset;
  Pel *pSrc3 = pSrc2 + iScrPtrOffset;

  for( Int i = 0; i < iWidth; i++ )
  {
    *pDst  = ( (*pDst) * 3  + (*pSrc)  + 2 ) >> 2;
    if( !bOBMCSimp || eComp == COMPONENT_Y )
    {
      *pDst1 = ( (*pDst1) * 7 + (*pSrc1) + 4 ) >> 3;
    }
    pDst += ioffsetDst; pDst1 += ioffsetDst; pSrc += ioffsetSrc; pSrc1 += ioffsetSrc;

    if( !bOBMCSimp && ( eComp == COMPONENT_Y ) )
    {
      *pDst2 = ( (*pDst2) * 15 + (*pSrc2) +  8 ) >> 4;
      *pDst3 = ( (*pDst3) * 31 + (*pSrc3) + 16 ) >> 5;
      pDst2 += ioffsetDst; pDst3 += ioffsetDst; pSrc2 += ioffsetSrc; pSrc3 += ioffsetSrc;
    }
  }
}

// Function for subtracting (scaled) predictors generated by applying neighboring motions to current block from the original signal of current block.
Void TComPrediction::xSubtractOBMC( TComDataCU* pcCU, Int uiAbsPartIdx, TComYuv* pcYuvPredDst, TComYuv* pcYuvPredSrc, Int iWidth, Int iHeight, Int iDir, Bool bOBMCSimp )
{
  Int iDstStride = pcYuvPredDst->getWidth( COMPONENT_Y );
  Int iSrcStride = pcYuvPredSrc->getWidth( COMPONENT_Y );
  Pel *pDst      = pcYuvPredDst->getAddr( COMPONENT_Y, uiAbsPartIdx );
  Pel *pSrc      = pcYuvPredSrc->getAddr( COMPONENT_Y, uiAbsPartIdx );

  if( iDir == 0 ) //above
  {
    for( Int i = 0; i < iWidth; i++ )
    {
      pDst[i] += ( pDst[i] - pSrc[i] + 2 ) >> 2;
    }

    pDst += iDstStride;
    pSrc += iSrcStride;

    for( Int i = 0; i < iWidth; i++ )
    {
      pDst[i] += ( pDst[i] - pSrc[i] + 4 ) >> 3;
    }

    if( !bOBMCSimp )
    {
      pDst += iDstStride;
      pSrc += iSrcStride;

      for( Int i = 0; i < iWidth; i++ )
      {
        pDst[i] += ( pDst[i] - pSrc[i] + 8 ) >> 4;
      }

      pDst += iDstStride;
      pSrc += iSrcStride;

      for( Int i = 0; i < iWidth; i++ )
      {
        pDst[i] += ( pDst[i] - pSrc[i] + 16 ) >> 5;
      }
    }
  }

  if( iDir == 1 ) //left
  {
    for( Int i = 0; i < iHeight; i++ )
    {
      pDst[i*iDstStride] += ( pDst[i*iDstStride] - pSrc[i*iSrcStride] + 2 ) >> 2;
    }

    pDst++;
    pSrc++;

    for( Int i = 0; i < iHeight; i++ )
    {
      pDst[i*iDstStride] += ( pDst[i*iDstStride] - pSrc[i*iSrcStride] + 4 ) >> 3;
    }

    if( !bOBMCSimp )
    {
      pDst++;
      pSrc++;

      for( Int i = 0; i < iHeight; i++ )
      {
        pDst[i*iDstStride] += ( pDst[i*iDstStride] - pSrc[i*iSrcStride] + 8 ) >> 4;
      }
      pDst++;
      pSrc++;

      for( Int i = 0; i < iHeight; i++ )
      {
        pDst[i*iDstStride] += ( pDst[i*iDstStride] - pSrc[i*iSrcStride] + 16 ) >> 5;
      }
    }
  }

  if( iDir == 2 ) //below
  {
    pDst += ( iHeight - 4 )*iDstStride;
    pSrc += ( iHeight - 4 )*iSrcStride;
    if( !bOBMCSimp )
    {
      for( Int i = 0; i < iWidth; i++ )
      {
        pDst[i] += ( pDst[i] - pSrc[i] + 16 ) >> 5;
      }

      pDst += iDstStride;
      pSrc += iSrcStride;

      for( Int i = 0; i < iWidth; i++ )
      {
        pDst[i] += ( pDst[i] - pSrc[i] + 8 ) >> 4;
      }
    }
    else
    {
      pDst += iDstStride;
      pSrc += iSrcStride;
    }

    pDst += iDstStride;
    pSrc += iSrcStride;

    for( Int i = 0; i < iWidth; i++ )
    {
      pDst[i] += ( pDst[i] - pSrc[i] + 4 ) >> 3;
    }

    pDst += iDstStride;
    pSrc += iSrcStride;

    for( Int i = 0; i < iWidth; i++ )
    {
      pDst[i] += ( pDst[i] - pSrc[i] + 2 ) >> 2;
    }
  }

  if( iDir == 3 ) //right
  {
    pDst += iWidth - 4;
    pSrc += iWidth - 4;
    if( !bOBMCSimp )
    {
      for( Int i = 0; i < iHeight; i++ )
      {
        pDst[i*iDstStride] += ( pDst[i*iDstStride] - pSrc[i*iSrcStride] + 16 ) >> 5;
      }

      pDst++;
      pSrc++;

      for( Int i = 0; i < iHeight; i++ )
      {
        pDst[i*iDstStride] += ( pDst[i*iDstStride] - pSrc[i*iSrcStride] + 8 ) >> 4;
      }
    }
    else
    {
      pDst++;
      pSrc++;
    }

    pDst++;
    pSrc++;

    for( Int i = 0; i < iHeight; i++ )
    {
      pDst[i*iDstStride] += ( pDst[i*iDstStride] - pSrc[i*iSrcStride] + 4 ) >> 3;
    }
    pDst++;
    pSrc++;

    for( Int i = 0; i < iHeight; i++ )
    {
      pDst[i*iDstStride] += ( pDst[i*iDstStride] - pSrc[i*iSrcStride] + 2 ) >> 2;
    }
  }
}

Void TComPrediction::xSubBlockMotionCompensation ( TComDataCU* pcCU, TComYuv* pcYuvPred, Int uiPartAddr, Int iWidth, Int iHeight 
#if JVET_E0052_DMVR
    , Bool bRefineflag
#endif
    )
{
  if ( xCheckIdenticalMotion( pcCU, uiPartAddr ) )
  {
    xPredInterUni (pcCU, uiPartAddr, iWidth, iHeight, REF_PIC_LIST_0, pcYuvPred 
#if JVET_E0052_DMVR
    , bRefineflag
#endif
#if VCEG_AZ05_BIO 
      ,  false 
#endif
#if VCEG_AZ07_FRUC_MERGE
      , false , true
#endif
      );
  }
  else
  {
    xPredInterBi  (pcCU, uiPartAddr, iWidth, iHeight, pcYuvPred 
#if JVET_E0052_DMVR
    , bRefineflag
#endif
#if VCEG_AZ07_FRUC_MERGE || JVET_G0082
      , true
#endif
      );
  }
}
#endif

#if VCEG_AZ07_FRUC_MERGE

static const Int FRUC_MERGE_MV_SEARCHPATTERN_CROSS    = 0;
static const Int FRUC_MERGE_MV_SEARCHPATTERN_SQUARE   = 1;
static const Int FRUC_MERGE_MV_SEARCHPATTERN_DIAMOND  = 2;
static const Int FRUC_MERGE_MV_SEARCHPATTERN_HEXAGON  = 3;

/**
 * \brief calculate the cost of template matching in FRUC
 *
 * \param pcCU            Pointer to current CU
 * \param uiAbsPartIdx    Address of block within CU
 * \param nWidth          Width of the block
 * \param nHeight         Height of the block
 * \param eCurRefPicList  Reference picture list
 * \param rCurMvField     Mv to be checked
 * \param uiMVCost        Cost of the Mv
 */
UInt TComPrediction::xFrucGetTempMatchCost( TComDataCU * pcCU , UInt uiAbsPartIdx , Int nWidth , Int nHeight , RefPicList eCurRefPicList , const TComMvField & rCurMvField , UInt uiMVCost )
{
#if VCEG_AZ07_MV_ADD_PRECISION_BIT_FOR_STORE
  const Int nMVUnit = 2 + VCEG_AZ07_MV_ADD_PRECISION_BIT_FOR_STORE;
#else
  const Int nMVUnit = 2;
#endif

  UInt uiCost = uiMVCost;
  DistParam cDistParam;
  cDistParam.bApplyWeight = false;
  TComPicYuv * pRefPicYuv = pcCU->getSlice()->getRefPic( eCurRefPicList , rCurMvField.getRefIdx() )->getPicYuvRec();
  TComYuv * pYuvPredRefTop = &m_acYuvPred[0] , * pYuvPredRefLeft = &m_acYuvPred[1];
  TComYuv * pYuvPredCurTop = &m_cYuvPredFrucTemplate[0] , * pYuvPredCurLeft = &m_cYuvPredFrucTemplate[1];
  if( m_bFrucTemplateAvailabe[0] )
  {
    TComMv mvTop( 0 , - ( FRUC_MERGE_TEMPLATE_SIZE << nMVUnit ) );
    mvTop += rCurMvField.getMv();
    xPredInterBlk( COMPONENT_Y , pcCU , pRefPicYuv , uiAbsPartIdx , &mvTop , nWidth , FRUC_MERGE_TEMPLATE_SIZE , pYuvPredRefTop , false , pcCU->getSlice()->getSPS()->getBitDepth( CHANNEL_TYPE_LUMA ) , 
#if VCEG_AZ05_BIO
      false,
#endif
      FRUC_MERGE_TEMPLATE );
    m_cFRUCRDCost.setDistParam( cDistParam , pcCU->getSlice()->getSPS()->getBitDepth(CHANNEL_TYPE_LUMA) , pYuvPredCurTop->getAddr( COMPONENT_Y , 0 ) , pYuvPredCurTop->getStride( COMPONENT_Y ) ,
      pYuvPredRefTop->getAddr( COMPONENT_Y , 0 ) , pYuvPredRefTop->getStride( COMPONENT_Y ) , nWidth , FRUC_MERGE_TEMPLATE_SIZE , false );
#if VCEG_AZ06_IC
    cDistParam.bMRFlag = pcCU->getICFlag( uiAbsPartIdx );
#endif
    uiCost += cDistParam.DistFunc( &cDistParam );

  }
  if( m_bFrucTemplateAvailabe[1] )
  {
    TComMv mvLeft( - ( FRUC_MERGE_TEMPLATE_SIZE << nMVUnit ) , 0 );
    mvLeft += rCurMvField.getMv();
    xPredInterBlk( COMPONENT_Y , pcCU , pRefPicYuv , uiAbsPartIdx , &mvLeft , FRUC_MERGE_TEMPLATE_SIZE , nHeight , pYuvPredRefLeft , false , pcCU->getSlice()->getSPS()->getBitDepth( CHANNEL_TYPE_LUMA ) , 
#if VCEG_AZ05_BIO
      false,
#endif
      FRUC_MERGE_TEMPLATE );
    m_cFRUCRDCost.setDistParam( cDistParam , pcCU->getSlice()->getSPS()->getBitDepth(CHANNEL_TYPE_LUMA) , pYuvPredCurLeft->getAddr( COMPONENT_Y , 0 ) , pYuvPredCurLeft->getStride( COMPONENT_Y ) ,
      pYuvPredRefLeft->getAddr( COMPONENT_Y , 0 ) , pYuvPredRefLeft->getStride( COMPONENT_Y ) , FRUC_MERGE_TEMPLATE_SIZE , nHeight , false );
#if VCEG_AZ06_IC
    cDistParam.bMRFlag = pcCU->getICFlag( uiAbsPartIdx );
#endif
    uiCost += cDistParam.DistFunc( &cDistParam );

  }

  return( uiCost );
}

#if JVET_F0032_UNI_BI_SELECTION
Void TComPrediction::xFrucUpdateTemplate(TComDataCU * pcCU, UInt uiAbsPartIdx, Int nWidth, Int nHeight, RefPicList eCurRefPicList, TComMvField rCurMvField)
{
#if VCEG_AZ07_MV_ADD_PRECISION_BIT_FOR_STORE
    const Int nMVUnit = 2 + VCEG_AZ07_MV_ADD_PRECISION_BIT_FOR_STORE;
#else
    const Int nMVUnit = 2;
#endif

    TComMvField *pMvFieldOther = &rCurMvField;
    TComYuv * pYuvPredRefTop = &m_acYuvPred[0], *pYuvPredRefLeft = &m_acYuvPred[1];
    TComYuv * pYuvPredCurTop = &m_cYuvPredFrucTemplate[0], *pYuvPredCurLeft = &m_cYuvPredFrucTemplate[1];
    if (m_bFrucTemplateAvailabe[0])
    {
        TComMv mvOther(0, -(FRUC_MERGE_TEMPLATE_SIZE << nMVUnit));
        mvOther += pMvFieldOther->getMv();
        TComPicYuv * pRefPicYuvOther = pcCU->getSlice()->getRefPic(eCurRefPicList, pMvFieldOther->getRefIdx())->getPicYuvRec();
        xPredInterBlk(COMPONENT_Y, pcCU, pRefPicYuvOther, uiAbsPartIdx, &mvOther, nWidth, FRUC_MERGE_TEMPLATE_SIZE, pYuvPredRefTop
            , false
            , pcCU->getSlice()->getSPS()->getBitDepth(CHANNEL_TYPE_LUMA),
#if VCEG_AZ05_BIO
            false,
#endif
            FRUC_MERGE_TEMPLATE
        );
        TComYuv*  pcYuvOther = pYuvPredRefTop;
        TComYuv*  pcYuvOrg = pYuvPredCurTop;
        pcYuvOrg->removeHighFreq(pcYuvOther, uiAbsPartIdx, nWidth, FRUC_MERGE_TEMPLATE_SIZE, pcCU->getSlice()->getSPS()->getBitDepths().recon, false);
    }

    if (m_bFrucTemplateAvailabe[1])
    {
        TComMv mvOther(-(FRUC_MERGE_TEMPLATE_SIZE << nMVUnit), 0);
        mvOther += pMvFieldOther->getMv();
        TComPicYuv * pRefPicYuvOther = pcCU->getSlice()->getRefPic(eCurRefPicList, pMvFieldOther->getRefIdx())->getPicYuvRec();
        xPredInterBlk(COMPONENT_Y, pcCU, pRefPicYuvOther, uiAbsPartIdx, &mvOther, FRUC_MERGE_TEMPLATE_SIZE, nHeight, pYuvPredRefLeft
            , false
            , pcCU->getSlice()->getSPS()->getBitDepth(CHANNEL_TYPE_LUMA),
#if VCEG_AZ05_BIO
            false,
#endif
            FRUC_MERGE_TEMPLATE
        );
        TComYuv*  pcYuvOther = pYuvPredRefLeft;
        TComYuv*  pcYuvOrg = pYuvPredCurLeft;
        pcYuvOrg->removeHighFreq(pcYuvOther, uiAbsPartIdx, FRUC_MERGE_TEMPLATE_SIZE, nHeight, pcCU->getSlice()->getSPS()->getBitDepths().recon, false);
    }
}
#endif

/**
 * \brief calculate the cost of bilateral matching in FRUC
 *
 * \param pcCU            Pointer to current CU
 * \param uiAbsPartIdx    Address of block within CU
 * \param nWidth          Width of the block
 * \param nHeight         Height of the block
 * \param eCurRefPicList  Reference picture list
 * \param rCurMvField     Mv to be checked
 * \param rPairMVField    paired Mv based on bilateral assumption
 * \param uiMVCost        Cost of the Mv
 */
UInt TComPrediction::xFrucGetBilaMatchCost( TComDataCU * pcCU , UInt uiAbsPartIdx , Int nWidth , Int nHeight , RefPicList eCurRefPicList , const TComMvField & rCurMvField , TComMvField & rPairMVField , UInt uiMVCost )
{
  UInt uiCost = MAX_UINT;
  if( pcCU->getMvPair( eCurRefPicList , rCurMvField , rPairMVField ) )
  {
    RefPicList eTarRefPicList = ( RefPicList )( 1 - ( Int )eCurRefPicList );
    TComPicYuv * pRefPicYuvA = pcCU->getSlice()->getRefPic( eCurRefPicList , rCurMvField.getRefIdx() )->getPicYuvRec();
    TComPicYuv * pRefPicYuvB = pcCU->getSlice()->getRefPic( eTarRefPicList , rPairMVField.getRefIdx() )->getPicYuvRec();
    TComYuv * pYuvPredA = &m_acYuvPred[0];
    TComYuv * pYuvPredB = &m_acYuvPred[1];
    TComMv mvOffset( 0 , 0 );
    TComMv mvAp = rCurMvField.getMv() + mvOffset;
    xPredInterBlk( COMPONENT_Y , pcCU , pRefPicYuvA , uiAbsPartIdx , &mvAp , nWidth , nHeight , pYuvPredA , false , pcCU->getSlice()->getSPS()->getBitDepth( CHANNEL_TYPE_LUMA ) 
#if VCEG_AZ05_BIO                  
      ,false
#endif
, FRUC_MERGE_BILATERALMV );
    TComMv mvBp = rPairMVField.getMv() + mvOffset;
    xPredInterBlk( COMPONENT_Y , pcCU , pRefPicYuvB , uiAbsPartIdx , &mvBp , nWidth , nHeight , pYuvPredB , false , pcCU->getSlice()->getSPS()->getBitDepth( CHANNEL_TYPE_LUMA ) 
#if VCEG_AZ05_BIO                  
      ,false
#endif
, FRUC_MERGE_BILATERALMV );
    DistParam cDistParam;
    cDistParam.bApplyWeight = false;
    m_cFRUCRDCost.setDistParam( cDistParam , pcCU->getSlice()->getSPS()->getBitDepth( CHANNEL_TYPE_LUMA ) , pYuvPredA->getAddr( COMPONENT_Y , 0 ) , pYuvPredA->getStride( COMPONENT_Y ) ,
      pYuvPredB->getAddr( COMPONENT_Y , 0 ) , pYuvPredB->getStride( COMPONENT_Y ) , nWidth , nHeight , false );
#if VCEG_AZ06_IC
    cDistParam.bMRFlag = pcCU->getICFlag( uiAbsPartIdx );
#endif
    uiCost = cDistParam.DistFunc( &cDistParam ) + uiMVCost;
  }

  return( uiCost );
}

/**
 * \brief refine Mv for a block with bilateral matching or template matching and return the min cost so far
 *
 * \param pBestMvField    Pointer to the best Mv (Mv pair) so far
 * \param eCurRefPicList  Reference picture list
 * \param uiMinCost       Min cost so far
 * \param nSearchMethod   Search pattern to be used
 * \param pcCU            Pointer to current CU
 * \param uiAbsPartIdx    Address of block within CU
 * \param rMvStart        Searching center
 * \param nBlkWidth       Width of the block
 * \param nBlkHeight      Height of the block
 * \param bTM             Whether is template matching
 */
UInt TComPrediction::xFrucRefineMv( TComMvField * pBestMvField , RefPicList eCurRefPicList , UInt uiMinCost , Int nSearchMethod , TComDataCU * pCU , UInt uiAbsPartIdx , const TComMvField & rMvStart , Int nBlkWidth , Int nBlkHeight , Bool bTM 
#if JVET_F0032_UNI_BI_SELECTION
    , Bool bMvCostZero
#endif
)
{
#if VCEG_AZ07_MV_ADD_PRECISION_BIT_FOR_STORE
  Int nSearchStepShift = VCEG_AZ07_MV_ADD_PRECISION_BIT_FOR_STORE;
#else
  Int nSearchStepShift = 0;
#endif

  switch( nSearchMethod )
  {
  case 0:
    // cross
    uiMinCost = xFrucRefineMvSearch<FRUC_MERGE_MV_SEARCHPATTERN_CROSS>( pBestMvField , eCurRefPicList , pCU , uiAbsPartIdx , rMvStart , nBlkWidth , nBlkHeight , uiMinCost , bTM , nSearchStepShift );
    if( nSearchStepShift > 0 )
    {
#if JVET_B058_HIGH_PRECISION_MOTION_VECTOR_MC
      uiMinCost = xFrucRefineMvSearch<FRUC_MERGE_MV_SEARCHPATTERN_CROSS>( pBestMvField , eCurRefPicList , pCU , uiAbsPartIdx , rMvStart , nBlkWidth , nBlkHeight , uiMinCost , bTM , nSearchStepShift-1 , 1 );
#else
      uiMinCost = xFrucRefineMvSearch<FRUC_MERGE_MV_SEARCHPATTERN_CROSS>( pBestMvField , eCurRefPicList , pCU , uiAbsPartIdx , rMvStart , nBlkWidth , nBlkHeight , uiMinCost , bTM , 0 , 1 );
#endif
    }
    break;
  case 1:
    // square
    uiMinCost = xFrucRefineMvSearch<FRUC_MERGE_MV_SEARCHPATTERN_SQUARE>( pBestMvField , eCurRefPicList , pCU , uiAbsPartIdx , rMvStart , nBlkWidth , nBlkHeight , uiMinCost , bTM , nSearchStepShift
#if JVET_F0032_UNI_BI_SELECTION
        , MAX_UINT, bMvCostZero
#endif
        );
    if( nSearchStepShift > 0 )
    {
#if JVET_B058_HIGH_PRECISION_MOTION_VECTOR_MC
      uiMinCost = xFrucRefineMvSearch<FRUC_MERGE_MV_SEARCHPATTERN_CROSS>( pBestMvField , eCurRefPicList , pCU , uiAbsPartIdx , rMvStart , nBlkWidth , nBlkHeight , uiMinCost , bTM , nSearchStepShift-1 , 1 
#if JVET_F0032_UNI_BI_SELECTION
          , bMvCostZero
#endif
          );
#else
      uiMinCost = xFrucRefineMvSearch<FRUC_MERGE_MV_SEARCHPATTERN_CROSS>( pBestMvField , eCurRefPicList , pCU , uiAbsPartIdx , rMvStart , nBlkWidth , nBlkHeight , uiMinCost , bTM , 0 , 1 
#if JVET_F0032_UNI_BI_SELECTION
          , bMvCostZero
#endif
          );
#endif
    }
    break;
  case 2:
    // diamond
    uiMinCost = xFrucRefineMvSearch<FRUC_MERGE_MV_SEARCHPATTERN_DIAMOND>( pBestMvField , eCurRefPicList , pCU , uiAbsPartIdx , rMvStart , nBlkWidth , nBlkHeight , uiMinCost , bTM , nSearchStepShift
#if JVET_F0032_UNI_BI_SELECTION
        , MAX_UINT , bMvCostZero
#endif
        );
    uiMinCost = xFrucRefineMvSearch<FRUC_MERGE_MV_SEARCHPATTERN_CROSS>( pBestMvField , eCurRefPicList , pCU , uiAbsPartIdx , rMvStart , nBlkWidth , nBlkHeight , uiMinCost , bTM , nSearchStepShift , 1 
#if JVET_F0032_UNI_BI_SELECTION
        , bMvCostZero
#endif
        );
    if( nSearchStepShift > 0 )
    {
#if JVET_B058_HIGH_PRECISION_MOTION_VECTOR_MC
      uiMinCost = xFrucRefineMvSearch<FRUC_MERGE_MV_SEARCHPATTERN_CROSS>( pBestMvField , eCurRefPicList , pCU , uiAbsPartIdx , rMvStart , nBlkWidth , nBlkHeight , uiMinCost , bTM , nSearchStepShift-1  , 1 
#if JVET_F0032_UNI_BI_SELECTION
          , bMvCostZero
#endif
          );
#else
      uiMinCost = xFrucRefineMvSearch<FRUC_MERGE_MV_SEARCHPATTERN_CROSS>( pBestMvField , eCurRefPicList , pCU , uiAbsPartIdx , rMvStart , nBlkWidth , nBlkHeight , uiMinCost , bTM , 0 , 1 
#if JVET_F0032_UNI_BI_SELECTION
          , bMvCostZero
#endif
          );
#endif
    }
    break;
  case 3:
    // no refinement
    break;
  case 4:
    // hexagon
    uiMinCost = xFrucRefineMvSearch<FRUC_MERGE_MV_SEARCHPATTERN_HEXAGON>( pBestMvField , eCurRefPicList , pCU , uiAbsPartIdx , rMvStart , nBlkWidth , nBlkHeight , uiMinCost , bTM , nSearchStepShift 
#if JVET_F0032_UNI_BI_SELECTION
        , MAX_UINT, bMvCostZero
#endif
        );
    uiMinCost = xFrucRefineMvSearch<FRUC_MERGE_MV_SEARCHPATTERN_CROSS>( pBestMvField , eCurRefPicList , pCU , uiAbsPartIdx , rMvStart , nBlkWidth , nBlkHeight , uiMinCost , bTM , nSearchStepShift , 1 
#if JVET_F0032_UNI_BI_SELECTION
        , bMvCostZero
#endif
        );
    if( nSearchStepShift > 0 )
    {
#if JVET_B058_HIGH_PRECISION_MOTION_VECTOR_MC
      uiMinCost = xFrucRefineMvSearch<FRUC_MERGE_MV_SEARCHPATTERN_CROSS>( pBestMvField , eCurRefPicList , pCU , uiAbsPartIdx , rMvStart , nBlkWidth , nBlkHeight , uiMinCost , bTM , nSearchStepShift-1  , 1 
#if JVET_F0032_UNI_BI_SELECTION
          , bMvCostZero
#endif
          );
#else
      uiMinCost = xFrucRefineMvSearch<FRUC_MERGE_MV_SEARCHPATTERN_CROSS>( pBestMvField , eCurRefPicList , pCU , uiAbsPartIdx , rMvStart , nBlkWidth , nBlkHeight , uiMinCost , bTM , 0 , 1 
#if JVET_F0032_UNI_BI_SELECTION
          , bMvCostZero
#endif
          );
#endif
    }
    break;
  case 5:
    // adaptive cross 
    if( rMvStart.getRefIdx() != pBestMvField[eCurRefPicList].getRefIdx() || rMvStart.getMv() != pBestMvField[eCurRefPicList].getMv() )
    {
      uiMinCost = xFrucRefineMvSearch<FRUC_MERGE_MV_SEARCHPATTERN_CROSS>( pBestMvField , eCurRefPicList , pCU , uiAbsPartIdx , rMvStart , nBlkWidth , nBlkHeight , uiMinCost , bTM , nSearchStepShift 
#if JVET_F0032_UNI_BI_SELECTION
          , MAX_UINT, bMvCostZero
#endif
          );
      if( nSearchStepShift > 0 )
      {
#if JVET_B058_HIGH_PRECISION_MOTION_VECTOR_MC
        uiMinCost = xFrucRefineMvSearch<FRUC_MERGE_MV_SEARCHPATTERN_CROSS>( pBestMvField , eCurRefPicList , pCU , uiAbsPartIdx , rMvStart , nBlkWidth , nBlkHeight , uiMinCost , bTM , nSearchStepShift-1 , 1 
#if JVET_F0032_UNI_BI_SELECTION
            , bMvCostZero
#endif
            );
#else
        uiMinCost = xFrucRefineMvSearch<FRUC_MERGE_MV_SEARCHPATTERN_CROSS>( pBestMvField , eCurRefPicList , pCU , uiAbsPartIdx , rMvStart , nBlkWidth , nBlkHeight , uiMinCost , bTM , 0 , 1 
#if JVET_F0032_UNI_BI_SELECTION
            , bMvCostZero
#endif
            );
#endif
      }
    }
    break;
  default:
    assert( 0 );
  }
  
  return( uiMinCost );
}

/**
 * \brief refine Mv for a block with bilateral matching or template matching and return the min cost so far
 *
 * \param pBestMvField    Pointer to the best Mv (Mv pair) so far
 * \param eCurRefPicList  Reference picture list
 * \param pcCU            Pointer to current CU
 * \param uiAbsPartIdx    Address of block within CU
 * \param rMvStart        Searching center
 * \param nBlkWidth       Width of the block
 * \param nBlkHeight      Height of the block
 * \param uiMinDist       Min cost so far
 * \param bTM             Whether is template matching
 * \param nSearchStepShift Indicate the searching step, 0 for 1/8 pel, 1 for 1/4 pel
 * \param uiMaxSearchRounds Max rounds of pattern searching
 */
template<Int SearchPattern>
UInt TComPrediction::xFrucRefineMvSearch( TComMvField * pBestMvField , RefPicList eCurRefPicList , TComDataCU * pCU , UInt uiAbsPartIdx , TComMvField const & rMvStart , Int nBlkWidth , Int nBlkHeight , UInt uiMinDist , Bool bTM , Int nSearchStepShift , UInt uiMaxSearchRounds 
#if JVET_F0032_UNI_BI_SELECTION
    , Bool bMvCostZero
#endif
)
{
  const TComMv mvSearchOffsetCross[4] = { TComMv( 0 , 1 ) , TComMv( 1 , 0 ) , TComMv( 0 , -1 ) , TComMv( -1 , 0 ) };
  const TComMv mvSearchOffsetSquare[8] = { TComMv( -1 , 1 ) , TComMv( 0 , 1 ) , TComMv( 1 , 1 ) , TComMv( 1 , 0 ) , TComMv( 1 , -1 ) , TComMv( 0 , -1 ) , TComMv( -1 , -1 ) , TComMv( -1 , 0 )  };
  const TComMv mvSearchOffsetDiamond[8] = { TComMv( 0 , 2 ) , TComMv( 1 , 1 ) , TComMv( 2 , 0 ) , TComMv( 1 , -1 ) , TComMv( 0 , -2 ) , TComMv( -1 , -1 ) , TComMv( -2 , 0 ) , TComMv( -1 , 1 ) };
  const TComMv mvSearchOffsetHexagon[6] = { TComMv( 2 , 0 ) , TComMv( 1 , 2 ) , TComMv( -1 , 2 ) , TComMv( -2 , 0 ) , TComMv( -1 , -2 ) , TComMv( 1 , -2 ) };

  Int nDirectStart = 0 , nDirectEnd = 0 , nDirectRounding = 0 , nDirectMask = 0;
  const TComMv * pSearchOffset;
  if( SearchPattern == FRUC_MERGE_MV_SEARCHPATTERN_CROSS )
  {
    nDirectEnd = 3;
    nDirectRounding = 4;
    nDirectMask = 0x03;
    pSearchOffset = mvSearchOffsetCross;
  }
  else if( SearchPattern == FRUC_MERGE_MV_SEARCHPATTERN_SQUARE )
  {
    nDirectEnd = 7;
    nDirectRounding = 8;
    nDirectMask = 0x07;
    pSearchOffset = mvSearchOffsetSquare;
  }
  else if( SearchPattern == FRUC_MERGE_MV_SEARCHPATTERN_DIAMOND )
  {
    nDirectEnd = 7;
    nDirectRounding = 8;
    nDirectMask = 0x07;
    pSearchOffset = mvSearchOffsetDiamond;
  }
  else if( SearchPattern == FRUC_MERGE_MV_SEARCHPATTERN_HEXAGON )
  {
    nDirectEnd = 5;
    pSearchOffset = mvSearchOffsetHexagon;
  }
  else
  {
    assert( 0 );
  }

  Int nBestDirect;
  const Int & rSearchRange = pCU->getSlice()->getSPS()->getFRUCRefineRange();
  for( UInt uiRound = 0 ; uiRound < uiMaxSearchRounds ; uiRound++ )
  {
    nBestDirect = -1;
    TComMvField mvCurCenter = pBestMvField[eCurRefPicList];
    for( Int nIdx = nDirectStart ; nIdx <= nDirectEnd ; nIdx++ )
    {
      Int nDirect;
      if( SearchPattern == FRUC_MERGE_MV_SEARCHPATTERN_HEXAGON )
      {
        nDirect = nIdx < 0 ? nIdx + 6 : nIdx >= 6 ? nIdx - 6 : nIdx;
      }
      else
      {
        nDirect = ( nIdx + nDirectRounding ) & nDirectMask;
      }

      TComMv mvOffset = pSearchOffset[nDirect];
      mvOffset <<= nSearchStepShift;
      TComMvField mvCand = mvCurCenter;
      mvCand.getMv() += mvOffset;
      UInt uiCost = xFrucGetMvCost( rMvStart.getMv() , mvCand.getMv() , rSearchRange , FRUC_MERGE_REFINE_MVWEIGHT );
#if JVET_F0032_UNI_BI_SELECTION
      if (bMvCostZero && uiCost != MAX_UINT)
      {
          uiCost = 0;
      }
#endif
      if( uiCost > uiMinDist )
        continue;
      TComMvField mvPair;

      if( bTM )
      {
        if( pCU->isSameMVField( eCurRefPicList , mvCand , ( RefPicList )( !eCurRefPicList ) , pBestMvField[!eCurRefPicList] ) )
          continue;
        uiCost = xFrucGetTempMatchCost( pCU , uiAbsPartIdx , nBlkWidth , nBlkHeight , eCurRefPicList , mvCand , uiCost );
      }
      else
      {
        uiCost = xFrucGetBilaMatchCost( pCU , uiAbsPartIdx , nBlkWidth , nBlkHeight , eCurRefPicList , mvCand , mvPair , uiCost ); 
      }
      if( uiCost < uiMinDist )
      {
        uiMinDist = uiCost;
        pBestMvField[eCurRefPicList] = mvCand;
        if( !bTM )
          pBestMvField[!eCurRefPicList] = mvPair;
        nBestDirect = nDirect;
      }
    }

    if( nBestDirect == -1 )
      break;
    Int nStep = 1;
    if( SearchPattern == FRUC_MERGE_MV_SEARCHPATTERN_SQUARE || SearchPattern == FRUC_MERGE_MV_SEARCHPATTERN_DIAMOND )
    {
      nStep = 2 - ( nBestDirect & 0x01 );
    }
    nDirectStart = nBestDirect - nStep;
    nDirectEnd = nBestDirect + nStep;
  }

  return( uiMinDist );
}

/**
 * \brief Find Mv predictor for a block based on template matching
 *
 * \param pcCU            Pointer to current CU
 * \param uiPUIdx         PU Index
 * \param eTargetRefPicList The reference list for the Mv predictor
 * \param nTargetRefIdx   The reference index for the Mv predictor
 */
Bool TComPrediction::xFrucFindBlkMv4Pred( TComDataCU * pCU , UInt uiPUIdx , RefPicList eTargetRefPicList , Int nTargetRefIdx
#if JVET_E0060_FRUC_CAND
                                        , AMVPInfo* pInfo
#endif
                                        )
{
  Bool bAvailable = false;
  if( pCU->getSlice()->getSPS()->getUseFRUCMgrMode() )
  {
    UInt uiAbsPartIdx = 0;
    Int nWidth = 0 , nHeight = 0;
    pCU->getPartIndexAndSize( uiPUIdx , uiAbsPartIdx , nWidth , nHeight );
    if( xFrucGetCurBlkTemplate( pCU , uiAbsPartIdx , nWidth , nHeight ) )
    {
      // find best start
#if JVET_E0060_FRUC_CAND
      xFrucCollectBlkStartMv( pCU , uiPUIdx , eTargetRefPicList , nTargetRefIdx , pInfo );
#else
      xFrucCollectBlkStartMv( pCU , uiPUIdx , eTargetRefPicList , nTargetRefIdx );
#endif
      TComMvField mvStart[2] , mvFinal[2];
      UInt uiMinCost = xFrucFindBestMvFromList( mvStart , eTargetRefPicList , pCU , uiAbsPartIdx , mvStart[eTargetRefPicList] , nWidth , nHeight , true , false );
      if( mvStart[eTargetRefPicList].getRefIdx() >= 0 )
      {
        // refine Mv
        mvFinal[eTargetRefPicList] = mvStart[eTargetRefPicList];
        uiMinCost = xFrucRefineMv( mvFinal , eTargetRefPicList , uiMinCost , 2 , pCU , uiAbsPartIdx , mvStart[eTargetRefPicList] , nWidth , nHeight , true );
        bAvailable = true;
        // save Mv
#if JVET_C0024_QTBT
        pCU->getCUMvField( eTargetRefPicList )->setAllMv( mvFinal[eTargetRefPicList].getMv(), SIZE_2Nx2N , uiAbsPartIdx , 0 , uiPUIdx ); 
#else
        pCU->getCUMvField( eTargetRefPicList )->setAllMv( mvFinal[eTargetRefPicList].getMv(), pCU->getPartitionSize( 0 ) , uiAbsPartIdx , 0 , uiPUIdx ); 
#endif
      }
    }
  }

  return( bAvailable );
}

/**
 * \brief Find Mv for a block based on template matching or bilateral matching
 *
 * \param pcCU            Pointer to current CU
 * \param uiPUIdx         PU Index
 */
Bool TComPrediction::xFrucFindBlkMv( TComDataCU * pCU , UInt uiPUIdx )
{
  Bool bAvailable = false;
  UInt uiAbsPartIdx = 0;
  Int nWidth = 0 , nHeight = 0;
  pCU->getPartIndexAndSize( uiPUIdx , uiAbsPartIdx , nWidth , nHeight );
  TComMvField mvStart[2] , mvFinal[2];

  const Int nSearchMethod = 2;
  if( pCU->getFRUCMgrMode( uiAbsPartIdx ) == FRUC_MERGE_BILATERALMV )
  {
    if( !pCU->getSlice()->getSPS()->getUseFRUCMgrMode() || pCU->getSlice()->isInterP() )
      return( false );

    xFrucCollectBlkStartMv( pCU , uiPUIdx );
    RefPicList eBestRefPicList = REF_PIC_LIST_0;
    UInt uiMinCost = xFrucFindBestMvFromList( mvStart , eBestRefPicList , pCU , uiAbsPartIdx , mvStart[eBestRefPicList] , nWidth , nHeight , false , false );
    if( mvStart[eBestRefPicList].getRefIdx() >= 0 )
    {
      mvFinal[0] = mvStart[0];
      mvFinal[1] = mvStart[1];
      uiMinCost = xFrucRefineMv( mvFinal , eBestRefPicList , uiMinCost , nSearchMethod , pCU , uiAbsPartIdx , mvStart[eBestRefPicList] , nWidth , nHeight , false );
#if JVET_E0060_FRUC_CAND
      //Save best list for sub-blocks
      m_bilatBestRefPicList = eBestRefPicList;
#endif
      bAvailable = true;
    }
  }
  else if( pCU->getFRUCMgrMode( uiAbsPartIdx ) == FRUC_MERGE_TEMPLATE )
  {
    if( !pCU->getSlice()->getSPS()->getUseFRUCMgrMode() )
      return( false );
    if( !xFrucGetCurBlkTemplate( pCU , uiAbsPartIdx , nWidth , nHeight ) )
      return( false );

    xFrucCollectBlkStartMv( pCU , uiPUIdx );
    UInt uiMinCost[2];
    // find the best Mvs from the two lists first and then refine Mvs: try to avoid duplicated Mvs
    for( Int nRefPicList = 0 ; nRefPicList < 2 ; nRefPicList++ )
    {
      RefPicList eCurRefPicList = ( RefPicList )nRefPicList;
      uiMinCost[nRefPicList] = xFrucFindBestMvFromList( mvStart , eCurRefPicList , pCU , uiAbsPartIdx , mvStart[nRefPicList] , nWidth , nHeight , true , false );
    }
    mvFinal[0] = mvStart[0];
    mvFinal[1] = mvStart[1];
    for( Int nRefPicList = 0 ; nRefPicList < 2 ; nRefPicList++ )
    {
      if( mvStart[nRefPicList].getRefIdx() >= 0 )
      {
        uiMinCost[nRefPicList] = xFrucRefineMv( mvFinal , ( RefPicList )nRefPicList , uiMinCost[nRefPicList] , nSearchMethod , pCU , uiAbsPartIdx , mvStart[nRefPicList] , nWidth , nHeight , true 
#if JVET_F0032_UNI_BI_SELECTION
            , true
#endif
        );
        bAvailable = true;
      }
    }
#if JVET_F0032_UNI_BI_SELECTION
    if (mvFinal[0].getRefIdx() >= 0 && mvFinal[1].getRefIdx() >= 0)
    {
        //calculate cost for bi-refinement
        xFrucUpdateTemplate(pCU, uiAbsPartIdx, nWidth, nHeight, REF_PIC_LIST_0, mvFinal[REF_PIC_LIST_0]);
        UInt uiCostBi = xFrucGetTempMatchCost(pCU, uiAbsPartIdx, nWidth, nHeight, REF_PIC_LIST_1, mvFinal[REF_PIC_LIST_1], 0);

        if (2 * uiCostBi <= 5 * min(uiMinCost[0], uiMinCost[1]) )  // if (uiMinCostBi <= 5/4*2*min(uiMinCost[0], uiMinCost[1]) )
        {
            //do nothing
        }
        else if (uiMinCost[0] <= uiMinCost[1])
        {
            mvFinal[1].setRefIdx(-1);
        }
        else
        {
            mvFinal[0].setRefIdx(-1);
        }
    }
#endif
  }
  else
  {
    assert( 0 );
  }

  if( bAvailable )
  {
    // save Mv
#if JVET_C0024_QTBT
    pCU->getCUMvField( REF_PIC_LIST_0 )->setAllMvField( mvFinal[0] , SIZE_2Nx2N , uiAbsPartIdx , 0 , uiPUIdx ); 
    pCU->getCUMvField( REF_PIC_LIST_1 )->setAllMvField( mvFinal[1] , SIZE_2Nx2N , uiAbsPartIdx , 0 , uiPUIdx ); 
#else
    pCU->getCUMvField( REF_PIC_LIST_0 )->setAllMvField( mvFinal[0] , pCU->getPartitionSize( uiAbsPartIdx ) , uiAbsPartIdx , 0 , uiPUIdx ); 
    pCU->getCUMvField( REF_PIC_LIST_1 )->setAllMvField( mvFinal[1] , pCU->getPartitionSize( uiAbsPartIdx ) , uiAbsPartIdx , 0 , uiPUIdx ); 
#endif
    UInt uiDir = ( mvFinal[0].getRefIdx() >= 0 ) + ( ( mvFinal[1].getRefIdx() >=0 ) << 1 );
    pCU->setInterDirSubParts( uiDir , uiAbsPartIdx , uiPUIdx , pCU->getDepth( uiAbsPartIdx ) );
  }

  return( bAvailable );
}

/**
 * \brief Refine Mv for each sub-block of the block based on bilateral matching or template matching
 *
 * \param pcCU            Pointer to current CU
 * \param uiDepth         CU depth
 * \param uiPUIdx         PU Index
 * \param bTM             Whether is template matching
 */
Bool TComPrediction::xFrucRefineSubBlkMv( TComDataCU * pCU , UInt uiDepth , UInt uiPUIdx , Bool bTM )
{
  TComCUMvField * pCuMvField0 = pCU->getCUMvField( REF_PIC_LIST_0 );
  TComCUMvField * pCuMvField1 = pCU->getCUMvField( REF_PIC_LIST_1 );
  UInt uiAbsPartIdx = 0;
  Int nWidth = 0 , nHeight = 0;
  pCU->getPartIndexAndSize( uiPUIdx , uiAbsPartIdx , nWidth , nHeight );
  Int nRefineBlockSize = xFrucGetSubBlkSize( pCU , uiAbsPartIdx , nWidth , nHeight );
  UInt uiIdxStep = ( nRefineBlockSize * nRefineBlockSize ) >> 4;
  Int nRefineBlkStep = nRefineBlockSize >> 2;
  const Int nSearchMethod = 5;
#if COM16_C806_VCEG_AZ10_SUB_PU_TMVP
  UInt uiSubBlkRasterIdx = 0;
  UInt uiSubBlkRasterStep = nRefineBlkStep * nRefineBlkStep;
#endif
  for( Int y = 0 , yBlk4Offset = 0 ; y < nHeight ; y += nRefineBlockSize , yBlk4Offset += pCU->getPic()->getNumPartInCtuWidth() * nRefineBlkStep )
  {
    for( Int x = 0 , xBlk4Offset = 0 ; x < nWidth ; x += nRefineBlockSize , xBlk4Offset += nRefineBlkStep )
    {
      UInt uiRasterOrder = g_auiZscanToRaster[uiAbsPartIdx+pCU->getZorderIdxInCtu()] + yBlk4Offset + xBlk4Offset;
      UInt uiSubBlkIdx = g_auiRasterToZscan[uiRasterOrder] - pCU->getZorderIdxInCtu();

      // start from the best Mv of the full block
      TComMvField mvStart[2] , mvFinal[2];
      mvStart[0].setMvField( pCuMvField0->getMv( uiSubBlkIdx ) , pCuMvField0->getRefIdx( uiSubBlkIdx ) );
      mvStart[1].setMvField( pCuMvField1->getMv( uiSubBlkIdx ) , pCuMvField1->getRefIdx( uiSubBlkIdx ) );
      mvFinal[0] = mvStart[0];
      mvFinal[1] = mvStart[1];

      // refinement
      if( bTM )
      {
        if( !xFrucGetCurBlkTemplate( pCU , uiSubBlkIdx , nRefineBlockSize , nRefineBlockSize ) )
        {
#if JVET_E0060_FRUC_CAND && COM16_C806_VCEG_AZ10_SUB_PU_TMVP
          uiSubBlkRasterIdx += uiSubBlkRasterStep;
#endif
          continue;
        }
        for( Int nRefPicList = 0 ; nRefPicList < 2 ; nRefPicList++ )
        {
          if( mvStart[nRefPicList].getRefIdx() >= 0 )
          {
            RefPicList eCurRefPicList = ( RefPicList )nRefPicList;
            xFrucCollectSubBlkStartMv( pCU , uiSubBlkIdx , eCurRefPicList , mvStart[eCurRefPicList] , nRefineBlockSize , nRefineBlockSize 
#if COM16_C806_VCEG_AZ10_SUB_PU_TMVP
              , uiSubBlkRasterIdx , uiSubBlkRasterStep
#endif
#if FRUC_FIX
              , nWidth >> 2, (UInt)((y >> 2) * (nWidth >> 2) + (x >> 2))
#endif

              );
            UInt uiMinCost = xFrucFindBestMvFromList( mvFinal , eCurRefPicList , pCU , uiSubBlkIdx , mvStart[eCurRefPicList] , nRefineBlockSize , nRefineBlockSize , bTM , true );
            uiMinCost = xFrucRefineMv( mvFinal , eCurRefPicList , uiMinCost , nSearchMethod , pCU , uiSubBlkIdx , mvStart[eCurRefPicList] , nRefineBlockSize , nRefineBlockSize , bTM );
          }
        }
      }
      else
      {
#if JVET_E0060_FRUC_CAND
        //Use same reference frame as for entire block, i.e. same refidx and same list
        RefPicList eBestRefPicList = m_bilatBestRefPicList;
        xFrucCollectSubBlkStartMv( pCU , uiSubBlkIdx , eBestRefPicList , mvStart[eBestRefPicList] , nRefineBlockSize , nRefineBlockSize
#if COM16_C806_VCEG_AZ10_SUB_PU_TMVP
          , uiSubBlkRasterIdx , uiSubBlkRasterStep
#endif
#if FRUC_FIX
          , nWidth >> 2, (UInt)((y >> 2) * (nWidth >> 2) + (x >> 2))
#endif
          );
#else
        xFrucCollectSubBlkStartMv( pCU , uiSubBlkIdx , REF_PIC_LIST_0 , mvStart[REF_PIC_LIST_0] , nRefineBlockSize , nRefineBlockSize 
#if COM16_C806_VCEG_AZ10_SUB_PU_TMVP
          , uiSubBlkRasterIdx , uiSubBlkRasterStep
#endif
#if FRUC_FIX
          , nWidth >> 2, (UInt)((y >> 2) * (nWidth >> 2) + (x >> 2))
#endif

          );
        RefPicList eBestRefPicList = REF_PIC_LIST_0;
#endif
        UInt uiMinCost = xFrucFindBestMvFromList( mvFinal , eBestRefPicList , pCU , uiSubBlkIdx , mvStart[eBestRefPicList] , nRefineBlockSize , nRefineBlockSize , bTM , true );
        uiMinCost = xFrucRefineMv( mvFinal , eBestRefPicList , uiMinCost , nSearchMethod , pCU , uiSubBlkIdx , mvStart[eBestRefPicList] , nRefineBlockSize , nRefineBlockSize , bTM );
      }

      // save Mv
      if( !( mvFinal[0] == mvStart[0] && mvFinal[1] == mvStart[1] ) )
      {
        UInt uiDir = ( mvFinal[0].getRefIdx() >= 0 ) + ( ( mvFinal[1].getRefIdx() >=0 ) << 1 );
        for( UInt n = 0 , uiOffset = uiSubBlkIdx ; n < uiIdxStep ; n++ , uiOffset++ )
        {
          pCU->setInterDir( uiOffset , uiDir );
          pCuMvField0->setMv( mvFinal[0].getMv() , uiOffset );
          pCuMvField0->setRefIdx( mvFinal[0].getRefIdx() , uiOffset );
          pCuMvField1->setMv( mvFinal[1].getMv() , uiOffset );
          pCuMvField1->setRefIdx( mvFinal[1].getRefIdx() , uiOffset );
        }
      }
#if COM16_C806_VCEG_AZ10_SUB_PU_TMVP
      uiSubBlkRasterIdx += uiSubBlkRasterStep;
#endif
    }
  }

  return( true );
}

/**
 * \brief Whether a Mv has been checked (in a temp list)
 *
 * \param rMvField        Mv info
 * \param rList           Temp list of Mv
 */
Bool TComPrediction::xFrucIsInList( const TComMvField & rMvField , std::list<TComMvField> & rList )
{
  std::list<TComMvField>::iterator pos = rList.begin();
  while( pos != rList.end() )
  {
    if( rMvField == *pos )
      return( true );
    pos++;
  }
  return( false );
}

/**
 * \brief Insert a Mv to the list to be checked
 *
 * \param rMvField        Mv info
 * \param rList           Temp list of Mv
 */
Void TComPrediction::xFrucInsertMv2StartList( const TComMvField & rMvField , std::list<TComMvField> & rList )
{
  // do not use zoom in FRUC for now
  if( xFrucIsInList( rMvField , rList ) == false )
    rList.push_back( rMvField );
}


/**
 * \brief Collect Mv candidates for a block
 *
 * \param pcCU            Pointer to current CU
 * \param uiPUIdx         PU Index
 * \param eTargetRefPicList The reference list for the Mv predictor
 * \param nTargetRefIdx   The reference index for the Mv predictor
 */
Void TComPrediction::xFrucCollectBlkStartMv( TComDataCU * pCU , UInt uiPUIdx , RefPicList eTargetRefList , Int nTargetRefIdx
#if JVET_E0060_FRUC_CAND
                                           , AMVPInfo* pInfo
#endif
                                           )
{
#if JVET_E0060_FRUC_CAND
  UInt uiAddrOffset = 0;
  Int nWidth = 0, nHeight = 0;
  pCU->getPartIndexAndSize( uiPUIdx , uiAddrOffset , nWidth , nHeight );

  m_listMVFieldCand[0].clear();
  m_listMVFieldCand[1].clear();

  if ((nTargetRefIdx >= 0) && pInfo)   //Here we are in AMVP mode
  {
    // add AMVP candidates to the list
    for (Int nAMVPIndex=0; nAMVPIndex<pInfo->iN; nAMVPIndex++)
    {
      TComMvField mvCand;
      mvCand.setMvField(pInfo->m_acMvCand[nAMVPIndex], nTargetRefIdx);
      xFrucInsertMv2StartList(mvCand, m_listMVFieldCand[eTargetRefList]);
    }
  }
#endif

  // get merge candidates
  TComMvField  cMvFieldNeighbours[ 2 * MRG_MAX_NUM_CANDS ]; // double length for mv of both lists
  UChar uhInterDirNeighbours[MRG_MAX_NUM_CANDS];
  Int numValidMergeCand = 0;
#if !JVET_E0060_FRUC_CAND
  UInt uiAddrOffset = 0;
  Int nWidth = 0 , nHeight = 0;
#endif
#if VCEG_AZ06_IC
  Bool abICFlag[MRG_MAX_NUM_CANDS];
#endif
#if !JVET_E0060_FRUC_CAND
  pCU->getPartIndexAndSize( uiPUIdx , uiAddrOffset , nWidth , nHeight );
#endif
  pCU->getInterMergeCandidates( uiAddrOffset, uiPUIdx, cMvFieldNeighbours, uhInterDirNeighbours, numValidMergeCand 
#if VCEG_AZ06_IC
    , abICFlag
#endif
#if COM16_C806_VCEG_AZ10_SUB_PU_TMVP
    , m_eMergeCandTypeNieghors , m_cMvFieldSP , m_uhInterDirSP 
#endif
    );

  // add merge candidates to the list
#if !JVET_E0060_FRUC_CAND
  m_listMVFieldCand[0].clear();
  m_listMVFieldCand[1].clear();
#endif
  for( Int nMergeIndex = 0; nMergeIndex < numValidMergeCand + numValidMergeCand ; nMergeIndex++ )
  {
    if( cMvFieldNeighbours[nMergeIndex].getRefIdx() >= 0 
#if COM16_C806_VCEG_AZ10_SUB_PU_TMVP
     && m_eMergeCandTypeNieghors[nMergeIndex>>1] == MGR_TYPE_DEFAULT_N      
#endif
      )
    {
      if( nTargetRefIdx >= 0 
        && ( cMvFieldNeighbours[nMergeIndex].getRefIdx() != nTargetRefIdx 
        || ( nMergeIndex & 0x01 ) != ( Int )eTargetRefList ) )
        continue;
      xFrucInsertMv2StartList( cMvFieldNeighbours[nMergeIndex] , m_listMVFieldCand[nMergeIndex&0x01] );
    }
  }

  // add uni-lateral candidates to the list
  if( pCU->getSlice()->getSPS()->getUseFRUCMgrMode() )
  {
    TComDataCU * pCUFRUC = pCU->getPic()->getCtu( pCU->getCtuRsAddr() );
    TComMvField mvCand;
    UInt uiRasterBase = g_auiZscanToRaster[pCU->getZorderIdxInCtu() + uiAddrOffset];
    for( Int y = 0 , yOffset = 0 ; y < nHeight ; y += MIN_PU_SIZE , yOffset += pCU->getPic()->getNumPartInCtuWidth() )
    {
      for( Int x = 0 , xOffset = 0 ; x < nWidth ; x += MIN_PU_SIZE , xOffset++ )
      {
        if( x != 0 && x + x != nWidth )
          continue;
        if( y != 0 && y + y != nHeight )
          continue;
        UInt idx = g_auiRasterToZscan[uiRasterBase+yOffset+xOffset];
        for( Int nList = 0 ; nList < 2 ; nList++ )
        {
          RefPicList eCurList = ( RefPicList )nList;
          if( pCUFRUC->getFRUCUniLateralMVField( eCurList )->getRefIdx( idx ) >= 0 )
          {
            if( nTargetRefIdx >= 0 
              && ( pCUFRUC->getFRUCUniLateralMVField( eCurList )->getRefIdx( idx ) != nTargetRefIdx || eCurList != eTargetRefList ) )
              continue;
            mvCand.setMvField( pCUFRUC->getFRUCUniLateralMVField( eCurList )->getMv( idx ) , pCUFRUC->getFRUCUniLateralMVField( eCurList )->getRefIdx( idx ) );
            xFrucInsertMv2StartList( mvCand , m_listMVFieldCand[nList&0x01] );
          }
        }
      }
    }
  }

#if JVET_E0060_FRUC_CAND
  // add some neighbors if not already present
  UInt uiAbsPartIdxNeighbor = 0;
  TComDataCU *pNeighbor = NULL;
  Int nbSpatialCandTested = NB_FRUC_CAND_ADDED;
  for (Int neighbor=0; neighbor<nbSpatialCandTested; neighbor++)
  {
    pNeighbor = NULL;
    if (neighbor == 0)       // top neighbor
      pNeighbor = pCU->getPUAbove(uiAbsPartIdxNeighbor, uiAddrOffset + pCU->getZorderIdxInCtu());
    else if (neighbor == 1)  // left neighbor
      pNeighbor = pCU->getPULeft(uiAbsPartIdxNeighbor, uiAddrOffset + pCU->getZorderIdxInCtu());
    else if (neighbor == 2)  // top-left neighbor
      pNeighbor = pCU->getPUAboveLeft(uiAbsPartIdxNeighbor, uiAddrOffset + pCU->getZorderIdxInCtu());
    else if (neighbor == 3)  // top-right neighbor
    {
      UInt uiPartIdxLT, uiPartIdxRT;
      pCU->deriveLeftRightTopIdxGeneral(uiAddrOffset, uiPUIdx, uiPartIdxLT, uiPartIdxRT);
      pNeighbor = pCU->getPUAboveRight(uiAbsPartIdxNeighbor, uiPartIdxRT);
    }
    else if (neighbor == 4)  // below-left neighbor
    {
      UInt uiPartIdxLB;
      pCU->deriveLeftBottomIdxGeneral(uiAddrOffset, uiPUIdx, uiPartIdxLB);
      pNeighbor = pCU->getPUBelowLeft(uiAbsPartIdxNeighbor, uiPartIdxLB);
    }
    else
      assert(0);

    if (pNeighbor)
    {
      for (Int nList=0; nList<2; nList++)
      {
        RefPicList eCurList = (RefPicList)nList;
        if (pNeighbor->getCUMvField(eCurList)->getRefIdx(uiAbsPartIdxNeighbor) >= 0)
        {
          if ((nTargetRefIdx >= 0) &&
              ((pNeighbor->getCUMvField(eCurList)->getRefIdx(uiAbsPartIdxNeighbor) != nTargetRefIdx) ||
               (eCurList != eTargetRefList)))
            continue;
          TComMvField mvCand;
          mvCand.setMvField(pNeighbor->getCUMvField(eCurList)->getMv(uiAbsPartIdxNeighbor), pNeighbor->getCUMvField(eCurList)->getRefIdx(uiAbsPartIdxNeighbor));
          xFrucInsertMv2StartList(mvCand, m_listMVFieldCand[nList&0x01]);
        }
      }
    }
  }
#endif
}

/**
 * \brief Collect Mv candidates for a sub-block
 *
 * \param pcCU            Pointer to current CU
 * \param uiPUIdx         PU Index
 * \param eRefPicList     The reference list for the Mv predictor
 * \param rMvStart        The searching center
 * \param nSubBlkWidth    Block width
 * \param nSubBlkHeight   Block height
 * \param uiSubBlkRasterIdx Sub-block index in raster scan
 * \param uiSubBlkRasterStep Sub-block step in raster scan
 */
Void TComPrediction::xFrucCollectSubBlkStartMv( TComDataCU * pCU , UInt uiAbsPartIdx , RefPicList eRefPicList , const TComMvField & rMvStart , Int nSubBlkWidth , Int nSubBlkHeight 
#if COM16_C806_VCEG_AZ10_SUB_PU_TMVP
  , UInt uiSubBlkRasterIdx , UInt uiSubBlkRasterStep
#endif
#if FRUC_FIX
  , UInt numPartPerLine, UInt uiFidx
#endif
  )
{
  std::list<TComMvField> & rStartMvList = m_listMVFieldCand[eRefPicList];
  rStartMvList.clear();

  // start Mv
  xFrucInsertMv2StartList( rMvStart , rStartMvList );

#if JVET_E0060_FRUC_CAND
  // add some neighbors in addition to top and left ones
  UInt uiAbsPartIdxNeighbor = 0;
  TComDataCU *pNeighbor = NULL;
  for (Int neighbor=0; neighbor<max(2,NB_FRUC_CAND_ADDED_SUB); neighbor++)
  {
    pNeighbor = NULL;
    if (neighbor == 0)       // top neighbor
      pNeighbor = pCU->getPUAbove(uiAbsPartIdxNeighbor, uiAbsPartIdx + pCU->getZorderIdxInCtu());
    else if (neighbor == 1)  // left neighbor
      pNeighbor = pCU->getPULeft(uiAbsPartIdxNeighbor, uiAbsPartIdx + pCU->getZorderIdxInCtu());
    else if (neighbor == 2)  // top-left neighbor
      pNeighbor = pCU->getPUAboveLeft(uiAbsPartIdxNeighbor, uiAbsPartIdx + pCU->getZorderIdxInCtu());
    else if (neighbor == 3)  // top-right neighbor
    {
      UInt uiPartIdxLT, uiPartIdxRT;
      pCU->deriveLeftRightTopIdx(uiAbsPartIdx, uiPartIdxLT, uiPartIdxRT);
      pNeighbor = pCU->getPUAboveRight(uiAbsPartIdxNeighbor, uiPartIdxRT);
    }
    else if (neighbor == 4)  // below-left neighbor
    {
      UInt uiPartIdxLB;
      pCU->deriveLeftBottomIdx(uiAbsPartIdx, uiPartIdxLB);
      pNeighbor = pCU->getPUBelowLeft(uiAbsPartIdxNeighbor, uiPartIdxLB);
    }
    else
      assert(0);

    if (pNeighbor && (pNeighbor->getCUMvField(eRefPicList)->getRefIdx(uiAbsPartIdxNeighbor) == rMvStart.getRefIdx()))
    {
      TComMvField mvCand;
      mvCand.setMvField(pNeighbor->getCUMvField(eRefPicList)->getMv(uiAbsPartIdxNeighbor), rMvStart.getRefIdx());
      xFrucInsertMv2StartList(mvCand, rStartMvList);
    }
  }
#endif

#if !JVET_E0060_FRUC_CAND
  // zero Mv
  TComMvField mvZero;
  mvZero.setRefIdx( rMvStart.getRefIdx() );
  mvZero.getMv().setZero();
  xFrucInsertMv2StartList( mvZero , rStartMvList );
#endif

  Int nCurPOC = pCU->getSlice()->getPOC();
  Int nCurRefPOC = pCU->getSlice()->getRefPOC( eRefPicList , rMvStart.getRefIdx() );

  // scaled TMVP, collocated positions and bottom right positions
  UInt uiCUAddr[2] = { pCU->getCtuRsAddr() , 0 };
  UInt uiColBlockIdx[2] = { uiAbsPartIdx + pCU->getZorderIdxInCtu() , 0 };
#if JVET_E0060_FRUC_CAND
  Int nMaxPositions = 1;
#else
  Int nMaxPositions = 1 + pCU->getBlockBelowRight( uiAbsPartIdx , nSubBlkWidth , nSubBlkHeight , uiCUAddr[1] , uiColBlockIdx[1] );
#endif
  for( Int n = 0 ; n < nMaxPositions ; n++ )
  {
    for( Int nRefIdx = pCU->getSlice()->getNumRefIdx( eRefPicList ) - 1 ; nRefIdx >= 0 ; nRefIdx-- )
    {
      TComMvField mvCand;
      TComPic * pColPic = pCU->getSlice()->getRefPic( eRefPicList , nRefIdx );
      Int nColPOC = pColPic->getPOC();
      TComDataCU * pColCU = pColPic->getCtu( uiCUAddr[n] );
      for( Int nRefListColPic = 0 ; nRefListColPic < 2 ; nRefListColPic++ )
      {
        Int nRefIdxColPic = pColCU->getCUMvField( ( RefPicList )nRefListColPic )->getRefIdx( uiColBlockIdx[n] );
        if( nRefIdxColPic >= 0 )
        {
          const TComMv & rColMv = pColCU->getCUMvField( ( RefPicList )nRefListColPic )->getMv( uiColBlockIdx[n] );
          mvCand.setRefIdx( rMvStart.getRefIdx() );
          mvCand.getMv() = pCU->scaleMV( rColMv , nCurPOC , nCurRefPOC , nColPOC , pColCU->getSlice()->getRefPOC( ( RefPicList )nRefListColPic , nRefIdxColPic ) );
          xFrucInsertMv2StartList( mvCand , rStartMvList );
        }
      }
    }
  }

#if COM16_C806_VCEG_AZ10_SUB_PU_TMVP
  if( pCU->getSlice()->getSPS()->getAtmvpEnableFlag() )
  {
#if FRUC_FIX
    UInt row = nSubBlkHeight >> 2;
    UInt col = nSubBlkWidth >> 2;
    for( UInt y = 0; y < row; y++ )
    {
      for( UInt x = 0; x < col; x++)
      {
        UInt uiIdx = ( ( uiFidx+x+y*numPartPerLine ) << 1 ) + eRefPicList;
#if JVET_C0035_ATMVP_SIMPLIFICATION
        if( rMvStart.getRefIdx() == m_cMvFieldSP[MGR_TYPE_SUBPU_ATMVP][uiIdx].getRefIdx() )
        {
          xFrucInsertMv2StartList( m_cMvFieldSP[MGR_TYPE_SUBPU_ATMVP][uiIdx] , rStartMvList );
        }
        if( rMvStart.getRefIdx() == m_cMvFieldSP[MGR_TYPE_SUBPU_ATMVP_EXT][uiIdx].getRefIdx() )
        {
          xFrucInsertMv2StartList( m_cMvFieldSP[MGR_TYPE_SUBPU_ATMVP_EXT][uiIdx] , rStartMvList );
        }
#else
        if( rMvStart.getRefIdx() == m_cMvFieldSP[0][uiIdx].getRefIdx() )
        {
          xFrucInsertMv2StartList( m_cMvFieldSP[0][uiIdx] , rStartMvList );
        }
        if( rMvStart.getRefIdx() == m_cMvFieldSP[1][uiIdx].getRefIdx() )
        {
          xFrucInsertMv2StartList( m_cMvFieldSP[1][uiIdx] , rStartMvList );
        }
#endif
      }
    }
#else
#if JVET_E0060_FRUC_CAND
    for( UInt n = 0 ; n < min(NB_FRUC_CAND_ATMVP, uiSubBlkRasterStep) ; n++ )
#else
    for( UInt n = 0 ; n < uiSubBlkRasterStep ; n++ )
#endif
    {
      UInt uiIdx = ( ( n + uiSubBlkRasterIdx ) << 1 ) + eRefPicList;
#if JVET_C0035_ATMVP_SIMPLIFICATION
      if( rMvStart.getRefIdx() == m_cMvFieldSP[MGR_TYPE_SUBPU_ATMVP][uiIdx].getRefIdx() )
      {
        xFrucInsertMv2StartList( m_cMvFieldSP[MGR_TYPE_SUBPU_ATMVP][uiIdx] , rStartMvList );
      }
      if( rMvStart.getRefIdx() == m_cMvFieldSP[MGR_TYPE_SUBPU_ATMVP_EXT][uiIdx].getRefIdx() )
      {
        xFrucInsertMv2StartList( m_cMvFieldSP[MGR_TYPE_SUBPU_ATMVP_EXT][uiIdx] , rStartMvList );
      }
#else
      if( rMvStart.getRefIdx() == m_cMvFieldSP[0][uiIdx].getRefIdx() )
      {
        xFrucInsertMv2StartList( m_cMvFieldSP[0][uiIdx] , rStartMvList );
      }
      if( rMvStart.getRefIdx() == m_cMvFieldSP[1][uiIdx].getRefIdx() )
      {
        xFrucInsertMv2StartList( m_cMvFieldSP[1][uiIdx] , rStartMvList );
      }
#endif
    }
#endif
  }
#endif

#if !JVET_E0060_FRUC_CAND
  // scaled interpolated MV
  if( pCU->getSlice()->getSPS()->getUseFRUCMgrMode() )
  {
    TComCUMvField * pFRUCUniLateralMVField = pCU->getPic()->getCtu( pCU->getCtuRsAddr() )->getFRUCUniLateralMVField( eRefPicList );
    Int nRefIdx = pFRUCUniLateralMVField->getRefIdx( uiAbsPartIdx );
    if( nRefIdx >= 0 )
    {
      TComMvField mvCand;
      const TComMv & rMv = pFRUCUniLateralMVField->getMv( uiAbsPartIdx );
      mvCand.setRefIdx( rMvStart.getRefIdx() );
      mvCand.getMv() = pCU->scaleMV( rMv , nCurPOC , nCurRefPOC , nCurPOC , pCU->getSlice()->getRefPOC( eRefPicList , nRefIdx ) );
      xFrucInsertMv2StartList( mvCand , rStartMvList );
    }
  }
#endif

#if !JVET_E0060_FRUC_CAND
  // top neighbor
  UInt uiAbsPartIdxTop = 0;
  TComDataCU * pTop = pCU->getPUAbove( uiAbsPartIdxTop , uiAbsPartIdx + pCU->getZorderIdxInCtu() );
  if( pTop != NULL && pTop->getCUMvField( eRefPicList )->getRefIdx( uiAbsPartIdxTop ) == rMvStart.getRefIdx() )
  {
    TComMvField mvCand;
    mvCand.setMvField( pTop->getCUMvField( eRefPicList )->getMv( uiAbsPartIdxTop ) , pTop->getCUMvField( eRefPicList )->getRefIdx( uiAbsPartIdxTop ) );
    xFrucInsertMv2StartList( mvCand , rStartMvList );
  }

  // left neighbor
  UInt uiAbsPartIdxLeft = 0;
  TComDataCU * pLeft = pCU->getPULeft( uiAbsPartIdxLeft , uiAbsPartIdx + pCU->getZorderIdxInCtu() );
  if( pLeft != NULL && pLeft->getCUMvField( eRefPicList )->getRefIdx( uiAbsPartIdxLeft ) == rMvStart.getRefIdx() )
  {
    TComMvField mvCand;
    mvCand.setMvField( pLeft->getCUMvField( eRefPicList )->getMv( uiAbsPartIdxLeft ) , pLeft->getCUMvField( eRefPicList )->getRefIdx( uiAbsPartIdxLeft ) );
    xFrucInsertMv2StartList( mvCand , rStartMvList );
  }
#endif
}

/**
 * \brief Find the best Mv for Mv candidate list
 *
 * \param pBestMvField    Pointer to the best Mv (Mv pair)
 * \param rBestRefPicList Best reference list
 * \param pcCU            Pointer to current CU
 * \param uiAbsPartIdx    Address of block within CU
 * \param rMvStart        Searching center
 * \param nBlkWidth       Width of the block
 * \param nBlkHeight      Height of the block
 * \param bTM             Whether is template matching
 * \param bMvCost         Whether count Mv cost
 */
UInt TComPrediction::xFrucFindBestMvFromList( TComMvField * pBestMvField , RefPicList & rBestRefPicList , TComDataCU * pCU , UInt uiAbsPartIdx , const TComMvField & rMvStart , Int nBlkWidth , Int nBlkHeight , Bool bTM , Bool bMvCost )
{
  UInt uiMinCost = MAX_UINT;

  Int nRefPicListStart = 0;
  Int nRefPicListEnd = 1;
#if JVET_E0060_FRUC_CAND
  if( bTM || bMvCost)  // Limit search to bestList in Template mode and for all sub-blocks (Template and Bilateral modes)
#else
  if( bTM )
#endif
  {
    nRefPicListStart = nRefPicListEnd = rBestRefPicList;
  }
  for( Int nRefPicList = nRefPicListStart ; nRefPicList <= nRefPicListEnd ; nRefPicList++ )
  {
    RefPicList eCurRefPicList = ( RefPicList )nRefPicList;
    for( std::list<TComMvField>::iterator pos = m_listMVFieldCand[eCurRefPicList].begin() ; pos != m_listMVFieldCand[eCurRefPicList].end() ; pos++ )
    {
      TComMvField mvPair;

      if( !bTM && eCurRefPicList == REF_PIC_LIST_1 && !pCU->getSlice()->getCheckLDC() )
      {       
        // for normal B picture
        if( !pCU->getMvPair( REF_PIC_LIST_1 , *pos , mvPair ) || xFrucIsInList( mvPair , m_listMVFieldCand[0] ) )
          // no paired MV or the pair has been checked in list0
          continue;
      }

      UInt uiCost = 0;
      if( bMvCost )
      {
        uiCost = xFrucGetMvCost( rMvStart.getMv() , pos->getMv() , MAX_INT , FRUC_MERGE_REFINE_MVWEIGHT );
        if( uiCost > uiMinCost )
          continue;
      }

      if( bTM )
      {
        uiCost = xFrucGetTempMatchCost( pCU , uiAbsPartIdx , nBlkWidth , nBlkHeight , eCurRefPicList , *pos , uiCost );
      }
      else
      {
        uiCost = xFrucGetBilaMatchCost( pCU , uiAbsPartIdx , nBlkWidth , nBlkHeight , eCurRefPicList , *pos , mvPair , uiCost ); 
      }

      if( uiCost < uiMinCost )
      {
        uiMinCost = uiCost;
        pBestMvField[eCurRefPicList] = *pos;
        if( !bTM )
        {
          rBestRefPicList = eCurRefPicList;
          pBestMvField[!eCurRefPicList] = mvPair;
        }
      }
    }
  }

  return( uiMinCost );
}

/**
 * \brief Interface of FRUC. Derive Mv information for a block and its sub-blocks
 *
 * \param pcCU            Pointer to current CU
 * \param uiDepth         CU depth
 * \param uiAbsPartIdx    Address of block within CU
 * \param uiPUIdx         PU index
 * \param nTargetRefIdx   The target reference index for Mv predictor
 * \param eTargetRefPicList The target reference list for Mv predictor
 */
Bool TComPrediction::deriveFRUCMV( TComDataCU * pCU , UInt uiDepth , UInt uiAbsPartIdx , UInt uiPUIdx , Int nTargetRefIdx , RefPicList eTargetRefList
#if JVET_E0060_FRUC_CAND
                                 , AMVPInfo* pInfo
#endif
                                 )
{
  Bool bAvailable = false;

  if( pCU->getMergeFlag( uiAbsPartIdx ) )
  {
    bAvailable = xFrucFindBlkMv( pCU , uiPUIdx );
    if( bAvailable )
      xFrucRefineSubBlkMv( pCU , uiDepth , uiPUIdx , pCU->getFRUCMgrMode( uiAbsPartIdx ) == FRUC_MERGE_TEMPLATE );
  }
  else
  {
    // for AMVP
#if JVET_E0060_FRUC_CAND
    bAvailable = xFrucFindBlkMv4Pred( pCU , uiPUIdx , eTargetRefList , nTargetRefIdx , pInfo );
#else
    bAvailable = xFrucFindBlkMv4Pred( pCU , uiPUIdx , eTargetRefList , nTargetRefIdx );
#endif
  }

  return( bAvailable );
}

/**
 * \brief Check whether top template is available
 *
 * \param pcCU            Pointer to current CU
 * \param uiAbsPartIdx    Address of block within CU
 */
Bool TComPrediction::xFrucIsTopTempAvailable( TComDataCU * pCU , UInt uiAbsPartIdx )
{
  // must be used in sub-CU mode, namely getZorderIdxInCU indicates the CU position
  UInt uiAbsPartIdxTop = 0;
  TComDataCU * pPUTop = pCU->getPUAbove( uiAbsPartIdxTop , uiAbsPartIdx + pCU->getZorderIdxInCtu() );
  return( pPUTop != NULL && ( pPUTop->getCtuRsAddr() < pCU->getCtuRsAddr() || pPUTop->getZorderIdxInCtu() < pCU->getZorderIdxInCtu() ) );
}

/**
 * \brief Check whether left template is available
 *
 * \param pcCU            Pointer to current CU
 * \param uiAbsPartIdx    Address of block within CU
 */
Bool TComPrediction::xFrucIsLeftTempAvailable( TComDataCU * pCU , UInt uiAbsPartIdx )
{
  // must be used in sub-CU mode, namely getZorderIdxInCU indicates the CU position
  UInt uiAbsPartIdxLeft = 0;
  TComDataCU * pPULeft = pCU->getPULeft( uiAbsPartIdxLeft , uiAbsPartIdx + pCU->getZorderIdxInCtu() );
  return( pPULeft != NULL && ( pPULeft->getCtuRsAddr() < pCU->getCtuRsAddr() || pPULeft->getZorderIdxInCtu() < pCU->getZorderIdxInCtu() ) );
}

/**
 * \brief Get sub-block size for further refinement
 *
 * \param pcCU            Pointer to current CU
 * \param uiAbsPartIdx    Address of block within CU
 * \param nBlkWidth       Width of the block
 * \param nBlkHeight      Height of the block
 */
Int TComPrediction::xFrucGetSubBlkSize( TComDataCU * pcCU , UInt uiAbsPartIdx , Int nBlkWidth , Int nBlkHeight )
{
#if JVET_C0024_QTBT
  Int avgLength = 1<<(((g_aucConvertToBit[pcCU->getWidth( uiAbsPartIdx )] + g_aucConvertToBit[pcCU->getHeight( uiAbsPartIdx )] + 1)>>1) + MIN_CU_LOG2);
  Int nRefineBlkSize = max( avgLength >> pcCU->getSlice()->getSPS()->getFRUCSmallBlkRefineDepth() , FRUC_MERGE_REFINE_MINBLKSIZE );
#else
  Int nRefineBlkSize = max( pcCU->getWidth( uiAbsPartIdx ) >> pcCU->getSlice()->getSPS()->getFRUCSmallBlkRefineDepth() , FRUC_MERGE_REFINE_MINBLKSIZE );
#endif
  while( true ) 
  {
    Int nMask = nRefineBlkSize - 1;
    if( nRefineBlkSize > min( nBlkWidth , nBlkHeight ) || ( nBlkWidth & nMask ) || ( nBlkHeight & nMask ) )
      nRefineBlkSize >>= 1;
    else
      break;
  }
  assert( nRefineBlkSize >= FRUC_MERGE_REFINE_MINBLKSIZE );
  return( nRefineBlkSize );
}

/**
 * \brief Get the top and left templates for the current block
 *
 * \param pcCU            Pointer to current CU
 * \param uiAbsPartIdx    Address of block within CU
 * \param nCurBlkWidth    Width of the block
 * \param nCurBlkHeight   Height of the block
 */
Bool TComPrediction::xFrucGetCurBlkTemplate( TComDataCU * pCU , UInt uiAbsPartIdx , Int nCurBlkWidth , Int nCurBlkHeight )
{
  m_bFrucTemplateAvailabe[0] = xFrucIsTopTempAvailable( pCU , uiAbsPartIdx );
  m_bFrucTemplateAvailabe[1] = xFrucIsLeftTempAvailable( pCU , uiAbsPartIdx );
  if( !m_bFrucTemplateAvailabe[0] && !m_bFrucTemplateAvailabe[1] )
    return false;

#if VCEG_AZ07_MV_ADD_PRECISION_BIT_FOR_STORE
  const Int nMVUnit = 2 + VCEG_AZ07_MV_ADD_PRECISION_BIT_FOR_STORE;
#else
  const Int nMVUnit = 2;
#endif
  TComPicYuv * pCurPicYuv = pCU->getPic()->getPicYuvRec();
  if( m_bFrucTemplateAvailabe[0] )
  {
    TComYuv * pTemplateTop = &m_cYuvPredFrucTemplate[0];
    TComMv mvTop( 0 , - ( FRUC_MERGE_TEMPLATE_SIZE << nMVUnit ) );
    xPredInterBlk( COMPONENT_Y , pCU , pCurPicYuv , uiAbsPartIdx , &mvTop , nCurBlkWidth , FRUC_MERGE_TEMPLATE_SIZE , pTemplateTop , false , pCU->getSlice()->getSPS()->getBitDepth( CHANNEL_TYPE_LUMA ) , 
#if VCEG_AZ05_BIO
      false,
#endif
      FRUC_MERGE_TEMPLATE );
  }
  if( m_bFrucTemplateAvailabe[1] )
  {
    TComYuv * pTemplateLeft = &m_cYuvPredFrucTemplate[1];
    TComMv mvLeft( - ( FRUC_MERGE_TEMPLATE_SIZE << nMVUnit ) , 0 );
    xPredInterBlk( COMPONENT_Y , pCU , pCurPicYuv , uiAbsPartIdx , &mvLeft , FRUC_MERGE_TEMPLATE_SIZE , nCurBlkHeight , pTemplateLeft , false , pCU->getSlice()->getSPS()->getBitDepth( CHANNEL_TYPE_LUMA ) , 
#if VCEG_AZ05_BIO
      false,
#endif
      FRUC_MERGE_TEMPLATE );
  }

  return( true );
}

/**
 * \brief calculate the Mv cost
 *
 * \param rMvStart        Searching center
 * \param rMvCur          Current Mv
 * \param nSearchRange    Search range
 * \param nWeighting      Weighting factor
 */
UInt TComPrediction::xFrucGetMvCost( const TComMv & rMvStart , const TComMv & rMvCur , Int nSearchRange , Int nWeighting )
{
  TComMv mvDist = rMvStart - rMvCur;
  UInt uiCost = MAX_UINT;
  if( mvDist.getAbsHor() <= nSearchRange && mvDist.getAbsVer() <= nSearchRange )
  {
    uiCost = ( mvDist.getAbsHor() + mvDist.getAbsVer() ) * nWeighting;
#if VCEG_AZ07_MV_ADD_PRECISION_BIT_FOR_STORE
    uiCost >>= VCEG_AZ07_MV_ADD_PRECISION_BIT_FOR_STORE;
#endif
  }

  return( uiCost );
}

#endif

#if VCEG_AZ05_INTRA_MPI
Void TComPrediction::xMPIredFiltering( Pel* pSrc, Int iSrcStride, Pel*& rpDst, Int iDstStride, Int iWidth, Int iHeight , Int idxMPI)
{
  Pel* pDst = rpDst;
  Int x, y;
  switch(idxMPI)
  {
  case 7:
     // 0* 0*
     // 1* 1*
    {
      // boundary pixels processing
      //            top                left           top-left        current                
      pDst[0] = ((                    pSrc[-1] +                      3*pDst[0] + 2) >> 2);//left-top corner

      for ( x = 1; x < iWidth; x++ )
      {
      //                 top                left           top-left          current  
      pDst[x] = ((                     pDst[x-1] +                     3*pDst[x] + 2) >> 2); //upper-line
      }
      //left column
      for ( y = 1; y < iHeight; y++ )
      {
      //                                   top                left                    top-left                   current  
        pDst[iDstStride*y] = ((                         pSrc[iSrcStride*y-1]  +                               3*pDst[iDstStride*y]+2) >> 2);
      }
 
      //inner samples
      pDst++; pDst+=iDstStride;
      Pel* pDstLeft    = pDst-1;
      for ( y = 0; y < iHeight-1; y++ )
      {
        for ( x = 0; x < iWidth-1; x++ )
        {
        pDst[x] = (Pel)((
         pDstLeft[x]          // left neighbour
        +3*pDst[x] + 2) >> 2); 
        }
        pDst+=iDstStride;
        pDstLeft+=iDstStride;
      }
    }
    break;
  case 6:
    // 0* 1*
    // 0* 3*
    {
      // boundary pixels processing
      //            top                left           top-left        current                
      pDst[0] = ((pSrc[-iSrcStride] +                                    3*pDst[0]  +2) >> 2);//left-top corner

      for ( x = 1; x < iWidth; x++ )
      {
      //                 top                left           top-left          current  
      pDst[x] = ((pSrc[x - iSrcStride]  +                                  3*pDst[x]  +2) >> 2); //upper-line
      }     
      //left column
      for ( y = 1; y < iHeight; y++ )
      {
      //                   top                left                top-left        current  
        pDst[iDstStride*y] = 
          ((pDst[iDstStride*(y-1)]   +                                     3*pDst[iDstStride*y]+2) >> 2);
      }
 
      //inner samples
      pDst++; pDst+=iDstStride;
      Pel* pDstTop     = pDst-iDstStride;
      for ( y = 0; y < iHeight-1; y++ )
      {
        for ( x = 0; x < iWidth-1; x++ )
        {
        pDst[x] = (Pel)((
        +pDstTop[x]          // top neighbour
        +3*pDst[x] +2) >> 2); 
        }
        pDst+=iDstStride;
        pDstTop+=iDstStride;
      }
    }
    break;
  case 5: 
    // 0* 1*
    // 1* 6*
    {
      // boundary pixels processing
      //            top                left           top-left        current                
      pDst[0] = ((pSrc[-iSrcStride] + pSrc[-1] +                      6*pDst[0] + 4) >> 3);//left-top corner

      for ( x = 1; x < iWidth; x++ )
      {
      //                 top                left           top-left          current  
      pDst[x] = ((pSrc[x - iSrcStride]  +pDst[x-1] +                         6*pDst[x] + 4) >> 3); //upper-line
      }
      //left column
      for ( y = 1; y < iHeight; y++ )
      {
      //                                   top                left                    top-left                   current  
        pDst[iDstStride*y] = ((pDst[iDstStride*(y-1)] +  pSrc[iSrcStride*y-1]  +                               6*pDst[iDstStride*y]+ 4) >> 3);
      }
 
      //inner samples
      pDst++; pDst+=iDstStride;
      Pel* pDstLeft    = pDst-1;
      Pel* pDstTop     = pDst-iDstStride;
      for ( y = 0; y < iHeight-1; y++ )
      {
        for ( x = 0; x < iWidth-1; x++ )
        {
        pDst[x] = (Pel)((
         pDstLeft[x]          // left neighbour
        +pDstTop[x]          // top neighbour
        +6*pDst[x] + 4) >> 3); 
        }
        pDst+=iDstStride;
        pDstLeft+=iDstStride;
        pDstTop+=iDstStride;
      }
    }
    break;
  case 3:
    // 0* 0*
    // 1* 1*
    {
      // boundary pixels processing
      //            top                left           top-left        current                
      pDst[0] = ((                    pSrc[-1] +                      pDst[0] + 1) >> 1);//left-top corner

      for ( x = 1; x < iWidth; x++ )
      {
      //                 top                left           top-left          current  
      pDst[x] = ((                     pDst[x-1] +                     pDst[x] + 1) >> 1); //upper-line
      }
      //left column
      for ( y = 1; y < iHeight; y++ )
      {
      //                                   top                left                    top-left                   current  
        pDst[iDstStride*y] = ((                         pSrc[iSrcStride*y-1]  +                               pDst[iDstStride*y]+1) >> 1);
      }
 
      //inner samples
      pDst++; pDst+=iDstStride;
      Pel* pDstLeft    = pDst-1;
      for ( y = 0; y < iHeight-1; y++ )
      {
        for ( x = 0; x < iWidth-1; x++ )
        {
        pDst[x] = (Pel)((
         pDstLeft[x]          // left neighbour
        +pDst[x] + 1) >> 1); 
        }
        pDst+=iDstStride;
        pDstLeft+=iDstStride;
      }
    }
    break;
  case 2:
    // 0* 1*
    // 0* 1*
    {
      // boundary pixels processing
      //            top                left           top-left        current                
      pDst[0] = ((pSrc[-iSrcStride] +                                    pDst[0]  +1) >> 1);//left-top corner

      for ( x = 1; x < iWidth; x++ )
      {
      //                 top                left           top-left          current  
      pDst[x] = ((pSrc[x - iSrcStride]  +                                   pDst[x]  +1) >> 1); //upper-line
      }     
      //left column
      for ( y = 1; y < iHeight; y++ )
      {
      //                   top                left                top-left        current  
        pDst[iDstStride*y] = 
          ((pDst[iDstStride*(y-1)]   +                                      pDst[iDstStride*y]+1) >> 1);
      }
 
      //inner samples
      pDst++; pDst+=iDstStride;
      Pel* pDstTop     = pDst-iDstStride;
      for ( y = 0; y < iHeight-1; y++ )
      {
        for ( x = 0; x < iWidth-1; x++ )
        {
        pDst[x] = (Pel)((
        +pDstTop[x]          // top neighbour
        +pDst[x] +1) >> 1); 
        }
        pDst+=iDstStride;
        pDstTop+=iDstStride;
      }
    }
    break;
  case 1: 
    // 0* 1*
    // 1* 2*
    {
      // boundary pixels processing
      //            top                left           top-left        current                
      pDst[0] = ((pSrc[-iSrcStride] + pSrc[-1] +                      2*pDst[0] + 2) >> 2);//left-top corner

      for ( x = 1; x < iWidth; x++ )
      {
      //                 top                left           top-left          current  
      pDst[x] = ((pSrc[x - iSrcStride]  +pDst[x-1] +                         2*pDst[x] + 2) >> 2); //upper-line
      }
      //left column
      for ( y = 1; y < iHeight; y++ )
      {
      //                                   top                left                    top-left                   current  
        pDst[iDstStride*y] = ((pDst[iDstStride*(y-1)] +  pSrc[iSrcStride*y-1]  +                               2*pDst[iDstStride*y]+ 2) >> 2);
      }
 
      //inner samples
      pDst++; pDst+=iDstStride;
      Pel* pDstLeft    = pDst-1;
      Pel* pDstTop     = pDst-iDstStride;
      for ( y = 0; y < iHeight-1; y++ )
      {
        for ( x = 0; x < iWidth-1; x++ )
        {
        pDst[x] = (Pel)((
         pDstLeft[x]          // left neighbour
        +pDstTop[x]          // top neighbour
        +2*pDst[x] + 2) >> 2); 
        }
        pDst+=iDstStride;
        pDstLeft+=iDstStride;
        pDstTop+=iDstStride;
      }
    }
    break;
  case 0:
  case 4:
    {

    }
    break;
  }
  return;
}
#endif

#if COM16_C806_LMCHROMA

Int   isAboveAvailable      ( TComDataCU* pcCU, UInt uiPartIdxLT, UInt uiPartIdxRT, Bool* bValidFlags ); // ??? to be updated
Int   isLeftAvailable       ( TComDataCU* pcCU, UInt uiPartIdxLT, UInt uiPartIdxLB, Bool* bValidFlags ); 

/** Function for deriving chroma LM intra prediction.
 * \param pcPattern pointer to neighbouring pixel access pattern
 * \param piSrc pointer to reconstructed chroma sample array
 * \param pPred pointer for the prediction sample array
 * \param uiPredStride the stride of the prediction sample array
 * \param uiCWidth the width of the chroma block
 * \param uiCHeight the height of the chroma block
 *
 * This function derives the prediction samples for chroma LM mode (chroma intra coding)
 */
Void TComPrediction::predLMIntraChroma( TComTU& rTu, const ComponentID compID, Pel* pPred, UInt uiPredStride, UInt uiCWidth, UInt uiCHeight 
#if JVET_E0077_ENHANCED_LM
    , Int LMtype
#endif
    )
{
#if JVET_E0077_MMLM
    if (LMtype == MMLM_CHROMA_IDX
#if JVET_E0077_LM_MF
        || (LMtype >= LM_CHROMA_F1_IDX && LMtype < (LM_CHROMA_F1_IDX + LM_FILTER_NUM))
#endif        
        )
    {
#if JVET_E0077_LM_MF
        Pel *pLumaSaved = m_pLumaRecBuffer;
        if (LMtype >= LM_CHROMA_F1_IDX && LMtype < (LM_CHROMA_F1_IDX + LM_FILTER_NUM))
        {
            Int iLumaIdx = LMtype - LM_CHROMA_F1_IDX;
            m_pLumaRecBuffer = m_pLumaRecBufferMul[iLumaIdx];
        }
#endif
        // LLS parameters estimation -->
        TComPrediction::MMLM_parameter parameters[2];
        Int iGroupNum = 2;
        xGetMMLMParameters(rTu, compID, uiCWidth, uiCHeight, iGroupNum, parameters);

        // get prediction -->
        Int  iLumaStride = m_iLumaRecStride;

        Pel  *pLuma = m_pLumaRecBuffer + (iLumaStride + 1) * MMLM_SAMPLE_NEIGHBOR_LINES;
#if !JVET_D0033_ADAPTIVE_CLIPPING
        const TComSPS &sps = *(rTu.getCU()->getSlice()->getSPS());
        Int maxV = (1 << sps.getBitDepth(CHANNEL_TYPE_CHROMA)) - 1;
#endif
        for (Int i = 0; i < uiCHeight; i++)
        {
            for (Int j = 0; j < uiCWidth; j++)
            {
                Int a, b, iShift;
                if (pLuma[j] <= parameters[0].Sup)
                {
                    a = parameters[0].a;
                    b = parameters[0].b;
                    iShift = parameters[0].shift;
                }
                else
                {
                    a = parameters[1].a;
                    b = parameters[1].b;
                    iShift = parameters[1].shift;
                }
#if JVET_D0033_ADAPTIVE_CLIPPING
                pPred[j] = ClipA(((a * pLuma[j]) >> iShift) + b, compID);
#else
                pPred[j] = Clip3(0, maxV, ((a * pLuma[j]) >> iShift) + b);
#endif

            }

            pPred += uiPredStride;
            pLuma += iLumaStride;
        }

#if JVET_E0077_LM_MF
        m_pLumaRecBuffer = pLumaSaved;
#endif
    }
    else 
#endif
#if JVET_E0077_MMLM
    {
#endif
  // LLS parameters estimation -->
  Int a, b, iShift;
  xGetLMParameters( rTu, compID, uiCWidth, uiCHeight, 0, a, b, iShift );

  // get prediction -->
  Int  iLumaStride = m_iLumaRecStride;

#if JVET_E0077_MMLM
  Pel  *pLuma = m_pLumaRecBuffer + (iLumaStride + 1) * MMLM_SAMPLE_NEIGHBOR_LINES;
#else
  Pel  *pLuma = m_pLumaRecBuffer + iLumaStride + 1;
#endif

#if !JVET_D0033_ADAPTIVE_CLIPPING
  const TComSPS &sps = *(rTu.getCU()->getSlice()->getSPS());
  Int maxV = (1 << sps.getBitDepth(CHANNEL_TYPE_CHROMA)) - 1;
#endif

  for( Int i = 0; i < uiCHeight; i++ )
  {
    for( Int j = 0; j < uiCWidth; j++ )
    {
#if JVET_D0033_ADAPTIVE_CLIPPING
        pPred[j] = ClipA(( ( a * pLuma[j] ) >> iShift ) + b ,  compID);
#else
      pPred[j] = Clip3(0, maxV, ( ( a * pLuma[j] ) >> iShift ) + b );
#endif
    }

    pPred += uiPredStride;
    pLuma += iLumaStride;
  }
  // <-- end of get prediction

#if JVET_E0077_MMLM
    }
#endif
}

#if JVET_E0077_LM_MF
Void TComPrediction::xFilterGroup(Pel* pMulDst[], Int i, Pel* piSrc, Int iRecStride, Bool bAboveAvaillable, Bool bLeftAvaillable)
{
    pMulDst[0][i] = (piSrc[1] + piSrc[iRecStride + 1] + 1) >> 1;

    pMulDst[1][i] = (piSrc[iRecStride] + piSrc[iRecStride + 1] + 1) >> 1;

    pMulDst[3][i] = (piSrc[0] + piSrc[1] + 1) >> 1;

    pMulDst[2][i] = (piSrc[0] + piSrc[1] + piSrc[iRecStride] + piSrc[iRecStride + 1] + 2) >> 2;

}
#endif



/** Function for deriving downsampled luma sample of current chroma block and its above, left causal pixel
 * \param pcPattern pointer to neighbouring pixel access pattern
 * \param uiCWidth the width of the chroma block
 * \param uiCHeight the height of the chroma block
 * \param bLeftPicBoundary indication of the chroma block located on the left picture boundary
 *
 * This function derives downsampled luma sample of current chroma block and its above, left causal pixel
 */
Void TComPrediction::getLumaRecPixels( TComTU& rTu, UInt uiCWidth, UInt uiCHeight )
{

#if JVET_E0077_MMLM
    Pel  *pDst0 = m_pLumaRecBuffer + (m_iLumaRecStride + 1) * MMLM_SAMPLE_NEIGHBOR_LINES;
#else
  Pel* pDst0 = m_pLumaRecBuffer + m_iLumaRecStride + 1;
#endif

  Int iDstStride = m_iLumaRecStride;

#if JVET_E0077_LM_MF
  Pel *pMulDst0[LM_FILTER_NUM];
  for (Int i = 0; i < LM_FILTER_NUM; i++)
  {
#if JVET_E0077_MMLM
      pMulDst0[i] = m_pLumaRecBufferMul[i] + (m_iLumaRecStride + 1) * MMLM_SAMPLE_NEIGHBOR_LINES;
#else
      pMulDst0[i] = m_pLumaRecBufferMul[i] + m_iLumaRecStride + 1;
#endif
  }
  Pel* pMulDst[LM_FILTER_NUM];
#endif

  TComDataCU *pcCU=rTu.getCU();
  const UInt uiZorderIdxInPart=rTu.GetAbsPartIdxTU();
  Pel *pRecSrc0 = pcCU->getPic()->getPicYuvRec()->getAddr(COMPONENT_Y, pcCU->getCtuRsAddr(), pcCU->getZorderIdxInCtu()+uiZorderIdxInPart);
  Int iRecStride = pcCU->getPic()->getPicYuvRec()->getStride(COMPONENT_Y);;

  Int iRecStride2 = iRecStride << 1;

  Pel* pDst;  
  Pel* piSrc;

  const TComSPS &sps = *(pcCU->getSlice()->getSPS());
  const UInt uiTuWidth        = rTu.getRect(COMPONENT_Y).width;
  const UInt uiTuHeight       = rTu.getRect(COMPONENT_Y).height;
#if JVET_C0024_QTBT
  const Int  iBaseUnitSize    = sps.getCTUSize() >> sps.getMaxTotalCUDepth();
  assert(iBaseUnitSize == (1<<MIN_CU_LOG2));
#else
  const Int  iBaseUnitSize    = sps.getMaxCUWidth() >> sps.getMaxTotalCUDepth();
#endif
  const Int  iUnitWidth       = iBaseUnitSize;
  const Int  iUnitHeight      = iBaseUnitSize;
  const Int  iTUWidthInUnits  = uiTuWidth  / iUnitWidth;
  const Int  iTUHeightInUnits = uiTuHeight / iUnitHeight;

  const Int  iPartIdxStride   = pcCU->getPic()->getNumPartInCtuWidth();
  const UInt uiPartIdxLT      = pcCU->getZorderIdxInCtu() + uiZorderIdxInPart;
  const UInt uiPartIdxRT      = g_auiRasterToZscan[ g_auiZscanToRaster[ uiPartIdxLT ] +   iTUWidthInUnits  - 1                   ];
  const UInt uiPartIdxLB      = g_auiRasterToZscan[ g_auiZscanToRaster[ uiPartIdxLT ] + ((iTUHeightInUnits - 1) * iPartIdxStride)];
  
  Bool tempbuf[MAX_NUM_PART_IDXS_IN_CTU_WIDTH*4+1];
  Int availlableUnit = isLeftAvailable ( pcCU, uiPartIdxLT, uiPartIdxLB, tempbuf+MAX_NUM_PART_IDXS_IN_CTU_WIDTH);
#if JVET_C0024_QTBT
  Bool bLeftAvaillable = availlableUnit == iTUHeightInUnits ? true : false; 
#else
  Bool bLeftAvaillable = availlableUnit == iTUWidthInUnits ? true : false; 
#endif
  availlableUnit = isAboveAvailable( pcCU, uiPartIdxLT, uiPartIdxRT, tempbuf+MAX_NUM_PART_IDXS_IN_CTU_WIDTH);
#if JVET_C0024_QTBT
  Bool bAboveAvaillable = availlableUnit == iTUWidthInUnits ? true : false; 
#else
  Bool bAboveAvaillable = availlableUnit == iTUHeightInUnits ? true : false; 
#endif

  if (bAboveAvaillable)
  {
    pDst = pDst0 - iDstStride;  
    piSrc = pRecSrc0 - iRecStride2;

    for (Int i = 0; i < uiCWidth; i++)
    {
      if(i == 0 && !bLeftAvaillable)
      {
        pDst[i] = ( piSrc[2*i] + piSrc[2*i + iRecStride] + 1) >> 1;
      }
      else
      {
         pDst[i] = ( ((piSrc[2*i]              * 2 ) + piSrc[2*i - 1]              + piSrc[2*i + 1]             )
        + ((piSrc[2*i + iRecStride] * 2 ) + piSrc[2*i - 1 + iRecStride] + piSrc[2*i + 1 + iRecStride])
        + 4) >> 3;
      }
    }

#if JVET_E0077_MMLM
    for (int line = 2; line <= MMLM_SAMPLE_NEIGHBOR_LINES; line++)
    {
        pDst = pDst0 - iDstStride * line;
        piSrc = pRecSrc0 - iRecStride2 * line;

        for (Int i = 0; i < uiCWidth; i++)
        {
            if (i == 0 && !bLeftAvaillable)
            {
                pDst[i] = (piSrc[2 * i] + piSrc[2 * i + iRecStride] + 1) >> 1;
            }
            else
            {
                pDst[i] = (((piSrc[2 * i] * 2) + piSrc[2 * i - 1] + piSrc[2 * i + 1])
                    + ((piSrc[2 * i + iRecStride] * 2) + piSrc[2 * i - 1 + iRecStride] + piSrc[2 * i + 1 + iRecStride])
                    + 4) >> 3;
            }
        }
    }
#endif

#if JVET_E0077_LM_MF
    for (Int i = 0; i < LM_FILTER_NUM; i++)
    {
        pMulDst[i] = pMulDst0[i] - iDstStride;
    }

    piSrc = pRecSrc0 - iRecStride2;

    for (Int i = 0; i < uiCWidth; i++)
    {

        xFilterGroup(pMulDst, i, &piSrc[2 * i], iRecStride, bAboveAvaillable, i != 0 || bLeftAvaillable);
    }

#if JVET_E0077_MMLM
    for (int line = 2; line <= MMLM_SAMPLE_NEIGHBOR_LINES; line++)
    {
        for (Int i = 0; i < LM_FILTER_NUM; i++)
        {
            pMulDst[i] = pMulDst0[i] - iDstStride * line;
        }

        piSrc = pRecSrc0 - iRecStride2 * line;

        for (Int i = 0; i < uiCWidth; i++)
        {
            xFilterGroup(pMulDst, i, &piSrc[2 * i], iRecStride, bAboveAvaillable, i != 0 || bLeftAvaillable);
        }
    }
#endif

#endif

  }

  if (bLeftAvaillable)
  {
    pDst = pDst0 - 1;  
    piSrc = pRecSrc0 - 3;
    for (Int j = 0; j < uiCHeight; j++)
    {
      pDst[0] = (  (piSrc[1]              *2 + piSrc[0]          + piSrc[2]             ) 
        + (piSrc[1 + iRecStride] *2 + piSrc[iRecStride] + piSrc[2 + iRecStride])
        + 4) >> 3;
      piSrc += iRecStride2; 
      pDst += iDstStride;    
    }

#if JVET_E0077_MMLM
    for (int line = 2; line <= MMLM_SAMPLE_NEIGHBOR_LINES; line++)
    {
        pDst = pDst0 - line;
        piSrc = pRecSrc0 - 2 * line - 1;

        {
            for (Int j = 0; j < uiCHeight; j++)
            {
                pDst[0] = ((piSrc[1] * 3 + piSrc[2])
                    + (piSrc[1 + iRecStride] * 3 + piSrc[2 + iRecStride])
                    + 4) >> 3;
                piSrc += iRecStride2;
                pDst += iDstStride;
            }
        }
    }
#endif

#if JVET_E0077_LM_MF
    for (Int i = 0; i < LM_FILTER_NUM; i++)
    {
        pMulDst[i] = pMulDst0[i] - 1;
    }

    piSrc = pRecSrc0 - 2;
    for (Int j = 0; j < uiCHeight; j++)
    {
        //Filter group 1

        xFilterGroup(pMulDst, 0, piSrc, iRecStride, j != 0 || bAboveAvaillable, bLeftAvaillable);

        piSrc += iRecStride2;

        for (Int i = 0; i < LM_FILTER_NUM; i++)
        {
            pMulDst[i] += iDstStride;
        }
    }

#if JVET_E0077_MMLM
    for (int line = 2; line <= MMLM_SAMPLE_NEIGHBOR_LINES; line++)
    {
        for (Int i = 0; i < LM_FILTER_NUM; i++)
        {
            pMulDst[i] = pMulDst0[i] - line;
        }
        piSrc = pRecSrc0 - 2 * line;

        for (Int j = 0; j < uiCHeight; j++)
        {

            xFilterGroup(pMulDst, 0, piSrc, iRecStride, j != 0 || bAboveAvaillable, bLeftAvaillable);

            piSrc += iRecStride2;

            for (Int i = 0; i < LM_FILTER_NUM; i++)
            {
                pMulDst[i] += iDstStride;
            }
        }
    }
#endif
#endif
  }

  // inner part from reconstructed picture buffer
  for( Int j = 0; j < uiCHeight; j++ )
  {
    for (Int i = 0; i < uiCWidth; i++)
    {
      if(i==0 && !bLeftAvaillable)
      {
        pDst0[i] = ( pRecSrc0[2*i] + pRecSrc0[2*i + iRecStride] + 1) >> 1;
      }
      else
      {
        pDst0[i] = ( pRecSrc0[2*i]              * 2 + pRecSrc0[2*i + 1]                 + pRecSrc0[2*i -1 ] 
        + pRecSrc0[2*i + iRecStride]* 2 + pRecSrc0[2*i + 1 + iRecStride] + pRecSrc0[2*i -1 + iRecStride]
        + 4) >> 3;
      }
    }
    pDst0 += iDstStride;
    pRecSrc0 += iRecStride2;
  }

#if JVET_E0077_LM_MF
  pRecSrc0 = pcCU->getPic()->getPicYuvRec()->getAddr(COMPONENT_Y, pcCU->getCtuRsAddr(), pcCU->getZorderIdxInCtu() + uiZorderIdxInPart);

  for (Int j = 0; j < uiCHeight; j++)
  {
      for (Int i = 0; i < uiCWidth; i++)
      {
          xFilterGroup(pMulDst0, i, &pRecSrc0[2 * i], iRecStride, j != 0 || bAboveAvaillable, i != 0 || bLeftAvaillable);
      }
      for (Int i = 0; i < LM_FILTER_NUM; i++)
      {
          pMulDst0[i] += iDstStride;
      }

      pRecSrc0 += iRecStride2;
  }
#endif
}

/** Function for deriving LM parameter for predciton of Cr from Cb.
 * \param pcPattern pointer to neighbouring pixel access pattern
 * \param uiWidth the width of the chroma block
 * \param uiHeight the height of the chroma block
 * \param a the weight of the linear prediction model
 * \param b the offset of the linear prediction model
 * \param iShift the shifting bits of of the linear prediction model
 *
 * This function derives the parameters of linear prediction model
 */
Void TComPrediction::addCrossColorResi( TComTU& rTu, const ComponentID compID, Pel* piPred, UInt uiPredStride, UInt uiWidth, UInt uiHeight, Pel* piResi, UInt uiResiStride )
{
  Int a, b, iShift;

  xGetLMParameters( rTu, compID, uiWidth, uiHeight, 1, a, b, iShift );

  Int offset = 1 << (iShift - 1);

  if (a >= 0)
  {
    return;
  }

#if !JVET_D0033_ADAPTIVE_CLIPPING
  const TComSPS &sps = *(rTu.getCU()->getSlice()->getSPS());
  Int maxV = (1 << sps.getBitDepth(CHANNEL_TYPE_CHROMA)) - 1;
#endif

  Pel*  pPred   = piPred;
  Pel*  pResi   = piResi;

  for( UInt uiY = 0; uiY < uiHeight; uiY++ )
  {
    for( UInt uiX = 0; uiX < uiWidth; uiX++ )
    {
#if JVET_D0033_ADAPTIVE_CLIPPING
        pPred[ uiX ] = ClipA(pPred[ uiX ] + (( pResi[ uiX ] * a + offset) >> iShift  ) ,  compID);
#else
      pPred[ uiX ] = Clip3(0, maxV, pPred[ uiX ] + (( pResi[ uiX ] * a + offset) >> iShift  ) );
#endif
    }
    pPred += uiPredStride;
    pResi += uiResiStride;
  }
}

/** Function for deriving the positon of first non-zero binary bit of a value
 * \param x input value
 *
 * This function derives the positon of first non-zero binary bit of a value
 */
Int GetFloorLog2( UInt x )
{
  int bits = -1;
  while( x > 0 )
  {
    bits ++;
    x >>= 1;
  }
  return bits;
}

#if JVET_E0077_MMLM

Int TComPrediction::xCalcLMParametersGeneralized(Int x, Int y, Int xx, Int xy, Int count, Int bitDepth, Int &a, Int &b, Int &iShift)
{

    UInt uiInternalBitDepth = bitDepth;
    if (count == 0)
    {
        a = 0;
        b = 1 << (uiInternalBitDepth - 1);
        iShift = 0;
        return -1;
    }
    assert(count <= 512);

    //Int avgX = x / count;
    //Int avgY = y / count;


    Int avgX = (x * g_aiLMDivTableLow[count - 1] + 32768) >> 16;
    Int avgY = (y * g_aiLMDivTableLow[count - 1] + 32768) >> 16;
    avgX = (x * g_aiLMDivTableHigh[count - 1] + avgX) >> 16;
    avgY = (y * g_aiLMDivTableHigh[count - 1] + avgY) >> 16;


    Int RErrX = x - avgX * count;// x % count;
    Int RErrY = y - avgY * count;// y % count;

    Int iB = 7;
    iShift = 13 - iB;

    {
        Int a1 = xy - (avgX*avgY * count) - avgX*RErrY - avgY*RErrX;
        Int a2 = xx - (avgX*avgX * count) - 2 * avgX*RErrX;

        const Int iShiftA1 = uiInternalBitDepth - 2;
        const Int iShiftA2 = 5;
        const Int iAccuracyShift = uiInternalBitDepth + 4;

        Int iScaleShiftA2 = 0;
        Int iScaleShiftA1 = 0;
        Int a1s = a1;
        Int a2s = a2;

        iScaleShiftA1 = a1 == 0 ? 0 : GetFloorLog2(abs(a1)) - iShiftA1;
        iScaleShiftA2 = a2 == 0 ? 0 : GetFloorLog2(abs(a2)) - iShiftA2;

        if (iScaleShiftA1 < 0)
        {
            iScaleShiftA1 = 0;
        }

        if (iScaleShiftA2 < 0)
        {
            iScaleShiftA2 = 0;
        }

        Int iScaleShiftA = iScaleShiftA2 + iAccuracyShift - iShift - iScaleShiftA1;

        a2s = a2 >> iScaleShiftA2;

        a1s = a1 >> iScaleShiftA1;

        if (a2s >= 32)
        {
            UInt a2t = m_uiaLMShift[a2s - 32];
            // a2t = ClipC( a2t );  //???????????? to be updated
            a = a1s * a2t;
        }
        else
        {
            a = 0;
        }

        if (iScaleShiftA < 0)
        {
            a = a << -iScaleShiftA;
        }
        else
        {
            a = a >> iScaleShiftA;
        }
        a = Clip3(-(1 << (15 - iB)), (1 << (15 - iB)) - 1, a);
        a = a << iB;

        Short n = 0;
        if (a != 0)
        {
            n = GetFloorLog2(abs(a) + ((a < 0 ? -1 : 1) - 1) / 2) - 5;
        }

        iShift = (iShift + iB) - n;
        a = a >> n;

        b = avgY - ((a * avgX) >> iShift);

        return 0;
    }
}


/*
Return: Group Num
count: sample count
LumaSamples & ChrmSamples: Samples of luma and chroma components
GroupNum: Number of groups to be classified.
0 if not specified. (Output the self-determined number )
1 if not specified but at least 2
*/

Int TComPrediction::xLMSampleClassifiedTraining(Int count, Int LumaSamples[], Int ChrmSamples[], Int GroupNum,
    Int bitDepth, MMLM_parameter parameters[])
{
    //assert(GroupNum == 2); // Currently only support 2 groups

    //Initialize

    for (Int i = 0; i < GroupNum; i++)
    {
        parameters[i].Inf = 0;
        parameters[i].Sup = (1 << bitDepth) - 1;
        parameters[i].a = 0;
        parameters[i].b = 1 << (bitDepth - 1);
        parameters[i].shift = 0;
    }

    if (count < 4)//
    {
        return -1;
    }

    Int GroupTag[1024];

    Int GroupCount[3] = { 0, 0, 0 };

    Int mean = 0;
    Int meanC = 0;

    Int iMaxLuma = -1;
    Int iMinLuma = 0xffffff;
    for (int i = 0; i < count; i++)
    {
        mean += LumaSamples[i];
        meanC += ChrmSamples[i];
        if (LumaSamples[i] < iMinLuma)
        {
            iMinLuma = LumaSamples[i];
        }
        if (LumaSamples[i] > iMaxLuma)
        {
            iMaxLuma = LumaSamples[i];
        }
    }

    assert(count <= 512);

    Int meand = (mean  * g_aiLMDivTableLow[count - 1] + 32768) >> 16;
    Int meanCd = (meanC * g_aiLMDivTableLow[count - 1] + 32768) >> 16;
    mean = (mean  * g_aiLMDivTableHigh[count - 1] + meand + 32768) >> 16;
    meanC = (meanC * g_aiLMDivTableHigh[count - 1] + meanCd + 32768) >> 16;

    //Division should be replaced later
    //mean = (mean + count / 2) / count;
    //meanC = (meanC + count / 2) / count;




    Int meanDiff = meanC - mean;

    mean = max(1, mean);

    Int iTh[2] = { 0, 0 };

    if (GroupNum == 2)
    {
        iTh[0] = mean;

        parameters[0].Inf = 0;
        parameters[0].Sup = mean - 1;

        parameters[1].Inf = mean;
        parameters[1].Sup = (1 << bitDepth) - 1;

    }
    else if (GroupNum == 3)
    {
        iTh[0] = max(iMinLuma + 1, (iMinLuma + mean + 1) >> 1);
        iTh[1] = min(iMaxLuma - 1, (iMaxLuma + mean + 1) >> 1);

        parameters[0].Inf = 0;
        parameters[0].Sup = iTh[0] - 1;

        parameters[1].Inf = iTh[0];
        parameters[1].Sup = iTh[1] - 1;

        parameters[2].Inf = iTh[1];
        parameters[2].Sup = (1 << bitDepth) - 1;
    }
    else
    {
        assert(0);
    }
    for (Int i = 0; i < count; i++)
    {
        if (LumaSamples[i] < iTh[0])
        {
            GroupTag[i] = 0;
            GroupCount[0]++;
        }
        else if (LumaSamples[i] < iTh[1] || GroupNum == 2)
        {
            GroupTag[i] = 1;
            GroupCount[1]++;
        }
        else
        {
            GroupTag[i] = 2;
            GroupCount[2]++;
        }
    }
    Int iBiggestGroup = 0;
    for (Int i = 1; i < GroupNum; i++)
    {
        if (GroupCount[i] > iBiggestGroup)
        {
            iBiggestGroup = i;
        }
    }

    for (int group = 0; group < GroupNum; group++)
    {
        // If there is only 1 sample in a group, add the nearest value of the two neighboring pixels to the group.
        if (GroupCount[group] < 2)
        {
            for (int i = 0; i < count; i++)
            {
                if (GroupTag[i] == group)
                {
                    for (Int k = 1; (i + k < count) || (i - k >= 0); k++)
                    {
                        if (i + k < count && GroupTag[i + k] == iBiggestGroup)
                        {
                            GroupTag[i + k] = group;
                            GroupCount[group]++;
                            GroupCount[iBiggestGroup]--;
                            break;
                        }
                        if (i - k >= 0 && GroupTag[i - k] == iBiggestGroup)
                        {
                            GroupTag[i - k] = group;
                            GroupCount[group]++;
                            GroupCount[iBiggestGroup]--;
                            break;
                        }
                    }
                    break;
                }
            }
        }
    }


    Int x[3], y[3], xy[3], xx[3];
    for (Int group = 0; group < GroupNum; group++)
    {
        x[group] = y[group] = xy[group] = xx[group] = 0;
    }
    for (Int i = 0; i < count; i++)
    {
        Int group = GroupTag[i];
        x[group] += LumaSamples[i];
        y[group] += ChrmSamples[i];
        xx[group] += LumaSamples[i] * LumaSamples[i];
        xy[group] += LumaSamples[i] * ChrmSamples[i];
    }

    for (Int group = 0; group < GroupNum; group++)
    {
        Int a, b, iShift;
        if (GroupCount[group] > 1)
        {
            xCalcLMParametersGeneralized(x[group], y[group], xx[group], xy[group], GroupCount[group], bitDepth, a, b, iShift);

            parameters[group].a = a;
            parameters[group].b = b;
            parameters[group].shift = iShift;
        }
        else
        {
            parameters[group].a = 0;
            parameters[group].b = meanDiff;
            parameters[group].shift = 0;
        }
    }
    return 0;
}

Int TComPrediction::xGetMMLMParameters(TComTU& rTu, const ComponentID compID, UInt uiWidth, UInt uiHeight, Int &numClass, MMLM_parameter parameters[])
{

    Pel *pSrcColor0, *pCurChroma0;
    Int iSrcStride, iCurStride;

    TComDataCU *pcCU = rTu.getCU();
    const TComSPS &sps = *(pcCU->getSlice()->getSPS());
    const UInt uiZorderIdxInPart = rTu.GetAbsPartIdxTU();
    const UInt uiTuWidth = rTu.getRect(compID).width;
    const UInt uiTuHeight = rTu.getRect(compID).height;
#if JVET_C0024_QTBT
    assert(uiTuWidth == uiWidth && uiTuHeight == uiHeight);
    const Int  iBaseUnitSize = sps.getCTUSize() >> sps.getMaxTotalCUDepth();
#else
    const Int  iBaseUnitSize = sps.getMaxCUWidth() >> sps.getMaxTotalCUDepth();
#endif
    const Int  iUnitWidth = iBaseUnitSize >> pcCU->getPic()->getPicYuvRec()->getComponentScaleX(compID);
    const Int  iUnitHeight = iBaseUnitSize >> pcCU->getPic()->getPicYuvRec()->getComponentScaleY(compID);
    const Int  iTUWidthInUnits = uiTuWidth / iUnitWidth;
    const Int  iTUHeightInUnits = uiTuHeight / iUnitHeight;

    const Int  iPartIdxStride = pcCU->getPic()->getNumPartInCtuWidth();
    const UInt uiPartIdxLT = pcCU->getZorderIdxInCtu() + uiZorderIdxInPart;
    const UInt uiPartIdxRT = g_auiRasterToZscan[g_auiZscanToRaster[uiPartIdxLT] + iTUWidthInUnits - 1];
    const UInt uiPartIdxLB = g_auiRasterToZscan[g_auiZscanToRaster[uiPartIdxLT] + ((iTUHeightInUnits - 1) * iPartIdxStride)];

    Bool tempbuf[MAX_NUM_PART_IDXS_IN_CTU_WIDTH * 4 + 1];
    Int availlableUnit = isLeftAvailable(pcCU, uiPartIdxLT, uiPartIdxLB, tempbuf + MAX_NUM_PART_IDXS_IN_CTU_WIDTH);
#if JVET_C0024_QTBT
    Bool bLeftAvaillable = availlableUnit == iTUHeightInUnits ? true : false;
#else
    Bool bLeftAvaillable = availlableUnit == iTUWidthInUnits ? true : false;
#endif
    availlableUnit = isAboveAvailable(pcCU, uiPartIdxLT, uiPartIdxRT, tempbuf + MAX_NUM_PART_IDXS_IN_CTU_WIDTH);
#if JVET_C0024_QTBT
    Bool bAboveAvaillable = availlableUnit == iTUWidthInUnits ? true : false;
#else
    Bool bAboveAvaillable = availlableUnit == iTUHeightInUnits ? true : false;
#endif

    UInt uiInternalBitDepth = sps.getBitDepth(CHANNEL_TYPE_CHROMA);


    iSrcStride = m_iLumaRecStride;
    pSrcColor0 = m_pLumaRecBuffer + (iSrcStride + 1) * MMLM_SAMPLE_NEIGHBOR_LINES;


    pCurChroma0 = pcCU->getPic()->getPicYuvRec()->getAddr(compID, pcCU->getCtuRsAddr(), pcCU->getZorderIdxInCtu() + uiZorderIdxInPart);
    iCurStride = pcCU->getPic()->getPicYuvRec()->getStride(compID);;
    //pCurChroma0 -= (iCurStride + 1);


    Int count = 0;
    Int LumaSamples[512];
    Int ChrmSamples[512];


    Int i, j;

    Pel *pSrc = pSrcColor0 - iSrcStride;
    Pel *pCur = pCurChroma0 - iCurStride;

    Bool bAdditionalLine = true;

    if (bAboveAvaillable)
    {
        for (j = 0; j < uiWidth; j++)
        {
            LumaSamples[count] = pSrc[j];
            ChrmSamples[count] = pCur[j];
            count++;
        }
        if (bAdditionalLine)
        {
            for (int line = 2; line <= MMLM_SAMPLE_NEIGHBOR_LINES; line++)
            {
                pSrc = pSrcColor0 - line * iSrcStride;
                pCur = pCurChroma0 - line * iCurStride;

                for (j = 0; j < uiWidth; j++)
                {
                    LumaSamples[count] = pSrc[j];
                    ChrmSamples[count] = pCur[j];
                    count++;
                }
            }
        }
    }

    if (bLeftAvaillable)
    {
        pSrc = pSrcColor0 - 1;
        pCur = pCurChroma0 - 1;

        for (i = 0; i < uiHeight; i++)
        {
            LumaSamples[count] = pSrc[0];
            ChrmSamples[count] = pCur[0];
            count++;

            pSrc += iSrcStride;
            pCur += iCurStride;
        }

        if (bAdditionalLine)
        {
            for (int line = 2; line <= MMLM_SAMPLE_NEIGHBOR_LINES; line++)
            {
                pSrc = pSrcColor0 - line;
                pCur = pCurChroma0 - line;

                for (i = 0; i < uiHeight; i++)
                {
                    LumaSamples[count] = pSrc[0];
                    ChrmSamples[count] = pCur[0];
                    count++;

                    pSrc += iSrcStride;
                    pCur += iCurStride;
                }
            }
        }
    }

    xLMSampleClassifiedTraining(count, LumaSamples, ChrmSamples, numClass, uiInternalBitDepth, parameters);
    return 2;
}
#endif

/** Function for deriving the parameters of linear prediction model.
 * \param x, y, xx, yy sum of reference samples of source component, target component, square of source component and multiplication of source component and target component
 * \param iCountShift, count of reference samples
 * \param iPredType indication of the cross-componennt preidciton type, 0: chroma from luma, 1: Cr from Cb
 * \param a the weight of the linear prediction model
 * \param b the offset of the linear prediction model
 * \param iShift the shifting bits of of the linear prediction model
 *
 * This function derives the parameters of linear prediction model
 */

Void TComPrediction::xCalcLMParameters( Int x, Int y, Int xx, Int xy, Int iCountShift, Int iPredType, Int bitDepth, Int &a, Int &b, Int &iShift )
{
  Int avgX =  x  >> iCountShift;
  Int avgY =  y  >> iCountShift;

  Int RErrX = x & ( ( 1 << iCountShift ) - 1 );
  Int RErrY =  y & ( ( 1 << iCountShift ) - 1 );

  Int iB = 7;
  iShift = 13 - iB;

  UInt uiInternalBitDepth = bitDepth; // need consider different bit depth later ????????????

  if( iCountShift == 0 )
  {
    a = 0;
    b = 1 << (uiInternalBitDepth - 1);
    iShift = 0;
  }
  else
  {
    Int a1 = xy - ( avgX*avgY << iCountShift ) - avgX*RErrY - avgY*RErrX;
    Int a2 = xx - ( avgX*avgX << iCountShift ) - 2*avgX*RErrX ;

    if ( iPredType == 1) // Cr residual predicted from Cb residual, Cr from Cb
    {
      a1 += -1*( xx >> (CR_FROM_CB_REG_COST_SHIFT + 1 ));
      a2 += xx >> CR_FROM_CB_REG_COST_SHIFT;
    }

    const Int iShiftA1 = uiInternalBitDepth - 2;
    const Int iShiftA2 = 5;
    const Int iAccuracyShift = uiInternalBitDepth + 4;

    Int iScaleShiftA2 = 0;
    Int iScaleShiftA1 = 0;
    Int a1s = a1;
    Int a2s = a2;

    iScaleShiftA1 = a1 == 0 ? 0 : GetFloorLog2( abs( a1 ) ) - iShiftA1;
    iScaleShiftA2 = a2 == 0 ? 0 : GetFloorLog2( abs( a2 ) ) - iShiftA2;

    if( iScaleShiftA1 < 0 )
    {
      iScaleShiftA1 = 0;
    }
    
    if( iScaleShiftA2 < 0 )
    {
      iScaleShiftA2 = 0;
    }

    Int iScaleShiftA = iScaleShiftA2 + iAccuracyShift - iShift - iScaleShiftA1;

    a2s = a2 >> iScaleShiftA2;

    a1s = a1 >> iScaleShiftA1;

    if (a2s >= 32)
    {
      UInt a2t = m_uiaLMShift[ a2s - 32 ] ;
     // a2t = ClipC( a2t );  //???????????? to be updated
      a = a1s * a2t;
    }
    else
    {
      a = 0;
    }
    
    if( iScaleShiftA < 0 )
    {
      a = a << -iScaleShiftA;
    }
    else
    {
      a = a >> iScaleShiftA;
    }
    a = Clip3(-( 1 << (15-iB) ), ( 1 << (15-iB )) - 1, a);
    a = a << iB;
   
    Short n = 0;
    if (a != 0)
    {
      n = GetFloorLog2(abs( a ) + ( (a < 0 ? -1 : 1) - 1)/2 ) - 5;
    }
    
    iShift =(iShift+iB)-n;
    a = a>>n;

    b =  avgY - ( (  a * avgX ) >> iShift );
  }   

}
/** Function for deriving LM parameter for predciton of Cr from Cb.
 * \param pcPattern pointer to neighbouring pixel access pattern
 * \param uiWidth the width of the chroma block
 * \param uiHeight the height of the chroma block
 * \param a the weight of the linear prediction model
 * \param b the offset of the linear prediction model
 * \param iShift the shifting bits of of the linear prediction model
 *
 * This function derives the parameters of linear prediction model
 */
Void TComPrediction::xGetLMParameters( TComTU& rTu, const ComponentID compID, UInt uiWidth, UInt uiHeight, Int iPredType, Int &a, Int &b, Int &iShift )
{
  Pel *pSrcColor0, *pCurChroma0; 
  Int iSrcStride, iCurStride;

  TComDataCU *pcCU=rTu.getCU();
  const TComSPS &sps = *(pcCU->getSlice()->getSPS());
  const UInt uiZorderIdxInPart=rTu.GetAbsPartIdxTU();
  const UInt uiTuWidth        = rTu.getRect(compID).width;
  const UInt uiTuHeight       = rTu.getRect(compID).height;
#if JVET_C0024_QTBT
  assert(uiTuWidth==uiWidth && uiTuHeight==uiHeight);
  const Int  iBaseUnitSize    = sps.getCTUSize() >> sps.getMaxTotalCUDepth();
#else
  const Int  iBaseUnitSize    = sps.getMaxCUWidth() >> sps.getMaxTotalCUDepth();
#endif
  const Int  iUnitWidth       = iBaseUnitSize  >> pcCU->getPic()->getPicYuvRec()->getComponentScaleX(compID);
  const Int  iUnitHeight      = iBaseUnitSize  >> pcCU->getPic()->getPicYuvRec()->getComponentScaleY(compID);
  const Int  iTUWidthInUnits  = uiTuWidth  / iUnitWidth;
  const Int  iTUHeightInUnits = uiTuHeight / iUnitHeight;

  const Int  iPartIdxStride   = pcCU->getPic()->getNumPartInCtuWidth();
  const UInt uiPartIdxLT      = pcCU->getZorderIdxInCtu() + uiZorderIdxInPart;
  const UInt uiPartIdxRT      = g_auiRasterToZscan[ g_auiZscanToRaster[ uiPartIdxLT ] +   iTUWidthInUnits  - 1                   ];
  const UInt uiPartIdxLB      = g_auiRasterToZscan[ g_auiZscanToRaster[ uiPartIdxLT ] + ((iTUHeightInUnits - 1) * iPartIdxStride)];

  Bool tempbuf[MAX_NUM_PART_IDXS_IN_CTU_WIDTH*4+1];
  Int availlableUnit = isLeftAvailable ( pcCU, uiPartIdxLT, uiPartIdxLB, tempbuf+MAX_NUM_PART_IDXS_IN_CTU_WIDTH);
#if JVET_C0024_QTBT
  Bool bLeftAvaillable = availlableUnit == iTUHeightInUnits ? true : false; 
#else
  Bool bLeftAvaillable = availlableUnit == iTUWidthInUnits ? true : false; 
#endif
  availlableUnit = isAboveAvailable( pcCU, uiPartIdxLT, uiPartIdxRT, tempbuf+MAX_NUM_PART_IDXS_IN_CTU_WIDTH);
#if JVET_C0024_QTBT
  Bool bAboveAvaillable = availlableUnit == iTUWidthInUnits ? true : false; 
#else
  Bool bAboveAvaillable = availlableUnit == iTUHeightInUnits ? true : false; 
#endif

  if (iPredType == 0) //chroma from luma
  {
    iSrcStride = m_iLumaRecStride;

#if JVET_E0077_MMLM
    pSrcColor0 = m_pLumaRecBuffer + (iSrcStride + 1) * MMLM_SAMPLE_NEIGHBOR_LINES;
#else
    pSrcColor0 = m_pLumaRecBuffer + iSrcStride + 1;
#endif

    pCurChroma0  = m_piYuvExt[compID][PRED_BUF_UNFILTERED];

#if JVET_C0024_QTBT
    iCurStride   = uiWidth + uiHeight + 1;
#else
    iCurStride   = 2 * uiWidth+ 1;
#endif
    pCurChroma0 += iCurStride + 1;
  }
  else
  {
    assert (compID == COMPONENT_Cr);

//    pSrcColor0   = pcPattern->getAdiCbBuf( uiWidth, uiHeight, getPredicBuf() );
    pSrcColor0   = m_piYuvExt[COMPONENT_Cb][PRED_BUF_UNFILTERED];
    pCurChroma0  = m_piYuvExt[COMPONENT_Cr][PRED_BUF_UNFILTERED];
//    pCurChroma0  = pcPattern->getAdiCrBuf( uiWidth, uiHeight, getPredicBuf() ); 

#if JVET_C0024_QTBT
    iSrcStride = uiWidth + uiHeight + 1;
    iCurStride = uiWidth + uiHeight + 1;
#else
    iSrcStride = 2 * uiWidth+ 1;
    iCurStride = 2 * uiWidth+ 1;
#endif
    pSrcColor0  += iSrcStride + 1;
    pCurChroma0 += iCurStride + 1;
  }

  Int x = 0, y = 0, xx = 0, xy = 0;
  Int i, j;
  Int iCountShift = 0;
  UInt uiInternalBitDepth = sps.getBitDepth(CHANNEL_TYPE_CHROMA); 

  Pel *pSrc = pSrcColor0 - iSrcStride;
  Pel *pCur = pCurChroma0 - iCurStride;

#if  JVET_C0024_QTBT
  Int xStep = 1;
  Int yStep = 1;

  if (bLeftAvaillable && bAboveAvaillable)
  {
//    assert ( uiWidth ==  uiHeight);
    if (uiWidth > uiHeight)
    {
      xStep = uiWidth / uiHeight;
    }
    else
    {
      yStep = uiHeight / uiWidth;
    }
  }
#endif
  if (bAboveAvaillable)
  {
#if  JVET_C0024_QTBT
    for( j = 0; j < uiWidth; j+=xStep )
#else 
    for( j = 0; j < uiWidth; j++ )
#endif
    {
      x += pSrc[j];
      y += pCur[j];
      xx += pSrc[j] * pSrc[j];
      xy += pSrc[j] * pCur[j];
    }
  }

  if (bLeftAvaillable)
  {
    pSrc  = pSrcColor0 - 1;
    pCur = pCurChroma0 - 1;

#if  JVET_C0024_QTBT
    for( i = 0; i < uiHeight; i+=yStep )
#else 
    for( i = 0; i < uiHeight; i++ )
#endif
    {
      x += pSrc[0];
      y += pCur[0];
      xx += pSrc[0] * pSrc[0];
      xy += pSrc[0] * pCur[0];

#if  JVET_C0024_QTBT
      pSrc += yStep*iSrcStride;
      pCur += yStep*iCurStride;
#else
      pSrc += iSrcStride;
      pCur += iCurStride;
#endif
    }
  }
  
  if (bLeftAvaillable && bAboveAvaillable)
  {
#if JVET_C0024_QTBT
    iCountShift = g_aucConvertToBit[ min(uiWidth, uiHeight) ] + MIN_CU_LOG2 + 1;
#else
    iCountShift = g_aucConvertToBit[ uiWidth ] + 3;
#endif
  }
  else if (bLeftAvaillable || bAboveAvaillable)
  {
#if JVET_C0024_QTBT
    iCountShift = g_aucConvertToBit[ bLeftAvaillable? uiHeight: uiWidth ] + MIN_CU_LOG2;
#else
    iCountShift = g_aucConvertToBit[ uiWidth ] + 2;
#endif
  }
  else
  {
     a = 0;
     if (iPredType == 0)
     {
        b = 1 << (uiInternalBitDepth - 1);
     }
     else
     {  
       b = 0;
     }
     iShift = 0;
     return;
  }

  Int iTempShift = uiInternalBitDepth + iCountShift - 15;

  if( iTempShift > 0 )
  {
    x  = ( x +  ( 1 << ( iTempShift - 1 ) ) ) >> iTempShift;
    y  = ( y +  ( 1 << ( iTempShift - 1 ) ) ) >> iTempShift;
    xx = ( xx + ( 1 << ( iTempShift - 1 ) ) ) >> iTempShift;
    xy = ( xy + ( 1 << ( iTempShift - 1 ) ) ) >> iTempShift;
    iCountShift -= iTempShift;
  }

  // LLS parameters estimation -->
  xCalcLMParameters( x, y, xx, xy, iCountShift, iPredType, uiInternalBitDepth, a, b, iShift );
}

#endif

#if COM16_C1016_AFFINE
/** Function for checking identical motion.
 * \param TComDataCU* pcCU
 * \param UInt PartAddr
 */
Bool TComPrediction::xCheckIdenticalAffineMotion ( TComDataCU* pcCU, UInt PartAddr, Int iWidth, Int iHeight )
{
  if( pcCU->getSlice()->isInterB() && !pcCU->getSlice()->getPPS()->getWPBiPred() )
  {
    if( pcCU->getCUMvField(REF_PIC_LIST_0)->getRefIdx(PartAddr) >= 0 && pcCU->getCUMvField(REF_PIC_LIST_1)->getRefIdx(PartAddr) >= 0)
    {
      Int RefPOCL0 = pcCU->getSlice()->getRefPic(REF_PIC_LIST_0, pcCU->getCUMvField(REF_PIC_LIST_0)->getRefIdx(PartAddr))->getPOC();
      Int RefPOCL1 = pcCU->getSlice()->getRefPic(REF_PIC_LIST_1, pcCU->getCUMvField(REF_PIC_LIST_1)->getRefIdx(PartAddr))->getPOC();
      if ( RefPOCL0 == RefPOCL1 )
      {
        // Get Part Index in CU and get Mv
        UInt uiPartIdxLT, uiPartIdxRT, uiPartIdxLB, uiAbsIndexInLCU;
        uiAbsIndexInLCU = pcCU->getZorderIdxInCtu();
        uiPartIdxLT = PartAddr + uiAbsIndexInLCU;
        uiPartIdxRT = g_auiRasterToZscan [ g_auiZscanToRaster[ uiPartIdxLT ] + iWidth / pcCU->getPic()->getMinCUWidth() - 1 ] - uiAbsIndexInLCU;
        uiPartIdxLB = g_auiRasterToZscan [ g_auiZscanToRaster[ uiPartIdxLT ] + ( (iHeight / pcCU->getPic()->getMinCUHeight()) - 1 ) * pcCU->getPic()->getNumPartInCtuWidth() ] - uiAbsIndexInLCU;
        uiPartIdxLT = PartAddr;

        if ( pcCU->getCUMvField(REF_PIC_LIST_0)->getMv(uiPartIdxLT) == pcCU->getCUMvField(REF_PIC_LIST_1)->getMv(uiPartIdxLT)
          && pcCU->getCUMvField(REF_PIC_LIST_0)->getMv(uiPartIdxRT) == pcCU->getCUMvField(REF_PIC_LIST_1)->getMv(uiPartIdxRT)
          && pcCU->getCUMvField(REF_PIC_LIST_0)->getMv(uiPartIdxLB) == pcCU->getCUMvField(REF_PIC_LIST_1)->getMv(uiPartIdxLB) )
        {
          return true;
        }
      }
    }
  }
  return false;
}


/**
 * \brief Generate motion-compensated block
 *
 * \param compID     Colour component ID
 * \param cu         Pointer to current CU
 * \param refPic     Pointer to reference picture
 * \param partAddr   Address of block within CU
 * \param mv         Motion vector
 * \param width      Width of block
 * \param height     Height of block
 * \param dstPic     Pointer to destination picture
 * \param bi         Flag indicating whether bipred is used
 * \param  bitDepth  Bit depth
 */
Void TComPrediction::xPredAffineBlk(const ComponentID compID, TComDataCU *cu, TComPicYuv *refPic, UInt partAddr, TComMv acMv[3], Int width, Int height, TComYuv *dstPic, Bool bi, const Int bitDepth )
{
#if COM16_C1016_AFFINE
  if ( acMv[0] == acMv[1] )
  {
    xPredInterBlk( compID, cu, refPic, partAddr, &acMv[0], width, height, dstPic, bi, bitDepth );
    return;
  }
#endif

  Int     refStride  = refPic->getStride(compID);
  Int     dstStride  = dstPic->getStride(compID);

  Pel*    refOrg     = refPic->getAddr(compID, cu->getCtuRsAddr(), cu->getZorderIdxInCtu() + partAddr );
  Pel*    dst = dstPic->getAddr( compID, partAddr );

  Int iScaleX = refPic->getComponentScaleX(compID);
  Int iScaleY = refPic->getComponentScaleY(compID);

  // get affine sub-block width and height
  Int blockWidth  = width;
  Int blockHeight = height;
  Int mvWx = max( abs((acMv[1] - acMv[0]).getHor()), abs((acMv[1] - acMv[0]).getVer()) );
  Int mvWy = max( abs((acMv[2] - acMv[0]).getHor()), abs((acMv[2] - acMv[0]).getVer()) );

  Int iMvPrecision = 4;
#if VCEG_AZ07_MV_ADD_PRECISION_BIT_FOR_STORE
  iMvPrecision -= VCEG_AZ07_MV_ADD_PRECISION_BIT_FOR_STORE;
#endif

  if (mvWx)
  {
    blockWidth = max( (Int)( ( width >> iMvPrecision ) / mvWx ), 1 );
    while (width % blockWidth)
    {
      blockWidth--;
    }
    blockWidth = max( AFFINE_MIN_BLOCK_SIZE, blockWidth );
  }
  if (mvWy)
  {
    blockHeight = max( (Int)( ( height >> iMvPrecision ) / mvWy ), 1 );
    while (height % blockHeight)
    {
      blockHeight--;
    }
    blockHeight = max( AFFINE_MIN_BLOCK_SIZE, blockHeight );
  }

  blockWidth  >>= iScaleX;
  blockHeight >>= iScaleY;
  Int cxWidth  = width  >> iScaleX;
  Int cxHeight = height >> iScaleY;
  Int iHalfBW  = blockWidth  >> 1;
  Int iHalfBH  = blockHeight >> 1;

  // convert to 2^(storeBit + iBit) precision
  Int iBit = 8;
  Int iDMvHorX = ( (acMv[1] - acMv[0]).getHor() << iBit ) / cxWidth;  // deltaMvHor
  Int iDMvHorY = ( (acMv[1] - acMv[0]).getVer() << iBit ) / cxWidth;
  Int iDMvVerX = -iDMvHorY;                                           // deltaMvVer
  Int iDMvVerY =  iDMvHorX;

  Int iMvScaleHor = acMv[0].getHor() << iBit;
  Int iMvScaleVer = acMv[0].getVer() << iBit;
  Int iMvYHor = iMvScaleHor;
  Int iMvYVer = iMvScaleVer;
  Int iMvScaleTmpHor, iMvScaleTmpVer;

  // get clip MV Range
  const TComSPS &sps=*(cu->getSlice()->getSPS());
#if JVET_C0025_AFFINE_FILTER_SIMPLIFICATION
  Int iMvShift = 4;
#else
  Int iMvShift = 6;
#endif
  Int iOffset  = 8;

  Int iHorMax = ( sps.getPicWidthInLumaSamples()  + iOffset - cu->getCUPelX() - 1 ) << iMvShift;
#if JVET_C0024_QTBT
  Int iHorMin = (      -(Int)sps.getCTUSize()  - iOffset - (Int)cu->getCUPelX() + 1 ) << iMvShift;
#else
  Int iHorMin = (      -(Int)sps.getMaxCUWidth()  - iOffset - (Int)cu->getCUPelX() + 1 ) << iMvShift;
#endif
  Int iVerMax = ( sps.getPicHeightInLumaSamples() + iOffset - cu->getCUPelY() - 1 ) << iMvShift;
#if JVET_C0024_QTBT
  Int iVerMin = (      -(Int)sps.getCTUSize() - iOffset - (Int)cu->getCUPelY() + 1 ) << iMvShift;
#else
  Int iVerMin = (      -(Int)sps.getMaxCUHeight() - iOffset - (Int)cu->getCUPelY() + 1 ) << iMvShift;
#endif

  const ChromaFormat chFmt = cu->getPic()->getChromaFormat();
  Int   tmpStride = m_filteredBlockTmp[0].getStride(compID);
  Pel*  tmp       = m_filteredBlockTmp[0].getAddr(compID);
  const Int vFilterSize = isLuma(compID) ? NTAPS_LUMA : NTAPS_CHROMA;

  Int shift = iBit - 4;
#if VCEG_AZ07_MV_ADD_PRECISION_BIT_FOR_STORE
  shift += VCEG_AZ07_MV_ADD_PRECISION_BIT_FOR_STORE;
#endif
#if JVET_C0025_AFFINE_FILTER_SIMPLIFICATION
  shift += 2;
#endif

  // get prediction block by block
  for ( Int h = 0; h < cxHeight; h += blockHeight )
  {
    for ( Int w = 0; w < cxWidth; w += blockWidth )
    {
      iMvScaleTmpHor = ( iMvScaleHor + iDMvHorX * iHalfBW + iDMvVerX * iHalfBH ) >> shift;
      iMvScaleTmpVer = ( iMvScaleVer + iDMvHorY * iHalfBW + iDMvVerY * iHalfBH ) >> shift;

      // clip and scale
#if JVET_C0025_AFFINE_FILTER_SIMPLIFICATION
      iMvScaleTmpHor = min( iHorMax, max( iHorMin, iMvScaleTmpHor ) );
      iMvScaleTmpVer = min( iVerMax, max( iVerMin, iMvScaleTmpVer ) );
#else
      iMvScaleTmpHor = min( iHorMax, max( iHorMin, iMvScaleTmpHor ) ) >> iScaleX;
      iMvScaleTmpVer = min( iVerMax, max( iVerMin, iMvScaleTmpVer ) ) >> iScaleY;
#endif

      // get the MV in high precision
      Int xFrac, yFrac, xInt, yInt;
#if JVET_C0025_AFFINE_FILTER_SIMPLIFICATION
      if (!iScaleX)
      {
        xInt  = iMvScaleTmpHor >> 4;        
        xFrac = iMvScaleTmpHor & 15;
      }
      else
      {
        xInt  = iMvScaleTmpHor >> 5;        
        xFrac = iMvScaleTmpHor & 31;
      }
      if (!iScaleY)
      {
        yInt  = iMvScaleTmpVer >> 4;
        yFrac = iMvScaleTmpVer & 15;
      }
      else
      {
        yInt  = iMvScaleTmpVer >> 5;
        yFrac = iMvScaleTmpVer & 31;
      }
#else      
      xInt  = iMvScaleTmpHor >> 6;
      yInt  = iMvScaleTmpVer >> 6;
      xFrac = iMvScaleTmpHor & 63;
      yFrac = iMvScaleTmpVer & 63;
#endif

      Int refOffset = xInt + w + yInt * refStride;
      Pel *ref = refOrg + refOffset;

#if JVET_C0025_AFFINE_FILTER_SIMPLIFICATION
      if ( yFrac == 0 )
      {
        m_if.filterHor(compID, ref, refStride, dst+w,  dstStride, blockWidth, blockHeight, xFrac, !bi, chFmt, bitDepth);
      }
      else if ( xFrac == 0 )
      {
        m_if.filterVer(compID, ref, refStride, dst+w, dstStride, blockWidth, blockHeight, yFrac, true, !bi, chFmt, bitDepth);
      }
      else
      {
        m_if.filterHor(compID, ref - ((vFilterSize>>1) -1)*refStride, refStride, tmp, tmpStride, blockWidth, blockHeight+vFilterSize-1, xFrac, false,      chFmt, bitDepth);
        m_if.filterVer(compID, tmp + ((vFilterSize>>1) -1)*tmpStride, tmpStride, dst+w, dstStride, blockWidth, blockHeight,               yFrac, false, !bi, chFmt, bitDepth);
      }
#else
      if ( yFrac == 0 )
      {
        m_if.filterHorAffine(compID, ref, refStride, dst+w,  dstStride, blockWidth, blockHeight, xFrac, !bi, chFmt, bitDepth);
      }
      else if ( xFrac == 0 )
      {
        m_if.filterVerAffine(compID, ref, refStride, dst+w, dstStride, blockWidth, blockHeight, yFrac, true, !bi, chFmt, bitDepth);
      }
      else
      {
        m_if.filterHorAffine(compID, ref - ((vFilterSize>>1) -1)*refStride, refStride, tmp, tmpStride, blockWidth, blockHeight+vFilterSize-1, xFrac, false,      chFmt, bitDepth);
        m_if.filterVerAffine(compID, tmp + ((vFilterSize>>1) -1)*tmpStride, tmpStride, dst+w, dstStride, blockWidth, blockHeight,               yFrac, false, !bi, chFmt, bitDepth);
      }
#endif

      // switch from x to x+AffineBlockSize, add deltaMvHor
      iMvScaleHor += (iDMvHorX*blockWidth);
      iMvScaleVer += (iDMvHorY*blockWidth);
    }

    dst     += dstStride*blockHeight;
    refOrg  += refStride*blockHeight;

    // switch from y to y+AffineBlockSize add deltaMvVer
    iMvYHor += (iDMvVerX*blockHeight);
    iMvYVer += (iDMvVerY*blockHeight);

    iMvScaleHor = iMvYHor;
    iMvScaleVer = iMvYVer;
  }
}

Void TComPrediction::getMvPredAffineAMVP( TComDataCU* pcCU, UInt uiPartIdx, UInt uiPartAddr, RefPicList eRefPicList, TComMv acMvPred[3] )
{
  AffineAMVPInfo* pcInfo = pcCU->getCUMvField(eRefPicList)->getAffineAMVPInfo();

  if( pcInfo->iN <= 1 )
  {
    memcpy( acMvPred, pcInfo->m_acMvCand[0], sizeof(TComMv)*3 );
    pcCU->setMVPIdxSubParts( 0, eRefPicList, uiPartAddr, uiPartIdx, pcCU->getDepth(uiPartAddr) );
    pcCU->setMVPNumSubParts( pcInfo->iN, eRefPicList, uiPartAddr, uiPartIdx, pcCU->getDepth(uiPartAddr) );
    return;
  }

  assert( pcCU->getMVPIdx(eRefPicList,uiPartAddr) >= 0 );
  memcpy( acMvPred, pcInfo->m_acMvCand[ pcCU->getMVPIdx(eRefPicList, uiPartAddr) ], sizeof(TComMv)*3 );
  return;
}
#endif


#if COM16_C1046_PDPC_INTRA
#if JVET_C0024_QTBT
void TComPrediction::xReferenceFilter(Int iDoubleSize, Int iOrigWeight, Int iFilterOrder, Int * piRefrVector, Int * piLowPassRef)
#else
void TComPrediction::xReferenceFilter(Int iBlkSize, Int iOrigWeight, Int iFilterOrder, Int * piRefrVector, Int * piLowPassRef)
#endif
{
  const Int imCoeff[3][4] = 
  {
    { 20, 15, 6, 1 },
    { 16, 14, 7, 3 },
    { 14, 12, 9, 4 } 
  };

  const Int * piFc;
#if !JVET_C0024_QTBT
  const Int iDoubleSize = 2 * iBlkSize;                   // symmetric representation
#endif
  Int * piTmp = &piBinBuff[2 * MAX_CU_SIZE + 4];   // to  use negative indexes
  Int * piDat = piRefrVector;
  Int * piRes = piLowPassRef;
  
  for (Int k = -iDoubleSize; k <= iDoubleSize; k++)
    piTmp[k] = piDat[k];

  for (Int n = 1; n <= 3; n++)
  {
    piTmp[-iDoubleSize - n] = piTmp[-iDoubleSize - 1 + n];
    piTmp[iDoubleSize + n] = piTmp[iDoubleSize + 1 - n];
  }

  switch (iFilterOrder) 
  {
  case 0:
    break;
  case 1:
    for (Int k = -iDoubleSize; k <= iDoubleSize; k++)
      piRes[k] = ((piTmp[k] << 1) + piTmp[k - 1] + piTmp[k + 1] + 2) >> 2;
    break;
  case 2:
    for (Int k = -iDoubleSize; k <= iDoubleSize; k++)
      piRes[k] = ((piTmp[k] << 1) + ((piTmp[k] + piTmp[k - 1] + piTmp[k + 1]) << 2) + piTmp[k - 2] + piTmp[k + 2] + 8) >> 4;
    break;
  case 3:
  case 5:
  case 7:
    piFc = imCoeff[(iFilterOrder - 3) >> 1];
    for (Int k = -iDoubleSize; k <= iDoubleSize; k++)
    {
      Int s = 32 + piFc[0] * piTmp[k];
      for (Int n = 1; n < 4; n++)
        s += piFc[n] * (piTmp[k - n] + piTmp[k + n]);
      piRes[k] = s >> 6;
    }
    break;
  default:
    printf("Invalid intra prediction reference filter order %d", iFilterOrder);
    exit(1);
  }

  Int ParShift = 6; //normalization factor
  Int ParScale = 1 << ParShift;
  Int ParOffset = 1 << (ParShift - 1);

  if (iOrigWeight != 0) 
  {
    Int iCmptWeight = ParScale - iOrigWeight;
    for (Int k = -iDoubleSize; k <= iDoubleSize; k++)
      piLowPassRef[k] = (iOrigWeight * piRefrVector[k] + iCmptWeight * piLowPassRef[k] + ParOffset) >> ParShift;
  }
}

#endif

#if VCEG_AZ08_INTER_KLT
Void TComPrediction::interpolatePic(TComPic* pcPic)
{
    //only perform over luma
    TComPicYuv *refPic = pcPic->getPicYuvRec();

    Pel *srcPtr;
    Pel *dstPtr;
    const ChromaFormat chFmt = pcPic->getChromaFormat();
    Int bitDepth = pcPic->getSlice(0)->getSPS()->getBitDepth(CHANNEL_TYPE_LUMA);

    TComPicYuv *refPicArray[4][4];
    pcPic->m_apcQuaPicYuv[0][0] = pcPic->getPicYuvRec();
    for (UInt uiRow = 0; uiRow < 4; uiRow++)
    {
        for (UInt uiCol = 0; uiCol < 4; uiCol++)
        {
            refPicArray[uiRow][uiCol] = pcPic->m_apcQuaPicYuv[uiRow][uiCol];
            refPicArray[uiRow][uiCol]->setBorderExtension(false);
        }
    }
    
    refPic->setBorderExtension(false);
    refPic->extendPicBorder();
    Int componentnum = 1; 
    for (UInt comp = COMPONENT_Y; comp < componentnum; comp++)
    {
        const ComponentID compID = ComponentID(comp);
        UInt uiDstStride = refPicArray[0][0]->getStride(compID);
        UInt uiRefStride = pcPic->getStride(compID);
        UInt uiWidth = refPic->getWidth(compID);
        UInt uiHeight = refPic->getHeight(compID);
        srcPtr = refPic->getAddr(compID);

        //--------------
        //Interpolation over luma
        //yFrac = 0 : (0,1)(0,2)(0,3)
        for (Int xFrac = 1; xFrac <= 3; xFrac++)
        {
            dstPtr = refPicArray[0][xFrac]->getAddr(compID);
            m_if.filterHor(compID, srcPtr, uiRefStride, dstPtr, uiDstStride, uiWidth, uiHeight, xFrac<<VCEG_AZ07_MV_ADD_PRECISION_BIT_FOR_STORE, true, chFmt, bitDepth);
        }

        //xFrac = 0: (1,0)(2,0)(3,0)
        for (Int yFrac = 1; yFrac <= 3; yFrac++)
        {
            dstPtr = refPicArray[yFrac][0]->getAddr(compID);
            m_if.filterVer(compID, srcPtr, uiRefStride, dstPtr, uiDstStride, uiWidth, uiHeight, yFrac<<VCEG_AZ07_MV_ADD_PRECISION_BIT_FOR_STORE, true, true, chFmt, bitDepth);
        }

        //other positions
        //(1,1)(2,1)(3,1)
        //(2,1)(2,2)(2,3)
        //(3,1)(3,2)(3,3)
        Pel* tmpPtr = m_tempPicYuv->getAddr( compID );
        Int tmpStride = m_tempPicYuv->getStride( compID );
        Int vFilterSize = isLuma(compID) ? NTAPS_LUMA : NTAPS_CHROMA;
        for (Int xFrac = 1; xFrac <= 3; xFrac++)
        {
            for (Int yFrac = 1; yFrac <= 3; yFrac++)
            {
                dstPtr = refPicArray[yFrac][xFrac]->getAddr(compID);
                m_if.filterHor( compID, srcPtr-((vFilterSize>>1) -1)*uiRefStride  , uiRefStride , tmpPtr , tmpStride , uiWidth , uiHeight + vFilterSize - 1, xFrac<< VCEG_AZ07_MV_ADD_PRECISION_BIT_FOR_STORE , false , chFmt , bitDepth );
                m_if.filterVer( compID, tmpPtr+((vFilterSize>>1) -1)*tmpStride , tmpStride, dstPtr, uiDstStride, uiWidth, uiHeight, yFrac<<VCEG_AZ07_MV_ADD_PRECISION_BIT_FOR_STORE, false, true, chFmt, bitDepth);
            }
        }
    }

    for (Int yFrac = 0; yFrac <= 3; yFrac++)
    {
        for (Int xFrac = 0; xFrac <= 3; xFrac++)
        {
            refPicArray[yFrac][xFrac]->extendPicBorder();
        }
    }
}
#endif
//! \}
