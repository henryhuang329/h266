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

/** \file     TComPattern.cpp
    \brief    neighbouring pixel access classes
*/

#include "TComPic.h"
#include "TComPattern.h"
#include "TComDataCU.h"
#include "TComTU.h"
#include "Debug.h"
#include "TComPrediction.h"

//! \ingroup TLibCommon
//! \{

// Forward declarations

/// padding of unavailable reference samples for intra prediction
Void fillReferenceSamples( const Int bitDepth, 
#if O0043_BEST_EFFORT_DECODING
                           const Int bitDepthDelta, 
#endif
                           const Pel* piRoiOrigin, 
                                 Pel* piIntraTemp,
                           const Bool* bNeighborFlags,
                           const Int iNumIntraNeighbor, 
                           const Int unitWidth, 
                           const Int unitHeight, 
                           const Int iAboveUnits, 
                           const Int iLeftUnits,
                           const UInt uiWidth, 
                           const UInt uiHeight, 
                           const Int iPicStride );

/// constrained intra prediction
Bool  isAboveLeftAvailable  ( TComDataCU* pcCU, UInt uiPartIdxLT );
Int   isAboveAvailable      ( TComDataCU* pcCU, UInt uiPartIdxLT, UInt uiPartIdxRT, Bool* bValidFlags );
Int   isLeftAvailable       ( TComDataCU* pcCU, UInt uiPartIdxLT, UInt uiPartIdxLB, Bool* bValidFlags );
#if JVET_C0024_QTBT
Int   isAboveRightAvailable ( TComDataCU* pcCU, UInt uiPartIdxRT, UInt uiNumUnitsInPU, Bool *bValidFlags );
Int   isBelowLeftAvailable  ( TComDataCU* pcCU, UInt uiPartIdxLB, UInt uiNumUnitsInPU, Bool* bValidFlags );
#else
Int   isAboveRightAvailable ( TComDataCU* pcCU, UInt uiPartIdxLT, UInt uiPartIdxRT, Bool* bValidFlags );
Int   isBelowLeftAvailable  ( TComDataCU* pcCU, UInt uiPartIdxLT, UInt uiPartIdxLB, Bool* bValidFlags );
#endif


// ====================================================================================================================
// Public member functions (TComPatternParam)
// ====================================================================================================================

/** 
 \param  piTexture     pixel data
 \param  iRoiWidth     pattern width
 \param  iRoiHeight    pattern height
 \param  iStride       buffer stride
 \param  bitDepth      bit depth
 */
Void TComPatternParam::setPatternParamPel ( Pel* piTexture,
                                           Int iRoiWidth,
                                           Int iRoiHeight,
                                           Int iStride,
                                           Int bitDepth
                                           )
{
  m_piROIOrigin    = piTexture;
  m_iROIWidth      = iRoiWidth;
  m_iROIHeight     = iRoiHeight;
  m_iPatternStride = iStride;
  m_bitDepth       = bitDepth;
}


// ====================================================================================================================
// Initialize static members (TComPattern)
// ====================================================================================================================

#if COM16_C983_RSAF
//Implementations of low-pass filters used in RSAF
inline Pel lp3tapFilterHor(const Pel* pCurPxl) //[1, 2, 1]/4 low-pass filter for processing rows
{
  return((pCurPxl[-1] + (pCurPxl[0]<<1) + pCurPxl[1] + 2)>>2);
}

inline Pel lp5tapFilterHor(const Pel* pCurPxl) //[2, 3, 6, 3, 2]/16 low-pass filter for processing rows
{
  return((pCurPxl[-1] + pCurPxl[1] + ((pCurPxl[-2] + pCurPxl[-1] + pCurPxl[0] + pCurPxl[1] + pCurPxl[2])<<1) + (pCurPxl[0]<<2) + 8)>>4);
}

inline Pel lp3tapFilterVer(const Pel* pCurPxl, Int iStride) //[1, 2, 1]/4 low-pass filter for processing columns
{
  return((pCurPxl[-iStride] + (pCurPxl[0]<<1) + pCurPxl[iStride] + 2)>>2);
}

inline Pel lp5tapFilterVer(const Pel* pCurPxl, Int iStride) //[2, 3, 6, 3, 2]/16 low-pass filter for processing columns
{
  Int iDoubleStride = (iStride<<1);
  return((pCurPxl[-iStride] + pCurPxl[iStride] + ((pCurPxl[-iDoubleStride] + pCurPxl[-iStride] + pCurPxl[0] + pCurPxl[iStride] + pCurPxl[iDoubleStride])<<1) + (pCurPxl[0]<<2) + 8)>>4);
}
#endif

// ====================================================================================================================
// Public member functions (TComPattern)
// ====================================================================================================================

Void TComPattern::initPattern (Pel* piY,
                               Int iRoiWidth,
                               Int iRoiHeight,
                               Int iStride,
                               Int bitDepthLuma)
{
  m_cPatternY. setPatternParamPel( piY,  iRoiWidth, iRoiHeight, iStride, bitDepthLuma);
}



// TODO: move this function to TComPrediction.cpp.
Void TComPrediction::initIntraPatternChType( TComTU &rTu, const ComponentID compID, const Bool bFilterRefSamples 
#if COM16_C983_RSAF
   , Bool bRSAF
#endif
  DEBUG_STRING_FN_DECLARE(sDebug)
  )
{
#if COM16_C983_RSAF
  bRSAF &= (compID==COMPONENT_Y);
#endif

  const ChannelType chType    = toChannelType(compID);

  TComDataCU *pcCU=rTu.getCU();
  const TComSPS &sps = *(pcCU->getSlice()->getSPS());
  const UInt uiZorderIdxInPart=rTu.GetAbsPartIdxTU();
  const UInt uiTuWidth        = rTu.getRect(compID).width;
  const UInt uiTuHeight       = rTu.getRect(compID).height;
#if JVET_C0024_QTBT
  const UInt uiTuWidth2       = uiTuWidth  + uiTuHeight;
  const UInt uiTuHeight2      = uiTuHeight + uiTuWidth;
#else
  const UInt uiTuWidth2       = uiTuWidth  << 1;
  const UInt uiTuHeight2      = uiTuHeight << 1;
#endif

#if JVET_C0024_QTBT
  assert(uiZorderIdxInPart==0);
  const Int  iBaseUnitSize    = sps.getCTUSize() >> sps.getMaxTotalCUDepth();
#else
  const Int  iBaseUnitSize    = sps.getMaxCUWidth() >> sps.getMaxTotalCUDepth();
#endif
  const Int  iUnitWidth       = iBaseUnitSize  >> pcCU->getPic()->getPicYuvRec()->getComponentScaleX(compID);
  const Int  iUnitHeight      = iBaseUnitSize  >> pcCU->getPic()->getPicYuvRec()->getComponentScaleY(compID);
  const Int  iTUWidthInUnits  = uiTuWidth  / iUnitWidth;
  const Int  iTUHeightInUnits = uiTuHeight / iUnitHeight;
#if JVET_C0024_QTBT
  const Int  iAboveUnits      = iTUWidthInUnits  + iTUHeightInUnits;
  const Int  iLeftUnits       = iTUHeightInUnits + iTUWidthInUnits;
#else
  const Int  iAboveUnits      = iTUWidthInUnits  << 1;
  const Int  iLeftUnits       = iTUHeightInUnits << 1;
#endif
  const Int  bitDepthForChannel = sps.getBitDepth(chType);

  assert(iTUHeightInUnits > 0 && iTUWidthInUnits > 0);

  const Int  iPartIdxStride   = pcCU->getPic()->getNumPartInCtuWidth();
  const UInt uiPartIdxLT      = pcCU->getZorderIdxInCtu() + uiZorderIdxInPart;
  const UInt uiPartIdxRT      = g_auiRasterToZscan[ g_auiZscanToRaster[ uiPartIdxLT ] +   iTUWidthInUnits  - 1                   ];
  const UInt uiPartIdxLB      = g_auiRasterToZscan[ g_auiZscanToRaster[ uiPartIdxLT ] + ((iTUHeightInUnits - 1) * iPartIdxStride)];

  Int   iPicStride = pcCU->getPic()->getStride(compID);
  Bool  bNeighborFlags[4 * MAX_NUM_PART_IDXS_IN_CTU_WIDTH + 1];
  Int   iNumIntraNeighbor = 0;

  bNeighborFlags[iLeftUnits] = isAboveLeftAvailable( pcCU, uiPartIdxLT );
  iNumIntraNeighbor += bNeighborFlags[iLeftUnits] ? 1 : 0;
  iNumIntraNeighbor  += isAboveAvailable     ( pcCU, uiPartIdxLT, uiPartIdxRT, (bNeighborFlags + iLeftUnits + 1)                    );
#if JVET_C0024_QTBT
  iNumIntraNeighbor  += isAboveRightAvailable( pcCU, uiPartIdxRT, iTUHeightInUnits, (bNeighborFlags + iLeftUnits + 1 + iTUWidthInUnits ) );
#else
  iNumIntraNeighbor  += isAboveRightAvailable( pcCU, uiPartIdxLT, uiPartIdxRT, (bNeighborFlags + iLeftUnits + 1 + iTUWidthInUnits ) );
#endif
  iNumIntraNeighbor  += isLeftAvailable      ( pcCU, uiPartIdxLT, uiPartIdxLB, (bNeighborFlags + iLeftUnits - 1)                    );
#if JVET_C0024_QTBT
  iNumIntraNeighbor  += isBelowLeftAvailable ( pcCU, uiPartIdxLB, iTUWidthInUnits, (bNeighborFlags + iLeftUnits - 1 - iTUHeightInUnits) );
#else
  iNumIntraNeighbor  += isBelowLeftAvailable ( pcCU, uiPartIdxLT, uiPartIdxLB, (bNeighborFlags + iLeftUnits - 1 - iTUHeightInUnits) );
#endif

  const UInt         uiROIWidth  = uiTuWidth2+1;
  const UInt         uiROIHeight = uiTuHeight2+1;

  assert(uiROIWidth*uiROIHeight <= m_iYuvExtSize);

#if DEBUG_STRING
  std::stringstream ss(stringstream::out);
#endif

  {
    Pel *piIntraTemp   = m_piYuvExt[compID][PRED_BUF_UNFILTERED];
    Pel *piRoiOrigin = pcCU->getPic()->getPicYuvRec()->getAddr(compID, pcCU->getCtuRsAddr(), pcCU->getZorderIdxInCtu()+uiZorderIdxInPart);
#if O0043_BEST_EFFORT_DECODING
    const Int  bitDepthForChannelInStream = sps.getStreamBitDepth(chType);
    fillReferenceSamples (bitDepthForChannelInStream, bitDepthForChannelInStream - bitDepthForChannel,
#else
    fillReferenceSamples (bitDepthForChannel,
#endif
                          piRoiOrigin, piIntraTemp, bNeighborFlags, iNumIntraNeighbor,  iUnitWidth, iUnitHeight, iAboveUnits, iLeftUnits,
                          uiROIWidth, uiROIHeight, iPicStride);

#if DEBUG_STRING
    if (DebugOptionList::DebugString_Pred.getInt()&DebugStringGetPredModeMask(MODE_INTRA))
    {
      ss << "###: generating Ref Samples for channel " << compID << " and " << rTu.getRect(compID).width << " x " << rTu.getRect(compID).height << "\n";
      for (UInt y=0; y<uiROIHeight; y++)
      {
        ss << "###: - ";
        for (UInt x=0; x<uiROIWidth; x++)
        {
          if (x==0 || y==0)
          {
            ss << piIntraTemp[y*uiROIWidth + x] << ", ";
//          if (x%16==15) ss << "\nPart size: ~ ";
          }
        }
        ss << "\n";
      }
    }
#endif

#if !COM16_C983_RSAF
    if (bFilterRefSamples)
#else
    if (bFilterRefSamples || bRSAF)
#endif
    {
      // generate filtered intra prediction samples

            Int          stride    = uiROIWidth;
      const Pel         *piSrcPtr  = piIntraTemp                           + (stride * uiTuHeight2); // bottom left
            Pel         *piDestPtr = m_piYuvExt[compID][PRED_BUF_FILTERED] + (stride * uiTuHeight2); // bottom left

      //------------------------------------------------

      Bool useStrongIntraSmoothing = isLuma(chType) && sps.getUseStrongIntraSmoothing();
#if COM16_C983_RSAF
      if (pcCU->getSlice()->getSPS()->getUseRSAF())
      {
        useStrongIntraSmoothing = false;
      }
      const Bool bIsWeakSmoothing = !bRSAF || !bFilterRefSamples;
#endif
      const Pel bottomLeft = piIntraTemp[stride * uiTuHeight2];
      const Pel topLeft    = piIntraTemp[0];
      const Pel topRight   = piIntraTemp[uiTuWidth2];

      if (useStrongIntraSmoothing)
      {
#if O0043_BEST_EFFORT_DECODING
        const Int  threshold     = 1 << (bitDepthForChannelInStream - 5);
#else
        const Int  threshold     = 1 << (bitDepthForChannel - 5);
#endif
        const Bool bilinearLeft  = abs((bottomLeft + topLeft ) - (2 * piIntraTemp[stride * uiTuHeight])) < threshold; //difference between the
        const Bool bilinearAbove = abs((topLeft    + topRight) - (2 * piIntraTemp[         uiTuWidth ])) < threshold; //ends and the middle
        if ((uiTuWidth < 32) || (!bilinearLeft) || (!bilinearAbove))
        {
          useStrongIntraSmoothing = false;
        }
      }

      *piDestPtr = *piSrcPtr; // bottom left is not filtered
      piDestPtr -= stride;
      piSrcPtr  -= stride;

      //------------------------------------------------

      //left column (bottom to top)

      if (useStrongIntraSmoothing)
      {
#if JVET_C0024_QTBT
        const Int shift = g_aucConvertToBit[uiTuHeight] + MIN_CU_LOG2 + 1; //log2(uiTuHeight2)    //it is a bug for non-square PU for strong filter, JCA
#else
        const Int shift = g_aucConvertToBit[uiTuHeight] + 3; //log2(uiTuHeight2)    //it is a bug for non-square PU for strong filter, 
#endif

        for(UInt i=1; i<uiTuHeight2; i++, piDestPtr-=stride)
        {
          *piDestPtr = (((uiTuHeight2 - i) * bottomLeft) + (i * topLeft) + uiTuHeight) >> shift;
        }

        piSrcPtr -= stride * (uiTuHeight2 - 1);
      }
      else
      {
#if COM16_C983_RSAF
        //First pixel always use weak smoothing
        *piDestPtr = lp3tapFilterVer(piSrcPtr,stride);
        piDestPtr-=stride; 
        piSrcPtr-=stride;

        for(UInt i=2; i<uiTuHeight2; i++, piDestPtr-=stride, piSrcPtr-=stride)
#else
        for(UInt i=1; i<uiTuHeight2; i++, piDestPtr-=stride, piSrcPtr-=stride)
#endif
        {
#if COM16_C983_RSAF
          *piDestPtr = ( bIsWeakSmoothing || (i == uiTuHeight2-1) ) ? lp3tapFilterVer(piSrcPtr,stride) : lp5tapFilterVer(piSrcPtr,stride);
#else
          *piDestPtr = ( piSrcPtr[stride] + 2*piSrcPtr[0] + piSrcPtr[-stride] + 2 ) >> 2;
#endif
        }
      }

      //------------------------------------------------

      //top-left

      if (useStrongIntraSmoothing)
      {
        *piDestPtr = piSrcPtr[0];
      }
      else
      {
        *piDestPtr = ( piSrcPtr[stride] + 2*piSrcPtr[0] + piSrcPtr[1] + 2 ) >> 2;
      }
      piDestPtr += 1;
      piSrcPtr  += 1;

      //------------------------------------------------

      //top row (left-to-right)

      if (useStrongIntraSmoothing)
      {
#if JVET_C0024_QTBT
        const Int shift = g_aucConvertToBit[uiTuWidth] + MIN_CU_LOG2 + 1; //log2(uiTuWidth2)
#else
        const Int shift = g_aucConvertToBit[uiTuWidth] + 3; //log2(uiTuWidth2)
#endif

        for(UInt i=1; i<uiTuWidth2; i++, piDestPtr++)
        {
          *piDestPtr = (((uiTuWidth2 - i) * topLeft) + (i * topRight) + uiTuWidth) >> shift;
        }

        piSrcPtr += uiTuWidth2 - 1;
      }
      else
      {
#if COM16_C983_RSAF
        //First pixel always use weak smoothing
        *piDestPtr = lp3tapFilterHor(piSrcPtr);
        piDestPtr++; 
        piSrcPtr++;

        for(UInt i=2; i<uiTuWidth2; i++, piDestPtr++, piSrcPtr++)
#else
        for(UInt i=1; i<uiTuWidth2; i++, piDestPtr++, piSrcPtr++)
#endif
        {
#if COM16_C983_RSAF         
          *piDestPtr = ( bIsWeakSmoothing || (i == uiTuWidth2-1) ) ? lp3tapFilterHor(piSrcPtr) : lp5tapFilterHor(piSrcPtr);
#else
          *piDestPtr = ( piSrcPtr[1] + 2*piSrcPtr[0] + piSrcPtr[-1] + 2 ) >> 2;
#endif
        }
      }

      //------------------------------------------------

      *piDestPtr=*piSrcPtr; // far right is not filtered

#if DEBUG_STRING
    if (DebugOptionList::DebugString_Pred.getInt()&DebugStringGetPredModeMask(MODE_INTRA))
    {
      ss << "###: filtered result for channel " << compID <<"\n";
      for (UInt y=0; y<uiROIHeight; y++)
      {
        ss << "###: - ";
        for (UInt x=0; x<uiROIWidth; x++)
        {
          if (x==0 || y==0)
          {
            ss << m_piYuvExt[compID][PRED_BUF_FILTERED][y*uiROIWidth + x] << ", ";
//          if (x%16==15) ss << "\nPart size: ~ ";
          }
        }
        ss << "\n";
      }
    }
#endif
    }
  }
  DEBUG_STRING_APPEND(sDebug, ss.str())
}

Void fillReferenceSamples( const Int bitDepth, 
#if O0043_BEST_EFFORT_DECODING
                           const Int bitDepthDelta, 
#endif
                           const Pel* piRoiOrigin, 
                                 Pel* piIntraTemp,
                           const Bool* bNeighborFlags,
                           const Int iNumIntraNeighbor, 
                           const Int unitWidth, 
                           const Int unitHeight, 
                           const Int iAboveUnits, 
                           const Int iLeftUnits,
                           const UInt uiWidth, 
                           const UInt uiHeight, 
                           const Int iPicStride )
{
  const Pel* piRoiTemp;
  Int  i, j;
  Int  iDCValue = 1 << (bitDepth - 1);
  const Int iTotalUnits = iAboveUnits + iLeftUnits + 1; //+1 for top-left

  if (iNumIntraNeighbor == 0)
  {
    // Fill border with DC value
    for (i=0; i<uiWidth; i++)
    {
      piIntraTemp[i] = iDCValue;
    }
    for (i=1; i<uiHeight; i++)
    {
      piIntraTemp[i*uiWidth] = iDCValue;
    }
  }
  else if (iNumIntraNeighbor == iTotalUnits)
  {
    // Fill top-left border and top and top right with rec. samples
    piRoiTemp = piRoiOrigin - iPicStride - 1;

    for (i=0; i<uiWidth; i++)
    {
#if O0043_BEST_EFFORT_DECODING
      piIntraTemp[i] = piRoiTemp[i] << bitDepthDelta;
#else
      piIntraTemp[i] = piRoiTemp[i];
#endif
    }

    // Fill left and below left border with rec. samples
    piRoiTemp = piRoiOrigin - 1;

    for (i=1; i<uiHeight; i++)
    {
#if O0043_BEST_EFFORT_DECODING
      piIntraTemp[i*uiWidth] = (*(piRoiTemp)) << bitDepthDelta;
#else
      piIntraTemp[i*uiWidth] = *(piRoiTemp);
#endif
      piRoiTemp += iPicStride;
    }
  }
  else // reference samples are partially available
  {
    // all above units have "unitWidth" samples each, all left/below-left units have "unitHeight" samples each
    const Int  iTotalSamples = (iLeftUnits * unitHeight) + ((iAboveUnits + 1) * unitWidth);
    Pel  piIntraLine[5 * MAX_CU_SIZE];
    Pel  *piIntraLineTemp;
    const Bool *pbNeighborFlags;


    // Initialize
    for (i=0; i<iTotalSamples; i++)
    {
      piIntraLine[i] = iDCValue;
    }

    // Fill top-left sample
    piRoiTemp = piRoiOrigin - iPicStride - 1;
    piIntraLineTemp = piIntraLine + (iLeftUnits * unitHeight);
    pbNeighborFlags = bNeighborFlags + iLeftUnits;
    if (*pbNeighborFlags)
    {
#if O0043_BEST_EFFORT_DECODING
      Pel topLeftVal=piRoiTemp[0] << bitDepthDelta;
#else
      Pel topLeftVal=piRoiTemp[0];
#endif
      for (i=0; i<unitWidth; i++)
      {
        piIntraLineTemp[i] = topLeftVal;
      }
    }

    // Fill left & below-left samples (downwards)
    piRoiTemp += iPicStride;
    piIntraLineTemp--;
    pbNeighborFlags--;

    for (j=0; j<iLeftUnits; j++)
    {
      if (*pbNeighborFlags)
      {
        for (i=0; i<unitHeight; i++)
        {
#if O0043_BEST_EFFORT_DECODING
          piIntraLineTemp[-i] = piRoiTemp[i*iPicStride] << bitDepthDelta;
#else
          piIntraLineTemp[-i] = piRoiTemp[i*iPicStride];
#endif
        }
      }
      piRoiTemp += unitHeight*iPicStride;
      piIntraLineTemp -= unitHeight;
      pbNeighborFlags--;
    }

    // Fill above & above-right samples (left-to-right) (each unit has "unitWidth" samples)
    piRoiTemp = piRoiOrigin - iPicStride;
    // offset line buffer by iNumUints2*unitHeight (for left/below-left) + unitWidth (for above-left)
    piIntraLineTemp = piIntraLine + (iLeftUnits * unitHeight) + unitWidth;
    pbNeighborFlags = bNeighborFlags + iLeftUnits + 1;
    for (j=0; j<iAboveUnits; j++)
    {
      if (*pbNeighborFlags)
      {
        for (i=0; i<unitWidth; i++)
        {
#if O0043_BEST_EFFORT_DECODING
          piIntraLineTemp[i] = piRoiTemp[i] << bitDepthDelta;
#else
          piIntraLineTemp[i] = piRoiTemp[i];
#endif
        }
      }
      piRoiTemp += unitWidth;
      piIntraLineTemp += unitWidth;
      pbNeighborFlags++;
    }

    // Pad reference samples when necessary
    Int iCurrJnit = 0;
    Pel  *piIntraLineCur   = piIntraLine;
    const UInt piIntraLineTopRowOffset = iLeftUnits * (unitHeight - unitWidth);

    if (!bNeighborFlags[0])
    {
      // very bottom unit of bottom-left; at least one unit will be valid.
      {
        Int   iNext = 1;
        while (iNext < iTotalUnits && !bNeighborFlags[iNext])
        {
          iNext++;
        }
        Pel *piIntraLineNext = piIntraLine + ((iNext < iLeftUnits) ? (iNext * unitHeight) : (piIntraLineTopRowOffset + (iNext * unitWidth)));
        const Pel refSample = *piIntraLineNext;
        // Pad unavailable samples with new value
        Int iNextOrTop = std::min<Int>(iNext, iLeftUnits);
        // fill left column
        while (iCurrJnit < iNextOrTop)
        {
          for (i=0; i<unitHeight; i++)
          {
            piIntraLineCur[i] = refSample;
          }
          piIntraLineCur += unitHeight;
          iCurrJnit++;
        }
        // fill top row
        while (iCurrJnit < iNext)
        {
          for (i=0; i<unitWidth; i++)
          {
            piIntraLineCur[i] = refSample;
          }
          piIntraLineCur += unitWidth;
          iCurrJnit++;
        }
      }
    }

    // pad all other reference samples.
    while (iCurrJnit < iTotalUnits)
    {
      if (!bNeighborFlags[iCurrJnit]) // samples not available
      {
        {
          const Int numSamplesInCurrUnit = (iCurrJnit >= iLeftUnits) ? unitWidth : unitHeight;
          const Pel refSample = *(piIntraLineCur-1);
          for (i=0; i<numSamplesInCurrUnit; i++)
          {
            piIntraLineCur[i] = refSample;
          }
          piIntraLineCur += numSamplesInCurrUnit;
          iCurrJnit++;
        }
      }
      else
      {
        piIntraLineCur += (iCurrJnit >= iLeftUnits) ? unitWidth : unitHeight;
        iCurrJnit++;
      }
    }

    // Copy processed samples

    piIntraLineTemp = piIntraLine + uiHeight + unitWidth - 2;
    // top left, top and top right samples
    for (i=0; i<uiWidth; i++)
    {
      piIntraTemp[i] = piIntraLineTemp[i];
    }

    piIntraLineTemp = piIntraLine + uiHeight - 1;
    for (i=1; i<uiHeight; i++)
    {
      piIntraTemp[i*uiWidth] = piIntraLineTemp[-i];
    }
  }
}

Bool TComPrediction::filteringIntraReferenceSamples(const ComponentID compID, UInt uiDirMode, UInt uiTuChWidth, UInt uiTuChHeight, const ChromaFormat chFmt, const Bool intraReferenceSmoothingDisabled
#if COM16_C983_RSAF_PREVENT_OVERSMOOTHING
                                                   , Bool enableRSAF
#endif
                                                   )
{
  Bool bFilter;

  if (!filterIntraReferenceSamples(toChannelType(compID), chFmt, intraReferenceSmoothingDisabled))
  {
    bFilter=false;
  }
  else
  {
#if COM16_C806_LARGE_CTU || JVET_C0024_QTBT
    assert(uiTuChWidth>=MIN_PU_SIZE && uiTuChHeight>=MIN_PU_SIZE && uiTuChWidth<=MAX_CU_SIZE && uiTuChHeight<=MAX_CU_SIZE);
#else
    assert(uiTuChWidth>=4 && uiTuChHeight>=4 && uiTuChWidth<128 && uiTuChHeight<128);
#endif

    if (uiDirMode == DC_IDX)
    {
      bFilter=false; //no smoothing for DC or LM chroma
    }
    else
    {
      Int diff = min<Int>(abs((Int) uiDirMode - HOR_IDX), abs((Int)uiDirMode - VER_IDX));
#if JVET_C0024_QTBT
      UInt log2Size=((g_aucConvertToBit[uiTuChWidth]+g_aucConvertToBit[uiTuChHeight])>>1) + MIN_CU_LOG2;
      UInt sizeIndex = log2Size - 1;
#else
      UInt sizeIndex=g_aucConvertToBit[uiTuChWidth];
#endif
      assert(sizeIndex < MAX_INTRA_FILTER_DEPTHS);
#if COM16_C983_RSAF_PREVENT_OVERSMOOTHING
      bFilter = diff > m_aucIntraFilter[toChannelType(compID)][sizeIndex] -   ( (compID==COMPONENT_Y && enableRSAF && sizeIndex==1) ? 2 : 0);
#else
      bFilter = diff > m_aucIntraFilter[toChannelType(compID)][sizeIndex];
#endif
    }
  }
  return bFilter;
}

Bool isAboveLeftAvailable( TComDataCU* pcCU, UInt uiPartIdxLT )
{
  Bool bAboveLeftFlag;
  UInt uiPartAboveLeft;
  TComDataCU* pcCUAboveLeft = pcCU->getPUAboveLeft( uiPartAboveLeft, uiPartIdxLT );
  if(pcCU->getSlice()->getPPS()->getConstrainedIntraPred())
  {
    bAboveLeftFlag = ( pcCUAboveLeft && pcCUAboveLeft->isIntra( uiPartAboveLeft ) );
  }
  else
  {
    bAboveLeftFlag = (pcCUAboveLeft ? true : false);
  }
  return bAboveLeftFlag;
}

Int isAboveAvailable( TComDataCU* pcCU, UInt uiPartIdxLT, UInt uiPartIdxRT, Bool *bValidFlags )
{
  const UInt uiRasterPartBegin = g_auiZscanToRaster[uiPartIdxLT];
  const UInt uiRasterPartEnd = g_auiZscanToRaster[uiPartIdxRT]+1;
  const UInt uiIdxStep = 1;
  Bool *pbValidFlags = bValidFlags;
  Int iNumIntra = 0;

  for ( UInt uiRasterPart = uiRasterPartBegin; uiRasterPart < uiRasterPartEnd; uiRasterPart += uiIdxStep )
  {
    UInt uiPartAbove;
    TComDataCU* pcCUAbove = pcCU->getPUAbove( uiPartAbove, g_auiRasterToZscan[uiRasterPart] );
    if(pcCU->getSlice()->getPPS()->getConstrainedIntraPred())
    {
      if ( pcCUAbove && pcCUAbove->isIntra( uiPartAbove ) )
      {
        iNumIntra++;
        *pbValidFlags = true;
      }
      else
      {
        *pbValidFlags = false;
      }
    }
    else
    {
      if (pcCUAbove)
      {
        iNumIntra++;
        *pbValidFlags = true;
      }
      else
      {
        *pbValidFlags = false;
      }
    }
    pbValidFlags++;
  }
  return iNumIntra;
}

Int isLeftAvailable( TComDataCU* pcCU, UInt uiPartIdxLT, UInt uiPartIdxLB, Bool *bValidFlags )
{
  const UInt uiRasterPartBegin = g_auiZscanToRaster[uiPartIdxLT];
  const UInt uiRasterPartEnd = g_auiZscanToRaster[uiPartIdxLB]+1;
  const UInt uiIdxStep = pcCU->getPic()->getNumPartInCtuWidth();
  Bool *pbValidFlags = bValidFlags;
  Int iNumIntra = 0;

  for ( UInt uiRasterPart = uiRasterPartBegin; uiRasterPart < uiRasterPartEnd; uiRasterPart += uiIdxStep )
  {
    UInt uiPartLeft;
    TComDataCU* pcCULeft = pcCU->getPULeft( uiPartLeft, g_auiRasterToZscan[uiRasterPart] );
    if(pcCU->getSlice()->getPPS()->getConstrainedIntraPred())
    {
      if ( pcCULeft && pcCULeft->isIntra( uiPartLeft ) )
      {
        iNumIntra++;
        *pbValidFlags = true;
      }
      else
      {
        *pbValidFlags = false;
      }
    }
    else
    {
      if ( pcCULeft )
      {
        iNumIntra++;
        *pbValidFlags = true;
      }
      else
      {
        *pbValidFlags = false;
      }
    }
    pbValidFlags--; // opposite direction
  }

  return iNumIntra;
}

#if JVET_C0024_QTBT
Int isAboveRightAvailable( TComDataCU* pcCU, UInt uiPartIdxRT, UInt uiNumUnitsInPU, Bool *bValidFlags )
#else
Int isAboveRightAvailable( TComDataCU* pcCU, UInt uiPartIdxLT, UInt uiPartIdxRT, Bool *bValidFlags )
#endif
{
#if !JVET_C0024_QTBT
  const UInt uiNumUnitsInPU = g_auiZscanToRaster[uiPartIdxRT] - g_auiZscanToRaster[uiPartIdxLT] + 1;
#endif
  Bool *pbValidFlags = bValidFlags;
  Int iNumIntra = 0;

  for ( UInt uiOffset = 1; uiOffset <= uiNumUnitsInPU; uiOffset++ )
  {
    UInt uiPartAboveRight;
    TComDataCU* pcCUAboveRight = pcCU->getPUAboveRight( uiPartAboveRight, uiPartIdxRT, uiOffset );
    if(pcCU->getSlice()->getPPS()->getConstrainedIntraPred())
    {
      if ( pcCUAboveRight && pcCUAboveRight->isIntra( uiPartAboveRight ) )
      {
        iNumIntra++;
        *pbValidFlags = true;
      }
      else
      {
        *pbValidFlags = false;
      }
    }
    else
    {
      if ( pcCUAboveRight )
      {
        iNumIntra++;
        *pbValidFlags = true;
      }
      else
      {
        *pbValidFlags = false;
      }
    }
    pbValidFlags++;
  }

  return iNumIntra;
}

#if JVET_C0024_QTBT
Int isBelowLeftAvailable( TComDataCU* pcCU, UInt uiPartIdxLB, UInt uiNumUnitsInPU, Bool *bValidFlags )
#else
Int isBelowLeftAvailable( TComDataCU* pcCU, UInt uiPartIdxLT, UInt uiPartIdxLB, Bool *bValidFlags )
#endif
{
#if !JVET_C0024_QTBT
  const UInt uiNumUnitsInPU = (g_auiZscanToRaster[uiPartIdxLB] - g_auiZscanToRaster[uiPartIdxLT]) / pcCU->getPic()->getNumPartInCtuWidth() + 1;
#endif
  Bool *pbValidFlags = bValidFlags;
  Int iNumIntra = 0;

  for ( UInt uiOffset = 1; uiOffset <= uiNumUnitsInPU; uiOffset++ )
  {
    UInt uiPartBelowLeft;
    TComDataCU* pcCUBelowLeft = pcCU->getPUBelowLeft( uiPartBelowLeft, uiPartIdxLB, uiOffset );
    if(pcCU->getSlice()->getPPS()->getConstrainedIntraPred())
    {
      if ( pcCUBelowLeft && pcCUBelowLeft->isIntra( uiPartBelowLeft ) )
      {
        iNumIntra++;
        *pbValidFlags = true;
      }
      else
      {
        *pbValidFlags = false;
      }
    }
    else
    {
      if ( pcCUBelowLeft )
      {
        iNumIntra++;
        *pbValidFlags = true;
      }
      else
      {
        *pbValidFlags = false;
      }
    }
    pbValidFlags--; // opposite direction
  }

  return iNumIntra;
}
//! \}
