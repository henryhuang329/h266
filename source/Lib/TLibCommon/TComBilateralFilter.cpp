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

#include <stdio.h>
#include <math.h>
#include <string.h>
#include <algorithm>
#include "TComBilateralFilter.h"

const int TComBilateralFilter::SpatialSigmaValue = 62;

const int TComBilateralFilter::spatialSigmaBlockLengthOffsets[] = {20, 10, -10, 0, -10};
TComBilateralFilter* TComBilateralFilter::m_bilateralFilterInstance = NULL;

const UShort maxPosList[34] = {6, 12, 18, 23, 29, 35, 41, 46, 52, 58, 64, 69, 75, 81, 87, 92, 98, 104, 110, 115, 121, 127, 133, 138, 144, 150, 156, 161, 167, 173, 179, 184, 190, 196};

TComBilateralFilter::TComBilateralFilter()
{
  int numQP = MAX_QP-18+1;
  // allocation
  m_bilateralFilterTable = new UShort*[numQP];
  for(int i = 0; i < numQP; i++)
  {
    m_bilateralFilterTable[i] = new UShort[maxPosList[i]+1];
  }

  // initialization
  for(int i = 0; i < numQP; i++)
  {
    for(int k = 0; k < (maxPosList[i]+1); k++)
    {
      m_bilateralFilterTable[i][k] = 0;
    }
  }
  m_initFlag = false;
}

TComBilateralFilter::~TComBilateralFilter()
{
  int numQP = MAX_QP-18+1;
  for(int i = 0; i < numQP; ++i)
  {
    delete [] m_bilateralFilterTable[i];
  }
  delete [] m_bilateralFilterTable;
}

TComBilateralFilter* TComBilateralFilter::instance()
{
  if (m_bilateralFilterInstance == NULL)
    m_bilateralFilterInstance = new TComBilateralFilter();
  return m_bilateralFilterInstance;
}

Void TComBilateralFilter::createdivToMulLUTs()
{
  UInt one = 1 << BITS_PER_DIV_LUT_ENTRY; // 1 is represented by 2^14 (not 2^14 -1)
  divToMulOneOverN[0] = one; // We can never divide by zero since the centerweight is non-zero, so we can set this value to something arbitrary.
  divToMulShift[0] = 0;
  
  for (UInt n=1; n<BILATERAL_FILTER_MAX_DENOMINATOR_PLUS_ONE; n++)
  {
    UInt tryLUT = one / n;
    
    UInt tryShift = 0;
    // Make sure the LUT entry stored does not start with (binary) zeros.
    while(tryLUT <= one)
    {
      // This value of tryLUT
      divToMulOneOverN[n] = tryLUT;
      divToMulShift[n] = tryShift;
      
      tryShift++;
      tryLUT = (one << tryShift) / n;
    }
    
    // We may need to add 1 to the LUT entry in order to make 3/3, 4/4, 5/5, ... come out right.
    UInt adiv = divToMulOneOverN[n] * n / (one << divToMulShift[n]);
    if(adiv != 1)
      divToMulOneOverN[n]++;
  }
}

Void TComBilateralFilter::createBilateralFilterTable(Int qp)
{
  Int spatialSigmaValue;
  Int intensitySigmaValue = (qp - 17) * 50;
  Int sqrtSpatialSigmaMulTwo;
  Int sqrtIntensitySigmaMulTwo = 2 * intensitySigmaValue * intensitySigmaValue;
  int centerWeightTableSize = 5;

  spatialSigmaValue = SpatialSigmaValue;;
  for (Int i = 0; i < centerWeightTableSize; i++)
  {
    sqrtSpatialSigmaMulTwo = 2 * (spatialSigmaValue + spatialSigmaBlockLengthOffsets[i]) * (spatialSigmaValue + spatialSigmaBlockLengthOffsets[i]);

    // Calculate the multiplication factor that we will use to convert the first table (with the strongest filter) to one of the
    // tables that gives weaker filtering (such as when TU = 8 or 16 or when we have inter filtering). 
    Int sqrtSpatialSigmaMulTwoStrongestFiltering = 2 * (spatialSigmaValue + spatialSigmaBlockLengthOffsets[0]) * (spatialSigmaValue + spatialSigmaBlockLengthOffsets[0]);

    // multiplication factor equals exp(-1/stronger)/exp(-1/weaker)
    double centerWeightMultiplier = exp(-(10000.0 / sqrtSpatialSigmaMulTwoStrongestFiltering))/exp(-(10000.0 / sqrtSpatialSigmaMulTwo));
    m_bilateralCenterWeightTable[i] = (Int)(centerWeightMultiplier*65 + 0.5);
  }
  Int i = 0;
  sqrtSpatialSigmaMulTwo = 2 * (spatialSigmaValue + spatialSigmaBlockLengthOffsets[i]) * (spatialSigmaValue + spatialSigmaBlockLengthOffsets[i]);
  for (Int j = 0; j < (maxPosList[qp-18]+1); j++)
  {        
    Int temp = j * 25;
    m_bilateralFilterTable[qp-18][j] = UShort(exp(-(10000.0 / sqrtSpatialSigmaMulTwo) - (temp * temp / (sqrtIntensitySigmaMulTwo * 1.0))) * 65 + 0.5);
  }
}

Void TComBilateralFilter::smoothBlockBilateralFilter(TComDataCU* pcCU, UInt uiWidth, UInt uiHeight, Short block[], Int length, Int isInterBlock, Int qp)
{
  Int rightPixel, centerPixel;
  Int rightWeight, bottomWeight, centerWeight;
  Int sumWeights[MAX_CU_SIZE];
  Int sumDelta[MAX_CU_SIZE];
  Int blockLengthIndex;
  
  Int dIB, dIR;
    
  switch (length)
  {
    case 4:
      blockLengthIndex = 0;
      break;
    case 8:
      blockLengthIndex = 1;
      break;
    default:
      blockLengthIndex = 2;
      break;
  }
  
  UShort *lookupTablePtr;
  
  centerWeight = m_bilateralCenterWeightTable[blockLengthIndex + 3 * isInterBlock];
  
  Int theMaxPos = maxPosList[qp-18];
  lookupTablePtr = m_bilateralFilterTable[qp-18];

  // for each pixel in block
  
  // These are the types of pixels:
  //
  // A BB C
  //
  // D EE F
  // D EE F
  //
  // G HH I
  //
  // If the block is larger than 4x4, the E-part is larger.
  //
  // Filter types:
  //
  // AA  BBB   CC
  // A    B     C
  //
  // D    E     F
  // DD  EEE   FF
  // D    E     F
  //
  // G    H     I
  // GG  HHH   II
  // C uses a filter of type x
  Int currentPixelDeltaSum;
  Int currentPixelSumWeights;
  Int rightPixelDeltaSum;
  Int rightPixelSumWeights;
  Int rightWeightTimesdIR;
  Int bottomWeightTimesdIB;
  
  Int mySignIfNeg;
  Int mySign;
  
  Short *blockCurrentPixelPtr = block;
  Short *blockRightPixelPtr = blockCurrentPixelPtr+1;
  Short *blockNextLinePixelPtr = blockCurrentPixelPtr + uiWidth;
  Int *sumWeightsPtr = sumWeights;
  Int *sumDeltaPtr = sumDelta;
  
  // A pixel. uses filter type xx
  //                           x
  //
  // No information from previous row
  // No information from previous pixel
  // Storing information to next row
  // Storing information to next pixel
  
  // top left pixel; i = 0, j = 0;
  
  centerPixel = *(blockCurrentPixelPtr);
  rightPixel = *(blockRightPixelPtr++);
  dIR = rightPixel - centerPixel;
  dIB = *(blockNextLinePixelPtr++) - centerPixel;
  
  rightWeight = lookupTablePtr[std::min(theMaxPos, abs(dIR))];
  bottomWeight = lookupTablePtr[std::min(theMaxPos, abs(dIB))];
  
  rightWeightTimesdIR = rightWeight*dIR;
  bottomWeightTimesdIB = bottomWeight*dIB;
  
  currentPixelSumWeights = centerWeight + rightWeight + bottomWeight;
  currentPixelDeltaSum = rightWeightTimesdIR + bottomWeightTimesdIB;
  
  rightPixelSumWeights = rightWeight; //next pixel to the right
  rightPixelDeltaSum = rightWeightTimesdIR;
  
  *(sumWeightsPtr++) = bottomWeight; //next pixel to the bottom
  *(sumDeltaPtr++) = bottomWeightTimesdIB; //next pixel to the bottom
  
  mySignIfNeg = SIGN_IF_NEG(currentPixelDeltaSum);
  mySign = 1 | mySignIfNeg;

  *(blockCurrentPixelPtr++) = centerPixel + mySign*((((mySign*currentPixelDeltaSum + ((currentPixelSumWeights+mySignIfNeg) >> 1))*divToMulOneOverN[currentPixelSumWeights]) >> (BITS_PER_DIV_LUT_ENTRY + divToMulShift[currentPixelSumWeights])));
  
  for (Int i = 1; i < (uiWidth-1); i++)
  {
    // B pixel. uses filter type xxx
    //                            x
    //
    // No information from previous row
    // Information reused from previous pixel
    // Storing information to next row
    // Storing information to next pixel
    
    centerPixel = rightPixel;
    rightPixel = *(blockRightPixelPtr++);
    dIR = rightPixel - centerPixel;
    dIB = *(blockNextLinePixelPtr++) - centerPixel;
    
    rightWeight = lookupTablePtr[std::min(theMaxPos, abs(dIR))];
    bottomWeight = lookupTablePtr[std::min(theMaxPos, abs(dIB))];
    
    rightWeightTimesdIR = rightWeight*dIR;
    bottomWeightTimesdIB = bottomWeight*dIB;
    
    currentPixelSumWeights = centerWeight + rightPixelSumWeights + rightWeight + bottomWeight;
    currentPixelDeltaSum = rightWeightTimesdIR + bottomWeightTimesdIB - rightPixelDeltaSum;
    
    rightPixelSumWeights = rightWeight; //next pixel to the right
    rightPixelDeltaSum = rightWeightTimesdIR; //next pixel to the right
    
    *(sumWeightsPtr++) = bottomWeight; //next pixel to the bottom
    *(sumDeltaPtr++) = bottomWeightTimesdIB; //next pixel to the bottom
    
    mySignIfNeg = SIGN_IF_NEG(currentPixelDeltaSum);
    mySign = 1 | mySignIfNeg;

    *(blockCurrentPixelPtr++) = centerPixel + mySign*((((mySign*currentPixelDeltaSum + ((currentPixelSumWeights+mySignIfNeg) >> 1))*divToMulOneOverN[currentPixelSumWeights]) >> (BITS_PER_DIV_LUT_ENTRY + divToMulShift[currentPixelSumWeights])));
    
  }
  
  // C pixel. uses filter type xx
  //                            x
  //
  // No information from previous row
  // Information reused from previous pixel
  // Storing information to next row
  // No information to store to next pixel
  
  centerPixel = rightPixel;
  blockRightPixelPtr++;
  dIB = *(blockNextLinePixelPtr++) - centerPixel;
  
  bottomWeight = lookupTablePtr[std::min(theMaxPos, abs(dIB))];
  bottomWeightTimesdIB = bottomWeight*dIB;
  
  currentPixelSumWeights = centerWeight + rightPixelSumWeights + bottomWeight;
  currentPixelDeltaSum = bottomWeightTimesdIB - rightPixelDeltaSum;
  
  *(sumWeightsPtr) = bottomWeight; //next pixel to the bottom
  *(sumDeltaPtr++) = bottomWeightTimesdIB; //next pixel to the bottom
  
  mySignIfNeg = SIGN_IF_NEG(currentPixelDeltaSum);
  mySign = 1 | mySignIfNeg;

  *(blockCurrentPixelPtr++) = centerPixel + mySign*((((mySign*currentPixelDeltaSum + ((currentPixelSumWeights+mySignIfNeg) >> 1))*divToMulOneOverN[currentPixelSumWeights]) >> (BITS_PER_DIV_LUT_ENTRY + divToMulShift[currentPixelSumWeights])));
  
  for (Int j = 1; j < (uiHeight-1); j++)
  {
    sumWeightsPtr = sumWeights;
    sumDeltaPtr = sumDelta;
    
    //                           x
    // D pixel. uses filter type xx
    //                           x
    //
    // Uses information from previous row
    // No information from previous pixel
    // Storing information to next row
    // Storing information to next pixel
    
    centerPixel = *(blockCurrentPixelPtr);
    rightPixel = *(blockRightPixelPtr++);
    dIR = rightPixel - centerPixel;
    dIB = *(blockNextLinePixelPtr++) - centerPixel;
    
    rightWeight = lookupTablePtr[std::min(theMaxPos, abs(dIR))];
    bottomWeight = lookupTablePtr[std::min(theMaxPos, abs(dIB))];
    
    rightWeightTimesdIR = rightWeight*dIR;
    bottomWeightTimesdIB = bottomWeight*dIB;
    
    currentPixelSumWeights = centerWeight + *(sumWeightsPtr) + rightWeight + bottomWeight;
    currentPixelDeltaSum = rightWeightTimesdIR + bottomWeightTimesdIB - *(sumDeltaPtr);
    
    rightPixelSumWeights = rightWeight; //next pixel to the right
    rightPixelDeltaSum = rightWeightTimesdIR;
    
    *(sumWeightsPtr++) = bottomWeight; //next pixel to the bottom
    *(sumDeltaPtr++) = bottomWeightTimesdIB; //next pixel to the bottom
    
    mySignIfNeg = SIGN_IF_NEG(currentPixelDeltaSum);
    mySign = 1 | mySignIfNeg;

    *(blockCurrentPixelPtr++) = centerPixel + mySign*((((mySign*currentPixelDeltaSum + ((currentPixelSumWeights+mySignIfNeg) >> 1))*divToMulOneOverN[currentPixelSumWeights]) >> (BITS_PER_DIV_LUT_ENTRY + divToMulShift[currentPixelSumWeights])));
    
    for (Int i = 1; i < (uiWidth-1); i++)
    {
      //                            x
      // E pixel. uses filter type xxx
      //                            x
      //
      // Uses information from previous row
      // Uses information from previous pixel
      // Storing information to next row
      // No information to store to next pixel
      
      centerPixel = rightPixel;
      rightPixel = *(blockRightPixelPtr++);
      dIR = rightPixel - centerPixel;
      dIB = *(blockNextLinePixelPtr++) - centerPixel;
      
      rightWeight = lookupTablePtr[std::min(theMaxPos, abs(dIR))];
      bottomWeight = lookupTablePtr[std::min(theMaxPos, abs(dIB))];
      
      rightWeightTimesdIR = rightWeight*dIR;
      bottomWeightTimesdIB = bottomWeight*dIB;
      
      currentPixelSumWeights = centerWeight + *(sumWeightsPtr) + rightPixelSumWeights + rightWeight + bottomWeight;
      currentPixelDeltaSum = rightWeightTimesdIR + bottomWeightTimesdIB - rightPixelDeltaSum - *(sumDeltaPtr);
      
      rightPixelSumWeights = rightWeight; //next pixel to the right
      rightPixelDeltaSum = rightWeightTimesdIR;
      
      *(sumWeightsPtr++) = bottomWeight; //next pixel to the bottom
      *(sumDeltaPtr++) = bottomWeightTimesdIB; //next pixel to the bottom
      
      mySignIfNeg = SIGN_IF_NEG(currentPixelDeltaSum);
      mySign = 1 | mySignIfNeg;

      *(blockCurrentPixelPtr++) = centerPixel + mySign*((((mySign*currentPixelDeltaSum + ((currentPixelSumWeights+mySignIfNeg) >> 1))*divToMulOneOverN[currentPixelSumWeights]) >> (BITS_PER_DIV_LUT_ENTRY + divToMulShift[currentPixelSumWeights])));
    }
    
    //                            x
    // F pixel. uses filter type xx
    //                            x
    //
    // Uses information from previous row
    // Uses information from previous pixel
    // Storing information to next row
    // Storing information to next pixel
    
    centerPixel = rightPixel;
    blockRightPixelPtr++;
    dIB = *(blockNextLinePixelPtr++) - centerPixel;
    
    bottomWeight = lookupTablePtr[std::min(theMaxPos, abs(dIB))];
    bottomWeightTimesdIB = bottomWeight*dIB;
    
    currentPixelSumWeights = centerWeight + *(sumWeightsPtr) + rightPixelSumWeights + bottomWeight;
    currentPixelDeltaSum = bottomWeightTimesdIB - rightPixelDeltaSum - *(sumDeltaPtr);
    
    *(sumWeightsPtr) = bottomWeight; //next pixel to the bottom
    *(sumDeltaPtr++) = bottomWeightTimesdIB; //next pixel to the bottom
    
    mySignIfNeg = SIGN_IF_NEG(currentPixelDeltaSum);
    mySign = 1 | mySignIfNeg;

    *(blockCurrentPixelPtr++) = centerPixel + mySign*((((mySign*currentPixelDeltaSum + ((currentPixelSumWeights+mySignIfNeg) >> 1))*divToMulOneOverN[currentPixelSumWeights]) >> (BITS_PER_DIV_LUT_ENTRY + divToMulShift[currentPixelSumWeights])));
    
  }
  
  sumWeightsPtr = sumWeights;
  sumDeltaPtr = sumDelta;
  
  //                           x
  // G pixel. uses filter type xx
  //
  // Uses information from previous row
  // No information from previous pixel
  // No information to store to next row
  // Storing information to next pixel
  
  
  centerPixel = *(blockCurrentPixelPtr);
  rightPixel = *(blockRightPixelPtr++);
  dIR = rightPixel - centerPixel;
  
  rightWeight = lookupTablePtr[std::min(theMaxPos, abs(dIR))];
  rightWeightTimesdIR = rightWeight*dIR;
  
  currentPixelSumWeights = centerWeight + *(sumWeightsPtr++) + rightWeight;
  currentPixelDeltaSum = rightWeightTimesdIR - *(sumDeltaPtr++);
  
  rightPixelSumWeights = rightWeight; //next pixel to the right
  rightPixelDeltaSum = rightWeightTimesdIR;
  
  mySignIfNeg = SIGN_IF_NEG(currentPixelDeltaSum);
  mySign = 1 | mySignIfNeg;

  *(blockCurrentPixelPtr++) = centerPixel + mySign*((((mySign*currentPixelDeltaSum + ((currentPixelSumWeights+mySignIfNeg) >> 1))*divToMulOneOverN[currentPixelSumWeights]) >> (BITS_PER_DIV_LUT_ENTRY + divToMulShift[currentPixelSumWeights])));
  
  for (Int i = 1; i < (uiWidth-1); i++)
  {
    //                            x
    // H pixel. uses filter type xxx
    //
    // Uses information from previous row
    // Uses information from previous pixel
    // No information to store to next row
    // Storing information to next pixel
    
    centerPixel = rightPixel;
    rightPixel = *(blockRightPixelPtr++);
    dIR = rightPixel - centerPixel;
    
    rightWeight = lookupTablePtr[std::min(theMaxPos, abs(dIR))];
    rightWeightTimesdIR = rightWeight*dIR;
    
    currentPixelSumWeights = centerWeight + *(sumWeightsPtr++) + rightWeight + rightPixelSumWeights;
    currentPixelDeltaSum = rightWeightTimesdIR - rightPixelDeltaSum - *(sumDeltaPtr++);
    
    rightPixelSumWeights = rightWeight; //next pixel to the right
    rightPixelDeltaSum = rightWeightTimesdIR;
    
    mySignIfNeg = SIGN_IF_NEG(currentPixelDeltaSum);
    mySign = 1 | mySignIfNeg;

    *(blockCurrentPixelPtr++) = centerPixel + mySign*((((mySign*currentPixelDeltaSum + ((currentPixelSumWeights+mySignIfNeg) >> 1))*divToMulOneOverN[currentPixelSumWeights]) >> (BITS_PER_DIV_LUT_ENTRY + divToMulShift[currentPixelSumWeights])));
    
  }
  
  //                            x
  // I pixel. uses filter type xx
  //
  // Uses information from previous row
  // Uses information from previous pixel
  // No information to store to next row
  // No information to store to nex pixel
  
  centerPixel = rightPixel;
  
  currentPixelSumWeights = centerWeight + *(sumWeightsPtr) + rightPixelSumWeights;
  currentPixelDeltaSum = - rightPixelDeltaSum - *(sumDeltaPtr);
  
  mySignIfNeg = SIGN_IF_NEG(currentPixelDeltaSum);
  mySign = 1 | mySignIfNeg;

  *(blockCurrentPixelPtr) = centerPixel + mySign*((((mySign*currentPixelDeltaSum + ((currentPixelSumWeights+mySignIfNeg) >> 1))*divToMulOneOverN[currentPixelSumWeights]) >> (BITS_PER_DIV_LUT_ENTRY + divToMulShift[currentPixelSumWeights])));
  
}

Void TComBilateralFilter::bilateralFilterIntra(TComDataCU *pcCU, UInt uiWidth, UInt uiHeight, Pel *piReco, UInt uiStride, Int qp)
{
  UInt uiMinSize = std::min(uiWidth, uiHeight);
  Short *tempblock = new Short[ uiWidth * uiHeight ];
  
  for (UInt j = 0; j < uiHeight; j++)   
  {
    memcpy(tempblock + j * uiWidth, piReco + j * uiStride, uiWidth * sizeof(Short));
  }
  smoothBlockBilateralFilter(pcCU, uiWidth, uiHeight, tempblock, uiMinSize, 0, qp);
  for (UInt j = 0; j < uiHeight; j++)
  {
    memcpy(piReco + j * uiStride, tempblock + j * uiWidth, uiWidth * sizeof(Short));
  }
  delete[] tempblock;
}

Void TComBilateralFilter::bilateralFilterInter(TComDataCU *pcCU, UInt uiWidth, UInt uiHeight, Pel *piResi, UInt uiStrideRes, Pel *piPred, UInt uiPredStride, Pel *piReco, UInt uiRecStride, Int clipbd, Int qp)
{
  UInt uiMinSize = std::min(uiWidth, uiHeight);
  Short *tempblock = new Short[ uiWidth * uiHeight ];
  Pel *piPredTemp = piPred;
  Pel *piResiTemp = piResi;
  Pel *piRecoTemp = piReco;
  // Reco = Pred + Resi
  for (UInt uiY = 0; uiY < uiHeight; ++uiY)
  {
    for (UInt uiX = 0; uiX < uiWidth; ++uiX)
    {
#if JVET_D0033_ADAPTIVE_CLIPPING
      piReco[uiX] = ClipA(piPred[uiX] + piResi[uiX], COMPONENT_Y);
#else
      piReco[uiX] = ClipBD(piPred[uiX] + piResi[uiX], clipbd);
#endif
    }
    piPred += uiPredStride;
    piResi += uiStrideRes;
    piReco += uiRecStride;
  }

  piPred = piPredTemp;
  piResi = piResiTemp;
  piReco = piRecoTemp;

  // Reco' = filter(Reco)
  for (UInt j = 0; j < uiHeight; j++)
  {
    memcpy(tempblock + j * uiWidth, piReco + j * uiRecStride, uiWidth * sizeof(Short));
  }
  smoothBlockBilateralFilter(pcCU, uiWidth, uiHeight, tempblock, uiMinSize, 1, qp);
  for (UInt j = 0; j < uiHeight; j++)
  {
    memcpy(piReco + j * uiRecStride, tempblock + j * uiWidth, uiWidth * sizeof(Short));
  }
  delete[] tempblock;

  // need to be performed if residual  is used
  // Resi' = Reco' - Pred
  for (UInt uiY = 0; uiY < uiHeight; ++uiY)
  {
    for (UInt uiX = 0; uiX < uiWidth; ++uiX)
    {
      piResi[uiX] = piReco[uiX] - piPred[uiX];
    }
    piPred += uiPredStride;
    piResi += uiStrideRes;
    piReco += uiRecStride;
  }
}

