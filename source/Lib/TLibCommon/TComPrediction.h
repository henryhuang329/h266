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

/** \file     TComPrediction.h
    \brief    prediction class (header)
*/

#ifndef __TCOMPREDICTION__
#define __TCOMPREDICTION__


// Include files
#include "TComYuv.h"
#include "TComInterpolationFilter.h"
#include "TComWeightPrediction.h"
#if VCEG_AZ07_FRUC_MERGE
#include "TComRdCost.h"
#include <list>
#endif
#if COM16_C1046_PDPC_INTRA
#include "TComRom.h"
#endif
#include "TComPic.h"

// forward declaration
class TComMv;
class TComTU; 
#if VCEG_AZ07_FRUC_MERGE || JVET_C0024_QTBT
class TComMvField;
#endif

//! \ingroup TLibCommon
//! \{

// ====================================================================================================================
// Class definition
// ====================================================================================================================

/// prediction class
typedef enum PRED_BUF_E
{
  PRED_BUF_UNFILTERED=0,
  PRED_BUF_FILTERED=1,
  NUM_PRED_BUF=2
} PRED_BUF;

#if JVET_C0024_QTBT
static const UInt MAX_INTRA_FILTER_DEPTHS=MAX_CU_DEPTH;
#else
#if COM16_C806_LARGE_CTU
static const UInt MAX_INTRA_FILTER_DEPTHS=MAX_CU_DEPTH-1;
#else
static const UInt MAX_INTRA_FILTER_DEPTHS=5;
#endif
#endif

class TComPrediction : public TComWeightPrediction
{
private:
  static const UChar m_aucIntraFilter[MAX_NUM_CHANNEL_TYPE][MAX_INTRA_FILTER_DEPTHS];
#if COM16_C806_VCEG_AZ10_SUB_PU_TMVP
  UInt*  m_puiW;
  UInt*  m_puiH;
  UInt*  m_puiSPAddr;
#endif

protected:
#if VCEG_AZ05_BIO  
  Pel*   m_pGradX0;
  Pel*   m_pGradY0;
  Pel*   m_pGradX1;
  Pel*   m_pGradY1;
#if !JVET_F0028_BIO_NO_BLOCK_EXTENTION
  Pel*   m_pPred0 ;
  Pel*   m_pPred1 ;
#endif
  Int    iRefListIdx;
#endif

#if COM16_C1046_PDPC_INTRA
  Int* piTempRef;
  Int* piFiltRef;
  Int* piBinBuff;
#endif
  Pel*      m_piYuvExt[MAX_NUM_COMPONENT][NUM_PRED_BUF];
  Int       m_iYuvExtSize;

  TComYuv   m_acYuvPred[NUM_REF_PIC_LIST_01];
  TComYuv   m_cYuvPredTemp;
  TComYuv m_filteredBlock[LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS][LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS];
  TComYuv m_filteredBlockTmp[LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS];

  TComInterpolationFilter m_if;

  Pel*   m_pLumaRecBuffer;       ///< array for downsampled reconstructed luma sample
  Int    m_iLumaRecStride;       ///< stride of #m_pLumaRecBuffer array

#if JVET_E0077_LM_MF
  Pel*   m_pLumaRecBufferMul[LM_FILTER_NUM];
#endif

#if JVET_E0077_ENHANCED_LM
  Int  m_iCurAngMode;
#endif

#if COM16_C806_LMCHROMA
  UInt m_uiaLMShift[ 32 ];       // Table for multiplication to substitue of division operation
#endif

#if VCEG_AZ06_IC
  UInt   m_uiaICShift[ 64 ];     // Table for multiplication to substitue of division operation
  static const Int m_ICRegCostShift = 7;
  static const Int m_ICConstShift = 5;
  static const Int m_ICShiftDiff = 12;
#endif

#if JVET_G0082
  UInt   m_uiaBIOShift[64];
#endif

#if VCEG_AZ08_INTER_KLT
  TComPicYuv* m_tempPicYuv;
#endif

    Void xPredIntraAng            ( Int bitDepth, const Pel* pSrc, Int srcStride, Pel* pDst, Int dstStride, UInt width, UInt height,
#if JVET_D0033_ADAPTIVE_CLIPPING
                                    ComponentID compID,
#else
                                    ChannelType channelType,
                                #endif
                                    UInt dirMode, const Bool bEnableEdgeFilters
#if VCEG_AZ07_INTRA_4TAP_FILTER
    , Bool enable4TapFilter = false
#endif
#if COM16_C983_RSAF_PREVENT_OVERSMOOTHING
    , Bool enableRSAF = false 
#endif
    );
  Void xPredIntraPlanar         ( const Pel* pSrc, Int srcStride, Pel* rpDst, Int dstStride, UInt width, UInt height );

#if VCEG_AZ07_FRUC_MERGE
  TComRdCost              m_cFRUCRDCost;
  std::list <TComMvField> m_listMVFieldCand[2];
#if JVET_E0060_FRUC_CAND
  RefPicList              m_bilatBestRefPicList;
#endif
  TComYuv                 m_cYuvPredFrucTemplate[2];      // 0: top, 1: left
  Bool                    m_bFrucTemplateAvailabe[2];
#if COM16_C806_VCEG_AZ10_SUB_PU_TMVP
  UChar                   m_eMergeCandTypeNieghors[MRG_MAX_NUM_CANDS];
#if JVET_C0035_ATMVP_SIMPLIFICATION
  TComMvField *           m_cMvFieldSP[NUM_MGR_TYPE];
  UChar *                 m_uhInterDirSP[NUM_MGR_TYPE];
#else
  TComMvField *           m_cMvFieldSP[2];
  UChar *                 m_uhInterDirSP[2];
#endif
#endif
#endif

  // motion compensation functions
#if VCEG_AZ05_BIO  
#define BIO_FILTER_LENGTH                 6
#define BIO_FILTER_LENGTH_MINUS_1         (BIO_FILTER_LENGTH-1)
#define BIO_FILTER_HALF_LENGTH_MINUS_1    ((BIO_FILTER_LENGTH>>1)-1)
#if !JVET_F0028_BIO_NO_BLOCK_EXTENTION
  Void  xPredInterFrac(Pel* ref,Pel* dst,Int dstStride,Int refStride,Int xFrac,Int yFrac,Int width, Int height,Bool bi,ChromaFormat chFmt, const Int bitDepth);
#endif
  Void  xGradFilterX(Pel*  piRefY, Int iRefStride,Pel*  piDstY,Int iDstStride, Int iWidth, Int iHeight,Int iMVyFrac,Int iMVxFrac, const Int bitDepth);
  Void  xGradFilterY(Pel*  piRefY, Int iRefStride,Pel*  piDstY,Int iDstStride, Int iWidth, Int iHeight,Int iMVyFrac,Int iMVxFrac, const Int bitDepth);
  __inline Void gradFilter2DVer (Pel* piSrc, Int iSrcStride,  Int iWidth, Int iHeight, Int iDstStride,  Pel*& rpiDst, Int iMv, const Int iShift);
  __inline Void gradFilter2DHor (Pel* piSrc, Int iSrcStride,  Int iWidth, Int iHeight, Int iDstStride,  Pel*& rpiDst, Int iMV, const Int iShift);
  __inline Void fracFilter2DHor(Pel* piSrc, Int iSrcStride,  Int iWidth, Int iHeight, Int iDstStride,  Pel*& rpiDst, Int iMV, const Int iShift);
  __inline Void fracFilter2DVer(Pel* piSrc, Int iSrcStride,  Int iWidth, Int iHeight, Int iDstStride,  Pel*& rpiDst, Int iMv, const Int iShift);
  __inline Void gradFilter1DHor (Pel* piSrc, Int iSrcStride,  Int iWidth, Int iHeight, Int iDstStride,  Pel*& rpiDst, Int iMV, const Int iShift);
  __inline Void gradFilter1DVer (Pel* piSrc, Int iSrcStride,  Int iWidth, Int iHeight, Int iDstStride,  Pel*& rpiDst, Int iMV, const Int iShift);

#if JVET_G0082
  __inline Int64 divide64(Int64 numer, Int64 denom);
  __inline Void calcBlkGradient(Int sx, Int sy, Int64 *arraysGx2, Int64 *arraysGxGy, Int64 *arraysGxdI, Int64 *arraysGy2, Int64 *arraysGydI,
    Int64 &sGx2, Int64 &sGy2, Int64 &sGxGy, Int64 &sGxdI, Int64 &sGydI, Int iWidth, Int iHeight);
#endif
#endif
  Void xPredInterUni            ( TComDataCU* pcCU,                          UInt uiPartAddr,               Int iWidth, Int iHeight, RefPicList eRefPicList, TComYuv* pcYuvPred
#if JVET_E0052_DMVR
    , Bool bRefineflag
#endif
#if VCEG_AZ05_BIO
    , Bool bBIOApplied =false
#endif
    , Bool bi=false          
#if VCEG_AZ07_FRUC_MERGE
    , Bool bOBMC = false
#endif
    );
  Void xPredInterBi             ( TComDataCU* pcCU,                          UInt uiPartAddr,               Int iWidth, Int iHeight,                         TComYuv* pcYuvPred          
#if JVET_E0052_DMVR    
    , Bool bRefineflag
#endif
#if VCEG_AZ07_FRUC_MERGE || JVET_G0082
    , Bool bOBMC = false
#endif
    );
  Void xPredInterBlk(const ComponentID compID, TComDataCU *cu, TComPicYuv *refPic, UInt partAddr, TComMv *mv, Int width, Int height, TComYuv *dstPic, Bool bi, const Int bitDepth
#if VCEG_AZ05_BIO                  
    , Bool bBIOapplied =false
#endif
#if VCEG_AZ07_FRUC_MERGE
    , Int nFRUCMode = FRUC_MERGE_OFF
#endif
#if VCEG_AZ06_IC
    , Bool bICFlag      = false
#endif
    );
#if JVET_E0052_DMVR
  Void xBIPMVRefine(TComDataCU* pcCU, UInt uiAbsPartIdx, RefPicList eRefPicList, Int iWidth, Int iHeight, TComYuv* pOrgYuv, TComYuv* pDstYuv, UInt uiMaxSearchRounds, UInt nSearchStepShift, UInt& uiMinCost);
  UInt xDirectMCCost(Int iBitDepth, Pel* pRef, UInt uiRefStride, Pel* pOrg, UInt uiOrgStride, Int iWidth, Int iHeight);
  Void xPredInterLines(TComDataCU *cu, TComPicYuv *refPic, UInt partAddr, TComMv *mv, Int width, Int height, Pel* dstPix, Int dstStride, Bool bi, const Int bitDepth);
  Void xFillPredBorder(TComDataCU* pcCU, UInt uiAbsPartIdx, RefPicList eRefPicList, Int iWidth, Int iHeight, TComYuv* pDstYuv);
#if DMVR_HALF_ME
  Void xGenerateFracPixel(TComDataCU* pcCU, UInt uiAbsPartIdx, RefPicList eRefPicList, Int iWidth, Int iHeight, UInt nSearchStepShift);
#endif
#endif
  Void xWeightedAverage         ( TComYuv* pcYuvSrc0, TComYuv* pcYuvSrc1, Int iRefIdx0, Int iRefIdx1, UInt uiPartAddr, Int iWidth, Int iHeight, TComYuv* pcYuvDst, const BitDepths &clipBitDepths  
#if VCEG_AZ05_BIO                  
    , Bool bBIOapplied 
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
    );

  Void xGetLLSPrediction ( const Pel* pSrc0, Int iSrcStride, Pel* pDst0, Int iDstStride, UInt uiWidth, UInt uiHeight, UInt uiExt0, const ChromaFormat chFmt  DEBUG_STRING_FN_DECLARE(sDebug) );
#if COM16_C1046_PDPC_INTRA
  Void xReferenceFilter  (Int iBlkSize, Int iOrigWeight, Int iFilterOrder, Int * piRefVector, Int * piLowPassRef);
#endif
  Void xDCPredFiltering( const Pel* pSrc, Int iSrcStride, Pel* pDst, Int iDstStride, Int iWidth, Int iHeight, ChannelType channelType );

#if VCEG_AZ05_INTRA_MPI
  Void xMPIredFiltering(Pel* pSrc, Int iSrcStride, Pel*& rpDst, Int iDstStride, Int iWidth, Int iHeight, Int idxMPI);
#endif

#if VCEG_AZ07_INTRA_BOUNDARY_FILTER
  Void xIntraPredFilteringModeDGL( const Pel* pSrc, Int iSrcStride, Pel*& rpDst, Int iDstStride, Int iWidth, Int iHeight, UInt uiMode );
  Void xIntraPredFilteringMode34 ( const Pel* pSrc, Int iSrcStride, Pel*& rpDst, Int iDstStride, Int iWidth, Int iHeight);
  Void xIntraPredFilteringMode02 ( const Pel* pSrc, Int iSrcStride, Pel*& rpDst, Int iDstStride, Int iWidth, Int iHeight);
#endif
  Bool xCheckIdenticalMotion    ( TComDataCU* pcCU, UInt PartAddr);
  Void destroy();

#if COM16_C806_VCEG_AZ10_SUB_PU_TMVP
  Bool xCheckTwoSPMotion ( TComDataCU* pcCU, UInt PartAddr0, UInt PartAddr1 );
  Void xGetSubPUAddrAndMerge(TComDataCU* pcCU, UInt uiPartAddr, Int iSPWidth, Int iSPHeight, Int iNumSPInOneLine, Int iNumSP, UInt* uiMergedSPW, UInt* uiMergedSPH, UInt* uiSPAddr );
#endif
#if COM16_C806_OBMC
  Void xSubblockOBMC ( const ComponentID eComp, TComDataCU* pcCU, Int uiAbsPartIdx, TComYuv* pcYuvPredDst, TComYuv* pcYuvPredSrc, Int iWidth, Int iHeight, Int iDir, Bool bOBMCSimp );
  Void xSubtractOBMC ( TComDataCU* pcCU, Int uiAbsPartIdx, TComYuv* pcYuvPredDst, TComYuv* pcYuvPredSrc, Int iWidth, Int iHeight, Int iDir, Bool bOBMCSimp );
  Void xSubBlockMotionCompensation ( TComDataCU* pcCU, TComYuv* pcYuvPred, Int uiPartAddr, Int iWidth, Int iHeight  
#if JVET_E0052_DMVR
    , Bool bRefineflag
#endif
    );
#endif
#if VCEG_AZ07_FRUC_MERGE
  Bool xFrucFindBlkMv( TComDataCU * pCU , UInt uiPUIdx );
  Bool xFrucFindBlkMv4Pred( TComDataCU * pCU , UInt uiPUIdx , RefPicList eTargetRefPicList , Int nTargetRefIdx
#if JVET_E0060_FRUC_CAND
                          , AMVPInfo* pInfo = NULL
#endif
                          );
  Bool xFrucRefineSubBlkMv( TComDataCU * pCU , UInt uiDepth , UInt uiPUIdx , Bool bTM );

  Void xFrucCollectBlkStartMv( TComDataCU * pCU , UInt uiPUIdx , RefPicList eTargetRefList = REF_PIC_LIST_0 , Int nTargetRefIdx = -1
#if JVET_E0060_FRUC_CAND
                             , AMVPInfo* pInfo = NULL
#endif
                             );
  Void xFrucCollectSubBlkStartMv( TComDataCU * pCU , UInt uiAbsPartIdx , RefPicList eRefPicList , const TComMvField & rMvStart , Int nSubBlkWidth , Int nSubBlkHeight 
#if COM16_C806_VCEG_AZ10_SUB_PU_TMVP
    , UInt uiSubBlkRasterIdx , UInt uiSubBlkRasterStep
#endif
#if FRUC_FIX
                              , UInt numPartPerLine, UInt uiFidx
#endif
    );

  UInt xFrucFindBestMvFromList( TComMvField * pBestMvField , RefPicList & rBestRefPicList , TComDataCU * pCU , UInt uiAbsPartIdx , const TComMvField & rMvStart , Int nBlkWidth , Int nBlkHeight , Bool bTM , Bool bMvCost );

  UInt xFrucRefineMv( TComMvField * pBestMvField , RefPicList eCurRefPicList , UInt uiMinCost , Int nSearchMethod , TComDataCU * pCU , UInt uiAbsPartIdx , const TComMvField & rMvStart , Int nBlkWidth , Int nBlkHeight , Bool bTM
#if JVET_F0032_UNI_BI_SELECTION
      , Bool bMvCostZero = false
#endif
  );
  template<Int SearchPattern>
  UInt xFrucRefineMvSearch( TComMvField * pBestMvField , RefPicList eCurRefPicList , TComDataCU * pCU , UInt uiAbsPartIdx , TComMvField const & rMvStart , Int nBlkWidth , Int nBlkHeight , UInt uiMinDist , Bool bTM , Int nSearchStepShift , UInt uiMaxSearchRounds = MAX_UINT 
#if JVET_F0032_UNI_BI_SELECTION
      , Bool bMvCostZero = false
#endif
  );

  UInt xFrucGetMvCost( const TComMv & rMvStart , const TComMv & rMvCur , Int nSearchRange , Int nWeighting );
  UInt xFrucGetBilaMatchCost( TComDataCU * pcCU , UInt uiAbsPartIdx , Int nWidth , Int nHeight , RefPicList eCurRefPicList , const TComMvField & rCurMvField , TComMvField & rPairMVField , UInt uiMVCost );
  UInt xFrucGetTempMatchCost( TComDataCU * pcCU , UInt uiAbsPartIdx , Int nWidth , Int nHeight , RefPicList eCurRefPicList , const TComMvField & rCurMvField , UInt uiMVCost );
#if JVET_F0032_UNI_BI_SELECTION
  Void xFrucUpdateTemplate(TComDataCU * pcCU, UInt uiAbsPartIdx, Int nWidth, Int nHeight, RefPicList eCurRefPicList, TComMvField rCurMvField);
#endif

  Void xFrucInsertMv2StartList( const TComMvField & rMvField , std::list<TComMvField> & rList );
  Bool xFrucIsInList( const TComMvField & rMvField , std::list<TComMvField> & rList );

  Bool xFrucGetCurBlkTemplate( TComDataCU * pCU , UInt uiAbsPartIdx , Int nCurBlkWidth , Int nCurBlkHeight );
  Bool xFrucIsTopTempAvailable( TComDataCU * pCU , UInt uiAbsPartIdx );
  Bool xFrucIsLeftTempAvailable( TComDataCU * pCU , UInt uiAbsPartIdx );
  Int  xFrucGetSubBlkSize( TComDataCU * pcCU , UInt uiAbsPartIdx , Int nBlkWidth , Int nBlkHeight );
#endif
#if VCEG_AZ06_IC
  Void xGetLLSICPrediction( TComDataCU* pcCU, TComMv *pMv, TComPicYuv *pRefPic, Int &a, Int &b, const ComponentID eComp, Int nBitDepth );
#endif
public:
  TComPrediction();
  virtual ~TComPrediction();
#if COM16_C806_OBMC
  Void subBlockOBMC ( TComDataCU*  pcCU, UInt uiAbsPartIdx, TComYuv *pcYuvPred, TComYuv *pcYuvTmpPred1, TComYuv *pcYuvTmpPred2
#if JVET_E0052_DMVR
    , Bool bRefineflag = true
#endif
    , Bool bOBMC4ME = false );
#endif
#if COM16_C806_LMCHROMA
  Void    initTempBuff(ChromaFormat chromaFormatIDC, Int bitDepthY
#if VCEG_AZ08_INTER_KLT
    , bool interKLT , const Int iPicWidth, const Int iPicHeight, const UInt uiMaxCUWidth, const UInt uiMaxCUHeight, const UInt uiMaxCUDepth
#endif
    );
#else
  Void    initTempBuff(ChromaFormat chromaFormatIDC
#if VCEG_AZ08_INTER_KLT
    , bool interKLT , const Int iPicWidth, const Int iPicHeight, const UInt uiMaxCUWidth, const UInt uiMaxCUHeight, const UInt uiMaxCUDepth
#endif
    );
#endif

  ChromaFormat getChromaFormat() const { return m_cYuvPredTemp.getChromaFormat(); }

  // inter
  Void motionCompensation         ( TComDataCU*  pcCU, TComYuv* pcYuvPred
#if JVET_E0052_DMVR
    , Bool bRefineflag = true
#endif
    , RefPicList eRefPicList = REF_PIC_LIST_X, Int iPartIdx = -1 );

#if VCEG_AZ07_FRUC_MERGE
  Bool deriveFRUCMV( TComDataCU * pCU , UInt uiDepth , UInt uiAbsPartIdx , UInt uiPUIdx , Int nTargetRefIdx = -1 , RefPicList eTargetRefList = REF_PIC_LIST_0
#if JVET_E0060_FRUC_CAND
                   , AMVPInfo* pInfo = NULL
#endif
                   );
#endif

  // motion vector prediction
  Void getMvPredAMVP              ( TComDataCU* pcCU, UInt uiPartIdx, UInt uiPartAddr, RefPicList eRefPicList, TComMv& rcMvPred );

  // Angular Intra
  Void predIntraAng               ( const ComponentID compID, UInt uiDirMode, Pel *piOrg /* Will be null for decoding */, UInt uiOrgStride, Pel* piPred, UInt uiStride, TComTU &rTu, const Bool bUseFilteredPredSamples, const Bool bUseLosslessDPCM = false );

#if JVET_E0077_MMLM
  struct MMLM_parameter
  {
      Int Inf;  // Inferio boundary
      Int Sup;  // Superior bounday
      Int a;
      Int b;
      Int shift;
  };
  Int xCalcLMParametersGeneralized(Int x, Int y, Int xx, Int xy, Int iCountShift, Int bitDepth, Int &a, Int &b, Int &iShift);
  Int xLMSampleClassifiedTraining(Int count, Int LumaSamples[], Int ChrmSamples[], Int GroupNum, Int bitDepth, MMLM_parameter parameters[]);
  Int xGetMMLMParameters(TComTU& rTu, const ComponentID compID, UInt uiWidth, UInt uiHeight, Int &numClass, MMLM_parameter parameters[]);
#endif



#if COM16_C806_LMCHROMA
#if JVET_E0077_LM_MF
  Void xFilterGroup(Pel* pMulDst[LM_FILTER_NUM], Int i, Pel* piSrc, Int iRecStride, Bool bAboveAvaillable, Bool bLeftAvaillable);
#endif

  Void predLMIntraChroma(TComTU& rTu, ComponentID compID, Pel* pPred, UInt uiPredStride, UInt uiCWidth, UInt uiCHeight
#if JVET_E0077_ENHANCED_LM
      , Int LMtype = LM_CHROMA_IDX
#endif
      );
  Void getLumaRecPixels  ( TComTU& rTu, UInt uiCWidth, UInt uiCHeight );
  Void addCrossColorResi ( TComTU& rTu, ComponentID compID, Pel* piPred, UInt uiPredStride, UInt uiWidth, UInt uiHeight, Pel* piResi, UInt uiResiStride );
  Void xGetLMParameters  ( TComTU& rTu,  ComponentID compID, UInt uiWidth, UInt uiHeight, Int iPredType, Int &a, Int &b, Int &iShift );
  Void xCalcLMParameters ( Int x, Int y, Int xx, Int xy, Int iCountShift, Int iPredType, Int bitDepth, Int &a, Int &b, Int &iShift );
#endif

#if COM16_C1016_AFFINE
  Bool xCheckIdenticalAffineMotion ( TComDataCU* pcCU, UInt PartAddr, Int iWidth, Int iHeight );
  Void xPredAffineBlk              ( const ComponentID compID, TComDataCU *cu, TComPicYuv *refPic, UInt partAddr, TComMv acMv[3], Int width, Int height, TComYuv *dstPic, Bool bi, const Int bitDepth );

  Void getMvPredAffineAMVP         ( TComDataCU* pcCU, UInt uiPartIdx, UInt uiPartAddr, RefPicList eRefPicList, TComMv acMvPred[3] );
#endif

  Pel  predIntraGetPredValDC      ( const Pel* pSrc, Int iSrcStride, UInt iWidth, UInt iHeight);

  Pel*  getPredictorPtr           ( const ComponentID compID, const Bool bUseFilteredPredictions )
  {
    return m_piYuvExt[compID][bUseFilteredPredictions?PRED_BUF_FILTERED:PRED_BUF_UNFILTERED];
  }

  // This function is actually still in TComPattern.cpp
  /// set parameters from CU data for accessing intra data
  Void initIntraPatternChType ( TComTU &rTu,
                              const ComponentID compID, const Bool bFilterRefSamples
#if COM16_C983_RSAF
                              , Bool bRSAF = false
#endif
                              DEBUG_STRING_FN_DECLARE(sDebug)
                              );

  static Bool filteringIntraReferenceSamples(const ComponentID compID, UInt uiDirMode, UInt uiTuChWidth, UInt uiTuChHeight, const ChromaFormat chFmt, const Bool intraReferenceSmoothingDisabled
#if COM16_C983_RSAF_PREVENT_OVERSMOOTHING
                                            , Bool enableRSAF
#endif
                                            );

  static Bool UseDPCMForFirstPassIntraEstimation(TComTU &rTu, const UInt uiDirMode);
#if VCEG_AZ08_INTER_KLT
  Void interpolatePic(TComPic* pcPic);
#endif
};

//! \}

#endif // __TCOMPREDICTION__

