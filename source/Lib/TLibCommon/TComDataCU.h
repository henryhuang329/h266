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

/** \file     TComDataCU.h
    \brief    CU data structure (header)
    \todo     not all entities are documented
*/

#ifndef __TCOMDATACU__
#define __TCOMDATACU__

#include <algorithm>
#include <vector>

// Include files
#include "CommonDef.h"
#include "TComMotionInfo.h"
#include "TComSlice.h"
#include "TComRdCost.h"
#include "TComPattern.h"

//! \ingroup TLibCommon
//! \{

class TComTU; // forward declaration
#if VCEG_AZ07_FRUC_MERGE || JVET_C0024_QTBT
class TComPrediction;
#endif

static const UInt NUM_MOST_PROBABLE_MODES=
#if VCEG_AZ07_INTRA_65ANG_MODES
  6;
#else
  3;
#endif

// ====================================================================================================================
// Class definition
// ====================================================================================================================

/// CU data structure class
class TComDataCU
{
private:

  // -------------------------------------------------------------------------------------------------------------------
  // class pointers
  // -------------------------------------------------------------------------------------------------------------------

  TComPic*      m_pcPic;              ///< picture class pointer
  TComSlice*    m_pcSlice;            ///< slice header pointer

  // -------------------------------------------------------------------------------------------------------------------
  // CU description
  // -------------------------------------------------------------------------------------------------------------------

  UInt          m_ctuRsAddr;          ///< CTU (also known as LCU) address in a slice (Raster-scan address, as opposed to tile-scan/encoding order).
  UInt          m_absZIdxInCtu;       ///< absolute address in a CTU. It's Z scan order
  UInt          m_uiCUPelX;           ///< CU position in a pixel (X)
  UInt          m_uiCUPelY;           ///< CU position in a pixel (Y)
  UInt          m_uiNumPartition;     ///< total number of minimum partitions in a CU
#if JVET_C0024_QTBT
  UChar*        m_puhWidth[MAX_NUM_CHANNEL_TYPE];           ///< array of widths, for both luma and chroma
  UChar*        m_puhHeight[MAX_NUM_CHANNEL_TYPE];          ///< array of heights, for both luma and chroma
#else
#if COM16_C806_LARGE_CTU 
  UShort*       m_puhWidth;           ///< array of widths
  UShort*       m_puhHeight;          ///< array of heights
#else
  UChar*        m_puhWidth;           ///< array of widths
  UChar*        m_puhHeight;          ///< array of heights
#endif
#endif
#if JVET_C0024_QTBT  
  UChar*        m_puhDepth[MAX_NUM_CHANNEL_TYPE];           ///< array of QT depths, for both luma and chroma

  UChar*         m_puhBTSplitMode[MAX_NUM_CHANNEL_TYPE][2];     ///< [luma/chroma][0~3 BTDepth/4~7 BTDepth]: array of PU split mode for both luma and chroma: 0: no split; 1: hor split; 2: ver split. 
  //e.g., m_puhBTSplitMode[] = 6 = 00 01 10 means: noSplit, horSplit, verSplit for BT depth=2, 1, 0, respectively.
#else
  UChar*        m_puhDepth;           ///< array of depths
#endif
  Int           m_unitSize;           ///< size of a "minimum partition"
#if JVET_C0024_PBINTRA_FAST
  UInt          m_uiInterHAD;
#endif
#if JVET_C0024_ITSKIP
  UInt*          m_puiSkipWidth[MAX_NUM_COMPONENT]; //#of trailing 0 coeff in each row, only used for decoder
  UInt*          m_puiSkipHeight[MAX_NUM_COMPONENT]; //#of trailing 0 coeff in each column, only used for decoder
#endif
  // -------------------------------------------------------------------------------------------------------------------
  // CU data
  // -------------------------------------------------------------------------------------------------------------------

  Bool*          m_skipFlag;           ///< array of skip flags
#if VCEG_AZ05_INTRA_MPI
  Char*          m_MPIIdx;             ///< array of MPIIdxs
#endif
#if COM16_C1046_PDPC_INTRA && !JVET_G0104_PLANAR_PDPC
  Char*          m_PDPCIdx;             ///< array of PDPCIdxs
#endif
#if VCEG_AZ05_ROT_TR || COM16_C1044_NSST
#if JVET_C0024_QTBT
  Char*          m_ROTIdx[MAX_NUM_CHANNEL_TYPE]; // 0-> Luma, 1-> Chroma
#else
  Char*          m_ROTIdx;          ///< array of ROTIdxs; When NSST is enabled, the same member varaible of ROT is re-used
#endif
#endif
#if !JVET_C0024_QTBT
  Char*          m_pePartSize;         ///< array of partition sizes
#endif
  Char*          m_pePredMode;         ///< array of prediction modes
  Char*          m_crossComponentPredictionAlpha[MAX_NUM_COMPONENT]; ///< array of cross-component prediction alpha values
  Bool*          m_CUTransquantBypass;   ///< array of cu_transquant_bypass flags
#if JVET_C0024_DELTA_QP_FIX
  Char*          m_phQP[MAX_NUM_CHANNEL_TYPE]; ///< array of QP values       0-> Luma, 1-> Chroma
#else
  Char*          m_phQP;               ///< array of QP values
#endif
  UChar*         m_ChromaQpAdj;        ///< array of chroma QP adjustments (indexed). when value = 0, cu_chroma_qp_offset_flag=0; when value>0, indicates cu_chroma_qp_offset_flag=1 and cu_chroma_qp_offset_idx=value-1
  UInt           m_codedChromaQpAdj;
#if !JVET_C0024_QTBT
  UChar*         m_puhTrIdx;           ///< array of transform indices
#endif
  UChar*         m_puhTransformSkip[MAX_NUM_COMPONENT];///< array of transform skipping flags
#if VCEG_AZ08_KLT_COMMON
  UChar*         m_puhKLTFlag[MAX_NUM_COMPONENT];      ///< array of KLT flags
#endif
  UChar*         m_puhCbf[MAX_NUM_COMPONENT];          ///< array of coded block flags (CBF)
  TComCUMvField  m_acCUMvField[NUM_REF_PIC_LIST_01];    ///< array of motion vectors.
#if VCEG_AZ07_FRUC_MERGE
  TComCUMvField  m_acFRUCUniLateralMVField[2];
#endif
  TCoeff*        m_pcTrCoeff[MAX_NUM_COMPONENT];       ///< array of transform coefficient buffers (0->Y, 1->Cb, 2->Cr)
#if ADAPTIVE_QP_SELECTION
  TCoeff*        m_pcArlCoeff[MAX_NUM_COMPONENT];  // ARL coefficient buffer (0->Y, 1->Cb, 2->Cr)
  Bool           m_ArlCoeffIsAliasedAllocation;  ///< ARL coefficient buffer is an alias of the global buffer and must not be free()'d
#endif

  Pel*           m_pcIPCMSample[MAX_NUM_COMPONENT];    ///< PCM sample buffer (0->Y, 1->Cb, 2->Cr)

  // -------------------------------------------------------------------------------------------------------------------
  // neighbour access variables
  // -------------------------------------------------------------------------------------------------------------------

  TComDataCU*   m_pCtuAboveLeft;      ///< pointer of above-left CTU.
  TComDataCU*   m_pCtuAboveRight;     ///< pointer of above-right CTU.
  TComDataCU*   m_pCtuAbove;          ///< pointer of above CTU.
  TComDataCU*   m_pCtuLeft;           ///< pointer of left CTU
  TComDataCU*   m_apcCUColocated[NUM_REF_PIC_LIST_01];  ///< pointer of temporally colocated CU's for both directions
  TComMvField   m_cMvFieldA;          ///< motion vector of position A
  TComMvField   m_cMvFieldB;          ///< motion vector of position B
  TComMvField   m_cMvFieldC;          ///< motion vector of position C
  TComMv        m_cMvPred;            ///< motion vector predictor

#if JVET_C0024_BT_RMV_REDUNDANT
  UInt m_uiSplitConstrain;
#endif
  // -------------------------------------------------------------------------------------------------------------------
  // coding tool information
  // -------------------------------------------------------------------------------------------------------------------

  Bool*         m_pbMergeFlag;        ///< array of merge flags
  UChar*        m_puhMergeIndex;      ///< array of merge candidate indices
#if COM16_C806_VCEG_AZ10_SUB_PU_TMVP
  UChar*        m_peMergeType;        ///< array of merge Types flags to indicate whehter a block uses sub-PU TMVP
#endif
#if COM16_C806_OBMC
  Bool*         m_OBMCFlag;           ///< array of OBMC flags
#endif

#if COM16_C983_RSAF
  UChar*        m_puhIntraFiltFlag;
  Bool*        m_pbFiltFlagHidden;
#endif

#if VCEG_AZ07_FRUC_MERGE
  UChar*        m_puhFRUCMgrMode;
#endif
#if VCEG_AZ07_IMV
#if JVET_E0076_MULTI_PEL_MVD
  UChar*         m_iMVFlag;            ///< array of integer MV flags
#else
  Bool*         m_iMVFlag;            ///< array of integer MV flags
#endif
  Char*         m_piMVCandNum;        ///< encoder only array
#endif
#if VCEG_AZ06_IC
  Bool*         m_pbICFlag;           ///< array of IC flags
#endif
#if AMP_MRG && !JVET_C0024_QTBT
  Bool          m_bIsMergeAMP;
#endif
  UChar*        m_puhIntraDir[MAX_NUM_CHANNEL_TYPE]; // 0-> Luma, 1-> Chroma
  UChar*        m_puhInterDir;        ///< array of inter directions
  Char*         m_apiMVPIdx[NUM_REF_PIC_LIST_01];       ///< array of motion vector predictor candidates
  Char*         m_apiMVPNum[NUM_REF_PIC_LIST_01];       ///< array of number of possible motion vectors predictors
  Bool*         m_pbIPCMFlag;         ///< array of intra_pcm flags

#if ALF_HM3_REFACTOR
#if JVET_C0024_QTBT
  UChar*        m_puhAlfCtrlFlag;     ///< array of ALF flags
  UChar*        m_puhTmpAlfCtrlFlag;  ///< temporal array of ALF flags
#else
  UInt*         m_puiAlfCtrlFlag;     ///< array of ALF flags
  UInt*         m_puiTmpAlfCtrlFlag;  ///< temporal array of ALF flags
#endif
#endif

#if COM16_C806_EMT
  UChar*        m_puhEmtTuIdx;        ///< array of TU-level transform indexes
  UChar*        m_puhEmtCuFlag;       ///< array of CU-level flags enabling EMT
#endif

#if COM16_C1016_AFFINE
  Bool*         m_affineFlag;         ///< array of affine flags
#endif

  // -------------------------------------------------------------------------------------------------------------------
  // misc. variables
  // -------------------------------------------------------------------------------------------------------------------

  Bool          m_bDecSubCu;          ///< indicates decoder-mode
  Double        m_dTotalCost;         ///< sum of partition RD costs
  Distortion    m_uiTotalDistortion;  ///< sum of partition distortion
  UInt          m_uiTotalBits;        ///< sum of partition bits
  UInt          m_uiTotalBins;        ///< sum of partition bins
#if JVET_C0024_DELTA_QP_FIX
  UInt          m_uiQuPartIdx;
  Char          m_QuLastCodedQP;
  Char          m_codedQP[MAX_NUM_CHANNEL_TYPE];
#else
  Char          m_codedQP;
#endif
  UChar*        m_explicitRdpcmMode[MAX_NUM_COMPONENT]; ///< Stores the explicit RDPCM mode for all TUs belonging to this CU

protected:

  /// add possible motion vector predictor candidates
  Bool          xAddMVPCand           ( AMVPInfo* pInfo, RefPicList eRefPicList, Int iRefIdx, UInt uiPartUnitIdx, MVP_DIR eDir
#if COM16_C1016_AFFINE
    , bool bAffine=false
#endif
    );
  Bool          xAddMVPCandOrder      ( AMVPInfo* pInfo, RefPicList eRefPicList, Int iRefIdx, UInt uiPartUnitIdx, MVP_DIR eDir 
#if COM16_C1016_AFFINE
    , bool bAffine=false
#endif
    );

  Void          deriveRightBottomIdx        ( UInt uiPartIdx, UInt& ruiPartIdxRB );
#if VCEG_AZ06_IC
  Bool          xGetColMVP( RefPicList eRefPicList, Int ctuRsAddr, Int uiPartUnitIdx, TComMv& rcMv, Int& riRefIdx, Bool* bICFlag = NULL );
#else
  Bool          xGetColMVP( RefPicList eRefPicList, Int ctuRsAddr, Int uiPartUnitIdx, TComMv& rcMv, Int& riRefIdx );
#endif
  /// compute scaling factor from POC difference
  Int           xGetDistScaleFactor   ( Int iCurrPOC, Int iCurrRefPOC, Int iColPOC, Int iColRefPOC );

  Void xDeriveCenterIdx( UInt uiPartIdx, UInt& ruiPartIdxCenter );

public:
  TComDataCU();
  virtual ~TComDataCU();

  // -------------------------------------------------------------------------------------------------------------------
  // create / destroy / initialize / copy
  // -------------------------------------------------------------------------------------------------------------------

  Void          create                ( ChromaFormat chromaFormatIDC, UInt uiNumPartition, UInt uiWidth, UInt uiHeight, Bool bDecSubCu, Int unitSize
#if JVET_C0024_QTBT
    , UInt uiCUWidth, UInt uiCUHeight
#endif
#if ADAPTIVE_QP_SELECTION
    , TCoeff *pParentARLBuffer = 0
#endif
    );
  Void          destroy               ();

  Void          initCtu               ( TComPic* pcPic, UInt ctuRsAddr );
#if JVET_C0024_QTBT
#if JVET_C0024_BT_RMV_REDUNDANT
  Void          setSplitConstrain( UInt uiTag ){ m_uiSplitConstrain = uiTag; }
  UInt          getSplitConstrain(            ){ return m_uiSplitConstrain ; }
#endif
  Void          initSubBT             ( TComDataCU* pcCU, UInt uiPartUnitIdx, UInt uiCUDepth, UInt uiWidth, UInt uiHeight, UInt uiSplitMode, Int qp );
  Void          initEstData           ( const UInt uiDepth, const Int qp, const Bool bTransquantBypass, UInt uiWidth=0, UInt uiHeight=0, Int iBTSplitMode=-1 );
#else
  Void          initEstData           ( const UInt uiDepth, const Int qp, const Bool bTransquantBypass );
#endif
  Void          initSubCU             ( TComDataCU* pcCU, UInt uiPartUnitIdx, UInt uiDepth, Int qp );
  Void          setOutsideCUPart      ( UInt uiAbsPartIdx, UInt uiDepth );

  Void          copySubCU             ( TComDataCU* pcCU, UInt uiPartUnitIdx );
  Void          copyInterPredInfoFrom ( TComDataCU* pcCU, UInt uiAbsPartIdx, RefPicList eRefPicList );
#if JVET_C0024_QTBT 
  Void          copyPartFrom          ( TComDataCU* pcCU, UInt uiPartUnitIdx, UInt uiDepth, UInt uiWidth, UInt uiHeight );
  Void          copyToPic             ( UChar uiDepth, UInt uiWidth, UInt uiHeight );
#else
  Void          copyPartFrom          ( TComDataCU* pcCU, UInt uiPartUnitIdx, UInt uiDepth );
#endif
#if VCEG_AZ08_INTER_KLT 
  Void          copySameSizeCUFrom    ( TComDataCU* pcCU, UInt uiPartUnitIdx, UInt uiDepth);
#endif
#if !JVET_C0024_QTBT
  Void          copyToPic             ( UChar uiDepth );
#endif
  // -------------------------------------------------------------------------------------------------------------------
  // member functions for CU description
  // -------------------------------------------------------------------------------------------------------------------

  TComPic*        getPic              ()                        { return m_pcPic;           }
  const TComPic*  getPic              () const                  { return m_pcPic;           }
  TComSlice*       getSlice           ()                        { return m_pcSlice;         }
  const TComSlice* getSlice           () const                  { return m_pcSlice;         }
  UInt&         getCtuRsAddr          ()                        { return m_ctuRsAddr;       }
  UInt          getCtuRsAddr          () const                  { return m_ctuRsAddr;       }
  UInt          getZorderIdxInCtu     () const                  { return m_absZIdxInCtu;    }
  UInt          getCUPelX             () const                  { return m_uiCUPelX;        }
  UInt          getCUPelY             () const                  { return m_uiCUPelY;        }

#if JVET_C0024_QTBT
  ChannelType   getTextType           () const                  { return m_pcSlice->getTextType();}

  UChar*        getDepth              ()                        { return m_puhDepth[getTextType()];        }
  UChar         getDepth              ( UInt uiIdx ) const      { return m_puhDepth[getTextType()][uiIdx]; }
  Void          setDepth              ( UInt uiIdx, UChar  uh ) { m_puhDepth[getTextType()][uiIdx] = uh;   }
#else
  UChar*        getDepth              ()                        { return m_puhDepth;        }
  UChar         getDepth              ( UInt uiIdx ) const      { return m_puhDepth[uiIdx]; }
  Void          setDepth              ( UInt uiIdx, UChar  uh ) { m_puhDepth[uiIdx] = uh;   }
#endif

  Void          setDepthSubParts      ( UInt uiDepth, UInt uiAbsPartIdx );

  // -------------------------------------------------------------------------------------------------------------------
  // member functions for CU data
  // -------------------------------------------------------------------------------------------------------------------

#if JVET_C0024_QTBT
  UChar*        getBTSplitModePart (UInt uiBTDepthIdx) { return m_puhBTSplitMode[getTextType()][uiBTDepthIdx];}
  UChar         getBTSplitModePart ( UInt uiBTDepthIdx, UInt uiIdx) { return m_puhBTSplitMode[getTextType()][uiBTDepthIdx][uiIdx];}
  UInt          getBTSplitMode (UInt uiIdx) { return ((UInt)m_puhBTSplitMode[getTextType()][1][uiIdx]<<8) + m_puhBTSplitMode[getTextType()][0][uiIdx];}
  Void          setBTSplitModeSubParts ( UInt uiSplitMode, UInt uiAbsPartIdx, UInt uiWidth, UInt uiHeight);
  UInt          getBTSplitModeForBTDepth(UInt uiAbsPartIdx, UInt uiBTDepth);
  UInt          getBTDepth(UInt uiAbsPartIdx, UInt uiBTWidth, UInt uiBTHeight);
  UInt          getBTDepth(UInt uiAbsPartIdx);
#else
  Char*         getPartitionSize      ()                        { return m_pePartSize;        }
  PartSize      getPartitionSize      ( UInt uiIdx )            { return static_cast<PartSize>( m_pePartSize[uiIdx] ); }
  Void          setPartitionSize      ( UInt uiIdx, PartSize uh){ m_pePartSize[uiIdx] = uh;   }
  Void          setPartSizeSubParts   ( PartSize eMode, UInt uiAbsPartIdx, UInt uiDepth );
#endif
  Void          setCUTransquantBypassSubParts( Bool flag, UInt uiAbsPartIdx, UInt uiDepth );

  Bool*         getSkipFlag            ()                        { return m_skipFlag;          }
  Bool          getSkipFlag            (UInt idx)                { return m_skipFlag[idx];     }
  Void          setSkipFlag           ( UInt idx, Bool skip)     { m_skipFlag[idx] = skip;   }
  Void          setSkipFlagSubParts   ( Bool skip, UInt absPartIdx, UInt depth );

#if VCEG_AZ05_INTRA_MPI
  Char*         getMPIIdx               ()                        { return m_MPIIdx;          }
  Char          getMPIIdx               ( UInt idx)               { return m_MPIIdx[idx];     }
  Void          setMPIIdx               ( UInt idx, Char MPIIdx)  { m_MPIIdx[idx] = MPIIdx;   }
  Void          setMPIIdxSubParts       ( Char MPIIdx, UInt absPartIdx, UInt depth );
#endif

#if COM16_C1046_PDPC_INTRA && !JVET_G0104_PLANAR_PDPC
  Char*         getPDPCIdx               ()                          { return m_PDPCIdx; }
  Char          getPDPCIdx               (UInt idx)                  { return m_PDPCIdx[idx]; }
  Void          setPDPCIdx               (UInt idx, Char PDPCIdx)    { m_PDPCIdx[idx] = PDPCIdx; }
  Void          setPDPCIdxSubParts       (Char PDPCIdx, UInt absPartIdx, UInt depth);
#endif

#if VCEG_AZ05_ROT_TR || COM16_C1044_NSST
#if JVET_C0024_QTBT
  Char*         getROTIdx               ( const ChannelType channelType ) { return m_ROTIdx[channelType];      } // When NSST is enabled, the same interface functions of ROT is re-used
  Char          getROTIdx               ( const ChannelType channelType, UInt idx)               { return m_ROTIdx[channelType][idx];     }
  Void          setROTIdx               ( const ChannelType channelType, UInt idx, Char ROTIdx)  { m_ROTIdx[channelType][idx] = ROTIdx;   }
  Void          setROTIdxSubParts       ( const ChannelType channelType, Char ROTIdx, UInt absPartIdx, UInt depth );
#else
  Char*         getROTIdx               ()                        { return m_ROTIdx;          } // When NSST is enabled, the same interface functions of ROT is re-used
  Char          getROTIdx               ( UInt idx)               { return m_ROTIdx[idx];     }
  Void          setROTIdx               ( UInt idx, Char ROTIdx)  { m_ROTIdx[idx] = ROTIdx;   }
  Void          setROTIdxSubParts       ( Char ROTIdx, UInt absPartIdx, UInt depth );
#endif
#endif
  Char*         getPredictionMode     ()                        { return m_pePredMode;        }
  PredMode      getPredictionMode     ( UInt uiIdx )            { return static_cast<PredMode>( m_pePredMode[uiIdx] ); }
  Void          setPredictionMode     ( UInt uiIdx, PredMode uh){ m_pePredMode[uiIdx] = uh;   }
  Void          setPredModeSubParts   ( PredMode eMode, UInt uiAbsPartIdx, UInt uiDepth );

  Char*         getCrossComponentPredictionAlpha( ComponentID compID )             { return m_crossComponentPredictionAlpha[compID];         }
  Char          getCrossComponentPredictionAlpha( UInt uiIdx, ComponentID compID ) { return m_crossComponentPredictionAlpha[compID][uiIdx];  }

  Bool*         getCUTransquantBypass ()                        { return m_CUTransquantBypass;        }
  Bool          getCUTransquantBypass( UInt uiIdx )             { return m_CUTransquantBypass[uiIdx]; }

#if JVET_C0024_QTBT  
#if JVET_C0024_CTU_256
  UChar*        getWidth              ()                        { return m_puhWidth[getTextType()];          }
  UShort        getWidth              ( UInt uiIdx )            { return (UShort)m_puhWidth[getTextType()][uiIdx] << MIN_CU_LOG2;   }

  UChar*        getHeight             ()                        { return m_puhHeight[getTextType()];         }
  UShort        getHeight             ( UInt uiIdx )            { return (UShort)m_puhHeight[getTextType()][uiIdx] << MIN_CU_LOG2;  }
#else
  UChar*        getWidth              ()                        { return m_puhWidth[getTextType()];          }
  UChar         getWidth              ( UInt uiIdx )            { return m_puhWidth[getTextType()][uiIdx];   }

  UChar*        getHeight             ()                        { return m_puhHeight[getTextType()];         }
  UChar         getHeight             ( UInt uiIdx )            { return m_puhHeight[getTextType()][uiIdx];  }
#endif
#else
#if COM16_C806_LARGE_CTU
  UShort*       getWidth              ()                        { return m_puhWidth;          }
  UShort        getWidth              ( UInt uiIdx )            { return m_puhWidth[uiIdx];   }
  Void          setWidth              ( UInt uiIdx, UShort uh ) { m_puhWidth[uiIdx] = uh;     }

  UShort*       getHeight             ()                        { return m_puhHeight;         }
  UShort        getHeight             ( UInt uiIdx )            { return m_puhHeight[uiIdx];  }
  Void          setHeight             ( UInt uiIdx, UShort uh ) { m_puhHeight[uiIdx] = uh;    }
#else
  UChar*        getWidth              ()                        { return m_puhWidth;          }
  UChar         getWidth              ( UInt uiIdx )            { return m_puhWidth[uiIdx];   }
  Void          setWidth              ( UInt uiIdx, UChar  uh ) { m_puhWidth[uiIdx] = uh;     }

  UChar*        getHeight             ()                        { return m_puhHeight;         }
  UChar         getHeight             ( UInt uiIdx )            { return m_puhHeight[uiIdx];  }
  Void          setHeight             ( UInt uiIdx, UChar  uh ) { m_puhHeight[uiIdx] = uh;    }
#endif
#endif

  Void          setSizeSubParts       ( UInt uiWidth, UInt uiHeight, UInt uiAbsPartIdx, UInt uiDepth );

#if JVET_C0024_DELTA_QP_FIX
  Char*         getQP                 ( const ChannelType channelType ) { return m_phQP[channelType]; }
  Char*         getQP                 () { return m_phQP[getTextType()]; }
  Char          getQP                 ( UInt uiIdx ) const      
  { 
    if( !getSlice()->isIntra() )
    {
      assert( getTextType() == CHANNEL_TYPE_LUMA );
    }
    return m_phQP[getTextType()][uiIdx];       
  }
  //Void          setQP                 ( UInt uiIdx, Char value ){ m_phQP[getTextType()][uiIdx] =  value;     }
  Void          setQPSubParts         ( Int qp,   UInt uiAbsPartIdx, UInt uiWidth, UInt uiHeight );
#else
  Char*         getQP                 ()                        { return m_phQP;              }
  Char          getQP                 ( UInt uiIdx ) const      { return m_phQP[uiIdx];       }
  Void          setQP                 ( UInt uiIdx, Char value ){ m_phQP[uiIdx] =  value;     }
  Void          setQPSubParts         ( Int qp,   UInt uiAbsPartIdx, UInt uiDepth );
#endif
#if JVET_C0024_DELTA_QP_FIX
  Char          getCtuLastCodedQP     (  );
#else
  Int           getLastValidPartIdx   ( Int iAbsPartIdx );
  Char          getLastCodedQP        ( UInt uiAbsPartIdx );
#endif
#if JVET_C0024_DELTA_QP_FIX
  Void          setQPSubCUs           ( Int qp, UInt absPartIdx, UInt depth, UInt uiWidth, UInt uiHeight, UInt& ruiFirstNonZeroPartIdx, Bool &foundNonZeroCbf );
#else
  Void          setQPSubCUs           ( Int qp, UInt absPartIdx, UInt depth, Bool &foundNonZeroCbf );
#endif
#if JVET_C0024_DELTA_QP_FIX
  Void          setQuLastCodedQP      ( Char qp )               { m_QuLastCodedQP = qp;       }
  Char          getQuLastCodedQP      ()                        { return m_QuLastCodedQP;     }
  Void          setQuPartIdx          ( UInt uiPartIdx )        { m_uiQuPartIdx = uiPartIdx;  }
  UInt          getQuPartIdx          ()                        { return m_uiQuPartIdx;       }
  Void          setCodedQP            ( Char qp )               { m_codedQP[getTextType()] = qp;   }
  Char          getCodedQP            ()                        { return m_codedQP[getTextType()]; }
#else
  Void          setCodedQP            ( Char qp )               { m_codedQP = qp;             }
  Char          getCodedQP            ()                        { return m_codedQP;           }
#endif

  UChar*        getChromaQpAdj        ()                        { return m_ChromaQpAdj;       } ///< array of chroma QP adjustments (indexed). when value = 0, cu_chroma_qp_offset_flag=0; when value>0, indicates cu_chroma_qp_offset_flag=1 and cu_chroma_qp_offset_idx=value-1
  UChar         getChromaQpAdj        (Int idx)           const { return m_ChromaQpAdj[idx];  } ///< When value = 0, cu_chroma_qp_offset_flag=0; when value>0, indicates cu_chroma_qp_offset_flag=1 and cu_chroma_qp_offset_idx=value-1
  Void          setChromaQpAdj        (Int idx, UChar val)      { m_ChromaQpAdj[idx] = val;   } ///< When val = 0,   cu_chroma_qp_offset_flag=0; when val>0,   indicates cu_chroma_qp_offset_flag=1 and cu_chroma_qp_offset_idx=val-1
  Void          setChromaQpAdjSubParts( UChar val, Int absPartIdx, Int depth );
  Void          setCodedChromaQpAdj   ( Char qp )               { m_codedChromaQpAdj = qp;    }
  Char          getCodedChromaQpAdj   ()                        { return m_codedChromaQpAdj;  }

  Bool          isLosslessCoded       ( UInt absPartIdx );

#if !JVET_C0024_QTBT
  UChar*        getTransformIdx       ()                        { return m_puhTrIdx;          }
  UChar         getTransformIdx       ( UInt uiIdx )            { return m_puhTrIdx[uiIdx];   }
  Void          setTrIdxSubParts      ( UInt uiTrIdx, UInt uiAbsPartIdx, UInt uiDepth );
#endif

  UChar*        getTransformSkip      ( ComponentID compID )    { return m_puhTransformSkip[compID];}
  UChar         getTransformSkip      ( UInt uiIdx, ComponentID compID)    { return m_puhTransformSkip[compID][uiIdx];}
  Void          setTransformSkipSubParts  ( UInt useTransformSkip, ComponentID compID, UInt uiAbsPartIdx, UInt uiDepth);
  Void          setTransformSkipSubParts  ( const UInt useTransformSkip[MAX_NUM_COMPONENT], UInt uiAbsPartIdx, UInt uiDepth );

#if VCEG_AZ08_KLT_COMMON
  UChar*        getKLTFlag            ( ComponentID compID )    { return m_puhKLTFlag[compID]; }
  UChar         getKLTFlag            ( UInt uiIdx, ComponentID compID)    { return m_puhKLTFlag[compID][uiIdx]; }
  Void          setKLTFlagSubParts    ( UInt useKLT, ComponentID compID, UInt uiAbsPartIdx, UInt uiDepth);
  Void          setKLTFlagSubParts    (const UInt useKLT[MAX_NUM_COMPONENT], UInt uiAbsPartIdx, UInt uiDepth);
  Void          setKLTPartRange(UInt useKLTY, ComponentID compID, UInt uiAbsPartIdx, UInt uiCoveredPartIdxes);

#endif

  UChar*        getExplicitRdpcmMode      ( ComponentID component ) { return m_explicitRdpcmMode[component]; }
  UChar         getExplicitRdpcmMode      ( ComponentID component, UInt partIdx ) {return m_explicitRdpcmMode[component][partIdx]; }
  Void          setExplicitRdpcmModePartRange ( UInt rdpcmMode, ComponentID compID, UInt uiAbsPartIdx, UInt uiCoveredPartIdxes );

  Bool          isRDPCMEnabled         ( UInt uiAbsPartIdx )  { return getSlice()->getSPS()->getSpsRangeExtension().getRdpcmEnabledFlag(isIntra(uiAbsPartIdx) ? RDPCM_SIGNAL_IMPLICIT : RDPCM_SIGNAL_EXPLICIT); }

  Void          setCrossComponentPredictionAlphaPartRange    ( Char alphaValue, ComponentID compID, UInt uiAbsPartIdx, UInt uiCoveredPartIdxes );
  Void          setTransformSkipPartRange                    ( UInt useTransformSkip, ComponentID compID, UInt uiAbsPartIdx, UInt uiCoveredPartIdxes );
#if !JVET_C0024_QTBT

  UInt          getQuadtreeTULog2MinSizeInCU( UInt uiIdx );
#endif

  TComCUMvField* getCUMvField         ( RefPicList e )          { return  &m_acCUMvField[e];  }
#if VCEG_AZ07_FRUC_MERGE
  TComCUMvField* getFRUCUniLateralMVField        ( RefPicList e )          { return  &m_acFRUCUniLateralMVField[e]; }
  TComMv        scaleMV( const TComMv & rColMV , Int iCurrPOC, Int iCurrRefPOC, Int iColPOC, Int iColRefPOC );
  Bool          isSameMVField( RefPicList eListA , TComMvField & rMVFieldA , RefPicList eListB , TComMvField & rMVFieldB );
  Bool          getMvPair( RefPicList eCurRefPicList , const TComMvField & rCurMvField , TComMvField & rMvPair );
  Bool          getBlockBelowRight( UInt uiAbsPartIdx, Int nCurBlkWidth , Int nCurBlkHeight , UInt & rCUAddr , UInt & rBRAbsPartIdx );
#endif

  TCoeff*       getCoeff              (ComponentID component)   { return m_pcTrCoeff[component]; }

#if ADAPTIVE_QP_SELECTION
  TCoeff*       getArlCoeff           ( ComponentID component ) { return m_pcArlCoeff[component]; }
#endif
  Pel*          getPCMSample          ( ComponentID component ) { return m_pcIPCMSample[component]; }

  UChar         getCbf    ( UInt uiIdx, ComponentID eType )                  { return m_puhCbf[eType][uiIdx];  }
  UChar*        getCbf    ( ComponentID eType )                              { return m_puhCbf[eType];         }
  UChar         getCbf    ( UInt uiIdx, ComponentID eType, UInt uiTrDepth )  { return ( ( getCbf( uiIdx, eType ) >> uiTrDepth ) & 0x1 ); }
  Void          setCbf    ( UInt uiIdx, ComponentID eType, UChar uh )        { m_puhCbf[eType][uiIdx] = uh;    }
  Void          clearCbf  ( UInt uiIdx, ComponentID eType, UInt uiNumParts );
  UChar         getQtRootCbf          ( UInt uiIdx );

  Void          setCbfSubParts        ( const UInt uiCbf[MAX_NUM_COMPONENT],  UInt uiAbsPartIdx, UInt uiDepth           );
  Void          setCbfSubParts        ( UInt uiCbf, ComponentID compID, UInt uiAbsPartIdx, UInt uiDepth                    );
#if !JVET_C0024_QTBT
  Void          setCbfSubParts        ( UInt uiCbf, ComponentID compID, UInt uiAbsPartIdx, UInt uiPartIdx, UInt uiDepth    );
#endif
  Void          setCbfPartRange       ( UInt uiCbf, ComponentID compID, UInt uiAbsPartIdx, UInt uiCoveredPartIdxes      );
#if !JVET_C0024_QTBT
  Void          bitwiseOrCbfPartRange ( UInt uiCbf, ComponentID compID, UInt uiAbsPartIdx, UInt uiCoveredPartIdxes      );
#endif

  // -------------------------------------------------------------------------------------------------------------------
  // member functions for coding tool information
  // -------------------------------------------------------------------------------------------------------------------

  Bool*         getMergeFlag          ()                        { return m_pbMergeFlag;               }
  Bool          getMergeFlag          ( UInt uiIdx )            { return m_pbMergeFlag[uiIdx];        }
  Void          setMergeFlag          ( UInt uiIdx, Bool b )    { m_pbMergeFlag[uiIdx] = b;           }
  Void          setMergeFlagSubParts  ( Bool bMergeFlag, UInt uiAbsPartIdx, UInt uiPartIdx, UInt uiDepth );

  UChar*        getMergeIndex         ()                        { return m_puhMergeIndex;                         }
  UChar         getMergeIndex         ( UInt uiIdx )            { return m_puhMergeIndex[uiIdx];                  }
  Void          setMergeIndex         ( UInt uiIdx, UInt uiMergeIndex ) { m_puhMergeIndex[uiIdx] = uiMergeIndex;  }
  Void          setMergeIndexSubParts ( UInt uiMergeIndex, UInt uiAbsPartIdx, UInt uiPartIdx, UInt uiDepth );

#if COM16_C806_VCEG_AZ10_SUB_PU_TMVP
  UChar*        getMergeType         ()                         { return m_peMergeType;          }
  UChar         getMergeType         ( UInt uiIdx )             { return m_peMergeType[uiIdx];   }
  Void          setMergeType         ( UInt uiIdx, UChar e )    { m_peMergeType[uiIdx] = e;      }
  Void          setMergeTypeSubParts ( UChar eMergeType, UInt uiAbsPartIdx, UInt uiPartIdx, UInt uiDepth );
#endif
#if VCEG_AZ07_FRUC_MERGE
  UChar*        getFRUCMgrMode          ()                      { return m_puhFRUCMgrMode;               }
  UChar         getFRUCMgrMode          ( UInt uiIdx )          { return m_puhFRUCMgrMode[uiIdx];        }
  Void          setFRUCMgrMode          ( UInt uiIdx, UChar b ) { m_puhFRUCMgrMode[uiIdx] = b;           }
  Void          setFRUCMgrModeSubParts  ( UChar uhFRUCMgrMode, UInt uiAbsPartIdx, UInt uiPartIdx, UInt uiDepth );
#endif
#if VCEG_AZ07_IMV
#if JVET_E0076_MULTI_PEL_MVD
  UChar*         getiMVFlag            ()                        { return m_iMVFlag;          }
  UChar          getiMVFlag            (UInt idx)                { return m_iMVFlag[idx];     }
  Void          setiMVFlag            ( UInt idx, UChar iMV)     { m_iMVFlag[idx] = iMV;      }
  Void          setiMVFlagSubParts    ( UChar iMV, UInt absPartIdx, UInt depth );
#else
  Bool*         getiMVFlag            ()                        { return m_iMVFlag;          }
  Bool          getiMVFlag            (UInt idx)                { return m_iMVFlag[idx];     }
  Void          setiMVFlag            ( UInt idx, Bool iMV)     { m_iMVFlag[idx] = iMV;      }
  Void          setiMVFlagSubParts    ( Bool iMV, UInt absPartIdx, UInt depth );
#endif
  Char*         getiMVCandNum         ()                        { return m_piMVCandNum;          }
  Char          getiMVCandNum         (UInt idx)                { return m_piMVCandNum[idx];     }
  Void          setiMVCandNum         ( UInt idx, Char ciMVCandNum)     { m_piMVCandNum[idx] = ciMVCandNum;   }
  Void          setiMVCandNumSubParts ( Char ciMVCandNum, UInt absPartIdx, UInt depth );
#endif
#if COM16_C806_OBMC
  Bool*         getOBMCFlag          ()                        { return m_OBMCFlag;          }
  Bool          getOBMCFlag          (UInt idx)                { return m_OBMCFlag[idx];     }
  Void          setOBMCFlag          ( UInt idx, Bool OBMC )   { m_OBMCFlag[idx] = OBMC;     }
  Void          setOBMCFlagSubParts  ( Bool OBMC, UInt absPartIdx, UInt depth );
  Bool          isOBMCFlagCoded      ( UInt uiAbsPartIdx );
  Bool          getNeigMotion( UInt uiAbsPartIdx, TComMvField cNeigMvField[2], Int &irNeigPredDir, Int iDir, TComMvField cCurMvField[2], Int &iCurrDir, UInt uiZeroIdx, Bool &bTobeStored);
#endif
#if VCEG_AZ06_IC
  Bool*         getICFlag            ()                        { return m_pbICFlag;               }
  Bool          getICFlag            ( UInt uiIdx )            { return m_pbICFlag[uiIdx];        }
  Void          setICFlag            ( UInt uiIdx, Bool  uh )  { m_pbICFlag[uiIdx] = uh;          }
  Void          setICFlagSubParts    ( Bool bICFlag,  UInt uiAbsPartIdx, UInt uiDepth );
  Bool          isICFlagCoded        ( UInt uiAbsPartIdx );
#endif
  template <typename T>
#if JVET_C0024_QTBT
  Void          setSubPart            ( T bParameter, T* pbBaseCtu, UInt uiCUAddr, UInt uiWidth, UInt uiHeight );
#else
  Void          setSubPart            ( T bParameter, T* pbBaseCtu, UInt uiCUAddr, UInt uiCUDepth, UInt uiPUIdx );
#endif

#if AMP_MRG && !JVET_C0024_QTBT
  Void          setMergeAMP( Bool b )      { m_bIsMergeAMP = b; }
  Bool          getMergeAMP( )             { return m_bIsMergeAMP; }
#endif

  UChar*        getIntraDir         ( const ChannelType channelType )                   const { return m_puhIntraDir[channelType];         }
  UChar         getIntraDir         ( const ChannelType channelType, const UInt uiIdx ) const { return m_puhIntraDir[channelType][uiIdx];  }

  Void          setIntraDirSubParts ( const ChannelType channelType,
                                      const UInt uiDir,
                                      const UInt uiAbsPartIdx,
                                      const UInt uiDepth );

  UChar*        getInterDir           ()                        { return m_puhInterDir;               }
  UChar         getInterDir           ( UInt uiIdx )            { return m_puhInterDir[uiIdx];        }
  Void          setInterDir           ( UInt uiIdx, UChar  uh ) { m_puhInterDir[uiIdx] = uh;          }
  Void          setInterDirSubParts   ( UInt uiDir,  UInt uiAbsPartIdx, UInt uiPartIdx, UInt uiDepth );
  Bool*         getIPCMFlag           ()                        { return m_pbIPCMFlag;               }
  Bool          getIPCMFlag           (UInt uiIdx )             { return m_pbIPCMFlag[uiIdx];        }
  Void          setIPCMFlag           (UInt uiIdx, Bool b )     { m_pbIPCMFlag[uiIdx] = b;           }
  Void          setIPCMFlagSubParts   (Bool bIpcmFlag, UInt uiAbsPartIdx, UInt uiDepth);

#if COM16_C983_RSAF
  UChar*        getLumaIntraFilter    ()                        const { return m_puhIntraFiltFlag;         }
  UChar         getLumaIntraFilter    (UInt uiIdx)              const { return m_puhIntraFiltFlag[uiIdx];  }
  Void          setLumaIntraFilter    (UInt uiIdx, Bool value)  const { m_puhIntraFiltFlag[uiIdx] = value; }

  Bool*         isLumaIntraFilterHidden  ()                       const { return m_pbFiltFlagHidden;         }
  Bool          isLumaIntraFilterHidden  (UInt uiIdx)             const { return m_pbFiltFlagHidden[uiIdx];  }
  Void          setLumaIntraFilterHidden (UInt uiIdx, Bool value) const { m_pbFiltFlagHidden[uiIdx] = value; }
#endif

#if COM16_C1045_BIO_HARMO_IMPROV
  Bool          isBIOLDB( UInt uiAbsPartIdx );
#endif

#if ALF_HM3_REFACTOR
#if JVET_C0024_QTBT
  UChar*         getAlfCtrlFlag        ()                        { return m_puhAlfCtrlFlag;            }
  UChar          getAlfCtrlFlag        ( UInt uiIdx )            { return m_puhAlfCtrlFlag[uiIdx];     }
  Void          setAlfCtrlFlag        ( UInt uiIdx, UChar uhFlag){ m_puhAlfCtrlFlag[uiIdx] = uhFlag;   }
  Void          setAlfCtrlFlagSubParts( UChar uhFlag, UInt uiAbsPartIdx, UInt uiWidth, UInt uiHeight );
#else
  UInt*         getAlfCtrlFlag        ()                        { return m_puiAlfCtrlFlag;            }
  UInt          getAlfCtrlFlag        ( UInt uiIdx )            { return m_puiAlfCtrlFlag[uiIdx];     }
  Void          setAlfCtrlFlag        ( UInt uiIdx, UInt uiFlag){ m_puiAlfCtrlFlag[uiIdx] = uiFlag;   }
  Void          setAlfCtrlFlagSubParts( UInt uiFlag, UInt uiAbsPartIdx, UInt uiDepth );
#endif
  Void          createTmpAlfCtrlFlag  ();
  Void          destroyTmpAlfCtrlFlag ();
  Void          copyAlfCtrlFlagToTmp  ();
  Void          copyAlfCtrlFlagFromTmp();
  UInt          getCtxAlfCtrlFlag               ( UInt   uiAbsPartIdx                                 );
#endif

#if COM16_C806_EMT
  UChar*        getEmtTuIdx           ()                        { return m_puhEmtTuIdx;          }
  UChar         getEmtTuIdx           ( UInt uiIdx )            { return m_puhEmtTuIdx[uiIdx];   }
  Void          setEmtTuIdxSubParts   ( UInt uiTrMode, UInt uiAbsPartIdx, UInt uiDepth );
  Void          setEmtTuIdxPartsRange ( UInt uiTrMode, ComponentID compID, UInt uiAbsPartIdx, UInt uiCoveredPartIdxes );
  UChar*        getEmtCuFlag          ()                        { return m_puhEmtCuFlag;          }
  UChar         getEmtCuFlag          ( UInt uiIdx )            { return m_puhEmtCuFlag[uiIdx];   }
  Void          setEmtCuFlagSubParts  ( UInt uiTuOptTrFlag, UInt uiAbsPartIdx, UInt uiDepth );
#endif  

#if VCEG_AZ07_INTRA_65ANG_MODES
  TComDataCU*   getPULeftOffset             ( UInt& uiPartUnitIdx, 
                                              UInt uiCurrPartUnitIdx, 
                                              UInt uiPartOffset=0,
                                              Bool bEnforceSliceRestriction=true, 
                                              Bool bEnforceTileRestriction=true );
  TComDataCU*   getPUAboveOffset            ( UInt& uiPartUnitIdx, 
                                              UInt uiCurrPartUnitIdx, 
                                              UInt uiPartOffset=0,
                                              Bool bEnforceSliceRestriction=true, 
                                              Bool planarAtLCUBoundary = true,
                                              Bool bEnforceTileRestriction=true );
#endif

#if COM16_C1016_AFFINE
  Bool*        getAffineFlag            ()                       { return m_affineFlag;            }
  Bool         getAffineFlag            ( UInt idx )             { return m_affineFlag[idx];       }
  Void         setAffineFlag            ( UInt idx, Bool affine) { m_affineFlag[idx] = affine;     }
  Void         setAffineFlagSubParts    ( Bool bAffineFlag, UInt uiAbsPartIdx, UInt uiPartIdx, UInt uiDepth );
  Bool         isAffine                 ( UInt uiAbsPartIdx );
  UInt         getCtxAffineFlag         ( UInt uiAbsPartIdx );

  Void         setAllAffineMvField      ( UInt uiAbsPartIdx, UInt uiPuIdx, TComMvField *pcMvField, RefPicList eRefPicList, UInt uiDepth );
  Void         setAllAffineMv           ( UInt uiAbsPartIdx, UInt uiPuIdx, TComMv acMv[3], RefPicList eRefPicList, UInt uiDepth );
  Void         setAllAffineMvd          ( UInt uiAbsPartIdx, UInt uiPuIdx, TComMv acMvd[3], RefPicList eRefPicList, UInt uiDepth );

  // construct affine inter candidate list
  Void         fillAffineMvpCand ( UInt uiPartIdx, UInt uiPartAddr, RefPicList eRefPicList, Int iRefIdx, AffineAMVPInfo* pInfo );
  Bool         isValidAffineCandidate ( UInt uiAbsPartIdx, UInt uiPUIdx, TComMv cMv0, TComMv cMv1, TComMv cMv2, Int& riDV );

  // construct affine merge candidate list
  Void         getAffineMergeCandidates ( UInt uiAbsPartIdx, UInt uiPuIdx, TComMvField (*pcMFieldNeighbours)[3], UChar* puhInterDirNeighbours, Int& numValidMergeCand, Int mrgCandIdx = -1 );
  Bool         isAffineMrgFlagCoded     ( UInt uiAbsPartIdx, UInt uiPUIdx );
#endif

  // -------------------------------------------------------------------------------------------------------------------
  // member functions for accessing partition information
  // -------------------------------------------------------------------------------------------------------------------

  Void          getPartIndexAndSize   ( UInt uiPartIdx, UInt& ruiPartAddr, Int& riWidth, Int& riHeight ); // This is for use by a leaf/sub CU object only, with no additional AbsPartIdx
#if COM16_C806_VCEG_AZ10_SUB_PU_TMVP
  Void          getPartIndexAndSize   ( UInt uiPartIdx, UInt& ruiPartAddr, Int& riWidth, Int& riHeight, UInt uiAbsPartIdx, Bool bLCU) ;
#endif
  UChar         getNumPartitions      ( const UInt uiAbsPartIdx = 0 );
#if !JVET_C0024_QTBT
  Bool          isFirstAbsZorderIdxInDepth (UInt uiAbsPartIdx, UInt uiDepth);
#endif

  // -------------------------------------------------------------------------------------------------------------------
  // member functions for motion vector
  // -------------------------------------------------------------------------------------------------------------------

  Void          getMvField            ( TComDataCU* pcCU, UInt uiAbsPartIdx, RefPicList eRefPicList, TComMvField& rcMvField );

  Void          fillMvpCand           ( UInt uiPartIdx, UInt uiPartAddr, RefPicList eRefPicList, Int iRefIdx, AMVPInfo* pInfo 
#if VCEG_AZ07_FRUC_MERGE
    , TComPrediction * pPred = NULL
#endif
    );
  Bool          isDiffMER             ( Int xN, Int yN, Int xP, Int yP);
  Void          getPartPosition       ( UInt partIdx, Int& xP, Int& yP, Int& nPSW, Int& nPSH);

  Void          setMVPIdx             ( RefPicList eRefPicList, UInt uiIdx, Int iMVPIdx)  { m_apiMVPIdx[eRefPicList][uiIdx] = iMVPIdx;  }
  Int           getMVPIdx             ( RefPicList eRefPicList, UInt uiIdx)               { return m_apiMVPIdx[eRefPicList][uiIdx];     }
  Char*         getMVPIdx             ( RefPicList eRefPicList )                          { return m_apiMVPIdx[eRefPicList];            }

  Void          setMVPNum             ( RefPicList eRefPicList, UInt uiIdx, Int iMVPNum ) { m_apiMVPNum[eRefPicList][uiIdx] = iMVPNum;  }
  Int           getMVPNum             ( RefPicList eRefPicList, UInt uiIdx )              { return m_apiMVPNum[eRefPicList][uiIdx];     }
  Char*         getMVPNum             ( RefPicList eRefPicList )                          { return m_apiMVPNum[eRefPicList];            }

  Void          setMVPIdxSubParts     ( Int iMVPIdx, RefPicList eRefPicList, UInt uiAbsPartIdx, UInt uiPartIdx, UInt uiDepth );
  Void          setMVPNumSubParts     ( Int iMVPNum, RefPicList eRefPicList, UInt uiAbsPartIdx, UInt uiPartIdx, UInt uiDepth );

  Void          clipMv                ( TComMv&     rcMv     );
#if VCEG_AZ08_INTRA_KLT
  Void          clipMvIntraConstraint ( Int regionId, Int &iHorMin, Int &iHorMax, Int &iVerMin, Int &iVerMax, Int iRange, UInt uiTemplateSize, UInt uiBlkSize, Int iY, Int iCurrX, Int offsetLCUY, Int offsetLCUX);
#endif
  Void          getMvPredLeft         ( TComMv&     rcMvPred )   { rcMvPred = m_cMvFieldA.getMv(); }
  Void          getMvPredAbove        ( TComMv&     rcMvPred )   { rcMvPred = m_cMvFieldB.getMv(); }
  Void          getMvPredAboveRight   ( TComMv&     rcMvPred )   { rcMvPred = m_cMvFieldC.getMv(); }

  Void          compressMV            ();

  // -------------------------------------------------------------------------------------------------------------------
  // utility functions for neighbouring information
  // -------------------------------------------------------------------------------------------------------------------

  TComDataCU*   getCtuLeft                  () { return m_pCtuLeft;       }
  TComDataCU*   getCtuAbove                 () { return m_pCtuAbove;      }
  TComDataCU*   getCtuAboveLeft             () { return m_pCtuAboveLeft;  }
  TComDataCU*   getCtuAboveRight            () { return m_pCtuAboveRight; }
  TComDataCU*   getCUColocated              ( RefPicList eRefPicList ) { return m_apcCUColocated[eRefPicList]; }
  Bool          CUIsFromSameSlice           ( const TComDataCU *pCU /* Can be NULL */) const { return ( pCU!=NULL && pCU->getSlice()->getSliceCurStartCtuTsAddr() == getSlice()->getSliceCurStartCtuTsAddr() ); }
  Bool          CUIsFromSameTile            ( const TComDataCU *pCU /* Can be NULL */) const;
  Bool          CUIsFromSameSliceAndTile    ( const TComDataCU *pCU /* Can be NULL */) const;
  Bool          CUIsFromSameSliceTileAndWavefrontRow( const TComDataCU *pCU /* Can be NULL */) const;
  Bool          isLastSubCUOfCtu(const UInt absPartIdx);


  TComDataCU*   getPULeft                   ( UInt&  uiLPartUnitIdx,
                                              UInt uiCurrPartUnitIdx,
                                              Bool bEnforceSliceRestriction=true,
                                              Bool bEnforceTileRestriction=true );

  TComDataCU*   getPUAbove                  ( UInt&  uiAPartUnitIdx,
                                              UInt uiCurrPartUnitIdx,
                                              Bool bEnforceSliceRestriction=true,
                                              Bool planarAtCTUBoundary = false,
                                              Bool bEnforceTileRestriction=true );
  TComDataCU*   getPUAboveLeft              ( UInt&  uiALPartUnitIdx, UInt uiCurrPartUnitIdx, Bool bEnforceSliceRestriction=true );
  TComDataCU*   getQpMinCuLeft              ( UInt&  uiLPartUnitIdx , UInt uiCurrAbsIdxInCtu );
  TComDataCU*   getQpMinCuAbove             ( UInt&  uiAPartUnitIdx , UInt uiCurrAbsIdxInCtu );
  Char          getRefQP                    ( UInt   uiCurrAbsIdxInCtu                       );

  /// returns CU and part index of the PU above the top row of the current uiCurrPartUnitIdx of the CU, at a horizontal offset (to the right) of uiPartUnitOffset (in parts)
  TComDataCU*   getPUAboveRight             ( UInt&  uiARPartUnitIdx, UInt uiCurrPartUnitIdx, UInt uiPartUnitOffset = 1, Bool bEnforceSliceRestriction=true );
  /// returns CU and part index of the PU left of the lefthand column of the current uiCurrPartUnitIdx of the CU, at a vertical offset (below) of uiPartUnitOffset (in parts)
  TComDataCU*   getPUBelowLeft              ( UInt&  uiBLPartUnitIdx, UInt uiCurrPartUnitIdx, UInt uiPartUnitOffset = 1, Bool bEnforceSliceRestriction=true );

  Void          deriveLeftRightTopIdx       ( UInt uiPartIdx, UInt& ruiPartIdxLT, UInt& ruiPartIdxRT );
  Void          deriveLeftBottomIdx         ( UInt uiPartIdx, UInt& ruiPartIdxLB );

  Bool          hasEqualMotion              ( UInt uiAbsPartIdx, TComDataCU* pcCandCU, UInt uiCandAbsPartIdx );
  Void          getInterMergeCandidates       ( UInt uiAbsPartIdx, UInt uiPUIdx, TComMvField* pcMFieldNeighbours, UChar* puhInterDirNeighbours, Int& numValidMergeCand
#if VCEG_AZ06_IC
    , Bool*         pbICFlag
#endif
#if COM16_C806_VCEG_AZ10_SUB_PU_TMVP
  , UChar*          peMergeTypeNeighbors
#if JVET_C0035_ATMVP_SIMPLIFICATION
  , TComMvField*    pcMvFieldSP[NUM_MGR_TYPE]
  , UChar*          puhInterDirSP[NUM_MGR_TYPE]
#else
  , TComMvField*    pcMvFieldSP[2]
  , UChar*          puhInterDirSP[2]
#endif
  , UInt            uiDecCurrAbsPartIdx = 0
  , TComDataCU*     pDecCurrCU = NULL
#endif
    , Int mrgCandIdx = -1 );
#if COM16_C806_VCEG_AZ10_SUB_PU_TMVP
  
  Void          getSPPara(Int iPUWidth, Int iPUHeight, Int& iNumSP, Int& iNumSPInOneLine, Int& iSPWidth, Int& iSPHeight);
  Void          getSPAbsPartIdx(UInt uiBaseAbsPartIdx, Int iWidth, Int iHeight, Int iPartIdx, Int iNumPartLine, UInt& ruiPartAddr );
  Void          setInterDirSP( UInt uiDir, UInt uiAbsPartIdx, Int iWidth, Int iHeight );

#if JVET_C0035_ATMVP_SIMPLIFICATION
  Bool          getInterMergeSubPUTmvpCandidate ( UInt uiAbsPartIdx, UInt uiPUIdx,UInt uiCount,TComMvField* pcMvFieldNeighbours, UChar* puhInterDirNeighbours,
                                                   TComMvField* pcMvFieldSP, UChar* puhInterDirSP, TComMvField* pcMvFieldDefault, UChar* pcInterDirDefault, Bool bMrgIdxMatchATMVPCan, 
#if VCEG_AZ06_IC
    Bool& rbICFlag,
#endif
    TComDataCU* pDecCurrCU = NULL, UInt uiDecCurrAbsPartIdx=0);
#else
  Void          get1stTvFromSpatialNeighbor ( UInt uiAbsPartIdx, UInt uiPUIdx, Bool &bTvAva, Int &iPOC, TComMv &rcMv);
  Bool          getInterMergeSubPUTmvpCandidate ( UInt uiPUIdx,  TComMvField* pcMvFieldSP, UChar* puhInterDirSP, TComMvField* pcMvFieldDefault, UChar* pcInterDirDefault, TComMv cTMv, Bool bMrgIdxMatchATMVPCan, 
#if VCEG_AZ06_IC
    Bool& rbICFlag,
#endif
    Int iPocColPic =0, TComDataCU* pDecCurrCU = NULL, UInt uiDecCurrAbsPartIdx=0);
#endif

  Bool          deriveScaledMotionTemporalForOneDirection( TComDataCU* pcTempCU,RefPicList eCurrRefPicList, TComMv &cColMv, UInt uiAbsPartIdx, Int iTargetRefIdx, 
#if VCEG_AZ06_IC
    Bool& rbICFlag,
#endif
#if JVET_C0035_ATMVP_SIMPLIFICATION
    TComPic *pColPic,  RefPicList eFetchRefPicList=REF_PIC_LIST_0);
#else
    TComPic *pColPic);
#endif
  TComPic*      getPicfromPOC (Int iPocColPic);

  Void getNeighboringMvField(TComDataCU *pcCU, UInt uiPartIdx, TComMvField *cMvField,UChar *pucInterDir);
  Void generateMvField(TComMvField *cMvField,UChar* pucInterDir, UInt uiMvNum,TComMvField* cMvFieldMedian,UChar &ucInterDirMedian);  
  Bool getInterMergeSubPURecursiveCandidate( UInt uiAbsPartIdx, UInt uiPUIdx, TComMvField* pcMvFieldNeighbours, UChar* puhInterDirNeighbours, Int& numValidMergeCand
  , UChar*          peMergeTypeNeighbors  , 
#if JVET_C0035_ATMVP_SIMPLIFICATION
    TComMvField* pcMvFieldSP[NUM_MGR_TYPE] , UChar* puhInterDirSP[NUM_MGR_TYPE] ,
#else
    TComMvField* pcMvFieldSP[2] , UChar* puhInterDirSP[2] ,
#endif
  Int iCount );
#endif
#if VCEG_AZ07_IMV
#if VCEG_AZ07_MV_ADD_PRECISION_BIT_FOR_STORE
  Void          xRoundMV( TComMv & rMV ) 
  { 
    rMV += TComMv( 2 << VCEG_AZ07_MV_ADD_PRECISION_BIT_FOR_STORE , 2 << VCEG_AZ07_MV_ADD_PRECISION_BIT_FOR_STORE ); 
    rMV >>= ( 2 + VCEG_AZ07_MV_ADD_PRECISION_BIT_FOR_STORE ); 
    rMV <<= ( 2 + VCEG_AZ07_MV_ADD_PRECISION_BIT_FOR_STORE ); 
  }

#if JVET_E0076_MULTI_PEL_MVD                  
  Void xRoundMV( TComMv & rMV , UInt uiAbsPartIdx) 
  { 
    Int shift = 2 + VCEG_AZ07_MV_ADD_PRECISION_BIT_FOR_STORE;
    if (getiMVFlag(uiAbsPartIdx) == 2)
    {
      shift+=MULTI_PEL_MVD_BITS;
    }
    rMV += TComMv( 1<<( shift-1), 1<<( shift-1) ); 
    rMV >>= shift; 
    rMV <<= shift; 
  }
#endif

#else
  Void          xRoundMV( TComMv & rMV ) { rMV += TComMv( 2 , 2 ); rMV >>= 2; rMV <<= 2; }
#endif
  Char          getMaxNeighboriMVCandNum( UInt uiAbsPartIdx );
  Bool          resetMVDandMV2Int( UInt uiAbsPartIdx , UInt uiPartIdx , Bool bResetMV 
#if VCEG_AZ07_FRUC_MERGE
    , TComPrediction * pPred
#endif
    );
  Bool          resetMVDandMV2Int( Bool bResetMV 
#if VCEG_AZ07_FRUC_MERGE
    , TComPrediction * pPred
#endif
    );
#endif

  Void          deriveLeftRightTopIdxGeneral  ( UInt uiAbsPartIdx, UInt uiPartIdx, UInt& ruiPartIdxLT, UInt& ruiPartIdxRT );
  Void          deriveLeftBottomIdxGeneral    ( UInt uiAbsPartIdx, UInt uiPartIdx, UInt& ruiPartIdxLB );

  // -------------------------------------------------------------------------------------------------------------------
  // member functions for modes
  // -------------------------------------------------------------------------------------------------------------------

  Bool          isIntra            ( UInt uiPartIdx )  const { return m_pePredMode[ uiPartIdx ] == MODE_INTRA;                                              }
  Bool          isInter            ( UInt uiPartIdx )  const { return m_pePredMode[ uiPartIdx ] == MODE_INTER;                                              }
  Bool          isSkipped          ( UInt uiPartIdx );                                                     ///< returns true, if the partiton is skipped
  Bool          isBipredRestriction( UInt puIdx );

  // -------------------------------------------------------------------------------------------------------------------
  // member functions for symbol prediction (most probable / mode conversion)
  // -------------------------------------------------------------------------------------------------------------------

  UInt          getIntraSizeIdx                 ( UInt uiAbsPartIdx                                       );

#if JVET_E0077_ENHANCED_LM
  Int           getLMSymbolList(Int *pModeList, Int uiAbsPartIdx);
  Int           getAllowedChromaDir(UInt uiAbsPartIdx, UInt* uiModeList);
#else
  Void          getAllowedChromaDir             ( UInt uiAbsPartIdx, UInt* uiModeList );
#endif


#if JVET_E0062_MULTI_DMS
  UInt          getDMMode                       ( UInt uiAbsPartIdx, UInt uiDMIdx, UInt uiChMode[NUM_DM_MODES] = NULL, UInt* iTotalCnt = NULL) ;
#endif


  Void          getIntraDirPredictor            ( UInt uiAbsPartIdx, Int uiIntraDirPred[NUM_MOST_PROBABLE_MODES], const ComponentID compID
#if VCEG_AZ07_INTRA_65ANG_MODES && !JVET_C0055_INTRA_MPM
    , Int &iAboveLeftCase
#endif
    , Int* piMode = NULL );

  // -------------------------------------------------------------------------------------------------------------------
  // member functions for SBAC context
  // -------------------------------------------------------------------------------------------------------------------

  UInt          getCtxSplitFlag                 ( UInt   uiAbsPartIdx, UInt uiDepth                   );
#if JVET_C0024_QTBT
  UInt          getCtxBTSplitFlag( UInt uiAbsPartIdx, UInt uiWidth, UInt uiHeight );
#endif
#if COM16_C806_LARGE_CTU
  Void          getMaxMinCUDepth                ( UChar & rucMinDepth , UChar & rucMaxDepth , UInt uiAbsPartIdx );
#endif
  UInt          getCtxQtCbf                     ( TComTU &rTu, const ChannelType chType );

  UInt          getCtxSkipFlag                  ( UInt   uiAbsPartIdx                                 );
  UInt          getCtxInterDir                  ( UInt   uiAbsPartIdx                                 );
#if VCEG_AZ07_FRUC_MERGE
  UInt          getCtxFRUCMgrMode               ( UInt   uiAbsPartIdx                                 );
  UInt          getCtxFRUCME                    ( UInt   uiAbsPartIdx                                 );
#endif
#if VCEG_AZ07_IMV
  UInt          getCtxiMVFlag                   ( UInt   uiAbsPartIdx                                 );
  Bool          hasSubCUNonZeroMVd              ();
#endif

  UInt&         getTotalBins            ()                            { return m_uiTotalBins;                              }
  // -------------------------------------------------------------------------------------------------------------------
  // member functions for RD cost storage
  // -------------------------------------------------------------------------------------------------------------------

  Double&       getTotalCost()                  { return m_dTotalCost;        }
  Distortion&   getTotalDistortion()            { return m_uiTotalDistortion; }
  UInt&         getTotalBits()                  { return m_uiTotalBits;       }
  UInt&         getTotalNumPart()               { return m_uiNumPartition;    }
#if JVET_C0024_PBINTRA_FAST
  UInt&         getInterHAD()                   { return m_uiInterHAD; }
#endif
#if JVET_C0024_ITSKIP
  UInt*         getTUSkipWidth(ComponentID compID)                { return m_puiSkipWidth[compID];}
  UInt*         getTUSkipHeight(ComponentID compID)               { return m_puiSkipHeight[compID];}
  UInt&         getTUSkipWidth(ComponentID compID, UInt uiAbsPartIdx)                { return m_puiSkipWidth[compID][uiAbsPartIdx];}
  UInt&         getTUSkipHeight(ComponentID compID, UInt uiAbsPartIdx)               { return m_puiSkipHeight[compID][uiAbsPartIdx];}
#endif

  UInt          getCoefScanIdx(const UInt uiAbsPartIdx, const UInt uiWidth, const UInt uiHeight, const ComponentID compID) const ;

#if VCEG_AZ08_INTER_KLT
  Void          interpolatePic                 ( TComPic* pcPic );
#endif
};

namespace RasterAddress
{
  /** Check whether 2 addresses point to the same column
   * \param addrA          First address in raster scan order
   * \param addrB          Second address in raters scan order
   * \param numUnitsPerRow Number of units in a row
   * \return Result of test
   */
  static inline Bool isEqualCol( Int addrA, Int addrB, Int numUnitsPerRow )
  {
    // addrA % numUnitsPerRow == addrB % numUnitsPerRow
    return (( addrA ^ addrB ) &  ( numUnitsPerRow - 1 ) ) == 0;
  }

  /** Check whether 2 addresses point to the same row
   * \param addrA          First address in raster scan order
   * \param addrB          Second address in raters scan order
   * \param numUnitsPerRow Number of units in a row
   * \return Result of test
   */
  static inline Bool isEqualRow( Int addrA, Int addrB, Int numUnitsPerRow )
  {
    // addrA / numUnitsPerRow == addrB / numUnitsPerRow
    return (( addrA ^ addrB ) &~ ( numUnitsPerRow - 1 ) ) == 0;
  }

  /** Check whether 2 addresses point to the same row or column
   * \param addrA          First address in raster scan order
   * \param addrB          Second address in raters scan order
   * \param numUnitsPerRow Number of units in a row
   * \return Result of test
   */
  static inline Bool isEqualRowOrCol( Int addrA, Int addrB, Int numUnitsPerRow )
  {
    return isEqualCol( addrA, addrB, numUnitsPerRow ) | isEqualRow( addrA, addrB, numUnitsPerRow );
  }

  /** Check whether one address points to the first column
   * \param addr           Address in raster scan order
   * \param numUnitsPerRow Number of units in a row
   * \return Result of test
   */
  static inline Bool isZeroCol( Int addr, Int numUnitsPerRow )
  {
    // addr % numUnitsPerRow == 0
    return ( addr & ( numUnitsPerRow - 1 ) ) == 0;
  }

  /** Check whether one address points to the first row
   * \param addr           Address in raster scan order
   * \param numUnitsPerRow Number of units in a row
   * \return Result of test
   */
  static inline Bool isZeroRow( Int addr, Int numUnitsPerRow )
  {
    // addr / numUnitsPerRow == 0
    return ( addr &~ ( numUnitsPerRow - 1 ) ) == 0;
  }

  /** Check whether one address points to a column whose index is smaller than a given value
   * \param addr           Address in raster scan order
   * \param val            Given column index value
   * \param numUnitsPerRow Number of units in a row
   * \return Result of test
   */
  static inline Bool lessThanCol( Int addr, Int val, Int numUnitsPerRow )
  {
    // addr % numUnitsPerRow < val
    return ( addr & ( numUnitsPerRow - 1 ) ) < val;
  }

  /** Check whether one address points to a row whose index is smaller than a given value
   * \param addr           Address in raster scan order
   * \param val            Given row index value
   * \param numUnitsPerRow Number of units in a row
   * \return Result of test
   */
  static inline Bool lessThanRow( Int addr, Int val, Int numUnitsPerRow )
  {
    // addr / numUnitsPerRow < val
    return addr < val * numUnitsPerRow;
  }
}

//! \}

#endif
