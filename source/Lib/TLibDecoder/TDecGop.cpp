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

/** \file     TDecGop.cpp
    \brief    GOP decoder class
*/

#include "TDecGop.h"
#include "TDecCAVLC.h"
#include "TDecSbac.h"
#include "TDecBinCoder.h"
#include "TDecBinCoderCABAC.h"
#include "libmd5/MD5.h"
#include "TLibCommon/SEI.h"

#include <time.h>

//! \ingroup TLibDecoder
//! \{
static Void calcAndPrintHashStatus(TComPicYuv& pic, const SEIDecodedPictureHash* pictureHashSEI, const BitDepths &bitDepths, UInt &numChecksumErrors);
// ====================================================================================================================
// Constructor / destructor / initialization / destroy
// ====================================================================================================================

TDecGop::TDecGop()
 : m_numberOfChecksumErrorsDetected(0)
{
  m_dDecTime = 0;
}

TDecGop::~TDecGop()
{

}

Void TDecGop::create()
{

}


Void TDecGop::destroy()
{
#if COM16_C806_ALF_TEMPPRED_NUM
  for( Int i = 0; i < COM16_C806_ALF_TEMPPRED_NUM; i++ )
  {
#if JVET_E0104_ALF_TEMP_SCALABILITY
    for (Int j = 0; j < JVET_E0104_ALF_MAX_TEMPLAYERID; j++)
    {
      m_pcAdaptiveLoopFilter->freeALFParam(&m_acStoredAlfPara[j][i]);
    }    
#else
    m_pcAdaptiveLoopFilter->freeALFParam( &m_acStoredAlfPara[i] );
#endif
  }
#endif
}

#if COM16_C806_ALF_TEMPPRED_NUM
#if JVET_E0104_ALF_TEMP_SCALABILITY
Int TDecGop::m_iStoredAlfParaNum[JVET_E0104_ALF_MAX_TEMPLAYERID] = { 0, 0, 0, 0, 0 };
#else
Int TDecGop::m_iStoredAlfParaNum = 0;
#endif
#endif

Void TDecGop::init( TDecEntropy*            pcEntropyDecoder,
                   TDecSbac*               pcSbacDecoder,
                   TDecBinCABAC*           pcBinCABAC,
                   TDecCavlc*              pcCavlcDecoder,
                   TDecSlice*              pcSliceDecoder,
                   TComLoopFilter*         pcLoopFilter,
#if ALF_HM3_REFACTOR
                   TComAdaptiveLoopFilter* pcAdaptiveLoopFilter, 
#endif
                   TComSampleAdaptiveOffset* pcSAO
                   )
{
  m_pcEntropyDecoder      = pcEntropyDecoder;
  m_pcSbacDecoder         = pcSbacDecoder;
  m_pcBinCABAC            = pcBinCABAC;
  m_pcCavlcDecoder        = pcCavlcDecoder;
  m_pcSliceDecoder        = pcSliceDecoder;
  m_pcLoopFilter          = pcLoopFilter;
  m_pcSAO                 = pcSAO;
  m_numberOfChecksumErrorsDetected = 0;
#if ALF_HM3_REFACTOR
  m_pcAdaptiveLoopFilter  = pcAdaptiveLoopFilter;
#endif
}


// ====================================================================================================================
// Private member functions
// ====================================================================================================================
// ====================================================================================================================
// Public member functions
// ====================================================================================================================
Void TDecGop::decompressSlice(TComInputBitstream* pcBitstream, TComPic* pcPic
#if VCEG_AZ07_BAC_ADAPT_WDOW || VCEG_AZ07_INIT_PREVFRAME
                            , TComStats*  m_apcStats
#endif
  )
{
  TComSlice*  pcSlice = pcPic->getSlice(pcPic->getCurrSliceIdx());
  // Table of extracted substreams.
  // These must be deallocated AND their internal fifos, too.
  TComInputBitstream **ppcSubstreams = NULL;

  //-- For time output for each slice
  clock_t iBeforeTime = clock();
  m_pcSbacDecoder->init( (TDecBinIf*)m_pcBinCABAC );
  m_pcEntropyDecoder->setEntropyDecoder (m_pcSbacDecoder);
#if VCEG_AZ07_BAC_ADAPT_WDOW || VCEG_AZ07_INIT_PREVFRAME
  pcSlice->setStatsHandle( m_apcStats );
  m_pcEntropyDecoder->setStatsHandle ( m_apcStats );
#endif

#if JVET_D0033_ADAPTIVE_CLIPPING
    pcPic->m_aclip_prm=pcSlice->m_clip_decoded;
    g_ClipParam=pcPic->m_aclip_prm;
#endif
  const UInt uiNumSubstreams = pcSlice->getNumberOfSubstreamSizes()+1;

  // init each couple {EntropyDecoder, Substream}
  ppcSubstreams    = new TComInputBitstream*[uiNumSubstreams];
  for ( UInt ui = 0 ; ui < uiNumSubstreams ; ui++ )
  {
    ppcSubstreams[ui] = pcBitstream->extractSubstream(ui+1 < uiNumSubstreams ? (pcSlice->getSubstreamSize(ui)<<3) : pcBitstream->getNumBitsLeft());
  }

#if ALF_HM3_REFACTOR
  if ( pcSlice->getSPS()->getUseALF() )
  {
    m_pcAdaptiveLoopFilter->setNumCUsInFrame(pcPic);
    m_pcAdaptiveLoopFilter->allocALFParam(&m_cAlfParam);
    m_pcAdaptiveLoopFilter->resetALFParam(&m_cAlfParam);
#if COM16_C806_ALF_TEMPPRED_NUM
    static int iFirstLoop = 0;
    for( Int i = 0; i < COM16_C806_ALF_TEMPPRED_NUM && !iFirstLoop; i++ )
    {
#if JVET_E0104_ALF_TEMP_SCALABILITY
      for (Int j = 0; j < JVET_E0104_ALF_MAX_TEMPLAYERID; j++)
      {
        m_pcAdaptiveLoopFilter->allocALFParam(&m_acStoredAlfPara[j][i]);
      }
#else
      m_pcAdaptiveLoopFilter->allocALFParam( &m_acStoredAlfPara[i] );
#endif
    }
    iFirstLoop++;
#endif
  }
#endif
#if COM16_C806_ALF_TEMPPRED_NUM
  else
  {
    for( Int i = 0; i < COM16_C806_ALF_TEMPPRED_NUM; i++ )
    {
#if JVET_E0104_ALF_TEMP_SCALABILITY
      for (Int j = 0; j < JVET_E0104_ALF_MAX_TEMPLAYERID; j++)
      {
#if !JVET_C0038_GALF
        m_acStoredAlfPara[j][i].coeff = NULL;
#endif
        m_acStoredAlfPara[j][i].coeff_chroma = NULL;
        m_acStoredAlfPara[j][i].coeffmulti = NULL;
        m_acStoredAlfPara[j][i].alf_cu_flag = NULL;
        m_acStoredAlfPara[j][i].alfCoeffLuma = NULL;
        m_acStoredAlfPara[j][i].alfCoeffChroma = NULL;
      }
#else
#if !JVET_C0038_GALF
      m_acStoredAlfPara[i].coeff        = NULL;
#endif
      m_acStoredAlfPara[i].coeff_chroma = NULL;
      m_acStoredAlfPara[i].coeffmulti = NULL;
      m_acStoredAlfPara[i].alf_cu_flag = NULL;
      m_acStoredAlfPara[i].alfCoeffLuma   = NULL;
      m_acStoredAlfPara[i].alfCoeffChroma = NULL;
#endif
    }
  }
#endif

#if VCEG_AZ07_INIT_PREVFRAME
  pcSlice->setStatsHandle(m_apcStats);
  pcSlice->initStatsGlobal();
#endif

  m_pcSliceDecoder->decompressSlice( ppcSubstreams, pcPic, m_pcSbacDecoder
#if ALF_HM3_REFACTOR
    , m_cAlfParam
#endif
    );

#if VCEG_AZ07_BAC_ADAPT_WDOW || VCEG_AZ07_INIT_PREVFRAME
  m_pcEntropyDecoder->updateStates (pcSlice->getSliceType(), pcSlice->getSliceQp(), m_apcStats);
  pcSlice->updateStatsGlobal();
#endif

  // deallocate all created substreams, including internal buffers.
  for (UInt ui = 0; ui < uiNumSubstreams; ui++)
  {
    delete ppcSubstreams[ui];
  }
  delete[] ppcSubstreams;

  m_dDecTime += (Double)(clock()-iBeforeTime) / CLOCKS_PER_SEC;
}

Void TDecGop::filterPicture(TComPic* pcPic)
{
  TComSlice*  pcSlice = pcPic->getSlice(pcPic->getCurrSliceIdx());

  //-- For time output for each slice
  clock_t iBeforeTime = clock();

  // deblocking filter
  Bool bLFCrossTileBoundary = pcSlice->getPPS()->getLoopFilterAcrossTilesEnabledFlag();
  m_pcLoopFilter->setCfg(bLFCrossTileBoundary);
  m_pcLoopFilter->loopFilterPic( pcPic );

  if( pcSlice->getSPS()->getUseSAO() )
  {
    m_pcSAO->reconstructBlkSAOParams(pcPic, pcPic->getPicSym()->getSAOBlkParam());
    m_pcSAO->SAOProcess(pcPic);
    m_pcSAO->PCMLFDisableProcess(pcPic);
  }

#if ALF_HM3_REFACTOR
  // adaptive loop filter
  if( pcSlice->getSPS()->getUseALF() )
  {
#if COM16_C806_ALF_TEMPPRED_NUM
    if( m_pcAdaptiveLoopFilter->refreshAlfTempPred( pcSlice->getNalUnitType() , pcSlice->getPOC() ) )
    {
#if JVET_E0104_ALF_TEMP_SCALABILITY
      memset(m_iStoredAlfParaNum, 0, sizeof(Int)*JVET_E0104_ALF_MAX_TEMPLAYERID);
#else
      m_iStoredAlfParaNum = 0;
#endif
      assert( m_cAlfParam.temproalPredFlag == false );
    }
    if( m_cAlfParam.temproalPredFlag )
    {
#if JVET_E0104_ALF_TEMP_SCALABILITY
      m_pcAdaptiveLoopFilter->copyALFParam(&m_cAlfParam, &m_acStoredAlfPara[pcSlice->getTLayer()][m_cAlfParam.prevIdx]);
#else
      m_pcAdaptiveLoopFilter->copyALFParam( &m_cAlfParam, &m_acStoredAlfPara[m_cAlfParam.prevIdx] );
#endif
    }
#endif
    m_pcAdaptiveLoopFilter->ALFProcess(pcPic, &m_cAlfParam);
#if COM16_C806_ALF_TEMPPRED_NUM
    if( m_cAlfParam.alf_flag && !m_cAlfParam.temproalPredFlag && m_cAlfParam.filtNo >= 0 )
    {
#if JVET_E0104_ALF_TEMP_SCALABILITY
      Int iCurrTempIdx = pcSlice->getTLayer();
      assert(iCurrTempIdx < JVET_E0104_ALF_MAX_TEMPLAYERID);
      for (Int iLoopedTempIdx = iCurrTempIdx; iLoopedTempIdx < JVET_E0104_ALF_MAX_TEMPLAYERID; iLoopedTempIdx++)
      {
        Int iIdx = m_iStoredAlfParaNum[iLoopedTempIdx] % COM16_C806_ALF_TEMPPRED_NUM;
        m_iStoredAlfParaNum[iLoopedTempIdx] ++;
        m_acStoredAlfPara[iLoopedTempIdx][iIdx].temproalPredFlag = false;
        m_pcAdaptiveLoopFilter->copyALFParam(&m_acStoredAlfPara[iLoopedTempIdx][iIdx], &m_cAlfParam);
#if JVET_C0038_GALF
        m_pcAdaptiveLoopFilter->resetALFPredParam(&m_acStoredAlfPara[iLoopedTempIdx][iIdx], (pcSlice->getSliceType() == I_SLICE ? true : false));
#endif
      }
#else
      Int iIdx = m_iStoredAlfParaNum % COM16_C806_ALF_TEMPPRED_NUM;
      m_iStoredAlfParaNum++;
      m_acStoredAlfPara[iIdx].temproalPredFlag = false;
      m_pcAdaptiveLoopFilter->copyALFParam( &m_acStoredAlfPara[iIdx], &m_cAlfParam );
#if JVET_C0038_GALF
      m_pcAdaptiveLoopFilter->resetALFPredParam(&m_acStoredAlfPara[iIdx], (pcSlice->getSliceType()== I_SLICE? true: false));
#endif
#endif
    }
#endif
    m_pcAdaptiveLoopFilter->freeALFParam(&m_cAlfParam);
  }
#endif

#if COM16_C806_HEVC_MOTION_CONSTRAINT_REMOVAL
  if ( !pcSlice->getSPS()->getAtmvpEnableFlag() )
  {
    pcPic->compressMotion();
  }
#else
  pcPic->compressMotion();
#endif
  Char c = (pcSlice->isIntra() ? 'I' : pcSlice->isInterP() ? 'P' : 'B');
  if (!pcSlice->isReferenced())
  {
    c += 32;
  }

  //-- For time output for each slice
  printf("POC %4d TId: %1d ( %c-SLICE, QP%3d ) ", pcSlice->getPOC(),
                                                  pcSlice->getTLayer(),
                                                  c,
                                                  pcSlice->getSliceQp() );

  m_dDecTime += (Double)(clock()-iBeforeTime) / CLOCKS_PER_SEC;
  printf ("[DT %6.3f] ", m_dDecTime );
  m_dDecTime  = 0;

  for (Int iRefList = 0; iRefList < 2; iRefList++)
  {
    printf ("[L%d ", iRefList);
    for (Int iRefIndex = 0; iRefIndex < pcSlice->getNumRefIdx(RefPicList(iRefList)); iRefIndex++)
    {
      printf ("%d ", pcSlice->getRefPOC(RefPicList(iRefList), iRefIndex));
    }
    printf ("] ");
  }
  if (m_decodedPictureHashSEIEnabled)
  {
    SEIMessages pictureHashes = getSeisByType(pcPic->getSEIs(), SEI::DECODED_PICTURE_HASH );
    const SEIDecodedPictureHash *hash = ( pictureHashes.size() > 0 ) ? (SEIDecodedPictureHash*) *(pictureHashes.begin()) : NULL;
    if (pictureHashes.size() > 1)
    {
      printf ("Warning: Got multiple decoded picture hash SEI messages. Using first.");
    }
    calcAndPrintHashStatus(*(pcPic->getPicYuvRec()), hash, pcSlice->getSPS()->getBitDepths(), m_numberOfChecksumErrorsDetected);
  }

  printf("\n");

  pcPic->setOutputMark(pcPic->getSlice(0)->getPicOutputFlag() ? true : false);
  pcPic->setReconMark(true);
#if VCEG_AZ08_INTER_KLT
#if VCEG_AZ08_USE_KLT
  if (pcSlice->getSPS()->getUseInterKLT())
  {
#endif
      m_pcSliceDecoder->InterpolatePic(pcPic);
#if VCEG_AZ08_USE_KLT
  }
#endif
#endif
}

/**
 * Calculate and print hash for pic, compare to picture_digest SEI if
 * present in seis.  seis may be NULL.  Hash is printed to stdout, in
 * a manner suitable for the status line. Theformat is:
 *  [Hash_type:xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx,(yyy)]
 * Where, x..x is the hash
 *        yyy has the following meanings:
 *            OK          - calculated hash matches the SEI message
 *            ***ERROR*** - calculated hash does not match the SEI message
 *            unk         - no SEI message was available for comparison
 */
static Void calcAndPrintHashStatus(TComPicYuv& pic, const SEIDecodedPictureHash* pictureHashSEI, const BitDepths &bitDepths, UInt &numChecksumErrors)
{
  /* calculate MD5sum for entire reconstructed picture */
  TComPictureHash recon_digest;
  Int numChar=0;
  const Char* hashType = "\0";

  if (pictureHashSEI)
  {
    switch (pictureHashSEI->method)
    {
      case SEIDecodedPictureHash::MD5:
        {
          hashType = "MD5";
          numChar = calcMD5(pic, recon_digest, bitDepths);
          break;
        }
      case SEIDecodedPictureHash::CRC:
        {
          hashType = "CRC";
          numChar = calcCRC(pic, recon_digest, bitDepths);
          break;
        }
      case SEIDecodedPictureHash::CHECKSUM:
        {
          hashType = "Checksum";
          numChar = calcChecksum(pic, recon_digest, bitDepths);
          break;
        }
      default:
        {
          assert (!"unknown hash type");
          break;
        }
    }
  }

  /* compare digest against received version */
  const Char* ok = "(unk)";
  Bool mismatch = false;

  if (pictureHashSEI)
  {
    ok = "(OK)";
    if (recon_digest != pictureHashSEI->m_pictureHash)
    {
      ok = "(***ERROR***)";
      mismatch = true;
    }
  }

  printf("[%s:%s,%s] ", hashType, hashToString(recon_digest, numChar).c_str(), ok);

  if (mismatch)
  {
    numChecksumErrors++;
    printf("[rx%s:%s] ", hashType, hashToString(pictureHashSEI->m_pictureHash, numChar).c_str());
  }
}
//! \}
