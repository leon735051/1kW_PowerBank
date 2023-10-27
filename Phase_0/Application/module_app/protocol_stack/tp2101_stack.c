/**
  ******************************************************************************
  * @file    lpiTimer_bsp.c
  * @author  Oliver .Chiu.
  * @brief   Real Time Clock Board Support Package Driver.
  *          This file provides firmware functions to manage the following
  *          functionalities of the System Timer Counter Process:
  *           + Initialization and de-initialization functions
  *           + IO operation functions
  *
  @verbatim
  ==============================================================================
                    ##### Peripheral features #####
  ==============================================================================
  [..]
    (+) None
  @endverbatim
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 NucalTech, Inc.
  * All rights reserved.</center></h2>
  *
  * THIS SOFTWARE IS PROVIDED BY NUCALTECH "AS IS" AND ANY EXPRESSED OR
  * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
  * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
  * IN NO EVENT SHALL NUCALTECH OR ITS CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
  * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
  * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
  * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
  * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
  * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
  * THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Including necessary module. Cpu.h contains other modules needed for compiling.*/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "driver_bsp/CANpal_bsp.h"
#include "module_app/bms_mod/bms_mod.h"
#include "module_app/bms_mod/err_mod.h"
#include "module_app/bqMaximo/bqMaximo.h"
#include "tp2101_stack.h"
#include "module_app/CoulCount/CoulCount.h"
#include "module_app/flash_mod/flash_mod.h"
#include "driver_bsp/rtc_bsp.h"
#include "driver_bsp/PortIRQ_bsp.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define TP2101_CMDIDX 0
#define TP2101_SUBIDX 1

/*******************************************************************************
 * Variables
 ******************************************************************************/
static const uint8_t TPSWAK_KEY[] = { 0x36, 0x39, 0x36, 0x39, 0x37, 0x32, 0x34};
TP2101_cfg_params_t TP2101_cfg;
TP2101_rpo_flags_t  TP2101_flags;
/*******************************************************************************
 * Private Functions
 ******************************************************************************/

/*FUNCTION**********************************************************************
 *
 * Function Name : sysLED_ProcessInit
 * Description   : Initial sysLED process.
 *
 * Implements    : sysLED Process Initial
 *END**************************************************************************/
extern BMS_ERROR_ALARM_t  bms_error_alarm;
static bool TP2101ReqBatSn(uint8_t *u8buf);
static uint16_t TP2101ReqBatSTS_1A(void);
static uint16_t TP2101ReqBmsSTS_1B(void);
static uint16_t TP2101ReqBmsPRT_1C(void);

void  TP2101ValueInitial(void) {
  TP2101_cfg.run_mode = tp1_Boot;
  TP2101_flags.SYSWAKE = false;
  TP2101_flags.FACTORY = false;
  TP2101_flags.BLT = false;
  TP2101_flags.PROTENB = false;
  TP2101_flags.ESD = false;
}

bool TP2101GetFlags(uint8_t u8Idx) {
  bool nRes = false;
  switch(u8Idx) {
    case tp_FLG_SYSWAKE:
      nRes = TP2101_flags.SYSWAKE;
    break;
    case tp_FLG_FACTORY:
      nRes = TP2101_flags.FACTORY;
    break;
    case tp_FLG_BLT:
      nRes = TP2101_flags.BLT;
    case tp_FLG_PROTENB:
      nRes = TP2101_flags.PROTENB;
    break;
    default: break;
  }
  return nRes;
}

uint8_t TP2101GetConfig(uint8_t u8Idx) {
  uint8_t u8tmp = false;
  switch(u8Idx) {
    case tp_Cfg_RunMode:
      u8tmp = TP2101_cfg.run_mode;
    break;
    default: break;
  }
  return u8tmp;
}

bool TP2101SetRunMode(uint8_t u8idx) {
  
  TP2101_cfg.run_mode = u8idx;
  return false;
}

bool TP2101ReqWakProcess(uint32_t can_id, uint8_t *msg) {
    bool nErr = false;
    uint8_t *u8ptr, *u8buf;
    uint8_t u8i, u8len;
    uint8_t u8tmp;
    u8buf = (uint8_t *)calloc( 8, sizeof(char));
    if(u8buf == NULL) {
        nErr = true;
    } else {
        u8ptr = u8buf;
    }
    if(nErr != true) {
      switch(*(msg + TP2101_CMDIDX)) {
        case tp1_RUN_MODE:  /* System Run Mode */
          u8len = 8;
          *u8ptr++ = TP2101_cfg.run_mode;
          *u8ptr++ = 0x00;
          *u8ptr++ = 0x01;
        break;        
        case tp1_WAK_SYSWAKE: /* Wake Up Systeme */ 
          u8len = 8;
          u8tmp = sizeof(TPSWAK_KEY)/sizeof(*TPSWAK_KEY); 
          for(u8i = TP2101_SUBIDX; u8i <= u8tmp; u8i++) {
            if(*(msg + u8i) != TPSWAK_KEY[u8i -TP2101_SUBIDX]) {
              nErr = true;
            }
          }
          if(nErr != true) {
            TP2101_flags.SYSWAKE = true;
          } else {
            TP2101_flags.SYSWAKE = false;
          }
          *u8ptr++ = TP2101_cfg.run_mode;
        break;
        default:
          nErr = true;
        break;
      }
      if((nErr != true) && (TP2101_flags.ESD != true)) {
        if(SendCANpalTxData(TP2101_REQDEG_ID, u8buf, u8len,CAN_TYPE_STD) != false) {
          nErr = true;
        }
      }
    }
    free(u8buf);
    return nErr;
}


bool TP2101ReqModProcess(uint32_t can_id, uint8_t *msg) {
    bool nErr = false;
    uint8_t *u8ptr, *u8buf;
    uint8_t u8len;
    u8buf = (uint8_t *)calloc( 8, sizeof(char));
    if(u8buf == NULL) {
        nErr = true;
    } else {
        u8ptr = u8buf;
    }
    if((TP2101_flags.SYSWAKE != false) && (nErr != true)) {
      switch(*(msg + TP2101_CMDIDX)) {
        case tp1_RUN_MODE:  /* System Run Mode */
          u8len = 8;
          *u8ptr++ = 0x02;
          *u8ptr++ = 0x00;          
          *u8ptr++ = 0x01;
          *u8ptr++ = TP2101_cfg.run_mode;
        break;
        case tp1_MOD_SN: /* Report alived model id and serial */
          u8len = 8;
          *u8ptr++ = 0x02;
          *u8ptr++ = tp1_MOD_SN;
          *u8ptr++ = tp1_MOD_SN;
        break;
        case tp1_MOD_BLT: /* Selected device enter bootloader mode */
          u8len = 8;
          TP2101_flags.BLT = true;
          *u8ptr++ = 0x02;
          *u8ptr++ = 0x00;
          *u8ptr++ = 0x01;
          *u8ptr++ = tp1_Bootloader;
        break;
        case tp1_MOD_FACT: /* Selected device enter factory mode */
          u8len = 8;
          TP2101_flags.FACTORY = true;
          *u8ptr++ = 0x02;
          *u8ptr++ = 0x00;
          *u8ptr++ = 0x01;
          *u8ptr++ = tp1_Factory;
        break;
        default:
          if(TP2101_flags.FACTORY == true) {
            if(DataFlashSaveProcess() != true)
              TP2101_flags.FACTORY =false;
          }
         break;
      }
      if((nErr != true) && (TP2101_flags.ESD != true)) {
        if(SendCANpalTxData(TP2101_REQMODE_ID, u8buf, u8len,CAN_TYPE_STD) != false) {
          nErr = true;
        }
      }
    }
    free(u8buf);
    return nErr;
}

bool TP2101ReqFacProcess(uint32_t can_id, uint8_t *msg) {
    bool nErr = false;
    RTC_TimeTypeDef sTime;
    RTC_DateTypeDef sDate;
    uint8_t *u8ptr, *u8buf;
    uint8_t u8len, u8tmp, u8i;
    u8buf = (uint8_t *)calloc( 8, sizeof(char));
    if(u8buf == NULL) {
        nErr = true;
    } else {
        u8ptr = u8buf;
    }
    if (TP2101_flags.FACTORY != true) {
      nErr = true;
    }

    if(nErr != true) {
      switch(*(msg + TP2101_CMDIDX)) {
        case tp1_RUN_MODE: /* System Run Mode */
          u8len = 8;
          *u8ptr++ = TP2101_cfg.run_mode;
        break;
        case tp1_FAT_EXIT: /* Exit Lunched Mode */
          u8len = 8;
          TP2101_flags.FACTORY = false;
          *u8ptr++ = tp1_FAT_EXIT;
        break;
        case tp1_FAT_SETRTC_INFO: /* Write TB2101 RTC information */
          u8len = 8;
          for(u8i =1; u8i < 7; u8i++) {
           *u8ptr++ = *(msg + u8i);
          }
          if(RTCTimer_SET_Clcok(u8buf) != true) {
            u8ptr = u8buf;
            sDate = RTC_RetrunDate();
            sTime = RTC_RetrunTime();
            *u8ptr++ = tp1_FAT_SETRTC_INFO;          
            *u8ptr++ = sDate.Year;
            *u8ptr++ = sDate.Month;
            *u8ptr++ = sDate.Date;
            *u8ptr++ = sTime.Hours;
            *u8ptr++ = sTime.Minutes;
            *u8ptr++ = sTime.Seconds;
          }
        break;
        case tp1_FAT_REQRTC_INFO: /* Query RTC information */
          u8len = 8;
          sDate = RTC_RetrunDate();
          sTime = RTC_RetrunTime();
          *u8ptr++ = tp1_FAT_REQRTC_INFO;          
          *u8ptr++ = sDate.Year;
          *u8ptr++ = sDate.Month;
          *u8ptr++ = sDate.Date;
          *u8ptr++ = sTime.Hours;
          *u8ptr++ = sTime.Minutes;
          *u8ptr++ = sTime.Seconds;
        break;
        case tp1_FAT_BARCOD_SN: /* Write barcode serial number */
          u8len = 8;
          *u8ptr++ = tp1_FAT_BARCOD_SN;
          u8tmp = *(msg + TP2101_SUBIDX);
          if(u8tmp != false) {
            u8tmp = (u8tmp *6);
          }
          for(u8i = 2; u8i < u8len; u8i++) {
            if(u8tmp > sizeof(g_sConfig.SN)) {
              u8tmp = 0xff;
            } else {
              g_sConfig.SN[u8tmp++] = *(msg + u8i);;
            }
          }
        break;
        case tp1_FAT_DATE_MAUF: /* Write date of manufacture  */
          u8len = 8;
          *u8ptr++ = tp1_FAT_DATE_MAUF;
          u8tmp = true;
          for(u8i = false; u8i < sizeof(g_sConfig.DATE_MAF) ;u8i++) {
             g_sConfig.DATE_MAF[u8i] = *(msg + u8tmp);
             u8tmp++;
          }
        break;
        case tp1_FAT_ENB_ESD: /* Enter ESD test mode */
          u8len = 8;
          TP2101_flags.ESD = true;
          *u8ptr++ = tp1_FAT_ENB_ESD;        
        break; 
        case tp1_FAT_DIS_ESD: /* Exit ESD test mode */
          u8len = 8;
          TP2101_flags.ESD = false;
          *u8ptr++ = tp1_FAT_DIS_ESD;        
        break;
        case tp1_FAT_ENB_PROT: /* Protection_Enable */
          u8len = 8;
          *u8ptr++ = tp1_FAT_ENB_PROT;        
        break;
        case tp1_FAT_DIS_PROT:
          u8len = 8;
          *u8ptr++ = tp1_FAT_DIS_PROT;  
        break;
        case tp1_FAT_ENB_BAL: /* Force enable/disable cell balance */
          u8len = 8;
          *u8ptr++ = tp1_FAT_ENB_BAL;        
        break;            
        case tp1_FAT_MOS_CTL: /* AFE MosFET Control */
          u8len = 8;
          *u8ptr++ = tp1_FAT_MOS_CTL;        
        break;        
        case tp1_FAT_AFE_SHUT: /* AFE Shutdown */
          u8len = 8;
          bqDeviceSetSTS(fAFE_SHIP);
          *u8ptr++ = tp1_FAT_AFE_SHUT;
        break;
        case tp1_FAT_CLR_PF: /* Clear AFE PF */
          u8len = 8;
          *u8ptr++ = tp1_FAT_CLR_PF;        
        break;        
        case tp1_FAT_RST_PF: /* Reset AFE PF */
          u8len = 8;
          *u8ptr++ = tp1_FAT_RST_PF;
        break;   
        case tp1_FAT_SET_SOC: /* Set Soc */
          u8len = 8;
          *u8ptr++ = tp1_FAT_SET_SOC;
          u8tmp = *(msg + TP2101_SUBIDX);
          if(u8tmp > 100) {
            u8tmp = 100;
          }
          bms_data.BAT_SOC = u8tmp *10;
          *u8ptr++ = bms_data.BAT_SOC /10;
        break;   
        default:
          nErr = true;
         break;
      }
      if((nErr != true) && (TP2101_flags.ESD != true)) {
        if(SendCANpalTxData(TP2101_REQWAK_ID, u8buf, u8len,CAN_TYPE_STD) != false) {
          nErr = true;
        }
      }
    }
    free(u8buf);
    return nErr;
}

static bool TP2101ReqBatSn(uint8_t *u8buf) {
  bool nErr = false;
  uint8_t u8i, u8tmp, u8y;
  uint8_t *u8msg;
  
  for(u8i= false; u8i < 3; u8i++) {
    u8msg = u8buf;
    *u8msg++ = tp1_BAT_SN;
    *u8msg++ = u8i;
    u8tmp = u8i;
    if(u8tmp != false) {
      u8tmp = (u8i *6);
    } 
    for(u8y = 2; u8y < 8; u8y++) {
      if(u8tmp > sizeof(g_sConfig.SN)) {
        u8tmp = 0xff;
      } else {
        *u8msg++ = g_sConfig.SN[u8tmp++];
      }
    }
    if(TP2101_flags.ESD != true) {
      if(SendCANpalTxData(TP2101_REQINFO_ID, u8buf, 0x08,CAN_TYPE_STD) != false) {
        nErr = true;
      }
    }
  }
  return nErr;
}
  volatile TP2101_ReqBatSts_t  uMsg;
static uint16_t TP2101ReqBatSTS_1A(void) {

  uMsg.SUM = false;
  uint16_t u16tmp = false;
  u16tmp = bqDeviceGetValue(bqBat_CurAVG, false);
  if((bqDeviceGetValue(bqAFE_BATSTS, false) & BITF) == BITF) {
    if(u16tmp == false) {
        uMsg.IDLE = true;
    }
    if((bqDeviceGetValue(bqAFE_FETSTS,false) & BIT0) == BIT0) {
      if(u16tmp > 50) {
        uMsg.CHG_RDY = true;
      }
    }
    if((bqDeviceGetValue(bqAFE_FETSTS,false) & BIT3) == BIT3) {
      if(((u16tmp & 0x8000) == 0x8000) && (u16tmp < 0xFF9B)) {
        uMsg.DSG_RDY = true;
      }
    }
  } else {
    uMsg.SLEP = true;
  }
  if((bqDeviceGetValue(bqAFE_PFAB, false) & BIT8) == BIT8) {
    uMsg.CHG_FAUT = true;
  }
  if((bqDeviceGetValue(bqAFE_PFAB, false) & BIT9) == BIT9) {
    uMsg.DSG_FAUT = true;
  }  
  if((bqDeviceGetValue(bqAFE_CTLSTS, false) & BIT2) == BIT2) {
    uMsg.DEEP_SLEP = true;
  }
  if(bqDeviceGetValue(bqAFE_ALART, false) == true) {
    uMsg.PROTECT = true;
  }
  if(bqDeviceGetValue(bqAFE_PFAB, false) == true) {
    if(bqDeviceGetValue(bqAFE_PFCD, false) == true) {
      uMsg.PF = true;
    }
  }  
  return uMsg.SUM;
}

static uint16_t TP2101ReqBmsSTS_1B(void) {
  volatile TP2101_ReqBmsSts_t  uMsg;
  uMsg.SUM = false;
  if(PortIrq_PMuxFlags(PIRQ_WAKE_SIG) == false) {
    uMsg.CHG_IN = true;
  }
  if(BMS_ReturnStatus() == BMS_MODE_CHG) {
    uMsg.CHG_RDY = true;
  } else if(BMS_ReturnStatus() == BMS_MODE_Enable){
    uMsg.DSG_RDY = true;
  }
  if((bqDeviceGetValue(bqAFE_FETSTS,false) & BIT0) == BIT0) {
    uMsg.CHG_BRK = true;
  }
  if((bqDeviceGetValue(bqAFE_BATSTS, false) & BITF) == BITF) {
    uMsg.IDLE = true;
  }
  if((bqDeviceGetValue(bqAFE_ALART, false) & BIT2) == BIT2) {
    uMsg.BAL = true;
  }
  if(bqDeviceGetValue(bqAFE_ALART, false) >= 999) {
    uMsg.FULL_CHG = true;
  }
  if(bqDeviceGetValue(bqAFE_ALART, false) < 250) {
    uMsg.LOW_CAP = true;
  }
  if(BMS_ReturnStatus() == BMS_MODE_Error){
    uMsg.FD = true;
  }   
   if((bqDeviceGetValue(bqAFE_PFAB, false) & BIT8) == BIT8) {
    uMsg.C_FAULT = true;
  }
  if((bqDeviceGetValue(bqAFE_PFAB, false) & BIT9) == BIT9) {
    uMsg.D_FAULT = true;
  }       
  if(bqDeviceGetValue(bqAFE_PFAB, false) == true) {
    if(bqDeviceGetValue(bqAFE_PFCD, false) == true) {
      uMsg.PF = true;
    }
  }        
  return uMsg.SUM;
}


static uint16_t TP2101ReqBmsPRT_1C(void) {
  volatile TP2101_ReqBmsPrt_t  uMsg;
  uMsg.SUM = false;
  if((bqDeviceGetValue(bqAFE_SAFT_A, false) & BIT3) == BIT3) {
    uMsg.OV = true;
  }
  if((bqDeviceGetValue(bqAFE_PFAB, false) & BIT1) == BIT1) {
    uMsg.pfOV = true;
  }  
  if((bqDeviceGetValue(bqAFE_SAFT_A, false) & BIT2) == BIT2) {
    uMsg.OV = true;
  }
  if((bqDeviceGetValue(bqAFE_PFAB, false) & BIT0) == BIT0) {
    uMsg.pfOV = true;
  }    
  if((bqDeviceGetValue(bqAFE_ALART, false) & BIT2) == BIT2) {
    uMsg.IMB = true;
  }  
  if((bqDeviceGetValue(bqAFE_PFAB, false) & BITA) == BITA) {
    uMsg.pfIMB = true;
  }
  if((bqDeviceGetValue(bqAFE_PFAB, false) & BITB) == BITB) {
    uMsg.pfIMB = true;
  }  
  if((bqDeviceGetValue(bqAFE_SAFT_A, false) & BIT4) == BIT4) {
    uMsg.COC = true;
  }  
  if((bqDeviceGetValue(bqAFE_PFAB, false) & BIT2) == BIT2) {
    uMsg.pfCOC = true;
  }
  if((bqDeviceGetValue(bqAFE_SAFT_C, false) & BIT7) == BIT7) {
    uMsg.DOC = true;
  }  
  if((bqDeviceGetValue(bqAFE_SAFT_A, false) & BIT5) == BIT5) {
    uMsg.DOC2 = true;
  }     
  if((bqDeviceGetValue(bqAFE_PFAB, false) & BIT3) == BIT3) {
    uMsg.pfCOC = true;
  }
  if((bqDeviceGetValue(bqAFE_SAFT_A, false) & BIT2) == BIT2) {
    uMsg.SC = true;
  }
  if((bqDeviceGetValue(bqAFE_PFAB, false) & BITE) == BITE) {
    uMsg.pfSC = true;
  }
  if((bqDeviceGetValue(bqAFE_SAFT_B, false) & BIT4) == BIT4) {
    uMsg.COT = true;
  }  
  if((bqDeviceGetValue(bqAFE_SAFT_B, false) & BIT5) == BIT5) {
    uMsg.DOT = true;
  }
  if((bqDeviceGetValue(bqAFE_PFAB, false) & BIT4) == BIT4) {
    uMsg.pfOT = true;
  }
  if((bqDeviceGetValue(bqAFE_SAFT_B, false) & BIT0) == BIT0) {
    uMsg.CUT = true;
  }  
  if((bqDeviceGetValue(bqAFE_SAFT_B, false) & BIT1) == BIT1) {
    uMsg.DUT = true;
  }
  if((bqDeviceGetValue(bqAFE_SAFT_B, false) & BIT7) == BIT7) {
    uMsg.CFETOT = true;
    uMsg.DFETOT = true;
  }
  if((bqDeviceGetValue(bqAFE_PFAB, false) & BIT6) == BIT6) {
    uMsg.pfCFETOT = true;
    uMsg.pfDFETOT = true;
  }
  if((bqDeviceGetValue(bqAFE_PFAB, false) & BIT8) == BIT8) {
    uMsg.pfCFETFAIL = true;
  }   
  if((bqDeviceGetValue(bqAFE_PFAB, false) & BIT9) == BIT9) {
    uMsg.pfDFETFAIL = true;
  }
  if(BMS_ReturnStatus() == BMS_MODE_DSGEND){
    uMsg.LowCapP = true;
  }
  return uMsg.SUM;
}

bool TP2101ReqInfoProcess(uint32_t can_id, uint8_t *msg) {
    bool nErr = false;
    uint8_t *u8ptr, *u8buf;
    uint8_t u8i, u8len = false;
    uint32_t u32tmp;
    RTC_TimeTypeDef sTime;
    RTC_DateTypeDef sDate;    
    u8buf = (uint8_t *)calloc( 8, sizeof(char));
    if(u8buf == NULL) {
        nErr = true;
    } else {
        u8ptr = u8buf;
    }

    if(nErr != true) {
      switch (*(msg + TP2101_CMDIDX)) {
        case tp1_MODEL_NAME:
          u8len = 8;
          *u8ptr++ = tp1_MODEL_NAME;
          for(u8i = 1; u8i < u8len; u8i++) {
            *u8ptr++ = g_sConfig.MODEL_NAME[u8i -1];
          }
        break;
        case tp1_BAT_SN:
          TP2101ReqBatSn(u8buf);
          nErr = true;
        break;
        case tp1_HW_VER:   /*/ TBD    */
          u8len = 8;
          *u8ptr++ = tp1_HW_VER;
          for(u8i = 1; u8i < u8len; u8i++) {
            *u8ptr++ = 0xff;
          }
        break;
        case tp1_SW_VER:
           u8len = 8;
           *u8ptr++ = tp1_SW_VER;
          for(u8i = 1; u8i < u8len; u8i++) {
            *u8ptr++ = g_sConfig.FW_VERN[u8i -1];
          }
        break;
        case tp1_DATE_MAF: // Date of manufacture 
          u8len = 4;
          *u8ptr++ = tp1_DATE_MAF;
          for(u8i = 1; u8i < sizeof(g_sConfig.DATE_MAF) +1; u8i++) {
            *u8ptr++ = g_sConfig.DATE_MAF[u8i -1];
          }
        break;
        case tp1_PACK_PARM:
          u8len = 6;
          for(u8i = 1; u8i < sizeof(g_sConfig.PACK_PARM) +1; u8i++) {
            *u8ptr++ = g_sConfig.PACK_PARM[u8i -1];
          }
        break;
        case tp1_PACK_SPEC:
          u8len = 5;
          *u8ptr++ = tp1_PACK_SPEC;
          for(u8i = 1; u8i < sizeof(g_sConfig.PACK_SPEC) +1; u8i++) {
            *u8ptr++ = g_sConfig.PACK_SPEC[u8i -1];
          }
        break;
        case tp1_PACK_VOLT:
          u8len = 5;
          u32tmp = bqDeviceGetValue(bqBat_Pack, false);
          *u8ptr++ = tp1_PACK_VOLT;
          *u8ptr++ =  0x00;
          *u8ptr++ =  0x00;
          *u8ptr++ =  u32tmp >> 0x08;           
          *u8ptr++ =  u32tmp & 0xff;
        break;
        case tp1_PACK_CURR:
          u8len = 5;
          u32tmp = bqDeviceGetValue(bqBat_CurAVG, false);
          *u8ptr++ = tp1_PACK_CURR;
          *u8ptr++ =  0x00;
          *u8ptr++ =  0x00;         
          *u8ptr++ =  u32tmp >> 0x08;           
          *u8ptr++ =  u32tmp & 0xff;
        break;
        case tp1_MXIN_CURR:
          u8len = 5;
          u32tmp = bqDeviceGetValue(bqBat_InMaxCur, false);
          *u8ptr++ = tp1_MXIN_CURR;
          *u8ptr++ =  0x00;
          *u8ptr++ =  0x00;         
          *u8ptr++ =  u32tmp >> 0x08;           
          *u8ptr++ =  u32tmp & 0xff;
        break;
        case tp1_MIOT_CURR:
          u8len = 5;
          u32tmp = bqDeviceGetValue(bqBat_OtMaxCur, false);
          *u8ptr++ = tp1_MIOT_CURR;
          *u8ptr++ =  0x00;
          *u8ptr++ =  0x00;         
          *u8ptr++ =  u32tmp >> 0x08;           
          *u8ptr++ =  u32tmp & 0xff;          
        break;
        case tp1_CELL_VOLT:
          u8len = 4;
          u32tmp = bqDeviceGetValue(bqCell_Val, *(msg + TP2101_SUBIDX));
          *u8ptr++ = tp1_CELL_VOLT;
          *u8ptr++ = *(msg + TP2101_SUBIDX);
          *u8ptr++ = u32tmp >> 0x08;
          *u8ptr++ = u32tmp & 0xff;
        break;
        case tp1_MAX_CELLV:
          u8len = 3;
          u32tmp = bqDeviceGetValue(bqCell_MaxV, false);
          *u8ptr++ = tp1_MAX_CELLV; 
          *u8ptr++ =  u32tmp >> 0x08;  
          *u8ptr++ =  u32tmp & 0xff;
        break;
        case tp1_MIN_CELLV:
          u8len = 3;
          u32tmp = bqDeviceGetValue(bqCell_MinV, false);          
          *u8ptr++ = tp1_MIN_CELLV; 
          *u8ptr++ =  u32tmp >> 0x08;
          *u8ptr++ =  u32tmp & 0xff;
        break;
        case tp1_CELL_DELTA:
          u8len = 3;
          u32tmp = bqDeviceGetValue(bqCell_MaxV, false);
          u32tmp = u32tmp - bqDeviceGetValue(bqCell_MinV, false);
          *u8ptr++ = tp1_CELL_DELTA;             
          *u8ptr++ = u32tmp >> 0x08;
          *u8ptr++ = u32tmp & 0xff;
        break;
        case tp1_CELL_TEMP:
          u8len = 4;
          u32tmp = bqDeviceGetValue(bqCell_Tmp, (*(msg + TP2101_SUBIDX)-1));          
          u32tmp = u32tmp /100; 
          *u8ptr++ = tp1_CELL_TEMP;
          *u8ptr++ =  0x00;
          *u8ptr++ =  u32tmp >> 0x08;            
          *u8ptr++ =  u32tmp & 0xff;  
        break;
        case tp1_MAX_CELLT:
          u8len = 3;
          u32tmp = bqDeviceGetValue(bqCell_MaxT, false);
          u32tmp = u32tmp /100; 
          *u8ptr++ = tp1_MAX_CELLT;
          *u8ptr++ =  u32tmp >> 0x08;              
          *u8ptr++ =  u32tmp & 0xff;
          
        break;
        case tp1_MIN_CELLT:
          u8len = 3;
          u32tmp = bqDeviceGetValue(bqCell_MinT, false); 
          u32tmp = u32tmp /100;           
          *u8ptr++ = tp1_MIN_CELLT;
          *u8ptr++ =  u32tmp >> 0x08;            
          *u8ptr++ =  u32tmp & 0xff;
        break;
        case tp1_FET_TEMP:
          u8len = 5;
          u32tmp = bqDeviceGetValue(bqAFE_FetT, false); 
          u32tmp = u32tmp /100;
          *u8ptr++ = tp1_FET_TEMP;
          *u8ptr++ =  u32tmp >> 0x08;           
          *u8ptr++ =  u32tmp & 0xff;
          *u8ptr++ =  u32tmp >> 0x08;           
          *u8ptr++ =  u32tmp & 0xff;          
        break;
        case tp1_FCHG_CAP:
          u8len = 5;
          u32tmp = CoulCountRetValue(CC_CYC_CHG);
          *u8ptr++ = tp1_FCHG_CAP;
          *u8ptr++ =  u32tmp >> 0x18;
          *u8ptr++ =  (u32tmp >> 0x10) & 0xff;
          *u8ptr++ =  (u32tmp >> 0x08) & 0xff;
          *u8ptr++ =  u32tmp & 0xff;
        break;
        case tp1_REMN_CAP:
          u8len = 3;
          u32tmp = CoulCountRetValue(CC_R_CAP);
          u32tmp = u32tmp *10;
          *u8ptr++ = tp1_REMN_CAP;
          *u8ptr++ =  u32tmp >> 0x08;
          *u8ptr++ =  u32tmp & 0xff;
        break;
        case tp1_SOC:
          u8len = 2;
          *u8ptr++ = tp1_SOC;
          *u8ptr++ = bms_data.BAT_SOC /10;
        break;
        case tp1_SOH:
          u8len = 2;
          *u8ptr++ = tp1_SOH;
          *u8ptr++ = bms_data.BAT_SOH;
          *u8ptr++ = tp1_SOH;
        break;                
        case tp1_CYCLE_CNT:
          u8len = 3;
          u32tmp = CoulCountRetValue(CC_CYC_CNT);
          *u8ptr++ = tp1_CYCLE_CNT;   
          *u8ptr++ =  u32tmp >> 0x08;
          *u8ptr++ =  u32tmp & 0xff;       
        break; 
        case tp1_RTC_INFO:
          u8len = 7;
          *u8ptr++ = tp1_RTC_INFO;
          sDate = RTC_RetrunDate();
          sTime = RTC_RetrunTime();
          *u8ptr++ = sDate.Year;
          *u8ptr++ = sDate.Month;
          *u8ptr++ = sDate.Date;
          *u8ptr++ = sTime.Hours;
          *u8ptr++ = sTime.Minutes;
          *u8ptr++ = sTime.Seconds;
        break; 
        case tp1_LST_EOCT:
          u8len = 7;
          *u8ptr++ = tp1_LST_EOCT;
          sDate = g_sConfig.LCHG_DATE;
          sTime = g_sConfig.LCHG_TIME;
          *u8ptr++ = sDate.Year;
          *u8ptr++ = sDate.Month;
          *u8ptr++ = sDate.Date;
          *u8ptr++ = sTime.Hours;
          *u8ptr++ = sTime.Minutes;
          *u8ptr++ = sTime.Seconds;
        break; 
        case tp1_BAT_STS:
          u8len = 5;
          u32tmp = TP2101ReqBatSTS_1A();
          *u8ptr++ = tp1_BAT_STS;
          *u8ptr++ =  u32tmp & 0xff;
          *u8ptr++ =  (u32tmp >> 0x08) & 0xff;          
          *u8ptr++ =  (u32tmp >> 0x10) & 0xff;
          *u8ptr++ =  u32tmp >> 0x18;
        break; 
        case tp1_BMS_STS:
          u8len = 5;
          u32tmp = TP2101ReqBmsSTS_1B();
          *u8ptr++ = tp1_BMS_STS;
          *u8ptr++ =  u32tmp & 0xff;
          *u8ptr++ =  (u32tmp >> 0x08) & 0xff;
          *u8ptr++ =  (u32tmp >> 0x10) & 0xff;
          *u8ptr++ =  u32tmp >> 0x18;
        break; 
        case tp1_BMS_PROT:
          u8len = 5;
          *u8ptr++ = tp1_BMS_PROT;
          u32tmp = TP2101ReqBmsPRT_1C();
          *u8ptr++ =  u32tmp & 0xff;        
          *u8ptr++ =  (u32tmp >> 0x08) & 0xff;  
          *u8ptr++ =  (u32tmp >> 0x10) & 0xff;          
          *u8ptr++ =  u32tmp >> 0x18;
        break; 
        case tp1_FET_CTL:
          u8len = 2;
          *u8ptr++ = tp1_FET_CTL;
          u32tmp = bqDeviceGetSTS(fCSG);
          u32tmp |= bqDeviceGetSTS(fDSG) << 0x01;
          u32tmp |= bqDeviceGetSTS(fPCHG) << 0x02;
          *u8ptr++ =  u32tmp & 0xff;
        break; 
        case tp1_BAT_ASOC:
          u8len = 2;
          *u8ptr++ = tp1_BAT_ASOC;
          *u8ptr++ = bms_data.BAT_SOC /10;          
        break; 
        case tp1_PACK_INFO:
          u8len = 5;
          *u8ptr++ = tp1_PACK_INFO;
          *u8ptr++ =  g_sConfig.def_max_voltage & 0xff;  
          *u8ptr++ =  g_sConfig.def_max_voltage >> 0x08;           
          *u8ptr++ =  g_sConfig.DisChgCur_max_L1 & 0xff;
          *u8ptr++ =  g_sConfig.DisChgCur_max_L1 >> 0x08;           
        break; 
        case tp1_FCC:
          u8len = 3;
          u32tmp = CoulCountRetValue(CC_AVEL_CAP);
          u32tmp = u32tmp *10;
          *u8ptr++ = tp1_FCC;
          *u8ptr++ =  u32tmp >> 0x08;
          *u8ptr++ =  u32tmp & 0xff;
        break;
        default:
          nErr = true;
        break;
      }
      if((nErr != true) && (TP2101_flags.ESD != true)) {
        if(SendCANpalTxData(TP2101_REQINFO_ID, u8buf, u8len,CAN_TYPE_STD) != false) {
          nErr = true;
        }
      }
    }
    free(u8buf);
    return nErr;
}
/******************************************************************************
 * EOF
 *****************************************************************************/
