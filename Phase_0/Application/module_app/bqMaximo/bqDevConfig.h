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
#ifndef BQDEVCONFIG_H
#define BQDEVCONFIG_H

typedef enum BQ_DEV_TYPE {

  bqSCD_THRESH_10mV   = 0x0,
  bqSCD_THRESH_20mV   = 0x01,
  bqSCD_THRESH_40mV   = 0x02,
  bqSCD_THRESH_60mV   = 0x03,
  bqSCD_THRESH_80mV   = 0x04,
  bqSCD_THRESH_100mV  = 0x05,
  bqSCD_THRESH_125mV  = 0x06,
  bqSCD_THRESH_150mV  = 0x07,
  bqSCD_THRESH_175mV  = 0x08,
  bqSCD_THRESH_200mV  = 0x09,
  bqSCD_THRESH_250mV  = 0x0A,
  bqSCD_THRESH_300mV  = 0x0B,
  bqSCD_THRESH_350mV  = 0x0C,
  bqSCD_THRESH_400mV  = 0x0D,
  bqSCD_THRESH_450mV  = 0x0E,
  bqSCD_THRESH_500mV  = 0x0F,

  bqCUV_VOLTUNIT  = 506,  // Unit: 50.6mV
  bqCOV_VOLTUNIT  = 506,  // Unit: 50.6mV
  bqCUV_DELYUNIT  = 33,   // Unit: 3.3ms
  bqOCC_VOLTUNIT  = 2,    // Unit: 2mV
  bqOCD_VOLTUNIT  = 2,    // Unit: 2mV
  bqSCD_DELAYUNIT = 15,   // Unit: 15uS


  bqSCD_DELAY_50us  = 0x0,
  bqSCD_DELAY_100us = 0x1,
  bqSCD_DEALY_200us = 0x2,
  bqSCD_DELAY_400us = 0x3,

  bqOCD_DEALY_10ms   = 0x00,
  bqOCD_DELAY_20ms   = 0x01,
  bqOCD_DELAY_40ms   = 0x02,
  bqOCD_DELAY_80ms   = 0x03,
  bqOCD_DELAY_160ms  = 0x04,
  bqOCD_DELAY_320ms  = 0x05,
  bqOCD_DELAY_640ms  = 0x06,
  bqOCD_DELAY_1280ms = 0x07,

  bqOCD_THRESH_17mV_8mV   = 0x00,
  bqOCD_THRESH_22mV_11mV  = 0x01,
  bqOCD_THRESH_28mV_14mV  = 0x02,
  bqOCD_THRESH_33mV_17mV  = 0x03,
  bqOCD_THRESH_39mV_19mV  = 0x04,
  bqOCD_THRESH_44mV_22mV  = 0x05,
  bqOCD_THRESH_50mV_25mV  = 0x06,
  bqOCD_THRESH_56mV_28mV  = 0x07,
  bqOCD_THRESH_61mV_31mV  = 0x08,
  bqOCD_THRESH_67mV_33mV  = 0x09,
  bqOCD_THRESH_72mV_36mV  = 0xA,
  bqOCD_THRESH_78mV_39mV  = 0xB,
  bqOCD_THRESH_83mV_42mV  = 0xC,
  bqOCD_THRESH_89mV_44mV  = 0xD,
  bqOCD_THRESH_94mV_47mV  = 0xE,
  bqOCD_THRESH_100mV_50mV = 0xF,

  bqUV_DELAY_1s  = 0x00,
  bqUV_DELAY_4s  = 0x01,
  bqUV_DELAY_8s  = 0x02,
  bqUV_DELAY_16s = 0x03,

  bqOV_DELAY_1s  = 0,
  bqOV_DELAY_2s  = 1,
  bqOV_DELAY_4s  = 2,
  bqOV_DELAY_8s  = 3,
}bq_dev_types;

typedef enum BQ_DEVCFG_VAL {
  bqCFG_POWER = 0x3d82,  
  bqCFG_REG12 = 0xfD,
  bqCFG_REG0 = 0x01,

  bqCFG_CFETOFF = 0x8e, // CFET Pin Config Set: CFETOFF /Active High
  bqCFG_DFETOFF = 0x8e, // DFET Pin Config Set: BOTHOFF /Active High
  bqCFG_ALERT = 0x2A,
  bqCFG_ALARMASK = 0xf82,

  bqCFG_TS1 = 0x07, // TS1 Config Set: OPT_0x01, FXN_0x07
  bqCFG_TS2 = 0x07, // TS2 Config Set: OPT_0x01, FXN_0x07
  bqCFG_TS3 = 0x07, // TS3 Config Set: OPT_0x01, FXN_0x07
  
  bqCFG_DDSG = 0x0f, // DDSG Pin Config Set: ADC,Thermo, NoneProt, 18K

  bqCFG_CellMod  = 0x81FF, // Vcell Mode Set: 10 Serial
  bqCFG_ENBProtA = 0x88, // Enable Protect_A Set: SCD, OCD1, OCC, COV,CUV
  bqCFG_ENBProtB = 0xf7, // Enable Protect_B Set: OTINT, OTD,OTC
  bqCFG_ENBProtC = 0xf6, // Enable Protect_C Set: OTINT, OTD,OTC
  bqCFG_ENBPfC   = 0x00, // Enable PF C.

  bqCFG_FETOPT   = 0x1f, // Fet Options
  bqCFG_PreDSGTmr = 0x32,  // Predischarge Timeout

  bqCFG_BALCFG  = 0x09, // Balance Config set:CB_RL, CB_CHG
  bqCFG_BALMiTmp = 0xfd, // Balance Min Cell Temp
  bqCFG_BALMxTmp = 0x35, // Balance Max Cell Temp
  bqCFG_BALIntTmp = 0x3c, // Balance Intel Chip Temp
  bqCFG_BALMiCV_CH = 0xED8, // Balance Min Cell V (Charge)
  bqCFG_BALMiDT_CH = 0x64, // Balance Min Delta (Charge)
  bqCFG_BALSTPDT_CH = 0x1f, // Balance Stop Delta(Charge)
  bqCFG_BALMiCV_RX = 0xED8, // Balance Min Cell V (Relax)
  bqCFG_BALMiDT_RX = 0x64, // Balance Min Delta (Relax)
  bqCFG_BALSTPDT_RX = 0x1f, // Balance Stop Delta (Relax)

  bqCFG_CUVTHR   = (2350 *10) / bqCUV_VOLTUNIT, // CUV Threshold Unit:mV
  bqCFG_CUVDELY  = (1000 * 10) / bqCUV_DELYUNIT, // CUV Delay Unit:ms
  bqCFG_CUVRECY  = (400  * 10) / bqCUV_VOLTUNIT, // Recovery Hystersis Unit: mV

  bqCFG_COVTHR   = (4300 *10) / bqCOV_VOLTUNIT, // COV Threshold Unit:mV
  bqCFG_COVDELY  = (1000 * 10) / bqCUV_DELYUNIT, // CUV Delay Unit:ms
  bqCFG_COVRECY  = (250  * 10) / bqCUV_VOLTUNIT, // Recovery Hystersis Unit: mV

  //bqCFG_OCCTHR   = 10         / bqOCC_VOLTUNIT, // OCC Threshold Unit: Unit:A

  bqCFG_OCCDELY = (500 * 10) / bqCUV_DELYUNIT, // OCC Delay Unit:ms
  bqCFG_PackTOSDT =  0xc8,   // Pack_TOS Delat

  bqCFG_OCD1THR  = 12, // OCD1 Threshold Unit:mA
  bqCFG_OCD1DELY  = (70 *10)  / bqCUV_DELYUNIT, // OCD1 Delay Unit:ms

  bqCFG_OCD2THR  = 16, // OCD2 Threshold Unit:mA
  bqCFG_OCD2DELY  = (30 *10)  / bqCUV_DELYUNIT, // OCD2 Delay Unit:ms

  bqCFG_OCD3THR  = 0xC567, // OCD3 Threshold Unit:mA
  bqCFG_OCD3DELY  = 3, // OCD3 Delay Unit:Sec

  bqCFG_SCDTHR   = bqSCD_THRESH_20mV,          // SCD Threshold
  bqCFG_SCDDELY  = 30,   // SCD Delay Unit:uS

  bqCFG_OTCTHR = 0x32, // OTC Threshold Unit:oC
  bqCFG_OTCDLELY = 3, // OTC Delay Unit: Sec
  bqCFG_OTCRECY = 0x28, // OTC Recovery Unit: oC

  bqCFG_OTDTHR = 0x43, // OTD Threshold Unit:oC
  bqCFG_OTDDLELY = 3, // OTD Delay Unit: Sec
  bqCFG_OTDRECY = 0x36, // OTD Recovery Unit: oC

  bqCFG_UTCTHR = 0x00, // UTC Threshold Unit:oC
  bqCFG_UTCDLELY = 3, // UTC Delay Unit: Sec
  bqCFG_UTCRECY = 0x08, // UTC Recovery Unit: oC
  bqCFG_CCGAIN = 1, // CC Gain Set

  bqCFG_UKST1 = 0x9696, // Unseal Key Step 1
  bqCFG_UKST2 = 0x9724, // Unseal Key Step 2
} bq_DeviceConfig_Val;

#endif