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
#ifndef TP2101_STACK_H
#define TP2101_STACK_H


/*! @file adConv_mod.h*/

/*******************************************************************************
 * Definitions
 ******************************************************************************/

#define TP2101_CMDID_NUM  4    
    
typedef enum TP2101_BusCmdID_enum {
 TP2101_REQINFO_ID = 0x9c,
 TP2101_REQDEG_ID   = 0x777,
 TP2101_REQWAK_ID   = 0x66A,
 TP2101_REQFAT_ID   = 0x779,
 TP2101_REQMODE_ID  = 0x66C,

 TP2101_INFOCMD_ID  = 0xc9,
 TP2101_WAKCMD_ID   = 0x666,
 TP2101_MODCMD_ID   = 0x667,
 TP2101_FATCMD_ID   = 0x668,
}TP2101_BusCmdID_enum;

/************************************************************
* STANDARD BITS
************************************************************/
#define BIT0                (0x0001)
#define BIT1                (0x0002)
#define BIT2                (0x0004)
#define BIT3                (0x0008)
#define BIT4                (0x0010)
#define BIT5                (0x0020)
#define BIT6                (0x0040)
#define BIT7                (0x0080)
#define BIT8                (0x0100)
#define BIT9                (0x0200)
#define BITA                (0x0400)
#define BITB                (0x0800)
#define BITC                (0x1000)
#define BITD                (0x2000)
#define BITE                (0x4000)
#define BITF                (0x8000)

typedef enum TP2101_ReqInfoCmdReg_enum { 
    tp1_MODEL_NAME = 0x00, // Model name
    tp1_BAT_SN,    // Battery serial number
    tp1_HW_VER,    // Hardware version
    tp1_SW_VER,    // Software version
    tp1_DATE_MAF,  // Date of manufacture
    tp1_PACK_PARM, // pack parameters 
    tp1_PACK_SPEC, // pack specification
    tp1_PACK_VOLT, //pack voltag
    tp1_PACK_CURR, // pack current
    tp1_MXIN_CURR, // Maximum pack input current
    tp1_MIOT_CURR, // Minimum pack output current
    tp1_CELL_VOLT, // Cell Voltage
    tp1_MAX_CELLV, //maximum cell voltage
    tp1_MIN_CELLV, // minimum cell voltage
    tp1_CELL_DELTA,// cell voltage difference 
    tp1_CELL_TEMP ,//cell temperature
    tp1_MAX_CELLT, // maximum cell temperature
    tp1_MIN_CELLT, // Minimum cell temperature
    tp1_FET_TEMP,  // CFET & DFET MOSFET temperature
    tp1_FCHG_CAP,  // cumulative charge capacity
    tp1_REMN_CAP,  // Remain capacity
    tp1_SOC,       // Battery state of charge
    tp1_SOH,       // Battery state of health
    tp1_CYCLE_CNT, // cycle count
    tp1_RTC_INFO,  // RTC infomation
    tp1_LST_EOCT,  // Last end of charge time
    tp1_BAT_STS,   // Battery status
    tp1_BMS_STS,   // BMS status flags
    tp1_BMS_PROT,  // BMS protection flags
    tp1_FET_CTL,   // FET Control 
    tp1_BAT_ASOC,  // Battery absolute state of charge
    tp1_PACK_INFO, // Pack information
    tp1_FCC,       // Full charge capacity

} TP2101_ReqInfoCmdReg_enum;

typedef enum TP2101_ModeSeltReg_enum { 
    tp1_Boot = 0x00,          // Booting Mode, Still in Initial
    tp1_Cbim = 0x01,          // C-Bim Mode
    tp1_Bootloader = 0x02,    // Bootloader Mode
    tp1_Factory = 0x03,       // Factory Mode
    tp1_Usr_Normal = 0x10,    // User Mode, Normal /
    tp1_Usr_Charge = 0x11,    // User Mode, Charging
    tp1_Usr_Discharge = 0x12, // User Mode, Discharging
    tp1_Usr_Err = 0x13,       // ser Mode, Error 
    tp1_Usr_Sleep = 0x14,     // User Other Mode
} TP2101_ModeSeltReg_enum;

typedef enum TP2101_ReqWakCmd_enum { 
    tp1_WAK_SYSWAKE = 0x39, // Wake Up System
    tp1_RUN_MODE = 0x00, // System Run Mode
} TP2101_ReqWakCmd_enum;

typedef enum TP2101_ReqModCmd_enum { 
    tp1_MOD_BLT = 0x34,  // Selected device enter bootloader mode
    tp1_MOD_FACT = 0x55, // Selected device enter factory mode
    tp1_MOD_SN = 0x02,   // Report alived model id and serial number
} TP2101_ReqModCmd_enum;

typedef enum TP2101_ReqFatCmd_enum {
  tp1_FAT_EXIT = 0x01, // Exit Lunched Mode
  tp1_FAT_SETRTC_INFO = 0x03, // Write TB2101 RTC information
  tp1_FAT_REQRTC_INFO = 0x04, // Query RTC information
  tp1_FAT_BARCOD_SN = 0x05,   // Write barcode serial number
  tp1_FAT_DATE_MAUF = 0x06,   // Write date of manufacture 
  tp1_FAT_ENB_ESD = 0x07,     // Enter ESD test mode
  tp1_FAT_DIS_ESD = 0x08,     // Exit ESD test mode
  tp1_FAT_ENB_PROT = 0x09,    // Protection_Enable
  tp1_FAT_DIS_PROT = 0x0a,
  tp1_FAT_ENB_BAL = 0x0b,     // Force enable/disable cell balance
  tp1_FAT_MOS_CTL = 0x0c,     // AFE MosFET Control
  tp1_FAT_AFE_SHUT = 0x0d,    // AFE Shutdown
  tp1_FAT_CLR_PF = 0x0e,      // Clear AFE PF
  tp1_FAT_RST_PF = 0x0f,      // Reset AFE PF
  tp1_FAT_SET_SOC = 0x10,     // Set Soc
} TP2101_ReqFatCmd_enum;

typedef enum TP2101_Flages_enum {
  tp_FLG_SYSWAKE = 0x00,
  tp_FLG_FACTORY,
  tp_FLG_BLT,
  tp_FLG_PROTENB,
} TP2101_Flages_enum;

typedef enum TP2101_Cfg_enum {
  tp_Cfg_RunMode = 0x00,
} TP2101_Cfg_enum;

typedef struct {
  uint8_t run_mode;
} TP2101_cfg_params_t;

typedef struct {
  bool SYSWAKE;
  bool BLT;
  bool FACTORY;
  bool PROTENB;
  bool ESD;
} TP2101_rpo_flags_t;

typedef union{
  uint16_t SUM;
  struct {
    uint16_t IDLE: 1;
    uint16_t CHG_INI: 1;
    uint16_t CHG_RDY: 1;
    uint16_t CHG_FAUT: 1;
    uint16_t DSG_INT: 1;
    uint16_t DSG_RDY: 1;
    uint16_t DSG_FAUT: 1;
    uint16_t SLEP: 1;
    uint16_t DEEP_SLEP: 1;
    uint16_t PROTECT: 1;
    uint16_t PF: 1;
    uint16_t TEST: 1;
    uint16_t RSV1: 1;
    uint16_t RSV2: 1;
    uint16_t RSV3: 1;
    uint16_t RSV4: 1;
  };
} TP2101_ReqBatSts_t;

typedef union{
  uint16_t SUM;
  struct {
    uint16_t PWR_SW: 1;
    uint16_t CHG_IN: 1;
    uint16_t CHG_RDY: 1;
    uint16_t CHG_BRK: 1;
    uint16_t DSG_RDY: 1;
    uint16_t IDLE: 1;
    uint16_t BAL: 1;
    uint16_t FULL_CHG: 1;
    uint16_t LOW_CAP: 1;
    uint16_t FD: 1;
    uint16_t C_FAULT: 1;
    uint16_t D_FAULT: 1;
    uint16_t CAN_ERR: 1;
    uint16_t PF: 1;
  };
} TP2101_ReqBmsSts_t;

typedef union{
  uint32_t SUM;
  struct {
    uint32_t OV: 1;
    uint32_t pfOV: 1;   
    uint32_t UV: 1;
    uint32_t pfUV: 1;      
    uint32_t IMB: 1;
    uint32_t pfIMB: 1;
    uint32_t COC: 1;
    uint32_t pfCOC: 1;
    uint32_t DOC: 1;
    uint32_t DOC2: 1;         
    uint32_t pfDOC: 1;  
    uint32_t SC: 1;   
    uint32_t pfSC: 1;     
    uint32_t RSV13: 1;     
    uint32_t RSV14: 1;     
    uint32_t RSV15: 1;
    uint32_t COT: 1;     
    uint32_t DOT: 1;    
    uint32_t pfOT: 1;     
    uint32_t CUT: 1;       
    uint32_t DUT: 1;    
    uint32_t pfUT: 1;       
    uint32_t CFETOT: 1;  
    uint32_t pfCFETOT: 1;  
    uint32_t DFETOT: 1;  
    uint32_t pfDFETOT: 1; 
    uint32_t pfCFETFAIL: 1;  
    uint32_t pfDFETFAIL: 1;
    uint32_t LowCapP: 1;
  };
} TP2101_ReqBmsPrt_t;
/*******************************************************************************
 * API
 ******************************************************************************/
void  TP2101ValueInitial(void);
bool TP2101GetFlags(uint8_t u8Idx);
bool TP2101SetRunMode(uint8_t u8idx);
uint8_t TP2101GetConfig(uint8_t u8Idx);
bool TP2101ReqWakProcess(uint32_t can_id, uint8_t *msg);
bool TP2101ReqModProcess(uint32_t can_id, uint8_t *msg);
bool TP2101ReqFacProcess(uint32_t can_id, uint8_t *msg);
bool TP2101ReqInfoProcess(uint32_t can_id, uint8_t *msg);

#if defined (__cplusplus)
extern "C" {
#endif

/*!
 * @name Converter
 * General functions.
 */
/*! @{*/


/*!
 * @brief sysLED Process Initial.
 *
 * @param[in] The sysLED initial process.
 * @return NONE
 */

#if defined (__cplusplus)
}
#endif

#endif /* SYSLED_MOD_H */
/*******************************************************************************
 * EOF
 ******************************************************************************/
