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
#ifndef BQMAXIMO_H
#define BQMAXIMO_H

#include <stdint.h>
#include <stdbool.h>


#define DISABLE_INT asm(" BIC #8,SR")

#define DELAY_LIMIT 0xffff


#define AFE_CELL_CH   16
#define AFE_SET_TMP    3
#define AFE_SORT_COUNT 5

#define CELL_BALANCE_DELTA 30

#define AFE_1ST_TMR_CLK 19
#define AFE_2ND_TMR_CLK 4

/* Application's AFE Interface Command initial state. */
#define AFE_DIRCMD_MTD false
#define AFE_SUBCMD_RTD true

typedef enum
{
	/* Application's state machine's initial state. */
	AFE_STATE_INIT = 0x00U,
	AFE_STATE_SERVICE_TASKS,
  AFE_STATE_SYSTEM_OFF
} afeState_t;


typedef enum {
	fCV = 0x00,
	fCC,
	fTM,
	fDSG,
	fCSG,
	fPCHG,
	fFET_ENB,
	fAFE_SHIP,
	fAFE_BALANCE,
} afe_status_t;

typedef enum {
	fAFE_ALART = 0x00U,
	fDEV_XD,
	fOVRDAL,
	fUV,
	fOV,
	fSCD,
	fOCD,
	fALL,
	fAFE_IRQ,
} afe_alart_t;

typedef enum {
	bqCell_Val = 0x00U,
	bqCell_MaxV,
	bqCell_MinV,
	bqCell_MaxT,
	bqCell_MinT,
	bqCell_AvgT,
	bqCell_Tmp,
	bqBat_Stack,
	bqBat_Pack,
	bqBat_Cur,
        bqBat_CurAVG,
        bqBat_InMaxCur,
        bqBat_OtMaxCur,
        bqAFE_FetT,
        bqAFE_ALART,
        bqAFE_PFAB,
        bqAFE_PFCD,
        bqAFE_FETSTS,
        bqAFE_BATSTS,
        bqAFE_CTLSTS,
        bqAFE_SAFT_A,
        bqAFE_SAFT_B,
        bqAFE_SAFT_C,        
} afe_ValIdx_t;

typedef struct {
	uint16_t Vol[AFE_CELL_CH];
	uint16_t Temp[AFE_SET_TMP];
	uint8_t  MaxIdx[AFE_SORT_COUNT];
	uint8_t  MinIdx[AFE_SORT_COUNT];
	uint16_t MaxTemp;
	uint16_t MinTemp;
	uint16_t AvgTemp;
	uint16_t Cur;
        uint16_t RLInMxCur;
        uint16_t RLOuxCur;
  uint16_t LD;
  uint16_t Pack;
  uint16_t Stack;
} afe_cell_params_t;

typedef struct {
	uint16_t VREG18;
	uint16_t VSS;
	uint16_t MaxCellV;
	uint16_t MinCellV;
	uint16_t Bat_SumV;
	uint16_t CellTmp;
	uint16_t FetTmp;
	uint16_t MaxCellT;
	uint16_t MinCellT;
	uint16_t AvgCellT;
	uint16_t CC3Curr;
	uint16_t CC1Curr;
	uint16_t BatSTS;
	uint8_t  FET;
        uint8_t  CTLSTS;
} afe_pack_params_t;

typedef struct {
	uint8_t tmr_sec;
	uint8_t tmr_ms;
	uint8_t state;
} afe_repo_cfg_t;

typedef struct {
	bool IRQ;
	bool PWR_ON;  
	bool CV_DRY;
	bool CC_DRY;
	bool TM_DRY;

	bool TASK_1st;
	bool TASK_2nd;

	bool DSG_ON;
	bool CSG_ON;
	bool PCHG_ON;
	bool FET_ON;
	bool SHIP;
	bool BALANCE;

	bool ADSCAN;   // ADC CV Scan Done
	bool FULLSCAN; // ADC SYS Scan Done 
} afe_repo_flags_t;







typedef struct {
	uint16_t AltSTS;
	uint8_t  SaftA;
	uint8_t  SaftB;
	uint8_t  SaftC;
	uint8_t  PFA;
	uint8_t  PFB;
	uint8_t  PFC;
	uint8_t  PFD;
} afe_alart_flags_t;

typedef struct {
    uint8_t    index;
    uint16_t   value;
}compare_buf_t;

bool bqDeviceInitAFE(void);
bool bqDeviceFETdriverCTL(uint8_t nIdx, bool nAct);
void bqDeviceAlartTrig(uint8_t nIdx);
bool bqDeviceGetAlartFlags(uint8_t nIdx);
void bqDeviceSetSTS(uint8_t nIdx);
bool bqDeviceGetSTS(uint8_t nIdx);
void bqDeviceClearSTS(uint8_t nIdx);
uint16_t bqDeviceGetValue(uint8_t nIdx , uint8_t nCh);
bool bqDeviceProcessTask(void);

bool bqDirectCommand(uint16_t sCmd, uint16_t sBuf, uint8_t sLen, bool nTsk);
bool bqDirectRead(uint16_t sCmd, uint8_t *ptr, uint8_t sLen, bool nTsk);

/*
extern uint16_t bqDevice_GetValue(uint8_t idx);
extern void bqDeviceWake(bool active);


extern bool bqDevice_DsgFETdriver_Task(bool action);
extern bool bqDevice_ChgFETdriver_Task(bool action);
extern bool bqDevice_StatusFlag(uint8_t idx);
extern void bqDevice_StatusTrig(uint8_t idx);
extern bool bqDevice_StatusClear(uint8_t idx);
extern bool bqDevice_AlartFlag(uint8_t idx);
extern bool  bqDevice_AlartClear(uint8_t idx);
*/
#endif
