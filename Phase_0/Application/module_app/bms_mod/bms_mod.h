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

#ifndef BMS_MOD_H
#define	BMS_MOD_H

#include <stdbool.h>
#include <stdint.h>

#define BMS_1ST_TMR_CLK 1  // 100ms
#define BMS_2ND_TMR_CLK 4  // 500ms
#define BMS_3ND_TMR_CLK 4  // 1000ms
#define AFE_FAIL_TRIG   4  // trig 5times
#define BMS_IDLE_TMR    3600 // 15min * 60sec * 250ms
#define BMS_STBY_TMR    60   // 60 Sec
#define BMS_IDLE_CUR    20   // 200mA

#define MonRawH 1763   //X1
#define MonRawL  781   // X0
#define MonnRefA 6000  // Y1
#define MonRefB  2600  // Y0
#define MonScale 1000
#define MonOffset MonRefB
#define MonFactor (MonnRefA - MonRefB) / (MonRawH - MonRawL)

#define DELTA_V_ERR  1500  //15V

#define BMS_MODE_NONE   0x00
#define BMS_MODE_CHG    0x01
#define BMS_MODE_Ready  0x02
#define BMS_MODE_Enable 0x03
#define BMS_MODE_Error  0x04
#define BMS_MODE_Sleep  0x06
#define BMS_MODE_DSGEND 0x07
#define BMS_MODE_CHGEND 0x08
#define BMS_MODE_SHUTDOWN 0x09


#define BMS_CURRENT_OFFSET 0x1770

#define BMSMAX_PWR_LIMT  0x7D0

#define BMS_THERMO_THRESHOLD (35 + ThermistorOFFSET)
#define BMS_THERMO_RELEASE   (30 + ThermistorOFFSET)

typedef enum
{
	/* Application's state machine's initial state. */
    BMS_STATE_STBY = 0,
	BMS_STATE_READY,
	BMS_STATE_PDSG_TASKS,
	BMS_STATE_DSG_TASKS,
	BMS_STATE_DSGEND_TASKS,
	BMS_STATE_CHG_TASKS,
	BMS_STATE_CHGEND_TASKS,
        BMS_STATE_ERR_TASKS,
	BMS_STATE_SHUTDOWN_TASK,
	BMS_STATE_SYSTEM_OFF,
	BMS_STATE_NULL,
	/* TODO: Define states used by the application state machine. */
} bmsStatus_t;

typedef struct
{
	bmsStatus_t state;
} bmsProcess_t;

typedef enum {
	bmsBatVolt = 0x00,
	bmsPackVolt,
	bmsBATCurr,
	bmsBatSoc,
	bmsBatSoh,
	bmsChgStat,
	bmsStatus,
	bmsErr,
	bmsLife,
	bmsCellV_H,
	bmsCellV_L,
	bmsTemp_H,
	bmsTempL,
	bmsDsgPwrLimt,
	bmsChgPwrLimt,
	bmsCellTmpAVG,
	bmsTotalNRG,
	bmsTotalCap,
	bmsRateVolt,
	bmsSN,
	bmsCellNUM,
	bmsBatKwhLift,
	bmsXchgSig,
	bmsSilV_Count,
	bmsTaskTMR,
}bmsValue_t;

typedef struct 
{
    bool BAT_VOLTAGE;
    bool CC2;
    bool BAT_CURRENT;
    bool SOC;
    bool SOH;
    bool BAT_KWH_LEFT;
    bool FET;
    bool CHG;
    bool SYS_WAKE;
    bool PWR_TRG;
    bool fPWR;
    bool fWAKE;
} BMS_VALUE_STATUS_t;


typedef struct 
{
    uint16_t BAT_VOLTAGE;
    uint16_t PACK_VOLTAGE;
    uint16_t BAT_CURRENT;
    uint16_t BAT_SOC;
    uint16_t BAT_SOH;
    uint8_t CHG_STATUS;
    uint8_t BMS_STATUS;
    uint8_t ERROR;
    uint8_t LIFE;
    
    uint16_t SYS_CELL_V_MAX;
    uint16_t SYS_CELL_V_MIN;
    uint16_t SYS_CELL_T_MAX;
    uint16_t SYS_CELL_T_MIN;
    uint16_t DELTA_BAT_PACK;
    uint16_t MAX_DSG_PWR_LIMT;
    uint16_t MAX_CHG_PWR_LIMT;
    uint16_t CELL_TMP_AVG;
    uint16_t TOTAL_ENERGY;
    uint16_t TOTAL_CAPACITY;
    uint16_t RATED_VOLTAGE;
    uint8_t SERIAL_NUM;
    uint8_t CELL_NUM;
    uint16_t BAT_KWH_LEFT;
    uint8_t xChg_Signal;
    uint8_t SLI_V_COUNT;
    uint16_t TASK_TIMER;

} BMS_Parameters_t;


typedef struct {
	bool TASK_1st;
	bool TASK_2nd;
	bool TASK_3rd;
} bms_status_flag_t;

typedef struct {
	uint8_t tmr_1st;
	uint8_t tmr_2nd;
	uint8_t tmr_3rd;
    uint8_t tmr_even;
	uint8_t afe_trig;
	uint16_t chg_event;
	uint16_t idel_tmr;
	uint8_t cal_packV;
} bms_config_info_t;

extern uint16_t BMS_GetDataValue(uint8_t idx);
extern BMS_VALUE_STATUS_t bms_value_status;
extern BMS_Parameters_t   bms_data;

extern void BMS_ProcessInitial(void);
extern void BMS_MainProcessTask(void);
extern uint8_t BMS_ReturnStatus(void);
extern void BMS_PMuxProcessTask(uint8_t idx);

    
#ifdef	__cplusplus
}
#endif

#endif	/* BMS_MOD_H */
