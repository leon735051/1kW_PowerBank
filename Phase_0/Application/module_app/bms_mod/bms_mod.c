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
#include "module_app/bms_mod/bms_mod.h"
#include "module_app/bqMaximo/bqMaximo.h"
#include "module_app/CoulCount/CoulCount.h"
#include "err_mod.h"
#include "module_app/flash_mod/flash_mod.h"
#include "driver_bsp/PortIRQ_bsp.h"
#include "driver_bsp/PortMux_bsp.h"
#include "driver_bsp/rtc_bsp.h"
#include "module_app/protocol_stack/tp2101_stack.h"
/******************************************************************************
 * Definitions
 ******************************************************************************/

BMS_VALUE_STATUS_t bms_value_status;
BMS_Parameters_t   bms_data;
bmsProcess_t bmsProcess;
bms_status_flag_t bms_status_flag;
bms_config_info_t bms_config_info;

/******************************************************************************
 * Function prototypes
 ******************************************************************************/

static void BMS_ProcessTimer_Task(void);
static void BMS_FET_DriverTask(void);
static void BMS_CURR_Info_Calue(void);
static void BMS_Main_Info_Calue(void);

void BMS_ProcessInitial(void) {    
    bms_value_status.BAT_VOLTAGE = false;
    bms_value_status.CC2 = false;
    bms_value_status.BAT_CURRENT = false;
    bms_value_status.SOC = false;
    bms_value_status.SOH = false;
    bms_value_status.BAT_KWH_LEFT = false;
    bms_value_status.SOH = false;
    bms_value_status.BAT_KWH_LEFT = false;
    bms_value_status.SYS_WAKE = false;
    bms_value_status.PWR_TRG = false;
    bms_value_status.CHG = false;
    
    bms_value_status.fWAKE = false;
    bms_value_status.FET = false;
             
    bms_data.BAT_VOLTAGE = false;
    bms_data.BAT_CURRENT = false;
    bms_data.BAT_SOC = g_sConfig.soc;
    bms_data.BAT_SOH = g_sConfig.soh;
    bms_data.CHG_STATUS = false;
    bms_data.BMS_STATUS = false;
    bms_data.ERROR = false;
    bms_data.LIFE = false;
    
    bms_data.SYS_CELL_V_MAX = false;
    bms_data.SYS_CELL_V_MIN = false;
    bms_data.SYS_CELL_T_MAX = false;
    bms_data.SYS_CELL_T_MIN = false;
    bms_data.DELTA_BAT_PACK = false;
    bms_data.MAX_DSG_PWR_LIMT = false;
    bms_data.MAX_CHG_PWR_LIMT = false;
    bms_data.CELL_TMP_AVG = false;
    bms_data.BAT_KWH_LEFT = false;
    bms_data.xChg_Signal = false;
    bms_data.SLI_V_COUNT = false;
    bms_data.TASK_TIMER = false;
    
    bms_data.TOTAL_ENERGY  = (g_sConfig.aval_capacity * g_sConfig.def_rate_voltage) /10000;
    bms_data.TOTAL_CAPACITY  = g_sConfig.aval_capacity;
    bms_data.RATED_VOLTAGE = g_sConfig.def_rate_voltage;
    
    bms_data.SERIAL_NUM = 0xa0;
    bms_data.CELL_NUM = 0x0e;

    bms_config_info.tmr_1st = false;
    bms_config_info.tmr_2nd = false;
	bms_config_info.tmr_3rd = false;
	bms_config_info.afe_trig = false;
	bms_config_info.chg_event = false;
	bms_config_info.idel_tmr = false;
        bms_config_info.tmr_even = false;
	bms_status_flag.TASK_1st = false;
	bms_status_flag.TASK_2nd = false;
	bms_status_flag.TASK_3rd = false;
        bmsProcess.state = BMS_STATE_STBY;
}

static void BMS_ProcessTimer_Task(void) {
    if(bms_config_info.tmr_1st >= BMS_1ST_TMR_CLK) {
        bms_config_info.tmr_1st = false;
        bms_status_flag.TASK_1st = true;
    } else {
        bms_config_info.tmr_1st++;
    }
    if(bms_status_flag.TASK_1st == true) {
        if(bms_config_info.tmr_2nd >= BMS_2ND_TMR_CLK ) {
            bms_config_info.tmr_2nd = false;
            bms_status_flag.TASK_2nd = true;
        } else {
            bms_config_info.tmr_2nd++;
        }
        if(bms_config_info.tmr_3rd >= BMS_3ND_TMR_CLK) {
            bms_config_info.tmr_3rd = false;
            bms_status_flag.TASK_3rd = true;
        } else {
            bms_config_info.tmr_3rd++;
        }
    }
}

static void BMS_FET_DriverTask(void) {
    if(bms_value_status.FET == true) {
        if(PortMux_OutStatus(POUT_AFE_DSG) == true) {
            PortMux_OutDisable(POUT_AFE_DSG);
        }
        if(bqDeviceGetSTS(fFET_ENB) != true) {
            bqDeviceFETdriverCTL(fFET_ENB, true);
        }
    } else {
        if(bqDeviceGetSTS(fFET_ENB) != false) {
            bqDeviceFETdriverCTL(fFET_ENB, false);
            PortMux_OutEnable(POUT_AFE_DSG);
        }
    }
}

void BMS_MainProcessTask(void) {
    BMS_ProcessTimer_Task();
    if(bqDeviceGetSTS(fCC) == true) {
        BMS_CURR_Info_Calue();
        bqDeviceClearSTS(fCC);
    }
    if(bqDeviceGetSTS(fCV) == true) {
        BMS_Main_Info_Calue();
    	bqDeviceClearSTS(fCV);
    }
    
    if(bms_status_flag.TASK_1st == true) {  // 100mS
        ERR_ProcessTask();
        bms_config_info.tmr_even++;
    }
    if(bms_status_flag.TASK_2nd == true) { // 500ms
        if(BMS_ERR_COMPOUND() >= LEVEL_3_ALARM) {
            if(bmsProcess.state != BMS_STATE_SHUTDOWN_TASK) {
                if(bmsProcess.state != BMS_STATE_ERR_TASKS) {
                    bms_config_info.idel_tmr = false;
                    bmsProcess.state = BMS_STATE_ERR_TASKS;
                }
            }
        }
        if(TP2101GetFlags(tp_FLG_FACTORY) == true) {
          bmsProcess.state = BMS_STATE_ERR_TASKS;
        } else if(TP2101GetFlags(tp_FLG_BLT) == true) {
          if(bmsProcess.state < BMS_STATE_SHUTDOWN_TASK) {
            bmsProcess.state = BMS_STATE_SHUTDOWN_TASK;
          }
        }
    	bms_config_info.afe_trig++;
    	bms_status_flag.TASK_2nd = false;
    }
    if(bms_status_flag.TASK_3rd == true) { //1000ms
        bms_config_info.idel_tmr++;
        bms_config_info.chg_event++;
    }
    
    if(PortIrq_PMuxFlags(PIRQ_WAKE_SIG) == false) {
      bms_value_status.CHG = true;
    } else {
      bms_value_status.CHG = false;
    }

   
    switch (bmsProcess.state) {
    case BMS_STATE_STBY:
        if(bms_value_status.PWR_TRG == false) {
            ERR_ProcessInitial();
            bms_value_status.PWR_TRG = true;
            bms_data.BMS_STATUS = BMS_MODE_Ready;
            bmsProcess.state = BMS_STATE_READY;
            bms_config_info.tmr_even = false;
        }
    break;
    case BMS_STATE_READY:
      if(bms_value_status.CHG == true) {
        bmsProcess.state = BMS_STATE_CHG_TASKS;
      } else {
        bms_config_info.tmr_even = false;
        bms_value_status.FET = true;
        bms_data.BMS_STATUS = BMS_MODE_Enable;
        bmsProcess.state = BMS_STATE_PDSG_TASKS;
      }
        break;
    case BMS_STATE_PDSG_TASKS:
        if(bms_config_info.tmr_even > 0x08) {
          if (bms_data.PACK_VOLTAGE < DELTA_V_ERR) {
                ERR_SetErrorFlag(ErrPRECHG);
                bmsProcess.state = BMS_STATE_ERR_TASKS;
            } else {
                bmsProcess.state = BMS_STATE_DSG_TASKS;
            }
        }
        break;
    case BMS_STATE_DSG_TASKS:
        if(bms_data.BAT_SOC == false) {
            bms_value_status.FET = false;
            bms_data.BMS_STATUS = BMS_MODE_DSGEND;
            bmsProcess.state = BMS_STATE_DSGEND_TASKS;
        } else if(bms_value_status.CHG == true) {
            bms_config_info.chg_event = false;
            bmsProcess.state = BMS_STATE_READY;
        }
        bms_config_info.tmr_even = false;
        break;
    case BMS_STATE_DSGEND_TASKS:
        if(bms_config_info.tmr_even > 0x08){
            bmsProcess.state = BMS_STATE_SHUTDOWN_TASK;
        }
        break;
    case BMS_STATE_CHG_TASKS:
        bms_config_info.tmr_even = false;
        if(bms_value_status.CHG == false) {
          bmsProcess.state = BMS_STATE_CHGEND_TASKS;
        } else if (bms_config_info.chg_event <= 28800) {
          if((bms_data.BAT_SOC >= 1000) || (bms_config_info.idel_tmr >= 0x0a)) {
           bmsProcess.state = BMS_STATE_CHGEND_TASKS;
           bms_data.BMS_STATUS = BMS_MODE_CHGEND;
           bms_value_status.FET = false;
         } else {
         bms_data.BMS_STATUS = BMS_MODE_CHG;
         }
        } else {
            bmsProcess.state = BMS_STATE_CHGEND_TASKS;
            bms_data.BMS_STATUS = BMS_MODE_CHGEND;
       }
        break;
    case BMS_STATE_CHGEND_TASKS:
        if(bms_config_info.tmr_even > 0x09) {
            if(bqDeviceGetSTS(fAFE_BALANCE) == true) {
                bqDeviceClearSTS(fAFE_BALANCE);
            }
            g_sConfig.LCHG_DATE = RTC_RetrunDate();
            g_sConfig.LCHG_TIME = RTC_RetrunTime();
            bmsProcess.state = BMS_STATE_READY;
        }
        break;
    case BMS_STATE_ERR_TASKS:
        bms_data.BMS_STATUS = BMS_MODE_Error;
        if(bqDeviceGetSTS(fAFE_SHIP) == true) {
          bmsProcess.state = BMS_STATE_SHUTDOWN_TASK;
        }
        if(TP2101GetFlags(tp_FLG_FACTORY) == false) {
            if(bms_config_info.idel_tmr >= BMS_STBY_TMR) {
                bmsProcess.state = BMS_STATE_SHUTDOWN_TASK;
            } else if(BMS_ERR_COMPOUND() == false) {
                bmsProcess.state = BMS_STATE_READY;
            }
        }
        break;
    case BMS_STATE_SHUTDOWN_TASK:
        bms_value_status.FET = false;
        bms_data.BMS_STATUS = BMS_MODE_Sleep;
        bqDeviceSetSTS(fAFE_SHIP);
     //   if(bqDeviceGetSTS(fFET_ENB) == false) {
            bmsProcess.state = BMS_STATE_SYSTEM_OFF;
      //      bms_value_status.PWR_TRG = false;
    //}
        break;
    case BMS_STATE_SYSTEM_OFF:
        bms_data.BMS_STATUS = BMS_MODE_SHUTDOWN;
        bmsProcess.state = BMS_STATE_NULL;
        break;
    case BMS_STATE_NULL:
        if(bms_value_status.CHG == true) {
            bmsProcess.state = BMS_STATE_STBY;
        }
        break;
    default: break;
    }
    BMS_FET_DriverTask();
    bms_status_flag.TASK_1st = false;
    bms_status_flag.TASK_2nd = false;
    bms_status_flag.TASK_3rd = false;
}

static void BMS_CURR_Info_Calue(void) {
	uint16_t u16tmp;
	u16tmp = bqDeviceGetValue(bqBat_Cur, false);
	if((u16tmp & 0x8000) == 0x8000) {    // set to discharge mode
		u16tmp = (0xffff - u16tmp) / 10;
		bms_data.BAT_CURRENT = u16tmp;
	} else {
		u16tmp = u16tmp / 10;
		bms_data.BAT_CURRENT = 0xffff - u16tmp;  // set to charge mode
	}
	if(u16tmp < BMS_IDLE_CUR) {
		if(bms_config_info.idel_tmr >= BMS_IDLE_TMR) {
                  bms_config_info.idel_tmr = false;
//			bmsProcess.state = BMS_STATE_SHUTDOWN_TASK;
                } else {
                  bms_config_info.idel_tmr++;
                }
	} else {
		bms_config_info.idel_tmr = false;
	}
	bms_value_status.BAT_CURRENT = true;
}

static void BMS_Main_Info_Calue(void) {
    if(bqDeviceGetSTS(fTM) == true) {
        bms_data.SYS_CELL_T_MAX =  bqDeviceGetValue(bqCell_MaxT, false);
        bms_data.SYS_CELL_T_MIN =  bqDeviceGetValue(bqCell_MinT, false);
        bms_data.CELL_TMP_AVG = bqDeviceGetValue(bqCell_AvgT, false);
        bqDeviceClearSTS(fCV);
    }
    bms_data.SYS_CELL_V_MAX = bqDeviceGetValue(bqCell_MaxV, false);
    bms_data.SYS_CELL_V_MIN = bqDeviceGetValue(bqCell_MinV, false);
    bms_data.BAT_VOLTAGE = bqDeviceGetValue(bqBat_Stack, false);
    bms_data.PACK_VOLTAGE = bqDeviceGetValue(bqBat_Pack, false);
    if(TP2101GetFlags(tp_FLG_FACTORY) == false) {
      bms_data.BAT_SOC = CoulCountRetValue(CC_SOC);
    }
    bms_data.BAT_SOH = CoulCountRetValue(CC_SOH);
    bms_data.BAT_KWH_LEFT = CoulCountRetValue(CC_R_ENEGY) / 10;
    bms_data.TOTAL_CAPACITY = CoulCountRetValue(CC_AVEL_CAP);
    if(bms_data.BAT_VOLTAGE != false) {
        if(bms_data.PACK_VOLTAGE > bms_data.BAT_VOLTAGE) {
            bms_data.DELTA_BAT_PACK = bms_data.PACK_VOLTAGE -  bms_data.BAT_VOLTAGE;
        } else {
            bms_data.DELTA_BAT_PACK = bms_data.BAT_VOLTAGE - bms_data.PACK_VOLTAGE;
        }
        bms_value_status.BAT_VOLTAGE = true;
    	bms_config_info.afe_trig = false;
    }
    else {
        bms_value_status.BAT_VOLTAGE = false;
    }
    if(bms_error_alarm.compound >= LEVEL_1_ALARM) {
        if(bms_error_alarm.compound >= LEVEL_2_ALARM) {
            if(bms_error_alarm.compound == LEVEL_3_ALARM) {
                bms_data.MAX_CHG_PWR_LIMT = g_sConfig.MaxPwr_Limt_L3;
                bms_data.MAX_DSG_PWR_LIMT = g_sConfig.MaxPwr_Limt_L3;
            }
            else {
                bms_data.MAX_CHG_PWR_LIMT = g_sConfig.MaxPwr_Limt_L2;
                bms_data.MAX_DSG_PWR_LIMT = g_sConfig.MaxPwr_Limt_L2;
            }
        }
        else {
            bms_data.MAX_CHG_PWR_LIMT = g_sConfig.MaxPwr_Limt_L1;
            bms_data.MAX_DSG_PWR_LIMT = g_sConfig.MaxPwr_Limt_L1;
        }
    }
    else {
        bms_data.MAX_CHG_PWR_LIMT = BMSMAX_PWR_LIMT;
        bms_data.MAX_DSG_PWR_LIMT = BMSMAX_PWR_LIMT;
    }
}

uint16_t BMS_GetDataValue(uint8_t idx) {
	uint16_t u16tmp = false;
	switch(idx) {
	case bmsBatVolt: u16tmp =bms_data.BAT_VOLTAGE;  break;
	case bmsPackVolt: u16tmp =bms_data.PACK_VOLTAGE; break;
	case bmsBATCurr: u16tmp = bms_data.BAT_CURRENT; break;
	case bmsBatSoc: u16tmp =bms_data.BAT_SOC; break;
	case bmsBatSoh: u16tmp =bms_data.BAT_SOH; break;
	case bmsChgStat: u16tmp =bms_data.CHG_STATUS; break;
	case bmsStatus: u16tmp =bms_data.BMS_STATUS; break;
	case bmsErr: u16tmp =bms_data.ERROR; break;
	case bmsLife: u16tmp =bms_data.LIFE; break;
	case bmsCellV_H: u16tmp =bms_data.SYS_CELL_V_MAX; break;
	case bmsCellV_L: u16tmp =bms_data.SYS_CELL_V_MIN; break;
	case bmsTemp_H: u16tmp =bms_data.SYS_CELL_T_MAX; break;
	case bmsTempL: u16tmp =bms_data.SYS_CELL_T_MIN; break;
	case bmsDsgPwrLimt: u16tmp =bms_data.MAX_DSG_PWR_LIMT; break;
	case bmsChgPwrLimt: u16tmp =bms_data.MAX_CHG_PWR_LIMT; break;
	case bmsCellTmpAVG: u16tmp =bms_data.CELL_TMP_AVG; break;
	case bmsTotalNRG: u16tmp =bms_data.TOTAL_ENERGY; break;
	case bmsTotalCap: u16tmp =bms_data.TOTAL_CAPACITY; break;
	case bmsRateVolt: u16tmp =bms_data.RATED_VOLTAGE; break;
	case bmsSN: u16tmp =bms_data.SERIAL_NUM; break;
	case bmsCellNUM: u16tmp =bms_data.CELL_NUM; break;
	case bmsBatKwhLift: u16tmp =bms_data.BAT_KWH_LEFT; break;
	case bmsXchgSig: u16tmp =bms_data.xChg_Signal; break;
	case bmsSilV_Count: u16tmp =bms_data.SLI_V_COUNT; break;
	case bmsTaskTMR: u16tmp =bms_data.TASK_TIMER; break;
	default: break;
	}
	return u16tmp;
}

uint8_t BMS_ReturnStatus(void) {
	return bms_data.BMS_STATUS;
}

