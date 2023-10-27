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
#include "module_app/bms_mod/err_mod.h"

#include "module_app/flash_mod/flash_mod.h"
#include "module_app/bqMaximo/bqMaximo.h"
#include "module_app/CoulCount/CoulCount.h"

static void ERR_BmsProcessTask (void);
static void ERR_TempProcessTask (void);
static void ERR_CellVProcessTask(void);
static void ERR_BatVProcessTask(void);
static void ERR_BatVProcessTask (void);
static void ERR_CurrProcessTask (void);
static void ERR_SocProcessTask(void);
static void ERR_CompoundProcessTask(void);
BMS_ERROR_ALARM_t  bms_error_alarm;
extern bms_config_info_t bms_config_info;
extern bmsProcess_t bmsProcess;
void ERR_ProcessInitial(void) {
    
    bms_error_alarm.Temp_High = false;
    bms_error_alarm.Temp_Low = false;
    bms_error_alarm.Temp_Diff = false;
    bms_error_alarm.CellV_Diff = false;
    bms_error_alarm.CellV_High = false;
    bms_error_alarm.CellV_Low = false;
    bms_error_alarm.BatV_High = false;
    bms_error_alarm.BatV_Low = false;
    bms_error_alarm.ChgCur = false;
    bms_error_alarm.DisCur = false;
    bms_error_alarm.SOC_High = false;
    bms_error_alarm.SOC_Low = false;
    bms_error_alarm.Open_subcircuit = false;
    bms_error_alarm.MSD = false;
    bms_error_alarm.smoke = false;
    bms_error_alarm.charger_inlerT = false;
    bms_error_alarm.Balance = false;
    bms_error_alarm.charger = false;
    bms_error_alarm.vcc_low = false;
    bms_error_alarm.current_mod = false;
    bms_error_alarm.isolation_mod = false;
    bms_error_alarm.LECU_lose = false;
    bms_error_alarm.VCU_lose = false;
    bms_error_alarm.prechg = false;
    bms_error_alarm.P1_relay = false;
    bms_error_alarm.Chg_relay = false;
    bms_error_alarm.Heat_relay = false;
    bms_error_alarm.soc_skip = false;
    bms_error_alarm.REES_mismatching = false;
    bms_error_alarm.Cell_inconformity = false;
    bms_error_alarm.water_cooling = false;
    bms_error_alarm.over_chg = false;
    bms_error_alarm.acchg_relay = false;
    bms_error_alarm.chg_err = false;
    bms_error_alarm.compound = false;
    bms_error_alarm.err_trig = false;
    bms_error_alarm.err_lock = false;
}


void ERR_SetErrorFlag(uint8_t idx) {

    switch (idx) {
    case ErrPRECHG:
            bms_error_alarm.prechg = true;
    }
}

void ERR_ProcessTask(void) {
    ERR_BmsProcessTask();
    if(bms_value_status.PWR_TRG == true) {
        ERR_TempProcessTask();
        ERR_CellVProcessTask();
        ERR_BatVProcessTask();
        ERR_CurrProcessTask ();
        ERR_SocProcessTask();
        ERR_CompoundProcessTask();
    }
}


static void ERR_TempProcessTask (void) {
    uint16_t u16tmp, u16max, u16min;
      if(bms_data.SYS_CELL_T_MAX & 0x8000) {
        u16max = 0xffff - bms_data.SYS_CELL_T_MAX;    
} else {
  u16max = bms_data.SYS_CELL_T_MAX;
}
    if(u16max > g_sConfig.DSG_Temp_H_L1) {
        if(u16max > g_sConfig.DSG_Temp_H_L2) {
            if(u16max >= g_sConfig.DSG_Temp_H_L3) {
                bms_error_alarm.Temp_High = LEVEL_3_ALARM;
            }
            else {
                bms_error_alarm.Temp_High = LEVEL_2_ALARM;
            }
        }
        else {
            bms_error_alarm.Temp_High = LEVEL_1_ALARM;            
        }
    }
    else {
        bms_error_alarm.Temp_High  = false;
    }
    /******************************************/
      if(bms_data.SYS_CELL_T_MIN & 0x8000) {
        u16min = 0xffff - bms_data.SYS_CELL_T_MIN;
} else {
  u16min = bms_data.SYS_CELL_T_MIN;
}
    if(u16min < g_sConfig.DSG_Temp_L_L1) {
        if(u16min < g_sConfig.DSG_Temp_L_L2) {
            if(u16min < g_sConfig.DSG_Temp_L_L3) {
                bms_error_alarm.Temp_Low = LEVEL_3_ALARM;
            }
            else {
                bms_error_alarm.Temp_Low = LEVEL_2_ALARM;
            }
        }
        else {
            bms_error_alarm.Temp_Low = LEVEL_1_ALARM;           
        }
    }
    else {
        bms_error_alarm.Temp_Low  = false;
    }
    /******************************************/
    u16tmp = u16max - u16min;
    if(u16tmp > g_sConfig.Temp_Diff_L1) {
        if(u16tmp > g_sConfig.Temp_Diff_L2) {
            if(u16tmp > g_sConfig.Temp_Diff_L3) {
                bms_error_alarm.Temp_Diff = LEVEL_3_ALARM;
            }
            else {
                bms_error_alarm.Temp_Diff = LEVEL_2_ALARM;
            }
        }
        else {
            bms_error_alarm.Temp_Diff = LEVEL_1_ALARM;            
        }
    }
    else {
        bms_error_alarm.Temp_Diff  = false;
    }
}

static void ERR_CellVProcessTask (void) {
   uint16_t u16tmp;
    if(bms_data.SYS_CELL_V_MAX > g_sConfig.Cell_high_L1) {
        if(bms_data.SYS_CELL_V_MAX > g_sConfig.Cell_high_L2) {
            if(bms_data.SYS_CELL_V_MAX >= g_sConfig.Cell_high_L3) {
                bms_error_alarm.CellV_High = LEVEL_3_ALARM;
            }
            else {
                bms_error_alarm.CellV_High = LEVEL_2_ALARM;
            }
        }
        else {
            bms_error_alarm.CellV_High = LEVEL_1_ALARM;            
        }
    }
    else {
        bms_error_alarm.CellV_High  = false;
    }
    /******************************************/
    if(bms_data.SYS_CELL_V_MIN < g_sConfig.Cell_low_L1) {
        if(bms_data.SYS_CELL_V_MIN < g_sConfig.Cell_low_L2) {
            if(bms_data.SYS_CELL_V_MIN < g_sConfig.Cell_low_L3) {
                bms_error_alarm.CellV_Low = LEVEL_3_ALARM;
            }
            else {
                bms_error_alarm.CellV_Low = LEVEL_2_ALARM;
            }
        }
        else {
            bms_error_alarm.CellV_Low = LEVEL_1_ALARM;           
        }
    }
    else {
        bms_error_alarm.CellV_Low  = false;
    }
    /******************************************/
    u16tmp = bms_data.SYS_CELL_V_MAX - bms_data.SYS_CELL_V_MIN;
    if(u16tmp > g_sConfig.CellV_Diff_L1) {
        if(u16tmp > g_sConfig.CellV_Diff_L2) {
            if(u16tmp > g_sConfig.CellV_Diff_L3) {
                bms_error_alarm.CellV_Diff = LEVEL_3_ALARM;
            }
            else {
                bms_error_alarm.CellV_Diff = LEVEL_2_ALARM;
            }
        }
        else {
            bms_error_alarm.CellV_Diff = LEVEL_1_ALARM;            
        }
    }
    else {
        bms_error_alarm.CellV_Diff  = false;
    }
    if(bqDeviceGetAlartFlags(fUV) == true) {
        bms_error_alarm.CellV_Low = LEVEL_3_ALARM; 
    }
    if(bqDeviceGetAlartFlags(fOV) == true) {
        bms_error_alarm.CellV_High = LEVEL_3_ALARM; 
    }    
}

static void ERR_BatVProcessTask (void) {
    if(bms_data.BAT_VOLTAGE > g_sConfig.BatV_max_L1) {
        if(bms_data.BAT_VOLTAGE > g_sConfig.BatV_max_L2) {
            if(bms_data.BAT_VOLTAGE >= g_sConfig.BatV_max_L3) {
                bms_error_alarm.BatV_High = LEVEL_3_ALARM;
            }
            else {
                bms_error_alarm.BatV_High = LEVEL_2_ALARM;
            }
        }
        else {
            bms_error_alarm.BatV_High = LEVEL_1_ALARM;            
        }
    }
    else {
        bms_error_alarm.BatV_High  = false;
    }
    /******************************************/
    if(bms_data.BAT_VOLTAGE < g_sConfig.BatV_min_L1) {
        if(bms_data.BAT_VOLTAGE < g_sConfig.BatV_min_L2) {
            if(bms_data.BAT_VOLTAGE < g_sConfig.BatV_min_L3) {
                bms_error_alarm.BatV_Low = LEVEL_3_ALARM;
            }
            else {
                bms_error_alarm.BatV_Low = LEVEL_2_ALARM;
            }
        }
        else {
            bms_error_alarm.BatV_Low = LEVEL_1_ALARM;           
        }
    }
    else {
        bms_error_alarm.BatV_Low  = false;
    }
}


static void ERR_CurrProcessTask (void) {
    uint16_t u16tmp;
    if(bms_data.BAT_CURRENT > 0x8000) {
        u16tmp = 0x10000 - bms_data.BAT_CURRENT;
        if(u16tmp > g_sConfig.ChgCur_max_L1) {
            if(u16tmp > g_sConfig.ChgCur_max_L2) {
                if(u16tmp >= g_sConfig.ChgCur_max_L3) {
                    bms_error_alarm.ChgCur = LEVEL_3_ALARM;
                    bms_error_alarm.err_lock = true;
                }
                else {
                    bms_error_alarm.ChgCur = LEVEL_2_ALARM;
                }
            }
            else {
                bms_error_alarm.ChgCur = LEVEL_1_ALARM;            
            }
        }
        else {
            bms_error_alarm.ChgCur  = false;
        }
    }
    else {
        if(bms_data.BAT_CURRENT > g_sConfig.DisChgCur_max_L1) {
            if(bms_data.BAT_CURRENT > g_sConfig.DisChgCur_max_L2) {
                if(bms_data.BAT_CURRENT > g_sConfig.DisChgCur_max_L3) {
                    bms_error_alarm.DisCur = LEVEL_3_ALARM;
                    bms_error_alarm.err_lock = true;
                }
                else {
                    bms_error_alarm.DisCur = LEVEL_2_ALARM;
                }
            }
            else {
                bms_error_alarm.DisCur = LEVEL_1_ALARM;           
            }
        }
        else {
            bms_error_alarm.DisCur  = false;
        }
    }
    if((bqDeviceGetAlartFlags(fSCD) == true) || (bqDeviceGetAlartFlags(fOCD) == true) || (bms_error_alarm.err_lock == true)) {
        bms_error_alarm.DisCur = LEVEL_3_ALARM;
    }
}

static void ERR_BmsProcessTask (void) {
    if((Flash_GetStatus(FL_ERR) == true) || (bms_config_info.afe_trig > AFE_FAIL_TRIG)) {
        bms_data.ERROR = LEVEL_3_ALARM;
    }
    else {
    	bms_data.ERROR = NO_ERROR_ALARM;
    }
    if(bqDeviceGetAlartFlags(fAFE_ALART) == true) {
        bms_data.ERROR = LEVEL_1_ALARM;
    } else {
    	bms_data.ERROR = NO_ERROR_ALARM;
    }
}

static void ERR_SocProcessTask(void) {
   if(bms_data.BAT_SOC < g_sConfig.soc_low_L1) {
        if(bms_data.BAT_SOC < g_sConfig.soc_low_L2) {
            if(bms_data.BAT_SOC <= g_sConfig.soc_low_L3) {
                bms_error_alarm.SOC_Low = LEVEL_3_ALARM;
            }
            else {
                bms_error_alarm.SOC_Low = LEVEL_2_ALARM;
            }
        }
        else {
            bms_error_alarm.SOC_Low = LEVEL_1_ALARM;            
        }
    }
    else {
        bms_error_alarm.SOC_Low  = false;
    }
}

static void ERR_CompoundProcessTask(void) {
    uint8_t u8tmp;
    u8tmp = false;
    if(bms_data.ERROR > u8tmp) {
        u8tmp = bms_data.ERROR;
    }
    if(bms_error_alarm.Temp_High > u8tmp) {
    	u8tmp = bms_error_alarm.Temp_High;
    }
    if(bms_error_alarm.Temp_Low > u8tmp) {
    	u8tmp = bms_error_alarm.Temp_Low;
    }
    if(bms_error_alarm.Temp_Diff > u8tmp) {
    	u8tmp = bms_error_alarm.Temp_Diff;
    }        
    if(bms_error_alarm.CellV_High > u8tmp) {
    	u8tmp = bms_error_alarm.CellV_High;
    }
    if(bms_error_alarm.CellV_Low > u8tmp) {
    	u8tmp = bms_error_alarm.CellV_Low;
    }
    if(bms_error_alarm.CellV_Diff > u8tmp) {
    	u8tmp = bms_error_alarm.CellV_Diff;
    }
    if(bms_error_alarm.BatV_High > u8tmp) {
    	u8tmp = bms_error_alarm.BatV_High;
    }    
    if(bms_error_alarm.BatV_Low > u8tmp) {
    	u8tmp = bms_error_alarm.BatV_Low;
    }
    if(bms_error_alarm.ChgCur > u8tmp) {
    	u8tmp = bms_error_alarm.ChgCur;
    }
    if(bms_error_alarm.DisCur > u8tmp) {
    	u8tmp = bms_error_alarm.DisCur;
    }
    if(bms_error_alarm.SOC_Low > u8tmp) {
    	u8tmp = bms_error_alarm.SOC_Low;
    }
    if(u8tmp != NO_ERROR_ALARM) {
        if(bms_error_alarm.err_trig > 2) {
            bms_error_alarm.compound = u8tmp;
        }
    	else {
            bms_error_alarm.err_trig++;
    	}
    }
    else {
        bms_error_alarm.err_trig = false;
        bms_error_alarm.compound = NO_ERROR_ALARM;
    }
}

uint8_t BMS_ERR_COMPOUND(void) {
	return bms_error_alarm.compound;
}
