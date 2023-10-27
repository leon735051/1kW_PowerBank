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
#include "CoulCount.h"
#include "module_app/flash_mod/flash_mod.h"
#include "module_app/bms_mod/bms_mod.h"
#include "driver_bsp/rtc_bsp.h"
cc_Parameters cc_data;
cc_DATA_STS   cc_status;

bool CoulCountIDF(uint16_t current);
void CoulCountSOC(uint16_t voltage, uint16_t current);
void CoulCountSOH(uint16_t current);


void CoulCount_ProcessInit(void) {
    cc_status.CH_Update_RDY = g_sConfig.CH_Update_RDY;
    cc_status.DS_Update_RDY = g_sConfig.DS_Update_RDY;
    cc_status.RDY = false;
    
    cc_data.idf_timer = 0;
    cc_data.idf_current = 0;        
    cc_data.cycle_timer = 0;
    cc_data.full_count = 0;
    cc_data.cycle_cap = 0;
    cc_data.soc = g_sConfig.soc;
    cc_data.soh = g_sConfig.soh;
    
    cc_data.rem_energy = g_sConfig.rem_energy;
    cc_data.cycle_dis_capacity = g_sConfig.cycle_dis_capacity;
    cc_data.cycle_count = g_sConfig.cycle_count;
    cc_data.proc_chged = g_sConfig.RPO_CHD;

    if(g_sConfig.def_capacity == false) {
        g_sConfig.def_capacity = ((g_sConfig.def_energy * 1000) / g_sConfig.def_rate_voltage);
        cc_data.aval_capacity = g_sConfig.def_capacity;
        cc_data.rem_capacity  = g_sConfig.def_capacity / 2 ;
        cc_data.cycle_chg_capacity = g_sConfig.def_capacity;
        g_sConfig.aval_capacity = cc_data.aval_capacity;
        g_sConfig.LCHG_DATE.Year = 0x15;
        g_sConfig.LCHG_DATE.Month = 0x01;
        g_sConfig.LCHG_DATE.Date = 0x01;
    }
    else {
        cc_data.aval_capacity = g_sConfig.aval_capacity;
        cc_data.rem_capacity  = g_sConfig.rem_capacity;
        cc_data.cycle_chg_capacity = g_sConfig.cycle_chg_capacity;
    }
    cc_data.def_capacity = g_sConfig.def_capacity;
    cc_data.avel_energy = (cc_data.aval_capacity * g_sConfig.def_rate_voltage) / 10000;
    cc_data.cal_capacity =  cc_data.rem_capacity * milliTimeCycle;
    cc_data.cal_aval_capacity = cc_data.aval_capacity * milliTimeCycle;
    cc_data.cal_cycle_chg_capacity = cc_data.cycle_chg_capacity * milliTimeCycle;
    cc_data.cal_cycle_dis_capacity = cc_data.cycle_dis_capacity * milliTimeCycle;
}

void CoulCountProcessTASK(void) {
    uint16_t u16Volt = 0, u16Curr = 0;
    u16Volt = BMS_GetDataValue(bmsBatVolt);
    u16Curr = BMS_GetDataValue(bmsBATCurr);
    if(u16Volt != false) {
      if(CoulCountIDF(u16Curr) == true) {
        CoulCountSOC(u16Volt, u16Curr);
        CoulCountSOH(u16Curr);
        cc_data.idf_current = 0;
      }
    }
}

bool CoulCountIDF(uint16_t current) {
    bool blcomp;
    uint16_t u16tmp;
    uint32_t u32idf;
    u32idf = cc_data.idf_current;

    if((current & 0x8000) == 0x8000) {
        blcomp = true;
        u16tmp = 0x10000 - current;
    }
    else {
        blcomp = false;
        u16tmp = current;
    }
    if(u32idf != false) {
        if((u32idf & 0x80000) == 0x80000) { //set charge
            u32idf = 0x100000 - u32idf;
            if(blcomp == true) {
                u32idf = u32idf + u16tmp;
            }
            else {
                if(u32idf >= u16tmp) {
                    blcomp = true;
                    u32idf = u32idf - u16tmp;
                } else {
                    u32idf = u16tmp - u32idf;
                }
            }
        }//set discharge mode
        else {
            if(blcomp == true) {  
                if(u32idf >= u16tmp) {
                    u32idf = u32idf - u16tmp;
                }
                else {
                    blcomp = true;
                    u32idf = u16tmp - u32idf;
                }
            }
            else {
                u32idf = u32idf + u16tmp;
            }
        }
        if(blcomp == true) {
            cc_data.idf_current = 0x100000 - u32idf;
        }
        else {
            cc_data.idf_current = u32idf;
        }
    }
    else {
        if(blcomp == true) {
            cc_data.idf_current = 0x100000 - u16tmp;
        }
        else {
            cc_data.idf_current = u16tmp;
        }
    }
    if(cc_data.idf_timer >= (current_update_cycle -1)) {
        if((cc_data.idf_current & 0x80000) == 0x80000) {
             u32idf = 0x100000 - cc_data.idf_current;
             cc_data.idf_current = 0x10000 - (u32idf / current_update_cycle);
             cc_data.cycle_cap += 0x10000 - cc_data.idf_current;
         } else {
             cc_data.idf_current = cc_data.idf_current / current_update_cycle;
         }
        cc_data.idf_timer = 0;
        return true;
    }
    else {
        cc_data.idf_timer++;
        return false;
    }
}


void CoulCountSOC(uint16_t voltage, uint16_t current) {
    uint16_t u16cur, u16tmp;
    uint32_t u32rm_cap;
    bool bl_dischg;
    float float_tmp;
    RTC_DateTypeDef rtc_tmp;
    rtc_tmp = RTC_RetrunDate();
    u32rm_cap = cc_data.cal_capacity;
    if((current & 0x8000) == 0x8000) {        // charge mode  //********************************
        u16cur = 0x10000 - current;
        bl_dischg = false;
        if(voltage >= g_sConfig.def_max_voltage) {
            if(u16cur < g_sConfig.def_cutof_current) {
                if(cc_data.full_count >= (full_chg_counte -1)) {
                    u32rm_cap = cc_data.aval_capacity * milliTimeCycle;
                }
                else {
                    cc_data.full_count++;
                }                
            }
        } else {
            cc_data.full_count = 0;
            u32rm_cap = u32rm_cap + u16cur;
            u16tmp = u32rm_cap / milliTimeCycle;
            if(u16tmp >=  cc_data.aval_capacity) {
                u32rm_cap = cc_data.aval_capacity * milliTimeCycle;
            }
        }
    }
    else {    // discharge mode
        bl_dischg = true;
        if(u32rm_cap > current) {
            u32rm_cap = u32rm_cap - current;
        } else {
            u32rm_cap = false;
        }
    }
    cc_data.cal_capacity = u32rm_cap;
    cc_data.rem_capacity = cc_data.cal_capacity / milliTimeCycle;
    cc_data.rem_energy = (cc_data.rem_capacity * g_sConfig.def_rate_voltage) / 10000;
    float_tmp = ((float) cc_data.rem_capacity / cc_data.aval_capacity);

    if(float_tmp >= true) {
        if(bl_dischg == false) {
            if((u16cur < g_sConfig.def_cutof_current)
                    && (voltage >= g_sConfig.def_max_voltage)) {
                cc_data.soc = 1000;
                g_sConfig.LCHG_DATE.Year = rtc_tmp.Year;
                g_sConfig.LCHG_DATE.Month = rtc_tmp.Month;
                g_sConfig.LCHG_DATE.Date = rtc_tmp.Date;
                cc_data.proc_chged = true;
            } else {
                cc_data.soc = 999;
            }
        }
        else {
            cc_data.soc = 1000;
        }
    }
    else {
        if(bl_dischg == false) {
            cc_data.soc = (uint16_t)(float_tmp * 1000);
        } else {
            u16tmp = cc_data.aval_capacity -  cc_data.rem_capacity;
            if(u16tmp < 0x0a) {
                cc_data.soc = 1000;
            } else {
                cc_data.soc = (uint16_t)(float_tmp * 1000);
            }
        }
    }
    cc_status.RDY = true;
}

void CoulCountSOH(uint16_t current) {
    bool blcomp;
    uint16_t u16cur, u16tmp;
    float float_tmp;    
    if((current & 0x8000) == 0x8000) {
        blcomp = false;
        u16cur = 0x10000 - current;
    }
    else {
        blcomp = true;
        u16cur = current;
    }
    if((cc_status.CH_Update_RDY == true) && (cc_status.DS_Update_RDY == false)) {

        if(blcomp == true) {
            if(cc_data.cal_cycle_dis_capacity <  cc_data.cal_aval_capacity) {
                cc_data.cal_cycle_dis_capacity += u16cur;
            }
            else {
                    cc_data.cal_cycle_dis_capacity = cc_data.cal_aval_capacity;
                    cc_data.cal_cycle_chg_capacity = 0;
                    cc_status.DS_Update_RDY = true;
            }
        }
    }
    
    // for the fist initinal must set  cc_status.DS_Update_RDY = true,
   //   and  cc_data.cycle_chg_capacity = cc_data.aval_capacity;
    if((cc_status.CH_Update_RDY == false) && (cc_status.DS_Update_RDY == true)) {
        if(blcomp == false) {
            if((cc_data.soc >= 999) && (cc_data.cal_cycle_chg_capacity >  cc_data.cal_aval_capacity)) {
                    cc_data.cal_cycle_chg_capacity = cc_data.cal_aval_capacity;
                    cc_data.cycle_dis_capacity = 0;
                    cc_status.CH_Update_RDY = true;
            }
            else {
                if(cc_data.cal_cycle_chg_capacity < cc_data.cal_aval_capacity) {
                    cc_data.cal_cycle_chg_capacity += u16cur;
                }
                else {
                    if(cc_data.soc >= 999) {
                        if(cc_data.cycle_dis_capacity  > cc_data.cal_cycle_chg_capacity) {
                            cc_data.cal_cycle_chg_capacity = cc_data.cycle_dis_capacity;
                        }
                        cc_data.cycle_dis_capacity = 0;
                        cc_status.CH_Update_RDY = true;
                    }
                }
            }
        }
    }
    cc_data.cycle_chg_capacity = cc_data.cal_cycle_chg_capacity / milliTimeCycle;
    cc_data.cycle_dis_capacity  = cc_data.cal_cycle_dis_capacity / milliTimeCycle;
    
    if((cc_status.DS_Update_RDY == true) && (cc_status.CH_Update_RDY == true)) {
        if(cc_data.cycle_timer > cycle_data_update_time -1) {
            cc_data.cycle_count++;
            cc_data.cycle_timer = 0;
            cc_status.DS_Update_RDY = false;
            cc_data.aval_capacity = cc_data.cycle_chg_capacity;
            float_tmp = ((float) cc_data.aval_capacity / cc_data.def_capacity);
            if(float_tmp >= 1) {
                u16tmp = 1000;
            }
            else {
                u16tmp = (uint16_t)(float_tmp * 1000);
            }
            cc_data.soh = u16tmp;            
        }
        else {
            cc_data.cycle_timer++;
        }
    }    
}

uint32_t CoulCountRetValue(uint8_t idx) {
        uint32_t u32tmp = false;
	switch (idx) {
	case CC_SOC:
		return cc_data.soc;
		break;
	case CC_SOH:
		return cc_data.soh;
		break;
	case CC_R_ENEGY:
		return cc_data.rem_energy;
		break;
	case CC_R_CAP:
		return cc_data.rem_capacity;
		break;
	case CC_D_CAP:
		return cc_data.def_capacity;
		break;
	case CC_CYC_CNT:
		return cc_data.cycle_count;
		break;
	case CC_CYC_DSG:
		return cc_data.cycle_dis_capacity;
		break;
	case CC_CYC_CHG:
                u32tmp = cc_data.cycle_cap / milliTimeCycle;
                if(u32tmp <= 3000000) {
                  u32tmp = 30000000;
                }
		return u32tmp;
		break;
	case CC_AVEL_CAP:
		return cc_data.aval_capacity;
		break;
	case CC_AVEL_ENEGY:
		return cc_data.avel_energy;
		break;

	default:
		return false;
		break;
	}
}

bool CoulCountRetStatus(uint8_t idx) {
  bool n_res;
	switch(idx) {
	case CC_CH_UPRDY:
		n_res =  cc_status.CH_Update_RDY;
		break;
	case CC_DS_UPRDY:
		 n_res =  cc_status.DS_Update_RDY;
		break;
        case CC_PROC_CHG:
            n_res = cc_data.proc_chged;
            break;
        case CC_RDY:
            n_res = cc_status.RDY;
	default: n_res = false;
		break;
	}
        return n_res; 
}





