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

#include <stdbool.h>
#include <math.h>
#include <float.h>
#include <stdlib.h>
#include <math.h>

#include "driver_bsp/i2c_bsp.h"
#include "module_app/bqMaximo/bqMaximo.h"
#include "bqCommands.h"
#include "bqDevConfig.h" 

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define bqTEMPNOMINAL 27315
/*******************************************************************************
 * Variables
 ******************************************************************************/
afe_cell_params_t afe_cell;
afe_pack_params_t afe_pack;
afe_repo_cfg_t afe_repoCFG;
afe_repo_flags_t afe_repoFLG;
afe_alart_flags_t afe_alart;

/*******************************************************************************
 * Private Functions
 ******************************************************************************/
static void bqProcTimerTask(void);
static void bqValueInitial(void);
static bool bqCheckAlartStatus(void);
static bool bqClearAlartStatus(uint16_t uIdx);
static bool bqUpdateVoltage(uint8_t cal_Cell, uint8_t cal_bat);
static bool bqUpdateTemperature(uint8_t cal_tmp);
static bool bqUpdateCurrent(uint8_t cal_Dsgcur, uint8_t cal_Chgcur);
static bool bqUpdatePackSum(void);
static int cmpfunc (const void * a, const void * b);
static void bqDevice_VoltageSort(void);
static void bqDevice_TempSort(void);
static bool bqDeviceFETdriverStatus(void);
static bool bqDeviceShip(bool nAct);

static void bqProcTimerTask(void) {
	if(afe_repoCFG.tmr_ms >= AFE_1ST_TMR_CLK) {   // 1sec
		afe_repoCFG.tmr_ms = false;
		afe_repoFLG.TASK_1st = true;
		if(afe_repoCFG.tmr_sec >= AFE_2ND_TMR_CLK ) {   //5sec
			afe_repoCFG.tmr_sec = false;
			afe_repoFLG.TASK_2nd = true;
		} else{
			afe_repoCFG.tmr_sec++;
		}
	} else {
		afe_repoCFG.tmr_ms++;
	}
}

static void bqValueInitial(void) {
	uint8_t u8i;

	for(u8i=0; u8i< AFE_CELL_CH; u8i++) {
		afe_cell.Vol[u8i] = false;
	}
	for(u8i=0; u8i< AFE_SET_TMP; u8i++) {
		afe_cell.Temp[u8i] = false;
	}
	for(u8i=0; u8i< AFE_SORT_COUNT; u8i++) {
		afe_cell.MaxIdx[u8i] = false;
		afe_cell.MinIdx[u8i] = false;
	}
    afe_cell.MaxTemp = false;
    afe_cell.MinTemp = false;
    afe_cell.AvgTemp = false;
    afe_cell.Cur = false;
    afe_cell.RLInMxCur = false;
    afe_cell.RLOuxCur = false;
    afe_cell.LD = false;
    afe_cell.Pack = false;
    afe_cell.Stack = false;

    afe_pack.VREG18 = false;
    afe_pack.VSS = false;
    afe_pack.MaxCellV = false;
    afe_pack.MinCellV = false;
    afe_pack.Bat_SumV = false;
    afe_pack.CellTmp = false;
    afe_pack.FetTmp = false;
    afe_pack.MaxCellT = false;
    afe_pack.MinCellT = false;
    afe_pack.AvgCellT = false;
    afe_pack.CC3Curr = false;
    afe_pack.CC1Curr = false;
    afe_pack.BatSTS = false;
    afe_pack.CTLSTS = false;

    afe_repoCFG.tmr_sec = false;
    afe_repoCFG.tmr_ms = false;
    afe_repoCFG.state = AFE_STATE_INIT;

    afe_repoFLG.PWR_ON = false;
    afe_repoFLG.CV_DRY = false;
    afe_repoFLG.CC_DRY = false;
    afe_repoFLG.TM_DRY = false;
    afe_repoFLG.TASK_1st = false;
    afe_repoFLG.TASK_2nd = false;
    afe_repoFLG.DSG_ON = false;
    afe_repoFLG.CSG_ON = false;
    afe_repoFLG.FET_ON = false;
    afe_repoFLG.SHIP = false;
    afe_repoFLG.BALANCE = false;
    afe_repoFLG.ADSCAN = false;
    afe_repoFLG.FULLSCAN = false;

    afe_alart.AltSTS = false;
    afe_alart.SaftA = false;
    afe_alart.SaftB = false;
    afe_alart.SaftC = false;
    afe_alart.PFA = false;
    afe_alart.PFB = false;
    afe_alart.PFC = false;
    afe_alart.PFD = false;
}

static bool bqCheckAlartStatus(void) {
    bool nErr = false;
    uint16_t u16tmp;
    uint8_t *u8buf, *u8ptr;
    u8ptr = NULL;
    u8buf = (uint8_t *)calloc( 2, sizeof(char));
    if(u8buf == NULL) {
        nErr = true;
    } else {
        u8ptr = u8buf;
    }
    if(nErr != true) {
        
        if(bqDirectRead(bqBAT_STS, u8buf, 0x02, AFE_DIRCMD_MTD) != true) {
            u16tmp = *(u8ptr +1) << 0x08;
            u16tmp += *u8ptr;
            afe_pack.BatSTS = u16tmp;
        } else {
            nErr = true;
        }
        if(bqDirectRead(bqALT_STS, u8buf, 0x02, AFE_DIRCMD_MTD) != true) {
            u16tmp = *(u8ptr +1) << 0x08;
            u16tmp += *u8ptr;
            afe_alart.AltSTS = u16tmp;
        } else {
            nErr = true;
        }
    }
    if((nErr != true) && (afe_alart.AltSTS != false)) {
        if((afe_alart.AltSTS & bqAS_SSA) == bqAS_SSA) {
            if(bqDirectRead(bqSFT_STS_A, u8buf, 0x01, AFE_DIRCMD_MTD) != true) {
                afe_alart.SaftA = *u8ptr;
            } else {
                nErr = true;
            }
        }
        if((afe_alart.AltSTS & bqAS_SSBC) == bqAS_SSBC) {
            if(bqDirectRead(bqSFT_STS_B, u8buf, 0x01, AFE_DIRCMD_MTD) != true) {
                afe_alart.SaftB = *u8ptr;
            } else {
                nErr = true;
            }
            if(bqDirectRead(bqSFT_STS_C, u8buf, 0x01, AFE_DIRCMD_MTD) != true) {
                afe_alart.SaftC = *u8ptr;
            } else {
                nErr = true;
            }
        }
        if((afe_alart.AltSTS & bqAS_PF) == bqAS_PF) {
            if(bqDirectRead(bqPF_STS_A, u8buf, 0x01, AFE_DIRCMD_MTD) != true) {
                afe_alart.PFA = *u8ptr;
            } else {
                nErr = true;
            }
            if(bqDirectRead(bqPF_STS_B, u8buf, 0x01, AFE_DIRCMD_MTD) != true) {
                afe_alart.PFB = *u8ptr;
            } else {
                nErr = true;
            }
            if(bqDirectRead(bqPF_STS_C, u8buf, 0x01, AFE_DIRCMD_MTD) != true) {
                afe_alart.PFC = *u8ptr;
            } else {
                nErr = true;
            }
            if(bqDirectRead(bqPF_STS_D, u8buf, 0x01, AFE_DIRCMD_MTD) != true) {
                afe_alart.PFD = *u8ptr;
            } else {
                nErr = true;
            }
        }
        if((afe_alart.AltSTS & bqAS_ADSCAN) == bqAS_ADSCAN) {
            afe_repoFLG.ADSCAN = true;
        }
        if((afe_alart.AltSTS & bqAS_FULLSCAN) == bqAS_FULLSCAN) {
            afe_repoFLG.FULLSCAN = true;
        }
    }
    afe_repoFLG.IRQ = false;
    free(u8buf);
    return nErr;
}

static bool bqClearAlartStatus(uint16_t uIdx) {
    bool nErr = false;
    uint16_t u16tmp;
    u16tmp = afe_alart.AltSTS;
    switch(uIdx) {
        case bqAS_WAKE:
            u16tmp &= ~bqAS_WAKE; break;
        case bqAS_ADSCAN:
            u16tmp &= ~bqAS_ADSCAN; break;
        case bqAS_CB:
            u16tmp &= ~bqAS_CB; break;
        case bqAS_FUSE:
            u16tmp &= ~bqAS_FUSE; break;
        case bqAS_SHUTV:
            u16tmp &= ~bqAS_SHUTV; break;
        case bqAS_XDSG:
            u16tmp &= ~bqAS_XDSG; break;
        case bqAS_XCHG:
            u16tmp &= ~bqAS_XCHG; break;
        case bqAS_FULLSCAN:
            u16tmp &= ~bqAS_FULLSCAN; break;
        case bqAS_INITCOMP:
            u16tmp &= ~bqAS_INITCOMP; break;
        case bqAS_INITSTART:
            u16tmp &= ~bqAS_INITSTART; break;
        case bqAS_MSK_PFALERT:
            u16tmp &= ~bqAS_MSK_PFALERT; break;
        case bqAS_MSK_SFALERT:
            u16tmp &= ~bqAS_MSK_SFALERT; break;
        case bqAS_PF:
            u16tmp &= ~bqAS_PF; break;
        case bqAS_SSA:
            u16tmp &= ~bqAS_SSA; break;
        case bqAS_SSBC:
            u16tmp &= ~bqAS_SSBC; break;
        case bqAS_FIELD:
            u16tmp = false; break;
        default: break;
    }
    if(bqDirectCommand(bqALT_STS, u16tmp, 0x02, AFE_DIRCMD_MTD) != false) {
        nErr = true;
    }
    return nErr;
}

static bool bqUpdateVoltage(uint8_t cal_Cell, uint8_t cal_bat) {
    bool nErr = false;
    uint8_t u8i;
    uint16_t u16cmd, u16tmp;
    uint8_t *u8buf, *u8ptr;
    u16tmp = false;
    u8buf = (uint8_t *)calloc( 2, sizeof(char));
    if(u8buf == NULL) {
        nErr = true;
    } else {
      u8ptr = u8buf;
    }
    u8ptr = u8buf;
    u16cmd = bqCELL1_VOL;

    // Reads all cell voltages
    if(nErr != true) {
      for(u8i = false; u8i < AFE_CELL_CH; u8i++) {
        if(bqDirectRead(u16cmd, u8buf, 0x02, AFE_DIRCMD_MTD) != true) {
          u16tmp = *(u8ptr +1) << 0x08;
          u16tmp += *u8ptr;
        } else {
          nErr = true;
        }
        if(u16tmp != false) {
            if((cal_Cell & 0x80) == 0x80) {
                u16tmp = u16tmp - cal_Cell;
            } else {
                u16tmp = u16tmp + cal_Cell;
            }
        }
        if(u8i == (AFE_CELL_CH -2)) {
          u16cmd = bqCELL16_VOL;
        } else {
          u16cmd += 2;
        }
        afe_cell.Vol[u8i] = u16tmp;
        u16tmp = false;
      }
    }
    
    // Read Stack Voltage
    if(nErr != true) {
        if(bqDirectRead(bqSTACK_VOL, u8buf, 0x02, AFE_DIRCMD_MTD) != true) {
            u16tmp = *(u8ptr +1) << 0x08;
            u16tmp += *u8ptr;
            if(u16tmp != false) {
                if((cal_bat & 0x80) == 0x80) {
                    u16tmp = u16tmp - cal_bat;
                } else {
                    u16tmp = u16tmp + cal_bat;
                }
            }
            afe_cell.Stack = u16tmp;
            u16tmp = false;
        } else {
            nErr = true;
        }
    }

    // Read Pack Side Voltage
    if(nErr != true) {
        if(bqDirectRead(bqPACK_VOL, u8buf, 0x02, AFE_DIRCMD_MTD) != true) {
            u16tmp = *(u8ptr +1) << 0x08;
            u16tmp += *u8ptr;
            if(u16tmp != false) {
                if((cal_bat & 0x80) == 0x80) {
                    u16tmp = u16tmp - cal_bat;
                } else {
                    u16tmp = u16tmp + cal_bat;
                }
            }
            afe_cell.Pack = u16tmp;
            u16tmp = false;
        } else {
            nErr = true;
        }
    }

    // Read LD Side Voltage
    if(nErr != true) {
        if(bqDirectRead(bqLD_VOL, u8buf, 0x02, AFE_DIRCMD_MTD) != true) {
            u16tmp = *(u8ptr +1) << 0x08;
            u16tmp += *u8ptr;
            if(u16tmp != false) {
                if((cal_bat & 0x80) == 0x80) {
                    u16tmp = u16tmp - cal_bat;
                } else {
                    u16tmp = u16tmp + cal_bat;
                }
            }
            afe_cell.LD = u16tmp;
        } else {
            nErr = true;
        }
    }
    if(nErr != true) {
        afe_repoFLG.CV_DRY = true;
    } else {
        afe_repoFLG.CV_DRY = false;
    }
    free(u8buf);
    return nErr;
}

static bool bqUpdateTemperature(uint8_t cal_tmp) {
    bool nErr = false;
    uint8_t u8i, u8buf[2];
    uint16_t u16cmd, u16buf;
    volatile float fol_tmp = false;
    u16cmd = bqTSP_TEMP;
    // Reads all Temperature
    for(u8i = false; u8i < AFE_SET_TMP; u8i++) {
        if(bqDirectRead(u16cmd, u8buf, 0x02, AFE_DIRCMD_MTD) != true) {
            u16cmd += 2;
            u16buf = u8buf[1] << 0x08;
            u16buf += u8buf[0];
            if((u16buf & 0x8000) == 0x8000) { // Subzero
              u16buf = bqTEMPNOMINAL - (u16buf *10);
            } else {
              u16buf = (u16buf *10) - bqTEMPNOMINAL;
            }
            if((cal_tmp & 0x80) == 0x80) {
                afe_cell.Temp[u8i] = u16buf - cal_tmp;
            } else {
                afe_cell.Temp[u8i] = u16buf + cal_tmp;
            } 
            
        } else {
            nErr = true;
        }
    }
    if(nErr != true) {
        afe_repoFLG.TM_DRY = true;
    } else {
        afe_repoFLG.TM_DRY = false;
    }
    return nErr;
}

static bool bqUpdateCurrent(uint8_t cal_Dsgcur, uint8_t cal_Chgcur) {
  bool nErr = false;
    uint8_t u8buf[4];
    uint16_t u16tmp;
    if(bqDirectRead(bqCC2_CURR, u8buf, 0x02, AFE_DIRCMD_MTD) != true) {
        u16tmp = u8buf[1] << 0x08;
        u16tmp += u8buf[0];
        if((u16tmp & 0x8000) == 0x8000) {  // Discharge
            u16tmp = 0xffff - u16tmp;
            if((cal_Dsgcur & 0x80) == 0x80) {
                u16tmp = u16tmp - cal_Dsgcur;
            } else {
                u16tmp = u16tmp + cal_Dsgcur;
            }
            afe_cell.Cur = 0xffff - u16tmp;
            if((afe_cell.RLOuxCur >= afe_cell.Cur) || (afe_cell.RLOuxCur == false)) {
              afe_cell.RLOuxCur = afe_cell.Cur;
            }
        } else {
            if((cal_Chgcur & 0x80) == 0x80) {
                afe_cell.Cur = u16tmp - cal_Chgcur;
            } else {
                afe_cell.Cur = u16tmp + cal_Chgcur;
            }
            if((afe_cell.RLInMxCur <= afe_cell.Cur) || (afe_cell.RLInMxCur == false)) {
              afe_cell.RLInMxCur = afe_cell.Cur;
            }
        }
    } else {
              nErr = true;
    }
    if(nErr != true) {
        afe_repoFLG.CC_DRY = true;
    } else {
        afe_repoFLG.CC_DRY = false;
    }
    return nErr;
}

static bool bqUpdatePackSum(void) {
    bool nErr = false;
    uint8_t u8i;
    uint16_t u16tmp;
    uint8_t *u8buf, *u8ptr;
    u16tmp = false;
    u8buf = (uint8_t *)calloc( 32, sizeof(char));
    if(u8buf == NULL) {
        nErr = true;
    } else {
      u8ptr = u8buf;
    }
    if(nErr != true) {
        if(bqDirectRead(bqDASTATUS5, u8buf, 32, AFE_SUBCMD_RTD) == true) {
            nErr = true;
        }
    }
    u8i = false;
    while((u8i <= 22) && nErr != true) {
        u16tmp = false;
        switch(u8i) {
            case 0:
                u16tmp = *u8ptr++;
                u16tmp += (*u8ptr++) << 0x08;
                afe_pack.VREG18 = u16tmp;
                u8i += 2;
            break;
            case 2:
                u16tmp = *u8ptr++;
                u16tmp += (*u8ptr++) << 0x08;
                afe_pack.VSS = u16tmp;
                u8i += 2;
            break;
            case 4:
                u16tmp = *u8ptr++;
                u16tmp += (*u8ptr++) << 0x08;
                afe_pack.MaxCellV = u16tmp;
                u8i += 2;
            break;            
            case 6:
                u16tmp = *u8ptr++;
                u16tmp += (*u8ptr++) << 0x08;
                afe_pack.MinCellV = u16tmp;
                u8i += 2;
            break;                    
            case 8:
                u16tmp = *u8ptr++;
                u16tmp += (*u8ptr++) << 0x08;
                afe_pack.Bat_SumV = u16tmp;
                u8i += 2;
            break;                    
            case 10:
                u16tmp = *u8ptr++;
                u16tmp += (*u8ptr++) << 0x08;
                if((u16tmp & 0x8000) == 0x8000) { // Subzero
                    u16tmp = bqTEMPNOMINAL - (u16tmp *10);
                } else {
                u16tmp = (u16tmp *10) - bqTEMPNOMINAL;
                }
                afe_pack.CellTmp = u16tmp;
                u8i += 2;
            break;                    
            case 12:
                u16tmp = *u8ptr++;
                u16tmp += (*u8ptr++) << 0x08;
               if((u16tmp & 0x8000) == 0x8000) { // Subzero
                      u16tmp = bqTEMPNOMINAL - (u16tmp *10);
                } else {
                    u16tmp = (u16tmp *10) - bqTEMPNOMINAL;
                }         
                afe_pack.FetTmp = u16tmp;
                u8i += 2;
            break;                    
            case 14:
                u16tmp = *u8ptr++;
                u16tmp += (*u8ptr++) << 0x08;               
                if((u16tmp & 0x8000) == 0x8000) { // Subzero
                    u16tmp = bqTEMPNOMINAL - (u16tmp *10);
                } else {
                    u16tmp = (u16tmp *10) - bqTEMPNOMINAL;
                }
                afe_pack.MaxCellT = u16tmp;
                u8i += 2;
            break;                    
            case 16:
                u16tmp = *u8ptr++;
                u16tmp += (*u8ptr++) << 0x08;
                if((u16tmp & 0x8000) == 0x8000) { // Subzero
                    u16tmp = bqTEMPNOMINAL - (u16tmp *10);
                } else {
                    u16tmp = (u16tmp *10) - bqTEMPNOMINAL;
                }
                afe_pack.MinCellT = u16tmp;
                u8i += 2;
            break;                    
            case 18:
                u16tmp = *u8ptr++;
                u16tmp += (*u8ptr++) << 0x08;
                if((u16tmp & 0x8000) == 0x8000) { // Subzero
                    u16tmp = bqTEMPNOMINAL - (u16tmp *10);
                } else {
                    u16tmp = (u16tmp *10) - bqTEMPNOMINAL;
                }
                afe_pack.AvgCellT = u16tmp;
                u8i += 2;
            break;
            case 20:
                u16tmp = *u8ptr++;
                u16tmp += (*u8ptr++) << 0x08;
                afe_pack.CC3Curr = u16tmp;
                u8i += 2;
            break;
            case 22:
                u16tmp = *u8ptr++;
                u16tmp += (*u8ptr++) << 0x08;
                afe_pack.CC1Curr = u16tmp;
                u8i += 2;
            break;
            default: break;
        }
    }
    if(nErr != true) {
      if(bqDirectRead(bqCTL_STS, u8buf, 0x02, AFE_DIRCMD_MTD) != true) {
            u16tmp = *(u8ptr +1) << 0x08;
            u16tmp += *u8ptr;
            afe_pack.CTLSTS = u16tmp;
        }
    }
    free(u8buf);
    return nErr;
}


static int cmpfunc (const void * a, const void * b) {
    return ((compare_buf_t *)b)->value - ((compare_buf_t *)a)->value;
}

static void bqDevice_VoltageSort(void) {
    uint16_t u8i;
    compare_buf_t cmp_buf[AFE_CELL_CH];
    for(u8i = 0; u8i < AFE_CELL_CH; u8i++) {
        cmp_buf[u8i].index = u8i;
        cmp_buf[u8i].value = afe_cell.Vol[u8i];
    }
    qsort(cmp_buf, AFE_CELL_CH , sizeof(compare_buf_t) , cmpfunc);
    for(u8i=0 ; u8i< AFE_SORT_COUNT; u8i++) {
        afe_cell.MaxIdx[u8i] = cmp_buf[u8i].index;
        afe_cell.MinIdx[u8i] = cmp_buf[(AFE_CELL_CH -1) - u8i].index;
    }
}

static void bqDevice_TempSort(void ) {
    uint32_t u32tmp_avg;
    uint16_t u16maxT, u16minT;
    uint8_t u8i;
    u32tmp_avg = 0;

    for(u8i = 0; u8i < AFE_SET_TMP; u8i++) {
        if(u8i == 0) {
            u16maxT = afe_cell.Temp[0];
            u16minT = afe_cell.Temp[0];
        }
        else {
            if(u16maxT <= afe_cell.Temp[u8i]) {
                u16maxT = afe_cell.Temp[u8i];
            }
            if(u16minT >= afe_cell.Temp[u8i]) {
                u16minT = afe_cell.Temp[u8i];
            }
        }
        u32tmp_avg += afe_cell.Temp[u8i];
    }
    u32tmp_avg = u32tmp_avg / AFE_SET_TMP;
    afe_cell.MaxTemp = u16maxT;
    afe_cell.MinTemp = u16minT;
    afe_cell.AvgTemp = u32tmp_avg;
}

static bool bqDeviceFETdriverStatus(void) {
    bool nErr = false;
    uint8_t u8buf[2];
    if(bqDirectRead(bqFET_STS, u8buf, 1, AFE_DIRCMD_MTD) != true) {
        if(u8buf[0] != afe_pack.FET) {
            afe_pack.FET = u8buf[0];
        }
    } else {
        nErr = true;
    }
    if((afe_pack.FET & bqFET_CHG) ==bqFET_CHG) {
        afe_repoFLG.CSG_ON = true;
    } else {
        afe_repoFLG.CSG_ON = false;
    }
    if((afe_pack.FET & bqFET_DSG) ==bqFET_DSG) {
        afe_repoFLG.DSG_ON = true;
    } else {
        afe_repoFLG.DSG_ON = false;
    }
    if((afe_pack.FET & bqFET_PDSG) ==bqFET_PDSG) {
        afe_repoFLG.PCHG_ON = true;
    } else {
        afe_repoFLG.PCHG_ON = false;
    }
  return nErr;
}

static bool bqDeviceShip(bool nAct) {
    bool nErr = false;
    uint8_t u8i;
    if(nAct == true) {
        nErr = bqDeviceFETdriverCTL(fFET_ENB, false);
        for(u8i= false; u8i < 2; u8i++) {
            if(nErr != true) {
                nErr = bqDirectCommand(bqSHUTDOWN, false, false, AFE_SUBCMD_RTD);
            }
        }
    } else {
        nErr = bqDirectCommand(bqEXT_DEEP_SLP, false, false, AFE_SUBCMD_RTD);
    }
    return nErr;
}

bool bqDeviceInitAFE(void) {
    bool n_err = false;
    // Resets the device registers
   // bqDirectCommand(bqRESET_DEV, false, AFE_DIRCMD_MTD);
    bqValueInitial();
    // Enter CONFIGUPDATE mode
    
    bqDirectCommand(bqUnsealKeyStep1    , bqCFG_UKST1, 2, AFE_SUBCMD_RTD);
    bqDirectCommand(bqUnsealKeyStep2  , bqCFG_UKST2, 2, AFE_SUBCMD_RTD);
    
    bqDirectCommand(bqSET_CFGUPDATE, false, false, AFE_SUBCMD_RTD);

    bqDirectCommand(bqPowerConfig, bqCFG_POWER, 2, AFE_SUBCMD_RTD);
    bqDirectCommand(bqREG12Config, bqCFG_REG12, 1, AFE_SUBCMD_RTD);
    bqDirectCommand(bqREG0Config, bqCFG_REG0, 1, AFE_SUBCMD_RTD);

    bqDirectCommand(bqCFETOFFPinConfig, bqCFG_CFETOFF, 1, AFE_SUBCMD_RTD);
    bqDirectCommand(bqDFETOFFPinConfig, bqCFG_DFETOFF, 1, AFE_SUBCMD_RTD);
    bqDirectCommand(bqALERTPinConfig, bqCFG_ALERT, 1, AFE_SUBCMD_RTD);
    bqDirectCommand(bqDefaultAlarmMask, bqCFG_ALARMASK, 2, AFE_SUBCMD_RTD);

    bqDirectCommand(bqTS1Config, bqCFG_TS1, 1, AFE_SUBCMD_RTD);
    bqDirectCommand(bqTS2Config, bqCFG_TS2, 1, AFE_SUBCMD_RTD);
    bqDirectCommand(bqTS3Config, bqCFG_TS3, 1, AFE_SUBCMD_RTD);

    bqDirectCommand(bqDDSGPinConfig , bqCFG_DDSG, 1, AFE_SUBCMD_RTD);

    bqDirectCommand(bqVCellMode  , bqCFG_CellMod, 2, AFE_SUBCMD_RTD);

    bqDirectCommand(bqEnableProtectionsA  , bqCFG_ENBProtA, 1, AFE_SUBCMD_RTD);
    bqDirectCommand(bqEnableProtectionsB  , bqCFG_ENBProtB, 1, AFE_SUBCMD_RTD);
    bqDirectCommand(bqEnableProtectionsC  , bqCFG_ENBProtC, 1, AFE_SUBCMD_RTD);
    bqDirectCommand(bqEnabledPFC , bqCFG_ENBPfC,   1, AFE_SUBCMD_RTD);

    bqDirectCommand(bqFETOptions , bqCFG_FETOPT, 1, AFE_SUBCMD_RTD);
    bqDirectCommand(bqPredischargeTimeout , bqCFG_PreDSGTmr, 1, AFE_SUBCMD_RTD);

    bqDirectCommand(bqBalancingConfiguration   , bqCFG_BALCFG, 1, AFE_SUBCMD_RTD);
    bqDirectCommand(bqMinCellTemp  , bqCFG_BALMiTmp, 1, AFE_SUBCMD_RTD);
    bqDirectCommand(bqMaxCellTemp   , bqCFG_BALMxTmp, 1, AFE_SUBCMD_RTD);
    bqDirectCommand(bqMaxInternalTemp  , bqCFG_BALIntTmp, 1, AFE_SUBCMD_RTD);
    bqDirectCommand(bqCellBalanceMinCellVCharge  , bqCFG_BALMiCV_CH, 2, AFE_SUBCMD_RTD);
    bqDirectCommand(bqCellBalanceMinDeltaCharge  , bqCFG_BALMiDT_CH, 1, AFE_SUBCMD_RTD);
    bqDirectCommand(bqCellBalanceStopDeltaCharge  , bqCFG_BALSTPDT_CH, 1, AFE_SUBCMD_RTD);
    bqDirectCommand(bqCellBalanceMinCellVRelax  , bqCFG_BALMiCV_RX, 2, AFE_SUBCMD_RTD);
    bqDirectCommand(bqCellBalanceMinDeltaRelax   , bqCFG_BALMiDT_RX, 1, AFE_SUBCMD_RTD);
    bqDirectCommand(bqCellBalanceStopDeltaRelax   , bqCFG_BALSTPDT_RX, 1, AFE_SUBCMD_RTD);

    bqDirectCommand(bqCUVThreshold , bqCFG_CUVTHR, 1, AFE_SUBCMD_RTD);
    bqDirectCommand(bqCUVDelay     , bqCFG_CUVDELY, 1, AFE_SUBCMD_RTD);
    bqDirectCommand(bqCUVRecoveryHysteresis    , bqCFG_CUVRECY, 1, AFE_SUBCMD_RTD);

    bqDirectCommand(bqCOVThreshold  , bqCFG_COVTHR, 1, AFE_SUBCMD_RTD);
    bqDirectCommand(bqCOVDelay      , bqCFG_COVDELY, 1, AFE_SUBCMD_RTD);
    bqDirectCommand(bqCOVRecoveryHysteresis , bqCFG_COVRECY, 1, AFE_SUBCMD_RTD);

    bqDirectCommand(bqOCD1Threshold       , bqCFG_OCCDELY, 1, AFE_SUBCMD_RTD);
    bqDirectCommand(bqOCD1Delay , bqCFG_PackTOSDT, 1, AFE_SUBCMD_RTD);

    bqDirectCommand(bqOCCDelay       , bqCFG_OCD1THR, 1, AFE_SUBCMD_RTD);
    bqDirectCommand(bqCOVRecoveryHysteresis , bqCFG_OCD1DELY, 1, AFE_SUBCMD_RTD);

    bqDirectCommand(bqOCD2Threshold       , bqCFG_OCD2THR, 1, AFE_SUBCMD_RTD);
    bqDirectCommand(bqOCD2Delay , bqCFG_OCD2DELY, 1, AFE_SUBCMD_RTD);

    bqDirectCommand(bqOCD3Threshold       , bqCFG_OCD3THR, 2, AFE_SUBCMD_RTD);
    bqDirectCommand(bqOCD3Delay , bqCFG_OCD3DELY, 1, AFE_SUBCMD_RTD);

    bqDirectCommand(bqSCDThreshold       , bqCFG_SCDTHR, 1, AFE_SUBCMD_RTD);
    bqDirectCommand(bqSCDDelay , bqCFG_SCDDELY, 1, AFE_SUBCMD_RTD);


    bqDirectCommand(bqOTCThreshold       , bqCFG_OTCTHR, 1, AFE_SUBCMD_RTD);
    bqDirectCommand(bqOTCDelay , bqCFG_OTCDLELY, 1, AFE_SUBCMD_RTD);
    bqDirectCommand(bqOTCRecovery , bqCFG_OTCRECY, 1, AFE_SUBCMD_RTD);

    bqDirectCommand(bqOTDThreshold       , bqCFG_OTDTHR, 1, AFE_SUBCMD_RTD);
    bqDirectCommand(bqOTDDelay , bqCFG_OTDDLELY, 1, AFE_SUBCMD_RTD);
    bqDirectCommand(bqOTDRecovery , bqCFG_OTDRECY, 1, AFE_SUBCMD_RTD);

    bqDirectCommand(bqUTCThreshold       , bqCFG_UTCTHR, 1, AFE_SUBCMD_RTD);
    bqDirectCommand(bqUTCDelay , bqCFG_UTCDLELY, 1, AFE_SUBCMD_RTD);
    bqDirectCommand(bqUTCRecovery , bqCFG_UTCRECY, 1, AFE_SUBCMD_RTD);
    
    bqDirectCommand(bqCCGain , bqCFG_CCGAIN, 1, AFE_SUBCMD_RTD);

    // Exit CONFIGUPDATE mode  - Subcommand 0x0092
    bqDirectCommand(bqEXIT_CFGUPDATE, false, false, AFE_SUBCMD_RTD);
    bqDirectCommand(bqSLEEP_DISABLE, false, false, AFE_SUBCMD_RTD);
   
    return n_err;
}

bool bqDeviceFETdriverCTL(uint8_t nIdx, bool nAct) {
    bool nErr = false;
    uint8_t u8buf[2];
    nErr = bqDirectRead(bqMANUFACTURINGSTATUS, u8buf, 2, AFE_SUBCMD_RTD);
    if((nErr != true) && (nIdx == fFET_ENB)) {
        if(nAct != true) {
            afe_repoFLG.FET_ON = false;
            bqDirectCommand(bqALL_FETS_OFF, false, false, AFE_SUBCMD_RTD);
        } else {
            if((u8buf[0] & bqFET_ENB) != bqFET_ENB) {
                nErr = bqDirectCommand(bqFET_ENABLE, false, false, AFE_SUBCMD_RTD);
            } else {
                nErr = bqDirectCommand(bqALL_FETS_ON, false, false, AFE_SUBCMD_RTD);
            }
            afe_repoFLG.FET_ON = true;
        }
    } else if(nErr != true) {
        if((u8buf[0] & bqFET_ENB) == bqFET_ENB) {
            nErr = bqDirectCommand(bqFET_ENABLE, false, false, AFE_SUBCMD_RTD);
        }
        switch(nIdx) {
            case fPCHG:
                if(((u8buf[0] & bqTES_PCHG) != bqTES_PCHG) || (nAct != true)) {
                    
                    nErr = bqDirectCommand(bqPCHGTEST, false, false, AFE_SUBCMD_RTD);
                }
                afe_repoFLG.PCHG_ON = nAct;
            break;
            case fCSG:
                if(((u8buf[0] & bqTES_CHG) != bqTES_CHG) || (nAct != true)) {
                    nErr = bqDirectCommand(bqCHGTEST, false, false, AFE_SUBCMD_RTD);
                }
                afe_repoFLG.CSG_ON = nAct;
            break;
            case fDSG:
                if(((u8buf[0] & bqTES_DSG) != bqTES_DSG) || (nAct != true)) {
                    nErr = bqDirectCommand(bqDSGTEST, false, false, AFE_SUBCMD_RTD);
                }
                afe_repoFLG.DSG_ON = nAct;
            break;
            default: break;
        }
    }
    return nErr;
}
 
void bqDeviceAlartTrig(uint8_t nIdx) {
    switch(nIdx) {
    case fAFE_IRQ:
        afe_repoFLG.IRQ = true;
        break;
    default: break;
    }
}

bool bqDeviceGetAlartFlags(uint8_t nIdx) {
    bool nRes = false;
    switch (nIdx) {
    case fAFE_ALART:
      //  nRes = afe_alart_flag.AFE_ALART;
        break;
    case fDEV_XD:
      //  nRes = afe_alart_flag.DEV_XD;
        break;
    case fOVRDAL:
       // nRes = afe_alart_flag.OVRDAL;
        break;
    case fUV:
      //  nRes = afe_alart_flag.UV;
        break;
    case fOV:
       // nRes = afe_alart_flag.OV;
        break;
    case fSCD:
       // nRes = afe_alart.SCD;
        break;
    case fOCD:
       // nRes = afe_alart.OCD;
        break;
    default: nRes = false;
        break;
    }
    return nRes;
}

void bqDeviceSetSTS(uint8_t nIdx) {
    switch(nIdx) {
    case fFET_ENB:
        afe_repoFLG.FET_ON = true;
        break; 
    case fAFE_SHIP:
        afe_repoFLG.SHIP = true;
        break;
    case fAFE_BALANCE:
        afe_repoFLG.BALANCE = true;
        break;
    default: break;
    }
}

bool bqDeviceGetSTS(uint8_t nIdx) {
    bool nRes = false;
    switch (nIdx) {
    case fCV:
        nRes = afe_repoFLG.CV_DRY;
        break;
    case fCC:
        nRes = afe_repoFLG.CC_DRY;
        break;
    case fTM:
        nRes = afe_repoFLG.TM_DRY;
        break;
    case fDSG:
        nRes = afe_repoFLG.DSG_ON;
        break;
    case fCSG:
        nRes = afe_repoFLG.CSG_ON;
        break;
    case fPCHG:
      nRes = afe_repoFLG.PCHG_ON;
      break;
    case fFET_ENB:
        nRes = afe_repoFLG.FET_ON;
        break;    
    case fAFE_SHIP:
        nRes = afe_repoFLG.SHIP;
        break;
    case fAFE_BALANCE:
        nRes = afe_repoFLG.BALANCE;
        break;
    default: nRes = false;
        break;
    }
    return nRes;
}


void bqDeviceClearSTS(uint8_t nIdx) {

    switch (nIdx) {
    case fCV:
        afe_repoFLG.CV_DRY = false;
        break;
    case fCC:
        afe_repoFLG.CC_DRY = false;
        break;
    case fTM:
        afe_repoFLG.TM_DRY = false;
        break;      
    case fAFE_BALANCE:
        afe_repoFLG.BALANCE = false;
        break;
    case fFET_ENB:
        afe_repoFLG.FET_ON = false;
        break;           
    case fAFE_SHIP:
        afe_repoFLG.SHIP = false;
        break;
    default: 
        break;
    }
}

uint16_t bqDeviceGetValue(uint8_t nIdx , uint8_t nCh) {
    uint16_t u16tmp = false;
    switch(nIdx) {
        case bqCell_Val:
            if(nCh >= AFE_CELL_CH) {
                u16tmp = false;
            } else {
                u16tmp = afe_cell.Vol[nCh];
            }
        break;
    case bqCell_MaxV:
        u16tmp = afe_cell.Vol[afe_cell.MaxIdx[0]];
        break;
    case bqCell_MinV:
        u16tmp = afe_cell.Vol[afe_cell.MinIdx[0]];
        break;
    case bqCell_MaxT:
        u16tmp = afe_cell.MaxTemp;
        break;
    case bqCell_MinT:
        u16tmp = afe_cell.MinTemp;
        break;
    case bqCell_AvgT:
        u16tmp = afe_cell.AvgTemp;
        break;
    case bqCell_Tmp:
        if(nCh >= AFE_SET_TMP) {
            u16tmp = false;
        } else {
            u16tmp = afe_cell.Temp[nCh];
        }
        break;        
    case bqBat_Stack:
        u16tmp = afe_cell.Stack;
        break;
    case bqBat_Pack:
        u16tmp = afe_cell.Pack;
        break;
    case bqBat_Cur:
        u16tmp = afe_cell.Cur;
        break;
    case bqBat_CurAVG:
        u16tmp = afe_pack.CC3Curr;
        break;
    case bqBat_InMaxCur:
        u16tmp = afe_cell.RLInMxCur;
        break;
    case bqBat_OtMaxCur:
        u16tmp = afe_cell.RLOuxCur;
        break;
    case bqAFE_FetT:
        u16tmp = afe_pack.FetTmp;
        break;
    case bqAFE_ALART:
       u16tmp = afe_alart.AltSTS;
       break;
    case bqAFE_PFAB:
      u16tmp = afe_alart.PFA;
      u16tmp += afe_alart.PFB << 0x08;
      break;
    case bqAFE_PFCD:
      u16tmp = afe_alart.PFC;
      u16tmp += afe_alart.PFD << 0x08;      
      break;
    case bqAFE_FETSTS:
      u16tmp = afe_pack.FET;
      break;
    case bqAFE_BATSTS:
      u16tmp = afe_pack.BatSTS;
      break;
    case bqAFE_CTLSTS:
      u16tmp = afe_pack.CTLSTS;
      break;
    case bqAFE_SAFT_A:
      u16tmp = afe_alart.SaftA;
      break;
    case bqAFE_SAFT_B:
      u16tmp = afe_alart.SaftB;
      break;    
    case bqAFE_SAFT_C:
      u16tmp = afe_alart.SaftC;
      break;        
    default: u16tmp =  false;
        break;
    }
    return u16tmp;
}

bool bqDeviceProcessTask(void) {
	bqProcTimerTask();
	if(afe_repoFLG.IRQ == true) {
          bqCheckAlartStatus();
	}
	switch(afe_repoCFG.state) {
	case AFE_STATE_INIT: {
        bqCheckAlartStatus();
        if(afe_repoFLG.FULLSCAN == true) {
            if(bqUpdateVoltage(false, false) != true) {
                bqDevice_VoltageSort();
            }
            if(bqUpdateTemperature(false) != true) {
                bqDevice_TempSort();
            }
            if(bqUpdateCurrent(false,false) != true) {
                bqUpdatePackSum();
            }
            if((afe_repoFLG.CV_DRY == true) && (afe_repoFLG.TM_DRY == true)) {
                afe_repoFLG.FULLSCAN = false;
                afe_repoFLG.ADSCAN = false;
                bqClearAlartStatus(bqAS_FIELD);
                afe_repoCFG.state = AFE_STATE_SERVICE_TASKS;
            }
        }
    }
	break;
	case AFE_STATE_SERVICE_TASKS: {
        bqCheckAlartStatus();
        if(afe_repoFLG.FULLSCAN == true) {
            if(bqUpdateVoltage(false, false) != true) {
                bqDevice_VoltageSort();
            }
            if(bqUpdateTemperature(false) != true) {
                bqDevice_TempSort();
            }
            if(bqUpdateCurrent(false,false) != true) {
                bqUpdatePackSum();
            }
            if((afe_repoFLG.CV_DRY == true) && (afe_repoFLG.TM_DRY == true)) {
                afe_repoFLG.FULLSCAN = false;
                afe_repoFLG.ADSCAN = false;
                bqClearAlartStatus(bqAS_ADSCAN);
                bqClearAlartStatus(bqAS_FULLSCAN);
            }
        }
		if(afe_repoFLG.TASK_1st == true) {
            bqDeviceFETdriverStatus();
			afe_repoFLG.TASK_1st = false;
		}

		if(afe_repoFLG.TASK_2nd) {
            if(afe_repoFLG.SHIP == true) {
                if(bqDeviceShip(true) == false) {
                    afe_repoCFG.state = AFE_STATE_SYSTEM_OFF;
                }
            }
            afe_repoFLG.TASK_2nd = false;
        }
    }
	break;
	case AFE_STATE_SYSTEM_OFF:
		if(afe_repoFLG.SHIP == false) {
            if(bqDeviceShip(false) == false) {
                afe_repoCFG.state = AFE_STATE_INIT;
            }
		}
		break;

	default: break;
	}
	return false;
}
