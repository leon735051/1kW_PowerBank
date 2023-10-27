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
#ifndef COULCOUNT_H
#define	COULCOUNT_H

#include <stdint.h>
#include <stdbool.h>

#define current_update_cycle   10   // update current value @ 100ms
#define milliTimeCycle         3600
#define max_current_c_rate     20
#define full_chg_counte        30    //  check 30 sec to full charge
#define cycle_data_update_time 3000  //  update cycle time at 5Min  

typedef struct
{
    uint32_t idf_current;
    uint8_t  idf_timer;
    uint16_t def_capacity;
    uint16_t aval_capacity;
    uint16_t avel_energy;
    uint16_t rem_energy;       // Kwh
    uint16_t rem_capacity;     // Ah
    uint32_t cal_capacity;
    uint32_t cycle_chg_capacity; // AH
    uint32_t cycle_dis_capacity; // AH
    uint32_t cal_aval_capacity;
    uint32_t cal_cycle_chg_capacity; // AH
    uint32_t cal_cycle_dis_capacity; // AH    
    uint64_t cycle_cap;
    uint16_t cycle_count;
    uint16_t cycle_timer;
    uint16_t  soc;
    uint16_t  soh;
    uint8_t full_count;
    bool proc_chged;
}cc_Parameters;

typedef struct {
    bool CH_Update_RDY;
    bool DS_Update_RDY;
    bool RDY;
} cc_DATA_STS;

typedef enum
{
	/* Application's state machine's initial state. */
	CC_SOC,
	CC_SOH,
	CC_R_ENEGY,
	CC_R_CAP,
	CC_D_CAP,
	CC_CYC_CNT,
	CC_CYC_DSG,
	CC_CYC_CHG,
	CC_AVEL_CAP,
	CC_AVEL_ENEGY,
	/* TODO: Define states used by the application state machine. */
} cc_return_value;

typedef enum {
	CC_CH_UPRDY,
	CC_DS_UPRDY,
        CC_PROC_CHG,
        CC_RDY,
} cc_return_status;




extern void CoulCount_ProcessInit(void);
extern void CoulCountProcessTASK(void);
extern uint32_t CoulCountRetValue(uint8_t idx);
bool CoulCountRetStatus(uint8_t idx);

#ifdef	__cplusplus
extern "C" {
    
}


#ifdef	__cplusplus
}
#endif
/*================== Function Implementations =============================*/
#endif
#endif	/* COULOMB_COUNTER_H */
