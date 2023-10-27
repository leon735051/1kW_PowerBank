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

#ifndef ERR_MOD_H
#define	ERR_MOD_H

#define NO_ERROR_ALARM 0x00
#define LEVEL_1_ALARM  0x01
#define LEVEL_2_ALARM  0x02
#define LEVEL_3_ALARM  0x03

typedef enum
{
	/* Application's state machine's initial state. */
	ErrTEMP_H = 0,
	ErrTEMP_L,
	ErrCELLV_DF,
	ErrCELLV_H,
	ErrCELLV_L,
	ErrBAT_H,
	ErrCHG_CUR,
	ErrDIS_CUR,
	ErrSOC_H,
	ErrSOC_L,
	ErrOPEN_S,
	ErrMDS,
	ErrSMOKE,
	ErrCHG_TMP,
	ErrBALANCE,
	ErrCHGER,
	ErrVCC_L,
	ErrPRECHG,
	ErrCUR,
	/* TODO: Define states used by the application state machine. */
} errStatus_t;



typedef struct 
{
    uint8_t Temp_High;
    uint8_t Temp_Low;
    uint8_t Temp_Diff;
    uint8_t CellV_Diff;
    uint8_t CellV_High;
    uint8_t CellV_Low;
    uint8_t BatV_High;
    uint8_t BatV_Low;
    uint8_t ChgCur;
    uint8_t DisCur;
    uint8_t SOC_High;
    uint8_t SOC_Low;
    uint8_t Open_subcircuit;
    uint8_t MSD;
    uint8_t smoke;
    uint8_t charger_inlerT;
    bool Balance;
    bool charger;
    bool vcc_low;
    bool current_mod;
    bool isolation_mod;
    bool LECU_lose;
    bool VCU_lose;
    bool prechg;
    bool P1_relay;
    bool Chg_relay;
    bool Heat_relay;
    bool soc_skip;
    bool REES_mismatching;
    bool Cell_inconformity;
    uint8_t water_cooling;
    bool over_chg;
    bool acchg_relay;
    bool err_lock;
    uint8_t chg_err;
    uint8_t compound;
    uint8_t err_trig;
} BMS_ERROR_ALARM_t;

extern BMS_ERROR_ALARM_t  bms_error_alarm;
extern void ERR_SetErrorFlag(uint8_t idx);
extern void ERR_ProcessInitial(void);
extern void ERR_ProcessTask(void);
extern uint8_t BMS_ERR_COMPOUND(void);
#ifdef	__cplusplus
extern "C" {
#endif

#ifdef	__cplusplus
}
#endif

#endif	/* ERR_MOD_H */

