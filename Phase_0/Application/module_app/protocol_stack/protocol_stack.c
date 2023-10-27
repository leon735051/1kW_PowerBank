/*
 * Copyright (c) 2013 - 2020, Industrial Technology Research Institute.
 * All rights reserved.
 *
 */

/* Including necessary module. Cpu.h contains other modules needed for compiling.*/
#include <stdbool.h>
#include "driver_bsp/CANpal_bsp.h"
#include "module_app/bms_mod/bms_mod.h"
#include "module_app/bms_mod/err_mod.h"
#include "module_app/bqMaximo/bqMaximo.h"
#include "tp2101_stack.h"
#include "protocol_stack.h"


/*******************************************************************************
 * Definitions
 ******************************************************************************/


/*******************************************************************************
 * Variables
 ******************************************************************************/


/*******************************************************************************
 * Private Functions
 ******************************************************************************/

/*FUNCTION**********************************************************************
 *
 * Function Name : sysLED_ProcessInit
 * Description   : Initial sysLED process.
 *
 * Implements    : sysLED Process Initial
 *END**************************************************************************/
extern BMS_ERROR_ALARM_t  bms_error_alarm;
//extern afe_cell_parmeters_t afe_cell_parmeters;


ps_status_flag_t ps_status_flag;
ps_config_info_t ps_config_info;

/*FUNCTION**********************************************************************
 *
 * Function Name : sysLED_ProcessTask
 * Description   : sysLED Task process.
 *
 * Implements    : sysLED Task Process.
 *END**************************************************************************/


bool PtoclStack_ProcessInit(void) {
	bool nErr = false;
   uint8_t u8tmp[2];
  uint32_t u32sTbuf[] = {TP2101_WAKCMD_ID ,TP2101_MODCMD_ID, TP2101_FATCMD_ID };  
  uint32_t u32eXbuf[] = {TP2101_INFOCMD_ID};
    
	ps_status_flag.TASK_1st = false;
	ps_status_flag.TASK_2nd = false;
	ps_status_flag.TASK_3rd = false;
	ps_status_flag.TASK_4th = false;
	ps_status_flag.TASK_5th = false;

	ps_status_flag.TASK_1st = false;
	ps_status_flag.TASK_2nd = false;
	ps_status_flag.TASK_3rd = false;
	ps_status_flag.TASK_4th = false;
	ps_status_flag.TASK_5th = false;

   u8tmp[0] = sizeof(u32sTbuf) / sizeof(u32sTbuf[0]);
   u8tmp[1] = sizeof(u32eXbuf) / sizeof(u32eXbuf[0]);

   if(CANpal_ProcessInit() != false) {
   	nErr = true;
   }
   if(CANpal_ProcessFilter(u32sTbuf,u32eXbuf, u8tmp[0], u8tmp[1]) != false) {
   	nErr = true;
   }
   if(nErr != true) {
   	if(CANpal_ProcessEnable(true) != false) {
   		nErr = true;
   	}
   }
   return nErr;
}

static void PtoclStack_ProcessTimer_Task(void) {
	if(ps_config_info.tmr_1st >= PS_1ST_TMR_CLK) {
		ps_config_info.tmr_1st = false;
		ps_status_flag.TASK_1st = true;
		ps_config_info.tmr_2nd++;
		ps_config_info.tmr_3rd++;
	} else {
		ps_config_info.tmr_1st++;
	}

	if(ps_config_info.tmr_2nd >= PS_2ND_TMR_CLK) {
		ps_config_info.tmr_2nd = false;
		ps_status_flag.TASK_2nd = true;
	}

	if(ps_config_info.tmr_3rd >= PS_3RD_TMR_CLK ) {
		ps_config_info.tmr_3rd = false;
		ps_status_flag.TASK_3rd = true;
		ps_config_info.tmr_4th++;
	}

	if(ps_config_info.tmr_4th >= PS_4TH_TMR_CLK) {
		ps_config_info.tmr_4th = false;
		ps_status_flag.TASK_4th = true;
		ps_config_info.tmr_5th++;
	}

	if(ps_config_info.tmr_5th >= PS_5TH_TMR_CLK) {
		ps_config_info.tmr_5th = false;
		ps_status_flag.TASK_5th = true;
	}
}


void PtoclStack_MessageTask(void) {
	uint8_t u8idx;
	CANpalRecvMsg_t psMsg;
   psMsg.id = false;
   u8idx = CANpal_GetStatus();
   if(u8idx != 0xff) {
   	if(CANpal_GetMessage(u8idx, &psMsg) != true) {
   		CANpal_ClearStatus(u8idx);
   	}
   }
   if(psMsg.id != false) {
   	switch(psMsg.id) {
   		case TP2101_INFOCMD_ID:
   			TP2101ReqInfoProcess(psMsg.id, psMsg.msg);
   		break;
   		case TP2101_WAKCMD_ID:
   			TP2101ReqWakProcess(psMsg.id,psMsg.msg);
   		break;
   		case TP2101_MODCMD_ID:
   			TP2101ReqModProcess(psMsg.id, psMsg.msg);
   		break;
   		case TP2101_FATCMD_ID:
   			TP2101ReqFacProcess(psMsg.id, psMsg.msg);
   		break;
   		default: break;
   	}
   }
}

void PtoclStack_ProcessTask(void) {
	uint8_t u8idx;
	PtoclStack_ProcessTimer_Task();

	if(ps_status_flag.TASK_1st == true) {  // 100mS
		ps_status_flag.TASK_1st = false;
		u8idx = BMS_ReturnStatus();
		switch(u8idx) {
		case BMS_MODE_NONE:
                  TP2101SetRunMode(tp1_Boot);
                  break;                 
		case BMS_MODE_CHG:
                  TP2101SetRunMode(tp1_Usr_Charge);
                  break;
		case BMS_MODE_Ready:
                  TP2101SetRunMode(tp1_Usr_Normal);
                  break;
		case BMS_MODE_Enable:
                  TP2101SetRunMode(tp1_Usr_Discharge);
                  break;
		case BMS_MODE_Error:
                  if(TP2101GetFlags(tp_FLG_FACTORY) != true) {
                    TP2101SetRunMode(tp1_Usr_Err);
                  } else {
                    TP2101SetRunMode(tp1_Factory);
                  }
                  break;
		case BMS_MODE_Sleep:
                  if(TP2101GetFlags(tp1_MOD_BLT) == true) {
                    TP2101SetRunMode(tp1_Bootloader);
                  } else {
                    TP2101SetRunMode(tp1_Usr_Sleep);
                  }
			break;
		default: break;
		}
	}

	if(ps_status_flag.TASK_2nd == true) {  // 200mS
		ps_status_flag.TASK_2nd = false;

	}
	if(ps_status_flag.TASK_3rd == true) { // 500ms
		ps_status_flag.TASK_3rd = false;

	}
	if(ps_status_flag.TASK_4th == true) { //1000ms
		ps_status_flag.TASK_4th = false;


	}
	if(ps_status_flag.TASK_5th == true) { //3000ms
		ps_status_flag.TASK_5th = false;

	}
}

/******************************************************************************
 * EOF
 *****************************************************************************/
