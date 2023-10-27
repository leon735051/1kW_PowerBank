/**
  ******************************************************************************
  * @file    mainThread.c
  * @brief   Main thread program body
  * @author  Oliver .Chiu.
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

/* Includes ------------------------------------------------------------------*/
#include "mainThread.h"
#include "driver_bsp/lpiTimer_bsp.h"
#include "driver_bsp/PortMux_bsp.h"
#include "driver_bsp/PortIRQ_bsp.h"
#include "driver_bsp/i2c_bsp.h"
#include "driver_bsp/daConv_bsp.h"
#include "driver_bsp/rtc_bsp.h"
#include "system_app/sysLED_mod.h"
#include "module_app/bms_mod/bms_mod.h"
#include "module_app/bqMaximo/bqMaximo.h"
#include "module_app/flash_mod/flash_mod.h"
#include "module_app/protocol_stack/protocol_stack.h"
#include "module_app/CoulCount/CoulCount.h"
#include "module_app/protocol_stack/tp2101_stack.h"

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
appProcess_t appProcess;


/* Private function prototypes -----------------------------------------------*/

/* Private user code ---------------------------------------------------------*/


/* Public function prototypes -----------------------------------------------*/

/* Public user code ----------------------------------------------------------*/ 

/**
  * @brief  Application Process Init.
  * @retval None
  */
void MainThreadProcessInit(void) {
  appProcess.state = APP_STATE_INIT;
}

/**
 * @brief The application entry point.
 */
void MainThreadProcessTask(void) {
  bool nErr = false;

  if(LPIT_FlagStatus(lptig) == true) {
    if(PortIrq_PMuxFlags(PIRQ_GLB) == true) {
      PortIrq_MuxProcessTask();
    }
    LPIT_FlagClear(lptig);
  }

  switch ( appProcess.state) {
    /* Application's initial state. */
    case APP_STATE_INIT: {
      PortMux_ProcessInit();
      PortIrq_ProcessInit();
      sysLED_ProcessInit();
      /* Enable System Power Control. */
      if(PortMux_OutStatus(POUT_PWR_CTL) != true) {
        PortMux_OutEnable(POUT_PWR_CTL);
        PortMux_OutEnable(POUT_AFE_WAKE);
      }
      /* Enable System Power RON Select. */
      if(PortMux_OutStatus(POUT_PWR_RON) != true) {
        PortMux_OutEnable(POUT_PWR_RON);
      }
      if(LPIT_ProcessEnable(true) != false) {
        nErr = true;
      }
     // if(RTC_ProcessInit() != false) {
      //  nErr = true;
     // }
      if(bspI2C_ProcessEnable(true) != false) {
        nErr = true;
      }
      if(PtoclStack_ProcessInit() != false) {
        nErr = true;
      }
      if(nErr != true) {
        appProcess.state = APP_STATE_POWER_UP_TASKS;
      } else {
        appProcess.state =  APP_STATE_SYSTEM_ERROR;
      }
      break;
    }
    /* Application's Power-Up state. */
    case APP_STATE_POWER_UP_TASKS: {
      BMS_ProcessInitial();
      PortMux_OutDisable(POUT_AFE_WAKE);
      if(Flash_ProcessInit() != false) {
        nErr = true;
      } else {
        CoulCount_ProcessInit();      
      }
      if(bqDeviceInitAFE() != false) {
        nErr = true;
      }
      if(nErr != true) {
        appProcess.state = App_STATE_WAKE_UP_TASKS;
      } else {
        appProcess.state =  APP_STATE_SYSTEM_ERROR;
      }
      break;
    }
    /* Application's System Wake-Up state. */
    case App_STATE_WAKE_UP_TASKS: {
      if(LPIT_FlagStatus(lp100ms) == true) {
        LPIT_FlagClear(lp100ms);
        sysLED_ProcessTask();
        appProcess.state = APP_STATE_SERVICE_TASKS;
      }
      break;
    }
    /* Application's Main_Service state. */
    case APP_STATE_SERVICE_TASKS: {
      PtoclStack_MessageTask();
      if(PortIrq_PMuxFlags(PIRQ_AFE_ALERT) == true) {
        bqDeviceAlartTrig(fAFE_IRQ);
        PortIrq_PMuxClear(PIRQ_AFE_ALERT);
      }
      if(LPIT_FlagStatus(lp10ms) == true) {
        LPIT_FlagClear(lp10ms);
      }
      if(LPIT_FlagStatus(lp50ms) == true) {
        LPIT_FlagClear(lp50ms);
        bqDeviceProcessTask();
        BMS_MainProcessTask();
        PtoclStack_ProcessTask();
      }
      
      if(LPIT_FlagStatus(lp100ms) == true) {
        LPIT_FlagClear(lp100ms);
        PortMux_OutProcessTask();
        sysLED_ProcessTask();
        CoulCountProcessTASK();
      }
      if(LPIT_FlagStatus(lpsec) == true) {
        LPIT_FlagClear(lpsec);
        RTCTimer_ProcessTask();
      }
      if(BMS_ReturnStatus() ==BMS_MODE_SHUTDOWN) {
        appProcess.state = APP_STATE_PWROFF_TASKS;
      }
      break;
    }
    /* Application's Power_Off Process state. */
    case APP_STATE_PWROFF_TASKS: {
      PtoclStack_MessageTask();
      BMS_MainProcessTask();
      if(BMS_ReturnStatus() == BMS_MODE_SHUTDOWN) {
        DataFlashSaveProcess();
        appProcess.state = APP_STATE_SYSTEM_OFF;
      }
      break;
    }
    /* Application's System Sleep state. */
    case APP_STATE_SYSTEM_SLEEP: {
      appProcess.state = APP_STATE_PWROFF_TASKS;
      break;
    }
    /* Application's System Shutdown state. */
    case APP_STATE_SYSTEM_OFF: {
      if(TP2101GetFlags(tp_FLG_BLT) == true) {
        NVIC_SystemReset();
      } else if(LPIT_FlagStatus(lp50ms) == true) {
        LPIT_FlagClear(lp50ms);
        bqDeviceProcessTask();
      }
      break;
    }
    case APP_STATE_SYSTEM_ERROR: {
      appProcess.state =  APP_STATE_PWROFF_TASKS;
      break;
    }
    default: break;
  }
}

/************************ (C) COPYRIGHT NucalTech **************END OF FILE****/