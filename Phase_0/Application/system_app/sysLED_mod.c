/**
  ******************************************************************************
  * @file    sysLED_mod.c
  * @author  Oliver .Chiu.
  * @brief   LED Display HAL Application Process.
  *          This file provides firmware functions to manage the following
  *          functionalities of the LED Display Process:
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

/* Includes ------------------------------------------------------------------*/
#include "sysLED_mod.h"
#include "stm32l4xx_hal.h"
#include "main.h"
#include "module_app/flash_mod/flash_mod.h"
#include "module_app/bms_mod/bms_mod.h"
/** @addtogroup SYS_MOD_Application
  * @{
  */

/** @defgroup SYSLED
  * @brief LED Display App
  * @{
  */
#define SYSLED_MOD_ENABLE
#ifdef SYSLED_MOD_ENABLE

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define LED_FLASH_1ST 4  // 500ms
#define LED_FLASH_2ST 9  // 1Sec    
/* Private macro -------------------------------------------------------------*/
/** @defgroup SYSLED_Private_Macro SYSLED Private Macro
  * @{
  */
static void sysLED_ActiveToggle(void);
static void sysLED_FaultToggle(void);
static void sysLED_ActiveOn(bool active);
static void sysLED_FaultOn(bool active);
static void sysLED_Disable(void);
/**
  * @}
  */

/* Private variables ---------------------------------------------------------*/
/** @defgroup SYSLED_Private_Variables SYSLED Private Variables
  * @{
  */
sysLED_status_flag_t sysLED_flag;
sysLED_config_info_t sysLED_info;
/**
  * @}
  */

/* Private function prototypes -----------------------------------------------*/

/** @addtogroup SYSLED_Private_Functions
  * @{
  */

/** @addtogroup SYSLED_Private_Functions_Group1
 *  @brief    Configuration functions
 *
@verbatim
 ===============================================================================
              ##### Configuration functions #####
 ===============================================================================

@endverbatim
  * @{
  */

/**
  * @brief  Set Stby Led Toggle..
  * @param  None.
  * @retval None.
  */
static void sysLED_ActiveToggle(void) {
  HAL_GPIO_TogglePin(LED_STBY_PORT, LED_STBY_PIN);
}

/**
  * @brief  Set Fault Led Toggle.
  * @param  None.
  * @retval None.
  */
static void sysLED_FaultToggle(void) {
  HAL_GPIO_TogglePin(LED_ALARM_PORT, LED_ALARM_PIN);
}

/**
  * @brief  Set Stby Led Display Always On.
  * @param  None.
  * @retval None.
  */
static void sysLED_ActiveOn(bool active) {
  if(active == true) {
    HAL_GPIO_WritePin(LED_STBY_PORT, LED_STBY_PIN,GPIO_PIN_SET);
  } else {
    HAL_GPIO_WritePin(LED_STBY_PORT, LED_STBY_PIN,GPIO_PIN_RESET);
  }
}
/**
  * @brief  Set Alarm Led Dplay Always on.
  * @param  None.
  * @retval None.
  */
static void sysLED_FaultOn(bool active) {
  if(active == true) {
    HAL_GPIO_WritePin(LED_ALARM_PORT, LED_ALARM_PIN,GPIO_PIN_SET);
  } else {
    HAL_GPIO_WritePin(LED_ALARM_PORT, LED_ALARM_PIN,GPIO_PIN_RESET);
  }
}

/**
  * @brief  Set Led Display All off.
  * @param  None.
  * @retval None.
  */
static void sysLED_Disable(void) {
    HAL_GPIO_WritePin(LED_STBY_PORT, LED_STBY_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED_ALARM_PORT, LED_ALARM_PIN, GPIO_PIN_RESET);
}

/**
  * @}
  */

/**
  * @}
  */

/* Exported variables --------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/

/** @addtogroup SYSLED_Exported_Functions
  * @{
  */

/** @addtogroup SYSLED_Exported_Functions_Config
 *  @brief    Configuration functions
 *
@verbatim
 ===============================================================================
              ##### Configuration functions #####
 ===============================================================================

@endverbatim
  * @{
  */

/**
  * @brief  sysLED_ProcessInit.
  * @param  None.
  * @retval None.
  */
void sysLED_ProcessInit(void) {
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /*Configure SYS LED Ready Control Output Level */
  HAL_GPIO_WritePin(LED_STBY_PORT, LED_STBY_PIN, GPIO_PIN_RESET);
  GPIO_InitStruct.Pin = LED_STBY_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_STBY_PORT, &GPIO_InitStruct);
  
  /*Configure SYS LED Fault Control Output Level */
  HAL_GPIO_WritePin(LED_ALARM_PORT, LED_ALARM_PIN, GPIO_PIN_RESET);
  GPIO_InitStruct.Pin = LED_ALARM_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_ALARM_PORT, &GPIO_InitStruct);

  /*Configure SYS LED Stage_1 Control Output Level */
  HAL_GPIO_WritePin(LED_STG1_PORT, LED_STG1_PIN, GPIO_PIN_RESET);
  GPIO_InitStruct.Pin = LED_STG1_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_STG1_PORT, &GPIO_InitStruct);
  
  /*Configure SYS LED Stage_2 Control Output Level */
  HAL_GPIO_WritePin(LED_STG2_PORT, LED_STG2_PIN, GPIO_PIN_RESET);
  GPIO_InitStruct.Pin = LED_STG2_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_STG2_PORT, &GPIO_InitStruct);
  
  /*Configure SYS LED Stage_3 Control Output Level */
  HAL_GPIO_WritePin(LED_STG3_PORT, LED_STG3_PIN, GPIO_PIN_RESET);
  GPIO_InitStruct.Pin = LED_STG3_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_STG3_PORT, &GPIO_InitStruct);
  
  /*Configure SYS LED Stage_4 Control Output Level */
  HAL_GPIO_WritePin(LED_STG4_PORT, LED_STG4_PIN, GPIO_PIN_RESET);
  GPIO_InitStruct.Pin = LED_STG4_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_STG4_PORT, &GPIO_InitStruct);  
}

/**
  * @}
  */

/** @addtogroup SYSLED_Exported_Functions_Task
 *  @brief SYSLED Task functions.
 *
@verbatim
 ===============================================================================
                       ##### Process operation functions #####
 ===============================================================================

@endverbatim
  * @{
  */

/**
  * @brief  sysLED Main Process Task.
  * @param  None.
  * @retval None.
  */

void sysLED_ProcessTask(void) {
	uint8_t u8idx;
	if(sysLED_info.tmr_1st >= LED_FLASH_1ST) {   // 500ms
		sysLED_info.tmr_1st = false;
		sysLED_flag.TASK_1st = true;        
	} else {
		sysLED_info.tmr_1st++;
	}
	if(sysLED_info.tmr_2nd >= LED_FLASH_2ST) {   // 1Sec
		sysLED_info.tmr_2nd = false;
		sysLED_flag.TASK_2nd = true;
	}
	else {
		sysLED_info.tmr_2nd++;
	}
	if(g_sConfig.LED_DISPLAY == true) {
		u8idx = BMS_ReturnStatus();
		switch(u8idx) {
		case BMS_MODE_NONE:
			sysLED_ActiveOn(true);
			if(sysLED_flag.TASK_1st == true) {
				sysLED_FaultToggle();
			}
			break;
		case BMS_MODE_CHG:
			if(sysLED_flag.TASK_2nd == true) {
				sysLED_ActiveOn(true);
				sysLED_FaultToggle();
			}
			break;
		case BMS_MODE_Ready:
      if(sysLED_flag.TASK_1st == true) {
			sysLED_ActiveToggle();
			sysLED_FaultOn(false);
      }
			break;
		case BMS_MODE_Enable:
			if(sysLED_flag.TASK_2nd == true) {
				sysLED_ActiveToggle();
				sysLED_FaultOn(false);
			}
			break;
		case BMS_MODE_Error:
			sysLED_ActiveOn(false);
			sysLED_FaultToggle();
			break;
		case BMS_MODE_Sleep:
			sysLED_Disable();
			break;
		case BMS_MODE_DSGEND:
      sysLED_ActiveOn(true);
			if(sysLED_flag.TASK_2nd == true) {
				sysLED_FaultToggle();
			}
			break;
		case BMS_MODE_CHGEND:
      sysLED_ActiveOn(true);
			if(sysLED_flag.TASK_2nd == true) {
        sysLED_FaultToggle();
			}
			break;
		case BMS_MODE_SHUTDOWN:
			sysLED_Disable();
			break;
		default: break;
		}
	}
	sysLED_flag.TASK_1st = false;
	sysLED_flag.TASK_2nd = false;
}
/**
  * @}
  */

/**
  * @}
  */

#endif /* SYS_MODULE_ENABLED */
/**
  * @}
  */

/**
  * @}
  */

/********************************* (C) COPYRIGHT NucalTech *****END OF FILE****/