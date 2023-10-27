/**
  ******************************************************************************
  * @file    lpiTimer_bsp.h
  * @author  Oliver .Chiu.
  * @brief   Low Power Timer Board Support Package Driver.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef LPITIMER_BSP_H
#define LPITIMER_BSP_H

#if defined (__cplusplus)
extern "C" {
#endif
    
/* Includes ------------------------------------------------------------------*/
#include <stdbool.h>
#include <stdint.h>
#include "stm32l4xx_hal.h"

/** @addtogroup SYS_BSP_Driver
  * @{
  */

/** @addtogroup LPITIMER_BSP
  * @{
  */
/* Exported types ------------------------------------------------------------*/
/** @defgroup LPITIMER_Private_Variables LPITIMER Exported Types
  * @{
  */

typedef enum {
        lptig = 0x00U,
        lp10ms,
	lp50ms,
	lp100ms,
	lpsec,
}lpiTimer_enum_t;

typedef struct
{	uint16_t tsec;
	uint8_t t100ms;
	uint8_t t50ms;
        uint8_t t10ms;
} lpiTimer_val_t;

typedef struct
{
	bool tsec;
	bool t100ms;
	bool t10ms;
	bool t50ms;
	bool ftrig;
} lpiTimer_flag_t;
/**
  * @}
  */

/* Exported constants --------------------------------------------------------*/
/* Exported macros -----------------------------------------------------------*/
/* Private macros ------------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
/** @addtogroup SYSLED_Exported_Functions
  * @{
  */

/** @addtogroup SYSLED_Exported_Functions_Config
  *  @brief    Configuration functions
  *
  * @{
  */
/* Initialization and  functions  */

/**
  * @}
  */
     
/** @addtogroup SYSLED_Exported_Functions_Task
  * @{
  */
/* Peripheral Control functions  */
extern bool LPIT_ProcessEnable(bool active);
extern bool LPIT_FlagStatus(uint8_t idx);
extern bool LPIT_FlagClear(uint8_t idx);
extern void HAL_LPTIM_AutoReloadMatchCallback(LPTIM_HandleTypeDef *hlptim);
/**
  * @}
  */


/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

#if defined (__cplusplus)
}
#endif

#endif /* LPITIMER_BSP_H */

/********************************* (C) COPYRIGHT NucalTech *****END OF FILE****/