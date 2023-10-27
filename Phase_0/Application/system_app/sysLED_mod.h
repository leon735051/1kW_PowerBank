/**
  ******************************************************************************
  * @file    sysLED_mod.h
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef SYSLED_MOD_H
#define SYSLED_MOD_H

#if defined (__cplusplus)
extern "C" {
#endif
    
/* Includes ------------------------------------------------------------------*/
#include <stdbool.h>
#include <stdint.h>

/** @addtogroup SYS_MOD_Application
  * @{
  */

/** @addtogroup SYSLED
  * @{
  */
/* Exported types ------------------------------------------------------------*/
/** @defgroup SYSLED_Private_Variables SYSLED Exported Types
  * @{
  */
typedef struct {
	bool TASK_1st;
	bool TASK_2nd;
}sysLED_status_flag_t;


typedef struct{
	uint8_t tmr_1st;
	uint8_t tmr_2nd;
} sysLED_config_info_t;
/**
  * @}
  */

// SYS LED Ready Control Port
#define LED_STBY_BASE PORTB
#define LED_STBY_PORT GPIOB
#define LED_STBY_PIN  GPIO_PIN_6

// SYS LED Fault Control Port
#define LED_ALARM_BASE PORTB
#define LED_ALARM_PORT GPIOB
#define LED_ALARM_PIN  GPIO_PIN_7
    
// SYS LED Stage_1 Control Port
#define LED_STG1_BASE PORTA
#define LED_STG1_PORT GPIOA
#define LED_STG1_PIN  GPIO_PIN_15

// SYS LED Stage_2 Control Port
#define LED_STG2_BASE PORTC
#define LED_STG2_PORT GPIOC
#define LED_STG2_PIN  GPIO_PIN_10
    
// SYS LED Stage_3 Control Port
#define LED_STG3_BASE PORTC
#define LED_STG3_PORT GPIOC
#define LED_STG3_PIN  GPIO_PIN_11
    
// SYS LED Stage_4 Control Port
#define LED_STG4_BASE PORTC
#define LED_STG4_PORT GPIOC
#define LED_STG4_PIN  GPIO_PIN_12 


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
extern void sysLED_ProcessInit(void);
/**
  * @}
  */
     
/** @addtogroup SYSLED_Exported_Functions_Task
  * @{
  */
/* Peripheral Control functions  */
extern void sysLED_ProcessTask(void);
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

#endif /* SYSLED_MOD_H */

/********************************* (C) COPYRIGHT NucalTech *****END OF FILE****/