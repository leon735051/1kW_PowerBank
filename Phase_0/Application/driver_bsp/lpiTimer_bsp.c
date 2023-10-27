/**
  ******************************************************************************
  * @file    lpiTimer_bsp.c
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


/* Includes ------------------------------------------------------------------*/
#include "lpiTimer_bsp.h"

/** @addtogroup  SYS_BSP_Driver @{
  */

/** @defgroup    LPITIMER_BSP (group_title)
  * @brief       Low Power Timer bsp Driver @{
  */
#define LPITIMER_BSP_ENABLE
#ifdef LPITIMER_BSP_ENABLE

/* Private typedef ----------------------------------------------------------- */
/* Private define ------------------------------------------------------------ */
/* Private macro -------------------------------------------------------------*/
/** @defgroup SYSLED_Private_Macro LPITIMER Private Macro
  * @{
  */
static void LPIT_ProcessInit(void);
/**
  * @}
  */

/* Private variables --------------------------------------------------------- */
/** @defgroup   SYSLED_Private_Variables LPITIMER Private Variables
  * @{
  */
lpiTimer_val_t lpiTimer_val;
lpiTimer_flag_t lpiTimer_flag;
LPTIM_HandleTypeDef hlptim1;
/**
  * @}
  */

/* Private function prototypes -----------------------------------------------*/

/** @addtogroup LPITIMER_Private_Functions
  * @{
  */

/** @addtogroup LPITIMER_Private_Functions_Group1
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
  * @brief LPTIM1 Initialization Function
  * @param None
  * @retval None
  */ 
 


 /**
  * 
  */
 
/**
  * @brief LPTIM1 Initialization Function
  * @param None
  * @retval None
  */
static void LPIT_ProcessInit(void) {
    lpiTimer_val.tsec = false;
    lpiTimer_val.t100ms = false;
    lpiTimer_val.t50ms  = false;
    lpiTimer_val.t10ms  = false;
    lpiTimer_flag.tsec = false;
    lpiTimer_flag.t100ms = false;
    lpiTimer_flag.t10ms = false;
    lpiTimer_flag.t50ms = false;
    lpiTimer_flag.ftrig = false;
    
  hlptim1.Instance = LPTIM1;
  hlptim1.Init.Clock.Source = LPTIM_CLOCKSOURCE_APBCLOCK_LPOSC;
  hlptim1.Init.Clock.Prescaler = LPTIM_PRESCALER_DIV128;
  hlptim1.Init.Trigger.Source = LPTIM_TRIGSOURCE_SOFTWARE;
  hlptim1.Init.OutputPolarity = LPTIM_OUTPUTPOLARITY_HIGH;
  hlptim1.Init.UpdateMode = LPTIM_UPDATE_IMMEDIATE;
  hlptim1.Init.CounterSource = LPTIM_COUNTERSOURCE_INTERNAL;
  hlptim1.Init.Input1Source = LPTIM_INPUT1SOURCE_GPIO;
  hlptim1.Init.Input2Source = LPTIM_INPUT2SOURCE_GPIO;
}

/**
  * @}
  */

/**
  * @}
  */

/* Exported variables --------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/

/** @addtogroup LPITIMER_Exported_Functions
  * @{
  */

/** @addtogroup LPITIMER_Exported_Functions_Config
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

bool LPIT_ProcessEnable(bool active) {
    bool n_err;
    n_err = false;
    if(active == true) {
        LPIT_ProcessInit();
        if (HAL_LPTIM_Init(&hlptim1) != HAL_OK) {
            n_err = true;
        } else {
            if(HAL_LPTIM_Counter_Stop_IT(&hlptim1) != HAL_OK) {
                n_err = true;
            }
        }
        if(n_err != true) {
            if(HAL_LPTIM_Counter_Start_IT(&hlptim1, 380) != HAL_OK) {
              n_err = true;
            }
        }
    } else {
        if(HAL_LPTIM_Counter_Stop_IT(&hlptim1) != HAL_OK) {
            n_err = true;
        } else if(HAL_LPTIM_DeInit(&hlptim1) != HAL_OK) {
          n_err = true;
        }
    }
    return n_err;
}
/**
 * @param
 * @return
 */
bool LPIT_FlagStatus(uint8_t idx) {
    bool n_res;
    n_res = false;
    switch(idx) {
    case lp10ms:
        n_res = lpiTimer_flag.t10ms;
        break;
    case lp50ms:
        n_res = lpiTimer_flag.t50ms;
        break;
    case lp100ms:
        n_res = lpiTimer_flag.t100ms;
        break;
    case lpsec:
        n_res = lpiTimer_flag.tsec;
        break;
    case lptig:
        n_res = lpiTimer_flag.ftrig;
        break;
    default: break;
    }
    return n_res;
}

bool LPIT_FlagClear(uint8_t idx) {
    bool n_res;
    switch(idx) {
    case lp10ms:
        lpiTimer_flag.t10ms = false;
        n_res = lpiTimer_flag.t10ms;
        break;
    case lp50ms:
        lpiTimer_flag.t50ms = false;
        n_res = lpiTimer_flag.t50ms;
        break;
    case lp100ms:
        lpiTimer_flag.t100ms = false;
        n_res = lpiTimer_flag.t100ms;
        break;
    case lpsec:
        lpiTimer_flag.tsec = false;
        n_res = lpiTimer_flag.tsec;
        break;
    case lptig:
        lpiTimer_flag.ftrig = false;
        n_res = lpiTimer_flag.ftrig;
        break;
	default: break;
    }
    return n_res;
}

/**
  * @}
  */

/** @addtogroup LPITIMER_Handle_Interrupt_Task
 *  @brief LPITIMER Interrupt Task.
 *
@verbatim
 ===============================================================================
                       ##### Interrupt Process operation Task #####
 ===============================================================================

@endverbatim
  * @{
  */

/**
  * @brief  LpiTimer Interrupt Task.
  * @param  LPTIM_HandleTypeDef.
  * @retval None.
  */

void HAL_LPTIM_AutoReloadMatchCallback(LPTIM_HandleTypeDef *hlptim) {
    lpiTimer_flag.ftrig = true;
    if(lpiTimer_val.t10ms >= 9) {
        lpiTimer_flag.t10ms = true;
        lpiTimer_val.t10ms = false;
    } else {
        lpiTimer_val.t10ms++;
    }
    if(lpiTimer_val.t50ms >= 49) {
        lpiTimer_val.t50ms = false;
        lpiTimer_flag.t50ms = true;
    }
    else {
        lpiTimer_val.t50ms++;
    }
    if(lpiTimer_val.t100ms >= 99) {
        lpiTimer_flag.t100ms = true;
        lpiTimer_val.t100ms = false;
    }else {
            lpiTimer_val.t100ms++;
        }
    if(lpiTimer_val.tsec >= 999) {
        lpiTimer_flag.tsec = true;
        lpiTimer_val.tsec = false;
    } else {
        lpiTimer_val.tsec++;
    }
}
/**
  * @}
  */

/**
  * @}
  */

#endif /* SYS_BSP_ENABLED */
/**
  * @}
  */

/**
  * @}
  */
/********************************* (C) COPYRIGHT NucalTech *****END OF FILE****/