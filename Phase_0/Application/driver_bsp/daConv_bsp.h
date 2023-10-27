/**
  ******************************************************************************
  * @file    daConv_bsp.c
  * @author  Oliver .Chiu.
  * @brief   Board Support Package Driver.
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
#ifndef DACONV_BSP_H
#define DACONV_BSP_H

#include <stdint.h>
#include <stdbool.h>
#include "stm32l4xx_hal.h"
/*! @file adConv_bsp.h*/

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/* USER CODE END Private defines */
/************************************************************
* Initial Other Variable
************************************************************/
#define daConvScanCH 1
typedef enum {
	daCovFETPACK = 0x00U,
	daConvVREF,
  daCovINP_TEMP,

} daConv_ValIdx_t;

typedef enum {
    daCv_L0 = false,
    daCv_L1,
    daCv_L2,
    daCv_L3,
    daCv_L4,
} daConv_Level_enum;

typedef struct
{
  uint8_t DmaStatus;
	uint16_t ConvRawValue[daConvScanCH];
	uint16_t ConvCalValue[daConvScanCH];
	bool ConvResult[daConvScanCH];
	bool ConvFlags;
  bool ConvEnable;
} daConv_mod_t;

/*******************************************************************************
 * API
 ******************************************************************************/

#if defined (__cplusplus)
extern "C" {
#endif

/*!
 * @name Converter
 * General functions.
 */
/*! @{*/


/*!
 * @brief ADC Process Initial.
 *
 * @param[in] The ADC initial process.
 * @return NONE
 */


/*!
 * @brief ADC Process Task.
 *
 * @param[in] The ADC Task process.
 * @return NONE
 */
extern bool daConv_ProcessInit(bool active);
extern void daConv_ProcessTask(void);
extern bool daConv_ProcessEnable(bool active);
extern bool daConv_GetFlags(uint8_t idx);
extern bool daConv_SetMode(uint8_t nIdx, uint8_t cal);
extern uint16_t daConv_GetValue(uint8_t idx);

/*! @}*/

#if defined (__cplusplus)
}
#endif

#endif /* ADCONV_MOD_H */
/*******************************************************************************
 * EOF
 ******************************************************************************/
