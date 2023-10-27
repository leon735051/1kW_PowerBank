/**
  ******************************************************************************
  * @file    rtc_bsp.c
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
#ifndef RTC_BSP_H
#define RTC_BSP_H

#include "stm32l4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

/*! @file rtc_bsp.h*/

/*******************************************************************************
 * Definitions
 ******************************************************************************/


/************************************************************
* Initial Other Variable
************************************************************/


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
 * @brief LPIT process Initial.
 *
 * @param[in] The LPIT initial process.
 * @return NONE
 */


/*!
 * @brief Interrupt handler for a LPIT instance.
 *
 * @param[in] The LPIT instance process.
 * @return NONE
 */
extern bool RTC_ProcessInit(void);
extern bool RTCTimer_SET_Clcok(uint8_t *msg);
extern bool RTCTimer_ProcessTask(void);
extern RTC_TimeTypeDef RTC_RetrunTime(void);
extern RTC_DateTypeDef RTC_RetrunDate(void);


/*! @}*/

#if defined (__cplusplus)
}
#endif

#endif /* PORTIRQ_MOD_H */
/*******************************************************************************
 * EOF
 ******************************************************************************/
