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
#ifndef __I2C_BSP_H
#define __I2C_BSP_H


/*! @file lpiTimer_mod.h*/
#include "stm32l4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

/*******************************************************************************
 * Definitions
 ******************************************************************************/

#define I2C_WAIT_TIME 0xffff
#define I2C_CRC_KEY 0x107
#define I2C_TIMEOUT_TXIS_MS 100

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

extern bool bspI2C_ProcessEnable(bool active);
extern bool bspI2C_Write(uint8_t slv_addr, uint8_t *ptr, uint8_t len);
extern bool bspI2C_Read(uint8_t slv_addr, uint8_t reg, uint8_t *ptr, uint8_t len);
extern void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *I2cHandle);
extern void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *I2cHandle);
/*! @}*/

#if defined (__cplusplus)
}
#endif

#endif /* LPII2C_MOD_H */
/*******************************************************************************
 * EOF
 ******************************************************************************/
