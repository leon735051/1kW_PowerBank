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

/* Including necessary module. Cpu.h contains other modules needed for compiling.*/

#include "i2c_bsp.h"
#include "main.h"
#include <stdlib.h>

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define CRC8_ENB true
#define CRC8_KEY 0x107
/*******************************************************************************
 * Variables
 ******************************************************************************/
I2C_HandleTypeDef hi2c2;
DMA_HandleTypeDef hdma_i2c2_rx;
DMA_HandleTypeDef hdma_i2c2_tx;

/*******************************************************************************
 * Private Functions
 ******************************************************************************/
static void bspI2C_ProcessInit(void);
static uint8_t bspI2C_CRC(uint8_t *ptr, uint8_t len, uint16_t key);

/*FUNCTION**********************************************************************
 *
 * Function Name : LPIT_ProcessInit
 * Description   : Initial LPIT process.
 *
 * Implements    : LPIT Process Initial
 *END**************************************************************************/
/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void bspI2C_ProcessInit(void) {
/* Initialize I2C Master configuration;*/

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x20303E5D;
  hi2c2.Init.OwnAddress1 = 16;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */
}

static uint8_t bspI2C_CRC(uint8_t *ptr, uint8_t len, uint16_t key) {
    uint8_t i;
    uint8_t u8crc = false;
    while(len-- != false) {
        for(i=0x80; i!= false; i/=2) {
            if((u8crc & 0x80) != false) {
                u8crc *= 2;
                u8crc ^= key;
            } else {
                u8crc *= 2;            
            }
            if((*ptr & i)!=0) {
                u8crc ^= key;
            }
        }
        ptr++;
    }
    return(u8crc);
}

bool bspI2C_ProcessEnable(bool active) {
    bool nErr = false;
    if(active == true) {
        bspI2C_ProcessInit();
        if (HAL_I2C_Init(&hi2c2) != HAL_OK) {
            nErr = true;
            /** Configure Analogue filter */
        } else if(HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK) {
            nErr = true;
            /** Configure Digital filter */
        } else if(HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK) {
            nErr = true;
        }
    } else {
        if(HAL_I2C_DeInit(&hi2c2) != HAL_OK) {
            nErr = true;
        }
    }
    return nErr;
}


bool bspI2C_Write(uint8_t slv_addr, uint8_t *ptr, uint8_t len) {
    bool n_err = false;
    uint8_t u8TimeOut, u8i;
    uint8_t u8crc[3];
    uint8_t u8buf[10];
    u8TimeOut = I2C_TIMEOUT_TXIS_MS;
    /*
    if(CRC8_ENB == true) {
      // calculated over single-byte
      u8crc[0] = slv_addr;
      u8crc[1] = *(ptr);  // reg_addr
      u8crc[2] = *(ptr +1);  // byte0_data
      bspI2C_CRC(u8crc,3, CRC8_KEY);
      
      u8buf[0] = *ptr;       // reg_addr
      u8buf[1] = *ptr;       // reg_addr
      bspI2C_CRC(u8crc,3, CRC8_KEY);
      for(u8i = true; u8i < len; u8i++)
        u8buf[++] = *ptr++;
      
      }
    */
      
    
    
    
    
   
    while((HAL_I2C_GetState(&hi2c2) != HAL_I2C_STATE_READY) && (u8TimeOut != false)) {
        HAL_Delay(true);
        u8TimeOut--;
        if(u8TimeOut == false) {
            n_err = true;
        }
    }
    if(n_err != true) {

        if (HAL_I2C_Master_Transmit_DMA(&hi2c2, slv_addr, ptr, len) != HAL_OK) {
            n_err = true;
        } else {
            u8TimeOut = I2C_TIMEOUT_TXIS_MS;
            while ((HAL_I2C_GetState(&hi2c2) != HAL_I2C_STATE_READY) && (u8TimeOut != false)) {
                HAL_Delay(true);
                u8TimeOut--;
                if(u8TimeOut == false) {
                    n_err = true;
                }
            }
        }
    }
    if(HAL_I2C_GetError(&hi2c2) != HAL_I2C_ERROR_NONE) {
        n_err = true;
    }
    return n_err;
}


bool bspI2C_Read(uint8_t slv_addr, uint8_t reg, uint8_t *ptr, uint8_t len) {
    bool nErr = false;
    uint8_t u8step, u8TimeOut;
    uint8_t u8addr;
    uint8_t u8buf[1];
    u8buf[0] = reg;
    u8step = false;
    u8addr = slv_addr;
    while((nErr != true) && (u8step != 5)) {
        u8TimeOut = I2C_TIMEOUT_TXIS_MS;
        switch(u8step) {
        case 0:
            while((HAL_I2C_GetState(&hi2c2) != HAL_I2C_STATE_READY) && (u8TimeOut != false)) {
                HAL_Delay(true);
                u8TimeOut--;
                if(u8TimeOut == false) {
                    nErr = true;
                }
            }
            break;
        case 1: 
            if (HAL_I2C_Master_Transmit_DMA(&hi2c2, u8addr, u8buf, 1) != HAL_OK) {
                nErr = true;
            } else {
                while ((HAL_I2C_GetState(&hi2c2) != HAL_I2C_STATE_READY) && (u8TimeOut != false)) {
                    HAL_Delay(true);
                    u8TimeOut--;
                    if(u8TimeOut == false) {
                        nErr = true;
                    }
                }
            }
            break;
        case 2:
            if(HAL_I2C_GetError(&hi2c2) != HAL_I2C_ERROR_NONE) {
                nErr = true;
            }
            break;
        case 3:
            if(HAL_I2C_Master_Receive_DMA(&hi2c2, u8addr +1 , ptr, len) != HAL_OK) {
                nErr = true;
            } else {
                while ((HAL_I2C_GetState(&hi2c2) != HAL_I2C_STATE_READY) && (u8TimeOut != false)) {
                    HAL_Delay(true);
                    u8TimeOut--;
                    if(u8TimeOut == false) {
                        nErr = true;
                    }
                }
            }
            break;
        case 4:
            if(HAL_I2C_GetError(&hi2c2) != HAL_I2C_ERROR_NONE) {
                nErr = true;
            }
            break;
        default: break;
        }
        u8step++;
    }
    return nErr;
}

/**
  * @brief  Tx Transfer completed callback.
  * @param  I2cHandle: I2C handle.
  * @note   This example shows a simple way to report end of DMA Tx transfer, and
  *         you can add your own implementation.
  * @retval None
  */

void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *I2cHandle)
{
    if (I2cHandle->Instance == hi2c2.Instance)
    {
        HAL_DMA_Abort_IT(I2cHandle->hdmatx);
    }
}

/**
  * @brief  Rx Transfer completed callback.
  * @param  I2cHandle: I2C handle
  * @note   This example shows a simple way to report end of DMA Rx transfer, and
  *         you can add your own implementation.
  * @retval None
  */
void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *I2cHandle)
{
    if (I2cHandle->Instance == hi2c2.Instance)
    {
        HAL_DMA_Abort_IT(I2cHandle->hdmarx);
    }
}
/******************************************************************************
 * EOF
 *****************************************************************************/
