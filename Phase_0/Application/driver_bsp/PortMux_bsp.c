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
#include "PortMux_bsp.h"
#include "stm32l4xx_hal.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/
port_OutFlags_t port_OutFlags;
port_OutErr_t port_OutErr;

/*******************************************************************************
 * Private Functions
 ******************************************************************************/


/* FUNCTION**********************************************************************
 *
 * Function Name : LPIT_ProcessInit Description   : Initial LPIT process.
 *
 * Implements    : LPIT Process Initial
 * END*************************************************************************
 */
   /**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */  
void PortMux_ProcessInit(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    port_OutFlags.PWR_CTL = false;
    port_OutFlags.PWR_RON = false;
    port_OutFlags.PWR_ISO = false;
    port_OutFlags.BAT_CTMON = false;
    port_OutFlags.AFE_DSG = false;
    port_OutFlags.AFE_SHUT = false;
    port_OutFlags.AFE_WAKE = false;
    port_OutFlags.UART_ES = false;
    port_OutFlags.FANS_PWR = false;
    
    
    port_OutErr.PWR_CTL = false;
    port_OutErr.PWR_RON = false;
    port_OutErr.PWR_ISO = false;
    port_OutErr.BAT_CTMON = false;
    port_OutErr.AFE_DSG = false;
    port_OutErr.AFE_SHUT = false;
    port_OutErr.AFE_WAKE = false;    
    port_OutErr.UART_ES = false;
    port_OutErr.FANS_PWR = false;    
    
 /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOH_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();

  /* Configure SYS Main Power Control Output Level */
  HAL_GPIO_WritePin(PWR_CTL_PORT, PWR_CTL_PIN, GPIO_PIN_RESET);
  GPIO_InitStruct.Pin = PWR_CTL_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(PWR_CTL_PORT, &GPIO_InitStruct);
  
  /* Configure  SYS Main Power RON Select Output Level */
  HAL_GPIO_WritePin(PWR_RON_PORT, PWR_RON_PIN, GPIO_PIN_RESET);
  GPIO_InitStruct.Pin = PWR_RON_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(PWR_RON_PORT, &GPIO_InitStruct);  
    
  /* Configure  Isolation Power Control Output Level */
  HAL_GPIO_WritePin(PWR_ISO_PORT, PWR_ISO_PIN, GPIO_PIN_RESET);
  GPIO_InitStruct.Pin = PWR_ISO_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(PWR_ISO_PORT, &GPIO_InitStruct); 
  
  /* Configure  BAT Voltage Monitor Control Output Level */
  HAL_GPIO_WritePin(BAT_CTMON_PORT, BAT_CTMON_PIN, GPIO_PIN_RESET);
  GPIO_InitStruct.Pin = BAT_CTMON_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(BAT_CTMON_PORT, &GPIO_InitStruct);
  
  /* Configure  AFE DisCharge FET Output Level */
  HAL_GPIO_WritePin(AFE_DSG_PORT, AFE_DSG_PIN, GPIO_PIN_RESET);
  GPIO_InitStruct.Pin = AFE_DSG_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(AFE_DSG_PORT, &GPIO_InitStruct);  
  
  /* Configure  AFE Power Shutdown Output Level */
  HAL_GPIO_WritePin(AFE_SHUT_PORT, AFE_SHUT_PIN, GPIO_PIN_RESET);      
  GPIO_InitStruct.Pin = AFE_SHUT_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(AFE_SHUT_PORT, &GPIO_InitStruct);    
  
  /* Configure  AFE Power Wakeup Output Level */
  HAL_GPIO_WritePin(AFE_WAKE_PORT, AFE_WAKE_PIN, GPIO_PIN_RESET);   
  GPIO_InitStruct.Pin = AFE_WAKE_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(AFE_WAKE_PORT, &GPIO_InitStruct);
  
  /* Configure  AFE Power Wakeup Output Level */
  HAL_GPIO_WritePin(UART_ES_PORT, UART_ES_PIN, GPIO_PIN_RESET);   
  GPIO_InitStruct.Pin = UART_ES_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(UART_ES_PORT, &GPIO_InitStruct);
  
  /* Configure  AFE Power Wakeup Output Level */
  HAL_GPIO_WritePin(FANS_PWR_PORT, FANS_PWR_PIN, GPIO_PIN_RESET);   
  GPIO_InitStruct.Pin = FANS_PWR_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(FANS_PWR_PORT, &GPIO_InitStruct);  
}

bool PortMux_OutProcessTask(void) {
    bool n_err = false;
    uint8_t u8step = false;
    while(u8step != POUT_DEF_EOL) {
        switch(u8step) {
        case POUT_PWR_CTL:
            if(port_OutFlags.PWR_CTL == true) {
                if(HAL_GPIO_ReadPin(PWR_CTL_PORT, PWR_CTL_PIN)
                   != GPIO_PIN_SET) {
                       port_OutErr.PWR_CTL = true;
                       n_err = true;
                   }
            }
            break;
        case POUT_PWR_RON:
            if(port_OutFlags.PWR_RON == true) {
                if(HAL_GPIO_ReadPin(PWR_RON_PORT, PWR_RON_PIN)
                   != GPIO_PIN_SET) {
                       port_OutErr.PWR_RON = true;
                       n_err = true;
                   }
            }
            break;
        case POUT_PWR_ISO:
            if(port_OutFlags.PWR_ISO == true) {
                if(HAL_GPIO_ReadPin(PWR_ISO_PORT, PWR_ISO_PIN)
                   != GPIO_PIN_SET) {
                       port_OutErr.PWR_ISO = true;
                       n_err = true;
                   }
            }
            break;
        case POUT_BAT_CTMON:
            if(port_OutFlags.BAT_CTMON == true) {
                if(HAL_GPIO_ReadPin(BAT_CTMON_PORT, BAT_CTMON_PIN)
                   != GPIO_PIN_SET) {
                       port_OutErr.BAT_CTMON = true;
                       n_err = true;
                   }
            }
            break;

        case POUT_AFE_DSG:
            if(port_OutFlags.AFE_DSG == true) {
                if(HAL_GPIO_ReadPin(AFE_DSG_PORT, AFE_DSG_PIN)
                   != GPIO_PIN_SET) {
                       port_OutErr.AFE_DSG = true;
                       n_err = true;
                   }
            }
            break;

        case POUT_AFE_SHUT:
            if(port_OutFlags.AFE_SHUT == true) {
                if(HAL_GPIO_ReadPin(AFE_SHUT_PORT, AFE_SHUT_PIN)
                   != GPIO_PIN_SET) {
                       port_OutErr.AFE_SHUT = true;
                       n_err = true;
                   }
            }
            break;

        case POUT_AFE_WAKE:
            if(port_OutFlags.AFE_WAKE == true) {
                if(HAL_GPIO_ReadPin(AFE_WAKE_PORT, AFE_WAKE_PIN)
                   != GPIO_PIN_SET) {
                       port_OutErr.AFE_WAKE = true;
                       n_err = true;
                   }
            }
            break;
        case POUT_UART_ES:
            if(port_OutFlags.UART_ES == true) {
                if(HAL_GPIO_ReadPin(UART_ES_PORT, UART_ES_PIN)
                   != GPIO_PIN_SET) {
                       port_OutErr.UART_ES = true;
                       n_err = true;
                   }
            }
            break;
        case POUT_FANS_PWR:
            if(port_OutFlags.FANS_PWR == true) {
                if(HAL_GPIO_ReadPin(FANS_PWR_PORT, FANS_PWR_PIN)
                   != GPIO_PIN_SET) {
                       port_OutErr.FANS_PWR = true;
                       n_err = true;
                   }
            }
            break;            
        default: break;
      }
      u8step++;
    }
    return n_err;
}

bool PortMux_OutEnable(uint8_t idx) {
    bool n_err = false;
    switch(idx) {
    case POUT_PWR_CTL:
        if(port_OutErr.PWR_CTL != true) {
            HAL_GPIO_WritePin(PWR_CTL_PORT, PWR_CTL_PIN, GPIO_PIN_SET);
            port_OutFlags.PWR_CTL = true;
        } else {
            n_err = true;
        }
        break;
    case POUT_PWR_RON :
        if(port_OutErr.PWR_RON != true) {
            HAL_GPIO_WritePin(PWR_RON_PORT, PWR_RON_PIN, GPIO_PIN_SET);
            port_OutFlags.PWR_RON = true;
        } else {
            n_err = true;
        }
        break;
    case POUT_PWR_ISO :
        if(port_OutErr.PWR_ISO != true) {        
            HAL_GPIO_WritePin(PWR_ISO_PORT, PWR_ISO_PIN, GPIO_PIN_SET);
            port_OutFlags.PWR_ISO = true;
        } else {
            n_err = true;
        }
        break;
    case POUT_BAT_CTMON :
        if(port_OutErr.BAT_CTMON != true) {             
            HAL_GPIO_WritePin(BAT_CTMON_PORT, BAT_CTMON_PIN, GPIO_PIN_SET);
            port_OutFlags.BAT_CTMON = true;
        } else {
            n_err = true;
        }
        break;
    case POUT_AFE_DSG :
        if(port_OutErr.AFE_DSG != true) {             
            HAL_GPIO_WritePin(AFE_DSG_PORT, AFE_DSG_PIN, GPIO_PIN_SET);
            port_OutFlags.AFE_DSG = true;
        } else {
            n_err = true;
        }
        break;
    case POUT_AFE_SHUT :
        if(port_OutErr.AFE_SHUT != true) {             
            HAL_GPIO_WritePin(AFE_SHUT_PORT, AFE_SHUT_PIN, GPIO_PIN_SET);
            port_OutFlags.AFE_SHUT = true;
        } else {
            n_err = true;
        }
        break;
    case POUT_AFE_WAKE :
        if(port_OutErr.AFE_WAKE != true) {             
            HAL_GPIO_WritePin(AFE_WAKE_PORT, AFE_WAKE_PIN, GPIO_PIN_SET);
            port_OutFlags.AFE_WAKE = true;
        } else {
            n_err = true;
        }
    case POUT_UART_ES :
        if(port_OutErr.UART_ES != true) {             
            HAL_GPIO_WritePin(UART_ES_PORT, UART_ES_PIN, GPIO_PIN_SET);
            port_OutFlags.UART_ES = true;
        } else {
            n_err = true;
        }
    case POUT_FANS_PWR :
        if(port_OutErr.FANS_PWR != true) {             
            HAL_GPIO_WritePin(FANS_PWR_PORT, FANS_PWR_PIN, GPIO_PIN_SET);
            port_OutFlags.FANS_PWR = true;
        } else {
            n_err = true;
        }        
        break;        
    default:
        n_err = true;
        break;
    }
    return n_err;
}

void PortMux_OutDisable(uint8_t idx) {
    switch(idx) {
    case POUT_PWR_CTL:
        HAL_GPIO_WritePin(PWR_CTL_PORT, PWR_CTL_PIN,GPIO_PIN_RESET);
        port_OutFlags.PWR_CTL = false;
        port_OutErr.PWR_CTL = false;
        break;
    case POUT_PWR_RON :
        HAL_GPIO_WritePin(PWR_RON_PORT, PWR_RON_PIN,GPIO_PIN_RESET);
        port_OutFlags.PWR_RON = false;
        port_OutErr.PWR_RON = false;
        break;
    case POUT_PWR_ISO :
        HAL_GPIO_WritePin(PWR_ISO_PORT, PWR_ISO_PIN,GPIO_PIN_RESET);
        port_OutFlags.PWR_ISO = false;
        port_OutErr.PWR_ISO = false;
        break;
    case POUT_BAT_CTMON :
        HAL_GPIO_WritePin(BAT_CTMON_PORT, BAT_CTMON_PIN,GPIO_PIN_RESET);
        port_OutFlags.BAT_CTMON = false;
        port_OutErr.BAT_CTMON = false;
        break;
    case POUT_AFE_DSG :
        HAL_GPIO_WritePin(AFE_DSG_PORT, AFE_DSG_PIN,GPIO_PIN_RESET);
        port_OutFlags.AFE_DSG = false;
        port_OutErr.AFE_DSG = false;
        break;
    case POUT_AFE_SHUT :
        HAL_GPIO_WritePin(AFE_SHUT_PORT, AFE_SHUT_PIN,GPIO_PIN_RESET);
        port_OutFlags.AFE_SHUT = false;
        port_OutErr.AFE_SHUT = false;
        break;
    case POUT_AFE_WAKE :
        HAL_GPIO_WritePin(AFE_WAKE_PORT, AFE_WAKE_PIN,GPIO_PIN_RESET);
        port_OutFlags.AFE_WAKE = false;
        port_OutErr.AFE_WAKE = false;
        break;
    case POUT_UART_ES :
        HAL_GPIO_WritePin(UART_ES_PORT, UART_ES_PIN,GPIO_PIN_RESET);
        port_OutFlags.UART_ES = false;
        port_OutErr.UART_ES = false;
        break;   
    case POUT_FANS_PWR :
        HAL_GPIO_WritePin(FANS_PWR_PORT, FANS_PWR_PIN,GPIO_PIN_RESET);
        port_OutFlags.FANS_PWR = false;
        port_OutErr.FANS_PWR = false;
        break;           
    default: break;
    }
}

bool PortMux_OutStatus(uint8_t idx) {
    bool n_res = false;
    switch(idx) {
    case POUT_PWR_CTL:
        n_res = port_OutErr.PWR_CTL;
        break;
    case POUT_PWR_RON :
        n_res = port_OutErr.PWR_RON;
        break;
    case POUT_PWR_ISO :
        n_res = port_OutErr.PWR_ISO;
        break;
    case POUT_BAT_CTMON :
        n_res = port_OutErr.BAT_CTMON;
        break;
    case POUT_AFE_DSG :
        n_res = port_OutErr.AFE_DSG;
        break;
    case POUT_AFE_SHUT :
        n_res = port_OutErr.AFE_SHUT;
        break;
    case POUT_AFE_WAKE :
        n_res = port_OutErr.AFE_WAKE;
        break;
    case POUT_UART_ES :
        n_res = port_OutErr.UART_ES;
        break;   
    case POUT_FANS_PWR :
        n_res = port_OutErr.FANS_PWR;
        break;           

    default: break;
    }
    return n_res;
}
