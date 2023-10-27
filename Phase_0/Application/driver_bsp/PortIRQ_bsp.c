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

#include "PortIRQ_bsp.h"
#include "stm32l4xx_hal.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/


/*******************************************************************************
 * Variables
 ******************************************************************************/
portIrq_Trig_t portIrqTRG;
portIrq_Flags_t portIrqFLG;
portIrq_Timer_t portIrqTMR;
/*******************************************************************************
 * Private Functions
 ******************************************************************************/



/*FUNCTION**********************************************************************
 *
 * Function Name : LPIT_ProcessInit
 * Description   : Initial LPIT process.
 *
 * Implements    : LPIT Process Initial
 *END**************************************************************************/

void PortIrq_ProcessInit(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    portIrqFLG.GLB = false;
    
    portIrqFLG.AFE_ALT = false;
    portIrqTRG.AFE_ALT = false;
    
    /* GPIO Ports Clock Enable */
    // Port Clock Enable Init by PortMux_bsp.c
    
  /*Configure GPIO pins : AFE_ALERT_Pin  */
  GPIO_InitStruct.Pin = AFE_ALERT_PIN | WAKE_SIG_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(WAKE_SIG_PORT, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);
  if(HAL_GPIO_ReadPin(WAKE_SIG_PORT, WAKE_SIG_PIN) == true) {
      portIrqFLG.WAKE_SIG = true;
  }
}

void PortIrq_MuxProcessTask(void) {
    bool nRes = false;
    if(portIrqTRG.AFE_ALT != false) {
      nRes = HAL_GPIO_ReadPin(AFE_ALERT_PORT, AFE_ALERT_PIN);
      if(nRes != portIrqFLG.AFE_ALT) {
        portIrqFLG.AFE_ALT = nRes;
      }
      portIrqTRG.AFE_ALT= false;
    }
    if(portIrqTRG.WAKE_SIG != false) {
      nRes = HAL_GPIO_ReadPin(WAKE_SIG_PORT, WAKE_SIG_PIN);
      if(nRes != portIrqFLG.WAKE_SIG) {
          if(portIrqTMR.WAKE_SIG > PORT_IRQ_TRIG_MS -1) {
              portIrqFLG.WAKE_SIG = nRes;
              portIrqTRG.WAKE_SIG = false;
              portIrqTMR.WAKE_SIG = false;
          } else {
            portIrqTMR.WAKE_SIG++;
          }
      } else {
          portIrqTRG.WAKE_SIG = false;
          portIrqTMR.WAKE_SIG = false;      
      }
    }
    portIrqFLG.GLB = false;
}

bool PortIrq_PMuxFlags(uint8_t idx) {
    bool nRes;
    
    switch(idx) {
    case PIRQ_GLB:
        nRes = portIrqFLG.GLB;
        break;
    case PIRQ_AFE_ALERT:
        nRes = portIrqFLG.AFE_ALT;
        break;
    case PIRQ_WAKE_SIG:
        nRes = portIrqFLG.WAKE_SIG;
        break;          
    default:
        nRes = false;
        break;
    }
    return nRes;
}

bool PortIrq_PMuxClear(uint8_t idx) {
    bool n_res;
    switch(idx) {
    case PIRQ_GLB:
        portIrqFLG.GLB = false;
        n_res = portIrqFLG.GLB;
        break;
    case PIRQ_AFE_ALERT:
        portIrqFLG.AFE_ALT = false;
        n_res = portIrqFLG.AFE_ALT;
        break;
    case PIRQ_WAKE_SIG:
        portIrqFLG.WAKE_SIG = false;
        break;           
    default:
        n_res = false;
        break;
    }
    return n_res;
}

/**
  * @brief EXTI line detection callbacks
  * @param GPIO_Pin: Specifies the pins connected EXTI line
  * @retval None
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    portIrqFLG.GLB = true;
    switch(GPIO_Pin) {
    case AFE_ALERT_PIN:
        portIrqTRG.AFE_ALT  = true;
        break;
    case WAKE_SIG_PIN:
        portIrqTRG.WAKE_SIG = true;
        break;
    default:
        break;
    }
}

/******************************************************************************
 * EOF
 *****************************************************************************/
