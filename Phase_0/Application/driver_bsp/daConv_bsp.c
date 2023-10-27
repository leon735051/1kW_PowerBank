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

/* Including necessary module. Cpu.h contains other modules needed for compiling.*/
#include "daConv_bsp.h"
#include "stm32l4xx_hal.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define AdConv_Gain 435

/*******************************************************************************
 * Variables
 ******************************************************************************/
/* Variables in which we store data from ADC */
     uint16_t adcRawValue;
     uint16_t adcMax;
     
     DMA_HandleTypeDef hdma_dac_ch1;
     DAC_HandleTypeDef hdac1;
 
/*******************************************************************************
 * Private Functions
 ******************************************************************************/




/**
  * @brief DAC1 Initialization Function
  * @param None
  * @retval None
  */
bool daConv_ProcessInit(bool active) {
    bool n_err = false;
    DAC_ChannelConfTypeDef sConfig = {0};
    hdac1.Instance = DAC1;
    if(active == false) {
        if(HAL_DAC_DeInit(&hdac1) != HAL_OK) {
          n_err = true;
        }
    }
    else {
        if (HAL_DAC_Init(&hdac1) != HAL_OK) {
            n_err = true;
        }
        /** Configure Regular Channel*/
        sConfig.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;
        sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
        sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
        sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_DISABLE;
        sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
        if (HAL_DAC_ConfigChannel(&hdac1, &sConfig,DAC_CHANNEL_1) != HAL_OK) {
            n_err = true;
        }

    }
    return n_err;
}

/**
  * @brief DAC1 Initialization Function
  * @param None
  * @retval None
  */
bool daConv_ProcessEnable(bool active) {
    bool n_err = false;
    if(active == true) {
	   HAL_DAC_Start(&hdac1,DAC_CHANNEL_1);
           daConv_SetMode(daCv_L0, false);
    } else {
        HAL_DAC_Stop(&hdac1,DAC_CHANNEL_1);
    }
    return n_err;
}

/**
  * @brief DAC1 Initialization Function
  * @param None
  * @retval None
  */
bool daConv_SetMode(uint8_t nIdx, uint8_t cal) {
    bool nErr = false;
    uint16_t u16tmp;
    switch(nIdx) {
        case daCv_L0:
            u16tmp = 0; 
            break;
        case daCv_L1:
            u16tmp = 2548; 
            break;        
        case daCv_L2:
            u16tmp = 3048; 
            break;
        case daCv_L3:
            u16tmp = 4048; 
            break;
        case daCv_L4:
            u16tmp = 5048;
        default: break;
    }
    if(cal & 0x80 != 0x80) {
        u16tmp = u16tmp + cal;
    } else {
        u16tmp = u16tmp - cal;
    }
    if(HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, u16tmp) != HAL_OK) {
        nErr = true;
    }
    return nErr;
}

/******************************************************************************
 * EOF
 *****************************************************************************/
