/**
  ******************************************************************************
  * @file    CANpal_bsp.c
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
#include "CANpal_bsp.h"
#include <stdbool.h>
/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define TX_STD_MAILBOX (0UL)
#define TX_EXT_MAILBOX (1UL)
/*******************************************************************************
 * Variables
 ******************************************************************************/
/* Set information about the data to be sent
 *  - Standard message ID
 *  - Bit rate switch enabled to use a different bitrate for the data segment
 *  - Flexible data rate enabled
 *  - Use zeros for FD padding
 */
CAN_HandleTypeDef hcan1;
CAN_TxHeaderTypeDef CanTxBuffCfg;
CAN_RxHeaderTypeDef CanRxBuffCfg;
CANpalRecvMsg_t CANpalRecvMsg[CANBUS_MSG_SIZE];
CANpalMsgFlag_t CANpalMsgFlag;

/*******************************************************************************
 * Private Functions
 ******************************************************************************/


/*FUNCTION**********************************************************************
 *
 * Function Name : CANpal_ProcessInit
 * Description   : Initial CANpal process.
 *
 * Implements    : CANpal Process Initial
 *END**************************************************************************/
/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
*/
bool CANpal_ProcessInit(void) {
    
  bool nErr = false;
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 6;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_13TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
    if (HAL_CAN_Init(&hcan1) != HAL_OK) {
      nErr = false;
    }
    return nErr;
}


bool CANpal_ProcessFilter(uint32_t *busStID, uint32_t *busExID, uint8_t stLen,uint8_t exLen) {
    bool nErr = false;
    CAN_FilterTypeDef  CanFilterConfig;
    uint32_t *u32eXptr, *u32sTptr;
    uint32_t u32StMsk, u32ExMsk, u32tmp, u32buf, u32sum;
    uint8_t u8i;
    
    u32eXptr = busExID;
    u32sTptr = busStID;
    /*##-1- Configure the CAN peripheral ##*/
    CanFilterConfig.FilterBank = 0;
    CanFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
    CanFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
    CanFilterConfig.FilterActivation = ENABLE;
    u32tmp = *u32eXptr;
    
    if((u32tmp & 0x10000000) == 0x10000000) {  // Extern ID
        CanFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
        CanFilterConfig.FilterIdHigh = ((u32tmp << 0x03) >> 0x10) & 0xffff;
        CanFilterConfig.FilterIdLow = ((u32tmp << 0x03) &0xffff);
        u32StMsk = 0x7ff;
        u32tmp = *u32sTptr;
        for(u8i = 0; u8i < stLen; u8i++) {
            u32buf = *(u32sTptr + u8i) ^ (~u32tmp);
            u32StMsk &= u32buf;
        }
        u32ExMsk = 0x1fffffff;
        u32tmp = *u32eXptr;
        for(u8i = 0; u8i < exLen; u8i++) {
            u32buf = *(u32eXptr + u8i) ^ (~u32tmp);
            u32ExMsk &= u32buf;
        }
        u32tmp = ((*u32sTptr) <<0x16) ^ ~(*u32eXptr);
    
        u32sum = (u32StMsk << 0x12) & u32ExMsk & u32tmp;
        u32sum  <<= 0x03;
        CanFilterConfig.FilterMaskIdHigh = (u32sum >> 0x10) & 0xffff;
        CanFilterConfig.FilterMaskIdLow = (u32sum & 0xffff);
    } else {
        CanFilterConfig.FilterScale = CAN_FILTERSCALE_16BIT;
        u32tmp = *u32sTptr;
        CanFilterConfig.FilterIdLow  = u32tmp << 5;
        u32StMsk = 0x7ff;
        u32tmp = *u32sTptr;
        for(u8i = 0; u8i < stLen; u8i++) {
            u32buf = *(u32sTptr + u8i) ^ (~u32tmp);
            u32StMsk &= u32buf;      
        }
        CanFilterConfig.FilterMaskIdLow = (u32StMsk << 0x05) | 0x10;
        u32tmp = *u32eXptr;
        CanFilterConfig.FilterIdHigh = u32tmp << 0x05;
        u32StMsk = 0x7ff;
        for(u8i = 0; u8i < exLen; u8i++) {
            u32buf = *(u32eXptr + u8i) ^ (~u32tmp);
            u32StMsk &= u32buf;      
        }
        CanFilterConfig.FilterMaskIdHigh  = (u32StMsk << 0x05) | 0x10;
    }
    if (HAL_CAN_ConfigFilter(&hcan1, &CanFilterConfig) != HAL_OK) {
      nErr = false;
    }
    return nErr;
}

bool CANpal_ProcessEnable(bool active) {
    bool n_err = false;
    if(active == true) {
        if (HAL_CAN_Start(&hcan1) != HAL_OK) {
            n_err = true;
        }
        if(n_err != true) {
            if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK) {
                n_err = true;
            }
            if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_ERROR) != HAL_OK) {
                n_err = true;
            }
            if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_BUSOFF) != HAL_OK) {
                n_err = true;
            }
        }        
    } else {
        if(HAL_CAN_Stop(&hcan1) != HAL_OK) {
            n_err = false;
        }
        if(HAL_CAN_DeInit(&hcan1) != HAL_OK) {
            n_err = false;
        }

    }
    return n_err;
}


/*FUNCTION**********************************************************************
 *
 * Function Name: SendCANpalData
 * Description: Send data via CAN to the specified mailbox with the specified message id
 * Implements:
 * param mailbox   : Destination mailbox number
 * param messageId : Message ID
 * param data      : Pointer to the TX data
 * param len       : Length of the TX data
 * return          : None
 *END**************************************************************************/

bool SendCANpalTxData(uint32_t messageId, uint8_t * data, uint32_t len, bool ide) {
    bool n_err = false;
    uint8_t u8i, u8TimeOut;
    uint8_t u8buf[8];
    uint32_t CanMailBox;
    
    if(ide == CAN_TYPE_STD) {
        CanTxBuffCfg.StdId = messageId;
        CanTxBuffCfg.IDE = CAN_ID_STD;
    } else {
        CanTxBuffCfg.ExtId = messageId;
        CanTxBuffCfg.IDE = CAN_ID_EXT;
    }
    for(u8i = 0; u8i < len; u8i++) {
        u8buf[u8i] = *data++;
    }
    CanTxBuffCfg.RTR = CAN_RTR_DATA;
    CanTxBuffCfg.DLC = len;
    CanTxBuffCfg.TransmitGlobalTime = ENABLE;
    u8TimeOut = CANBUS_TIMEOUT_MS;
    
    while((HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == false) && (u8TimeOut != false)) {
        HAL_Delay(true); 
        if(u8TimeOut-- == false) {
            HAL_CAN_AbortTxRequest(&hcan1, CAN_TX_MAILBOX0);
            n_err = true;
            
        }
    }
    if(n_err != true) {
        u8TimeOut = CANBUS_TIMEOUT_MS;
        while((HAL_CAN_AddTxMessage(&hcan1, &CanTxBuffCfg, u8buf, &CanMailBox) != HAL_OK) && (u8TimeOut != false))  {
            HAL_Delay(true); 
            if(u8TimeOut-- == false) {
                n_err = true;
            }
        }
    }
    return n_err;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : CANpal_ProcessTask
 * Description   : CANpal Task process
 *
 * Implements    : CANpal Process Task
 *END**************************************************************************/

uint8_t CANpal_GetStatus(void) {
    uint8_t u8i, u8idx;
    u8idx = 0xff;
    if(CANpalMsgFlag.trg == true) {
        for(u8i = false; u8i < CANBUS_MSG_SIZE; u8i++) {
            if(CANpalMsgFlag.dry[u8i] == true) {
                u8idx = u8i;
            }
        }
        CANpalMsgFlag.trg = false;
    }
    return u8idx;
}

void CANpal_ClearStatus(uint8_t idx) {
    bool nRes = false;
    uint8_t u8i;
    if(idx < CANBUS_MSG_SIZE) {
        CANpalMsgFlag.dry[idx] = false;    
    }
    for(u8i = false; u8i < CANBUS_MSG_SIZE; u8i++) {
        if(CANpalMsgFlag.dry[u8i] == true) {
            nRes = true;
        }
    }
    if(nRes == false) {
        CANpalMsgFlag.trg = false;
    }
}


bool CANpal_GetMessage(uint8_t idx, CANpalRecvMsg_t *msg) {
    uint8_t u8i;
    bool nErr = false;
    CANpalRecvMsg_t *CANptr;
    CANptr = msg;
    if(idx < CANBUS_MSG_SIZE) {
        CANptr->id = CANpalRecvMsg[idx].id;
        CANptr->dlc = CANpalRecvMsg[idx].dlc;
        for(u8i = false; u8i < CANptr->dlc; u8i++) {
          CANptr->msg[u8i] = CANpalRecvMsg[idx].msg[u8i];
        }
    } else {
      nErr= true;
    }
  return nErr;
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    uint8_t u8drx, u8i;
    uint8_t u8buf[8];
    CanRxBuffCfg.ExtId = false;
    CanRxBuffCfg.StdId = false;
    if(HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &CanRxBuffCfg, u8buf) == HAL_OK) {
        u8drx = false;
        while(CANpalMsgFlag.dry[u8drx] != false) {
            if(u8drx < CANBUS_MSG_SIZE)  {
                u8drx++;
            }
            else {
                u8drx = false;
            	CANpalMsgFlag.dry[u8drx] = false;
            }
        }
        if(CanRxBuffCfg.ExtId != false) {
            CANpalRecvMsg[u8drx].id = CanRxBuffCfg.ExtId;
        }
        else {
            CANpalRecvMsg[u8drx].id = CanRxBuffCfg.StdId;
        }
        CANpalRecvMsg[u8drx].dlc = CanRxBuffCfg.DLC;
        for(u8i = false; u8i < 8; u8i++) {
            CANpalRecvMsg[u8drx].msg[u8i] = u8buf[u8i];
        }
        CANpalMsgFlag.dry[u8drx] = true;
        CANpalMsgFlag.trg = true;
    }
}

void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan) {
  __HAL_CAN_CLEAR_FLAG(hcan, CAN_FLAG_FOV0);
}

/******************************************************************************
 * EOF
 *****************************************************************************/
