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
#ifndef CANPAL_BSP_H
#define CANPAL_BSP_H

#include "stm32l4xx_hal.h"
#include <stdbool.h>
/*! @file CANpal_bsp.h*/

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define CANBUS_TIMEOUT_MS 10
#define CANBUS_MSG_SIZE 3

#define CAN_TYPE_EXT  true
#define CAN_TYPE_STD  false

#define TP101_REQ_PROCS_ID     0xc9
#define TP102_REQ_PROCS_ID     0xac
 
#define CT_ENGR_CMD_ID        0x66F

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
typedef struct {
	bool dry[CANBUS_MSG_SIZE];
	bool trg;
} CANpalMsgFlag_t;

typedef struct {
	uint32_t id;
        uint8_t dlc;
        uint8_t msg[8];
} CANpalRecvMsg_t;



/*!
 * @brief Interrupt handler for a LPIT instance.
 *
 * @param[in] The LPIT instance process.
 * @return NONE
 */
bool CANpal_ProcessInit(void);
bool CANpal_ProcessFilter(uint32_t *busStID, uint32_t *busExID, uint8_t stLen,uint8_t exLen);
extern bool CANpal_ProcessEnable(bool active);
extern bool SendCANpalTxData(uint32_t messageId, uint8_t * data, uint32_t len, bool ide);
uint8_t CANpal_GetStatus(void);
void CANpal_ClearStatus(uint8_t idx);
bool CANpal_GetMessage(uint8_t idx, CANpalRecvMsg_t *msg);
extern void CANpal_VECTOR_TASK(CAN_HandleTypeDef *hcan);
extern void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan);



/*! @}*/

#if defined (__cplusplus)
}
#endif

#endif /* FLEXCAN_MOD_H */
/*******************************************************************************
 * EOF
 ******************************************************************************/
