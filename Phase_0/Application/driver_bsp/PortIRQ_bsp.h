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
#ifndef PORTIRQ_MOD_H
#define PORTIRQ_MOD_H

#include <stdbool.h>
#include <stdint.h>


/*! @file lpiTimer_mod.h*/

/*******************************************************************************
 * Definitions
 ******************************************************************************/

// AFE Alarm Signal Port
#define AFE_ALERT_BASE PORTB
#define AFE_ALERT_PORT GPIOB
#define AFE_ALERT_PIN  GPIO_PIN_2

#define WAKE_SIG_BASE PORTB
#define WAKE_SIG_PORT GPIOB
#define WAKE_SIG_PIN  GPIO_PIN_1


#define PORT_TIMEOUT_PCS_MS 5
#define PORT_TIMEOUT_LDT_MS 3
#define PORT_TIMEOUT_USD_MS 500
/* Definition of power modes indexes, as configured in Power Manager Component
 *  Refer to the Reference Manual for details about the power modes.
 */
#define PWR_RUN   (0u) /* Run                 */
#define PWR_VLPR  (1u) /* Very low power run  */
#define PWR_STOP1 (2u) /* Stop option 1       */
#define PWR_STOP2 (3u) /* Stop option 2       */
#define PWR_VLPS  (4u) /* Very low power stop */

#define PORT_IRQ_TRIG_MS   4


/************************************************************
* Initial Other Variable
************************************************************/
typedef enum {
  PIRQ_GLB = 0x00,
  PIRQ_AFE_ALERT,
  PIRQ_WAKE_SIG,
} portIrq_emum;

typedef struct {
  bool GLB;
  bool AFE_ALT;
  bool WAKE_SIG;
} portIrq_Flags_t;

typedef struct {
  bool AFE_ALT;
  bool WAKE_SIG;
} portIrq_Trig_t;

typedef struct {
  uint8_t AFE_ALT;
  uint8_t WAKE_SIG;
} portIrq_Timer_t;


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
extern void PortIrq_ProcessInit(void);
extern void PortIrq_MuxProcessTask(void);
extern bool PortIrq_PMuxFlags(uint8_t idx);
extern bool PortIrq_PMuxClear(uint8_t idx);
extern void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
/*!
 * @brief Interrupt handler for a LPIT instance.
 *
 * @param[in] The LPIT instance process.
 * @return NONE
 */


/*! @}*/

#if defined (__cplusplus)
}
#endif

#endif /* PORTIRQ_MOD_H */
/*******************************************************************************
 * EOF
 ******************************************************************************/
