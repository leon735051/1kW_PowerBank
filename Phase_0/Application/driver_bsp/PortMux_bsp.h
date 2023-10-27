/**
  ******************************************************************************
  * @file    PortMux_bsp.c
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
#ifndef PORTMUX_MOD_H
#define PORTMUX_MOD_H
#include <stdbool.h>
#include <stdint.h>

// SYS Main Power Control Port
#define PWR_CTL_BASE PORTC
#define PWR_CTL_PORT GPIOC
#define PWR_CTL_PIN  GPIO_PIN_0

// SYS Main Power RON Select Port
#define PWR_RON_BASE PORTC
#define PWR_RON_PORT GPIOC
#define PWR_RON_PIN  GPIO_PIN_1

// SYS Isolation Power Control Port
#define PWR_ISO_BASE PORTC
#define PWR_ISO_PORT GPIOC
#define PWR_ISO_PIN  GPIO_PIN_2

// BAT Voltage Monitor Control Port
#define BAT_CTMON_BASE PORTC
#define BAT_CTMON_PORT GPIOC
#define BAT_CTMON_PIN  GPIO_PIN_5

// AFE DisCharge FET Port
#define AFE_DSG_BASE PORTB
#define AFE_DSG_PORT GPIOB
#define AFE_DSG_PIN  GPIO_PIN_15

// AFE Power Shutdown Port
#define AFE_SHUT_BASE PORTC
#define AFE_SHUT_PORT GPIOC
#define AFE_SHUT_PIN  GPIO_PIN_6

// AFE Power Wakeup Port
#define AFE_WAKE_BASE PORTC
#define AFE_WAKE_PORT GPIOC
#define AFE_WAKE_PIN  GPIO_PIN_7

// UART ES Control Port
#define UART_ES_BASE PORTB
#define UART_ES_PORT GPIOB
#define UART_ES_PIN  GPIO_PIN_4

// FANS PWR Control Port
#define FANS_PWR_BASE PORTB
#define FANS_PWR_PORT GPIOB
#define FANS_PWR_PIN  GPIO_PIN_5

typedef enum {
    POUT_PWR_CTL = false,
    POUT_PWR_RON,
    POUT_PWR_ISO,
    POUT_BAT_CTMON,
    POUT_AFE_DSG,
    POUT_AFE_SHUT,
    POUT_AFE_WAKE,
    POUT_UART_ES,
    POUT_FANS_PWR,
    POUT_DEF_EOL,
} port_Output_enum;

typedef struct {
    bool PWR_CTL;
    bool PWR_RON;
    bool PWR_ISO;
    bool BAT_CTMON;
    bool AFE_DSG;    
    bool AFE_SHUT;    
    bool AFE_WAKE;
    bool UART_ES;
    bool FANS_PWR;
} port_OutErr_t;

typedef struct {
    bool PWR_CTL;
    bool PWR_RON;
    bool PWR_ISO;
    bool BAT_CTMON;
    bool AFE_DSG;    
    bool AFE_SHUT;    
    bool AFE_WAKE;
    bool UART_ES;
    bool FANS_PWR;    
} port_OutFlags_t;

#if defined (__cplusplus)
extern "C" {
#endif

extern void PortMux_ProcessInit(void);
extern bool PortMux_OutProcessTask(void);
extern bool PortMux_OutEnable(uint8_t idx);
extern void PortMux_OutDisable(uint8_t idx);
extern bool PortMux_OutStatus(uint8_t idx);

#if defined (__cplusplus)
}
#endif

#endif /* PORTIRQ_MOD_H */
/*******************************************************************************
 * EOF
 ******************************************************************************/
