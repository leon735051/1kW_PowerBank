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

#include <stdbool.h>
#include <math.h>
#include <float.h>
#include <stdlib.h>
#include <math.h>

#include "driver_bsp/i2c_bsp.h"
#include "module_app/bqMaximo/bqMaximo.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define BQ_DEV_ADDR   0x10
#define BQ_SUBCMD_LOW 0x3E
#define BQ_SUBCMD_HIG 0x3F
#define BQ_START_RSP  0x40
#define BQ_CHKSUM_RSP 0x60

/*******************************************************************************
 * Variables
 ******************************************************************************/

/*******************************************************************************
 * Private Functions
 ******************************************************************************/
static uint8_t bqCalCheckSum(uint8_t *ptr, uint8_t sLen);

static uint8_t bqCalCheckSum(uint8_t *ptr, uint8_t sLen)
{
    uint8_t u8i, u8sum;
    u8sum = false;
    // Calculates the checksum when writing to a RAM register.
    // The checksum is the inverse of the sum of the bytes.     
    for(u8i = 0; u8i < sLen; u8i++) {
        u8sum += *(ptr + u8i);
    }
    u8sum = 0xff & ~u8sum;
    return(u8sum);
}

bool bqDirectCommand(uint16_t sCmd, uint16_t sBuf, uint8_t sLen, bool nTsk) {
    bool nErr = false;
    uint8_t u8i = false, u8tmp = false;
    uint8_t u8buf[6];

    if(nTsk == AFE_DIRCMD_MTD) {  // Direct Command
        u8buf[0] = sCmd & 0xff;
        u8buf[1] = sBuf & 0xff;
        u8buf[2] = sBuf >> 0x08;
        nErr = bspI2C_Write(BQ_DEV_ADDR, u8buf, sLen +1);
    } else {
        u8buf[0] = BQ_SUBCMD_LOW;
        u8buf[1] = sCmd & 0xff;
        u8buf[2] = sCmd >> 0x08;
        u8buf[3] = sBuf & 0xff;
        u8buf[4] = sBuf >> 0x08;
        nErr = bspI2C_Write(BQ_DEV_ADDR, u8buf, sLen +3);
        if(sLen != false) {
            for(u8i = false; u8i< (sLen +3); u8i++) {
                u8buf[u8i] = u8buf[u8i +1];
            }
            u8tmp = bqCalCheckSum(u8buf, sLen +2); 
            u8buf[0] = BQ_CHKSUM_RSP;
            u8buf[1] = u8tmp;
            u8buf[2] = sLen + 4;
            nErr = bspI2C_Write(BQ_DEV_ADDR, u8buf, 3);
        }
    }
    return nErr;
}

bool bqDirectRead(uint16_t sCmd, uint8_t *ptr, uint8_t sLen, bool nTsk) {
    bool nErr = false;
    uint8_t u8buf[3];
    if(nTsk == AFE_DIRCMD_MTD) {
        u8buf[0] = sCmd & 0xff;
        nErr = bspI2C_Read(BQ_DEV_ADDR, sCmd & 0xff, ptr, sLen);
    } else {
        u8buf[0] = BQ_SUBCMD_LOW;
        u8buf[1] = sCmd & 0xff;
        u8buf[2] = sCmd >> 0x08;
        nErr = bspI2C_Write(BQ_DEV_ADDR, u8buf, 3);
        nErr = bspI2C_Read(BQ_DEV_ADDR, BQ_START_RSP, ptr, sLen);
    }
    return nErr;
}