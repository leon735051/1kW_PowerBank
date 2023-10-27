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
 #include "rtc_bsp.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/


/*******************************************************************************
 * Variables
 ******************************************************************************/
RTC_HandleTypeDef hrtc;
RTC_TimeTypeDef g_RTC_Timer = {0};
RTC_DateTypeDef g_RTC_Date = {0};
/*******************************************************************************
 * Private Functions
 ******************************************************************************/



/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
bool RTC_ProcessInit(void) {
    bool n_err = false;
    RTC_TimeTypeDef sTime;
    RTC_DateTypeDef sDate;
    hrtc.Instance = RTC;
    hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
    hrtc.Init.AsynchPrediv = 127;
    hrtc.Init.SynchPrediv = 255;
    hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
    hrtc.Init.OutPutRemap = RTC_OUTPUT_REMAP_NONE;
    hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
    hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
    if (HAL_RTC_Init(&hrtc) != HAL_OK) {
        n_err = true;
    }
    sDate = RTC_RetrunDate();
    if(sDate.Year == false) {
    /** Initialize RTC and set the Time and Date */
      sTime.Hours = 0x0;
      sTime.Minutes = 0x0;
      sTime.Seconds = 0x0;
      sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
      sTime.StoreOperation = RTC_STOREOPERATION_RESET;
      if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK) {
        n_err = true;
      }
      sDate.WeekDay = RTC_WEEKDAY_FRIDAY;
      sDate.Month = RTC_MONTH_JANUARY;
      sDate.Date = 0x01;
      sDate.Year = 0x15;
      if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN) != HAL_OK) {
        n_err = true;
      }
    }
    return n_err;
}

bool RTCTimer_SET_Clcok(uint8_t *msg) {
    bool n_err = false;
    RTC_TimeTypeDef sTime;
    RTC_DateTypeDef sDate;
    /** Initialize RTC and set the Time and Date */
    sDate.WeekDay = RTC_WEEKDAY_FRIDAY;
    sDate.Year = *(msg);
    sDate.Month = *(msg +1);
    sDate.Date = *(msg  +2);
    if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN) != HAL_OK) {
        n_err = true;
    }
    sTime.Hours   = *(msg +3);
    sTime.Minutes = *(msg +4);
    sTime.Seconds = *(msg +5);
    sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
    sTime.StoreOperation = RTC_STOREOPERATION_RESET;
    if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK) {
        n_err = true;
    }
    return n_err;
}

bool RTCTimer_ProcessTask(void) {
    bool n_err = false;
    if(HAL_RTC_GetTime(&hrtc, &g_RTC_Timer, RTC_FORMAT_BIN) != HAL_OK) {
      n_err = false;
    } else if(HAL_RTC_GetDate(&hrtc, &g_RTC_Date, RTC_FORMAT_BIN)) {
        n_err = false;
    }
    return n_err;
}

RTC_TimeTypeDef RTC_RetrunTime(void) {
	return g_RTC_Timer;
}

RTC_DateTypeDef RTC_RetrunDate(void) {
	return g_RTC_Date;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : PORT_VECTOR_TASK
 * Description   : Trigger an interrupt 10ms.
 *
 * Implements    : PORT Interrupt Handler
 *END**************************************************************************/


/******************************************************************************
 * EOF
 *****************************************************************************/
