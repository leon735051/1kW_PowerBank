/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    FLASH/FLASH_EraseProgram/Src/main.c
  * @author  MCD Application Team
  * @brief   This example provides a description of how to erase and program the
  *          STM32G0xx FLASH.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics. 
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the 
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "flash_mod.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "driver_bsp/flash_bsp.h"
#include "module_app/CoulCount/CoulCount.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
gParameters g_sConfig;
gParameters *gParams;
FLASH_STATUS_t FlashStatus;
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static uint8_t *g_pui8FlashPBStart;
static uint8_t *g_pui8FlashPBEnd;
static uint32_t g_ui32FlashPBSize;
static uint8_t *g_pui8FlashPBCurrent;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/

/* USER CODE BEGIN PFP */


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
bool FlashNVM_DF_Save(uint8_t *pui8Buffer);
void Flash_DF_Init(uint32_t ui32Start, uint32_t ui32End, uint32_t ui32Size);
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */


bool FlashNVM_DF_Save(uint8_t *pui8Buffer) {
    bool n_done = false;
    uint8_t *pui8New;
    uint32_t ui32Idx, ui32Sum;
    /* See if there is a valid parameter block in flash. */
    if(g_pui8FlashPBCurrent != false) {
        /* Set the sequence number to one greater than the most recent parameter block. */
        pui8Buffer[0] = g_pui8FlashPBCurrent[0] + 1;
        /* Try to write the new parameter block immediately after the most
         * recent parameter block.*/
        pui8New = g_pui8FlashPBCurrent + g_ui32FlashPBSize;
        if(pui8New == g_pui8FlashPBEnd) {
            pui8New = g_pui8FlashPBStart;
        }
    } else {
        /* There is not a valid parameter block in flash, so set the sequence
    	 * number of this parameter block to zero. */
        pui8Buffer[0] = 0;
        /* Try to write the new parameter block at the beginning of the flash
         * space for parameter blocks. */
        pui8New = g_pui8FlashPBStart;
    }
    /* Compute the checksum of the parameter block to be written. */
    for(ui32Idx = 0, ui32Sum = 0; ui32Idx < g_ui32FlashPBSize; ui32Idx++) {
        ui32Sum -= pui8Buffer[ui32Idx];
    }
    /* Store the checksum into the parameter block. */
    pui8Buffer[1] += ui32Sum;
    /* Look for a location to store this parameter block.  This infinite loop
     * will be explicitly broken out of when a valid location is found. */
    while(n_done == false) {
    	/* See if this location is at the start of an erase block. */
        if(*pui8New != 0xff) {
            /* Erase this block of the flash.  This does not assume that the
             * erase succeeded in case this block of the flash has become bad
             * through too much use.  Given the extremely low frequency that
             * the parameter blocks are written, this will likely never fail.
             * But, that assumption is not made in order to be safe. */
            /* Fill EraseInit structure*/
          if(FlashBSP_Erase((uint32_t)pui8New,FLASH_PAGE_SIZE) != false) {
            FlashStatus.flags = STATE_FLASH_ERASE_ERROR;
            FlashStatus.ERR = true;
          } else {
            FlashStatus.flags = STATE_FLASH_FILL_DATABUF_AND_ERASE;
          }
        }
        /* Loop through this portion of flash to see if is all ones (in other
         * words, it is an erased portion of flash). */
        for(ui32Idx = 0; ui32Idx < g_ui32FlashPBSize -1; ui32Idx++) {
          FlashStatus.flags = STATE_FLASH_ERASE_COMPLETION_CHECK;
          if(pui8New[ui32Idx] != 0xff) {
              FlashStatus.ERR = true;      
              break;
            }
        }
        /* If all bytes in this portion of flash are ones, then break out of
         * the loop since this is a good location for storing the parameter block */
        if(ui32Idx == g_ui32FlashPBSize -1) {
          n_done = true;
        } else {
          /* Increment to the next parameter block location. */
          pui8New += g_ui32FlashPBSize;
          if(pui8New == g_pui8FlashPBEnd) {
            pui8New = g_pui8FlashPBStart;
          }
          /* If every possible location has been checked and none are valid, then
          * it will not be possible to write this parameter block.  Simply
          * return without writing it. */
          if((g_pui8FlashPBCurrent && (pui8New == g_pui8FlashPBCurrent)) ||
             (!g_pui8FlashPBCurrent && (pui8New == g_pui8FlashPBStart))) {
               FlashStatus.ERR = true;
               n_done = true;
               FlashStatus.flags = STATE_FLASH_ERROR;
             }
        }
    }
    if(FlashStatus.ERR != true) {
      FlashStatus.flags = STATE_FLASH_WRITE_START;
      /* Write this parameter block to flash.*/
      if(FlashBSP_Program((uint32_t)pui8New, (uint64_t *)pui8Buffer, FLASH_SECTOR_SIZE) != false) {
        FlashStatus.flags = STATE_FLASH_ERROR;
        FlashStatus.ERR = true;      
      } else {
        /* Compare the parameter block data to the data that should now be in
        * flash.  Return if any of the data does not compare, leaving the previous
        * parameter block in flash as the most recent (since the current parameter
        * block failed to properly program). */
        for(ui32Idx = 0; ui32Idx < g_ui32FlashPBSize; ui32Idx++) {
          FlashStatus.flags = STATE_FLASH_WRITE_COMPLETION_CHECK_AND_VERIFY_CHECK;
          if(pui8New[ui32Idx] != pui8Buffer[ui32Idx]) {
            FlashStatus.flags = STATE_FLASH_ERROR;
            FlashStatus.ERR = true;
            break;
          }
        }
        if(FlashStatus.ERR != true) {
          /* The new parameter block becomes the most recent parameter block. */
          g_pui8FlashPBCurrent = pui8New;     
          FlashStatus.flags = STATE_FLASH_SUCCESS;
        }
      }
    }
    return FlashStatus.ERR;
}

void Flash_DF_Init(uint32_t ui32Start, uint32_t ui32End, uint32_t ui32Size) {
    bool n_idx;
    uint32_t ui32Idx, ui32Sum;
    uint8_t *pui8Offset, *pui8Current;
    uint8_t ui8One, ui8Two;
    
    g_pui8FlashPBStart = (uint8_t *)ui32Start;
    g_pui8FlashPBEnd = (uint8_t *)ui32End;
    g_ui32FlashPBSize = ui32Size;
    pui8Current = false;
    n_idx = false;
    for(pui8Offset = g_pui8FlashPBStart; pui8Offset < g_pui8FlashPBEnd; pui8Offset += g_ui32FlashPBSize) {
    	ui32Sum = false;
    	/*Loop through the bytes in the block, computing the checksum. */
    	for(ui32Idx = false; ui32Idx < g_ui32FlashPBSize; ui32Idx++) {
    		ui32Sum += pui8Offset[ui32Idx];
        }

    	/* The checksum should be zero.
    	 * If the sum is equal to the size * 255, then the block is all ones and
    	 * should not be considered valid. */

        if(((ui32Sum & 0xff) != false) || ((g_ui32FlashPBSize * 0xff) != ui32Sum)) {
            /* See if a valid parameter block has been previously found. */
            if(pui8Current != false) {
            	  /* Get the sequence numbers for the current and new parameter blocks. */
                ui8One = pui8Current[0];
                ui8Two = pui8Offset[0];
                /* See if the sequence number for the new parameter block is
                 * greater than the current block.  The comparison isn't
                 * straightforward since the one byte sequence number will wrap
                 * after 128 parameter blocks.*/
                if(((ui8One > ui8Two) && ((ui8One - ui8Two) < 128)) ||
                   ((ui8Two > ui8One) && ((ui8Two - ui8One) > 128))) {
                    /* The new parameter block is older than the current
                     * parameter block, so skip the new parameter block and keep searching. */
                    n_idx = false;
                   } else {
                       n_idx = true;
                   }
            } else {
                n_idx = true;
            }
            /* The new parameter block is more recent than the current one, so
             * make it the new current parameter block. */
            if(n_idx == true) {
                pui8Current = pui8Offset;
                n_idx = false;
            }
        }
    }
    /* Save the address of the most recent parameter block found.  If no valid
     * parameter blocks were found, this will be a NULL pointer. */
    g_pui8FlashPBCurrent = pui8Current;
}

bool Flash_ProcessInit(void) {
    bool nErr = false;
    FlashStatus.RDY = false;
    FlashStatus.ERR = false;
    FlashStatus.flags = STATE_INIT;
    if(FlashBSP_Init() == false) {
        Flash_DF_Init(FLASH_PROGRAM_BASE_ADDRESS, FLASH_PROGRAM_END_ADDRESS_VALUE, FLASH_SECTOR_SIZE);
        gParams = (gParameters *)g_pui8FlashPBCurrent;
        if(gParams == false) {
            g_sConfig = g_sDefaultParams;
            if(FlashNVM_DF_Save((uint8_t *)&g_sConfig) != false) {
                nErr = false;
            }
        }
        else {
            g_sConfig = *gParams;
        }
    } else {
        nErr = true;
    }
    if(nErr != true) {
        FlashStatus.RDY = true;
    }
    return nErr;
}
                
uint8_t Flash_GetStatus(uint8_t idx) {
  switch(idx) {
  case FL_STAT:
    return FlashStatus.flags; break;
  case FL_RDY:
    return FlashStatus.RDY; break;
  case FL_ERR:
    return FlashStatus.ERR; break;
  default:
    return false; break;
  }
}

bool DataFlashSaveProcess(void) {
    bool n_err = false;
    if(FlashStatus.RDY == false) {
        g_sConfig.rem_capacity =  CoulCountRetValue(CC_R_CAP);
        g_sConfig.rem_energy = CoulCountRetValue(CC_R_ENEGY);
        g_sConfig.cycle_count =  CoulCountRetValue(CC_CYC_CNT);
        g_sConfig.cycle_dis_capacity = CoulCountRetValue(CC_CYC_DSG);
        g_sConfig.aval_capacity = CoulCountRetValue(CC_AVEL_CAP);
        g_sConfig.cycle_chg_capacity = CoulCountRetValue(CC_CYC_CHG);
        g_sConfig.CH_Update_RDY =  CoulCountRetStatus(CC_CH_UPRDY);
        g_sConfig.DS_Update_RDY =  CoulCountRetStatus(CC_DS_UPRDY);
        g_sConfig.RPO_CHD = CoulCountRetStatus(CC_PROC_CHG);
        g_sConfig.soc = CoulCountRetValue(CC_SOC);
        g_sConfig.soh = CoulCountRetValue(CC_SOH);
        if(FlashNVM_DF_Save((uint8_t *)&g_sConfig) == true) {
            n_err = true;
        }
    }
    return  n_err;
}
//  HAL_PWREx_EnableFlashPowerDown(PWR_FLASHPD_LPSLEEP);

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
