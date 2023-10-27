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
#include "flash_bsp.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/

/* USER CODE BEGIN PFP */

HAL_StatusTypeDef FlashBSP_Init(void) {
  HAL_StatusTypeDef n_res;
  n_res = HAL_FLASH_Unlock();
  return n_res;
}

HAL_StatusTypeDef FlashBSP_Erase(uint32_t puiAddr, uint16_t sector) {
  HAL_StatusTypeDef n_res;
  FLASH_EraseInitTypeDef FlashIntStruct = {0};
  uint32_t u32err;
  FlashIntStruct.TypeErase = FLASH_TYPEERASE_PAGES;
  FlashIntStruct.Page = (puiAddr - FLASH_BASE) / FLASH_PAGE_SIZE;
  FlashIntStruct.NbPages = 1;
  FlashIntStruct.Banks = FLASH_BANK_BOTH;
  n_res = HAL_FLASHEx_Erase(&FlashIntStruct, &u32err);
  
  return n_res;
}

HAL_StatusTypeDef FlashBSP_Program(uint32_t puiAddr, uint64_t *ptr, uint16_t sector) {
  HAL_StatusTypeDef n_res;
  uint16_t u16i;
  for(u16i = 0; u16i <= (sector - 8); u16i += 8) {
      n_res = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, puiAddr + u16i, *ptr++);
  }
  return n_res;
}



/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
