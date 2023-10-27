/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : flash_mod.h
  * @brief          : Header for flash_mod.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 NucalTech.
  * All rights reserved.</center></h2>
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __HAL_FLASH_H
#define __HAL_FLASH_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdint.h>
#include <stdbool.h>
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
extern HAL_StatusTypeDef FlashBSP_Init(void);
extern HAL_StatusTypeDef FlashBSP_Erase(uint32_t puiAddr, uint16_t sector);
extern HAL_StatusTypeDef FlashBSP_Program(uint32_t puiAddr, uint64_t *ptr, uint16_t sector);
/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/

/* USER CODE BEGIN Private defines */
#define FLASH_SECTOR_SIZE                       256
#define NVM_DATABUFF_SIZE                       (sizeof(databuff) / sizeof(uint32_t))
#define ALIGN(x)                                 __attribute__((coherent, aligned(x)));
/* Row size for MZ device is 2Kbytes */
#define FLASH_DEVICE_ROW_SIZE_DIVIDED_BY_4        (DRV_FLASH_ROW_SIZE/4)
/* Page size for MZ device is 16Kbytes */
#define FLASH_DEVICE_PAGE_SIZE_DIVIDED_BY_4       (DRV_FLASH_PAGE_SIZE/4) //0x0803F800~0803FFFF

#define FLASH_PROGRAM_BASE_ADDRESS          (FLASH_BASE + (0x7f * FLASH_PAGE_SIZE))   /* Start @ of user Flash area */
#define FLASH_PROGRAM_END_ADDRESS_VALUE      FLASH_PROGRAM_BASE_ADDRESS + FLASH_PAGE_SIZE /* End @ of user Flash area */
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __FLASH_MOD_H */

/************************ (C) COPYRIGHT NucalTech *****END OF FILE****/
