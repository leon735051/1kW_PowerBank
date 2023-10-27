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
#ifndef __FLASH_MOD_H
#define __FLASH_MOD_H

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
#define FIRMWARE_VERSION "001-000"
typedef struct
{
    // Reserved space used by the flash program block code.
    uint32_t ui32PBReserved;
    uint16_t def_max_voltage;
    uint16_t def_rate_voltage;
    uint16_t def_cutof_current;
    uint16_t def_capacity;
    uint16_t def_energy;
    uint16_t aval_capacity;
    uint16_t rem_energy;
    uint16_t rem_capacity;
    uint16_t cycle_chg_capacity;
    uint16_t cycle_dis_capacity;
    uint16_t cycle_count;
    uint16_t soc;
    uint16_t soh;

    uint16_t  DSG_Temp_H_L1;
    uint16_t  DSG_Temp_H_L2;
    uint16_t  DSG_Temp_H_L3;

    uint16_t  CHG_Temp_H_L1;
    uint16_t  CHG_Temp_H_L2;
    uint16_t  CHG_Temp_H_L3;

    uint16_t  DSG_Temp_L_L1;
    uint16_t  DSG_Temp_L_L2;
    uint16_t  DSG_Temp_L_L3;
    
    uint16_t  CHG_Temp_L_RL;

    uint16_t  Temp_Diff_L1;
    uint16_t  Temp_Diff_L2;
    uint16_t  Temp_Diff_L3;
    
    uint16_t  CellV_Diff_L1;
    uint16_t  CellV_Diff_L2;
    uint16_t  CellV_Diff_L3;    
    
    uint16_t  Cell_high_L1;
    uint16_t  Cell_high_L2;
    uint16_t  Cell_high_L3;
    uint16_t  Cell_low_L1;
    uint16_t  Cell_low_L2;
    uint16_t  Cell_low_L3;  
        
    uint16_t  BatV_max_L1;
    uint16_t  BatV_max_L2;
    uint16_t  BatV_max_L3;
    uint16_t  BatV_min_L1;
    uint16_t  BatV_min_L2;
    uint16_t  BatV_min_L3;
    
    uint16_t  ChgCur_max_L1;
    uint16_t  ChgCur_max_L2;
    uint16_t  ChgCur_max_L3;
    
    uint16_t  DisChgCur_max_L1;
    uint16_t  DisChgCur_max_L2;
    uint16_t  DisChgCur_max_L3;
    
    uint8_t  soc_high_L1;
    uint8_t  soc_high_L2;
    uint8_t  soc_high_L3;
    
    uint8_t  soc_low_L1;
    uint8_t  soc_low_L2;
    uint8_t  soc_low_L3;
    
    uint16_t MaxPwr_Limt_L1;
    uint16_t MaxPwr_Limt_L2;
    uint16_t MaxPwr_Limt_L3;
    
    uint8_t open_subcircuit_L1;
    uint8_t open_subcircuit_L2;

    uint8_t calibation_DsgCur;
    uint8_t calibation_ChgCur;
    uint8_t calibation_tmp;
    uint8_t calibation_cellV;
    uint8_t calibation_batV;
    uint8_t calibation_packV;
        
    bool CH_Update_RDY;
    bool DS_Update_RDY;
    bool LED_DISPLAY;
    
    bool RPO_CHD;
    uint8_t MODEL_NAME[7];
    uint8_t SN[18];
    uint8_t FW_VERN[7];
    uint8_t DATE_MAF[3];
    uint8_t PACK_PARM[6];
    uint8_t PACK_SPEC[4];
    RTC_DateTypeDef LCHG_DATE;
    RTC_TimeTypeDef LCHG_TIME;
} gParameters;
   // cell balance 30mA   @ 4.0V

static const gParameters g_sDefaultParams =
{
    0,
    4200,     // def_max_voltage
    3690,     //  def_rate_voltage
    60,       // def_cutof_current   // 500mA
    0,        // def_capacity;
    3653,     // def_energy
    0,        // aval_capacity
    0,        // rem_energy
    0,        // rem_capacity
    0,        // cycle_chg_capacity
    0,        // cycle_dis_capacity
    0,        // cycle_count
    50,        // soc
    100,       // soh
    
    5500,       //  DSG_Temp_high_Release;    // 55C
    6000,       //  DSG_Temp_high_L1;    // 60oC
    6500,       //  DSG_Temp_high_L2;     // 65oC

    4500,       //  CHG_Temp_high_Release;    // 45oC
    5000,       //  CHG_Temp_high_L1;    // 50oC
    5500,       //  CHG_Temp_high_L2;     // 55oC

    500,        //  DSGTemp_low_Release;    // -5oC
    1000,        //  DSGTemp_lowLow_L1; // -10
    2000,        //  DSGTemp_low_L2;    // -20
    
    0,          //  CHG_Temp_Low_Relase  // 0oC
    
    300,          // Temp_Diff_Release;
    500,         // Temp_Diff_L1;
    700,         // Temp_Diff_L2;
    
    600,        //  CellV_Diff_Release;
    1000,        // CellV_Diff_L1;
    1500,        //  CellV_Diff_L2;
    
    4223,      // Cell_high_Release;
    4250,      // Cell_high_L1;
    4300,      // Cell_high_L2;
    
    2500,      // Cell_low_Release;
    2450,      // Cell_low_L1;
    2400,      // Cell_low_L2;
        
    4200,      // BatV_max_Release;   suge as cell high value
    4250,      // BatV_max_L1;
    4275,      // BatV_max_L2;
    
    3000,       // BatV_min_Release;  suge as cell low value
    2750,       // BatV_min_L1;
    2500,       //  BatV_min_L2;
    
    350,       // ChgCur_max_Release;
    400,       // ChgCur_max_L1;
    500,       // ChgCur_max_L2;
    
    2000,       // DisChgCur_max_Release;
    2100,       // DisChgCur_max_L1;
    2500,       // DisChgCur_max_L2;
    
    101,        //  soc_high_Release;
    105,        // uint16_t  soc_high_L1;
    110,        // uint16_t  soc_high_L2;
    
    10,         // soc_low_Release;
    5,          // soc_low_L1;
    0,          // soc_low_L2;
    
    1000,        //  MaxPwr_Limt_L1;  pic 1300 w
    500,         // MaxPwr_Limt_L2;   pic 1400  w
    20,          //  MaxPwr_Limt_L3;  pic 1500  w
    
    1,          // open_subcircuit_L1;
    2,          // open_subcircuit_L2;

    0,          //calibation_DsgCur;
    0,          //calibation_ChgCur;
    0,          // calibation_tmp;
	0,          // calibation_cellV
    0,          // calibation_BatV;
	0,          // calibation_PackV
    
    false,    // CH_Update_RDY
    true,     //  DS_Update_RDY
	true,      // LED Display
    false,   // Battery Charged


    {0x32,0x31,0x30,0x31,0x20,0x20,0x20},  // MODEL_NAME
    {0U}, // SN
    {FIRMWARE_VERSION}, //FW_VERN
    {0U}, // DATE_MAF
    {0x05,0x03,0x03,0x0A,0x02,0x04}, // PACK_PARM
    {0x24,0x90,0xac,0x26}, // PACK_SPEC
    {0U}, // Last Charge Date
};

typedef enum
{
	STATE_INIT = false,
	STATE_FLASH_FILL_DATABUF_AND_ERASE,
        STATE_FLASH_ERASE_ERROR,
	STATE_FLASH_ERASE_COMPLETION_CHECK,
	STATE_FLASH_WRITE_START,
	STATE_FLASH_WRITE_COMPLETION_CHECK_AND_VERIFY_CHECK,
	STATE_FLASH_ERROR,
	STATE_FLASH_SUCCESS,
} FLASH_FLAGS;

typedef enum
{
  FL_STAT = false,
  FL_RDY,
  FL_ERR,
} FLASH_FLAG_ENUM;

typedef struct {
    FLASH_FLAGS flags;
    bool RDY;
    bool ERR;
}
FLASH_STATUS_t;
/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */
extern gParameters g_sConfig;
extern gParameters *gParams;
/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
bool Flash_ProcessInit(void);
extern bool DataFlashSaveProcess(void);
bool DataFlashSaveProcess(void);
extern uint8_t Flash_GetStatus(uint8_t idx);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __FLASH_MOD_H */

/************************ (C) COPYRIGHT NucalTech *****END OF FILE****/
