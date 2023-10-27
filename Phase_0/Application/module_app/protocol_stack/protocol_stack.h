/*
 * Copyright (c) 2013 - 2020, Industrial Technology Research Institute.
 * All rights reserved.
 *
 */
#ifndef PROTOCOL_STACK_H
#define PROTOCOL_STACK_H


/*! @file adConv_mod.h*/

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define PS_1ST_TMR_CLK 1  // 100ms
#define PS_2ND_TMR_CLK 2  // 200ms
#define PS_3RD_TMR_CLK 5  // 500ms
#define PS_4TH_TMR_CLK 2  // 1Sec
#define PS_5TH_TMR_CLK 3  // 3Sec

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
	uint8_t tmr_1st;
	uint8_t tmr_2nd;
	uint8_t tmr_3rd;
	uint8_t tmr_4th;
	uint8_t tmr_5th;
} ps_config_info_t;

typedef struct {
	bool TASK_1st;
	bool TASK_2nd;
	bool TASK_3rd;
	bool TASK_4th;
	bool TASK_5th;
} ps_status_flag_t;

/*!
 * @brief sysLED Process Initial.
 *
 * @param[in] The sysLED initial process.
 * @return NONE
 */
bool PtoclStack_ProcessInit(void);
void PtoclStack_MessageTask(void);
void PtoclStack_ProcessTask(void);
     
/*!
 * @brief sysLED Process Task.
 *
 * @param[in] The sysLED Task process.
 * @return NONE
 */


/*! @}*/

#if defined (__cplusplus)
}
#endif

#endif /* SYSLED_MOD_H */
/*******************************************************************************
 * EOF
 ******************************************************************************/
