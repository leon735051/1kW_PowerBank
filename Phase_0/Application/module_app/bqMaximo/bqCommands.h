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
#ifndef BQCOMMANDS_H
#define BQCOMMANDS_H



#define BQ_DEV_ADDR   0x10
#define BQ_SUBCMD_LOW 0x3E
#define BQ_SUBCMD_HIG 0x3F
#define BQ_START_RSP  0x40
#define BQ_CHKSUM_RSP 0x60
#define BQ_LEN_RSP    0x61

#define BIT0                (0x0001)
#define BIT1                (0x0002)
#define BIT2                (0x0004)
#define BIT3                (0x0008)
#define BIT4                (0x0010)
#define BIT5                (0x0020)
#define BIT6                (0x0040)
#define BIT7                (0x0080)
#define BIT8                (0x0100)
#define BIT9                (0x0200)
#define BITA                (0x0400)
#define BITB                (0x0800)
#define BITC                (0x1000)
#define BITD                (0x2000)
#define BITE                (0x4000)
#define BITF                (0x8000)

typedef enum BQ_DRTCMD_REG {
  bqCTL_STS    = 0x00, // Control Status      Unit:Hex
  bqSFT_ALT_A  = 0x02, // Safety Alert  A     Unit:Hex
  bqSFT_STS_A  = 0x03, // Safety Status A     Unit:Hex
  bqSFT_ALT_B  = 0x04, // Safety Alert  B     Unit:Hex
  bqSFT_STS_B  = 0x05, // Safety Status B     Unit:Hex
  bqSFT_ALT_C  = 0x06, // Safety Alert  C     Unit:Hex
  bqSFT_STS_C  = 0x07, // Safety Status C     Unit:Hex
  bqPF_ALT_A   = 0x0A, // PF Alert  A         Unit:Hex
  bqPF_STS_A   = 0x0B, // PF Status A         Unit:Hex
  bqPF_ALT_B   = 0x0C, // PF Alert  B         Unit:Hex
  bqPF_STS_B   = 0x0D, // PF Status B         Unit:Hex
  bqPF_ALT_C   = 0x0E, // PF Alert  C         Unit:Hex
  bqPF_STS_C   = 0x0F, // PF Status C         Unit:Hex
  bqPF_ALT_D   = 0x10, // PF Alert  D         Unit:Hex
  bqPF_STS_D   = 0x11, // PF Status D         Unit:Hex
  bqBAT_STS    = 0x12, // Battery Status      Unit:Hex
  bqCELL1_VOL  = 0x14, // First Cell Voltage  Unit:mV
  bqCELL16_VOL = 0x32, // Last Cell Voltage   Unit:mV
  bqSTACK_VOL  = 0x34, // Stack Voltage       Unit:userV
  bqPACK_VOL   = 0x36, // Pack Pin Voltage    Unit:userV
  bqLD_VOL     = 0x38, // LD Pin Voltage      Unit:userV
  bqCC2_CURR   = 0x3A, // CC2 Current         Unit:userA
  bqALT_STS    = 0x62, // Alarm Status        Unit:Hex
  bqALT_RW_STS = 0x64, // Alarm RAW Status    Unit:Hex
  bqALT_ENB    = 0x66, // Alarm Enable        Unit:Hex
  bqINT_TEMP   = 0x68, // Int Temperature     Unit:0.1K
  bqCFET_TEMP  = 0x6A, // CFETOFF Temperature Unit:0.1K
  bqDFET_TEMP  = 0x6C, // DFETOFF Temperature Unit:0.1K
  bqALT_TEMP   = 0x6E, // ALERT Temperature   Unit:0.1K
  bqTSP_TEMP   = 0x70, // TS1 Temperature     Unit:0.1K
  bqHDQ_TEMP   = 0x76, // HDQ Temperature     Unit:0.1K
  bqDCHG_TEMP  = 0x78, // DCHG Temperature    Unit:0.1K
  bqDDSG_TEMP  = 0x7a, // DDSG Temperature    Unit:0.1K
  bqFET_STS    = 0x7f, // FET Status          Unit:Hex
} bq_DirectCMD_reg;

typedef enum BQ_SUB_DRTCMD_OLY_REG {
  bqEXT_DEEP_SLP = 0x000E, // Exit Deep Sleep Mode.
  bqDEEP_SLP     = 0x000F, // Enter Deep Sleep Mode.
                           // Must be sent twice in a row within 
                           // 4s to take effect.
  bqSHUTDOWN    = 0x0010,  // Shutdown squence.
                           // Must be sent twice in a row within 4s
                           // to take effect if sealed.
	                         // If sent twice while unsealed,
                           // the shutdown delays are skipped 
  bqRESET       = 0x0012,  // Reset the device.
  bqPDSGTEST    = 0x001C,  // In FET Test mode, toggle the PDSG FET enable.
  bqFUSE_TOGGLE = 0x001D,  // Toggle FUSE state.
  bqPCHGTEST    = 0x001E,  // In FET Test mode, toggle the PCHG FET enable.
  bqCHGTEST     = 0x001F,  // In FET Test mode, toggle the CHG FET enable.
  bqDSGTEST     = 0x0020,  // In FET Test mode, toggle the DSG FET enable.
  bqFET_ENABLE  = 0x0022,  // Toggle FET_EN in Manufacturing Status.
  bqPF_ENABLE   = 0x0024,  // Toggle PF_EN in Manufacturing Status.
  bqPF_RESET    = 0x0029,  // Clear PF Status.
  bqSEAL        = 0x0030,  // Places the device in SEALED mode.
  bqRESET_PASSQ = 0x0082,  // Resets the integrated charge and timer.
  bqPTO_RECOVER = 0x008A,  // Triggers recovery from a Precharge Timeout
                           // (PTO) safety event.
  bqSET_CFGUPDATE  = 0x0090, // Enters CONFIG_UPDATE mode.
  bqEXIT_CFGUPDATE = 0x0092, // Exits CONFIG_UPDATE mode.
  bqDSG_PDSG_OFF   = 0x0093, // Disables DSG and PDSG FET drivers.
  bqCHG_PCHG_OFF   = 0x0094, // Disables CHG and PCHG FET drivers.
  bqALL_FETS_OFF   = 0x0095, // Disables CHG, DSG, PCHG,
                             // and PDSG FET driver.
  bqALL_FETS_ON    = 0x0096, // Allows all four FETs to be on 
                             // if other safety conditions are met.
                             // This clears the states set by the
                             // DSG_PDSG_OFF, CHG_PCHG_OFF, 
                             // and ALL_FETS_OFF commands.
  bqSLEEP_ENABLE  = 0x0099, // Enable SLEEP mode.
  bqSLEEP_DISABLE = 0x009a, // Disable SLEEP mode.
  bqOCDL_RECOVER  = 0x009b, // Recovers Overcurrent in Discharge Latch
                            // (OCDL) in the next execution of the safety
                            // engine (~1 second)
  bqSCDL_RECOVER = 0x009c,  // Recovers Short Circuit in Discharge Latch
                            // (SCDL) in the next execution of the safety
                            // engine (~1 second
  bqLOAD_DETECT_RESTART = 0x009d, // Restarts the timeout on the Load 
                                  // Detect (LD) pin current source
                                 // if it has already triggered.
  bqLOAD_DETECT_ON  = 0x009e, // Force the Load Detect (LD).
  bqLOAD_DETECT_OFF = 0x009f, // Force the Load Detect (LD).
  bqCFETOFF_LO = 0x2800, // Drive the CFETOFF pin to a low state.
  bqDFETOFF_LO = 0x2801, // Drive the DFETOFF pin to a low state.
  bqALERT_LO   = 0x2802, // Drive the ALERT pin to a low state.
  bqHDQ_LO     = 0x2806, // Drive the HDQ pin to a low state.
  bqDCHG_LO    = 0x2807, // Drive the DCHG pin to a low state.
  bqDDSG_LO    = 0x2808, // Drive the DDSG pin to a low state.
  bqCFETOFF_HI = 0x2810, // Drive the CFETOFF pin to a high state.
  bqDFETOFF_HI = 0x2811, // Drive the DFETOFF pin to a high state.
  bqALERT_HI   = 0x2812, // Drive the ALERT pin to a high state.
  bqHDQ_HI     = 0x2816, // Drive the HDQ pin to a high state.
  bqDCHG_HI    = 0x2817, // Drive the DCHG pin to a high state.
  bqDDSG_HI    = 0x2818, // Drive the DDSG pin to a high state.
  bqPF_FORCE_A = 0x2857, // First part of two-word command to force
                         // the command-based PF. M
  bqPF_FORCE_B = 0x29A3, // Second part of two-word command to force
                         // the command-based PF.
  bqSWAP_COMM_MODE = 0x29BC, // Change to the communications mode 
                             // previously configured by changing
  bqSWAP_TO_I2C = 0x29E7, // Select I2C Fast mode.
  bqSWAP_TO_SPI = 0x7C35, // Select SPI with CRC mode.
  bqSWAP_TO_HDQ = 0x7C40, // Select HDQ using ALERT pin mode.
}bq_SubDirectCMD_OLY_reg;

typedef enum BQ_SUB_DRTCMD_REG {
    bqDEVICE_NUMBER = 0x0001,
  bqFW_VERSION = 0x0002,
  bqHW_VERSION = 0x0003,
  bqIROM_SIG = 0x0004,
  bqSTATIC_CFG_SIG = 0x0005,
  bqPREV_MACWRITE = 0x0007,
  bqDROM_SIG = 0x0009,
  bqSECURITY_KEYS = 0x0035,
  bqSAVED_PF_STATUS = 0x0053,
  bqMANUFACTURINGSTATUS = 0x0057,
  bqMANU_DATA = 0x0070,
  bqDASTATUS1 = 0x0071,
  bqDASTATUS2 = 0x0072,
  bqDASTATUS3 = 0x0073,
  bqDASTATUS4 = 0x0074,
  bqDASTATUS5 = 0x0075,
  bqDASTATUS6 = 0x0076,
  bqDASTATUS7 = 0x0077,
  bqCUV_SNAPSHOT = 0x0080,
  bqCOV_SNAPSHOT = 0x0081,
  bqCB_ACTIVE_CELLS = 0x0083,
  bqCB_SET_LVL = 0x0084,
  bqCBSTATUS1 = 0x0085,
  bqCBSTATUS2 = 0x0086,
  bqCBSTATUS3 = 0x0087,
  bqFET_CONTROL = 0x0097,
  bqREG12_CONTROL = 0x0098,
  bqOTP_WR_CHECK = 0x00A0,
  bqOTP_WRITE = 0x00A1,
  bqREAD_CAL1 = 0xF081,
  bqCAL_CUV = 0xF090,
  bqCAL_COV = 0xF091,
}bq_SubDirectCMD_reg;

typedef enum BQ_DATA_MEM_REG {
  bqCell1Gain = 0x9180, // Calibration:Voltage:Cell 1 Gain			
  bqCell2Gain = 0x9182, // Calibration:Voltage:Cell 2 Gain			
  bqCell3Gain = 0x9184, // Calibration:Voltage:Cell 3 Gain			
  bqCell4Gain = 0x9186, // Calibration:Voltage:Cell 4 Gain			
  bqCell5Gain = 0x9188, // Calibration:Voltage:Cell 5 Gain			
  bqCell6Gain = 0x918A, // Calibration:Voltage:Cell 6 Gain			
  bqCell7Gain = 0x918C, // Calibration:Voltage:Cell 7 Gain			
  bqCell8Gain = 0x918E, // Calibration:Voltage:Cell 8 Gain			
  bqCell9Gain = 0x9190, // Calibration:Voltage:Cell 9 Gain			
  bqCell10Gain = 0x9192, // Calibration:Voltage:Cell 10 Gain			
  bqCell11Gain = 0x9194, // Calibration:Voltage:Cell 11 Gain			
  bqCell12Gain = 0x9196, // Calibration:Voltage:Cell 12 Gain			
  bqCell13Gain = 0x9198, // Calibration:Voltage:Cell 13 Gain			
  bqCell14Gain = 0x919A, // Calibration:Voltage:Cell 14 Gain			
  bqCell15Gain = 0x919C, // Calibration:Voltage:Cell 15 Gain			
  bqCell16Gain = 0x919E, // Calibration:Voltage:Cell 16 Gain			
  bqPackGain = 0x91A0, // Calibration:Voltage:Pack Gain			
  bqTOSGain = 0x91A2, // Calibration:Voltage:TOS Gain			
  bqLDGain = 0x91A4, // Calibration:Voltage:LD Gain			
  bqADCGain = 0x91A6, // Calibration:Voltage:ADC Gain			
  bqCCGain = 0x91A8, // Calibration:Current:CC Gain			
  bqCapacityGain = 0x91AC, // Calibration:Current:Capacity Gain			
  bqVcellOffset = 0x91B0, // Calibration:Vcell Offset:Vcell Offset			
  bqVdivOffset = 0x91B2, // Calibration:V Divider Offset:Vdiv Offset			
  bqCoulombCounterOffsetSamples = 0x91C6, // Calibration:Current Offset:
                                        // Coulomb Counter Offset Samples			
  bqBoardOffset = 0x91C8, // Calibration:Current Offset:Board Offset			
  bqInternalTempOffset = 0x91CA, // Calibration:Temperature:Internal Temp Offset			
  bqCFETOFFTempOffset = 0x91CB, // Calibration:Temperature:CFETOFF Temp Offset			
  bqDFETOFFTempOffset = 0x91CC, // Calibration:Temperature:DFETOFF Temp Offset			
  bqALERTTempOffset = 0x91CD, // Calibration:Temperature:ALERT Temp Offset			
  bqTS1TempOffset = 0x91CE, // Calibration:Temperature:TS1 Temp Offset			
  bqTS2TempOffset = 0x91CF, // Calibration:Temperature:TS2 Temp Offset			
  bqTS3TempOffset = 0x91D0, // Calibration:Temperature:TS3 Temp Offset			
  bqHDQTempOffset = 0x91D1, // Calibration:Temperature:HDQ Temp Offset			
  bqDCHGTempOffset = 0x91D2, // Calibration:Temperature:DCHG Temp Offset			
  bqDDSGTempOffset = 0x91D3, // Calibration:Temperature:DDSG Temp Offset			
  bqIntGain = 0x91E2, // Calibration:Internal Temp Model:Int Gain			
  bqIntbaseoffset = 0x91E4, // Calibration:Internal Temp Model:Int base offset			
  bqIntMaximumAD = 0x91E6, // Calibration:Internal Temp Model:Int Maximum AD			
  bqIntMaximumTemp = 0x91E8, // Calibration:Internal Temp Model:Int Maximum Temp			
  bqT18kCoeffa1 = 0x91EA, // Calibration:18K Temperature Model:Coeff a1			
  bqT18kCoeffa2 = 0x91EC, // Calibration:18K Temperature Model:Coeff a2			
  bqT18kCoeffa3 = 0x91EE, // Calibration:18K Temperature Model:Coeff a3			
  bqT18kCoeffa4 = 0x91F0, // Calibration:18K Temperature Model:Coeff a4			
  bqT18kCoeffa5 = 0x91F2, // Calibration:18K Temperature Model:Coeff a5			
  bqT18kCoeffb1 = 0x91F4, // Calibration:18K Temperature Model:Coeff b1			
  bqT18kCoeffb2 = 0x91F6, // Calibration:18K Temperature Model:Coeff b2			
  bqT18kCoeffb3 = 0x91F8, // Calibration:18K Temperature Model:Coeff b3			
  bqT18kCoeffb4 = 0x91FA, // Calibration:18K Temperature Model:Coeff b4			
  bqT18kAdc0 = 0x91FE, // Calibration:18K Temperature Model:Adc0			
  bqT180kCoeffa1 = 0x9200, // Calibration:180K Temperature Model:Coeff a1			
  bqT180kCoeffa2 = 0x9202, // Calibration:180K Temperature Model:Coeff a2			
  bqT180kCoeffa3 = 0x9204, // Calibration:180K Temperature Model:Coeff a3			
  bqT180kCoeffa4 = 0x9206, // Calibration:180K Temperature Model:Coeff a4			
  bqT180kCoeffa5 = 0x9208, // Calibration:180K Temperature Model:Coeff a5			
  bqT180kCoeffb1 = 0x920A, // Calibration:180K Temperature Model:Coeff b1			
  bqT180kCoeffb2 = 0x920C, // Calibration:180K Temperature Model:Coeff b2			
  bqT180kCoeffb3 = 0x920E, // Calibration:180K Temperature Model:Coeff b3			
  bqT180kCoeffb4 = 0x9210, // Calibration:180K Temperature Model:Coeff b4			
  bqT180kAdc0 = 0x9214, // Calibration:180K Temperature Model:Adc0			
  bqCustomCoeffa1 = 0x9216, // Calibration:Custom Temperature Model:Coeff a1			
  bqCustomCoeffa2 = 0x9218, // Calibration:Custom Temperature Model:Coeff a2			
  bqCustomCoeffa3 = 0x921A, // Calibration:Custom Temperature Model:Coeff a3			
  bqCustomCoeffa4 = 0x921C, // Calibration:Custom Temperature Model:Coeff a4			
  bqCustomCoeffa5 = 0x921E, // Calibration:Custom Temperature Model:Coeff a5			
  bqCustomCoeffb1 = 0x9220, // Calibration:Custom Temperature Model:Coeff b1			
  bqCustomCoeffb2 = 0x9222, // Calibration:Custom Temperature Model:Coeff b2			
  bqCustomCoeffb3 = 0x9224, // Calibration:Custom Temperature Model:Coeff b3			
  bqCustomCoeffb4 = 0x9226, // Calibration:Custom Temperature Model:Coeff b4			
  bqCustomRc0 = 0x9228, // Calibration:Custom Temperature Model:Rc0			
  bqCustomAdc0 = 0x922A, // Calibration:Custom Temperature Model:Adc0			
  bqCoulombCounterDeadband = 0x922D, // Calibration:Current Deadband:
                                   // Coulomb Counter Deadband			
  bqCUVThresholdOverride = 0x91D4, // Calibration:CUV:CUV Threshold Override			
  bqCOVThresholdOverride = 0x91D6, // Calibration:COV:COV Threshold Override			
  bqMinBlowFuseVoltage = 0x9231, // Settings:Fuse:Min Blow Fuse Voltage			
  bqFuseBlowTimeout = 0x9233, // Settings:Fuse:Fuse Blow Timeout			
  bqPowerConfig = 0x9234, // Settings:Configuration:Power Config			
  bqREG12Config = 0x9236, // Settings:Configuration:REG12 Config			
  bqREG0Config = 0x9237, // Settings:Configuration:REG0 Config			
  bqHWDRegulatorOptions = 0x9238, // Settings:Configuration:
                                  // HWD Regulator Options			
  bqCommType = 0x9239, // Settings:Configuration:Comm Type			
  bqI2CAddress = 0x923A, // Settings:Configuration:I2C Address			
  bqSPIConfiguration = 0x923C, // Settings:Configuration:SPI Configuration			
  bqCommIdleTime = 0x923D, // Settings:Configuration:Comm Idle Time			
  bqCFETOFFPinConfig = 0x92FA, // Settings:Configuration:CFETOFF Pin Config			
  bqDFETOFFPinConfig = 0x92FB, // Settings:Configuration:DFETOFF Pin Config			
  bqALERTPinConfig = 0x92FC, // Settings:Configuration:ALERT Pin Config			
  bqTS1Config = 0x92FD, // Settings:Configuration:TS1 Config			
  bqTS2Config = 0x92FE, // Settings:Configuration:TS2 Config			
  bqTS3Config = 0x92FF, // Settings:Configuration:TS3 Config			
  bqHDQPinConfig = 0x9300, // Settings:Configuration:HDQ Pin Config			
  bqDCHGPinConfig = 0x9301, // Settings:Configuration:DCHG Pin Config			
  bqDDSGPinConfig = 0x9302, // Settings:Configuration:DDSG Pin Config			
  bqDAConfiguration = 0x9303, // Settings:Configuration:DA Configuration			
  bqVCellMode = 0x9304, // Settings:Configuration:Vcell Mode			
  bqCC3Samples = 0x9307, // Settings:Configuration:CC3 Samples			
  bqProtectionConfiguration = 0x925F, // Settings:Protection:
                                    // Protection Configuration			
  bqEnableProtectionsA = 0x9261, // Settings:Protection:Enabled Protections A			
  bqEnableProtectionsB = 0x9262, // Settings:Protection:Enabled Protections B			
  bqEnableProtectionsC = 0x9263, // Settings:Protection:Enabled Protections C			
  bqCHGFETProtectionsA = 0x9265, // Settings:Protection:CHG FET Protections A			
  bqCHGFETProtectionsB = 0x9266, // Settings:Protection:CHG FET Protections B			
  bqCHGFETProtectionsC = 0x9267, // Settings:Protection:CHG FET Protections C			
  bqDSGFETProtectionsA = 0x9269, // Settings:Protection:DSG FET Protections A			
  bqDSGFETProtectionsB = 0x926A, // Settings:Protection:DSG FET Protections B			
  bqDSGFETProtectionsC = 0x926B, // Settings:Protection:DSG FET Protections C			
  bqBodyDiodeThreshold = 0x9273, // Settings:Protection:Body Diode Threshold			
  bqDefaultAlarmMask = 0x926D, // Settings:Alarm:Default Alarm Mask			
  bqSFAlertMaskA = 0x926F, // Settings:Alarm:SF Alert Mask A			
  bqSFAlertMaskB = 0x9270, // Settings:Alarm:SF Alert Mask B			
  bqSFAlertMaskC = 0x9271, // Settings:Alarm:SF Alert Mask C			
  bqPFAlertMaskA = 0x92C4, // Settings:Alarm:PF Alert Mask A			
  bqPFAlertMaskB = 0x92C5, // Settings:Alarm:PF Alert Mask B			
  bqPFAlertMaskC = 0x92C6, // Settings:Alarm:PF Alert Mask C			
  bqPFAlertMaskD = 0x92C7, // Settings:Alarm:PF Alert Mask D			
  bqEnabledPFA = 0x92C0, // Settings:Permanent Failure:Enabled PF A			
  bqEnabledPFB = 0x92C1, // Settings:Permanent Failure:Enabled PF B			
  bqEnabledPFC = 0x92C2, // Settings:Permanent Failure:Enabled PF C			
  bqEnabledPFD = 0x92C3, // Settings:Permanent Failure:Enabled PF D			
  bqFETOptions = 0x9308, // Settings:FET:FET Options			
  bqChgPumpControl = 0x9309, // Settings:FET:Chg Pump Control			
  bqPrechargeStartVoltage = 0x930A, // Settings:FET:Precharge Start Voltage			
  bqPrechargeStopVoltage = 0x930C, // Settings:FET:Precharge Stop Voltage			
  bqPredischargeTimeout = 0x930E, // Settings:FET:Predischarge Timeout			
  bqPredischargeStopDelta = 0x930F, // Settings:FET:Predischarge Stop Delta			
  bqDsgCurrentThreshold = 0x9310, // Settings:Current Thresholds:
                                // Dsg Current Threshold			
  bqChgCurrentThreshold = 0x9312, // Settings:Current Thresholds:
                                // Chg Current Threshold			
  bqCheckTime = 0x9314, // Settings:Cell Open-Wire:Check Time			
  bqCell1Interconnect = 0x9315, // Settings:Interconnect Resistances:Cell 1
                              // Interconnect			
  bqCell2Interconnect = 0x9317, // Settings:Interconnect Resistances:Cell 2 	
                              // Interconnect			
  bqCell3Interconnect = 0x9319, // Settings:Interconnect Resistances:Cell 3
                              // Interconnect
  bqCell4Interconnect = 0x931B, // Settings:Interconnect Resistances:Cell 4 	
                              // Interconnect			
  bqCell5Interconnect = 0x931D, // Settings:Interconnect Resistances:Cell 5 	
                              // Interconnect			
  bqCell6Interconnect = 0x931F, // Settings:Interconnect Resistances:Cell 6 	
                              // Interconnect			
  bqCell7Interconnect = 0x9321, // Settings:Interconnect Resistances:Cell 7 	
                              // Interconnect			
  bqCell8Interconnect = 0x9323, // Settings:Interconnect Resistances:Cell 8 	
                              // Interconnect			
  bqCell9Interconnect = 0x9325, // Settings:Interconnect Resistances:Cell 9 	
                              // Interconnect			
  bqCell10Interconnect = 0x9327, // Settings:Interconnect Resistances:Cell 10 	
                               // Interconnect			
  bqCell11Interconnect = 0x9329, // Settings:Interconnect Resistances:Cell 11 	
                               // Interconnect			
  bqCell12Interconnect = 0x932B, // Settings:Interconnect Resistances:Cell 12 
                               // Interconnect			
  bqCell13Interconnect = 0x932D, // Settings:Interconnect Resistances:Cell 13 
                               // Interconnect			
  bqCell14Interconnect = 0x932F, // Settings:Interconnect Resistances:Cell 14 
                               // Interconnect			
  bqCell15Interconnect = 0x9331, // Settings:Interconnect Resistances:Cell 15 	
                               // Interconnect			
  bqCell16Interconnect = 0x9333, // Settings:Interconnect Resistances:Cell 16 	
                               // Interconnect			
  bqMfgStatusInit = 0x9343, // Settings:Manufacturing:Mfg Status Init			
  bqBalancingConfiguration = 0x9335, // Settings:Cell Balancing Config:
                                   // Balancing Configuration			
  bqMinCellTemp = 0x9336, // Settings:Cell Balancing Config:Min Cell Temp			
  bqMaxCellTemp = 0x9337, // Settings:Cell Balancing Config:Max Cell Temp			
  bqMaxInternalTemp = 0x9338, // Settings:Cell Balancing Config:
                              // Max Internal Temp			
  bqCellBalanceInterval = 0x9339, // Settings:Cell Balancing Config:
                                // Cell Balance Interval			
  bqCellBalanceMaxCells = 0x933A, // Settings:Cell Balancing Config:
                                // Cell Balance Max Cells			
  bqCellBalanceMinCellVCharge = 0x933B, // Settings:Cell Balancing Config:
                                      // Cell Balance Min Cell V (Charge)			
  bqCellBalanceMinDeltaCharge = 0x933D, // Settings:Cell Balancing Config:
                                      // Cell Balance Min Delta (Charge)			
  bqCellBalanceStopDeltaCharge = 0x933E, // Settings:Cell Balancing Config:
                                       // Cell Balance Stop Delta (Charge)			
  bqCellBalanceMinCellVRelax = 0x933F, // Settings:Cell Balancing Config:
                                     // Cell Balance Min Cell V (Relax)			
  bqCellBalanceMinDeltaRelax = 0x9341, // Settings:Cell Balancing Config:
                                     // Cell Balance Min Delta (Relax)			
  bqCellBalanceStopDeltaRelax = 0x9342, // Settings:Cell Balancing Config:
                                      // Cell Balance Stop Delta (Relax)			
  bqShutdownCellVoltage = 0x923F, // Power:Shutdown:Shutdown Cell Voltage			
  bqShutdownStackVoltage = 0x9241, // Power:Shutdown:Shutdown Stack Voltage			
  bqLowVShutdownDelay = 0x9243, // Power:Shutdown:Low V Shutdown Delay			
  bqShutdownTemperature = 0x9244, // Power:Shutdown:Shutdown Temperature			
  bqShutdownTemperatureDelay = 0x9245, // Power:Shutdown:
                                     // Shutdown Temperature Delay			
  bqFETOffDelay = 0x9252, // Power:Shutdown:FET Off Delay			
  bqShutdownCommandDelay = 0x9253, // Power:Shutdown:Shutdown Command Delay			
  bqAutoShutdownTime = 0x9254, // Power:Shutdown:Auto Shutdown Time			
  bqRAMFailShutdownTime = 0x9255, // Power:Shutdown:RAM Fail Shutdown Time			
  bqSleepCurrent = 0x9248, // Power:Sleep:Sleep Current			
  bqVoltageTime = 0x924A, // Power:Sleep:Voltage Time			
  bqWakeComparatorCurrent = 0x924B, // Power:Sleep:Wake Comparator Current			
  bqSleepHysteresisTime = 0x924D, // Power:Sleep:Sleep Hysteresis Time			
  bqSleepChargerVoltageThreshold = 0x924E, // Power:Sleep:
                                         // Sleep Charger Voltage Threshold			
  bqSleepChargerPACKTOSDelta = 0x9250, // Power:Sleep:Sleep 
                                       // Charger PACK-TOS Delta			
  bqConfigRAMSignature = 0x91E0, // System Data:Integrity:Config RAM Signature			
  bqCUVThreshold = 0x9275, // Protections:CUV:Threshold			
  bqCUVDelay = 0x9276, // Protections:CUV:Delay			
  bqCUVRecoveryHysteresis = 0x927B, // Protections:CUV:Recovery Hysteresis			
  bqCOVThreshold = 0x9278, // Protections:COV:Threshold			
  bqCOVDelay = 0x9279, // Protections:COV:Delay			
  bqCOVRecoveryHysteresis = 0x927C, // Protections:COV:Recovery Hysteresis			
  bqCOVLLatchLimit = 0x927D, // Protections:COVL:Latch Limit			
  bqCOVLCounterDecDelay = 0x927E, // Protections:COVL:Counter Dec Delay			
  bqCOVLRecoveryTime = 0x927F, // Protections:COVL:Recovery Time			
  bqOCCThreshold = 0x9280, // Protections:OCC:Threshold			
  bqOCCDelay = 0x9281, // Protections:OCC:Delay			
  bqOCCRecoveryThreshold = 0x9288, // Protections:OCC:Recovery Threshold			
  bqOCCPACKTOSDelta = 0x92B0, // Protections:OCC:PACK-TOS Delta			
  bqOCD1Threshold = 0x9282, // Protections:OCD1:Threshold			
  bqOCD1Delay = 0x9283, // Protections:OCD1:Delay			
  bqOCD2Threshold = 0x9284, // Protections:OCD2:Threshold			
  bqOCD2Delay = 0x9285, // Protections:OCD2:Delay			
  bqSCDThreshold = 0x9286, // Protections:SCD:Threshold			
  bqSCDDelay = 0x9287, // Protections:SCD:Delay			
  bqSCDRecoveryTime = 0x9294, // Protections:SCD:Recovery Time			
  bqOCD3Threshold = 0x928A, // Protections:OCD3:Threshold			
  bqOCD3Delay = 0x928C, // Protections:OCD3:Delay			
  bqOCDRecoveryThreshold = 0x928D, // Protections:OCD:Recovery Threshold			
  bqOCDLLatchLimit = 0x928F, // Protections:OCDL:Latch Limit			
  bqOCDLCounterDecDelay = 0x9290, // Protections:OCDL:Counter Dec Delay			
  bqOCDLRecoveryTime = 0x9291, // Protections:OCDL:Recovery Time			
  bqOCDLRecoveryThreshold = 0x9292, // Protections:OCDL:Recovery Threshold			
  bqSCDLLatchLimit = 0x9295, // Protections:SCDL:Latch Limit			
  bqSCDLCounterDecDelay = 0x9296, // Protections:SCDL:Counter Dec Delay			
  bqSCDLRecoveryTime = 0x9297, // Protections:SCDL:Recovery Time			
  bqSCDLRecoveryThreshold = 0x9298, // Protections:SCDL:Recovery Threshold			
  bqOTCThreshold = 0x929A, // Protections:OTC:Threshold			
  bqOTCDelay = 0x920B, // Protections:OTC:Delay			
  bqOTCRecovery = 0x929C, // Protections:OTC:Recovery			
  bqOTDThreshold = 0x929D, // Protections:OTD:Threshold			
  bqOTDDelay = 0x929E, // Protections:OTD:Delay			
  bqOTDRecovery = 0x929F, // Protections:OTD:Recovery			
  bqOTFThreshold = 0x92A0, // Protections:OTF:Threshold			
  bqOTFDelay = 0x92A1, // Protections:OTF:Delay			
  bqOTFRecovery = 0x92A2, // Protections:OTF:Recovery			
  bqOTINTThreshold = 0x92A3, // Protections:OTINT:Threshold			
  bqOTINTDelay = 0x92A4, // Protections:OTINT:Delay			
  bqOTINTRecovery = 0x92A5, // Protections:OTINT:Recovery			
  bqUTCThreshold = 0x92A6, // Protections:UTC:Threshold			
  bqUTCDelay = 0x92A7, // Protections:UTC:Delay			
  bqUTCRecovery = 0x92A8, // Protections:UTC:Recovery			
  bqUTDThreshold = 0x92A9, // Protections:UTD:Threshold			
  bqUTDDelay = 0x92AA, // Protections:UTD:Delay			
  bqUTDRecovery = 0x92AB, // Protections:UTD:Recovery			
  bqUTINTThreshold = 0x92AC, // Protections:UTINT:Threshold			
  bqUTINTDelay = 0x92AD, // Protections:UTINT:Delay			
  bqUTINTRecovery = 0x92AE, // Protections:UTINT:Recovery			
  bqProtectionsRecoveryTime = 0x92AF, // Protections:Recovery:Time			
  bqHWDDelay = 0x92B2, // Protections:HWD:Delay			
  bqLoadDetectActiveTime = 0x92B4, // Protections:Load Detect:Active Time			
  bqLoadDetectRetryDelay = 0x92B5, // Protections:Load Detect:Retry Delay			
  bqLoadDetectTimeout = 0x92B6, // Protections:Load Detect:Timeout			
  bqPTOChargeThreshold = 0x92BA, // Protections:PTO:Charge Threshold			
  bqPTODelay = 0x92BC, // Protections:PTO:Delay			
  bqPTOReset = 0x92BE, // Protections:PTO:Reset			
  bqCUDEPThreshold = 0x92C8, // Permanent Fail:CUDEP:Threshold			
  bqCUDEPDelay = 0x92CA, // Permanent Fail:CUDEP:Delay			
  bqSUVThreshold = 0x92CB, // Permanent Fail:SUV:Threshold			
  bqSUVDelay = 0x92CD, // Permanent Fail:SUV:Delay			
  bqSOVThreshold = 0x92CE, // Permanent Fail:SOV:Threshold			
  bqSOVDelay = 0x92D0, // Permanent Fail:SOV:Delay			
  bqTOSSThreshold = 0x92D1, // Permanent Fail:TOS:Threshold			
  bqTOSSDelay = 0x92D3, // Permanent Fail:TOS:Delay			
  bqSOCCThreshold = 0x92D4, // Permanent Fail:SOCC:Threshold			
  bqSOCCDelay = 0x92D6, // Permanent Fail:SOCC:Delay			
  bqSOCDThreshold = 0x92D7, // Permanent Fail:SOCD:Threshold			
  bqSOCDDelay = 0x92D9, // Permanent Fail:SOCD:Delay			
  bqSOTThreshold = 0x92DA, // Permanent Fail:SOT:Threshold			
  bqSOTDelay = 0x92DB, // Permanent Fail:SOT:Delay			
  bqSOTFThreshold = 0x92DC, // Permanent Fail:SOTF:Threshold			
  bqSOTFDelay = 0x92DD, // Permanent Fail:SOTF:Delay			
  bqVIMRCheckVoltage = 0x92DE, // Permanent Fail:VIMR:Check Voltage			
  bqVIMRMaxRelaxCurrent = 0x92E0, // Permanent Fail:VIMR:Max Relax Current			
  bqVIMRThreshold = 0x92E2, // Permanent Fail:VIMR:Threshold			
  bqVIMRDelay = 0x92E4, // Permanent Fail:VIMR:Delay			
  bqVIMRRelaxMinDuration = 0x92E5, // Permanent Fail:VIMR:Relax Min Duration			
  bqVIMACheckVoltage = 0x92E7, // Permanent Fail:VIMA:Check Voltage			
  bqVIMAMinActiveCurrent = 0x92E9, // Permanent Fail:VIMA:Min Active Current			
  bqVIMAThreshold = 0x92EB, // Permanent Fail:VIMA:Threshold			
  bqVIMADelay = 0x92ED, // Permanent Fail:VIMA:Delay			
  bqCFETFOFFThreshold = 0x92EE, // Permanent Fail:CFETF:OFF Threshold			
  bqCFETFOFFDelay = 0x92F0, // Permanent Fail:CFETF:OFF Delay			
  bqDFETFOFFThreshold = 0x92F1, // Permanent Fail:DFETF:OFF Threshold			
  bqDFETFOFFDelay = 0x92F3, // Permanent Fail:DFETF:OFF Delay			
  bqVSSFFailThreshold = 0x92F4, // Permanent Fail:VSSF:Fail Threshold			
  bqVSSFDelay = 0x92F6, // Permanent Fail:VSSF:Delay			
  bqPF2LVLDelay = 0x92F7, // Permanent Fail:2LVL:Delay			
  bqLFOFDelay = 0x92F8, // Permanent Fail:LFOF:Delay			
  bqHWMXDelay = 0x92F9, // Permanent Fail:HWMX:Delay			
  bqSecuritySettings = 0x9256, // Security:Settings:Security Settings			
  bqUnsealKeyStep1 = 0x9257, // Security:Keys:Unseal Key Step 1			
  bqUnsealKeyStep2 = 0x9259, // Security:Keys:Unseal Key Step 2			
  bqFullAccessKeyStep1 = 0x925B, // Security:Keys:Full Access Key Step 1			
  bqFullAccessKeyStep2 = 0x925D, // Security:Keys:Full Access Key Step 2  
}bq_DataMemoryCMD_reg;

typedef enum {
  bqBS_CFGUPDATE = BIT0, // Device is in CONFIG_UPDATE mod
  bqBS_PCHG_MODE = BIT1, // Device is in PRECHARGE mode.
  bqBS_SLEEP_EN  = BIT2, // SLEEP mode is allowed when other SLEEP conditions are met.
  bqBS_POR       = BIT3, // Full reset has not occurred since last exit of CONFIG_UPDATE mode
  bqBS_WD        = BIT4, // Previous reset was caused by the watchdog timer.
  bqBS_COW_CHK   = BIT5, // Device is actively performing a cell open-wire check.
  bqBS_OTPW      = BIT6, // Writes to OTP are pending
  bqBS_OTPB      = BIT7, // Writes to OTP are blocke
  bqBS_SEC0      = BIT8, // Device is in SEALED 
  bqBS_SEC1      = BIT9, // Device is in SEALED 
  bqBS_FUSE      = BITA, // FUSE pin was asserted by device or secondary protector.
  bqBS_SS        = BITB, // safety fault is triggered.
  bqBS_PF        = BITC, // Permanent Fail fault has triggered.
  bqBS_SD_CMD    = BITD, // Shutdown due to command or pin is pending
  bqBS_SLEEP     = BITF, // Device in SLEEP mod
} bq_batSTS_enum_t;


typedef enum {
  bqAS_WAKE        = BIT0, // Wakened From Sleep Mode.
  bqAS_ADSCAN      = BIT1, // Voltage ADC Scan Complete.
  bqAS_CB          = BIT2, // Cell Balancing is Active.
  bqAS_FUSE        = BIT3, // FUSE Pin Driven.
  bqAS_SHUTV       = BIT4, // Shutdown Stack Voltage.
  bqAS_XDSG        = BIT5, // DSG FET Off.
  bqAS_XCHG        = BIT6, // CHG FET Off.
  bqAS_FULLSCAN    = BIT7, // Full Voltage Scan Complete.
  bqAS_INITCOMP    = BIT9, // Initiallization Complete.
  bqAS_INITSTART   = BITA, // Initialization Started.
  bqAS_MSK_PFALERT = BITB, // PF_Alarm Mask A_B_C_D Trigged.
  bqAS_MSK_SFALERT = BITC, // SF_Alarm Mask A_B_C Trigged.
  bqAS_PF          = BITD, // Permanent Fail 
  bqAS_SSA         = BITE, // Safety Status A
  bqAS_SSBC        = BITF, // Safety Status B_C
  bqAS_FIELD       = 0xFF, // Fiueld Status
} bq_altSTS_enum_t;

typedef enum {
  bqSA_CUV  = BIT2, // Cell Undervoltage Protect
  bqSA_COV  = BIT3, // Cell Overvoltage Protection
  bqSA_OCC  = BIT4, // Overcurrent in Charge Protectio
  bqSA_OCD1 = BIT5, // Overcurrent in Discharge 1st Tier Protection
  bqSA_OCD2 = BIT6, // Overcurrent in Discharge 2nd Tier Protectio
  bqSA_OCD  = BIT7, // Short Circuit in Discharge Protection
} afe_safeA_enum_t;

typedef enum {
  bqSB_UTC   = BIT0, // Undertemperature in Charge
  bqSB_UTD   = BIT1, // Undertemperature in Discharg
  bqSB_UTINT = BIT2, // Internal Undertemperature
  bqSB_OTC   = BIT4, // Overtemperature in Charge
  bqSB_OTD   = BIT5, // Overtemperature in Discharge
  bqSB_OTINT = BIT6, // Internal Overtemperature
  bqSB_OTF   = BIT7, // FET Overtemperature
} afe_safeB_enum_t;

typedef enum {
  bqSC_HWDF = BIT1, // Host Watchdog Fault
  bqSC_PTOS = BIT2, // Precharge Timeout
  bqSC_COVL = BIT4, // Cell Overvoltage Latch
  bqSC_OCDL = BIT5, // Overcurrent in Discharge Latch
  bqSC_SCDL = BIT6, // Short Circuit in Discharge Latch
  bqSC_OCD3 = BIT7, // Overcurrent in Discharge 3rd Tier Protection
} afe_safeC_enum_t;

typedef enum {
  bqPA_SUV   = BIT0, // Safety Cell Undervoltage Permanent Fail
  bqPA_SOV   = BIT1, // Safety Cell Overvoltage Permanent Fail
  bqPA_SOCC  = BIT2, // Safety Overcurrent in Charge Permanent Fail
  bqPA_SOCD  = BIT3, // Safety Overcurrent in Discharge Permanent Fail
  bqPA_SOT   = BIT4, // Safety Overtemperature Permanent Fail
  bqPA_SOTF  = BIT6, // Safety Overtemperature FET Permanent Fail
  bqPA_CUDEP = BIT7, // Copper Deposition Permanent Fail
} afe_pfA_enum_t;

typedef enum {
  bqPB_CFETF = BIT0, // Charge FET Permanent Fail
  bqPB_DFETF = BIT1, // Discharge FET Permanent Fail
  bqPB_LVL2  = BIT2, // Second Level Protector Permanent Fail
  bqPB_VIMR  = BIT3, // Voltage Imbalance at Rest Permanent Fail
  bqPB_VIMA  = BIT4, // Voltage Imbalance Active Permanent Fail
  bqPB_SCDL  = BIT7, // Short Circuit in Discharge Latch Permanent Fail
} afe_pfB_enum_t;

typedef enum {
  bqPC_OTPF = BIT0, // OTP Memory Permanent Fail
  bqPC_DRMF = BIT1, // Data ROM Permanent Fail
  bqPC_IRMF = BIT2, // Instruction ROM Permanent Fail
  bqPC_LFOF = BIT3, // Internal LFO Permanent Fail
  bqPC_VREF = BIT4, // Internal Voltage Reference Permanent Fail
  bqPC_VSSF = BIT5, // Internal VSS Measurement Permanent Fail
  bqPC_HWMX = BIT6, // Hardware Mux Permanent Fail
  bqPC_CMDF = BIT7, // Commanded Permanent Fail
} afe_pfC_enum_t;

typedef enum {
  bqPD_TOSF = BIT0, // PF_Alert_D Top of Stack vs Cell Sum Permanent Fail
} afe_pfD_enum_t;

typedef enum {
  bqFET_CHG   = BIT0, // Indicates the status of the CHG FET.
  bqFET_PCHG  = BIT1, // Indicates the status of the PCHG FET.
  bqFET_DSG   = BIT2, // Indicates the status of the DSG FET.
  bqFET_PDSG  = BIT3, // Indicates the status of the PDSG FET.
  bqFET_DCHG  = BIT4, // Indicates the status of the DCHG pin.
  bqFET_DDSG  = BIT5, // Indicates the status of the DDSG pin.
  bqFET_ALART = BIT6, // Indicates the status of the ALERT pin.
} afe_FET_enum_t;

typedef enum {
  bqTES_PCHG  = BIT0, //  PCHG FET is enabled in FET Test Mod.
  bqTES_CHG   = BIT1, // CHG FET is enabled in FET Test Mod.
  bqTES_DSG   = BIT2, // DSG FET is enabled in FET Test Mode.
  bqFET_ENB   = BIT4, // Normal FET control is enable.
  bqTES_PDHG  = BIT5, // PDSG FET is enabled in FET Test Mode.
  bqPF_ENB    = BIT6, // Permanent Failure checks are enabled.
  bqOTPW_EN   = BIT7, // Device may program OTP during normal operatio.
} afe_MANF_enum_t;
#endif