
#ifndef SX1276_7_8_H
#define SX1276_7_8_H

#include <stdbool.h>
#include <string.h>
#include <stdlib.h>
#include "pin_description.h"
#include "stm32f1xx_hal.h"
#include "cmsis_os.h"
#include "GUID.h"

#define LoRaMaxMessageLength 25
#define LoRaAddressCount 6

extern const uint8_t NetworkID;
extern const uint16_t LoRaAddress;

#define LoRaGUIDSize				16
#define LoRaNetworkIDSize			1
#define LoRaAddressSize				2



#define SX1278Drv_RegFifo							0x00
#define SX1278Drv_RegOpMode							0x01
#define SX1278Drv_RegFrMsb							0x06
#define SX1278Drv_RegFrMid							0x07
#define SX1278Drv_RegFrLsb							0x08
#define SX1278Drv_RegPaConfig						0x09
#define SX1278Drv_RegPaRamp							0x0A
#define SX1278Drv_RegOcp							0x0B
#define SX1278Drv_RegLna							0x0C
#define SX1278Drv_RegDioMapping1					0x40
#define SX1278Drv_RegDIOMapping2					0x41
#define SX1278Drv_RegVersion						0x42
#define SX1278Drv_RegTcxo							0x4B
#define SX1278Drv_RegPaDac							0x4D
#define SX1278Drv_FormerTemp						0x5B
#define SX1278Drv_RegAgcRef							0x61
#define SX1278Drv_RegAgcThresh1						0x62
#define SX1278Drv_RegAgcThresh2						0x63
#define SX1278Drv_RegAgcThresh3						0x64

// LoRa registers
#define SX1278Drv_RegLoRaFifoAddrPtr				0x0D
#define SX1278Drv_RegLoRaFifoTxBaseAddr				0x0E
#define SX1278Drv_RegLoRaFifoRxBaseAddr				0x0F
#define SX1278Drv_RegLoRaFifoRxCurrentaddr			0x10
#define SX1278Drv_RegLoRaIrqFlagsMask				0x11
#define SX1278Drv_RegLoRaIrqFlags					0x12
#define SX1278Drv_RegLoRaRxNbBytes					0x13
#define SX1278Drv_RegLoRaRxHeaderCntValueMsb		0x14
#define SX1278Drv_RegLoRaRxHeaderCntValueLsb		0x15
#define SX1278Drv_RegLoRaRxPacketCntValueMsb		0x16
#define SX1278Drv_RegLoRaRxPacketCntValueLsb		0x17
#define SX1278Drv_RegLoRaModemStat					0x18
#define SX1278Drv_RegLoRaPktSnrValue				0x19
#define SX1278Drv_RegLoRaPktRssiValue				0x1A
#define SX1278Drv_RegLoRaRssiValue					0x1B
#define SX1278Drv_RegLoRaHopChannel					0x1C
#define SX1278Drv_RegLoRaModemConfig1				0x1D
#define SX1278Drv_RegLoRaModemConfig2				0x1E
#define SX1278Drv_RegLoRaSymbTimeoutLsb				0x1F
#define SX1278Drv_RegLoRaPreambleMsb				0x20
#define SX1278Drv_RegLoRaPreambleLsb				0x21
#define SX1278Drv_RegLoRaPayloadLength				0x22
#define SX1278Drv_RegLoRaMaxPayloadLength			0x23
#define SX1278Drv_RegLoRaHopPeriod					0x24
#define SX1278Drv_RegLoRaFifoRxByteAddr				0x25
#define SX1278Drv_RegLoRaModemConfig3				0x26
#define SX1278Drv_RegLoRaFeiMsb						0x28
#define SX1278Drv_RegLoRaFeiMid						0x29
#define SX1278Drv_RegLoRaFeiLsb						0x2A
#define SX1278Drv_RegLoRaRssiWideband				0x2C
#define SX1278Drv_RegLoRaDetectOptimize				0x31
#define SX1278Drv_RegLoRaInvertIQ					0x33
#define SX1278Drv_RegLoRaDetectionThreshold			0x37
#define SX1278Drv_RegLoRaSyncWord					0x39

//FSK/OOK Registers
#define SX1278Drv_RegFskBitRateMsb					0x02
#define SX1278Drv_RegFskBitRateLsb					0x03
#define SX1278Drv_RegFskFdevMsb						0x04
#define SX1278Drv_RegFskFdevLsb						0x05
#define SX1278Drv_RegFskRxConfig					0x0D
#define SX1278Drv_RegFskRssiConfig					0x0E
#define SX1278Drv_RegFskRssiCollision				0x0F
#define SX1278Drv_RegFskRssiThresh					0x10
#define SX1278Drv_RegFskRssiValue					0x11
#define SX1278Drv_RegFskRxBw						0x12
#define SX1278Drv_RegFskAfcBw						0x13
#define SX1278Drv_RegFskOokPeak						0x14
#define SX1278Drv_RegFskOokFix						0x15
#define SX1278Drv_RegFskOokAvg						0x16
#define SX1278Drv_RegFskAfcFei						0x1A
#define SX1278Drv_RegFskAfcMsb						0x1B
#define SX1278Drv_RegFskAfcLsb						0x1C
#define SX1278Drv_RegFskFeiMsb						0x1D
#define SX1278Drv_RegFskFeiLsb						0x1E
#define SX1278Drv_RegFskPreambleDetect				0x1F
#define SX1278Drv_RegFskRxTimeout1					0x20
#define SX1278Drv_RegFskRxTimeout2					0x21
#define SX1278Drv_RegFskRxTimeout3					0x22
#define SX1278Drv_RegFskRxDelay						0x23
#define SX1278Drv_RegFskOsc							0x24
#define SX1278Drv_RegFskPreambleMsb					0x25
#define SX1278Drv_RegFskPreambleLsb					0x26
#define SX1278Drv_RegFskSyncConfig					0x27
#define SX1278Drv_RegFskSyncValue1					0x28
#define SX1278Drv_RegFskSyncValue2					0x29
#define SX1278Drv_RegFskSyncValue3					0x2A
#define SX1278Drv_RegFskSyncValue4					0x2B
#define SX1278Drv_RegFskSyncValue5					0x2C
#define SX1278Drv_RegFskSyncValue6					0x2D
#define SX1278Drv_RegFskSyncValue7					0x2E
#define SX1278Drv_RegFskSyncValue8					0x2F
#define SX1278Drv_RegFskPacketConfig1				0x30
#define SX1278Drv_RegFskPacketConfig2				0x31
#define SX1278Drv_RegFskPayloadLength				0x32
#define SX1278Drv_RegFskNodeAdrs					0x33
#define SX1278Drv_RegFskBroadcastAdrs				0x34
#define SX1278Drv_RegFskFifoThresh					0x35
#define SX1278Drv_RegFskSeqConfig1					0x36
#define SX1278Drv_RegFskSeqConfig2					0x37
#define SX1278Drv_RegFskTimerResol					0x38
#define SX1278Drv_RegFskTimer1Coef					0x39
#define SX1278Drv_RegFskTimer2Coef					0x3A
#define SX1278Drv_RegFskImageCal					0x3B
#define SX1278Drv_RegFskTemp						0x3C
#define SX1278Drv_RegFskLowBat						0x3D
#define SX1278Drv_RegFskIrqFlags1					0x3E
#define SX1278Drv_RegFskIrqFlags2					0x3F
#define SX1278Drv_RegFskPllHop						0x44
#define	SX1278Drv_RegFskBitRateFrac					0x5D

enum{
	SX1278Drv_RegPaConfig_PaSelect = 0x80,
	SX1278Drv_RegPaConfig_MaxPowerMask = 0x70,
	SX1278Drv_RegPaConfig_OutputPowerMask = 0x0F
} typedef SX1278Drv_RegPaConfig_;

enum{
	SX1278Drv_RegOcp_OcpOn = 0x20,
	SX1278Drv_RegOcp_OcpTrimMask = 0x1F
} typedef SX1278Drv_RegOcp_;

enum{
	SX1278Drv_RegLna_GainMask = 0xE0,
	SX1278Drv_RegLna_LnaBoostLfOff = 0x00,
	SX1278Drv_RegLna_LnaBoostHfOff = 0x00,
	SX1278Drv_RegLna_LnaBoostHfOn = 0x03
} typedef SX1278Drv_RegLna_;

enum{
	SX1278Drv_RegLoRaIrqFlagsMask_RxTimeout = 0x80,
	SX1278Drv_RegLoRaIrqFlagsMask_RxDone = 0x40,
	SX1278Drv_RegLoRaIrqFlagsMask_PayloadCrcError = 0x20,
	SX1278Drv_RegLoRaIrqFlagsMask_ValidHeader = 0x10,
	SX1278Drv_RegLoRaIrqFlagsMask_TxDone = 0x08,
	SX1278Drv_RegLoRaIrqFlagsMask_CadDone = 0x04,
	SX1278Drv_RegLoRaIrqFlagsMask_FhssChangeChannel = 0x02,
	SX1278Drv_RegLoRaIrqFlagsMask_CadDetected = 0x01
} typedef SX1278Drv_RegLoRaIrqFlagsMask_;

enum{
	SX1278Drv_RegLoRaIrqFlags_RxTimeout = 0x80,
	SX1278Drv_RegLoRaIrqFlags_RxDone = 0x40,
	SX1278Drv_RegLoRaIrqFlags_PayloadCrcError = 0x20,
	SX1278Drv_RegLoRaIrqFlags_ValidHeader = 0x10,
	SX1278Drv_RegLoRaIrqFlags_TxDone = 0x08,
	SX1278Drv_RegLoRaIrqFlags_CadDone = 0x04,
	SX1278Drv_RegLoRaIrqFlags_FhssChangeChannel = 0x02,
	SX1278Drv_RegLoRaIrqFlags_CadDetected = 0x01
} typedef SX1278Drv_RegLoRaIrqFlags_;

enum{
	SX1278Drv_RegLoRaModemStat_RxCodingRateMask = 0xE0,
	SX1278Drv_RegLoRaModemStat_ModemClear = 0x10,
	SX1278Drv_RegLoRaModemStat_HeaderInfoValid = 0x08,
	SX1278Drv_RegLoRaModemStat_RxOnGoing = 0x04,
	SX1278Drv_RegLoRaModemStat_SignalSynchronized = 0x02,
	SX1278Drv_RegLoRaModemStat_SignalDetected = 0x01
} typedef SX1278Drv_RegLoRaModemStat_;

enum{
	SX1278Drv_RegLoRaHopChannel_PllTimeout = 0x80,
	SX1278Drv_RegLoRaHopChannel_CrcOnPayload = 0x40,
	SX1278Drv_RegLoRaHopChannel_FhssPresentChannelMask = 0x3F,
} typedef SX1278Drv_RegLoRaHopChannel_;


enum{
	SX1278Drv_RegLoRaModemConfig1_BW_7_8  = 0x00,
	SX1278Drv_RegLoRaModemConfig1_BW_10_4 = 0x10,
	SX1278Drv_RegLoRaModemConfig1_BW_15_6 = 0x20,
	SX1278Drv_RegLoRaModemConfig1_BW_20_8 = 0x30,
	SX1278Drv_RegLoRaModemConfig1_BW_31_2 = 0x40,
	SX1278Drv_RegLoRaModemConfig1_BW_41_7 = 0x50,
	SX1278Drv_RegLoRaModemConfig1_BW_62_5 = 0x60,
	SX1278Drv_RegLoRaModemConfig1_BW_125  = 0x70,
	SX1278Drv_RegLoRaModemConfig1_BW_250  = 0x80,
	SX1278Drv_RegLoRaModemConfig1_BW_500  = 0x90,
} typedef SX1278Drv_RegLoRaModemConfig1_BW;

static const uint32_t SX1278Drv_RegLoRaModemConfig1_BW_Values[] = {
	7800, 10400, 15600, 20800, 31200, 41700, 62500, 125000, 250000, 500000
};

enum{
	SX1278Drv_RegLoRaModemConfig1_CR_4_5 = 0x02,
	SX1278Drv_RegLoRaModemConfig1_CR_4_6 = 0x04,
	SX1278Drv_RegLoRaModemConfig1_CR_4_7 = 0x06,
	SX1278Drv_RegLoRaModemConfig1_CR_4_8 = 0x08,
} typedef SX1278Drv_RegLoRaModemConfig1_CR;

enum{
	SX1278Drv_RegLoRaModemConfig1_HdrMode_Explicit = 0x00,
	SX1278Drv_RegLoRaModemConfig1_HdrMode_Implicit = 0x01
} typedef SX1278Drv_RegLoRaModemConfig1_HdrMode;

enum{
	SX1278Drv_RegLoRaModemConfig2_SF_6  = 0x60,
	SX1278Drv_RegLoRaModemConfig2_SF_7  = 0x70,
	SX1278Drv_RegLoRaModemConfig2_SF_8  = 0x80,
	SX1278Drv_RegLoRaModemConfig2_SF_9  = 0x90,
	SX1278Drv_RegLoRaModemConfig2_SF_10 = 0xA0,
	SX1278Drv_RegLoRaModemConfig2_SF_11 = 0xB0,
	SX1278Drv_RegLoRaModemConfig2_SF_12 = 0xC0
} typedef SX1278Drv_RegLoRaModemConfig2_SF;

enum{
	SX1278Drv_RegLoRaModemConfig2_PayloadCrc_OFF = 0x00,
	SX1278Drv_RegLoRaModemConfig2_PayloadCrc_ON = 0x04
} typedef SX1278Drv_RegLoRaModemConfig2_PayloadCrc;

enum{
	SX1278Drv_RegLoRaModemConfig3_LowDataRateOptimize = 0x08,
	SX1278Drv_RegLoRaModemConfig3_AgcAutoOn = 0x04,
} typedef SX1278Drv_RegLoRaModemConfig3_;

enum{
	SX1278Drv_RegLoRaDetectOptimize_SF7toSF12 = 0xC3,
	SX1278Drv_RegLoRaDetectOptimize_SF6 = 0xC5,
} typedef SX1278Drv_RegLoRaDetectOptimize_;

enum{
	SX1278Drv_RegLoRaInvertIQ_Normal = 0x27,
	SX1278Drv_RegLoRaInvertIQ_Invert = 0x67,
} typedef SX1278Drv_RegLoRaInvertIQ_;

enum{
	SX1278Drv_RegLoRaDetectThreshold_SF7toSF12 = 0x0A,
	SX1278Drv_RegLoRaDetectThreshold_SF6 = 0x0C,
} typedef SX1278Drv_RegLoRaDetectThreshold_;

enum{
	SX1278Drv_RegOpMode_LoRa		= 0x80,
	SX1278Drv_RegOpMode_LowFreq		= 0x08,
	SX1278Drv_RegOpMode_Sleep		= 0x00,
	SX1278Drv_RegOpMode_Standby		= 0x01,
	SX1278Drv_RegOpMode_FSTX		= 0x02,
	SX1278Drv_RegOpMode_TX			= 0x03,
	SX1278Drv_RegOpMode_FSRX		= 0x04,
	SX1278Drv_RegOpMode_RX_Cont		= 0x05,
	SX1278Drv_RegOpMode_RX_Single	= 0x06,
	SX1278Drv_RegOpMode_CAD			= 0x07,
	SX1278Drv_RegOpMode_ModeMask	= 0x07
} typedef SX1278Drv_RegOpMode_;

struct{
	SX1278Drv_RegLoRaModemConfig1_BW			bw;
	SX1278Drv_RegLoRaModemConfig1_CR			cr;
	SX1278Drv_RegLoRaModemConfig1_HdrMode		hdrMode;
	SX1278Drv_RegLoRaModemConfig2_SF			sf;
	SX1278Drv_RegLoRaModemConfig2_PayloadCrc	crc;
	uint16_t									preambleLength;
	uint8_t										power;
	double										frequency;
	SPI_HandleTypeDef							*spi;
	PinDescription								*spi_css_pin;
	PinDescription								*tx_led;
	PinDescription								*rx_led;
	PinDescription								*tx_en;
	PinDescription								*rx_en;
	bool										sleepInIdle;
} typedef SX1278Drv_LoRaConfiguration;

struct{
	uint16_t address;						//sender/receiver address (depends on whether message is received or transmitted)
	uint8_t payload[LoRaMaxMessageLength];
	uint8_t payloadLength;
} typedef LoRa_Message;

#define SX1278Drv_FrequencyStep 61.03516

int16_t			SX1278Drv_LoRaGetRSSI(void);												//gets RSSI
void 			SX1278Drv_Init(SX1278Drv_LoRaConfiguration *cfg);							//initializes LoRa task, pins, etc. Call in main().
void 			SX1278Drv_Config();															//reconfigs lora using SX1278Drv_LoRaConfiguration given in SX1278Drv_Init
void			SX1278Drv_LoRaRxCallback(LoRa_Message *msg);								//called when valid message received by LoRA
void 			SX1278Drv_LoRaRxError(void);												//called when received mesage with incorrect CRC or from wrong address, network, app.
void 			SX1278Drv_LoRaTxCallback(LoRa_Message *msg);								//called when message transmitted by LoRa
void			SX1278Drv_SendMessage(LoRa_Message *msg);									//transmits message
void			SX1278Drv_SetAdresses(uint8_t first, uint16_t *adresses, uint8_t count);	//initializes valid address table. Call in main().
void			SX1278Drv_Suspend();														//suspends LoRa task. Use in low power apps
void			SX1278Drv_Resume();															//resumes LoRa task. Use in low power apps
bool			SX1278Drv_IsBusy();															//checks if LoRa in Tx/Rx
#endif
