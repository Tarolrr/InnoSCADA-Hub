#include "SX1278Drv.h"

SX1278Drv_LoRaConfiguration *SX1278Drv_LoRaCfg;
uint16_t LoRaAddresses[LoRaAddressCount];
uint8_t LoRaRxData[256];
uint8_t LoRaTxData[256];
LoRa_Message loraMsg;
osThreadId hLoRaServiceTask;
osMutexId hLoRaMutex;
osMessageQId hLoRaSendQueue;
volatile bool pendingSuspend;
volatile bool isBusy;
bool lastCAD = true;
static uint32_t	SX1278Drv_GetSymbolLengthInUs();
static uint8_t SX1278Drv_SPIRead(uint8_t adr);
static void SX1278Drv_SPIWrite(uint8_t adr, uint8_t data);
static void SX1278Drv_SPIBurstRead(uint8_t adr, uint8_t *ptr, uint8_t length);
static void SX1278Drv_SPIBurstWrite(uint8_t adr, uint8_t *ptr, uint8_t length);
bool 			SX1278Drv_LoRaEntryRx(void);
uint8_t 		SX1278Drv_LoRaRxPacket(uint8_t *buf, uint32_t timeoutMs);
uint8_t 		SX1278Drv_LoRaEntryTx(void);
bool 			SX1278Drv_LoRaTxPacket(uint8_t *buf, uint8_t size);
uint16_t		SX1278Drv_GetRandomDelay(uint16_t from, uint16_t to);
bool			SX1278Drv_GetCADResult(void);
uint32_t 		SX1278Drv_GetMessageDurationMs(uint8_t payloadLength);
uint8_t			SX1278Drv_GetOpMode(void);
void 			SX1278Drv_SetOpMode(uint8_t mode);
void			SX1278Drv_LoRaServiceTaskFxn(void const * argument);
void			SX1278Drv_SetFrequency(double frequency);
void 			SX1278Drv_EntryLoRa(void);
void 			SX1278Drv_ClearIrq(void);

bool SX1278Drv_GetCADResult(void){
	while(!(SX1278Drv_SPIRead(SX1278Drv_RegLoRaIrqFlags)&SX1278Drv_RegLoRaIrqFlags_CadDone));
	if(SX1278Drv_SPIRead(SX1278Drv_RegLoRaIrqFlags)&SX1278Drv_RegLoRaIrqFlags_CadDetected){
		SX1278Drv_ClearIrq();
		return true;
	}
	else{
		SX1278Drv_ClearIrq();
		return false;
	}
}

uint32_t SX1278Drv_GetMessageDurationMs(uint8_t payloadLength){
	uint32_t symT = SX1278Drv_GetSymbolLengthInUs()/1000;
	uint8_t lowDataRateOptimize = 0;
	if(symT > 16)
		lowDataRateOptimize = 1;
	float symNum = (8*payloadLength-4*(SX1278Drv_LoRaCfg->sf>>4)+28+16*(SX1278Drv_LoRaCfg->crc==SX1278Drv_RegLoRaModemConfig2_PayloadCrc_ON)-20*(SX1278Drv_LoRaCfg->hdrMode==SX1278Drv_RegLoRaModemConfig1_HdrMode_Implicit))*((SX1278Drv_LoRaCfg->cr>>1)+4) \
																	/(4*((SX1278Drv_LoRaCfg->sf>>4)-2*lowDataRateOptimize));
	if(symNum < 0)
		symNum = 0;
	symNum += SX1278Drv_LoRaCfg->preambleLength + 8.25;
	return symNum*symT;
}

uint8_t SX1278Drv_GetOpMode(void){
    return SX1278Drv_SPIRead(SX1278Drv_RegOpMode);                              		//Standby//Low Frequency Mode
}

void SX1278Drv_SetOpMode(uint8_t mode){
    if(mode>7)
		return;
    SX1278Drv_SPIWrite(SX1278Drv_RegOpMode,(SX1278Drv_GetOpMode()&(~SX1278Drv_RegOpMode_ModeMask))|mode|((SX1278Drv_LoRaCfg->frequency < 525e6)*SX1278Drv_RegOpMode_LowFreq));
}

void SX1278Drv_SetPower(uint8_t power){
	//if(SX1278Drv_SPIRead(SX1278Drv_RegPaConfig)&SX1278Drv_RegPaConfig_PaSelect){
		if(power > 17) power = 17;
		if(power < 2)  power = 2;
		power-=2;
		SX1278Drv_SPIWrite(SX1278Drv_RegPaConfig,SX1278Drv_RegPaConfig_PaSelect|(SX1278Drv_RegPaConfig_OutputPowerMask&power));
	//}
	//else
		;//TODO implement power setting for PaSelect == 0
}

void SX1278Drv_SetFrequency(double frequency){
	if((frequency < 137e6) || (frequency > 1020e6))// || ((frequency > 175e6) && (frequency < 410e6)))
		return;//TODO implement errors
	uint32_t freqInt = frequency / SX1278Drv_FrequencyStep;
	SX1278Drv_SPIWrite(SX1278Drv_RegFrMsb,freqInt>>16);
	SX1278Drv_SPIWrite(SX1278Drv_RegFrMid,freqInt>>8);
	SX1278Drv_SPIWrite(SX1278Drv_RegFrLsb,freqInt);
}

double SX1278Drv_GetFrequency(void){
	return ((SX1278Drv_SPIRead(SX1278Drv_RegFrMsb)<<16)+(SX1278Drv_SPIRead(SX1278Drv_RegFrMid)<<8)+SX1278Drv_SPIRead(SX1278Drv_RegFrLsb))*SX1278Drv_FrequencyStep;
}

int16_t SX1278Drv_GetRSSI(void){
	uint8_t temp;
	temp=SX1278Drv_SPIRead(SX1278Drv_RegLoRaPktRssiValue);
	return (int16_t)temp-164;
}

void SX1278Drv_EntryLoRa(void){
	SX1278Drv_SPIWrite(SX1278Drv_RegOpMode,(0x70&SX1278Drv_GetOpMode())|((SX1278Drv_LoRaCfg->frequency < 525e6)*SX1278Drv_RegOpMode_LowFreq)|SX1278Drv_RegOpMode_LoRa);
}

void SX1278Drv_ClearIrq(void){
	if(SX1278Drv_GetOpMode() & SX1278Drv_RegOpMode_LoRa)
		SX1278Drv_SPIWrite(SX1278Drv_RegLoRaIrqFlags,0xFF);
	else{
		SX1278Drv_SPIWrite(SX1278Drv_RegFskIrqFlags1,0xFF);
		SX1278Drv_SPIWrite(SX1278Drv_RegFskIrqFlags2,0xFF);
	}
}

bool SX1278Drv_LoRaEntryRx(void){
	SX1278Drv_ClearIrq();
	if((SX1278Drv_GetOpMode() & SX1278Drv_RegOpMode_RX_Cont) != SX1278Drv_RegOpMode_RX_Cont)
		SX1278Drv_SetOpMode(SX1278Drv_RegOpMode_RX_Cont);
	uint32_t timer =  HAL_GetTick();
	while(1){
		if(SX1278Drv_SPIRead(SX1278Drv_RegLoRaModemStat) & SX1278Drv_RegLoRaModemStat_RxOnGoing)
			return true;
		if( HAL_GetTick()-timer>=3000)
			return false;
	}
}

uint8_t SX1278Drv_LoRaRxPacket(uint8_t *buf, uint32_t timeoutMs){
	uint8_t IrqFlags;
	uint8_t size;
	uint32_t timer = HAL_GetTick();
	if(SX1278Drv_LoRaCfg->rx_led)
		GPIO_PIN_RESET(SX1278Drv_LoRaCfg->rx_led);
	do{
		IrqFlags = SX1278Drv_SPIRead(SX1278Drv_RegLoRaIrqFlags);
		if(IrqFlags & SX1278Drv_RegLoRaIrqFlags_RxDone){
			if((SX1278Drv_LoRaCfg->crc == SX1278Drv_RegLoRaModemConfig2_PayloadCrc_ON) &&
				(IrqFlags & SX1278Drv_RegLoRaIrqFlags_PayloadCrcError))
				return 0;

			SX1278Drv_SPIWrite(SX1278Drv_RegLoRaFifoAddrPtr,0); //RxBaseAddr -> FiFoAddrPtr
			size = SX1278Drv_SPIRead(SX1278Drv_RegLoRaRxNbBytes);
			SX1278Drv_SPIBurstRead(0x00, buf, size);
			SX1278Drv_ClearIrq();
			if(SX1278Drv_LoRaCfg->rx_led)
				GPIO_PIN_SET(SX1278Drv_LoRaCfg->rx_led);
			return size;
		}
	} while(HAL_GetTick() - timer < timeoutMs);
	SX1278Drv_ClearIrq();
	if(SX1278Drv_LoRaCfg->rx_led)
		GPIO_PIN_SET(SX1278Drv_LoRaCfg->rx_led);
	return 0;
}

uint8_t SX1278Drv_LoRaEntryTx(void){
	SX1278Drv_ClearIrq();
	return 1;
}

bool SX1278Drv_LoRaTxPacket(uint8_t *buf, uint8_t size){
	uint8_t IrqFlags;
	if(SX1278Drv_LoRaCfg->tx_led)
		GPIO_PIN_RESET(SX1278Drv_LoRaCfg->tx_led);
	SX1278Drv_SPIWrite(SX1278Drv_RegLoRaFifoAddrPtr, 0);
	SX1278Drv_SPIWrite(SX1278Drv_RegLoRaPayloadLength,size);
	SX1278Drv_SPIBurstWrite(0x00, buf, size);
	SX1278Drv_SetOpMode(SX1278Drv_RegOpMode_TX);
	while(1){
		IrqFlags = SX1278Drv_SPIRead(SX1278Drv_RegLoRaIrqFlags);
		if(IrqFlags & 0x08){                      //Packet sent
			SX1278Drv_SPIRead(SX1278Drv_RegLoRaIrqFlags);
			SX1278Drv_ClearIrq();
			if(SX1278Drv_LoRaCfg->tx_led)
				GPIO_PIN_SET(SX1278Drv_LoRaCfg->tx_led);
			return true;
		}
	}

}

void SX1278Drv_Init(SX1278Drv_LoRaConfiguration *cfg){

	srand(LoRaAddress);


	memcpy(LoRaTxData,(uint8_t *)GUID,16);
	LoRaTxData[LoRaGUIDSize] = NetworkID;		//Номер сети
	memcpy(LoRaTxData+LoRaGUIDSize+LoRaNetworkIDSize,(uint8_t *)&LoRaAddress,2);

	osThreadDef(LoRaServiceTask, SX1278Drv_LoRaServiceTaskFxn, osPriorityNormal, 0, 512);
	hLoRaServiceTask = osThreadCreate(osThread(LoRaServiceTask), NULL);

	osMutexDef(LoRaMutex);
	hLoRaMutex = osMutexCreate(osMutex(LoRaMutex));

	osMessageQDef(LoRaSendQueue, 16, void *);
	hLoRaSendQueue = osMessageCreate(osMessageQ(LoRaSendQueue), NULL);

	//TODO Проверка соединения с модулем SX
	/*if(!sx1276_7_8_LoRaTest()){
		while(1);
	}*/

	GPIO_InitTypeDef GPIO_InitStruct;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;

	SX1278Drv_LoRaCfg = cfg;//TODO add checks for SPI, etc
	if(!cfg)
		return;
	if(cfg->spi_css_pin){
		GPIO_PIN_SET(SX1278Drv_LoRaCfg->spi_css_pin);
		GPIO_InitStruct.Pin = cfg->spi_css_pin->pin;
		HAL_GPIO_Init(cfg->spi_css_pin->port, &GPIO_InitStruct);
	}
	if(cfg->rx_led){
		GPIO_PIN_SET(SX1278Drv_LoRaCfg->rx_led);
		GPIO_InitStruct.Pin = cfg->rx_led->pin;
		HAL_GPIO_Init(cfg->rx_led->port, &GPIO_InitStruct);
	}
	if(cfg->tx_led){
		GPIO_PIN_SET(SX1278Drv_LoRaCfg->tx_led);
		GPIO_InitStruct.Pin = cfg->tx_led->pin;
		HAL_GPIO_Init(cfg->tx_led->port, &GPIO_InitStruct);
	}
	if(cfg->rx_en){
		GPIO_PIN_SET(SX1278Drv_LoRaCfg->rx_en);
		GPIO_InitStruct.Pin = cfg->rx_en->pin;
		HAL_GPIO_Init(cfg->rx_en->port, &GPIO_InitStruct);
	}
	if(cfg->tx_en){
		GPIO_PIN_RESET(SX1278Drv_LoRaCfg->tx_en);
		GPIO_InitStruct.Pin = cfg->tx_en->pin;
		HAL_GPIO_Init(cfg->tx_en->port, &GPIO_InitStruct);
	}


	pendingSuspend = false;
	isBusy = false;
	lastCAD = true;
}

void SX1278Drv_Config(){
	SX1278Drv_SetOpMode(SX1278Drv_RegOpMode_Sleep);//Change modem mode Must in Sleep mode
	HAL_Delay(15);
	SX1278Drv_EntryLoRa();

	SX1278Drv_SetFrequency(SX1278Drv_LoRaCfg->frequency);
	SX1278Drv_SetPower(SX1278Drv_LoRaCfg->power);
	SX1278Drv_SPIWrite(SX1278Drv_RegOcp,SX1278Drv_RegOcp_OcpOn|(SX1278Drv_RegOcp_OcpTrimMask&0x0B));									//RegOcp,Close Ocp
	SX1278Drv_SPIWrite(SX1278Drv_RegLna,0x03|(SX1278Drv_RegLna_GainMask&0x20));									//RegLNA,High & LNA Enable

	if(SX1278Drv_LoRaCfg->sf == SX1278Drv_RegLoRaModemConfig2_SF_6){
		SX1278Drv_SPIWrite(SX1278Drv_RegLoRaDetectOptimize,SX1278Drv_RegLoRaDetectOptimize_SF6);
		SX1278Drv_SPIWrite(SX1278Drv_RegLoRaDetectionThreshold,SX1278Drv_RegLoRaDetectThreshold_SF6);
		SX1278Drv_LoRaCfg->hdrMode = SX1278Drv_RegLoRaModemConfig1_HdrMode_Implicit;
	}
	else{
		SX1278Drv_SPIWrite(SX1278Drv_RegLoRaDetectOptimize,SX1278Drv_RegLoRaDetectOptimize_SF7toSF12);
		SX1278Drv_SPIWrite(SX1278Drv_RegLoRaDetectionThreshold,SX1278Drv_RegLoRaDetectThreshold_SF7toSF12);
	}
	SX1278Drv_SPIWrite(SX1278Drv_RegLoRaModemConfig1,SX1278Drv_LoRaCfg->bw|SX1278Drv_LoRaCfg->cr|SX1278Drv_LoRaCfg->hdrMode);
	SX1278Drv_SPIWrite(SX1278Drv_RegLoRaModemConfig2,SX1278Drv_LoRaCfg->sf|SX1278Drv_LoRaCfg->crc|0x03);

	SX1278Drv_SPIWrite(SX1278Drv_RegLoRaSymbTimeoutLsb,0xFF);//RegSymbTimeoutLsb Timeout = 0x3FF(Max)
	SX1278Drv_SPIWrite(SX1278Drv_RegLoRaPreambleMsb,0x00);
	SX1278Drv_SPIWrite(SX1278Drv_RegLoRaPreambleLsb,SX1278Drv_LoRaCfg->preambleLength-4);

	if(SX1278Drv_GetSymbolLengthInUs(SX1278Drv_LoRaCfg) > 16000)
		SX1278Drv_SPIWrite(SX1278Drv_RegLoRaModemConfig3,SX1278Drv_RegLoRaModemConfig3_AgcAutoOn|SX1278Drv_RegLoRaModemConfig3_LowDataRateOptimize);
	else
		SX1278Drv_SPIWrite(SX1278Drv_RegLoRaModemConfig3,SX1278Drv_RegLoRaModemConfig3_AgcAutoOn);

	SX1278Drv_SPIWrite(SX1278Drv_RegPaDac,0x84);                              //Normal and Rx
	SX1278Drv_SPIWrite(SX1278Drv_RegLoRaHopPeriod,0xFF);                          //RegHopPeriod NO FHSS
	SX1278Drv_SPIWrite(SX1278Drv_RegLoRaIrqFlagsMask,~(
		SX1278Drv_RegLoRaIrqFlagsMask_RxDone |
		SX1278Drv_RegLoRaIrqFlagsMask_PayloadCrcError |
		SX1278Drv_RegLoRaIrqFlagsMask_TxDone |
		SX1278Drv_RegLoRaIrqFlagsMask_CadDone |
		SX1278Drv_RegLoRaIrqFlagsMask_CadDetected));

	SX1278Drv_SPIWrite(SX1278Drv_RegLoRaFifoTxBaseAddr, 0);
	SX1278Drv_SPIWrite(SX1278Drv_RegLoRaFifoRxBaseAddr, 0);
	SX1278Drv_SetOpMode(SX1278Drv_RegOpMode_Standby);
}

static uint32_t	SX1278Drv_GetSymbolLengthInUs(){
	return (1000000*((uint64_t)1<<(SX1278Drv_LoRaCfg->sf>>4)))/SX1278Drv_RegLoRaModemConfig1_BW_Values[(SX1278Drv_LoRaCfg->bw>>4)];
}

static uint8_t SX1278Drv_SPIRead(uint8_t adr){
	uint8_t res;
	GPIO_PIN_RESET(SX1278Drv_LoRaCfg->spi_css_pin);
	HAL_SPI_Transmit(SX1278Drv_LoRaCfg->spi,&adr,1,-1);
	HAL_SPI_Receive(SX1278Drv_LoRaCfg->spi,&res,1,-1);
	GPIO_PIN_SET(SX1278Drv_LoRaCfg->spi_css_pin);
	return res;
}

static void SX1278Drv_SPIWrite(uint8_t adr, uint8_t data){
	adr |= 0x80;
	GPIO_PIN_RESET(SX1278Drv_LoRaCfg->spi_css_pin);
	HAL_SPI_Transmit(SX1278Drv_LoRaCfg->spi,&adr,1,-1);
	HAL_SPI_Transmit(SX1278Drv_LoRaCfg->spi,&data,1,-1);
	GPIO_PIN_SET(SX1278Drv_LoRaCfg->spi_css_pin);
}

static void SX1278Drv_SPIBurstRead(uint8_t adr, uint8_t *ptr, uint8_t length){
	GPIO_PIN_RESET(SX1278Drv_LoRaCfg->spi_css_pin);
	HAL_SPI_Transmit(SX1278Drv_LoRaCfg->spi,&adr,1,-1);
	HAL_SPI_Receive(SX1278Drv_LoRaCfg->spi,ptr,length,-1);
	GPIO_PIN_SET(SX1278Drv_LoRaCfg->spi_css_pin);
}

static void SX1278Drv_SPIBurstWrite(uint8_t adr, uint8_t *ptr, uint8_t length){
	adr |= 0x80;
	GPIO_PIN_RESET(SX1278Drv_LoRaCfg->spi_css_pin);
	HAL_SPI_Transmit(SX1278Drv_LoRaCfg->spi,&adr,1,-1);
	HAL_SPI_Transmit(SX1278Drv_LoRaCfg->spi,ptr,length,-1);
	GPIO_PIN_SET(SX1278Drv_LoRaCfg->spi_css_pin);
}

/*void SX1278Drv_GetRegDump(UART_HandleTypeDef *huart){
	uint8_t buf[0x6f];
	SX1278Drv_SPIBurstRead(0x01, buf, 0x6f);
	HAL_UART_Transmit(huart,buf,0x6f,-1);
}*/

void SX1278Drv_SendMessage(LoRa_Message *msg){
	LoRa_Message *msg2;
	msg2 = pvPortMalloc(sizeof(LoRa_Message));
	memcpy(msg2, msg, sizeof(LoRa_Message));
	osMessagePut(hLoRaSendQueue,(uint32_t)msg2,0);
}

void SX1278Drv_SetAdresses(uint8_t first, uint16_t *adresses, uint8_t count){
	memcpy(LoRaAddresses+first,adresses,count*2);
}

void SX1278Drv_LoRaServiceTaskFxn(void const * argument){
	SX1278Drv_Config();			//конфигурация лоры

	while(1){

		if(pendingSuspend){
			pendingSuspend = false;
			vTaskSuspend(hLoRaServiceTask);
		}

		if(SX1278Drv_LoRaCfg->sleepInIdle)
			SX1278Drv_Config();

		osStatus s = osMutexWait(hLoRaMutex,0);
		if((s == osOK) || (lastCAD)){
			//HAL_Delay(10);
			if(SX1278Drv_LoRaCfg->tx_en && SX1278Drv_LoRaCfg->rx_en){
				GPIO_PIN_SET(SX1278Drv_LoRaCfg->rx_en);
				GPIO_PIN_RESET(SX1278Drv_LoRaCfg->tx_en);
				HAL_Delay(1);
			}
			SX1278Drv_SetOpMode(SX1278Drv_RegOpMode_CAD);
			if(SX1278Drv_GetCADResult()){

				if(lastCAD)
					continue;
				lastCAD = true;
				isBusy = true;

				SX1278Drv_LoRaEntryRx();
				uint8_t rxSize = 0;
				bool isOk = false;
				do{
					uint32_t maxTimeout = SX1278Drv_GetMessageDurationMs(LoRaMaxMessageLength)+500;
					rxSize = SX1278Drv_LoRaRxPacket(LoRaRxData,maxTimeout);

					if(rxSize < (LoRaGUIDSize + LoRaNetworkIDSize + LoRaAddressSize + LoRaAddressSize))
						continue;

					if(!checkGUID(LoRaRxData))
						continue;

					if(LoRaRxData[LoRaGUIDSize] != NetworkID)
						continue;

					bool wrongAddress = true;
					uint8_t idx;
					uint16_t addressFrom =  *((uint16_t *)(LoRaRxData + LoRaGUIDSize + LoRaNetworkIDSize));

					for(idx = 0; idx < LoRaAddressCount; idx++)
						if(addressFrom == LoRaAddresses[idx]){
							wrongAddress = false;
							break;
						}

					if(wrongAddress)
						continue;

					uint16_t addressTo = *((uint16_t *)(LoRaRxData + LoRaGUIDSize + LoRaNetworkIDSize + LoRaAddressSize));
					if(addressTo != LoRaAddress)
						continue;

					isOk = true;

					loraMsg.address = addressFrom;
					loraMsg.payloadLength = rxSize - LoRaGUIDSize - LoRaNetworkIDSize - LoRaAddressSize - LoRaAddressSize;
					memcpy(loraMsg.payload, LoRaRxData + LoRaGUIDSize + LoRaNetworkIDSize + LoRaAddressSize + LoRaAddressSize, loraMsg.payloadLength);
					SX1278Drv_LoRaRxCallback(&loraMsg);
					osMutexRelease(hLoRaMutex);
					lastCAD = false;
					isBusy = false;
				}while(0);
				if(!isOk)
					SX1278Drv_LoRaRxError();
			}
			else{
				if(lastCAD)
					SX1278Drv_LoRaRxError();
				osMutexRelease(hLoRaMutex);
				lastCAD = false;
				isBusy = false;
			}
			SX1278Drv_SetOpMode(SX1278Drv_RegOpMode_Standby);
		}


		s = osMutexWait(hLoRaMutex,0);
		if(s == osOK){
			osEvent e = osMessageGet(hLoRaSendQueue,0);
			if(e.status == osEventMessage){

				isBusy = true;

				LoRa_Message *msg;
				msg = (LoRa_Message *)e.value.p;

				//message format: [GUID NetworkID addressFrom addressTo payload]
				uint8_t txSize = LoRaGUIDSize + LoRaNetworkIDSize + LoRaAddressSize + LoRaAddressSize + msg->payloadLength;
				memcpy(LoRaTxData + LoRaGUIDSize + LoRaNetworkIDSize + LoRaAddressSize, (uint8_t *)&msg->address, LoRaAddressSize);
				memcpy(LoRaTxData + LoRaGUIDSize + LoRaNetworkIDSize + LoRaAddressSize + LoRaAddressSize, msg->payload, msg->payloadLength);

				//if(SX1278Drv_LoRaCfg->sleepInIdle)
				//	SX1278Drv_Config();

				if(SX1278Drv_LoRaCfg->tx_en && SX1278Drv_LoRaCfg->rx_en){
					GPIO_PIN_SET(SX1278Drv_LoRaCfg->tx_en);
					GPIO_PIN_RESET(SX1278Drv_LoRaCfg->rx_en);
					HAL_Delay(1);
				}

				SX1278Drv_LoRaEntryTx();
				SX1278Drv_LoRaTxPacket(LoRaTxData, txSize);
				SX1278Drv_SetOpMode(SX1278Drv_RegOpMode_Standby);

				SX1278Drv_LoRaTxCallback(msg);

				vPortFree(msg);
			}

			osMutexRelease(hLoRaMutex);

			isBusy = false;
		}

		if(SX1278Drv_LoRaCfg->sleepInIdle)
			SX1278Drv_SetOpMode(SX1278Drv_RegOpMode_Sleep);
	}
}

void SX1278Drv_Suspend(){
	pendingSuspend = true;
}

void SX1278Drv_Resume(){
	vTaskResume(hLoRaServiceTask);
}

bool SX1278Drv_IsBusy(){
	return isBusy;
}

uint16_t SX1278Drv_GetRandomDelay(uint16_t from, uint16_t to){
	return from + rand()%(to-from);
}

__weak void SX1278Drv_LoRaRxCallback(LoRa_Message *msg){
	 UNUSED(msg);
}

__weak void SX1278Drv_LoRaRxError(void){
}

__weak void SX1278Drv_LoRaTxCallback(LoRa_Message *msg){
	 UNUSED(msg);
}
