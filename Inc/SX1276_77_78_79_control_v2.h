/*
 * SX1276_77_78_79_control_v2.h
 *
 *  Created on: Feb 16, 2021
 *      Author: i.mishustin
 */

#ifndef SX1276_77_78_79_SX1276_77_78_79_CONTROL_V2_H_
#define SX1276_77_78_79_SX1276_77_78_79_CONTROL_V2_H_

#include <stdint.h>
#include "SX1276_77_78_79_REGISTERS.h"
#include "main.h"

#define SPI1_CS_ON()								HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET)			//or redefine your pin
#define SPI1_CS_OFF()								HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET)
#define LORA_RST_ON()								HAL_GPIO_WritePin(LORA_RST_GPIO_Port, LORA_RST_Pin, GPIO_PIN_RESET)
#define LORA_RST_OFF()								HAL_GPIO_WritePin(LORA_RST_GPIO_Port, LORA_RST_Pin, GPIO_PIN_SET)
#define DIO_0_STATUS()								HAL_GPIO_ReadPin(DIO_0_GPIO_Port, DIO_0_Pin)

enum OUTPUT_PORT
{
	HF_OUTPUT_PORT		= 0x00,
	LF_OUTPUT_PORT		= 0x01
};

enum LORA_SPREADING_FACTOR
{
	LORA_SF_6			= 0x00,
	LORA_SF_7			= 0x01,
	LORA_SF_8			= 0x02,
	LORA_SF_9			= 0x03,
	LORA_SF_10			= 0x04,
	LORA_SF_11			= 0x05,
	LORA_SF_12			= 0x06
};

enum LORA_BANDWIDTH
{
	LORA_BW_7_8_KHZ		= 0x00,
	LORA_BW_10_4_KHZ	= 0x01,
	LORA_BW_15_6_KHZ	= 0x02,
	LORA_BW_20_8_KHZ	= 0x03,
	LORA_BW_31_25_KHZ	= 0x04,
	LORA_BW_41_7_KHZ	= 0x05,
	LORA_BW_62_5_KHZ	= 0x06,
	LORA_BW_125_KHZ		= 0x07,
	LORA_BW_250_KHZ		= 0x08,
	LORA_BW_500_KHZ		= 0x09
};


enum TXRXMode
{
	RX_MODE = 0x00,
	TX_MODE = 0x01
};

enum MODULATION
{
	FSK_OOK_MODULATION		= 0x00,
	LORA_MODULATION			= 0x01
};

enum SPI_READ_WRITE_STATUS
{
	SPI_READ			= 0x00,
	SPI_WRITE			= 0x01
};



typedef struct RegistersConfig_t
{
	 uint8_t RegOpMode;
 	 uint8_t RegBitrateMsb;
 	 uint8_t RegBitrateLsb;
 	 uint8_t RegFdevMsb;
 	 uint8_t RegFdevLsb;
 	 uint8_t RegFrfMsb;
 	 uint8_t RegFrfMid;
 	 uint8_t RegFrfLsb;
 	 uint8_t RegPaConfig;
 	 uint8_t RegPaRamp;
 	 uint8_t RegOcp;
 	 uint8_t RegLna;
 	 uint8_t RegRxConfig;
 	 uint8_t RegRssiConfig;
 	 uint8_t RegRssiCollision;
 	 uint8_t RegRssiThresh;
 	 uint8_t RegRssiValue;
 	 uint8_t RegRxBw;
 	 uint8_t RegAfcBw;
 	 uint8_t RegOokPeak;
 	 uint8_t RegOokFix;
 	 uint8_t RegOokAvg;
 	 uint8_t RegAfcFei;
 	 uint8_t RegAfcMsb;
 	 uint8_t RegAfcLsb;
 	 uint8_t RegFeiMsb;
 	 uint8_t RegFeiLsb;
 	 uint8_t RegPreambleDetect;
 	 uint8_t RegRxTimeout1;
 	 uint8_t RegRxTimeout2;
 	 uint8_t RegRxTimeout3;
 	 uint8_t RegRxDelay;
 	 uint8_t RegOsc;
 	 uint8_t RegPreambleMsb;
 	 uint8_t RegPreambleLsb;
 	 uint8_t RegSyncConfig;
 	 uint8_t RegSyncValue1;
 	 uint8_t RegSyncValue2;
 	 uint8_t RegSyncValue3;
 	 uint8_t RegSyncValue4;
 	 uint8_t RegSyncValue5;
 	 uint8_t RegSyncValue6;
 	 uint8_t RegSyncValue7;
 	 uint8_t RegSyncValue8;
 	 uint8_t RegPacketConfig1;
 	 uint8_t RegPacketConfig2;
 	 uint8_t RegPayloadLength;
 	 uint8_t RegNodeAdrs;
 	 uint8_t RegBroadcastAdrs;
 	 uint8_t RegFifoThresh;
 	 uint8_t RegSeqConfig1;
 	 uint8_t RegSeqConfig2;
 	 uint8_t RegTimerResol;
 	 uint8_t RegTimer1Coef;
 	 uint8_t RegTimer2Coef;
 	 uint8_t RegImageCal;
 	 uint8_t RegLowBat;
 	 uint8_t RegDioMapping1;
 	 uint8_t RegDioMapping2;
 	 uint8_t RegPllHop;
 	 uint8_t RegTcxo;
 	 uint8_t RegPaDac;
 	 uint8_t RegBitRateFrac;
 	 uint8_t RegAgcRefLf;
 	 uint8_t RegAgcThresh1Lf;
 	 uint8_t RegAgcThresh2Lf;
 	 uint8_t RegAgcThresh3Lf;
 	 uint8_t RegPllLf;

 	 //LoRa Registers
 	 uint8_t LoRaRegFifoAddrPtr;
 	 uint8_t LoRaRegFifoTxBaseAddr;
 	 uint8_t LoRaRegFifoRxBaseAddr;
 	 uint8_t LoRaRegFifoRxCurrentAddr; //read only
 	 uint8_t LoRaRegIrqFlagsMask;
 	 uint8_t LoRaRegIrqFlags; 			//read clear
 	 uint8_t LoRaRegRxNbBytes;			// read only
 	 uint8_t LoRaRegRxHeaderCntValueMsb;
 	 uint8_t LoRaRegRxHeaderCntValueLsb;
 	 uint8_t LoRaRegRxPacketCntValueMsb;
 	 uint8_t LoRaRegRxPacketCntValueLsb;
 	 uint8_t LoRaRegModemStat;            //read only
 	 uint8_t LoRaRegPktSnrValue;		  //read only
 	 uint8_t LoRaPktRssiValue;			  //read only
 	 uint8_t LoRaRssiValue;				  //read only
 	 uint8_t LoRaHopChannel;			  //read only
 	 uint8_t LoRaRegModemConfig1;
 	 uint8_t LoRaRegModemConfig2;
 	 uint8_t LoRaRegSymbTimeoutLsb;
 	 uint8_t LoRaRegPreambleMsb;
 	 uint8_t LoRaRegPreambleLsb;
 	 uint8_t LoRaRegPayloadLength;
 	 uint8_t LoRaRegPayloadMaxLength;
 	 uint8_t LoRaRegHopPeriod;
 	 uint8_t LoRaRegFifoRxBytesAddr;
 	 uint8_t LoRaRegModemConfig3;
 	 uint8_t LoRaPpmCorrection;
 	 uint8_t LoRaRegFeiMsb;				//read only;
 	 uint8_t LoRaRegFeiMid;				//read only;
 	 uint8_t LoRaRegFeiLsb;				//read only;
 	 uint8_t LoRaRegRssiWideband;		//read only;
 	 uint8_t LoRaRegLfFreq2;				//errata
 	 uint8_t LoRaRegLfFreq1;				//errata
 	 uint8_t LoRaRegDetectOptimize;
 	 uint8_t LoRaRegInvertIQ;
 	 uint8_t LoRaRegHighBWOptimize1;	//errata
 	 uint8_t LoRaRegDetectionThreshold;
 	 uint8_t LoRaRegSyncWord;
 	 uint8_t LoRaRegHighBWOptimize2;    //errata
 	 uint8_t LoRaRegInvertIQ2;



} RegistersConfig_t;


typedef struct SX1278_State_t
{
 	 uint8_t PacketPayloadSize;
 	 uint8_t RXTXMode;
 	 uint8_t NodeAddress;
 	 uint8_t BroadcastAddress;
 	 uint8_t LastRSSI;
 	 int8_t LastTemperature;
 	 uint8_t LastIrqFlags1;
 	 uint8_t LastIrqFlags2;
 	 uint8_t spiAsyncStatus;
 	 uint8_t OpMode;
 	 uint8_t modulationType;
 	 float LastRSSIFloat;
 	 float LastTemperatureFloat;
 	 uint8_t LoRaSpreadingFactor;
 	 uint8_t LoRaBandWidth;
 	 uint8_t OutputPort;
 	 uint32_t LoRaTimeOnAir;
 	 uint8_t LoraIrqFlags;
 	 uint8_t LoraFifoRxCurrentAddrLast;
 	 uint8_t LoraRxBytesNbLast;
 	 uint8_t LoraModemStatusLast;
 	 uint8_t LoraPacketSnrLast;
 	 uint8_t LoraPacketRssiLast;
 	 uint8_t LoraRssiLast;
 	 uint8_t LoraPacketReceivingOnGoing;
}SX1278_State_t;

typedef struct SPI_dma_req
{
	uint8_t addr;
	uint8_t writeData[65];
	uint8_t readData[65];
	uint16_t size;
	uint8_t readWriteStatus;
	uint8_t ready;
}SPI_dma_req;


void SPI_dma_request_read_write(SPI_dma_req *req);
void SPI_dma_handler(void);

void SX1276_77_78_79_LORA_Init(uint8_t nodeAddress, uint8_t broadcastAddress, uint8_t payloadSize, float frequency_kHz, uint8_t spreadingFactor);

void SX1276_77_78_79_FSK_Init(uint8_t nodeAddress, uint8_t broadcastAddress, uint8_t payloadSize, float frequency_kHz);
void SX1276_77_78_79_Reset(void);
SX1278_State_t* SX1276_77_78_79_getDeviceState(void);

void SX1276_77_78_79_sendData(uint8_t* data, uint8_t size);
void SX1276_77_78_79_setReceivingBuffer(uint8_t* buffer);
void SX1276_77_78_79_setMode(uint8_t mode);
void SX1276_77_78_79_Delay_ms(uint32_t delay);

//
void SX1276_77_78_79_writeFifo(uint8_t *data, uint8_t size);
void SX1276_77_78_79_readFifo(uint8_t *data, uint8_t size);
void SX1276_77_78_79_writeOpMode(uint8_t value);
void SX1276_77_78_79_readOpMode(void);
void SX1276_77_78_79_readRSSI(void);
void SX1276_77_78_79_writePayloadLength(uint8_t value);
void SX1276_77_78_79_readPayloadLength(void);
void SX1276_77_78_79_writeNodeAndBroadcastAddress(uint8_t nodeAddress, uint8_t broadcastAddress);
void SX1276_77_78_79_readNodeAndBroadcastAddress(void);
void SX1276_77_78_79_writeFifoThreshold(uint8_t value);
void SX1276_77_78_79_readFifoThreshold(void);
void SX1276_77_78_79_writeIrqFlags_1_2(uint8_t flagClear1, uint8_t flagClear2);
void SX1276_77_78_79_readIrqFlags_1_2(void);
void SX1276_77_78_79_readTemperature(void);
void SX1276_77_78_79_writeDIOMapping(uint8_t dioMapping1, uint8_t dioMapping2);
void SX1276_77_78_79_writeFifoPtrAddr_L(uint8_t value);
void SX1276_77_78_79_readFifoPtrAddr_L(void);
void SX1276_77_78_79_readLoRaIrqFlags_L(void);
void SX1276_77_78_79_readFifoRxCurrentAddr_L(void);
void SX1276_77_78_79_readFifoRxBytesNb_L(void);
void SX1276_77_78_79_readModemStatusAndRSSI_L(void);


uint8_t writeToRegisterSync(uint8_t address, uint8_t value);
uint8_t readFromRegisterSync(uint8_t address);

uint8_t writeRegOpMode(uint8_t value);
uint8_t readRegOpMode(void);

uint8_t writeRegBitrateMsb(uint8_t value);
uint8_t readRegBitrateMsb(void);
uint8_t writeRegBitrateLsb(uint8_t value);
uint8_t readRegBitrateLsb(void);
uint8_t writeRegFdevMsb(uint8_t value);
uint8_t readRegFdevMsb(void);
uint8_t writeRegFdevLsb(uint8_t value);
uint8_t readRegFdevLsb(void);
uint8_t writeRegFrfMsb(uint8_t value);
uint8_t readRegFrfMsb(void);
uint8_t writeRegFrfMid(uint8_t value);
uint8_t readRegFrfMid(void);
uint8_t writeRegFrfLsb(uint8_t value);
uint8_t readRegFrfLsb(void);
uint8_t writeRegPaConfig(uint8_t value);
uint8_t readRegPaConfig(void);
uint8_t writeRegPaRamp(uint8_t value);
uint8_t readRegPaRamp(void);
uint8_t writeRegOcp(uint8_t value);
uint8_t readRegOcp(void);
uint8_t writeRegLna(uint8_t value);
uint8_t readRegLna(void);
uint8_t writeRegRxConfig(uint8_t value);
uint8_t readRegRxConfig(void);
uint8_t writeRegRssiConfig(uint8_t value);
uint8_t readRegRssiConfig(void);
uint8_t writeRegRssiCollision(uint8_t value);
uint8_t readRegRssiCollision(void);
uint8_t writeRegRssiThreshold(uint8_t value);
uint8_t readRegRssiThreshold(void);
uint8_t writeRegRssiValue(uint8_t value);
uint8_t readRegRssiValue(void);
uint8_t writeRegRxBw(uint8_t value);
uint8_t readRegRxBw(void);
uint8_t writeRegAfcBw(uint8_t value);
uint8_t readRegAfcBw(void);
uint8_t writeRegOokPeak(uint8_t value);
uint8_t readRegOokPeak(void);
uint8_t writeRegOokFix(uint8_t value);
uint8_t readRegOokFix(void);
uint8_t writeRegOokAvg(uint8_t value);
uint8_t readRegOokAvg(void);
uint8_t writeRegAfcFei(uint8_t value);
uint8_t readRegAfcFei(void);
uint8_t writeRegAfcMsb(uint8_t value);
uint8_t readRegAfcMsb(void);
uint8_t writeRegAfcLsb(uint8_t value);
uint8_t readRegAfcLsb(void);
uint8_t writeRegFeiMsb(uint8_t value);
uint8_t readRegFeiMsb(void);
uint8_t writeRegFeiLsb(uint8_t value);
uint8_t readRegFeiLsb(void);
uint8_t writeRegPreambleDetect(uint8_t value);
uint8_t readRegPreambleDetect(void);
uint8_t writeRegRxTimeout1(uint8_t value);
uint8_t readRegRxTimeout1(void);
uint8_t writeRegRxTimeout2(uint8_t value);
uint8_t readRegRxTimeout2(void);
uint8_t writeRegRxTimeout3(uint8_t value);
uint8_t readRegRxTimeout3(void);
uint8_t writeRegRxDelay(uint8_t value);
uint8_t readRegRxDelay(void);
uint8_t writeRegOsc(uint8_t value);
uint8_t readRegOsc(void);
uint8_t writeRegPreambleMsb(uint8_t value);
uint8_t readRegPreambleMsb(void);
uint8_t writeRegPreambleLsb(uint8_t value);
uint8_t readRegPreambleLsb(void);
uint8_t writeRegSyncConfig(uint8_t value);
uint8_t readRegSyncConfig(void);
uint8_t writeRegSyncValue1(uint8_t value);
uint8_t readRegSyncValue1(void);
uint8_t writeRegSyncValue2(uint8_t value);
uint8_t readRegSyncValue2(void);
uint8_t writeRegSyncValue3(uint8_t value);
uint8_t readRegSyncValue3(void);
uint8_t writeRegSyncValue4(uint8_t value);
uint8_t readRegSyncValue4(void);
uint8_t writeRegSyncValue5(uint8_t value);
uint8_t readRegSyncValue5(void);
uint8_t writeRegSyncValue6(uint8_t value);
uint8_t readRegSyncValue6(void);
uint8_t writeRegSyncValue7(uint8_t value);
uint8_t readRegSyncValue7(void);
uint8_t writeRegSyncValue8(uint8_t value);
uint8_t readRegSyncValue8(void);
uint8_t writeRegPacketConfig1(uint8_t value);
uint8_t readRegPacketConfig1(void);
uint8_t writeRegPacketConfig2(uint8_t value);
uint8_t readRegPacketConfig2(void);
uint8_t writeRegPayloadLength(uint8_t value);
uint8_t readRegPayloadLength(void);
uint8_t writeRegNodeAdrs(uint8_t value);
uint8_t readRegNodeAdrs(void);
uint8_t writeRegBroadcastAdrs(uint8_t value);
uint8_t readRegBroadcastAdrs(void);
uint8_t writeRegFifoThresh(uint8_t value);
uint8_t readRegFifoThresh(void);
uint8_t writeRegSeqConfig1(uint8_t value);
uint8_t readRegSeqConfig1(void);
uint8_t writeRegSeqConfig2(uint8_t value);
uint8_t readRegSeqConfig2(void);
uint8_t writeRegTimerResol(uint8_t value);
uint8_t readRegTimerResol(void);
uint8_t writeRegTimer1Coef(uint8_t value);
uint8_t readRegTimer1Coef(void);
uint8_t writeRegTimer2Coef(uint8_t value);
uint8_t readRegTimer2Coef(void);
uint8_t writeRegImageCal(uint8_t value);
uint8_t readRegImageCal(void);
uint8_t readRegTemp(void);
uint8_t writeRegLowBat(uint8_t value);
uint8_t readRegLowBat(void);
uint8_t writeRegIrqFlags1(uint8_t value); //!!
uint8_t readRegIrqFlags1(void);
uint8_t writeRegIrqFlags2(uint8_t value);
uint8_t readRegIrqFlags2(void);
uint8_t writeRegDioMapping1(uint8_t value);
uint8_t readRegDioMapping1(void);
uint8_t writeRegDioMapping2(uint8_t value);
uint8_t readRegDioMapping2(void);
uint8_t readRegVersion(void);
uint8_t writeRegPllHop(uint8_t value);
uint8_t readRegPllHop(void);
uint8_t writeRegTcxo(uint8_t value);
uint8_t readRegTcxo(void);
uint8_t writeRegPaDac(uint8_t value);
uint8_t readRegPaDac(void);
uint8_t readRegFormerTemp(void);
uint8_t writeRegBitrateFrac(uint8_t value);
uint8_t readRegBitrateFrac(void);
uint8_t writeRegAgcRefLf(uint8_t value);
uint8_t readRegAgcRefLf(void);
uint8_t writeRegAgcThresh1Lf(uint8_t value);
uint8_t readRegAgcThresh1Lf(void);
uint8_t writeRegAgcThresh2Lf(uint8_t value);
uint8_t readRegAgcThresh2Lf(void);
uint8_t writeRegAgcThresh3Lf(uint8_t value);
uint8_t readRegAgcThresh3Lf(void);
uint8_t writeRegPllLf(uint8_t value);
uint8_t readRegPllLf(void);

//lora regs sync read write
uint8_t writeLoRaRegFifoAddrPtr(uint8_t value);
uint8_t readLoRaRegFifoAddrPtr(void);
uint8_t writeLoRaRegFifoTxBaseAddr(uint8_t value);
uint8_t readLoRaRegFifoTxBaseAddr(void);
uint8_t writeLoRaRegFifoRxBaseAddr(uint8_t value);
uint8_t readLoRaRegFifoRxBaseAddr(void);
uint8_t readLoRaRegFifoRxCurrentAddr(void);
uint8_t writeLoRaRegIrqFlagsMask(uint8_t value);
uint8_t readLoRaRegIrqFlagsMask(void);
uint8_t writeLoRaRegModemConfig1(uint8_t value);
uint8_t readLoRaRegModemConfig1(void);
uint8_t writeLoRaRegModemConfig2(uint8_t value);
uint8_t readLoRaRegModemConfig2(void);
uint8_t writeLoRaRegSymbolTimeoutLsb(uint8_t value);
uint8_t readLoRaRegSymbolTimeoutLsb(void);
uint8_t writeLoRaRegPreambleLengthMsb(uint8_t value);
uint8_t readLoRaRegPreambleLengthMsb(void);
uint8_t writeLoRaRegPreambleLengthLsb(uint8_t value);
uint8_t readLoRaRegPreambleLengthLsb(void);
uint8_t writeLoRaRegPayloadLength(uint8_t value);
uint8_t readLoRaRegPayloadLength(void);
uint8_t writeLoRaRegPayloadMaxLength(uint8_t value);
uint8_t readLoRaRegPayloadMaxLength(void);
uint8_t writeLoRaRegFreqHoppingPeriod(uint8_t value);
uint8_t readLoRaRegFreqHoppingPeriod(void);
uint8_t writeLoRaRegModemConfig3(uint8_t value);
uint8_t readLoRaRegModemConfig3(void);
uint8_t writeLoRaRegLfFreq2(uint8_t value);
uint8_t readLoRaRegLfFreq2(void);
uint8_t writeLoRaRegLfFreq1(uint8_t value);
uint8_t readLoRaRegLfFreq1(void);
uint8_t writeLoRaRegDetectOptimize(uint8_t value);
uint8_t readLoRaRegDetectOptimize(void);
uint8_t writeLoRaRegInvertIQ(uint8_t value);
uint8_t readLoRaRegInvertIQ(void);
uint8_t writeLoRaRegHighBWOptimize1(uint8_t value);
uint8_t readLoRaRegHighBWOptimize1(void);
uint8_t writeLoRaRegDetectionThreshold(uint8_t value);
uint8_t readLoRaRegDetectionThreshold(void);
uint8_t writeLoRaRegSyncWord(uint8_t value);
uint8_t readLoRaRegSyncWord(void);
uint8_t writeLoRaRegHighBWOptimize2(uint8_t value);
uint8_t readLoRaRegHighBWOptimize2(void);
uint8_t writeLoRaRegInvertIQ2(uint8_t value);
uint8_t readLoRaRegInvertIQ2(void);

uint32_t calculate_time_on_air_for_LoRa_ms(void);















#endif /* SX1276_77_78_79_SX1276_77_78_79_CONTROL_V2_H_ */
