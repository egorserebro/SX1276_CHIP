//#include "sx1276Regs.h"

#include "main.h"



#include "SX1276_77_78_79_control_v2.h"

#define FIFO_MAX_SIZE					(64)
#define SPI_READ_WRITE_QUEUE_SIZE 		(64)
#define WRITE_OP_MODE_REQ_SIZE			(10)




RegistersConfig_t registersConfig;
RegistersConfig_t registersConfigWriteErrors;
SX1278_State_t deviceState;

static SPI_dma_req *rwqueue[SPI_READ_WRITE_QUEUE_SIZE];
static SPI_dma_req **rwqueue_wrptr = rwqueue;

static int rwQueueCnt = 0;

static uint8_t *fifoReadPtr;

static SPI_dma_req writeFifoReq;
static SPI_dma_req readFifoReq;
static SPI_dma_req writeOpModeReq[WRITE_OP_MODE_REQ_SIZE]; //need for sequence same requests
static uint8_t writeOpModeIndex = 0;
static SPI_dma_req readOpModeReq;
static SPI_dma_req readRSSIReq;
static SPI_dma_req writePayloadLengthReq;
static SPI_dma_req readPayloadLengthReq;
static SPI_dma_req writeNodeAndBroadcastAddressReq;
static SPI_dma_req readNodeAndBroadcastAddressReq;
static SPI_dma_req writeFifoThresholdReq;
static SPI_dma_req readFifoThresholdReq;
static SPI_dma_req writeIrqFlags_1_2Req;
static SPI_dma_req readIrqFlags_1_2Req;
static SPI_dma_req readTemperatureReq;
static SPI_dma_req writeDIOMappingReq;
//LoRa mode requests
static SPI_dma_req writeFifoPtrAddrReq;
static SPI_dma_req readFifoPtrAddrReq;
static SPI_dma_req writeLoRaIrqFlagsReq;
static SPI_dma_req readLoRaIrqFlagsReq;
static SPI_dma_req readFifoRxCurrentAddrReq;
static SPI_dma_req readFifoRxBytesNbReq;
static SPI_dma_req readModemStatusAndRSSIReq;





void SX1276_77_78_79_Reset(void)
{	
	LL_GPIO_ResetOutputPin(Reset_sx1276_GPIO_Port,Reset_sx1276_Pin);
	LL_mDelay(10);
	LL_GPIO_SetOutputPin(Reset_sx1276_GPIO_Port,Reset_sx1276_Pin);
	LL_mDelay(10);
}


void SX1276_77_78_79_LORA_Init(uint8_t nodeAddress, uint8_t broadcastAddress, uint8_t payloadSize, float frequency_kHz, uint8_t spreadingFactor)
{
	//go to sleep mode for config
	writeRegOpMode(MODE_SLEEP_MODE);
	deviceState.modulationType = LORA_MODULATION;
	registersConfig.RegDioMapping1 = L_DIO_0_TX_DONE;
	registersConfig.RegDioMapping2 = L_DIO_4_PLL_LOCK | L_DIO_5_CLK_OUT;

	registersConfigWriteErrors.RegDioMapping1 = writeRegDioMapping1(registersConfig.RegDioMapping1);
	registersConfigWriteErrors.RegDioMapping2 = writeRegDioMapping2(registersConfig.RegDioMapping2);

	float frequency = (uint32_t)(frequency_kHz * 1000.0f);
	uint32_t frequency_div_f_step = (uint32_t)(frequency / 61.03515625f);
	uint8_t frfMsb = (uint8_t)(frequency_div_f_step >> 16);
	uint8_t frfMid = (uint8_t)(frequency_div_f_step >> 8);
	uint8_t frfLsb = (uint8_t)frequency_div_f_step;

	//general settings
	registersConfig.RegFrfMsb = frfMsb; //434 Mhz carrier frequency
	registersConfig.RegFrfMid = frfMid;
	registersConfig.RegFrfLsb = frfLsb;
	registersConfig.RegPaConfig = PA_SELECT_PA_BOOST_PIN | MAX_POWER_MAX | OUTPUT_POWER; //PA_SELECT_PA_BOOST_PIN
	registersConfig.RegPaRamp = MODULATION_SHAPING_NO_SHAPING | PA_RAMP_TIME_UP_DOWN_FSK_40_US;
	registersConfig.RegOcp = OCP_ON_ENABLE | OCP_TRIM_MAX_240_MA;
	registersConfig.RegLna = 0 | LNA_BOOST_LF_DEFAULT | LNA_BOOST_HF_BOOST_ON;

	registersConfigWriteErrors.RegFrfMsb = writeRegFrfMsb(registersConfig.RegFrfMsb);
	registersConfigWriteErrors.RegFrfMid = writeRegFrfMid(registersConfig.RegFrfMid);
	registersConfigWriteErrors.RegFrfLsb = writeRegFrfLsb(registersConfig.RegFrfLsb);
	registersConfigWriteErrors.RegPaConfig = writeRegPaConfig(registersConfig.RegPaConfig);
	registersConfigWriteErrors.RegPaRamp = writeRegPaRamp(registersConfig.RegPaRamp);
	registersConfigWriteErrors.RegOcp = writeRegOcp(registersConfig.RegOcp);
	registersConfigWriteErrors.RegLna = writeRegLna(registersConfig.RegLna);


	registersConfig.RegOpMode = LONG_RANGE_MODE_LORA_MODE | MODE_SLEEP_MODE | LOW_FREQ_MODE_ON_HIGH_FREQ;
	registersConfigWriteErrors.RegOpMode = writeRegOpMode(registersConfig.RegOpMode);
	registersConfigWriteErrors.RegOpMode = writeRegOpMode(registersConfig.RegOpMode); //ACCESS SHARED BIT CHANGE ONLY WHEN  LORA_MODE BIT ALREADY SET IN OP MODE REG

readRegOpMode();

	//fsk settings in lora mode
	registersConfig.RegRxConfig = RESTART_RX_ON_COLLISION_AUTOMATIC_RESTART | AGC_AUTO_ON_LNA_CONTROLED_AGC | AGC_AUTO_ON_LNA_CONTROLED_AGC | RX_TRIGGER_DEFAULT;
	registersConfig.RegRssiConfig = RSSI_OFFSET_DEFAULT | RSSI_SMOOTHING_8_SAMPLES;
	registersConfig.RegRssiCollision = REG_RSSI_COLLISION_DEFAULT;
	registersConfig.RegRssiThresh = REG_RSSI_THRESHOLD_DEFAULT;
	registersConfig.RegRxBw = RX_BW_EXP_DEFAULT | RX_BW_MANT_24;
	registersConfig.RegAfcBw = RX_BW_MANT_AFC_DEFAULT | RX_BW_EXP_AFC_DEFAULT;
	registersConfig.RegOokPeak = BIT_SYNC_ON_ENABLE | OOK_THRESHOLD_TYPE_PEAK_MODE | OOK_PEAK_TRESHOLD_STEP_0_5_DB;
	registersConfig.RegOokFix = REG_OOK_FIXED_THRESHOLD_DEFAULT;
	registersConfig.RegOokAvg = OOK_PEAK_THRESHOLD_DEC_ONCE_PER_CHIP | OOK_AVERAGE_OFFSET_0_DB | OOK_AVERAGE_THRESHOLD_FILTER_CHIP_RATE_4;
	registersConfig.RegAfcFei = AFC_AUTO_CLEAR_ON_DISABLE;
	registersConfig.RegAfcMsb = REG_AFC_MSB_DEFAULT;
	registersConfig.RegAfcLsb = REG_AFC_LSB_DEFAULT;
	registersConfig.RegFeiMsb = REG_FEI_MSB_DEFAULT;
	registersConfig.RegFeiLsb = REG_FEI_LSB_DEFAULT;
	registersConfig.RegPreambleDetect = PREAMBLE_DETECTOR_ON_ENABLE | PREAMBLE_DETECTOR_SIZE_2_BYTE | PREAMBLE_DETECTOR_TOLERANCE_DEFAULT;
	registersConfig.RegRxTimeout1 = REG_RX_TIMEOUT_1_DEFAULT;
	registersConfig.RegRxTimeout2 = REG_RX_TIMEOUT_2_DEFAULT;
	registersConfig.RegRxTimeout3 = REG_RX_TIMEOUT_3_DEFAULT;
	registersConfig.RegRxDelay = REG_RX_DELAY_DEFAULT;
	registersConfig.RegOsc = CLK_OUT_RC;
	registersConfig.RegPreambleMsb = REG_PREAMBLE_MSB_DEFAULT;
	registersConfig.RegPreambleLsb = REG_PREAMBLE_LSB_DEFAULT;
	registersConfig.RegSyncConfig = AUTO_RESTART_RX_MODE_ON_W_PLL_RL | PREAMBLE_POLARITY_0xAA | SYNC_ON | SYNC_SIZE_DEFAULT;
	registersConfig.RegSyncValue1 = 0x55;
	registersConfig.RegSyncValue2 = 0x55;
	registersConfig.RegSyncValue3 = 0x55;
	registersConfig.RegSyncValue4 = 0x55;
	registersConfig.RegSyncValue5 = 0x55;
	registersConfig.RegSyncValue6 = 0x55;
	registersConfig.RegSyncValue7 = 0x55;
	registersConfig.RegSyncValue8 = 0x55;
	registersConfig.RegPacketConfig1 = PACKET_FORMAT_FIXED_LENGTH | DC_FREE_NONE | CRC_ON | CRC_AUTO_CLEAR_OFF_CLEAR_FIFO | ADDRESS_FILTERING_NODE_ADDRESS | CRC_WHITENING_TYPE_CCITT_CRC;
	registersConfig.RegPacketConfig2 = DATA_MODE_PACKET_MODE | IO_HOME_ON_DISABLE | BEACON_ON_DISABLE_DEFAULT | PAYLOAD_LENGTH_DEFAULT;
	registersConfig.RegPayloadLength = payloadSize; //packet size
	registersConfig.RegNodeAdrs = nodeAddress;
	registersConfig.RegBroadcastAdrs = broadcastAddress;
	registersConfig.RegFifoThresh = TX_START_CONDITION_FIFO_NOT_EMPTY | (payloadSize - 1);
	registersConfig.RegSeqConfig1 = IDLE_MODE_STANDBY_MODE | FROM_START_TRANSMIT_STATE | LOW_POWER_SELECTION_IDLE_MODE_STATE | FROM_IDLE_TO_TRANSMIT_STATE | FROM_TRANSMIT_TO_RECEIVE_STATE;
	registersConfig.RegSeqConfig2 = FROM_RECEIVE_TO_PACKET_RECEIVED_PRDY | FROM_RX_TIMEOUT_TO_RECEIVE | FROM_PACKET_RECEIVED_TO_RECEIVE;
	registersConfig.RegTimerResol = TIMER_1_RESOLUTION_TIMER_1_DISABLE | TIMER_2_RESOLUTION_TIMER_2_DISABLE;
	registersConfig.RegTimer1Coef = REG_TIMER_1_COEFFICIENT_DEFAULT;
	registersConfig.RegTimer2Coef = REG_TIMER_2_COEFFICIENT_DEFAULT;
	registersConfig.RegImageCal = AUTO_IMAGE_CAL_ON_ENABLE | TEMPERATURE_THRESHOLD_10_DEGREE | TEMPERATURE_MONITOR_OFF_DISABLE;
	registersConfig.RegLowBat = LOW_BAT_ON_DISABLE | LOW_BAT_TRIM_THRESHOLD_1_835_VOLT;


	registersConfigWriteErrors.RegRxConfig = writeRegRxConfig(registersConfig.RegRxConfig);
	registersConfigWriteErrors.RegRssiConfig = writeRegRssiConfig(registersConfig.RegRssiConfig);
	registersConfigWriteErrors.RegRssiCollision = writeRegRssiCollision(registersConfig.RegRssiCollision);
//	registersConfigWriteErrors.RegRssiThresh = writeRegRssiThreshold(registersConfig.RegRssiThresh);
//	registersConfigWriteErrors.RegRxBw = writeRegRxBw(registersConfig.RegRxBw);
//	registersConfigWriteErrors.RegAfcBw = writeRegAfcBw(registersConfig.RegAfcBw);
//	registersConfigWriteErrors.RegOokPeak = writeRegOokPeak(registersConfig.RegOokPeak);
//	registersConfigWriteErrors.RegOokFix = writeRegOokFix(registersConfig.RegOokFix);
//	registersConfigWriteErrors.RegOokAvg = writeRegOokAvg(registersConfig.RegOokAvg);
//	registersConfigWriteErrors.RegAfcFei = writeRegAfcFei(registersConfig.RegAfcFei);
	registersConfigWriteErrors.RegAfcMsb = writeRegAfcMsb(registersConfig.RegAfcMsb);
	registersConfigWriteErrors.RegAfcLsb = writeRegAfcLsb(registersConfig.RegAfcLsb);
	registersConfigWriteErrors.RegFeiMsb = writeRegFeiMsb(registersConfig.RegFeiMsb);
	registersConfigWriteErrors.RegFeiLsb = writeRegFeiLsb(registersConfig.RegFeiLsb);
	registersConfigWriteErrors.RegPreambleDetect = writeRegPreambleDetect(registersConfig.RegPreambleDetect);
	registersConfigWriteErrors.RegRxTimeout1 = writeRegRxTimeout1(registersConfig.RegRxTimeout1);
	registersConfigWriteErrors.RegRxTimeout2 = writeRegRxTimeout2(registersConfig.RegRxTimeout2);
	registersConfigWriteErrors.RegRxTimeout3 = writeRegRxTimeout3(registersConfig.RegRxTimeout3);
	registersConfigWriteErrors.RegRxDelay = writeRegRxDelay(registersConfig.RegRxDelay);
	registersConfigWriteErrors.RegOsc = writeRegOsc(registersConfig.RegOsc);
	registersConfigWriteErrors.RegPreambleMsb = writeRegPreambleMsb(registersConfig.RegPreambleMsb);
	registersConfigWriteErrors.RegPreambleLsb = writeRegPreambleLsb(registersConfig.RegPreambleLsb);
	registersConfigWriteErrors.RegSyncConfig = writeRegSyncConfig(registersConfig.RegSyncConfig);
	registersConfigWriteErrors.RegSyncValue1 = writeRegSyncValue1(registersConfig.RegSyncValue1);
	registersConfigWriteErrors.RegSyncValue2 = writeRegSyncValue2(registersConfig.RegSyncValue2);
	registersConfigWriteErrors.RegSyncValue3 = writeRegSyncValue3(registersConfig.RegSyncValue3);
	registersConfigWriteErrors.RegSyncValue4 = writeRegSyncValue4(registersConfig.RegSyncValue4);
	registersConfigWriteErrors.RegSyncValue5 = writeRegSyncValue5(registersConfig.RegSyncValue5);
	registersConfigWriteErrors.RegSyncValue6 = writeRegSyncValue6(registersConfig.RegSyncValue6);
	registersConfigWriteErrors.RegSyncValue7 = writeRegSyncValue7(registersConfig.RegSyncValue7);
	registersConfigWriteErrors.RegSyncValue8 = writeRegSyncValue8(registersConfig.RegSyncValue8);
	registersConfigWriteErrors.RegPacketConfig1 = writeRegPacketConfig1(registersConfig.RegPacketConfig1);
	registersConfigWriteErrors.RegPacketConfig2 = writeRegPacketConfig2(registersConfig.RegPacketConfig2);
	registersConfigWriteErrors.RegPayloadLength = writeRegPayloadLength(registersConfig.RegPayloadLength);
	registersConfigWriteErrors.RegNodeAdrs = writeRegNodeAdrs(registersConfig.RegNodeAdrs);
	registersConfigWriteErrors.RegBroadcastAdrs = writeRegBroadcastAdrs(registersConfig.RegBroadcastAdrs);
	registersConfigWriteErrors.RegFifoThresh = writeRegFifoThresh(registersConfig.RegFifoThresh);
	registersConfigWriteErrors.RegSeqConfig1 = writeRegSeqConfig1(registersConfig.RegSeqConfig1);
	registersConfigWriteErrors.RegSeqConfig2 = writeRegSeqConfig2(registersConfig.RegSeqConfig2);
	registersConfigWriteErrors.RegTimerResol = writeRegTimerResol(registersConfig.RegTimerResol);
	registersConfigWriteErrors.RegTimer1Coef = writeRegTimer1Coef(registersConfig.RegTimer1Coef);
	registersConfigWriteErrors.RegTimer2Coef = writeRegTimer2Coef(registersConfig.RegTimer2Coef);
	registersConfigWriteErrors.RegImageCal = writeRegImageCal(registersConfig.RegImageCal);
	registersConfigWriteErrors.RegLowBat = writeRegLowBat(registersConfig.RegLowBat);

	deviceState.NodeAddress = registersConfig.RegNodeAdrs;
	deviceState.BroadcastAddress = registersConfig.RegBroadcastAdrs;
	deviceState.PacketPayloadSize = registersConfig.RegPayloadLength;

	registersConfig.RegOpMode = LONG_RANGE_MODE_LORA_MODE | MODE_SLEEP_MODE | LOW_FREQ_MODE_ON_HIGH_FREQ;
	registersConfigWriteErrors.RegOpMode = writeRegOpMode(registersConfig.RegOpMode);

	//lora settings
	registersConfig.LoRaRegFifoAddrPtr = FIFO_ADDR_PTR_LORA_DEFAULT;
	registersConfig.LoRaRegFifoTxBaseAddr = FIFO_TX_BASE_ADDR_LORA_DEFAULT;
	registersConfig.LoRaRegFifoRxBaseAddr = FIFO_RX_BASE_ADDR_LORA_DEFAULT;
	registersConfig.LoRaRegIrqFlagsMask = L_RX_TIMEOUT_MASK | L_FHSS_CHANGE_CHANNEL_MASK | L_CAD_DETECTED_MASK; //setting this bit deactivate specific IRQ
	registersConfig.LoRaRegModemConfig1 = L_BW_125_KHZ | L_CODING_RATE_4_5 | L_IMPLICIT_HEADER_MODE_ON_EXPLICIT;
	registersConfig.LoRaRegModemConfig2 = spreadingFactor | L_TX_CONTINUOUS_MODE_NORMAL_MODE | L_RX_PAYLOAD_CRC_ON_ENABLE;
	registersConfig.LoRaRegSymbTimeoutLsb = SYMBOL_TIMEOUT_LSB_LORA_DEFAULT;
	registersConfig.LoRaRegPreambleMsb = PREAMBLE_LENGTH_MSB_LORA_DEFAULT;
	registersConfig.LoRaRegPreambleLsb = PREAMBLE_LENGTH_LSB_LORA_DEFAULT;
	registersConfig.LoRaRegPayloadLength = payloadSize;
	registersConfig.LoRaRegPayloadMaxLength = PAYLOAD_MAX_LENGTH_LORA_DEFAULT;
	registersConfig.LoRaRegHopPeriod = FREQ_HOPPING_PERIOD_LORA_DEFAULT;
	registersConfig.LoRaRegModemConfig3 = L_LOW_DATA_RATE_OPTIMIZE_DISABLE | L_AGC_AUTO_ON_LNA_AGC_LOOP;


	registersConfigWriteErrors.LoRaRegFifoAddrPtr = writeLoRaRegFifoAddrPtr(registersConfig.LoRaRegFifoAddrPtr);
	registersConfigWriteErrors.LoRaRegFifoTxBaseAddr = writeLoRaRegFifoTxBaseAddr(registersConfig.LoRaRegFifoTxBaseAddr);
	registersConfigWriteErrors.LoRaRegFifoRxBaseAddr = writeLoRaRegFifoRxBaseAddr(registersConfig.LoRaRegFifoRxBaseAddr);
	registersConfigWriteErrors.LoRaRegIrqFlagsMask = writeLoRaRegIrqFlagsMask(registersConfig.LoRaRegIrqFlagsMask);
	registersConfigWriteErrors.LoRaRegModemConfig1 = writeLoRaRegModemConfig1(registersConfig.LoRaRegModemConfig1);
	registersConfigWriteErrors.LoRaRegModemConfig2 = writeLoRaRegModemConfig2(registersConfig.LoRaRegModemConfig2);
	registersConfigWriteErrors.LoRaRegSymbTimeoutLsb = writeLoRaRegSymbolTimeoutLsb(registersConfig.LoRaRegSymbTimeoutLsb);
	registersConfigWriteErrors.LoRaRegPreambleMsb = writeLoRaRegPreambleLengthMsb(registersConfig.LoRaRegPreambleMsb);
	registersConfigWriteErrors.LoRaRegPreambleLsb = writeLoRaRegPreambleLengthLsb(registersConfig.LoRaRegPreambleLsb);
	registersConfigWriteErrors.LoRaRegPayloadLength = writeLoRaRegPayloadLength(registersConfig.LoRaRegPayloadLength);
	registersConfigWriteErrors.LoRaRegPayloadMaxLength = writeLoRaRegPayloadMaxLength(registersConfig.LoRaRegPayloadMaxLength);
	registersConfigWriteErrors.LoRaRegHopPeriod = writeLoRaRegFreqHoppingPeriod(registersConfig.LoRaRegHopPeriod);
	registersConfigWriteErrors.LoRaRegModemConfig3 = writeLoRaRegModemConfig3(registersConfig.LoRaRegModemConfig3);

	registersConfig.LoRaRegDetectOptimize = L_AUTOMATIC_IF_ON_ERRATA | L_DETECT_OPTIMIZE_SF7_SF12;
	registersConfig.LoRaRegLfFreq2 = IF_FREQ_2_ERRATA_FIX_BW_125_KHZ_LORA;
	registersConfig.LoRaRegLfFreq1 = IF_FREQ_1_ERRATA_FIX_BW_125_KHZ_LORA;

	registersConfigWriteErrors.LoRaRegDetectOptimize = writeLoRaRegDetectOptimize(registersConfig.LoRaRegDetectOptimize);
	registersConfigWriteErrors.LoRaRegLfFreq2 = writeLoRaRegLfFreq2(registersConfig.LoRaRegLfFreq2);
	registersConfigWriteErrors.LoRaRegLfFreq1 = writeLoRaRegLfFreq1(registersConfig.LoRaRegLfFreq1);

	registersConfig.LoRaRegInvertIQ = INVERT_IQ_LORA_DEFAULT;
	registersConfig.LoRaRegHighBWOptimize1 = HW_BW_OPTIMIZE_1_LORA_DEFAULT;
	registersConfig.LoRaRegDetectionThreshold = HW_DETECTION_THRESHOLD_LORA_DEFAULT; // SF7-SF12
	registersConfig.LoRaRegSyncWord = SYNC_WORD_LORA_DEFAULT;
	registersConfig.LoRaRegInvertIQ2 = INVERT_IQ_2_LORA_DEFAULT;

	registersConfigWriteErrors.LoRaRegHighBWOptimize1 = writeLoRaRegHighBWOptimize1(registersConfig.LoRaRegHighBWOptimize1);
	registersConfigWriteErrors.LoRaRegDetectionThreshold = writeLoRaRegDetectionThreshold(registersConfig.LoRaRegDetectionThreshold);
	registersConfigWriteErrors.LoRaRegSyncWord = writeLoRaRegSyncWord(registersConfig.LoRaRegSyncWord);

	deviceState.LoRaBandWidth = L_BW_125_KHZ;
	deviceState.LoRaSpreadingFactor = spreadingFactor;
	deviceState.OutputPort = LF_OUTPUT_PORT;
	deviceState.LoRaTimeOnAir = calculate_time_on_air_for_LoRa_ms();
}

void SX1276_77_78_79_FSK_Init(uint8_t nodeAddress, uint8_t broadcastAddress, uint8_t payloadSize, float frequency_kHz)
{
	float frequency = (uint32_t)(frequency_kHz * 1000.0f);
	uint32_t frequency_div_f_step = (uint32_t)(frequency / 61.03515625f);
	uint8_t frfMsb = (uint8_t)(frequency_div_f_step >> 16);
	uint8_t frfMid = (uint8_t)(frequency_div_f_step >> 8);
	uint8_t frfLsb = (uint8_t)frequency_div_f_step;
	//go to sleep mode for config
	writeRegOpMode(MODE_SLEEP_MODE);

	registersConfig.RegOpMode = LONG_RANGE_MODE_FSK_OOK_MODE | MODULATION_TYPE_FSK  | MODE_SLEEP_MODE | LOW_FREQ_MODE_ON_LOW_FREQ;
	registersConfig.RegBitrateMsb = 0x0D; //9.6 kbps
	registersConfig.RegBitrateLsb = 0x05;
	registersConfig.RegFdevMsb = REG_FDEV_MSB_DEFAULT; //frequency deviation 5kHz
	registersConfig.RegFdevLsb = REG_FDEV_LSB_DEFAULT;

	registersConfig.RegFrfMsb = frfMsb; //X Mhz carrier frequency
	registersConfig.RegFrfMid = frfMid;
	registersConfig.RegFrfLsb = frfLsb;

	registersConfig.RegPaConfig = PA_SELECT_PA_BOOST_PIN | MAX_POWER_MAX | OUTPUT_POWER; //PA_SELECT_PA_BOOST_PIN
	registersConfig.RegPaRamp = MODULATION_SHAPING_NO_SHAPING | PA_RAMP_TIME_UP_DOWN_FSK_40_US;
	registersConfig.RegOcp = OCP_ON_ENABLE | OCP_TRIM_MAX_240_MA;
	registersConfig.RegLna = LNA_GAIN_HIGHEST_GAIN_MINUS_6_DBM | LNA_BOOST_LF_DEFAULT | LNA_BOOST_HF_DEFAULT;
	registersConfig.RegRxConfig = RESTART_RX_ON_COLLISION_AUTOMATIC_RESTART | AGC_AUTO_ON_LNA_CONTROLED_AGC | AGC_AUTO_ON_LNA_CONTROLED_AGC | RX_TRIGGER_DEFAULT;
	registersConfig.RegRssiConfig = RSSI_OFFSET_DEFAULT | RSSI_SMOOTHING_8_SAMPLES;
	registersConfig.RegRssiCollision = REG_RSSI_COLLISION_DEFAULT;
	registersConfig.RegRssiThresh = REG_RSSI_THRESHOLD_DEFAULT;
	registersConfig.RegRxBw = RX_BW_EXP_DEFAULT | RX_BW_MANT_24;
	registersConfig.RegAfcBw = RX_BW_MANT_AFC_DEFAULT | RX_BW_EXP_AFC_DEFAULT;
	registersConfig.RegOokPeak = BIT_SYNC_ON_ENABLE | OOK_THRESHOLD_TYPE_PEAK_MODE | OOK_PEAK_TRESHOLD_STEP_0_5_DB;
	registersConfig.RegOokFix = REG_OOK_FIXED_THRESHOLD_DEFAULT;
	registersConfig.RegOokAvg = OOK_PEAK_THRESHOLD_DEC_ONCE_PER_CHIP | OOK_AVERAGE_OFFSET_0_DB | OOK_AVERAGE_THRESHOLD_FILTER_CHIP_RATE_4;
	registersConfig.RegAfcFei = AFC_AUTO_CLEAR_ON_DISABLE;
	registersConfig.RegAfcMsb = REG_AFC_MSB_DEFAULT;
	registersConfig.RegAfcLsb = REG_AFC_LSB_DEFAULT;
	registersConfig.RegFeiMsb = REG_FEI_MSB_DEFAULT;
	registersConfig.RegFeiLsb = REG_FEI_LSB_DEFAULT;
	registersConfig.RegPreambleDetect = PREAMBLE_DETECTOR_ON_ENABLE | PREAMBLE_DETECTOR_SIZE_2_BYTE | PREAMBLE_DETECTOR_TOLERANCE_DEFAULT;
	registersConfig.RegRxTimeout1 = REG_RX_TIMEOUT_1_DEFAULT;
	registersConfig.RegRxTimeout2 = REG_RX_TIMEOUT_2_DEFAULT;
	registersConfig.RegRxTimeout3 = REG_RX_TIMEOUT_3_DEFAULT;
	registersConfig.RegRxDelay = REG_RX_DELAY_DEFAULT;
	registersConfig.RegOsc = CLK_OUT_RC;
	registersConfig.RegPreambleMsb = REG_PREAMBLE_MSB_DEFAULT;
	registersConfig.RegPreambleLsb = REG_PREAMBLE_LSB_DEFAULT;
	registersConfig.RegSyncConfig = AUTO_RESTART_RX_MODE_ON_W_PLL_RL | PREAMBLE_POLARITY_0xAA | SYNC_ON | SYNC_SIZE_DEFAULT;
	registersConfig.RegSyncValue1 = 0x55;
	registersConfig.RegSyncValue2 = 0x55;
	registersConfig.RegSyncValue3 = 0x55;
	registersConfig.RegSyncValue4 = 0x55;
	registersConfig.RegSyncValue5 = 0x55;
	registersConfig.RegSyncValue6 = 0x55;
	registersConfig.RegSyncValue7 = 0x55;
	registersConfig.RegSyncValue8 = 0x55;
	registersConfig.RegPacketConfig1 = PACKET_FORMAT_FIXED_LENGTH | DC_FREE_NONE | CRC_ON | CRC_AUTO_CLEAR_OFF_CLEAR_FIFO | ADDRESS_FILTERING_NODE_ADDRESS | CRC_WHITENING_TYPE_CCITT_CRC;
	registersConfig.RegPacketConfig2 = DATA_MODE_PACKET_MODE | IO_HOME_ON_DISABLE | BEACON_ON_DISABLE_DEFAULT | PAYLOAD_LENGTH_DEFAULT;
	registersConfig.RegPayloadLength = payloadSize; //packet size
	registersConfig.RegNodeAdrs = nodeAddress;
	registersConfig.RegBroadcastAdrs = broadcastAddress;
	registersConfig.RegFifoThresh = TX_START_CONDITION_FIFO_NOT_EMPTY | (payloadSize - 1);
	registersConfig.RegSeqConfig1 = IDLE_MODE_STANDBY_MODE | FROM_START_TRANSMIT_STATE | LOW_POWER_SELECTION_IDLE_MODE_STATE | FROM_IDLE_TO_TRANSMIT_STATE | FROM_TRANSMIT_TO_RECEIVE_STATE;
	registersConfig.RegSeqConfig2 = FROM_RECEIVE_TO_PACKET_RECEIVED_PRDY | FROM_RX_TIMEOUT_TO_RECEIVE | FROM_PACKET_RECEIVED_TO_RECEIVE;
	registersConfig.RegTimerResol = TIMER_1_RESOLUTION_TIMER_1_DISABLE | TIMER_2_RESOLUTION_TIMER_2_DISABLE;
	registersConfig.RegTimer1Coef = REG_TIMER_1_COEFFICIENT_DEFAULT;
	registersConfig.RegTimer2Coef = REG_TIMER_2_COEFFICIENT_DEFAULT;
	registersConfig.RegImageCal = AUTO_IMAGE_CAL_ON_ENABLE | TEMPERATURE_THRESHOLD_10_DEGREE | TEMPERATURE_MONITOR_OFF_DISABLE;
	registersConfig.RegLowBat = LOW_BAT_ON_DISABLE | LOW_BAT_TRIM_THRESHOLD_1_835_VOLT;
	registersConfig.RegDioMapping1 = DIO_0_RX_PAYLOAD_READY_TX_PACKET_SENT | DIO_1_FIFO_EMPTY | DIO_2_RX_READY | DIO_3_TX_READY;
	registersConfig.RegDioMapping2 = DIO_4_RX_RSSI_PREAMBLE_DETECT | DIO_5_PLL_LOCK | MAP_PREAMBLE_DETECT_RSSI_INTERUPT;
	registersConfig.RegPllHop = 0x2D; // Retain default value
	registersConfig.RegTcxo = 0x09; // Retain default value
	registersConfig.RegPaDac = 0x84; //Retain default value
	registersConfig.RegBitRateFrac = REG_BITRATE_FRAC_DEFAULT;
	registersConfig.RegAgcRefLf =  REG_AGC_REFERENCE_DEFAULT;
	registersConfig.RegAgcThresh1Lf = REG_AGC_THRESHOLD_1_DEFAULT;
	registersConfig.RegAgcThresh2Lf = REG_AGC_THRESHOLD_2_DEFAULT;
	registersConfig.RegAgcThresh3Lf = REG_AGC_THRESHOLD_3_DEFAULT;
	registersConfig.RegPllLf = REG_PLL_LF_DEFAULT;

	registersConfigWriteErrors.RegOpMode = writeRegOpMode(registersConfig.RegOpMode); //long range mode changing only in sleep mode
	registersConfigWriteErrors.RegBitrateMsb = writeRegBitrateMsb(registersConfig.RegBitrateMsb);
	registersConfigWriteErrors.RegBitrateLsb = writeRegBitrateLsb(registersConfig.RegBitrateLsb);
	registersConfigWriteErrors.RegFdevMsb = writeRegFdevMsb(registersConfig.RegFdevMsb);
	registersConfigWriteErrors.RegFdevLsb = writeRegFdevLsb(registersConfig.RegFdevLsb);
	registersConfigWriteErrors.RegFrfMsb = writeRegFrfMsb(registersConfig.RegFrfMsb);
	registersConfigWriteErrors.RegFrfMid = writeRegFrfMid(registersConfig.RegFrfMid);
	registersConfigWriteErrors.RegFrfLsb = writeRegFrfLsb(registersConfig.RegFrfLsb);
	registersConfigWriteErrors.RegPaConfig = writeRegPaConfig(registersConfig.RegPaConfig);
	registersConfigWriteErrors.RegPaRamp = writeRegPaRamp(registersConfig.RegPaRamp);
	registersConfigWriteErrors.RegOcp = writeRegOcp(registersConfig.RegOcp);

	registersConfigWriteErrors.RegRxConfig = writeRegRxConfig(registersConfig.RegRxConfig);
	registersConfigWriteErrors.RegLna = writeRegLna(registersConfig.RegLna);
	registersConfigWriteErrors.RegRssiConfig = writeRegRssiConfig(registersConfig.RegRssiConfig);
	registersConfigWriteErrors.RegRssiCollision = writeRegRssiCollision(registersConfig.RegRssiCollision);
	registersConfigWriteErrors.RegRssiThresh = writeRegRssiThreshold(registersConfig.RegRssiThresh);
	registersConfigWriteErrors.RegRxBw = writeRegRxBw(registersConfig.RegRxBw);
	registersConfigWriteErrors.RegAfcBw = writeRegAfcBw(registersConfig.RegAfcBw);
	registersConfigWriteErrors.RegOokPeak = writeRegOokPeak(registersConfig.RegOokPeak);
	registersConfigWriteErrors.RegOokFix = writeRegOokFix(registersConfig.RegOokFix);
	registersConfigWriteErrors.RegOokAvg = writeRegOokAvg(registersConfig.RegOokAvg);
	registersConfigWriteErrors.RegAfcFei = writeRegAfcFei(registersConfig.RegAfcFei);
	registersConfigWriteErrors.RegAfcMsb = writeRegAfcMsb(registersConfig.RegAfcMsb);
	registersConfigWriteErrors.RegAfcLsb = writeRegAfcLsb(registersConfig.RegAfcLsb);
	registersConfigWriteErrors.RegFeiMsb = writeRegFeiMsb(registersConfig.RegFeiMsb);
	registersConfigWriteErrors.RegFeiLsb = writeRegFeiLsb(registersConfig.RegFeiLsb);
	registersConfigWriteErrors.RegPreambleDetect = writeRegPreambleDetect(registersConfig.RegPreambleDetect);
	registersConfigWriteErrors.RegRxTimeout1 = writeRegRxTimeout1(registersConfig.RegRxTimeout1);
	registersConfigWriteErrors.RegRxTimeout2 = writeRegRxTimeout2(registersConfig.RegRxTimeout2);
	registersConfigWriteErrors.RegRxTimeout3 = writeRegRxTimeout3(registersConfig.RegRxTimeout3);
	registersConfigWriteErrors.RegRxDelay = writeRegRxDelay(registersConfig.RegRxDelay);
	registersConfigWriteErrors.RegOsc = writeRegOsc(registersConfig.RegOsc);
	registersConfigWriteErrors.RegPreambleMsb = writeRegPreambleMsb(registersConfig.RegPreambleMsb);
	registersConfigWriteErrors.RegPreambleLsb = writeRegPreambleLsb(registersConfig.RegPreambleLsb);
	registersConfigWriteErrors.RegSyncConfig = writeRegSyncConfig(registersConfig.RegSyncConfig);
	registersConfigWriteErrors.RegSyncValue1 = writeRegSyncValue1(registersConfig.RegSyncValue1);
	registersConfigWriteErrors.RegSyncValue2 = writeRegSyncValue2(registersConfig.RegSyncValue2);
	registersConfigWriteErrors.RegSyncValue3 = writeRegSyncValue3(registersConfig.RegSyncValue3);
	registersConfigWriteErrors.RegSyncValue4 = writeRegSyncValue4(registersConfig.RegSyncValue4);
	registersConfigWriteErrors.RegSyncValue5 = writeRegSyncValue5(registersConfig.RegSyncValue5);
	registersConfigWriteErrors.RegSyncValue6 = writeRegSyncValue6(registersConfig.RegSyncValue6);
	registersConfigWriteErrors.RegSyncValue7 = writeRegSyncValue7(registersConfig.RegSyncValue7);
	registersConfigWriteErrors.RegSyncValue8 = writeRegSyncValue8(registersConfig.RegSyncValue8);
	registersConfigWriteErrors.RegPacketConfig1 = writeRegPacketConfig1(registersConfig.RegPacketConfig1);
	registersConfigWriteErrors.RegPacketConfig2 = writeRegPacketConfig2(registersConfig.RegPacketConfig2);
	registersConfigWriteErrors.RegPayloadLength = writeRegPayloadLength(registersConfig.RegPayloadLength);
	registersConfigWriteErrors.RegNodeAdrs = writeRegNodeAdrs(registersConfig.RegNodeAdrs);
	registersConfigWriteErrors.RegBroadcastAdrs = writeRegBroadcastAdrs(registersConfig.RegBroadcastAdrs);
	registersConfigWriteErrors.RegFifoThresh = writeRegFifoThresh(registersConfig.RegFifoThresh);
	registersConfigWriteErrors.RegSeqConfig1 = writeRegSeqConfig1(registersConfig.RegSeqConfig1);
	registersConfigWriteErrors.RegSeqConfig2 = writeRegSeqConfig2(registersConfig.RegSeqConfig2);
	registersConfigWriteErrors.RegTimerResol = writeRegTimerResol(registersConfig.RegTimerResol);
	registersConfigWriteErrors.RegTimer1Coef = writeRegTimer1Coef(registersConfig.RegTimer1Coef);
	registersConfigWriteErrors.RegTimer2Coef = writeRegTimer2Coef(registersConfig.RegTimer2Coef);
	registersConfigWriteErrors.RegImageCal = writeRegImageCal(registersConfig.RegImageCal);
	registersConfigWriteErrors.RegLowBat = writeRegLowBat(registersConfig.RegLowBat);
	registersConfigWriteErrors.RegDioMapping1 = writeRegDioMapping1(registersConfig.RegDioMapping1);
	registersConfigWriteErrors.RegDioMapping2 = writeRegDioMapping2(registersConfig.RegDioMapping2);
	registersConfigWriteErrors.RegPllHop = writeRegPllHop(registersConfig.RegPllHop);
	registersConfigWriteErrors.RegTcxo = writeRegTcxo(registersConfig.RegTcxo);
	registersConfigWriteErrors.RegPaDac = writeRegPaDac(registersConfig.RegPaDac);
	registersConfigWriteErrors.RegBitRateFrac = writeRegBitrateFrac(registersConfig.RegBitRateFrac);
	registersConfigWriteErrors.RegAgcRefLf = writeRegAgcRefLf(registersConfig.RegAgcRefLf);
	registersConfigWriteErrors.RegAgcThresh1Lf = writeRegAgcThresh1Lf(registersConfig.RegAgcThresh1Lf);
	registersConfigWriteErrors.RegAgcThresh2Lf = writeRegAgcThresh2Lf(registersConfig.RegAgcThresh2Lf);
	registersConfigWriteErrors.RegAgcThresh3Lf = writeRegAgcThresh3Lf(registersConfig.RegAgcThresh3Lf);

	deviceState.NodeAddress = registersConfig.RegNodeAdrs;
	deviceState.BroadcastAddress = registersConfig.RegBroadcastAdrs;
	deviceState.PacketPayloadSize = registersConfig.RegPayloadLength;
	deviceState.modulationType = FSK_OOK_MODULATION;


}

SX1278_State_t* SX1276_77_78_79_getDeviceState(void)
{
	return &deviceState;
}

void SX1276_77_78_79_sendData(uint8_t* data, uint8_t size)
{
	if(deviceState.modulationType == FSK_OOK_MODULATION)
	{
		SX1276_77_78_79_setMode(MODE_STDBY_MODE);
		SX1276_77_78_79_writeFifo(data, size);
		SX1276_77_78_79_setMode(MODE_TX_MODE);
		deviceState.RXTXMode = TX_MODE;
	}
	else if(deviceState.modulationType == LORA_MODULATION)
	{
		SX1276_77_78_79_setMode(MODE_SLEEP_MODE);
		SX1276_77_78_79_writeDIOMapping(L_DIO_0_TX_DONE, L_DIO_4_PLL_LOCK | L_DIO_5_CLK_OUT);
		SX1276_77_78_79_writeFifoPtrAddr_L(registersConfig.LoRaRegFifoTxBaseAddr);
		SX1276_77_78_79_setMode(MODE_STDBY_MODE);
		SX1276_77_78_79_writeFifo(data, size);
		SX1276_77_78_79_setMode(MODE_TX_MODE);
		deviceState.RXTXMode = TX_MODE;

	}
}
void SX1276_77_78_79_setReceivingBuffer(uint8_t* buffer)
{
	fifoReadPtr = buffer;
	if(deviceState.modulationType == FSK_OOK_MODULATION)
	{
		SX1276_77_78_79_setMode(MODE_RX_MODE);
	}
	else if(deviceState.modulationType == LORA_MODULATION)
	{
		SX1276_77_78_79_writeFifoPtrAddr_L(registersConfig.LoRaRegFifoRxBaseAddr);
		SX1276_77_78_79_setMode(L_MODE_RX_CONTINUOUS_MODE);
	}
	deviceState.RXTXMode = RX_MODE;
}
void SX1276_77_78_79_setMode(uint8_t mode)
{
	uint8_t value;
	value = (registersConfig.RegOpMode & 0xF8) | mode;
	SX1276_77_78_79_writeOpMode(value);
}

void SX1276_77_78_79_writeFifo(uint8_t *data, uint8_t size)
{
	uint8_t writeBit = 0x80;
	writeFifoReq.addr = REG_FIFO_ADDR;
	writeFifoReq.readWriteStatus = SPI_WRITE;
	writeFifoReq.writeData[0] = writeBit | writeFifoReq.addr;
	memcpy(writeFifoReq.writeData + 1, data, size);
	writeFifoReq.size = size + 1; //размер запроса + байт адреса
	SPI_dma_request_read_write(&writeFifoReq);
}
void SX1276_77_78_79_readFifo(uint8_t *data, uint8_t size)
{
	readFifoReq.addr = REG_FIFO_ADDR;
	readFifoReq.readWriteStatus = SPI_READ;
	readFifoReq.size = size + 1;
	readFifoReq.writeData[0] = readFifoReq.addr;
	fifoReadPtr = data;
	SPI_dma_request_read_write(&readFifoReq);
}
void SX1276_77_78_79_writeOpMode(uint8_t value)
{
	uint8_t writeBit = 0x80;
	writeOpModeReq[writeOpModeIndex].addr = REG_OP_MODE_ADDR;
	writeOpModeReq[writeOpModeIndex].readWriteStatus = SPI_WRITE;
	writeOpModeReq[writeOpModeIndex].size = 2;
	writeOpModeReq[writeOpModeIndex].writeData[0] = writeBit | writeOpModeReq[writeOpModeIndex].addr;
	writeOpModeReq[writeOpModeIndex].writeData[1] = value;
	SPI_dma_request_read_write(&writeOpModeReq[writeOpModeIndex]);
	//writeOpModeIndex++;
	if(writeOpModeIndex > WRITE_OP_MODE_REQ_SIZE - 1)
	{
		writeOpModeIndex = 0;
	}
}
void SX1276_77_78_79_readOpMode(void)
{
	readOpModeReq.addr = REG_OP_MODE_ADDR;
	readOpModeReq.readWriteStatus = SPI_READ;
	readOpModeReq.size = 2;
	readOpModeReq.writeData[0] = readOpModeReq.addr;
	SPI_dma_request_read_write(&readOpModeReq);
}
void SX1276_77_78_79_readRSSI(void)
{
	readRSSIReq.addr = FSK_OOK_REG_RSSI_VALUE_ADDR;
	readRSSIReq.readWriteStatus = SPI_READ;
	readRSSIReq.size = 2;
	readRSSIReq.writeData[0] = readRSSIReq.addr;
	SPI_dma_request_read_write(&readRSSIReq);
}
void SX1276_77_78_79_writePayloadLength(uint8_t value) //!!!!!!!
{
	uint8_t writeBit = 0x80;
	writePayloadLengthReq.addr = FSK_OOK_REG_PAYLOAD_LENGTH_ADDR;
	writePayloadLengthReq.readWriteStatus = SPI_WRITE;
	writePayloadLengthReq.size = 2;
	writePayloadLengthReq.writeData[0] = writeBit | writePayloadLengthReq.addr;
	writePayloadLengthReq.writeData[1] = value;
	SPI_dma_request_read_write(&writePayloadLengthReq);
}

void SX1276_77_78_79_Delay_ms(uint32_t delay)
{
	//delayTime = HAL_GetTick() + delay; //HAL_GetTick (systick) have to be setted to 1 ms tick
}

uint8_t writeToRegisterSync(uint8_t address, uint8_t value)
{
	
	uint8_t status = 0;//HAL_OK;
	uint8_t writeBit = 0x80;
	uint8_t sendBuf[2] = {0, 0};
	uint8_t temp;
	sendBuf[0] = writeBit | address;
	sendBuf[1] = value;
	LL_GPIO_ResetOutputPin(NSS_SX1276_GPIO_Port, NSS_SX1276_Pin);
	
	while (!(SPI1->SR & SPI_SR_TXE)){};  
	SPI1->DR = sendBuf[0];
	while(SPI1->SR & SPI_SR_BSY){};
	while(!(SPI1->SR & SPI_SR_RXNE)){}; 
	temp=SPI1->DR;
	while (!(SPI1->SR & SPI_SR_TXE)){};	 
	SPI1->DR = sendBuf[1];
	while (SPI1->SR & SPI_SR_BSY){};
	while (!(SPI1->SR & SPI_SR_RXNE)){}; 
	temp = SPI1->DR;	
	LL_GPIO_SetOutputPin(NSS_SX1276_GPIO_Port, NSS_SX1276_Pin);
	LL_mDelay(1);


		

	return status;	





		
}

uint8_t readFromRegisterSync(uint8_t address)
{
	
	//uint8_t status = HAL_OK;
	uint8_t sendBuf[2] = {0, 0};
	uint8_t receiveData[2] = {0, 0};
	uint8_t size;

	sendBuf[0] = address;
	sendBuf[1] = 0x00;


	
	
	LL_GPIO_ResetOutputPin(NSS_SX1276_GPIO_Port, NSS_SX1276_Pin);//CS_LO();	

	
while (!(SPI1->SR & SPI_SR_TXE)){}; 
SPI1->DR = sendBuf[0];
while (SPI1->SR & SPI_SR_BSY){};
while (!(SPI1->SR & SPI_SR_RXNE)){}; 
size = SPI1->DR;
while (!(SPI1->SR & SPI_SR_TXE)){};	 
SPI1->DR = sendBuf[1];
while (SPI1->SR & SPI_SR_BSY){};
while (!(SPI1->SR & SPI_SR_RXNE)){};
receiveData[0] = SPI1->DR;
LL_GPIO_SetOutputPin(NSS_SX1276_GPIO_Port, NSS_SX1276_Pin);//CS_LO();	
LL_mDelay(1);	

 	return receiveData[0];
	

	
	

}
void SX1276_77_78_79_readPayloadLength(void)
{
	readPayloadLengthReq.addr = FSK_OOK_REG_PAYLOAD_LENGTH_ADDR;
	readPayloadLengthReq.readWriteStatus = SPI_READ;
	readPayloadLengthReq.size = 2;
	readPayloadLengthReq.writeData[0] = readPayloadLengthReq.addr;
	SPI_dma_request_read_write(&readPayloadLengthReq);
}
void SX1276_77_78_79_writeNodeAndBroadcastAddress(uint8_t nodeAddress, uint8_t broadcastAddress)
{
	uint8_t writeBit = 0x80;
	writeNodeAndBroadcastAddressReq.addr = FSK_OOK_REG_NODE_ADDRESS_ADDR;
	writeNodeAndBroadcastAddressReq.readWriteStatus = SPI_WRITE;
	writeNodeAndBroadcastAddressReq.size = 3;
	writeNodeAndBroadcastAddressReq.writeData[0] = writeBit | writeNodeAndBroadcastAddressReq.addr;
	writeNodeAndBroadcastAddressReq.writeData[1] = nodeAddress;
	writeNodeAndBroadcastAddressReq.writeData[2] = broadcastAddress;
	SPI_dma_request_read_write(&writeNodeAndBroadcastAddressReq);
}
void SX1276_77_78_79_readNodeAndBroadcastAddress(void)
{
	readNodeAndBroadcastAddressReq.addr = FSK_OOK_REG_NODE_ADDRESS_ADDR;
	readNodeAndBroadcastAddressReq.readWriteStatus = SPI_READ;
	readNodeAndBroadcastAddressReq.size = 3;
	readNodeAndBroadcastAddressReq.writeData[0] = readNodeAndBroadcastAddressReq.addr;
	SPI_dma_request_read_write(&readNodeAndBroadcastAddressReq);
}
void SX1276_77_78_79_writeFifoThreshold(uint8_t value)
{
	uint8_t writeBit = 0x80;
	writeFifoThresholdReq.addr = FSK_OOK_REG_FIFO_THRESHOLD_ADDR;
	writeFifoThresholdReq.readWriteStatus = SPI_WRITE;
	writeFifoThresholdReq.size = 2;
	writeFifoThresholdReq.writeData[0] = writeBit | writeFifoThresholdReq.addr;
	writeFifoThresholdReq.writeData[1] = value;
	SPI_dma_request_read_write(&writeFifoThresholdReq);
}
void SX1276_77_78_79_readFifoThreshold(void) //!!!!!!!
{
	readFifoThresholdReq.addr = FSK_OOK_REG_FIFO_THRESHOLD_ADDR;
	readFifoThresholdReq.readWriteStatus = SPI_READ;
	readFifoThresholdReq.size = 2;
	readFifoThresholdReq.writeData[0] = readFifoThresholdReq.addr;
	SPI_dma_request_read_write(&readFifoThresholdReq);
}
void SX1276_77_78_79_writeIrqFlags_1_2(uint8_t flagClear1, uint8_t flagClear2)
{
	uint8_t writeBit = 0x80;
	writeIrqFlags_1_2Req.addr = FSK_OOK_REG_IRQ_FLAGS_1_ADDR;
	writeIrqFlags_1_2Req.readWriteStatus = SPI_WRITE;
	writeIrqFlags_1_2Req.size = 3;
	writeIrqFlags_1_2Req.writeData[0] = writeBit | writeIrqFlags_1_2Req.addr;
	writeIrqFlags_1_2Req.writeData[1] = flagClear1 & 0x0B; //only 3 bit avalible for clearing
	writeIrqFlags_1_2Req.writeData[2] = flagClear2 & 0x11; //only 2 bit avalible for clearing
	SPI_dma_request_read_write(&writeIrqFlags_1_2Req);
}
void SX1276_77_78_79_readIrqFlags_1_2(void) //read 2 flags in row
{
	readIrqFlags_1_2Req.addr = FSK_OOK_REG_IRQ_FLAGS_1_ADDR;
	readIrqFlags_1_2Req.readWriteStatus = SPI_READ;
	readIrqFlags_1_2Req.size = 3;
	readIrqFlags_1_2Req.writeData[0] = readIrqFlags_1_2Req.addr;
	SPI_dma_request_read_write(&readIrqFlags_1_2Req);
}
void SX1276_77_78_79_readTemperature(void)//not realized, needed to read in rx mode
{
	readTemperatureReq.addr = FSK_OOK_REG_TEMP_ADDR;
	readTemperatureReq.readWriteStatus = SPI_READ;
	readTemperatureReq.size = 2;
	readTemperatureReq.writeData[0] = readTemperatureReq.addr;
	SPI_dma_request_read_write(&readTemperatureReq);
}
void SX1276_77_78_79_writeDIOMapping(uint8_t dioMapping1, uint8_t dioMapping2)
{
	uint8_t writeBit = 0x80;
	writeDIOMappingReq.addr = FSK_OOK_REG_DIO_MAPPING_1_ADDR;
	writeDIOMappingReq.readWriteStatus = SPI_WRITE;
	writeDIOMappingReq.size = 3;
	writeDIOMappingReq.writeData[0] = writeBit | writeDIOMappingReq.addr;
	writeDIOMappingReq.writeData[1] = dioMapping1;
	writeDIOMappingReq.writeData[2] = dioMapping2;
	SPI_dma_request_read_write(&writeDIOMappingReq);
}
void SX1276_77_78_79_writeFifoPtrAddr_L(uint8_t value)
{
	uint8_t writeBit = 0x80;
	writeFifoPtrAddrReq.addr = LORA_REG_FIFO_ADDRESS_PTR_ADDR;
	writeFifoPtrAddrReq.readWriteStatus = SPI_WRITE;
	writeFifoPtrAddrReq.size = 2;
	writeFifoPtrAddrReq.writeData[0] = writeBit | writeFifoPtrAddrReq.addr;
	writeFifoPtrAddrReq.writeData[1] = value;

	SPI_dma_request_read_write(&writeFifoPtrAddrReq);
}

void SX1276_77_78_79_readFifoPtrAddr_L(void)
{
	readFifoPtrAddrReq.addr = LORA_REG_FIFO_ADDRESS_PTR_ADDR;
	readFifoPtrAddrReq.readWriteStatus = SPI_READ;
	readFifoPtrAddrReq.size = 2;
	readFifoPtrAddrReq.writeData[0] = readFifoPtrAddrReq.addr;
	SPI_dma_request_read_write(&readFifoPtrAddrReq);
}
void SX1276_77_78_79_writeLoRaIrqFlags_L(uint8_t value)
{
	uint8_t writeBit = 0x80;
	writeLoRaIrqFlagsReq.addr = LORA_REG_IRQ_FLAGS_ADDR;
	writeLoRaIrqFlagsReq.readWriteStatus = SPI_WRITE;
	writeLoRaIrqFlagsReq.size = 2;
	writeLoRaIrqFlagsReq.writeData[0] = writeBit | writeLoRaIrqFlagsReq.addr;
	writeLoRaIrqFlagsReq.writeData[1] = value;

	SPI_dma_request_read_write(&writeLoRaIrqFlagsReq);
}
void SX1276_77_78_79_readLoRaIrqFlags_L(void)
{
	readLoRaIrqFlagsReq.addr = LORA_REG_IRQ_FLAGS_ADDR;
	readLoRaIrqFlagsReq.readWriteStatus = SPI_READ;
	readLoRaIrqFlagsReq.size = 2;
	readLoRaIrqFlagsReq.writeData[0] = readLoRaIrqFlagsReq.addr;
	SPI_dma_request_read_write(&readLoRaIrqFlagsReq);
}
void SX1276_77_78_79_readFifoRxCurrentAddr_L(void)
{
	readFifoRxCurrentAddrReq.addr = LORA_REG_FIFO_RX_CURRENT_ADDRESS_ADDR;
	readFifoRxCurrentAddrReq.readWriteStatus = SPI_READ;
	readFifoRxCurrentAddrReq.size = 2;
	readFifoRxCurrentAddrReq.writeData[0] = readFifoRxCurrentAddrReq.addr;
	SPI_dma_request_read_write(&readFifoRxCurrentAddrReq);
}
void SX1276_77_78_79_readFifoRxBytesNb_L(void)
{
	readFifoRxBytesNbReq.addr = LORA_REG_RX_NB_BYTES_ADDR;
	readFifoRxBytesNbReq.readWriteStatus = SPI_READ;
	readFifoRxBytesNbReq.size = 2;
	readFifoRxBytesNbReq.writeData[0] = readFifoRxBytesNbReq.addr;
	SPI_dma_request_read_write(&readFifoRxBytesNbReq);
}
void SX1276_77_78_79_readModemStatusAndRSSI_L(void)
{
	readModemStatusAndRSSIReq.addr = LORA_REG_MODEM_STATUS_ADDR;
	readModemStatusAndRSSIReq.readWriteStatus = SPI_READ;
	readModemStatusAndRSSIReq.size = 5; //at one time we will read modem status, packet snr, packet rssi, rssi
	readModemStatusAndRSSIReq.writeData[0] = readModemStatusAndRSSIReq.addr;
	SPI_dma_request_read_write(&readModemStatusAndRSSIReq);
}

__weak void SX1276_77_78_79_writeFifoCallback(void)
{

}
__weak void SX1276_77_78_79_readFifoCallback(void)
{

}
__weak void SX1276_77_78_79_writeOpModeCallback(void)
{

}
__weak void SX1276_77_78_79_readOpModeCallback(void)
{

}
__weak void SX1276_77_78_79_readRSSICallback(void)
{

}
__weak void SX1276_77_78_79_writePayloadLengthCallback(void)
{

}
__weak void SX1276_77_78_79_readPayloadLengthCallback(void)
{

}
__weak void SX1276_77_78_79_writeNodeAndBroadcastAddressCallback(void)
{

}
__weak void SX1276_77_78_79_readNodeAndBroadcastAddressCallback(void)
{

}
__weak void SX1276_77_78_79_writeFifoThresholdCallback(void)
{

}
__weak void SX1276_77_78_79_readFifoThresholdCallback(void)
{

}
__weak void SX1276_77_78_79_writeIrqFlags_1_2Callback(void)
{

}
__weak void SX1276_77_78_79_readIrqFlags_1_2Callback(void)
{
}
__weak void SX1276_77_78_79_readTemperatureCallback(void)
{

}
__weak void SX1276_77_78_79_writeFifoPtrAddr_L_Callback(void)
{

}
__weak void SX1276_77_78_79_readFifoPtrAddr_L_Callback(void)
{

}
__weak void SX1276_77_78_79_writeLoRaIrqFlags_L_Callback(void)
{

}
__weak void SX1276_77_78_79_readLoRaIrqFlags_L_Callback(void)
{

}
__weak void SX1276_77_78_79_readFifoRxCurrentAddr_L_Callback(void)
{

}
__weak void SX1276_77_78_79_readFifoRxBytesNb_L_Callback(void)
{

}
__weak void SX1276_77_78_79_readModemStatusAndRSSI_L_Callback(void)
{

}





uint8_t writeRegOpMode(uint8_t value)
{
	uint8_t received_value = 0;
	writeToRegisterSync(REG_OP_MODE_ADDR, value);
	received_value = readFromRegisterSync(REG_OP_MODE_ADDR);
	if(received_value == value)
	{
		return 0; //success
	}
	return 1; //not success
}
uint8_t readRegOpMode(void)
{
	uint8_t result = 0x00;
	result = readFromRegisterSync(0x18);
	return result;
}

uint8_t writeRegBitrateMsb(uint8_t value)
{
	uint8_t received_value = 0;
	writeToRegisterSync(FSK_OOK_REG_BIT_RATE_MSB_ADDR, value);
	received_value = readFromRegisterSync(FSK_OOK_REG_BIT_RATE_MSB_ADDR);
	if(received_value == value)
	{
		return 0; //success
	}
	return 1; //not success
}
uint8_t readRegBitrateMsb(void)
{
	uint8_t result = 0x00;
	result = readFromRegisterSync(FSK_OOK_REG_BIT_RATE_MSB_ADDR);
	return result;
}
uint8_t writeRegBitrateLsb(uint8_t value)
{
	uint8_t received_value = 0;
	writeToRegisterSync(FSK_OOK_REG_BIT_RATE_LSB_ADDR, value);
	received_value = readFromRegisterSync(FSK_OOK_REG_BIT_RATE_LSB_ADDR);
	if(received_value == value)
	{
		return 0; //success
	}
	return 1; //not success
}
uint8_t readRegBitrateLsb(void)
{
	uint8_t result = 0x00;
	result = readFromRegisterSync(FSK_OOK_REG_BIT_RATE_LSB_ADDR);
	return result;
}
uint8_t writeRegFdevMsb(uint8_t value)
{
	uint8_t received_value = 0;
	writeToRegisterSync(FSK_OOK_REG_F_DEV_MSB_ADDR, value);
	received_value = readFromRegisterSync(FSK_OOK_REG_F_DEV_MSB_ADDR);
	if(received_value == value)
	{
		return 0; //success
	}
	return 1; //not success
}
uint8_t readRegFdevMsb(void)
{
	uint8_t result = 0x00;
	result = readFromRegisterSync(FSK_OOK_REG_F_DEV_MSB_ADDR);
	return result;
}
uint8_t writeRegFdevLsb(uint8_t value)
{
	uint8_t received_value = 0;
	writeToRegisterSync(FSK_OOK_REG_F_DEV_LSB_ADDR, value);
	received_value = readFromRegisterSync(FSK_OOK_REG_F_DEV_LSB_ADDR);
	if(received_value == value)
	{
		return 0; //success
	}
	return 1; //not success
}
uint8_t readRegFdevLsb(void)
{
	uint8_t result = 0x00;
	result = readFromRegisterSync(FSK_OOK_REG_F_DEV_LSB_ADDR);
	return result;
}
uint8_t writeRegFrfMsb(uint8_t value)
{
	uint8_t received_value = 0;
	writeToRegisterSync(FSK_OOK_REG_F_RF_MSB_ADDR, value);
	received_value = readFromRegisterSync(FSK_OOK_REG_F_RF_MSB_ADDR);
	if(received_value == value)
	{
		return 0; //success
	}
	return 1; //not success
}
uint8_t readRegFrfMsb(void)
{
	uint8_t result = 0x00;
	result = readFromRegisterSync(FSK_OOK_REG_F_RF_MSB_ADDR);
	return result;
}
uint8_t writeRegFrfMid(uint8_t value)
{
	uint8_t received_value = 0;
	writeToRegisterSync(FSK_OOK_REG_F_RF_MID_ADDR, value);
	received_value = readFromRegisterSync(FSK_OOK_REG_F_RF_MID_ADDR);
	if(received_value == value)
	{
		return 0; //success
	}
	return 1; //not success
}
uint8_t readRegFrfMid(void)
{
	uint8_t result = 0x00;
	result = readFromRegisterSync(FSK_OOK_REG_F_RF_MID_ADDR);
	return result;
}
uint8_t writeRegFrfLsb(uint8_t value)
{
	uint8_t received_value = 0;
	writeToRegisterSync(FSK_OOK_REG_F_RF_LSB_ADDR, value);
	received_value = readFromRegisterSync(FSK_OOK_REG_F_RF_LSB_ADDR);
	if(received_value == value)
	{
		return 0; //success
	}
	return 1; //not success
}
uint8_t readRegFrfLsb(void)
{
	uint8_t result = 0x00;
	result = readFromRegisterSync(FSK_OOK_REG_F_RF_LSB_ADDR);
	return result;
}
uint8_t writeRegPaConfig(uint8_t value)
{
	uint8_t received_value = 0;
	writeToRegisterSync(FSK_OOK_REG_PA_CONFIG_ADDR, value);
	received_value = readFromRegisterSync(FSK_OOK_REG_PA_CONFIG_ADDR);
	if(received_value == value)
	{
		return 0; //success
	}
	return 1; //not success
}
uint8_t readRegPaConfig(void)
{
	uint8_t result = 0x00;
	result = readFromRegisterSync(FSK_OOK_REG_PA_CONFIG_ADDR);
	return result;
}
uint8_t writeRegPaRamp(uint8_t value)
{
	uint8_t received_value = 0;
	writeToRegisterSync(FSK_OOK_REG_PA_RAMP_ADDR, value);
	received_value = readFromRegisterSync(FSK_OOK_REG_PA_RAMP_ADDR);
	if(received_value == value)
	{
		return 0; //success
	}
	return 1; //not success
}
uint8_t readRegPaRamp(void)
{
	uint8_t result = 0x00;
	result = readFromRegisterSync(FSK_OOK_REG_PA_RAMP_ADDR);
	return result;
}
uint8_t writeRegOcp(uint8_t value)
{
	uint8_t received_value = 0;
	writeToRegisterSync(FSK_OOK_REG_O_C_P_ADDR, value);
	received_value = readFromRegisterSync(FSK_OOK_REG_O_C_P_ADDR);
	if(received_value == value)
	{
		return 0; //success
	}
	return 1; //not success
}
uint8_t readRegOcp(void)
{
	uint8_t result = 0x00;
	result = readFromRegisterSync(FSK_OOK_REG_O_C_P_ADDR);
	return result;
}
uint8_t writeRegLna(uint8_t value)
{
	uint8_t received_value = 0;
	writeToRegisterSync(FSK_OOK_REG_LNA_ADDR, value);
	received_value = readFromRegisterSync(FSK_OOK_REG_LNA_ADDR);
	if(received_value == value)
	{
		return 0; //success
	}
	return 1; //not success
}
uint8_t readRegLna(void)
{
	uint8_t result = 0x00;
	result = readFromRegisterSync(FSK_OOK_REG_LNA_ADDR);
	return result;
}
uint8_t writeRegRxConfig(uint8_t value)
{
	uint8_t received_value = 0;
	writeToRegisterSync(FSK_OOK_REG_RX_CONFIG_ADDR, value);
	received_value = readFromRegisterSync(FSK_OOK_REG_RX_CONFIG_ADDR);
	if(received_value == value)
	{
		return 0; //success
	}
	return 1; //not success
}
uint8_t readRegRxConfig(void)
{
	uint8_t result = 0x00;
	result = readFromRegisterSync(FSK_OOK_REG_RX_CONFIG_ADDR);
	return result;
}
uint8_t writeRegRssiConfig(uint8_t value)
{
	uint8_t received_value = 0;
	writeToRegisterSync(FSK_OOK_REG_RSSI_CONFIG_ADDR, value);
	received_value = readFromRegisterSync(FSK_OOK_REG_RSSI_CONFIG_ADDR);
	if(received_value == value)
	{
		return 0; //success
	}
	return 1; //not success
}
uint8_t readRegRssiConfig(void)
{
	uint8_t result = 0x00;
	result = readFromRegisterSync(FSK_OOK_REG_RSSI_CONFIG_ADDR);
	return result;
}
uint8_t writeRegRssiCollision(uint8_t value)
{
	uint8_t received_value = 0;
	writeToRegisterSync(FSK_OOK_REG_RSSI_COLLISION_ADDR, value);
	received_value = readFromRegisterSync(FSK_OOK_REG_RSSI_COLLISION_ADDR);
	if(received_value == value)
	{
		return 0; //success
	}
	return 1; //not success
}
uint8_t readRegRssiCollision(void)
{
	uint8_t result = 0x00;
	result = readFromRegisterSync(FSK_OOK_REG_RSSI_COLLISION_ADDR);
	return result;
}
uint8_t writeRegRssiThreshold(uint8_t value)
{
	uint8_t received_value = 0;
	writeToRegisterSync(FSK_OOK_REG_RSSI_THRESHOLD_ADDR, value);
	received_value = readFromRegisterSync(FSK_OOK_REG_RSSI_THRESHOLD_ADDR);
	if(received_value == value)
	{
		return 0; //success
	}
	return 1; //not success
}
uint8_t readRegRssiThreshold(void)
{
	uint8_t result = 0x00;
	result = readFromRegisterSync(FSK_OOK_REG_RSSI_THRESHOLD_ADDR);
	return result;
}
uint8_t writeRegRssiValue(uint8_t value) //???
{
	uint8_t received_value = 0;
	writeToRegisterSync(FSK_OOK_REG_RSSI_VALUE_ADDR, value);
	received_value = readFromRegisterSync(FSK_OOK_REG_RSSI_VALUE_ADDR);
	if(received_value == value)
	{
		return 0; //success
	}
	return 1; //not success
}
uint8_t readRegRssiValue(void)
{
	uint8_t result = 0x00;
	result = readFromRegisterSync(FSK_OOK_REG_RSSI_VALUE_ADDR);
	return result;
}
uint8_t writeRegRxBw(uint8_t value)
{
	uint8_t received_value = 0;
	writeToRegisterSync(FSK_OOK_REG_RX_BW_ADDR, value);
	received_value = readFromRegisterSync(FSK_OOK_REG_RX_BW_ADDR);
	if(received_value == value)
	{
		return 0; //success
	}
	return 1; //not success
}
uint8_t readRegRxBw(void)
{
	uint8_t result = 0x00;
	result = readFromRegisterSync(FSK_OOK_REG_RX_BW_ADDR);
	return result;
}
uint8_t writeRegAfcBw(uint8_t value)
{
	uint8_t received_value = 0;
	writeToRegisterSync(FSK_OOK_REG_AFC_BW_ADDR, value);
	received_value = readFromRegisterSync(FSK_OOK_REG_AFC_BW_ADDR);
	if(received_value == value)
	{
		return 0; //success
	}
	return 1; //not success
}
uint8_t readRegAfcBw(void)
{
	uint8_t result = 0x00;
	result = readFromRegisterSync(FSK_OOK_REG_AFC_BW_ADDR);
	return result;
}
uint8_t writeRegOokPeak(uint8_t value)
{
	uint8_t received_value = 0;
	writeToRegisterSync(FSK_OOK_REG_OOK_PEAK_ADDR, value);
	received_value = readFromRegisterSync(FSK_OOK_REG_OOK_PEAK_ADDR);
	if(received_value == value)
	{
		return 0; //success
	}
	return 1; //not success
}
uint8_t readRegOokPeak(void)
{
	uint8_t result = 0x00;
	result = readFromRegisterSync(FSK_OOK_REG_OOK_PEAK_ADDR);
	return result;
}
uint8_t writeRegOokFix(uint8_t value)
{
	uint8_t received_value = 0;
	writeToRegisterSync(FSK_OOK_REG_OOK_FIX_ADDR, value);
	received_value = readFromRegisterSync(FSK_OOK_REG_OOK_FIX_ADDR);
	if(received_value == value)
	{
		return 0; //success
	}
	return 1; //not success
}
uint8_t readRegOokFix(void)
{
	uint8_t result = 0x00;
	result = readFromRegisterSync(FSK_OOK_REG_OOK_FIX_ADDR);
	return result;
}
uint8_t writeRegOokAvg(uint8_t value)
{
	uint8_t received_value = 0;
	writeToRegisterSync(FSK_OOK_REG_OOK_AVG_ADDR, value);
	received_value = readFromRegisterSync(FSK_OOK_REG_OOK_AVG_ADDR);
	if(received_value == value)
	{
		return 0; //success
	}
	return 1; //not success
}
uint8_t readRegOokAvg(void)
{
	uint8_t result = 0x00;
	result = readFromRegisterSync(FSK_OOK_REG_OOK_AVG_ADDR);
	return result;
}
uint8_t writeRegAfcFei(uint8_t value)
{
	uint8_t received_value = 0;
	writeToRegisterSync(FSK_OOK_REG_AFC_FEI_ADDR, value);
	received_value = readFromRegisterSync(FSK_OOK_REG_AFC_FEI_ADDR);
	if(received_value == value)
	{
		return 0; //success
	}
	return 1; //not success
}
uint8_t readRegAfcFei(void)
{
	uint8_t result = 0x00;
	result = readFromRegisterSync(FSK_OOK_REG_AFC_FEI_ADDR);
	return result;
}
uint8_t writeRegAfcMsb(uint8_t value)
{
	uint8_t received_value = 0;
	writeToRegisterSync(FSK_OOK_REG_AFC_MSB_ADDR, value);
	received_value = readFromRegisterSync(FSK_OOK_REG_AFC_MSB_ADDR);
	if(received_value == value)
	{
		return 0; //success
	}
	return 1; //not success
}
uint8_t readRegAfcMsb(void)
{
	uint8_t result = 0x00;
	result = readFromRegisterSync(FSK_OOK_REG_AFC_MSB_ADDR);
	return result;
}
uint8_t writeRegAfcLsb(uint8_t value)
{
	uint8_t received_value = 0;
	writeToRegisterSync(FSK_OOK_REG_AFC_LSB_ADDR, value);
	received_value = readFromRegisterSync(FSK_OOK_REG_AFC_LSB_ADDR);
	if(received_value == value)
	{
		return 0; //success
	}
	return 1; //not success
}
uint8_t readRegAfcLsb(void)
{
	uint8_t result = 0x00;
	result = readFromRegisterSync(FSK_OOK_REG_AFC_LSB_ADDR);
	return result;
}
uint8_t writeRegFeiMsb(uint8_t value)
{
	uint8_t received_value = 0;
	writeToRegisterSync(FSK_OOK_REG_FEI_MSB_ADDR, value);
	received_value = readFromRegisterSync(FSK_OOK_REG_FEI_MSB_ADDR);
	if(received_value == value)
	{
		return 0; //success
	}
	return 1; //not success
}
uint8_t readRegFeiMsb(void)
{
	uint8_t result = 0x00;
	result = readFromRegisterSync(FSK_OOK_REG_FEI_MSB_ADDR);
	return result;
}
uint8_t writeRegFeiLsb(uint8_t value)
{
	uint8_t received_value = 0;
	writeToRegisterSync(FSK_OOK_REG_FEI_LSB_ADDR, value);
	received_value = readFromRegisterSync(FSK_OOK_REG_FEI_LSB_ADDR);
	if(received_value == value)
	{
		return 0; //success
	}
	return 1; //not success
}
uint8_t readRegFeiLsb(void)
{
	uint8_t result = 0x00;
	result = readFromRegisterSync(FSK_OOK_REG_FEI_LSB_ADDR);
	return result;
}
uint8_t writeRegPreambleDetect(uint8_t value)
{
	uint8_t received_value = 0;
	writeToRegisterSync(FSK_OOK_REG_PREAMBLE_DETECTOR_ADDR, value);
	received_value = readFromRegisterSync(FSK_OOK_REG_PREAMBLE_DETECTOR_ADDR);
	if(received_value == value)
	{
		return 0; //success
	}
	return 1; //not success
}
uint8_t readRegPreambleDetect(void)
{
	uint8_t result = 0x00;
	result = readFromRegisterSync(FSK_OOK_REG_PREAMBLE_DETECTOR_ADDR);
	return result;
}
uint8_t writeRegRxTimeout1(uint8_t value)
{
	uint8_t received_value = 0;
	writeToRegisterSync(FSK_OOK_REG_RX_TIMEOUT_1_ADDR, value);
	received_value = readFromRegisterSync(FSK_OOK_REG_RX_TIMEOUT_1_ADDR);
	if(received_value == value)
	{
		return 0; //success
	}
	return 1; //not success
}
uint8_t readRegRxTimeout1(void)
{
	uint8_t result = 0x00;
	result = readFromRegisterSync(FSK_OOK_REG_RX_TIMEOUT_1_ADDR);
	return result;
}
uint8_t writeRegRxTimeout2(uint8_t value)
{
	uint8_t received_value = 0;
	writeToRegisterSync(FSK_OOK_REG_RX_TIMEOUT_2_ADDR, value);
	received_value = readFromRegisterSync(FSK_OOK_REG_RX_TIMEOUT_2_ADDR);
	if(received_value == value)
	{
		return 0; //success
	}
	return 1; //not success
}
uint8_t readRegRxTimeout2(void)
{
	uint8_t result = 0x00;
	result = readFromRegisterSync(FSK_OOK_REG_RX_TIMEOUT_2_ADDR);
	return result;
}
uint8_t writeRegRxTimeout3(uint8_t value)
{
	uint8_t received_value = 0;
	writeToRegisterSync(FSK_OOK_REG_RX_TIMEOUT_3_ADDR, value);
	received_value = readFromRegisterSync(FSK_OOK_REG_RX_TIMEOUT_3_ADDR);
	if(received_value == value)
	{
		return 0; //success
	}
	return 1; //not success
}
uint8_t readRegRxTimeout3(void)
{
	uint8_t result = 0x00;
	result = readFromRegisterSync(FSK_OOK_REG_RX_TIMEOUT_3_ADDR);
	return result;
}
uint8_t writeRegRxDelay(uint8_t value)
{
	uint8_t received_value = 0;
	writeToRegisterSync(FSK_OOK_REG_RX_DELAY_ADDR, value);
	received_value = readFromRegisterSync(FSK_OOK_REG_RX_DELAY_ADDR);
	if(received_value == value)
	{
		return 0; //success
	}
	return 1; //not success
}
uint8_t readRegRxDelay(void)
{
	uint8_t result = 0x00;
	result = readFromRegisterSync(FSK_OOK_REG_RX_DELAY_ADDR);
	return result;
}
uint8_t writeRegOsc(uint8_t value)
{
	uint8_t received_value = 0;
	writeToRegisterSync(FSK_OOK_REG_OSC_ADDR, value);
	received_value = readFromRegisterSync(FSK_OOK_REG_OSC_ADDR);
	if(received_value == value)
	{
		return 0; //success
	}
	return 1; //not success
}
uint8_t readRegOsc(void)
{
	uint8_t result = 0x00;
	result = readFromRegisterSync(FSK_OOK_REG_OSC_ADDR);
	return result;
}
uint8_t writeRegPreambleMsb(uint8_t value)
{
	uint8_t received_value = 0;
	writeToRegisterSync(FSK_OOK_REG_PREAMBLE_MSB_ADDR, value);
	received_value = readFromRegisterSync(FSK_OOK_REG_PREAMBLE_MSB_ADDR);
	if(received_value == value)
	{
		return 0; //success
	}
	return 1; //not success
}
uint8_t readRegPreambleMsb(void)
{
	uint8_t result = 0x00;
	result = readFromRegisterSync(FSK_OOK_REG_PREAMBLE_MSB_ADDR);
	return result;
}
uint8_t writeRegPreambleLsb(uint8_t value)
{
	uint8_t received_value = 0;
	writeToRegisterSync(FSK_OOK_REG_PREAMBLE_LSB_ADDR, value);
	received_value = readFromRegisterSync(FSK_OOK_REG_PREAMBLE_LSB_ADDR);
	if(received_value == value)
	{
		return 0; //success
	}
	return 1; //not success
}
uint8_t readRegPreambleLsb(void)
{
	uint8_t result = 0x00;
	result = readFromRegisterSync(FSK_OOK_REG_PREAMBLE_LSB_ADDR);
	return result;
}
uint8_t writeRegSyncConfig(uint8_t value)
{
	uint8_t received_value = 0;
	writeToRegisterSync(FSK_OOK_REG_SYNC_CONFIG_ADDR, value);
	received_value = readFromRegisterSync(FSK_OOK_REG_SYNC_CONFIG_ADDR);
	if(received_value == value)
	{
		return 0; //success
	}
	return 1; //not success
}
uint8_t readRegSyncConfig(void)
{
	uint8_t result = 0x00;
	result = readFromRegisterSync(FSK_OOK_REG_SYNC_CONFIG_ADDR);
	return result;
}
uint8_t writeRegSyncValue1(uint8_t value)
{
	uint8_t received_value = 0;
	writeToRegisterSync(FSK_OOK_REG_SYNC_VALUE_1_ADDR, value);
	received_value = readFromRegisterSync(FSK_OOK_REG_SYNC_VALUE_1_ADDR);
	if(received_value == value)
	{
		return 0; //success
	}
	return 1; //not success
}
uint8_t readRegSyncValue1(void)
{
	uint8_t result = 0x00;
	result = readFromRegisterSync(FSK_OOK_REG_SYNC_VALUE_1_ADDR);
	return result;
}
uint8_t writeRegSyncValue2(uint8_t value)
{
	uint8_t received_value = 0;
	writeToRegisterSync(FSK_OOK_REG_SYNC_VALUE_2_ADDR, value);
	received_value = readFromRegisterSync(FSK_OOK_REG_SYNC_VALUE_2_ADDR);
	if(received_value == value)
	{
		return 0; //success
	}
	return 1; //not success
}
uint8_t readRegSyncValue2(void)
{
	uint8_t result = 0x00;
	result = readFromRegisterSync(FSK_OOK_REG_SYNC_VALUE_2_ADDR);
	return result;
}
uint8_t writeRegSyncValue3(uint8_t value)
{
	uint8_t received_value = 0;
	writeToRegisterSync(FSK_OOK_REG_SYNC_VALUE_3_ADDR, value);
	received_value = readFromRegisterSync(FSK_OOK_REG_SYNC_VALUE_3_ADDR);
	if(received_value == value)
	{
		return 0; //success
	}
	return 1; //not success
}
uint8_t readRegSyncValue3(void)
{
	uint8_t result = 0x00;
	result = readFromRegisterSync(FSK_OOK_REG_SYNC_VALUE_3_ADDR);
	return result;
}
uint8_t writeRegSyncValue4(uint8_t value)
{
	uint8_t received_value = 0;
	writeToRegisterSync(FSK_OOK_REG_SYNC_VALUE_4_ADDR, value);
	received_value = readFromRegisterSync(FSK_OOK_REG_SYNC_VALUE_4_ADDR);
	if(received_value == value)
	{
		return 0; //success
	}
	return 1; //not success
}
uint8_t readRegSyncValue4(void)
{
	uint8_t result = 0x00;
	result = readFromRegisterSync(FSK_OOK_REG_SYNC_VALUE_4_ADDR);
	return result;
}
uint8_t writeRegSyncValue5(uint8_t value)
{
	uint8_t received_value = 0;
	writeToRegisterSync(FSK_OOK_REG_SYNC_VALUE_5_ADDR, value);
	received_value = readFromRegisterSync(FSK_OOK_REG_SYNC_VALUE_5_ADDR);
	if(received_value == value)
	{
		return 0; //success
	}
	return 1; //not success
}
uint8_t readRegSyncValue5(void)
{
	uint8_t result = 0x00;
	result = readFromRegisterSync(FSK_OOK_REG_SYNC_VALUE_5_ADDR);
	return result;
}
uint8_t writeRegSyncValue6(uint8_t value)
{
	uint8_t received_value = 0;
	writeToRegisterSync(FSK_OOK_REG_SYNC_VALUE_6_ADDR, value);
	received_value = readFromRegisterSync(FSK_OOK_REG_SYNC_VALUE_6_ADDR);
	if(received_value == value)
	{
		return 0; //success
	}
	return 1; //not success
}
uint8_t readRegSyncValue6(void)
{
	uint8_t result = 0x00;
	result = readFromRegisterSync(FSK_OOK_REG_SYNC_VALUE_6_ADDR);
	return result;
}
uint8_t writeRegSyncValue7(uint8_t value)
{
	uint8_t received_value = 0;
	writeToRegisterSync(FSK_OOK_REG_SYNC_VALUE_7_ADDR, value);
	received_value = readFromRegisterSync(FSK_OOK_REG_SYNC_VALUE_7_ADDR);
	if(received_value == value)
	{
		return 0; //success
	}
	return 1; //not success
}
uint8_t readRegSyncValue7(void)
{
	uint8_t result = 0x00;
	result = readFromRegisterSync(FSK_OOK_REG_SYNC_VALUE_7_ADDR);
	return result;
}
uint8_t writeRegSyncValue8(uint8_t value)
{
	uint8_t received_value = 0;
	writeToRegisterSync(FSK_OOK_REG_SYNC_VALUE_8_ADDR, value);
	received_value = readFromRegisterSync(FSK_OOK_REG_SYNC_VALUE_8_ADDR);
	if(received_value == value)
	{
		return 0; //success
	}
	return 1; //not success
}
uint8_t readRegSyncValue8(void)
{
	uint8_t result = 0x00;
	result = readFromRegisterSync(FSK_OOK_REG_SYNC_VALUE_8_ADDR);
	return result;
}
uint8_t writeRegPacketConfig1(uint8_t value)
{
	uint8_t received_value = 0;
	writeToRegisterSync(FSK_OOK_REG_PACKET_CONFIG_1_ADDR, value);
	received_value = readFromRegisterSync(FSK_OOK_REG_PACKET_CONFIG_1_ADDR);
	if(received_value == value)
	{
		return 0; //success
	}
	return 1; //not success
}
uint8_t readRegPacketConfig1(void)
{
	uint8_t result = 0x00;
	result = readFromRegisterSync(FSK_OOK_REG_PACKET_CONFIG_1_ADDR);
	return result;
}
uint8_t writeRegPacketConfig2(uint8_t value)
{
	uint8_t received_value = 0;
	writeToRegisterSync(FSK_OOK_REG_PACKET_CONFIG_2_ADDR, value);
	received_value = readFromRegisterSync(FSK_OOK_REG_PACKET_CONFIG_2_ADDR);
	if(received_value == value)
	{
		return 0; //success
	}
	return 1; //not success
}
uint8_t readRegPacketConfig2(void)
{
	uint8_t result = 0x00;
	result = readFromRegisterSync(FSK_OOK_REG_PACKET_CONFIG_2_ADDR);
	return result;
}
uint8_t writeRegPayloadLength(uint8_t value)
{
	uint8_t received_value = 0;
	writeToRegisterSync(FSK_OOK_REG_PAYLOAD_LENGTH_ADDR, value);
	received_value = readFromRegisterSync(FSK_OOK_REG_PAYLOAD_LENGTH_ADDR);
	if(received_value == value)
	{
		return 0; //success
	}
	return 1; //not success
}
uint8_t readRegPayloadLength(void)
{
	uint8_t result = 0x00;
	result = readFromRegisterSync(FSK_OOK_REG_PAYLOAD_LENGTH_ADDR);
	return result;
}
uint8_t writeRegNodeAdrs(uint8_t value)
{
	uint8_t received_value = 0;
	writeToRegisterSync(FSK_OOK_REG_NODE_ADDRESS_ADDR, value);
	received_value = readFromRegisterSync(FSK_OOK_REG_NODE_ADDRESS_ADDR);
	if(received_value == value)
	{
		return 0; //success
	}
	return 1; //not success
}
uint8_t readRegNodeAdrs(void)
{
	uint8_t result = 0x00;
	result = readFromRegisterSync(FSK_OOK_REG_NODE_ADDRESS_ADDR);
	return result;
}
uint8_t writeRegBroadcastAdrs(uint8_t value)
{
	uint8_t received_value = 0;
	writeToRegisterSync(FSK_OOK_REG_BROADCAST_ADDRESS_ADDR, value);
	received_value = readFromRegisterSync(FSK_OOK_REG_BROADCAST_ADDRESS_ADDR);
	if(received_value == value)
	{
		return 0; //success
	}
	return 1; //not success
}
uint8_t readRegBroadcastAdrs(void)
{
	uint8_t result = 0x00;
	result = readFromRegisterSync(FSK_OOK_REG_BROADCAST_ADDRESS_ADDR);
	return result;
}
uint8_t writeRegFifoThresh(uint8_t value)
{
	uint8_t received_value = 0;
	writeToRegisterSync(FSK_OOK_REG_FIFO_THRESHOLD_ADDR, value);
	received_value = readFromRegisterSync(FSK_OOK_REG_FIFO_THRESHOLD_ADDR);
	if(received_value == value)
	{
		return 0; //success
	}
	return 1; //not success
}
uint8_t readRegFifoThresh(void)
{
	uint8_t result = 0x00;
	result = readFromRegisterSync(FSK_OOK_REG_FIFO_THRESHOLD_ADDR);
	return result;
}
uint8_t writeRegSeqConfig1(uint8_t value)
{
	uint8_t received_value = 0;
	writeToRegisterSync(FSK_OOK_REG_SEQUENCER_CONFIG_1_ADDR, value);
	received_value = readFromRegisterSync(FSK_OOK_REG_SEQUENCER_CONFIG_1_ADDR);
	if(received_value == value)
	{
		return 0; //success
	}
	return 1; //not success
}
uint8_t readRegSeqConfig1(void)
{
	uint8_t result = 0x00;
	result = readFromRegisterSync(FSK_OOK_REG_SEQUENCER_CONFIG_1_ADDR);
	return result;
}
uint8_t writeRegSeqConfig2(uint8_t value)
{
	uint8_t received_value = 0;
	writeToRegisterSync(FSK_OOK_REG_SEQUENCER_CONFIG_2_ADDR, value);
	received_value = readFromRegisterSync(FSK_OOK_REG_SEQUENCER_CONFIG_2_ADDR);
	if(received_value == value)
	{
		return 0; //success
	}
	return 1; //not success
}
uint8_t readRegSeqConfig2(void)
{
	uint8_t result = 0x00;
	result = readFromRegisterSync(FSK_OOK_REG_SEQUENCER_CONFIG_2_ADDR);
	return result;
}
uint8_t writeRegTimerResol(uint8_t value)
{
	uint8_t received_value = 0;
	writeToRegisterSync(FSK_OOK_REG_TIMER_RESOLUTION_ADDR, value);
	received_value = readFromRegisterSync(FSK_OOK_REG_TIMER_RESOLUTION_ADDR);
	if(received_value == value)
	{
		return 0; //success
	}
	return 1; //not success
}
uint8_t readRegTimerResol(void)
{
	uint8_t result = 0x00;
	result = readFromRegisterSync(FSK_OOK_REG_TIMER_RESOLUTION_ADDR);
	return result;
}
uint8_t writeRegTimer1Coef(uint8_t value)
{
	uint8_t received_value = 0;
	writeToRegisterSync(FSK_OOK_REG_TIMER_1_COEF_ADDR, value);
	received_value = readFromRegisterSync(FSK_OOK_REG_TIMER_1_COEF_ADDR);
	if(received_value == value)
	{
		return 0; //success
	}
	return 1; //not success
}
uint8_t readRegTimer1Coef(void)
{
	uint8_t result = 0x00;
	result = readFromRegisterSync(FSK_OOK_REG_TIMER_1_COEF_ADDR);
	return result;
}
uint8_t writeRegTimer2Coef(uint8_t value)
{
	uint8_t received_value = 0;
	writeToRegisterSync(FSK_OOK_REG_TIMER_2_COEF_ADDR, value);
	received_value = readFromRegisterSync(FSK_OOK_REG_TIMER_2_COEF_ADDR);
	if(received_value == value)
	{
		return 0; //success
	}
	return 1; //not success
}
uint8_t readRegTimer2Coef(void)
{
	uint8_t result = 0x00;
	result = readFromRegisterSync(FSK_OOK_REG_TIMER_2_COEF_ADDR);
	return result;
}
uint8_t writeRegImageCal(uint8_t value)
{
	uint8_t received_value = 0;
	writeToRegisterSync(FSK_OOK_REG_IMAGE_CALIBRATION_ADDR, value);
	received_value = readFromRegisterSync(FSK_OOK_REG_IMAGE_CALIBRATION_ADDR);
	if(received_value == value)
	{
		return 0; //success
	}
	return 1; //not success
}
uint8_t readRegImageCal(void)
{
	uint8_t result = 0x00;
	result = readFromRegisterSync(FSK_OOK_REG_IMAGE_CALIBRATION_ADDR);
	return result;
}
uint8_t readRegTemp(void)
{
	uint8_t result = 0x00;
	result = readFromRegisterSync(FSK_OOK_REG_TEMP_ADDR);
	return result;
}
uint8_t writeRegLowBat(uint8_t value)
{
	uint8_t received_value = 0;
	writeToRegisterSync(FSK_OOK_REG_LOW_BAT_ADDR, value);
	received_value = readFromRegisterSync(FSK_OOK_REG_LOW_BAT_ADDR);
	if(received_value == value)
	{
		return 0; //success
	}
	return 1; //not success
}
uint8_t readRegLowBat(void)
{
	uint8_t result = 0x00;
	result = readFromRegisterSync(FSK_OOK_REG_LOW_BAT_ADDR);
	return result;
}
uint8_t writeRegIrqFlags1(uint8_t value)//!!!
{
	uint8_t received_value = 0;
	writeToRegisterSync(FSK_OOK_REG_IRQ_FLAGS_1_ADDR, value & 0x0B); //you can clear only 3 spesific bits via writing 1
	received_value = readFromRegisterSync(FSK_OOK_REG_IRQ_FLAGS_1_ADDR);
	if(received_value == value)
	{
		return 0; //success
	}
	return 1; //not success
}
uint8_t readRegIrqFlags1(void)
{
	uint8_t result = 0x00;
	result = readFromRegisterSync(FSK_OOK_REG_IRQ_FLAGS_1_ADDR);
	return result;
}
uint8_t writeRegIrqFlags2(uint8_t value) //!!!
{
	uint8_t received_value = 0;
	writeToRegisterSync(FSK_OOK_REG_IRQ_FLAGS_2_ADDR, value & 0x11); //you can clear only 2 spesific bits via writing 1
	received_value = readFromRegisterSync(FSK_OOK_REG_IRQ_FLAGS_2_ADDR);
	if(received_value == value)
	{
		return 0; //success
	}
	return 1; //not success
}
uint8_t readRegIrqFlags2(void)
{
	uint8_t result = 0x00;
	result = readFromRegisterSync(FSK_OOK_REG_IRQ_FLAGS_2_ADDR);
	return result;
}
uint8_t writeRegDioMapping1(uint8_t value)
{
	uint8_t received_value = 0;
	writeToRegisterSync(FSK_OOK_REG_DIO_MAPPING_1_ADDR, value);
	received_value = readFromRegisterSync(FSK_OOK_REG_DIO_MAPPING_1_ADDR);
	if(received_value == value)
	{
		return 0; //success
	}
	return 1; //not success
}
uint8_t readRegDioMapping1(void)
{
	uint8_t result = 0x00;
	result = readFromRegisterSync(FSK_OOK_REG_DIO_MAPPING_1_ADDR);
	return result;
}
uint8_t writeRegDioMapping2(uint8_t value)
{
	uint8_t received_value = 0;
	writeToRegisterSync(FSK_OOK_REG_DIO_MAPPING_2_ADDR, value);
	received_value = readFromRegisterSync(FSK_OOK_REG_DIO_MAPPING_2_ADDR);
	if(received_value == value)
	{
		return 0; //success
	}
	return 1; //not success
}
uint8_t readRegDioMapping2(void)
{
	uint8_t result = 0x00;
	result = readFromRegisterSync(FSK_OOK_REG_DIO_MAPPING_2_ADDR);
	return result;
}
uint8_t readRegVersion(void)
{
	uint8_t result = 0x00;
	result = readFromRegisterSync(FSK_OOK_REG_VERSION_ADDR);
	return result;
}
uint8_t writeRegPllHop(uint8_t value)
{
	uint8_t received_value = 0;
	writeToRegisterSync(FSK_OOK_REG_PLL_HOP_ADDR, value);
	received_value = readFromRegisterSync(FSK_OOK_REG_PLL_HOP_ADDR);
	if(received_value == value)
	{
		return 0; //success
	}
	return 1; //not success
}
uint8_t readRegPllHop(void)
{
	uint8_t result = 0x00;
	result = readFromRegisterSync(FSK_OOK_REG_PLL_HOP_ADDR);
	return result;
}
uint8_t writeRegTcxo(uint8_t value)
{
	uint8_t received_value = 0;
	writeToRegisterSync(FSK_OOK_REG_TCXO_ADDR, value);
	received_value = readFromRegisterSync(FSK_OOK_REG_TCXO_ADDR);
	if(received_value == value)
	{
		return 0; //success
	}
	return 1; //not success
}
uint8_t readRegTcxo(void)
{
	uint8_t result = 0x00;
	result = readFromRegisterSync(FSK_OOK_REG_TCXO_ADDR);
	return result;
}
uint8_t writeRegPaDac(uint8_t value)
{
	uint8_t received_value = 0;
	writeToRegisterSync(FSK_OOK_REG_PA_DAC_ADDR, value);
	received_value = readFromRegisterSync(FSK_OOK_REG_PA_DAC_ADDR);
	if(received_value == value)
	{
		return 0; //success
	}
	return 1; //not success
}
uint8_t readwriteRegPaDac(void)
{
	uint8_t result = 0x00;
	result = readFromRegisterSync(FSK_OOK_REG_PA_DAC_ADDR);
	return result;
}
uint8_t writeRegBitrateFrac(uint8_t value)
{
	uint8_t received_value = 0;
	writeToRegisterSync(FSK_OOK_REG_BIT_RATE_FRACTIONAL_ADDR, value);
	received_value = readFromRegisterSync(FSK_OOK_REG_BIT_RATE_FRACTIONAL_ADDR);
	if(received_value == value)
	{
		return 0; //success
	}
	return 1; //not success
}
uint8_t readRegBitrateFrac(void)
{
	uint8_t result = 0x00;
	result = readFromRegisterSync(FSK_OOK_REG_BIT_RATE_FRACTIONAL_ADDR);
	return result;
}
uint8_t writeRegAgcRefLf(uint8_t value)
{
	uint8_t received_value = 0;
	writeToRegisterSync(FSK_OOK_REG_AGC_REF_ADDR, value);
	received_value = readFromRegisterSync(FSK_OOK_REG_AGC_REF_ADDR);
	if(received_value == value)
	{
		return 0; //success
	}
	return 1; //not success
}
uint8_t readRegAgcRefLf(void)
{
	uint8_t result = 0x00;
	result = readFromRegisterSync(FSK_OOK_REG_AGC_REF_ADDR);
	return result;
}
uint8_t writeRegAgcThresh1Lf(uint8_t value)
{
	uint8_t received_value = 0;
	writeToRegisterSync(FSK_OOK_REG_AGC_THRESHOLD_1_ADDR, value);
	received_value = readFromRegisterSync(FSK_OOK_REG_AGC_THRESHOLD_1_ADDR);
	if(received_value == value)
	{
		return 0; //success
	}
	return 1; //not success
}
uint8_t readRegAgcThresh1Lf(void)
{
	uint8_t result = 0x00;
	result = readFromRegisterSync(FSK_OOK_REG_AGC_THRESHOLD_1_ADDR);
	return result;
}
uint8_t writeRegAgcThresh2Lf(uint8_t value)
{
	uint8_t received_value = 0;
	writeToRegisterSync(FSK_OOK_REG_AGC_THRESHOLD_2_ADDR, value);
	received_value = readFromRegisterSync(FSK_OOK_REG_AGC_THRESHOLD_2_ADDR);
	if(received_value == value)
	{
		return 0; //success
	}
	return 1; //not success
}
uint8_t readRegAgcThresh2Lf(void)
{
	uint8_t result = 0x00;
	result = readFromRegisterSync(FSK_OOK_REG_AGC_THRESHOLD_2_ADDR);
	return result;
}
uint8_t writeRegAgcThresh3Lf(uint8_t value)
{
	uint8_t received_value = 0;
	writeToRegisterSync(FSK_OOK_REG_AGC_THRESHOLD_3_ADDR, value);
	received_value = readFromRegisterSync(FSK_OOK_REG_AGC_THRESHOLD_3_ADDR);
	if(received_value == value)
	{
		return 0; //success
	}
	return 1; //not success
}
uint8_t readRegAgcThresh3Lf(void)
{
	uint8_t result = 0x00;
	result = readFromRegisterSync(FSK_OOK_REG_AGC_THRESHOLD_3_ADDR);
	return result;
}
uint8_t writeRegPllLf(uint8_t value)
{
	uint8_t received_value = 0;
	writeToRegisterSync(FSK_OOK_REG_PLL_ADDR, value);
	received_value = readFromRegisterSync(FSK_OOK_REG_PLL_ADDR);
	if(received_value == value)
	{
		return 0; //success
	}
	return 1; //not success
}
uint8_t readRegPllLf(void)
{
	uint8_t result = 0x00;
	result = readFromRegisterSync(FSK_OOK_REG_PLL_ADDR);
	return result;
}

//---------------------------------------------------------------
uint8_t writeLoRaRegFifoAddrPtr(uint8_t value)
{
	uint8_t received_value = 0;
	writeToRegisterSync(LORA_REG_FIFO_ADDRESS_PTR_ADDR, value);
	received_value = readFromRegisterSync(LORA_REG_FIFO_ADDRESS_PTR_ADDR);
	if(received_value == value)
	{
		return 0; //success
	}
	return 1; //not success
}
uint8_t readLoRaRegFifoAddrPtr(void)
{
	uint8_t result = 0x00;
	result = readFromRegisterSync(LORA_REG_FIFO_ADDRESS_PTR_ADDR);
	return result;
}
uint8_t writeLoRaRegFifoTxBaseAddr(uint8_t value)
{
	uint8_t received_value = 0;
	writeToRegisterSync(LORA_REG_FIFO_TX_BASE_ADDRESS_ADDR, value);
	received_value = readFromRegisterSync(LORA_REG_FIFO_TX_BASE_ADDRESS_ADDR);
	if(received_value == value)
	{
		return 0; //success
	}
	return 1; //not success
}
uint8_t readLoRaRegFifoTxBaseAddr(void)
{
	uint8_t result = 0x00;
	result = readFromRegisterSync(LORA_REG_FIFO_TX_BASE_ADDRESS_ADDR);
	return result;
}
uint8_t writeLoRaRegFifoRxBaseAddr(uint8_t value)
{
	uint8_t received_value = 0;
	writeToRegisterSync(LORA_REG_FIFO_RX_BASE_ADDRESS_ADDR, value);
	received_value = readFromRegisterSync(LORA_REG_FIFO_RX_BASE_ADDRESS_ADDR);
	if(received_value == value)
	{
		return 0; //success
	}
	return 1; //not success
}
uint8_t readLoRaRegFifoRxBaseAddr(void)
{
	uint8_t result = 0x00;
	result = readFromRegisterSync(LORA_REG_FIFO_RX_BASE_ADDRESS_ADDR);
	return result;
}
uint8_t readLoRaRegFifoRxCurrentAddr(void)
{
	uint8_t result = 0x00;
	result = readFromRegisterSync(LORA_REG_FIFO_RX_CURRENT_ADDRESS_ADDR);
	return result;
}
uint8_t writeLoRaRegIrqFlagsMask(uint8_t value)
{
	uint8_t received_value = 0;
	writeToRegisterSync(LORA_REG_IRQ_FLAGS_MASK_ADDR, value);
	received_value = readFromRegisterSync(LORA_REG_IRQ_FLAGS_MASK_ADDR);
	if(received_value == value)
	{
		return 0; //success
	}
	return 1; //not success
}
uint8_t readLoRaRegIrqFlagsMask(void)
{
	uint8_t result = 0x00;
	result = readFromRegisterSync(LORA_REG_IRQ_FLAGS_MASK_ADDR);
	return result;
}
uint8_t writeLoRaRegModemConfig1(uint8_t value)
{
	uint8_t received_value = 0;
	writeToRegisterSync(LORA_REG_MODEM_CONFIG_1_ADDR, value);
	received_value = readFromRegisterSync(LORA_REG_MODEM_CONFIG_1_ADDR);
	if(received_value == value)
	{
		return 0; //success
	}
	return 1; //not success
}
uint8_t readLoRaRegModemConfig1(void)
{
	uint8_t result = 0x00;
	result = readFromRegisterSync(LORA_REG_MODEM_CONFIG_1_ADDR);
	return result;
}
uint8_t writeLoRaRegModemConfig2(uint8_t value)
{
	uint8_t received_value = 0;
	writeToRegisterSync(LORA_REG_MODEM_CONFIG_2_ADDR, value);
	received_value = readFromRegisterSync(LORA_REG_MODEM_CONFIG_2_ADDR);
	if(received_value == value)
	{
		return 0; //success
	}
	return 1; //not success
}
uint8_t readLoRaRegModemConfig2(void)
{
	uint8_t result = 0x00;
	result = readFromRegisterSync(LORA_REG_MODEM_CONFIG_2_ADDR);
	return result;
}
uint8_t writeLoRaRegSymbolTimeoutLsb(uint8_t value)
{
	uint8_t received_value = 0;
	writeToRegisterSync(LORA_REG_SYMB_TIMEOUT_LSB_ADDR, value);
	received_value = readFromRegisterSync(LORA_REG_SYMB_TIMEOUT_LSB_ADDR);
	if(received_value == value)
	{
		return 0; //success
	}
	return 1; //not success
}
uint8_t readLoRaRegSymbolTimeoutLsb(void)
{
	uint8_t result = 0x00;
	result = readFromRegisterSync(LORA_REG_SYMB_TIMEOUT_LSB_ADDR);
	return result;
}
uint8_t writeLoRaRegPreambleLengthMsb(uint8_t value)
{
	uint8_t received_value = 0;
	writeToRegisterSync(LORA_REG_PREAMBLE_MSB_ADDR, value);
	received_value = readFromRegisterSync(LORA_REG_PREAMBLE_MSB_ADDR);
	if(received_value == value)
	{
		return 0; //success
	}
	return 1; //not success
}
uint8_t readLoRaRegPreambleLengthMsb(void)
{
	uint8_t result = 0x00;
	result = readFromRegisterSync(LORA_REG_PREAMBLE_MSB_ADDR);
	return result;
}
uint8_t writeLoRaRegPreambleLengthLsb(uint8_t value)
{
	uint8_t received_value = 0;
	writeToRegisterSync(LORA_REG_PREAMBLE_LSB_ADDR, value);
	received_value = readFromRegisterSync(LORA_REG_PREAMBLE_LSB_ADDR);
	if(received_value == value)
	{
		return 0; //success
	}
	return 1; //not success
}
uint8_t readLoRaRegPreambleLengthLsb(void)
{
	uint8_t result = 0x00;
	result = readFromRegisterSync(LORA_REG_PREAMBLE_LSB_ADDR);
	return result;
}
uint8_t writeLoRaRegPayloadLength(uint8_t value)
{
	uint8_t received_value = 0;
	writeToRegisterSync(LORA_REG_PAYLOAD_LENGTH_ADDR, value);
	received_value = readFromRegisterSync(LORA_REG_PAYLOAD_LENGTH_ADDR);
	if(received_value == value)
	{
		return 0; //success
	}
	return 1; //not success
}
uint8_t readLoRaRegPayloadLength(void)
{
	uint8_t result = 0x00;
	result = readFromRegisterSync(LORA_REG_PAYLOAD_LENGTH_ADDR);
	return result;
}
uint8_t writeLoRaRegPayloadMaxLength(uint8_t value)
{
	uint8_t received_value = 0;
	writeToRegisterSync(LORA_REG_MAX_PAYLOAD_LENGTH_ADDR, value);
	received_value = readFromRegisterSync(LORA_REG_MAX_PAYLOAD_LENGTH_ADDR);
	if(received_value == value)
	{
		return 0; //success
	}
	return 1; //not success
}
uint8_t readLoRaRegPayloadMaxLength(void)
{
	uint8_t result = 0x00;
	result = readFromRegisterSync(LORA_REG_MAX_PAYLOAD_LENGTH_ADDR);
	return result;
}
uint8_t writeLoRaRegFreqHoppingPeriod(uint8_t value)
{
	uint8_t received_value = 0;
	writeToRegisterSync(LORA_REG_HOP_PERIOD_ADDR, value);
	received_value = readFromRegisterSync(LORA_REG_HOP_PERIOD_ADDR);
	if(received_value == value)
	{
		return 0; //success
	}
	return 1; //not success
}
uint8_t readLoRaRegFreqHoppingPeriod(void)
{
	uint8_t result = 0x00;
	result = readFromRegisterSync(LORA_REG_HOP_PERIOD_ADDR);
	return result;
}
uint8_t writeLoRaRegModemConfig3(uint8_t value)
{
	uint8_t received_value = 0;
	writeToRegisterSync(LORA_REG_MODEM_CONFIG_3_ADDR, value);
	received_value = readFromRegisterSync(LORA_REG_MODEM_CONFIG_3_ADDR);
	if(received_value == value)
	{
		return 0; //success
	}
	return 1; //not success
}
uint8_t readLoRaRegModemConfig3(void)
{
	uint8_t result = 0x00;
	result = readFromRegisterSync(LORA_REG_MODEM_CONFIG_3_ADDR);
	return result;
}
uint8_t writeLoRaRegLfFreq2(uint8_t value)
{
	uint8_t received_value = 0;
	writeToRegisterSync(LORA_REG_LF_FREQ_2_ADDR, value);
	received_value = readFromRegisterSync(LORA_REG_LF_FREQ_2_ADDR);
	if(received_value == value)
	{
		return 0; //success
	}
	return 1; //not success
}
uint8_t readLoRaRegLfFreq2(void)
{
	uint8_t result = 0x00;
	result = readFromRegisterSync(LORA_REG_LF_FREQ_2_ADDR);
	return result;
}
uint8_t writeLoRaRegLfFreq1(uint8_t value)
{
	uint8_t received_value = 0;
	writeToRegisterSync(LORA_REG_LF_FREQ_1_ADDR, value);
	received_value = readFromRegisterSync(LORA_REG_LF_FREQ_1_ADDR);
	if(received_value == value)
	{
		return 0; //success
	}
	return 1; //not success
}
uint8_t readLoRaRegLfFreq1(void)
{
	uint8_t result = 0x00;
	result = readFromRegisterSync(LORA_REG_LF_FREQ_1_ADDR);
	return result;
}
uint8_t writeLoRaRegDetectOptimize(uint8_t value)
{
	uint8_t received_value = 0;
	writeToRegisterSync(LORA_REG_DETECT_OPTIMIZE_ADDR, value);
	received_value = readFromRegisterSync(LORA_REG_DETECT_OPTIMIZE_ADDR);
	if(received_value == value)
	{
		return 0; //success
	}
	return 1; //not success
}
uint8_t readLoRaRegDetectOptimize(void)
{
	uint8_t result = 0x00;
	result = readFromRegisterSync(LORA_REG_DETECT_OPTIMIZE_ADDR);
	return result;
}
uint8_t writeLoRaRegInvertIQ(uint8_t value)
{
	uint8_t received_value = 0;
	writeToRegisterSync(LORA_REG_INVERT_I_Q_ADDR, value);
	received_value = readFromRegisterSync(LORA_REG_INVERT_I_Q_ADDR);
	if(received_value == value)
	{
		return 0; //success
	}
	return 1; //not success
}
uint8_t readLoRaRegInvertIQ(void)
{
	uint8_t result = 0x00;
	result = readFromRegisterSync(LORA_REG_INVERT_I_Q_ADDR);
	return result;
}
uint8_t writeLoRaRegHighBWOptimize1(uint8_t value)
{
	uint8_t received_value = 0;
	writeToRegisterSync(LORA_REG_HIGH_BW_OPTIMIZE_1_ADDR, value);
	received_value = readFromRegisterSync(LORA_REG_HIGH_BW_OPTIMIZE_1_ADDR);
	if(received_value == value)
	{
		return 0; //success
	}
	return 1; //not success
}
uint8_t readLoRaRegHighBWOptimize1(void)
{
	uint8_t result = 0x00;
	result = readFromRegisterSync(LORA_REG_HIGH_BW_OPTIMIZE_1_ADDR);
	return result;
}
uint8_t writeLoRaRegDetectionThreshold(uint8_t value)
{
	uint8_t received_value = 0;
	writeToRegisterSync(LORA_REG_DETECTION_THRESHOLD_ADDR, value);
	received_value = readFromRegisterSync(LORA_REG_DETECTION_THRESHOLD_ADDR);
	if(received_value == value)
	{
		return 0; //success
	}
	return 1; //not success
}
uint8_t readLoRaRegDetectionThreshold(void)
{
	uint8_t result = 0x00;
	result = readFromRegisterSync(LORA_REG_DETECTION_THRESHOLD_ADDR);
	return result;
}
uint8_t writeLoRaRegSyncWord(uint8_t value)
{
	uint8_t received_value = 0;
	writeToRegisterSync(LORA_REG_SYNC_WORD_ADDR, value);
	received_value = readFromRegisterSync(LORA_REG_SYNC_WORD_ADDR);
	if(received_value == value)
	{
		return 0; //success
	}
	return 1; //not success
}
uint8_t readLoRaRegSyncWord(void)
{
	uint8_t result = 0x00;
	result = readFromRegisterSync(LORA_REG_SYNC_WORD_ADDR);
	return result;
}
uint8_t writeLoRaRegHighBWOptimize2(uint8_t value)
{
	uint8_t received_value = 0;
	writeToRegisterSync(LORA_REG_HIGH_BW_OPTIMIZE_2_ADDR, value);
	received_value = readFromRegisterSync(LORA_REG_HIGH_BW_OPTIMIZE_2_ADDR);
	if(received_value == value)
	{
		return 0; //success
	}
	return 1; //not success
}
uint8_t readLoRaRegHighBWOptimize2(void)
{
	uint8_t result = 0x00;
	result = readFromRegisterSync(LORA_REG_HIGH_BW_OPTIMIZE_2_ADDR);
	return result;
}
uint8_t writeLoRaRegInvertIQ2(uint8_t value)
{
	uint8_t received_value = 0;
	writeToRegisterSync(LORA_REG_INVERT_I_Q_2_ADDR, value);
	received_value = readFromRegisterSync(LORA_REG_INVERT_I_Q_2_ADDR);
	if(received_value == value)
	{
		return 0; //success
	}
	return 1; //not success
}
uint8_t readLoRaRegInvertIQ2(void)
{
	uint8_t result = 0x00;
	result = readFromRegisterSync(LORA_REG_INVERT_I_Q_2_ADDR);
	return result;
}

uint32_t calculate_time_on_air_for_LoRa_ms(void)
{
	uint32_t result;
	//TODO realize calculator using parameters
	// Spreading factor, bandwidth, code rating, low data rate optimizer,
	// payload length, programble preamble
	// explicit header, crc enable


	result = 700; //for my configuration SF-12 BW-125kHz, etc
	return result;
}
// логика обработки очереди запросов SPI по DMA



void handleRequestFromSPI(SPI_dma_req *req)
{

		switch(req->readWriteStatus)
		{
			case SPI_WRITE:
			{
				if(deviceState.modulationType == FSK_OOK_MODULATION)
				{
					switch(req->addr)
					{
						//here callbacks
						case REG_FIFO_ADDR:
							SX1276_77_78_79_writeFifoCallback();
							break;
						case REG_OP_MODE_ADDR:
							SX1276_77_78_79_writeOpModeCallback();
							break;
						case FSK_OOK_REG_PAYLOAD_LENGTH_ADDR:
							SX1276_77_78_79_writePayloadLengthCallback();
							break;
						case FSK_OOK_REG_NODE_ADDRESS_ADDR:
							SX1276_77_78_79_writeNodeAndBroadcastAddressCallback();
							break;
						case FSK_OOK_REG_FIFO_THRESHOLD_ADDR:
							SX1276_77_78_79_writeFifoThresholdCallback();
							break;
						case FSK_OOK_REG_IRQ_FLAGS_1_ADDR:
							SX1276_77_78_79_writeIrqFlags_1_2Callback();
							break;
					}
				}
				else if(deviceState.modulationType == LORA_MODULATION)
				{
					switch(req->addr)
					{
						//here callbacks
						case REG_FIFO_ADDR:
							SX1276_77_78_79_writeFifoCallback();
							break;
						case REG_OP_MODE_ADDR:
							SX1276_77_78_79_writeOpModeCallback();
							break;
						case LORA_REG_FIFO_ADDRESS_PTR_ADDR:
							SX1276_77_78_79_writeFifoPtrAddr_L_Callback();
							break;
						case LORA_REG_IRQ_FLAGS_ADDR:
							SX1276_77_78_79_writeLoRaIrqFlags_L_Callback();
							break;
					}
				}
				break;
			}
			case SPI_READ:
			{
				if(deviceState.modulationType == FSK_OOK_MODULATION)
				{
					switch(req->addr)
					{
						case REG_FIFO_ADDR:
							memcpy(fifoReadPtr, req->readData + 1, deviceState.PacketPayloadSize);
							SX1276_77_78_79_Delay_ms(2);
							SX1276_77_78_79_readFifoCallback();
							break;
						case REG_OP_MODE_ADDR:
							deviceState.OpMode = req->readData[1];
							registersConfig.RegOpMode = deviceState.OpMode;
							SX1276_77_78_79_readOpModeCallback();
							break;
						case FSK_OOK_REG_PAYLOAD_LENGTH_ADDR:
							deviceState.PacketPayloadSize = req->readData[1];
							registersConfig.RegPayloadLength = deviceState.PacketPayloadSize;
							SX1276_77_78_79_readPayloadLengthCallback();
							break;
						case FSK_OOK_REG_NODE_ADDRESS_ADDR:
							deviceState.NodeAddress = req->readData[1];
							deviceState.BroadcastAddress = req->readData[2];
							registersConfig.RegNodeAdrs = deviceState.NodeAddress;
							registersConfig.RegBroadcastAdrs = deviceState.BroadcastAddress;
							SX1276_77_78_79_readNodeAndBroadcastAddressCallback();
							break;
						case FSK_OOK_REG_FIFO_THRESHOLD_ADDR:
							//will be reading in future if it will be needed
							SX1276_77_78_79_readFifoThresholdCallback();
							break;
						case FSK_OOK_REG_IRQ_FLAGS_1_ADDR:
							deviceState.LastIrqFlags1 = req->readData[1];
							deviceState.LastIrqFlags2 = req->readData[2];
							SX1276_77_78_79_readIrqFlags_1_2Callback();
							break;
						case FSK_OOK_REG_RSSI_VALUE_ADDR:
							deviceState.LastRSSI = req->readData[1];
							registersConfig.RegRssiValue = deviceState.LastRSSI;
							deviceState.LastRSSIFloat =  (float)deviceState.LastRSSI / -2.0f;
							SX1276_77_78_79_readRSSICallback();
							break;
						case FSK_OOK_REG_TEMP_ADDR:

							if((req->readData[1] & 0x80) == 0x80)
							{
								deviceState.LastTemperature = 255 - req->readData[1];
							}
							else
							{
								deviceState.LastTemperature = -1 * req->readData[1];
							}
							SX1276_77_78_79_readTemperatureCallback();
							break;
					}
				}
				else if(deviceState.modulationType == LORA_MODULATION)
				{
					switch(req->addr)
					{
						case REG_FIFO_ADDR:
							memcpy(fifoReadPtr, req->readData + 1, deviceState.PacketPayloadSize); //if no errors PacketPayloadSize = LoraRxBytesNbLast
							SX1276_77_78_79_Delay_ms(2);
							SX1276_77_78_79_readFifoCallback();
							break;
						case REG_OP_MODE_ADDR:
							deviceState.OpMode = req->readData[1];
							registersConfig.RegOpMode = deviceState.OpMode;
							SX1276_77_78_79_readOpModeCallback();
							break;
						case LORA_REG_FIFO_ADDRESS_PTR_ADDR:
							//TODO req->readData[1] - ptr value
							SX1276_77_78_79_readFifoPtrAddr_L_Callback();
							break;
						case LORA_REG_IRQ_FLAGS_ADDR:
							deviceState.LoraIrqFlags = req->readData[1];
							SX1276_77_78_79_readLoRaIrqFlags_L_Callback();
							break;
						case LORA_REG_FIFO_RX_CURRENT_ADDRESS_ADDR:
							deviceState.LoraFifoRxCurrentAddrLast = req->readData[1];
							SX1276_77_78_79_writeFifoPtrAddr_L(deviceState.LoraFifoRxCurrentAddrLast);
							//read fifo
							if(deviceState.LoraPacketReceivingOnGoing == 1)
							{
								SX1276_77_78_79_readFifo(fifoReadPtr, deviceState.LoraRxBytesNbLast);
								SX1276_77_78_79_readModemStatusAndRSSI_L();
								deviceState.LoraPacketReceivingOnGoing = 0;
							}
							SX1276_77_78_79_readFifoRxCurrentAddr_L_Callback();
							break;
						case LORA_REG_RX_NB_BYTES_ADDR:
							deviceState.LoraRxBytesNbLast = req->readData[1];
							SX1276_77_78_79_readFifoRxCurrentAddr_L();
							SX1276_77_78_79_readFifoRxBytesNb_L_Callback();
							break;
						case LORA_REG_MODEM_STATUS_ADDR:
							deviceState.LoraModemStatusLast = req->readData[1];
							deviceState.LoraPacketSnrLast 	= req->readData[2];
							deviceState.LoraPacketRssiLast 	= req->readData[3];
							deviceState.LoraRssiLast 		= req->readData[4];

							SX1276_77_78_79_readModemStatusAndRSSI_L_Callback();
							break;
					}
				}
				break;
			}
		}

}


void SPI_dma_request_read_write(SPI_dma_req *req)
{	uint8_t i = 0;
	uint8_t  size;
	req->ready = 0;
	*rwqueue_wrptr++ = req;
	if (rwqueue_wrptr == rwqueue + SPI_READ_WRITE_QUEUE_SIZE)
	{
		rwqueue_wrptr = rwqueue;
	}
	if (0 == 0) // Start SPI transfer for first request
	{

//	while(((DMA1_Channel3->CCR) & DMA_CCR_EN)){};
//	LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_3, req->size);
//	LL_DMA_ConfigAddresses(DMA1, LL_DMA_CHANNEL_3, (uint32_t)req->writeData,  LL_SPI_DMA_GetRegAddr(SPI1), LL_DMA_GetDataTransferDirection(DMA1, LL_DMA_CHANNEL_3));
//	LL_GPIO_ResetOutputPin(NSS_SX1276_GPIO_Port, NSS_SX1276_Pin);//CS_LO();						
//	LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_3);
		
		
		LL_GPIO_ResetOutputPin(NSS_SX1276_GPIO_Port, NSS_SX1276_Pin);//CS_LO();	

	for(i= 0 ; i< req->size;i++){
	while (!(SPI1->SR & SPI_SR_TXE)){}; 
	SPI1->DR = req->writeData[i];
	while (SPI1->SR & SPI_SR_BSY){};
	while (!(SPI1->SR & SPI_SR_RXNE)){}; 
	size = SPI1->DR;
	while (!(SPI1->SR & SPI_SR_TXE)){};	 
	}
		
		
LL_GPIO_SetOutputPin(NSS_SX1276_GPIO_Port, NSS_SX1276_Pin);//CS_LO();		
		
		
		
		
	}
	

	rwQueueCnt++;
	
	
	
	
	
	
	
	
	
}



