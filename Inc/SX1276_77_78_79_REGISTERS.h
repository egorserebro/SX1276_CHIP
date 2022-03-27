/*
 * SX1276_77_78_79_REGISTERS.h
 *
 *  Created on: Jan 11, 2021
 *      Author: i.mishustin
 */

#ifndef SX1276_77_78_79_SX1276_77_78_79_REGISTERS_H_
#define SX1276_77_78_79_SX1276_77_78_79_REGISTERS_H_



/*
 * module have 72 + 38 + 2 regs = 112 registers
 * wt - write trigger
 * wc - write clear
 * rw - read/write
 * r  - read only
 * rwc - read/ write clear
 */


enum SX1276_77_78_79_GENERAL_REGS_ADDRESSES
{
	REG_FIFO_ADDR = 0x00, 							//FIFO read/write access
	REG_OP_MODE_ADDR = 0x01							//Operating mode & LoRa / FSK selection
};


enum SX1276_77_78_79_FSK_OOK_MODE_REGS_ADDRESSES
{
	FSK_OOK_REG_BIT_RATE_MSB_ADDR 				= 0x02,					//	Bit Rate setting, Most Significant Bits							POR 0x1A Default 0x1A
	FSK_OOK_REG_BIT_RATE_LSB_ADDR 				= 0x03,					//	Bit Rate setting, Least Significant Bits						POR 0x0B Default 0x0B
	FSK_OOK_REG_F_DEV_MSB_ADDR 	  				= 0x04,					//	Frequency Deviation setting, Most Significant Bits				POR 0x00
	FSK_OOK_REG_F_DEV_LSB_ADDR	  				= 0x05,					//	Frequency Deviation setting, Least Significant Bits				POR 0x52
	FSK_OOK_REG_F_RF_MSB_ADDR	  				= 0x06,					//	RF Carrier Frequency, Most Significant Bits						POR 0x6C
	FSK_OOK_REG_F_RF_MID_ADDR					= 0x07,					//	RF Carrier Frequency, Intermediate Bits							POR 0x80
	FSK_OOK_REG_F_RF_LSB_ADDR					= 0x08,					//	RF Carrier Frequency, Least Significant Bits					POR 0x00
	FSK_OOK_REG_PA_CONFIG_ADDR					= 0x09,					//	PA selection and Output Power control							POR 0x4F
	FSK_OOK_REG_PA_RAMP_ADDR					= 0x0A,					//	Control of PA ramp time, low phase noise PLL					POR 0x09
	FSK_OOK_REG_O_C_P_ADDR						= 0x0B,					//	Over Current Protection control									POR 0x2B
	FSK_OOK_REG_LNA_ADDR						= 0x0C,					//	LNA settings													POR 0x20
	FSK_OOK_REG_RX_CONFIG_ADDR					= 0x0D,					//	AFC, AGC, ctrl													POR 0x08 Default 0x0E
	FSK_OOK_REG_RSSI_CONFIG_ADDR				= 0x0E,					//	RSSI
	FSK_OOK_REG_RSSI_COLLISION_ADDR 			= 0x0F,					//	RSSI Collision detector
	FSK_OOK_REG_RSSI_THRESHOLD_ADDR  			= 0x10,					//	RSSI Threshold control
	FSK_OOK_REG_RSSI_VALUE_ADDR					= 0x11,					// 	RSSI value in dBm
	FSK_OOK_REG_RX_BW_ADDR						= 0x12,					//	Channel Filter BW Control
	FSK_OOK_REG_AFC_BW_ADDR						= 0x13,					//	AFC Channel Filter BW
	FSK_OOK_REG_OOK_PEAK_ADDR					= 0x14,					//	OOK demodulator
	FSK_OOK_REG_OOK_FIX_ADDR					= 0x15,					//	Threshold of the OOK demod
	FSK_OOK_REG_OOK_AVG_ADDR					= 0x16,					//	Average of the OOK demod
	FSK_OOK_REG_AFC_FEI_ADDR					= 0x1A,					//	AFC and FEI control
	FSK_OOK_REG_AFC_MSB_ADDR					= 0x1B,					//	Frequency correction value of the AFC
	FSK_OOK_REG_AFC_LSB_ADDR					= 0x1C,					//
	FSK_OOK_REG_FEI_MSB_ADDR					= 0x1D,					//	Value of the calculated frequency error
	FSK_OOK_REG_FEI_LSB_ADDR					= 0x1E,					//
	FSK_OOK_REG_PREAMBLE_DETECTOR_ADDR			= 0x1F,					//	Settings of the Preamble Detector
	FSK_OOK_REG_RX_TIMEOUT_1_ADDR				= 0x20,					//	Timeout Rx request and RSSI
	FSK_OOK_REG_RX_TIMEOUT_2_ADDR				= 0x21,					//	Timeout RSSI and Pay-loadReady
	FSK_OOK_REG_RX_TIMEOUT_3_ADDR				= 0x22,					//	Timeout RSSI and Sync Address
	FSK_OOK_REG_RX_DELAY_ADDR					= 0x23,					//	Delay between Rx cycles
	FSK_OOK_REG_OSC_ADDR						= 0x24,					//	RC Oscillators Settings, CLK-OUT frequency
	FSK_OOK_REG_PREAMBLE_MSB_ADDR				= 0x25,					//	Preamble length, MSB
	FSK_OOK_REG_PREAMBLE_LSB_ADDR				= 0x26,					//	Preamble length, LSB
	FSK_OOK_REG_SYNC_CONFIG_ADDR				= 0x27,					//	Sync Word Recognition control
	FSK_OOK_REG_SYNC_VALUE_1_ADDR				= 0x28,					//	Sync Word bytes 1
	FSK_OOK_REG_SYNC_VALUE_2_ADDR				= 0x29,					//	Sync Word bytes 2
	FSK_OOK_REG_SYNC_VALUE_3_ADDR				= 0x2A,					//	Sync Word bytes 3
	FSK_OOK_REG_SYNC_VALUE_4_ADDR				= 0x2B,					//	Sync Word bytes 4
	FSK_OOK_REG_SYNC_VALUE_5_ADDR				= 0x2C,					//	Sync Word bytes 5
	FSK_OOK_REG_SYNC_VALUE_6_ADDR				= 0x2D,					//	Sync Word bytes 6
	FSK_OOK_REG_SYNC_VALUE_7_ADDR				= 0x2E,					//	Sync Word bytes 7
	FSK_OOK_REG_SYNC_VALUE_8_ADDR				= 0x2F,					//	Sync Word bytes 8
	FSK_OOK_REG_PACKET_CONFIG_1_ADDR			= 0x30,					//	Packet mode settings
	FSK_OOK_REG_PACKET_CONFIG_2_ADDR			= 0x31,					//	Packet mode settings
	FSK_OOK_REG_PAYLOAD_LENGTH_ADDR				= 0x32,					//	Payload length setting
	FSK_OOK_REG_NODE_ADDRESS_ADDR				= 0x33,					//	Node address
	FSK_OOK_REG_BROADCAST_ADDRESS_ADDR			= 0x34,					//	Broadcast address
	FSK_OOK_REG_FIFO_THRESHOLD_ADDR				= 0x35,					//	Fifo threshold, Tx start condition
	FSK_OOK_REG_SEQUENCER_CONFIG_1_ADDR			= 0x36,					//	Top level Sequencer settings
	FSK_OOK_REG_SEQUENCER_CONFIG_2_ADDR			= 0x37,					//	Top level Sequencer settings
	FSK_OOK_REG_TIMER_RESOLUTION_ADDR			= 0x38,					//	Timer 1 and 2 resolution control
	FSK_OOK_REG_TIMER_1_COEF_ADDR				= 0x39,					//	Timer 1 setting
	FSK_OOK_REG_TIMER_2_COEF_ADDR				= 0x3A,					//	Timer 2 setting
	FSK_OOK_REG_IMAGE_CALIBRATION_ADDR			= 0x3B,					//	Image calibration engine control
	FSK_OOK_REG_TEMP_ADDR						= 0x3C,					//	Temperature Sensor value
	FSK_OOK_REG_LOW_BAT_ADDR					= 0x3D,					//	Low Battery Indicator Settings
	FSK_OOK_REG_IRQ_FLAGS_1_ADDR				= 0x3E,					//	Status register: PLL Lock state, Timeout, RSSI
	FSK_OOK_REG_IRQ_FLAGS_2_ADDR				= 0x3F,					//	Status register: FIFO handling flags, Low Battery
	FSK_OOK_REG_DIO_MAPPING_1_ADDR				= 0x40,					//	Mapping of pins DIO0 to DIO3
	FSK_OOK_REG_DIO_MAPPING_2_ADDR				= 0x41,					//	Mapping of pins DIO4 and DIO5, ClkOut frequency
	FSK_OOK_REG_VERSION_ADDR					= 0x42,					//	Semtech ID relating the silicon revision
	FSK_OOK_REG_PLL_HOP_ADDR					= 0x44,					//	Control the fast frequency hopping mode
	FSK_OOK_REG_TCXO_ADDR						= 0x4B,					//	TCXO or XTAL input setting
	FSK_OOK_REG_PA_DAC_ADDR						= 0x4D,					//	Higher power settings of the PA
	FSK_OOK_REG_FORMER_TEMP_ADDR				= 0x5B,					//	Stored temperature during the former IQ Calibration
	FSK_OOK_REG_BIT_RATE_FRACTIONAL_ADDR		= 0x5D,					//	Fractional part in the Bit Rate division ratio
	FSK_OOK_REG_AGC_REF_ADDR					= 0x61,					//	Adjustment of the AGC thresholds
	FSK_OOK_REG_AGC_THRESHOLD_1_ADDR			= 0x62,					//	Adjustment of the AGC thresholds
	FSK_OOK_REG_AGC_THRESHOLD_2_ADDR			= 0x63,					//	Adjustment of the AGC thresholds
	FSK_OOK_REG_AGC_THRESHOLD_3_ADDR			= 0x64,					//	Adjustment of the AGC thresholds
	FSK_OOK_REG_PLL_ADDR						= 0x70					//	Control of the PLL bandwidth
};

enum SX1276_77_78_79_LORA_MODE_REGS_ADDRESSES
{

	LORA_REG_FIFO_ADDRESS_PTR_ADDR 			= 0x0D,				// FIFO SPI pointer
	LORA_REG_FIFO_TX_BASE_ADDRESS_ADDR 		= 0x0E,				// Start Tx data
	LORA_REG_FIFO_RX_BASE_ADDRESS_ADDR		= 0x0F,				// Start Rx data
	LORA_REG_FIFO_RX_CURRENT_ADDRESS_ADDR 	= 0x10,				// Start address of last packet received
	LORA_REG_IRQ_FLAGS_MASK_ADDR			= 0x11,				// Optional IRQ flag mask
	LORA_REG_IRQ_FLAGS_ADDR					= 0x12,				// IRQ flags
	LORA_REG_RX_NB_BYTES_ADDR				= 0x13,				// Number of received bytes
	LORA_REG_RX_HEADER_CNT_VALUE_MSB_ADDR	= 0x14,				// Number of valid headers received
	LORA_REG_RX_HEADER_CNT_VALUE_LSB_ADDR	= 0x15,
	LORA_REG_RX_PACKET_CNT_VALUE_MSB_ADDR	= 0x16,				// Number of valid packets received
	LORA_REG_RX_PACKET_CNT_VALUE_LSB_ADDR	= 0x17,
	LORA_REG_MODEM_STATUS_ADDR				= 0x18,				// Live LoRa modem status
	LORA_REG_PKT_SNR_VALUE_ADDR				= 0x19,				// Espimation of last packet SNR
	LORA_REG_PKT_RSSI_VALUE_ADDR			= 0x1A,				// RSSI of last packet
	LORA_REG_RSSI_VALUE_ADDR				= 0x1B,				// Current RSSI
	LORA_REG_HOP_CHANNEL_ADDR				= 0x1C,				// FHSS start channel
	LORA_REG_MODEM_CONFIG_1_ADDR			= 0x1D,				// Modem PHY config 1
	LORA_REG_MODEM_CONFIG_2_ADDR			= 0x1E,				// Modem PHY config 2
	LORA_REG_SYMB_TIMEOUT_LSB_ADDR			= 0x1F,				// Receiver timeout value
	LORA_REG_PREAMBLE_MSB_ADDR				= 0x20,				// Size of preamble
	LORA_REG_PREAMBLE_LSB_ADDR				= 0x21,
	LORA_REG_PAYLOAD_LENGTH_ADDR			= 0x22,				// LoRa payload length
	LORA_REG_MAX_PAYLOAD_LENGTH_ADDR		= 0x23,				// LoRa maximum payload length
	LORA_REG_HOP_PERIOD_ADDR				= 0x24,				// FHSS Hop period
	LORA_REG_FIFO_RX_BYTE_ADDRESS_ADDR		= 0x25,				// Address of last byte written in FIFO
	LORA_REG_MODEM_CONFIG_3_ADDR			= 0x26,				// Modem PHY config 3
	LORA_REG_FEI_MSB_ADDR					= 0x28,				//Estimated frequency error
	LORA_REG_FEI_MID_ADDR					= 0x29,
	LORA_REG_FEI_LSB_ADDR					= 0x2A,
	LORA_REG_RSSI_WIDEBAND_ADDR				= 0x2C,				// Wideband RSSI measurement
	LORA_REG_LF_FREQ_2_ADDR					= 0x2F,				// Optimize receiver
	LORA_REG_LF_FREQ_1_ADDR					= 0x30,
	LORA_REG_DETECT_OPTIMIZE_ADDR			= 0x31,				// LoRa detection Optimize for SF6
	LORA_REG_INVERT_I_Q_ADDR				= 0x33,				// Invert LoRa I and Q signals
	LORA_REG_HIGH_BW_OPTIMIZE_1_ADDR		= 0x36,				// Sensitivity optimisation for 500 kHz bandwidth
	LORA_REG_DETECTION_THRESHOLD_ADDR		= 0x37,				// LoRa detection threshold for SF6
	LORA_REG_SYNC_WORD_ADDR					= 0x39,				// LoRa Sync Word
	LORA_REG_HIGH_BW_OPTIMIZE_2_ADDR		= 0x3A,				// Sensitivity optimisation for 500 kHz bandwidth
	LORA_REG_INVERT_I_Q_2_ADDR				= 0x3B				// Optimize for inverted IQ

};

enum REG_OP_MODE_FSK_OOK
{
	LONG_RANGE_MODE_FSK_OOK_MODE 		= 0x00,
	LONG_RANGE_MODE_LORA_MODE 			= 0x80,
	MODULATION_TYPE_FSK 				= 0x00,
	MODULATION_TYPE_OOK 				= 0x20,
	LOW_FREQ_MODE_ON_HIGH_FREQ 			= 0x00,
	LOW_FREQ_MODE_ON_LOW_FREQ 			= 0x08,				//below 525MHz
	MODE_SLEEP_MODE 					= 0x00,
	MODE_STDBY_MODE						= 0x01,
	MODE_FSTX_MODE						= 0x02,
	MODE_TX_MODE						= 0x03,
	MODE_FSRX_MODE						= 0x04,
	MODE_RX_MODE						= 0x05
};


enum REG_PA_CONFIG_FSK_OOK
{
	PA_SELECT_RFO_PIN					= 0x00,
	PA_SELECT_PA_BOOST_PIN				= 0x80,
	MAX_POWER_MAX						= 0x70,				//or set up your value in 6-4 bits
	MAX_POWER_MEDIUM					= 0x60,
	OUTPUT_POWER						= 0x0F				//or set up your value in 3-0 bits
};


enum PEG_PA_RAMP_FSK
{
	MODULATION_SHAPING_NO_SHAPING 				= 0x00,
	MODULATION_SHAPING_GAUSSIAN_FILTER_BT_1_0	= 0x20,
	MODULATION_SHAPING_GAUSSIAN_FILTER_BT_0_5	= 0x40,
	MODULATION_SHAPING_GAUSSIAN_FILTER_BT_0_3	= 0x60,
	PA_RAMP_TIME_UP_DOWN_FSK_3_4_MS				= 0x00,
	PA_RAMP_TIME_UP_DOWN_FSK_2_0_MS				= 0x01,
	PA_RAMP_TIME_UP_DOWN_FSK_1_0_MS				= 0x02,
	PA_RAMP_TIME_UP_DOWN_FSK_500_US				= 0x03,
	PA_RAMP_TIME_UP_DOWN_FSK_250_US				= 0x04,
	PA_RAMP_TIME_UP_DOWN_FSK_125_US				= 0x05,
	PA_RAMP_TIME_UP_DOWN_FSK_100_US				= 0x06,
	PA_RAMP_TIME_UP_DOWN_FSK_62_US				= 0x07,
	PA_RAMP_TIME_UP_DOWN_FSK_50_US				= 0x08,
	PA_RAMP_TIME_UP_DOWN_FSK_40_US				= 0x09, 	//default
	PA_RAMP_TIME_UP_DOWN_FSK_31_US				= 0x0A,
	PA_RAMP_TIME_UP_DOWN_FSK_25_US				= 0x0B,
	PA_RAMP_TIME_UP_DOWN_FSK_20_US				= 0x0C,
	PA_RAMP_TIME_UP_DOWN_FSK_15_US				= 0x0D,
	PA_RAMP_TIME_UP_DOWN_FSK_12_US				= 0x0E,
	PA_RAMP_TIME_UP_DOWN_FSK_10_US				= 0x0F,
};

enum REG_OCP_FSK												//over current protection
{
	OCP_ON_DISABLE								= 0x00,
	OCP_ON_ENABLE								= 0x20,
	OCP_TRIM_DEFAULT							= 0x0B,
	OCP_TRIM_MAX_240_MA							= 0x1F
};

enum REG_LNA_FSK
{
	LNA_GAIN_HIGHEST_GAIN						= 0x20,
	LNA_GAIN_HIGHEST_GAIN_MINUS_6_DBM			= 0x40,
	LNA_GAIN_HIGHEST_GAIN_MINUS_12_DBM			= 0x60,
	LNA_GAIN_HIGHEST_GAIN_MINUS_24_DBM			= 0x80,
	LNA_GAIN_HIGHEST_GAIN_MINUS_36_DBM			= 0xA0,
	LNA_GAIN_HIGHEST_GAIN_MINUS_48_DBM			= 0xC0,
	LNA_BOOST_LF_DEFAULT						= 0x00,
	LNA_BOOST_HF_DEFAULT						= 0x00,
	LNA_BOOST_HF_BOOST_ON						= 0x03
};


enum REG_RX_CONFIG_FSK
{
	RESTART_RX_ON_COLLISION_NO_RESTART			= 0x00,
	RESTART_RX_ON_COLLISION_AUTOMATIC_RESTART	= 0x80,
	RESTART_RX_WITHOUT_PLL_LOCK					= 0x40,			//wt
	RESTART_RX_WITH_PLL_LOCK					= 0x20,			//wt
	AFC_AUTO_ON_NO_AFC							= 0x00,
	AFC_AUTO_ON_AFC_PERFORMED					= 0x10,
	AGC_AUTO_ON_LNA_FORCED_LNA_GAIN				= 0x00,
	AGC_AUTO_ON_LNA_CONTROLED_AGC				= 0x08,
	RX_TRIGGER_DEFAULT							= 0x06			//page 60 semtech datasheet

};


enum REG_RSSI_CONFIG_FSK
{
	RSSI_OFFSET_DEFAULT							= 0x00,			//or set your value in 7-3 bits
	RSSI_SMOOTHING_2_SAMPLES					= 0x00,
	RSSI_SMOOTHING_4_SAMPLES					= 0x01,
	RSSI_SMOOTHING_8_SAMPLES					= 0x02,
	RSSI_SMOOTHING_16_SAMPLES					= 0x03,
	RSSI_SMOOTHING_32_SAMPLES					= 0x04,
	RSSI_SMOOTHING_64_SAMPLES					= 0x05,
	RSSI_SMOOTHING_128_SAMPLES					= 0x06,
	RSSI_SMOOTHING_256_SAMPLES					= 0x07
};

enum REG_RX_BW_FSK
{
	RX_BW_MANT_16								= 0x00,			//channel filter bandwidth control
	RX_BW_MANT_20								= 0x08,
	RX_BW_MANT_24								= 0x10,
	RX_BW_EXP_DEFAULT							= 0x05,
	RX_BW_EXP_1									= 0x01
};


enum REG_AFC_BW_FSK
{
	RX_BW_MANT_AFC_DEFAULT						= 0x08,
	RX_BW_EXP_AFC_DEFAULT						= 0x03
};


enum REG_OOK_PEAK
{
	BIT_SYNC_ON_DISABLE							= 0x00,
	BIT_SYNC_ON_ENABLE							= 0x20,
	OOK_THRESHOLD_TYPE_FIXED_THRESHOLD			= 0x00,
	OOK_THRESHOLD_TYPE_PEAK_MODE				= 0x08,
	OOK_THRESHOLD_TYPE_AVERAGE_MODE				= 0x10,
	OOK_PEAK_TRESHOLD_STEP_0_5_DB				= 0x00,
	OOK_PEAK_TRESHOLD_STEP_1_0_DB				= 0x01,
	OOK_PEAK_TRESHOLD_STEP_1_5_DB				= 0x02,
	OOK_PEAK_TRESHOLD_STEP_2_0_DB				= 0x03,
	OOK_PEAK_TRESHOLD_STEP_3_0_DB				= 0x04,
	OOK_PEAK_TRESHOLD_STEP_4_0_DB				= 0x05,
	OOK_PEAK_TRESHOLD_STEP_5_0_DB				= 0x06,
	OOK_PEAK_TRESHOLD_STEP_6_0_DB				= 0x07,
};


enum REG_OOK_AVG
{
	OOK_PEAK_THRESHOLD_DEC_ONCE_PER_CHIP		= 0x00,			//default
	OOK_PEAK_THRESHOLD_DEC_ONCE_EVERY_2_CHIP	= 0x20,
	OOK_PEAK_THRESHOLD_DEC_ONCE_EVERY_4_CHIP	= 0x40,
	OOK_PEAK_THRESHOLD_DEC_ONCE_EVERY_8_CHIP	= 0x60,
	OOK_PEAK_THRESHOLD_DEC_TWICE_EACH_CHIP		= 0x80,
	OOK_PEAK_THRESHOLD_DEC_FOUR_EACH_CHIP		= 0xA0,
	OOK_PEAK_THRESHOLD_DEC_EIGHT_EACH_CHIP		= 0xC0,
	OOK_PEAK_THRESHOLD_DEC_SIXTEEN_EACH_CHIP	= 0xE0,
	OOK_AVERAGE_OFFSET_0_DB						= 0x00,
	OOK_AVERAGE_OFFSET_2_DB						= 0x04,
	OOK_AVERAGE_OFFSET_4_DB						= 0x08,
	OOK_AVERAGE_OFFSET_6_DB						= 0x0C,
	OOK_AVERAGE_THRESHOLD_FILTER_CHIP_RATE_32	= 0x00,
	OOK_AVERAGE_THRESHOLD_FILTER_CHIP_RATE_8	= 0x01,
	OOK_AVERAGE_THRESHOLD_FILTER_CHIP_RATE_4	= 0x02,			//default
	OOK_AVERAGE_THRESHOLD_FILTER_CHIP_RATE_2	= 0x03,
};

enum REG_AFC_FEI
{
	AGC_START									= 0x10,			//wt trigger an AGC sequence when set to 1.
	AFC_CLEAR									= 0x02,			//wc clear afc set in rx mode
	AFC_AUTO_CLEAR_ON_DISABLE					= 0x00,
	AFC_AUTO_CLEAR_ON_ENABLE					= 0x01			//AFC reg is cleared at the beginning of the automatic AFC phase
};

enum REG_PREAMBLE_DETECTOR
{
	PREAMBLE_DETECTOR_ON_DISABLE				= 0x00,
	PREAMBLE_DETECTOR_ON_ENABLE					= 0x80,
	PREAMBLE_DETECTOR_SIZE_1_BYTE				= 0x00,
	PREAMBLE_DETECTOR_SIZE_2_BYTE				= 0x20,			//default
	PREAMBLE_DETECTOR_SIZE_3_BYTE				= 0x40,
	PREAMBLE_DETECTOR_TOLERANCE_DEFAULT			= 0x0A			//Number or chip errors tolerated over PreambleDetectorSize. 4 chips per bit

};


enum REG_OSC
{
	RC_CALIBRATION_START						= 0x08,			//Triggers the calibration of the RC oscillator when set. Always reads 0. RC calibration must be triggered in Standby mode.
	CLK_OUT_FXOSC								= 0x00,
	CLK_OUT_FXOSC_DIV_2							= 0x01,
	CLK_OUT_FXOSC_DIV_4							= 0x02,
	CLK_OUT_FXOSC_DIV_8							= 0x03,
	CLK_OUT_FXOSC_DIV_16						= 0x04,
	CLK_OUT_FXOSC_DIV_32						= 0x05,
	CLK_OUT_RC									= 0x06,
	CLK_OUT_OFF									= 0x07			//default
};


enum REG_SYNC_CONFIG
{
	AUTO_RESTART_RX_MODE_OFF					= 0x00,
	AUTO_RESTART_RX_MODE_ON_W_W_PLL_RL			= 0x40,			//without waiting for the PLL to RE-LOCK
	AUTO_RESTART_RX_MODE_ON_W_PLL_RL			= 0x80,			//wait for the PLL to LOCK
	PREAMBLE_POLARITY_0xAA						= 0x00,
	PREAMBLE_POLARITY_0x55						= 0x20,
	SYNC_OFF									= 0x00,
	SYNC_ON										= 0x10,
	SYNC_SIZE_DEFAULT							= 0x03			//Size of the Sync word:(SyncSize + 1) bytes, (SyncSize) bytes if ioHomeOn=1

};

enum REG_PACKET_CONFIG_1
{
	PACKET_FORMAT_FIXED_LENGTH					= 0x00,
	PACKET_FORMAT_VARIABLE_LENGTH				= 0x80,
	DC_FREE_NONE								= 0x00,
	DC_FREE_MANCHESTER							= 0x20,
	DC_FREE_WHITENING							= 0x40,
	CRC_OFF										= 0x00,
	CRC_ON										= 0x10,
	CRC_AUTO_CLEAR_OFF_CLEAR_FIFO				= 0x00,
	CRC_AUTO_CLEAR_OFF_NOT_CLEAR_FIFO			= 0x08,
	ADDRESS_FILTERING_NONE						= 0x00,
	ADDRESS_FILTERING_NODE_ADDRESS				= 0x02,
	ADDRESS_FILTERING_NODE_BROADCAST_ADDRESS	= 0x04,
	CRC_WHITENING_TYPE_CCITT_CRC				= 0x00,
	CRC_WHITENING_TYPE_IBM_CRC					= 0x01
};


enum REG_PACKET_CONFIG_2
{
	DATA_MODE_CONTINUOUS_MODE					= 0x00,
	DATA_MODE_PACKET_MODE						= 0x40,
	IO_HOME_ON_DISABLE							= 0x00,
	IO_HOME_ON_ENABLE							= 0x20,
	BEACON_ON_DISABLE_DEFAULT					= 0x00,
	BEACON_ON_ENABLE							= 0x08,
	PAYLOAD_LENGTH_DEFAULT						= 0x00			//??? Packet Length Most significant bits ???????
};

enum REG_FIFO_THRESHOLD
{
	TX_START_CONDITION_FIFO_LEVEL				= 0x00,
	TX_START_CONDITION_FIFO_NOT_EMPTY			= 0x80,
	FIFO_THRESHOLD_DEFAULT						= 0x0F			//or set your value
																//Used to trigger FifoLevel interrupt, when:number of bytes in FIFO >= FifoThreshold + 1
};

enum REG_SEQ_CONFIG_1
{
	SEQUENCER_START								= 0x80,			// wt When set to ‘1’, executes the “Start” transition.The sequencer can only be enabled when the chip is in Sleep or Standby mode.
	SEQUENCER_STOP								= 0x40,
	IDLE_MODE_STANDBY_MODE						= 0x00,
	IDLE_MODE_SLEEP_MODE						= 0x20,
	FROM_START_LOW_POWER_SELECTION				= 0x00,
	FROM_START_RECEIVE_STATE					= 0x08,
	FROM_START_TRANSMIT_STATE					= 0x10,
	FROM_START_TRANSMIT_STATE_ON_FIFO_LEVEL		= 0x18,
	LOW_POWER_SELECTION_SEQ_OFF					= 0x00,
	LOW_POWER_SELECTION_IDLE_MODE_STATE			= 0x04,
	FROM_IDLE_TO_TRANSMIT_STATE					= 0x00,
	FROM_IDLE_TO_RECEIVE_STATE					= 0x02,
	FROM_TRANSMIT_TO_LOW_POWER_STATE			= 0x00,
	FROM_TRANSMIT_TO_RECEIVE_STATE				= 0x01
};

enum REG_SEQ_CONFIG_2
{
	FROM_RECEIVE_DEFAULT						= 0x00,
	FROM_RECEIVE_TO_PACKET_RECEIVED_PRDY  		= 0x20,
	FROM_RECEIVE_TO_LOW_POWER_SELECTION_PRDY	= 0x40,
	FROM_RECEIVE_TO_PACKET_RECEIVED_CRC_OK		= 0x60,
	FROM_RECEIVE_TO_SEQ_OFF_RSSI				= 0x80,
	FROM_RECEIVE_TO_SEQ_OFF_SYNC_ADDRESS		= 0xA0,
	FROM_RECEIVE_TO_SEQ_OFF_PREAMBLE_DETECT		= 0xC0,
	FROM_RX_TIMEOUT_TO_RECEIVE					= 0x00,
	FROM_RX_TIMEOUT_TO_TRANSMIT					= 0x08,
	FROM_RX_TIMEOUT_TO_LOW_POWER_SELECTION		= 0x10,
	FROM_RX_TIMEOUT_TO_SEQUENCER_OFF			= 0x18,
	FROM_PACKET_RECEIVED_TO_SEQUENCER_OFF		= 0x00,
	FROM_PACKET_RECEIVED_TO_TRANSMIT			= 0x01,  		//on fifoEmply interrupt
	FROM_PACKET_RECEIVED_TO_LOW_POWER_SELECTION = 0x02,
	FROM_PACKET_RECEIVED_TO_RECEIVE_FS_MODE		= 0x03,			//if frequency was changed
	FROM_PACKET_RECEIVED_TO_RECEIVE				= 0x04
};

enum REG_TIMER_RESOLUTION
{
	TIMER_1_RESOLUTION_TIMER_1_DISABLE			= 0x00,
	TIMER_1_RESOLUTION_64_US					= 0x04,
	TIMER_1_RESOLUTION_4_1_MS					= 0x08,
	TIMER_1_RESOLUTION_262_MS					= 0x0C,
	TIMER_2_RESOLUTION_TIMER_2_DISABLE			= 0x00,
	TIMER_2_RESOLUTION_64_US					= 0x01,
	TIMER_2_RESOLUTION_4_1_MS					= 0x02,
	TIMER_2_RESOLUTION_262_MS					= 0x03
};

enum REG_IMAGE_CALIBRATION
{
	AUTO_IMAGE_CAL_ON_DISABLE					= 0x00,
	AUTO_IMAGE_CAL_ON_ENABLE					= 0x80,
	IMAGE_CALIBRATION_START						= 0x40,			//wt
	TEMPERATURE_THRESHOLD_5_DEGREE				= 0x00,
	TEMPERATURE_THRESHOLD_10_DEGREE				= 0x02,
	TEMPERATURE_THRESHOLD_15_DEGREE				= 0x04,
	TEMPERATURE_THRESHOLD_20_DEGREE				= 0x06,
	TEMPERATURE_MONITOR_OFF_DISABLE				= 0x00,			//with temperature monitoring in all modes
	TEMPERATURE_MONITOR_OFF_ENABLE				= 0x01			//without temperature monitoring
};

enum REG_LOW_BATTERY
{
	LOW_BAT_ON_DISABLE							= 0x00,
	LOW_BAT_ON_ENABLE							= 0x08,
	LOW_BAT_TRIM_THRESHOLD_1_695_VOLT			= 0x00,
	LOW_BAT_TRIM_THRESHOLD_1_764_VOLT			= 0x01,
	LOW_BAT_TRIM_THRESHOLD_1_835_VOLT			= 0x02,			//default
	LOW_BAT_TRIM_THRESHOLD_1_905_VOLT			= 0x03,
	LOW_BAT_TRIM_THRESHOLD_1_976_VOLT			= 0x04,
	LOW_BAT_TRIM_THRESHOLD_2_045_VOLT			= 0x05,
	LOW_BAT_TRIM_THRESHOLD_2_116_VOLT			= 0x06,
	LOW_BAT_TRIM_THRESHOLD_2_185_VOLT			= 0x07,

};


enum REG_IRQ_FLAGS_1
{
	MODE_READY_MASK								= 0x80,			//r
	RX_READY_MASK								= 0x40,			//r
	TX_READY_MASK								= 0x20,			//r
	PLL_LOCK_MASK								= 0x10,			//r
	RSSI_MASK									= 0x08,			//rwc
	TIMEOUT_MASK								= 0x04,			//r
	PREAMBLE_DETECT_MASK						= 0x02,			//rwc
	SYNC_ADDRESS_MATCH_MASK						= 0x01			//rwc
};

enum REG_IRQ_FLAGS_2
{
	FIFO_FULL_MASK								= 0x80,			//r
	FIFO_EMPTY_MASK								= 0x40,			//r
	FIFO_LEVEL_MASK								= 0x20,			//r
	FIFO_OVERRUN_MASK							= 0x10,			//rwc
	PACKET_SENT_MASK							= 0x08,			//r
	PAYLOAD_READY_MASK							= 0x04,			//r
	CRC_OK_MASK									= 0x02,			//r
	LOW_BAT_MASK								= 0x01			//rwc

};


enum REG_DIO_MAPPING_1_PACKET_MODE
{
	DIO_0_RX_PAYLOAD_READY_TX_PACKET_SENT		= 0x00,
	DIO_0_CRC_OK								= 0x40,
	DIO_0_TEMPERATURE_CHANGE					= 0xC0,
	DIO_1_FIFO_LEVEL							= 0x00,
	DIO_1_FIFO_EMPTY							= 0x10,
	DIO_1_FIFO_FULL								= 0x20,
	DIO_2_FIFO_FULL								= 0x00,
	DIO_2_RX_READY								= 0x04,
	DIO_2_RX_TIMEOUT_TX_FIFO_FULL				= 0x08,
	DIO_2_RX_SYNC_ADDRESS_TX_FIFO_FULL			= 0x0C,
	DIO_3_FIFO_EMPTY							= 0x00,
	DIO_3_TX_READY								= 0x01

};
/*
enum REG_DIO_MAPPING_1_CONTINUOUS_MODE
{
	//TODO	fill for this mode
};
*/
enum REG_DIO_MAPPING_1_LORA_MODE
{
	//TODO	fill for this mode
	L_DIO_0_RX_DONE								= (0 << 6),
	L_DIO_0_TX_DONE								= (1 << 6),
	L_DIO_0_CAD_DONE							= (2 << 6),
	L_DIO_1_RX_TIMEOUT							= (0 << 4),
	L_DIO_1_FHSS_CHANGE_CHANNEL					= (1 << 4),
	L_DIO_1_CAD_DETECTED						= (2 << 4),
	L_DIO_2_FHSS_CHANGE_CHANNEL					= (0 << 2),
	L_DIO_3_CAD_DONE							= (0),
	L_DIO_3_VALID_HEADER						= (1),
	L_DIO_3_PAYLOAD_CRC_ERROR					= (2),
	L_DIO_4_CAD_DETECTED						= (0 << 6),
	L_DIO_4_PLL_LOCK							= (1 << 6),
	L_DIO_5_CLK_OUT								= (2 << 4)
};

enum REG_DIO_MAPPING_2_PACKET_MODE
{
	DIO_4_TEMP_CHANGE_LOW_BAT					= 0x00,
	DIO_4_PLL_LOCK								= 0x40,
	DIO_4_RX_TIME_OUT							= 0x80,
	DIO_4_RX_RSSI_PREAMBLE_DETECT				= 0xC0,
	DIO_5_CLK_OUT								= 0x00,
	DIO_5_PLL_LOCK								= 0x10,
	DIO_5_DATA									= 0x20,
	DIO_5_MODE_READY							= 0x30,
	MAP_PREAMBLE_DETECT_RSSI_INTERUPT			= 0x00,
	MAP_PREAMBLE_DETECT_PREAMBLE_DETECT_INTERUPT= 0x01
};
/*
enum REG_DIO_MAPPING_2_CONTINUOUS_MODE
{
	//TODO fill for this mode
};

enum REG_DIO_MAPPING_2_LORA_MODE
{
	//TODO fill for this mode
};*/


enum REG_PA_DAC
{
	PA_DAC_DEFAULT								= 0x04,
	PA_DAC_20DBM_PA_BOOST						= 0x07
};


#define REG_BITRATE_MSB_DEFAULT					(0x1A)
#define REG_BITRATE_LSB_DEFAULT					(0x0B)
#define REG_FDEV_MSB_DEFAULT					(0x00)
#define REG_FDEV_LSB_DEFAULT					(0x52)
#define REG_FRF_MSB_DEFAULT						(0x6C)
#define REG_FRF_MID_DEFAULT						(0x80)
#define REG_FRF_LSB_DEFAULT						(0x00)
#define REG_RSSI_COLLISION_DEFAULT				(0x0A)
#define REG_RSSI_THRESHOLD_DEFAULT				(0xFF)
#define REG_OOK_FIXED_THRESHOLD_DEFAULT			(0x0C)
#define REG_AFC_MSB_DEFAULT						(0x00)
#define REG_AFC_LSB_DEFAULT						(0x00)
//
#define REG_FEI_MSB_DEFAULT						(0x00) //????
#define	REG_FEI_LSB_DEFAULT						(0x00) //????
#define REG_RX_TIMEOUT_1_DEFAULT				(0x00)	//rx RSSI disabled
#define REG_RX_TIMEOUT_2_DEFAULT				(0x00)  //rx preamble disabled
#define REG_RX_TIMEOUT_3_DEFAULT				(0x00)  //rx signal sync disable
#define REG_RX_DELAY_DEFAULT					(0x00)
#define REG_PREAMBLE_MSB_DEFAULT				(0x00)	//preamble size
#define REG_PREAMBLE_LSB_DEFAULT				(0x03)
#define	REG_SYNC_VALUE_1_DEFAULT				(0x01)
#define REG_SYNC_VALUE_2_DEFAULT				(0x01)
#define REG_SYNC_VALUE_3_DEFAULT				(0x01)
#define REG_SYNC_VALUE_4_DEFAULT				(0x01)
#define REG_SYNC_VALUE_5_DEFAULT				(0x01)
#define	REG_SYNC_VALUE_6_DEFAULT				(0x01)
#define REG_SYNC_VALUE_7_DEFAULT				(0x01)
#define REG_SYNC_VALUE_8_DEFAULT				(0x01)
#define	REG_PAYLOAD_LENGTH_DEFAULT				(0x40)
#define REG_NODE_ADDRESS_DEFAULT				(0x00)
#define REG_BROADCAST_ADDRESS_DEFAULT			(0x00)
#define REG_TIMER_1_COEFFICIENT_DEFAULT			(0xF5)
#define	REG_TIMER_2_COEFFICIENT_DEFAULT			(0x20)
#define REG_BITRATE_FRAC_DEFAULT				(0x00)



//Low Frequency Additional Registers
#define REG_AGC_REFERENCE_DEFAULT				(0x19)
#define REG_AGC_THRESHOLD_1_DEFAULT				(0x0C)
#define REG_AGC_THRESHOLD_2_DEFAULT				(0x4B)
#define REG_AGC_THRESHOLD_3_DEFAULT				(0xCC)
#define REG_PLL_LF_DEFAULT						(0xD0)
//High Frequency Additional Registers
/*
 * TODO add here for high frequency default config
 */

enum REG_OP_MODE_LORA
{
	L_ACCESS_SHARED_REG					= 0x40,
	L_MODE_RX_CONTINUOUS_MODE			= 0x05,
	L_MODE_RX_SINGLE_MODE				= 0x06,
	L_MODE_CAD_MODE						= 0x07				//channel activity detection
};

enum REG_IRQ_FLAGS_MASK_LORA
{
	L_RX_TIMEOUT_MASK 					= 0x80,
	L_RX_DONE_MASK						= 0x40,
	L_PAYLOAD_CRC_ERROR_MASK			= 0x20,
	L_VALID_HEADER_MASK					= 0x10,
	L_TX_DONE_MASK						= 0x08,
	L_CAD_DONE_MASK						= 0x04,
	L_FHSS_CHANGE_CHANNEL_MASK			= 0x02,
	L_CAD_DETECTED_MASK					= 0x01
};

enum REG_MODEM_CONFIG_1_LORA
{
	L_BW_7_8_KHZ						= 0x00,
	L_BW_10_4_KHZ						= 0x10,
	L_BW_15_6_KHZ						= 0x20,
	L_BW_20_8_KHZ						= 0x30,
	L_BW_31_25_KHZ						= 0x40,
	L_BW_41_7_KHZ						= 0x50,
	L_BW_62_5_KHZ						= 0x60,
	L_BW_125_KHZ						= 0x70,
	L_BW_250_KHZ						= 0x80,
	L_BW_500_KHZ						= 0x90,
	L_CODING_RATE_4_5					= 0x02,
	L_CODING_RATE_4_6					= 0x04,
	L_CODING_RATE_4_7					= 0x06,
	L_CODING_RATE_4_8					= 0x08,
	L_IMPLICIT_HEADER_MODE_ON_EXPLICIT	= 0x00,
	L_IMPLICIT_HEADER_MODE_ON_IMPLICIT	= 0x01
};

enum REG_MODEM_CONFIG_2_LORA
{
	L_SPREADING_FACTOR_6					= (6<<4),
	L_SPREADING_FACTOR_7					= (7<<4),
	L_SPREADING_FACTOR_8					= (8<<4),
	L_SPREADING_FACTOR_9					= (9<<4),
	L_SPREADING_FACTOR_10					= (10<<4),
	L_SPREADING_FACTOR_11					= (11<<4),
	L_SPREADING_FACTOR_12					= (12<<4),
	L_TX_CONTINUOUS_MODE_NORMAL_MODE		= (0<<3),
	L_TX_CONTINUOUS_MODE_CONTINUOUS_MODE	= (1<<3),
	L_RX_PAYLOAD_CRC_ON_DISABLE				= (0<<2),
	L_RX_PAYLOAD_CRC_ON_ENABLE				= (1<<2)
};



enum REG_MODEM_CONFIG_3_LORA
{
	L_LOW_DATA_RATE_OPTIMIZE_DISABLE		= (0<<3),
	L_LOW_DATA_RATE_OPTIMIZE_ENABLE			= (1<<3),
	L_AGC_AUTO_ON_LNA_GAIN					= (0<<2),
	L_AGC_AUTO_ON_LNA_AGC_LOOP				= (1<<2)
};

enum REG_DETECT_OPTIMIZE_LORA
{
	L_AUTOMATIC_IF_ON_ERRATA				= (0<<7),
	L_DETECT_OPTIMIZE_SF7_SF12				= (0x03),
	L_DETECT_OPTIMIZE_SF6					= (0x05)
};

#define FIFO_ADDR_PTR_LORA_DEFAULT				(0x00)
#define FIFO_TX_BASE_ADDR_LORA_DEFAULT			(0x80)
#define FIFO_RX_BASE_ADDR_LORA_DEFAULT			(0x00)
#define SYMBOL_TIMEOUT_LSB_LORA_DEFAULT			(0x64) //??
#define PREAMBLE_LENGTH_MSB_LORA_DEFAULT		(0x00)
#define PREAMBLE_LENGTH_LSB_LORA_DEFAULT		(0x08)
#define PAYLOAD_LENGTH_LORA_DEFAULT				(0x01)
#define PAYLOAD_MAX_LENGTH_LORA_DEFAULT			(0xFF)
#define FREQ_HOPPING_PERIOD_LORA_DEFAULT		(0x00)
#define IF_FREQ_2_ERRATA_FIX_BW_125_KHZ_LORA	(0x40)
#define IF_FREQ_1_ERRATA_FIX_BW_125_KHZ_LORA	(0x00)
#define IF_FREQ_2_ERRATA_FIX_BW_7_8_KHZ_LORA	(0x48)
#define IF_FREQ_1_ERRATA_FIX_BW_7_8_KHZ_LORA	(0x00)
#define INVERT_IQ_LORA_DEFAULT					(0x26)
#define HW_BW_OPTIMIZE_1_LORA_DEFAULT			(0x03)
#define HW_DETECTION_THRESHOLD_LORA_DEFAULT		(0x0A)
#define SYNC_WORD_LORA_DEFAULT					(0x12)
#define HW_BW_OPTIMIZE_2_LORA_DEFAULT			(0x65) //do not config
#define INVERT_IQ_2_LORA_DEFAULT				(0x1D) //for not inverted IQ




#endif /* SX1276_77_78_79_SX1276_77_78_79_REGISTERS_H_ */
