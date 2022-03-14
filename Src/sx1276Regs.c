#include "sx1276Regs.h"
#include "sx1276Regs-Fsk.h"
#include "sx1276Regs-LoRa.h"
#include "main.h"

#define 	WRITE_SINGLE     				0x80
#define 	READ_SINGLE     				0x00


const unsigned char LoRa_config[]={			// 868MHz, SF12, 125kHz, 300bps, MaxPower, OcpOn, 9Byte info 
	
	REG_LR_OPMODE,					0x80, 									//Lora mode, HF, Sleep
	REG_LR_FRFMSB, 					0xD9,									//Freq = 868MHz
	REG_LR_FRFMID, 					0x00,									//Freq = 868MHz
	REG_LR_FRFLSB, 					0x04,									//Freq = 868MHz
	REG_LR_PACONFIG, 				11,//0b11111111,								//Max power
	REG_LR_OCP,						0x1F,									//OCP-on, Current 130 mA
	REG_LR_MODEMCONFIG1,			11,//0b01110010,								//125kHz,4/5 Coding Rate/ Explicit
	REG_LR_MODEMCONFIG2, 			0xC2,									//
	REG_LR_PAYLOADLENGTH,			0x10,									//16 bytes // Standart -1

    
	// -------------------Standart parameters	-----------------------	
/*
	REG_LR_IRQFLAGSMASK, 			0x48,									//Tx_Complete IRQ, RX_Complete IRQ	
	REG_LR_PARAMP,					0x09,									//Standart 40us
	REG_LR_FIFOADDRPTR,			0x00,									//Standart
	REG_LR_FIFOTXBASEADDR, 	0x80,									//Standart
	REG_LR_FIFORXBASEADDR, 	0x00,									//Standart
	REG_LR_LNA,							0b00100000,						//Standart
	REG_LR_SYMBTIMEOUTLSB,	0x64, 								//Standart
	REG_LR_PREAMBLEMSB,			0x00,									//Standart
	REG_LR_PREAMBLELSB,			0x08,									//Standart
	REG_LR_PAYLOADMAXLENGTH,0xFF,									//Standart
	REG_LR_HOPPERIOD,				0x00,									//Standart  
*/	
};


void SX1276_Init(void)	{//CC1101_Init
uint8_t qnt,i_temp=0;
qnt=sizeof (LoRa_config);
	
SX1276_RES();	
while (i_temp < qnt)
{
	SX1276_WriteSingle(LoRa_config[i_temp],LoRa_config[i_temp+1]);
	i_temp+=2;	
}
SX1276_WriteSingle(REG_LR_OPMODE, RFLR_OPMODE_SLEEP|0x80 );
	if(TX_RX){
		SX1276_WriteSingle(REG_LR_DIOMAPPING1,	RFLR_DIOMAPPING1_DIO0_00);
		SX1276_WriteSingle(REG_LR_SYNCWORD,0x12);	
		SX1276_WriteSingle(REG_LR_OPMODE,RFLR_OPMODE_STANDBY|0x80);
		SX1276_WriteSingle(REG_LR_FIFOADDRPTR,0x00);
		SX1276_WriteSingle(REG_LR_OPMODE,RFLR_OPMODE_RECEIVER|0x80);
		
	}



}


void SX1276_RES(void){
	LL_GPIO_ResetOutputPin(Reset_sx1276_GPIO_Port, Reset_sx1276_Pin);
	LL_mDelay(10);
	LL_GPIO_SetOutputPin(Reset_sx1276_GPIO_Port, Reset_sx1276_Pin);
	LL_mDelay(10);
	
}






uint8_t SX1276_WriteSingle(uint8_t command, uint8_t value) 													{//WriteSingle
uint8_t m[2];
	m[0] = WRITE_SINGLE | command;
	m[1] = value;
	while(((DMA1_Channel3->CCR) & DMA_CCR_EN)){};
	LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_3, 2);
	LL_DMA_ConfigAddresses(DMA1, LL_DMA_CHANNEL_3, (uint32_t)&m,  LL_SPI_DMA_GetRegAddr(SPI1), LL_DMA_GetDataTransferDirection(DMA1, LL_DMA_CHANNEL_3));
	LL_GPIO_ResetOutputPin(NSS_SX1276_GPIO_Port, NSS_SX1276_Pin);//CS_LO();						
	LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_3);
uint8_t temp;



//while (!(SPI1->SR & SPI_SR_TXE)){};  
//SPI1->DR = (WRITE_SINGLE | command);
//while (SPI1->SR & SPI_SR_BSY){};
//while (!(SPI1->SR & SPI_SR_RXNE)){}; 
//temp=SPI1->DR;
//	
//while (!(SPI1->SR & SPI_SR_TXE)){};	 
//SPI1->DR = value;
//while (SPI1->SR & SPI_SR_BSY){};
//while (!(SPI1->SR & SPI_SR_RXNE)){}; 
//temp = SPI1->DR;	
return temp;	
}

void 		SX1276_WriteBurst( uint8_t addr, char *buff, uint8_t size )							{//WriteBurst

	while(((DMA1_Channel3->CCR) & DMA_CCR_EN)){};
	LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_3, size);
	LL_DMA_ConfigAddresses(DMA1, LL_DMA_CHANNEL_3, (uint32_t)buff,  LL_SPI_DMA_GetRegAddr(SPI1), LL_DMA_GetDataTransferDirection(DMA1, LL_DMA_CHANNEL_3));
	LL_GPIO_ResetOutputPin(NSS_SX1276_GPIO_Port, NSS_SX1276_Pin);//CS_LO();						
	LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_3);
}
