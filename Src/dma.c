/**
  ******************************************************************************
  * File Name          : dma.c
  * Description        : This file provides code for the configuration
  *                      of all the requested memory to memory DMA transfers.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "dma.h"

/* USER CODE BEGIN 0 */
extern uint8_t buff_LORA_OUT;
extern uint8_t buff_LORA_IN;
/* USER CODE END 0 */

/*----------------------------------------------------------------------------*/
/* Configure DMA                                                              */
/*----------------------------------------------------------------------------*/

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/** 
  * Enable DMA controller clock
  */
void MX_DMA_Init(void) 
{

  /* Init with LL driver */
  /* DMA controller clock enable */
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);

  /* DMA interrupt init */
  /* DMA1_Channel2_IRQn interrupt configuration */
  NVIC_SetPriority(DMA1_Channel2_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(DMA1_Channel2_IRQn);
  /* DMA1_Channel3_IRQn interrupt configuration */
  NVIC_SetPriority(DMA1_Channel3_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(DMA1_Channel3_IRQn);

}

/* USER CODE BEGIN 2 */

void DMA_USER_INIT(void){
		//Настройка DMA дальномера
	LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_2);
	LL_DMA_ClearFlag_TC2(DMA1);
	LL_DMA_ClearFlag_TE2(DMA1);
	LL_SPI_EnableDMAReq_RX(SPI1);
	LL_DMA_EnableIT_TC(DMA1, LL_DMA_CHANNEL_2);
	LL_DMA_ConfigAddresses(DMA1, LL_DMA_CHANNEL_2, LL_SPI_DMA_GetRegAddr(SPI1), (uint32_t)&buff_LORA_OUT,  LL_DMA_GetDataTransferDirection(DMA1, LL_DMA_CHANNEL_2));
	
	
	LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_3);
	LL_DMA_ClearFlag_TC3(DMA1);
	LL_DMA_ClearFlag_TE3(DMA1);
	LL_SPI_EnableDMAReq_TX(SPI1);
	LL_DMA_EnableIT_TC(DMA1, LL_DMA_CHANNEL_3);
	//LL_DMA_ConfigAddresses(DMA1, LL_DMA_CHANNEL_3, LL_SPI_DMA_GetRegAddr(SPI1), (uint32_t)&buff_LORA_IN,  LL_DMA_GetDataTransferDirection(DMA1, LL_DMA_CHANNEL_3));
	
	
	
	
	
}
/* USER CODE END 2 */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
