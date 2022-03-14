#include "stm32f1xx_ll_utils.h"

#define TX_RX 0 	//TX - 0, RX - 1

void 		SX1276_WriteBurst( uint8_t addr, char *buff, uint8_t size );
void SX1276_Init(void);
void SX1276_RES(void);
uint8_t SX1276_WriteSingle(uint8_t command, uint8_t value);
//uint8_t SX1276_WriteSingle(uint8_t command,uint8_t value);
