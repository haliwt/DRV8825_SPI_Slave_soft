#ifndef __BSP_SPIx_H__
#define __BSP_SPIx_H__

/* 包含头文件 ----------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* 类型定义 ------------------------------------------------------------------*/
/* 宏定义 --------------------------------------------------------------------*/
#define SPIx                                  SPI1//SPI3  WT.edit 2018.03.12
#define SPIx_RCC_CLK_ENABLE()                 __HAL_RCC_SPI1_CLK_ENABLE()
#define SPIx_RCC_CLK_DISABLE()                __HAL_RCC_SPI1_CLK_DISABLE()

#define SPIx_GPIO_ClK_ENABLE()                __HAL_RCC_GPIOA_CLK_ENABLE() 
#define SPIx_GPIO_PORT                        GPIOA

#define SPIx_SCK_PIN                          GPIO_PIN_5
#define SPIx_MISO_PIN                         GPIO_PIN_6
#define SPIx_MOSI_PIN                         GPIO_PIN_7


#define SPIx_CS_CLK_ENABLE()                  __HAL_RCC_GPIOA_CLK_ENABLE()    
#define SPIx_CS_PORT                          GPIOA
#define SPIx_CS_PIN                           GPIO_PIN_4
#define SPIx_CS_ENABLE()                      HAL_GPIO_WritePin(SPIx_CS_PORT, SPIx_CS_PIN, GPIO_PIN_RESET)
#define SPIx_CS_DISABLE()                     HAL_GPIO_WritePin(SPIx_CS_PORT, SPIx_CS_PIN, GPIO_PIN_SET)
#define SPIx_CS_IRQn                          EXTI4_IRQn 
#define SPIx_EXTI_IRQHandler                  EXTI4_IRQHandler
                        
/* 扩展变量 ------------------------------------------------------------------*/
extern SPI_HandleTypeDef hspi_SPI;

/* 函数声明 ------------------------------------------------------------------*/

void SPIx_Init(void);
uint8_t SPI1_ReadWriteByte(uint8_t TxData);
void SPI1_RX_FUN(void);

#endif  /* __BSP_SPIx_H__ */

