

/* 包含头文件 ----------------------------------------------------------------*/
#include "spi/bsp_spi.h"
#include "usart/bsp_usartx.h"
#include "led/bsp_led.h"
#include "DRV8825/drv8825.h"
#include "StepMotor/bsp_StepMotor.h"
#include "i2c/bsp_EEPROM.h"
#include "key/bsp_key.h"
#include "hextodec/hextodec.h"
#include "lamp/bsp_lamp.h"
#include "GeneralTIM/bsp_GeneralTIM.h"
#include "DS18B20/bsp_DS18B20.h"
#include "i2c_slave/bsp_I2C.h"

/* 私有类型定义 --------------------------------------------------------------*/
/* 私有宏定义 ----------------------------------------------------------------*/                              
/* 私有变量 ------------------------------------------------------------------*/
SPI_HandleTypeDef hspi_SPI;

/* 扩展变量 ------------------------------------------------------------------*/
/* 私有函数原形 --------------------------------------------------------------*/
/* 函数体 --------------------------------------------------------------------*/
extern uint8_t SPI_aRxBuffer[7];
extern uint8_t SPI_aTxBuffer[6];
extern uint8_t SPI_RX_DATA;
extern uint8_t repcdata[3]; 
extern __IO uint32_t  home_position; 
extern uint8_t  SPI_RX_FLAG;
extern __IO uint8_t Brightness;
/*********************************************************************************
  * 函数功能: 串行FLASH初始化
  * 输入参数: huart：串口句柄类型指针
  * 返 回 值: 无
  * 说    明: 该函数被HAL库内部调用
***********************************************************************************/
void SPIx_Init(void)
{
  hspi_SPI.Instance = SPIx;
  hspi_SPI.Init.Mode = SPI_MODE_SLAVE;  //从
  hspi_SPI.Init.Direction = SPI_DIRECTION_2LINES;
  hspi_SPI.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi_SPI.Init.CLKPolarity = SPI_POLARITY_LOW; 
  hspi_SPI.Init.CLKPhase =SPI_PHASE_1EDGE; //SPI_PHASE_1EDGE;    //wt.edit 2018.03.13
  hspi_SPI.Init.NSS =SPI_NSS_SOFT; //wt.edit 2018.03.12
  hspi_SPI.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi_SPI.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi_SPI.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi_SPI.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi_SPI.Init.CRCPolynomial = 7;
  HAL_SPI_Init(&hspi_SPI);
  __HAL_SPI_ENABLE(&hspi_SPI);
}

/**
  * 函数功能: SPI外设系统级初始化
  * 输入参数: hspi：SPI句柄类型指针
  * 返 回 值: 无
  * 说    明: 该函数被HAL库内部调用
  */
void HAL_SPI_MspInit(SPI_HandleTypeDef* hspi)
{
  GPIO_InitTypeDef GPIO_InitStruct;
  if(hspi->Instance==SPIx)
  {
    /* SPI外设时钟使能 */
    SPIx_RCC_CLK_ENABLE();
    /* GPIO外设时钟使能 */
    SPIx_GPIO_ClK_ENABLE();
    SPIx_CS_CLK_ENABLE();
    
    /* Disable the Serial Wire Jtag Debug Port SWJ-DP */
   // __HAL_AFIO_REMAP_SWJ_DISABLE();  //不能用SWD 升级软件，此句屏蔽了SWD升级口 wt.2018.03.12
    
    /**SPI3 GPIO Configuration    
    PF11     ------> SPI3_NSS
    PB3      ------> SPI3_SCK
    PB4      ------> SPI3_MISO
    PB5      ------> SPI3_MOSI 
    */
    GPIO_InitStruct.Pin = SPIx_SCK_PIN|SPIx_MOSI_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(SPIx_GPIO_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = SPIx_MISO_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(SPIx_GPIO_PORT, &GPIO_InitStruct);
    
    HAL_GPIO_WritePin(SPIx_CS_PORT, SPIx_CS_PIN, GPIO_PIN_SET);
    GPIO_InitStruct.Pin = SPIx_CS_PIN;
    //GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;  //GPIO_MODE_OUTPUT_PP;wt.edit 2018.03.12, 从机
	GPIO_InitStruct.Pull=GPIO_PULLUP;      //wt.edit 2018.03.12
	//GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING ;//GPIO_MODE_IT_RISING_FALLING  ; //wt.edit 2018 
    HAL_GPIO_Init(SPIx_CS_PORT, &GPIO_InitStruct);
        
    HAL_NVIC_SetPriority(SPI1_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(SPI1_IRQn);      //wt.edit
 //   HAL_NVIC_SetPriority(EXTI4_IRQn, 1, 0); //停止按键
 //   HAL_NVIC_EnableIRQ(EXTI4_IRQHandler);
  }
}

/**
  * 函数功能: SPI外设系统级反初始化
  * 输入参数: hspi：SPI句柄类型指针
  * 返 回 值: 无
  * 说    明: 该函数被HAL库内部调用
  */
void HAL_SPI_MspDeInit(SPI_HandleTypeDef* hspi)
{

  if(hspi->Instance==SPIx)
  {
    /* SPI外设时钟禁用 */
    SPIx_RCC_CLK_DISABLE();
  
    /**SPI3 GPIO Configuration    
    PF11     ------> SPI3_NSS ----PA4
    PB3      ------> SPI3_SCK  ----PA5
    PB4      ------> SPI3_MISO ----PA6
    PB5      ------> SPI3_MOSI ----PA7
    */
    HAL_GPIO_DeInit(SPIx_GPIO_PORT, SPIx_SCK_PIN|SPIx_MISO_PIN|SPIx_MOSI_PIN);
    HAL_GPIO_DeInit(SPIx_CS_PORT, SPIx_CS_PIN);
    HAL_NVIC_DisableIRQ(SPI1_IRQn);
  }
  
} 



//SPI1 读写一个字节
//TxData:要写入的字节
//返回值:读取到的字节
uint8_t SPI1_ReadWriteByte(uint8_t TxData)
{
    uint8_t Rxdata;
    HAL_SPI_TransmitReceive(&hspi_SPI,&TxData,&Rxdata,1, 0xffff);       
 	return Rxdata;          		    //返回收到的数据		
}
