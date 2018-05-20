/********************************************************************************
*
*文件的功能：I2C主机，发送数据给 I2C从机
*
*
*
******************************************************************************/
/* 包含头文件 ----------------------------------------------------------------*/
#include "i2c_slave/bsp_I2C.h"


I2C_HandleTypeDef I2cHandle;

uint8_t i2c_tx_buffer[3]={0x00};
/*********************************************************
  * 函数功能: I2C外设初始化
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明：无
**********************************************************/
void MX_I2C_EEPROM_Init(void)
{
  I2cHandle.Instance             = I2Cx;
  I2cHandle.Init.ClockSpeed      = I2C_SPEEDCLOCK;
  I2cHandle.Init.DutyCycle       = I2C_DUTYCYCLE;
  I2cHandle.Init.OwnAddress1     = 0x30F;
  I2cHandle.Init.AddressingMode  = I2C_ADDRESSINGMODE_7BIT;
  I2cHandle.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  I2cHandle.Init.OwnAddress2     = 0xFF;
  I2cHandle.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  I2cHandle.Init.NoStretchMode   = I2C_NOSTRETCH_DISABLE;
  HAL_I2C_Init(&I2cHandle);
}

/**
  * 函数功能: I2C外设硬件初始化配置
  * 输入参数: hi2c：I2C句柄类型指针
  * 返 回 值: 无
  * 说    明: 该函数被HAL库内部调用
  */
void HAL_I2C_MspInit(I2C_HandleTypeDef* hi2c)
{
  GPIO_InitTypeDef GPIO_InitStruct;
  if(hi2c->Instance==I2Cx)
  {  
    /* 使能外设时钟 */
    I2C_RCC_CLK_ENABLE();        
    I2C_GPIO_CLK_ENABLE();
    
    /**I2C1 GPIO Configuration    
    PB6     ------> I2C1_SCL
    PB7     ------> I2C1_SDA 
    */
    GPIO_InitStruct.Pin = I2C_SCL_PIN|I2C_SDA_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(I2C_GPIO_PORT, &GPIO_InitStruct);
  }
}

/**
  * 函数功能: I2C外设硬件反初始化配置
  * 输入参数: hi2c：I2C句柄类型指针
  * 返 回 值: 无
  * 说    明: 该函数被HAL库内部调用
  */
void HAL_I2C_MspDeInit(I2C_HandleTypeDef* hi2c)
{
  if(hi2c->Instance==I2Cx)
  {
    /* 禁用外设时钟 */
    I2C_GPIO_CLK_DISABLE();
  
    /**I2C1 GPIO Configuration    
    PB6     ------> I2C1_SCL
    PB7     ------> I2C1_SDA 
    */
    HAL_GPIO_DeInit(I2C_GPIO_PORT, I2C_SCL_PIN|I2C_SDA_PIN);
  }
}
/************************************************
*
*函数名称:
*函数功能：发送数据给
*参数：无
*返回值：无
*
*************************************************/
void I2C_MASTER_TX_DATA(void)
{
    uint8_t i=0,pdata;
	//printf("i2c_master_tx_data()\n");
	for(i=0;i<3;i++)
	{
	  pdata=i2c_tx_buffer[i];
	  if(HAL_I2C_Master_Transmit(&I2cHandle,(uint16_t)I2C_ADDRESS,&pdata,3,0xFFFF)==HAL_OK)
	  {
		printf("I2C send data is successd \n");
		HAL_Delay(5);
	  }
	  else printf("i2c_tx is error \n");
	}
}

