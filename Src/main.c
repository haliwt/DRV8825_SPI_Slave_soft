
/* °üº¬Í·ÎÄ¼þ ----------------------------------------------------------------*/
#include "stm32f1xx_hal.h"
#include "usart/bsp_usartx.h"
#include "string.h"
#include "led/bsp_led.h"
#include "StepMotor/bsp_StepMotor.h"
#include "key/bsp_key.h"
#include "AdvancedTIM/bsp_AdvancedTIM.h"
#include "GeneralTIM/bsp_GeneralTIM.h"
#include "drv8825/drv8825.h"
#include "i2c/bsp_EEPROM.h"
#include <math.h>
#include "GeneralTIM/bsp_GeneralTIM.h"
#include "lamp/bsp_lamp.h"
#include "ds18b20/bsp_ds18b20.h"
#include "spi/bsp_spi.h"
#include "i2c_slave/bsp_I2C.h"
#include "a2_fun/a2_fun.h"
#include "CAN/bsp_CAN.h"

#define SENDBUFF_SIZE             100 // ´®¿ÚDMA·¢ËÍ»º³åÇø´óÐ¡
#define STEPMOTOR_MICRO_STEP      32  // ²½½øµç»úÇý¶¯Æ÷Ï¸·Ö£¬±ØÐëÓëÇý¶¯Æ÷Êµ¼ÊÉèÖÃ¶ÔÓ¦

/* Ë½ÓÐ±äÁ¿ ------------------------------------------------------------------*/
uint8_t aRxBuffer[256];                      // ½ÓÊÕÊý¾Ý 
uint8_t aTxBuffer[SENDBUFF_SIZE];       // ´®¿ÚDMA·¢ËÍ»º³åÇø
uint8_t repcdata[3]={0x00,0x00,0x00};                    //´ÓÉÏÎ»»ú½ÓÊÕµ½µÄ3¸ö×Ö½ÚÊý¾Ý¡£
extern uint8_t i2c_tx_buffer[3];
/* À©Õ¹±äÁ¿ ------------------------------------------------------------------*/
extern __IO uint16_t Toggle_Pulse; /* ²½½øµç»úËÙ¶È¿ØÖÆ£¬¿Éµ÷½Ú·¶Î§Îª 650 -- 3500 £¬ÖµÔ½Ð¡ËÙ¶ÈÔ½¿ì */
__IO uint32_t pulse_count; /*  Âö³å¼ÆÊý£¬Ò»¸öÍêÕûµÄÂö³å»áÔö¼Ó2 */
extern  __IO uint16_t  home_position    ; 
extern __IO uint8_t Brightness;   
extern __IO uint8_t save_flag;
__IO uint8_t re_intrrupt_flag=0; //´ÓÉÏÎ»»ú£¬½ÓÊÕÐÅºÅ±êÖ¾Î»£¬½øÈëÖÐ¶Ï±êÖ¾Î»
__IO uint16_t judge_data;    //½ÓÊÕµ½Êý¾ÝµÄ¹¦ÄÜÂëÊý¾Ý¡£
extern __IO uint8_t key_stop;
__IO uint8_t PB8_flag=0;
extern __IO uint8_t stop_key_flag;
uint8_t SPI_aRxBuffer[100];
uint8_t SPI_aTxBuffer[6];
uint8_t i=0;
uint8_t  SPI_RX_FLAG=0;
uint8_t SPI_RX_DATA=0;
uint8_t I2C_TX_FLAG=0;
uint8_t I2C_TX_DATA=0;
extern __IO uint8_t stop_flag; //¿ª»ú±êÖ¾Î»
extern __IO uint8_t END_STOP_FLAG;  //Âí´ïÔËÐÐµ½ÖÕµã£¬Í£Ö¹±êÖ¾Î»
extern __IO uint8_t A2_RX_STOP;
extern __IO uint8_t A2_ReadPulse; //¶ÁÈ¡µÚ¶þ¸öÂí´ïµÄÂö³åÊý±êÖ¾Î»
extern __IO uint8_t END_A2_Read_Pulse;
__IO uint8_t A1_ReadData_FLAG=0;  //A1¶ÁÈ¡A2Âí´ï±êÖ¾Î»¡£
extern __IO uint8_t END_A2_ReadData_FLAG;  //???????
__IO uint8_t A1_ReadData_Stop=0;  //Âí´ï¶ÁÈ¡ËÙ¶È£¬µÈÓÚÁã¡£
__IO uint8_t TX_Times=0;      //Âí´ï2¶ÁÈ¡ÊµÊ±·¢ËÍµÄÊý¾Ý´ÎÊý£¬Âí´ïÍ£Ö¹¡£
__IO uint8_t RX_JUDGE=0;   //½ÓÊÜÅÐ¶Ï
 uint8_t CAN_RX_Buf[4];
 extern uint32_t CAN_STD_ID;
 uint8_t CAN_TX_Buf[4]={0x00,0x00,0x00,0X00};  //CAN ·¢ËÍÊý¾Ý¡

/* À©Õ¹±äÁ¿ ------------------------------------------------------------------*/
/* Ë½ÓÐº¯ÊýÔ­ÐÎ --------------------------------------------------------------*/
/* º¯ÊýÌå --------------------------------------------------------------------*/
/**
  * º¯Êý¹¦ÄÜ: ÏµÍ³Ê±ÖÓÅäÖÃ
  * ÊäÈë²ÎÊý: ÎÞ
  * ·µ »Ø Öµ: ÎÞ
  * Ëµ    Ã÷: ÎÞ
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;  // Íâ²¿¾§Õñ£¬8MHz
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;  // 9±¶Æµ£¬µÃµ½72MHzÖ÷Ê±ÖÓ
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;       // ÏµÍ³Ê±ÖÓ£º72MHz
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;              // AHBÊ±ÖÓ£º72MHz
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;               // APB1Ê±ÖÓ£º36MHz
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;               // APB2Ê±ÖÓ£º72MHz
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2);

 	// HAL_RCC_GetHCLKFreq()/1000    1msÖÐ¶ÏÒ»´Î
	// HAL_RCC_GetHCLKFreq()/100000	 10usÖÐ¶ÏÒ»´Î
	// HAL_RCC_GetHCLKFreq()/1000000 1usÖÐ¶ÏÒ»´Î
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);  // ÅäÖÃ²¢Æô¶¯ÏµÍ³µÎ´ð¶¨Ê±Æ÷
 // HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/100000);  // ÅäÖÃ²¢Æô¶¯ÏµÍ³µÎ´ð¶¨Ê±Æ÷
  /* ÏµÍ³µÎ´ð¶¨Ê±Æ÷Ê±ÖÓÔ´ */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);
  /* ÏµÍ³µÎ´ð¶¨Ê±Æ÷ÖÐ¶ÏÓÅÏÈ¼¶ÅäÖÃ */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/**
  * º¯Êý¹¦ÄÜ: Ö÷º¯Êý.
  * ÊäÈë²ÎÊý: ÎÞ
  * ·µ »Ø Öµ: ÎÞ
  * Ëµ    Ã÷: ÎÞ
  */
int main(void)
{
  uint8_t txbuf[100];
  uint8_t Mode_Count;
  // uint8_t DS18B20ID[8],temp;
 // float ftemp;
  /* ¸´Î»ËùÓÐÍâÉè£¬³õÊ¼»¯Flash½Ó¿ÚºÍÏµÍ³µÎ´ð¶¨Ê±Æ÷ */
  HAL_Init();
  /* ÅäÖÃÏµÍ³Ê±ÖÓ */
  SystemClock_Config();
  LED_GPIO_Init();
  KEY_GPIO_Init();
  GENERAL_TIMx_Init();
	
  MX_USARTx_Init();
  CAN1_Mode_Init(CAN_SJW_1TQ,CAN_BS1_5TQ,CAN_BS2_2TQ,36,CAN_MODE_NORMAL ); //Ò»¶¨·ÅÔÚSTEPMOTOR_TIMx_Init();Ç°Ãæ
  STEPMOTOR_TIMx_Init();
  MX_I2C_EEPROM_Init(); 
  SPIx_Init(); 
  HAL_TIM_Base_Start(&htimx_STEPMOTOR);
 
  memcpy(txbuf,"This SPI_Slave code of version 9.03 . Data:2018.05.17 \n",100);
  HAL_UART_Transmit(&husartx,txbuf,strlen((char *)txbuf),1000);
  /* Ê¹ÄÜ½ÓÊÕ£¬½øÈëÖÐ¶Ï»Øµ÷º¯Êý */
  HAL_UART_Receive_IT(&husartx,aRxBuffer,7);
  HAL_TIM_PWM_Start(&htimx,TIM_CHANNEL_1);//HAL_TIM_PWM_Start(&htimx,TIM_CHANNEL_1);
   /* Ê¹ÄÜ½ÓÊÕ£¬½øÈëÖÐ¶Ï»Øµ÷º¯Êý */
  Brightness=LAMP_Read_BrightValue(); 
  GENERAL_TIMx_Init();
  HAL_TIM_PWM_Start(&htimx,TIM_CHANNEL_4); 
  HAL_SPI_Receive_IT(&hspi_SPI,&SPI_aRxBuffer[0],7);
   
  while (1)
  {
	  
	  HAL_SPI_Receive_IT(&hspi_SPI,&SPI_aRxBuffer[0],7); //wt.edit 2018.04.22
	   DRV8825_SLEEP_DISABLE() ; //¸ßµçÆ½Âí´ï¹¤×÷¡£
	  if(HAL_GPIO_ReadPin(GPIO_PB9,GPIO_PB9_PIN)==0)
		{
		  DRV8825_StopMove();
		  HAL_Delay(50);
		}
      if(SPI_RX_FLAG==1)
		{
		 SPI_RX_FLAG=0;
		 A1_ReadData_Stop=0;
		 END_A2_ReadData_FLAG=0;
		 A1_CONTROL_A2_MOTOR_FUN(); 
		}
	  if(I2C_TX_DATA==1)
		{
		   I2C_TX_DATA=0;
		   A1_ReadData_Stop=0;
		   END_A2_ReadData_FLAG=0;
		   A1_Read_A2_DATA();
		}
	  if(re_intrrupt_flag==1)
	  	{
            re_intrrupt_flag=0;
			A1_ReadData_Stop=0;
			END_A2_ReadData_FLAG=0;
			A2_MOTOR_FUN();
	  	}
	  if(KEY1_StateRead()==KEY_DOWN)
		 {
		     Mode_Count++;
			 if(Mode_Count ==4&&Mode_Count!=0)
             Mode_Count = 0;	 
			 STEPMOTOR_AxisMoveRel(-5*SPR, Toggle_Pulse);
			  
		}
		
		
		if( END_STOP_FLAG==1)  //Âí´ïÔËÐÐµ½ÖÕµã£¬Í£Ö¹±êÖ¾Î»
		{
           END_A2_ReadData_FLAG=1;
		   END_STOP_FLAG=0;
		   Motor_Save_EndPosition();
        }
		/*¶ÁÈ¡A2 Âö³åÊý*/
		if(A1_ReadData_FLAG==1)
		{
		   A1_ReadData_FLAG=0;
		   if((stop_flag==0) && (A2_ReadPulse==0))
	        {
                 A1_ReadEeprom_A2_Value();
				 I2C_MASTER_TX_DATA();
				 HAL_Delay(100);
				 printf("A1 read A2 DATA fun() stop_flag=0 0x03 \n");
				
	        }
	        else
			{
				 TX_Times=0;
				 A1_ReadRealTime_A2_Value();
		         I2C_MASTER_TX_DATA();
				 HAL_Delay(50);
				 printf("A1 read A2 DATA fun() 0x03 \n");
			}
		/*A2Âí´ïÍ£Ö¹*/
		if(A1_ReadData_Stop==1)
        {
                  
				  if(TX_Times < 1)
				   {
					   TX_Times++;
					   A1_ReadRealTime_A2_Value();
					   I2C_MASTER_TX_DATA(); 
					   HAL_Delay(100);
					   printf("TX_Times= %d \n",TX_Times);
                   }
				   #if 0
				  else if((TX_Times ==10) ||(TX_Times > 11)) //wt.edit 2018.04.24
				  	TX_Times=4;

				  else
				  { 
				     printf("Tx send TX_Times is over \n");
					 A1_ReadData_Stop=0;
				  }
                 #endif
		}

		if(RX_JUDGE==1) //wt.edit 2018.04.22
	    {
          RX_JUDGE=0;
	      printf("SPI_receive data error \n");
		  
	    }
		if((KEY3_StateRead()==KEY_DOWN)||(A2_RX_STOP==1))
	    {
            PB8_flag=1;
			A2_RX_STOP=0;
			A1_ReadData_Stop=0;
			END_A2_ReadData_FLAG=0;
			A1_ReadData_Stop=0; 
			DRV8825_StopMove();
			A2_RX_STOP=0;

		}
		
		
		
	
  } //end while(1)
	
}

/****************************end main()******************************/
/********************************************************************/
/********************************************************************/
/********************************************************************/
/********************************************************************/


/******************************************************************************
  *
  * º¯ÊýÃû³Æ£º
  *º¯Êý¹¦ÄÜ: ´®¿Ú½ÓÊÕ-------»Øµ÷º¯Êý
  * ÊäÈë²ÎÊý: ÎÞ
  * ·µ »Ø Öµ: ÎÞ
  * Ëµ    Ã÷£ºÎÞ
  *
  ********************************************************************************/
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle)
{
   
    __HAL_GPIO_EXTI_CLEAR_IT(KEY3_GPIO_PIN);
	if((KEY3_StateRead()==KEY_DOWN)||(A2_RX_STOP==1)||(HAL_GPIO_ReadPin(GPIO_PB9,GPIO_PB9_PIN)==0))
		{
		  DRV8825_StopMove();
		}

  
if(HAL_UART_Receive_IT(&husartx,aRxBuffer,7)==HAL_OK )
 {
	  //HAL_Delay(50);
		if(HAL_GPIO_ReadPin(GPIO_PB9,GPIO_PB9_PIN)==0)
		{
		 DRV8825_StopMove();
		}

	  if(aRxBuffer[0]==0xa1)
		{
          if(aRxBuffer[1]==0x00)
		   {
			switch(aRxBuffer[2])
            	{
                      case 0xff :
                         if(aRxBuffer[6]==0x0b)
                         	{
                               re_intrrupt_flag=1;
				               judge_data= 0xff;
					
					          __HAL_UART_CLEAR_IDLEFLAG(&husartx); //edit 18.02.23
						     }
					  
					  	break;
					  case 0x33 :
						  if(aRxBuffer[6]==0x0b)
                         	{
                                re_intrrupt_flag=1;
								repcdata[0]=	aRxBuffer[3]; //×î¸ß×Ö½Ú
							    repcdata[1]=	aRxBuffer[4]; //ÖÐ¼ä×Ö½Ú
							    repcdata[2]=	aRxBuffer[5]; //×îµÍ×Ö½Ú
							    judge_data= 0x33;
							    __HAL_UART_CLEAR_IDLEFLAG(&husartx); //edit 18.02.23 	  
						     
							}
						  
						  break;
						case 0xb0 :
						  if(aRxBuffer[6]==0x0b)
                         	{
                               re_intrrupt_flag=1;
				              judge_data= 0xb0;
				
				               __HAL_UART_CLEAR_IDLEFLAG(&husartx); //edit 18.02.23 	 
						    
							}
						  
						  break;
						case 0x02 :
							 if(aRxBuffer[6]==0x0b)
								{
									re_intrrupt_flag=1;
								   repcdata[0]=	aRxBuffer[3]; //×î¸ß×Ö½Ú
								   repcdata[1]=	aRxBuffer[4]; //ÖÐ¼ä×Ö½Ú
								   repcdata[2]=	aRxBuffer[5]; //×îµÍ×Ö½Ú
								   judge_data= 0x02;
								 
								   __HAL_UART_CLEAR_IDLEFLAG(&husartx); //edit 18.02.23 
								}
							
							break;
						case 0x82 :
							if(aRxBuffer[6]==0x0b)
								{
									re_intrrupt_flag=1;
									repcdata[0]=	aRxBuffer[3]; //×î¸ß×Ö½Ú
								   repcdata[1]=	aRxBuffer[4]; //ÖÐ¼ä×Ö½Ú
								   repcdata[2]=	aRxBuffer[5]; //×îµÍ×Ö½Ú
								   judge_data= 0x82;
								 
								   __HAL_UART_CLEAR_IDLEFLAG(&husartx); //edit 18.02.23 	 
								}
							
							break;
						case 0x90 :
							if(aRxBuffer[6]==0x0b)
								{	  
								   re_intrrupt_flag=1;
								   repcdata[1]=	aRxBuffer[4]; //ÖÐ¼ä×Ö½Ú
								   repcdata[2]=	aRxBuffer[5]; //×îµÍ×Ö½Ú
								   judge_data= 0x90;
									
									__HAL_UART_CLEAR_IDLEFLAG(&husartx); //edit 18.02.23
								}
							
							break;
						case 0xa0 :
							if(aRxBuffer[6]==0x0b)   //ÖØÐÂÉèÖÃÔ­µã
								{
									//DRV8825_SLEEP_DISABLE(); //¸ßµçÆ½¿ªÊ¼¹¤×÷,½â³ýÐÝÃß×´Ì¬
									 re_intrrupt_flag=1;
									 judge_data= 0xa0;
									__HAL_UART_CLEAR_IDLEFLAG(&husartx); //edit 18.02.23
									
								}
							break;
						case 0x00 :
							if(aRxBuffer[6]==0x0b)
								{
									DRV8825_StopMove();
									LED2_OFF;
									LED1_OFF;
									HAL_Delay(10);
									LED1_ON;
									LED2_ON;
							
									__HAL_UART_CLEAR_IDLEFLAG(&husartx); //edit 18.02.23
								}
							break;
						case 0xc0 :
							 if(aRxBuffer[6]==0x0b)
								{
								   re_intrrupt_flag=1;
								   judge_data= 0xc0;
                                   __HAL_UART_CLEAR_IDLEFLAG(&husartx); //edit 18.02.23
								}
							break;
						case 0xd0 :
							 if(aRxBuffer[6]==0x0b)
								{
								   re_intrrupt_flag=1;
								   judge_data= 0xd0;
									 __HAL_UART_CLEAR_IDLEFLAG(&husartx); //edit 18.02.23	
								}
							  break;
			    }
			} //end if(aRxBuffer[1]==0x00)
		   if(aRxBuffer[1]==0x01)   
		   {
		     switch(aRxBuffer[2])
			 {
				 case 0x03:   //¶ÁÈ¡Âí´ïÊµÊ±Î»ÖÃÂö³åÊý
				    if(aRxBuffer[6]==0x0b)
						{
						   re_intrrupt_flag=1;
						   judge_data= 0x103;
						  //Display_CurrentPosition();
							__HAL_UART_CLEAR_IDLEFLAG(&husartx); //edit 18.02.23
						}
				  break; 
				 case 0x04:      /* ¶ÁÈ¡LED µÆÁÁ¶ÈÖµ */
					 {
						if(aRxBuffer[6]==0x0b)
						{

							re_intrrupt_flag=1;
						   judge_data= 0x104;
							
						
						__HAL_UART_CLEAR_IDLEFLAG(&husartx); //edit 18.02.23
						}
					 }
				    break;
				  case 0x01 ://case 0x01 :     /*if(aRxBuffer[2]==0x01)¶ÁÈ¡Âí´ï×ªËÙÖµ*/
					 {
						if(aRxBuffer[6]==0x0b)
						{

							re_intrrupt_flag=1;
						   judge_data= 0x101;
							__HAL_UART_CLEAR_IDLEFLAG(&husartx); //edit 18.02.23
						}
					 }
					 break;
					 case 0x02:    //if(aRxBuffer[2]==0x02)  /*¶ÁÈ¡»·¾³ÎÂ¶ÈÖµ*/
					 {
						if(aRxBuffer[6]==0x0b)
						{
							
						   re_intrrupt_flag=1;
						   judge_data= 0x102;
						  
							__HAL_UART_CLEAR_IDLEFLAG(&husartx); //edit 18.02.23
						}
					 }
                     break;					 
				   
		   }
	     
	    }
  }// end if(aRxBuffer[0]==0xa1)

  }
}
 
/*******************************************************************
  *
  * º¯Êý¹¦ÄÜ: SPI´«ÊäÍê³É»Øµ÷º¯Êý
  * ÊäÈë²ÎÊý: ÎÞ
  * ·µ »Ø Öµ: ÎÞ
  * Ëµ    Ã÷: SPIÖÐ¶Ïº¯Êý
  *
  *******************************************************************/
#if 1
void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
{
 
  if(SPI_aRxBuffer[0]==0xa2)
  {
     RX_JUDGE=0;
	 if(SPI_aRxBuffer[1]==0x00)
	 {
        SPI_RX_FLAG=1;
		HAL_Delay(200);
		if(SPI_aRxBuffer[2]==0x00)
			{
              A2_RX_STOP=1;
			  SPI_RX_FLAG=0;
			}
	 }
	 else if (SPI_aRxBuffer[1]==0x01)
	 {
		I2C_TX_DATA=1;
		SPI_RX_FLAG=0;
		if(SPI_aRxBuffer[2]==0x03)
			{
                HAL_Delay(40);
				if(END_A2_ReadData_FLAG==1) //wt.edit 2018.04.22
            	{
				   I2C_TX_DATA=0;
				   A1_ReadData_Stop=1;
				   
				}
			    else
				{
				 A1_ReadData_FLAG=1;
				 I2C_TX_DATA=0;
				}
		    }
		
	 }
	 else
	    RX_JUDGE=1; //wt.edit 2018.04.25
  }
  else 
  {
      RX_JUDGE=1;
  }
  
	
}

#endif

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
