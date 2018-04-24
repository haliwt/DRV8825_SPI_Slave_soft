
/* ����ͷ�ļ� ----------------------------------------------------------------*/
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

#define SENDBUFF_SIZE             100 // ����DMA���ͻ�������С
#define STEPMOTOR_MICRO_STEP      32  // �������������ϸ�֣�������������ʵ�����ö�Ӧ

/* ˽�б��� ------------------------------------------------------------------*/
uint8_t aRxBuffer[256];                      // �������� 
uint8_t aTxBuffer[SENDBUFF_SIZE];       // ����DMA���ͻ�����
uint8_t repcdata[3]={0x00,0x00,0x00};                    //����λ�����յ���3���ֽ����ݡ�
extern uint8_t i2c_tx_buffer[3];
/* ��չ���� ------------------------------------------------------------------*/
extern __IO uint16_t Toggle_Pulse; /* ��������ٶȿ��ƣ��ɵ��ڷ�ΧΪ 650 -- 3500 ��ֵԽС�ٶ�Խ�� */
__IO uint32_t pulse_count; /*  ���������һ�����������������2 */
extern  __IO uint16_t  home_position    ; 
extern __IO uint8_t Brightness;   
extern __IO uint8_t save_flag;
__IO uint8_t re_intrrupt_flag=0; //����λ���������źű�־λ�������жϱ�־λ
__IO uint16_t judge_data;    //���յ����ݵĹ��������ݡ�
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
extern __IO uint8_t stop_flag; //������־λ
extern __IO uint8_t END_STOP_FLAG;  //������е��յ㣬ֹͣ��־λ
extern __IO uint8_t A2_RX_STOP;
extern __IO uint8_t A2_ReadPulse; //��ȡ�ڶ���������������־λ
extern __IO uint8_t END_A2_Read_Pulse;
__IO uint8_t A1_ReadData_FLAG=0;  //A1��ȡA2����־λ��
extern __IO uint8_t END_A2_ReadData_FLAG;  //???????
__IO uint8_t A1_ReadData_Stop=0;  //����ȡ�ٶȣ������㡣
__IO uint8_t TX_Times=0;      //���2��ȡʵʱ���͵����ݴ��������ֹͣ��
__IO uint8_t RX_JUDGE=0;   //�����ж�

/* ��չ���� ------------------------------------------------------------------*/
/* ˽�к���ԭ�� --------------------------------------------------------------*/
/* ������ --------------------------------------------------------------------*/
/**
  * ��������: ϵͳʱ������
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ��: ��
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;  // �ⲿ����8MHz
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;  // 9��Ƶ���õ�72MHz��ʱ��
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;       // ϵͳʱ�ӣ�72MHz
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;              // AHBʱ�ӣ�72MHz
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;               // APB1ʱ�ӣ�36MHz
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;               // APB2ʱ�ӣ�72MHz
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2);

 	// HAL_RCC_GetHCLKFreq()/1000    1ms�ж�һ��
	// HAL_RCC_GetHCLKFreq()/100000	 10us�ж�һ��
	// HAL_RCC_GetHCLKFreq()/1000000 1us�ж�һ��
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);  // ���ò�����ϵͳ�δ�ʱ��
 // HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/100000);  // ���ò�����ϵͳ�δ�ʱ��
  /* ϵͳ�δ�ʱ��ʱ��Դ */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);
  /* ϵͳ�δ�ʱ���ж����ȼ����� */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/**
  * ��������: ������.
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ��: ��
  */
int main(void)
{
  uint8_t txbuf[100];
  uint8_t Mode_Count;
  // uint8_t DS18B20ID[8],temp;
 // float ftemp;
  /* ��λ�������裬��ʼ��Flash�ӿں�ϵͳ�δ�ʱ�� */
  HAL_Init();
  /* ����ϵͳʱ�� */
  SystemClock_Config();
  LED_GPIO_Init();
  KEY_GPIO_Init();
  GENERAL_TIMx_Init();
	
  MX_USARTx_Init();
  STEPMOTOR_TIMx_Init();
  MX_I2C_EEPROM_Init(); 
  SPIx_Init(); 
  HAL_TIM_Base_Start(&htimx_STEPMOTOR);
 
  memcpy(txbuf,"This SPI_Slave code of version 7.16 . Data:2018.04.24 \n",100);
  HAL_UART_Transmit(&husartx,txbuf,strlen((char *)txbuf),1000);
  /* ʹ�ܽ��գ������жϻص����� */
  HAL_UART_Receive_IT(&husartx,aRxBuffer,7);
  HAL_TIM_PWM_Start(&htimx,TIM_CHANNEL_1);//HAL_TIM_PWM_Start(&htimx,TIM_CHANNEL_1);
   /* ʹ�ܽ��գ������жϻص����� */
  Brightness=LAMP_Read_BrightValue(); 
  GENERAL_TIMx_Init();
  HAL_TIM_PWM_Start(&htimx,TIM_CHANNEL_4); 
  HAL_SPI_Receive_IT(&hspi_SPI,&SPI_aRxBuffer[0],7);
   
  while (1)
  {
	  
	  HAL_SPI_Receive_IT(&hspi_SPI,&SPI_aRxBuffer[0],7); //wt.edit 2018.04.22
      if(SPI_RX_FLAG==1)
		{
		 SPI_RX_FLAG=0;
		 END_A2_ReadData_FLAG=0;
		 A1_CONTROL_A2_MOTOR_FUN(); 
		}
	  if(I2C_TX_DATA==1)
		{
		   I2C_TX_DATA=0;
		   END_A2_ReadData_FLAG=0;
		   A1_Read_A2_DATA();
		}
	  if(re_intrrupt_flag==1)
	  	{
            re_intrrupt_flag=0;
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
		
		if((HAL_GPIO_ReadPin(GPIO_PB8,GPIO_PB8_PIN)==0)||(A2_RX_STOP==1))
		{
			PB8_flag=1;
			END_A2_ReadData_FLAG=0;
		    A1_ReadData_Stop=0; 
			DRV8825_StopMove();
			A2_RX_STOP=0;
			//printf("a2_rx_stop=1\n");

		}
		if( END_STOP_FLAG==1)  //������е��յ㣬ֹͣ��־λ
		{
           END_A2_ReadData_FLAG=1;
		   END_STOP_FLAG=0;
		   Motor_Save_EndPosition();
        }
		/*��ȡA2 ������*/
		if(A1_ReadData_FLAG==1)
		{
		   A1_ReadData_FLAG=0;
		   if((stop_flag==0) && (A2_ReadPulse==0))
	        {
                 A1_ReadData_FLAG=0;
				 A1_ReadEeprom_A2_Value();
				 I2C_MASTER_TX_DATA();
				 HAL_Delay(50);
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

		}
		/*A2���ֹͣ*/
		if(A1_ReadData_Stop==1)
        {
                  A1_ReadData_Stop=0;
				  TX_Times++;
				  if(TX_Times < 3)
				   {
					   A1_ReadRealTime_A2_Value();
					   I2C_MASTER_TX_DATA(); 
					   HAL_Delay(100);
					   printf("TX_Times= %d \n",TX_Times);
                   }
				  else if((TX_Times ==10) ||(TX_Times > 11)) //wt.edit 2018.04.24
				  	TX_Times=4;

				  else
				  { 
				     printf("Tx send TX_Times is over \n");
				  }

		}

		if(RX_JUDGE==1) //wt.edit 2018.04.22
	    {
          RX_JUDGE=0;
	      printf("SPI_receive data error \n");
		  
	    }
		if(KEY3_StateRead()==KEY_DOWN)
	    {
            PB8_flag=1;
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
  * �������ƣ�
  *��������: ���ڽ���-------�ص�����
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ������
  *
  ********************************************************************************/
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle)
{
   
    __HAL_GPIO_EXTI_CLEAR_IT(KEY3_GPIO_PIN);
	if((KEY3_StateRead()==KEY_DOWN)||(A2_RX_STOP==1)||(HAL_GPIO_ReadPin(GPIO_PB8,GPIO_PB8_PIN)==0))
		{
		  DRV8825_StopMove();
		}

  
if(HAL_UART_Receive_IT(&husartx,aRxBuffer,7)==HAL_OK )
 {
	  //HAL_Delay(50);
		if(HAL_GPIO_ReadPin(GPIO_PB8,GPIO_PB8_PIN)==0)
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
								repcdata[0]=	aRxBuffer[3]; //����ֽ�
							    repcdata[1]=	aRxBuffer[4]; //�м��ֽ�
							    repcdata[2]=	aRxBuffer[5]; //����ֽ�
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
								   repcdata[0]=	aRxBuffer[3]; //����ֽ�
								   repcdata[1]=	aRxBuffer[4]; //�м��ֽ�
								   repcdata[2]=	aRxBuffer[5]; //����ֽ�
								   judge_data= 0x02;
								 
								   __HAL_UART_CLEAR_IDLEFLAG(&husartx); //edit 18.02.23 
								}
							
							break;
						case 0x82 :
							if(aRxBuffer[6]==0x0b)
								{
									re_intrrupt_flag=1;
									repcdata[0]=	aRxBuffer[3]; //����ֽ�
								   repcdata[1]=	aRxBuffer[4]; //�м��ֽ�
								   repcdata[2]=	aRxBuffer[5]; //����ֽ�
								   judge_data= 0x82;
								 
								   __HAL_UART_CLEAR_IDLEFLAG(&husartx); //edit 18.02.23 	 
								}
							
							break;
						case 0x90 :
							if(aRxBuffer[6]==0x0b)
								{	  
								   re_intrrupt_flag=1;
								   repcdata[1]=	aRxBuffer[4]; //�м��ֽ�
								   repcdata[2]=	aRxBuffer[5]; //����ֽ�
								   judge_data= 0x90;
									
									__HAL_UART_CLEAR_IDLEFLAG(&husartx); //edit 18.02.23
								}
							
							break;
						case 0xa0 :
							if(aRxBuffer[6]==0x0b)   //��������ԭ��
								{
									//DRV8825_SLEEP_DISABLE(); //�ߵ�ƽ��ʼ����,�������״̬
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
				 case 0x03:   //��ȡ���ʵʱλ��������
				    if(aRxBuffer[6]==0x0b)
						{
						   re_intrrupt_flag=1;
						   judge_data= 0x103;
						  //Display_CurrentPosition();
							__HAL_UART_CLEAR_IDLEFLAG(&husartx); //edit 18.02.23
						}
				  break; 
				 case 0x04:      /* ��ȡLED ������ֵ */
					 {
						if(aRxBuffer[6]==0x0b)
						{

							re_intrrupt_flag=1;
						   judge_data= 0x104;
							
						
						__HAL_UART_CLEAR_IDLEFLAG(&husartx); //edit 18.02.23
						}
					 }
				    break;
				  case 0x01 ://case 0x01 :     /*if(aRxBuffer[2]==0x01)��ȡ���ת��ֵ*/
					 {
						if(aRxBuffer[6]==0x0b)
						{

							re_intrrupt_flag=1;
						   judge_data= 0x101;
							__HAL_UART_CLEAR_IDLEFLAG(&husartx); //edit 18.02.23
						}
					 }
					 break;
					 case 0x02:    //if(aRxBuffer[2]==0x02)  /*��ȡ�����¶�ֵ*/
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
  * ��������: SPI������ɻص�����
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ��: SPI�жϺ���
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
                if(END_A2_ReadData_FLAG==1) //wt.edit 2018.04.22
            	{
				   I2C_TX_DATA=0;
				   A1_ReadData_Stop=1;
				   
				 
	            }
			    else
				{
				 A1_ReadData_FLAG=1;
				 A1_ReadData_Stop=0;
				 I2C_TX_DATA=0;
				}
		    }
		else
			{

		    }
	 }
  }
  else 
  {
      RX_JUDGE=1;
  }
  
	
}

#endif

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
