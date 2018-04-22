#include "a2_fun/a2_fun.h"
#include "DRV8825/drv8825.h"
#include "led/bsp_led.h"
#include "StepMotor/bsp_StepMotor.h"
#include "i2c/bsp_EEPROM.h"
#include "key/bsp_key.h"
#include "hextodec/hextodec.h"
#include "usart/bsp_usartx.h"
#include "lamp/bsp_lamp.h"
#include "GeneralTIM/bsp_GeneralTIM.h"
#include "DS18B20/bsp_DS18B20.h"
#include "i2c_slave/bsp_I2C.h"
#include "spi/bsp_spi.h"

extern uint8_t i2c_tx_buffer[3];
extern __IO uint16_t Toggle_Pulse ;
extern __IO uint32_t pulse_count; /*  ���������һ�����������������2 */
extern uint8_t I2c_Buf_Write[256];
extern uint8_t I2c_Buf_Read[256];
extern speedRampData srd  ;
static __IO int32_t  step_position=0 ;
extern __IO int32_t PulseNumbers;
extern __IO uint32_t step_count; 
extern __IO uint8_t home_flag; 
extern __IO uint32_t step_count;  //��PC�����ʹ洢�����������
extern __IO uint8_t NewOrigin_flag;
extern __IO uint8_t back_flag;
extern __IO uint8_t re_intrrupt_flag; //����λ���������źű�־λ�������жϱ�־λ
extern __IO uint8_t PB8_flag;
extern __IO uint16_t judge_data;    //���յ����ݵĹ��������ݡ�
extern __IO uint8_t Brightness; 
extern uint8_t repcdata[3]; 
extern uint8_t aTxBuffer[100];       // ����DMA���ͻ�����
extern uint8_t aRxBuffer[256]; 
extern  __IO uint16_t  home_position    ;
extern uint8_t SPI_aRxBuffer[100];
extern uint8_t SPI_aTxBuffer[6];

extern uint8_t  SPI_RX_FLAG;
extern uint8_t SPI_RX_DATA;
extern uint8_t I2C_TX_FLAG;
extern uint8_t I2C_TX_DATA;
__IO uint8_t A2_RX_STOP=0;    //�ڶ������������ֹͣ��־λ��
__IO uint8_t A2_ReadPulse=0; //��ȡ�ڶ���������������־λ
extern __IO uint8_t stop_flag; //������־λ
/**********************************************************
 *
 *�������ƣ�
 *�������ܣ�A2��������������
 *����:��
 *����ֵ����
 *
**********************************************************/
void A2_MOTOR_FUN(void)
{ 
    
	uint8_t DS18B20ID[8],temp;
	float temperature;
	
	 switch(judge_data)
		   	{
              case 0x02 :
				    re_intrrupt_flag=0;
					PB8_flag=0;
			  	   // DRV8825_SLEEP_DISABLE(); //�ߵ�ƽ��ʼ����,�������״̬
			  	   // HAL_Delay(10);
					DRV8825_CW_AxisMoveRel(repcdata[0],repcdata[1],repcdata[2],Toggle_Pulse);
					if(HAL_GPIO_ReadPin(GPIO_PB8,GPIO_PB8_PIN)==0)
					{
						DRV8825_StopMove();
					}
					HAL_Delay(2);
					if(HAL_GPIO_ReadPin(GPIO_PB8,GPIO_PB8_PIN)==0)
					{
						DRV8825_StopMove();
					}
					LED2_OFF;
					LED1_ON;
					HAL_Delay(2);
					if(HAL_GPIO_ReadPin(GPIO_PB8,GPIO_PB8_PIN)==0)
					{
						DRV8825_StopMove();
					}
				
					if(HAL_GPIO_ReadPin(GPIO_PB8,GPIO_PB8_PIN)==0)
					{
						DRV8825_StopMove();
					}
					 repcdata[0]=0;
			          repcdata[1]=0;
					  repcdata[2]=0;
 
			  break;
					
			  case 0x82 :
					  {
					  re_intrrupt_flag=0;
					  PB8_flag=0;
					 // DRV8825_SLEEP_DISABLE(); //�ߵ�ƽ��ʼ����,�������״̬	
					 //  HAL_Delay(10);
			          DRV8825_CCW_AxisMoveRel(repcdata[0],repcdata[1],repcdata[2],Toggle_Pulse);
			          if(HAL_GPIO_ReadPin(GPIO_PB8,GPIO_PB8_PIN)==0)
						{
							DRV8825_StopMove();
						}					
					HAL_Delay(2);
					LED2_ON;
					LED1_OFF;
					 if(HAL_GPIO_ReadPin(GPIO_PB8,GPIO_PB8_PIN)==0)
						{
							DRV8825_StopMove();
						}	
					HAL_Delay(2);
					//STEPMOTOR_AxisMoveRel(335544*SPR, Toggle_Pulse);---���ڲ���ģʽ
				
					//__HAL_UART_CLEAR_IDLEFLAG(&husartx); //edit 18.02.23
					 if(HAL_GPIO_ReadPin(GPIO_PB8,GPIO_PB8_PIN)==0)
						{
							DRV8825_StopMove();
						}
					  repcdata[0]=0;
			          repcdata[1]=0;
					  repcdata[2]=0;
                       // HAL_UART_Transmit(&husartx,tranbuffer,1,1);
					  }
				    break;
			  case 0x33 :
					  {
						re_intrrupt_flag=0; 
						PB8_flag=0;
                     // DRV8825_SLEEP_DISABLE() ; //�ߵ�ƽ��ʼ����
                     // HAL_Delay(10);
					  STEPMOTOR_PC_AxisMoveAbs( repcdata[0],repcdata[1],repcdata[2],Toggle_Pulse);
                      
					   if(HAL_GPIO_ReadPin(GPIO_PB8,GPIO_PB8_PIN)==0)
						{
							DRV8825_StopMove();
						}	
						
					 // printf("0X33 is OK \n");
					  repcdata[0]=0;
			          repcdata[1]=0;
					  repcdata[2]=0;
					
					  }
			        
					  break;
					  
			  case 0xb0 :
			    {
			     re_intrrupt_flag=0;
				 PB8_flag=0;
				 STEPMOTOR_AxisMoveAbs(0*SPR,Toggle_Pulse);
				 if(HAL_GPIO_ReadPin(GPIO_PB8,GPIO_PB8_PIN)==0)
						{
							DRV8825_StopMove();
						}
				 
					//	printf("0Xb0 is OK \n");
				}
			  break;

			 case 0xa0 :    //��������ԭ��
			 	    re_intrrupt_flag=0; 
			 	    Set_NewOrigin_Position();
					HAL_Delay(30);
					LED2_OFF;
					LED1_OFF;
					HAL_Delay(30);
					LED2_ON;
					LED1_ON;
					HAL_Delay(10);
			 	break;
			case 0x90 :
                     re_intrrupt_flag=0; 
			        DRV8825_SetSpeed(aRxBuffer[4],aRxBuffer[5]);
					HAL_Delay(30);
					LED2_OFF;
					LED1_OFF;
					HAL_Delay(30);
					LED2_ON;
					LED1_ON;
				break;
		   case 0xff :  //ͬ��λ��ͨѶ
                    re_intrrupt_flag=0; 
					LED2_ON;
					LED1_ON;		  
					HAL_Delay(200);
					LED2_OFF;
					LED1_OFF;
					HAL_Delay(200);
					LED2_ON;
					LED1_ON;
					HAL_Delay(200);
					LED2_OFF;
					LED1_OFF;
					HAL_Delay(200);
					LED2_ON;
					LED1_ON;
					HAL_Delay(200);
					LED2_OFF;
					LED1_OFF;
					HAL_Delay(200);
		   	break;
			case 0xc0 :
                    re_intrrupt_flag=0; 
				   Brightness=aRxBuffer[5];
				   LAMP_Save_BrightValue(Brightness);
				   GENERAL_TIMx_Init();
				   HAL_TIM_PWM_Start(&htimx,TIM_CHANNEL_4);
				
					LED2_ON;
					LED1_OFF;
					HAL_Delay(10);
				    LED2_OFF;
					LED1_ON;
				break;
			case 0xd0 :
                    re_intrrupt_flag=0; 
					EEPROM_Clear_Buf();
                    HAL_Delay(10);
					LED2_OFF;
					LED1_OFF;
					HAL_Delay(10);
					LED2_ON;
					LED1_ON;
					HAL_Delay(10);
				break;

            case 0x103 :   //��ȡָ��  0x 1xx 
				 re_intrrupt_flag=0; 
				Display_EEPROM_Value();  //��ȡ���ʵʱλ������ֵ
				break;
			case 0x104 :
				    re_intrrupt_flag=0; 
				    temp= LAMP_Read_BrightValue(); //��ȡ����ֵ
					printf("BRV = %d \n",temp);
					LED2_OFF;
					LED1_OFF;
					HAL_Delay(20);
					LED2_ON;
					LED1_ON;
					HAL_Delay(20);
				break;
			case 0x102 :
				    re_intrrupt_flag=0; 
				     temperature=DS18B20_GetTemp_MatchRom(DS18B20ID);
					/* ��ӡͨ�� DS18B20 ���кŻ�ȡ���¶�ֵ */
					printf("��ȡ�����к��������¶ȣ�%.1f\n",temperature);
					/* 1s ��ȡһ���¶�ֵ */
					HAL_Delay(1000);
					printf("��ȡ�����к��������¶ȣ�%.1f\n",temperature);
					HAL_Delay(1000);
					
				break;
			case 0x101:
                re_intrrupt_flag=0; 
				PC_DRV8825_ReadSpeed(); //��ȡ����ٶ�ֵ
				break;
		    }

}//END if(interrupt_flag==1)
	 


/*********************************************************
 *
 *�������ƣ�A1_CONTROL_A2_MOTOR_FUN()
 *�������ܣ���һ�������Ƶڶ�����﹦�ܺ���
 *�����������
 *����ֵ����
 *
**********************************************************/
void A1_CONTROL_A2_MOTOR_FUN(void)
{
    
	         switch(SPI_aRxBuffer[2]) 
			 {
				
				  case 0x02:
					if(SPI_aRxBuffer[6]==0xb)
					{
					   A2_ReadPulse=0;
			           repcdata[0] = SPI_aRxBuffer[3];
					   repcdata[1] = SPI_aRxBuffer[4];
					   repcdata[2] = SPI_aRxBuffer[5];
			  	    DRV8825_CW_AxisMoveRel(repcdata[0],repcdata[1],repcdata[2],Toggle_Pulse);
					if((KEY3_StateRead()==KEY_DOWN)||(A2_RX_STOP==1)||(HAL_GPIO_ReadPin(GPIO_PB8,GPIO_PB8_PIN)==0))
					{
						A2_RX_STOP=0;
						DRV8825_StopMove();
					}
					
					   printf("0x02 order \n");
	         	    }
					 break;
				   case 0x82 :   //�������ķ����ƶ���
					 if(SPI_aRxBuffer[6]==0xb) 
				      {
			               A2_ReadPulse=0;
						   repcdata[0] = SPI_aRxBuffer[3];
						   repcdata[1] = SPI_aRxBuffer[4];
						   repcdata[2] = SPI_aRxBuffer[5];
						
						  DRV8825_CCW_AxisMoveRel(repcdata[0],repcdata[1],repcdata[2],Toggle_Pulse);
					      
						  if((KEY3_StateRead()==KEY_DOWN)||(A2_RX_STOP==1)||(HAL_GPIO_ReadPin(GPIO_PB8,GPIO_PB8_PIN)==0))
					       {
								A2_RX_STOP=0;
								DRV8825_StopMove();
					       }   
						  
						printf("0x82 order \n");
					  }
				    break;
			  case 0x33 :
					  if(SPI_aRxBuffer[6]==0xb)
			           {
						A2_ReadPulse=0;
					    repcdata[0]=SPI_aRxBuffer[3];
						repcdata[1]=SPI_aRxBuffer[4];
						repcdata[2]=SPI_aRxBuffer[5];
						
						STEPMOTOR_PC_AxisMoveAbs( repcdata[0],repcdata[1],repcdata[2],Toggle_Pulse);
                        
						if((KEY3_StateRead()==KEY_DOWN)||(A2_RX_STOP==1)||(HAL_GPIO_ReadPin(GPIO_PB8,GPIO_PB8_PIN)==0))
					       {
								A2_RX_STOP=0;
								DRV8825_StopMove();
					       }
						 printf("motor works 0x33 order \n");
					    }
			        
					  break;
					  
			  case 0xb0 :
			    if(SPI_aRxBuffer[6]==0xb)     
			      {
                    A2_ReadPulse=0;
					STEPMOTOR_AxisMoveAbs(0*SPR,Toggle_Pulse);
					if((KEY3_StateRead()==KEY_DOWN)||(A2_RX_STOP==1)||(HAL_GPIO_ReadPin(GPIO_PB8,GPIO_PB8_PIN)==0))
					  {
						A2_RX_STOP=0;
						DRV8825_StopMove();
					   }   
					 printf("order 0xb0 \n");	
				    }
				  
			        break;

			 case 0xa0 :    //��������ԭ��
				 if(SPI_aRxBuffer[6]==0xb)
				 {
                    A2_ReadPulse=0;
					Set_NewOrigin_Position();
					printf("new origin psoition \n");
					
				 }
			 	break;
			case 0x90 :
                    if(SPI_aRxBuffer[6]==0xb)
                    {						
			     
					A2_ReadPulse=0;
			        aRxBuffer[4]=SPI_aRxBuffer[4];
				    aRxBuffer[5]=SPI_aRxBuffer[5];
					DRV8825_SetSpeed(aRxBuffer[4],aRxBuffer[5]);
					HAL_Delay(30);
					LED2_OFF;
					LED1_OFF;
					HAL_Delay(30);
					LED2_ON;
					LED1_ON;
					}
				break;
		   case 0xff:  //ͬ��λ��ͨѶ
			        if(SPI_aRxBuffer[6]==0xb)
					{
                    A2_ReadPulse=0;
					LED2_ON;
					LED1_ON;		  
					HAL_Delay(200);
					LED2_OFF;
					LED1_OFF;
					HAL_Delay(200);
					LED2_ON;
					LED1_ON;
					HAL_Delay(200);
					LED2_OFF;
					LED1_OFF;
					HAL_Delay(200);
					LED2_ON;
					LED1_ON;
					HAL_Delay(200);
					LED2_OFF;
					LED1_OFF;
					HAL_Delay(200);
					}
				break;
			case 0xee :
				   if(SPI_aRxBuffer[6]==0xb)
				   {
				    A2_ReadPulse=0;
					printf("This is 0xee order \n");
				    LED2_ON;
					LED1_ON;		  
					HAL_Delay(200);
					LED2_OFF;
					LED1_OFF;
					HAL_Delay(200);
					LED2_ON;
					LED1_ON;
					HAL_Delay(200);
					LED2_OFF;
					LED1_OFF;
					HAL_Delay(200);
					LED2_ON;
					LED1_ON;
					HAL_Delay(200);
					LED2_OFF;
					LED1_OFF;
					HAL_Delay(200);
				   }
				  break;
	
			case 0xc0 :
				   if(SPI_aRxBuffer[6]==0xb)
				   {
                   A2_ReadPulse=0;
				   Brightness=SPI_aRxBuffer[5];
				   LAMP_Save_BrightValue(Brightness);
				   GENERAL_TIMx_Init();
				   HAL_TIM_PWM_Start(&htimx,TIM_CHANNEL_4);
				    LED2_ON;
					LED1_OFF;
					HAL_Delay(10);
				    LED2_OFF;
					LED1_ON;
				   }
				break;
			case 0xd0 :
				     if(SPI_aRxBuffer[6]==0xb)
					 {
						A2_ReadPulse=0;
						EEPROM_Clear_Buf();
						HAL_Delay(100);
						LED2_OFF;
						LED1_OFF;
						HAL_Delay(100);
						LED2_ON;
						LED1_ON;
						HAL_Delay(10);
						__HAL_UART_CLEAR_IDLEFLAG(&husartx); //edit 18.02.23
					 }
				break;
			#if 0
			case 0x00 :
				// if(SPI_aRxBuffer[6]==0xb)  //wt.edit 18.04.07
				 {
				    A2_RX_STOP=1;
					DRV8825_StopMove();
				 }
				 break;
			#endif
				default:
					SPI_RX_FLAG=0;
	    }	
} 
    
/*************************************************
*
*��������A1_Read_A2_DATA()
*�������ܣ��������ư�A1 ����һ������
*�������: ��
*����ֵ����
*
**********************************************/
void A1_Read_A2_DATA(void)	
{	
	     uint8_t temp;
		  switch(SPI_aRxBuffer[2])
			{
                #if 1
				case 0x03 :   //��ȡָ��  ��ȡ���ʵʱλ��������
                  {
					if(stop_flag==0)
			        {
			             A1_ReadEeprom_A2_Value();
						 I2C_MASTER_TX_DATA();
			        }
			        else
			         A1_ReadRealTime_A2_Value();
			            
					
			        }
					
				 break;
				#endif 
				case 0x04 : //��ȡLED�Ƶ�����ֵ
				        A2_ReadPulse=0;
						temp= LAMP_Read_BrightValue(); //��ȡ����ֵ
						printf("BRV = %d \n",temp);
				        i2c_tx_buffer[2]=temp;
				        I2C_MASTER_TX_DATA();
					break;
				
				case 0x01:
					A2_ReadPulse=0;
					printf("SPI_aRxBuffer[2]=0x01\n");
					A1_ReadSpeed_A2_Value();
					I2C_MASTER_TX_DATA();
					break;
				case 0xe0 :
					A2_ReadPulse=0;
				    A1_ReadEeprom_A2_Value();
				    I2C_MASTER_TX_DATA();
					break;
				default:
					{
						I2C_TX_DATA=0;
						A2_ReadPulse=0;
					}
			    
		   } 
}
	    	  
 




