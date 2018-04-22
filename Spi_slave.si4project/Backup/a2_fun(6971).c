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
extern __IO uint32_t pulse_count; /*  脉冲计数，一个完整的脉冲会增加2 */
extern uint8_t I2c_Buf_Write[256];
extern uint8_t I2c_Buf_Read[256];
extern speedRampData srd  ;
static __IO int32_t  step_position=0 ;
extern __IO int32_t PulseNumbers;
extern __IO uint32_t step_count; 
extern __IO uint8_t home_flag; 
extern __IO uint32_t step_count;  //相PC机发送存储的马达脉冲数
extern __IO uint8_t NewOrigin_flag;
extern __IO uint8_t back_flag;
extern __IO uint8_t re_intrrupt_flag; //从上位机，接收信号标志位，进入中断标志位
extern __IO uint8_t PB8_flag;
extern __IO uint16_t judge_data;    //接收到数据的功能码数据。
extern __IO uint8_t Brightness; 
extern uint8_t repcdata[3]; 
extern uint8_t aTxBuffer[100];       // 串口DMA发送缓冲区
extern uint8_t aRxBuffer[256]; 
extern  __IO uint16_t  home_position    ;
extern uint8_t SPI_aRxBuffer[100];
extern uint8_t SPI_aTxBuffer[6];

extern uint8_t  SPI_RX_FLAG;
extern uint8_t SPI_RX_DATA;
extern uint8_t I2C_TX_FLAG;
extern uint8_t I2C_TX_DATA;
__IO uint8_t A2_RX_STOP=0;    //第二个马达，接受马达停止标志位。
__IO uint8_t A2_ReadPulse=0; //读取第二个马达的脉冲数标志位
extern __IO uint8_t stop_flag; //开机标志位
/**********************************************************
 *
 *函数名称：
 *函数功能：A2马达操作函数功能
 *参数:无
 *返回值：无
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
			  	   // DRV8825_SLEEP_DISABLE(); //高电平开始工作,解除休眠状态
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
					 // DRV8825_SLEEP_DISABLE(); //高电平开始工作,解除休眠状态	
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
					//STEPMOTOR_AxisMoveRel(335544*SPR, Toggle_Pulse);---用于测试模式
				
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
                     // DRV8825_SLEEP_DISABLE() ; //高电平开始工作
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

			 case 0xa0 :    //重新设置原点
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
		   case 0xff :  //同上位机通讯
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

            case 0x103 :   //读取指令  0x 1xx 
				 re_intrrupt_flag=0; 
				Display_EEPROM_Value();  //读取马达实时位置脉冲值
				break;
			case 0x104 :
				    re_intrrupt_flag=0; 
				    temp= LAMP_Read_BrightValue(); //读取亮度值
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
					/* 打印通过 DS18B20 序列号获取的温度值 */
					printf("获取该序列号器件的温度：%.1f\n",temperature);
					/* 1s 读取一次温度值 */
					HAL_Delay(1000);
					printf("获取该序列号器件的温度：%.1f\n",temperature);
					HAL_Delay(1000);
					
				break;
			case 0x101:
                re_intrrupt_flag=0; 
				PC_DRV8825_ReadSpeed(); //读取马达速度值
				break;
		    }

}//END if(interrupt_flag==1)
	 


/*********************************************************
 *
 *函数名称：A1_CONTROL_A2_MOTOR_FUN()
 *函数功能：第一个马达控制第二个马达功能函数
 *输入参数：无
 *返回值：无
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
				   case 0x82 :   //背离马达的方向移动。
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

			 case 0xa0 :    //重新设置原点
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
		   case 0xff:  //同上位机通讯
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
*函数名：A1_Read_A2_DATA()
*函数功能：向主控制板A1 发送一个数据
*输入参数: 无
*返回值：无
*
**********************************************/
void A1_Read_A2_DATA(void)	
{	
	     uint8_t temp;
		  switch(SPI_aRxBuffer[2])
			{
                #if 1
				case 0x03 :   //读取指令  读取马达实时位置脉冲数
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
				case 0x04 : //读取LED灯的亮度值
				        A2_ReadPulse=0;
						temp= LAMP_Read_BrightValue(); //读取亮度值
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
	    	  
 




