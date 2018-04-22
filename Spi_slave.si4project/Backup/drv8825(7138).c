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
extern __IO uint8_t A2_ReadPulse; //读取第二个马达的脉冲数标志位

extern uint8_t i2c_tx_buffer[4];
extern __IO uint16_t Toggle_Pulse ;
extern __IO uint32_t pulse_count; /*  脉冲计数，一个完整的脉冲会增加2 */
extern uint8_t I2c_Buf_Write[256];
extern uint8_t I2c_Buf_Read[256];
extern speedRampData srd  ;
static __IO int32_t  step_position=0 ;
__IO uint8_t stop_flag=0;
extern __IO int32_t PulseNumbers;
extern __IO uint32_t step_count; 
__IO uint8_t home_flag=0; 
extern __IO uint32_t step_count;  //相PC机发送存储的马达脉冲数
extern __IO uint8_t back_flag;
extern __IO uint8_t re_intrrupt_flag; //从上位机，接收信号标志位，进入中断标志位
extern __IO uint8_t PB8_flag;
extern __IO uint16_t judge_data;    //接收到数据的功能码数据。
extern __IO uint8_t Brightness; 
extern uint8_t repcdata[3]; 
extern uint8_t aTxBuffer[100];       // 串口DMA发送缓冲区
extern uint8_t aRxBuffer[256]; 
extern  __IO uint16_t  home_position    ; 
extern __IO uint8_t END_STOP_FLAG;  //马达运行到终点，停止标志位
extern __IO int32_t  LimPosi; //正方向极限标志位，背离马达方向移动的最大距离
__IO uint8_t NewOrigin_flag=0;  //设置新的原点，标志位。
__IO uint8_t  RealTime_value=0;  //读取实时位置，马达移动停止标志位
/***************************************
*
 *函数功能:马达速度设置
 *参数：设定马达速度
 *DRV8825 nENBL --- Hight level is diable ,
 *              ----Low leve is enable
 *返回值：无
 *
****************************************/
void DRV8825_SetSpeed(uint8_t high_b,uint8_t low_b)
{
	  uint8_t i,flag;
	  flag=EEPROM_CheckOk();
	   if(flag==1)
	   {
     // for ( i=0; i<2; i++ ) //填充缓冲 ,
      {
      I2c_Buf_Read[0]=0;      // 清空接收缓冲区
      I2c_Buf_Read[1]=0;
      I2c_Buf_Write[0] = high_b;   // 为发送缓冲区填充数据
	  I2c_Buf_Write[1] = low_b;
      }
     //将I2c_Buf_Write中顺序递增的数据写入EERPOM中 
	 EEPROM_WriteBytes(I2c_Buf_Write, 16, 2);  
     HAL_Delay(100);
	
     //将EEPROM读出数据顺序保持到I2c_Buf_Read中  
   	 EEPROM_ReadBytes(I2c_Buf_Read, 16, 2); 
     for (i=0;i<2;i++)
     {    
      if(I2c_Buf_Read[i] != I2c_Buf_Write[i])
      {
        LED1_ON;
		LED2_ON;
		while(1); //wt.edit
      }
     }
    if(i==2) //表示写入成功
        {
        LED1_ON;
		LED2_ON;
		HAL_Delay(200);
		LED1_OFF;
		LED2_OFF;
    	}
	}	
	
}	
/*******************************************
*
*函数功能：读取速度值
*参数：无
*返回值：读取到的速度值 ,存放在数组,[2][3]
*
********************************************/
uint16_t DRV8825_ReadSpeed(void)
{
    uint8_t flag;
	  uint16_t temp;
	   flag=EEPROM_CheckOk();
	   if(flag==1)
		 {
    	 //将EEPROM读出数据顺序保持到I2c_Buf_Read中  
       	 EEPROM_ReadBytes(I2c_Buf_Read, 16, 2);  //地址是 0x0F,字节数是2
		 temp=I2c_Buf_Read[0]<<8|I2c_Buf_Read[1];
		
     	 }
		return temp;
    
}

/******************************************************************************
*
*函数功能：相对位置移动，以当前的位置计算。
*参数: 输入的16进制，2个字节?---脉冲数驱动
*返回值：无        order: 0x82---往前移动
*
******************************************************************************/
void DRV8825_CCW_AxisMoveRel(uint8_t ccir_high,uint8_t ccir_mid,uint8_t ccir_low,uint16_t speed)
{
     uint32_t all_cir;
     uint32_t temp4;
	 all_cir=ccir_high<<16|ccir_mid<<8|ccir_low;
	// STEPMOTOR_AxisMoveRel(1*all_cir*SPR,speed); 
     if(stop_flag==0)    //第一次开机，执行此语句  wt.edit 2018.04.13
	{
	  
	  stop_flag=35;
	  temp4=DRV8825_Read_CurrentPosition();   //负数--远离马达方向移动
	  step_count=temp4;
	  PulseNumbers=temp4;
	  step_position=temp4;
	
	}
	 STEPMOTOR_AxisMoveRel(1*all_cir,speed);   

}

/***************************************************************
 *
 *函数功能：相对位置移动，以当前的位置计算。
 *参数: 输入的16进制，3个字节，以脉冲数驱动
 *返回值：无       order:  0x02---往后移动
 *
***************************************************************/
void DRV8825_CW_AxisMoveRel(uint8_t cir_high,uint8_t cir_mid,uint8_t cir_low,uint16_t speed)
{
     int32_t all_cir;
     uint32_t temp4;
	 all_cir=cir_high<<16|cir_mid<<8|cir_low;
	 //STEPMOTOR_AxisMoveRel(-1*all_cir*SPR,speed); 
      if(stop_flag==0)    //第一次开机，执行此语句  wt.edit 2018.04.13
	{
	  
	  stop_flag=35;
	  temp4=DRV8825_Read_CurrentPosition();   //负数--远离马达方向移动
	  step_count=temp4;
	  PulseNumbers=temp4;
	  step_position=temp4;
	
	}
	 STEPMOTOR_AxisMoveRel(-1*all_cir,speed); 

}
/*********************************************************
  *
  *函数名称： 
  *函数功能：马达停止移动
  *输入参数：无
  *返回值：无
  * 
****************************************************/
void DRV8825_StopMove(void)
{
       stop_flag=1;
		// PulseNumbers=0;  //
        HAL_TIM_OC_Stop_IT(&htimx_STEPMOTOR,STEPMOTOR_TIM_CHANNEL_x);
       // 关闭通道
       TIM_CCxChannelCmd(STEPMOTOR_TIMx, STEPMOTOR_TIM_CHANNEL_x, TIM_CCx_DISABLE);        
       __HAL_TIM_CLEAR_FLAG(&htimx_STEPMOTOR, STEPMOTOR_TIM_FLAG_CCx);
       DRV8825_Save_CurrentPosition(); //wt.edit 2018.01.17
       DRV8825_SLEEP_ENABLE(); //进入省电模?
       //DRV8825_Save_CurrentPosition(); //wt.edit 2018.01.17
       DRV8825_OUTPUT_DISABLE();    //DRV8825芯片高电平，没有输出?
       Display_EEPROM_Value();
	 //  GPIO_PB10_LOW;

}


/****************************************************************
*
*函数功能：存储马达当前的位置,step_position
*参数：无
*返回值：无
*
****************************************************************/
void DRV8825_Save_CurrentPosition(void)
{
     uint8_t flag,flag_w,i;
     uint32_t a,b,c,d;
    // a=PulseNumbers & 0xff;        //最高位
    // b=PulseNumbers >>8 & 0xff;   //第二位
    // c=PulseNumbers >>16 & 0xff;  //第三位
    // d=PulseNumbers >>24 & 0xff;  //第四位,最低位
	 a=step_count & 0xff;        //最高位
     b=step_count >>8 & 0xff;   //第二位
     c=step_count >>16 & 0xff;  //第三位
     d=step_count >>24 & 0xff;  //第四位,最低位
	
    
	  flag=EEPROM_CheckOk();
	  if(flag==1)
	  {
	      for ( i=0; i<4; i++ ) //填充缓冲 ,
	      {
	      I2c_Buf_Read[i]=0;      // 清空接收缓冲区
	      I2c_Buf_Write[i]=0;
	      
	      }
	      I2c_Buf_Write[0]=d;      
	      I2c_Buf_Write[1]=c;
	      I2c_Buf_Write[2]=b;
	      I2c_Buf_Write[3]=a;
	     
      
	     //将I2c_Buf_Write中顺序递增的数据写入EERPOM中 
		 flag_w=EEPROM_WriteBytes(I2c_Buf_Write, 5, 4); //0x05 ~0x09
		 HAL_Delay(200);
		 #if 0
		  EEPROM_ReadBytes(I2c_Buf_Read, 5, 4); 
	     if(I2c_Buf_Read[0] == I2c_Buf_Write[0]) //
		 {
		    if(I2c_Buf_Read[1] == I2c_Buf_Write[1]) //
				if(I2c_Buf_Read[2] == I2c_Buf_Write[2]) //
				if(I2c_Buf_Read[3] == I2c_Buf_Write[3]) //
					{
						LED1_ON;
						LED2_ON;
						printf("eeprom is save OK \n");
						//while(1);
					}
		 
		  }
		  else printf("eeprom is save error \n");
		 #endif 
	
     //将EEPROM读出数据顺序保持到I2c_Buf_Read中  
   	
	     if(flag_w==1) //表示写入成功
	        {
	        LED1_ON;
			HAL_Delay(10);
			LED1_OFF;
		    }
		   else
         printf("eeprom check error \n");				 
		   	//while(1);
	}	 

}
/****************************************************************
 *
 *函数功能：读取马达当前位置马达当前的位置,step_position
 *参数：无
 *返回值：无 I2c_Buf_Read[0]=MSB    I2c_Buf_Read[3]=LSB
 *
****************************************************************/
uint32_t DRV8825_Read_CurrentPosition(void)
{
       uint8_t flag;
	   uint32_t result,temp1,temp2,temp3,temp4;
	   flag=EEPROM_CheckOk();
	   if(flag==1)
		 {
    	 //将EEPROM读出数据顺序保持到I2c_Buf_Read中  
       	 EEPROM_ReadBytes(I2c_Buf_Read, 5, 4); 
		  HAL_Delay(200);
		 //temp=(0xff & I2c_Buf_Read[3]<<24)|(0xff & I2c_Buf_Read[2]<<16)|(0xff & I2c_Buf_Read[1]<<8)|(I2c_Buf_Read[0] & 0xff);
		 temp1=Hex2oct_MSB(I2c_Buf_Read[0]);  
         temp2=Hex2oct_MD1(I2c_Buf_Read[1]);
         temp3=Hex2oct_MD2(I2c_Buf_Read[2]);
         temp4=Hex2oct_LSB(I2c_Buf_Read[3]);
			 
		 i2c_tx_buffer[0]=temp2; //通过
		 i2c_tx_buffer[1]=temp3;
		 i2c_tx_buffer[2]=temp4;
	   // HAL_UART_Transmit(&husartx,I2c_Buf_Read,4,5);
		result=temp1+temp2+temp3+temp4;
		
     	}
		return result;
     
}
/*********************************************************
 *
 * 函数功能：显示一个当前马达位置的参数函数。 
 * 输入参数：无
 * 返回参数无：
 *
***********************************************************/
void Display_EEPROM_Value(void)
{
      int32_t temp1,temp2,temp3,temp4;
	   uint8_t flag,i;
	   uint32_t real_value,step_numbers;
	   uint8_t sendbuffer[6]={0xa1,0x03,00,00,00,0x0b};
	   real_value=step_count;
      
       
       if(stop_flag==0)
       {
       
         flag=EEPROM_CheckOk();
	     if(flag==1)
		 {
    	  for(i=0;i<4;i++)
		  {
		    I2c_Buf_Read[i]=0;
		  }
		  /*??EEPROM??????????????I2c_Buf_Read?? */ 
       	  EEPROM_ReadBytes(I2c_Buf_Read, 5, 4);  //马达走的位置的数据，存储位置
		  HAL_Delay(200);

          sendbuffer[4]=I2c_Buf_Read[3];
	      sendbuffer[3]=I2c_Buf_Read[2];
	      sendbuffer[2]=I2c_Buf_Read[1];
		  
		 temp1=Hex2oct_MSB(I2c_Buf_Read[0]);  
	     temp2=Hex2oct_MD1(I2c_Buf_Read[1]);
	     temp3=Hex2oct_MD2(I2c_Buf_Read[2]);
	     temp4=Hex2oct_LSB(I2c_Buf_Read[3]);
		 real_value=temp1+temp2+temp3+temp4;
		 
		  
		// temp=(I2c_Buf_Read[3]<<24)|(I2c_Buf_Read[2]<<16)|(I2c_Buf_Read[1]<<8)|(I2c_Buf_Read[0]);
      
	      printf("real position= %ld \n",real_value);
         HAL_UART_Transmit(&husartx,sendbuffer,6,12);		
       
         }
        }
       else 
       {
           sendbuffer[4]=real_value & 0xff;
           sendbuffer[3]=real_value>>8 & 0xff;
           sendbuffer[2]=real_value>>16 & 0xff;
           
           step_numbers=step_position;   
           
           printf("real position= %ld \n",real_value);
           printf("step_numbers= %d \n",step_numbers);
           HAL_UART_Transmit(&husartx,sendbuffer,6,12);  
       } 
		 
}

/******************************************
*
*函数功能：清楚EEPROM的存储器的值位0
*
*
*
************************************************/
void EEPROM_Clear_Buf(void)
{
 uint16_t i;
  uint8_t flag,flag_w;
  flag=EEPROM_CheckOk();
  
  if(flag==1)
  {
    for ( i=0; i<10; i++ ) //??仺?? ,
      {
      I2c_Buf_Read[i]=0;      // ???????????
      I2c_Buf_Write[i]=0;      
      }
     //??I2c_Buf_Write??????????????д??EERPOM?? 
	 flag_w=EEPROM_WriteBytes(I2c_Buf_Write, 0, 9); //
	 step_count=0; //wt.edit 2018.02.23
	 HAL_Delay(100);
  }
  if(flag_w==1)
 {
   LED1_ON;
   HAL_Delay(20);
   LED1_OFF;
 }
  else while(1);

}

/***************************************************
 *
 *函数功能：设置新的坐标原点
 *
 *
*******************************************************/
void Set_NewOrigin_Position(void)
{
    uint8_t flag,flag_w,i;
    NewOrigin_flag=1;  //wt.2018.04.11 add item

    PulseNumbers=0;
    step_count=0;
	step_position=0;  //wt.edit 18.04.03
	pulse_count=0;
    #if 1
	
    flag=EEPROM_CheckOk();
	  if(flag==1)
	  {
	      for ( i=0; i<12; i++ ) //娓呮 EEPROM 瀛樺偍鐨刡uffer娓呴浂
	      {
	       I2c_Buf_Read[i]=0;      // 清空接收缓冲区
	       I2c_Buf_Write[i]=0;
	      }
	     
     //将I2c_Buf_Write中顺序递增的数据写入EERPOM中 
	  flag_w=EEPROM_WriteBytes(I2c_Buf_Write, 0, 10); //0x05 ~0x09
		 HAL_Delay(10);
	   //将EEPROM读出数据顺序保持到I2c_Buf_Read中  
   	    DRV8825_Save_CurrentPosition();
	  }	 
	  if(flag_w==1) //表示写入成功
	  {
	    LED1_ON;
		HAL_Delay(50);
		LED1_OFF;
		HAL_Delay(50);
		LED1_ON;
	 }
	else 
   	{
      LED1_ON;
	  LED2_ON;
      HAL_Delay(500);
      LED1_OFF;
	  LED2_OFF;
	  HAL_Delay(500);
      LED1_ON;
	  LED2_ON;
	   HAL_Delay(500);
      LED1_OFF;
	  LED2_OFF;
    }
		   
    #endif 

}

/*******************************************
 *
 *函数功能：上位机读取马达的速度值
 *参数：无
 *返回值：读取到的速度值 ,存放在数组,[2][3]
 *
********************************************/
void PC_DRV8825_ReadSpeed(void)
{
		  uint8_t flag;
		  uint16_t temp;
		  uint8_t sendbuffer[6]={0xa1,0x01,00,00,00,0x0b};
		   flag=EEPROM_CheckOk();
		   if(flag==1)
			 {
			 //将EEPROM读出数据顺序保持到I2c_Buf_Read中	
			 EEPROM_ReadBytes(I2c_Buf_Read, 16, 2);  //地址是 0x0F,字节数是2
			 temp=I2c_Buf_Read[0]<<8|I2c_Buf_Read[1];
			 }
		    sendbuffer[4]=I2c_Buf_Read[1];
	        sendbuffer[3]=I2c_Buf_Read[0];
	       // sendbuffer[2]=I2c_Buf_Read[1];
           i2c_tx_buffer[1]=I2c_Buf_Read[0];   //传给上位机，通过马达1，传给上位机
		   i2c_tx_buffer[2]=I2c_Buf_Read[1];			 
		   printf("Motor Speed is : %d \n",temp);
		   HAL_UART_Transmit(&husartx,sendbuffer,6,12);	
		  
			
}
/***********************************************************
 *
 *函数名：
 *函数功能：第一马达读取第二马达速度值
 *参数：无
 *返回值：无
 *
************************************************************/
void A2_DRV8825_ReadSpeed(void)
{
           uint8_t flag;
		   //uint16_t temp;
		   flag=EEPROM_CheckOk();
		   if(flag==1)
			 {
			 //将EEPROM读出数据顺序保持到I2c_Buf_Read中	
			 EEPROM_ReadBytes(I2c_Buf_Read, 16, 2);  //地址是 0x0F,字节数是2
			// temp=I2c_Buf_Read[0]<<8|I2c_Buf_Read[1];
			 }
			 i2c_tx_buffer[2]=I2c_Buf_Read[1];
	         i2c_tx_buffer[1]=I2c_Buf_Read[0];  
}
/**********************************************
 *
 *函数名：A2_EEPROM_VALUE(void)
 *函数功能 ：传递一个实时脉冲值，给上位机
 *输入参数：无
 *返回值：无
 *
************************************************/
void  A1_ReadRealTime_A2_Value(void)
{
       
       int32_t temp1,temp2,temp3,temp4;
	   uint8_t flag,i;
	   uint32_t real_value,step_numbers,judge_value;
	   real_value=step_count;
      
	
	   if(stop_flag==0)
       {
       
         flag=EEPROM_CheckOk();
	     if(flag==1)
		 {
    	  for(i=0;i<4;i++)
		  {
		    I2c_Buf_Read[i]=0;
		  }
		  /*??EEPROM??????????????I2c_Buf_Read?? */ 
       	  EEPROM_ReadBytes(I2c_Buf_Read, 5, 4);  //马达走的位置的数据，存储位置
		  HAL_Delay(200);

          i2c_tx_buffer[2]=I2c_Buf_Read[3];
	      i2c_tx_buffer[1]=I2c_Buf_Read[2];
	      i2c_tx_buffer[0]=I2c_Buf_Read[1];
		  
		 
	     temp2=Hex2oct_MD1(I2c_Buf_Read[1]);
	     temp3=Hex2oct_MD2(I2c_Buf_Read[2]);
	     temp4=Hex2oct_LSB(I2c_Buf_Read[3]);
		 real_value=temp1+temp2+temp3+temp4;
		 
		  
		// temp=(I2c_Buf_Read[3]<<24)|(I2c_Buf_Read[2]<<16)|(I2c_Buf_Read[1]<<8)|(I2c_Buf_Read[0]);
      
	      printf("real position= %ld \n",real_value);
          I2C_MASTER_TX_DATA();
       
         }
        }
       else 
       {
            i2c_tx_buffer[2]=real_value & 0xff;
	        i2c_tx_buffer[1]=real_value>>8 & 0xff;
	        i2c_tx_buffer[0]=real_value>>16 & 0xff; //最高数据位
           
         
           
           printf("real position= %ld \n",real_value);
          
		   I2C_MASTER_TX_DATA();
          
       } 
       
        
      
}
/**********************************************************
 *
 *函数功能：判断马达是否停止
 *
 *
******************************************************/
#if 1
uint8_t  A2_ReadRealTime_Judge_Stop(void)
{
       uint32_t real_value,judge_value;
       uint8_t i;
	   real_value=step_count;
     

        judge_value=step_count;
        if(real_value==judge_value)
        {
           HAL_Delay(1000);
		   if(real_value==judge_value)
		   return 1; 
        }
        else 
            return 0;
 }
        

#endif
/***********************************************************
*
*函数名：
*函数功能：第一马达读取第二马达速度值
*参数：无
*返回值：无
*
************************************************************/
void A1_ReadSpeed_A2_Value(void)
{
           uint8_t flag;
		   //uint16_t temp;
		   flag=EEPROM_CheckOk();
		   if(flag==1)
			 {
			 //将EEPROM读出数据顺序保持到I2c_Buf_Read中	
			 EEPROM_ReadBytes(I2c_Buf_Read, 16, 2);  //地址是 0x0F,字节数是2
			// temp=I2c_Buf_Read[0]<<8|I2c_Buf_Read[1];
			 }
			 i2c_tx_buffer[2]=I2c_Buf_Read[1];
	         i2c_tx_buffer[1]=I2c_Buf_Read[0];  
}
/**************************************************************
*
*函数名： A1_ReadEeprom_A2_Value()
*函数功能：读取A2 马达EEPROM 存储的脉冲数
*参数：无
*返回值：无
*
***************************************************************/
void A1_ReadEeprom_A2_Value(void)
{
       uint32_t temp1,temp2,temp3,temp4;
	   uint8_t flag,i;
	   uint32_t value;
	   
	   flag=EEPROM_CheckOk();
	   if(flag==1)
		 {
    	  for(i=0;i<4;i++)
		  {
		    I2c_Buf_Read[i]=0;
		  }
		  /*??EEPROM??????????????I2c_Buf_Read?? */ 
       	 EEPROM_ReadBytes(I2c_Buf_Read, 5, 4);  //马达走的位置的数据，存储位置
		  HAL_Delay(10);
	    
		 
		 i2c_tx_buffer[0]=I2c_Buf_Read[1]; //通过马达1，上传给上位机，wt.edit 18.03.22
	     i2c_tx_buffer[1]=I2c_Buf_Read[2];
	     i2c_tx_buffer[2]=I2c_Buf_Read[3];
		
		 temp1=Hex2oct_MSB(I2c_Buf_Read[0]);  
	     temp2=Hex2oct_MD1(I2c_Buf_Read[1]);
	     temp3=Hex2oct_MD2(I2c_Buf_Read[2]);
	     temp4=Hex2oct_LSB(I2c_Buf_Read[3]);
		 
		
		 value=temp1+temp2+temp3+temp4;
		 
		  
		// temp=(I2c_Buf_Read[3]<<24)|(I2c_Buf_Read[2]<<16)|(I2c_Buf_Read[1]<<8)|(I2c_Buf_Read[0]);
          printf("EEPROM position= %d \n",value);
		
	    }  

}

/****************************************************************
 *
 *函数名称：
 *函数功能：存储马达到终点的位置，不清零，自动存取
 *参数：无
 *返回值：无
 *
****************************************************************/
void Motor_Save_EndPosition(void)
{
     uint8_t flag,flag_w,i;
     uint32_t a,b,c,d;
   
	
	 a=step_count & 0xff;        //最高位
     b=step_count >>8 & 0xff;   //第二位
     c=step_count >>16 & 0xff;  //第三位
     d=step_count >>24 & 0xff;  //第四位,最低位
    
	  flag=EEPROM_CheckOk();
	  if(flag==1)
	  {
	      for ( i=0; i<4; i++ ) //填充缓冲 ,
	      {
	      I2c_Buf_Read[i]=0;      // 清空接收缓冲区
	      I2c_Buf_Write[i]=0;
	      
	      }
	      I2c_Buf_Write[0]=d;      
	      I2c_Buf_Write[1]=c;
	      I2c_Buf_Write[2]=b;
	      I2c_Buf_Write[3]=a;
	     
      
	     //将I2c_Buf_Write中顺序递增的数据写入EERPOM中 
		 flag_w=EEPROM_WriteBytes(I2c_Buf_Write, 5, 4); //0x05 ~0x09
		 HAL_Delay(200);
		 #if 0
		  EEPROM_ReadBytes(I2c_Buf_Read, 5, 4); 
	   if(I2c_Buf_Read[0] == I2c_Buf_Write[0]) //
		 {
		    if(I2c_Buf_Read[1] == I2c_Buf_Write[1]) //
				if(I2c_Buf_Read[2] == I2c_Buf_Write[2]) //
				if(I2c_Buf_Read[3] == I2c_Buf_Write[3]) //
					{
						LED1_ON;
						LED2_ON;
						printf("eeprom is save OK \n");
						//while(1);
					}
		 
		 }
		 else printf("eeprom is save error \n");
		 #endif 
	
     //将EEPROM读出数据顺序保持到I2c_Buf_Read中  
   	
	     if(flag_w==1) //表示写入成功
	        {
	        LED1_ON;
			HAL_Delay(10);
			LED1_OFF;
		    }
		   else
         printf("eeprom check error \n");				 
		   	//while(1);
	}	 

}
