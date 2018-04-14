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
__IO uint8_t NewOrigin_flag=0;
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
__IO uint8_t NewOrigin_Flag=0;  //设置新的原点，标志位。
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

/****************************************
*
*函数功能：相对位置移动，以当前的位置计算。
*参数: 输入的16进制，2个字节?---脉冲数驱动
*返回值：无
*
***************************************/
void DRV8825_CCW_AxisMoveRel(uint8_t ccir_high,uint8_t ccir_mid,uint8_t ccir_low,uint16_t speed)
{
   uint32_t all_cir;
	 all_cir=ccir_high<<16|ccir_mid<<8|ccir_low;
	// STEPMOTOR_AxisMoveRel(1*all_cir*SPR,speed); 
	 STEPMOTOR_AxisMoveRel(1*all_cir,speed);   

}

/****************************************
 *
 *函数功能：相对位置移动，以当前的位置计算。
 *参数: 输入的16进制，3个字节，以脉冲数驱动
 *返回值：无
 *
***************************************/
void DRV8825_CW_AxisMoveRel(uint8_t cir_high,uint8_t cir_mid,uint8_t cir_low,uint16_t speed)
{
     int32_t all_cir;
	 all_cir=cir_high<<16|cir_mid<<8|cir_low;
	 //STEPMOTOR_AxisMoveRel(-1*all_cir*SPR,speed); 
	 STEPMOTOR_AxisMoveRel(-1*all_cir,speed); 

}
/**********************************
*
*函数功能：马达停止
*
**********************************/
void DRV8825_StopMove(void)
{
       
        
		stop_flag=1;
	   // PulseNumbers=0;  //
        HAL_TIM_OC_Stop_IT(&htimx_STEPMOTOR,STEPMOTOR_TIM_CHANNEL_x);
       // 关闭通道
       TIM_CCxChannelCmd(STEPMOTOR_TIMx, STEPMOTOR_TIM_CHANNEL_x, TIM_CCx_DISABLE);        
       __HAL_TIM_CLEAR_FLAG(&htimx_STEPMOTOR, STEPMOTOR_TIM_FLAG_CCx);
      // DRV8825_Save_CurrentPosition(); //wt.edit 2018.01.17
       DRV8825_SLEEP_ENABLE(); //进入省电模?
       DRV8825_Save_CurrentPosition(); //wt.edit 2018.01.17
       DRV8825_OUTPUT_DISABLE();    //DRV8825芯片高电平，没有输出?
       Display_EEPROM_Value();
	 //  GPIO_PB10_LOW;

}
/***********************************************************************
 *
 *函数功能：马达使能命令
 *输入参数：无
 *返回值：无
 *
************************************************************************/
void DRV8825_Turn_ENABLE(void)
{
    DRV8825_SLEEP_ENABLE(); //进入省电模式
	  DRV8825_OUTPUT_ENABLE();
	  LED1_OFF;
	  LED2_ON;
    HAL_Delay(2);
}
/************************************
*
*函数?  : Read_Origin_Position()?
*功能：读取原点的位置
*参数：无
*返回值：原点的位置坐标。
*
************************************/
uint16_t Read_Origin_Position(void)
{
     uint8_t flag;
	   uint16_t origin_pos;
	   flag=EEPROM_CheckOk();
	   if(flag==1)
		 {
	 //将EEPROM读出数据顺序保持到I2c_Buf_Read中  
   	 EEPROM_ReadBytes(I2c_Buf_Read, 0, 2); 
		 HAL_Delay(200);
     origin_pos=I2c_Buf_Read[0]<<8|I2c_Buf_Read[1];
		 return origin_pos;
    
	  }
		else return 0;
		
}

/************************************************************
*
 *底层计算函数
 * DRV8825_Pulse_AxisMvoeRel(int32_t pulse ,int32_t speed)
 *函数功能：以脉冲模式驱动,相对位移
 *    相对位置，转动一圈 ：200x32=6400个脉冲。
 *参数：pulse , 脉冲数。speed 速度。
 *返回值：无
*
*****************************************************************/
void DRV8825_Pulse_AxisMoveRel( int32_t pulse, uint32_t speed)
{
   uint16_t tim_count;//获取定时器的计数值
	 
  if(pulse < 0)      // 步数为负数
  {
    srd.dir = CCW; // 逆时针方向旋转
    STEPMOTOR_DIR_REVERSAL() ;
    pulse =-pulse;   // 获取步数绝对值
  }
  else
  {
    srd.dir = CW;  // 顺时针方向旋转
		STEPMOTOR_DIR_FORWARD();
	}
	
  srd.rel_step  = pulse;	//记录相对运动的步数

  if(pulse != 0)    // 如果目标运动步数不为0
  {
    // 我们的驱动器用户手册有详细的计算及推导过程

    // 设置最大速度极限, 计算得到min_delay用于定时器的计数器的值。
    // min_delay = (alpha / tt)/ w
   // srd.min_delay = (int32_t)(A_T_x10/speed);//以最大速度匀速运动
		Toggle_Pulse=DRV8825_ReadSpeed(); //wt.edit
		srd.min_delay=Toggle_Pulse;  //WT.edit
    srd.run_state = RUN;
  }
  tim_count=__HAL_TIM_GET_COUNTER(&htimx_STEPMOTOR);
  __HAL_TIM_SET_COMPARE(&htimx_STEPMOTOR,STEPMOTOR_TIM_CHANNEL_x,tim_count+srd.min_delay); // 设置定时器比较值
  TIM_CCxChannelCmd(STEPMOTOR_TIMx, STEPMOTOR_TIM_CHANNEL_x, TIM_CCx_ENABLE);// 使能定时器通道 
  DRV8825_SLEEP_DISABLE(); //高电平开始工作,解除休眠状态
	DRV8825_OUTPUT_ENABLE();

}
/******************************************************
*
* DRV8825_Pulse_AxisMvoeAbs(int32_t pulse ,int32_t speed)
*函数功能：以脉冲模式驱动，绝对位移。
*    相对位置，转动一圈 ：200x32=6400个脉冲。
*参数：pulse , 脉冲数。speed 速度
*返回值：无
*
*******************************************************/

void DRV8825_Pulse_AxisMoveAbs(int32_t targert_step, uint32_t speed)
{
   int32_t rel_step = 0;
	 int8_t dir = -1;
	 rel_step = step_position - targert_step ; 	//获取当前位置和目标位置之间的步数值
	
	if(rel_step == 0)	
	{
		dir = 0;
	}
	else dir = -1;
	DRV8825_Pulse_AxisMoveRel(dir*rel_step,speed);
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
       uint32_t temp1,temp2,temp3,temp4;
	   uint8_t flag,i;
	   uint8_t sendbuffer[6]={0xa1,0x03,00,00,00,0x0b};
	   uint32_t value,real_value;
	   
	   real_value=step_count;
      // send_realn=Decimal_TO_Hex(real_value);
	
	  
	   sendbuffer[4]=real_value & 0xff;
	   sendbuffer[3]=real_value>>8 & 0xff;
	   sendbuffer[2]=real_value>>16 & 0xff; 
      // HAL_UART_Transmit(&husartx,sendbuffer,6,20);
	   
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
		 
		 temp1=Hex2oct_MSB(I2c_Buf_Read[0]);  
	     temp2=Hex2oct_MD1(I2c_Buf_Read[1]);
	     temp3=Hex2oct_MD2(I2c_Buf_Read[2]);
	     temp4=Hex2oct_LSB(I2c_Buf_Read[3]);
		 value=temp1+temp2+temp3+temp4;
		 
		  
		// temp=(I2c_Buf_Read[3]<<24)|(I2c_Buf_Read[2]<<16)|(I2c_Buf_Read[1]<<8)|(I2c_Buf_Read[0]);
        // HAL_UART_Transmit(&husartx,I2c_Buf_Read,5,4);
		// printf("--------------------------- \n");
		// printf("temp current position= %ld \n",temp);
		 printf("EEPROM position= %ld \n",value);
		 printf("real position= %ld \n",real_value);
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

/****************************************************************
 *
 *函数功能:存储当前马达的位置,在EEPROM中的存储位置是:0~3-4个字节
 *输入参数：无
 *返回值：无
 *
****************************************************************/
void MOTOR_Save_NewHomePosition(void)
{
     uint8_t flag,flag_w,i;
     uint8_t a,b,c,d;
     a=PulseNumbers & 0xff;        //???λ
     b=PulseNumbers >>8 & 0xff;   //???λ
     c=PulseNumbers >>16 & 0xff;  //????λ
     d=PulseNumbers >>24 & 0xff;  //????λ,???λ
	
    
	  flag=EEPROM_CheckOk();
	  if(flag==1)
	  {
          for ( i=0; i<4; i++ ) //清除I2C_Buf_Read,and write
	      {
		      I2c_Buf_Read[i]=0;      // ???????????
		      I2c_Buf_Write[i]=0;      
	      }
		  I2c_Buf_Write[0]=d;      
	      I2c_Buf_Write[1]=c;
	      I2c_Buf_Write[2]=b;
	      I2c_Buf_Write[3]=a;
	     //??I2c_Buf_Write??????????????д??EERPOM?? 
		 flag_w=EEPROM_WriteBytes(I2c_Buf_Write, 0, 4); //位置是 0x00～0x03，四个字节
		 HAL_Delay(100);
		 #if 0
		 EEPROM_ReadBytes(I2c_Buf_Read, 4, 4); 
	     if(I2c_Buf_Read[0] == I2c_Buf_Write[0]) //
		 {
		    if(I2c_Buf_Read[1] == I2c_Buf_Write[1]) //
				if(I2c_Buf_Read[2] == I2c_Buf_Write[2]) //
					if(I2c_Buf_Read[3] == I2c_Buf_Write[3]) //
					{
						LED1_ON;
						LED2_ON;
						while(1);
					}
		
		
	      }
		  #endif 
		if(flag_w==1) //???д????
        {
        LED1_ON;
		LED2_ON;
		HAL_Delay(200);
		LED1_OFF;
		LED2_OFF;
    	}
		else 
		while(1);
     }
     else while(1);	  
	 
 }	 


/****************************************************************
 *
 *函数功能:存储马达设置原点的标志位，home_flag=1,
 *         home_flag==1,表示马达设置了新的原点,储存在EEPROM
           中的地址 0x04
 *输入参数：无
 *返回值：1---表示马达重新设置新的原点坐标。
 *
****************************************************************/
uint8_t MOTOR_NewHome_FalgSave(void)
{
     uint8_t flag,flag_w;
  
     flag=EEPROM_CheckOk();
	 if(flag==1)
	 {
	   I2c_Buf_Read[0]=0;      // ???????????
       I2c_Buf_Write[0]=1;      
      
    
	 flag_w=EEPROM_WriteBytes(I2c_Buf_Write, 4, 1); //地址：0x04,写入一个数据
	 HAL_Delay(100);
	 #if 0  //测试写入的数据
	 EEPROM_ReadBytes(I2c_Buf_Read, 4, 4); 
     if(I2c_Buf_Read[0] == I2c_Buf_Write[0]) //
	 {
	   LED1_ON;
	   LED2_ON;
	  while(1);
				
	 }
	 #endif 
	 }
     //??EEPROM??????????????I2c_Buf_Read??  
   	
     if(flag_w==1) //???д????
        {
        LED1_TOGGLE ;
		return 1;
    	}
	 else 
	 	return 0;
}	 


/******************************************************
  *
  *函数功能：是读取冲设置原点的标志位：home_flag
  *输入参数无：
  *返回参数： 1 ---表示有重新设置原点成功， 
  *           0----表示没有重新设置原点-失败。
  *
******************************************************/
uint8_t MOTOR_Read_NewHomeFlag(void)
{
       uint8_t flag;
	   uint32_t result;
	   flag=EEPROM_CheckOk();
	   if(flag==1)
		 {
    	 
        I2c_Buf_Read[0]=0;
		  EEPROM_ReadBytes(I2c_Buf_Read, 4, 1); //从地址：0x04,读一个字节。 
		  HAL_Delay(200);  
          result=I2c_Buf_Read[0];
		  LED2_TOGGLE ;
		 
     	}
		 return result;
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
    NewOrigin_Flag=1;

    PulseNumbers=0;
    step_count=0;
	step_position=0;  //wt.edit 18.04.03


	
    flag=EEPROM_CheckOk();
	  if(flag==1)
	  {
	      for ( i=0; i<10; i++ ) //娓呮 EEPROM 瀛樺偍鐨刡uffer娓呴浂
	      {
	      I2c_Buf_Read[i]=0;      // 清空接收缓冲区
	      I2c_Buf_Write[i]=0;
	      }
	     
     //将I2c_Buf_Write中顺序递增的数据写入EERPOM中 
	  flag_w=EEPROM_WriteBytes(I2c_Buf_Write, 0, 10); //0x05 ~0x09
		 HAL_Delay(10);
	   //将EEPROM读出数据顺序保持到I2c_Buf_Read中  
   	 //DRV8825_Save_CurrentPosition();
	  }	 
	  if(flag_w==1) //表示写入成功
	  {
	    LED1_ON;
		HAL_Delay(10);
		LED1_OFF;
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
void A1_ReadRealTime_A2_Value(void)
{
       uint32_t real_value;
	   real_value=step_count;
       i2c_tx_buffer[2]=real_value & 0xff;
	   i2c_tx_buffer[1]=real_value>>8 & 0xff;
	   i2c_tx_buffer[0]=real_value>>16 & 0xff; 
	 //  printf("RealTime= %ld \n",real_value);
      		  
}
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
