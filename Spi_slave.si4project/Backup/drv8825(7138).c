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
extern __IO uint8_t A2_ReadPulse; //��ȡ�ڶ����������������־λ

extern uint8_t i2c_tx_buffer[4];
extern __IO uint16_t Toggle_Pulse ;
extern __IO uint32_t pulse_count; /*  ���������һ�����������������2 */
extern uint8_t I2c_Buf_Write[256];
extern uint8_t I2c_Buf_Read[256];
extern speedRampData srd  ;
static __IO int32_t  step_position=0 ;
__IO uint8_t stop_flag=0;
extern __IO int32_t PulseNumbers;
extern __IO uint32_t step_count; 
__IO uint8_t home_flag=0; 
extern __IO uint32_t step_count;  //��PC�����ʹ洢������������
extern __IO uint8_t back_flag;
extern __IO uint8_t re_intrrupt_flag; //����λ���������źű�־λ�������жϱ�־λ
extern __IO uint8_t PB8_flag;
extern __IO uint16_t judge_data;    //���յ����ݵĹ��������ݡ�
extern __IO uint8_t Brightness; 
extern uint8_t repcdata[3]; 
extern uint8_t aTxBuffer[100];       // ����DMA���ͻ�����
extern uint8_t aRxBuffer[256]; 
extern  __IO uint16_t  home_position    ; 
extern __IO uint8_t END_STOP_FLAG;  //�������е��յ㣬ֹͣ��־λ
extern __IO int32_t  LimPosi; //�������ޱ�־λ���������﷽���ƶ���������
__IO uint8_t NewOrigin_flag=0;  //�����µ�ԭ�㣬��־λ��
__IO uint8_t  RealTime_value=0;  //��ȡʵʱλ�ã������ƶ�ֹͣ��־λ
/***************************************
*
 *��������:�����ٶ�����
 *�������趨�����ٶ�
 *DRV8825 nENBL --- Hight level is diable ,
 *              ----Low leve is enable
 *����ֵ����
 *
****************************************/
void DRV8825_SetSpeed(uint8_t high_b,uint8_t low_b)
{
	  uint8_t i,flag;
	  flag=EEPROM_CheckOk();
	   if(flag==1)
	   {
     // for ( i=0; i<2; i++ ) //��仺�� ,
      {
      I2c_Buf_Read[0]=0;      // ��ս��ջ�����
      I2c_Buf_Read[1]=0;
      I2c_Buf_Write[0] = high_b;   // Ϊ���ͻ������������
	  I2c_Buf_Write[1] = low_b;
      }
     //��I2c_Buf_Write��˳�����������д��EERPOM�� 
	 EEPROM_WriteBytes(I2c_Buf_Write, 16, 2);  
     HAL_Delay(100);
	
     //��EEPROM��������˳�򱣳ֵ�I2c_Buf_Read��  
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
    if(i==2) //��ʾд��ɹ�
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
*�������ܣ���ȡ�ٶ�ֵ
*��������
*����ֵ����ȡ�����ٶ�ֵ ,���������,[2][3]
*
********************************************/
uint16_t DRV8825_ReadSpeed(void)
{
    uint8_t flag;
	  uint16_t temp;
	   flag=EEPROM_CheckOk();
	   if(flag==1)
		 {
    	 //��EEPROM��������˳�򱣳ֵ�I2c_Buf_Read��  
       	 EEPROM_ReadBytes(I2c_Buf_Read, 16, 2);  //��ַ�� 0x0F,�ֽ�����2
		 temp=I2c_Buf_Read[0]<<8|I2c_Buf_Read[1];
		
     	 }
		return temp;
    
}

/******************************************************************************
*
*�������ܣ����λ���ƶ����Ե�ǰ��λ�ü��㡣
*����: �����16���ƣ�2���ֽ�?---����������
*����ֵ����        order: 0x82---��ǰ�ƶ�
*
******************************************************************************/
void DRV8825_CCW_AxisMoveRel(uint8_t ccir_high,uint8_t ccir_mid,uint8_t ccir_low,uint16_t speed)
{
     uint32_t all_cir;
     uint32_t temp4;
	 all_cir=ccir_high<<16|ccir_mid<<8|ccir_low;
	// STEPMOTOR_AxisMoveRel(1*all_cir*SPR,speed); 
     if(stop_flag==0)    //��һ�ο�����ִ�д����  wt.edit 2018.04.13
	{
	  
	  stop_flag=35;
	  temp4=DRV8825_Read_CurrentPosition();   //����--Զ�����﷽���ƶ�
	  step_count=temp4;
	  PulseNumbers=temp4;
	  step_position=temp4;
	
	}
	 STEPMOTOR_AxisMoveRel(1*all_cir,speed);   

}

/***************************************************************
 *
 *�������ܣ����λ���ƶ����Ե�ǰ��λ�ü��㡣
 *����: �����16���ƣ�3���ֽڣ�������������
 *����ֵ����       order:  0x02---�����ƶ�
 *
***************************************************************/
void DRV8825_CW_AxisMoveRel(uint8_t cir_high,uint8_t cir_mid,uint8_t cir_low,uint16_t speed)
{
     int32_t all_cir;
     uint32_t temp4;
	 all_cir=cir_high<<16|cir_mid<<8|cir_low;
	 //STEPMOTOR_AxisMoveRel(-1*all_cir*SPR,speed); 
      if(stop_flag==0)    //��һ�ο�����ִ�д����  wt.edit 2018.04.13
	{
	  
	  stop_flag=35;
	  temp4=DRV8825_Read_CurrentPosition();   //����--Զ�����﷽���ƶ�
	  step_count=temp4;
	  PulseNumbers=temp4;
	  step_position=temp4;
	
	}
	 STEPMOTOR_AxisMoveRel(-1*all_cir,speed); 

}
/*********************************************************
  *
  *�������ƣ� 
  *�������ܣ�����ֹͣ�ƶ�
  *�����������
  *����ֵ����
  * 
****************************************************/
void DRV8825_StopMove(void)
{
       stop_flag=1;
		// PulseNumbers=0;  //
        HAL_TIM_OC_Stop_IT(&htimx_STEPMOTOR,STEPMOTOR_TIM_CHANNEL_x);
       // �ر�ͨ��
       TIM_CCxChannelCmd(STEPMOTOR_TIMx, STEPMOTOR_TIM_CHANNEL_x, TIM_CCx_DISABLE);        
       __HAL_TIM_CLEAR_FLAG(&htimx_STEPMOTOR, STEPMOTOR_TIM_FLAG_CCx);
       DRV8825_Save_CurrentPosition(); //wt.edit 2018.01.17
       DRV8825_SLEEP_ENABLE(); //����ʡ��ģ?
       //DRV8825_Save_CurrentPosition(); //wt.edit 2018.01.17
       DRV8825_OUTPUT_DISABLE();    //DRV8825оƬ�ߵ�ƽ��û�����?
       Display_EEPROM_Value();
	 //  GPIO_PB10_LOW;

}


/****************************************************************
*
*�������ܣ��洢���ﵱǰ��λ��,step_position
*��������
*����ֵ����
*
****************************************************************/
void DRV8825_Save_CurrentPosition(void)
{
     uint8_t flag,flag_w,i;
     uint32_t a,b,c,d;
    // a=PulseNumbers & 0xff;        //���λ
    // b=PulseNumbers >>8 & 0xff;   //�ڶ�λ
    // c=PulseNumbers >>16 & 0xff;  //����λ
    // d=PulseNumbers >>24 & 0xff;  //����λ,���λ
	 a=step_count & 0xff;        //���λ
     b=step_count >>8 & 0xff;   //�ڶ�λ
     c=step_count >>16 & 0xff;  //����λ
     d=step_count >>24 & 0xff;  //����λ,���λ
	
    
	  flag=EEPROM_CheckOk();
	  if(flag==1)
	  {
	      for ( i=0; i<4; i++ ) //��仺�� ,
	      {
	      I2c_Buf_Read[i]=0;      // ��ս��ջ�����
	      I2c_Buf_Write[i]=0;
	      
	      }
	      I2c_Buf_Write[0]=d;      
	      I2c_Buf_Write[1]=c;
	      I2c_Buf_Write[2]=b;
	      I2c_Buf_Write[3]=a;
	     
      
	     //��I2c_Buf_Write��˳�����������д��EERPOM�� 
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
	
     //��EEPROM��������˳�򱣳ֵ�I2c_Buf_Read��  
   	
	     if(flag_w==1) //��ʾд��ɹ�
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
 *�������ܣ���ȡ���ﵱǰλ�����ﵱǰ��λ��,step_position
 *��������
 *����ֵ���� I2c_Buf_Read[0]=MSB    I2c_Buf_Read[3]=LSB
 *
****************************************************************/
uint32_t DRV8825_Read_CurrentPosition(void)
{
       uint8_t flag;
	   uint32_t result,temp1,temp2,temp3,temp4;
	   flag=EEPROM_CheckOk();
	   if(flag==1)
		 {
    	 //��EEPROM��������˳�򱣳ֵ�I2c_Buf_Read��  
       	 EEPROM_ReadBytes(I2c_Buf_Read, 5, 4); 
		  HAL_Delay(200);
		 //temp=(0xff & I2c_Buf_Read[3]<<24)|(0xff & I2c_Buf_Read[2]<<16)|(0xff & I2c_Buf_Read[1]<<8)|(I2c_Buf_Read[0] & 0xff);
		 temp1=Hex2oct_MSB(I2c_Buf_Read[0]);  
         temp2=Hex2oct_MD1(I2c_Buf_Read[1]);
         temp3=Hex2oct_MD2(I2c_Buf_Read[2]);
         temp4=Hex2oct_LSB(I2c_Buf_Read[3]);
			 
		 i2c_tx_buffer[0]=temp2; //ͨ��
		 i2c_tx_buffer[1]=temp3;
		 i2c_tx_buffer[2]=temp4;
	   // HAL_UART_Transmit(&husartx,I2c_Buf_Read,4,5);
		result=temp1+temp2+temp3+temp4;
		
     	}
		return result;
     
}
/*********************************************************
 *
 * �������ܣ���ʾһ����ǰ����λ�õĲ��������� 
 * �����������
 * ���ز����ޣ�
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
       	  EEPROM_ReadBytes(I2c_Buf_Read, 5, 4);  //�����ߵ�λ�õ����ݣ��洢λ��
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
*�������ܣ����EEPROM�Ĵ洢����ֵλ0
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
    for ( i=0; i<10; i++ ) //??��?? ,
      {
      I2c_Buf_Read[i]=0;      // ???????????
      I2c_Buf_Write[i]=0;      
      }
     //??I2c_Buf_Write??????????????��??EERPOM?? 
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
 *�������ܣ������µ�����ԭ��
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
	      for ( i=0; i<12; i++ ) //清楚 EEPROM 存储的buffer清零
	      {
	       I2c_Buf_Read[i]=0;      // ��ս��ջ�����
	       I2c_Buf_Write[i]=0;
	      }
	     
     //��I2c_Buf_Write��˳�����������д��EERPOM�� 
	  flag_w=EEPROM_WriteBytes(I2c_Buf_Write, 0, 10); //0x05 ~0x09
		 HAL_Delay(10);
	   //��EEPROM��������˳�򱣳ֵ�I2c_Buf_Read��  
   	    DRV8825_Save_CurrentPosition();
	  }	 
	  if(flag_w==1) //��ʾд��ɹ�
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
 *�������ܣ���λ����ȡ������ٶ�ֵ
 *��������
 *����ֵ����ȡ�����ٶ�ֵ ,���������,[2][3]
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
			 //��EEPROM��������˳�򱣳ֵ�I2c_Buf_Read��	
			 EEPROM_ReadBytes(I2c_Buf_Read, 16, 2);  //��ַ�� 0x0F,�ֽ�����2
			 temp=I2c_Buf_Read[0]<<8|I2c_Buf_Read[1];
			 }
		    sendbuffer[4]=I2c_Buf_Read[1];
	        sendbuffer[3]=I2c_Buf_Read[0];
	       // sendbuffer[2]=I2c_Buf_Read[1];
           i2c_tx_buffer[1]=I2c_Buf_Read[0];   //������λ����ͨ������1��������λ��
		   i2c_tx_buffer[2]=I2c_Buf_Read[1];			 
		   printf("Motor Speed is : %d \n",temp);
		   HAL_UART_Transmit(&husartx,sendbuffer,6,12);	
		  
			
}
/***********************************************************
 *
 *��������
 *�������ܣ���һ�����ȡ�ڶ������ٶ�ֵ
 *��������
 *����ֵ����
 *
************************************************************/
void A2_DRV8825_ReadSpeed(void)
{
           uint8_t flag;
		   //uint16_t temp;
		   flag=EEPROM_CheckOk();
		   if(flag==1)
			 {
			 //��EEPROM��������˳�򱣳ֵ�I2c_Buf_Read��	
			 EEPROM_ReadBytes(I2c_Buf_Read, 16, 2);  //��ַ�� 0x0F,�ֽ�����2
			// temp=I2c_Buf_Read[0]<<8|I2c_Buf_Read[1];
			 }
			 i2c_tx_buffer[2]=I2c_Buf_Read[1];
	         i2c_tx_buffer[1]=I2c_Buf_Read[0];  
}
/**********************************************
 *
 *��������A2_EEPROM_VALUE(void)
 *�������� ������һ��ʵʱ����ֵ������λ��
 *�����������
 *����ֵ����
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
       	  EEPROM_ReadBytes(I2c_Buf_Read, 5, 4);  //�����ߵ�λ�õ����ݣ��洢λ��
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
	        i2c_tx_buffer[0]=real_value>>16 & 0xff; //�������λ
           
         
           
           printf("real position= %ld \n",real_value);
          
		   I2C_MASTER_TX_DATA();
          
       } 
       
        
      
}
/**********************************************************
 *
 *�������ܣ��ж������Ƿ�ֹͣ
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
*��������
*�������ܣ���һ�����ȡ�ڶ������ٶ�ֵ
*��������
*����ֵ����
*
************************************************************/
void A1_ReadSpeed_A2_Value(void)
{
           uint8_t flag;
		   //uint16_t temp;
		   flag=EEPROM_CheckOk();
		   if(flag==1)
			 {
			 //��EEPROM��������˳�򱣳ֵ�I2c_Buf_Read��	
			 EEPROM_ReadBytes(I2c_Buf_Read, 16, 2);  //��ַ�� 0x0F,�ֽ�����2
			// temp=I2c_Buf_Read[0]<<8|I2c_Buf_Read[1];
			 }
			 i2c_tx_buffer[2]=I2c_Buf_Read[1];
	         i2c_tx_buffer[1]=I2c_Buf_Read[0];  
}
/**************************************************************
*
*�������� A1_ReadEeprom_A2_Value()
*�������ܣ���ȡA2 ����EEPROM �洢��������
*��������
*����ֵ����
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
       	 EEPROM_ReadBytes(I2c_Buf_Read, 5, 4);  //�����ߵ�λ�õ����ݣ��洢λ��
		  HAL_Delay(10);
	    
		 
		 i2c_tx_buffer[0]=I2c_Buf_Read[1]; //ͨ������1���ϴ�����λ����wt.edit 18.03.22
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
 *�������ƣ�
 *�������ܣ��洢���ﵽ�յ��λ�ã������㣬�Զ���ȡ
 *��������
 *����ֵ����
 *
****************************************************************/
void Motor_Save_EndPosition(void)
{
     uint8_t flag,flag_w,i;
     uint32_t a,b,c,d;
   
	
	 a=step_count & 0xff;        //���λ
     b=step_count >>8 & 0xff;   //�ڶ�λ
     c=step_count >>16 & 0xff;  //����λ
     d=step_count >>24 & 0xff;  //����λ,���λ
    
	  flag=EEPROM_CheckOk();
	  if(flag==1)
	  {
	      for ( i=0; i<4; i++ ) //��仺�� ,
	      {
	      I2c_Buf_Read[i]=0;      // ��ս��ջ�����
	      I2c_Buf_Write[i]=0;
	      
	      }
	      I2c_Buf_Write[0]=d;      
	      I2c_Buf_Write[1]=c;
	      I2c_Buf_Write[2]=b;
	      I2c_Buf_Write[3]=a;
	     
      
	     //��I2c_Buf_Write��˳�����������д��EERPOM�� 
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
	
     //��EEPROM��������˳�򱣳ֵ�I2c_Buf_Read��  
   	
	     if(flag_w==1) //��ʾд��ɹ�
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