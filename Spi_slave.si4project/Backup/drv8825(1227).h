#ifndef __DRV8825_H
#define __DRV8825_H
#include "stm32f1xx_hal.h"

void DRV8825_CCW_AxisMoveRel(uint8_t ccir_high,uint8_t ccir_mid,uint8_t ccir_low,uint16_t speed); //�����ʱ�뷽����ת

void DRV8825_CW_AxisMoveRel(uint8_t cir_high,uint8_t cir_mdi, uint8_t cir_low,uint16_t speed); //������,�����ʱ��

void DRV8825_StopMove(void);   //���ͣ?
void DRV8825_Turn_ENABLE(void); //���ʹ������
void DRV8825_SetSpeed(uint8_t mid_byte,uint8_t low_byte); //����ٶ��趨
uint16_t DRV8825_ReadSpeed(void);  //��ȡд���EEPROM���ٶ�ֵ
void Brightness_SetValue(uint8_t lumedat); //�ƹ�����ֵ�趨
void KEY_Test_Fun(void);//�������Ժ���
uint16_t Read_Origin_Position(void);  //��ȡԭ���λ��


void DRV8825_Pulse_AxisMoveRel( int32_t rpulse, uint32_t rspeed); //�ײ���㺯����
void DRV8825_Pulse_AxisMoveAbs(int32_t targert_step, uint32_t speed); //����ƶ���������ģʽ����������λ�ơ�

void DRV8825_Save_CurrentPosition(void); //�洢��ǰ����λ�ò�����
uint32_t DRV8825_Read_CurrentPosition(void);  //��ȡ��ﵱǰλ�ò���?
void STEPMOTOR_CCW_AxisMoveAbs( uint8_t abs_high,uint8_t abs_mid,uint8_t abs_low, uint32_t speed); //����λ���ƶ�����-��ʱ��
void STEPMOTOR_CW_AxisMoveAbs( uint8_t abs_high,uint8_t abs_mid,uint8_t abs_low, uint32_t speed); //����λ���ƶ�����-˳ʱ��

void Display_EEPROM_Value(void);
void EEPROM_Clear_Buf(void);   //���eeprom �Ĵ��� ���㡣

void DRV8825_Save_CurrentPosition(void); //�洢��ǰ����λ�ò�����
uint32_t DRV8825_Read_CurrentPosition(void);  //��ȡ��ﵱǰλ�ò���?


void Display_EEPROM_Value(void);
void EEPROM_Clear_Buf(void);   //���eeprom �Ĵ��� ���㡣
void MOTOR_Save_NewHomePosition(void);   //�洢��ǰ��λ�ã���Ϊ��������ԭ�㣨��ǰ����ֵ��
void Set_NewOrigin_Position(void);    //�����µ�����ԭ��
void PC_DRV8825_ReadSpeed(void); //��λ����ȡ�����ٶ�ֵ

uint8_t A1_ReadRealTime_A2_Value(void);
void A1_ReadSpeed_A2_Value(void);
void A1_ReadEeprom_A2_Value(void);
void Motor_Save_EndPosition(void);//wt.edit 2018.04.03
uint8_t  A2_ReadRealTime_Judge_Stop(void);
#endif

