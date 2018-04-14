#ifndef __DRV8825_H
#define __DRV8825_H
#include "stm32f1xx_hal.h"

void DRV8825_CCW_AxisMoveRel(uint8_t ccir_high,uint8_t ccir_mid,uint8_t ccir_low,uint16_t speed); //马达逆时针方向旋转

void DRV8825_CW_AxisMoveRel(uint8_t cir_high,uint8_t cir_mdi, uint8_t cir_low,uint16_t speed); //马达后退,马达逆时针

void DRV8825_StopMove(void);   //马达停?

void DRV8825_SetSpeed(uint8_t mid_byte,uint8_t low_byte); //马达速度设定
uint16_t DRV8825_ReadSpeed(void);  //读取写入的EEPROM的速度值
void Brightness_SetValue(uint8_t lumedat); //灯光亮度值设定
void KEY_Test_Fun(void);//按键测试函数




void DRV8825_Save_CurrentPosition(void); //存储当前马达的位置步数。
uint32_t DRV8825_Read_CurrentPosition(void);  //读取马达当前位置步数?

void Display_EEPROM_Value(void);
void EEPROM_Clear_Buf(void);   //清楚eeprom 寄存器 清零。

void DRV8825_Save_CurrentPosition(void); //存储当前马达的位置步数。



void Display_EEPROM_Value(void);
void EEPROM_Clear_Buf(void);   //清楚eeprom 寄存器 清零。
void Set_NewOrigin_Position(void);    //设置新的坐标原点
void PC_DRV8825_ReadSpeed(void); //上位机读取马达的速度值

void A1_ReadRealTime_A2_Value(void);
void A1_ReadSpeed_A2_Value(void);
void A1_ReadEeprom_A2_Value(void);
void Motor_Save_EndPosition(void);//wt.edit 2018.04.03
uint8_t  A2_ReadRealTime_Judge_Stop(void);
#endif

