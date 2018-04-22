#ifndef __DRV8825_H
#define __DRV8825_H
#include "stm32f1xx_hal.h"

void DRV8825_CCW_AxisMoveRel(uint8_t ccir_high,uint8_t ccir_mid,uint8_t ccir_low,uint16_t speed); //马达逆时针方向旋转

void DRV8825_CW_AxisMoveRel(uint8_t cir_high,uint8_t cir_mdi, uint8_t cir_low,uint16_t speed); //马达后退,马达逆时针

void DRV8825_StopMove(void);   //马达停?
void DRV8825_Turn_ENABLE(void); //马达使能命令
void DRV8825_SetSpeed(uint8_t mid_byte,uint8_t low_byte); //马达速度设定
uint16_t DRV8825_ReadSpeed(void);  //读取写入的EEPROM的速度值
void Brightness_SetValue(uint8_t lumedat); //灯光亮度值设定
void KEY_Test_Fun(void);//按键测试函数
uint16_t Read_Origin_Position(void);  //读取原点的位置


void DRV8825_Pulse_AxisMoveRel( int32_t rpulse, uint32_t rspeed); //底层计算函数，
void DRV8825_Pulse_AxisMoveAbs(int32_t targert_step, uint32_t speed); //马达移动，以脉冲模式驱动，绝对位移。

void DRV8825_Save_CurrentPosition(void); //存储当前马达的位置步数。
uint32_t DRV8825_Read_CurrentPosition(void);  //读取马达当前位置步数?
void STEPMOTOR_CCW_AxisMoveAbs( uint8_t abs_high,uint8_t abs_mid,uint8_t abs_low, uint32_t speed); //绝对位置移动函数-逆时针
void STEPMOTOR_CW_AxisMoveAbs( uint8_t abs_high,uint8_t abs_mid,uint8_t abs_low, uint32_t speed); //绝对位置移动函数-顺时针

void Display_EEPROM_Value(void);
void EEPROM_Clear_Buf(void);   //清楚eeprom 寄存器 清零。

void DRV8825_Save_CurrentPosition(void); //存储当前马达的位置步数。
uint32_t DRV8825_Read_CurrentPosition(void);  //读取马达当前位置步数?


void Display_EEPROM_Value(void);
void EEPROM_Clear_Buf(void);   //清楚eeprom 寄存器 清零。
void MOTOR_Save_NewHomePosition(void);   //存储当前的位置，作为马达的坐标原点（当前坐标值）
void Set_NewOrigin_Position(void);    //设置新的坐标原点
void PC_DRV8825_ReadSpeed(void); //上位机读取马达的速度值

uint8_t A1_ReadRealTime_A2_Value(void);
void A1_ReadSpeed_A2_Value(void);
void A1_ReadEeprom_A2_Value(void);
void Motor_Save_EndPosition(void);//wt.edit 2018.04.03
uint8_t  A2_ReadRealTime_Judge_Stop(void);
#endif

