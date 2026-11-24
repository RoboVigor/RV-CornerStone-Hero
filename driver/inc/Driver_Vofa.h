#ifndef _DRIVER_VOFA_H
#define _DRIVER_VOFA_H

#include "stm32f4xx_conf.h"
#include "stm32f4xx_can.h"
#include "Driver_Motor.h"
#include "Driver_Protocol.h"
#include "Driver_CAN.h"
#include "Driver_Bridge.h"
#include "Driver_BSP.h"
#include "config.h"
#include "handle.h"

#define vofaTail_t 0x7F800000
/* @note 注意此协议数据为十六进制float，debug赋值时记得强转为float
*/
extern VofaData_type VofaShadow; // VofaData 指向它, 由 Task_Vofa_Send 整帧拷贝后发送
void Vofa_Send(uint16_t frequency, uint16_t heapDeth, Node_Type * node, uint32_t commandID);
void Task_Vofa_Send(void * Parameters);
// 文本输出 (命令回复). 仅允许单一生产者调用: Task_Cmd, 或调度器启动前的 main
void Vofa_Print(const char *fmt, ...);
// 通道回复: holdMs 内 debug0..5 显示 values, debug6 显示 code (负数状态码), 之后恢复正常数据流
void Vofa_Reply(const float values[6], float code, uint16_t holdMs);
// 暂停/恢复 JustFloat 数据流 (文本仍会发送), 便于在 VOFA 接收窗口阅读文本
void Vofa_Stream_Enable(uint8_t enable);
#endif 
