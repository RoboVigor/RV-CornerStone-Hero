#ifndef __CMD_H
#define __CMD_H

#include <stdint.h>

/*******************************************************************************
 * 串口命令系统 (VOFA 命令面板 / 任意串口终端, USART6 460800)
 * - 一行一条命令, 以 '\r' / '\n' 或一次突发接收结束 (USART 空闲中断) 作为行尾,
 *   因此 VOFA 命令不必带换行; 但不支持逐字符键入 (每个字符都会被当作一行)
 * - 回复同时以两种方式输出:
 *     文本: 写入 VOFA 接收窗口 (数据流开着时会被二进制帧淹没, 可先 stream off)
 *     通道: 约 3s 内 debug0..5 显示数值, debug6 显示负数状态码 (见 cmd.c 头部表格)
 *******************************************************************************/

void Cmd_Rx_Bytes(const uint8_t *data, uint16_t len); // 在 USART6 中断中调用, 不调用任何 FreeRTOS API
void Task_Cmd(void *Parameters);

#endif
