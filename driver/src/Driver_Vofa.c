#include "Driver_Vofa.h"
#include <stdarg.h>
#include <stdio.h>

static uint8_t buffer[sizeof(debug_info_t)];

// 写入方只写 VofaShadow; 发送任务在 DMA 停止后整帧拷贝到 DMA 缓冲区, 避免 DMA 读到写了一半的帧(撕裂)
// 写入方需在临界区内写完整帧, 见 tasks.c
VofaData_type VofaShadow;

/*******************************************************************************
 * 文本输出 (命令回复): 单生产者 (Task_Cmd / 调度器启动前的 main) 单消费者 (Task_Vofa_Send) 环形缓冲
 * 有文本待发时, 发送任务每周期发一段文本代替一帧 JustFloat 数据 (64B @460800 ≈ 1.4ms < 2ms 周期)
 * 注意: 文本夹在 JustFloat 帧之间, VOFA 会把文本与下一帧拼成一个错误帧丢弃 (曲线丢 1 帧)
 *******************************************************************************/
#define VOFA_TEXT_BUF   1024
#define VOFA_TEXT_CHUNK 64
static char              textRing[VOFA_TEXT_BUF];
static volatile uint16_t textHead = 0, textTail = 0;
static uint8_t           textChunk[VOFA_TEXT_CHUNK];

/*******************************************************************************
 * 通道回复: 在 holdMs 内用回复值替换 debug0..debug6 (debug6 = 状态码, 负数, 与帧号区分)
 *******************************************************************************/
static volatile float      replyVal[7];
static volatile TickType_t replyUntil  = 0;
static volatile uint8_t    replyActive = 0;
static volatile uint8_t    streamOn    = 1;

void Vofa_Print(const char *fmt, ...) {
    char     line[192];
    va_list  args;
    int      n, i;
    uint16_t next;

    va_start(args, fmt);
    n = vsnprintf(line, sizeof(line), fmt, args);
    va_end(args);
    if (n <= 0) return;
    if (n >= (int) sizeof(line)) n = sizeof(line) - 1;
    for (i = 0; i < n; i++) {
        next = (textHead + 1) % VOFA_TEXT_BUF;
        if (next == textTail) return; // 满: 丢弃剩余
        textRing[textHead] = line[i];
        textHead           = next;
    }
}

void Vofa_Reply(const float values[6], float code, uint16_t holdMs) {
    int i;
    taskENTER_CRITICAL();
    for (i = 0; i < 6; i++) replyVal[i] = values ? values[i] : 0;
    replyVal[6] = code;
    replyUntil  = xTaskGetTickCount() + holdMs;
    replyActive = 1;
    taskEXIT_CRITICAL();
}

void Vofa_Stream_Enable(uint8_t enable) {
    streamOn = enable;
}

void Task_Vofa_Send(void *Parameters) {
    TickType_t          LastWakeTime = xTaskGetTickCount();
    ProtocolInfo_Type  *protocolInfo = Parameters;
    Node_Type          *node         = protocolInfo->node;
    uint32_t            deviceID     = node->deviceID;
    DMA_Stream_TypeDef *stream       = DMA_Get_Stream(USARTx_Tx);
    uint16_t            n;
    while (1) {
        DMA_Disable(USARTx_Tx);
        while (DMA_GetCmdStatus(stream) != DISABLE) {
        }
        if (textTail != textHead) {
            n = 0;
            while (n < VOFA_TEXT_CHUNK && textTail != textHead) {
                textChunk[n++] = textRing[textTail];
                textTail       = (textTail + 1) % VOFA_TEXT_BUF;
            }
            stream->M0AR = (uint32_t) textChunk;
            DMA_Enable(USARTx_Tx, n);
        } else if (streamOn) {
            taskENTER_CRITICAL();
            ProtocolData.debugInfo.vofaData = VofaShadow;
            if (replyActive) {
                if ((int32_t) (xTaskGetTickCount() - replyUntil) < 0) {
                    ProtocolData.debugInfo.vofaData.debug0 = replyVal[0];
                    ProtocolData.debugInfo.vofaData.debug1 = replyVal[1];
                    ProtocolData.debugInfo.vofaData.debug2 = replyVal[2];
                    ProtocolData.debugInfo.vofaData.debug3 = replyVal[3];
                    ProtocolData.debugInfo.vofaData.debug4 = replyVal[4];
                    ProtocolData.debugInfo.vofaData.debug5 = replyVal[5];
                    ProtocolData.debugInfo.vofaData.debug6 = replyVal[6];
                } else {
                    replyActive = 0;
                }
            }
            taskEXIT_CRITICAL();
            stream->M0AR = (uint32_t) ProtocolData.debugInfo.data;
            DMA_Enable(USARTx_Tx, protocolInfo->length);
        }
        vTaskDelayUntil(&LastWakeTime, 1000.0f / protocolInfo->frequency);
    }
}

void Vofa_Send(uint16_t frequency, uint16_t heapDeth, Node_Type *node, uint32_t commandID) {
    ProtocolData.debugInfo.vofaData.vofaTail = vofaTail_t;
    VofaShadow.vofaTail                      = vofaTail_t;
    ProtocolInfo_Type *protocolInfo          = Protocol_Get_Info_Handle(commandID);
    if (protocolInfo->taskHandle != 0) {
        vTaskDelete(protocolInfo->taskHandle);
    }
    protocolInfo->node        = node;
    protocolInfo->frequency   = frequency;
    uint32_t deviceID         = node->deviceID;
    BSP_DMA_Init(USARTx_Tx, ProtocolData.debugInfo.data, Protocol_Buffer_Length);
    xTaskCreate(Task_Vofa_Send, "Vofa_Send", heapDeth, (void *) protocolInfo, 6, protocolInfo->taskHandle);
};
