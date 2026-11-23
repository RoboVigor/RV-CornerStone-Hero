#include "Driver_Vofa.h"

static uint8_t buffer[sizeof(debug_info_t)];

// 写入方只写 VofaShadow; 发送任务在 DMA 停止后整帧拷贝到 DMA 缓冲区, 避免 DMA 读到写了一半的帧(撕裂)
// 写入方需在临界区内写完整帧, 见 tasks.c
VofaData_type VofaShadow;

void Task_Vofa_Send(void * Parameters){
  TickType_t LastWakeTime = xTaskGetTickCount();
  ProtocolInfo_Type *protocolInfo = Parameters;
  Node_Type * node = protocolInfo->node;
  uint32_t deviceID = node->deviceID;
  while (1)
  {
    /* code */
    DMA_Disable(USARTx_Tx);
    taskENTER_CRITICAL();
    ProtocolData.debugInfo.vofaData = VofaShadow;
    taskEXIT_CRITICAL();
    DMA_Enable(USARTx_Tx, protocolInfo->length);
    vTaskDelayUntil(&LastWakeTime, 1000.0f/protocolInfo->frequency);
  }
  
}

void Vofa_Send(uint16_t frequency, uint16_t heapDeth, Node_Type * node, uint32_t commandID){
  ProtocolData.debugInfo.vofaData.vofaTail = vofaTail_t;
  VofaShadow.vofaTail = vofaTail_t;
  ProtocolInfo_Type *protocolInfo = Protocol_Get_Info_Handle(commandID);
  if (protocolInfo->taskHandle != 0) {
    vTaskDelete(protocolInfo->taskHandle);
  }
  protocolInfo->node = node;
  protocolInfo->frequency = frequency;
  uint32_t deviceID = node->deviceID;
  BSP_DMA_Init(USARTx_Tx, ProtocolData.debugInfo.data, Protocol_Buffer_Length);
  xTaskCreate(Task_Vofa_Send, "Vofa_Send", heapDeth, (void *)protocolInfo, 6, protocolInfo->taskHandle);
};