/**
 * @brief  中断服务函数根据地
 */

#include "handle.h"
// EXTI9_5 陀螺仪中断
void EXTI9_5_IRQHandler(void) {
    uint8_t    suc;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    if (EXTI_GetITStatus(EXTI_Line8) != RESET) {
        EXTI_ClearFlag(EXTI_Line8);
        EXTI_ClearITPendingBit(EXTI_Line8);
        xSemaphoreGiveFromISR(ImuDataReady, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

// DBus空闲中断(USART1)
void USART1_IRQHandler(void) {
    uint8_t UARTtemp;

    UARTtemp = USART1->DR;
    UARTtemp = USART1->SR;

    DMA_Cmd(DMA2_Stream2, DISABLE);

    // disabe DMA
    DMA_Disable(USART1_Rx);

    // 数据量正确
    if (DMA_Get_Stream(USART1_Rx)->NDTR == REMOTE_BACK_LENGTH) {
        Remote_Update(&remoteData, &keyboardData, &mouseData, remoteBuffer); // 解码
    }

    // enable DMA
    DMA_Enable(USART1_Rx, REMOTE_LENGTH + REMOTE_BACK_LENGTH);
} // DBus空闲中断(USART1)

void UART4_IRQHandler(void) {
    uint8_t UARTtemp;

    UARTtemp = UART4->DR;
    UARTtemp = UART4->SR;

    DMA_Cmd(DMA1_Stream2, DISABLE);

    // disabe DMA
    DMA_Disable(UART4_Rx);

    // 数据量正确
    if (DMA_Get_Stream(UART4_Rx)->NDTR == REMOTE_BACK_LENGTH) {
        Remote_Update(&remoteData, &keyboardData, &mouseData, remoteBuffer); // 解码
    }

    // enable DMA
    DMA_Enable(UART4_Rx, REMOTE_LENGTH + REMOTE_BACK_LENGTH);
}

/**
 * @brief USART3 串口中断
 */
void USART3_IRQHandler(void) {
    Bridge_Receive_USART(&BridgeData, USART_BRIDGE, 3);
}

/**
 * @brief USART6 串口中断
 * @note  裁判系统读取
 */
void USART6_IRQHandler(void) {
    Bridge_Receive_USART(&BridgeData, USART_BRIDGE, 6);
}

/**
 * @brief UART7 串口中断
 */
void UART7_IRQHandler(void) {
    Bridge_Receive_USART(&BridgeData, USART_BRIDGE, 7);
}

/**
 * @brief UART8 串口中断
 */
void UART8_IRQHandler(void) {
    Bridge_Receive_USART(&BridgeData, USART_BRIDGE, 8);
}

// CAN1数据接收中断服务函数
void CAN1_RX0_IRQHandler(void) {
    Bridge_Receive_CAN(&BridgeData, CAN1_BRIDGE);
}

// CAN2数据接收中断服务函数
void CAN2_RX0_IRQHandler(void) {
    Bridge_Receive_CAN(&BridgeData, CAN2_BRIDGE);
}

// TIM2 高频计数器
extern volatile uint32_t ulHighFrequencyTimerTicks;

void TIM2_IRQHandler(void) {
    if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET) {
        ulHighFrequencyTimerTicks++;
        TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
        TIM_ClearFlag(TIM2, TIM_FLAG_Update);
    }
}

/**
 * @brief  This function handles NMI exception.
 * @param  None
 * @return None
 */

void NMI_Handler(void) {
    while (1) {
    }
}

/**
 * @brief  This function handles Hard Fault exception.
 * @param  None
 * @return None
 */
void HardFault_Handler(void) {
    while (1) {
    }
}

/**
 * @brief  This function handles Memory Manage exception.
 * @param  None
 * @return None
 */
void MemManage_Handler(void) {
    while (1) {
    }
}

/**
 * @brief  This function handles Bus Fault exception.
 * @param  None
 * @return None
 */
void BusFault_Handler(void) {
    while (1) {
    }
}

/**
 * @brief  This function handles Usage Fault exception.
 * @param  None
 * @return None
 */
void UsageFault_Handler(void) {
    while (1) {
    }
}

/**
 * @brief  This function handles Debug Monitor exception.
 * @param  None
 * @return None
 */
void DebugMon_Handler(void) {
    while (1) {
    }
}

// /**
//  * @brief  This function handles SVCall exception.
//  * @param  None
//  * @return None
//  */
// void SVC_Handler(void) {
//     //while(1){}
// }

// /**
//  * @brief  This function handles PendSVC exception.
//  * @param  None
//  * @return None
//  */
// void PendSV_Handler(void) {
//    //while(1){}
// }

// /**
//  * @brief  This function handles SysTick Handler.
//  * @param  None
//  * @return None
//  */
// void SysTick_Handler(void) {
//      //while(1){}
// }
