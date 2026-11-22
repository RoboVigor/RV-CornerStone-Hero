#include "sys.h"

#if defined(__CC_ARM)
//THUMB指令不支持汇编内联
//采用如下方法实现执行汇编指令WFI  
__asm void WFI_SET(void)
{
	WFI;		  
}
//关闭所有中断(但是不包括fault和NMI中断)
__asm void INTX_DISABLE(void)
{
	CPSID   I
	BX      LR	  
}
//开启所有中断
__asm void INTX_ENABLE(void)
{
	CPSIE   I
	BX      LR  
}
//设置栈顶地址
//addr:栈顶地址
__asm void MSR_MSP(u32 addr) 
{
	MSR MSP, r0 			//set Main Stack value
	BX r14
}
#else
// GCC 构建: 与上面 ARMCC 嵌入汇编等价
void WFI_SET(void)      { __asm volatile("wfi"); }
void INTX_DISABLE(void) { __asm volatile("cpsid i" ::: "memory"); }
void INTX_ENABLE(void)  { __asm volatile("cpsie i" ::: "memory"); }
void MSR_MSP(u32 addr)  { __asm volatile("msr msp, %0" : : "r"(addr) : "memory"); }
#endif
















