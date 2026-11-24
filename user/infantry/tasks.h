#ifndef __TASK_SYSTEMINIT_H
#define __TASK_SYSTEMINIT_H

#include <stdint.h>

void Task_Sys_Init(void *Parameters);      // 硬件初始化和任务初始化
void Task_Blink(void *Parameters);         // 跑马灯
void Task_Startup_Music(void *Parameters); // 开机音乐
void Task_Chassis(void *Parameters);       // 底盘运动
void Task_Safe_Mode(void *Parameters);     // 安全模式
void Task_Gimbal(void *Parameters);        // 云台任务
void Task_Fire_Stir(void *Parameters);     // 发射机构 (拨弹轮)
void Task_Fire_Frict(void *Parameters);    // 发射机构 (摩擦轮)
void Task_Control(void *Parameters);
void Task_Can_Send(void *Parameters);
void Task_Host(void *Parameters);
void Task_Wait(void *Parameters);
void Task_IWDG(void *Parameters);
void Task_Imu(void *Parameters);

/*******************************************************************************
 * Pitch 前馈在线标定 (串口命令 cal pitch 触发, 在 Task_Gimbal 中执行)
 * 4dps 往返扫描, 匀速段最小二乘拟合 u = C + D·θ + Fc·sign(ω)
 *******************************************************************************/
typedef enum { PCAL_IDLE = 0, PCAL_PREP, PCAL_SWEEP, PCAL_DONE, PCAL_ABORTED } PitchCalState_e;
typedef enum { PCAL_ABORT_NONE = 0, PCAL_ABORT_STOP, PCAL_ABORT_OUTPUT_OFF, PCAL_ABORT_PREP_TIMEOUT, PCAL_ABORT_NOT_READY, PCAL_ABORT_SINGULAR } PitchCalAbort_e;
// Driver_Protocol.h 的 #pragma pack(1) 没有恢复, 会泄漏到其后包含的头文件; 这里固定对齐, 使布局与包含顺序无关
#pragma pack(push, 8)
typedef struct {
    volatile uint8_t         request;     // Task_Cmd 写: 1 开始 2 停止; Task_Gimbal 处理后清零
    volatile PitchCalState_e state;       // Task_Gimbal 写; DONE/ABORTED 由 Task_Cmd 输出后置回 IDLE
    volatile PitchCalAbort_e abortReason;
    volatile float           elapsed;     // 扫描已进行时间 (s)
    volatile float           duration;    // 扫描总时长 (s)
    // 结果 (state == PCAL_DONE 时有效)
    float    C, D, Fc, sd;
    uint32_t nPos, nNeg;
} PitchCal_Type;
#pragma pack(pop)
extern PitchCal_Type PitchCal;

#endif
