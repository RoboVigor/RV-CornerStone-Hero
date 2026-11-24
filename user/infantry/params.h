#ifndef __PARAMS_H
#define __PARAMS_H

#include <stdint.h>

/*******************************************************************************
 * 运行期可调参数 (pitch 控制器)
 * - 上电时 Params_Init() 先装入编译期默认值, 再尝试从 flash 读取已保存的参数 (校验通过才覆盖)
 * - 串口命令 get/set 读写 RAM 中的值, save 写入 flash, defaults 恢复编译期默认值 (不写 flash)
 * - 注意: flash 中存在有效参数时, 修改代码里的默认值不会生效; 先 defaults + save, 或改 PARAMS_VERSION
 *******************************************************************************/
typedef struct {
    float kAngle;  // 角度环 Kθ (dps/deg)
    float kpSpeed; // 速度环 P (电流单位/dps)
    float kiSpeed; // 速度环 I (电流单位/dps/每帧)
    float iMax;    // 积分限幅
    float fc;      // 库仑摩擦前馈 (实际使用值, 一般取实测 Fc 的 PITCH_FC_RATIO 倍)
    float fcEps;   // tanh 平滑宽度 (dps)
    float aBrake;  // 角度环开方制动曲线减速度 (dps²)
    float loadC;   // 负载前馈 L(θ) = loadC + loadD·θ
    float loadD;
} PitchParams_Type;

extern volatile PitchParams_Type PitchParams;
extern const PitchParams_Type    PitchParamsDefault;

typedef enum { PARAMS_SRC_DEFAULT = 0, PARAMS_SRC_FLASH = 1 } ParamsSource_e;

ParamsSource_e Params_Init(void);     // 在 main 中调度器启动前调用
void           Params_Set_Default(void);
int            Params_Save(void);     // 0 成功; 负数失败. 会擦除 flash 扇区 (约 1~2s, 调用者阻塞)

#endif
