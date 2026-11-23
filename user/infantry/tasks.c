/* @brief 任务*/
#include "tasks.h"
#include "config.h"
#include "macro.h"
#include "handle.h"
#include "math.h"

int qian = 0, zuo = 0;

void Task_Control(void *Parameters) {
    xEventGroupWaitBits(InitEventGroup, INIT_EVENT_ALL, pdFALSE, pdTRUE, portMAX_DELAY);
    TickType_t LastWakeTime = xTaskGetTickCount();
    // LASER_ON;

    while (1) {
        ControlMode = remoteData.gearSwitch;
        if (ControlMode == 1) {
            // 遥控器模式
            PsAimEnabled = BUTTON_RIGHT_PRESSED;
            // FastmoveMode  = LEFT_SWITCH_TOP && RIGHT_SWITCH_TOP;
            // MagzineOpened = LEFT_SWITCH_MIDDLE && RIGHT_SWITCH_TOP;
            FrictEnabled = 1;
            StirEnabled  = TRIGGER_PRESSED;

            // unused
            // FastShootMode = StirEnabled;
            // PsShootEnabled = SWITCH_RIGHT;
            SwingMode  = (remoteData.buttonLeft.state == ON);
            SafetyMode = (remoteData.buttonPause.state == ON);

        } else if (ControlMode == 2) {
            // 键鼠模式
            PsShootEnabled = 0;
            StirEnabled    = mouseData.pressLeft;
            PsAimEnabled   = mouseData.pressRight;
            // 摩擦轮
            if (keyboardData.G && !keyboardData.Ctrl) {
                FrictEnabled = 1;
            } else if (keyboardData.G && keyboardData.Ctrl) {
                FrictEnabled = 0;
                Key_Disable(&keyboardData, KEY_G, 100);
            }
            FrictEnabled = 1;
            // 弹舱盖
            MagzineOpened = keyboardData.F;
            // 小陀螺
            if (keyboardData.C) {
                SwingMode = 1;
            }
            if (keyboardData.V) {
                SwingMode = 0;
            }
            // 高射速模式
            if (keyboardData.E) ShootMode = fastShoot;
            // 高速移动模式(关闭底盘功率上限，飞坡用)
            if (keyboardData.Shift) moveMode = fastMove;
        }

        // 调试视觉用
        // FrictEnabled   = (remoteData.switchLeft == 2) || (remoteData.switchLeft == 1) && (remoteData.switchRight != 2);
        // PsAimEnabled   = (remoteData.switchLeft == 1) && (remoteData.switchRight != 3);
        // PsShootEnabled = (remoteData.switchLeft == 1) && (remoteData.switchRight == 1);
        vTaskDelayUntil(&LastWakeTime, 5);
    }
    vTaskDelete(NULL);
}

void Task_Can_Send(void *Parameters) {
    xEventGroupWaitBits(InitEventGroup, INIT_EVENT_ALL, pdFALSE, pdTRUE, portMAX_DELAY);
    TickType_t LastWakeTime = xTaskGetTickCount(); // 时钟
    float      interval     = 0.01;                // 任务运行间隔 s
    int        intervalms   = interval * 1000;     // 任务运行间隔 ms
    while (1) {
        Bridge_Send_Motor(&BridgeData, SafetyMode);
        // DM_Motor_Control(&Motor_Stir);
        //  DM_Motor_Read_Param(&Motor_Stir, 0x0A);
        vTaskDelayUntil(&LastWakeTime, intervalms); // 发送频率
    }
    vTaskDelete(NULL);
}

/*******************************************************************************
 * Pitch 调参观测工具 (阶段1): DWT 精确计时 + 触发式录波
 * - 录波: pitch 目标跳变 > REC_TRIGGER_DEG 时触发, 以控制频率记录 REC_LEN 帧(含 REC_PRE 帧触发前数据),
 *         随后每 REC_DUMP_PERIOD_MS 输出一帧到 VOFA; debug6 = 相对触发时刻的微秒数 (VOFA 会重复发送同一帧, 按 debug6 去重)
 * - 实时模式 debug6 = REC_LIVE_MARK, 用于区分录波帧
 *******************************************************************************/
#define REC_LEN            1000 // 500Hz 下约 2s
#define REC_PRE            100
#define REC_CH             6
#define REC_TRIGGER_DEG    0.5f
#define REC_DUMP_PERIOD_MS 10
#define REC_LIVE_MARK      (-999999.0f)

static float    recBuf[REC_LEN][REC_CH];
static uint32_t recTime[REC_LEN]; // DWT 周期计数
static uint16_t recHead     = 0;  // 下一个写入位置 (环形)
static uint16_t recCount    = 0;  // 已写入帧数 (封顶 REC_LEN)
static uint16_t recPostLeft = 0;
static uint16_t recDumpIdx  = 0;
static uint8_t  recState    = 0; // 0: 预录 1: 触发后采集 2: 输出
static uint32_t recTrigTime = 0;
static uint32_t recDumpLast = 0;

// 最近 1s 窗口内的最大值
typedef struct {
    float    cur;
    float    out;
    uint32_t start;
} ProbeMax_Type;
static ProbeMax_Type ProbeImuUpdate, ProbeImuPeriod, ProbeGimbalCalc, ProbeImuRead, ProbeImuSolve, ProbeGimbalPeriod;
static ProbeMax_Type ProbeImuSeg[GYRO_SEG_NUM];

static void DWT_Init(void) {
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

static float DWT_Us(uint32_t cycles) {
    return (float) cycles / (SystemCoreClock / 1000000);
}

static void ProbeMax_Feed(ProbeMax_Type *p, float value) {
    uint32_t now = DWT->CYCCNT;
    if (value > p->cur) p->cur = value;
    if (now - p->start > SystemCoreClock) {
        p->out   = p->cur;
        p->cur   = 0;
        p->start = now;
    }
}

/**
 * @brief 每个控制周期调用一次
 * @return 1: 正在输出录波, 调用者不要写 VofaData
 */
static uint8_t Recorder_Update(const float sample[REC_CH], uint8_t trigger) {
    uint32_t now = DWT->CYCCNT;
    int      i;

    if (recState == 2) {
        if (now - recDumpLast >= REC_DUMP_PERIOD_MS * (SystemCoreClock / 1000)) {
            if (recDumpIdx >= recCount) {
                recState = 0;
                recHead  = 0;
                recCount = 0;
                return 0;
            }
            uint16_t k       = (recHead + REC_LEN - recCount + recDumpIdx) % REC_LEN;
            VofaData->debug0 = recBuf[k][0];
            VofaData->debug1 = recBuf[k][1];
            VofaData->debug2 = recBuf[k][2];
            VofaData->debug3 = recBuf[k][3];
            VofaData->debug4 = recBuf[k][4];
            VofaData->debug5 = recBuf[k][5];
            VofaData->debug6 =(float) (int32_t) (recTime[k] - recTrigTime) / (SystemCoreClock / 1000000);
            recDumpIdx++;
            recDumpLast = now;
        }
        return 1;
    }

    for (i = 0; i < REC_CH; i++) recBuf[recHead][i] = sample[i];
    recTime[recHead] = now;
    recHead          = (recHead + 1) % REC_LEN;
    if (recCount < REC_LEN) recCount++;

    if (recState == 0) {
        if (trigger) {
            // 触发前最多保留 REC_PRE 帧
            if (recCount > REC_PRE + 1) recCount = REC_PRE + 1;
            recTrigTime = now;
            recPostLeft = REC_LEN - recCount;
            recState    = 1;
        }
    } else if (recState == 1) {
        if (--recPostLeft == 0) {
            recState    = 2;
            recDumpIdx  = 0;
            recDumpLast = now - REC_DUMP_PERIOD_MS * (SystemCoreClock / 1000);
        }
    }
    return 0;
}

/*******************************************************************************
 * Pitch 电流环控制 (GM6020 已在调参助手切为电流环, 发 CAN2 0x1FE 第3槽, ±16384 <-> ±3A)
 *   ω_cmd = Kθ·(θ_ref-θ) + ω_ff
 *   u     = Kp·(ω_cmd-ω) + 积分(抗饱和/可分离) + L(θ) + Fc·tanh(ω_cmd/ε)
 *******************************************************************************/
#define PITCH_K_ANGLE    40.0f    // 角度环 (dps/deg)
#define PITCH_KP_SPEED   220.0f   // 速度环 P (电流单位/dps)
#define PITCH_KI_SPEED   3.0f    // 速度环 I (电流单位/dps/每帧)
#define PITCH_I_MAX      4500.0f // 积分限幅: 需覆盖静摩擦超出前馈的部分
#define PITCH_I_SEP_DEG  1000.0f // 积分分离阈值 (摩擦主导系统: 静止时只能靠积分破开静摩擦, 故不分离)
#define PITCH_I_LEAK_DPS 10.0f   // 运动中 (|ω| 超过该值) 积分按 PITCH_I_LEAK 衰减: 运动摩擦由前馈承担, 防止破开静摩擦后积分残留导致过冲
#define PITCH_I_LEAK     0.98f   // 每帧乘数, 500Hz 下 τ≈100ms
#define PITCH_OUT_LIM    16000.0f // 电流指令限幅 (16384 = 3A)
#define PITCH_FC         3700.0f // 库仑摩擦前馈: 实测 Fc=5340±250 (2026-11 机械调整后, 4dps 往返扫描), 取 70% 防止过补偿引起极限环
#define PITCH_FC_EPS     8.0f    // dps, tanh 平滑宽度
#define PITCH_A_BRAKE    3000.0f // 角度环开方制动曲线的减速度 (dps²): 大误差时限制 ω_cmd, 保证能及时刹住不超调 (实测可达 3200~5200)
#define PITCH_TARGET_MIN -30.0f
#define PITCH_TARGET_MAX 3.0f
#define PITCH_SAFE_MIN   -31.0f // 越界锁存清零 (下限 +6: 下机械限位在 +3.7, 否则靠在限位上时抬不起来)
#define PITCH_SAFE_MAX   6.0f
#define PITCH_TEMP_MAX   70     // 电机温度保护 (°C), 超过锁存清零

// 负载前馈 L(θ) = C + D·θ (电流单位): 2026-11 机械调整后实测 4dps 往返扫描 [-26°,-1°] 两方向平均, 残差 sd 249
// 云台未配平 (断电会落回 +3.7° 下限位), 负载随抬起角度线性增大; 改动机械/配重后需重新辨识 (PITCH_TEST_MODE 2)
#define PITCH_LOAD_C -815.0f
#define PITCH_LOAD_D 185.7f // 每度
static float PitchLoadFeedforward(float pitchDeg) {
    return PITCH_LOAD_C + PITCH_LOAD_D * pitchDeg;
}

/**
 * 测试模式 (编译期选择), 由 pitchInit 之后的计时驱动, 无需遥控器
 * 0 正常(遥控) 1 速度环整定: -10°附近 ±2° 小阶跃 2 负载辨识: 释放测试 + 4dps 往返扫描 3 验收阶跃序列
 */
#define PITCH_TEST_MODE 0
static float PitchTest_Target(float t, uint8_t *release) {
    *release = 0;
#if PITCH_TEST_MODE == 1
    static const float seq[] = {-10, -8, -10, -12};
    return seq[(uint32_t) (t / 1.5f) % 4];
#elif PITCH_TEST_MODE == 2
    // A: 在 -5/-12/-20 各保持 2s 后释放 0.5s (共 7.5s); B: 2 -> -28 -> 2 以 4dps 往返两次 (30s)
    static const float holds[] = {-5, -12, -20};
    float              cycle   = fmodf(t, 37.5f);
    if (cycle < 7.5f) {
        int   k  = (int) (cycle / 2.5f);
        float tk = cycle - k * 2.5f;
        *release = tk >= 2.0f;
        return holds[k];
    }
    float tb  = fmodf(cycle - 7.5f, 15.0f); // 一个往返 15s
    float pos = tb < 7.5f ? 2.0f - 4.0f * tb : -28.0f + 4.0f * (tb - 7.5f);
    return pos;
#elif PITCH_TEST_MODE == 3
    static const float seq[] = {3, -2, 3, -6, 3, -10, 3, -14, 3, -18, 3, 1, 3, 0, -1, 3,
                                -8, -7, -8, -5, -8, -3, -8, -9, -8, -11, -8, -13, -8};
    return seq[(uint32_t) (t / 2.0f) % (sizeof(seq) / sizeof(seq[0]))];
#else
    return 0;
#endif
}

void Task_Gimbal(void *Parameters) {
    xEventGroupWaitBits(InitEventGroup, INIT_EVENT_ALL, pdFALSE, pdTRUE, portMAX_DELAY);
    // 任务
    // 阶段2: 由 Task_Imu 每帧通知唤醒, 与 IMU 同步运行 (IMU_SAMPLE_RATE_HZ); pitch 每帧计算, yaw 每 YAW_DECIMATION 帧计算
#define YAW_DECIMATION 2
    float           interval     = 1.0f / IMU_SAMPLE_RATE_HZ; // pitch 控制周期 s
    static uint32_t lastTime     = 0;
    uint32_t        loopCount    = 0;
    uint32_t        lastLoopStart = DWT->CYCCNT;

    // 反馈值
    float yawAngle, yawSpeed, pitchAngle, pitchSpeed, chassisAngle, motorYawSpeed;

    // 目标值
    float pitchAngleTarget = 0; // 目标Pitch
    float yawAngleTarget   = 0; // 目标Yaw

    float  autoAimPorgress           = 0.0f;
    float  autoAimYawTarget          = 0.0f;
    float  pitchT                    = 0;
    float  yawAngleTargetControl     = 0; // 遥控器输入
    float  pitchAngleTargetControl   = 0; // 遥控器输入
    float  pitchAngleTargetFix       = 0; // 上坡补偿
    float  pitchAngleTargetFixStable = 0; // 上坡补偿
    float  yawAngleTargetPs          = 0; // 视觉辅助
    float  yaw_angle_diff_new        = 0; // 视觉辅助
    float  yaw_angle_diff_old        = 0; // 视觉辅助
    float  pitchAngleTargetPs        = 0; // 视觉辅助
    float  pitchLastAngleTargetPs    = 0; // 视觉辅助
    int8_t pitchInit                 = 0; // pitch启动初始校准

    // 输出量
    int16_t yawCurrent   = 0;
    int16_t pitchCurrent = 0;

    // Pitch轴斜坡参数
    float pitchRampProgress    = 0;
    float pitchRampStart       = Gyroscope_EulerData.pitch;
    float pitchAngleTargetRamp = 0;

    // 初始化云台PID
    PID_Init(&PID_Cloud_YawAngle, 8, 0, 0, 4000, 100);
    // D 为每次计算的差分增益: yaw 由 200Hz(5ms) 变为 250Hz(4ms), D 乘 5/4 保持等效微分增益不变
    PID_Init(&PID_Cloud_YawSpeed, 500, 0, 50 * 5.0f / 4.0f, 23000, 40);
    PID_Init(&PID_Cloud_PitchAngle, 16, 0, 0, 4000, 120);
    PID_Init(&PID_Cloud_PitchSpeed, 40, 0, 0, 12000, 0); // 调参期间输出限幅 23000->12000
    PID_Init(&PID_Cloud_MotorYawSpeed, 3, 1, 0, 23000, 0);

    float lastPitchAngleTarget = 0; // 录波触发判断

    while (1) {
        ulTaskNotifyTake(pdTRUE, 3); // 等 IMU 新数据; 3ms 超时兜底, IMU 异常时云台仍以约 333Hz 运行
        uint32_t loopStart = DWT->CYCCNT;
        ProbeMax_Feed(&ProbeGimbalPeriod, DWT_Us(loopStart - lastLoopStart));
        lastLoopStart  = loopStart;
        uint8_t yawTick = (loopCount++ % YAW_DECIMATION) == 0;
        // 重置目标
        yawAngleTargetControl   = 0;
        pitchAngleTargetControl = 0;

        // 设置反馈
        yawAngle      = Gyroscope_EulerData.yaw;                                                                             // 逆时针为正
        yawSpeed      = Gyroscope_EulerData.yawSpeed;                                                                        // 逆时针为正
        pitchAngle    = Gyroscope_EulerData.pitch;                                                                           //
        // 原: >20dps 时切到 Motor_Pitch.speed(rpm, 约为dps的1/6), 实际起到"高速时增益降为1/6"的限幅作用, 掩盖了速度环增益过高
        pitchSpeed    = Gyroscope_EulerData.pitchSpeed; // deg/s
        chassisAngle  = -Motor_Pitch.angle + pitchAngle; // 计算底盘的旋转角度，此处要求一点，电机角度和imu要保持一致
        motorYawSpeed = Motor_Yaw.speed * RPM2RPS;

        // 遥控器输入角度目标
        if (ControlMode == 1) {
            if (ABS(remoteData.rx) > 30) yawAngleTargetControl = -remoteData.rx / 660.0f * 360 * interval * 0.4;
            if (ABS(remoteData.ry) > 30) pitchAngleTargetControl = remoteData.ry / 660.0f * 360 * interval * 0.05;
        } else if (ControlMode == 2) {
            yawAngleTargetControl   = -mouseData.x * 0.5 * interval; // 0.005
            pitchAngleTargetControl = -mouseData.y * 0.2 * interval;
        }
        // 调参用 pitch 阶跃测试: 右拨杆 上=-18° 下=3° 中=摇杆控制; 测完把 PITCH_STEP_TEST 改为 0
#define PITCH_STEP_TEST 1
#if PITCH_STEP_TEST
        if (ControlMode == 1 && (RIGHT_SWITCH_TOP || RIGHT_SWITCH_BOTTOM)) {
            pitchAngleTargetControl = 0; // 阶跃目标不加速度前馈
            pitchAngleTarget        = RIGHT_SWITCH_TOP ? -18.0f : 3.0f;
        }
#endif
        // 测试模式: 覆盖 pitch 目标 (PITCH_TEST_MODE, 见 PitchTest_Target)
        uint8_t pitchRelease = 0;
#if PITCH_TEST_MODE
        {
            static uint32_t testTick = 0;
            if (pitchInit) {
                pitchAngleTargetControl = 0;
                pitchAngleTarget        = PitchTest_Target(testTick++ * interval, &pitchRelease);
            }
        }
#endif
        yawAngleTarget += yawAngleTargetControl;
        pitchAngleTarget += pitchAngleTargetControl;
        // VofaData->debug5 = pitchAngleTarget;

        // 视觉辅助
        yawAngleTargetPs   = ProtocolData.autoaimData.yaw_angle_diff;
        pitchAngleTargetPs = HostAutoaimData.pitch_angle_diff;
        if (PsAimEnabled) {
            if (ProtocolData.autoaimData.yaw_angle_diff != 0) {
                yawAngleTarget                          = yawAngle + ProtocolData.autoaimData.yaw_angle_diff;
                ProtocolData.autoaimData.yaw_angle_diff = 0;
                autoAimPorgress                         = 0;
            }
            // if (autoAimPorgress < 1.0f) {
            //     autoAimPorgress += 0.125f;
            //     yawAngleTarget = RAMP(yawAngle, autoAimYawTarget, autoAimPorgress);
            // }
            // if (yaw_angle_diff_new / yaw_angle_diff_old < 0)
            //     yawAngleTargetPs = 0.3 * yaw_angle_diff_new;
            // else
            //     yawAngleTargetPs = yaw_angle_diff_new + (yaw_angle_diff_new - yaw_angle_diff_old) / intervalms * 100;
            // yaw_angle_diff_old = yaw_angle_diff_new;
            // yawAngleTarget     = yawAngle + yawAngleTargetPs;
            // pitchAngleTarget   = pitchAngle + pitchAngleTargetPs;
        }
        // } else {
        //     yaw_angle_diff_new = HostAutoaimData.yaw_angle_diff;

        //     VofaData->debug4 = yaw_angle_diff_old;

        //     yawAngleTargetPs   = HostAutoaimData.yaw_angle_diff;
        //     yaw_angle_diff_old = HostAutoaimData.yaw_angle_diff;
        // }

        // 限制云台运动范围即斜坡补偿
        float pitchTargetUnclamped = pitchAngleTarget;
        MIAO(pitchAngleTarget, GIMBAL_PITCH_MIN + chassisAngle, GIMBAL_PITCH_MAX + chassisAngle);
        float pitchSpeedFeedforward = (pitchInit && pitchAngleTarget == pitchTargetUnclamped) ? pitchAngleTargetControl / interval : 0; // 被限位时不加前馈

        // 开机时pitch轴匀速抬起
        if (!pitchInit) {
            pitchAngleTarget = RAMP(pitchRampStart, 0, pitchRampProgress);
            if (pitchRampProgress < 1) {
                pitchRampProgress += interval / 1.0f; // 斜坡 1s (原 200Hz 下每帧 0.005)
            } else {
                Motor_Set_Angle_Bias(&Motor_Pitch, Motor_Pitch.angle);
                pitchInit = 1;
            }
        }

        // if(!Motor_Pitch.online){
        //     pitchRampProgress = 0;
        //     pitchRampStart = Gyroscope_EulerData.pitch;
        //     pitchInit = 0;
        // }

        if (!PsAimEnabled) {
            PID_Cloud_YawSpeed.p = 500;
        }

        // 计算PID
        if (yawTick) {
            PID_Calculate(&PID_Cloud_YawAngle, yawAngleTarget, Gyroscope_EulerData.yaw);
            PID_Calculate(&PID_Cloud_YawSpeed, PID_Cloud_YawAngle.output, yawSpeed);
            PID_Calculate(&PID_Cloud_MotorYawSpeed, -1 * ChassisData.realvw, motorYawSpeed);
        }
        // ---- pitch 电流环控制器 ----
        static float   pitchIntegral = 0;
        static uint8_t pitchTripped  = 0; // 安全锁存, 需复位恢复
        MIAO(pitchAngleTarget, PITCH_TARGET_MIN, PITCH_TARGET_MAX);
        if (pitchAngle < PITCH_SAFE_MIN || pitchAngle > PITCH_SAFE_MAX || Motor_Pitch.temperature > PITCH_TEMP_MAX) pitchTripped = 1;
        float pitchAngleErr = pitchAngleTarget - pitchAngle;
        // 角度环: 小误差线性 Kθ·e; 大误差按开方制动曲线 √(2·A·|e|) 限速 (交点 |e| = 2A/Kθ²)
        float pitchOmegaFb  = PITCH_K_ANGLE * pitchAngleErr;
        float pitchOmegaLim = sqrtf(2.0f * PITCH_A_BRAKE * ABS(pitchAngleErr));
        MIAO(pitchOmegaFb, -pitchOmegaLim, pitchOmegaLim);
        float pitchOmegaCmd = pitchOmegaFb + pitchSpeedFeedforward; // 目标速度前馈: 摇杆/自瞄
        float pitchSpeedErr = pitchOmegaCmd - pitchSpeed;
        float pitchFF       = PitchLoadFeedforward(pitchAngle) + PITCH_FC * tanhf(pitchOmegaCmd / PITCH_FC_EPS);
        float pitchU0       = PITCH_KP_SPEED * pitchSpeedErr + pitchFF;
        uint8_t pitchOutputOn = !pitchTripped && !SafetyMode && Motor_Pitch.online && !pitchRelease;
        if (pitchOutputOn) {
            float inc  = ABS(pitchAngleErr) < PITCH_I_SEP_DEG ? PITCH_KI_SPEED * pitchSpeedErr : 0;
            float uTry = pitchU0 + pitchIntegral + inc;
            if (!((uTry > PITCH_OUT_LIM && inc > 0) || (uTry < -PITCH_OUT_LIM && inc < 0))) pitchIntegral += inc; // 抗饱和
            if (ABS(pitchSpeed) > PITCH_I_LEAK_DPS) pitchIntegral *= PITCH_I_LEAK;
            MIAO(pitchIntegral, -PITCH_I_MAX, PITCH_I_MAX);
        }
        float pitchU = pitchU0 + pitchIntegral;
        MIAO(pitchU, -PITCH_OUT_LIM, PITCH_OUT_LIM);

        // 输出电流
        if (SwingMode) {
            yawCurrent = PID_Cloud_YawSpeed.output;
            // yawCurrent = (1 - pow(2.71828, -2.23*abs(PID_Cloud_YawAngle.error)))*PID_Cloud_YawSpeed.output + pow(2.71828,
            // -2.23*abs(PID_Cloud_YawAngle.error)) * PID_Cloud_MotorYawSpeed.output;   //动态权重融合
        } else {
            yawCurrent = PID_Cloud_YawSpeed.output;
        }
        pitchCurrent      = pitchOutputOn ? (int16_t) pitchU : 0;
        Motor_Yaw.input   = yawCurrent;
        Motor_Pitch.input = pitchCurrent;

        // pitch 直接发送 (CAN2 0x1FE 第3槽 = ID3/0x207, 电流环模式), 与 IMU 同步 500Hz
        // Motor_Pitch.inputEnabled 已关闭, Bridge_Send_Motor 不发 pitch; 挂起调度防止与 Task_Can_Send 抢邮箱
        int16_t pitchOut = pitchCurrent;
        {
            vTaskSuspendAll();
            Can_Send_NoWait(CAN2, 0x1FE, 0, 0, pitchOut, 0);
            xTaskResumeAll();
        }

        // 录波: 0 目标角 1 实际角 2 速度环目标(dps) 3 陀螺速度(dps) 4 速度环输出 5 电机转速(dps)
        float recSample[REC_CH] = {pitchAngleTarget, pitchAngle, pitchOmegaCmd, pitchSpeed, (float) pitchCurrent, pitchIntegral};
#define REC_ENABLED 0 // 921600 + 1kHz VOFA 下改为连续流式记录, 录波器暂停用
        uint8_t recTrigger = REC_ENABLED && pitchInit && ABS(pitchAngleTarget - lastPitchAngleTarget) > REC_TRIGGER_DEG;
        lastPitchAngleTarget = pitchAngleTarget;
        ProbeMax_Feed(&ProbeGimbalCalc, DWT_Us(DWT->CYCCNT - loopStart));
        if (!Recorder_Update(recSample, recTrigger)) {
            // 实时: 0 目标角 1 实际角 2 误差 3 IMU读取+解算(us) 4 云台循环周期(us) 5 IMU周期(us); 3~5为最近1s最大值
            // (分段耗时在 ProbeImuSeg[], 需要时可换到这里看)
#define VOFA_VIEW_GYRO_SCALE_CALIB 0 // 1: 陀螺刻度系数标定视图 (换板后重新标定用)
            taskENTER_CRITICAL(); // 整帧写入, 与 Task_Vofa_Send 的拷贝互斥
#if VOFA_VIEW_GYRO_SCALE_CALIB
            // 标定: 0 EKF pitch 1 陀螺Y积分(deg) 2 加计倾角(deg) 3 陀螺Y(dps) 4 roll 5 IMU周期(us)
            VofaData->debug0 = pitchAngle;
            VofaData->debug1 = GyroCalibIntegralY;
            VofaData->debug2 = GyroCalibAccTilt;
            VofaData->debug3 = GyroCalibRateY;
            VofaData->debug4 = Gyroscope_EulerData.roll;
            VofaData->debug5 = ProbeImuPeriod.out;
#else
            // 流式(500Hz, 每帧): 0 目标角 1 实际角 2 陀螺速度(dps) 3 电流指令 4 积分项 5 电机温度(°C, 锁存时 +1000)
            VofaData->debug0 = pitchAngleTarget;
            VofaData->debug1 = pitchAngle;
            VofaData->debug2 = pitchSpeed;
            VofaData->debug3 = pitchOut;
            VofaData->debug4 = pitchIntegral;
            VofaData->debug5 = Motor_Pitch.temperature + (pitchTripped ? 1000 : 0);
#endif
            VofaData->debug6 = (float) loopCount; // 帧号: 去重/检测丢帧 (录波输出时为相对触发时刻的 us)
            taskEXIT_CRITICAL();
        }

        // VofaData->debug0 = yawAngleTargetPs;
        // VofaData->debug1 = yawAngle;
        // VofaData->debug2 = PID_Cloud_PitchSpeed.output;
        // VofaData->debug3 = Node_Host.receiveSeq;
        // VofaData->debug5 = yawAngleTarget;
        // VofaData->debug6 = Node_Host.receiveSeq;

        // 任务间隔
        uint32_t time    = getSysTimeMs();
        // VofaData->debug6 = time - lastTime;
        lastTime         = time;
    }
    vTaskDelete(NULL);
}

void Task_Chassis(void *Parameters) {
    xEventGroupWaitBits(InitEventGroup, INIT_EVENT_ALL, pdFALSE, pdTRUE, portMAX_DELAY);
    // 任务
    TickType_t LastWakeTime = xTaskGetTickCount(); // 时钟
    float      interval     = 0.005;               // 任务运行间隔 s
    int        intervalms   = interval * 1000;     // 任务运行间隔 ms

    // 底盘运动
    float   vx             = 0;
    float   vy             = 0;
    float   vw             = 0;
    float   vwRamp         = 0;
    float   vwRampProgress = 0;
    float   targetPower    = 0;
    float   motorCurrentOutput[4];
    int16_t MCO_With_PowerLimit[4];

    // 反馈值
    float motorAngle, motorSpeed;
    float lastMotorAngle    = Motor_Yaw.angle;
    float filter[6]         = {0, 0, 0, 0, 0, 0};
    int   filterp           = 0;
    float power             = 0;
    float powerBuffer       = 0;
    float realMotorSpeed[4] = {0, 0, 0, 0};

    // 小陀螺
    float   swingSpeed       = 0;
    uint8_t swingModeEnabled = 0;

    // 底盘跟随PID
    float followDeadRegion = 3.0;
    PID_Init(&PID_Follow_Angle, 3, 0.1, 0, 100, 50);
    PID_Init(&PID_Follow_Speed, 1, 0, 1, 100, 0);

    // 速度环PID
    PID_Init(&PID_LFCM, 1, 0, 0, 6000, 0);
    PID_Init(&PID_LBCM, 1, 0, 0, 6000, 0);
    PID_Init(&PID_RBCM, 1, 0, 0, 6000, 0);
    PID_Init(&PID_RFCM, 1, 0, 0, 6000, 0);

    // 扭矩前馈PID
    PID_Init(&PID_Fx, 4, 0, 0.5, 0, 0);
    PID_Init(&PID_Fy, 4, 0, 0.5, 0, 0);
    PID_Init(&PID_T, 5, 0, 0.5, 0, 0);

    // 闭环功率PID
    PID_Init(&PID_Power, 0.5, 0, 0, 40, 0);

    // 初始化底盘
    Chassis_Init(&ChassisData);

    // 底盘运动斜坡函数
    float xRampProgress = 0;
    float xRampStart    = 0;
    float xTargetRamp   = 0;
    float yRampProgress = 0;
    float yRampStart    = 0;
    float yTargetRamp   = 0;

    if (fricEnabled) moveMode = frictMove;

    while (1) {
        // 设置反馈值
        motorAngle = Motor_Yaw.angle;           // 电机角度
        motorSpeed = Motor_Yaw.speed * RPM2RPS; // 电机角速度
        // power             = ProtocolData.powerHeatData.chassis_power;         // 裁判系统功率
        powerBuffer       = ProtocolData.powerHeatData.chassis_power_buffer;  // 裁判系统功率缓冲
        targetPower       = ProtocolData.gameRobotstatus.chassis_power_limit; // 裁判系统功率限制
        realMotorSpeed[0] = Motor_LF.speed * RPM2RPS;
        realMotorSpeed[1] = Motor_LB.speed * RPM2RPS;
        realMotorSpeed[2] = Motor_RB.speed * RPM2RPS;
        realMotorSpeed[3] = Motor_RF.speed * RPM2RPS;

        // 视觉专属follow PID
        if (PsAimEnabled) {
            PID_Follow_Angle.p = 1.5;
        } else {
            PID_Follow_Angle.p = 1.3;
        }

        // 小陀螺模式
        switch (SwingMode) {
        case 1:
            swingSpeed       = 30 * RPM2RPS;
            swingModeEnabled = 1;
            break;

        default:
            swingModeEnabled = 0;
            break;
        }

        // 设置底盘总体移动速度
        vx = 0;
        vy = 0;
        vw = 0;
        if (ControlMode == 1) {
            vy = -remoteData.lx / 660.0f * 6.0;
            vx = remoteData.ly / 660.0f * 6.0;

        } else if (ControlMode == 2) {
            xTargetRamp = RAMP(xRampStart, 660, xRampProgress);
            if (xRampProgress <= 0.5) {
                xRampProgress += 0.1f;
            } else if (xRampProgress > 0.5 && xRampProgress < 1) {
                xRampProgress += 0.05f;
            }
            yTargetRamp = RAMP(yRampStart, 660, yRampProgress);
            if (yRampProgress <= 0.5) {
                yRampProgress += 0.1f;
            } else if (yRampProgress > 0.5 && yRampProgress < 1) {
                yRampProgress += 0.05f;
            }
            vy = (keyboardData.A - keyboardData.D) * xTargetRamp / 660.0f * 12;
            vx = (keyboardData.W - keyboardData.S) * yTargetRamp / 660.0f * 8;
        }

        if (keyboardData.W == 0 && keyboardData.S == 0) {
            yRampProgress = 0;
            yRampStart    = 0;
        }
        if (keyboardData.A == 0 && keyboardData.D == 0) {
            xRampProgress = 0;
            xRampStart    = 0;
        }

        // 运动学正解算底盘真实速度
        Chassis_Calculate_Real_Speed(&ChassisData, realMotorSpeed);

        // 地盘跟随和小陀螺
        if (!swingModeEnabled) {
            // 底盘跟随云台
            vw += Gyroscope_EulerData.yawSpeed * DPS2RPS * 0.75; // 前馈
            int fbAngle1 = (((int) motorAngle % 360) > 0 ? (((int) motorAngle % 360) - 360) : (((int) motorAngle % 360) + 360));
            int fbAngle2 = ((int) motorAngle % 360);
            int fbAngle  = abs(((int) motorAngle % 360) / 180) ? fbAngle1 : fbAngle2;
            PID_Calculate(&PID_Follow_Angle, 0, -fbAngle);
            if (abs((int) PID_Follow_Angle.error) > followDeadRegion) {
                PID_Calculate(&PID_Follow_Speed,
                              PID_Follow_Angle.output,
                              ChassisData.realvw); // 此处本质是为了让已有速度前馈下还存在的error进行一个速度小补偿，此处的速度环pid仅作scalar的作用并无反馈
                // if(Motor_Yaw.online) {
                vw += PID_Follow_Speed.output * DPS2RPS;
                // }else {
                // vw = 0;
                // }
            }
        } else {
            // 小陀螺
            vw = swingSpeed;
        }

        // 开机时底盘匀速回正
        vwRamp = RAMP(0, vw, vwRampProgress);
        if (vwRampProgress < 1) {
            vwRampProgress += 0.002f;
        }

        switch (moveMode) {
        case frictMove:
            PID_Calculate(&PID_Power, 40, ProtocolData.powerHeatData.chassis_power_buffer);
            targetPower = ProtocolData.gameRobotstatus.chassis_power_limit - PID_Power.output;
            break;

        case fastMove:
            targetPower = 360;
            break;

        default:
            targetPower = 80;
            break;
        }

        Chassis_Update(&ChassisData, vx, vy, vwRamp); // 更新麦轮转速
        Chassis_Fix(&ChassisData, motorAngle);        // 修正旋转后底盘的前进方向
        Chassis_Calculate_Rotor_Speed(&ChassisData);  // 麦轮解算

        PID_Calculate(&PID_Fx, ChassisData.vx, ChassisData.realvx);
        PID_Calculate(&PID_Fy, ChassisData.vy, ChassisData.realvy);
        PID_Calculate(&PID_T, vwRamp, ChassisData.realvw);
        Chassis_Updata_FT(&ChassisData, PID_Fx.output, PID_Fy.output, PID_T.output);
        Chassis_Calculate_Rotor_Torgue(&ChassisData);

        // 计算输出电流PID
        PID_Calculate(&PID_LFCM, ChassisData.rotorSpeed[0], Motor_LF.speed * RPM2RPS);
        PID_Calculate(&PID_LBCM, ChassisData.rotorSpeed[1], Motor_LB.speed * RPM2RPS);
        PID_Calculate(&PID_RBCM, ChassisData.rotorSpeed[2], Motor_RB.speed * RPM2RPS);
        PID_Calculate(&PID_RFCM, ChassisData.rotorSpeed[3], Motor_RF.speed * RPM2RPS);
        motorCurrentOutput[0] = PID_LFCM.output * CurrentMap_C620;
        motorCurrentOutput[1] = PID_LBCM.output * CurrentMap_C620;
        motorCurrentOutput[2] = PID_RBCM.output * CurrentMap_C620;
        motorCurrentOutput[3] = PID_RFCM.output * CurrentMap_C620;

        Chassis_Current_Output_Integrate(motorCurrentOutput, &ChassisData);
        Chassis_Calculate_Power_Limit(motorCurrentOutput, MCO_With_PowerLimit, realMotorSpeed, targetPower);

        // 输出电流值到电调 功率限制已修改完成
        Motor_LF.input = MCO_With_PowerLimit[0];
        Motor_LB.input = MCO_With_PowerLimit[1];
        Motor_RB.input = MCO_With_PowerLimit[2];
        Motor_RF.input = MCO_With_PowerLimit[3];

        // 调试信息
        // 底盘运动更新频率
        vTaskDelayUntil(&LastWakeTime, intervalms);
    }

    vTaskDelete(NULL);
}

void Task_Host(void *Parameters) {
    xEventGroupWaitBits(InitEventGroup, INIT_EVENT_ALL, pdFALSE, pdTRUE, portMAX_DELAY);
    TickType_t         LastWakeTime = xTaskGetTickCount();
    ProtocolInfo_Type *protocolInfo;
    int16_t            lastReceiveSeq = 0;
    int64_t            sinceReceive;
    float              interval   = 0.1;             // 任务运行间隔 s
    int                intervalms = interval * 1000; // 任务运行间隔 ms

    uint16_t targetPower   = 0;
    uint16_t refereePower  = 0;
    uint8_t  sendBuffer[8] = {0, 0, 0, 0, 0x12, 0x20, 0x12, 0x07};
    while (1) {

        // SuperCap
        if (fricEnabled) {
            targetPower = ProtocolData.gameRobotstatus.chassis_power_limit - 10; // 留点余量；
        } else {
            targetPower = 50;
        }
        refereePower  = ProtocolData.gameRobotstatus.chassis_power_limit;
        sendBuffer[0] = targetPower & 0xff;
        sendBuffer[1] = targetPower >> 8;
        sendBuffer[2] = refereePower & 0xff;
        sendBuffer[3] = refereePower >> 8;
        Can_Send_Msg(CAN1, 0x4ff, sendBuffer, 8);

        // transmit
        ProtocolData.gyroscopeData.pitch = Gyroscope_EulerData.pitch;
        ProtocolData.gyroscopeData.yaw   = Gyroscope_EulerData.yaw;
        ProtocolData.gyroscopeData.roll  = Gyroscope_EulerData.roll;
        memcpy(ProtocolData.dbusData.dbusBuffer, remoteBuffer, 19);

        // receive autoaim data
        protocolInfo = Protocol_Get_Info_Handle(0x401);
        if (protocolInfo->lastReceiveSeq != protocolInfo->receiveSeq) {
            memcpy(HostAutoaimData.data, ProtocolData.autoaimData.data, protocolInfo->length);
            protocolInfo->lastReceiveSeq = protocolInfo->receiveSeq;
        } else {
            memset(HostAutoaimData.data, 0, protocolInfo->length);
        }
        FacingEnemyMode = HostAutoaimData.yaw_angle_diff != 0 || HostAutoaimData.pitch_angle_diff != 0;
        // DebugData.debug5 = protocolInfo->receiveSeq;

        // // receive chassis data
        // protocolInfo = Protocol_Get_Info_Handle(0x402);
        // sinceReceive = xTaskGetTickCount() - protocolInfo->receiveTime;
        // if (sinceReceive > 0 && sinceReceive < 200) {
        //     memcpy(HostChassisData.data, ProtocolData.chassisData.data, protocolInfo->length);
        // } else {
        //     memset(HostChassisData.data, 0, protocolInfo->length);
        // }

        // debug
        // DebugData.debug1 = HostAutoaimData.yaw_angle_diff * 1000;
        // DebugData.debug2 = ProtocolData.autoaimData.yaw_angle_diff * 1000;
        // DebugData.debug3 = HostAutoaimData.pitch_angle_diff*1000;
        // DebugData.debug4 = ProtocolData.autoaimData.pitch_angle_diff*1000;

        vTaskDelayUntil(&LastWakeTime, intervalms);
    }
    vTaskDelete(NULL);
}

void Task_UI(void *Parameters) {
    TickType_t LastWakeTime  = xTaskGetTickCount();
    uint8_t    isInitialized = 0;
    while (1) {
    }
    vTaskDelete(NULL);
}

/**
 * @brief 发射机构 (拨弹轮)
 */

void Task_Fire_Stir(void *Parameters) {
    xEventGroupWaitBits(InitEventGroup, INIT_EVENT_ALL, pdFALSE, pdTRUE, portMAX_DELAY);
    TickType_t LastWakeTime = xTaskGetTickCount(); // 时钟
    float      interval     = 0.05;                // 任务运行间隔 s
    int        intervalms   = interval * 1000;
    float      targetSpeed  = 0; // 任务运行间隔 ms

    // 射击模式
    enum shootMode_e { shootIdle = 0, shootToDeath }; // 停止, 连发
    enum shootMode_e shootMode = shootIdle;

    // 热量控制
    int   shootNum        = 0;
    int   mayShootNum     = 0;
    int   maxBulletSpeed  = 0;
    float lastBulletSpeed = 0;
    float maxShootHeat    = 0;
    float stirSpeed       = 0;
    int   stirAngle       = 0;

    // 视觉系统
    int16_t lastSeq = 0;

    // PID 初始化
    DM_Motor_PID_Init(&Motor_Stir, 0, 0.20f);

    // TEST
    //	PID_Init(&PID_FireL, 3, 0, 0, 16384, 1000);
    //	PID_Init(&PID_FireR, 3, 0, 0, 16384, 1000);

    // 开启激光
    // LASER_ON;

    while (1) {
        // 弹舱盖开关
        if (ROBOT_MIAO) {
            PWM_Set_Compare(&PWM_Magazine_Servo, MagzineOpened ? 10 : 5);
        } else if (ROBOT_WANG) {
            PWM_Set_Compare(&PWM_Magazine_Servo, MagzineOpened ? 16 : 6);
        } else if (ROBOT_SHARK) {
            PWM_Set_Compare(&PWM_Magazine_Servo, MagzineOpened ? 6 : 15);
        }
        // 拨弹速度
        stirSpeed = 110;
        if (ProtocolData.gameRobotstatus.shooter_barrel_cooling_value == 20) {
            stirSpeed = 110;
        } else if (ProtocolData.gameRobotstatus.shooter_barrel_cooling_value == 30) {
            stirSpeed = 140;
        } else if (ProtocolData.gameRobotstatus.shooter_barrel_cooling_value == 40) {
            stirSpeed = 160;
        }
        stirSpeed * 3;

        // stirSpeed = 143; // 热量：120
        // stirSpeed = 120; // 热量：240
        // stirSpeed = 120; // 热量：360

        // X模式

        // 热量控制
        maxShootHeat = ProtocolData.gameRobotstatus.shooter_barrel_heat_limit - ProtocolData.powerHeatData.shooter_17mm_barrel_heat * 2;

        // 输入射击模式
        shootMode = shootIdle;

        if (StirEnabled) {
            shootMode = shootToDeath;
        }
        // 视觉辅助
        // if (PsShootEnabled && lastSeq != Ps.autoaimData.seq && Ps.autoaimData.biu_biu_state) {
        //     shootMode = shootToDeath;
        // }
        // lastSeq = Ps.autoaimData.seq;

        if (ProtocolData.powerHeatData.shooter_17mm_barrel_heat > maxShootHeat) {
            shootMode = shootIdle;
        }

        // 控制拨弹轮
        if (shootMode == shootIdle) {
            // 停止
            stirSpeed = 0;

        } else if (shootMode == shootToDeath) {
            // 连发
            stirSpeed = 3.0f;
            //            if (mouseData.pressRight) {
            //                stirSpeed = 400;
            //            }
        }

        DM_Motor_Input(&Motor_Stir, 0, stirSpeed, 0.5f);

        // DebugData.debug1 = PID_StirSpeed.output;
        // DebugData.debug2 = shootMode;

        vTaskDelayUntil(&LastWakeTime, intervalms);
    }

    vTaskDelete(NULL);
}

void Task_Fire_Frict(void *Parameters) {
    xEventGroupWaitBits(InitEventGroup, INIT_EVENT_ALL, pdFALSE, pdTRUE, portMAX_DELAY);
    TickType_t LastWakeTime = xTaskGetTickCount(); // 时钟
    float      interval     = 0.05;                // 任务运行间隔 s
    int        intervalms   = interval * 1000;     // 任务运行间隔 ms

    float motorLSpeed;
    float motorRSpeed;
    float targetSpeed = 0;

    // 射击模式
    enum shootMode_e { shootIdle = 0, shootToDeath }; // 停止, 连发
    enum shootMode_e shootMode = shootIdle;

    PID_Init(&PID_FireL, 3, 0, 0, 16384, 1200);
    PID_Init(&PID_FireR, 3, 0, 0, 16384, 1200);
    PID_Init(&PID_FireT, 3, 0, 0, 16384, 1200);

    while (1) {

        if (FrictEnabled) {
            LASER_ON;
        } else {
            LASER_OFF;
        }

        //		if (StirEnabled) {
        //            shootMode = shootToDeath;
        //        }
        //		if (shootMode == shootIdle) {
        //            // 停止
        //						targetSpeed = 0;
        //						PID_Calculate(&PID_FireL, targetSpeed, Motor_FL.speed);
        //						PID_Calculate(&PID_FireR, -1*targetSpeed, Motor_FR.speed);
        //						Motor_FL.input = PID_FireL.output;
        //						Motor_FR.input = PID_FireR.output;

        //        } else if (shootMode == shootToDeath) {
        //            // 连发
        //			PID_Calculate(&PID_FireL, targetSpeed, Motor_FL.speed);
        //			PID_Calculate(&PID_FireR, -1*targetSpeed, Motor_FR.speed);
        //			Motor_FL.input = PID_FireL.output;
        //			Motor_FR.input = PID_FireR.output;
        //        }
        targetSpeed = -4400;
        PID_Calculate(&PID_FireL, targetSpeed, Motor_FL.speed);
        PID_Calculate(&PID_FireR, targetSpeed, Motor_FR.speed);
        PID_Calculate(&PID_FireT, targetSpeed, Motor_FT.speed);
        Motor_FL.input = PID_FireL.output;
        Motor_FR.input = PID_FireR.output;
        Motor_FT.input = PID_FireT.output;

        /*else {
    targetSpeed = 4450;
              PID_Calculate(&PID_FireL, targetSpeed, Motor_FL.speed);
    PID_Calculate(&PID_FireR, -1*targetSpeed, Motor_FR.speed);

}
if (ROBOT_SHARK) {
    PID_Calculate(&PID_FireL, targetSpeed, Motor_FL.speed);
    PID_Calculate(&PID_FireR, -1*targetSpeed, Motor_FR.speed);
} else {
    PID_Calculate(&PID_FireL, -1*targetSpeed, Motor_FL.speed);
    PID_Calculate(&PID_FireR, -1 * targetSpeed, Motor_FR.speed);
}*/

        // DebugData.debug1 = Motor_FL.speed;
        // DebugData.debug2 = targetSpeed;
        // DebugData.debug3 = PID_FireR.output;

        vTaskDelayUntil(&LastWakeTime, intervalms);
    }
    vTaskDelete(NULL);
}

void Task_Blink(void *Parameters) {
    TickType_t LastWakeTime = xTaskGetTickCount();
    while (1) {
        LED_Run_Horse_XP(); // XP开机动画,建议延时200ms
        // LED_Run_Horse(); // 跑马灯,建议延时20ms
        vTaskDelayUntil(&LastWakeTime, 200);
    }

    vTaskDelete(NULL);
}

void Task_Startup_Music(void *Parameters) {
    TickType_t LastWakeTime = xTaskGetTickCount();
    while (1) {
        if (KTV_Play(Music_Earth)) break;
        vTaskDelayUntil(&LastWakeTime, 120);
    }
    vTaskDelete(NULL);
}

void Task_Wait(void *Parameters) {
    TickType_t LastWakeTime = xTaskGetTickCount();

    while (1) // 老6躲到基地后开陀螺代码，2023上理RoboVigor哨兵核心科技
    {
        //		if (ProtocolData.gameStatus.game_progress == 3)//裁判系统信号控制
        //		//if (LEFT_SWITCH_BOTTOM && RIGHT_SWITCH_BOTTOM)//遥控器控制（双朝下开始）
        //		{
        //			delay_ms(6000);//delay3000毫秒，等裁判系统（！！！最后改成6000！！！）
        //			//SwingMode = 6;//测试裁判系统信号用的陀螺开启代码，常为被注释状态
        //			while(1){qian=27; zuo=0; vTaskDelay(4000); break;}//向前速度（可为负，建议绝对值20-40）、向左速度、持续时间（毫秒）
        //			qian=0; zuo=0;//向前向左速度初始化
        //			while(1){qian=0; zuo=-25; vTaskDelay(2700); break;}//向前速度（可为负，建议绝对值20-40）、向左速度、持续时间（毫秒）
        //			qian=0; zuo=0;//向前、向左速度初始化
        //			while(1){qian=-20; zuo=0; vTaskDelay(600); break;}//向前速度（可为负，建议绝对值20-40）、向左速度、持续时间（毫秒）
        //			qian=0; zuo=0;//向前向左速度初始化
        //			while(1){qian=0; zuo=0; vTaskDelay(200); break;}//向前速度（可为负，建议绝对值20-40）、向左速度、持续时间（毫秒）
        //			qian=0; zuo=0;//向前向左速度初始化
        //				while(1){
        //				SwingMode = 3;//小陀螺6，大陀螺3
        //				vTaskDelay(2300);
        //				SwingMode = 0;//小陀螺6，大陀螺3
        //				vTaskDelay(500);
        //				SwingMode = 5;//小陀螺6，大陀螺3
        //				vTaskDelay(2300);
        //				if (LEFT_SWITCH_TOP && RIGHT_SWITCH_TOP){SwingMode = 0;break;}//遥控器控制（双朝上停止陀螺）
        //				}
        //		}
        if (LEFT_SWITCH_TOP && RIGHT_SWITCH_TOP) {
            SwingMode = 0;
        } // 遥控器控制（双朝上停止陀螺）

        vTaskDelayUntil(&LastWakeTime, 120);
    }
    vTaskDelete(NULL);
}

void Task_IWDG(void *Parameters) {
    TickType_t LastWakeTime = xTaskGetTickCount(); // 时钟
    float      interval     = 0.1;                 // 任务运行间隔 s
    int        intervalms   = interval * 1000;     // 任务运行间隔 ms

    while (1) {
        // IWDG
        BSP_IWDG_Feed();
        vTaskDelayUntil(&LastWakeTime, intervalms);
        /* code */
    }
    vTaskDelete(NULL);
}

void Task_Imu(void *Parameters) {
    ImuDataReady = xSemaphoreCreateBinary();
    configASSERT(ImuDataReady != NULL);
    BSP_IMU_Init();
    Gyroscope_Init(&Gyroscope_EulerData, 0);
    uint16_t counter = 1000;
    DWT_Init();
    uint32_t lastImuStart = DWT->CYCCNT;
    while (1) {
        if (xSemaphoreTake(ImuDataReady, portMAX_DELAY) == pdTRUE) {
            uint32_t imuStart = DWT->CYCCNT;
            Gyroscope_Update(&Gyroscope_EulerData);
            ProbeMax_Feed(&ProbeImuUpdate, DWT_Us(DWT->CYCCNT - imuStart));
            ProbeMax_Feed(&ProbeImuRead, DWT_Us(GyroReadCycles));
            ProbeMax_Feed(&ProbeImuSolve, DWT_Us(GyroSolveCycles));
            if (TaskHandle_Gimbal) xTaskNotifyGive(TaskHandle_Gimbal);
            for (int seg = 0; seg < GYRO_SEG_NUM; seg++) ProbeMax_Feed(&ProbeImuSeg[seg], DWT_Us(GyroSegCycles[seg]));
            ProbeMax_Feed(&ProbeImuPeriod, DWT_Us(imuStart - lastImuStart));
            lastImuStart = imuStart;
            if (!counter) {
                continue;
            }
        }

        // 至少等待1000帧, 且陀螺零偏校准完成后才放行控制任务
        if (counter > 1) {
            counter--;
        } else if (Gyroscope_Is_Calibrated()) {
            counter = 0;
            xEventGroupSetBits(InitEventGroup, INIT_EVENT_IMU);
        }
    }
}
