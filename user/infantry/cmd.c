/*******************************************************************************
 * 串口命令系统, 见 cmd.h
 *
 * 命令:
 *   help                 列出命令与参数
 *   get [name]           读取参数 (不带名字: 全部)
 *   set <name> <value>   修改参数 (立即生效, 仅 RAM; save 后才掉电保存)
 *   save                 参数写入 flash
 *   defaults             恢复编译期默认参数 (仅 RAM)
 *   cal pitch            pitch 前馈标定 (4dps 往返扫描约 32s); 成功后自动应用并保存
 *   cal stop             中止标定
 *   stream on|off        恢复/暂停 VOFA 二进制数据流
 *
 * 通道回复 (debug6 = 状态码):
 *   -1  成功        get <name>/set: debug0 = 参数值
 *                   get (全部): debug0..5 = loadC loadD fc kAngle kpSpeed kiSpeed
 *   -2  错误        未知命令 / 参数错误 / flash 写入失败 (debug0 = 错误码)
 *   -10 标定开始
 *   -12 标定完成    debug0 C, debug1 D, debug2 实测 Fc, debug3 应用的 fc, debug4 残差 sd, debug5 已保存(1)/拒绝(0)
 *   -13 标定中止    debug0 = 原因 (1 cal stop 2 输出关闭: 安全模式/电机掉线 3 到达起点超时 4 云台未完成上电初始化 5 样本不足)
 *******************************************************************************/
#include "cmd.h"
#include "handle.h"
#include "params.h"
#include "tasks.h"
#include <stdlib.h>
#include <string.h>

#define CMD_LINE_MAX     64
#define CMD_REPLY_HOLD   3000  // 通道回复保持时间 (ms)
#define CAL_REPLY_HOLD   10000 // 标定结果保持时间 (ms)
#define CAL_FC_RATIO     0.7f  // 应用的摩擦前馈 = 实测 Fc 的 70% (防止过补偿引起极限环)
#define CAL_MIN_SAMPLES  1000  // 每个方向至少的有效样本数
#define CAL_MAX_SD       600.0f // 残差 sd 上限 (正常约 250); 超过说明卡滞/爬行/线缆拉扯, 拟合不可信

#define CODE_OK          -1.0f
#define CODE_ERR         -2.0f
#define CODE_CAL_START   -10.0f
#define CODE_CAL_DONE    -12.0f
#define CODE_CAL_ABORT   -13.0f

/*******************************************************************************
 * 接收 (中断上下文): USART6 优先级 2 高于 configMAX_SYSCALL_INTERRUPT_PRIORITY, 不能调 FreeRTOS API,
 * 因此用单槽缓冲 + 标志位, Task_Cmd 轮询
 *******************************************************************************/
static char             rxLine[CMD_LINE_MAX];
static uint16_t         rxLen = 0;
static char             pendingLine[CMD_LINE_MAX];
static volatile uint8_t pendingReady = 0;

static void Cmd_Rx_End_Line(void) {
    if (rxLen > 0 && !pendingReady) { // 上一条未处理时丢弃新行
        memcpy(pendingLine, rxLine, rxLen);
        pendingLine[rxLen] = '\0';
        pendingReady       = 1;
    }
    rxLen = 0;
}

void Cmd_Rx_Bytes(const uint8_t *data, uint16_t len) {
    uint16_t i;
    for (i = 0; i < len; i++) {
        char c = (char) data[i];
        if (c == '\r' || c == '\n') {
            Cmd_Rx_End_Line();
        } else if (c >= 0x20 && c < 0x7F && rxLen < CMD_LINE_MAX - 1) {
            rxLine[rxLen++] = c;
        }
    }
    Cmd_Rx_End_Line(); // 一次突发结束 = 行尾
}

/*******************************************************************************
 * 参数表
 *******************************************************************************/
typedef struct {
    const char     *name;
    volatile float *value;
    float           min, max;
    const char     *desc;
} CmdParam_Type;

static const CmdParam_Type Params[] = {
    {"kangle", &PitchParams.kAngle, 0, 200, "pitch angle loop (dps/deg)"},
    {"kp", &PitchParams.kpSpeed, 0, 2000, "pitch speed loop P (cur/dps)"},
    {"ki", &PitchParams.kiSpeed, 0, 50, "pitch speed loop I (cur/dps/frame)"},
    {"imax", &PitchParams.iMax, 0, 16000, "pitch integral limit"},
    {"fc", &PitchParams.fc, 0, 16000, "pitch coulomb friction FF"},
    {"fceps", &PitchParams.fcEps, 0.1f, 100, "pitch friction tanh width (dps)"},
    {"abrake", &PitchParams.aBrake, 10, 20000, "pitch brake decel (dps^2)"},
    {"loadc", &PitchParams.loadC, -16000, 16000, "pitch load FF C"},
    {"loadd", &PitchParams.loadD, -2000, 2000, "pitch load FF D (per deg)"},
};
#define PARAM_NUM (sizeof(Params) / sizeof(Params[0]))

static const CmdParam_Type *Find_Param(const char *name) {
    uint32_t i;
    for (i = 0; i < PARAM_NUM; i++)
        if (strcmp(Params[i].name, name) == 0) return &Params[i];
    return 0;
}

/*******************************************************************************
 * 回复
 *******************************************************************************/
static void Reply_Value(float code, float v0, uint16_t holdMs) {
    float v[6] = {0};
    v[0]       = v0;
    Vofa_Reply(v, code, holdMs);
}

static void Reply_Error(const char *msg, float errCode) {
    Vofa_Print("ERR %s\r\n", msg);
    Reply_Value(CODE_ERR, errCode, CMD_REPLY_HOLD);
}

/*******************************************************************************
 * 命令实现
 *******************************************************************************/
static void Cmd_Help(int argc, char **argv) {
    uint32_t i;
    Vofa_Print("cmds: help | get [name] | set <name> <val> | save | defaults | cal pitch | cal stop | stream on|off\r\n");
    for (i = 0; i < PARAM_NUM; i++) Vofa_Print("  %-7s %s\r\n", Params[i].name, Params[i].desc);
    Reply_Value(CODE_OK, 0, CMD_REPLY_HOLD);
}

static void Cmd_Get(int argc, char **argv) {
    uint32_t i;
    if (argc >= 2) {
        const CmdParam_Type *p = Find_Param(argv[1]);
        if (!p) {
            Reply_Error("unknown param", 1);
            return;
        }
        Vofa_Print("%s = %.4f\r\n", p->name, *p->value);
        Reply_Value(CODE_OK, *p->value, CMD_REPLY_HOLD);
        return;
    }
    for (i = 0; i < PARAM_NUM; i++) Vofa_Print("%s = %.4f\r\n", Params[i].name, *Params[i].value);
    {
        float v[6] = {PitchParams.loadC, PitchParams.loadD, PitchParams.fc, PitchParams.kAngle, PitchParams.kpSpeed, PitchParams.kiSpeed};
        Vofa_Reply(v, CODE_OK, CMD_REPLY_HOLD);
    }
}

static void Cmd_Set(int argc, char **argv) {
    const CmdParam_Type *p;
    char                *end;
    float                v;
    if (argc < 3) {
        Reply_Error("usage: set <name> <value>", 2);
        return;
    }
    p = Find_Param(argv[1]);
    if (!p) {
        Reply_Error("unknown param", 1);
        return;
    }
    v = (float) strtod(argv[2], &end);
    if (end == argv[2] || *end != '\0') {
        Reply_Error("bad number", 3);
        return;
    }
    if (v < p->min || v > p->max) {
        Vofa_Print("ERR %s out of range [%.1f, %.1f]\r\n", p->name, p->min, p->max);
        Reply_Value(CODE_ERR, 4, CMD_REPLY_HOLD);
        return;
    }
    *p->value = v;
    Vofa_Print("%s = %.4f (not saved)\r\n", p->name, v);
    Reply_Value(CODE_OK, v, CMD_REPLY_HOLD);
}

static int Save_And_Report(void) {
    int ret;
    Vofa_Print("saving to flash...\r\n");
    ret = Params_Save();
    if (ret == 0)
        Vofa_Print("saved\r\n");
    else
        Vofa_Print("ERR flash save failed (%d)\r\n", ret);
    return ret;
}

static void Cmd_Save(int argc, char **argv) {
    if (PitchCal.state == PCAL_PREP || PitchCal.state == PCAL_SWEEP) {
        Reply_Error("calibration running", 5);
        return;
    }
    if (Save_And_Report() == 0)
        Reply_Value(CODE_OK, 0, CMD_REPLY_HOLD);
    else
        Reply_Value(CODE_ERR, 6, CMD_REPLY_HOLD);
}

static void Cmd_Defaults(int argc, char **argv) {
    Params_Set_Default();
    Vofa_Print("defaults restored (not saved)\r\n");
    Reply_Value(CODE_OK, 0, CMD_REPLY_HOLD);
}

static void Cmd_Cal(int argc, char **argv) {
    if (argc >= 2 && strcmp(argv[1], "pitch") == 0) {
        if (PitchCal.state == PCAL_PREP || PitchCal.state == PCAL_SWEEP) {
            Reply_Error("calibration already running", 5);
            return;
        }
        Vofa_Print("cal pitch: moving to start, then 4dps sweep (~%ds). 'cal stop' to abort\r\n", 32);
        PitchCal.request = 1;
        Reply_Value(CODE_CAL_START, 0, CMD_REPLY_HOLD);
    } else if (argc >= 2 && strcmp(argv[1], "stop") == 0) {
        PitchCal.request = 2;
        Vofa_Print("cal stop requested\r\n");
    } else {
        Reply_Error("usage: cal pitch | cal stop", 2);
    }
}

static void Cmd_Stream(int argc, char **argv) {
    if (argc >= 2 && strcmp(argv[1], "on") == 0) {
        Vofa_Stream_Enable(1);
    } else if (argc >= 2 && strcmp(argv[1], "off") == 0) {
        Vofa_Stream_Enable(0);
    } else {
        Reply_Error("usage: stream on|off", 2);
        return;
    }
    Vofa_Print("stream %s\r\n", argv[1]);
    Reply_Value(CODE_OK, 0, CMD_REPLY_HOLD);
}

typedef struct {
    const char *name;
    void (*handler)(int argc, char **argv);
} CmdEntry_Type;

static const CmdEntry_Type Commands[] = {
    {"help", Cmd_Help},
    {"get", Cmd_Get},
    {"set", Cmd_Set},
    {"save", Cmd_Save},
    {"defaults", Cmd_Defaults},
    {"cal", Cmd_Cal},
    {"stream", Cmd_Stream},
};

#define CMD_ARGC_MAX 4

static void Cmd_Execute(char *line) {
    char    *argv[CMD_ARGC_MAX];
    int      argc = 0;
    char    *p    = line;
    uint32_t i;

    Vofa_Print("> %s\r\n", line); // 回显 (切分前)
    // 按空白切分, 并转小写
    while (*p && argc < CMD_ARGC_MAX) {
        while (*p == ' ' || *p == '\t') p++;
        if (!*p) break;
        argv[argc++] = p;
        while (*p && *p != ' ' && *p != '\t') {
            if (*p >= 'A' && *p <= 'Z') *p += 'a' - 'A';
            p++;
        }
        if (*p) *p++ = '\0';
    }
    if (argc == 0) return;

    for (i = 0; i < sizeof(Commands) / sizeof(Commands[0]); i++) {
        if (strcmp(Commands[i].name, argv[0]) == 0) {
            Commands[i].handler(argc, argv);
            return;
        }
    }
    Reply_Error("unknown command, try 'help'", 0);
}

/*******************************************************************************
 * 标定结果处理: 合理性检查通过才应用并保存
 *******************************************************************************/
static void Cal_Report(void) {
    static const char *reasons[] = {"", "stopped", "output off (safety/offline)", "start not reached", "gimbal not initialized",
                                    "not enough samples"};
    if (PitchCal.state == PCAL_ABORTED) {
        uint8_t r = PitchCal.abortReason;
        Vofa_Print("cal pitch ABORTED: %s\r\n", r < sizeof(reasons) / sizeof(reasons[0]) ? reasons[r] : "?");
        Reply_Value(CODE_CAL_ABORT, r, CAL_REPLY_HOLD);
    } else if (PitchCal.state == PCAL_DONE) {
        float   fcApplied = CAL_FC_RATIO * PitchCal.Fc;
        uint8_t ok        = PitchCal.nPos >= CAL_MIN_SAMPLES && PitchCal.nNeg >= CAL_MIN_SAMPLES && PitchCal.Fc > 0 && fcApplied <= 16000 &&
                     PitchCal.C > -16000 && PitchCal.C < 16000 && PitchCal.D > -2000 && PitchCal.D < 2000 && PitchCal.sd <= CAL_MAX_SD;
        uint8_t saved = 0;
        Vofa_Print("cal pitch DONE: u = C + D*theta + Fc*sign(w)\r\n");
        Vofa_Print("  C  = %.1f\r\n  D  = %.2f /deg\r\n  Fc = %.1f (measured)\r\n", PitchCal.C, PitchCal.D, PitchCal.Fc);
        Vofa_Print("  residual sd = %.1f, samples +%lu / -%lu\r\n", PitchCal.sd, (unsigned long) PitchCal.nPos, (unsigned long) PitchCal.nNeg);
        if (ok) {
            PitchParams.loadC = PitchCal.C;
            PitchParams.loadD = PitchCal.D;
            PitchParams.fc    = fcApplied;
            Vofa_Print("applied: loadc %.1f loadd %.2f fc %.1f (= %.0f%% Fc)\r\n", PitchCal.C, PitchCal.D, fcApplied, CAL_FC_RATIO * 100);
            saved = Save_And_Report() == 0;
        } else {
            if (PitchCal.sd > CAL_MAX_SD) Vofa_Print("  residual sd above limit %.0f\r\n", CAL_MAX_SD);
            Vofa_Print("REJECTED: sanity check failed, params unchanged\r\n");
        }
        {
            float v[6] = {PitchCal.C, PitchCal.D, PitchCal.Fc, ok ? fcApplied : 0, PitchCal.sd, (float) saved};
            Vofa_Reply(v, CODE_CAL_DONE, CAL_REPLY_HOLD);
        }
    }
    PitchCal.state = PCAL_IDLE;
}

void Task_Cmd(void *Parameters) {
    char       line[CMD_LINE_MAX];
    TickType_t lastProgress = 0;

    while (1) {
        if (pendingReady) {
            memcpy(line, pendingLine, CMD_LINE_MAX);
            pendingReady = 0;
            Cmd_Execute(line);
        }
        if (PitchCal.state == PCAL_DONE || PitchCal.state == PCAL_ABORTED) {
            Cal_Report();
        } else if (PitchCal.state == PCAL_SWEEP && xTaskGetTickCount() - lastProgress >= 5000) {
            lastProgress = xTaskGetTickCount();
            Vofa_Print("cal pitch: %.0f/%.0fs, samples +%lu / -%lu\r\n", PitchCal.elapsed, PitchCal.duration, (unsigned long) PitchCal.nPos,
                       (unsigned long) PitchCal.nNeg);
        }
        vTaskDelay(10);
    }
}
