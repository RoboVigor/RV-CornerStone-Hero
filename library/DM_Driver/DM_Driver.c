#include "DM_Driver.h"

int float_to_uint(float X_float, float X_min, float X_max, int bits) {
    float span   = X_max - X_min;
    float offset = X_min;
    return (int) ((X_float - offset) * ((float) ((1 << bits) - 1)) / span);
}

void DM_Motor_Init(DM_Motor_Type *motor, uint8_t mode, uint8_t id) {
    motor->mode = mode;
    motor->id    = id;

    DM_Motor_Command(motor, Motor_Enble);
}

void DM_Motor_PID_Init(DM_Motor_Type *motor, float kp, float kd) {
    motor->kp = kp;
    motor->kd = kd;
}

void DM_Motor_Input(DM_Motor_Type *motor, float p_des, float v_des, float torque) {
    motor->p_des  = p_des;
    motor->v_des  = v_des;
    motor->torque = torque;
}

void DM_Motor_Command(DM_Motor_Type *motor, uint8_t command) {
    uint8_t sendbuff[8];
    for (int i = 0; i < 7; i++)
        sendbuff[i] = 0xff;

    switch (command) {
    case Motor_Enble:
        sendbuff[7] = 0xfc;
        break;
    case Motor_Disable:
        sendbuff[7] = 0xfd;
        break;

    default:
        break;
    }

    Can_Send_Msg(CAN1, motor->id, sendbuff, 8);
}

void DM_Motor_Control(DM_Motor_Type *motor) {
    uint8_t sendbuff[8];

    //if (motor->mode == MODE_MIT) {
        uint16_t Position_Tmp, Velocity_Tmp, Torque_Tmp, KP_Tmp, KD_Tmp;

        Position_Tmp = float_to_uint(motor->p_des, -12.5, 12.5, 16);
        Velocity_Tmp = float_to_uint(motor->v_des, -12.5, -12.5, 12);
        Torque_Tmp   = float_to_uint(motor->torque, -100, 100, 12);
        KP_Tmp       = float_to_uint(motor->kp, 0, 500, 12);
        KD_Tmp       = float_to_uint(motor->kd, 0, 5, 12);

        sendbuff[0] = (uint8_t) (Position_Tmp >> 8);
        sendbuff[1] = (uint8_t) (Position_Tmp);
        sendbuff[2] = (uint8_t) (Velocity_Tmp >> 4);
        sendbuff[3] = (uint8_t) ((Velocity_Tmp & 0x0F) << 4) | (KP_Tmp >> 8);
        sendbuff[4] = (uint8_t) (KP_Tmp);
        sendbuff[5] = (uint8_t) (KD_Tmp >> 4);
        sendbuff[6] = (uint8_t) ((KD_Tmp & 0x0F) << 4) | (Torque_Tmp >> 8);
        sendbuff[7] = (uint8_t) (Torque_Tmp);
    //}

    Can_Send_Msg(CAN1, motor->id, sendbuff, 8);
}