#ifndef DM_DRIVER_H
#define DM_DRIVER_H

#include "Driver_CAN.h"

enum DM_Motor_Command { Motor_Enble, Motor_Disable };
enum DM_Motor_Mode { MODE_MIT, MODE_POS_VEL, MODE_SPEED };

typedef struct {
    uint16_t id;
    uint8_t  mode;

    float kp;
    float kd;

    float p_des;
    float v_des;
    float torque;

} DM_Motor_Type;

int float_to_uint(float X_float, float X_min, float X_max, int bits);

void DM_Motor_Init(DM_Motor_Type *motor, uint8_t mode, uint8_t id);

void DM_Motor_Command(DM_Motor_Type *motor, uint8_t command);

void DM_Motor_PID_Init(DM_Motor_Type *motor, float kp, float kd);

void DM_Motor_Input(DM_Motor_Type *motor, float p_des, float v_des, float torque);

void DM_Motor_Control(DM_Motor_Type *motor);

#endif
