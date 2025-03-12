#pragma once
#include <stdio.h>
#include <iostream>
#include "controlcan.h"
#include "math_ops.h"

extern int leg_device_r;
extern int leg_device_l;
extern int body_device;

//电机IDs

// 控制参数范围


#define PI 3.14159265359f
#define KP_MINX 0.0f
#define KP_MAXX 500.0f
#define KD_MINX 0.0f
#define KD_MAXX 5.0f
#define POS_MINX -12.5f
#define POS_MAXX 12.5f
#define SPD_MINX -18.0f
#define SPD_MAXX 18.0f

// 电机类型枚举
enum ActuatorType {
    LSG_20_90_7090 = 0,
    LSG_10_414,
    LSG_17_80_6070_new,
    LSG_14_70_5060,
    LSG_20_90_7080
};

// 电机参数结构体
struct ActuatorParams {
    int actuatorType;
    int defRatio;
    float defKT;
    float tMinX;  // 最小力矩
    float tMaxX;  // 最大力矩
    float iMinX;  // 最小电流
    float iMaxX;  // 最大电流
};



// 控制数据结构体
typedef struct {
    float pos_des_;
    float vel_des_;
    float ff_;
    float kp_;
    float kd_;
} YKSMotorData;

// 电机状态结构体
typedef struct {
    float angle_actual_rad;
    float speed_actual_rad;
    float current_actual_float;
} OD_Motor_Msg;

extern int leg_device_r;
extern int arml_device;
extern int body_device;

extern OD_Motor_Msg rv_motor_msg[12];  // n 个电机

// 函数声明
void send_motor_data_convert(VCI_CAN_OBJ *message, int motor_id, float kp, float kd, float pos, float spd, float tor);
void sendCommand(int s_id, int numOfActuator, YKSMotorData *mot_data, int canIndex);
void sendCanCommand(YKSMotorData *mot_data);
bool init_can();
void close_canDevice(int dIndex);
bool send_fix_body_command(int canIndex);