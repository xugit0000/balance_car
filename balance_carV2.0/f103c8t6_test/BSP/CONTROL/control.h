#ifndef __CONTROL_H
#define __CONTROL_H

#include "main.h"

extern float g_med_angle;    //机械中值
extern int g_encoder_left;   //左编码器数据（速度）
extern int g_encoder_right;  //右编码器数据（速度）


void car_control(void);
int vertical_pid_control(float target_angle,float angle,float gyro_y);
int velocity_pid_control(int target_speed,int encoder_left,int encoder_right);
int turn_pid_control(float gyro_z,int target_turn);
void stop_detection(float *med_angle,float *angle);

#endif
