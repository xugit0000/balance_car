#include "control.h"
#include "mpu6050.h"
#include "inv_mpu.h"
#include "encoder.h"
#include "tim.h"
#include "motor.h"
#include "sr04.h"


/* 平衡时角度值偏移量（机械中值） */
float g_med_angle = -2.0;       //-2.0
/* 直立环PD */
float vertical_Kp = -120.0;     //直立环调完后乘0.6作为最终值,-200*0.6=-120
float vertical_Kd = -1.2;       //-2.0*0.6=-1.2
/* 速度环PI */
float velocity_Kp = 0.014;      //速度环Kp/Ki，200倍关系
float velocity_Ki = 0.00007;
/* 转向环PD */
float turn_Kp = 0.0;
float turn_Kd = -0.3;
/* 倒下标志 */
uint8_t g_stop_flag = 0;
/* 编码器读取值 */
int g_encoder_left;   //左编码器数据（速度）
int g_encoder_right;  //右编码器数据（速度）
/* 串级pid控制中间变量 */
int g_velocity_out;         //速度环输出
int g_vertical_out;         //直立环输出
int g_turn_out;             //转向环输出
int g_motor1_pwm;           //左电机pwm装载值
int g_motor2_pwm;           //右电机pwm装载值
int g_target_speed = 0;     //期望速度
int g_target_turn = 0;      //转向速度


/* 外部中断回调函数 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if(GPIO_Pin == ECHO_Pin)                  //超声波接收引脚，测距
    {
        sr04_callback();
    }
    else if(GPIO_Pin == MPU_INT_Pin)          //陀螺仪中断引脚，周期10ms，执行pid控制
    {
        car_control();
    }
}


/* 小车控制函数 */
void car_control(void)
{
    int pwm_out;
    
    //1、读取编码器和陀螺仪的数据
    g_encoder_left = -read_pulse(&htim2);            //左电机，电机是相对安装，为了编码器输出极性一致，就需要对其中一个取反。
    g_encoder_right = read_pulse(&htim4);            //右电机
    
    mpu_dmp_get_data(&g_pitch,&g_roll,&g_yaw);        //得到角度值
    MPU_Get_Gyroscope(&g_gyrox,&g_gyroy,&g_gyroz);    //得到角速度数据
    MPU_Get_Accelerometer(&g_aacx,&g_aacy,&g_aacz);   //得到加速度传感器数据
    
    //2、将数据压入闭环控制中，计算出控制输出量
    g_velocity_out = velocity_pid_control(g_target_speed,g_encoder_left,g_encoder_right);   //速度环，速度环输出作为角度环输入
    g_vertical_out = vertical_pid_control(g_velocity_out+g_med_angle,g_roll,g_gyrox);       //直立环，这里角度值和角速度值要根据陀螺仪安装位置交换xy轴
    g_turn_out = turn_pid_control(g_gyroz,g_target_turn);                                   //转向环
    pwm_out = g_vertical_out;
//    pwm_out = g_velocity_out;       //速度环极性调试用
    
    //3、把控制输出量加载到电机上，完成最终的的控制
    g_motor1_pwm = pwm_out - g_turn_out;              //差速转弯
    g_motor2_pwm = pwm_out + g_turn_out;
    limit_motor_pwm(&g_motor1_pwm,&g_motor2_pwm);     //限幅
    load_motor_pwm(g_motor1_pwm,g_motor2_pwm);        //装载
//    load_motor_pwm(7000,7000);                        //编码器测速测试用
    
    stop_detection(&g_med_angle,&g_roll);             //倒地安全检测
}




/**
 * @brief       直立环PD控制
 * @param       med     期望角度
 * @param       angle   真实角度
 * @param       gyro_y  y轴真实角速度
 * @retval      直立环输出
 */
int vertical_pid_control(float target_angle,float angle,float gyro_y)
{
    int vertical_out;
    vertical_out = vertical_Kp * (angle - target_angle) + vertical_Kd * gyro_y;
    return vertical_out;
}


/**
 * @brief       速度环PI控制
 * @param       target_speed   期望速度
 * @param       encoder_left   左编码器速度
 * @param       encoder_right  右编码器速度
 * @retval      速度环输出
 */
int velocity_pid_control(int target_speed,int encoder_left,int encoder_right)
{
    static int err_lowout_last,encoder_integral;
    static float a = 0.7;
    int err,err_lowout,velocity_out;
    
    //1、计算速度偏差
    err = (encoder_left + encoder_right) - target_speed;
    
    //2、对速度偏差进行低通滤波
    //low_out = (1 - a) * Ek + a * low_out_last;
    err_lowout = (1 - a) * err + a * err_lowout_last;//使得波形更加平滑，滤除高频干扰，防止速度突变
    err_lowout_last = err_lowout;
    
    //3、对速度偏差积分，积分出位移
    encoder_integral += err_lowout;
    
    //4、积分限幅(-10000~10000)
    encoder_integral = encoder_integral>10000 ? 10000 : (encoder_integral<(-10000) ? (-10000) : encoder_integral);
    if(g_stop_flag == 1)encoder_integral = 0,g_stop_flag = 0;//清零积分量，扶起来正常直立,要不然倒下会暴走
    
    //5、速度环控制输出计算
    velocity_out = velocity_Kp * err_lowout + velocity_Ki * encoder_integral;
    return velocity_out;
}


/**
 * @brief       转向环PD控制，这不是一个严格的PD控制器，Kd针对的是转向的约束，但Kp针对的是遥控的转向。
 * @param       gyro_z      z轴角速度
 * @param       target_turn 转向速度
 * @retval      转向环输出
 */
int turn_pid_control(float gyro_z,int target_turn)
{
    int turn_out;
    turn_out = turn_Kp * target_turn + turn_Kd * gyro_z;
    return turn_out;
}

/**
 * @brief       倒地检测
 * @param       med_angle  机械中值
 * @param       angle      当前角度
 * @retval      无
 */
void stop_detection(float *med_angle,float *angle)
{
    if(find_abs((int)(*angle-*med_angle))>60)
    {
        load_motor_pwm(0,0);
        g_stop_flag = 1;
    }
}
