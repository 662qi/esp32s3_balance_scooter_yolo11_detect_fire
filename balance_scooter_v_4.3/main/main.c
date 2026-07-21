#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "oled.h"
#include "gptim.h"
#include "key.h"
#include "mpu6050.h"
#include "myuart.h"
#include <math.h>
#include "pid.h"
#include "motor.h"
#include "encoder.h"

// MPU6050校正参数
#define MPU6050_AE_GY   -75       // 陀螺仪零漂误差校正
#define MPU6050_AE_ACC  -4.2      // 加速度计误差校正
#define MPU6050_FREQ    0.01      // 10 ms 获取一次(0.01 s)
// 控制周期定义
#define ANGLE_TIME      10        // 角度环控制周期(ms)
#define SPEED_TIME      40        // 速度环控制周期(ms)

// ===================== 全局交互变量(Core0读写,Core1只读/回写,volatile防编译器优化) =====================
uint8_t Key_Num;
volatile uint8_t RunFlag;

volatile int16_t AX, AY, AZ, GX, GY, GZ;
volatile float AngleAcc = 0, AngleGyro = 0, Angle = 0;
const float alpha = 0.01;

volatile int16_t LeftPWM = 0, RightPWM = 0;
volatile int16_t AvePWM = 0, DifPWM = 0;

volatile float LeftSpeed = 0, RightSpeed = 0;
volatile float AveSpeed = 0, DifSpeed = 0;

// 双核PID参数共享缓存(蓝牙滑块修改Core0原PID后同步到此,Core1读取)
volatile float g_angle_Kp, g_angle_Ki, g_angle_Kd;
volatile float g_speed_Kp, g_speed_Ki, g_speed_Kd;
volatile float g_turn_Kp, g_turn_Ki, g_turn_Kd;
volatile float g_angle_offset;

// 蓝牙摇杆下发目标值
volatile float g_speed_target = 0;
volatile float g_turn_target = 0;

// ===================== 原始PID结构体(完全保留你定义,OLED/蓝牙直接操作) =====================
PID_t angle_pid = {
    .Kp = 4.5,
    .Ki = 0,
    .Kd = 24.5,

    .out_max = 100,
    .out_min = -100,
    .int_max = 300,
    .int_min = -300,
    .out_offset = 0.5,
};

PID_t speed_pid = {
    .Kp = 2.21,
    .Ki = 0.27,
    .Kd = 0,

    .out_max = 10,
    .out_min = -10,
    .int_max = 10,
    .int_min = -10,
    .out_offset = 0,
};

PID_t turn_pid = {
    .Kp = 3,
    .Ki = 1.5,
    .Kd = 0,

    .out_max = 50,
    .out_min = -50,
    .int_max = 10,
    .int_min = -10,
    .out_offset = 0,
};

// ===================== Core1 实时平衡控制任务(高优先级,纯算法无UI阻塞) =====================
void BalanceCtrl_Task(void *arg)
{
    // 本地私有PID副本,避免双核同时读写结构体冲突
    PID_t loc_angle_pid = angle_pid;
    PID_t loc_speed_pid = speed_pid;
    PID_t loc_turn_pid = turn_pid;

    int16_t l_AX, l_AY, l_AZ, l_GX, l_GY, l_GZ;
    float l_AngleAcc = 0, l_AngleGyro = 0, l_Angle = 0;

    int16_t l_LeftPWM = 0, l_RightPWM = 0;
    int16_t l_AvePWM = 0, l_DifPWM = 0;
    float l_LeftSpeed = 0, l_RightSpeed = 0;
    float l_AveSpeed = 0, l_DifSpeed = 0;

    // PID初始化
    pid_init(&loc_angle_pid);
    pid_init(&loc_speed_pid);
    pid_init(&loc_turn_pid);

    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xCtrlPeriod = pdMS_TO_TICKS(ANGLE_TIME);
    uint8_t speed_cnt = 0;

    while(1)
    {
        // 严格固定10ms周期运行,不受其他任务干扰
        vTaskDelayUntil(&xLastWakeTime, xCtrlPeriod);

        // 同步Core0蓝牙下发更新的PID参数
        loc_angle_pid.Kp = g_angle_Kp;
        loc_angle_pid.Ki = g_angle_Ki;
        loc_angle_pid.Kd = g_angle_Kd;
        loc_angle_pid.out_offset = g_angle_offset;

        loc_speed_pid.Kp = g_speed_Kp;
        loc_speed_pid.Ki = g_speed_Ki;
        loc_speed_pid.Kd = g_speed_Kd;

        loc_turn_pid.Kp = g_turn_Kp;
        loc_turn_pid.Ki = g_turn_Ki;
        loc_turn_pid.Kd = g_turn_Kd;

        // 读取MPU6050原始数据
        mpu6050_get_data(&l_AX, &l_AY, &l_AZ, &l_GX, &l_GY, &l_GZ);
        l_GX += MPU6050_AE_GY;

        // 互补滤波倾角解算
        float gyro_y = -(float)(l_GY) / 32768.0f * 500.0f;
        l_AngleAcc = atan2((float)l_AX, (float)l_AZ) / 3.14159f * 180.0f;
        l_AngleAcc += MPU6050_AE_ACC;

        l_AngleGyro = l_Angle + gyro_y * MPU6050_FREQ;
        l_Angle = (1-alpha) * l_AngleGyro + alpha * l_AngleAcc;

        // 本地数据同步到全局变量,供Core0 OLED显示读取
        AX = l_AX; AY = l_AY; AZ = l_AZ;
        GX = l_GX; GY = l_GY; GZ = l_GZ;
        AngleAcc = l_AngleAcc;
        AngleGyro = l_AngleGyro;
        Angle = l_Angle;

        // 倾角过大保护,直接停机
        if(l_Angle > 45 || l_Angle < -45)
        {
            RunFlag = 0;
            motor_a_move(0);
            motor_b_move(0);
            continue;
        }

        // 未使能平衡,电机锁死
        if(RunFlag == 0)
        {
            motor_a_move(0);
            motor_b_move(0);
            continue;
        }

        // 每4个10ms周期(40ms)读取一次编码器,计算速度
        speed_cnt ++;
        if(speed_cnt >= (SPEED_TIME / ANGLE_TIME))
        {
            speed_cnt = 0;
            l_LeftSpeed = (float)encoder_get_pulse(1) * 6.8f / ENC_PPR / 0.04f;
            l_RightSpeed = (float)encoder_get_pulse(2) * 6.8f / ENC_PPR / 0.04f;

            l_AveSpeed = (l_LeftSpeed + l_RightSpeed) / 2.0;
            l_DifSpeed = l_LeftSpeed - l_RightSpeed;

            // 速度环PID计算
            loc_speed_pid.target = g_speed_target;
            loc_speed_pid.actual = l_AveSpeed;
            // 边沿触发单次清积分——只有摇杆从有速度变回0的瞬间清一次，静止时积分正常工作
            static uint8_t last_speed_has_target = 0; // 记录上一次是否有速度目标
            uint8_t now_has_target = fabs(loc_speed_pid.target) > 0.01f;
            // 下降沿：上次有目标，这次回零，只清一次积分
            if(last_speed_has_target && !now_has_target)
            {
                loc_speed_pid.errorint = 0;
            }
            last_speed_has_target = now_has_target;
            pid_update(&loc_speed_pid);

            // 新增：同步速度环运行值到全局结构体，供Core0 OLED显示
            speed_pid.target = loc_speed_pid.target;
            speed_pid.actual = loc_speed_pid.actual;
            speed_pid.out = loc_speed_pid.out;

            // 转向差速PID计算
            loc_turn_pid.target = g_turn_target;
            loc_turn_pid.actual = l_DifSpeed;
            // 摇杆回中，目标近似0，清空积分消除残留
            if(fabs(loc_turn_pid.target) < 0.01f)
            {
                loc_turn_pid.errorint = 0;
            }
            pid_update(&loc_turn_pid);
            l_DifPWM = loc_turn_pid.out;

            // 速度数据同步全局
            LeftSpeed = l_LeftSpeed;
            RightSpeed = l_RightSpeed;
            AveSpeed = l_AveSpeed;
            DifSpeed = l_DifSpeed;
            DifPWM = l_DifPWM;
        }

        // 串级外环:角度环PID
        loc_angle_pid.target = -loc_speed_pid.out;
        loc_angle_pid.actual = l_Angle;
        pid_update(&loc_angle_pid);
        l_AvePWM = loc_angle_pid.out;

        // 新增：同步角度环运行值到全局结构体，供Core0 OLED显示
        angle_pid.target = loc_angle_pid.target;
        angle_pid.actual = loc_angle_pid.actual;
        angle_pid.out = loc_angle_pid.out;

        // 左右轮PWM分配
        l_LeftPWM = l_AvePWM + l_DifPWM / 2;
        l_RightPWM = l_AvePWM - l_DifPWM / 2;

        // 电机输出
        motor_a_move(l_LeftPWM);
        motor_b_move(l_RightPWM);

        // PWM同步全局用于屏幕显示
        LeftPWM = l_LeftPWM;
        RightPWM = l_RightPWM;
        AvePWM = l_AvePWM;
    }
}

// ===================== Core0 主函数:外设初始化+按键+蓝牙+OLED显示(人机交互) =====================
void app_main(void)
{
    // 硬件初始化,顺序与原版保持不变
    oled_init();
    key_init();
    gptim_init();
    mpu6050_init();
    myuart_init();
    motor_init();
    encoder_init();

    // 初始化双核共享PID参数缓存
    g_angle_Kp = angle_pid.Kp;
    g_angle_Ki = angle_pid.Ki;
    g_angle_Kd = angle_pid.Kd;
    g_angle_offset = angle_pid.out_offset;

    g_speed_Kp = speed_pid.Kp;
    g_speed_Ki = speed_pid.Ki;
    g_speed_Kd = speed_pid.Kd;

    g_turn_Kp = turn_pid.Kp;
    g_turn_Ki = turn_pid.Ki;
    g_turn_Kd = turn_pid.Kd;

    // 创建平衡控制任务,绑定CPU1,优先级10(高于主线程,保证实时性)
    xTaskCreatePinnedToCore(
        BalanceCtrl_Task,
        "BalanceCtrl",
        4096,               // 任务栈大小,浮点滤波+PID占用较大
        NULL,
        10,
        NULL,
        APP_CPU_NUM
    );

    while(1)
    {
        Key_Num = key_get();
        // 实体按键启停平衡
        if(Key_Num == 1)
        {
            if(RunFlag == 0)
            {
                pid_init(&angle_pid);
                pid_init(&speed_pid);
                pid_init(&turn_pid);
                RunFlag = 1;
                encoder_get_pulse(1);
                encoder_get_pulse(2);
            }
            else
            {
                RunFlag = 0;
            }
        }

        // 蓝牙串口数据解析,协议完全保留原版
        if(myuart_GetRxFlag() == 1)
        {
            char *Tag = strtok(myuart_RxPacket, ",");

            if(strcmp(Tag, "key") == 0)
            {
                char *Name = strtok(NULL, ",");
                char *Action = strtok(NULL, ",");

                if(strcmp(Name, "1") == 0 && strcmp(Action, "up") == 0)
                {
                    myuart_printf("key,1,up\r\n");
                    if(RunFlag == 0)
                    {
                        pid_init(&angle_pid);
                        pid_init(&speed_pid);
                        pid_init(&turn_pid);
                        RunFlag = 1;
                        encoder_get_pulse(1);
                        encoder_get_pulse(2);
                    }
                    else
                    {
                        RunFlag = 0;
                    }
                }
            }
            else if(strcmp(Tag, "slider") == 0)
            {
                char *Name = strtok(NULL, ",");
                char *Value = strtok(NULL, ",");
                float val = atof(Value);

                if(strcmp(Name, "angle_kp") == 0)
                {
                    g_angle_Kp = val;
                    angle_pid.Kp = val;
                    myuart_printf("slider,angle_kp,%f\r\n",angle_pid.Kp);
                }
                else if(strcmp(Name, "angle_ki") == 0)
                {
                    g_angle_Ki = val;
                    angle_pid.Ki = val;
                    myuart_printf("slider,angle_ki,%f\r\n",angle_pid.Ki);
                }
                else if(strcmp(Name, "angle_kd") == 0)
                {
                    g_angle_Kd = val;
                    angle_pid.Kd = val;
                    myuart_printf("slider,angle_kd,%f\r\n",angle_pid.Kd);
                }
                else if(strcmp(Name, "speed_kp") == 0)
                {
                    g_speed_Kp = val;
                    speed_pid.Kp = val;
                    myuart_printf("slider,speed_kp,%f\r\n",speed_pid.Kp);
                }
                else if(strcmp(Name, "speed_ki") == 0)
                {
                    g_speed_Ki = val;
                    speed_pid.Ki = val;
                    myuart_printf("slider,speed_ki,%f\r\n",speed_pid.Ki);
                }
                else if(strcmp(Name, "speed_kd") == 0)
                {
                    g_speed_Kd = val;
                    speed_pid.Kd = val;
                    myuart_printf("slider,speed_kd,%f\r\n",speed_pid.Kd);
                }
                else if(strcmp(Name, "off") == 0)
                {
                    g_angle_offset = val;
                    angle_pid.out_offset = val;
                    myuart_printf("slider,angle_off,%f\r\n",angle_pid.out_offset);
                }
                else if(strcmp(Name, "turn_kp") == 0)
                {
                    g_turn_Kp = val;
                    turn_pid.Kp = val;
                    myuart_printf("slider,turn_kp,%f\r\n",turn_pid.Kp);
                }
                else if(strcmp(Name, "turn_ki") == 0)
                {
                    g_turn_Ki = val;
                    turn_pid.Ki = val;
                    myuart_printf("slider,turn_ki,%f\r\n",turn_pid.Ki);
                }
                else if(strcmp(Name, "turn_kd") == 0)
                {
                    g_turn_Kd = val;
                    turn_pid.Kd = val;
                    myuart_printf("slider,turn_kd,%f\r\n",turn_pid.Kd);
                }
            }
            else if(strcmp(Tag, "joystick") == 0)
            {
                int8_t LH = atoi(strtok(NULL, ","));	// 左摇杆横向值
                int8_t LV = atoi(strtok(NULL, ","));	// 左摇杆纵向值
                int8_t RH = atoi(strtok(NULL, ","));	// 右摇杆横向值
                int8_t RV = atoi(strtok(NULL, ","));	// 右摇杆纵向值

                g_speed_target = LV / 40.0f;
                g_turn_target = RH / 2.0f;

                myuart_printf("joystick,%d,%d,%d,%d\r\n", LH, LV, RH, RV);
            }
        }

        // OLED屏幕刷新逻辑完全保留原版
        oled_show_signed_num(0,0,GY,5,OLED_Size_F8x16);
        oled_show_float(56,0,AngleAcc,3,3,OLED_Size_F8x16);

        oled_show_float(0,2,angle_pid.target,3,1,OLED_Size_F6x8);
        oled_show_float(0,3,angle_pid.actual,3,1,OLED_Size_F6x8);
        oled_show_float(0,4,angle_pid.out,3,1,OLED_Size_F6x8);

        oled_show_float(56,2,speed_pid.target,3,1,OLED_Size_F6x8);
        oled_show_float(56,3,speed_pid.actual,3,1,OLED_Size_F6x8);
        oled_show_float(56,4,speed_pid.out,3,1,OLED_Size_F6x8);

        oled_show_num(0,6,RunFlag,1,OLED_Size_F8x16);

        vTaskDelay(1);
    }
}

// 定时器中断回调:仅按键扫描,移除原时分状态标志
bool Timer_Callback(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_ctx)
{
    key_tick();
    return true;
}
