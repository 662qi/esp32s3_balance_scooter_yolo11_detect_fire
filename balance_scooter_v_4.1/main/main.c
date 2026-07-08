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

#define MPU6050_AE_GY   -75       // 陀螺仪零漂误差校正
#define MPU6050_AE_ACC  -4.2    // 加速度计误差校正
#define MPU6050_FREQ    0.01    // 10 ms 获取一次(0.01 s)
#define ANGLE_TIME      10      // 运行角度计算与角度环控制(ms)：10 ms
#define SPEED_TIME      40      // 运行速度获取与速度环控制(ms)：40 ms

uint8_t Key_Num, RunFlag;

int16_t AX, AY, AZ, GX, GY, GZ;
float ax, ay, az, gx, gy, gz;

volatile uint8_t Angle_State;
volatile uint8_t Speed_State;

float AngleAcc = 0, AngleGyro = 0, Angle = 0;
float alpha = 0.01;

int16_t LeftPWM = 0, RightPWM = 0;
int16_t AvePWM = 0, DifPWM = 0;

float LeftSpeed = 0, RightSpeed = 0;
float AveSpeed = 0, DifSpeed = 0;

PID_t angle_pid = {
    .Kp = 21.66,
    .Ki = 0,
    .Kd = 11.78,

    .out_max = 100,
    .out_min = -100,
    .int_max = 300,
    .int_min = -300,
    .out_offset = 0.5,
};

PID_t speed_pid = {
    .Kp = 0.469,
    .Ki = 0.1323,
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

void app_main(void)
{
    oled_init();
    key_init();
    gptim_init();
    mpu6050_init();
    myuart_init();
    motor_init();
    encoder_init();

    while(1)
    {
        Key_Num = key_get();
        // 按键模块
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

        // 速度获取与速度环控制
        if(Speed_State == 1)
        {
            LeftSpeed = encoder_get_pulse(1) / 44.0 / 0.05 / 9.276;   // 44脉冲/转，0.05s获取一次，9.276为轮子周长(cm)
            RightSpeed = encoder_get_pulse(2) / 44.0 / 0.05 / 9.276;  // 44脉冲/转，0.05s获取一次，9.276为轮子周长(cm)

            AveSpeed = (LeftSpeed + RightSpeed) / 2.0;
            DifSpeed = LeftSpeed - RightSpeed;

            if(RunFlag)
            {
                speed_pid.actual = AveSpeed;
                pid_update(&speed_pid);
                angle_pid.target = speed_pid.out;

                turn_pid.actual = DifSpeed;
                pid_update(&turn_pid);
                DifPWM = turn_pid.out;
            }
            
            Speed_State = 0;
        }

        // 角度计算与角度环控制
        if(Angle_State == 1)
        {
            // mpu6050原始数据获取
            mpu6050_get_data(&AX, &AY, &AZ, &GX, &GY, &GZ);

            // 互补滤波
            GY += MPU6050_AE_GY;

            float gyro_y = -(float)(GY) / 32768.0f * 500.0f;
            AngleAcc = atan2((float)AX, (float)AZ) / 3.14159f * 180.0f;

            AngleAcc += MPU6050_AE_ACC;

            AngleGyro = Angle + gyro_y * MPU6050_FREQ;

            Angle = (1-alpha) * AngleGyro + alpha * AngleAcc;

            if(Angle > 45 || Angle < -45)
            {
                RunFlag = 0;
            }

            if(RunFlag)
            {
                // pid调控
                angle_pid.actual = -Angle;
                pid_update(&angle_pid);
                AvePWM = -angle_pid.out;

                // PWM计算
                LeftPWM = AvePWM + DifPWM / 2;
                RightPWM = AvePWM - DifPWM / 2;

                motor_a_move(LeftPWM);
                motor_b_move(RightPWM);
            }
            else
            {
                motor_a_move(0);
                motor_b_move(0);
            }
            
            Angle_State = 0;
        }

        // 蓝牙数据处理
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
				
				if(strcmp(Name, "angle_kp") == 0)
				{
					angle_pid.Kp = atof(Value);
				
					myuart_printf("slider,angle_kp,%f\r\n",angle_pid.Kp);
				}
				else if(strcmp(Name, "angle_ki") == 0)
				{
					angle_pid.Ki = atof(Value);

					myuart_printf("slider,angle_ki,%f\r\n",angle_pid.Ki);
				}
				else if(strcmp(Name, "angle_kd") == 0)
				{
					angle_pid.Kd = atof(Value);

					myuart_printf("slider,angle_kd,%f\r\n",angle_pid.Kd);
				}
				else if(strcmp(Name, "speed_kp") == 0)
				{
					speed_pid.Kp = atof(Value);
				
					myuart_printf("slider,speed_kp,%f\r\n",speed_pid.Kp);
				}
				else if(strcmp(Name, "speed_ki") == 0)
				{
					speed_pid.Ki = atof(Value);

					myuart_printf("slider,speed_ki,%f\r\n",speed_pid.Ki);
				}
				else if(strcmp(Name, "speed_kd") == 0)
				{
					speed_pid.Kd = atof(Value);

					myuart_printf("slider,speed_kd,%f\r\n",speed_pid.Kd);
				}
				else if(strcmp(Name, "off") == 0)
				{
					angle_pid.out_offset = atof(Value);

					myuart_printf("slider,angle_off,%f\r\n",angle_pid.out_offset);
				}
				else if(strcmp(Name, "turn_kp") == 0)
				{
					turn_pid.Kp = atof(Value);
				
					myuart_printf("slider,turn_kp,%f\r\n",turn_pid.Kp);
				}
				else if(strcmp(Name, "turn_ki") == 0)
				{
					turn_pid.Ki = atof(Value);

					myuart_printf("slider,turn_ki,%f\r\n",turn_pid.Ki);
				}
				else if(strcmp(Name, "turn_kd") == 0)
				{
					turn_pid.Kd = atof(Value);

					myuart_printf("slider,turn_kd,%f\r\n",turn_pid.Kd);
				}
			}
			else if(strcmp(Tag, "joystick") == 0)
			{
				int8_t LH = atoi(strtok(NULL, ","));	// 左摇杆横向值
				int8_t LV = atoi(strtok(NULL, ","));	// 左摇杆纵向值
				int8_t RH = atoi(strtok(NULL, ","));	// 右摇杆横向值
				int8_t RV = atoi(strtok(NULL, ","));	// 右摇杆纵向值

                speed_pid.target = LV / 40.0f;
                turn_pid.target = RH / 2.0f;

				myuart_printf("joystick,%d,%d,%d,%d\r\n", LH, LV, RH, RV);
			}
        }


        // myuart_printf("[plot,%f,%f]",Angle, angle_pid.out);
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

// gptim中断函数
bool Timer_Callback(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_ctx)
{
    static uint8_t Count;
    key_tick();
    Count++;
    if(Count % ANGLE_TIME == 0)
    {
        Angle_State = 1;
    }
    if(Count % SPEED_TIME == 0)
    {
        Speed_State = 1;
    }
    return true;
}
