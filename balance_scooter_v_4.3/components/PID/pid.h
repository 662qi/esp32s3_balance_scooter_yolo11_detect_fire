#ifndef __PID_H__
#define __PID_H__

typedef struct {
    float target;       // 目标值
    float actual;       // 实际值
    float actual1;      // 上一次实际值(用于微分先行)
    float out;          // 输出值

    float Kp;
    float Ki;
    float Kd;

    float error0;
    float error1;
    float errorint;

    float out_max;
    float out_min;
    float int_max;
    float int_min;

    float out_offset;
}PID_t;

void pid_init(PID_t *p);
void pid_update(PID_t *p);

#endif
