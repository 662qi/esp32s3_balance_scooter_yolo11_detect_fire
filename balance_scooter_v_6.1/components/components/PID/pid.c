#include "pid.h"

/**
  *@brief	pid初始化
  *@param   *p      PID_t的结构体地址
  *@retval	无
  */
void pid_init(PID_t *p)
{
    p->target = 0;
    p->actual = 0;
    p->actual1 = 0;
    p->out = 0;

    p->error0 = 0;
    p->error1 = 0;
    p->errorint = 0;
}

/**
  *@brief	pid计算
  *@param   *p      PID_t的结构体地址
  *@retval	无
  *@note    需要外部传入actual(实际值)；外部传给被控对象out(输出值)
  */
void pid_update(PID_t *p)
{
    p->error1 = p->error0;                  // 上一次的误差
    p->error0 = p->target - p->actual;      // 此次的误差(目标值 - 实际值)

    /* --- 误差积分 --- */
    if(p->Ki != 0)
    {
        if( p->out < p->out_max && p->out > p->out_min )
        {
            p->errorint += p->error0;
        }

        if(p->errorint > p->int_max) {p->errorint = p->int_max;}
        if(p->errorint < p->int_min) {p->errorint = p->int_min;}
    }
    else
    {
        p->errorint = 0;
    }

    /* --- pid公式计算(微分先行) --- */
    p->out = p->Kp * p->error0
           + p->Ki * p->errorint
        //    + p->Kd * (p->error0 - p->error1);
           - p->Kd * (p->actual - p->actual1);

    /* --- pid输出偏移--- */
    if(p->out > 0){p->out += p->out_offset;}
    if(p->out < 0){p->out -= p->out_offset;}

    /* --- pid输出限幅 --- */
    if(p->out > p->out_max){p->out = p->out_max;}
    if(p->out < p->out_min){p->out = p->out_min;}

    p->actual1 = p->actual;         // 存储实际值
}
