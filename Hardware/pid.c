#include "pid.h"

#include <math.h>

struct PIDCircle pidX, pidY;
struct PIDCircle* pids[2] = { &pidX, &pidY };

#ifdef USER_PID_DEBUG
#define user_pid_printf(format, ...) HostSendLog(LOG_COLOR_BLACK, format "\r\n", ##__VA_ARGS__)
#define user_pid_info(format, ...) HostSendLog(LOG_COLOR_BLACK, "[\tpid]info:" format "\r\n", ##__VA_ARGS__)
#define user_pid_debug(format, ...) HostSendLog(LOG_COLOR_GREEN, "[\tpid]debug:" format "\r\n", ##__VA_ARGS__)
#define user_pid_error(format, ...) HostSendLog(LOG_COLOR_RED, "[\tpid]error:" format "\r\n",##__VA_ARGS__)
#else
#define user_pid_printf(format, ...)
#define user_pid_info(format, ...)
#define user_pid_debug(format, ...)
#define user_pid_error(format, ...)
#endif

void PIDSetTarget(struct PIDCircle* pid, s16 target)
{
    
    pid->tgt = target;
    
}

s32 limit(s32 val, s32 tgt)
{
    
    if( abs(val) > tgt )
        return val > 0 ? tgt : -tgt;
    else
        return val;
    
}

//s32 Normalize(s32 val, s32 max)
//{
//    return (val + max * 5) % (max * 2) - max;
//}

void PIDInit()
{
    // pid x环
    pidX.kp = 0;
    pidX.ki = 0;
    pidX.kd = 0;
    pidX.tgt = 0;
    pidX.i_band = 0;
    pidX.i_limit = 0;
    pidX.sum_limit = 0;
    PIDReset(&pidX);
    
    
   
    // pid y环
    pidY.kp = 0;
    pidY.ki = 0;
    pidY.kd = 0;
    pidY.tgt = 0;
    pidY.i_band = 0;
    pidY.i_limit = 0;
    pidY.sum_limit = 0;
    PIDReset(&pidY);
    
    pids[0] = &pidX;
    pids[1] = &pidY;
}

void PIDReset(struct PIDCircle* pid)
{
    
    pid->e = 0;
    pid->last_cur = 0;
    pid->out = 0;
    pid->p_out = 0;
    pid->i_out = 0;
    pid->d_out = 0;
    
}

void PIDUpdate(struct PIDCircle* pid)
{
    
    pid->e = *(pid->cur) - pid->tgt;
    
    pid->d_out = pid->kd * *(pid->cur) - pid->last_cur; //计算d
    pid->last_cur = *(pid->cur);
    
    pid->p_out = pid->kp * pid->e;
    
    if( abs( pid->e ) > pid->i_band && pid->i_band > 0 )
        pid->i_out = 0;
    else
        pid->i_out += pid->ki * pid->e;
    
    pid->i_out = limit(pid->i_out, pid->i_limit );
    
    pid->out = pid->p_out + pid->i_out + pid->d_out;
    
    pid->out = limit(pid->out, pid->sum_limit);
    
    pid->func(pid ->obj, pid->out);
    
}


void PIDSetState(struct PIDCircle* pid, u8 state)
{
    pid->isOn = state;
}

void TIM6_IRQHandler()
{
    for (u8 i = 0; i < PID_NUM; i++)
    {
        if (pids[i]->isOn) PIDUpdate(pids[i]);
    }
}
