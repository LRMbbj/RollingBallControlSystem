#include "pid.h"
#include "host.h"

#include <math.h>

struct PIDCircle pidX, pidY;
struct PIDCircle* pids[2] = { &pidX, &pidY };

struct DataPack pack[4];

#define USER_PID_DEBUG

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
    pidX.kp = -( pidY.kp = -6); //-8
    pidX.ki = -( pidY.ki =-0.1 ); // -0.1
    pidX.kd = -(pidY.kd = -25); // -30
    pidX.tgt = pidY.tgt = 0;
    pidX.i_band = pidY.i_band = 20;
    pidX.i_limit = pidY.i_limit = 200;
    pidX.sum_limit = pidY.sum_limit = 400;
    
    pidX.cur = &posX;
    PIDReset(&pidX);
    
    
   
    // pid y环
    pidY.cur = &posY;
    PIDReset(&pidY);
    
    pids[0] = &pidX;
    pids[1] = &pidY;
    
    pack[0].type = DATA_TYPE_S16;
    pack[1].type = DATA_TYPE_S16;
    pack[2].type = DATA_TYPE_S16;
    pack[3].type = DATA_TYPE_S16;
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
    
    pid->d_out = pid->kd * (*(pid->cur) - pid->last_cur); //计算d
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
    if (state == PID_STATE_DEACTIVE) PIDReset(pid);
}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim == &htim6)
    {
        for (u8 i = 0; i < PID_NUM; i++)
        {
            if (pids[i]->isOn) 
            {
                PIDUpdate(pids[i]);
                (pids[i]->func)(pids[i]->obj, pids[i]->out);
            }
        }
    
        pack[0].val.i16 = posX;
        pack[1].val.i16 = posY;
        pack[2].val.i16 = pidX.out;
        pack[3].val.i16 = pidY.out;
    
        HostSendData(0xF1, pack, 4);
    }
}