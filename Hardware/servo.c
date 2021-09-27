#include "servo.h"

struct Servo servoX = { &htim3, TIM_CHANNEL_1, BIAS_90 + 90 }, servoY = { &htim3, TIM_CHANNEL_2, BIAS_90 + 121 };

void ServoSetState(struct Servo *servo, u8 state)
{
    if (state)
        HAL_TIM_PWM_Start(servo->htim, servo->Channel);
    else
    {
        ServoSetAngle(&servoX, 0);
        ServoSetAngle(&servoY, 0);
    
        HAL_Delay(1000);
        HAL_TIM_PWM_Stop(servo->htim, servo->Channel);
        
    }
}
void ServoSetAngle(struct Servo *servo, s16 val)
{
    u16 out = servo->bias + val;
    // 限幅
    if (out > BIAS_180) out = BIAS_180;
    if (out < BIAS_0) out = BIAS_0;
    
    __HAL_TIM_SET_COMPARE(servo->htim, servo->Channel, out);
}
