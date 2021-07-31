#ifndef __SERVO_H__
#define __SERVO_H__

#include "tim.h"

#define SERVO_STATE_ACTIVE 1
#define SERVO_STATE_DEACTIVE 0

#define u8 uint8_t
#define s16 int16_t
#define u16 uint16_t

#define BIAS_0 500
#define BIAS_90 1500
#define BIAS_180 2500

struct Servo
{
    TIM_HandleTypeDef *htim;
    uint32_t Channel;
    uint16_t bias;
};

extern struct Servo servoX, servoY;
void ServoSetState(struct Servo *servo, u8 state);
void ServoSetAngle(struct Servo *servo, s16 val);

#endif // !__SERVO_H__
