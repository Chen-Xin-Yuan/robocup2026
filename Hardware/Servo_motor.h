#ifndef SERVO_MOTOR_H
#define SERVO_MOTOR_H
#include "main.h"
#include "tim.h"

void Servo_SetAngle(uint8_t ch, uint16_t angle);
void Servo_test(void);
#endif
