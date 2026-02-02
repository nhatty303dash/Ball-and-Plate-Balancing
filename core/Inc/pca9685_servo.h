#ifndef __PCA9685_SERVO_H
#define __PCA9685_SERVO_H

#include "stm32f4xx_hal.h"
#include <stdbool.h>

#define PCA9685_ADDRESS 0x80

#define PCA9685_MODE1         0x0
#define PCA9685_PRE_SCALE     0xFE
#define PCA9685_LED0_ON_L     0x6
#define PCA9685_MODE1_SLEEP_BIT      4
#define PCA9685_MODE1_AI_BIT         5
#define PCA9685_MODE1_RESTART_BIT    7

// Servo channels - Changed to first 3 channels
#define SERVO1_CHANNEL 0
#define SERVO2_CHANNEL 1
#define SERVO3_CHANNEL 2

// Global variables to track current servo angles and PWM values
extern float current_servo_angles[3];
extern uint16_t current_servo_pwm[3];

void PCA9685_SetBit(uint8_t Register, uint8_t Bit, uint8_t Value);
void PCA9685_SetPWMFrequency(uint16_t frequency);
void PCA9685_Init(uint16_t frequency);
void PCA9685_SetPWM(uint8_t Channel, uint16_t OnTime, uint16_t OffTime);
// Set servo by pulse width in microseconds (e.g., 1500, 2000)
void PCA9685_SetServoPWMus(uint8_t Channel, uint16_t pulse_width_us);

// Các hàm điều khiển servo sử dụng bảng hiệu chuẩn riêng cho từng servo
uint16_t Servo1_AngleToTick(float angle_deg);
uint16_t Servo2_AngleToTick(float angle_deg);
uint16_t Servo3_AngleToTick(float angle_deg);
void SetServoAngles(float theta1, float theta2, float theta3); // theta1,2,3 là radian


#endif
