#include "servo_control.h"
#include "pca9685_servo.h"
#include "inverse_kinematics.h"
#include "stm32f4xx_hal.h"
#include <math.h>
#include <stdbool.h>
#include <stdint.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

#ifndef RAD_TO_DEG
#define RAD_TO_DEG(rad) ((rad) * 180.0f / M_PI)
#endif

#ifndef DEG_TO_RAD
#define DEG_TO_RAD(deg) ((deg) * M_PI / 180.0f)
#endif

// Servo angle limits (degrees) - based on calibration tables
#define SERVO1_MIN_ANGLE 0.0f
#define SERVO1_MAX_ANGLE 90.0f
#define SERVO2_MIN_ANGLE 0.0f
#define SERVO2_MAX_ANGLE 90.0f
#define SERVO3_MIN_ANGLE 0.0f
#define SERVO3_MAX_ANGLE 90.0f

// Global variable to track motion
ServoMotion current_motion = {0};

void ServoInit(void) {
    // Initialize PCA9685 with 50Hz frequency for servos
    PCA9685_Init(50);
    current_motion.in_progress = false;
}

bool IsKinematicSolutionValid(RobotState *robot) {
    float theta1_deg = RAD_TO_DEG(robot->theta[0]);
    float theta2_deg = RAD_TO_DEG(robot->theta[1]);
    float theta3_deg = RAD_TO_DEG(robot->theta[2]);
    if (theta1_deg < SERVO1_MIN_ANGLE || theta1_deg > SERVO1_MAX_ANGLE) return false;
    if (theta2_deg < SERVO2_MIN_ANGLE || theta2_deg > SERVO2_MAX_ANGLE) return false;
    if (theta3_deg < SERVO3_MIN_ANGLE || theta3_deg > SERVO3_MAX_ANGLE) return false;
    return true;
}

void StartMoveTo(RobotState *robot, float theta_deg, float phi_deg, float h, uint32_t duration_ms) { //IK - góc 3 servo - PWM PCA9685
    // Solve inverse kinematics for target position
    SolveInverseKinematicsSpherical(robot, theta_deg, phi_deg, h);
    
    if (duration_ms == 0) {
        // Immediate servo control without smoothing
        SetServoAngles(robot->theta[0], robot->theta[1], robot->theta[2]);
        current_motion.in_progress = false;
    } else {
        // Store current servo angles as start position
        for (int i = 0; i < 3; i++) {
            current_motion.start_theta[i] = current_servo_angles[i] * DEG_TO_RAD(1.0f);
            current_motion.end_theta[i] = robot->theta[i];
        }
        
        // Setup motion parameters
        current_motion.start_time = HAL_GetTick();
        current_motion.duration = duration_ms;
        current_motion.in_progress = true;
    }
}



void UpdateServoMotion(void) {
    if (!current_motion.in_progress) return;
    uint32_t now = HAL_GetTick();
    float t = (float)(now - current_motion.start_time) / current_motion.duration;
    if (t >= 1.0f) t = 1.0f;
    float theta[3];
    for (int i = 0; i < 3; i++) {
        theta[i] = current_motion.start_theta[i] + t * (current_motion.end_theta[i] - current_motion.start_theta[i]);
    }
    // Direct servo control for motion interpolation
    SetServoAngles(theta[0], theta[1], theta[2]);

    if (t >= 1.0f) {
        current_motion.in_progress = false;
    }
}

// Initialize servos to level position based on current h
void PerformInitialMovements(RobotState *robot) {
    // Calculate servo angles for level position at current height
    SolveInverseKinematicsSpherical(robot, 0.0f, 0.0f, robot->geom.h);
    
    // Move directly to level position with smooth motion
    StartMoveTo(robot, 0.0f, 0.0f, robot->geom.h, 1000);
    
    // Wait for movement to complete (non-blocking)
    uint32_t start_wait = HAL_GetTick();
    while (current_motion.in_progress && (HAL_GetTick() - start_wait < 2000)) {
        UpdateServoMotion();
        HAL_Delay(1);  // Reduced delay
    }
    
    // Small delay for stability
    HAL_Delay(100);
}
