#ifndef SERVO_CONTROL_H
#define SERVO_CONTROL_H

#include "robot_geometry.h"
#include "pca9685_servo.h"  // Changed from pca9685.h
#include <stdbool.h>

// Structure to track non-blocking motion
typedef struct {
    float start_theta[3];
    float end_theta[3];
    uint32_t start_time;
    uint32_t duration;
    bool in_progress;
} ServoMotion;

// External access to motion status
extern ServoMotion current_motion;

// Function prototypes for servo control
void ServoInit(void);
bool IsKinematicSolutionValid(RobotState *robot);
void StartMoveTo(RobotState *robot, float theta_deg, float phi_deg, float h, uint32_t duration_ms);
void UpdateServoMotion(void);

// Initial movement for system verification
void PerformInitialMovements(RobotState *robot);

#endif /* SERVO_CONTROL_H */
