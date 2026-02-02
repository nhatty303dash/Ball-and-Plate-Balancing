#include "pid_controller.h"
#include "robot_geometry.h"
#include <math.h>
#include <stdbool.h>

// Reset PID Controller
void PIDReset(PIDController *pid) {
    // Reset integral terms
    pid->integralX = 0.0f;
    pid->integralY = 0.0f;
    
    // Reset derivative terms
    pid->derivativeX = 0.0f;
    pid->derivativeY = 0.0f;
    
    // Reset filtered derivative terms
    pid->filtered_derivativeX = 0.0f;
    pid->filtered_derivativeY = 0.0f;
    
    // Reset previous values
    pid->prev_err_x = 0.0f;
    pid->prev_err_y = 0.0f;
    pid->prev_out_x = 0.0f;
    pid->prev_out_y = 0.0f;
    pid->prev_time = HAL_GetTick();
    
    // Reset output values
    pid->output_theta = 0.0f;
    pid->output_phi = 0.0f;
    

    
    // Note: is_first_call reset is now handled in main loop
}

// Dat diem dich
void PIDSetTarget(PIDController *pid, float targetX, float targetY) {
    pid->setpointX = targetX;
    pid->setpointY = targetY;
}

// Set target with PID reset to prevent overshoot
void PIDSetTargetWithReset(PIDController *pid, float targetX, float targetY) {
    // Set new target
    pid->setpointX = targetX;
    pid->setpointY = targetY;
    
    // Reset PID state to prevent integral windup and overshoot
    pid->integralX = 0.0f;
    pid->integralY = 0.0f;
    pid->derivativeX = 0.0f;
    pid->derivativeY = 0.0f;
    pid->filtered_derivativeX = 0.0f;
    pid->filtered_derivativeY = 0.0f;
    pid->prev_err_x = 0.0f;
    pid->prev_err_y = 0.0f;
    pid->prev_out_x = 0.0f;
    pid->prev_out_y = 0.0f;
}


// === NEW FUNCTION: PID WITH INTERNAL DT CALCULATION ===
// This function calculates dt internally using HAL_GetTick()
// Usage: tilt = PIDUpdateSimpleWithTime(&pid, currentX, currentY);
// No need to calculate dt in main loop anymore
float PIDUpdateSimpleWithTime(PIDController *pid, float currentX, float currentY) {
    // === 1. CALCULATE DT INTERNALLY ===
    uint32_t current_time = HAL_GetTick();
    float dt;
    
    // Handle first call (like Python: dt ≈ 0.0167 if last_time is None)
    if (pid->prev_time == 0) {
        dt = 0.0167f;  // Default dt for first call (60fps)
    } else {
        dt = (float)(current_time - pid->prev_time) / 1000.0f;  // Convert to seconds
    }

    // Clamp dt to a stable 60Hz window
    // - Too slow (>20ms) or invalid (<=0): force 16.7ms
    // - Too fast (<15ms): also force 16.7ms to avoid jitter amplification
    if (dt <= 0.0f || dt > 0.02f) {
        dt = 0.0167f;   // Fixed 60Hz
    } else if (dt < 0.015f) {
        dt = 0.0167f;   // Ignore too-fast updates
    }
    
    // === 2. ERROR CALCULATION ===
    float err_x = (currentX - pid->setpointX);
    float err_y = (currentY - pid->setpointY);
    
    // === 2.5. DEADZONE ===
    /*float error_magnitude = sqrtf(err_x*err_x + err_y*err_y);
    if (error_magnitude < 3.0f) {  // Deadzone 3mm
        err_x = 0.0f;
        err_y = 0.0f;
    }*/
    
    // === 3. PROPORTIONAL TERM ===
    float P_x = pid->Kp * err_x;
    float P_y = pid->Kp * err_y;
    
    // === 4. DERIVATIVE TERM ===
    float d_err_x = 0.0f;
    float d_err_y = 0.0f;
    
    if (dt > 0.0f) {
        d_err_x = (err_x - pid->prev_err_x) / dt;
        d_err_y = (err_y - pid->prev_err_y) / dt;
    }
    
    // Apply low-pass filter to derivative term (theo công thức 4.15)
    // u^D_n(k) = (1-α)u^D_n(k-1) + α*u^D(k)
    pid->filtered_derivativeX = (1.0f - pid->derivative_filter_alpha) * pid->filtered_derivativeX + 
                               pid->derivative_filter_alpha * d_err_x;
    pid->filtered_derivativeY = (1.0f - pid->derivative_filter_alpha) * pid->filtered_derivativeY + 
                               pid->derivative_filter_alpha * d_err_y;
    
    // Store unfiltered derivatives for reference
    pid->derivativeX = d_err_x;
    pid->derivativeY = d_err_y;
    
    // Use filtered derivatives in PID calculation
    float D_x = pid->Kd * pid->filtered_derivativeX;
    float D_y = pid->Kd * pid->filtered_derivativeY;
    
    // === 5. INTEGRAL TERM ===
    float I_x = pid->Ki * pid->integralX;
    float I_y = pid->Ki * pid->integralY;
    
    // === 6. PID OUTPUT ===
    float pid_output_x = P_x + I_x + D_x;
    float pid_output_y = P_y + I_y + D_y;
    
    // === 7. OUTPUT FILTERING ===
    float filtered_x = pid->alpha * pid_output_x + (1.0f - pid->alpha) * pid->prev_out_x;
    float filtered_y = pid->alpha * pid_output_y + (1.0f - pid->alpha) * pid->prev_out_y;
    
    // === 8. CONVERT TO POLAR COORDINATES ===
    float phi = atan2f(filtered_y, filtered_x) * 180.0f / M_PI;
    if (phi < 0.0f) phi += 360.0f;
    float r = sqrtf(filtered_x * filtered_x + filtered_y * filtered_y);
    // === 9. MAGNITUDE CONVERSION ===
    float theta_unsaturated;
    float theta_saturated;
    // This stage now effectively just scales 'r' back to a physical angle.
    // The actual saturation was done on the PID output magnitude 'u_mag'.
    // Magnitude conversion
    if (pid->magnitudeConvert == 1) { // Linear mode
        theta_unsaturated = pid->beta * r;
        // Giới hạn : min(max(0, beta*r), max_theta)
        if (theta_unsaturated < 0.0f) {
            theta_saturated = 0.0f;
        } else if (theta_unsaturated > pid->max_theta) {
            theta_saturated = pid->max_theta;
        } else {
            theta_saturated = theta_unsaturated;
        }
    } else if (pid->magnitudeConvert == 0) { // Tanh mode
        theta_unsaturated = 4.2f * tanhf(pid->beta * r);
        if (theta_unsaturated < 0.0f) {
            theta_saturated = 0.0f;
        } else {
            theta_saturated = theta_unsaturated;
        }
    } else { // Default to linear
        theta_unsaturated = pid->beta * r;
        if (theta_unsaturated < 0.0f) {
            theta_saturated = 0.0f;
        } else if (theta_unsaturated > pid->max_theta) {
            theta_saturated = pid->max_theta;
        } else {
            theta_saturated = theta_unsaturated;
        }
    }
    
    // === 10. INTEGRAL UPDATE ===
    pid->integralX += err_x * dt;
    pid->integralY += err_y * dt;
    
    // === 11. SAVE VALUES FOR NEXT ITERATION ===
    pid->prev_err_x = err_x;
    pid->prev_err_y = err_y;
    pid->prev_out_x = filtered_x;
    pid->prev_out_y = filtered_y;
    pid->prev_time = current_time;  // Update time for next iteration
    
    // === 12. OUTPUT ===
    pid->output_theta = theta_saturated;
    pid->output_phi = phi;
    
    return theta_saturated;
}

// Initialize PID with new parameters
void PIDInit(PIDController *pid, float Kp, float Ki, float Kd, float alpha, float beta, int conversion_mode) {
    // Basic PID gains
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    
    // Initialize setpoints
    pid->setpointX = 0.0f;
    pid->setpointY = 0.0f;
    
    // Initialize integral terms
    pid->integralX = 0.0f;
    pid->integralY = 0.0f;
    
    // Initialize derivative terms
    pid->derivativeX = 0.0f;
    pid->derivativeY = 0.0f;
    
    // Initialize filtered derivative terms
    pid->filtered_derivativeX = 0.0f;
    pid->filtered_derivativeY = 0.0f;
    
    // Initialize derivative filter coefficient (α = 0.2 cho low-pass filter mạnh)
    pid->derivative_filter_alpha = 0.2f;  // Giá trị nhỏ = filter mạnh hơn
    
    // Initialize previous values
    pid->prev_err_x = 0.0f;
    pid->prev_err_y = 0.0f;
    pid->prev_time = 0;  // Initialize to 0 for first call detection
    
    // Initialize output filter - truyền trực tiếp như Python
    pid->alpha = alpha;
    pid->prev_out_x = 0.0f;
    pid->prev_out_y = 0.0f;
    
    // Initialize magnitude conversion
    pid->beta = beta;
    pid->max_theta = 20.0f;  // Đặt riêng max_theta = 10.0°
    pid->magnitudeConvert = conversion_mode;  // 1=linear, 2=tanh
    
    // Initialize output values
    pid->output_theta = 0.0f;
    pid->output_phi = 0.0f;
    

}




// Set output filter coefficient
void PIDSetOutputFilter(PIDController *pid, float alpha) {
    pid->alpha = alpha;
}

// Set magnitude conversion parameters
void PIDSetMagnitudeParams(PIDController *pid, float beta, int conversion_mode) {
    pid->beta = beta;
    pid->magnitudeConvert = conversion_mode;  // 1=linear, 2=tanh
}

// Set derivative filter coefficient (theo công thức 4.15)
void PIDSetDerivativeFilter(PIDController *pid, float derivative_alpha) {
    // Clamp alpha between 0.05 and 1.0
    if (derivative_alpha < 0.05f) derivative_alpha = 0.05f;
    if (derivative_alpha > 1.0f) derivative_alpha = 1.0f;
    
    pid->derivative_filter_alpha = derivative_alpha;
    // α nhỏ = filter mạnh (ít noise), α lớn = response nhanh (nhiều noise)
}

// Thêm hàm set max_theta riêng biệt
void PIDSetMaxTheta(PIDController *pid, float max_theta) {
    pid->max_theta = max_theta;
}


