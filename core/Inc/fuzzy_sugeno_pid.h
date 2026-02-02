#ifndef FUZZY_SUGENO_PID_H
#define FUZZY_SUGENO_PID_H

#include "robot_geometry.h"
#include "stm32f4xx_hal.h"
#include <stdbool.h>

// Fuzzy Sugeno PID Controller Structure
typedef struct {
    // === BASIC PID PARAMETERS (same as original PID) ===
    float Kp, Ki, Kd;
    float setpointX, setpointY;
    
    // Integral terms with anti-windup
    float integralX, integralY;
    
    // Derivative terms
    float derivativeX, derivativeY;
    // Derivative low-pass filter (first-order): D_f(k)=d_alpha*D_f(k-1)+(1-d_alpha)*D(k)
    float d_alpha;             // 0..1, closer to 1 = heavier filtering
    float prev_D_x;            // previous filtered derivative X
    float prev_D_y;            // previous filtered derivative Y
    
    // Previous values for calculations
    float prev_err_x, prev_err_y;
    uint32_t prev_time;
    
    // Output filtering
    float alpha;
    float prev_out_x, prev_out_y;
    
    // Magnitude conversion
    float beta;
    float max_theta;
    int magnitudeConvert;      // 1=linear, 0=tanh
    
    // Output values
    float output_theta;
    float output_phi;
    
    // === FUZZY SUGENO PARAMETERS ===
    bool fuzzy_enabled;         // Enable/disable fuzzy control
    
    // Normalization parameters
    float E_max;               // Maximum error for normalization
    float D_max;               // Maximum derivative error for normalization
    float K_e;                 // Error scaling factor
    float K_d_in;              // Derivative error scaling factor
    
    // Fuzzy gain adjustment parameters
    float A_PD;                // PD gain adjustment coefficient
    float A_I;                 // I gain adjustment coefficient
    
    // Fuzzy output parameters
    float U_mid;               // Mid-level fuzzy output
    float U_max;               // Maximum fuzzy output
    
    // === FUZZY MEMBERSHIP FUNCTIONS ===
    // Triangular membership function parameters for 5 levels (NL, N, Z, P, PL)
    float mf_centers[5];       // Center points: [-1, -0.5, 0, 0.5, 1]
    float mf_left[5];          // Left points: [-1, -1, -0.5, 0, 0.5]
    float mf_right[5];         // Right points: [-0.5, 0, 0.5, 1, 1]
    
    // === SUGENO RULE MATRICES (per-axis) ===
    // 5x5 rule matrices for Sugeno singleton outputs
    float sugeno_c_x[5][5];    // X-axis PD rule matrix
    float sugeno_c_y[5][5];    // Y-axis PD rule matrix
    float sugeno_c_i_x[5][5];  // X-axis I rule matrix
    float sugeno_c_i_y[5][5];  // Y-axis I rule matrix
    
    // === FUZZY INFERENCE VARIABLES ===
    float mu_e[5];             // Error membership degrees
    float mu_de[5];            // Derivative error membership degrees
    float rule_weights[5][5];  // Rule firing strengths
    
    // === FUZZY OUTPUTS ===
    float u_f_x;               // PD fuzzy output for X axis
    float u_f_y;               // PD fuzzy output for Y axis
    float u_f_i_x;             // I fuzzy output for X axis
    float u_f_i_y;             // I fuzzy output for Y axis
    
    // === ADAPTIVE GAINS ===
    float Kp_x_use, Kp_y_use;  // Current P gains for X/Y
    float Ki_x_use, Ki_y_use;  // Current I gains for X/Y
    float Kd_x_use, Kd_y_use;  // Current D gains for X/Y
    
    // Gain limits
    float Kp_min, Kp_max;
    float Ki_min, Ki_max;
    float Kd_min, Kd_max;
    
} FuzzySugenoPID;

// === FUNCTION PROTOTYPES ===

// === INITIALIZATION FUNCTIONS ===
void FuzzyPIDInit(FuzzySugenoPID *fpid, float Kp, float Ki, float Kd, 
                  float alpha, float beta, int conversion_mode);

void FuzzyPIDInitFuzzy(FuzzySugenoPID *fpid, float E_max, float D_max, float K_e, float K_d_in,
                       float A_PD, float A_I, float U_mid, float U_max);

// === RESET AND TARGET FUNCTIONS ===
void FuzzyPIDReset(FuzzySugenoPID *fpid);
void FuzzyPIDSetTarget(FuzzySugenoPID *fpid, float targetX, float targetY);
void FuzzyPIDSetTargetWithReset(FuzzySugenoPID *fpid, float targetX, float targetY);

// === MAIN CONTROL FUNCTIONS ===
float FuzzyPIDUpdate(FuzzySugenoPID *fpid, float currentX, float currentY);
float FuzzyPIDUpdateWithTime(FuzzySugenoPID *fpid, float currentX, float currentY);

// === FUZZY CONTROL FUNCTIONS ===
void FuzzyPIDEnableFuzzy(FuzzySugenoPID *fpid, bool enabled);
void FuzzyPIDSetGainLimits(FuzzySugenoPID *fpid, float Kp_min, float Kp_max, 
                           float Ki_min, float Ki_max, float Kd_min, float Kd_max);

// Configure derivative LPF coefficient (0..1)
static inline void FuzzyPIDSetDerivativeFilter(FuzzySugenoPID *fpid, float d_alpha)
{
    fpid->d_alpha = d_alpha;
}

// Thêm hàm set max_theta riêng cho Fuzzy PID
static inline void FuzzyPIDSetMaxTheta(FuzzySugenoPID *fpid, float max_theta)
{
    fpid->max_theta = max_theta;
}

// === MEMBERSHIP FUNCTION CALCULATION ===
float TriangularMF(float x, float left, float center, float right);
void CalculateMembershipDegrees(FuzzySugenoPID *fpid, float e_norm, float de_norm);

// === FUZZY INFERENCE ===
float FuzzyInferenceAxis(FuzzySugenoPID *fpid, float e_norm, float de_norm, bool axisX);
float FuzzyInferenceI(FuzzySugenoPID *fpid, float e_norm, float de_norm, bool axisX);

// === GAIN ADAPTATION ===
void AdaptGains(FuzzySugenoPID *fpid, float u_f, float *Kp_use, float *Ki_use, float *Kd_use);
void AdaptGainsPD(FuzzySugenoPID *fpid, float u_f_pd, float *Kp_use, float *Kd_use);
void AdaptGainsI(FuzzySugenoPID *fpid, float u_f_i, float *Ki_use);

// === UTILITY FUNCTIONS ===
float Clip(float value, float min_val, float max_val);
float NormalizeError(float error, float max_error, float scale_factor);

#endif /* FUZZY_SUGENO_PID_H */

