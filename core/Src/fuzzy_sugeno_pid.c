#include "fuzzy_sugeno_pid.h"
#include "robot_geometry.h"
#include <math.h>
#include <stdbool.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

float Clip(float value, float min_val, float max_val) {
    if (value < min_val) return min_val;
    if (value > max_val) return max_val;
    return value;
}

// Normalize error with scaling and clipping
float NormalizeError(float error, float max_error, float scale_factor) {
    float normalized = (scale_factor * error) / max_error;
    return Clip(normalized, -1.0f, 1.0f);
}

// === MEMBERSHIP FUNCTION CALCULATION ===
// Triangular membership function
float TriangularMF(float x, float left, float center, float right) {
    // Handle edge cases where left == center or right == center
    if (left == center) {
        // Left shoulder membership function
        if (x <= center) return 1.0f;
        if (x >= right) return 0.0f;
        return (right - x) / (right - center);
    }
    else if (right == center) {
        // Right shoulder membership function
        if (x >= center) return 1.0f;
        if (x <= left) return 0.0f;
        return (x - left) / (center - left);
    }
    else {
        // Standard triangular membership function
        if (x <= left || x >= right) return 0.0f;
        if (x == center) return 1.0f;
        if (x < center) {
            return (x - left) / (center - left);
        } else {
            return (right - x) / (right - center);
        }
    }
}

void CalculateMembershipDegrees(FuzzySugenoPID *fpid, float e_norm, float de_norm) {
    // Calculate membership degrees for error (5 levels: NL, N, Z, P, PL)
    for (int i = 0; i < 5; i++) {
        fpid->mu_e[i] = TriangularMF(e_norm, fpid->mf_left[i], fpid->mf_centers[i], fpid->mf_right[i]);
        fpid->mu_de[i] = TriangularMF(de_norm, fpid->mf_left[i], fpid->mf_centers[i], fpid->mf_right[i]);
    }
}

// === FUZZY INFERENCE ===

// Common Sugeno inference function
static float SugenoInference(FuzzySugenoPID *fpid, float e_norm, float de_norm, const float rule_matrix[5][5]) {
    // Calculate membership degrees
    CalculateMembershipDegrees(fpid, e_norm, de_norm);
    
    // === RULE FIRING STRENGTHS ===
    float numerator = 0.0f;
    float denominator = 0.0f;
    
    // Calculate rule weights and weighted sum
    for (int i = 0; i < 5; i++) {
        for (int j = 0; j < 5; j++) {
            // Rule weight = min(mu_e[i], mu_de[j]) - Mamdani style
            fpid->rule_weights[i][j] = fminf(fpid->mu_e[i], fpid->mu_de[j]);
            // Weighted sum for Sugeno inference
            numerator += fpid->rule_weights[i][j] * rule_matrix[i][j];
            denominator += fpid->rule_weights[i][j];
        }
    }
    
    // === DEFUZZIFICATION ===
    float u_f = 0.0f;
    if (denominator > 0.001f) {  // Avoid division by zero
        u_f = numerator / denominator;
    }
    
    return u_f;
}

// PD fuzzy inference using PD rule matrix
float FuzzyInferenceAxis(FuzzySugenoPID *fpid, float e_norm, float de_norm, bool axisX) {
    const float (*rule_matrix)[5] = axisX ? fpid->sugeno_c_x : fpid->sugeno_c_y;
    return SugenoInference(fpid, e_norm, de_norm, rule_matrix);
}

// I fuzzy inference using I rule matrix
float FuzzyInferenceI(FuzzySugenoPID *fpid, float e_norm, float de_norm, bool axisX) {
    const float (*rule_matrix)[5] = axisX ? fpid->sugeno_c_i_x : fpid->sugeno_c_i_y;
    return SugenoInference(fpid, e_norm, de_norm, rule_matrix);
}

// === GAIN ADAPTATION ===

// Common gain adaptation function
static void AdaptGain(FuzzySugenoPID *fpid, float u_f, float A_coeff, float base_gain,
                      float min_gain, float max_gain, float *gain_use) {
    float delta_gain = A_coeff * u_f;
    *gain_use = Clip(base_gain + delta_gain, min_gain, max_gain);
}

// Adapt PID gains based on fuzzy output (legacy function)
void AdaptGains(FuzzySugenoPID *fpid, float u_f, float *Kp_use, float *Ki_use, float *Kd_use) {
    // Use PD coefficient for Kp and Kd, I coefficient for Ki
    AdaptGain(fpid, u_f, fpid->A_PD, fpid->Kp, fpid->Kp_min, fpid->Kp_max, Kp_use);
    AdaptGain(fpid, u_f, fpid->A_PD, fpid->Kd, fpid->Kd_min, fpid->Kd_max, Kd_use);
    AdaptGain(fpid, u_f, fpid->A_I, fpid->Ki, fpid->Ki_min, fpid->Ki_max, Ki_use);
}

// Adapt PD gains (Kp, Kd) based on PD fuzzy output
void AdaptGainsPD(FuzzySugenoPID *fpid, float u_f_pd, float *Kp_use, float *Kd_use) {
    AdaptGain(fpid, u_f_pd, fpid->A_PD, fpid->Kp, fpid->Kp_min, fpid->Kp_max, Kp_use);
    AdaptGain(fpid, u_f_pd, fpid->A_PD, fpid->Kd, fpid->Kd_min, fpid->Kd_max, Kd_use);
}

// Adapt I gain based on I fuzzy output
void AdaptGainsI(FuzzySugenoPID *fpid, float u_f_i, float *Ki_use) {
    AdaptGain(fpid, u_f_i, fpid->A_I, fpid->Ki, fpid->Ki_min, fpid->Ki_max, Ki_use);
}

// === INITIALIZATION FUNCTIONS ===

// Initialize basic PID parameters
void FuzzyPIDInit(FuzzySugenoPID *fpid, float Kp, float Ki, float Kd, float alpha, float beta, int conversion_mode) {
    // Basic PID gains
    fpid->Kp = Kp;
    fpid->Ki = Ki;
    fpid->Kd = Kd;
    
    // Initialize setpoints
    fpid->setpointX = 0.0f;
    fpid->setpointY = 0.0f;
    
    // Initialize integral terms
    fpid->integralX = 0.0f;
    fpid->integralY = 0.0f;
    
    // Initialize derivative terms
    fpid->derivativeX = 0.0f;
    fpid->derivativeY = 0.0f;
    // Derivative LPF
    fpid->d_alpha = 1.0f;
    fpid->prev_D_x = 0.0f;
    fpid->prev_D_y = 0.0f;
    
    // Initialize previous values
    fpid->prev_err_x = 0.0f;
    fpid->prev_err_y = 0.0f;
    fpid->prev_time = 0;
    
    // Initialize output filter
    fpid->alpha = alpha;
    fpid->prev_out_x = 0.0f;
    fpid->prev_out_y = 0.0f;
    
    // Initialize magnitude conversion
    fpid->beta = beta;
    fpid->max_theta = 10.0f;  // Đặt riêng max_theta = 10.0°
    fpid->magnitudeConvert = conversion_mode;
    
    // Initialize output values
    fpid->output_theta = 0.0f;
    fpid->output_phi = 0.0f;
    
    // Initialize fuzzy parameters (default values)
    fpid->fuzzy_enabled = true;
    fpid->E_max = 0.0f;
    fpid->D_max = 0.0f;
    fpid->K_e = 0.0f;
    fpid->K_d_in = 0.0f;
    fpid->A_PD = 0.0f;  // PD gain adjustment coefficient
    fpid->A_I = 0.0f;   // I gain adjustment coefficient
    fpid->U_mid = 0.0f;
    fpid->U_max = 0.0f;
    
    // Initialize gain limits
    fpid->Kp_min = 0.0f;
    fpid->Kp_max = 0.0f;
    fpid->Ki_min = 0.0f;
    fpid->Ki_max = 0.0f;
    fpid->Kd_min = 0.0f;
    fpid->Kd_max = 0.0f;
    
    // Initialize adaptive gains to base values
    fpid->Kp_x_use = Kp;
    fpid->Kp_y_use = Kp;
    fpid->Ki_x_use = Ki;
    fpid->Ki_y_use = Ki;
    fpid->Kd_x_use = Kd;
    fpid->Kd_y_use = Kd;
    
    // Initialize fuzzy outputs
    fpid->u_f_x = 0.0f;
    fpid->u_f_y = 0.0f;
    fpid->u_f_i_x = 0.0f;
    fpid->u_f_i_y = 0.0f;
}

// Initialize fuzzy parameters and rule matrix
void FuzzyPIDInitFuzzy(FuzzySugenoPID *fpid, float E_max, float D_max, float K_e, float K_d_in,
                       float A_PD, float A_I, float U_mid, float U_max) {
    // Set fuzzy parameters
    fpid->E_max = E_max;
    fpid->D_max = D_max;
    fpid->K_e = K_e;
    fpid->K_d_in = K_d_in;
    fpid->A_PD = A_PD;
    fpid->A_I = A_I;
    fpid->U_mid = U_mid;
    fpid->U_max = U_max;
    
    // Initialize membership function parameters for 5 levels (NL, N, Z, P, PL)
    // Centers: [-1, -0.5, 0, 0.5, 1] - Giữ nguyên các đỉnh
    fpid->mf_centers[0] = -0.15f;//-0.15f;  // NL-1.0f
    fpid->mf_centers[1] = -0.1f;//-0.1f;  // N-0.5f
    fpid->mf_centers[2] = 0.0f;//0.0f;   // Z 0.0f
    fpid->mf_centers[3] = 0.1f;//0.1f;   // P0.5f
    fpid->mf_centers[4] = 0.15f;//0.15f;   // PL1.0f

    // Left points: Thu hẹp vùng Z đến mức tối thiểu
    fpid->mf_left[0] = -0.2f;//-0.2f;-1.0f
    fpid->mf_left[1] = -0.2f;//-0.2f;-1.0f
    fpid->mf_left[2] = -0.1f;//-0.1f;-0.5f
    fpid->mf_left[3] = 0.0f;//0.0f;0.0f
    fpid->mf_left[4] = 0.1f;//0.1f;0.5f

    // Right points:
    fpid->mf_right[0] = -0.1f;//-0.1f;-0.5f
    fpid->mf_right[1] = 0.0f;//0.0f;0.0f
    fpid->mf_right[2] = 0.1f;//0.1f;0.5f
    fpid->mf_right[3] = 0.2f;//0.2f;1.0f
    fpid->mf_right[4] = 0.2f;//0.2f;1.0f
    
    // Initialize Sugeno rule matrices (5x5) with explicit singleton values
    float BN = -U_max;
    float N  = -U_mid;
    float Z  =  0.0f;
    float P  =  U_mid;
    float BP =  U_max;

    // X-axis rule matrix e\de 
    float rules_x[5][5] = {
        { BP, P , Z , N , N },
        { BP , P , Z , N , N },
        { P , Z , N , Z , P },
        { N , N , Z , P , BP },
        { N , N , Z , P , BP }
    };
    // Y-axis rule matrix
    float rules_y[5][5] = {
    	{ BP, P , Z , N , N },
    	{ BP , P , Z , N , N },
        { P , Z , N , Z , P },
        { N , N , Z , P , BP },
    	{ N , N , Z , P , BP }

    };

    for (int i = 0; i < 5; i++) {
        for (int j = 0; j < 5; j++) {
            fpid->sugeno_c_x[i][j] = rules_x[i][j];
            fpid->sugeno_c_y[i][j] = rules_y[i][j];
        }
    }
    
    // Initialize I rule matrix with same orientation as PD (e\de)
    // Philosophy: I should be small when |e| is large or |de| is large (avoid integral saturation),
    // and increase when e is near 0 to "eliminate steady-state error"
    float rules_i[5][5] = {
    		/* e\de:    NL,     NS,     Z,      PS,     PL  */
    		/* NL */ {  Z,      Z,      P,      Z,      Z  },
    		/* NS */ {  N,      Z,      P,      Z,      N  }, // Tăng cường khi e gần 0
    		/*  Z */ {  N,      N,      P,      N,     N  }, // DÙNG "BIG POSITIVE" KHI ĐỨNG YÊN
    		/* PS */ {  N,      Z,      P,      Z,      N }, // Sửa đổi nhỏ để đối xứng
    		/* PL */ {  Z,      Z,      P,      Z,      Z  }, // Sửa đổi nhỏ để đối xứng
    };

    // Copy I rules to I matrices (same for X and Y)
    for (int i = 0; i < 5; i++) {
        for (int j = 0; j < 5; j++) {
            fpid->sugeno_c_i_x[i][j] = rules_i[i][j];
            fpid->sugeno_c_i_y[i][j] = rules_i[i][j];
        }
    }

    // Initialize membership degrees and rule weights
    for (int i = 0; i < 5; i++) {
        fpid->mu_e[i] = 0.0f;
        fpid->mu_de[i] = 0.0f;
        for (int j = 0; j < 5; j++) {
            fpid->rule_weights[i][j] = 0.0f;
        }
    }
}

// === RESET AND TARGET FUNCTIONS ===

// Reset PID controller (preserves fuzzy parameters)
void FuzzyPIDReset(FuzzySugenoPID *fpid) {
    // Reset integral terms
    fpid->integralX = 0.0f;
    fpid->integralY = 0.0f;
    
    // Reset derivative terms
    fpid->derivativeX = 0.0f;
    fpid->derivativeY = 0.0f;
    fpid->prev_D_x = 0.0f;
    fpid->prev_D_y = 0.0f;
    
    // Reset previous values
    fpid->prev_err_x = 0.0f;
    fpid->prev_err_y = 0.0f;
    fpid->prev_out_x = 0.0f;
    fpid->prev_out_y = 0.0f;
    fpid->prev_time = HAL_GetTick();
    
    // Reset output values
    fpid->output_theta = 0.0f;
    fpid->output_phi = 0.0f;
    
    // Reset fuzzy outputs
    fpid->u_f_x = 0.0f;
    fpid->u_f_y = 0.0f;
    fpid->u_f_i_x = 0.0f;
    fpid->u_f_i_y = 0.0f;
    
    // Reset adaptive gains to base values
    fpid->Kp_x_use = fpid->Kp;
    fpid->Kp_y_use = fpid->Kp;
    fpid->Ki_x_use = fpid->Ki;
    fpid->Ki_y_use = fpid->Ki;
    fpid->Kd_x_use = fpid->Kd;
    fpid->Kd_y_use = fpid->Kd;
}

// Set target position
void FuzzyPIDSetTarget(FuzzySugenoPID *fpid, float targetX, float targetY) {
    fpid->setpointX = targetX;
    fpid->setpointY = targetY;
}

// Set target with PID reset to prevent overshoot for Fuzzy PID
void FuzzyPIDSetTargetWithReset(FuzzySugenoPID *fpid, float targetX, float targetY) {
    // Set new target
    fpid->setpointX = targetX;
    fpid->setpointY = targetY;
    
    // Reset PID state to prevent integral windup and overshoot
    fpid->integralX = 0.0f;
    fpid->integralY = 0.0f;
    fpid->derivativeX = 0.0f;
    fpid->derivativeY = 0.0f;
    fpid->prev_D_x = 0.0f;
    fpid->prev_D_y = 0.0f;
    fpid->prev_err_x = 0.0f;
    fpid->prev_err_y = 0.0f;
    fpid->prev_out_x = 0.0f;
    fpid->prev_out_y = 0.0f;
    
    // Reset fuzzy outputs
    fpid->u_f_x = 0.0f;
    fpid->u_f_y = 0.0f;
    fpid->u_f_i_x = 0.0f;
    fpid->u_f_i_y = 0.0f;
    
    // Reset adaptive gains to base values
    fpid->Kp_x_use = fpid->Kp;
    fpid->Kp_y_use = fpid->Kp;
    fpid->Ki_x_use = fpid->Ki;
    fpid->Ki_y_use = fpid->Ki;
    fpid->Kd_x_use = fpid->Kd;
    fpid->Kd_y_use = fpid->Kd;
}

// === FUZZY CONTROL FUNCTIONS ===

// Enable/disable fuzzy control
void FuzzyPIDEnableFuzzy(FuzzySugenoPID *fpid, bool enabled) {
    fpid->fuzzy_enabled = enabled;
    
    // Reset adaptive gains to base values when disabling
    if (!enabled) {
        fpid->Kp_x_use = fpid->Kp;
        fpid->Kp_y_use = fpid->Kp;
        fpid->Ki_x_use = fpid->Ki;
        fpid->Ki_y_use = fpid->Ki;
        fpid->Kd_x_use = fpid->Kd;
        fpid->Kd_y_use = fpid->Kd;
        fpid->u_f_x = 0.0f;
        fpid->u_f_y = 0.0f;
        fpid->u_f_i_x = 0.0f;
        fpid->u_f_i_y = 0.0f;
    }
}

// Set gain limits for adaptation
void FuzzyPIDSetGainLimits(FuzzySugenoPID *fpid, float Kp_min, float Kp_max, 
                           float Ki_min, float Ki_max, float Kd_min, float Kd_max) {
    fpid->Kp_min = Kp_min;
    fpid->Kp_max = Kp_max;
    fpid->Ki_min = Ki_min;
    fpid->Ki_max = Ki_max;
    fpid->Kd_min = Kd_min;
    fpid->Kd_max = Kd_max;
}

// === MAIN CONTROL FUNCTIONS ===

// Main fuzzy PID update function with internal time calculation
float FuzzyPIDUpdateWithTime(FuzzySugenoPID *fpid, float currentX, float currentY) {
    // === 1. CALCULATE DT INTERNALLY ===
    uint32_t current_time = HAL_GetTick();
    float dt;
    
    // Handle first call
    if (fpid->prev_time == 0) {
        dt = 0.0167f;  // Default dt for first call (60fps)
    } else {
        dt = (float)(current_time - fpid->prev_time) / 1000.0f;  // Convert to seconds
    }

    // Clamp dt to a stable 60Hz window
    if (dt <= 0.0f || dt > 0.02f) {
        dt = 0.0167f;  // Fixed 60Hz
    } else if (dt < 0.015f) {
        dt = 0.0167f;  // Ignore too-fast updates
    }
    
    // === 2. ERROR CALCULATION ===
    float err_x = (currentX - fpid->setpointX);
    float err_y = (currentY - fpid->setpointY);
    
    // === 3. FUZZY CONTROL (if enabled) ===
    if (fpid->fuzzy_enabled) {
        // Normalize errors
        float e_norm_x = NormalizeError(err_x, fpid->E_max, fpid->K_e);
        float e_norm_y = NormalizeError(err_y, fpid->E_max, fpid->K_e);
        
        // Calculate derivative errors
        float de_x = 0.0f, de_y = 0.0f;
        if (dt > 0.0f) {
            de_x = (err_x - fpid->prev_err_x) / dt;
            de_y = (err_y - fpid->prev_err_y) / dt;
        }
        
        // Normalize derivative errors
        float de_norm_x = NormalizeError(de_x, fpid->D_max, fpid->K_d_in);
        float de_norm_y = NormalizeError(de_y, fpid->D_max, fpid->K_d_in);
        
        // Fuzzy inference for PD and I terms separately
        fpid->u_f_x = FuzzyInferenceAxis(fpid, -e_norm_x, -de_norm_x, true);  // PD for X
        fpid->u_f_y = FuzzyInferenceAxis(fpid, -e_norm_y, -de_norm_y, false); // PD for Y
        fpid->u_f_i_x = FuzzyInferenceI(fpid, -e_norm_x, -de_norm_x, true);   // I for X
        fpid->u_f_i_y = FuzzyInferenceI(fpid, -e_norm_y, -de_norm_y, false);  // I for Y
        
        // Adapt gains separately: PD and I
        AdaptGainsPD(fpid, fpid->u_f_x, &fpid->Kp_x_use, &fpid->Kd_x_use);
        AdaptGainsPD(fpid, fpid->u_f_y, &fpid->Kp_y_use, &fpid->Kd_y_use);
        AdaptGainsI(fpid, fpid->u_f_i_x, &fpid->Ki_x_use);
        AdaptGainsI(fpid, fpid->u_f_i_y, &fpid->Ki_y_use);
    } else {
        // Use base gains when fuzzy control is disabled
        fpid->Kp_x_use = fpid->Kp;
        fpid->Kp_y_use = fpid->Kp;
        fpid->Ki_x_use = fpid->Ki;
        fpid->Ki_y_use = fpid->Ki;
        fpid->Kd_x_use = fpid->Kd;
        fpid->Kd_y_use = fpid->Kd;
        fpid->u_f_x = 0.0f;
        fpid->u_f_y = 0.0f;
        fpid->u_f_i_x = 0.0f;
        fpid->u_f_i_y = 0.0f;
    }
    
    // === 4. PROPORTIONAL TERM ===
    float P_x = fpid->Kp_x_use * err_x;
    float P_y = fpid->Kp_y_use * err_y;
    
    // === 5. DERIVATIVE TERM ===
    float d_err_x = 0.0f;
    float d_err_y = 0.0f;
    
    if (dt > 0.0f) {
        d_err_x = (err_x - fpid->prev_err_x) / dt;
        d_err_y = (err_y - fpid->prev_err_y) / dt;
    }
    
    fpid->derivativeX = d_err_x;
    fpid->derivativeY = d_err_y;
    
    // Low-pass filter for D term: D_f(k)= (1-d_alpha) D_f(k-1) + d_alpha * D_raw(k)
    float D_raw_x = fpid->Kd_x_use * fpid->derivativeX;
    float D_raw_y = fpid->Kd_y_use * fpid->derivativeY;
    float D_x = (1.0f - fpid->d_alpha) * fpid->prev_D_x + fpid->d_alpha * D_raw_x;
    float D_y = (1.0f - fpid->d_alpha) * fpid->prev_D_y + fpid->d_alpha * D_raw_y;
    fpid->prev_D_x = D_x;
    fpid->prev_D_y = D_y;
    
    // === 6. INTEGRAL TERM ===
    float I_x = fpid->Ki_x_use * fpid->integralX;
    float I_y = fpid->Ki_y_use * fpid->integralY;
    
    // === 7. PID OUTPUT ===
    float pid_output_x = P_x + I_x + D_x;
    float pid_output_y = P_y + I_y + D_y;
    
    // === 8. OUTPUT FILTERING ===
    float filtered_x = fpid->alpha * pid_output_x + (1.0f - fpid->alpha) * fpid->prev_out_x;
    float filtered_y = fpid->alpha * pid_output_y + (1.0f - fpid->alpha) * fpid->prev_out_y;
    
    // === 9. CONVERT TO POLAR COORDINATES ===
    float phi = atan2f(filtered_y, filtered_x) * 180.0f / M_PI;
    if (phi < 0.0f) phi += 360.0f;
    float r = sqrtf(filtered_x * filtered_x + filtered_y * filtered_y);
    
    // === 10. MAGNITUDE CONVERSION ===
    float theta_unsaturated;
    float theta_saturated;
    
    // Calculate unsaturated theta based on conversion mode
    // Magnitude conversion - giống Python PID.py
    if (fpid->magnitudeConvert == 1) { // Linear mode (giống Python)
        theta_unsaturated = fpid->beta * r;
        // Giới hạn như Python: min(max(0, beta*r), max_theta)
        if (theta_unsaturated < 0.0f) {
            theta_saturated = 0.0f;
        } else if (theta_unsaturated > fpid->max_theta) {
            theta_saturated = fpid->max_theta;
        } else {
            theta_saturated = theta_unsaturated;
        }
    } else if (fpid->magnitudeConvert == 0) { // Tanh mode (giống Python)
        theta_unsaturated = 2.75f * tanhf(fpid->beta * r);  // Giống Python: 15*tanh(beta*r)
        // Giới hạn như Python: max(0, 15*tanh(beta*r))
        if (theta_unsaturated < 0.0f) {
            theta_saturated = 0.0f;
        } else {
            theta_saturated = theta_unsaturated;
        }
    } else { // Default to linear
        theta_unsaturated = fpid->beta * r;
        if (theta_unsaturated < 0.0f) {
            theta_saturated = 0.0f;
        } else if (theta_unsaturated > fpid->max_theta) {
            theta_saturated = fpid->max_theta;
        } else {
            theta_saturated = theta_unsaturated;
        }
    }
    
    // === 11. INTEGRAL UPDATE ===
    fpid->integralX += err_x * dt;
    fpid->integralY += err_y * dt;
    
    // === 12. SAVE VALUES FOR NEXT ITERATION ===
    fpid->prev_err_x = err_x;
    fpid->prev_err_y = err_y;
    fpid->prev_out_x = filtered_x;
    fpid->prev_out_y = filtered_y;
    fpid->prev_time = current_time;
    
    // === 13. OUTPUT ===
    fpid->output_theta = theta_saturated;
    fpid->output_phi = phi;
    
    return theta_saturated;
}

// Legacy function for compatibility
float FuzzyPIDUpdate(FuzzySugenoPID *fpid, float currentX, float currentY) {
    return FuzzyPIDUpdateWithTime(fpid, currentX, currentY);
}

