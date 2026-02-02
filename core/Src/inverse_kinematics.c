#include "inverse_kinematics.h"
#include <stdbool.h>

// Tinh diem A tren mat phang tren cua robot
void SolveTop(RobotState *robot, float a, float b, float c, float h) {
    // A1 - Theo công thức lý thuyết trang 4
    float denom1 = sqrtf(4.0f * SQUARE(c) + SQUARE(a - SQRT3 * b));
    robot->A[0].x = -(robot->geom.lp * c) / denom1;
    robot->A[0].y = (SQRT3 * robot->geom.lp * c) / denom1;
    robot->A[0].z = h + ((a - SQRT3 * b) * robot->geom.lp) / denom1;

    // A2 - Theo công thức lý thuyết trang 5 - FIX: đồng nhất dấu với A1, A3
    float denom2 = sqrtf(SQUARE(c) + SQUARE(a));
    robot->A[1].x = (robot->geom.lp * c) / denom2;  
    robot->A[1].y =  0.0f;
    robot->A[1].z = h - ((robot->geom.lp * a) / denom2);

    // A3 - Theo công thức lý thuyết trang 6
    float denom3 = sqrtf(4.0f * SQUARE(c) + SQUARE(a + SQRT3 * b));
    robot->A[2].x = -(robot->geom.lp * c) / denom3;
    robot->A[2].y = -(SQRT3 * robot->geom.lp * c) / denom3;
    robot->A[2].z = h + ((a + SQRT3 * b) * robot->geom.lp) / denom3;
}

// Tinh diem C trung gian (cac diem khop)
void SolveMiddle(RobotState *robot) {
    // Lay diem A
    float a11 = robot->A[0].x, a12 = robot->A[0].y, a13 = robot->A[0].z;
    float a21 = robot->A[1].x, a22 = robot->A[1].y, a23 = robot->A[1].z;
    float a31 = robot->A[2].x, a32 = robot->A[2].y, a33 = robot->A[2].z;

    // C1 - MATCH MIKE.PY FORMULA
    float p1 = (-a11 + SQRT3 * a12 - 2.0f * robot->geom.lb) / a13;
    float q1 = (SQUARE(a11) + SQUARE(a12) + SQUARE(a13) + SQUARE(robot->geom.l2) -
                SQUARE(robot->geom.l1) - SQUARE(robot->geom.lb)) / (2.0f * a13);
    float r1 = SQUARE(p1) + 4.0f;
    float s1 = 2.0f * p1 * q1 + 4.0f * robot->geom.lb;
    float t1 = SQUARE(q1) + SQUARE(robot->geom.lb) - SQUARE(robot->geom.l2);
    float disc1 = SQUARE(s1) - 4.0f * r1 * t1;
    if (disc1 < 0.0f) disc1 = 0.0f;

    float c11 = (-s1 - sqrtf(disc1)) / (2.0f * r1);
    float c12 = -(SQRT3) * c11;
    float c13 = sqrtf(SQUARE(robot->geom.l2) - 4.0f*(SQUARE(c11)) - 4.0f*robot->geom.lb*c11 - SQUARE(robot->geom.lb));  // MATCH PYTHON: sqrt(l2² - 4*(c11²) - 4*lb*c11 - lb²)
    //float c13 = (p1 * (c11)) + q1; //(-s1 - sqrtf(disc1) = c11/ (2.0f * r1)

    robot->C[0].x = c11;
    robot->C[0].y = c12;
    robot->C[0].z = c13;

    // C2 - MATCH MIKE.PY FORMULA
    float p2 = (robot->geom.lb - a21) / a23;
    float q2 = (SQUARE(a21) + SQUARE(a23) - SQUARE(robot->geom.lb) +
                SQUARE(robot->geom.l2) - SQUARE(robot->geom.l1)) / (2.0f * a23);
    float r2 = SQUARE(p2) + 1.0f;
    float s2 = 2.0f * (p2 * q2 - robot->geom.lb);
    float t2 = SQUARE(q2) - SQUARE(robot->geom.l2) + SQUARE(robot->geom.lb);
    float disc2 = SQUARE(s2) - 4.0f * r2 * t2;
    if (disc2 < 0.0f) disc2 = 0.0f;

    float c21 = (-s2 + sqrtf(disc2)) / (2.0f * r2);
    float c22 = 0.0f;
    float c23 = sqrtf(SQUARE(robot->geom.l2) - SQUARE(c21 - robot->geom.lb));  // MATCH PYTHON: sqrt(l2² - (c21 - lb)²)
    //float c23 = (p2 * (-s2 + sqrtf(disc2)) / (2.0f * r2)) + q2;

    robot->C[1].x = c21;
    robot->C[1].y = c22;
    robot->C[1].z = c23;

    // C3 - MATCH MIKE.PY FORMULA
    float p3 = (-a31 - SQRT3 * a32 - 2.0f * robot->geom.lb) / a33;
    float q3 = (SQUARE(a31) + SQUARE(a32) + SQUARE(a33) + SQUARE(robot->geom.l2) -
                SQUARE(robot->geom.l1) - SQUARE(robot->geom.lb)) / (2.0f * a33);
    float r3 = SQUARE(p3) + 4.0f;
    float s3 = 2.0f * p3 * q3 + 4.0f * robot->geom.lb;
    float t3 = SQUARE(q3) + SQUARE(robot->geom.lb) - SQUARE(robot->geom.l2);
    float disc3 = SQUARE(s3) - 4.0f * r3 * t3;
    if (disc3 < 0.0f) disc3 = 0.0f;

    float c31 = (-s3 - sqrtf(disc3)) / (2.0f * r3);
    float c32 = (SQRT3) * c31;
    float c33 = sqrtf(SQUARE(robot->geom.l2) - 4.0f*(SQUARE(c31)) - 4.0f*robot->geom.lb*c31 - SQUARE(robot->geom.lb));  // MATCH PYTHON: sqrt(l2² - 4*(c31²) - 4*lb*c31 - lb²)
    //float c33 = (p3 * c31) + q3; //(-s3 - sqrtf(disc3)) / (2.0f * r3)

    robot->C[2].x = c31;
    robot->C[2].y = c32;
    robot->C[2].z = c33;
}

// Giai dong hoc nguoc tu vector huong (a,b,c)
void SolveInverseKinematicsVector(RobotState *robot, float a, float b, float c, float h) {
    // MODIFIED TO MATCH PYTHON PROJECT BASE POINTS MAPPING
    // Python mapping: B1=[-Lb/2,√3Lb/2,0], B2=[Lb,0,0], B3=[-Lb/2,-√3Lb/2,0]
    robot->B[0].x = -0.5f * robot->geom.lb;            // B₁: -Lb/2
    robot->B[0].y = SQRT3 * 0.5f * robot->geom.lb;     // B₁: √3·Lb/2
    robot->B[0].z = 0.0f;

    robot->B[1].x = robot->geom.lb;                     // B₂: Lb (SWAPPED
    robot->B[1].y = 0.0f;                               // B₂: 0 (SWAPPED
    robot->B[1].z = 0.0f;

    robot->B[2].x = -0.5f * robot->geom.lb;            // B₃: -Lb/2
    robot->B[2].y = -SQRT3 * 0.5f * robot->geom.lb;    // B₃: -√3·Lb/2
    robot->B[2].z = 0.0f;

    // Tinh diem A va diem C
    SolveTop(robot, a, b, c, h);
    SolveMiddle(robot);

    // Tinh goc servo

    //robot->theta[0] = PI/2.0f - atan2f(sqrtf(SQUARE(robot->C[0].x) + SQUARE(robot->C[0].y)) - robot->geom.lb, robot->C[0].z);
    //robot->theta[1] = atan2f(robot->C[1].z, robot->C[1].x - robot->geom.lb);
    //robot->theta[2] = PI/2.0f - atan2f(sqrtf(SQUARE(robot->C[2].x) + SQUARE(robot->C[2].y)) - robot->geom.lb, robot->C[2].z);
    robot->theta[0] = atan2f(robot->C[0].z, sqrtf(SQUARE(robot->C[0].x) + SQUARE(robot->C[0].y)) - robot->geom.lb);
    robot->theta[1] = atan2f(robot->C[1].z, robot->C[1].x - robot->geom.lb);
    robot->theta[2] = atan2f(robot->C[2].z, sqrtf(SQUARE(robot->C[2].x) + SQUARE(robot->C[2].y)) - robot->geom.lb);
}

// Giai dong hoc nguoc tu goc nghieng (theta, phi)
void SolveInverseKinematicsSpherical(RobotState *robot, float theta_deg, float phi_deg, float h) {
    // MATCH MIKE.PY: Update height and compute max theta
    robot->geom.h = h;
    ComputeMaxThetaAtHeight(robot, h);
    
    //Limit theta to maxtheta
    theta_deg = MIN(theta_deg, robot->geom.maxtheta);
    


    float theta = DEG_TO_RAD(theta_deg);
    float phi = DEG_TO_RAD(phi_deg);

    float a = sinf(theta) * cosf(phi); //vecto hướng a
    float b = sinf(theta) * sinf(phi); //vecto hướng b
    float c = cosf(theta); //vecto hướng c

    SolveInverseKinematicsVector(robot, a, b, c, h);
}

// Tinh goc nghieng toi da tai chieu cao h
void ComputeMaxThetaAtHeight(RobotState *robot, float h) {
    float theta_low = 0.0f;
    float theta_high = DEG_TO_RAD(20.0f);
    float tol = 1e-3f;

    // Safety check for height
    if (h <= 0.0f || h > 15.0f) {
        robot->geom.maxtheta = 12.0f;  // Fallback value
        return;
    }


    while (theta_high - theta_low > tol) {
        float theta_mid = (theta_low + theta_high) / 2.0f;
        bool valid = true;
        
        // Test both directions: for s in (1, -1)
        for (int s_sign = 1; s_sign >= -1 && valid; s_sign -= 2) {
        float c = cosf(theta_mid);
            float s = (float)s_sign * sinf(theta_mid);

        float a21 = robot->geom.lp * c;
        float a23 = h - robot->geom.lp * s;
            
            if (a23 <= 0.0f) {
                valid = false;
                break;
            }

        float p2 = (robot->geom.lb - a21) / a23;
        float q2 = (SQUARE(a21) + SQUARE(a23) - SQUARE(robot->geom.lb) +
                    SQUARE(robot->geom.l2) - SQUARE(robot->geom.l1)) / (2.0f * a23);
        float r2 = SQUARE(p2) + 1.0f;
        float s2 = 2.0f * (p2 * q2 - robot->geom.lb);
        float t2 = SQUARE(q2) - SQUARE(robot->geom.l2) + SQUARE(robot->geom.lb);
        float disc = SQUARE(s2) - 4.0f * r2 * t2;

        if (disc < 0.0f) {
                valid = false;
                break;
        }

        float c21 = (-s2 + sqrtf(disc)) / (2.0f * r2);
        float delta = SQUARE(robot->geom.l2) - SQUARE(c21 - robot->geom.lb);

        if (delta < 0.0f) {
                valid = false;
                break;
        }

        float c23 = sqrtf(delta);
        float d1 = sqrtf(SQUARE(a21 - c21) + SQUARE(a23 - c23));
        float d2 = sqrtf(SQUARE(robot->geom.lb - c21) + SQUARE(c23));

            if (fabsf(d1 - robot->geom.l1) > 1e-3f || fabsf(d2 - robot->geom.l2) > 1e-3f) {
                valid = false;
                break;
            }
        }
        
        if (valid) {
            theta_low = theta_mid;
        } else {
            theta_high = theta_mid;
        }
    }

    // Match Python: max(0, math.degrees(round(theta_low, 4)) - 0.5)
    robot->geom.maxtheta = MAX(0.0f, RAD_TO_DEG(theta_low) - 0.5f);  // Match Python safety margin
}
