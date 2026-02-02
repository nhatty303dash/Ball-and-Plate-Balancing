#ifndef INVERSE_KINEMATICS_H
#define INVERSE_KINEMATICS_H

#include "robot_geometry.h"

// Nguyen mau cac ham dong hoc nguoc
void SolveTop(RobotState *robot, float a, float b, float c, float h);
void SolveMiddle(RobotState *robot);
void SolveInverseKinematicsVector(RobotState *robot, float a, float b, float c, float h);
void SolveInverseKinematicsSpherical(RobotState *robot, float theta_deg, float phi_deg, float h);
void ComputeMaxThetaAtHeight(RobotState *robot, float h);

#endif /* INVERSE_KINEMATICS_H */
