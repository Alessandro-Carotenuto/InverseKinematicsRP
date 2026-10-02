#pragma once
#include <Eigen/Dense>
#include <vector>
#include "manipulator_ik/forward_kinematics.hpp"


Eigen::MatrixXd Pseudo_Inverse(Eigen::MatrixXd& Jacobian, double tolerance = 1e-9);
void ApplyJointConfiguration(std::vector<DHMatrix>& DHList, const Eigen::VectorXd& q);
Eigen::VectorXd Inverse_Kinematic(Eigen::Vector3d& Actual_Position, std::vector<DHMatrix>& DHList, const double tol=1e-4, const int max_it=20);