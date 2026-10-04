#pragma once
#include <Eigen/Dense>
#include <vector>
#include "manipulator_ik/forward_kinematics.hpp"
#include "manipulator_ik/kinematic_model.hpp"


Eigen::MatrixXd Pseudo_Inverse(Eigen::MatrixXd& Jacobian, double tolerance = 1e-9);
void ApplyJointConfiguration(std::vector<DHMatrix>& DHList, const Eigen::VectorXd& q);

Eigen::VectorXd Inverse_Kinematic(
    const Eigen::Vector3d& Target_Position,
    const KinematicModel& Model,
    const double tol=1e-4,
    const int max_it=20);

Eigen::VectorXd Inverse_Kinematic(
    Eigen::Vector3d& Actual_Position,
    std::vector<DHMatrix>& DHList,
    const double tol=1e-4,
    const int max_it=20);
