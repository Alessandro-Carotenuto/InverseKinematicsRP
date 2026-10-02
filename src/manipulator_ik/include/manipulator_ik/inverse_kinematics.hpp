#pragma once
#include <Eigen/Dense>

Eigen::MatrixXd Pseudo_Inverse(Eigen::MatrixXd& Jacobian, double tolerance = 1e-9);
