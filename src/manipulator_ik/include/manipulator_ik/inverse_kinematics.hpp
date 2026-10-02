#pragma once
#include <Eigen/Dense>
#include <vector>


Eigen::MatrixXd Pseudo_Inverse(Eigen::MatrixXd& Jacobian, double tolerance = 1e-9);
