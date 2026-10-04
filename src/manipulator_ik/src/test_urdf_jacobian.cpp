#include <cmath>
#include <iostream>

#include "manipulator_ik/kinematic_model.hpp"
#include "manipulator_ik/urdf_model.hpp"


Eigen::Matrix3d AnalyticalJacobian(const Eigen::VectorXd& q)
{
    double theta_1 = q(0);
    double theta_2 = q(1);
    double arm_length = 1.0 + q(2);

    double cos_1 = std::cos(theta_1);
    double sin_1 = std::sin(theta_1);
    double cos_2 = std::cos(theta_2);
    double sin_2 = std::sin(theta_2);

    Eigen::Matrix3d Jacobian;
    Jacobian << -arm_length * sin_1 * cos_2,
                -arm_length * cos_1 * sin_2,
                 cos_1 * cos_2,
                 arm_length * cos_1 * cos_2,
                -arm_length * sin_1 * sin_2,
                 sin_1 * cos_2,
                 0.0,
                 arm_length * cos_2,
                 sin_2;

    return Jacobian;
}


bool CheckConfiguration(
    const URDFModel& model,
    const Eigen::VectorXd& q,
    const std::string& name)
{
    const double numerical_step = 1e-7;
    const double tolerance = 1e-6;

    Eigen::MatrixXd Numerical = Jacobian_Linear(model, q, numerical_step);
    Eigen::Matrix3d Expected = AnalyticalJacobian(q);
    double error = (Numerical - Expected).norm();
    bool passed = error <= tolerance;

    std::cout << name << ": " << (passed ? "PASSED" : "NOT PASSED")
              << " (error = " << error << ")\n";

    return passed;
}


int main(int argc, char* argv[])
{
    if (argc != 2)
    {
        std::cout << "Usage: test_urdf_jacobian <kinematic_urdf>\n";
        return 1;
    }

    URDFModel Model = URDFModel::FromFile(argv[1], "base_link", "tool0");

    Eigen::VectorXd ZeroConfiguration = Eigen::VectorXd::Zero(3);

    Eigen::VectorXd GeneralConfiguration(3);
    GeneralConfiguration << -0.80, 0.60, 0.35;

    bool zero_passed = CheckConfiguration(Model, ZeroConfiguration, "Zero configuration");
    bool general_passed = CheckConfiguration(Model, GeneralConfiguration, "General configuration");

    return zero_passed && general_passed ? 0 : 1;
}
