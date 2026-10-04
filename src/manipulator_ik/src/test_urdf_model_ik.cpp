#include <iostream>
#include <string>

#include "manipulator_ik/inverse_kinematics.hpp"
#include "manipulator_ik/urdf_model.hpp"


bool CheckConfiguration(
    const URDFModel& model,
    const Eigen::VectorXd& expected_configuration,
    const std::string& name)
{
    const double tolerance = 1e-6;

    Eigen::Vector3d Target =
        model.ForwardKinematics(expected_configuration).block<3, 1>(0, 3);

    Eigen::VectorXd Solution =
        Inverse_Kinematic(Target, model, tolerance, 100);

    Eigen::Vector3d ReachedPosition =
        model.ForwardKinematics(Solution).block<3, 1>(0, 3);

    double position_error = (Target - ReachedPosition).norm();
    bool passed = position_error <= tolerance;

    std::cout << name << ": " << (passed ? "PASSED" : "NOT PASSED")
              << " (position error = " << position_error << ")\n";

    return passed;
}


int main(int argc, char* argv[])
{
    if (argc != 2)
    {
        std::cout << "Usage: test_urdf_model_ik <kinematic_urdf>\n";
        return 1;
    }

    URDFModel Model = URDFModel::FromFile(argv[1], "base_link", "tool0");

    Eigen::VectorXd Configuration1(3);
    Configuration1 << 0.50, 0.30, 0.20;

    Eigen::VectorXd Configuration2(3);
    Configuration2 << -0.60, -0.35, 0.60;

    bool first_passed = CheckConfiguration(Model, Configuration1, "Configuration 1");
    bool second_passed = CheckConfiguration(Model, Configuration2, "Configuration 2");

    return first_passed && second_passed ? 0 : 1;
}
