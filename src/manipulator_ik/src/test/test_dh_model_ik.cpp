#include <cmath>
#include <iostream>
#include <vector>

#include "manipulator_ik/dh_model.hpp"
#include "manipulator_ik/inverse_kinematics.hpp"


bool CheckConfiguration(
    const DHModel& model,
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


int main()
{
    const double pi = std::acos(-1.0);

    JointParameters p1{};
    p1.alpha = 0.0;
    p1.a = 1.0;
    p1.theta = 0.0;
    p1.d = 0.0;
    p1.jtype = JointType::Revolute;
    p1.joint_min = -pi / 2.0;
    p1.joint_max = pi / 2.0;
    p1.isLimited = true;

    JointParameters p2{};
    p2.alpha = 0.0;
    p2.a = 1.0;
    p2.theta = 0.0;
    p2.d = 0.0;
    p2.jtype = JointType::Revolute;
    p2.joint_min = 0.0;
    p2.joint_max = pi;
    p2.isLimited = true;

    std::vector<DHMatrix> DHList = {DHMatrix(p1), DHMatrix(p2)};
    DHModel Model(DHList);

    Eigen::VectorXd Configuration1(2);
    Configuration1 << pi / 6.0, pi / 3.0;

    Eigen::VectorXd Configuration2(2);
    Configuration2 << pi / 3.0, pi / 6.0;

    bool first_passed = CheckConfiguration(Model, Configuration1, "Configuration 1");
    bool second_passed = CheckConfiguration(Model, Configuration2, "Configuration 2");

    return first_passed && second_passed ? 0 : 1;
}
