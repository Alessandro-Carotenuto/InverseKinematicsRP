#include <cmath>
#include <iostream>
#include <stdexcept>

#include "manipulator_ik/urdf_model.hpp"


int main(int argc, char* argv[])
{
    if (argc != 2)
    {
        std::cout << "Usage: test_urdf_model <kinematic_urdf>\n";
        return 1;
    }

    const double pi = std::acos(-1.0);
    const double tolerance = 1e-9;
    bool all_passed = true;

    URDFModel Model = URDFModel::FromFile(argv[1], "base_link", "tool0");

    bool metadata_passed = Model.getDOF() == 3 &&
                           Model.isJointLimited(0) &&
                           Model.isJointLimited(1) &&
                           Model.isJointLimited(2) &&
                           std::abs(Model.getJointMin(0) + pi) <= tolerance &&
                           std::abs(Model.getJointMax(1) - 1.20) <= tolerance &&
                           std::abs(Model.getJointMin(2)) <= tolerance &&
                           std::abs(Model.getJointMax(2) - 0.80) <= tolerance &&
                           Model.getJointName(0) == "joint_1" &&
                           Model.getJointName(1) == "joint_2" &&
                           Model.getJointName(2) == "joint_3";

    all_passed = all_passed && metadata_passed;
    std::cout << "Model metadata: " << (metadata_passed ? "PASSED" : "NOT PASSED") << "\n";

    Eigen::VectorXd ZeroConfiguration = Eigen::VectorXd::Zero(3);
    Eigen::Matrix4d ZeroResult = Model.ForwardKinematics(ZeroConfiguration);
    Eigen::Matrix4d ZeroExpected = Eigen::Matrix4d::Identity();
    ZeroExpected(0, 3) = 1.0;
    ZeroExpected(2, 3) = 1.0;

    bool zero_passed = (ZeroResult - ZeroExpected).norm() <= tolerance;
    all_passed = all_passed && zero_passed;
    std::cout << "Zero configuration FK: " << (zero_passed ? "PASSED" : "NOT PASSED") << "\n";

    Eigen::VectorXd YawConfiguration(3);
    YawConfiguration << pi / 2.0, 0.0, 0.40;

    Eigen::Matrix4d YawResult = Model.ForwardKinematics(YawConfiguration);
    Eigen::Matrix4d YawExpected;
    YawExpected << 0.0, -1.0, 0.0, 0.0,
                   1.0,  0.0, 0.0, 1.4,
                   0.0,  0.0, 1.0, 1.0,
                   0.0,  0.0, 0.0, 1.0;

    bool yaw_passed = (YawResult - YawExpected).norm() <= tolerance;
    all_passed = all_passed && yaw_passed;
    std::cout << "Revolute and prismatic FK: " << (yaw_passed ? "PASSED" : "NOT PASSED") << "\n";

    Eigen::VectorXd PitchConfiguration(3);
    PitchConfiguration << 0.0, pi / 2.0, 0.0;

    Eigen::Matrix4d PitchResult = Model.ForwardKinematics(PitchConfiguration);
    Eigen::Matrix4d PitchExpected;
    PitchExpected << 0.0, 0.0, -1.0, 0.0,
                     0.0, 1.0,  0.0, 0.0,
                     1.0, 0.0,  0.0, 2.0,
                     0.0, 0.0,  0.0, 1.0;

    bool pitch_passed = (PitchResult - PitchExpected).norm() <= tolerance;
    all_passed = all_passed && pitch_passed;
    std::cout << "Arbitrary axis FK: " << (pitch_passed ? "PASSED" : "NOT PASSED") << "\n";

    bool size_check_passed = false;
    try
    {
        Model.ForwardKinematics(Eigen::VectorXd::Zero(2));
    }
    catch (const std::invalid_argument&)
    {
        size_check_passed = true;
    }

    all_passed = all_passed && size_check_passed;
    std::cout << "Configuration size check: " << (size_check_passed ? "PASSED" : "NOT PASSED") << "\n";

    return all_passed ? 0 : 1;
}
