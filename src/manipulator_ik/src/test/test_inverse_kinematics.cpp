#include <cmath>
#include <iostream>
#include <vector>

#include "manipulator_ik/forward_kinematics.hpp"
#include "manipulator_ik/inverse_kinematics.hpp"

int main()
{
    const double pi = std::acos(-1.0);
    const double position_tolerance = 1e-6;
    const double solution_tolerance = 1e-5;
    bool all_passed = true;

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

    // TEST 1
    {
        DHMatrix link1(p1);
        DHMatrix link2(p2);
        std::vector<DHMatrix> links = {link1, link2};

        Eigen::Vector3d desired_position;
        desired_position << std::sqrt(3.0) / 2.0, 1.5, 0.0;

        Eigen::VectorXd expected_solution(2);
        expected_solution << pi / 6.0, pi / 3.0;

        Eigen::VectorXd solution =
            Inverse_Kinematic(desired_position, links, position_tolerance, 100);

        ApplyJointConfiguration(links, solution);
        Eigen::Vector3d reached_position =
            ForwardKinematics(links).block<3, 1>(0, 3);

        double position_error = (desired_position - reached_position).norm();
        double solution_error = (expected_solution - solution).norm();
        bool passed = position_error <= position_tolerance &&
                      solution_error <= solution_tolerance;
        all_passed = all_passed && passed;

        std::cout << "TEST 1 - q = [pi/6, pi/3]\n";
        std::cout << "Desired position:\n" << desired_position << "\n\n";
        std::cout << "Joint solution:\n" << solution << "\n\n";
        std::cout << "Expected solution:\n" << expected_solution << "\n\n";
        std::cout << "Reached position:\n" << reached_position << "\n\n";
        std::cout << "Position error: " << position_error << "\n";
        std::cout << "Solution error: " << solution_error << "\n";
        std::cout << (passed ? "PASSED" : "NOT PASSED") << "\n\n";
    }

    // TEST 2
    {
        DHMatrix link1(p1);
        DHMatrix link2(p2);
        std::vector<DHMatrix> links = {link1, link2};

        Eigen::Vector3d desired_position;
        desired_position << 0.5, 1.0 + std::sqrt(3.0) / 2.0, 0.0;

        Eigen::VectorXd expected_solution(2);
        expected_solution << pi / 3.0, pi / 6.0;

        Eigen::VectorXd solution =
            Inverse_Kinematic(desired_position, links, position_tolerance, 100);

        ApplyJointConfiguration(links, solution);
        Eigen::Vector3d reached_position =
            ForwardKinematics(links).block<3, 1>(0, 3);

        double position_error = (desired_position - reached_position).norm();
        double solution_error = (expected_solution - solution).norm();
        bool passed = position_error <= position_tolerance &&
                      solution_error <= solution_tolerance;
        all_passed = all_passed && passed;

        std::cout << "TEST 2 - q = [pi/3, pi/6]\n";
        std::cout << "Desired position:\n" << desired_position << "\n\n";
        std::cout << "Joint solution:\n" << solution << "\n\n";
        std::cout << "Expected solution:\n" << expected_solution << "\n\n";
        std::cout << "Reached position:\n" << reached_position << "\n\n";
        std::cout << "Position error: " << position_error << "\n";
        std::cout << "Solution error: " << solution_error << "\n";
        std::cout << (passed ? "PASSED" : "NOT PASSED") << "\n\n";
    }

    return all_passed ? 0 : 1;
}
