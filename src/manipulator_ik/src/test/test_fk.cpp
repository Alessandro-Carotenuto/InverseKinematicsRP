#include <cmath>
#include <iostream>
#include <vector>

#include "manipulator_ik/forward_kinematics.hpp"

int main()
{
    const double pi = std::acos(-1.0);
    const double tolerance = 1e-9;
    bool all_passed = true;

    // TEST 1
    {
        JointParameters params{};
        params.alpha = 0.0;
        params.a = 1.0;
        params.theta = 0.0;
        params.d = 0.0;
        params.jtype = JointType::Revolute;

        DHMatrix link(params);
        std::vector<DHMatrix> links = {link};

        Eigen::Matrix4d result = ForwardKinematics(links);
        Eigen::Matrix4d expected;
        expected << 1.0, 0.0, 0.0, 1.0,
                    0.0, 1.0, 0.0, 0.0,
                    0.0, 0.0, 1.0, 0.0,
                    0.0, 0.0, 0.0, 1.0;

        double error = (result - expected).norm();
        bool passed = error <= tolerance;
        all_passed = all_passed && passed;

        std::cout << "TEST 1 - theta = 0\n";
        std::cout << "Result:\n" << result << "\n\n";
        std::cout << "Expected:\n" << expected << "\n\n";
        std::cout << "Error: " << error << "\n";
        std::cout << (passed ? "PASSED" : "NOT PASSED") << "\n\n";
    }

    // TEST 2
    {
        JointParameters params{};
        params.alpha = 0.0;
        params.a = 1.0;
        params.theta = pi / 2.0;
        params.d = 0.0;
        params.jtype = JointType::Revolute;

        DHMatrix link(params);
        std::vector<DHMatrix> links = {link};

        Eigen::Matrix4d result = ForwardKinematics(links);
        Eigen::Matrix4d expected;
        expected << 0.0, -1.0, 0.0, 0.0,
                    1.0,  0.0, 0.0, 1.0,
                    0.0,  0.0, 1.0, 0.0,
                    0.0,  0.0, 0.0, 1.0;

        double error = (result - expected).norm();
        bool passed = error <= tolerance;
        all_passed = all_passed && passed;

        std::cout << "TEST 2 - theta = pi/2\n";
        std::cout << "Result:\n" << result << "\n\n";
        std::cout << "Expected:\n" << expected << "\n\n";
        std::cout << "Error: " << error << "\n";
        std::cout << (passed ? "PASSED" : "NOT PASSED") << "\n\n";
    }

    // TEST 3
    {
        JointParameters p1{};
        p1.alpha = 0.0;
        p1.a = 1.0;
        p1.theta = pi / 2.0;
        p1.d = 0.0;
        p1.jtype = JointType::Revolute;

        JointParameters p2{};
        p2.alpha = 0.0;
        p2.a = 1.0;
        p2.theta = 0.0;
        p2.d = 0.0;
        p2.jtype = JointType::Revolute;

        DHMatrix link1(p1);
        DHMatrix link2(p2);
        std::vector<DHMatrix> links = {link1, link2};

        Eigen::Matrix4d result = ForwardKinematics(links);
        Eigen::Matrix4d expected;
        expected << 0.0, -1.0, 0.0, 0.0,
                    1.0,  0.0, 0.0, 2.0,
                    0.0,  0.0, 1.0, 0.0,
                    0.0,  0.0, 0.0, 1.0;

        double error = (result - expected).norm();
        bool passed = error <= tolerance;
        all_passed = all_passed && passed;

        std::cout << "TEST 3 - two planar links\n";
        std::cout << "Result:\n" << result << "\n\n";
        std::cout << "Expected:\n" << expected << "\n\n";
        std::cout << "Error: " << error << "\n";
        std::cout << (passed ? "PASSED" : "NOT PASSED") << "\n\n";
    }

    return all_passed ? 0 : 1;
}
