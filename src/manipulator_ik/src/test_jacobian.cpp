#include <iostream>
#include <vector>
#include "manipulator_ik/forward_kinematics.hpp"

int main()
{
    std::cout << "=== NUMERICAL JACOBIAN TESTS ===\n\n";

    {
        JointParameters params;
        params.alpha = 0.0;
        params.a = 1.0;
        params.theta = 0.0;
        params.d = 0.0;
        params.jtype = JointType::Revolute;

        DHMatrix link(params);
        std::vector<DHMatrix> links = {link};

        Eigen::MatrixXd J = Jacobian_Linear(links);

        std::cout << "TEST 1 - Single Revolute Link (theta = 0)\n";
        std::cout << "Jacobian size: " << J.rows() << "x" << J.cols() << "\n";
        std::cout << "Matrix J:\n" << J << "\n\n";
    }

    {
        JointParameters p1;
        p1.alpha = 0.0;
        p1.a = 1.0;
        p1.theta = 0.0;
        p1.d = 0.0;
        p1.jtype = JointType::Revolute;

        JointParameters p2;
        p2.alpha = 0.0;
        p2.a = 1.0;
        p2.theta = 0.0;
        p2.d = 0.0;
        p2.jtype = JointType::Revolute;

        DHMatrix link1(p1);
        DHMatrix link2(p2);

        std::vector<DHMatrix> links = {link1, link2};

        Eigen::MatrixXd J = Jacobian_Linear(links);

        std::cout << "TEST 2 - Two Revolute Links (theta1 = 0, theta2 = 0)\n";
        std::cout << "Jacobian size: " << J.rows() << "x" << J.cols() << "\n";
        std::cout << "Matrix J:\n" << J << "\n\n";
    }

    return 0;
}       