#include <iostream>
#include <cmath>
#include <vector>

#include "manipulator_ik/forward_kinematics.hpp"

int main()
{
    // TEST 1
    {
        DHParameters params;
        params.alpha = 0.0;
        params.a = 1.0;
        params.theta = 0.0;
        params.d = 0.0;
        params.jtype = JointType::Revolute;

        DHMatrix link(params);
        std::vector<DHMatrix> links = {link};

        Eigen::Matrix4d T = ForwardKinematics(links);

        std::cout << "TEST 1 - theta = 0\n";
        std::cout << T << "\n\n";
    }

    // TEST 2 
    {
        DHParameters params;
        params.alpha = 0.0;
        params.a = 1.0;
        params.theta = M_PI / 2.0;
        params.d = 0.0;
        params.jtype = JointType::Revolute;

        DHMatrix link(params);
        std::vector<DHMatrix> links = {link};

        Eigen::Matrix4d T = ForwardKinematics(links);

        std::cout << "TEST 2 - theta = pi/2\n";
        std::cout << T << "\n";
    }

        // TEST 3 () two planar links)
    {
        DHParameters p1;
        p1.alpha = 0.0;
        p1.a = 1.0;
        p1.theta = M_PI / 2.0;
        p1.d = 0.0;
        p1.jtype = JointType::Revolute;

        DHParameters p2;
        p2.alpha = 0.0;
        p2.a = 1.0;
        p2.theta = 0.0;
        p2.d = 0.0;
        p2.jtype = JointType::Revolute;

        DHMatrix link1(p1);
        DHMatrix link2(p2);

        std::vector<DHMatrix> links = {link1, link2};

        Eigen::Matrix4d T = ForwardKinematics(links);

        std::cout << "TEST 3 - two links\n";
        std::cout << T << "\n\n";
    }

    return 0;
}