#include <cmath>
#include <iostream>
#include <vector>

#include "manipulator_ik/dh_model.hpp"
#include "manipulator_ik/forward_kinematics.hpp"
#include "manipulator_ik/kinematic_model.hpp"


int main()
{
    const double pi = std::acos(-1.0);
    const double tolerance = 1e-9;

    JointParameters p1{};
    p1.alpha = 0.0;
    p1.a = 1.0;
    p1.theta = 0.0;
    p1.d = 0.0;
    p1.jtype = JointType::Revolute;
    p1.joint_min = -pi;
    p1.joint_max = pi;
    p1.isLimited = true;

    JointParameters p2{};
    p2.alpha = 0.0;
    p2.a = 0.0;
    p2.theta = 0.0;
    p2.d = 0.0;
    p2.jtype = JointType::Prismatic;
    p2.joint_min = 0.0;
    p2.joint_max = 1.0;
    p2.isLimited = true;

    std::vector<DHMatrix> DHList = {DHMatrix(p1), DHMatrix(p2)};
    DHModel model(DHList);

    Eigen::VectorXd q(2);
    q << pi / 3.0, 0.4;

    std::vector<DHMatrix> ConfiguredDHList = DHList;
    ConfiguredDHList[0].setTheta(q(0));
    ConfiguredDHList[1].setD(q(1));

    Eigen::Matrix4d ExpectedFK = ::ForwardKinematics(ConfiguredDHList);
    Eigen::Matrix4d ModelFK = model.ForwardKinematics(q);

    Eigen::MatrixXd ExpectedJacobian = Jacobian_Linear(ConfiguredDHList);
    Eigen::MatrixXd ModelJacobian = Jacobian_Linear(model, q);

    bool fk_passed = (ExpectedFK - ModelFK).norm() <= tolerance;
    bool jacobian_passed = (ExpectedJacobian - ModelJacobian).norm() <= tolerance;
    bool metadata_passed = model.getDOF() == 2 &&
                           model.isJointLimited(0) &&
                           model.isJointLimited(1) &&
                           std::abs(model.getJointMin(0) + pi) <= tolerance &&
                           std::abs(model.getJointMax(1) - 1.0) <= tolerance;

    std::cout << "FK equivalence: " << (fk_passed ? "PASSED" : "NOT PASSED") << "\n";
    std::cout << "Jacobian equivalence: " << (jacobian_passed ? "PASSED" : "NOT PASSED") << "\n";
    std::cout << "Model metadata: " << (metadata_passed ? "PASSED" : "NOT PASSED") << "\n";

    return fk_passed && jacobian_passed && metadata_passed ? 0 : 1;
}
