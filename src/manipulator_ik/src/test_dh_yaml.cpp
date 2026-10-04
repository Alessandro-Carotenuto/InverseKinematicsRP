#include <cmath>
#include <iostream>

#include "manipulator_ik/dh_model.hpp"


int main(int argc, char* argv[])
{
    if (argc != 2)
    {
        std::cerr << "Usage: test_dh_yaml <yaml_path>\n";
        return 1;
    }

    const double Tolerance = 1e-9;
    DHModel Model = DHModel::FromYAML(argv[1]);

    Eigen::VectorXd q(2);
    q << 0.4, 0.2;

    const double Angle = 0.2 + q(0);
    Eigen::Matrix4d Expected;
    Expected << std::cos(Angle), -std::sin(Angle), 0.0, std::cos(Angle),
                std::sin(Angle),  std::cos(Angle), 0.0, std::sin(Angle),
                0.0,              0.0,             1.0, 0.3 + q(1),
                0.0,              0.0,             0.0, 1.0;

    const Eigen::Matrix4d Actual = Model.ForwardKinematics(q);

    const bool MetadataPassed = Model.getDOF() == 2 &&
                                Model.isJointLimited(0) &&
                                Model.isJointLimited(1) &&
                                std::abs(Model.getJointMin(0) + 1.0) <= Tolerance &&
                                std::abs(Model.getJointMax(0) - 1.0) <= Tolerance &&
                                std::abs(Model.getJointMin(1)) <= Tolerance &&
                                std::abs(Model.getJointMax(1) - 0.8) <= Tolerance &&
                                Model.getJointName(0) == "base_rotation" &&
                                Model.getJointName(1) == "vertical_slide";
    const bool FKPassed = (Expected - Actual).norm() <= Tolerance;

    std::cout << "YAML metadata: " << (MetadataPassed ? "PASSED" : "NOT PASSED") << "\n";
    std::cout << "YAML FK and offsets: " << (FKPassed ? "PASSED" : "NOT PASSED") << "\n";

    return MetadataPassed && FKPassed ? 0 : 1;
}
