#include <cmath>
#include <iostream>
#include <vector>

#include "manipulator_ik/urdf_model.hpp"


int main(int argc, char* argv[])
{
    if (argc != 3)
    {
        std::cout << "Usage: test_urdf_equivalence <kinematic_urdf> <visual_urdf>\n";
        return 1;
    }

    const double pi = std::acos(-1.0);
    const double tolerance = 1e-9;
    bool all_passed = true;

    URDFModel KinematicModel = URDFModel::FromFile(argv[1], "base_link", "tool0");
    URDFModel VisualModel = URDFModel::FromFile(argv[2], "base_link", "tool0");

    bool metadata_passed = KinematicModel.getDOF() == VisualModel.getDOF();

    if (metadata_passed)
    {
        for (int i=0; i<KinematicModel.getDOF(); i++)
        {
            metadata_passed = metadata_passed &&
                              KinematicModel.isJointLimited(i) == VisualModel.isJointLimited(i) &&
                              std::abs(KinematicModel.getJointMin(i) - VisualModel.getJointMin(i)) <= tolerance &&
                              std::abs(KinematicModel.getJointMax(i) - VisualModel.getJointMax(i)) <= tolerance;
        }
    }

    all_passed = all_passed && metadata_passed;
    std::cout << "Model metadata equivalence: "
              << (metadata_passed ? "PASSED" : "NOT PASSED") << "\n";

    std::vector<Eigen::VectorXd> Configurations;

    Configurations.push_back(Eigen::VectorXd::Zero(3));

    Eigen::VectorXd Configuration1(3);
    Configuration1 << pi / 2.0, 0.0, 0.40;
    Configurations.push_back(Configuration1);

    Eigen::VectorXd Configuration2(3);
    Configuration2 << -1.0, 0.70, 0.80;
    Configurations.push_back(Configuration2);

    for (std::size_t i=0; i<Configurations.size(); i++)
    {
        Eigen::Matrix4d KinematicFK = KinematicModel.ForwardKinematics(Configurations[i]);
        Eigen::Matrix4d VisualFK = VisualModel.ForwardKinematics(Configurations[i]);

        bool configuration_passed = (KinematicFK - VisualFK).norm() <= tolerance;
        all_passed = all_passed && configuration_passed;

        std::cout << "Configuration " << i + 1 << ": "
                  << (configuration_passed ? "PASSED" : "NOT PASSED") << "\n";
    }

    return all_passed ? 0 : 1;
}
