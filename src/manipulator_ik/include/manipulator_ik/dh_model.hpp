#pragma once
#include <Eigen/Dense>
#include <string>
#include <vector>
#include "manipulator_ik/forward_kinematics.hpp"
#include "manipulator_ik/kinematic_model.hpp"


class DHModel : public KinematicModel
{
    private:
        std::vector<DHMatrix> DHList;
        std::vector<double> JointOffsets;
        std::vector<std::string> JointNames;

        DHModel(const std::vector<DHMatrix>& dh_list,
                const std::vector<double>& joint_offsets,
                const std::vector<std::string>& joint_names);

    public:
        DHModel(const std::vector<DHMatrix>& dh_list);
        ~DHModel() override;

        static DHModel FromYAML(const std::string& path);

        Eigen::Matrix4d ForwardKinematics(const Eigen::VectorXd& q) const override;

        int getDOF() const override;
        double getJointMin(int index) const override;
        double getJointMax(int index) const override;
        bool isJointLimited(int index) const override;
        std::string getJointName(int index) const override;
};
