#pragma once
#include <Eigen/Dense>
#include <vector>
#include "manipulator_ik/forward_kinematics.hpp"
#include "manipulator_ik/kinematic_model.hpp"


class DHModel : public KinematicModel
{
    private:
        std::vector<DHMatrix> DHList;

    public:
        DHModel(const std::vector<DHMatrix>& dh_list);
        ~DHModel() override;

        Eigen::Matrix4d ForwardKinematics(const Eigen::VectorXd& q) const override;

        int getDOF() const override;
        double getJointMin(int index) const override;
        double getJointMax(int index) const override;
        bool isJointLimited(int index) const override;
};
