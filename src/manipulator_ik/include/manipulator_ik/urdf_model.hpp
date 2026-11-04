#pragma once
#include <Eigen/Dense>
#include <cstddef>
#include <string>
#include <vector>
#include "manipulator_ik/kinematic_model.hpp"


class URDFModel : public KinematicModel
{
    private:
        enum class JointMotionType
        {
            Revolute,
            Prismatic,
            Fixed
        };

        struct JointData
        {
            std::string name;
            JointMotionType type;
            Eigen::Matrix4d origin;
            Eigen::Vector3d axis;
            double joint_min;
            double joint_max;
            bool is_limited;
            int q_index;
        };

        std::vector<JointData> JointChain;
        std::vector<std::size_t> ActiveJointIndices;
        IKTaskMode TaskMode = IKTaskMode::ONLY_POSITION;

        URDFModel(const std::vector<JointData>& joint_chain, IKTaskMode task_mode);
        const JointData& getActiveJoint(int index) const;

    public:
        static URDFModel FromFile(
            const std::string& path,
            const std::string& base_link,
            const std::string& tool_link,
            IKTaskMode task_mode = IKTaskMode::ONLY_POSITION);

        ~URDFModel() override;

        Eigen::Matrix4d ForwardKinematics(const Eigen::VectorXd& q) const override;

        int getDOF() const override;
        bool isJointRevolute(int index) const override;
        double getJointMin(int index) const override;
        double getJointMax(int index) const override;
        bool isJointLimited(int index) const override;
        std::string getJointName(int index) const override;
        IKTaskMode getIKTaskMode() const override;
};
