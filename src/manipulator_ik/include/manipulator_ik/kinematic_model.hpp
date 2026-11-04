#pragma once
#include <Eigen/Dense>
#include <string>

enum class IKTaskMode
{
    ONLY_POSITION,
    FULL_POSE
};


class KinematicModel
{
    public:
        virtual ~KinematicModel() = default;

        virtual Eigen::Matrix4d ForwardKinematics(const Eigen::VectorXd& q) const = 0;

        virtual int getDOF() const = 0;
        virtual bool isJointRevolute(int index) const = 0;
        virtual double getJointMin(int index) const = 0;
        virtual double getJointMax(int index) const = 0;
        virtual bool isJointLimited(int index) const = 0;
        virtual std::string getJointName(int index) const = 0;
        virtual IKTaskMode getIKTaskMode() const = 0;
};


Eigen::MatrixXd Jacobian_Linear(
    const KinematicModel& model,
    const Eigen::VectorXd& q,
    const double eps = 1e-6);

Eigen::MatrixXd Jacobian_Pose(
    const KinematicModel& model,
    const Eigen::VectorXd& q,
    const double eps = 1e-6,
    const double orientation_scale = 0.25);
