#pragma once
#include <Eigen/Dense>


class KinematicModel
{
    public:
        virtual ~KinematicModel() = default;

        virtual Eigen::Matrix4d ForwardKinematics(const Eigen::VectorXd& q) const = 0;

        virtual int getDOF() const = 0;
        virtual double getJointMin(int index) const = 0;
        virtual double getJointMax(int index) const = 0;
        virtual bool isJointLimited(int index) const = 0;
};


Eigen::MatrixXd Jacobian_Linear(
    const KinematicModel& model,
    const Eigen::VectorXd& q,
    const double eps = 1e-6);
