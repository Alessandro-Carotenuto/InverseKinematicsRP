#include "manipulator_ik/kinematic_model.hpp"
#include <Eigen/Geometry>
#include <cmath>
#include <stdexcept>

namespace
{
    Eigen::Vector3d RotationVector(const Eigen::Matrix3d& rotation)
    {
        const Eigen::AngleAxisd angle_axis(rotation);
        if (std::abs(angle_axis.angle()) < 1e-12)
        {
            return Eigen::Vector3d::Zero();
        }
        return angle_axis.axis() * angle_axis.angle();
    }
}

Eigen::MatrixXd Jacobian_Linear(
    const KinematicModel& model,
    const Eigen::VectorXd& q,
    const double eps)
{
    const int nlinks = model.getDOF();
    if (q.size() != nlinks)
    {
        throw std::invalid_argument("Joint configuration size does not match model DOF");
    }

    const Eigen::Matrix4d baseline_fk = model.ForwardKinematics(q);
    const Eigen::Vector3d baseline_pos = baseline_fk.block<3, 1>(0, 3);
    Eigen::MatrixXd jacobian(3, nlinks);

    for (int i = 0; i < nlinks; ++i)
    {
        Eigen::VectorXd new_configuration = q;
        new_configuration(i) += eps;

        const Eigen::Matrix4d new_fk = model.ForwardKinematics(new_configuration);
        const Eigen::Vector3d new_pos = new_fk.block<3, 1>(0, 3);
        jacobian.col(i) = (new_pos - baseline_pos) / eps;
    }

    return jacobian;
}

Eigen::MatrixXd Jacobian_Pose(
    const KinematicModel& model,
    const Eigen::VectorXd& q,
    const double eps,
    const double orientation_scale)
{
    const int nlinks = model.getDOF();
    if (q.size() != nlinks)
    {
        throw std::invalid_argument("Joint configuration size does not match model DOF");
    }

    const Eigen::Matrix4d baseline_fk = model.ForwardKinematics(q);
    const Eigen::Vector3d baseline_pos = baseline_fk.block<3, 1>(0, 3);
    const Eigen::Matrix3d baseline_rotation = baseline_fk.block<3, 3>(0, 0);
    Eigen::MatrixXd jacobian(6, nlinks);

    for (int i = 0; i < nlinks; ++i)
    {
        Eigen::VectorXd new_configuration = q;
        new_configuration(i) += eps;

        const Eigen::Matrix4d new_fk = model.ForwardKinematics(new_configuration);
        const Eigen::Vector3d new_pos = new_fk.block<3, 1>(0, 3);
        const Eigen::Matrix3d new_rotation = new_fk.block<3, 3>(0, 0);
        const Eigen::Matrix3d delta_rotation = new_rotation * baseline_rotation.transpose();

        jacobian.block<3, 1>(0, i) = (new_pos - baseline_pos) / eps;
        jacobian.block<3, 1>(3, i) =
            orientation_scale * RotationVector(delta_rotation) / eps;
    }

    return jacobian;
}
