#include "manipulator_ik/kinematic_model.hpp"
#include <stdexcept>


Eigen::MatrixXd Jacobian_Linear(
    const KinematicModel& model,
    const Eigen::VectorXd& q,
    const double eps)
{
    int nlinks = model.getDOF();
    if (q.size() != nlinks)
    {
        throw std::invalid_argument("Joint configuration size does not match model DOF");
    }

    Eigen::Matrix4d BaselineFK = model.ForwardKinematics(q);
    Eigen::Vector3d BaselinePos = BaselineFK.block<3, 1>(0, 3);
    Eigen::MatrixXd Jacobian(3, nlinks);

    for (int i=0; i<nlinks; i++)
    {
        Eigen::VectorXd NewConfiguration = q;
        NewConfiguration(i) += eps;

        Eigen::Matrix4d NewFK = model.ForwardKinematics(NewConfiguration);
        Eigen::Vector3d NewPos = NewFK.block<3, 1>(0, 3);
        Jacobian.col(i) = (NewPos - BaselinePos) / eps;
    }

    return Jacobian;
}
