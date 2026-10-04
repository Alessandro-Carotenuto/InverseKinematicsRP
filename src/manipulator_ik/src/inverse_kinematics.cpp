#include "manipulator_ik/inverse_kinematics.hpp"
#include "manipulator_ik/dh_model.hpp"
#include "manipulator_ik/forward_kinematics.hpp"



//PSEUDO INVERSE VIA SVD
Eigen::MatrixXd Pseudo_Inverse(Eigen::MatrixXd& Jacobian, double tolerance)
{
    Eigen::JacobiSVD<Eigen::MatrixXd> svd = Jacobian.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV);
    Eigen::VectorXd inverseSingularValues = svd.singularValues(); //initializing them as the singular val

    //INVERT SINGUALR VALUES

    for (int i =0; i<inverseSingularValues.size(); i++)
    {
        if (inverseSingularValues[i]<=tolerance)
        {
            inverseSingularValues[i]=0;
        }
        else
        {
            inverseSingularValues[i]=1/inverseSingularValues[i];
        }
    }

    Eigen::MatrixXd V = svd.matrixV();
    Eigen::MatrixXd U = svd.matrixU();
    Eigen::MatrixXd pinvSV = inverseSingularValues.asDiagonal();

    return V*pinvSV*U.transpose();

}

//Helper Function
void ApplyJointConfiguration(std::vector<DHMatrix>& DHList, const Eigen::VectorXd& q)
{

    //Set up the DHList to q_guess to compute FK

    for (int i=0; i<DHList.size(); i++)
    {
        switch (DHList[i].getJointType())
        {
            case JointType::Revolute:
            {
                DHList[i].setTheta(q(i));
                break;
            }
            case JointType::Prismatic:
            {
                DHList[i].setD(q(i));
                break;
            }
        }
    }
}

Eigen::VectorXd Inverse_Kinematic(
    const Eigen::Vector3d& Target_Position,
    const KinematicModel& Model,
    const double tol,
    const int max_it)
{
    int nlinks = Model.getDOF();

    // Keep the previous initial-guess policy: midpoint for limited joints, zero otherwise.
    Eigen::VectorXd Current_Guess = Eigen::VectorXd::Zero(nlinks);
    for (int i=0; i<nlinks; i++)
    {
        if (Model.isJointLimited(i))
        {
            Current_Guess(i)=(Model.getJointMax(i)+Model.getJointMin(i))/2;
        }
    }

    int iterations=0;
    Eigen::Matrix4d CurrentGuessFK;
    Eigen::MatrixXd J;

    Eigen::Vector3d error, CurrentGuessPOS;
    Eigen::VectorXd delta_q;

    do
    {
        // KinematicModel dispatches FK to DHModel or URDFModel without changing the solver.
        CurrentGuessFK = Model.ForwardKinematics(Current_Guess);
        CurrentGuessPOS = CurrentGuessFK.block<3, 1>(0, 3);
        error = Target_Position - CurrentGuessPOS;

        if (error.norm() < tol)
        {
            break;
        }

        J = Jacobian_Linear(Model, Current_Guess);
        delta_q = Pseudo_Inverse(J)*error;
        Current_Guess = Current_Guess+delta_q;
        iterations++;
    } while (iterations<max_it);

    return Current_Guess;
}

Eigen::VectorXd Inverse_Kinematic(
    Eigen::Vector3d& Actual_Position,
    std::vector<DHMatrix>& DHList,
    const double tol,
    const int max_it)
{
    // Preserve the old DHList API by adapting it to the generic model interface.
    DHModel Model(DHList);
    Eigen::VectorXd Result = Inverse_Kinematic(Actual_Position, Model, tol, max_it);

    ApplyJointConfiguration(DHList, Result);
    return Result;
}
