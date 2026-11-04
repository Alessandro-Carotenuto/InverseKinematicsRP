#include "manipulator_ik/inverse_kinematics.hpp"
#include "manipulator_ik/dh_model.hpp"
#include "manipulator_ik/forward_kinematics.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>

namespace
{
    constexpr double Pi = 3.14159265358979323846;
    constexpr double TwoPi = 6.28318530717958647692;
}




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


Eigen::VectorXd NaiveSeedGeneratorForIK(
    const Eigen::Vector3d& Target_Position,
    const KinematicModel& Model,
    const double tol,
    const int max_sweeps)
{
    const int joint_count = Model.getDOF();
    Eigen::VectorXd Current_Guess = Eigen::VectorXd::Zero(joint_count);
    Eigen::VectorXd Step_Size = Eigen::VectorXd::Zero(joint_count);

    if (joint_count == 0)
    {
        return Current_Guess;
    }

    for (int i = 0; i < joint_count; ++i)
    {
        if (Model.isJointLimited(i))
        {
            const double Joint_Min = Model.getJointMin(i);
            const double Joint_Max = Model.getJointMax(i);
            Current_Guess(i) = (Joint_Min + Joint_Max) / 2.0;
            // Start by testing one quarter of the full range in either direction.
            Step_Size(i) = (Joint_Max - Joint_Min) / 4.0;
        }
        else if (Model.isJointRevolute(i))
        {
            Step_Size(i) = Pi / 2.0;
        }
    }

    double Current_Error =
        (Target_Position - Model.ForwardKinematics(Current_Guess).block<3, 1>(0, 3)).norm();
    if (Current_Error < tol)
    {
        return Current_Guess;
    }

    // An unlimited prismatic joint has no model range to scale its trial step from.
    const double Default_Prismatic_Step = std::max(0.01, Current_Error / 4.0);
    for (int i = 0; i < joint_count; ++i)
    {
        if (!Model.isJointLimited(i) && !Model.isJointRevolute(i))
        {
            Step_Size(i) = Default_Prismatic_Step;
        }
    }

    constexpr double Minimum_Step = 1e-4;
    constexpr double Improvement_Epsilon = 1e-12;

    // Try both directions for one joint while holding the others fixed.
    // If an entire sweep makes no progress, halve all step sizes and retry.
    for (int sweep = 0; sweep < max_sweeps; ++sweep)
    {
        bool Improved_In_Sweep = false;

        for (int i = 0; i < joint_count; ++i)
        {
            Eigen::VectorXd Best_Candidate = Current_Guess;
            double Best_Error = Current_Error;

            for (int direction_index = 0; direction_index < 2; ++direction_index)
            {
                const double direction = direction_index == 0 ? -1.0 : 1.0;
                Eigen::VectorXd Candidate = Current_Guess;
                Candidate(i) += direction * Step_Size(i);

                if (Model.isJointRevolute(i))
                {
                    Candidate(i) = std::remainder(Candidate(i), TwoPi);
                }

                if (Model.isJointLimited(i) &&
                    (Candidate(i) < Model.getJointMin(i) || Candidate(i) > Model.getJointMax(i)))
                {
                    continue;
                }

                const Eigen::Vector3d Candidate_Position =
                    Model.ForwardKinematics(Candidate).block<3, 1>(0, 3);
                const double Candidate_Error = (Target_Position - Candidate_Position).norm();

                if (Candidate_Error + Improvement_Epsilon < Best_Error)
                {
                    Best_Error = Candidate_Error;
                    Best_Candidate = Candidate;
                }
            }

            if (Best_Error + Improvement_Epsilon < Current_Error)
            {
                Current_Guess = Best_Candidate;
                Current_Error = Best_Error;
                Improved_In_Sweep = true;

                if (Current_Error < tol)
                {
                    return Current_Guess;
                }
            }
        }

        if (!Improved_In_Sweep)
        {
            Step_Size *= 0.5;
            if (Step_Size.maxCoeff() < Minimum_Step)
            {
                break;
            }
        }
    }

    return Current_Guess;
}

//Helper Function
void ApplyJointConfiguration(std::vector<DHMatrix>& DHList, const Eigen::VectorXd& q)
{

    //Set up the DHList to q_guess to compute FK

    for (std::size_t i = 0; i < DHList.size(); ++i)
    {
        switch (DHList[i].getJointType())
        {
            case JointType::Revolute:
            {
                DHList[i].setTheta(q(static_cast<Eigen::Index>(i)));
                break;
            }
            case JointType::Prismatic:
            {
                DHList[i].setD(q(static_cast<Eigen::Index>(i)));
                break;
            }
        }
    }
}

Eigen::VectorXd Inverse_Kinematic(
    const Eigen::Vector3d& Target_Position,
    const KinematicModel& Model,
    bool& Converged,
    const double tol,
    const int max_it)
{
    int nlinks = Model.getDOF();

    Eigen::VectorXd Current_Guess = NaiveSeedGeneratorForIK(Target_Position, Model, tol);

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
        for (int i=0; i<nlinks; i++)
        {
            if (Model.isJointRevolute(i))
            {
                Current_Guess(i) = std::remainder(Current_Guess(i), TwoPi);
            }
            else if (Model.isJointLimited(i))
            {
                if (Current_Guess(i) < Model.getJointMin(i))
                {
                    Current_Guess(i) = Model.getJointMin(i);
                }
                else if (Current_Guess(i) > Model.getJointMax(i))
                {
                    Current_Guess(i) = Model.getJointMax(i);
                }
            }
        }
        iterations++;
    } while (iterations<max_it);

    CurrentGuessFK = Model.ForwardKinematics(Current_Guess);
    CurrentGuessPOS = CurrentGuessFK.block<3, 1>(0, 3);
    error = Target_Position - CurrentGuessPOS;
    Converged = error.norm() < tol;

    return Current_Guess;
}

Eigen::VectorXd Inverse_Kinematic(
    Eigen::Vector3d& Actual_Position,
    std::vector<DHMatrix>& DHList,
    bool& Converged,
    const double tol,
    const int max_it)
{
    // Preserve the old DHList API by adapting it to the generic model interface.
    DHModel Model(DHList);
    Eigen::VectorXd Result = Inverse_Kinematic(Actual_Position, Model, Converged, tol, max_it);

    ApplyJointConfiguration(DHList, Result);
    return Result;
}
