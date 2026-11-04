#include "manipulator_ik/inverse_kinematics.hpp"
#include "manipulator_ik/dh_model.hpp"
#include "manipulator_ik/forward_kinematics.hpp"

#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <cstddef>

namespace
{
    constexpr double Pi = 3.14159265358979323846;
    constexpr double TwoPi = 6.28318530717958647692;
    constexpr double OrientationLengthScale = 0.25;
    constexpr double OrientationTolerance = 1e-3;

    Eigen::Vector3d RotationVectorError(
        const Eigen::Matrix3d& Target_Rotation,
        const Eigen::Matrix3d& Current_Rotation)
    {
        const Eigen::Matrix3d Delta_Rotation = Target_Rotation * Current_Rotation.transpose();
        const Eigen::AngleAxisd Angle_Axis(Delta_Rotation);
        if (std::abs(Angle_Axis.angle()) < 1e-12)
        {
            return Eigen::Vector3d::Zero();
        }
        return Angle_Axis.axis() * Angle_Axis.angle();
    }

    double TaskErrorMagnitude(
        const Eigen::Matrix4d& Target_Pose,
        const Eigen::Matrix4d& Current_Pose,
        const IKTaskMode Task_Mode)
    {
        const Eigen::Vector3d Position_Error =
            Target_Pose.block<3, 1>(0, 3) - Current_Pose.block<3, 1>(0, 3);
        if (Task_Mode == IKTaskMode::ONLY_POSITION)
        {
            return Position_Error.norm();
        }

        const Eigen::Vector3d Orientation_Error = RotationVectorError(
            Target_Pose.block<3, 3>(0, 0), Current_Pose.block<3, 3>(0, 0));
        return std::sqrt(
            Position_Error.squaredNorm() +
            OrientationLengthScale * OrientationLengthScale * Orientation_Error.squaredNorm());
    }

    bool TaskConverged(
        const Eigen::Matrix4d& Target_Pose,
        const Eigen::Matrix4d& Current_Pose,
        const IKTaskMode Task_Mode,
        const double Position_Tolerance)
    {
        const Eigen::Vector3d Position_Error =
            Target_Pose.block<3, 1>(0, 3) - Current_Pose.block<3, 1>(0, 3);
        if (Position_Error.norm() >= Position_Tolerance)
        {
            return false;
        }

        return Task_Mode == IKTaskMode::ONLY_POSITION ||
            RotationVectorError(
                Target_Pose.block<3, 3>(0, 0), Current_Pose.block<3, 3>(0, 0)).norm() <
                OrientationTolerance;
    }
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
    const Eigen::Matrix4d& Target_Pose,
    const KinematicModel& Model,
    const double tol,
    const int max_sweeps)
{
    const IKTaskMode task_mode = Model.getIKTaskMode();
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

    Eigen::Matrix4d Current_Pose = Model.ForwardKinematics(Current_Guess);
    double Current_Error = TaskErrorMagnitude(Target_Pose, Current_Pose, task_mode);
    if (TaskConverged(Target_Pose, Current_Pose, task_mode, tol))
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

                const Eigen::Matrix4d Candidate_Pose = Model.ForwardKinematics(Candidate);
                const double Candidate_Error =
                    TaskErrorMagnitude(Target_Pose, Candidate_Pose, task_mode);

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
                Current_Pose = Model.ForwardKinematics(Current_Guess);
                Improved_In_Sweep = true;

                if (TaskConverged(Target_Pose, Current_Pose, task_mode, tol))
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

Eigen::VectorXd NaiveSeedGeneratorForIK(
    const Eigen::Vector3d& Target_Position,
    const KinematicModel& Model,
    const double tol,
    const int max_sweeps)
{
    Eigen::Matrix4d Target_Pose = Eigen::Matrix4d::Identity();
    Target_Pose.block<3, 1>(0, 3) = Target_Position;
    return NaiveSeedGeneratorForIK(Target_Pose, Model, tol, max_sweeps);
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

namespace
{
    struct IKAttempt
    {
        Eigen::VectorXd configuration;
        bool converged;
        double task_error;
    };

    IKAttempt RunIKFromSeed(
        const Eigen::Matrix4d& Target_Pose,
        const KinematicModel& Model,
        Eigen::VectorXd Current_Guess,
        const double tol,
        const int max_it)
    {
        const IKTaskMode task_mode = Model.getIKTaskMode();
        for (int iteration = 0; iteration < max_it; ++iteration)
        {
            const Eigen::Matrix4d Current_FK = Model.ForwardKinematics(Current_Guess);
            const Eigen::Vector3d Position_Error =
                Target_Pose.block<3, 1>(0, 3) - Current_FK.block<3, 1>(0, 3);
            const Eigen::Vector3d Orientation_Error = RotationVectorError(
                Target_Pose.block<3, 3>(0, 0), Current_FK.block<3, 3>(0, 0));

            if (Position_Error.norm() < tol &&
                (task_mode == IKTaskMode::ONLY_POSITION ||
                 Orientation_Error.norm() < OrientationTolerance))
            {
                break;
            }

            Eigen::MatrixXd Jacobian;
            Eigen::VectorXd Task_Error;
            if (task_mode == IKTaskMode::FULL_POSE)
            {
                Jacobian = Jacobian_Pose(Model, Current_Guess, 1e-6, OrientationLengthScale);
                Task_Error.resize(6);
                Task_Error.head<3>() = Position_Error;
                Task_Error.tail<3>() = OrientationLengthScale * Orientation_Error;
            }
            else
            {
                Jacobian = Jacobian_Linear(Model, Current_Guess);
                Task_Error = Position_Error;
            }

            const Eigen::VectorXd Delta_Q = Pseudo_Inverse(Jacobian) * Task_Error;
            Current_Guess += Delta_Q;

            for (int i = 0; i < Model.getDOF(); ++i)
            {
                if (Model.isJointRevolute(i))
                {
                    Current_Guess(i) = std::remainder(Current_Guess(i), TwoPi);
                }

                if (Model.isJointLimited(i))
                {
                    Current_Guess(i) = std::clamp(
                        Current_Guess(i), Model.getJointMin(i), Model.getJointMax(i));
                }
            }
        }

        const Eigen::Matrix4d Final_FK = Model.ForwardKinematics(Current_Guess);
        return {
            Current_Guess,
            TaskConverged(Target_Pose, Final_FK, task_mode, tol),
            TaskErrorMagnitude(Target_Pose, Final_FK, task_mode)};
    }
}

Eigen::VectorXd Inverse_Kinematic(
    const Eigen::Matrix4d& Target_Pose,
    const KinematicModel& Model,
    bool& Converged,
    const double tol,
    const int max_it)
{
    const IKTaskMode task_mode = Model.getIKTaskMode();
    const Eigen::VectorXd seed = NaiveSeedGeneratorForIK(Target_Pose, Model, tol);
    IKAttempt best_attempt = RunIKFromSeed(Target_Pose, Model, seed, tol, max_it);
    if (best_attempt.converged || task_mode == IKTaskMode::ONLY_POSITION)
    {
        Converged = best_attempt.converged;
        return best_attempt.configuration;
    }

    std::vector<int> revolute_joints;
    for (int i = 0; i < Model.getDOF(); ++i)
    {
        if (Model.isJointRevolute(i))
        {
            revolute_joints.push_back(i);
        }
    }

    if (revolute_joints.size() >= 3)
    {
        const int wrist_first = revolute_joints[revolute_joints.size() - 3];
        const int wrist_middle = revolute_joints[revolute_joints.size() - 2];
        const int wrist_last = revolute_joints[revolute_joints.size() - 1];

        const auto joint_center = [&Model](const int joint)
        {
            if (!Model.isJointLimited(joint))
            {
                return 0.0;
            }
            return (Model.getJointMin(joint) + Model.getJointMax(joint)) / 2.0;
        };
        const auto quarter_range = [&Model](const int joint)
        {
            if (!Model.isJointLimited(joint))
            {
                return Pi / 4.0;
            }
            return (Model.getJointMax(joint) - Model.getJointMin(joint)) / 4.0;
        };

        for (int combination = 0; combination < 4; ++combination)
        {
            const double middle_sign = combination < 2 ? 1.0 : -1.0;
            const double last_sign = combination % 2 == 0 ? 1.0 : -1.0;
            Eigen::VectorXd alternate_seed = seed;
            alternate_seed(wrist_first) = joint_center(wrist_first);
            alternate_seed(wrist_middle) =
                joint_center(wrist_middle) + middle_sign * quarter_range(wrist_middle);
            alternate_seed(wrist_last) =
                joint_center(wrist_last) + last_sign * quarter_range(wrist_last);

            IKAttempt attempt = RunIKFromSeed(
                Target_Pose, Model, alternate_seed, tol, max_it);
            if (attempt.converged)
            {
                Converged = true;
                return attempt.configuration;
            }
            if (attempt.task_error < best_attempt.task_error)
            {
                best_attempt = attempt;
            }
        }
    }

    Converged = best_attempt.converged;
    return best_attempt.configuration;
}

Eigen::VectorXd Inverse_Kinematic(
    const Eigen::Vector3d& Target_Position,
    const KinematicModel& Model,
    bool& Converged,
    const double tol,
    const int max_it)
{
    Eigen::Matrix4d Target_Pose = Eigen::Matrix4d::Identity();
    Target_Pose.block<3, 1>(0, 3) = Target_Position;
    return Inverse_Kinematic(Target_Pose, Model, Converged, tol, max_it);
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
