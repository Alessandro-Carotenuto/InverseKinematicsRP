#include "manipulator_ik/inverse_kinematics.hpp"
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

Eigen::VectorXd Inverse_Kinematic(Eigen::Vector3d& Actual_Position, std::vector<DHMatrix>& DHList,const double tol, const int max_it)
{
    int nlinks = DHList.size();   
    //Reasonanle Initial Guess
    Eigen::VectorXd Current_Guess = Eigen::VectorXd::Zero(nlinks); //initialized to zero
    for (int i=0; i<nlinks; i++)
    {
        if (DHList[i].isLimited())
        {
            Current_Guess(i)=(DHList[i].getJointMax()+DHList[i].getJointMin())/2;
        }
    }

    
    int iterations=0;
    Eigen::Matrix4d CurrentGuessFK;
    Eigen::MatrixXd J;

    Eigen::Vector3d error, CurrentGuessPOS;
    Eigen::VectorXd delta_q;

    do
    {
        ApplyJointConfiguration(DHList,Current_Guess);
        CurrentGuessFK = ForwardKinematics(DHList);
        CurrentGuessPOS = CurrentGuessFK.block<3, 1>(0, 3);
        error = Actual_Position - CurrentGuessPOS;


        if (error.norm() < tol)
        {
            break;
        }

        J = Jacobian_Linear(DHList);
        delta_q = Pseudo_Inverse(J)*error;
        Current_Guess = Current_Guess+delta_q;
        iterations++;
    } while (iterations<max_it);
    
    return Current_Guess;
}

