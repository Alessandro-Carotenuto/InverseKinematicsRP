#include "manipulator_ik/forward_kinematics.hpp"
#include <cmath>


DHMatrix::DHMatrix(DHParameters _dhpar) : dhparams(_dhpar)
{
    computeMatrix();
}

DHMatrix::~DHMatrix()
{
    
}

void DHMatrix::computeMatrix()
{
    dhmatrix(0,0) =  std::cos(dhparams.theta);
    dhmatrix(0,1) = -std::cos(dhparams.alpha) * std::sin(dhparams.theta);
    dhmatrix(0,2) =  std::sin(dhparams.alpha) * std::sin(dhparams.theta);
    dhmatrix(0,3) =  dhparams.a * std::cos(dhparams.theta);

    dhmatrix(1,0) =  std::sin(dhparams.theta);
    dhmatrix(1,1) =  std::cos(dhparams.alpha) * std::cos(dhparams.theta);
    dhmatrix(1,2) = -std::sin(dhparams.alpha) * std::cos(dhparams.theta);
    dhmatrix(1,3) =  dhparams.a * std::sin(dhparams.theta);

    dhmatrix(2,0) =  0.0;
    dhmatrix(2,1) =  std::sin(dhparams.alpha);
    dhmatrix(2,2) =  std::cos(dhparams.alpha);
    dhmatrix(2,3) =  dhparams.d;

    dhmatrix(3,0) =  0.0;
    dhmatrix(3,1) =  0.0;
    dhmatrix(3,2) =  0.0;
    dhmatrix(3,3) =  1.0;
}


const Eigen::Matrix4d& DHMatrix::getMatrix() const
{
    return dhmatrix;
}

JointType DHMatrix::getJointType() const
{
    return dhparams.jtype;
}


void DHMatrix::setAlpha(double alpha)
{
    dhparams.alpha = alpha;
    computeMatrix();
}

void DHMatrix::setA(double a)
{
    dhparams.a = a;
    computeMatrix();
}

void DHMatrix::setTheta(double theta)
{
    dhparams.theta = theta;
    computeMatrix();
}

void DHMatrix::setD(double d)
{
    dhparams.d = d;
    computeMatrix();
}

double DHMatrix::getAlpha() const
{
    return dhparams.alpha;
}

double DHMatrix::getA() const
{
    return dhparams.a;
}

double DHMatrix::getTheta() const
{
    return dhparams.theta;
}

double DHMatrix::getD() const
{
    return dhparams.d;
}



Eigen::Matrix4d ForwardKinematics(const std::vector<DHMatrix>& DHList)
{
    int size=DHList.size();
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    for (int i=0; i<size; i++)
    {
        T*=DHList[i].getMatrix();
    }
    return T;
}



Eigen::MatrixXd Jacobian_Linear(std::vector<DHMatrix> DHList, const double eps)
{
    int nlinks = DHList.size();   
    Eigen::Matrix4d BaselineFK = ForwardKinematics(DHList);
    Eigen::Vector3d BaselinePos = BaselineFK.block<3, 1>(0, 3);
    
    Eigen::MatrixXd Jacobian(3, nlinks); 

    for (int i = 0; i < nlinks; i++)
    {
        switch (DHList[i].getJointType())
        {
            case JointType::Revolute:
            {
                double old_theta = DHList[i].getTheta();
                DHList[i].setTheta(old_theta + eps);
                Eigen::Matrix4d NewFK = ForwardKinematics(DHList);
                Eigen::Vector3d NewPos = NewFK.block<3, 1>(0, 3);
                Jacobian.col(i) = (NewPos - BaselinePos) / eps;
                DHList[i].setTheta(old_theta);
                break;
            }
            case JointType::Prismatic:
            {
                double old_d = DHList[i].getD();
                DHList[i].setD(old_d + eps);
                Eigen::Matrix4d NewFK = ForwardKinematics(DHList);
                Eigen::Vector3d NewPos = NewFK.block<3, 1>(0, 3);
                Jacobian.col(i) = (NewPos - BaselinePos) / eps;
                DHList[i].setD(old_d);
                break;
            }
        }
    }

    return Jacobian;
}

