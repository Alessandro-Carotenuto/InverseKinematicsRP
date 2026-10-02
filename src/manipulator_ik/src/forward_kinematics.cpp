#include "manipulator_ik/forward_kinematics.hpp"
#include <cmath>


DHMatrix::DHMatrix(JointParameters joint_parameters) : jointparams(joint_parameters)
{
    computeMatrix();
}

DHMatrix::~DHMatrix()
{
    
}

void DHMatrix::computeMatrix()
{
    dhmatrix(0,0) =  std::cos(jointparams.theta);
    dhmatrix(0,1) = -std::cos(jointparams.alpha) * std::sin(jointparams.theta);
    dhmatrix(0,2) =  std::sin(jointparams.alpha) * std::sin(jointparams.theta);
    dhmatrix(0,3) =  jointparams.a * std::cos(jointparams.theta);

    dhmatrix(1,0) =  std::sin(jointparams.theta);
    dhmatrix(1,1) =  std::cos(jointparams.alpha) * std::cos(jointparams.theta);
    dhmatrix(1,2) = -std::sin(jointparams.alpha) * std::cos(jointparams.theta);
    dhmatrix(1,3) =  jointparams.a * std::sin(jointparams.theta);

    dhmatrix(2,0) =  0.0;
    dhmatrix(2,1) =  std::sin(jointparams.alpha);
    dhmatrix(2,2) =  std::cos(jointparams.alpha);
    dhmatrix(2,3) =  jointparams.d;

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
    return jointparams.jtype;
}

bool DHMatrix::isLimited() const
{
    return jointparams.isLimited;
}

double DHMatrix::getJointMin() const
{
    return jointparams.joint_min;
}

double DHMatrix::getJointMax() const
{
    return jointparams.joint_max;
}


void DHMatrix::setAlpha(double alpha)
{
    jointparams.alpha = alpha;
    computeMatrix();
}

void DHMatrix::setA(double a)
{
    jointparams.a = a;
    computeMatrix();
}

void DHMatrix::setTheta(double theta)
{
    jointparams.theta = theta;
    computeMatrix();
}

void DHMatrix::setD(double d)
{
    jointparams.d = d;
    computeMatrix();
}

double DHMatrix::getAlpha() const
{
    return jointparams.alpha;
}

double DHMatrix::getA() const
{
    return jointparams.a;
}

double DHMatrix::getTheta() const
{
    return jointparams.theta;
}

double DHMatrix::getD() const
{
    return jointparams.d;
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


