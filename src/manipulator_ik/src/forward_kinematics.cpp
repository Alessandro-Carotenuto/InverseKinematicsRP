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