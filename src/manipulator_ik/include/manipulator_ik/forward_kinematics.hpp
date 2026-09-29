#pragma once
#include <Eigen/Dense>
#include <vector>

enum class JointType
{
    Revolute,
    Prismatic
};

struct DHParameters
{
    double alpha;
    double a;
    double theta;
    double d;
    JointType jtype;
};


class DHMatrix
{
    private:
        DHParameters dhparams;
        Eigen::Matrix4d dhmatrix;
        

    public:
        //CONSTRUCTOR AND DESTRUCTOR
        DHMatrix(DHParameters _dhpar);
        ~DHMatrix();

        void computeMatrix();
        const Eigen::Matrix4d& getMatrix() const;

};

Eigen::Matrix4d ForwardKinematics(const std::vector<DHMatrix>& DHList);