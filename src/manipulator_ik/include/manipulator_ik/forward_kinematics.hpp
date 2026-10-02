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

        
        void setAlpha(double alpha);
        void setA(double a);
        void setTheta(double theta);
        void setD(double d);

        void computeMatrix();

        const Eigen::Matrix4d& getMatrix() const;
        
        double getAlpha() const;
        double getA() const;
        double getTheta() const;
        double getD() const;
        JointType getJointType() const;

};

Eigen::Matrix4d ForwardKinematics(const std::vector<DHMatrix>& DHList);

Eigen::MatrixXd Jacobian_Linear(std::vector<DHMatrix> DHList, const double eps = 1e-6);