#pragma once
#include <Eigen/Dense>
#include <vector>

enum class JointType
{
    Revolute,
    Prismatic
};

struct JointParameters
{
    double alpha;
    double a;
    double theta;
    double d;
    
    JointType jtype;
    double joint_min;
    double joint_max;
    bool isLimited;
};


class DHMatrix
{
    private:
        JointParameters jointparams;
        Eigen::Matrix4d dhmatrix;
    

    public:
        //CONSTRUCTOR AND DESTRUCTOR
        DHMatrix(JointParameters joint_parameters);
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
        bool isLimited() const;
        double getJointMin() const;
        double getJointMax() const;

};

Eigen::Matrix4d ForwardKinematics(const std::vector<DHMatrix>& DHList);

Eigen::MatrixXd Jacobian_Linear(std::vector<DHMatrix> DHList, const double eps = 1e-6);