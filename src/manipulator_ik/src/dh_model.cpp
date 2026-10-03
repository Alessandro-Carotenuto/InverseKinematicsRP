#include "manipulator_ik/dh_model.hpp"
#include <stdexcept>


DHModel::DHModel(const std::vector<DHMatrix>& dh_list) : DHList(dh_list)
{

}

DHModel::~DHModel()
{

}

Eigen::Matrix4d DHModel::ForwardKinematics(const Eigen::VectorXd& q) const
{
    if (q.size() != getDOF())
    {
        throw std::invalid_argument("Joint configuration size does not match model DOF");
    }

    std::vector<DHMatrix> ConfiguredDHList = DHList;

    for (int i=0; i<getDOF(); i++)
    {
        switch (ConfiguredDHList[i].getJointType())
        {
            case JointType::Revolute:
            {
                ConfiguredDHList[i].setTheta(q(i));
                break;
            }
            case JointType::Prismatic:
            {
                ConfiguredDHList[i].setD(q(i));
                break;
            }
        }
    }

    return ::ForwardKinematics(ConfiguredDHList);
}

int DHModel::getDOF() const
{
    return static_cast<int>(DHList.size());
}

double DHModel::getJointMin(int index) const
{
    return DHList[index].getJointMin();
}

double DHModel::getJointMax(int index) const
{
    return DHList[index].getJointMax();
}

bool DHModel::isJointLimited(int index) const
{
    return DHList[index].isLimited();
}
