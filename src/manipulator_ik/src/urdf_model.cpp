#include "manipulator_ik/urdf_model.hpp"

#include <Eigen/Geometry>
#include <algorithm>
#include <stdexcept>
#include <vector>

#include <urdf_parser/urdf_parser.h>


namespace
{
    Eigen::Matrix4d PoseToMatrix(const urdf::Pose& pose)
    {
        Eigen::Matrix4d Transform = Eigen::Matrix4d::Identity();

        Eigen::Quaterniond Rotation(
            pose.rotation.w,
            pose.rotation.x,
            pose.rotation.y,
            pose.rotation.z);

        Transform.block<3, 3>(0, 0) = Rotation.toRotationMatrix();
        Transform(0, 3) = pose.position.x;
        Transform(1, 3) = pose.position.y;
        Transform(2, 3) = pose.position.z;

        return Transform;
    }
}


URDFModel::URDFModel(const std::vector<JointData>& joint_chain) : JointChain(joint_chain)
{
    for (std::size_t i=0; i<JointChain.size(); i++)
    {
        if (JointChain[i].q_index >= 0)
        {
            ActiveJointIndices.push_back(i);
        }
    }
}

URDFModel::~URDFModel()
{

}

URDFModel URDFModel::FromFile(
    const std::string& path,
    const std::string& base_link,
    const std::string& tool_link)
{
    // urdfdom opens the file and converts its XML into a ModelInterface object.
    // ModelInterface contains the complete link and joint tree of the robot.
    // parseURDFFile returns nullptr when the file cannot produce a valid model.
    urdf::ModelInterfaceSharedPtr ParsedModel = urdf::parseURDFFile(path);

    if (!ParsedModel)
    {
        throw std::runtime_error("Unable to parse URDF file: " + path);
    }

    urdf::LinkConstSharedPtr BaseLink = ParsedModel->getLink(base_link);
    urdf::LinkConstSharedPtr CurrentLink = ParsedModel->getLink(tool_link);

    if (!BaseLink)
    {
        throw std::invalid_argument("Base link not found in URDF: " + base_link);
    }

    if (!CurrentLink)
    {
        throw std::invalid_argument("Tool link not found in URDF: " + tool_link);
    }

    std::vector<urdf::JointConstSharedPtr> ParsedJointChain;
    std::size_t TraversedLinks = 0;

    // Walk from the tool towards the base because every link has one parent joint.
    // The resulting list is reversed afterwards to obtain base-to-tool order.
    while (CurrentLink->name != BaseLink->name)
    {
        if (!CurrentLink->parent_joint)
        {
            throw std::invalid_argument(
                "Base link is not an ancestor of tool link: " +
                base_link + " -> " + tool_link);
        }

        ParsedJointChain.push_back(CurrentLink->parent_joint);
        CurrentLink = ParsedModel->getLink(CurrentLink->parent_joint->parent_link_name);

        if (!CurrentLink)
        {
            throw std::runtime_error("URDF chain contains a missing parent link");
        }

        TraversedLinks++;
        if (TraversedLinks > ParsedModel->links_.size())
        {
            throw std::runtime_error("URDF chain contains a cycle");
        }
    }

    std::reverse(ParsedJointChain.begin(), ParsedJointChain.end());

    std::vector<JointData> JointChain;
    int NextQIndex = 0;

    // Convert urdfdom objects into the small Eigen-based representation used by FK.
    for (const urdf::JointConstSharedPtr& ParsedJoint : ParsedJointChain)
    {
        JointData Joint{};
        Joint.name = ParsedJoint->name;
        Joint.origin = PoseToMatrix(ParsedJoint->parent_to_joint_origin_transform);
        Joint.axis = Eigen::Vector3d(
            ParsedJoint->axis.x,
            ParsedJoint->axis.y,
            ParsedJoint->axis.z);
        Joint.joint_min = 0.0;
        Joint.joint_max = 0.0;
        Joint.is_limited = false;
        Joint.q_index = -1;

        switch (ParsedJoint->type)
        {
            case urdf::Joint::REVOLUTE:
            {
                Joint.type = JointMotionType::Revolute;
                break;
            }
            case urdf::Joint::PRISMATIC:
            {
                Joint.type = JointMotionType::Prismatic;
                break;
            }
            case urdf::Joint::FIXED:
            {
                Joint.type = JointMotionType::Fixed;
                break;
            }
            default:
            {
                throw std::invalid_argument(
                    "Unsupported joint type in URDF: " + ParsedJoint->name);
            }
        }

        if (Joint.type != JointMotionType::Fixed)
        {
            if (!ParsedJoint->limits)
            {
                throw std::invalid_argument(
                    "Movable joint has no limits: " + ParsedJoint->name);
            }

            if (Joint.axis.norm() <= 1e-12)
            {
                throw std::invalid_argument(
                    "Movable joint has a zero axis: " + ParsedJoint->name);
            }

            Joint.axis.normalize();
            Joint.joint_min = ParsedJoint->limits->lower;
            Joint.joint_max = ParsedJoint->limits->upper;
            Joint.is_limited = true;
            Joint.q_index = NextQIndex;
            NextQIndex++;
        }

        JointChain.push_back(Joint);
    }

    return URDFModel(JointChain);
}

Eigen::Matrix4d URDFModel::ForwardKinematics(const Eigen::VectorXd& q) const
{
    if (q.size() != getDOF())
    {
        throw std::invalid_argument("Joint configuration size does not match model DOF");
    }

    Eigen::Matrix4d Transform = Eigen::Matrix4d::Identity();

    for (const JointData& Joint : JointChain)
    {
        Eigen::Matrix4d JointMotion = Eigen::Matrix4d::Identity();

        // Each URDF step first reaches the joint frame, then applies its motion.
        switch (Joint.type)
        {
            case JointMotionType::Revolute:
            {
                JointMotion.block<3, 3>(0, 0) =
                    Eigen::AngleAxisd(q(Joint.q_index), Joint.axis).toRotationMatrix();
                break;
            }
            case JointMotionType::Prismatic:
            {
                JointMotion.block<3, 1>(0, 3) = Joint.axis * q(Joint.q_index);
                break;
            }
            case JointMotionType::Fixed:
            {
                break;
            }
        }

        Transform = Transform * Joint.origin * JointMotion;
    }

    return Transform;
}

int URDFModel::getDOF() const
{
    return static_cast<int>(ActiveJointIndices.size());
}

const URDFModel::JointData& URDFModel::getActiveJoint(int index) const
{
    if (index < 0 || index >= getDOF())
    {
        throw std::out_of_range("Joint index is outside model DOF");
    }

    return JointChain[ActiveJointIndices[static_cast<std::size_t>(index)]];
}

double URDFModel::getJointMin(int index) const
{
    return getActiveJoint(index).joint_min;
}

double URDFModel::getJointMax(int index) const
{
    return getActiveJoint(index).joint_max;
}

bool URDFModel::isJointLimited(int index) const
{
    return getActiveJoint(index).is_limited;
}

std::string URDFModel::getJointName(int index) const
{
    return getActiveJoint(index).name;
}
