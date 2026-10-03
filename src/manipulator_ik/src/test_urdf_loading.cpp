#include <cmath>
#include <iostream>
#include <string>
#include <vector>

#include <urdf_parser/urdf_parser.h>


const double tolerance = 1e-9;


bool NearlyEqual(double first, double second)
{
    return std::abs(first - second) <= tolerance;
}


bool CheckVector(const urdf::Vector3& vector, double x, double y, double z)
{
    return NearlyEqual(vector.x, x) &&
           NearlyEqual(vector.y, y) &&
           NearlyEqual(vector.z, z);
}


bool CheckOrigin(const urdf::Pose& origin, double x, double y, double z)
{
    double roll;
    double pitch;
    double yaw;
    origin.rotation.getRPY(roll, pitch, yaw);

    return CheckVector(origin.position, x, y, z) &&
           NearlyEqual(roll, 0.0) &&
           NearlyEqual(pitch, 0.0) &&
           NearlyEqual(yaw, 0.0);
}


bool CheckJoint(
    const urdf::ModelInterfaceSharedPtr& model,
    const std::string& name,
    int type,
    const std::string& parent,
    const std::string& child,
    double axis_x,
    double axis_y,
    double axis_z,
    double origin_x,
    double origin_y,
    double origin_z,
    bool check_limits = false,
    double lower = 0.0,
    double upper = 0.0)
{
    urdf::JointConstSharedPtr joint = model->getJoint(name);
    bool passed = joint != nullptr;

    if (joint)
    {
        passed = passed && joint->type == type;
        passed = passed && joint->parent_link_name == parent;
        passed = passed && joint->child_link_name == child;
        passed = passed && CheckOrigin(
            joint->parent_to_joint_origin_transform,
            origin_x,
            origin_y,
            origin_z);

        if (type != urdf::Joint::FIXED)
        {
            passed = passed && CheckVector(joint->axis, axis_x, axis_y, axis_z);
        }

        if (check_limits)
        {
            passed = passed && joint->limits != nullptr;
            if (joint->limits)
            {
                passed = passed && NearlyEqual(joint->limits->lower, lower);
                passed = passed && NearlyEqual(joint->limits->upper, upper);
            }
        }
    }

    std::cout << "Joint " << name << ": " << (passed ? "PASSED" : "NOT PASSED") << "\n";
    return passed;
}


bool CheckModel(const std::string& path, bool expects_visual_data)
{
    // urdfdom is the library that converts a URDF document into C++ objects.
    // ModelInterface represents the complete robot as connected links and joints.
    // parseURDFFile opens and parses the file, returning nullptr when parsing fails.
    urdf::ModelInterfaceSharedPtr model = urdf::parseURDFFile(path);

    if (!model)
    {
        std::cout << "Unable to load " << path << "\n";
        return false;
    }

    bool passed = true;

    // ModelInterface exposes the parsed link and joint collections through maps.
    // It also identifies the root link after validating that the robot is a tree.
    passed = passed && model->getName() == "manipulator_rrp";
    passed = passed && model->links_.size() == 5;
    passed = passed && model->joints_.size() == 4;
    passed = passed && model->getRoot() != nullptr;
    passed = passed && model->getRoot()->name == "base_link";

    const std::vector<std::string> expected_links = {
        "base_link",
        "rotating_column",
        "pitch_link",
        "telescopic_link",
        "tool0"
    };

    for (const std::string& link_name : expected_links)
    {
        urdf::LinkConstSharedPtr link = model->getLink(link_name);
        passed = passed && link != nullptr;

        if (link)
        {
            bool has_visual_data = link->visual != nullptr;
            passed = passed && has_visual_data == expects_visual_data;
        }
    }

    passed = CheckJoint(
        model,
        "joint_1",
        urdf::Joint::REVOLUTE,
        "base_link",
        "rotating_column",
        0.0,
        0.0,
        1.0,
        0.0,
        0.0,
        0.20,
        true,
        -3.141592653589793,
        3.141592653589793) && passed;

    passed = CheckJoint(
        model,
        "joint_2",
        urdf::Joint::REVOLUTE,
        "rotating_column",
        "pitch_link",
        0.0,
        -1.0,
        0.0,
        0.0,
        0.0,
        0.80,
        true,
        -1.20,
        1.20) && passed;

    passed = CheckJoint(
        model,
        "joint_3",
        urdf::Joint::PRISMATIC,
        "pitch_link",
        "telescopic_link",
        1.0,
        0.0,
        0.0,
        0.20,
        0.0,
        0.0,
        true,
        0.0,
        0.80) && passed;

    passed = CheckJoint(
        model,
        "tool0_joint",
        urdf::Joint::FIXED,
        "telescopic_link",
        "tool0",
        0.0,
        0.0,
        0.0,
        0.80,
        0.0,
        0.0) && passed;

    std::cout << "Model " << path << ": " << (passed ? "PASSED" : "NOT PASSED") << "\n\n";
    return passed;
}


int main(int argc, char* argv[])
{
    if (argc != 3)
    {
        std::cout << "Usage: test_urdf_loading <kinematic_urdf> <visual_urdf>\n";
        return 1;
    }

    bool kinematic_model_passed = CheckModel(argv[1], false);
    bool visual_model_passed = CheckModel(argv[2], true);

    return kinematic_model_passed && visual_model_passed ? 0 : 1;
}
