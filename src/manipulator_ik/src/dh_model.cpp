#include "manipulator_ik/dh_model.hpp"
#include <set>
#include <stdexcept>
#include <yaml-cpp/yaml.h>


DHModel::DHModel(const std::vector<DHMatrix>& dh_list)
    : DHList(dh_list), JointOffsets(dh_list.size(), 0.0), JointNames(dh_list.size())
{

}

DHModel::DHModel(const std::vector<DHMatrix>& dh_list,
                 const std::vector<double>& joint_offsets,
                 const std::vector<std::string>& joint_names)
    : DHList(dh_list), JointOffsets(joint_offsets), JointNames(joint_names)
{
    if (DHList.size() != JointOffsets.size() || DHList.size() != JointNames.size())
    {
        throw std::invalid_argument("DH model metadata size does not match its DOF");
    }
}

DHModel DHModel::FromYAML(const std::string& path)
{
    try
    {
        // yaml-cpp parses the document into a tree of YAML::Node objects.
        // LoadFile reads the whole file and reports syntax or conversion errors
        // through YAML exceptions, which are translated below with file context.
        YAML::Node Root = YAML::LoadFile(path);
        YAML::Node Robot = Root["robot"];
        YAML::Node Joints = Robot["joints"];

        if (!Robot || !Robot.IsMap() || !Joints || !Joints.IsSequence() || Joints.size() == 0)
        {
            throw std::invalid_argument("DH YAML must contain a non-empty robot.joints sequence");
        }

        std::vector<DHMatrix> DHList;
        std::vector<double> JointOffsets;
        std::vector<std::string> JointNames;
        std::set<std::string> UsedNames;

        for (std::size_t i=0; i<Joints.size(); i++)
        {
            YAML::Node Joint = Joints[i];
            const std::string Prefix = "robot.joints[" + std::to_string(i) + "]";

            if (!Joint.IsMap())
            {
                throw std::invalid_argument(Prefix + " must be a map");
            }

            const char* RequiredFields[] = {"name", "type", "alpha", "a", "theta", "d"};
            for (const char* Field : RequiredFields)
            {
                if (!Joint[Field])
                {
                    throw std::invalid_argument(Prefix + " is missing field '" + Field + "'");
                }
            }

            // YAML::Node::as<T>() converts a YAML scalar to the requested C++ type.
            const std::string Name = Joint["name"].as<std::string>();
            const std::string Type = Joint["type"].as<std::string>();
            const bool Limited = Joint["limited"] ? Joint["limited"].as<bool>() : true;

            if (Name.empty() || !UsedNames.insert(Name).second)
            {
                throw std::invalid_argument(Prefix + " has an empty or duplicate joint name");
            }

            JointParameters Parameters{};
            Parameters.alpha = Joint["alpha"].as<double>();
            Parameters.a = Joint["a"].as<double>();
            Parameters.theta = Joint["theta"].as<double>();
            Parameters.d = Joint["d"].as<double>();
            Parameters.isLimited = Limited;

            if (Limited)
            {
                if (!Joint["lower"] || !Joint["upper"])
                {
                    throw std::invalid_argument(Prefix + " requires lower and upper limits");
                }
                Parameters.joint_min = Joint["lower"].as<double>();
                Parameters.joint_max = Joint["upper"].as<double>();
                if (Parameters.joint_min > Parameters.joint_max)
                {
                    throw std::invalid_argument(Prefix + " has lower greater than upper");
                }
            }

            double JointOffset = 0.0;
            if (Type == "revolute")
            {
                Parameters.jtype = JointType::Revolute;
                JointOffset = Parameters.theta;
                Parameters.theta = 0.0;
            }
            else if (Type == "prismatic")
            {
                Parameters.jtype = JointType::Prismatic;
                JointOffset = Parameters.d;
                Parameters.d = 0.0;
            }
            else
            {
                throw std::invalid_argument(Prefix + " has unsupported type '" + Type + "'");
            }

            DHList.emplace_back(Parameters);
            JointOffsets.push_back(JointOffset);
            JointNames.push_back(Name);
        }

        return DHModel(DHList, JointOffsets, JointNames);
    }
    catch (const YAML::Exception& error)
    {
        throw std::runtime_error("Unable to load DH YAML '" + path + "': " + error.what());
    }
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
                ConfiguredDHList[i].setTheta(JointOffsets[i] + q(i));
                break;
            }
            case JointType::Prismatic:
            {
                ConfiguredDHList[i].setD(JointOffsets[i] + q(i));
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

std::string DHModel::getJointName(int index) const
{
    if (index < 0 || index >= getDOF())
    {
        throw std::out_of_range("Joint index is outside model DOF");
    }

    return JointNames[static_cast<std::size_t>(index)];
}
