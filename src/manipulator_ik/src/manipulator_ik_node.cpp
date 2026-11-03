#include "manipulator_ik/kinematic_model.hpp"
#include "manipulator_ik/dh_model.hpp"
#include "manipulator_ik/urdf_model.hpp"
#include "manipulator_ik/inverse_kinematics.hpp"

#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"

#include <yaml-cpp/yaml.h>
#include <functional>
#include <filesystem>
#include <memory>
#include <vector>
#include <string>

class ManipulatorIKNode : public rclcpp::Node
{
    private:
        std::string target_position_file_path_ = "src/manipulator_ik/src/targetPosition.yaml";
        Eigen::VectorXd joint_configuration;

        void update_target_position(
                std::shared_ptr<std_srvs::srv::Trigger::Response> response)
        {
            YAML::Node root = YAML::LoadFile(target_position_file_path_);
            YAML::Node position = root["position"];
            double x = position["x"].as<double>();
            double y = position["y"].as<double>();
            double z = position["z"].as<double>();

            Eigen::Vector3d EE_Target_Position{x, y, z};
            bool ik_success_flag = false;
            joint_configuration = Inverse_Kinematic(EE_Target_Position, *model_, ik_success_flag);

            response->success = ik_success_flag;

            if (response->success)
            {
                response->message = "Target raggiunto";
            }
            else
            {
                response->message = "Target non raggiunto entro la tolleranza";
            }

            RCLCPP_INFO(get_logger(), "IK: %s",ik_success_flag ? "convergente" : "non convergente");

            for (int i = 0; i < joint_configuration.size(); ++i)
            {
                RCLCPP_INFO(get_logger(), "  %s: %.4f rad",model_->getJointName(i).c_str(),joint_configuration(i));
            }

        }

        std::filesystem::path robots_directory_ = "src/manipulator_ik/Robots";
        std::string model_file_path_;
        std::string model_file_extension_;
        std::unique_ptr<KinematicModel> model_;

        void load_model(const std::string model_file_)
        {
            //search the model file by name
            for (const auto& file : std::filesystem::directory_iterator(robots_directory_))
            {
                std::string temp_filename = file.path().stem().string();
                if (temp_filename == model_file_)
                {
                    model_file_path_=file.path().string();
                    model_file_extension_=file.path().extension().string();
                }

            }

            //load the model
            if (model_file_extension_ == ".yaml")
            {
                model_ = std::make_unique<DHModel>(DHModel::FromYAML(model_file_path_));
            }
            else if (model_file_extension_ == ".urdf")
            {
                model_ = std::make_unique<URDFModel>(URDFModel::FromFile(model_file_path_, "base_link", "tool0"));
            }

        }
    
        void on_model_name_change(const std::vector<rclcpp::Parameter>&)
        {
            load_model(this->get_parameter("model_name").as_string());
        }

    public:
        ManipulatorIKNode(const std::string model_file_) : Node("manipulator_ik")
        {

            const auto model_name = this->declare_parameter<std::string>("model_name", model_file_);
            load_model(model_name);

            model_name_callback_handle_ = this->add_post_set_parameters_callback(std::bind(&ManipulatorIKNode::on_model_name_change,this, std::placeholders::_1));
            update_eepos_service_ = this->create_service<std_srvs::srv::Trigger>("EE_pos_update",std::bind(&ManipulatorIKNode::update_target_position,this,std::placeholders::_2));
            RCLCPP_INFO(get_logger(),"Manipulator IK node started");
        }
        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr update_eepos_service_;
        rclcpp::node_interfaces::PostSetParametersCallbackHandle::SharedPtr model_name_callback_handle_;
};

int main(int argc, char* argv[])
{
    // Initializes ROS context, READS COMMAND LINES ARGS
    rclcpp::init(argc, argv);
    const std::string model_file = argv[1];

    auto Node = std::make_shared<ManipulatorIKNode>(model_file);

    // waits for ROS events and invokes this node's callbacks
    rclcpp::spin(Node);

    // Releases ROS resources after shutdown, usually triggered with Ctrl+C.
    rclcpp::shutdown();
    return 0;
}