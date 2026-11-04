#include "manipulator_ik/kinematic_model.hpp"
#include "manipulator_ik/dh_model.hpp"
#include "manipulator_ik/urdf_model.hpp"
#include "manipulator_ik/inverse_kinematics.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/parameter_client.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"

#include <yaml-cpp/yaml.h>
#include <chrono>
#include <exception>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iterator>
#include <future>
#include <memory>
#include <string>
#include <vector>

class ManipulatorIKNode : public rclcpp::Node
{
private:
    std::string target_position_file_path_ = "src/manipulator_ik/src/targetPosition.yaml";
    Eigen::VectorXd joint_configuration;

    std::filesystem::path robots_directory_ = "src/manipulator_ik/Robots";
    std::string model_file_path_;
    std::string model_file_extension_;
    std::unique_ptr<KinematicModel> model_;

    rclcpp::AsyncParametersClient::SharedPtr robot_state_publisher_client_;

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

            sensor_msgs::msg::JointState msg;
            msg.header.stamp = this->now();
            for (int i = 0; i < joint_configuration.size(); ++i)
            {
                msg.name.emplace_back(model_->getJointName(i));
                msg.position.emplace_back(joint_configuration(i));
            }

            joint_state_pub_->publish(msg);
        }
        else
        {
            response->message = "Target non raggiunto entro la tolleranza";
        }

        RCLCPP_INFO(get_logger(), "IK: %s", ik_success_flag ? "convergente" : "non convergente");

        for (int i = 0; i < joint_configuration.size(); ++i)
        {
            RCLCPP_INFO(
                get_logger(), "  %s: %.4f %s", model_->getJointName(i).c_str(),
                joint_configuration(i), model_->isJointRevolute(i) ? "rad" : "m");
        }
    }

    void on_robot_description_set(
        std::shared_future<std::vector<rcl_interfaces::msg::SetParametersResult>> result_future)
    {
        try
        {
            const auto results = result_future.get();
            if (results.empty() || !results.front().successful)
            {
                const std::string reason = results.empty() ? "risposta vuota" : results.front().reason;
                RCLCPP_ERROR(get_logger(), "robot_state_publisher ha rifiutato robot_description: %s", reason.c_str());
                return;
            }

            RCLCPP_INFO(get_logger(), "robot_description aggiornato in robot_state_publisher");
        }
        catch (const std::exception& error)
        {
            RCLCPP_ERROR(get_logger(), "Errore aggiornando robot_description: %s", error.what());
        }
    }

    void update_robot_description(const std::string& urdf_xml)
    {
        if (!robot_state_publisher_client_->wait_for_service(std::chrono::seconds(1)))
        {
            RCLCPP_WARN(
                get_logger(),
                "robot_state_publisher non trovato: robot_description non aggiornato. "
                "Avvia prima quel nodo e poi cambia model_name.");
            return;
        }

        robot_state_publisher_client_->set_parameters(
            {rclcpp::Parameter("robot_description", urdf_xml)},
            std::bind(
                &ManipulatorIKNode::on_robot_description_set, this,
                std::placeholders::_1));
    }

    bool load_model(const std::string& model_file)
    {
        model_file_path_.clear();
        model_file_extension_.clear();

        for (const auto& file : std::filesystem::directory_iterator(robots_directory_))
        {
            if (file.path().stem().string() == model_file)
            {
                model_file_path_ = file.path().string();
                model_file_extension_ = file.path().extension().string();
                break;
            }
        }

        if (model_file_path_.empty())
        {
            RCLCPP_ERROR(get_logger(), "Modello '%s' non trovato in %s", model_file.c_str(), robots_directory_.c_str());
            return false;
        }

        if (model_file_extension_ == ".yaml")
        {
            model_ = std::make_unique<DHModel>(DHModel::FromYAML(model_file_path_));
            RCLCPP_INFO(get_logger(), "Caricato modello DH '%s'; robot_description lasciato invariato", model_file.c_str());
            return true;
        }

        if (model_file_extension_ == ".urdf")
        {
            std::ifstream urdf_file(model_file_path_);
            if (!urdf_file)
            {
                RCLCPP_ERROR(get_logger(), "Impossibile aprire il file URDF: %s", model_file_path_.c_str());
                return false;
            }

            const std::string urdf_xml{
                std::istreambuf_iterator<char>{urdf_file}, std::istreambuf_iterator<char>{}};
            model_ = std::make_unique<URDFModel>(URDFModel::FromFile(model_file_path_, "base_link", "tool0"));
            update_robot_description(urdf_xml);
            return true;
        }

        RCLCPP_ERROR(get_logger(), "Estensione modello non supportata: %s", model_file_extension_.c_str());
        return false;
    }

    void on_model_name_change(const std::vector<rclcpp::Parameter>& parameters)
    {
        for (const auto& parameter : parameters)
        {
            if (parameter.get_name() == "model_name")
            {
                load_model(parameter.as_string());
                return;
            }
        }
    }

public:
    ManipulatorIKNode(const std::string model_file_) : Node("manipulator_ik")
    {
        joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
        robot_state_publisher_client_ = std::make_shared<rclcpp::AsyncParametersClient>(this, "robot_state_publisher");

        const auto model_name = this->declare_parameter<std::string>("model_name", model_file_);
        load_model(model_name);

        model_name_callback_handle_ = this->add_post_set_parameters_callback(
            std::bind(
                &ManipulatorIKNode::on_model_name_change, this,
                std::placeholders::_1));
        update_eepos_service_ = this->create_service<std_srvs::srv::Trigger>(
            "EE_pos_update",
            std::bind(
                &ManipulatorIKNode::update_target_position, this,
                std::placeholders::_2));
        RCLCPP_INFO(get_logger(), "Manipulator IK node started");
    }

    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr update_eepos_service_;
    rclcpp::node_interfaces::PostSetParametersCallbackHandle::SharedPtr model_name_callback_handle_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    const std::string model_file = argv[1];

    auto Node = std::make_shared<ManipulatorIKNode>(model_file);
    rclcpp::spin(Node);
    rclcpp::shutdown();
    return 0;
}
