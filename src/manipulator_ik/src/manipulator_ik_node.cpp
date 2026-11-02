#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"

#include <functional>
#include <memory>
#include <string>

class ManipulatorIKNode : public rclcpp::Node
{
    private:
        void update_target_position(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                std::shared_ptr<std_srvs::srv::Trigger::Response> response)
        {
                
        }

        std::string target_position_file_path_ = "src/manipulator_ik/src/targetPosition.yaml";


    public:
        ManipulatorIKNode() : Node("manipulator_ik")
        {
            update_service_ = this->create_service<std_srvs::srv::Trigger>("EE_pos_update",std::bind(&ManipulatorIKNode::update_target_position,this,std::placeholders::_1,std::placeholders::_2));
            RCLCPP_INFO(get_logger(),"Manipulator IK node started");
        }
        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr update_service_;
};

int main(int argc, char* argv[])
{
    // Initializes ROS context, READS COMMAND LINES ARGS
    rclcpp::init(argc, argv);

    auto Node = std::make_shared<ManipulatorIKNode>();

    // waits for ROS events and invokes this node's callbacks
    rclcpp::spin(Node);

    // Releases ROS resources after shutdown, usually triggered with Ctrl+C.
    rclcpp::shutdown();
    return 0;
}