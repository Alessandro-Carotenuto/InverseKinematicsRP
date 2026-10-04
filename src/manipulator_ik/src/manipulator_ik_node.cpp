#include <memory>
#include "rclcpp/rclcpp.hpp"

class ManipulatorIkNode : public rclcpp::Node
{
    public:
        ManipulatorIKNode() : Node("manipulator_ik")
        {
            RCLCPP_INFO(get_logger(),"Manipulator IK node started");
        }
};

int main(int argc, char* argv[])
{
    // Initializes ROS context and reads ROS command-line arguments i guess
    rclcpp::init(argc, argv);

    auto Node = std::make_shared<ManipulatorIKNode>();

    // waits for ROS events and invokes this node's callbacks
    rclcpp::spin(Node);

    // Releases ROS resources after shutdown, usually triggered with Ctrl+C.
    rclcpp::shutdown();
    return 0;
}