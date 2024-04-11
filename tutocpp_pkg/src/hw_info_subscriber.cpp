#include "rclcpp/rclcpp.hpp"
#include "gonca_interfaces/msg/hardware_info.hpp"

class HWInfoSubscriberNode : public rclcpp::Node
{
public:
    HWInfoSubscriberNode() : Node("cpp_test_node")
    {
        subscriber_ = create_subscription<gonca_interfaces::msg::HardwareInfo>("hw_info", 10, std::bind(&HWInfoSubscriberNode::subscriberCallback, this, std::placeholders::_1));
        RCLCPP_INFO(get_logger(), "Hardware Info Subscriber has been started.");
    }

private:
    rclcpp::Subscription<gonca_interfaces::msg::HardwareInfo>::SharedPtr subscriber_;

    void subscriberCallback(gonca_interfaces::msg::HardwareInfo::SharedPtr msg)
    {
        RCLCPP_INFO(get_logger(), "Temperature: %ld", msg->temperature);
        RCLCPP_INFO(get_logger(), "Are motors ready: %d", msg->are_motors_ready);
        RCLCPP_INFO(get_logger(), "Debug message: %s", msg->debug_message.c_str());
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<HWInfoSubscriberNode>();

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}