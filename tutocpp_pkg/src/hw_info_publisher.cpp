#include "rclcpp/rclcpp.hpp"
#include "gonca_interfaces/msg/hardware_info.hpp"

class HWInfoPublisherNode : public rclcpp::Node
{
public:
    HWInfoPublisherNode() : Node("hw_info_publisher")
    {
        publisher_ = create_publisher<gonca_interfaces::msg::HardwareInfo>("hw_info", 10);
        timer = create_wall_timer(std::chrono::seconds(1), std::bind(&HWInfoPublisherNode::publisherCallback, this));
        RCLCPP_INFO(get_logger(), "Hardware Info Publisher has been started.");
    }

private:
    rclcpp::TimerBase::SharedPtr timer;
    rclcpp::Publisher<gonca_interfaces::msg::HardwareInfo>::SharedPtr publisher_;

    void publisherCallback()
    {
        auto msg = gonca_interfaces::msg::HardwareInfo();
        msg.temperature = 50;
        msg.are_motors_ready = true;
        msg.debug_message = "Temperature is high! Please check the cooling system.";
        publisher_->publish(msg);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<HWInfoPublisherNode>();

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}