#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/string.hpp"

class RobotPubNode : public rclcpp::Node
{
public:
    RobotPubNode() : Node("robot_publisher")
    {
        declare_parameter("robot_name", "R2D2");
        get_parameter("robot_name", robot_name);
        publisher_ = create_publisher<example_interfaces::msg::String>("robot", 10);
        timer = create_wall_timer(std::chrono::seconds(1), std::bind(&RobotPubNode::publish, this));
        RCLCPP_INFO(get_logger(), "%s Node has been started", robot_name.c_str());
    }

private:
    rclcpp::Publisher<example_interfaces::msg::String>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer;
    std::string robot_name;

    void publish()
    {
        auto msg = example_interfaces::msg::String();
        msg.data = robot_name + " says hello!";
        publisher_->publish(msg);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<RobotPubNode>();

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}