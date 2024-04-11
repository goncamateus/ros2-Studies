#include "rclcpp/rclcpp.hpp"

class MyNode : public rclcpp::Node
{
public:
    MyNode() : Node("cpp_test_node"), counter(0)
    {
        timer = create_wall_timer(std::chrono::seconds(1), std::bind(&MyNode::timerCallback, this));
    }

private:
    rclcpp::TimerBase::SharedPtr timer;
    int counter;

    void timerCallback()
    {
        counter++;
        RCLCPP_INFO(get_logger(), "Hello %d", counter);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<MyNode>();

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}