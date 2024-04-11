#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/int16.hpp"

class SubscriberNode : public rclcpp::Node
{
public:
    SubscriberNode() : Node("subscriberCpp")
    {
        declare_parameter("topic_name", "first_publisher");
        get_parameter("topic_name", topic_name);
        subscriber_ = create_subscription<example_interfaces::msg::Int16>(topic_name, 10,
                                                                          std::bind(&SubscriberNode::publisherCallBack,
                                                                                    this, std::placeholders::_1));
        RCLCPP_INFO(get_logger(), "Subscriber cpp Node has been started");
    }

private:
    rclcpp::Subscription<example_interfaces::msg::Int16>::SharedPtr subscriber_;
    std::string topic_name;

    void publisherCallBack(const example_interfaces::msg::Int16::SharedPtr msg)
    {
        RCLCPP_INFO(get_logger(), "%d", msg->data);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<SubscriberNode>();

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}