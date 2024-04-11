#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/string.hpp"

class SubscriberNode : public rclcpp::Node
{
public:
    SubscriberNode() : Node("subscriberCpp")
    {
        subscriber_ = create_subscription<example_interfaces::msg::String>("first_publisher", 10,
                                                                           std::bind(&SubscriberNode::publisherCallBack,
                                                                                     this, std::placeholders::_1));
        RCLCPP_INFO(get_logger(), "Subscriber cpp Node has been started");
    }

private:
    rclcpp::Subscription<example_interfaces::msg::String>::SharedPtr subscriber_;

    void publisherCallBack(const example_interfaces::msg::String::SharedPtr msg)
    {
        RCLCPP_INFO(get_logger(), "%s", msg->data.c_str());
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