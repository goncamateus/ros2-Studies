#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/int16.hpp"

class PublisherNode : public rclcpp::Node
{
public:
    PublisherNode() : Node("publisherCpp")
    {
        publisher_ = create_publisher<example_interfaces::msg::Int16>("number", 10);
        timer = create_wall_timer(std::chrono::seconds(1), std::bind(&PublisherNode::publish, this));
        RCLCPP_INFO(get_logger(), "Publisher cpp Node has been started");
    }

private:
    rclcpp::Publisher<example_interfaces::msg::Int16>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer;

    void publish()
    {
        auto msg = example_interfaces::msg::Int16();
        msg.data = 2;
        publisher_->publish(msg);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<PublisherNode>();

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}