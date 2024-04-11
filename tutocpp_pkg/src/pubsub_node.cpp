#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/int16.hpp"

class PubSubNode : public rclcpp::Node
{
public:
    PubSubNode() : Node("pubsub_node"), counter(0)
    {
        subscriber_ = create_subscription<example_interfaces::msg::Int16>("number", 10,
                                                                           std::bind(&PubSubNode::publisherCallBack,
                                                                                     this, std::placeholders::_1));
        publisher_ = create_publisher<example_interfaces::msg::Int16>("number_count", 10);                                                                                    
        timer = create_wall_timer(std::chrono::seconds(1), std::bind(&PubSubNode::publish, this));
        RCLCPP_INFO(get_logger(), "PubSub cpp Node has been started");
    }

private:
    rclcpp::Subscription<example_interfaces::msg::Int16>::SharedPtr subscriber_;
    rclcpp::Publisher<example_interfaces::msg::Int16>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer;

    int counter;

    void publish()
    {
        auto msg = example_interfaces::msg::Int16();
        msg.data = counter;
        publisher_->publish(msg);
    }

    void publisherCallBack(const example_interfaces::msg::Int16::SharedPtr msg)
    {
        
        counter += msg->data;
        RCLCPP_INFO(get_logger(), "%d", counter);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<PubSubNode>();

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}