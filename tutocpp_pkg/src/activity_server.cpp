#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/int16.hpp"
#include "example_interfaces/srv/set_bool.hpp"

class ActServerNode : public rclcpp::Node
{
public:
    ActServerNode() : Node("activity_server"), counter(0)
    {
        server_ = create_service<example_interfaces::srv::SetBool>("reset_counter", std::bind(&ActServerNode::serverCallback, this, std::placeholders::_2, std::placeholders::_1));
        publisher_ = create_publisher<example_interfaces::msg::Int16>("number_counter", 10);
        subscriber_ = create_subscription<example_interfaces::msg::Int16>("number", 10, std::bind(&ActServerNode::subscriberCallback, this, std::placeholders::_1));
        timer = create_wall_timer(std::chrono::seconds(1), std::bind(&ActServerNode::publisherCallback, this));
        RCLCPP_INFO(get_logger(), "Activity Node has been started");
    }

private:
    rclcpp::TimerBase::SharedPtr timer;
    rclcpp::Service<example_interfaces::srv::SetBool>::SharedPtr server_;
    rclcpp::Publisher<example_interfaces::msg::Int16>::SharedPtr publisher_;
    rclcpp::Subscription<example_interfaces::msg::Int16>::SharedPtr subscriber_;

    int counter;

    void serverCallback(example_interfaces::srv::SetBool_Response::SharedPtr response, example_interfaces::srv::SetBool_Request::SharedPtr request)
    {
        response->success = true;
        response->message = "No reset";
        if (request->data)
        {
            response->success = true;
            response->message = "Reset done";
            counter = 0;
            RCLCPP_INFO(get_logger(), "Activity Server Node received Reset");
        }
    }

    void publisherCallback()
    {
        auto msg = example_interfaces::msg::Int16();
        msg.data = counter;
        publisher_->publish(msg);
    }

     void subscriberCallback(example_interfaces::msg::Int16::SharedPtr msg)
    {
        counter += msg->data;
        RCLCPP_INFO(get_logger(), "Activity Server Node received %d", counter);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<ActServerNode>();

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}