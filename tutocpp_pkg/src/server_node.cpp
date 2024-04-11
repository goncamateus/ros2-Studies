#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"

class ServerNode : public rclcpp::Node
{
public:
    ServerNode() : Node("server_node")
    {
        server_ = create_service<example_interfaces::srv::AddTwoInts>("add_ints", std::bind(&ServerNode::client_callback, this, std::placeholders::_1, std::placeholders::_2));
        RCLCPP_INFO(get_logger(), "Server Node has been started");
    }

private:
    rclcpp::Service<example_interfaces::srv::AddTwoInts>::SharedPtr server_;
    void client_callback(example_interfaces::srv::AddTwoInts::Request::SharedPtr request, example_interfaces::srv::AddTwoInts::Response::SharedPtr response)
    {
        response->sum = request->a + request->b;
        RCLCPP_INFO(get_logger(), "%ld + %ld = %ld", request->a, request->b, response->sum);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<ServerNode>();

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}