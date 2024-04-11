#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"

class ClientNode : public rclcpp::Node
{
public:
    ClientNode() : Node("client_node")
    {
        thread1_ = std::thread(std::bind(&ClientNode::callServer, this, 6, 7));
        RCLCPP_INFO(get_logger(), "Client Node has been started");
    }

    void callServer(int a, int b)
    {
        auto client = create_client<example_interfaces::srv::AddTwoInts>("add_ints");
        while (!client->wait_for_service(std::chrono::seconds(1)))
        {
            RCLCPP_WARN(get_logger(), "Waiting for the server to be up...");
        }
        auto request = std::make_shared<example_interfaces::srv::AddTwoInts::Request>();
        request->a = a;
        request->b = b;

        auto future = client->async_send_request(request);
        try
        {
            auto response = future.get();
            RCLCPP_INFO(get_logger(), "%ld + %ld = %ld", request->a, request->b, response->sum);
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(get_logger(), e.what());
        }
    }
private:
    std::thread thread1_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<ClientNode>();

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}