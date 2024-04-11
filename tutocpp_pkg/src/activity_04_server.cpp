#include "rclcpp/rclcpp.hpp"
#include "gonca_interfaces/msg/led3_states.hpp"
#include "gonca_interfaces/srv/battery_status.hpp"

class LEDServerNode : public rclcpp::Node
{
public:
    LEDServerNode() : Node("LED_server_node"), leds{false, false, false}
    {
        server_ = create_service<gonca_interfaces::srv::BatteryStatus>("set_led", std::bind(&LEDServerNode::serverCallback, this, std::placeholders::_2, std::placeholders::_1));
        publisher_ = create_publisher<gonca_interfaces::msg::Led3States>("led_states", 10);
        timer_ = create_wall_timer(std::chrono::seconds(4), std::bind(&LEDServerNode::publishLEDStates, this));
        RCLCPP_INFO(get_logger(), "LED server node has been started");
    }

private:
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Service<gonca_interfaces::srv::BatteryStatus>::SharedPtr server_;
    rclcpp::Publisher<gonca_interfaces::msg::Led3States>::SharedPtr publisher_;
    bool leds[3];
    void serverCallback(
        gonca_interfaces::srv::BatteryStatus_Response::SharedPtr response,
        gonca_interfaces::srv::BatteryStatus_Request::SharedPtr request)
    {
        if (request->led < 1 || request->led > 3)
        {
            response->success = false;
            return;
        }
        else
        {
            leds[request->led - 1] = request->state;
            response->success = true;
            publishLEDStates();
        }
    }
    void publishLEDStates()
    {
        auto message = gonca_interfaces::msg::Led3States();
        message.states.clear();
        message.states.push_back(leds[0]);
        message.states.push_back(leds[1]);
        message.states.push_back(leds[2]);
        publisher_->publish(message);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<LEDServerNode>();

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}