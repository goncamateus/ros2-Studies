#include "rclcpp/rclcpp.hpp"
#include "gonca_interfaces/srv/battery_status.hpp"

class LEDClientNode : public rclcpp::Node
{
public:
    LEDClientNode() : Node("LED_client_node"), ledToSet(3), batteryFull(true), changedState(false)
    {
        thread_ = std::thread(std::bind(&LEDClientNode::clientCallBack, this));
        RCLCPP_INFO(get_logger(), "LED client node has been started");
    }
    void clientCallBack()
    {
        auto client = create_client<gonca_interfaces::srv::BatteryStatus>("set_led");
        while (!client->wait_for_service(std::chrono::milliseconds(500)))
        {
            RCLCPP_WARN(get_logger(), "Client waiting for server...");
        }
        lastTime = std::chrono::duration_cast<std::chrono::seconds>(
            std::chrono::system_clock::now().time_since_epoch());
        while (true)
        {
            checkBatteryState();
            if (changedState)
            {
                changedState = false;
                auto request = std::make_shared<gonca_interfaces::srv::BatteryStatus::Request>();
                request->led = ledToSet;
                request->state = !batteryFull;

                auto future = client->async_send_request(request);
                try
                {
                    auto response = future.get();
                    if (response->success)
                        RCLCPP_INFO(get_logger(), "LED %ld was set %d", static_cast<long>(request->led), request->state);
                    else
                        RCLCPP_ERROR(get_logger(), "Error on setting LED");
                }
                catch (const std::exception &e)
                {
                    RCLCPP_ERROR(get_logger(), e.what());
                }
            }
        }
    }

private:
    int ledToSet;
    bool batteryFull, changedState;
    std::chrono::seconds lastTime;
    std::thread thread_;
    rclcpp::TimerBase::SharedPtr timer_;

    void checkBatteryState()
    {
        auto timeNow = std::chrono::duration_cast<std::chrono::seconds>(
            std::chrono::system_clock::now().time_since_epoch());
        auto timeDiff = timeNow - lastTime;
        if (batteryFull && timeDiff >= std::chrono::seconds(4))
        {
            lastTime = timeNow;
            batteryFull = false;
            changedState = true;
        }
        else if (!batteryFull && timeDiff >= std::chrono::seconds(6))
        {
            lastTime = timeNow;
            batteryFull = true;
            changedState = true;
        }
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<LEDClientNode>();

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}