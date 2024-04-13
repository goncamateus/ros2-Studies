#include "rclcpp/rclcpp.hpp"
#include "turtlesim/msg/pose.hpp"
#include "turtlesim/srv/spawn.hpp"
#include "turtlesim/srv/kill.hpp"
#include "std_srvs/srv/empty.hpp"

struct TurtlePose
{
    double x, y, theta, linear_velocity, angular_velocity;
    bool operator==(const TurtlePose &rhs) const
    {
        bool ex = x == rhs.x;
        bool ey = y == rhs.y;
        return ex && ey;
    }
};

class TurtleSpawnNode : public rclcpp::Node
{
public:
    TurtleSpawnNode() : Node("turtle_spawn"), targetCounter(2)
    {
        actualPose = TurtlePose();
        targetsPose.clear();
        targetsPose.push_back(getRandomPose());
        poseSub_ = create_subscription<turtlesim::msg::Pose>("/turtle1/pose", 10, std::bind(&TurtleSpawnNode::playerCallback, this, std::placeholders::_1));
        posePublisher_ = create_publisher<turtlesim::msg::Pose>("target_turtle", 10);
        pubTimer_ = create_wall_timer(std::chrono::milliseconds(100), std::bind(&TurtleSpawnNode::targetCallback, this));
        spawnTimer_ = create_wall_timer(std::chrono::milliseconds(500), std::bind(&TurtleSpawnNode::spawnCallback, this));
    }

private:
    rclcpp::TimerBase::SharedPtr pubTimer_, spawnTimer_;
    rclcpp::Publisher<turtlesim::msg::Pose>::SharedPtr posePublisher_;
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr poseSub_;
    TurtlePose actualPose;
    std::vector<TurtlePose> targetsPose;
    std::vector<std::shared_ptr<std::thread>> spawnThreads;
    std::vector<std::shared_ptr<std::thread>> killThreads;
    int targetCounter;

    double getRandom(double low, double high)
    {
        return low + static_cast<double>(rand()) / (static_cast<double>(RAND_MAX / (high - low)));
    }

    TurtlePose getRandomPose()
    {
        TurtlePose randPose = TurtlePose();
        randPose.x = getRandom(0.0, 11.0);
        randPose.y = getRandom(0.0, 11.0);
        randPose.theta = getRandom(-M_PI, M_PI);
        randPose.linear_velocity = 0.0;
        randPose.angular_velocity = 0.0;
        return randPose;
    }

    void killTurtle()
    {
        if (targetsPose.size() > 0)
        {
            int wichTurtle = targetCounter - targetsPose.size() + 1;
            std::string turtleName = "turtle" + std::to_string(wichTurtle);
            killThreads.push_back(
                std::make_shared<std::thread>(
                    std::bind(&TurtleSpawnNode::callKillService, this, turtleName)));
        }
    }

    void callKillService(std::string turtleName)
    {
        auto client = create_client<turtlesim::srv::Kill>("kill");
        while (!client->wait_for_service(std::chrono::milliseconds(100)))
        {
            RCLCPP_WARN(get_logger(), "Kill client waiting for server");
        }

        auto request = std::make_shared<turtlesim::srv::Kill::Request>();
        request->name = turtleName;

        auto future = client->async_send_request(request);
        try
        {
            auto response = future.get();
            targetsPose.erase(targetsPose.begin());
            RCLCPP_INFO(get_logger(), "%s has been killed succesfully", turtleName.c_str());
        }
        catch (const std::exception &e)
        {
            targetCounter++;
            RCLCPP_ERROR(get_logger(), e.what());
        }
    }

    void spawnCallback()
    {
        TurtlePose randPose(getRandomPose());
        targetsPose.push_back(randPose);
        spawnThreads.push_back(
            std::make_shared<std::thread>(
                std::bind(&TurtleSpawnNode::callSpawnService, this, randPose, targetCounter)));
        targetCounter++;
    }

    void callSpawnService(TurtlePose pose, int targetCounter)
    {
        auto client = create_client<turtlesim::srv::Spawn>("spawn");
        while (!client->wait_for_service(std::chrono::milliseconds(100)))
        {
            RCLCPP_WARN(get_logger(), "Spawn client waiting for server");
        }

        auto request = std::make_shared<turtlesim::srv::Spawn::Request>();
        request->x = pose.x;
        request->y = pose.y;
        request->theta = pose.theta;
        request->name = "turtle" + std::to_string(targetCounter);

        auto future = client->async_send_request(request);

        try
        {
            auto response = future.get();
            RCLCPP_INFO(get_logger(), "Turtle %s has been spawned!! GO GET IT!!", response->name.c_str());
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(get_logger(), e.what());
        }
    }

    void targetCallback()
    {
        if (targetsPose.size() > 0)
        {
            auto msg = turtlesim::msg::Pose();
            msg.x = targetsPose[0].x;
            msg.y = targetsPose[0].y;
            msg.theta = targetsPose[0].theta;
            msg.linear_velocity = targetsPose[0].linear_velocity;
            msg.angular_velocity = targetsPose[0].angular_velocity;
            posePublisher_->publish(msg);
        }
    }

    void playerCallback(turtlesim::msg::Pose::SharedPtr msg)
    {
        actualPose.x = msg->x;
        actualPose.y = msg->y;
        actualPose.theta = msg->theta;
        actualPose.linear_velocity = msg->linear_velocity;
        actualPose.angular_velocity = msg->angular_velocity;
        bool isOnTarget = actualPose == targetsPose[0];
        if (isOnTarget)
            killTurtle();
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<TurtleSpawnNode>();

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}