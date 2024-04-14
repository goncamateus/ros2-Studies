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
        bool ex = abs(x - rhs.x) < 0.1;
        bool ey = abs(y - rhs.y) < 0.1;
        return ex && ey;
    }
};

struct Turtle
{
    std::string name;
    TurtlePose pose;
    bool alive;
};

class TurtleSpawnNode : public rclcpp::Node
{
public:
    TurtleSpawnNode() : Node("turtle_spawn"), turtleCounter(2)
    {
        this->actualPose = TurtlePose();
        this->targets.clear();
        this->poseSub_ = create_subscription<turtlesim::msg::Pose>("/turtle1/pose", 10, std::bind(&TurtleSpawnNode::playerCallback, this, std::placeholders::_1));
        this->posePublisher_ = create_publisher<turtlesim::msg::Pose>("target_turtle", 10);
        this->pubTimer_ = create_wall_timer(std::chrono::milliseconds(250), std::bind(&TurtleSpawnNode::targetCallback, this));
        this->spawnTimer_ = create_wall_timer(std::chrono::milliseconds(500), std::bind(&TurtleSpawnNode::spawnCallback, this));
        RCLCPP_INFO(get_logger(), "TurtleControl node started");
    }

private:
    rclcpp::TimerBase::SharedPtr pubTimer_, spawnTimer_, killTimer_;
    rclcpp::Publisher<turtlesim::msg::Pose>::SharedPtr posePublisher_;
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr poseSub_;
    TurtlePose actualPose;
    std::vector<Turtle> targets;
    std::vector<std::shared_ptr<std::thread>> spawnThreads;
    std::vector<std::shared_ptr<std::thread>> killThreads;
    int turtleCounter;

    double getRandom(double low, double high)
    {
        return low + static_cast<double>(rand()) / (static_cast<double>(RAND_MAX / (high - low)));
    }

    Turtle createRandomTurtle()
    {
        auto pose = getRandomPose();
        auto name = "turtle" + std::to_string(this->turtleCounter);
        Turtle turtle = {name, pose, true};
        this->turtleCounter++;
        return turtle;
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

    void spawnCallback()
    {
        auto newTurtle = createRandomTurtle();
        spawnThreads.push_back(
            std::make_shared<std::thread>(
                std::bind(&TurtleSpawnNode::callSpawnService, this, newTurtle)));
    }

    void callSpawnService(Turtle newTurtle)
    {
        auto client = create_client<turtlesim::srv::Spawn>("spawn");
        while (!client->wait_for_service(std::chrono::milliseconds(100)))
        {
            RCLCPP_WARN(get_logger(), "Spawn client waiting for server");
        }

        auto request = std::make_shared<turtlesim::srv::Spawn::Request>();
        request->x = newTurtle.pose.x;
        request->y = newTurtle.pose.y;
        request->theta = newTurtle.pose.theta;
        request->name = newTurtle.name;

        auto future = client->async_send_request(request);

        try
        {
            auto response = future.get();
            this->targets.push_back(newTurtle);
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(get_logger(), e.what());
        }
    }

    void killTurtle()
    {
        if (this->targets.size() > 0)
        {
            if (this->targets[0].pose == this->actualPose && this->targets[0].alive)
            {
                this->targets[0].alive = false;
                killThreads.push_back(
                    std::make_shared<std::thread>(
                        std::bind(&TurtleSpawnNode::callKillService, this, this->targets[0].name)));
            }
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
            this->targets.erase(this->targets.begin());
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(get_logger(), e.what());
        }
    }

    void targetCallback()
    {
        if (this->targets.size() > 0)
        {
            auto msg = turtlesim::msg::Pose();
            msg.x = this->targets[this->targets.size() - 1].pose.x;
            msg.y = this->targets[this->targets.size() - 1].pose.y;
            msg.theta = this->targets[this->targets.size() - 1].pose.theta;
            msg.linear_velocity = this->targets[this->targets.size() - 1].pose.linear_velocity;
            msg.angular_velocity = this->targets[this->targets.size() - 1].pose.angular_velocity;
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