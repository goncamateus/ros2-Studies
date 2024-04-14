#include "rclcpp/rclcpp.hpp"
#include "turtlesim/msg/pose.hpp"
#include "turtlesim/srv/teleport_absolute.hpp"
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
class TurtleFollowNode : public rclcpp::Node
{
public:
  TurtleFollowNode() : Node("turtle_follow")
  {
    playerSub_ = create_subscription<turtlesim::msg::Pose>(
        "/turtle1/pose", 10,
        std::bind(&TurtleFollowNode::playerUpdate, this,
                  std::placeholders::_1));
    targetSub_ = create_subscription<turtlesim::msg::Pose>(
        "target_turtle", 10,
        std::bind(&TurtleFollowNode::goToTarget, this, std::placeholders::_1));
    RCLCPP_INFO(get_logger(), "Turtle follow node started");
  }

private:
  rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr playerSub_, targetSub_;
  std::vector<std::shared_ptr<std::thread>> goToThreads, clearThreads;
  TurtlePose playerPose;

  void playerUpdate(turtlesim::msg::Pose::SharedPtr msg)
  {
    playerPose.x = msg->x;
    playerPose.y = msg->y;
    playerPose.theta = msg->theta;
    playerPose.linear_velocity = msg->linear_velocity;
    playerPose.angular_velocity = msg->angular_velocity;
  }

  void goToTarget(turtlesim::msg::Pose::SharedPtr msg)
  {
    TurtlePose targetPose;
    targetPose.x = msg->x;
    targetPose.y = msg->y;
    targetPose.theta = msg->theta;
    targetPose.linear_velocity = msg->linear_velocity;
    targetPose.angular_velocity = msg->angular_velocity;
    goToThreads.push_back(std::make_shared<std::thread>(
        &TurtleFollowNode::goToTargetThread, this, targetPose));
    clearThreads.push_back(std::make_shared<std::thread>(
        &TurtleFollowNode::clearThread, this));
  }

  void goToTargetThread(TurtlePose targetPose)
  {
    auto client = create_client<turtlesim::srv::TeleportAbsolute>("turtle1/teleport_absolute");
    while (!client->wait_for_service(std::chrono::milliseconds(100)))
    {
      RCLCPP_WARN(get_logger(), "Follow client waiting for server");
    }

    auto request = std::make_shared<turtlesim::srv::TeleportAbsolute::Request>();
    request->x = targetPose.x;
    request->y = targetPose.y;
    request->theta = targetPose.theta;

    auto future = client->async_send_request(request);
    try
    {
      auto response = future.get();
    }
    catch (const std::exception &e)
    {
      RCLCPP_ERROR(get_logger(), e.what());
    }
  }

  void clearThread()
  {
    auto client = create_client<std_srvs::srv::Empty>("clear");
    while (!client->wait_for_service(std::chrono::milliseconds(100)))
    {
      RCLCPP_WARN(get_logger(), "Follow client waiting for server");
    }

    auto request = std::make_shared<std_srvs::srv::Empty::Request>();

    auto future = client->async_send_request(request);
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<TurtleFollowNode>();

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}