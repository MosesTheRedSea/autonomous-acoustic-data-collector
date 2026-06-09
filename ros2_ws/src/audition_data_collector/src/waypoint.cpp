#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/bool.hpp"
#include "audition_msgs/msg/collection_status.hpp"
#include <vector>
#include <string>
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <thread>


struct Waypoint {
  int id;
  std::string label;
  double x;
  double y;
  double yaw;
};


class Waypoint_Navigation : public rclcpp::Node
{
public:

  Waypoint_Navigation() : Node("waypoint"), current_index(0), waiting(false)
  {
    
    // I created parameters for the waypoint node
    declare_parameter("waypoints.labels", std::vector<std::string>{});
    declare_parameter("waypoints.x", std::vector<double>{});
    declare_parameter("waypoints.y", std::vector<double>{});
    declare_parameter("waypoints.yaw", std::vector<double>{});
  
    waypoint_pub = create_publisher<audition_msgs::msg::CollectionStatus>("/current_waypoint", 10);
    proceed_sub = create_subscription<std_msgs::msg::Bool>("/proceed_command", 10, std::bind(&Waypoint_Navigation::proceedCallback, this, std::placeholders::_1));
    goal_pub = create_publisher<geometry_msgs::msg::PoseStamped>("/goal_pose", 10);
    goal_reached_sub = create_subscription<std_msgs::msg::Bool>(
      "/goal_reached",
      10,
      std::bind(&Waypoint_Navigation::goalReachedCallback,
                this,
                std::placeholders::_1));
    

    loadWaypoints();

    current_index = 0;

    waiting = false;

    state = State::NAVIGATING;

    start_timer_ = create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&Waypoint_Navigation::startNavigation, this));

    RCLCPP_INFO(get_logger(), "Waypoint manager ready — %zu waypoints loaded", waypoints.size());
  }



private:
  enum class State { NAVIGATING, WAITING, DONE };

  State state = State::NAVIGATING;
  std::vector<Waypoint> waypoints;
  
  size_t current_index;
  bool waiting;
  rclcpp::Publisher<audition_msgs::msg::CollectionStatus>::SharedPtr waypoint_pub;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr proceed_sub;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr goal_reached_sub;
  rclcpp::TimerBase::SharedPtr start_timer_;

  void loadWaypoints()
  {
    auto labels = get_parameter("waypoints.labels").as_string_array();
    auto xs     = get_parameter("waypoints.x").as_double_array();
    auto ys     = get_parameter("waypoints.y").as_double_array();
    auto yaws   = get_parameter("waypoints.yaw").as_double_array();

    for (size_t i = 0; i < labels.size(); i++) {
      Waypoint wp;
      wp.id    = static_cast<int>(i + 1);
      wp.label = labels[i];
      wp.x     = xs[i];
      wp.y     = ys[i];
      wp.yaw   = yaws[i];
      waypoints.push_back(wp);

      if (
          labels.size() != xs.size() ||
          labels.size() != ys.size() ||
          labels.size() != yaws.size())
      {
        throw std::runtime_error("Waypoint parameter sizes mismatch");
      }

      RCLCPP_INFO(get_logger(), "Loaded waypoint [%d] — %s (%.2f, %.2f)",
                  wp.id, wp.label.c_str(), wp.x, wp.y);
    }
  }

  void startNavigation()
  {
    start_timer_->cancel();  // run only once

    RCLCPP_INFO(get_logger(), "Starting navigation...");
    sendNextGoal();
  }

  void goalReachedCallback(const std_msgs::msg::Bool::ConstSharedPtr msg) {

    if (!msg->data)
      return;
    
    // state = State::WAITING;
    // waiting = true;

    // RCLCPP_INFO(get_logger(),
    //         "Arrived at waypoint %zu/%zu",
    //         current_index + 1,
    //         waypoints.size());

    // publishCurrentWaypoint();

    if (state != State::NAVIGATING) {
      RCLCPP_WARN(get_logger(),
                  "Goal reached received but not navigating");
      return;
    }
  
    state = State::WAITING;
    waiting = true;
  
    RCLCPP_INFO(get_logger(),
                "Arrived at waypoint %zu/%zu",
                current_index + 1,
                waypoints.size());
  
    publishCurrentWaypoint();

  }

  void sendNextGoal()
  {

    if (current_index >= waypoints.size()) {
      RCLCPP_INFO(get_logger(), "All %zu waypoints complete — session finished",
                  waypoints.size());
      state = State::DONE;
      return;
    }

    auto & wp = waypoints[current_index];

    RCLCPP_INFO(get_logger(), "Navigating to waypoint [%d/%zu] — %s (%.2f, %.2f)",
                wp.id, waypoints.size(), wp.label.c_str(), wp.x, wp.y);

    geometry_msgs::msg::PoseStamped goal;

    goal.header.frame_id = "map";
    goal.header.stamp = this->now();

    goal.pose.position.x = wp.x;
    goal.pose.position.y = wp.y;
    goal.pose.position.z = 0.0;

    goal.pose.orientation.x = 0.0;
    goal.pose.orientation.y = 0.0;
    goal.pose.orientation.z = std::sin(wp.yaw / 2.0);
    goal.pose.orientation.w = std::cos(wp.yaw / 2.0);

    state = State::NAVIGATING;
    waiting = false;

    goal_pub->publish(goal);
    
  }

  void proceedCallback(const std_msgs::msg::Bool::ConstSharedPtr msg)
  {
      // if (!msg->data) {
      //     RCLCPP_WARN(get_logger(), "Abort signal received — stopping session");
      //     state = State::DONE;
      //     return;
      // }

      // if (state != State::WAITING) {
      //     RCLCPP_INFO(get_logger(), "Proceed received while not waiting");
      //     return;
      // }

      // current_index++;

      // RCLCPP_INFO(get_logger(), "Proceed received — moving to next waypoint %zu", current_index);
      // waiting = false;
          
      // sendNextGoal();

      if (!msg->data) {
        RCLCPP_WARN(get_logger(), "Abort signal received");
        state = State::DONE;
        return;
      }
    
      if (state != State::WAITING) {
        RCLCPP_WARN(get_logger(),
                    "Proceed received but not in WAITING state");
        return;
      }
    
      current_index++;
    
      RCLCPP_INFO(get_logger(),
                  "Proceeding to waypoint %zu",
                  current_index + 1);
    
      sendNextGoal();
      
  }

  void publishCurrentWaypoint()
  {
    auto & wp = waypoints[current_index];
    auto status = audition_msgs::msg::CollectionStatus();
    status.current_waypoint_id    = wp.id;
    status.current_waypoint_label = wp.label;
    status.state                  = "ARRIVED";
    status.total_waypoints        = static_cast<int>(waypoints.size());
    status.completed_waypoints    = static_cast<int>(current_index);
    status.recording = true;
    waypoint_pub->publish(status);
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Waypoint_Navigation>());
  rclcpp::shutdown();
  return 0;
}

