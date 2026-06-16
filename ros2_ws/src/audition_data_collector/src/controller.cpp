#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/bool.hpp"
#include <cmath>

class Controller : public rclcpp::Node
{
public:
    Controller() : Node("controller")
    {
        cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
            "/goal_pose", 10,
            std::bind(&Controller::goalCallback, this, std::placeholders::_1));

        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10,
            std::bind(&Controller::odomCallback, this, std::placeholders::_1));

        timer_ = create_wall_timer(
            std::chrono::milliseconds(50),
            std::bind(&Controller::controlLoop, this));

        goal_reached_pub = create_publisher<std_msgs::msg::Bool>("/goal_reached", 10);

        RCLCPP_INFO(get_logger(), "Controller node started");
    }

private:

    geometry_msgs::msg::PoseStamped goal_;
    nav_msgs::msg::Odometry odom_;

    bool has_goal_ = false;
    bool has_odom_ = false;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr goal_reached_pub;

    void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        goal_ = *msg;
        has_goal_ = true;
    }


    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        odom_ = *msg;
        has_odom_ = true;
    }

    void controlLoop()
    {
        if (!has_goal_ || !has_odom_) return;

        double dx = goal_.pose.position.x - odom_.pose.pose.position.x;
        double dy = goal_.pose.position.y - odom_.pose.pose.position.y;

        double distance = std::sqrt(dx*dx + dy*dy);
        double target_angle = std::atan2(dy, dx);

        double yaw = getYaw(odom_.pose.pose.orientation);
        double angle_error = normalize(target_angle - yaw);

        geometry_msgs::msg::Twist cmd;

        // Simple proportional controller
        if (std::abs(angle_error) > 0.3) {
            cmd.linear.x = 0.0;          
            cmd.angular.z = 2.0 * angle_error;
        } else {
            cmd.linear.x = 0.5 * distance;
            cmd.angular.z = 2.0 * angle_error;
        }

        RCLCPP_INFO(get_logger(), "distance = %.2f", distance);

        // Stop condition
        if (distance < 0.25 && std::abs(angle_error) < 0.2) {
            cmd.linear.x = 0.0;
            cmd.angular.z = 0.0;

            cmd_pub_->publish(cmd);

            auto reached = std_msgs::msg::Bool();
            reached.data = true;

            goal_reached_pub->publish(reached);

            has_goal_ = false;  // so next goal will be accepted
            return;
        }

        cmd_pub_->publish(cmd);

    }

    double getYaw(const geometry_msgs::msg::Quaternion &q)
    {
        return std::atan2(2.0*(q.w*q.z + q.x*q.y),
                          1.0 - 2.0*(q.y*q.y + q.z*q.z));
    }

    double normalize(double angle)
    {
        while (angle > M_PI) angle -= 2*M_PI;
        while (angle < -M_PI) angle += 2*M_PI;
        return angle;
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Controller>());
    rclcpp::shutdown();
    return 0;
}
