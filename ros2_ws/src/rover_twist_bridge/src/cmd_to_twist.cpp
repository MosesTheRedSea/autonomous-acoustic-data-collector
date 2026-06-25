#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

class CmdToTwist : public rclcpp::Node
{
public:
    CmdToTwist() : Node("cmd_to_twist")
    {
        pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
            "/rover_twist_odo", 10);

        sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel",
            10,
            std::bind(&CmdToTwist::callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "cmd_to_twist bridge started");
    }

private:
    void callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        pub_->publish(*msg);
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CmdToTwist>());
    rclcpp::shutdown();
    return 0;
}