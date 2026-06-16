#include <memory>
#include <string>
#include <cstdio>
#include <cstring>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

#define MS16_S_XS 0x90
#define MS16_S_ZS 0x92

class HardwareBridge : public rclcpp::Node
{
public:
    HardwareBridge()
    : Node("hardware_bridge")
    {
        serial_port_ = open("/dev/ttyUSB0", O_RDWR);

        if (serial_port_ < 0) {
            RCLCPP_ERROR(
                this->get_logger(),
                "Failed to open serial port");
            return;
        }

        struct termios tty;

        memset(&tty, 0, sizeof tty);

        if (tcgetattr(serial_port_, &tty) != 0) {
            RCLCPP_ERROR(
                this->get_logger(),
                "tcgetattr failed");
            return;
        }

        cfsetispeed(&tty, B115200);
        cfsetospeed(&tty, B115200);

        tty.c_cflag |= (CLOCAL | CREAD);
        tty.c_cflag &= ~CSIZE;
        tty.c_cflag |= CS8;
        tty.c_cflag &= ~PARENB;
        tty.c_cflag &= ~CSTOPB;
        tty.c_cflag &= ~CRTSCTS;

        tty.c_lflag = 0;
        tty.c_oflag = 0;
        tty.c_iflag = 0;

        tty.c_cc[VMIN] = 0;
        tty.c_cc[VTIME] = 10;

        tcsetattr(serial_port_, TCSANOW, &tty);

        sendRegister(0x10, 0x0F);

        subscription_ =
            this->create_subscription<geometry_msgs::msg::Twist>(
                "/cmd_vel",
                10,
                std::bind(
                    &HardwareBridge::cmdVelCallback,
                    this,
                    std::placeholders::_1));

        RCLCPP_INFO(
            this->get_logger(),
            "Hardware bridge started");
    }

    ~HardwareBridge()
    {
        if (serial_port_ >= 0) {
            close(serial_port_);
        }
    }

private:
    void sendRegister(uint8_t addr, int16_t value)
    {
        char buffer[64];

        snprintf(
            buffer,
            sizeof(buffer),
            "w %02x %04x\n",
            addr,
            (uint16_t)value);

        write(serial_port_, buffer, strlen(buffer));

        RCLCPP_INFO(
            this->get_logger(),
            "Sent: %s",
            buffer);
    }

    void cmdVelCallback(const geometry_msgs::msg::Twist::ConstSharedPtr msg)
    {
        double linear  = msg->linear.x;
        double angular = msg->angular.z;

        // Convert to mm/s and mrad/s
        int16_t xs = static_cast<int16_t>(linear  * 1000.0);   // m/s → mm/s
        int16_t zs = static_cast<int16_t>(angular * 1000.0);   // rad/s → mrad/s

        sendRegister(0x90, xs);  // MS16_S_XS forward/back
        sendRegister(0x94, zs);  // MS16_S_ZS rotation ← was 0x92, WRONG
    }

    // void cmdVelCallback(
    //     const geometry_msgs::msg::Twist::ConstSharedPtr msg)
    // {
    //     double linear  = msg->linear.x;
    //     double angular = msg->angular.z;

    //     const double wheel_separation = 0.35;  // metres — from your URDF

    //     double left  = linear - (angular * wheel_separation / 2.0);
    //     double right = linear + (angular * wheel_separation / 2.0);

    //     char buffer[64];
    //     snprintf(buffer, sizeof(buffer), "L%.3fR%.3f\n", left, right);

    //     write(serial_port_, buffer, strlen(buffer));

    //     RCLCPP_INFO(get_logger(), "Sent: %s", buffer);
    // }

    int serial_port_;

    rclcpp::Subscription<
        geometry_msgs::msg::Twist>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    rclcpp::spin(
        std::make_shared<HardwareBridge>());

    rclcpp::shutdown();

    return 0;
}
