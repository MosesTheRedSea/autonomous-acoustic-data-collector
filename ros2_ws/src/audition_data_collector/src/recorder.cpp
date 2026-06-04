#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "rosbag2_cpp/writers/sequential_writer.hpp"
#include "rosbag2_cpp/writer.hpp"
#include "rosbag2_cpp/storage_options.hpp"
#include "rosbag2_cpp/converter_options.hpp"
#include "rosbag2_storage/serialized_bag_message.hpp"
#include "rclcpp/serialization.hpp"
#include <cstring>
#include <memory>
#include <string>
#include <ctime>

class Recorder : public rclcpp::Node
{
public:
  Recorder() : Node("recorder"), recording(false)
  {
    declare_parameter("output_dir", "/tmp/bags");

    trigger_sub = create_subscription<std_msgs::msg::Bool>(
      "/start_record", 10,
      std::bind(&Recorder::triggerCallback, this, std::placeholders::_1));

    depth_sub = create_subscription<sensor_msgs::msg::Image>(
      "/camera/depth/image_raw", 10,
      std::bind(&Recorder::depthCallback, this, std::placeholders::_1));

    color_sub = create_subscription<sensor_msgs::msg::Image>(
      "/camera/color/image_raw", 10,
      std::bind(&Recorder::colorCallback, this, std::placeholders::_1));

    scan_sub = create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", 10,
      std::bind(&Recorder::scanCallback, this, std::placeholders::_1));

    pointcloud_sub = create_subscription<sensor_msgs::msg::PointCloud2>(
      "/camera/depth/points", 10,
      std::bind(&Recorder::pointcloudCallback, this, std::placeholders::_1));
  
    recording_complete_pub = create_publisher<std_msgs::msg::Bool>("/recording_complete", 10);

    RCLCPP_INFO(get_logger(), "Recorder ready");


  }

private:
  bool recording;
  std::unique_ptr<rosbag2_cpp::Writer> writer;

  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr trigger_sub;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr color_sub;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub;

  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr recording_complete_pub;

  void triggerCallback(const std_msgs::msg::Bool::ConstSharedPtr msg)
  {
    if (msg->data && !recording) startRecording();
    else if (!msg->data && recording) stopRecording();
  }

  rosbag2_storage::TopicMetadata makeTopicMetadata(
    const std::string & topic_name,
    const std::string & type_name)
  {
    rosbag2_storage::TopicMetadata metadata;
  
    metadata.name = topic_name;
    metadata.type = type_name;
    metadata.serialization_format = "cdr";
    metadata.offered_qos_profiles = "";
  
    return metadata;
  }

  void startRecording()
  {
    std::string dir = get_parameter("output_dir").as_string();
    std::string path = dir + "/session_" + std::to_string(std::time(nullptr));

    rosbag2_cpp::StorageOptions storage_options;
    storage_options.uri = path;
    storage_options.storage_id = "sqlite3";

    rosbag2_cpp::ConverterOptions converter_options;
    converter_options.input_serialization_format = "cdr";
    converter_options.output_serialization_format = "cdr";

    writer = std::make_unique<rosbag2_cpp::Writer>(
      std::make_unique<rosbag2_cpp::writers::SequentialWriter>());
    writer->open(storage_options, converter_options);

    writer->create_topic(makeTopicMetadata(
      "/camera/depth/image_raw",
      "sensor_msgs/msg/Image"));
    
    writer->create_topic(makeTopicMetadata(
      "/camera/color/image_raw",
      "sensor_msgs/msg/Image"));
    
    writer->create_topic(makeTopicMetadata(
      "/scan",
      "sensor_msgs/msg/LaserScan"));
    
    writer->create_topic(makeTopicMetadata(
      "/camera/depth/points",
      "sensor_msgs/msg/PointCloud2"));

    recording = true;
    RCLCPP_INFO(get_logger(), "Recording started");
  }

  void stopRecording()
  {
    writer.reset();
    recording = false;

    auto msg = std_msgs::msg::Bool():
    msg.data = true;

    recording_complete_pub->publish(msg);

    RCLCPP_INFO(get_logger(), "Recording stopped");
  }

  template<typename T>
  void writeMessage(const typename T::ConstSharedPtr msg, const std::string & topic)
  {
    if (!recording || !writer) {
      return;
    }

    // Serialize ROS2 message
    rclcpp::Serialization<T> serializer;
    rclcpp::SerializedMessage serialized_msg;

    serializer.serialize_message(msg.get(), &serialized_msg);

    // Create bag message
    auto bag_msg =
      std::make_shared<rosbag2_storage::SerializedBagMessage>();

    bag_msg->topic_name = topic;
    bag_msg->time_stamp = this->now().nanoseconds();

    // Copy serialized data
    auto & rcl_msg = serialized_msg.get_rcl_serialized_message();

    bag_msg->serialized_data =
      std::make_shared<rcutils_uint8_array_t>();

    *bag_msg->serialized_data =
      rcutils_get_zero_initialized_uint8_array();

    rcutils_allocator_t allocator =
      rcutils_get_default_allocator();

    auto ret = rcutils_uint8_array_init(
      bag_msg->serialized_data.get(),
      rcl_msg.buffer_length,
      &allocator);

    if (ret != RCUTILS_RET_OK) {
      RCLCPP_ERROR(
        get_logger(),
        "Failed to initialize serialized data");
      return;
    }

    std::memcpy(
      bag_msg->serialized_data->buffer,
      rcl_msg.buffer,
      rcl_msg.buffer_length);

    bag_msg->serialized_data->buffer_length =
      rcl_msg.buffer_length;

    writer->write(bag_msg);
  }

  void depthCallback(const sensor_msgs::msg::Image::ConstSharedPtr msg)
  { writeMessage<sensor_msgs::msg::Image>(msg, "/camera/depth/image_raw"); }

  void colorCallback(const sensor_msgs::msg::Image::ConstSharedPtr msg)
  { writeMessage<sensor_msgs::msg::Image>(msg, "/camera/color/image_raw"); }

  void scanCallback(const sensor_msgs::msg::LaserScan::ConstSharedPtr msg)
  { writeMessage<sensor_msgs::msg::LaserScan>(msg, "/scan"); }

  void pointcloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
  { writeMessage<sensor_msgs::msg::PointCloud2>(msg, "/camera/depth/points"); }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Recorder>());
  rclcpp::shutdown();
  return 0;
}

// ── Sensor setup notes ────────────────────────────────────────
// 1. Install your sensor driver
//    Intel RealSense:  sudo apt install ros-humble-realsense2-camera
//    Orbbec Astra:     sudo apt install ros-humble-astra-camera
//    ReSpeaker mic:    pip install pyaudio
//
// 2. Check what topic your driver publishes on
//    ros2 topic list
//    ros2 topic echo /camera/depth/image_raw --once
//
// 3. Update the topic names above to match your driver


// 1. Install driver - sensor has ros2 driver package

/*
# Intel RealSense
sudo apt install ros-humble-realsense2-camera

# Orbbec Astra
sudo apt install ros-humble-astra-camera

# ReSpeaker mic array
pip install pyaudio
git clone https://github.com/nickoala/respeaker_ros2.git
*/

// 2. Check which topic it publishes on - once driver is running

/*
ros2 topic list
ros2 topic echo /camera/depth/image_raw --once
*/

// 3. Update the topic names in recorder_node.cpp
/*
// change this
depth_sub = create_subscription<sensor_msgs::msg::Image>(
  "/camera/depth/image_raw", 10, ...);

// to whatever your driver publishes on
depth_sub = create_subscription<sensor_msgs::msg::Image>(
  "/your_actual_topic_name", 10, ...);

*/
