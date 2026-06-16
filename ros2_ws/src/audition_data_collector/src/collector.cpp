#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "audition_msgs/msg/collection_status.hpp"

class Collector : public rclcpp::Node
{
public:

  Collector() : Node("collector"), state(State::IDLE) {

    start_recording_pub = create_publisher<std_msgs::msg::Bool>("/start_record", 10);
    current_status_pub = create_publisher<audition_msgs::msg::CollectionStatus>("/collection_status", 10);

    proceed_pub = create_publisher<std_msgs::msg::Bool>("/proceed_command", 10);
    
    // proceed_sub = create_subscription<std_msgs::msg::Bool>("/proceed_command", 10, std::bind(&Collector::proceedCallback, this, std::placeholders::_1));

    waypoint_sub = create_subscription<audition_msgs::msg::CollectionStatus>("/current_waypoint", 10, std::bind(&Collector::waypointCallback, this, std::placeholders::_1));
    
    recording_complete_sub = create_subscription<std_msgs::msg::Bool>("/recording_complete", 10, std::bind(&Collector::recordingCompleteCallback, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "Collector node ready — waiting for waypoint arrival");
  }

private:

  enum class State { IDLE, RECORDING};

  State state;
  audition_msgs::msg::CollectionStatus::ConstSharedPtr current_waypoint;

  // These are publisher I made for recording, current_status and whether we should proceed to the next occlusion
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr start_recording_pub;

  rclcpp::Publisher<audition_msgs::msg::CollectionStatus>::SharedPtr current_status_pub;

  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr proceed_pub;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr proceed_sub;

  rclcpp::Subscription<audition_msgs::msg::CollectionStatus>::SharedPtr waypoint_sub;

  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr recording_complete_sub;

  rclcpp::TimerBase::SharedPtr record_timer_;

  void waypointCallback(const audition_msgs::msg::CollectionStatus::ConstSharedPtr msg)
  {

    // Robot is currently doing something 
    if (state != State::IDLE) {
      RCLCPP_WARN(get_logger(), "Waypoint arrived but not in IDLE state — ignoring");
      return;
    }

    current_waypoint = msg;
    state = State::RECORDING;

    RCLCPP_INFO(get_logger(), "Arrived at waypoint [%s] — starting recording",
                msg->current_waypoint_label.c_str());


    auto trigger = std_msgs::msg::Bool();
    trigger.data = true;

    start_recording_pub->publish(trigger);

    record_timer_ = create_wall_timer(
      std::chrono::seconds(10),  // or whatever duration you want
      std::bind(&Collector::onRecordTimeout, this));

    publishStatus();

  }

  // New method:
  void onRecordTimeout() {
    record_timer_->cancel();
    auto trigger = std_msgs::msg::Bool();
    trigger.data = false;
    start_recording_pub->publish(trigger);  // tells recorder to stop → triggers recording_complete
  }

  void recordingCompleteCallback(const std_msgs::msg::Bool::ConstSharedPtr msg)
  {

    if (msg->data && state == State::RECORDING) {
        RCLCPP_INFO(get_logger(),
        "Recording complete — waiting for operator proceed command");

        auto trigger = std_msgs::msg::Bool();
        trigger.data = false;
        start_recording_pub->publish(trigger);

        state = State::IDLE;

        publishStatus();
        // RCLCPP_INFO(get_logger(), "Acoustic recording complete — stopping bag and proceeding");

        // auto trigger = std_msgs::msg::Bool();
        // trigger.data = false;
        // start_recording_pub->publish(trigger);
        // state = State::IDLE;
        // auto proceed = std_msgs::msg::Bool();
        // proceed.data = true;
        // //proceed_pub->publish(proceed);
        
        // publishStatus();
    }
  }

  /* 
  void proceedCallback(const std_msgs::msg::Bool::ConstSharedPtr msg)
  {
    // If we're not given the message to proceed 
    if (!msg->data || state != State::RECORDING) {
      return;
    }

    RCLCPP_INFO(get_logger(), "Proceed received — stopping recording and moving on");

    auto trigger = std_msgs::msg::Bool();
    trigger.data = false;
    start_recording_pub->publish(trigger);

    state = State::IDLE;
    publishStatus();
  }
  */

  // This updates robot information and publishes it so we can grab that info
  void publishStatus()
  {

    auto status = audition_msgs::msg::CollectionStatus();

    status.state = (state == State::RECORDING) ? "RECORDING" : "IDLE";
    status.recording = (state == State::RECORDING); // true or false

    if (current_waypoint) {
      status.current_waypoint_id = current_waypoint->current_waypoint_id;
      status.current_waypoint_label = current_waypoint->current_waypoint_label;
    }

    current_status_pub->publish(status);
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Collector>());
  rclcpp::shutdown();
  return 0;
}
