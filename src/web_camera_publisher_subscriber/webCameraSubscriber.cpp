#include "rclcpp/rclcpp.hpp"
#include "rclcpp/serialization.hpp"
#include "rclcpp/serialized_message.hpp"

#include "rosbag2_cpp/converter_options.hpp"
#include "rosbag2_cpp/storage_options.hpp"
#include "rosbag2_cpp/writer.hpp"

class FrameSubscriberRecorder : public rclcpp::Node {
public:
  FrameSubscriberRecorder(
    const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
      : Node("frame_subscriber_recorder", options) {
    writer_ = std::make_unique<rosbag2_cpp::Writer>();

    rosbag2_storage::StorageOptions storage_options;
    storage_options.uri =
      std::to_string(this->get_clock()->now().nanoseconds()) + "_bag";
    storage_options.storage_id = "mcap";

    rosbag2_cpp::ConverterOptions converter_options{
      rmw_get_serialization_format(), rmw_get_serialization_format()};

    writer_->open(storage_options, converter_options);

    writer_->create_topic({"/camera/image_raw", "sensor_msgs/msg/Image",
                           rmw_get_serialization_format(), ""});
    auto qos = rclcpp::SensorDataQoS().keep_last(1).best_effort();
    cameraSubscriber_ = this->create_generic_subscription(
      "/camera/image_raw", "sensor_msgs/msg/Image", qos,
      [this](std::shared_ptr<rclcpp::SerializedMessage> msg) {
        writer_->write(msg, "/camera/image_raw", "sensor_msgs/msg/Image",
                       this->now());
      });

    stringSubscriber_ = this->create_generic_subscription(
      "/str_message", "std_msgs/msg/String", qos,
      [this](std::shared_ptr<rclcpp::SerializedMessage> msg) {
        writer_->write(msg, "/str_message", "std_msgs/msg/String", this->now());
      });
  }

private:
  std::unique_ptr<rosbag2_cpp::Writer> writer_;
  rclcpp::GenericSubscription::SharedPtr cameraSubscriber_;
  rclcpp::GenericSubscription::SharedPtr stringSubscriber_;

};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  auto options = rclcpp::NodeOptions().use_intra_process_comms(true);
  auto node = std::make_shared<FrameSubscriberRecorder>(options);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
