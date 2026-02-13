#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <rerun.hpp>

class RerunCameraNode : public rclcpp::Node {
public:
  RerunCameraNode() : Node("rerun_camera_node"), rec_("ros2_camera") {
    // Start Rerun viewer
    rec_.spawn(rerun::SpawnOptions{.memory_limit = "1GB"}).exit_on_failure();

    sub_ = create_subscription<sensor_msgs::msg::Image>(
      "/camera/image_raw", 10,
      std::bind(&RerunCameraNode::imageCallback, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "Rerun camera node started");
  }

private:
  void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
    // try {
      auto cv_ptr = cv_bridge::toCvCopy(msg, "rgb8");
      const cv::Mat& img = cv_ptr->image;
      rec_.log("camera/image", rerun::Image(img.data,
                                            {static_cast<uint32_t>(img.cols),
                                             static_cast<uint32_t>(img.rows)},
                                            rerun::datatypes::ColorModel::RGB));
    // } catch (const std::exception& e) {
    //   RCLCPP_ERROR(get_logger(), "cv_bridge error: %s", e.what());
    // }
  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
  const rerun::RecordingStream rec_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<RerunCameraNode>();
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
