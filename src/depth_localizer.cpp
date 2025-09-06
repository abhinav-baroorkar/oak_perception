#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>

class DepthLocalizer : public rclcpp::Node {
public:
  DepthLocalizer() : Node("depth_localizer")
  {
    // Params let you point to your actual topics from OAK-D
    declare_parameter<std::string>("camera_info_topic", "/rgb/camera_info");
    declare_parameter<std::string>("depth_topic", "/stereo/depth");
    declare_parameter<std::string>("detections_2d_topic", "/detections_2d");

    std::string cam_info_topic = get_parameter("camera_info_topic").as_string();
    std::string depth_topic    = get_parameter("depth_topic").as_string();
    std::string det2d_topic    = get_parameter("detections_2d_topic").as_string();

    cam_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      cam_info_topic, rclcpp::SensorDataQoS(),
      std::bind(&DepthLocalizer::onCamInfo, this, std::placeholders::_1));

    depth_sub_ = create_subscription<sensor_msgs::msg::Image>(
      depth_topic, rclcpp::SensorDataQoS(),
      std::bind(&DepthLocalizer::onDepth, this, std::placeholders::_1));

    det2d_sub_ = create_subscription<vision_msgs::msg::Detection2DArray>(
      det2d_topic, 10,
      std::bind(&DepthLocalizer::onDetections, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "depth_localizer node started");
    RCLCPP_INFO(get_logger(), "Listening: cam_info=%s, depth=%s, det2d=%s",
                cam_info_topic.c_str(), depth_topic.c_str(), det2d_topic.c_str());
  }

private:
  void onCamInfo(const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
    last_caminfo_ = msg;
    have_caminfo_ = true;
  }
  void onDepth(const sensor_msgs::msg::Image::SharedPtr msg) {
    last_depth_ = msg;
    have_depth_ = true;
  }
  void onDetections(const vision_msgs::msg::Detection2DArray::SharedPtr msg) {
    if (!have_caminfo_ || !have_depth_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Waiting for depth & camera_info...");
      return;
    }
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                         "Got %zu detections (syncing handled next sprint)", msg->detections.size());
    // Sprint 2: do 2D→3D projection using depth + camera model and publish Detection3DArray
  }

  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr cam_info_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
  rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr det2d_sub_;

  sensor_msgs::msg::Image::SharedPtr      last_depth_;
  sensor_msgs::msg::CameraInfo::SharedPtr last_caminfo_;
  bool have_caminfo_ = false, have_depth_ = false;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DepthLocalizer>());
  rclcpp::shutdown();
  return 0;
}
