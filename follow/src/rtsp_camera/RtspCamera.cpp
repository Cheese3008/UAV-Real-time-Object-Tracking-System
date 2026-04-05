#include "RtspCamera.hpp"

#include <sensor_msgs/image_encodings.hpp>
#include <chrono>

// =======================
// Constructor / Destructor
// =======================

RTSPCameraNode::RTSPCameraNode()
: Node("rtsp_camera_node"),
  cam_info_manager_(this),
  running_(true)
{
  declareParams();
  loadParams();
  setupPublishers();
  openStream();

  capture_thread_ = std::thread(&RTSPCameraNode::captureLoop, this);
}

RTSPCameraNode::~RTSPCameraNode()
{
  running_ = false;
  if (capture_thread_.joinable()) {
    capture_thread_.join();
  }
}

// =======================
// Parameter handling
// =======================

void RTSPCameraNode::declareParams()
{
  declare_parameter<std::string>("rtsp_url", "");
  declare_parameter<std::string>("image_topic", "/camera/image_raw");
  declare_parameter<std::string>("camera_info_topic", "/camera/camera_info");
  declare_parameter<std::string>("camera_info_url", "");
  declare_parameter<std::string>("frame_id", "camera_frame");
  declare_parameter<int>("latency", 50);
}

void RTSPCameraNode::loadParams()
{
  get_parameter("rtsp_url", rtsp_url_);
  get_parameter("image_topic", image_topic_);
  get_parameter("camera_info_topic", camera_info_topic_);
  get_parameter("camera_info_url", camera_info_url_);
  get_parameter("frame_id", frame_id_);
  get_parameter("latency", latency_);

  if (rtsp_url_.empty()) {
    RCLCPP_FATAL(get_logger(), "rtsp_url is empty");
    rclcpp::shutdown();
  }

  if (!camera_info_url_.empty()) {
    cam_info_manager_.loadCameraInfo(camera_info_url_);
  }
}

// =======================
// ROS publishers
// =======================

void RTSPCameraNode::setupPublishers()
{
  auto qos = rclcpp::SensorDataQoS();
  image_pub_ = create_publisher<sensor_msgs::msg::Image>(image_topic_, qos);
  cam_info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>(camera_info_topic_, qos);
}

// =======================
// RTSP / GStreamer
// =======================

void RTSPCameraNode::openStream()
{
  const std::string pipeline =
    "rtspsrc location=" + rtsp_url_ +
    " protocols=tcp latency=" + std::to_string(latency_) + " drop-on-latency=true ! "
    "rtph264depay ! h264parse ! avdec_h264 ! " 
    "videoconvert ! video/x-raw,format=GRAY8 ! "
    "appsink drop=true sync=false max-buffers=1 emit-signals=false";

  if (!cap_.open(pipeline, cv::CAP_GSTREAMER)) {
    RCLCPP_ERROR(get_logger(), "Failed to open RTSP stream. Retrying in loop...");
  } else {
    RCLCPP_INFO(get_logger(), "RTSP stream opened with optimized pipeline.");
  }
}

// =======================
// Capture loop
// =======================

void RTSPCameraNode::captureLoop()
{
  cv::Mat frame;
  int retry_count = 0;

  while (rclcpp::ok() && running_) {
    if (!cap_.isOpened() || !cap_.read(frame) || frame.empty()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Stream lost or empty frame. Attempting to reconnect (Try %d)...", ++retry_count);
      cap_.release();
      std::this_thread::sleep_for(std::chrono::milliseconds(2000));
      openStream();
      continue;
    }
    retry_count = 0;
    publishFrame(frame);
  }
}

// =======================
// Publish image + camera_info
// =======================

void RTSPCameraNode::publishFrame(const cv::Mat & frame)
{
  const auto stamp = this->now();
  std_msgs::msg::Header header;
  header.stamp = stamp;
  header.frame_id = frame_id_;

  // Tối ưu: Dùng GRAY8 (MONO8) rất nhẹ, copy mem rất nhanh
  auto img_msg = cv_bridge::CvImage(header, sensor_msgs::image_encodings::MONO8, frame).toImageMsg();
  image_pub_->publish(*img_msg);

  auto cam_info = cam_info_manager_.getCameraInfo();
  cam_info.header = header; 
  cam_info.width = frame.cols;
  cam_info.height = frame.rows;

  cam_info_pub_->publish(cam_info);
}

// =======================
// Main
// =======================

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RTSPCameraNode>());
  rclcpp::shutdown();
  return 0;
}