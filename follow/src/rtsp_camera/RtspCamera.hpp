#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <camera_info_manager/camera_info_manager.hpp>
#include <cv_bridge/cv_bridge.hpp>

#include <opencv2/opencv.hpp>
#include <thread>
#include <atomic>

class RTSPCameraNode : public rclcpp::Node
{
public:
  RTSPCameraNode();
  ~RTSPCameraNode();

private:
  // ===== Internal helpers =====
  void declareParams();
  void loadParams();
  void setupPublishers();
  void openStream();
  void captureLoop();
  void publishFrame(const cv::Mat & frame);

  // ===== Parameters =====
  std::string rtsp_url_;
  std::string image_topic_;
  std::string camera_info_topic_;
  std::string camera_info_url_;
  std::string frame_id_;
  int latency_{100};

  // ===== ROS interfaces =====
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr cam_info_pub_;

  camera_info_manager::CameraInfoManager cam_info_manager_;

  // ===== Video =====
  cv::VideoCapture cap_;

  // ===== Threading =====
  std::thread capture_thread_;
  std::atomic<bool> running_{true};



};
