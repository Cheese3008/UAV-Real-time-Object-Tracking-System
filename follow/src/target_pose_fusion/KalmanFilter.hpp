#pragma once

#include <atomic>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>
#include <Eigen/Dense>
#include <vector>
// #include <numeric>   // Để dùng std::accumulate
// #include <algorithm> // Để dùng std::minmax_element
// #include <cmath>     // Để dùng std::sqrt

class TargetPoseFusionNode : public rclcpp::Node
{
public:
    TargetPoseFusionNode();

private:
    // ===== Constants & Enums =====
    static constexpr int STATE_SIZE = 6;
    static constexpr int MEASUREMENT_SIZE = 3;
    static constexpr char FRAME_ID[] = "map";

    // ===== ROS Subscriptions =====
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr reset_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr valid_sub_;

    // ===== ROS Publishers =====
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr error_pub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr vel_pub_;

    // ===== ROS Timers =====
    rclcpp::TimerBase::SharedPtr timer_;

    // ===== Kalman Filter Variables =====
    Eigen::VectorXd x_; // [x, y, z, vx, vy, vz]^T
    Eigen::MatrixXd P_;
    Eigen::MatrixXd F_;
    Eigen::MatrixXd Q_;
    Eigen::MatrixXd H_;
    Eigen::MatrixXd R_;

    // ===== State Flags & Shared Variables =====
    bool initialized_ = false;
    std::atomic_bool force_zero_{false};
    std::atomic_bool target_valid_{false}; 

    // ===== Time Tracking =====
    rclcpp::Time last_predict_time_;    
    rclcpp::Time last_measurement_time_;            

    // ===== Cached Variables =====
    double q_acc_x_ = 0.0;
    double q_acc_y_ = 0.0;
    double q_acc_z_ = 0.0;
    geometry_msgs::msg::Quaternion last_orientation_;
    geometry_msgs::msg::Point last_position_;
    Eigen::Vector3d z;

    // ===== Core Logic Functions =====
    void declareParameters();
    void initKalman();
    void resetState();
    
    // Timer-driven operations
    void processAndPublish();
    void predict(double dt);
    
    // Data Output
    void publishEstimatedState(const rclcpp::Time& now);
    void publishZero(const rclcpp::Time& now);

    // Callbacks
    void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void resetCallback(const std_msgs::msg::String::SharedPtr msg);
    void validCallback(const std_msgs::msg::Bool::SharedPtr msg);  

};