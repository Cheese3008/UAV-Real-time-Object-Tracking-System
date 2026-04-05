#include "KalmanFilter.hpp"

//---------------------DEFINE TOPIC -------------------------//
constexpr char TARGET_POSE_TOPIC[] = "/target_pose";
constexpr char RESET_TOPIC[] = "/reset";
constexpr char KALMAN_ERROR_TOPIC[] = "/kalman_error";
constexpr char TARGET_VEL_FUSION_TOPIC[] = "/target_vel_fusion";
constexpr char TARGET_POSE_FUSION_TOPIC[] = "/target_pose_fusion";

TargetPoseFusionNode::TargetPoseFusionNode()
    : Node("target_pose_fusion_node")
{
    auto sub_qos = rclcpp::QoS(1).best_effort();
    auto pub_qos = rclcpp::QoS(1).best_effort();

    //--------------------------SUBSCRIPTIONS--------------------------------------//
    pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        TARGET_POSE_TOPIC, sub_qos,
        std::bind(&TargetPoseFusionNode::poseCallback, this, std::placeholders::_1)
    );

    reset_sub_ = create_subscription<std_msgs::msg::String>(
        RESET_TOPIC, sub_qos,
        std::bind(&TargetPoseFusionNode::resetCallback, this, std::placeholders::_1)
    ); 

    valid_sub_ = create_subscription<std_msgs::msg::Bool>(
        "/target_valid", sub_qos, 
        std::bind(&TargetPoseFusionNode::validCallback, this, std::placeholders::_1)
    );
    
    //--------------------------PUBLISHER--------------------------------------//
    error_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(KALMAN_ERROR_TOPIC, pub_qos);
    vel_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>(TARGET_VEL_FUSION_TOPIC, pub_qos);
    pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(TARGET_POSE_FUSION_TOPIC, pub_qos);

    //--------------------------MAIN LOGIC--------------------------------------//
    declareParameters();
    initKalman();

    //--------------------------TIMER--------------------------------------//
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(33), 
        std::bind(&TargetPoseFusionNode::processAndPublish, this)
    );

    // Initialize time references
    last_predict_time_ = this->now();
    last_measurement_time_ = this->now();
    
    RCLCPP_INFO(get_logger(), "TargetPoseFusionNode started and ready.");
}

//-------------------------INIT FUNCTIONS-----------------------------------//

void TargetPoseFusionNode::declareParameters() 
{
    declare_parameter<double>("q_acc_x", 0.1);
    declare_parameter<double>("q_acc_y", 0.1);
    declare_parameter<double>("q_acc_z", 0.1);
    declare_parameter<double>("r_pos_x", 0.00003136);
    declare_parameter<double>("r_pos_y", 0.000039);
    declare_parameter<double>("r_pos_z", 0.01833316);
}

void TargetPoseFusionNode::initKalman()
{
    // Retrieve and cache static parameters
    q_acc_x_ = get_parameter("q_acc_x").as_double();
    q_acc_y_ = get_parameter("q_acc_y").as_double();
    q_acc_z_ = get_parameter("q_acc_z").as_double();
    
    double r_pos_x = get_parameter("r_pos_x").as_double();
    double r_pos_y = get_parameter("r_pos_y").as_double();
    double r_pos_z = get_parameter("r_pos_z").as_double();

    // Init state vector & covariance
    x_ = Eigen::VectorXd::Zero(STATE_SIZE);  
    P_ = Eigen::MatrixXd::Identity(STATE_SIZE, STATE_SIZE);
    
    // Init state transition & measurement matrices
    F_ = Eigen::MatrixXd::Identity(STATE_SIZE, STATE_SIZE);
    H_ = Eigen::MatrixXd::Zero(MEASUREMENT_SIZE, STATE_SIZE);
    H_.block<MEASUREMENT_SIZE, MEASUREMENT_SIZE>(0,0) = Eigen::Matrix3d::Identity();

    // Process & Measurement Noise Covariances
    Q_ = Eigen::MatrixXd::Zero(STATE_SIZE, STATE_SIZE); 

    R_ = Eigen::MatrixXd::Identity(MEASUREMENT_SIZE, MEASUREMENT_SIZE); 
    R_(0,0) = r_pos_x; 
    R_(1,1) = r_pos_y; 
    R_(2,2) = r_pos_z;  
    
    RCLCPP_INFO(get_logger(), "Kalman matrices initialized successfully.");
}

//------------------------CALLBACKS------------------------------------//

void TargetPoseFusionNode::resetCallback(const std_msgs::msg::String::SharedPtr msg)
{
    if (msg->data == "RESET") {
        resetState();
        force_zero_.store(true, std::memory_order_relaxed);
        RCLCPP_WARN(get_logger(), "RESET received -> Kalman State Cleared. Outputting 0.");
    } else if (msg->data == "ACTIVE") {
        force_zero_.store(false, std::memory_order_relaxed);
    }
}

void TargetPoseFusionNode::validCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
    target_valid_.store(msg->data, std::memory_order_relaxed);
    if (msg->data) {
        force_zero_.store(false, std::memory_order_relaxed);
    } 
}

void TargetPoseFusionNode::poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    const rclcpp::Time now = this->now();
    last_measurement_time_ = now;

    if (!initialized_) {
        // Init target state with the first valid observation
        x_ << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z, 0, 0, 0;
        initialized_ = true;
        
        last_predict_time_ = now;
        last_orientation_ = msg->pose.orientation;

        RCLCPP_INFO(get_logger(), "Initial Tag Pose acquired. Kalman Tracking started.");
        return;
    }

    last_orientation_ = msg->pose.orientation;
    
    // --- Kalman Update (Correction) Step ---
    Eigen::Vector3d z(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
    Eigen::Vector3d y = z - H_ * x_;
    Eigen::MatrixXd S = H_ * P_ * H_.transpose() + R_;
    Eigen::MatrixXd K = P_ * H_.transpose() * S.inverse();
    
    x_ = x_ + K * y;
    P_ = (Eigen::MatrixXd::Identity(STATE_SIZE, STATE_SIZE) - K * H_) * P_;
}

//------------------------PREDICTION------------------------------------//

void TargetPoseFusionNode::processAndPublish()
{
    const rclcpp::Time now = this->now();

    if (force_zero_.load(std::memory_order_relaxed)) {
        publishZero(now);
        return;
    }
    
    if (!initialized_) {
        return; // Wait silently for the first camera measurement
    }

    // Fixed-step prediction based on Timer ticks
    double dt = (now - last_predict_time_).seconds();
    last_predict_time_ = now; 

    if (dt > 0.0) {
        predict(dt);
    }
    
    publishEstimatedState(now);
}

void TargetPoseFusionNode::predict(double dt)
{
    double dt2 = dt * dt;  
    double dt3 = dt2 * dt; 
    double dt4 = dt3 * dt;  

    // Process Noise
    Q_.setZero();
    Q_(0, 0) = 0.25 * dt4 * q_acc_x_; Q_(0, 3) = 0.5 * dt3 * q_acc_x_;
    Q_(3, 0) = 0.5 * dt3 * q_acc_x_;  Q_(3, 3) = dt2 * q_acc_x_;
    
    Q_(1, 1) = 0.25 * dt4 * q_acc_y_; Q_(1, 4) = 0.5 * dt3 * q_acc_y_;
    Q_(4, 1) = 0.5 * dt3 * q_acc_y_;  Q_(4, 4) = dt2 * q_acc_y_;
    
    Q_(2, 2) = 0.25 * dt4 * q_acc_z_; Q_(2, 5) = 0.5 * dt3 * q_acc_z_;
    Q_(5, 2) = 0.5 * dt3 * q_acc_z_;  Q_(5, 5) = dt2 * q_acc_z_;

    // Update state F matrix
    F_.setIdentity();
    F_(0,3) = dt;
    F_(1,4) = dt;
    F_(2,5) = dt;

    // Prediction matrix
    x_ = F_ * x_;
    P_ = F_ * P_ * F_.transpose() + Q_;
}

//----------------------------OUTPUT FUNCTIONS-------------------------------//

void TargetPoseFusionNode::publishEstimatedState(const rclcpp::Time & now)
{
    geometry_msgs::msg::PoseStamped out_pose;
    out_pose.header.stamp = now;
    out_pose.header.frame_id = FRAME_ID; 
    out_pose.pose.position.x = x_(0);
    out_pose.pose.position.y = x_(1);
    out_pose.pose.position.z = x_(2);
    out_pose.pose.orientation = last_orientation_; 
    pose_pub_->publish(out_pose);

    geometry_msgs::msg::TwistStamped out_vel;
    out_vel.header.stamp = now;
    out_vel.header.frame_id = FRAME_ID;
    out_vel.twist.linear.x = x_(3); 
    out_vel.twist.linear.y = x_(4); 
    out_vel.twist.linear.z = x_(5); 
    vel_pub_->publish(out_vel);

    // geometry_msgs::msg::PoseStamped error_kalman;
    // error_kalman.header.stamp = now;
    // error_kalman.header.frame_id = FRAME_ID;
    // error_kalman.pose.position.x = std::abs(z(0) - x_(0));
    // error_kalman.pose.position.y = std::abs(z(1) - x_(1));
    // error_kalman.pose.position.z = std::abs(z(2) - x_(2));
    // error_pub_->publish(error_kalman);
}

void TargetPoseFusionNode::publishZero(const rclcpp::Time & now) 
{
    geometry_msgs::msg::PoseStamped out_pose;
    out_pose.header.stamp = now;
    out_pose.header.frame_id = FRAME_ID;
    out_pose.pose.orientation = last_orientation_;
    pose_pub_->publish(out_pose);

    geometry_msgs::msg::TwistStamped out_vel;
    out_vel.header.stamp = now;
    out_vel.header.frame_id = FRAME_ID;
    vel_pub_->publish(out_vel);
}

//------- RESET STATE FUNCTION-----------//
void TargetPoseFusionNode::resetState()
{
    x_.setZero();
    P_.setIdentity();
    initialized_ = false;
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TargetPoseFusionNode>());
    rclcpp::shutdown();
    return 0;
}