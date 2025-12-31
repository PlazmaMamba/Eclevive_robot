/**
 * @file zed_zupt_filter_node.cpp
 * @brief ZED2i Odometry ZUPT (Zero-Velocity Update) Filter Node
 *
 * This node integrates ZED2i Visual Odometry and IMU data,
 * and uses ZUPT theory to prevent odometry drift.
 *
 * Principle:
 * - When IMU acceleration and angular velocity are nearly zero → Robot is stationary
 * - At this time, if Visual Odometry reports movement → Correct as anomalous value
 *
 * @author jetros
 * @date 2025-10-19
 */

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <deque>
#include <cmath>
#include <algorithm>

class ZedZuptFilterNode : public rclcpp::Node
{
public:
    ZedZuptFilterNode() : Node("zed_zupt_filter_node")
    {
        // Parameter declaration
        this->declare_parameter("input_odom_topic", "/zed/zed_node/odom");
        this->declare_parameter("input_imu_topic", "/zed/zed_node/imu/data");
        this->declare_parameter("output_odom_topic", "/zed/zed_node/odom_zupt");

        // ZUPT detection thresholds
        this->declare_parameter("zupt_accel_threshold", 0.5);      // m/s² (stationary detection)
        this->declare_parameter("zupt_gyro_threshold", 0.1);       // rad/s (stationary detection)
        this->declare_parameter("zupt_window_size", 10);           // Number of samples
        this->declare_parameter("zupt_confidence_threshold", 0.7); // Confidence for stationary detection

        // Odometry correction parameters
        this->declare_parameter("max_velocity_jump", 0.5);         // m/s (limit abrupt velocity changes)
        this->declare_parameter("max_angular_jump", 1.0);          // rad/s
        this->declare_parameter("velocity_decay_rate", 0.95);      // Velocity decay rate when stationary
        this->declare_parameter("enable_zupt", true);              // Enable ZUPT

        // Get parameters
        input_odom_topic_ = this->get_parameter("input_odom_topic").as_string();
        input_imu_topic_ = this->get_parameter("input_imu_topic").as_string();
        output_odom_topic_ = this->get_parameter("output_odom_topic").as_string();

        zupt_accel_threshold_ = this->get_parameter("zupt_accel_threshold").as_double();
        zupt_gyro_threshold_ = this->get_parameter("zupt_gyro_threshold").as_double();
        zupt_window_size_ = this->get_parameter("zupt_window_size").as_int();
        zupt_confidence_threshold_ = this->get_parameter("zupt_confidence_threshold").as_double();

        max_velocity_jump_ = this->get_parameter("max_velocity_jump").as_double();
        max_angular_jump_ = this->get_parameter("max_angular_jump").as_double();
        velocity_decay_rate_ = this->get_parameter("velocity_decay_rate").as_double();
        enable_zupt_ = this->get_parameter("enable_zupt").as_bool();

        // Subscriber
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            input_odom_topic_, 10,
            std::bind(&ZedZuptFilterNode::odomCallback, this, std::placeholders::_1));

        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            input_imu_topic_, 10,
            std::bind(&ZedZuptFilterNode::imuCallback, this, std::placeholders::_1));

        // Publisher
        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(output_odom_topic_, 10);

        // Statistics Publisher (for debugging)
        zupt_status_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/zupt/status", 10);

        RCLCPP_INFO(this->get_logger(), "ZED ZUPT Filter Node initialized");
        RCLCPP_INFO(this->get_logger(), "  Input Odom: %s", input_odom_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), "  Input IMU: %s", input_imu_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), "  Output Odom: %s", output_odom_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), "  ZUPT Enabled: %s", enable_zupt_ ? "true" : "false");
        RCLCPP_INFO(this->get_logger(), "  Accel Threshold: %.3f m/s²", zupt_accel_threshold_);
        RCLCPP_INFO(this->get_logger(), "  Gyro Threshold: %.3f rad/s", zupt_gyro_threshold_);
    }

private:
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        // Store IMU data in buffer
        imu_buffer_.push_back(*msg);

        // Limit buffer size
        if (imu_buffer_.size() > static_cast<size_t>(zupt_window_size_)) {
            imu_buffer_.pop_front();
        }

        // Update ZUPT detection
        is_stationary_ = detectZeroVelocity();
    }

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        if (!enable_zupt_) {
            // If ZUPT is disabled, output as-is
            odom_pub_->publish(*msg);
            return;
        }

        // Create filtered odometry
        auto filtered_odom = *msg;

        if (is_stationary_) {
            // When stationary: Correct velocity to zero
            filtered_odom.twist.twist.linear.x = 0.0;
            filtered_odom.twist.twist.linear.y = 0.0;
            filtered_odom.twist.twist.linear.z = 0.0;
            filtered_odom.twist.twist.angular.x = 0.0;
            filtered_odom.twist.twist.angular.y = 0.0;
            filtered_odom.twist.twist.angular.z = 0.0;

            // Suppress position drift (maintain previous position)
            if (last_odom_received_) {
                filtered_odom.pose.pose.position = last_filtered_odom_.pose.pose.position;
                filtered_odom.pose.pose.orientation = last_filtered_odom_.pose.pose.orientation;
            }

            zupt_correction_count_++;
        } else {
            // When moving: Limit abrupt velocity changes
            if (last_odom_received_) {
                filtered_odom.twist.twist.linear.x = limitJump(
                    filtered_odom.twist.twist.linear.x,
                    last_filtered_odom_.twist.twist.linear.x,
                    max_velocity_jump_
                );
                filtered_odom.twist.twist.linear.y = limitJump(
                    filtered_odom.twist.twist.linear.y,
                    last_filtered_odom_.twist.twist.linear.y,
                    max_velocity_jump_
                );
                filtered_odom.twist.twist.angular.z = limitJump(
                    filtered_odom.twist.twist.angular.z,
                    last_filtered_odom_.twist.twist.angular.z,
                    max_angular_jump_
                );
            }
        }

        // Output
        odom_pub_->publish(filtered_odom);

        // Store previous value
        last_filtered_odom_ = filtered_odom;
        last_odom_received_ = true;

        // Output status (for debugging)
        publishZuptStatus();
    }

    bool detectZeroVelocity()
    {
        if (imu_buffer_.size() < static_cast<size_t>(zupt_window_size_)) {
            return false;  // Insufficient data
        }

        // Analyze IMU data within the window
        double accel_sum = 0.0;
        double gyro_sum = 0.0;
        int stationary_count = 0;

        for (const auto& imu : imu_buffer_) {
            // Acceleration norm (with gravity removed)
            double accel_norm = std::sqrt(
                imu.linear_acceleration.x * imu.linear_acceleration.x +
                imu.linear_acceleration.y * imu.linear_acceleration.y +
                (imu.linear_acceleration.z - 9.81) * (imu.linear_acceleration.z - 9.81)
            );

            // Angular velocity norm
            double gyro_norm = std::sqrt(
                imu.angular_velocity.x * imu.angular_velocity.x +
                imu.angular_velocity.y * imu.angular_velocity.y +
                imu.angular_velocity.z * imu.angular_velocity.z
            );

            accel_sum += accel_norm;
            gyro_sum += gyro_norm;

            // Threshold check
            if (accel_norm < zupt_accel_threshold_ && gyro_norm < zupt_gyro_threshold_) {
                stationary_count++;
            }
        }

        // Calculate average values
        avg_accel_ = accel_sum / imu_buffer_.size();
        avg_gyro_ = gyro_sum / imu_buffer_.size();

        // Calculate confidence
        double confidence = static_cast<double>(stationary_count) / imu_buffer_.size();

        // Stationary detection
        return confidence >= zupt_confidence_threshold_;
    }

    double limitJump(double new_value, double old_value, double max_jump)
    {
        double diff = new_value - old_value;

        if (std::abs(diff) > max_jump) {
            // Limit abrupt changes
            return old_value + (diff > 0 ? max_jump : -max_jump);
        }

        return new_value;
    }

    void publishZuptStatus()
    {
        geometry_msgs::msg::Twist status;
        status.linear.x = is_stationary_ ? 1.0 : 0.0;  // Stationary flag
        status.linear.y = avg_accel_;                   // Average acceleration
        status.linear.z = avg_gyro_;                    // Average angular velocity
        status.angular.x = static_cast<double>(zupt_correction_count_);  // Correction count

        zupt_status_pub_->publish(status);
    }

    // Subscriber
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;

    // Publisher
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr zupt_status_pub_;

    // Parameters
    std::string input_odom_topic_;
    std::string input_imu_topic_;
    std::string output_odom_topic_;

    double zupt_accel_threshold_;
    double zupt_gyro_threshold_;
    int zupt_window_size_;
    double zupt_confidence_threshold_;

    double max_velocity_jump_;
    double max_angular_jump_;
    double velocity_decay_rate_;
    bool enable_zupt_;

    // State variables
    std::deque<sensor_msgs::msg::Imu> imu_buffer_;
    nav_msgs::msg::Odometry last_filtered_odom_;
    bool last_odom_received_ = false;
    bool is_stationary_ = false;

    double avg_accel_ = 0.0;
    double avg_gyro_ = 0.0;
    int zupt_correction_count_ = 0;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ZedZuptFilterNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
