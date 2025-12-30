#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include "tf2_ros/static_transform_broadcaster.h"
#include <tf2_ros/transform_listener.h>
#include "tf2_ros/buffer.h"

#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <std_msgs/msg/int32.hpp>

#include <zed_msgs/msg/object.hpp>
#include <zed_msgs/msg/objects_stamped.hpp>
#include <sensor_msgs/msg/joy.hpp>

#include <vector>
#include <queue>

using NavigateToPose = nav2_msgs::action::NavigateToPose;
using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;
using PoseStamped = geometry_msgs::msg::PoseStamped;

// Structure representing arm pose
struct ArmPose {
    int32_t joint0;
    int32_t joint1;
    int32_t joint2;
    int32_t joint3;
    int duration_ms;  // Time to hold this pose (milliseconds)
};

class GoalPublisher : public rclcpp::Node {
public:
    GoalPublisher() : Node("zed_goal_publisher") {
        // Topic name parameter declaration
        std::string skeleton_topic = this->declare_parameter<std::string>("skeleton_topic", "zed/zed_node/body_trk/skeletons");
        std::string target_arm_pose_topic = this->declare_parameter<std::string>("target_arm_pose_topic", "target_arm_pose");
        std::string current_arm_pose_topic = this->declare_parameter<std::string>("current_arm_pose_topic", "current_arm_pose");
        std::string cmd_vel_topic = this->declare_parameter<std::string>("cmd_vel_topic", "cmd_vel");
        std::string goal_pose_topic = this->declare_parameter<std::string>("goal_pose_topic", "goal_pose");
        std::string zed_goal_status_topic = this->declare_parameter<std::string>("zed_goal_status_topic", "zed_goal_status");

        // Velocity parameter declaration
        forward_vel_fast_ = this->declare_parameter<double>("forward_vel_fast", 0.5);
        forward_vel_slow_ = this->declare_parameter<double>("forward_vel_slow", 0.1);
        turn_right_vel_fast_ = this->declare_parameter<double>("turn_right_vel_fast", 1.6);
        turn_right_vel_slow_ = this->declare_parameter<double>("turn_right_vel_slow", 0.05);
        turn_left_vel_fast_ = this->declare_parameter<double>("turn_left_vel_fast", -1.6);
        turn_left_vel_slow_ = this->declare_parameter<double>("turn_left_vel_slow", -0.05);
        strafe_right_vel_fast_ = this->declare_parameter<double>("strafe_right_vel_fast", 0.70);
        strafe_right_vel_slow_ = this->declare_parameter<double>("strafe_right_vel_slow", 0.10);
        strafe_left_vel_fast_ = this->declare_parameter<double>("strafe_left_vel_fast", -0.70);
        strafe_left_vel_slow_ = this->declare_parameter<double>("strafe_left_vel_slow", -0.10);

        // Phase 1 improvement: Parameterization of magic numbers
        hand_raise_threshold_ = this->declare_parameter<double>("hand_raise_threshold", 0.3);
        target_distance_min_ = this->declare_parameter<double>("target_distance_min", 1.0);
        target_person_offset_ = this->declare_parameter<double>("target_person_offset", 0.6);
        close_approach_distance_ = this->declare_parameter<double>("close_approach_distance", 0.8);
        arm_motion_interval_ms_ = this->declare_parameter<int>("arm_motion_interval_ms", 100);

        RCLCPP_INFO(this->get_logger(), "=== ZED Goal Publisher Parameters ===");
        RCLCPP_INFO(this->get_logger(), "  hand_raise_threshold: %.2f m", hand_raise_threshold_);
        RCLCPP_INFO(this->get_logger(), "  target_distance_min: %.2f m", target_distance_min_);
        RCLCPP_INFO(this->get_logger(), "  target_person_offset: %.2f m", target_person_offset_);
        RCLCPP_INFO(this->get_logger(), "  close_approach_distance: %.2f m", close_approach_distance_);
        RCLCPP_INFO(this->get_logger(), "======================================");

        // Subscribe to human recognition content from ZED
        objectSubscriber_ = this->create_subscription<zed_msgs::msg::ObjectsStamped>(
            skeleton_topic, 1, std::bind(&GoalPublisher::objectCallback, this, std::placeholders::_1));

        arm_pose_Subscriber_ = this->create_subscription<std_msgs::msg::Int32MultiArray>(
            target_arm_pose_topic, 1, std::bind(&GoalPublisher::arm_pose_Callback, this, std::placeholders::_1));

        // Subscribe to joy topic
        joySubscriber_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "joy", 1, std::bind(&GoalPublisher::joyCallback, this, std::placeholders::_1));

        // TF2 Transform Listener
        recognization_frame_ = this->declare_parameter<std::string>("odom", "zed_camera_link");
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        target_tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        target_tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*target_tf_buffer_);
        timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&GoalPublisher::on_timer, this));

        // Publishers
        arm_pose_Publisher_= this->create_publisher<std_msgs::msg::Int32MultiArray>(target_arm_pose_topic, 10);
        current_arm_pose_Publisher_= this->create_publisher<std_msgs::msg::Int32MultiArray>(current_arm_pose_topic, 10);
        cmdVelPublisher_ = this->create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic, 1);
        goal_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(goal_pose_topic, 10);
        zed_goal_status_ =this->create_publisher<std_msgs::msg::Int32>(zed_goal_status_topic, 1);

        // Action client
        action_client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");

        // Phase 1 improvement: Asynchronous arm motion timer
        arm_motion_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(arm_motion_interval_ms_),
            std::bind(&GoalPublisher::executeArmMotionStep, this));

        // Initialize member variables
        arm_message_.data = {0, 0, 0, 0};
        cmdVelMsg_.linear.x = 0.0;
        cmdVelMsg_.linear.y = 0.0;
        cmdVelMsg_.linear.z = 0.0;
        cmdVelMsg_.angular.x = 0.0;
        cmdVelMsg_.angular.y = 0.0;
        cmdVelMsg_.angular.z = 0.0;

        navigation_activate_ = 0;

        RCLCPP_INFO(this->get_logger(), "ZED Goal Publisher initialized successfully");
    }

private:

//------------------------------------------------------------------------------------------
    void set_goal_point(){
        geometry_msgs::msg::TransformStamped tf;

        try {
            tf = target_tf_buffer_->lookupTransform(
                "map", "target_person",
                tf2::TimePointZero);

            RCLCPP_INFO(this->get_logger(), "=== Target Person Found ===");
            RCLCPP_INFO(this->get_logger(), "Position: (%.3f, %.3f, %.3f)",
                tf.transform.translation.x,
                tf.transform.translation.y,
                tf.transform.translation.z);
            RCLCPP_INFO(this->get_logger(), "Orientation: (%.3f, %.3f, %.3f, %.3f)",
                tf.transform.rotation.x,
                tf.transform.rotation.y,
                tf.transform.rotation.z,
                tf.transform.rotation.w);

        } catch (const tf2::TransformException & ex) {
            RCLCPP_WARN(this->get_logger(), "Could not transform map to target_person: %s", ex.what());
            return;
        }

        // Create action Goal
        auto goal_msg = NavigateToPose::Goal();
        goal_msg.pose.header.stamp = this->now();
        goal_msg.pose.header.frame_id = "map";
        goal_msg.pose.pose.position.x = tf.transform.translation.x;
        goal_msg.pose.pose.position.y = tf.transform.translation.y;
        goal_msg.pose.pose.position.z = tf.transform.translation.z;
        goal_msg.pose.pose.orientation.x = tf.transform.rotation.x;
        goal_msg.pose.pose.orientation.y = tf.transform.rotation.y;
        goal_msg.pose.pose.orientation.z = tf.transform.rotation.z;
        goal_msg.pose.pose.orientation.w = tf.transform.rotation.w;

        // Set feedback callback
        auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
        send_goal_options.feedback_callback = std::bind(&GoalPublisher::feedbackCallback, this, std::placeholders::_1, std::placeholders::_2);
        send_goal_options.result_callback   = std::bind(&GoalPublisher::resultCallback, this, std::placeholders::_1);

        // Send Goal to server
        action_client_->async_send_goal(goal_msg, send_goal_options);

        // Also publish to goal_pose topic
        PoseStamped goal_pose_msg;
        goal_pose_msg.header.stamp = this->now();
        goal_pose_msg.header.frame_id = "map";
        goal_pose_msg.pose.position.x = tf.transform.translation.x;
        goal_pose_msg.pose.position.y = tf.transform.translation.y;
        goal_pose_msg.pose.position.z = tf.transform.translation.z;
        goal_pose_msg.pose.orientation.x = tf.transform.rotation.x;
        goal_pose_msg.pose.orientation.y = tf.transform.rotation.y;
        goal_pose_msg.pose.orientation.z = tf.transform.rotation.z;
        goal_pose_msg.pose.orientation.w = tf.transform.rotation.w;
        goal_publisher_->publish(goal_pose_msg);

        RCLCPP_INFO(this->get_logger(),"=== Goal Point Published ===");
        RCLCPP_INFO(this->get_logger(),"Position: (%.3f, %.3f, %.3f)",
            goal_msg.pose.pose.position.x,
            goal_msg.pose.pose.position.y,
            goal_msg.pose.pose.position.z);
        RCLCPP_INFO(this->get_logger(),"Orientation: (%.3f, %.3f, %.3f, %.3f)",
            goal_msg.pose.pose.orientation.x,
            goal_msg.pose.pose.orientation.y,
            goal_msg.pose.pose.orientation.z,
            goal_msg.pose.pose.orientation.w);
    }

//---------------------- Phase 1 improvement: Arm motion management ---------------------------------------------------
    // Register arm sequence
    void scheduleArmWaveSequence(double target_angle) {
        int angle_deg = static_cast<int>(target_angle * 80.0);

        arm_motion_queue_ = std::queue<ArmPose>();  // Clear

        // Initial position
        arm_motion_queue_.push({angle_deg, 200, 200, -200, 600});

        // Hand waving sequence
        arm_motion_queue_.push({angle_deg, arm_message_.data[1], arm_message_.data[2], 20, 600});
        arm_motion_queue_.push({angle_deg, arm_message_.data[1], arm_message_.data[2], -160, 600});
        arm_motion_queue_.push({angle_deg, arm_message_.data[1], arm_message_.data[2], 0, 600});
        arm_motion_queue_.push({angle_deg, arm_message_.data[1], arm_message_.data[2], -160, 1500});
    }

    void scheduleArmGreetSequence(double target_angle) {
        int angle_deg = static_cast<int>(target_angle * 80.0);

        arm_motion_queue_ = std::queue<ArmPose>();  // Clear

        // Addition: First face the person (direction adjustment phase)
        arm_motion_queue_.push({angle_deg, 200, 200, -200, 600});

        // Addition: Wave hand to show recognition
        arm_motion_queue_.push({angle_deg, arm_message_.data[1], arm_message_.data[2], -120, 200});
        arm_motion_queue_.push({angle_deg, arm_message_.data[1], arm_message_.data[2], -200, 200});
        arm_motion_queue_.push({angle_deg, arm_message_.data[1], arm_message_.data[2], -120, 200});
        arm_motion_queue_.push({angle_deg, arm_message_.data[1], arm_message_.data[2], -200, 200});

        // Greeting sequence (original code)
        arm_motion_queue_.push({angle_deg, 200 ,   40, -200, 600});
        arm_motion_queue_.push({angle_deg, 100 ,  -40, -200, 600});
        arm_motion_queue_.push({angle_deg, 50  , -120, -200, 600});
        arm_motion_queue_.push({angle_deg, 0   , -190, -200, 600});
        arm_motion_queue_.push({angle_deg, -50 , -190, -200, 600});
        arm_motion_queue_.push({angle_deg, -150, -190, 50, 2000});
        arm_motion_queue_.push({angle_deg, -150, -190, -100, 600});
        arm_motion_queue_.push({angle_deg, 50  , -100, -200, 600});
        arm_motion_queue_.push({angle_deg, 100 ,   80, -200, 600});
        arm_motion_queue_.push({angle_deg, 200 ,  200, -200, 600});
    }

    // Called by timer: Execute one step at a time from the queue
    void executeArmMotionStep() {
        if (arm_motion_queue_.empty()) {
            return;  // Do nothing if queue is empty
        }

        // Check elapsed time from previous pose
        auto now = this->now();
        if (last_arm_motion_time_.seconds() > 0.0) {
            auto elapsed_ms = (now - last_arm_motion_time_).seconds() * 1000.0;
            if (elapsed_ms < current_arm_duration_ms_) {
                return;  // Still in waiting time
            }
        }

        // Extract and execute the next pose
        ArmPose pose = arm_motion_queue_.front();
        arm_motion_queue_.pop();

        arm_message_.data = {pose.joint0, pose.joint1, pose.joint2, pose.joint3};
        arm_pose_Publisher_->publish(arm_message_);

        current_arm_duration_ms_ = pose.duration_ms;
        last_arm_motion_time_ = now;

        // Output current angle if this is the last pose
        if (arm_motion_queue_.empty()) {
            current_arm_pose_Publisher_->publish(arm_message_);
        }
    }

//---------------------- Process human recognition content from ZED2i ---------------------------------------------------
    void objectCallback(const zed_msgs::msg::ObjectsStamped::SharedPtr objMsg)
    {
        // Stop processing if navigation_activate_ is 0 (manual mode)
        if (navigation_activate_ == 0) {
            return;
        }

        if (objMsg->objects.size() == 0) {
            return;
        }

        // Person recognition notification
        if (zed_goal_msg.data < 1) {
            zed_goal_msg.data = 1;
        }

        // Search for the person with raised hand who is farthest away
        for (uint i = 0; i < objMsg->objects.size(); i++) {
            const auto& obj = objMsg->objects[i];

            if (obj.tracking_state <= 0) {
                continue;
            }

            // Phase 1 improvement: Enhanced error handling
            const size_t keypoints_count = obj.skeleton_3d.keypoints.size();

            // Get skeleton keypoints (with range check)
            for (int pose_count = 2; pose_count <= 7; pose_count++) {
                if (pose_count >= static_cast<int>(keypoints_count)) {
                    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                        "Keypoint index %d out of range (size: %zu)", pose_count, keypoints_count);
                    body_pose_x[pose_count] = 0.0;
                    body_pose_y[pose_count] = 0.0;
                    continue;
                }

                // X direction (left-right)
                if (pose_count >= 3 && pose_count < static_cast<int>(keypoints_count)) {
                    if (std::isnan(obj.skeleton_3d.keypoints[3].kp[1])) {
                        body_pose_x[pose_count] = 0.0;
                    } else {
                        body_pose_x[pose_count] = obj.skeleton_3d.keypoints[pose_count].kp[1];
                    }
                }

                // Y direction (up-down)
                if (pose_count >= 3 && pose_count < static_cast<int>(keypoints_count)) {
                    if (std::isnan(obj.skeleton_3d.keypoints[3].kp[2])) {
                        body_pose_y[pose_count] = 0.0;
                    } else {
                        body_pose_y[pose_count] = obj.skeleton_3d.keypoints[pose_count].kp[2];
                    }
                }
            }

            target_point.x = obj.position[0];
            target_point.y = obj.position[1];
            target_point.z = obj.position[2];

            // Fix: Execute arm motion even if hand is not raised when person is nearby
            if (target_point.x < close_approach_distance_ && target_point.x > 0.1) {
                // Greeting sequence when person is nearby
                RCLCPP_INFO(this->get_logger(),"=== Person Nearby - Greeting ===");
                RCLCPP_INFO(this->get_logger(), "Distance: %.3f m", target_point.x);

                // Addition: Face the person before greeting
                double target_angle = std::atan(target_point.y / target_point.x);

                // Execute greeting sequence after facing and waving at the person
                scheduleArmGreetSequence(target_angle);

                zed_goal_msg.data = 3;  // Goal reached notification
                person_finding = false;
                break;  // Exit with priority on proximity processing
            }

            // Phase 1 improvement: Use parameterized thresholds
            // Find a person who has raised their hand above the shoulder
            // ZED coordinate system: Z-axis points upward (higher Z value means higher position)
            bool right_hand_raised = (body_pose_y[4] > (body_pose_y[2] + hand_raise_threshold_));
            bool left_hand_raised = (body_pose_y[7] > (body_pose_y[5] + hand_raise_threshold_));

            if (right_hand_raised || left_hand_raised) {
                person_finding = true;
                compare_point.x = obj.position[0];
                compare_point.y = obj.position[1];
                compare_point.z = obj.position[2];

                // Target the person who is farther away
                if (compare_point.x > target_point.x) {
                    target_point.x = obj.position[0];
                    target_point.y = obj.position[1];
                    target_point.z = obj.position[2];
                }

                RCLCPP_INFO(this->get_logger(),"=== Target Acquired ===");
                RCLCPP_INFO(this->get_logger(), "Position: (%.3f, %.3f, %.3f)",
                    target_point.x, target_point.y, target_point.z);

                zed_goal_msg.data = 2;  // Target discovery notification

                // Phase 1 improvement: Asynchronous arm motion
                double target_angle = std::atan(target_point.y / target_point.x);
                scheduleArmWaveSequence(target_angle);

                break;
            } else {
                person_finding = false;
            }
        }

        // Phase 1 improvement: Use parameterized distance threshold
        if (target_point.x > target_distance_min_) {
            // Generate TF (target_person) of the target person
            static_tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
            geometry_msgs::msg::TransformStamped transformStamped;
            transformStamped.header.stamp = this->now();
            transformStamped.header.frame_id = "zed_camera_link";
            transformStamped.child_frame_id  = "target_person";
            transformStamped.transform.translation = target_point;
            transformStamped.transform.translation.x = target_point.x - target_person_offset_;

            tf2::Quaternion quaternion;
            quaternion.setRPY(0, 0, (std::atan(target_point.y / target_point.x) / M_PI));
            transformStamped.transform.rotation.x = quaternion.x();
            transformStamped.transform.rotation.y = quaternion.y();
            transformStamped.transform.rotation.z = quaternion.z();
            transformStamped.transform.rotation.w = quaternion.w();

            static_tf_broadcaster_->sendTransform(transformStamped);

            // Generate target position and start navigation
            set_goal_point();
        }
        // else if (target_point.x < close_approach_distance_) is
        // already processed within for loop (lines 322-333), so it is removed

        // Initialize target_point
        target_point.x = 0.001;
        target_point.y = 0.0001;
        target_point.z = 0.0001;
    }

//------------------- Joy callback ---------------------------------------------------
    void joyCallback(const sensor_msgs::msg::Joy::SharedPtr joyMsg)
    {
        // Button 9 (OPTIONS): navigation mode
        if (joyMsg->buttons[9] == 1) {
            RCLCPP_INFO(this->get_logger(), "********************************");
            RCLCPP_INFO(this->get_logger(), "     NAVIGATION MODE            ");
            RCLCPP_INFO(this->get_logger(), "********************************");
            navigation_activate_ = 1;
        }
        // Button 12 (PS): manual mode
        else if (joyMsg->buttons[12] == 1) {
            RCLCPP_INFO(this->get_logger(), "********************************");
            RCLCPP_INFO(this->get_logger(), "     MANUAL MODE                ");
            RCLCPP_INFO(this->get_logger(), "********************************");
            navigation_activate_ = 0;
        }
    }

//------------------- Get current arm angle -------------------------------------------
    void arm_pose_Callback(const std_msgs::msg::Int32MultiArray::SharedPtr msgin)
    {
        if (msgin->data.size() >= 4) {
            arm_message_.data = {msgin->data[0], msgin->data[1], msgin->data[2], msgin->data[3]};
        } else {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                "Received arm pose with insufficient data size: %zu", msgin->data.size());
        }
    }

//------------------ Get odom and ZED2 camera position every second --------------------------------------
    void on_timer()
    {
        geometry_msgs::msg::TransformStamped t;

        try {
            t = tf_buffer_->lookupTransform(
                "odom", "zed_camera_link",
                tf2::TimePointZero);

            robot_pose.transform = t.transform;
            robot_pose_yaw = t.transform.rotation.z;

        } catch (const tf2::TransformException & ex) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                "Could not transform odom to zed_camera_link: %s", ex.what());
            return;
        }

        if (pre_zed_goal_msg.data != zed_goal_msg.data) {
            zed_goal_status_->publish(zed_goal_msg);
            pre_zed_goal_msg.data = zed_goal_msg.data;
        }
    }

    //------------ Generate feedback for action_client_ -----------------------------------------------------------
    void feedbackCallback(GoalHandleNavigateToPose::SharedPtr,
                         const std::shared_ptr<const NavigateToPose::Feedback> feedback)
    {
        RCLCPP_INFO(get_logger(), "Distance remaining = %.2f m", feedback->distance_remaining);
    }

    //------------ Generate result for action_client_ -----------------------------------------------------------
    void resultCallback(const GoalHandleNavigateToPose::WrappedResult & result)
    {
        switch (result.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
                RCLCPP_INFO(get_logger(), "Goal Succeeded!");
                zed_goal_msg.data = 3;
                zed_goal_status_->publish(zed_goal_msg);
                break;
            case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_ERROR(get_logger(), "Goal was aborted");
                zed_goal_msg.data = 4;
                zed_goal_status_->publish(zed_goal_msg);
                return;
            case rclcpp_action::ResultCode::CANCELED:
                RCLCPP_ERROR(get_logger(), "Goal was canceled");
                zed_goal_msg.data = 5;
                zed_goal_status_->publish(zed_goal_msg);
                return;
            default:
                RCLCPP_ERROR(get_logger(), "Unknown result code");
                return;
        }
    }

    // ========== メンバ変数 ==========
    rclcpp_action::Client<NavigateToPose>::SharedPtr action_client_;
    rclcpp::Subscription<zed_msgs::msg::ObjectsStamped>::SharedPtr objectSubscriber_;
    rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr  arm_pose_Subscriber_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joySubscriber_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmdVelPublisher_;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::shared_ptr<tf2_ros::Buffer> target_tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> target_tf_listener_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_publisher_;
    rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr  arm_pose_Publisher_;
    rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr  current_arm_pose_Publisher_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr zed_goal_status_;
    rclcpp::TimerBase::SharedPtr timer_{nullptr};

    // Phase 1 improvement: Timer and queue for arm motion
    rclcpp::TimerBase::SharedPtr arm_motion_timer_;
    std::queue<ArmPose> arm_motion_queue_;
    rclcpp::Time last_arm_motion_time_{0, 0, RCL_ROS_TIME};
    int current_arm_duration_ms_ = 0;

    std::string recognization_frame_;
    bool person_finding = false;
    geometry_msgs::msg::Vector3 compare_point;
    geometry_msgs::msg::Vector3 target_point;
    float body_pose_x[20] = {0};
    float body_pose_y[20] = {0};
    std_msgs::msg::Int32 zed_goal_msg;
    std_msgs::msg::Int32 pre_zed_goal_msg;
    geometry_msgs::msg::TransformStamped robot_pose;
    double robot_pose_yaw = 0.0001;
    double leave_distance = 0.60;

    std_msgs::msg::Int32MultiArray arm_message_;
    geometry_msgs::msg::Twist cmdVelMsg_;
    int navigation_activate_ = 0;

    // Velocity parameters
    double forward_vel_fast_;
    double forward_vel_slow_;
    double turn_right_vel_fast_;
    double turn_right_vel_slow_;
    double turn_left_vel_fast_;
    double turn_left_vel_slow_;
    double strafe_right_vel_fast_;
    double strafe_right_vel_slow_;
    double strafe_left_vel_fast_;
    double strafe_left_vel_slow_;

    // Phase 1 improvement: Parameterized thresholds
    double hand_raise_threshold_;
    double target_distance_min_;
    double target_person_offset_;
    double close_approach_distance_;
    int arm_motion_interval_ms_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GoalPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
