#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

class OdomPublisher : public rclcpp::Node
{
public:
    OdomPublisher() : Node("odom_publisher")
    {
        // Initialize TF2 buffer and listener
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // Create Odometry publisher
        odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 1);

        // Create timer (runs at 10Hz)
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50),
            std::bind(&OdomPublisher::timer_callback, this));

        RCLCPP_INFO(this->get_logger(), "Odometry publisher node has been started");
    }

private:
    void timer_callback()
    {
        try {
            // Get transform from odom frame to zed_camera_link frame
            geometry_msgs::msg::TransformStamped transform_stamped =
                tf_buffer_->lookupTransform("odom", "zed_camera_link", tf2::TimePointZero);

            // Create Odometry message
            nav_msgs::msg::Odometry odom_msg;
            odom_msg.header = transform_stamped.header;
            odom_msg.child_frame_id = transform_stamped.child_frame_id;

            // Set Pose information
            odom_msg.pose.pose.position.x = transform_stamped.transform.translation.x;
            odom_msg.pose.pose.position.y = transform_stamped.transform.translation.y;
            odom_msg.pose.pose.position.z = transform_stamped.transform.translation.z;
            //odom_msg.pose.pose.orientation = transform_stamped.transform.rotation;
            odom_msg.pose.pose.orientation.x = transform_stamped.transform.rotation.x;
            odom_msg.pose.pose.orientation.y = transform_stamped.transform.rotation.y;
            odom_msg.pose.pose.orientation.z = transform_stamped.transform.rotation.z;
            odom_msg.pose.pose.orientation.w = transform_stamped.transform.rotation.w;

            // Calculate Twist information (calculate velocity from difference with previous position)
            if (last_transform_) {
                double dt = (transform_stamped.header.stamp.sec - last_transform_->header.stamp.sec) +
                           (transform_stamped.header.stamp.nanosec - last_transform_->header.stamp.nanosec) * 1e-9;

                if (dt > 0) {
                    odom_msg.twist.twist.linear.x =
                        (transform_stamped.transform.translation.x - last_transform_->transform.translation.x) / dt;
                    odom_msg.twist.twist.linear.y =
                        (transform_stamped.transform.translation.y - last_transform_->transform.translation.y) / dt;
                    odom_msg.twist.twist.linear.z =
                        (transform_stamped.transform.translation.z - last_transform_->transform.translation.z) / dt;

                    // Angular velocity calculation is omitted for simplification
                }
            }

            // Store current transform
            last_transform_ = std::make_unique<geometry_msgs::msg::TransformStamped>(transform_stamped);

            // Publish Odometry message
            odom_publisher_->publish(odom_msg);
        }
        catch (const tf2::TransformException& ex) {
            RCLCPP_WARN(this->get_logger(), "Could not transform: %s", ex.what());
        }
    }

    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::unique_ptr<geometry_msgs::msg::TransformStamped> last_transform_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OdomPublisher>());
    rclcpp::shutdown();
    return 0;
}