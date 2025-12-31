#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/int32.hpp"

class MinValueDetector : public rclcpp::Node
{
public:
    MinValueDetector() : Node("safety_sensor")
    {
        pointcloud_subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "zed/zed_node/point_cloud/cloud_registered", 1,std::bind(&MinValueDetector::pointcloud_callback, this, std::placeholders::_1));
            
        // Subscribe to the scan topic
        subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            //"back_scanfiltered", 1, std::bind(&MinValueDetector::scanCallback, this, std::placeholders::_1));
            "scan_filtered", 1, std::bind(&MinValueDetector::scanCallback, this, std::placeholders::_1));

        // Create timer to publish ems_call at 30Hz
        timer_ = this->create_wall_timer(std::chrono::milliseconds(30), std::bind(&MinValueDetector::timerCallback, this));

        // Publish to the std_msgs topic
        publisher_ = this->create_publisher<std_msgs::msg::Int32>("ems_call", 2);
    }

private:
    void pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        // Get data from point cloud
        sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
        sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
        sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");

        // Initialize minimum value
        float min_distance = std::numeric_limits<float>::max();

        // Process each point in the point cloud data
        for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z)
        {
            // Get point coordinates
            float x = *iter_x;
            float y = *iter_y;
            float z = *iter_z;

            // Calculate distance from the acquired point
            float distance = std::sqrt(x*x + y*y + z*z);

            // Update minimum distance
            if (distance < min_distance)
            {
                min_distance = distance;
                //RCLCPP_INFO(get_logger(),"min_point_distance = %6.4f ",min_distance );
            }
        }

        // Stop forward movement if point cloud is found below 32cm
        if(min_distance <= 0.34){
            point_ems_call_value_ = 11;
            RCLCPP_INFO(get_logger(),"min_distance = %6.4f ",min_distance );
        }
        else if(min_distance <= 0.38){
            point_ems_call_value_ =  1;
            RCLCPP_INFO(get_logger(),"min_distance = %6.4f ",min_distance );
        }
        else{
            point_ems_call_value_ = 0;
        }

    }

    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        // Check if the minimum value is less than or equal to 0.15m
        float laser_min_range = std::numeric_limits<float>::infinity();
        for (float range : msg->ranges)
        {
            if (!std::isnan(range) && range < laser_min_range && range >0.15 )
            {
                laser_min_range = range;
                //RCLCPP_INFO(get_logger(),"min_laser_distance = %6.4f ",laser_min_range );
            }
        }

        // Stop backward movement if point cloud is found below 18cm
        if(laser_min_range <= 0.30){
            laser_ems_call_value_ = 11;
        }
        else if(laser_min_range <= 0.38){
            laser_ems_call_value_ =  1;
        }
        else{
            laser_ems_call_value_ = 0;
        }
    }

    void timerCallback()
    {
        // Publish ems_call_value_
        auto ems_value = std_msgs::msg::Int32();
        /* ems value menu
        13:all stop
        12:stop forward
        11:stop backward
        3:slow forward and backward
        2 :slow forward
        1 :slow backward
        */
        if(laser_ems_call_value_ == 11 || point_ems_call_value_ == 11){
            if(laser_ems_call_value_ == 11 && point_ems_call_value_ == 11){
                ems_value.data = 13;
                RCLCPP_INFO(get_logger(),"stop forward and backward !" );
            }
            else if(point_ems_call_value_ == 11){
                ems_value.data = 12;
                RCLCPP_INFO(get_logger(),"stop forward " );
            }
            else{
                ems_value.data = 11;
                RCLCPP_INFO(get_logger(),"stop backard !" );
            }
        }

        else if(laser_ems_call_value_ == 1 || point_ems_call_value_ == 1){
            if(laser_ems_call_value_ == 1 && point_ems_call_value_ == 1){
                ems_value.data = 3;
                RCLCPP_INFO(get_logger(),"slow forward amd backward !" );
            }
            else if(point_ems_call_value_ == 1){
                ems_value.data = 2;
                RCLCPP_INFO(get_logger(),"slow forward");
            }
            else{
                ems_value.data = 1;
                RCLCPP_INFO(get_logger(),"slow backward !" );
            }    
        }
        else{
            ems_value.data = 0;
        }

        publisher_->publish(ems_value);

    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_subscription_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    int point_ems_call_value_ = 0;
    int laser_ems_call_value_ = 0;

};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MinValueDetector>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}