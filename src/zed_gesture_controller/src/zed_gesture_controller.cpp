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

//#include <zed_interfaces/msg/object.hpp>
//#include <zed_interfaces/msg/objects_stamped.hpp>

#include <zed_msgs/msg/object.hpp>
#include <zed_msgs/msg/objects_stamped.hpp>
#include <sensor_msgs/msg/joy.hpp>


using NavigateToPose = nav2_msgs::action::NavigateToPose;
using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;
using PoseStamped = geometry_msgs::msg::PoseStamped;

class GoalPublisher : public rclcpp::Node {
public:
    GoalPublisher() : Node("zed_gesture_controller") {
        // Topic name parameter declaration
        std::string skeleton_topic = this->declare_parameter<std::string>("skeleton_topic", "zed/zed_node/body_trk/skeletons");
        std::string target_arm_pose_topic = this->declare_parameter<std::string>("target_arm_pose_topic", "target_arm_pose");
        std::string current_arm_pose_topic = this->declare_parameter<std::string>("current_arm_pose_topic", "current_arm_pose");
        std::string cmd_vel_topic = this->declare_parameter<std::string>("cmd_vel_topic", "cmd_vel");
        std::string goal_pose_topic = this->declare_parameter<std::string>("goal_pose_topic", "goal_pose");
        std::string zed_goal_status_topic = this->declare_parameter<std::string>("zed_goal_status_topic", "zed_goal_status");

        // Velocity parameter declaration (both hands raised: forward)
        forward_vel_fast_ = this->declare_parameter<double>("forward_vel_fast", 0.5);
        forward_vel_slow_ = this->declare_parameter<double>("forward_vel_slow", 0.1);

        // Velocity parameter declaration (right hand bent: right turn)
        turn_right_vel_fast_ = this->declare_parameter<double>("turn_right_vel_fast", 1.6);
        turn_right_vel_slow_ = this->declare_parameter<double>("turn_right_vel_slow", 0.05);

        // Velocity parameter declaration (left hand bent: left turn)
        turn_left_vel_fast_ = this->declare_parameter<double>("turn_left_vel_fast", -1.6);
        turn_left_vel_slow_ = this->declare_parameter<double>("turn_left_vel_slow", -0.05);

        // Velocity parameter declaration (right hand horizontal: right strafe)
        strafe_right_vel_fast_ = this->declare_parameter<double>("strafe_right_vel_fast", 0.70);
        strafe_right_vel_slow_ = this->declare_parameter<double>("strafe_right_vel_slow", 0.10);

        // Velocity parameter declaration (left hand horizontal: left strafe)
        strafe_left_vel_fast_ = this->declare_parameter<double>("strafe_left_vel_fast", -0.70);
        strafe_left_vel_slow_ = this->declare_parameter<double>("strafe_left_vel_slow", -0.10);

        // Subscribe to human recognition content from ZED
        objectSubscriber_ = this->create_subscription<zed_msgs::msg::ObjectsStamped>(
            skeleton_topic, 1, std::bind(&GoalPublisher::objectCallback, this, std::placeholders::_1));

        arm_pose_Subscriber_ = this->create_subscription<std_msgs::msg::Int32MultiArray>(
            target_arm_pose_topic, 1, std::bind(&GoalPublisher::arm_pose_Callback, this, std::placeholders::_1));

        // Subscribe to joy topic (for switching between navigation mode and manual mode)
        joySubscriber_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "joy", 1, std::bind(&GoalPublisher::joyCallback, this, std::placeholders::_1));

        // TF2 Transform Listener
        recognization_frame_ = this->declare_parameter<std::string>("odom", "zed_camera_link");
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        target_tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        target_tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*target_tf_buffer_);
        timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&GoalPublisher::on_timer, this));

        // Create a publisher for target_arm_position
        arm_pose_Publisher_= this->create_publisher<std_msgs::msg::Int32MultiArray>(target_arm_pose_topic, 10);

        // Create a publisher for target_arm_position
        current_arm_pose_Publisher_= this->create_publisher<std_msgs::msg::Int32MultiArray>(current_arm_pose_topic, 10);

        // Create a publisher for cmd_vel for movement from human recognition
        cmdVelPublisher_ = this->create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic, 1);

        // Create publisher for goal
        goal_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(goal_pose_topic, 10);

        //
        zed_goal_status_ =this->create_publisher<std_msgs::msg::Int32>(zed_goal_status_topic, 1);

        // Create action client for NavigateToPose action
        action_client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");

        // Initialize member variables
        arm_message_.data = {0, 0, 0, 0};  // Initialize as 4-element array
        cmdVelMsg_.linear.x = 0.0;
        cmdVelMsg_.linear.y = 0.0;
        cmdVelMsg_.linear.z = 0.0;
        cmdVelMsg_.angular.x = 0.0;
        cmdVelMsg_.angular.y = 0.0;
        cmdVelMsg_.angular.z = 0.0;

        // Initialize navigation mode / manual mode (default is manual mode)
        navigation_activate_ = 0;

    }

private:
    
//------------------------------------------------------------------------------------------
    void set_goal_point(){
        geometry_msgs::msg::TransformStamped tf;
        // lookupTransform in the try block retrieves the position to generate the TF (target_person) of the target person from the map
        try {
            tf = target_tf_buffer_->lookupTransform(
                "map", "target_person",
                tf2::TimePointZero);
           //robot_pose.translation = t.transform.translation;
            RCLCPP_INFO(this->get_logger(), "------------------- find target_person -----------------");
            RCLCPP_INFO(this->get_logger(), "target_person.x =   : %6.3f"  , tf.transform.translation.x);
            RCLCPP_INFO(this->get_logger(), "target_person.y =   : %6.3f"  , tf.transform.translation.y);
            RCLCPP_INFO(this->get_logger(), "target_person.z =   : %6.3f"  , tf.transform.translation.z);
            RCLCPP_INFO(this->get_logger(), "target_person.r =   : %6.3f"  , tf.transform.rotation.x);
            RCLCPP_INFO(this->get_logger(), "target_person.p =   : %6.3f"  , tf.transform.rotation.y);
            RCLCPP_INFO(this->get_logger(), "target_person.y =   : %6.3f"  , tf.transform.rotation.z);
            RCLCPP_INFO(this->get_logger(), "target_person.w =   : %6.3f"  , tf.transform.rotation.w);
            RCLCPP_INFO(this->get_logger(), "-------------------------------------------------------");

        } catch (const tf2::TransformException & ex) {
            RCLCPP_INFO(
                this->get_logger(), "Could not transform map to target_person: %s" , ex.what());
            return;
        }

        // Create action Goal
        auto goal_msg = NavigateToPose::Goal();
        goal_msg.pose.header.stamp = this->now();
        goal_msg.pose.header.frame_id = "map";
        // Define goal coordinates
        goal_msg.pose.pose.position.x =tf.transform.translation.x;
        goal_msg.pose.pose.position.y =tf.transform.translation.y;
        goal_msg.pose.pose.position.z =tf.transform.translation.z;
        goal_msg.pose.pose.orientation.x =tf.transform.rotation.x;
        goal_msg.pose.pose.orientation.y =tf.transform.rotation.y;
        goal_msg.pose.pose.orientation.z =tf.transform.rotation.z;
        goal_msg.pose.pose.orientation.w =tf.transform.rotation.w;

        // Set feedback callback to display progress
        auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
        send_goal_options.feedback_callback = std::bind(&GoalPublisher::feedbackCallback, this, std::placeholders::_1, std::placeholders::_2);
        send_goal_options.result_callback   = std::bind(&GoalPublisher::resultCallback, this, std::placeholders::_1);

        // Send Goal to server
        action_client_->async_send_goal(goal_msg, send_goal_options);

        PoseStamped goal_pose_msg;
        // Fill in the PoseStamped message fields
        goal_pose_msg.header.stamp = this->now();
        goal_pose_msg.header.frame_id = "map"; // Adjust the frame_id according to your application
        goal_pose_msg.pose.position.x =tf.transform.translation.x;
        goal_pose_msg.pose.position.y =tf.transform.translation.y;
        goal_pose_msg.pose.position.z =tf.transform.translation.z;
        goal_pose_msg.pose.orientation.x =tf.transform.rotation.x;
        goal_pose_msg.pose.orientation.y =tf.transform.rotation.y;
        goal_pose_msg.pose.orientation.z =tf.transform.rotation.z;
        goal_pose_msg.pose.orientation.w =tf.transform.rotation.w;
        goal_publisher_ ->publish(goal_pose_msg);

        // Display goal point
        RCLCPP_INFO(this->get_logger(),"---------- goal_point published --------------");
        RCLCPP_INFO(this->get_logger(),"goal_x       : %4.3f", goal_msg.pose.pose.position.x); 
        RCLCPP_INFO(this->get_logger(),"goal_y       : %4.3f", goal_msg.pose.pose.position.y);
        RCLCPP_INFO(this->get_logger(),"goal_z       : %4.3f", goal_msg.pose.pose.position.z );                
        RCLCPP_INFO(this->get_logger(),"goal_x       : %4.3f", goal_msg.pose.pose.position.x); 
        RCLCPP_INFO(this->get_logger(),"goal_rx      : %4.3f", goal_msg.pose.pose.orientation.x);
        RCLCPP_INFO(this->get_logger(),"goal_ry      : %4.3f", goal_msg.pose.pose.orientation.y);
        RCLCPP_INFO(this->get_logger(),"goal_rz      : %4.3f", goal_msg.pose.pose.orientation.z);
        RCLCPP_INFO(this->get_logger(),"goal_rw      : %4.3f", goal_msg.pose.pose.orientation.w);
        RCLCPP_INFO(this->get_logger(),"----------------------------------------------");

        //arm.header.stamp = ros::Time::now();
        //arm.position[0]= theta;
        //arm.position[1]= -0.5;
        //arm.position[2]=  1.0;    
        //joint_pub.publish(arm);
        //ros::Duration(1.0).sleep();
    }    

//---------------------- Process human recognition content from ZED2i ---------------------------------------------------
    void objectCallback(const zed_msgs::msg::ObjectsStamped::SharedPtr objMsg)
    {
        using namespace std::chrono_literals;

        // Stop processing if navigation_activate_ is 0 (manual mode)
        if (navigation_activate_ == 0) {
            // In manual mode, do not perform automatic control based on human recognition
            return;
        }

        // Skeleton joints indices
        //        16-14   15-17
        //             \ /
        //              0
        //              |
        //       2------1------5
        //       |    |   |    |
        //	     |    |   |    |
        //       3    |   |    6
        //       |    |   |    |
        //       |    |   |    |
        //       4    8   11   7
        //            |   |
        //            |   |
        //            |   |
        //            9   12
        //            |   |
        //            |   |
        //            |   |
        //           10   13
        if (objMsg->objects.size()>0){
            //target_pointを初期化する
            //target_point.x =0.001;
            //target_point.y =0.0001;
            //target_point.z =0.0001;
            //RCLCPP_INFO(this->get_logger(), "I find %ld person", objMsg->objects.size());
            if(zed_goal_msg.data<1){// Exclude person recognition if target has already been found or goal has been reached
                zed_goal_msg.data=1;// Person recognition notification
            }

            // Set arm to initial position just in case
            arm_message_.data = { int32_t(atan(target_point.y/target_point.x)*80)
                                            ,200
                                            ,200
                                            ,-200
                                            };
            arm_pose_Publisher_->publish(arm_message_);

            // When multiple people are found, set the person farther away as the target point
            for(uint i = 0 ; i < objMsg->objects.size()  ; i++ ){
                if(objMsg->objects[i].tracking_state>0){
                    for(int pose_count = 2 ; pose_count <= 7 ; pose_count++ ){// Get position in x direction below the neck
                        //if (isnan(objMsg->objects[i].skeleton_3d.keypoints[3].kp[2])) {
                        if (std::isnan(objMsg->objects[i].skeleton_3d.keypoints[3].kp[1])) {
                            body_pose_x[pose_count] =0;
                        }
                        else{
                            body_pose_x[pose_count] = objMsg->objects[i].skeleton_3d.keypoints[pose_count].kp[1];
                        }
                    }

                    for(int pose_count = 2 ; pose_count <= 7 ; pose_count++ ){// Get position in Y direction below the neck
                        //if (isnan(objMsg->objects[i].skeleton_3d.keypoints[3].kp[2])) {
                        if (std::isnan(objMsg->objects[i].skeleton_3d.keypoints[3].kp[2])) {
                            body_pose_y[pose_count] =0;
                        }
                        else{
                            body_pose_y[pose_count] = objMsg->objects[i].skeleton_3d.keypoints[pose_count].kp[2];
                        }    
                    }

                    target_point.x =objMsg->objects[i].position[0];
                    target_point.y =objMsg->objects[i].position[1];
                    target_point.z =objMsg->objects[i].position[2];    
                    //RCLCPP_INFO(this->get_logger(), "body_pose_y[2] =   : %6.3f", body_pose_y[2]);
                    //RCLCPP_INFO(this->get_logger(), "body_pose_y[4] =   : %6.3f", body_pose_y[4]);
                    //RCLCPP_INFO(this->get_logger(), "body_pose_y[5] =   : %6.3f", body_pose_y[5]);
                    //RCLCPP_INFO(this->get_logger(), "body_pose_y[7] =   : %6.3f", body_pose_y[7]);

                    // Find a person who has raised both hands above the shoulders - move forward when found
                    if(body_pose_y[2] < body_pose_y[3] && body_pose_y[5] < body_pose_y[6]){
                        for(int puc_count=0 ; puc_count < 12 ; puc_count++){
                            cmdVelMsg_.linear.x = forward_vel_fast_;
                            cmdVelPublisher_ ->publish(cmdVelMsg_);
                            rclcpp::sleep_for(20ms);
                        }
                        for(int puc_count=0 ; puc_count < 5 ; puc_count++){
                            cmdVelMsg_.linear.x = forward_vel_slow_;
                            cmdVelPublisher_ ->publish(cmdVelMsg_);
                            rclcpp::sleep_for(20ms);
                        }
                        for(int puc_count=0 ; puc_count < 5 ; puc_count++){
                            cmdVelMsg_.linear.x = 0.0;
                            cmdVelPublisher_ ->publish(cmdVelMsg_);
                            rclcpp::sleep_for(20ms);
                        }
                    }
                     // Turn right when right hand is bent and raised
                    else if(((body_pose_x[4]- 0.10) > body_pose_x[2]) && ((body_pose_y[4]- 0.05) > body_pose_y[2]) ){
                        for(int puc_count=0 ; puc_count < 8 ; puc_count++){
                            cmdVelMsg_.angular.z = turn_right_vel_fast_;
                            cmdVelPublisher_ ->publish(cmdVelMsg_);
                            rclcpp::sleep_for(25ms);
                        }
                        for(int puc_count=0 ; puc_count < 3 ; puc_count++){
                            cmdVelMsg_.angular.z = turn_right_vel_slow_;
                            cmdVelPublisher_ ->publish(cmdVelMsg_);
                            rclcpp::sleep_for(25ms);
                        }
                        for(int puc_count=0 ; puc_count < 5 ; puc_count++){
                            cmdVelMsg_.angular.z = 0.0;
                            cmdVelPublisher_ ->publish(cmdVelMsg_);
                            rclcpp::sleep_for(100ms);
                        }
                    }
                    // Turn left when left hand is bent and raised
                    else if( ((body_pose_x[7]+ 0.10) < body_pose_x[5]) && ((body_pose_y[7]- 0.05) > body_pose_y[5]) ){
                        for(int puc_count=0 ; puc_count < 8 ; puc_count++){
                            cmdVelMsg_.angular.z = turn_left_vel_fast_;
                            cmdVelPublisher_ ->publish(cmdVelMsg_);
                            rclcpp::sleep_for(25ms);
                        }
                        for(int puc_count=0 ; puc_count < 3 ; puc_count++){
                            cmdVelMsg_.angular.z = turn_left_vel_slow_;
                            cmdVelPublisher_ ->publish(cmdVelMsg_);
                            rclcpp::sleep_for(25ms);
                        }
                        for(int puc_count=0 ; puc_count < 5 ; puc_count++){
                            cmdVelMsg_.angular.z = 0.0;
                            cmdVelPublisher_ ->publish(cmdVelMsg_);
                            rclcpp::sleep_for(100ms);
                        }
                    }

                    else if((body_pose_x[4]- 0.35) > body_pose_x[2]){// Move right when right hand is horizontal
                        for(int puc_count=0 ; puc_count < 12 ; puc_count++){
                            cmdVelMsg_.linear.y = strafe_right_vel_fast_;
                            cmdVelPublisher_ ->publish(cmdVelMsg_);
                            rclcpp::sleep_for(20ms);
                        }
                        for(int puc_count=0 ; puc_count < 5 ; puc_count++){
                            cmdVelMsg_.linear.y = strafe_right_vel_slow_;
                            cmdVelPublisher_ ->publish(cmdVelMsg_);
                            rclcpp::sleep_for(20ms);
                        }
                        for(int puc_count=0 ; puc_count < 5 ; puc_count++){
                            cmdVelMsg_.linear.y = 0.0;
                            cmdVelPublisher_ ->publish(cmdVelMsg_);
                            rclcpp::sleep_for(20ms);
                        }
                    }
                    else if((body_pose_x[7]+ 0.35) < body_pose_x[5]){// Move left when left hand is horizontal
                        for(int puc_count=0 ; puc_count < 12 ; puc_count++){
                            cmdVelMsg_.linear.y = strafe_left_vel_fast_;
                            cmdVelPublisher_ ->publish(cmdVelMsg_);
                            rclcpp::sleep_for(20ms);
                        }
                        for(int puc_count=0 ; puc_count < 5 ; puc_count++){
                            cmdVelMsg_.linear.y = strafe_left_vel_slow_;
                            cmdVelPublisher_ ->publish(cmdVelMsg_);
                            rclcpp::sleep_for(20ms);
                        }
                        for(int puc_count=0 ; puc_count < 5 ; puc_count++){
                            cmdVelMsg_.linear.y = 0.0;
                            cmdVelPublisher_ ->publish(cmdVelMsg_);
                            rclcpp::sleep_for(20ms);
                        }
                    }
                    /*
                    // Find a person who has raised their hand above the shoulder
                    else if(body_pose_y[2] < (body_pose_y[4]-0.3) || body_pose_y[5] < (body_pose_y[7]-0.3)){
                        //RCLCPP_INFO(this->get_logger(), "I find you");
                        person_finding = true;//
                        compare_point.x =objMsg->objects[i].position[0];
                        compare_point.y =objMsg->objects[i].position[1];
                        compare_point.z =objMsg->objects[i].position[2];
                        //target_point.x =objMsg->objects[i].position[0];
                        //target_point.y =objMsg->objects[i].position[1];
                        //target_point.z =objMsg->objects[i].position[2];
                        if(compare_point.x > target_point.x){// Target the person who is farther away
                            target_point.x =objMsg->objects[i].position[0];
                            target_point.y =objMsg->objects[i].position[1];
                            target_point.z =objMsg->objects[i].position[2];
                        }
                        RCLCPP_INFO(this->get_logger(),"---------- get target point --------------");
                        RCLCPP_INFO(this->get_logger(), "target_point.x =   : %6.3f", target_point.x);
                        RCLCPP_INFO(this->get_logger(), "target_point.y =   : %6.3f", target_point.y);
                        RCLCPP_INFO(this->get_logger(), "target_point.z =   : %6.3f", target_point.z);
                        RCLCPP_INFO(this->get_logger(),"------------------------------------------");
                        zed_goal_msg.data=2;// Target discovery notification
                        // Align the arm direction to match the angle of the target person
                        arm_message_.data = { int32_t(atan(target_point.y/target_point.x)*80)
                                            ,arm_message_.data[1]
                                            ,arm_message_.data[2]
                                            ,arm_message_.data[3]
                                            };
                        arm_pose_Publisher_->publish(arm_message_);
                        // Make the arm gripper open and close repeatedly
                        rclcpp::sleep_for(600ms);
                        arm_message_.data = { int32_t(atan(target_point.y/target_point.x)*80)
                                            ,arm_message_.data[1]
                                            ,arm_message_.data[2]
                                            ,20
                                            };
                        arm_pose_Publisher_->publish(arm_message_);
                        rclcpp::sleep_for(600ms);
                        arm_message_.data = { int32_t(atan(target_point.y/target_point.x)*80)
                                            ,arm_message_.data[1]
                                            ,arm_message_.data[2]
                                            ,-160
                                            };
                        arm_pose_Publisher_->publish(arm_message_);
                        rclcpp::sleep_for(600ms);
                        arm_message_.data = { int32_t(atan(target_point.y/target_point.x)*80)
                                            ,arm_message_.data[1]
                                            ,arm_message_.data[2]
                                            ,0
                                            };
                        arm_pose_Publisher_->publish(arm_message_);
                        arm_message_.data = { int32_t(atan(target_point.y/target_point.x)*80)
                                            ,arm_message_.data[1]
                                            ,arm_message_.data[2]
                                            ,-160
                                            };
                        arm_pose_Publisher_->publish(arm_message_);

                        current_arm_pose_Publisher_->publish(arm_message_);// Finally output the current angle
                        rclcpp::sleep_for(1500ms);
                        //RCLCPP_INFO(this->get_logger(), "taget_point.x= %5.8f",target_point.x);
                        break;
                    }*/

                    else{
                        person_finding = false;
                    }//end else
                }//end if(objMsg->objects[i].tracking_state>0)
            }//end for
            if(target_point.x <0.8 ){// If a person is nearby, extend the arm forward

                arm_message_.data = { int32_t(atan(target_point.y/target_point.x)*80)
                                    ,200
                                    ,80
                                    ,-200
                                    };
                arm_pose_Publisher_->publish(arm_message_);
                //アームのグリッパーをパクパクさせる
                rclcpp::sleep_for(600ms);
                arm_message_.data = { int32_t(atan(target_point.y/target_point.x)*80)
                                    ,100
                                    ,-20
                                    ,-200
                                    };
                arm_pose_Publisher_->publish(arm_message_);
                rclcpp::sleep_for(600ms);
                arm_message_.data = { int32_t(atan(target_point.y/target_point.x)*80)
                                    ,50
                                    ,-70
                                    ,-200
                                    };
                arm_pose_Publisher_->publish(arm_message_);
                rclcpp::sleep_for(600ms);
                arm_message_.data = { int32_t(atan(target_point.y/target_point.x)*80)
                                    ,0
                                    ,-120
                                    ,-200
                                    };
                arm_pose_Publisher_->publish(arm_message_);
                rclcpp::sleep_for(600ms);
                arm_message_.data = { int32_t(atan(target_point.y/target_point.x)*80)
                                    ,-50
                                    ,-190
                                    ,-200
                                    };
                arm_pose_Publisher_->publish(arm_message_);
                rclcpp::sleep_for(600ms);
                arm_message_.data = { int32_t(atan(target_point.y/target_point.x)*80)
                                    ,-150
                                    ,-190
                                    ,50
                                    };
                arm_pose_Publisher_->publish(arm_message_);
                rclcpp::sleep_for(2000ms);
                arm_message_.data = { int32_t(atan(target_point.y/target_point.x)*80)
                                    ,-150
                                    ,-190
                                    ,-100
                                    };
                arm_pose_Publisher_->publish(arm_message_);
                rclcpp::sleep_for(600ms);
                arm_message_.data = { int32_t(atan(target_point.y/target_point.x)*80)
                                    ,50
                                    ,-100
                                    ,-200
                                    };
                arm_pose_Publisher_->publish(arm_message_);
                rclcpp::sleep_for(600ms);
                arm_message_.data = { int32_t(atan(target_point.y/target_point.x)*80)
                                    ,100
                                    ,80
                                    ,-200
                                    };
                arm_pose_Publisher_->publish(arm_message_);
                rclcpp::sleep_for(600ms);
                arm_message_.data = { int32_t(atan(target_point.y/target_point.x)*80)
                                    ,200
                                    ,200
                                    ,-200
                                    };
                arm_pose_Publisher_->publish(arm_message_);
                // Make the arm gripper open and close repeatedly
                rclcpp::sleep_for(600ms);

            }
        }//end if (objMsg->objects.size()>0)
        else{
        // Since something is being recognized at the callback point, nothing is done here.
        }
        // Initialize target point
        target_point.x =0.001;
        target_point.y =0.0001;
        target_point.z =0.0001;
    } //end objectCallback

//------------------- Joy callback (switch between navigation mode and manual mode) -----------------------------------------------
void joyCallback(const sensor_msgs::msg::Joy::SharedPtr joyMsg)
{
    // Button 9 (OPTIONS): navigation mode
    if(joyMsg->buttons[9] == 1){
        RCLCPP_INFO(this->get_logger(), "********************************");
        RCLCPP_INFO(this->get_logger(), "     NAVIGATION MODE            ");
        RCLCPP_INFO(this->get_logger(), "********************************");
        navigation_activate_ = 1;
    }
    // Button 12 (PS): manual mode
    else if(joyMsg->buttons[12] == 1){
        RCLCPP_INFO(this->get_logger(), "********************************");
        RCLCPP_INFO(this->get_logger(), "     MANUAL MODE                ");
        RCLCPP_INFO(this->get_logger(), "********************************");
        navigation_activate_ = 0;
    }
}

//------------------- Get current arm angle -----------------------------------------------
void arm_pose_Callback(const std_msgs::msg::Int32MultiArray::SharedPtr msgin)
    {
        arm_message_.data ={  msgin->data[0]
                            ,msgin->data[1]
                            ,msgin->data[2]
                            ,msgin->data[3]
                            };
        //arm_pose_Publisher_->publish(arm_message_);
    }
//------------------ Get odom and ZED2 camera position every second --------------------------------------
    void on_timer()
    {
        geometry_msgs::msg::TransformStamped t;
        // lookupTransform in the try block retrieves the camera position
        try {
            t = tf_buffer_->lookupTransform(
                "odom", "zed_camera_link",
                tf2::TimePointZero);
           //robot_pose.translation = t.transform.translation;
            robot_pose.transform      = t.transform;  
            robot_pose_yaw  = t.transform.rotation.z; 
            /*
            RCLCPP_INFO(this->get_logger(), "robot_pose.x =   : %6.3f", robot_pose.transform.translation.x);
            RCLCPP_INFO(this->get_logger(), "robot_pose.y =   : %6.3f", robot_pose.transform.translation.y);
            RCLCPP_INFO(this->get_logger(), "robot_pose.z =   : %6.3f", robot_pose.transform.translation.z);
            RCLCPP_INFO(this->get_logger(), "robot_rotate.z =   : %6.3f", robot_pose.transform.rotation.z);
            RCLCPP_INFO(this->get_logger(), "robot_rotate.w =   : %6.3f", robot_pose.transform.rotation.w);
            RCLCPP_INFO(this->get_logger(), "---------------------------------------------------");
            */

        } catch (const tf2::TransformException & ex) {
            RCLCPP_INFO(
                this->get_logger(), "Could not transform odom to zed_camera_link: %s" , ex.what());
            return;
        }
        if(pre_zed_goal_msg.data!=zed_goal_msg.data){
            zed_goal_status_->publish(zed_goal_msg);// Notify goal recognition state
            pre_zed_goal_msg.data=zed_goal_msg.data;
        }
    }

    //------------ Generate feedback for action_client_ -----------------------------------------------------------
    void feedbackCallback(GoalHandleNavigateToPose::SharedPtr,const std::shared_ptr<const NavigateToPose::Feedback> feedback)
    {
        RCLCPP_INFO(get_logger(), "Distance remaininf = %f", feedback->distance_remaining);
    }
    //------------ Generate result for action_client_ -----------------------------------------------------------
    void resultCallback(const GoalHandleNavigateToPose::WrappedResult & result)
    {
        switch (result.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
                RCLCPP_INFO(get_logger(), "Success!!!");
                zed_goal_msg.data=3;// Notify that goal has been reached
                zed_goal_status_->publish(zed_goal_msg);
                break;
            case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_ERROR(get_logger(), "Goal was aborted");
                zed_goal_msg.data=4;// Notify that goal has been reached
                zed_goal_status_->publish(zed_goal_msg);
                return;
            case rclcpp_action::ResultCode::CANCELED:
                RCLCPP_ERROR(get_logger(), "Goal was canceled");
                zed_goal_msg.data=5;// Notify that goal has been reached
                zed_goal_status_->publish(zed_goal_msg);
                return;
            default:
                RCLCPP_ERROR(get_logger(), "Unknown result code");
                return;
        }
    }

    rclcpp_action::Client<NavigateToPose>::SharedPtr action_client_;
    rclcpp::Subscription<zed_msgs::msg::ObjectsStamped>::SharedPtr objectSubscriber_;
    rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr  arm_pose_Subscriber_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joySubscriber_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr      cmdVelPublisher_;

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
    std::string recognization_frame_;
    bool person_finding;                        // Flag when person recognition is successful
    geometry_msgs::msg::Vector3 compare_point;  // For acquiring position of person with raised hand (each person's position)
    geometry_msgs::msg::Vector3 target_point;   // For acquiring position of person with raised hand (final target position of the selected person after comparison)
    float body_pose_x[20];
    float body_pose_y[20];
    std_msgs::msg::Int32 zed_goal_msg;
    std_msgs::msg::Int32 pre_zed_goal_msg;
    geometry_msgs::msg::TransformStamped robot_pose;  // For acquiring robot position between odom→zed_camera_link
    double robot_pose_yaw=0.0001;              // For acquiring target position angle from ZED2 object recognition
    double leave_distance=0.60;                // Specify one step distance to set goal at a position one step back

    // Member variables moved from global variables
    std_msgs::msg::Int32MultiArray arm_message_;
    geometry_msgs::msg::Twist cmdVelMsg_;

    // navigation mode / manual mode flag
    int navigation_activate_;

    // Velocity parameters
    double forward_vel_fast_;       // Both hands raised: forward (fast)
    double forward_vel_slow_;       // Both hands raised: forward (slow)
    double turn_right_vel_fast_;    // Right hand bent: right turn (fast)
    double turn_right_vel_slow_;    // Right hand bent: right turn (slow)
    double turn_left_vel_fast_;     // Left hand bent: left turn (fast)
    double turn_left_vel_slow_;     // Left hand bent: left turn (slow)
    double strafe_right_vel_fast_;  // Right hand horizontal: right strafe (fast)
    double strafe_right_vel_slow_;  // Right hand horizontal: right strafe (slow)
    double strafe_left_vel_fast_;   // Left hand horizontal: left strafe (fast)
    double strafe_left_vel_slow_;   // Left hand horizontal: left strafe (slow)

};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GoalPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}