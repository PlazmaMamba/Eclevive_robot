#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/int32.hpp"
//#include <cinterface/speaker.hpp>

sensor_msgs::msg::JointState jointStateMsg;
auto arm_message = std_msgs::msg::Int32MultiArray();
int navigation_activate=0;
geometry_msgs::msg::Twist cmdVelMsg;

class joy_mecanum_controller_node : public rclcpp::Node
{
public:
    joy_mecanum_controller_node() : Node("joy_mecanum_controller_node")
    {
        // Create a publisher for cmd_vel
        cmdVelPublisher_ = this->create_publisher<geometry_msgs::msg::Twist>("mecanum/cmd_vel", 1);

        // Create a publisher for joint_state
        jointStatePublisher_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_state", 1);
        
        // Create a publisher for target_arm_position
        intarrayPublisher_= this->create_publisher<std_msgs::msg::Int32MultiArray>("target_arm_pose", 1);
        
        // Create timer to publish ems_call at 30Hz
        timer_ = this->create_wall_timer(std::chrono::milliseconds(30), std::bind(&joy_mecanum_controller_node::timerCallback, this));

        // Create timer to publish ems_call at 20Hz
        arm_timer_ = this->create_wall_timer(std::chrono::milliseconds(50), std::bind(&joy_mecanum_controller_node::arm_timerCallback, this));

        // Create a subscriber for joy
        joySubscriber_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "joy", 1, std::bind(&joy_mecanum_controller_node::joyCallback, this, std::placeholders::_1));

        // Create a subscriber for joy
        cmdSubscriber_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", 1, std::bind(&joy_mecanum_controller_node::cmdCallback, this, std::placeholders::_1)); 
        
        loadSubscriber_ = this->create_subscription<std_msgs::msg::Float32>(
            "gripper_load", 1, std::bind(&joy_mecanum_controller_node::loadCallback, this, std::placeholders::_1)); 

        emsSubscriber_ = this->create_subscription<std_msgs::msg::Int32>(
            "ems_call", 1, std::bind(&joy_mecanum_controller_node::emsCallback, this, std::placeholders::_1)); 

        current_arm_pose_Subscriber_ = this->create_subscription<std_msgs::msg::Int32MultiArray>(
            "current_arm_pose", 1, std::bind(&joy_mecanum_controller_node::CurrentArmPoseCallback, this, std::placeholders::_1));    
    }

private:
    void joyCallback(const sensor_msgs::msg::Joy::SharedPtr joyMsg)
    {        
        if(joyMsg->buttons[9] == 1){
            RCLCPP_INFO(get_logger(), "********************************");
            RCLCPP_INFO(get_logger(), "     navigation mode            ");
            RCLCPP_INFO(get_logger(), "********************************");
            navigation_activate = 1;
            cmdVelMsg.linear.x = 0;
            cmdVelMsg.linear.y = 0;
            cmdVelMsg.linear.z = 0.0;
            cmdVelMsg.angular.x = 0.0;
            cmdVelMsg.angular.y = 0.0;
            cmdVelMsg.angular.z = 0;

            // Fill cmdVelMsg with appropriate values based on joyMsg
            cmdVelPublisher_->publish(cmdVelMsg);
        }
        else if(joyMsg->buttons[12] == 1){
            RCLCPP_INFO(get_logger(), "********************************");
            RCLCPP_INFO(get_logger(), "     manual mode                ");
            RCLCPP_INFO(get_logger(), "********************************");
            navigation_activate = 0;
        }

        else if(joyMsg->buttons[0] == 1){// Square button - Gripper grasp
             //jointStateMsg.position[3]=-0.96;
             gripper_activate=1;
        }
        else if(joyMsg->buttons[2] == 1){// Circle button - Gripper release
            jointStateMsg.position[3]=0.0;
            gripper_activate=0;
        }

        
        if(navigation_activate == 0){
            // Publish the cmd_vel message            
            cmdVelMsg.linear.x = joyMsg->axes[1]*1.00*(1+joyMsg->buttons[1]*1.3);
            cmdVelMsg.linear.y = joyMsg->axes[0]*1.00*(1+joyMsg->buttons[1]*1.3);
            cmdVelMsg.linear.z = 0.0;
            cmdVelMsg.angular.x = 0.0;
            cmdVelMsg.angular.y = 0.0;
            cmdVelMsg.angular.z = joyMsg->axes[2]*1.50*(1+joyMsg->buttons[1]*1.3);
            /* ems value menu
            13:all stop
            12:stop forward
            11:stop backward
            3:slow forward and backward
            2 :slow forward
            1 :slow backward
            */
            if(ems_value >0){
                if(ems_value == 13){
                    //cmdVelMsg.linear.x = cmdVelMsg.linear.x *0,13;
                    //cmdVelMsg.linear.y = cmdVelMsg.linear.y *0.13;
                    cmdVelMsg.linear.x = cmdVelMsg.linear.x *0.35;
                    cmdVelMsg.linear.y = cmdVelMsg.linear.y *0.35;
                    cmdVelMsg.angular.z = cmdVelMsg.angular.z*0.5;
                }
                else if(ems_value == 12 && cmdVelMsg.linear.x>0){//pointcloud2 ems
                    //cmdVelMsg.linear.x = cmdVelMsg.linear.x *0.15;
                    //cmdVelMsg.linear.y = cmdVelMsg.linear.y*0.2;
                    cmdVelMsg.linear.x = cmdVelMsg.linear.x *0.4;
                    cmdVelMsg.linear.y = cmdVelMsg.linear.y*0.5;
                }
                else if(ems_value == 11 && cmdVelMsg.linear.x < 0){//laser ems
                    //cmdVelMsg.linear.x = cmdVelMsg.linear.x*0.15;
                    //cmdVelMsg.linear.y = cmdVelMsg.linear.y*0.2;
                    cmdVelMsg.linear.x = cmdVelMsg.linear.x*0.4;
                    cmdVelMsg.linear.y = cmdVelMsg.linear.y*0.5;

                }
                else if(ems_value == 3){//pointcloud2 laser warn
                    //cmdVelMsg.linear.x = joyMsg->axes[1]*0.2;
                    //cmdVelMsg.linear.y = joyMsg->axes[0]*0.2;
                    cmdVelMsg.linear.x = joyMsg->axes[1]*0.3;
                    cmdVelMsg.linear.y = joyMsg->axes[0]*0.3;
                }
                else if(ems_value == 2 && cmdVelMsg.linear.x>0){//pointcloud2 warn
                    //cmdVelMsg.linear.x = joyMsg->axes[1]*0.15;
                    //cmdVelMsg.linear.y = joyMsg->axes[0]*0.2;
                    cmdVelMsg.linear.x = joyMsg->axes[1]*0.4;
                    cmdVelMsg.linear.y = joyMsg->axes[0]*0.5;
                }                
                else if(ems_value == 1){//laser warm
                    //cmdVelMsg.linear.x = joyMsg->axes[1]*0.25;
                    //cmdVelMsg.linear.y = joyMsg->axes[0]*0.25;
                    cmdVelMsg.linear.x = joyMsg->axes[1]*0.6;
                    cmdVelMsg.linear.y = joyMsg->axes[0]*0.6;
                    
                }
                

            }
            // Fill cmdVelMsg with appropriate values based on joyMsg
            //cmdVelPublisher_->publish(cmdVelMsg);
        }

        // Publish the joint_state message        
        //sensor_msgs::msg::JointState jointStateMsg;
        jointStateMsg.header.stamp = this->now();
        if(jointStateMsg.position[0]<=0.6 && jointStateMsg.position[0]>=-0.6){
            jointStateMsg.position[0] = jointStateMsg.position[0]+joyMsg->axes[6]*0.015;
        }
        else if(jointStateMsg.position[0]>=0.6){
            jointStateMsg.position[0]=0.6;
        }
        else{
            jointStateMsg.position[0]=-0.6;
        }
        jointStateMsg.position[1] = joyMsg->axes[4]*1.0;// R2 button - Lower joint 1
        jointStateMsg.position[2] = joyMsg->axes[3]*1.0;// L2 button - Raise joint 2
        // Gripper grasp by up/down buttons
        if(abs(joyMsg->axes[7])>0){
            gripper_activate=0;// Turn off if gripper was attempting to grasp
            if(jointStateMsg.position[3]<=0.3 && jointStateMsg.position[3]>=-1.05){
                jointStateMsg.position[3] = jointStateMsg.position[3]+joyMsg->axes[7]*0.04;
                
            }
            else if(jointStateMsg.position[3]>=0.3){
                jointStateMsg.position[3]=0.3;
            }
            else{
                jointStateMsg.position[3]=-1.05;
            }
        }
        //jointStatePublisher_->publish(jointStateMsg);        
        //intarrayPublisher_->publish(arm_message);

    }

    void cmdCallback(const geometry_msgs::msg::Twist::SharedPtr cmdMsg)
    {
        if(navigation_activate ==1){   // Publish immediately if in navigation mode
            cmdVelMsg.linear.x = cmdMsg->linear.x;
            cmdVelMsg.linear.y = cmdMsg->linear.y;
            cmdVelMsg.linear.z = 0.0;
            cmdVelMsg.angular.x = 0.0;
            cmdVelMsg.angular.y = 0.0;
            cmdVelMsg.angular.z = cmdMsg->angular.z;

            if(ems_value >0){
                if(ems_value ==13){//pointcloud2 laser ems
                    cmdVelMsg.linear.x  = cmdVelMsg.linear.x*0.6;
                    cmdVelMsg.linear.y  = cmdVelMsg.linear.y*0.6;
                    cmdVelMsg.angular.z = cmdVelMsg.angular.z*0.6;
                }
                if(ems_value ==12 && cmdVelMsg.linear.x>0){//pointcloud2 ems
                    cmdVelMsg.linear.x  = cmdVelMsg.linear.x*0.6;
                    cmdVelMsg.linear.y  = cmdVelMsg.linear.y*0.6;
                }
                else if(ems_value == 11 && cmdVelMsg.linear.x < 0){//laser ems
                    cmdVelMsg.linear.x  = cmdVelMsg.linear.x*0.6;
                    cmdVelMsg.linear.y  = cmdVelMsg.linear.y*0.6;
                }
            }
            cmdVelPublisher_ ->publish(cmdVelMsg);
        }
    }

    void timerCallback()
    {
        // Publish at timer interval if in manual mode
        if(navigation_activate == 0){
            cmdVelPublisher_ ->publish(cmdVelMsg);
        }
    }

    void arm_timerCallback()
    {
        // If gripper_Activate=1, close the gripper. Stop closing when it reaches 0.
        if(gripper_activate==1){
            if(jointStateMsg.position[3]>-1.05){
                jointStateMsg.position[3]=jointStateMsg.position[3]-0.07;
            }
        }
        arm_message.data ={  int(jointStateMsg.position[0]*200)
                ,int(jointStateMsg.position[1]*200)
                ,int(jointStateMsg.position[2]*200)
                ,int(jointStateMsg.position[3]*200)
                };

        // Publish at timer interval if in manual mode
        if(navigation_activate == 0){
            jointStatePublisher_->publish(jointStateMsg);
            intarrayPublisher_->publish(arm_message);
        }
    }

    // Detect gripper load and adjust gripper opening/closing
    void loadCallback(const std_msgs::msg::Float32::SharedPtr loadMsg)
    {
        if(loadMsg->data >1500){// Set maximum gripper load to 1500g
            gripper_activate=0;
            jointStateMsg.position[3] =jointStateMsg.position[3] +0.002;
            //jointStateMsg.position[3] = jointStateMsg.position[3]+0.002;// Open mouth
            arm_message.data ={int(jointStateMsg.position[0]*200)
                            ,int(jointStateMsg.position[1]  *200)
                            ,int(jointStateMsg.position[2]  *200)
                            ,int(jointStateMsg.position[3]  *200)
                            };
            jointStatePublisher_->publish(jointStateMsg);
            intarrayPublisher_->publish(arm_message);
        }
    }    

    void emsCallback(const std_msgs::msg::Int32::SharedPtr emsMsg)
    {
        ems_value = emsMsg ->data;
    }

    void CurrentArmPoseCallback(const std_msgs::msg::Int32MultiArray::SharedPtr Msgin)
    {
        arm_message.data ={  Msgin->data[0]
                            ,Msgin->data[1]
                            ,Msgin->data[2]
                            ,Msgin->data[3]
                            };
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr      cmdVelPublisher_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr   jointStatePublisher_;
    rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr intarrayPublisher_;
    rclcpp::TimerBase::SharedPtr                                 timer_;
    rclcpp::TimerBase::SharedPtr                                 arm_timer_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr       joySubscriber_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr   cmdSubscriber_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr      loadSubscriber_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr        emsSubscriber_;
    rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr current_arm_pose_Subscriber_;
    int ems_value = 0;
    int gripper_activate=0;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<joy_mecanum_controller_node>();
    jointStateMsg.position.resize(4);
    jointStateMsg.position[0]=0;
    jointStateMsg.position[1]=0;
    jointStateMsg.position[2]=0;
    jointStateMsg.position[3]=0;
    jointStateMsg.name.resize(4);
    jointStateMsg.name[0] = "arm_pan_joint";
    jointStateMsg.name[1] = "arm_tilt1_joint";
    jointStateMsg.name[2] = "arm_tilt2_joint";
    jointStateMsg.name[3] = "gripper_joint";
    

    RCLCPP_INFO(node->get_logger(), "joy_mecanum_controller start ");

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
