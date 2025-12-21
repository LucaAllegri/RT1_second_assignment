#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/int32.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <math.h>
using std::placeholders::_1;

class DistanceController: public rclcpp::Node{
    public:
        DistanceController(): Node("distance_controller"){
            
            //TIMERS

            //PUBLISHERS
            robot_vel_pub= this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

            //SUBSCRIBERS
            intermediate_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>("/intermediate_vel", 10, std::bind(&DistanceController::intermediate_vel_callback, this, _1));
            robot_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("/goal_pose", 10, std::bind(&DistanceController::robot_pose_callback, this, _1));
            
            //VARIABLES
            vel_input.linear.x = 0.0;
            vel_input.angular.z = 0.0;
        }
    private:

        void robot_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg){
            robot_pose.pose.position.x = msg->pose.position.x;
            robot_pose.pose.position.y = msg->pose.position.y;
            robot_pose.pose.position.z = msg->pose.position.z;
        }

        void intermediate_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg){
            vel_input.linear.x = msg->linear.x;
            vel_input.angular.z = msg->angular.z;
            robot_vel_pub->publish(vel_input);
        }

        //TIMERS

        //PUBLISHER
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr robot_vel_pub;
        
        //SUBSCRIBERS
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr intermediate_vel_sub_;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr robot_pose_sub_;

        //VARIABLES
        geometry_msgs::msg::Twist vel_input;
        geometry_msgs::msg::PoseStamped robot_pose;
        
};

int main(int argc, char * argv[]){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DistanceController>());
    rclcpp::shutdown();
    return 0;
}

