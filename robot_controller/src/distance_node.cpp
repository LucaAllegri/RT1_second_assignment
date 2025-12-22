#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/bool.hpp"
#include <math.h>
using std::placeholders::_1;

class DistanceController: public rclcpp::Node{
    public:
        DistanceController(): Node("distance_controller"){
            
            //TIMERS

            //PUBLISHERS
            robot_vel_pub= this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
            reverse_state_pub_ = this->create_publisher<std_msgs::msg::Bool>("/is_reversing", 10);

            //SUBSCRIBERS
            intermediate_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>("/intermediate_vel", 10, std::bind(&DistanceController::intermediate_vel_callback, this, _1));
            robot_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("/goal_pose", 10, std::bind(&DistanceController::robot_pose_callback, this, _1));
            scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>("/scan", 10, std::bind(&DistanceController::scan_callback, this, _1));
            
            //VARIABLES
            vel_input.linear.x = 0.0;
            vel_input.angular.z = 0.0;
            threshold = 2.0;
            is_reversing = false;
        }
    private:

        bool robot_in_danger(){ 
            for(int i=0; i<scan_ranges ; i++){
                if(msg->ranges[i] < threshold){
                    is_reversing=true;
                    reverse_state_pub_->publish(is_reversing);
                    vel_input.linear.x = -(msg->linear.x);
                    vel_input.angular.z = -(msg->angular.z);
                    robot_vel_pub->publish(vel_input);
                }
            }
        }

        void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg){
            scan_ranges = msg->ranges.size();
            scan_angle_min = msg->angle_min;
            scan_angle_max = msg->angle_max;
            scan_angle_increment = msg->angle_increment;

            if(robot_in_danger)

            is_reversing = false;

            for(int i=0; i<scan_ranges ; i++){
                if(msg->ranges[i] < threshold){
                    is_reversing=true;
                    reverse_state_pub_->publish(is_reversing);
                    vel_input.linear.x = -(msg->linear.x);
                    vel_input.angular.z = -(msg->angular.z);
                    robot_vel_pub->publish(vel_input);
                }
            }

            reverse_state_pub_->publish(is_reversing);

        }
        void robot_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg){
            robot_pose.pose.position.x = msg->pose.position.x;
            robot_pose.pose.position.y = msg->pose.position.y;
            robot_pose.pose.position.z = msg->pose.position.z;
        }

        void intermediate_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg){
            vel_input.linear.x = msg->linear.x;
            vel_input.angular.z = msg->angular.z;
            if(!is_reversing) {
                robot_vel_pub->publish(vel_input);
            }
        }

        //TIMERS

        //PUBLISHER
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr robot_vel_pub;
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr reverse_state_pub_;

        //SUBSCRIBERS
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr intermediate_vel_sub_;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr robot_pose_sub_;
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
        
        //VARIABLES
        geometry_msgs::msg::Twist vel_input;
        geometry_msgs::msg::PoseStamped robot_pose;
        float32 threshold;
        float32 scan_angle_min; 
        float32 scan_angle_max; 
        float32 scan_angle_increment; 
        float32 scan_time_increment; 
        float32 scan_angle_min; 
        float32 scan_ranges;
        bool is_reversing;
        
};

int main(int argc, char * argv[]){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DistanceController>());
    rclcpp::shutdown();
    return 0;
}

