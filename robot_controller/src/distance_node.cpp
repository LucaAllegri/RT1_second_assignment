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
            threshold = 0.8;
            is_reversing.data = false;
        }
    private:

        geometry_msgs::msg::Twist check_direction_robot(){
            geometry_msgs::msg::Twist reverse_robot_vel;
            if(vel_input.linear.x < 0){
                reverse_robot_vel.linear.x = 1;
            }else{
                reverse_robot_vel.linear.x = -1;
            }
            if(vel_input.angular.z != 0){
                reverse_robot_vel.angular.z = -vel_input.angular.z;
            }
            return reverse_robot_vel;
        }

        void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg){
     
            scan_ranges = msg->ranges.size();
            float min_dist = msg->range_max;

            for(int i=0; i<scan_ranges ; i++){
                
                float scan_distance = msg->ranges[i];
                if (!std::isnan(scan_distance) && !std::isinf(scan_distance)){
                    min_dist = std::min(min_dist,scan_distance);
                }   
            }

            if(min_dist > threshold){
                is_reversing.data = false;
            }else{
                is_reversing.data = true;
                reverse_state_pub_->publish(is_reversing);

                geometry_msgs::msg::Twist reverse_cmd;
                reverse_cmd = check_direction_robot();
                robot_vel_pub->publish(reverse_cmd);

            }

            if(is_reversing.data && ){
                robot_vel_pub->publish(stop_robot);
                reverse_state_pub_->publish(is_reversing);
            }
        }

        void robot_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg){
            current_robot_pose.pose.position.x = msg->pose.position.x;
            current_robot_pose.pose.position.y = msg->pose.position.y;
        }

        void intermediate_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg){
            vel_input.linear.x = msg->linear.x;
            vel_input.angular.z = msg->angular.z;
            if(!is_reversing.data) {
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
        geometry_msgs::msg::PoseStamped current_robot_pose;
        geometry_msgs::msg::Twist stop_robot;
        geometry_msgs::msg::Twist vel_input;
        std_msgs::msg::Bool is_reversing;
        float threshold;
        int scan_ranges;
        
};

int main(int argc, char * argv[]){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DistanceController>());
    rclcpp::shutdown();
    return 0;
}

