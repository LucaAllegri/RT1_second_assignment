#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/bool.hpp"
#include "robot_custom_msgs/msg/obstacle_info.hpp"
#include "robot_custom_msgs/srv/threshold.hpp"
#include <math.h>
using std::placeholders::_1;

class DistanceController: public rclcpp::Node{
    public:
        DistanceController(): Node("distance_controller"){
            
            //TIMERS

            //PUBLISHERS
            robot_vel_pub= this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
            reverse_state_pub_ = this->create_publisher<std_msgs::msg::Bool>("/is_reversing", 10);
            custom_msg_pub_ = this->create_publisher<robot_custom_msgs::msg::ObstacleInfo>("/obstacle_info", 10);
            robot_moving_state_pub_ = this->create_publisher<std_msgs::msg::Bool>("/robot_moving", 10);

            //SUBSCRIBERS
            intermediate_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>("/intermediate_vel", 10, std::bind(&DistanceController::intermediate_vel_callback, this, _1));
            robot_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("/goal_pose", 10, std::bind(&DistanceController::robot_pose_callback, this, _1));
            scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>("/scan", 10, std::bind(&DistanceController::scan_callback, this, _1));
            
            //PARAMETER
            this->declare_parameter<double>("threshold", 0.8);
            threshold = this->get_parameter("threshold").as_double();
            RCLCPP_INFO(this->get_logger(),"Initial threshold: %.2f", threshold);

            //VARIABLES
            min_dist=10.0;
            is_reversing.data = false;
            dircetion_obstacle = "right";

            //CALLBACK PER UPDATE PARAMETRO 
            param_callback_handle_ = this->add_on_set_parameters_callback( std::bind(&DistanceController::on_param_change,this, std::placeholders::_1));
        }

    private:

        bool robot_in_danger(){
            if (min_dist > threshold){
                return false;
            }else{
                return true;
            }
        }

        // CALLBACK PARAMETRI
        rcl_interfaces::msg::SetParametersResult on_param_change(const std::vector<rclcpp::Parameter> & params) {
            for (const auto & param : params) {
                if (param.get_name() == "threshold") {
                    threshold = param.as_double();
                    RCLCPP_INFO(this->get_logger(),
                                "Threshold updated: %.2f", threshold);
                }
            }
            rcl_interfaces::msg::SetParametersResult result;
            result.successful = true;
            return result;
        }

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
            min_dist = msg->range_max;
            direction_index= -1;

            for(int i=0; i<scan_ranges ; i++){
                
                float scan_distance = msg->ranges[i];
                if (std::isnan(scan_distance) || std::isinf(scan_distance)) {
                    continue;
                }

                if (scan_distance < min_dist) {
                    min_dist = scan_distance;
                    direction_index = i;
                }
            }

            float angle = msg->angle_min + direction_index * msg->angle_increment;

            if (angle >= -M_PI/4 && angle <= M_PI/4){
                dircetion_obstacle = "front";
            }else if (angle > M_PI/4 && angle <= 3*M_PI/4){
                dircetion_obstacle = "left";
            }else if (angle < -M_PI/4 && angle >= -3*M_PI/4){
                dircetion_obstacle = "right";
            }else{
                dircetion_obstacle = "behind";
            }


            msg_obst_info.min_distance_obstacle = min_dist;
            msg_obst_info.direction = dircetion_obstacle;
            msg_obst_info.threshold = threshold;
            custom_msg_pub_->publish(msg_obst_info);

            RCLCPP_INFO(this->get_logger(),
                "Min dist: %.2f | Angle: %.2f rad | Direction: %s",
                min_dist, angle, dircetion_obstacle.c_str());

            if(robot_in_danger()){
                if(!is_reversing.data){
                    is_reversing.data = true;
                    reverse_state_pub_->publish(is_reversing);

                    geometry_msgs::msg::Twist reverse_cmd;
                    reverse_cmd = check_direction_robot();
                    robot_vel_pub->publish(reverse_cmd);
                }
            }else{
                if(is_reversing.data){
                    is_reversing.data = false;
                    robot_vel_pub->publish(stop_robot);
                    robot_moving_state_pub_->publish(is_reversing);
                    reverse_state_pub_->publish(is_reversing);
                }
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
        rclcpp::TimerBase::SharedPtr threshold_timer_;

        //PUBLISHER
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr robot_vel_pub;
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr reverse_state_pub_;
        rclcpp::Publisher<robot_custom_msgs::msg::ObstacleInfo>::SharedPtr custom_msg_pub_;
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr robot_moving_state_pub_;

        //SUBSCRIBERS
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr intermediate_vel_sub_;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr robot_pose_sub_;
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
        
        //SERVICES
        rclcpp::Client<robot_custom_msgs::srv::Threshold>::SharedPtr threshold_client_;

        //PARAMETER
        rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

        //VARIABLES
        robot_custom_msgs::msg::ObstacleInfo msg_obst_info;
        geometry_msgs::msg::PoseStamped current_robot_pose;
        geometry_msgs::msg::Twist stop_robot;
        geometry_msgs::msg::Twist vel_input;
        std_msgs::msg::Bool is_reversing;
        std::string dircetion_obstacle;
        double threshold;
        int scan_ranges;
        float min_dist;
        int direction_index;
        
};

int main(int argc, char * argv[]){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DistanceController>());
    rclcpp::shutdown();
    return 0;
}

