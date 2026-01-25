#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/bool.hpp"
#include "robot_custom_msgs/msg/obstacle_info.hpp"
#include "robot_custom_msgs/srv/threshold.hpp"
#include "robot_custom_msgs/srv/fixed_point.hpp"
#include "std_msgs/msg/float32.hpp"
#include <math.h>
using std::placeholders::_1;
using std::placeholders::_2;

class DistanceController: public rclcpp::Node{
    public:
        DistanceController(): Node("distance_controller"){
            
            //TIMERS

            //PUBLISHERS
            custom_msg_pub_ = this->create_publisher<robot_custom_msgs::msg::ObstacleInfo>("/obstacle_info", 10);
            reverse_state_pub_ = this->create_publisher<std_msgs::msg::Bool>("/is_reversing", 10);
            robot_vel_pub= this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
            dist_rob_point_pub_= this->create_publisher<std_msgs::msg::Float32>("/dis_robot_point", 10);
            
            //SUBSCRIBERS
            intermediate_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>("/intermediate_vel", 10, std::bind(&DistanceController::intermediate_vel_callback, this, _1));
            robot_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("/goal_pose", 10, std::bind(&DistanceController::robot_pose_callback, this, _1));
            scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>("/scan", 10, std::bind(&DistanceController::scan_callback, this, _1));
            
            //SERVICES
            threshold_service_ = this->create_service<robot_custom_msgs::srv::Threshold>("/set_threshold",std::bind(&DistanceController::handle_threshold_service, this, _1, _2));
            fixedpoint_service_ = this->create_service<robot_custom_msgs::srv::FixedPoint>("/set_fixed_point",std::bind(&DistanceController::handle_fixedpoint_service, this, _1, _2));

            //PARAMETER
            this->declare_parameter<double>("threshold", 0.8);
            threshold = this->get_parameter("threshold").as_double();
            RCLCPP_INFO(this->get_logger(),"Initial threshold: %.2f", threshold);

            this->declare_parameter<int>("fixed_point_x", 10);
            fixed_point_x = this->get_parameter("fixed_point_x").as_int();

            this->declare_parameter<int>("fixed_point_y", 10);
            fixed_point_y = this->get_parameter("fixed_point_y").as_int();


            //VARIABLES
            dircetion_obstacle = "right";
            is_reversing.data = false;
            min_dist=10.0;
            new_dist.data=0.0;

            //CALLBACK PER UPDATE PARAMETRO 
            param_callback_handle_ = this->add_on_set_parameters_callback( std::bind(&DistanceController::on_param_change,this, std::placeholders::_1));
        }

    private:

        // FIXEDPOINT SERVICE
        void handle_fixedpoint_service(const std::shared_ptr<robot_custom_msgs::srv::FixedPoint::Request> request, 
            std::shared_ptr<robot_custom_msgs::srv::FixedPoint::Response> response) 
        {

            this->set_parameters({rclcpp::Parameter("fixed_point_x", request->fixed_point_x)});
            this->set_parameters({rclcpp::Parameter("fixed_point_y", request->fixed_point_y)});

            response->success = true;


        }

        // THRESHOLD SERVICE
        void handle_threshold_service(const std::shared_ptr<robot_custom_msgs::srv::Threshold::Request> request, 
            std::shared_ptr<robot_custom_msgs::srv::Threshold::Response> response) 
        {
            RCLCPP_INFO(this->get_logger(), "Ricevuta richiesta servizio: %.2f", request->threshold);

            this->set_parameters({rclcpp::Parameter("threshold", request->threshold)});

            response->ts = request->threshold;
            RCLCPP_INFO(this->get_logger(), "Risposta inviata all'interfaccia");
        }

        // CHECK IF THE ROBOT IS TOO NEAR AN OBSTACLE
        bool robot_in_danger(){
            if (min_dist > threshold){
                return false;
            }else{
                return true;
            }
        }

        // SET NEW THRESHOLD PARAMETER
        rcl_interfaces::msg::SetParametersResult on_param_change(const std::vector<rclcpp::Parameter> & params) {
            for (const auto & param : params) {
                if (param.get_name() == "threshold") {
                    threshold = param.as_double();
                    RCLCPP_INFO(this->get_logger(),
                                "Threshold updated: %.2f", threshold);
                }else if(param.get_name() == "fixed_point_x"){
                    fixed_point_x = param.as_double();
                }else if(param.get_name() == "fixed_point_y"){
                    fixed_point_y = param.as_double();
                }
            }
            rcl_interfaces::msg::SetParametersResult result;
            result.successful = true;
            return result;
        }

        // CHECK ROBOT MOVING DIRECTION TO REVERSE IT FOR THE REVERSE-PHASE
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

        void compute_distance(){

        }

        // CHECK WITH LASER SCAN THE DISTANCES WRT THE OBJECTS
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
                "Min dist: %.2f | Angle: %.2f rad | Direction: %s | New Dist: %.2f",
                min_dist, angle, dircetion_obstacle.c_str(), new_dist.data);

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
                    reverse_state_pub_->publish(is_reversing);
                }
            }

        }

        // UPDATING ROBOT POSE
        void robot_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg){
            current_robot_pose.pose.position.x = msg->pose.position.x;
            current_robot_pose.pose.position.y = msg->pose.position.y;

            new_dist.data = sqrt(pow((fixed_point_x-current_robot_pose.pose.position.x),2) + pow((fixed_point_y-current_robot_pose.pose.position.y),2));
            dist_rob_point_pub_->publish(new_dist);
        }

        // PUBLUSHING NEW VELOCITY
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
        rclcpp::Publisher<robot_custom_msgs::msg::ObstacleInfo>::SharedPtr custom_msg_pub_;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr robot_vel_pub;
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr reverse_state_pub_;
        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr dist_rob_point_pub_;
        

        //SUBSCRIBERS
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr intermediate_vel_sub_;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr robot_pose_sub_;
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
        
        //SERVICES
        rclcpp::Client<robot_custom_msgs::srv::Threshold>::SharedPtr threshold_client_;

        // SERVICE
        rclcpp::Service<robot_custom_msgs::srv::Threshold>::SharedPtr threshold_service_;
        rclcpp::Service<robot_custom_msgs::srv::FixedPoint>::SharedPtr fixedpoint_service_;


        //PARAMETER
        rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

        //VARIABLES
        robot_custom_msgs::msg::ObstacleInfo msg_obst_info;
        geometry_msgs::msg::PoseStamped current_robot_pose;
        geometry_msgs::msg::Twist stop_robot;
        geometry_msgs::msg::Twist vel_input;
        std_msgs::msg::Bool is_reversing;
        std::string dircetion_obstacle;
        std_msgs::msg::Float32 new_dist;
        int direction_index;
        double threshold;
        int fixed_point_x;
        int fixed_point_y;
        int scan_ranges;
        float min_dist;
        
};

int main(int argc, char * argv[]){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DistanceController>());
    rclcpp::shutdown();
    return 0;
}

