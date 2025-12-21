#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/int32.hpp"
#include <math.h>
using std::placeholders::_1;

class DistanceController: public rclcpp::Node{
    public:
        DistanceController(): Node("distance_controller"){
            
            //TIMERS
            check_timer = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&DistanceController::distance_boundaries_timer, this));

            //PUBLISHERS
            reverse_state_pub_ = this->create_publisher<std_msgs::msg::Int32>("/is_reversing", 10);

            //SUBSCRIBERS
            intermediate_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>("/intermediate_vel", 10, std::bind(&DistanceController::intermediate_vel_callback, this, _1));

            //VARIABLES
            command_input.linear.x = 0.0;
            command_input.angular.z = 0.0;
            is_reversing = false;
        }
    private:

        void stop() {
            if(id_turtle.data == 1){
                t1_vel_pub_->publish(stop_turtle);
                is_reversing_t1 = false;
            }else if(id_turtle.data == 2){
                t2_vel_pub_->publish(stop_turtle);
                is_reversing_t2 = false;

            }
            id_turtle.data=0;
            reverse_state_pub_->publish(id_turtle);
        }

        geometry_msgs::msg::Twist check_direction_turtle(){
            geometry_msgs::msg::Twist reverse_turtle_vel;
            if(command_input.linear.x < 0){
                reverse_turtle_vel.linear.x = 1;
            }else{
                reverse_turtle_vel.linear.x = -1;
            }
            if(command_input.angular.z != 0){
                reverse_turtle_vel.angular.z = -command_input.angular.z;
            }
            return reverse_turtle_vel;
        }

        bool is_t1_in_danger(){
            bool boundary_danger = t1_pose.x > 10.0 || t1_pose.y > 10.0 || t1_pose.x < 1.0 || t1_pose.y < 1.0;
            bool proximity_danger = (id_turtle.data == 1 && distance.data < 1.0);
            return boundary_danger || proximity_danger;
        }

        bool is_t2_in_danger(){
            bool boundary_danger = t2_pose.x > 10.0 || t2_pose.y > 10.0 || t2_pose.x < 1.0 || t2_pose.y < 1.0;
            bool proximity_danger = (id_turtle.data == 2 && distance.data < 1.0);
            return boundary_danger || proximity_danger;
        }
        
        void distance_boundaries_timer(){

            if (!t1_pose_received_ || !t2_pose_received_) {
                return; 
            }
            geometry_msgs::msg::Twist reverse_vel;

            distance.data = sqrt(pow((t2_pose.x-t1_pose.x),2) + pow((t2_pose.y-t1_pose.y),2));
            std::cout << "Distance:" << distance.data <<std::endl;
            distance_pub_->publish(distance);


            if(is_t1_in_danger()){
                if (!is_reversing_t1) {
                    is_reversing_t1 = true;    //problem if turtle doesn't come back in one only timer's step
                    reverse_state_pub_->publish(id_turtle);
                    reverse_vel = check_direction_turtle();
                    t1_vel_pub_->publish(reverse_vel);
                }
            }else{
                if(is_reversing_t1){
                    stop();
                }
            }

            if(is_t2_in_danger()){
                if (!is_reversing_t2) {
                    is_reversing_t2 = true;    //problem if turtle doesn't come back in one only timer's step
                    reverse_state_pub_->publish(id_turtle);
                    reverse_vel = check_direction_turtle();
                    t2_vel_pub_->publish(reverse_vel);
                }
            }else{
                if(is_reversing_t2){
                    stop();
                }
            }
        }

        void intermediate_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg){
            command_input = *msg;
            if(id_turtle.data == 1 && !is_reversing_t1) {
                t1_vel_pub_->publish(command_input);
            } else if (id_turtle.data == 2 && !is_reversing_t2) {
                t2_vel_pub_->publish(command_input);
            }
        }

        //TIMERS
        rclcpp::TimerBase::SharedPtr check_timer;

        //PUBLISHER
        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr distance_pub_;
        rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr reverse_state_pub_;
        
        //SUBSCRIBERS
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr intermediate_vel_sub_;
        rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr id_turtle_managed_sub_;

        //VARIABLES
        geometry_msgs::msg::Twist command_input;
        bool is_reversing;
        
};

int main(int argc, char * argv[]){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DistanceController>());
    rclcpp::shutdown();
    return 0;
}

