#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/bool.hpp"
#include "robot_custom_msgs/msg/obstacle_info.hpp"
#include "robot_custom_msgs/srv/threshold.hpp"
#include <iostream>
#include <limits>
using std::placeholders::_1;

class InputController : public rclcpp::Node{ 
    public:
        InputController(): Node("input_controller"){ 

            //TIMERS 
            this->start();

            //PUBLISHERS
            intermediate_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/intermediate_vel", 10);
            
            //SUBSCRIBERS
            reverse_state_sub_ = this->create_subscription<std_msgs::msg::Bool>("/is_reversing", 10, std::bind(&InputController::reverse_state_callback, this, _1));

            //SERVICES
            threshold_client_ = this->create_client<robot_custom_msgs::srv::Threshold>("/set_threshold");

            //VARIABLES
            stop_vel.linear.x = 0.0;
            stop_vel.angular.z = 0.0;
            vel_input.linear.x = 0.0;
            vel_input.angular.z = 0.0;
            is_reversing=false;
            is_moving = false;
        }
        
        private:

            void set_threshold(float ts_value){
                auto thershold_request = std::make_shared<robot_custom_msgs::srv::Threshold::Request>();

                if (!threshold_client_->wait_for_service(std::chrono::seconds(1))) {
                    RCLCPP_WARN(this->get_logger(), "Waiting for threshold service...");
                    return;
                }

                thershold_request->threshold = ts_value;

                auto result_future = threshold_client_->async_send_request(
                    thershold_request,
                    [this](rclcpp::Client<robot_custom_msgs::srv::Threshold>::SharedFuture future)
                    {
                        auto response = future.get();
                        new_threshold = response->ts;
                        RCLCPP_INFO(this->get_logger(), "Updated Threshold from service: ts=%.2f", new_threshold);
                    }
                );
                RCLCPP_WARN(this->get_logger(), "Request sent, waiting for user input...");

            }

            void reverse_state_callback(const std_msgs::msg::Bool::SharedPtr msg){
                is_reversing = msg->data;
                if(is_reversing){
                    input_timer_.reset();
                }else{
                    this->start();
                }
            }

            void stop_turtles(){
                intermediate_vel_pub_->publish(stop_vel);
                is_moving = false;
                stop_timer_.reset();
                this->start();
            }

            void start(){
                if(!input_timer_ && !is_reversing){
                    input_timer_ = this->create_wall_timer(
                        std::chrono::milliseconds(1000), 
                        std::bind(&InputController::input_timer_callback, this));        
                }
            }

            void input_timer_callback(){
                if(is_moving){
                    return;        
                }
                if(is_reversing){        //block if coming back
                    return;
                }

                std::cout << "Insert threshold: ";
                if (!(std::cin >> new_threshold)) {
                    std::cout << "Invalid input for Threshold.\n";
                    std::cin.clear();
                    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
                    return;
                }
                set_threshold(new_threshold);


                double linear, angular;
                std::cout<< "Insert Velocity of the Robot\n";

                std::cout<< "Linear Velocity:";
                if (!(std::cin >> linear)) {
                    std::cout << "Invalid input for Linear Velocity.\n";
                    std::cin.clear();
                    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
                    return;
                }

                std::cout << "Angular Velocity:";
                if(!(std::cin >> angular)) {
                    std::cout << "Invalid input for Angular Velocity.\n";
                    std::cin.clear();
                    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
                    return;
                }

                vel_input.linear.x = linear;
                vel_input.angular.z = angular;

                intermediate_vel_pub_->publish(vel_input);

                is_moving=true;
                input_timer_.reset();

                stop_timer_ = this->create_wall_timer(
                    std::chrono::milliseconds(3000), 
                    std::bind(&InputController::stop_turtles, this)
                );

            } 

            //TIMERS
            rclcpp::TimerBase::SharedPtr input_timer_;
            rclcpp::TimerBase::SharedPtr stop_timer_;

            //PUBLISHERS
            rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr intermediate_vel_pub_;

            //SUBSCRIBERS
            rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr reverse_state_sub_;

            //SERVICES
            rclcpp::Client<robot_custom_msgs::srv::Threshold>::SharedPtr threshold_client_;


            //VARIABLES
            geometry_msgs::msg::Twist vel_input;
            geometry_msgs::msg::Twist stop_vel;
            float new_threshold;
            bool is_reversing;
            bool is_moving;
            
        
};

int main(int argc, char * argv[]){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<InputController>());
    rclcpp::shutdown();
    return 0;
}