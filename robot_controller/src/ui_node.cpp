#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/int32.hpp"
#include <iostream>
using std::placeholders::_1;

class InputController : public rclcpp::Node{ 
    public:
        InputController(): Node("input_controller"){ 

            //TIMERS 
            this->start();

            //PUBLISHERS
            intermediate_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/intermediate_vel", 10);
            
            //SUBSCRIBERS

            //VARIABLES
            stop_vel.linear.x = 0.0;
            stop_vel.angular.z = 0.0;
            vel_input.linear.x = 0.0;
            vel_input.angular.z = 0.0;
            is_moving = false;
        }
        
        private:

            void stop_turtles(){
                intermediate_vel_pub_->publish(stop_vel);
                is_moving = false;
                stop_timer_.reset();
                this->start();
            }

            void start(){
                if(!input_timer_){
                    input_timer_ = this->create_wall_timer(
                        std::chrono::milliseconds(1000), 
                        std::bind(&InputController::input_timer_callback, this));        
                }
            }

            void input_timer_callback(){
                if(is_moving){
                    return;        
                }
                double linear, angular;
                
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
                    std::chrono::milliseconds(1000), 
                    std::bind(&InputController::stop_turtles, this)
                );

            } 

            //TIMERSs
            rclcpp::TimerBase::SharedPtr input_timer_;
            rclcpp::TimerBase::SharedPtr stop_timer_;

            //PUBLISHERS
            rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr intermediate_vel_pub_;

            //SUBSCRIBERS

            //VARIABLES
            geometry_msgs::msg::Twist vel_input;
            geometry_msgs::msg::Twist stop_vel;
            bool is_moving;
        
};

int main(int argc, char * argv[]){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<InputController>());
    rclcpp::shutdown();
    return 0;
}