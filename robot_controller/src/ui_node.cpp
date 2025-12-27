#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/bool.hpp"
#include "robot_custom_msgs/srv/threshold.hpp"
#include "robot_custom_msgs/srv/average_velocity.hpp"
#include <iostream>
#include <limits>
#include <array>
using std::placeholders::_1;
using std::placeholders::_1;

class InputController : public rclcpp::Node{ 
    public:
        InputController(): Node("input_controller"){ 
            //TIMER
            this->start();

            //PUBLISHERS
            intermediate_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/intermediate_vel", 10);
            avg_vel_pub_ = this->create_publisher<robot_custom_msgs::srv::AverageVelocity::Response>("/avg_vel", 10);
            
            //SUBSCRIBERS
            reverse_state_sub_ = this->create_subscription<std_msgs::msg::Bool>("/is_reversing", 10, std::bind(&InputController::reverse_state_callback, this, _1));

            //SERVICES
            avg_vel_service_ = this->create_service<robot_custom_msgs::srv::AverageVelocity>("/average_vel",std::bind(&RobotServiceNode::handle_avg_vel_service, this, std::placeholders::_1, std::placeholders::_2));
            threshold_client_ = this->create_client<robot_custom_msgs::srv::Threshold>("/set_threshold");

            //VARIABLES
            stop_vel.linear.x = 0.0;
            stop_vel.angular.z = 0.0;
            vel_input.linear.x = 0.0;
            vel_input.angular.z = 0.0;
            is_reversing=false;
            is_moving = false;
            count = 0;
            avg_vel_linear_x = 0.0;
            avg_vel_angular_z = 0.0;
        }

    private:

        void handle_average_vel_service (const std::shared_ptr<robot_custom_msgs::srv::Threshold::Request> request, 
            std::shared_ptr<robot_custom_msgs::srv::Threshold::Response> response) 
        {
            if(count < 4){
                response->avg_linear_x = 0.0;
                response->avg_angular_z = 0.0;
                avg_vel_pub_->publish(response);
            }else{
                response->avg_linear_x = avg_vel_linear_x;
                response->avg_angular_z = avg_vel_linear_x;
                avg_vel_pub_->publish(response);
            }
        }

        void reverse_state_callback(const std_msgs::msg::Bool::SharedPtr msg){
            is_reversing = msg->data;
            if(is_reversing){
                input_timer_.reset();
            }else{
                this->start();
            }
        }

        void stop_robot() {
            intermediate_vel_pub_->publish(stop_vel);
            is_moving = false;
            stop_timer_->reset();
            this->start();
        }

        void start(){
            if(!input_timer_ && !is_reversing){
                input_timer_ = this->create_wall_timer(
                    std::chrono::milliseconds(1000), 
                    std::bind(&InputController::input_timer_callback, this));        
            }
        }

        void input_timer_callback() {
            if(is_moving){
                return;        
            }
            if(is_reversing){        //block if coming back
                return;
            }

            double linear, angular, new_threshold;

            std::cout << "\n=== COMANDO ROBOT ===" << std::endl;
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

            std::cout << "Insert threshold: ";
            if (!(std::cin >> new_threshold)) {
                std::cout << "Invalid input for Threshold.\n";
                std::cin.clear();
                std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
                return;
            }

            if(count<4){
                last_vel[count].linear_x = abs(linear);
                last_vel[count].angular_z = abs(angular);
            }else{
                for(int i=0; i<5; i++){
                    if(i==0){
                        continue;
                    }else{
                        last_vel[i-1] = last_vel[i];
                    }
                }
                last_vel[4].linear_x = abs(linear);
                last_vel[4].angular_z = abs(angular);
            }
            count+=1;

            auto threshold_request = std::make_shared<robot_custom_msgs::srv::Threshold::Request>();
            threshold_request->threshold = new_threshold;
            threshold_client_->async_send_request(threshold_request);

            vel_input.linear.x = linear;
            vel_input.angular.z = angular;
            intermediate_vel_pub_->publish(vel_input);

            is_moving=true;
            input_timer_.reset();

            stop_timer_ = this->create_wall_timer(
                std::chrono::seconds(3),
                std::bind(&InputController::stop_robot, this)
            );

            if(count == 4){
                for(int i=0; i<5; i++){
                    avg_vel_linear_x = avg_vel_linear_x + last_vel[i].linear_x;
                    avg_vel_angular_z = avg_vel_angular_z + last_vel[i].angular_z;
                }
                avg_vel_linear_x = avg_vel_linear_x/5;
                avg_vel_angular_z = avg_vel_angular_z/5;
            }
            
        }

        //TIMER
        rclcpp::TimerBase::SharedPtr input_timer_;
        rclcpp::TimerBase::SharedPtr stop_timer_;

        //PUBLISHER
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr intermediate_vel_pub_;
        rclcpp::Publisher<robot_custom_msgs::srv::AverageVelocity::Response>::SharedPtr avg_vel_pub_;
        
        //SUBSCRIBERS
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr reverse_state_sub_;

        //SERVICE
        rclcpp::Client<robot_custom_msgs::srv::Threshold>::SharedPtr threshold_client_;
        rclcpp::Service<robot_custom_msgs::srv::AverageVelocity>::SharedPtr avg_vel_service_;

        //VARIABLES
        geometry_msgs::msg::Twist vel_input;
        geometry_msgs::msg::Twist stop_vel;
        bool is_reversing;
        bool is_moving;
        double avg_vel_linear_x;
        double avg_vel_angular_z;
        int count;

        struct Vel{
            double linear_x;
            double angular_z;
        };

        std::array<Vel, 5> last_vel;
};

int main(int argc, char * argv[]){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<InputController>());
    rclcpp::shutdown();
    return 0;
}