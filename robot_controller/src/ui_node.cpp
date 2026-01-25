#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/bool.hpp"
#include "robot_custom_msgs/srv/threshold.hpp"
#include "robot_custom_msgs/srv/average_velocity.hpp"
#include "robot_custom_msgs/srv/fixed_point.hpp"
#include <iostream>
#include <limits>
#include <array>
using std::placeholders::_1;

class InputController : public rclcpp::Node{ 
    public:
        InputController(): Node("input_controller"){ 
            //TIMER
            this->start();

            //PUBLISHERS
            intermediate_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/intermediate_vel", 10);
            robot_moving_state_pub_ = this->create_publisher<std_msgs::msg::Bool>("/robot_moving", 10);

            //SUBSCRIBERS
            reverse_state_sub_ = this->create_subscription<std_msgs::msg::Bool>("/is_reversing", 10, std::bind(&InputController::reverse_state_callback, this, _1));

            //SERVICES
            avg_vel_service_ = this->create_service<robot_custom_msgs::srv::AverageVelocity>("/average_vel",std::bind(&InputController::handle_avg_vel_service, this, std::placeholders::_1, std::placeholders::_2));
            threshold_client_ = this->create_client<robot_custom_msgs::srv::Threshold>("/set_threshold");
            fixedpoint_client_ = this->create_client<robot_custom_msgs::srv::FixedPoint>("/set_fixed_point");

            //VARIABLES
            vel_input.angular.z = 0.0;
            vel_input.linear.x = 0.0;
            stop_vel.angular.z = 0.0;
            stop_vel.linear.x = 0.0;
            avg_vel_angular_z = 0.0;
            avg_vel_linear_x = 0.0;
            is_moving.data = false;
            is_reversing=false;
            count = 0;
        }

    private:

        // COMPUTE AVERAGE VELOCITY AND UPDATE VALUES
        void handle_avg_vel_service (const std::shared_ptr<robot_custom_msgs::srv::AverageVelocity::Request>, 
            std::shared_ptr<robot_custom_msgs::srv::AverageVelocity::Response> response) 
        {
            if(count < 5){
                response->avg_linear_x = 0.0;
                response->avg_angular_z = 0.0;
                return;
            }else{
                for(int i=0; i<5; i++){
                    avg_vel_linear_x = avg_vel_linear_x + last_vel[i].linear_x;
                    avg_vel_angular_z = avg_vel_angular_z + last_vel[i].angular_z;
                }
                avg_vel_linear_x = avg_vel_linear_x/5;
                avg_vel_angular_z = avg_vel_angular_z/5;

                response->avg_linear_x = avg_vel_linear_x;
                response->avg_angular_z = avg_vel_angular_z;
            }
        }

        // CHECK IF THE ROBOT IS IN REVERSE MODE
        void reverse_state_callback(const std_msgs::msg::Bool::SharedPtr msg){
            is_reversing = msg->data;
            if(is_reversing){
                input_timer_.reset();
            }else{
                this->start();
            }
        }

        // STOP ROBOT AFTER 5 SEC
        void stop_robot() {
            if (is_reversing) {
                is_moving.data = true;
                robot_moving_state_pub_->publish(is_moving);
                return;
            }
            intermediate_vel_pub_->publish(stop_vel);
            is_moving.data = false;
            robot_moving_state_pub_->publish(is_moving);
            
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
            if(is_moving.data){
                return;        
            }
            if(is_reversing){        //block if coming back
                return;
            }

            double linear, angular, new_threshold;
            int fixed_point_x, fixed_point_y;
            char direction;
            bool valid_direction = false;
            bool valid_threshold = false;
            bool valid_fixed_point = false;

            std::cout << "\n============= ROBOT CONTROL =============";
            std::cout << "\n=========================================";
            std::cout << "\n        w \\      e |      / r      t    ";
            std::cout << "\n        s <-             -> f            ";
            std::cout << "\n        x /      c |      \\ v      b    ";
            std::cout << "\n=========================================";
            std::cout << "\n=== Change direction: w/e/r/s/f/x/c/v ===";
            std::cout << "\n=== Change threshold (min 0.4): t     ===" << std::endl << std::endl;

            while(!valid_direction){
                std::cout<< "Insert Command : ";
                if (!(std::cin >> direction)) {
                    std::cin.clear();
                    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
                    continue;
                }

                switch (direction){
                    case 'w':
                        linear = 1.0;
                        angular = 1.0;
                        valid_direction = true;
                        break;

                    case 'e':
                        linear = 1.0;
                        angular = 0.0 ;
                        valid_direction = true;
                        break;

                    case 'r':
                        linear = 1.0;
                        angular = -1.0;
                        valid_direction = true;
                        break;
                    
                    case 'f':
                        linear = 0.0;
                        angular = -1.0;
                        valid_direction = true;
                        break;

                    case 'v':
                        linear = -1.0;
                        angular = 1.0;
                        valid_direction = true;
                        break;

                    case 'c':
                        linear = -1.0;
                        angular = 0.0;
                        valid_direction = true;
                        break;

                    case 'x':
                        linear = -1.0;
                        angular = -1.0;
                        valid_direction = true;
                        break;

                    case 's':
                        linear = 0.0;
                        angular = 1.0;
                        valid_direction = true;
                        break;

                    case 't':
                        while(!valid_threshold){
                            std::cout << "Insert threshold: ";
                            if (!(std::cin >> new_threshold)) {
                                std::cout << "Invalid Threshold input for Threshold.\n";
                                std::cin.clear();
                                std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
                            }else{
                                valid_threshold = true;
                                valid_direction = true;
                            }
                        }
                        
                        break;

                    case 'b':
                        while(!valid_fixed_point){
                            std::cout << "Insert fixed point's coordiante x: ";
                            if (!(std::cin >> fixed_point_x)) {
                                std::cout << "Invalid input for fixed_point_x.\n";
                                std::cin.clear();
                                std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
                            }else{
                                std::cout << "Insert fixed point's coordiante y: ";
                                if (!(std::cin >> fixed_point_y)) {
                                    std::cout << "Invalid input for fixed_point_y.\n";
                                    std::cin.clear();
                                    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
                                }else{
                                    valid_fixed_point = true;
                                    valid_direction   = true;
                                }
                            }
                        }
                        
                        break;
                    
                    default:
                        std::cout << "Invalid Command! Try again.\n";
                        std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
                }
            }

            if(direction != 't' && direction != 'b'){
                if(count<5){
                    last_vel[count].linear_x = abs(linear);
                    last_vel[count].angular_z = abs(angular);
                }else{
                    for(int i=0; i<5; i++){
                        if(i!=4){
                            last_vel[i] = last_vel[i+1];
                        }else{
                            last_vel[i].linear_x = abs(linear);
                            last_vel[i].angular_z = abs(angular);
                        }
                    }
                }
                count+=1;

                vel_input.linear.x = linear;
                vel_input.angular.z = angular;
                intermediate_vel_pub_->publish(vel_input);

            }else if(direction != 'b'){
                auto threshold_request = std::make_shared<robot_custom_msgs::srv::Threshold::Request>();
                if(new_threshold < 0.4){
                    std::cout << "Threshold insered too low, it imposted to the minumum: 0.4\n";
                    new_threshold = 0.4;   /* 0.4 perchè dall'urdf il sensore è al centro del robot, che è lungo 40cm. Gli ho lasciato un pò di margine
                                              Quindi il sensore dista 0.2 dalla coda e dalla testa del robot.
                                              Gli ho lasciato un pò di margine, di 0.2 */
                }
                threshold_request->threshold = new_threshold;  
                threshold_client_->async_send_request(threshold_request);
            }else{
                auto fixed_point_request = std::make_shared<robot_custom_msgs::srv::FixedPoint::Request>();
                
                fixed_point_request->fixed_point_x = fixed_point_x;  
                fixed_point_request->fixed_point_y = fixed_point_y;
                fixedpoint_client_->async_send_request(fixed_point_request);
            }

            is_moving.data=true;
            robot_moving_state_pub_->publish(is_moving);
            input_timer_.reset();

            stop_timer_ = this->create_wall_timer(
                std::chrono::seconds(3),
                std::bind(&InputController::stop_robot, this)
            );
            
        }

        //TIMER
        rclcpp::TimerBase::SharedPtr publish_state_timer_;
        rclcpp::TimerBase::SharedPtr input_timer_;
        rclcpp::TimerBase::SharedPtr stop_timer_;

        //PUBLISHER
        rclcpp::Publisher<robot_custom_msgs::srv::AverageVelocity::Response>::SharedPtr avg_vel_pub_;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr intermediate_vel_pub_;
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr robot_moving_state_pub_;
        
        //SUBSCRIBERS
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr reverse_state_sub_;

        //SERVICE
        rclcpp::Service<robot_custom_msgs::srv::AverageVelocity>::SharedPtr avg_vel_service_;
        rclcpp::Client<robot_custom_msgs::srv::Threshold>::SharedPtr threshold_client_;
        rclcpp::Client<robot_custom_msgs::srv::FixedPoint>::SharedPtr fixedpoint_client_;
        

        //VARIABLES
        geometry_msgs::msg::Twist vel_input;
        geometry_msgs::msg::Twist stop_vel;
        std_msgs::msg::Bool is_moving;
        double avg_vel_angular_z;
        double avg_vel_linear_x;
        bool is_reversing;
        int count;

        struct Vel{            
            double linear_x;
            double angular_z;
        };

        std::array<Vel, 5> last_vel;  //WHERE I STORE THE LAST 5 VELOCITIES
};

int main(int argc, char * argv[]){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<InputController>());
    rclcpp::shutdown();
    return 0;
}