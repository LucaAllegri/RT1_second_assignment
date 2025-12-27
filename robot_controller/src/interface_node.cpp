#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "robot_custom_msgs/msg/obstacle_info.hpp"
#include "robot_custom_msgs/srv/average_velocity.hpp"

using std::placeholders::_1;

class InterfaceNode : public rclcpp::Node {
    public:
        InterfaceNode() : Node("interface_node") {

            //SUBSCRIBERS
            obst_sub_ = this->create_subscription<robot_custom_msgs::msg::ObstacleInfo>("/obstacle_info", 10, std::bind(&InterfaceNode::obstacle_callback, this, _1));
            ui_sub_ = this->create_subscription<geometry_msgs::msg::Twist>("/intermediate_vel", 10, std::bind(&InterfaceNode::ui_callback, this, _1));

            //SERVICE
            avg_vel_client_ = this->create_client<robot_custom_msgs::srv::AverageVelocity>("/average_vel");

        }

    private:

        void request_avg_velocity(){
            if (!avg_vel_client_->wait_for_service(std::chrono::milliseconds(100))){
                RCLCPP_WARN(this->get_logger(), "Average velocity service not available");
                return;
            }

            auto request = std::make_shared<robot_custom_msgs::srv::AverageVelocity::Request>();

            avg_vel_client_->async_send_request(
                request,
                std::bind(&InterfaceNode::avg_vel_response_callback, this, _1)
            );
        }

        void avg_vel_response_callback(rclcpp::Client<robot_custom_msgs::srv::AverageVelocity>::SharedFuture future){
            auto response = future.get();
            avg_vel.avg_linear_x = response->avg_linear_x;
            avg_vel.avg_angular_z = response->avg_angular_z;
        }
 

        void obstacle_callback(const robot_custom_msgs::msg::ObstacleInfo::SharedPtr msg) {
            last_obst_info_.min_distance_obstacle = msg->min_distance_obstacle;
            last_obst_info_.direction = msg->direction;
            last_obst_info_.threshold = msg->threshold;
        }

        void ui_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
            request_avg_velocity();

            // STAMPA UNICA DATI
            std::cout << "\n--------------------------------------------" << std::endl;
            std::cout << "   RIEPILOGO STATO ROBOT    " << std::endl;
            std::cout << "--------------------------------------------" << std::endl;
            std::cout << "Distanza Ostacolo più vicino: " << last_obst_info_.min_distance_obstacle << " m" << std::endl;
            std::cout << "Direzione Ostacolo: " << last_obst_info_.direction << std::endl;
            std::cout << "Soglia di Sicurezza: " << last_obst_info_.threshold << std::endl;
            std::cout << "Media ultime 5 velocità: "<< std::endl;
            std::cout << "        linear velocity= "<< avg_vel.avg_linear_x  << std::endl;
            std::cout << "       angular velocity= "<< avg_vel.avg_angular_z  << std::endl;
            std::cout << "--------------------------------------------\n" << std::endl;
        }

        //SUBSCRIBERS
        rclcpp::Subscription<robot_custom_msgs::msg::ObstacleInfo>::SharedPtr obst_sub_;
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr ui_sub_;
        
        //SERVICE
        rclcpp::Client<robot_custom_msgs::srv::AverageVelocity>::SharedPtr avg_vel_client_;
        
        //VARIABLES
        robot_custom_msgs::srv::AverageVelocity::Response avg_vel;
        robot_custom_msgs::msg::ObstacleInfo last_obst_info_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<InterfaceNode>());
    rclcpp::shutdown();
    return 0;
}