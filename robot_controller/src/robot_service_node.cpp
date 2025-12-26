#include <chrono>
#include <memory>
#include <random>
#include "rclcpp/rclcpp.hpp"
#include "robot_custom_msgs/srv/threshold.hpp" 

//using Threshold = robot_custom_msgs::srv::Threshold;
using namespace std::chrono_literals;

class RobotServiceNode : public rclcpp::Node{
    public:
        RobotServiceNode() : Node("robot_service_node"){

            //PARAMETER
            param_client_ = std::make_shared<rclcpp::AsyncParametersClient>(this, "/distance_node");

            //SERVICE
            thershold_service_ = this->create_service<robot_custom_msgs::srv::Threshold>("/set_threshold",std::bind(&RobotServiceNode::handle_threshold_service, this, std::placeholders::_1, std::placeholders::_2));

            RCLCPP_INFO(this->get_logger(), "Threshold service ready.");
        }

    private:
        void handle_threshold_service(const std::shared_ptr<robot_custom_msgs::srv::Threshold::Request> request, 
            std::shared_ptr<robot_custom_msgs::srv::Threshold::Response> response) 
        {
            RCLCPP_INFO(this->get_logger(), "Ricevuta richiesta servizio: %.2f", request->threshold);

            param_client_->set_parameters({rclcpp::Parameter("threshold", request->threshold)});

            response->ts = request->threshold;
            RCLCPP_INFO(this->get_logger(), "Risposta inviata all'interfaccia");
        }

        //SERVICE
        rclcpp::Service<robot_custom_msgs::srv::Threshold>::SharedPtr thershold_service_;
        
        //PARAMETER
        std::shared_ptr<rclcpp::AsyncParametersClient> param_client_; //chiamata asincrona

};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RobotServiceNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}