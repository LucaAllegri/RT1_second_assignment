#include <chrono>
#include <memory>
#include <random>

#include "rclcpp/rclcpp.hpp"
#include "robot_custom_msgs/srv/threshold.hpp" 

//using Threshold = robot_custom_msgs::srv::Threshold;
using namespace std::chrono_literals;

class RobotServiceNode : public rclcpp::Node
{
public:
    RobotServiceNode() : Node("Robot_service_node")
    {
        //SERVICE
        thershold_service_ = this->create_service<robot_custom_msgs::srv::Threshold>("set_threshold",std::bind(&RobotServiceNode::handle_threshold_service, this, std::placeholders::_1, std::placeholders::_2));

        RCLCPP_INFO(this->get_logger(), "Threshold service ready.");
    }

private:
    void handle_threshold_service( const std::shared_ptr<robot_custom_msgs::srv::Threshold::Request> request, std::shared_ptr<robot_custom_msgs::srv::Threshold::Response> response)
    {
        threshold_ = request->threshold;
        response->ts = threshold_;
        RCLCPP_INFO(this->get_logger(), "Threshold updated to %.2f", threshold_);
    }

    //SERVICE
    rclcpp::Service<robot_custom_msgs::srv::Threshold>::SharedPtr thershold_service_;
    
    //VARIABLE
    float threshold_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RobotServiceNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}