#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "robot_custom_msgs/msg/obstacle_info.hpp"

using std::placeholders::_1;

class InterfaceNode : public rclcpp::Node {
    public:
        InterfaceNode() : Node("interface_node") {

            //SUBSCRIBERS
            sub_obst_ = this->create_subscription<robot_custom_msgs::msg::ObstacleInfo>("/obstacle_info", 10, std::bind(&InterfaceNode::obstacle_callback, this, _1));
            sub_ui_ = this->create_subscription<geometry_msgs::msg::Twist>("/intermediate_vel", 10, std::bind(&InterfaceNode::ui_callback, this, _1));
        }

    private:
        void obstacle_callback(const robot_custom_msgs::msg::ObstacleInfo::SharedPtr msg) {
            last_obst_info_.min_distance_obstacle = msg->min_distance_obstacle;
            last_obst_info_.direction = msg->direction;
            last_obst_info_.threshold = msg->threshold;
        }

        void ui_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
            // STAMPA UNICA DATI
            std::cout << "\n--------------------------------------------" << std::endl;
            std::cout << "   RIEPILOGO STATO ROBOT    " << std::endl;
            std::cout << "--------------------------------------------" << std::endl;
            std::cout << "Distanza Ostacolo più vicino: " << last_obst_info_.min_distance_obstacle << " m" << std::endl;
            std::cout << "Direzione Ostacolo: " << last_obst_info_.direction << std::endl;
            std::cout << "Soglia di Sicurezza: " << last_obst_info_.threshold << std::endl;
            std::cout << "--------------------------------------------\n" << std::endl;
        }

        //SUBSCRIBERS
        rclcpp::Subscription<robot_custom_msgs::msg::ObstacleInfo>::SharedPtr sub_obst_;
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_ui_;
        
        //VARIABLES
        robot_custom_msgs::msg::ObstacleInfo last_obst_info_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<InterfaceNode>());
    rclcpp::shutdown();
    return 0;
}