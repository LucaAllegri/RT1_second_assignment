#include "rclcpp/rclcpp.hpp"
#include <iostream>
using std::placeholders::_1;

class InputController : public rclcpp::Node{ 
    public:
        InputController(): Node("input_controller"){ 

        }
    private:
        
};

int main(int argc, char * argv[]){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<InputController>());
    rclcpp::shutdown();
    return 0;
}